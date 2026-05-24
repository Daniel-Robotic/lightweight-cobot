package ros;

// Sunrise OS 1.16 | FRI 1.16 | KUKA iiwa 7
//
// Режимы команды FRI:
//   POSITION      - ROS2 задаёт целевые углы суставов
//   TORQUE        - ROS2 задаёт добавочные моменты суставов
//   NO_COMMAND_MODE - FRI только читает состояние, ведение рукой
//
// Режимы управления Sunrise (только для POSITION и TORQUE):
//   POSITION_CONTROL         - жёсткое позиционирование
//   JOINT_IMPEDANCE_CONTROL  - упругое позиционирование с заданной жёсткостью
//
// Сетевые интерфейсы:
//   KONI - 192.170.10.10 (рекомендуется)
//   KLI  - 192.168.21.31

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.IFRISessionListener;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.AbstractMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;


public class ServerFriRos2 extends RoboticsAPIApplication {

    // Позиции для движения перед запуском FRI
    private static final double[] ZERO_POSITION = {0, 0, 0, 0, 0, 0, 0};
    private static final double[] MONITOR_WORKING_POSITION = {0, 0, 0, -1.57, 0, 1.57, 0};

    // Сетевые адреса
    private static final String KONI_IP = "192.170.10.10";
    private static final String KLI_IP = "192.168.21.31";

    private static final int FRI_CONNECT_TIMEOUT_SEC = 30;
    private static final double APPROACH_VEL = 0.30;

    // Параметры для NO_COMMAND_MODE: нулевая жёсткость позволяет свободно вести робота рукой
    private static final double MONITOR_JOINT_STIFFNESS = 0.0;
    private static final double MONITOR_JOINT_DAMPING = 0.7;


    // Режим команды FRI - что именно отправляет ROS2 в каждом цикле
    private enum CommandMode {
        POSITION,
        TORQUE,
        NO_COMMAND_MODE
    }

    // Режим управления Sunrise - как контроллер обрабатывает команды
    private enum ControlMode {
        POSITION_CONTROL,
        JOINT_IMPEDANCE_CONTROL
    }

    // Сетевой интерфейс для подключения FRI
    private enum NetworkInterface {
        KONI("KONI (192.170.10.10) - выделенная FRI-сеть", KONI_IP),
        KLI("KLI (192.168.21.31) - основная сеть KRC", KLI_IP);

        final String label;
        final String ip;

        NetworkInterface(String label, String ip) {
            this.label = label;
            this.ip = ip;
        }
    }


    private LBR _lbr;
    private Controller _lbrController;

    // Инструмент из Sunrise WB - Object Templates tool1
    // Масса и CoM берутся из Load Data этого объекта
    @Inject
    @Named("tool1")
    private Tool _tool;

    private NetworkInterface _selectedNetwork;
    private CommandMode _selectedCommandMode;
    private ControlMode _selectedControlMode;
    private int _sendPeriodMs;
    private double _jointStiffness;

    private FRIConfiguration _friConfig;
    private FRISession _friSession;
    private FRIJointOverlay _friOverlay;
    private IFRISessionListener _friListener;


    @Override
    public void initialize() {

        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];

        // Прикрепляем инструмент к фланцу для корректной гравкомпенсации
        _tool.attachTo(_lbr.getFlange());

        getLogger().info("ServerFriRos2 | KUKA iiwa 7 + ROS2 FRI");
        getLogger().info("Sunrise OS 1.16 | FRI 1.16");
        getLogger().info("Робот: " + _lbr.getName());
        getLogger().info("Инструмент: " + _tool.getName());

        initFriListener();
        requestUserConfig();
    }

    @Override
    public void run() throws Exception {

        moveToInitialPosition();

        switch (_selectedCommandMode) {
            case POSITION:       runPositionMode();    break;
            case TORQUE:         runTorqueMode();      break;
            case NO_COMMAND_MODE: runMonitorMode();    break;
        }

        getLogger().info("Программа завершена.");
    }

    @Override
    public void dispose() {

        if (_friSession != null) {
            getLogger().info("Закрытие FRI-сессии...");
            try {
                _friSession.close();
            } catch (Exception e) {
                getLogger().warn("Ошибка при закрытии FRI: " + e.getMessage());
            }
            _friSession = null;
        }

        super.dispose();
    }


    // Последовательный опрос конфигурации - каждый шаг зависит от предыдущего
    private void requestUserConfig() {

        // Шаг 1: сетевой интерфейс
        int netChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Шаг 1 - Сетевой интерфейс FRI\n\n"
            + "KONI: выделенная высокоскоростная сеть (рекомендуется)\n"
            + "KLI: основная сеть KRC",
            NetworkInterface.KONI.label,
            NetworkInterface.KLI.label
        );
        _selectedNetwork = (netChoice == 0) ? NetworkInterface.KONI : NetworkInterface.KLI;
        getLogger().info("Сетевой интерфейс: " + _selectedNetwork.label);

        // Шаг 2: режим команды FRI
        int modeChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Шаг 2 - Режим команды FRI\n\n"
            + "Position: ROS2 задаёт угловые позиции суставов\n"
            + "Torque: ROS2 задаёт добавочные моменты суставов\n"
            + "Monitor: только чтение, ведение рукой (NO_COMMAND_MODE)",
            "Position",
            "Torque",
            "Monitor"
        );

        if (modeChoice == 0) {
            _selectedCommandMode = CommandMode.POSITION;
            selectControlMode();
            selectSendPeriodForPosition();

        } else if (modeChoice == 1) {
            _selectedCommandMode = CommandMode.TORQUE;
            // TORQUE всегда требует JointImpedanceControlMode на стороне Sunrise
            _selectedControlMode = ControlMode.JOINT_IMPEDANCE_CONTROL;
            selectJointStiffness();
            selectSendPeriodForTorque();

        } else {
            _selectedCommandMode = CommandMode.NO_COMMAND_MODE;
            // Нулевая жёсткость задана константой, пользователю выбирать нечего
            _selectedControlMode = ControlMode.JOINT_IMPEDANCE_CONTROL;
            _sendPeriodMs = 2;
            _jointStiffness = 0.0;
            getLogger().info("Monitor (NO_COMMAND_MODE): период = " + _sendPeriodMs + " мс");
        }

        logConfiguration();
    }


    // Шаг 3a (только для POSITION): выбор режима управления Sunrise
    private void selectControlMode() {

        int ctrlChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Шаг 3 - Режим управления Sunrise (Position)\n\n"
            + "PositionControl: жёсткое позиционирование, максимальная точность следования\n"
            + "JointImpedance: упругое позиционирование, задаётся жёсткость суставов",
            "PositionControl",
            "JointImpedance"
        );

        if (ctrlChoice == 0) {
            _selectedControlMode = ControlMode.POSITION_CONTROL;
            _jointStiffness = 0.0;
            getLogger().info("Режим управления: PositionControlMode");
        } else {
            _selectedControlMode = ControlMode.JOINT_IMPEDANCE_CONTROL;
            selectJointStiffness();
        }
    }


    // Выбор жёсткости суставов для JointImpedanceControlMode
    private void selectJointStiffness() {

        int sChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Жёсткость суставов [Нм/рад]\n\n"
            + "Высокая жёсткость: точное следование, меньше отклонение от траектории\n"
            + "Низкая жёсткость: мягкое взаимодействие со средой\n\n"
            + "Внимание: 1500 Нм/рад только в производственном режиме без людей в зоне!",
            "1500 - жёсткий / производственный",
            "1000 - стандарт",
            "800 - средний",
            "500 - мягкий / взаимодействие",
            "300 - очень мягкий"
        );
        _jointStiffness = new double[]{1500, 1000, 800, 500, 300}[sChoice];
        getLogger().info("Жёсткость суставов: " + _jointStiffness + " Нм/рад");
    }


    private void selectSendPeriodForPosition() {

        int pChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Шаг 4 - Период отправки FRI [мс] (Position)\n\n"
            + "10 мс: стабильно, подходит для большинства сетей\n"
            + "5 мс: стандарт для ros2_control на 200 Гц\n"
            + "2 мс: быстро, требует низкого джиттера сети",
            "10 мс",
            "5 мс",
            "2 мс"
        );
        _sendPeriodMs = new int[]{10, 5, 2}[pChoice];
        getLogger().info("Период отправки: " + _sendPeriodMs + " мс");
    }


    private void selectSendPeriodForTorque() {

        int pChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Шаг 4 - Период отправки FRI [мс] (Torque)\n\n"
            + "При пропуске пакета Sunrise автоматически переходит в PositionHold.\n"
            + "Рекомендуется 1-2 мс при стабильной KONI-сети.",
            "5 мс",
            "2 мс (рекомендуется)",
            "1 мс (максимальная частота)"
        );
        _sendPeriodMs = new int[]{5, 2, 1}[pChoice];
        getLogger().info("Период отправки: " + _sendPeriodMs + " мс");
    }


    private void logConfiguration() {

        getLogger().info("Итоговая конфигурация:");
        getLogger().info("  Сеть: " + _selectedNetwork.label);
        getLogger().info("  IP: " + _selectedNetwork.ip);
        getLogger().info("  Режим команды FRI: " + _selectedCommandMode.name());
        getLogger().info("  Режим управления Sunrise: " + _selectedControlMode.name());
        getLogger().info("  Период отправки: " + _sendPeriodMs + " мс");
        getLogger().info("  Инструмент: " + _tool.getName());
        if (_selectedControlMode == ControlMode.JOINT_IMPEDANCE_CONTROL && _jointStiffness > 0) {
            getLogger().info("  Жёсткость суставов: " + _jointStiffness + " Нм/рад");
        }
    }


    private void moveToInitialPosition() {

        if (_selectedCommandMode == CommandMode.NO_COMMAND_MODE) {
            getLogger().info("Движение в рабочую позицию Monitor (через нулевую)...");
            _lbr.move(
                BasicMotions.batch(
                    BasicMotions.ptp(ZERO_POSITION).setBlendingRel(0.5),
                    BasicMotions.ptp(MONITOR_WORKING_POSITION)
                ).setJointVelocityRel(APPROACH_VEL)
            );
            getLogger().info("Рабочая позиция Monitor достигнута.");
        } else {
            getLogger().info("Движение в нулевую позицию...");
            _lbr.move(
                BasicMotions.ptp(ZERO_POSITION).setJointVelocityRel(APPROACH_VEL)
            );
            getLogger().info("Нулевая позиция достигнута.");
        }
    }


    private void runPositionMode() {

        getLogger().info("Запуск Position режима, управление: " + _selectedControlMode.name());

        if (!setupFriSession(ClientCommandMode.POSITION)) {
            return;
        }

        AbstractMotionControlMode ctrlMode = buildControlMode();
        PositionHold posHold = new PositionHold(ctrlMode, -1, TimeUnit.SECONDS);

        getLogger().info("Position режим активен. Ожидаю команды от ROS2...");
        _lbr.move(posHold.addMotionOverlay(_friOverlay));

        _friSession.close();
        _friSession = null;
        getLogger().info("Position режим завершён. FRI закрыт.");
    }


    private void runTorqueMode() {

        getLogger().info("Запуск Torque режима, жёсткость: " + _jointStiffness + " Нм/рад");

        if (!setupFriSession(ClientCommandMode.TORQUE)) {
            return;
        }

        JointImpedanceControlMode ctrlMode = new JointImpedanceControlMode(
            _jointStiffness, _jointStiffness, _jointStiffness,
            _jointStiffness, _jointStiffness, _jointStiffness,
            _jointStiffness
        );
        ctrlMode.setDampingForAllJoints(0.7);

        PositionHold posHold = new PositionHold(ctrlMode, -1, TimeUnit.SECONDS);

        getLogger().info("Torque режим активен. Ожидаю команды от ROS2...");
        _lbr.move(posHold.addMotionOverlay(_friOverlay));

        _friSession.close();
        _friSession = null;
        getLogger().info("Torque режим завершён. FRI закрыт.");
    }


    private void runMonitorMode() {

        getLogger().info("Запуск Monitor режима (NO_COMMAND_MODE).");

        // Фаза A: валидация Load Data инструмента из Sunrise WB
        validateLoadModel();

        // Фаза B: ждём подтверждения оператора, что ROS2 FRI-узел запущен
        getApplicationUI().displayModalDialog(
            ApplicationDialogType.INFORMATION,
            "Запустите ROS2 FRI-узел на ПК (" + _selectedNetwork.ip + ").\n\n"
            + "Нажмите OK когда ros2_control_node активен.",
            "OK - ROS2 готов"
        );

        // Фаза C: FRI в режиме только чтения (NO_COMMAND_MODE)
        if (!setupFriSession(ClientCommandMode.NO_COMMAND_MODE)) {
            return;
        }

        // Фаза D: PositionHold с нулевой жёсткостью - свободное ведение рукой
        JointImpedanceControlMode guidingMode = new JointImpedanceControlMode(
            MONITOR_JOINT_STIFFNESS, MONITOR_JOINT_STIFFNESS, MONITOR_JOINT_STIFFNESS,
            MONITOR_JOINT_STIFFNESS, MONITOR_JOINT_STIFFNESS, MONITOR_JOINT_STIFFNESS,
            MONITOR_JOINT_STIFFNESS
        );
        guidingMode.setDampingForAllJoints(MONITOR_JOINT_DAMPING);

        PositionHold posHold = new PositionHold(guidingMode, -1, TimeUnit.SECONDS);

        getLogger().info("Monitor режим активен.");
        getLogger().info("Ведите робота рукой - он не сопротивляется.");
        getLogger().info("Данные суставов транслируются в ROS2 каждые " + _sendPeriodMs + " мс.");
        getLogger().info("Остановите FRI-клиент для завершения.");

        _lbr.move(posHold);

        _friSession.close();
        _friSession = null;
        getLogger().info("Monitor режим завершён. FRI закрыт.");
    }


    // Создаёт объект режима управления на основе выбора пользователя
    private AbstractMotionControlMode buildControlMode() {

        if (_selectedControlMode == ControlMode.POSITION_CONTROL) {
            getLogger().info("Создан PositionControlMode.");
            return new PositionControlMode();
        }

        // JointImpedanceControlMode - одинаковая жёсткость для всех суставов
        JointImpedanceControlMode mode = new JointImpedanceControlMode(
            _jointStiffness, _jointStiffness, _jointStiffness,
            _jointStiffness, _jointStiffness, _jointStiffness,
            _jointStiffness
        );
        mode.setDampingForAllJoints(0.7);
        getLogger().info("Создан JointImpedanceControlMode, жёсткость = " + _jointStiffness + " Нм/рад");
        return mode;
    }


    // Проверяет Load Data инструмента из Sunrise WB через SmartServo.validateForImpedanceMode.
    // Корректные данные нагрузки обязательны для точной гравкомпенсации в Monitor режиме.
    private void validateLoadModel() {

        getLogger().info("Валидация нагрузки инструмента: " + _tool.getName());

        boolean valid = SmartServo.validateForImpedanceMode(_tool);

        if (valid) {
            getLogger().info("Load Data валидны. Гравкомпенсация будет работать точно.");
        } else {
            getLogger().warn("Валидация Load Data не прошла для инструмента: " + _tool.getName());
            getLogger().warn("Задайте данные в Sunrise WB -> Object Templates -> "
                + _tool.getName() + " -> Load Data");
            getLogger().warn("(Mass [кг], Centre of Mass [мм], Inertia [кг/м2])");
            getLogger().warn("Гравкомпенсация в Monitor режиме может работать некорректно.");

            int choice = getApplicationUI().displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Load Data инструмента '" + _tool.getName() + "' не заданы.\n\n"
                + "Без корректных данных нагрузки гравкомпенсация\n"
                + "будет работать с ошибкой.\n\n"
                + "Задайте данные в Sunrise WB -> Object Templates -> "
                + _tool.getName() + " -> Load Data\nи перезапустите программу.\n\n"
                + "Или продолжите без корректной нагрузки (на свой риск).",
                "Продолжить",
                "Остановить"
            );

            if (choice == 1) {
                throw new RuntimeException(
                    "Остановлено оператором: Load Data не заданы для " + _tool.getName());
            }
        }
    }


    private boolean setupFriSession(ClientCommandMode commandMode) {

        _friConfig = FRIConfiguration.createRemoteConfiguration(_lbr, _selectedNetwork.ip);
        _friConfig.setSendPeriodMilliSec(_sendPeriodMs);
        _friConfig.setReceiveMultiplier(1);

        getLogger().info("Создание FRI-сессии...");
        getLogger().info("Хост: " + _friConfig.getHostName()
            + ", порт: " + _friConfig.getPortOnRemote());
        getLogger().info("Режим команды: " + commandMode.name());
        getLogger().info("Период отправки: " + _friConfig.getSendPeriodMilliSec() + " мс");

        _friSession = new FRISession(_friConfig);
        _friSession.addFRISessionListener(_friListener);

        try {
            getLogger().info("Ожидание FRI-клиента на " + _selectedNetwork.ip
                + " (таймаут " + FRI_CONNECT_TIMEOUT_SEC + " с)...");
            _friSession.await(FRI_CONNECT_TIMEOUT_SEC, TimeUnit.SECONDS);
        } catch (TimeoutException e) {
            getLogger().error("Таймаут FRI! Клиент не ответил за " + FRI_CONNECT_TIMEOUT_SEC + " с.");
            getLogger().error("Убедитесь, что ROS2 FRI-узел запущен на " + _selectedNetwork.ip);
            _friSession.close();
            _friSession = null;
            return false;
        }

        getLogger().info("FRI-соединение установлено!");
        logFriChannelInfo();

        if (commandMode != ClientCommandMode.NO_COMMAND_MODE) {
            _friOverlay = new FRIJointOverlay(_friSession, commandMode);
            getLogger().info("FRIJointOverlay создан для режима: " + commandMode.name());
        } else {
            _friOverlay = null;
            getLogger().info("Monitor: FRIJointOverlay не создаётся (только чтение данных).");
        }

        return true;
    }


    private void initFriListener() {
        _friListener = new IFRISessionListener() {

            @Override
            public void onFRIConnectionQualityChanged(FRIChannelInformation info) {
                getLogger().info("FRI качество изменилось: " + info.getQuality()
                    + ", jitter=" + info.getJitter() + " мс"
                    + ", latency=" + info.getLatency() + " мс");
            }

            @Override
            public void onFRISessionStateChanged(FRIChannelInformation info) {
                getLogger().info("FRI состояние изменилось: " + info.getFRISessionState()
                    + ", jitter=" + info.getJitter() + " мс"
                    + ", latency=" + info.getLatency() + " мс");
            }
        };
    }

    private void logFriChannelInfo() {
        FRIChannelInformation info = _friSession.getFRIChannelInformation();
        getLogger().info("FRI состояние: " + info.getFRISessionState());
        getLogger().info("FRI качество: " + info.getQuality());
        getLogger().info("FRI jitter: " + info.getJitter() + " мс");
        getLogger().info("FRI latency: " + info.getLatency() + " мс");
    }


    public static void main(final String[] args) {
        ServerFriRos2 app = new ServerFriRos2();
        app.runApplication();
    }
}
