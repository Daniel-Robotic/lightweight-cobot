package ros;

/**
 * FRI bridge between KUKA iiwa 7 and ROS 2 via lbr_ros2_control.
 * FRI-мост между KUKA iiwa 7 и ROS 2 через lbr_ros2_control.
 *
 * Tested on: Sunrise OS 1.16 / FRI 1.16 / iiwa 7 R800
 * Проверено на: Sunrise OS 1.16 / FRI 1.16 / iiwa 7 R800
 *
 * Two network interfaces are supported:
 * Поддерживаются два сетевых интерфейса:
 *
 *   KONI (X66) — dedicated high-speed FRI network, recommended.
 *               Recommended send period: 5–10 ms.
 *               Поддерживает все режимы включая Monitor (ведение рукой).
 *
 *   KLI  (X6)  — shared KRC control network, fallback option.
 *               Send period fixed at 10 ms to avoid packet loss on a shared bus.
 *               Monitor mode not available: KLI latency is too high for gravity
 *               compensation to be safe without a dedicated FRI stream.
 *               KLI latency слишком высока для безопасной гравкомпенсации.
 *
 * Control modes / Режимы управления:
 *   Position
 *   JointImpedance
 *   Monitor
 */

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

    // CONFIGURE BEFORE DEPLOYMENT — проверить перед запуском на новом стенде

    // IP address of the PC running the ROS 2 FRI node, as seen from the robot.
    // IP-адрес ПК с ROS 2 FRI-узлом со стороны робота.
    // KONI (X66 connector): default subnet is 192.170.10.x — change the last octet to match your PC.
    // KLI  (X6  connector): depends on your KRC network config.
    private static final String KONI_IP = "192.170.10.10";  // <<< CHANGE THIS / ИЗМЕНИТЬ
    private static final String KLI_IP  = "192.168.21.31";  // <<< CHANGE THIS / ИЗМЕНИТЬ

    // Tool name as defined in Sunrise Workbench -> Object Templates.
    // Имя инструмента из Sunrise Workbench -> Object Templates.
    // Must have valid Load Data (mass, CoM, inertia) for Monitor mode gravity compensation.
    // Для Monitor режима обязательно заполните Load Data (масса, ЦМ, инерция).
    // @Named is set below on the _tool field 

    // Safe joint-space pose the robot moves to before FRI starts.
    // Безопасная поза (в пространстве суставов) куда робот едет перед запуском FRI.
    // Adjust to avoid collisions with your cell layout / инструментом / оснасткой.
    private static final double[] ZERO_POSITION = {0, 0, 0, 0, 0, 0, 0}; // <<< CHECK / ПРОВЕРИТЬ
    private static final double[] MONITOR_WORKING_POSITION = {0, 0, 0, -1.57, 0, 1.57, 0}; // <<< CHECK / ПРОВЕРИТЬ

    // TUNING — fine-tune if needed / настройки при необходимости

    // How long to wait for the ROS 2 client to connect before giving up.
    // Время ожидания подключения FRI-клиента до отмены.
    private static final int    FRI_CONNECT_TIMEOUT_SEC = 30;

    // Relative joint velocity used for approach moves (0.0–1.0 of rated speed).
    // Относительная скорость для подъездных движений (0.0–1.0 от номинальной).
    private static final double APPROACH_VEL = 0.30;

    // Stiffness/damping for Monitor (gravity-comp, zero-stiffness hand guiding).
    // Жёсткость/демпфирование для Monitor: нулевая жёсткость = робот не сопротивляется руке.
    private static final double MONITOR_JOINT_STIFFNESS = 0.0;
    private static final double MONITOR_JOINT_DAMPING = 0.7;

    private enum CommandMode {
        POSITION,
        NO_COMMAND_MODE
    }

    private enum ControlMode {
        POSITION_CONTROL,
        JOINT_IMPEDANCE_CONTROL
    }

    /**
     * Encapsulates a network interface choice together with the IP address
     * that will be passed to FRIConfiguration.
     * Хранит выбор сетевого интерфейса и соответствующий IP для FRIConfiguration.
     *
     * The label is built from the IP constant so the dialog button always
     * reflects the actual address without manual string maintenance.
     * Label строится из IP-константы - при изменении IP адрес в кнопке обновится сам.
     */
    private enum NetworkInterface {
        KONI(KONI_IP),
        KLI (KLI_IP);

        final String label;
        final String ip;

        NetworkInterface(String ip) {
            this.ip = ip;
            this.label = name() + " — " + ip;
        }
    }


    private LBR _lbr;
    private Controller _lbrController;

    // Must match the Object Template name in Sunrise Workbench.
    // Должно совпадать с именем объекта в Sunrise Workbench -> Object Templates.
    @Inject
    @Named("tool1")   // <<< CHANGE THIS / ИЗМЕНИТЬ
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

        // Attach tool so Sunrise accounts for its mass in all motion planning.
        // Крепим инструмент, чтобы Sunrise учитывал его массу при всех движениях.
        _tool.attachTo(_lbr.getFlange());

        getLogger().info("ServerFriRos2 | KUKA iiwa 7 | ROS2 FRI");
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
            case POSITION:        runPositionMode(); break;
            case NO_COMMAND_MODE: runMonitorMode();  break;
        }

        getLogger().info("Программа завершена.");
    }

    @Override
    public void dispose() {

        // Sunrise calls dispose() even if run() threw - make sure the session is always released.
        // Sunrise вызывает dispose() даже при исключении в run() - сессия должна быть освобождена.
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


    private void requestUserConfig() {

        int netChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Шаг 1 — Сетевой интерфейс FRI",
            NetworkInterface.KONI.label,
            NetworkInterface.KLI.label
        );
        _selectedNetwork = (netChoice == 0) ? NetworkInterface.KONI : NetworkInterface.KLI;
        getLogger().info("Сетевой интерфейс: " + _selectedNetwork.label);

        if (_selectedNetwork == NetworkInterface.KLI) {
            configureKli();
        } else {
            configureKoni();
        }

        logConfiguration();
    }


    // KLI: Position and JointImpedance only; send period fixed at 10 ms.
    // KLI: доступны только Position и JointImpedance; период зафиксирован на 10 мс.
    // Monitor is excluded because KLI latency makes zero-stiffness guiding unsafe.
    // Monitor исключён: задержки KLI делают ведение рукой с нулевой жёсткостью небезопасным.
    private void configureKli() {

        int modeChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Шаг 2 — Режим управления",
            "Position",
            "JointImpedance"
        );

        _selectedCommandMode = CommandMode.POSITION;

        if (modeChoice == 0) {
            _selectedControlMode = ControlMode.POSITION_CONTROL;
            _jointStiffness = 0.0;
        } else {
            _selectedControlMode = ControlMode.JOINT_IMPEDANCE_CONTROL;
            selectJointStiffness();
        }

        // KLI shared bus cannot reliably sustain 5 ms cycles; 10 ms is the safe floor.
        // Общая шина KLI не выдерживает стабильные циклы 5 мс; 10 мс — безопасный минимум.
        _sendPeriodMs = 10;
        getLogger().info("KLI: период зафиксирован 10 мс");
    }


    // KONI: all three modes available; operator picks the send period.
    // KONI: доступны все три режима; оператор выбирает период отправки.
    private void configureKoni() {

        int modeChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Шаг 2 — Режим управления",
            "Position",
            "JointImpedance",
            "Monitor"
        );

        if (modeChoice == 0) {
            _selectedCommandMode = CommandMode.POSITION;
            _selectedControlMode = ControlMode.POSITION_CONTROL;
            _jointStiffness = 0.0;
            selectSendPeriod();

        } else if (modeChoice == 1) {
            _selectedCommandMode = CommandMode.POSITION;
            _selectedControlMode = ControlMode.JOINT_IMPEDANCE_CONTROL;
            selectJointStiffness();
            selectSendPeriod();

        } else {
            _selectedCommandMode = CommandMode.NO_COMMAND_MODE;
            _selectedControlMode = ControlMode.JOINT_IMPEDANCE_CONTROL;
            // Monitor doesn't drive joints, so 2 ms gives the densest state stream to ROS 2.
            // Monitor не командует суставами, поэтому 2 мс дают максимально плотный поток в ROS 2.
            _sendPeriodMs   = 2;
            _jointStiffness = 0.0;
            getLogger().info("Monitor: период = " + _sendPeriodMs + " мс");
        }
    }


    private void selectJointStiffness() {

        int sChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Жёсткость суставов [Нм/рад]",
            "1500",
            "1000",
            "800",
            "500"
        );
        _jointStiffness = new double[]{1500, 1000, 800, 500}[sChoice];
        getLogger().info("Жёсткость суставов: " + _jointStiffness + " Нм/рад");
    }


    private void selectSendPeriod() {

        int pChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Шаг 3 — Период отправки FRI",
            "10 мс",
            "5 мс"
        );
        _sendPeriodMs = (pChoice == 0) ? 10 : 5;
        getLogger().info("Период отправки: " + _sendPeriodMs + " мс");
    }


    private void logConfiguration() {

        getLogger().info("── Конфигурация ──────────────────");
        getLogger().info("  Network   : " + _selectedNetwork.label);
        getLogger().info("  FRI mode  : " + _selectedCommandMode.name());
        getLogger().info("  Ctrl mode : " + _selectedControlMode.name());
        getLogger().info("  Period    : " + _sendPeriodMs + " мс");
        getLogger().info("  Tool      : " + _tool.getName());
        if (_selectedControlMode == ControlMode.JOINT_IMPEDANCE_CONTROL && _jointStiffness > 0) {
            getLogger().info("  Stiffness : " + _jointStiffness + " Нм/рад");
        }
        getLogger().info("──────────────────────");
    }


    private void moveToInitialPosition() {

        if (_selectedCommandMode == CommandMode.NO_COMMAND_MODE) {
            // Move through zero first to avoid large single-joint swings.
            // Сначала едем через ноль, чтобы не было больших движений по одному суставу.
            getLogger().info("Движение в рабочую позицию Monitor...");
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
        try {
            _lbr.move(posHold.addMotionOverlay(_friOverlay));
        } catch (Exception e) {
            // Normal exit path when the ROS 2 client closes the FRI session.
            // Штатный путь выхода при закрытии FRI-сессии со стороны ROS 2.
            getLogger().info("FRI сеанс закрыт.");
        }

        closeFriSession();
        getLogger().info("Position режим завершён.");
    }


    private void runMonitorMode() {

        getLogger().info("Запуск Monitor режима...");

        // Validate tool Load Data before enabling zero-stiffness guiding —
        // incorrect inertia will make gravity compensation fight the operator.
        // Проверяем Load Data до включения нулевой жёсткости:
        // некорректная инерция заставит гравкомпенсацию работать против оператора.
        validateLoadModel();

        getApplicationUI().displayModalDialog(
            ApplicationDialogType.INFORMATION,
            "Запустите ROS2 FRI-узел на ПК (" + _selectedNetwork.ip + ").\n\n"
            + "Нажмите OK когда ros2_control_node активен.",
            "OK — ROS2 готов"
        );

        if (!setupFriSession(ClientCommandMode.NO_COMMAND_MODE)) {
            return;
        }

        // Zero stiffness + moderate damping = the robot doesn't resist hand guiding
        // but also doesn't flop around. Gravity is compensated by the controller.
        // Нулевая жёсткость + умеренное демпфирование: робот не сопротивляется руке,
        // но и не болтается. Гравитация компенсируется контроллером.
        JointImpedanceControlMode guidingMode = new JointImpedanceControlMode(
            MONITOR_JOINT_STIFFNESS, MONITOR_JOINT_STIFFNESS, MONITOR_JOINT_STIFFNESS,
            MONITOR_JOINT_STIFFNESS, MONITOR_JOINT_STIFFNESS, MONITOR_JOINT_STIFFNESS,
            MONITOR_JOINT_STIFFNESS
        );
        guidingMode.setDampingForAllJoints(MONITOR_JOINT_DAMPING);

        PositionHold posHold = new PositionHold(guidingMode, -1, TimeUnit.SECONDS);

        getLogger().info("Monitor режим активен.");
        getLogger().info("Ведите робота рукой, команды будут транслироваться в ROS2.");
        getLogger().info("Данные суставов транслируются в ROS2 каждые " + _sendPeriodMs + " мс.");
        getLogger().info("Остановите FRI-клиент для завершения.");

        try {
            _lbr.move(posHold);
        } catch (Exception e) {
            getLogger().info("FRI сеанс закрыт.");
        }

        closeFriSession();
        getLogger().info("Monitor режим завершён.");
    }


    // Silently closes the session — the client may have already torn it down.
    // Тихо закрывает сессию — клиент мог уже закрыть её со своей стороны.
    private void closeFriSession() {
        if (_friSession != null) {
            try {
                _friSession.close();
            } catch (Exception ignored) {}
            _friSession = null;
        }
    }


    private AbstractMotionControlMode buildControlMode() {

        if (_selectedControlMode == ControlMode.POSITION_CONTROL) {
            getLogger().info("Создан PositionControlMode.");
            return new PositionControlMode();
        }

        // Uniform stiffness across all joints is a reasonable default; tune per-joint
        // if the task requires asymmetric compliance (e.g. soft wrist, stiff elbow).
        // Одинаковая жёсткость по всем суставам — разумный старт; при необходимости
        // настройте каждый сустав отдельно (напр. мягкое запястье, жёсткий локоть).
        JointImpedanceControlMode mode = new JointImpedanceControlMode(
            _jointStiffness, _jointStiffness, _jointStiffness,
            _jointStiffness, _jointStiffness, _jointStiffness,
            _jointStiffness
        );
        mode.setDampingForAllJoints(0.7);
        getLogger().info("Создан JointImpedanceControlMode, жёсткость = " + _jointStiffness + " Нм/рад");
        return mode;
    }


    // SmartServo.validateForImpedanceMode checks that mass/CoM/inertia are non-zero.
    // SmartServo.validateForImpedanceMode проверяет, что масса/ЦМ/инерция заданы ненулевыми.
    private void validateLoadModel() {

        getLogger().info("Валидация нагрузки инструмента: " + _tool.getName());

        boolean valid = SmartServo.validateForImpedanceMode(_tool);

        if (valid) {
            getLogger().info("Load Data валидны. Гравкомпенсация будет работать точно.");
        } else {
            getLogger().warn("Load Data не заданы для: " + _tool.getName());
            getLogger().warn("Sunrise WB → Object Templates → " + _tool.getName() + " → Load Data");
            getLogger().warn("Требуются: Mass [кг], Centre of Mass [мм], Inertia [кг·м²]");
            getLogger().warn("Без них гравкомпенсация в Monitor режиме будет неточной.");

            int choice = getApplicationUI().displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Load Data инструмента '" + _tool.getName() + "' не заданы.\n\n"
                + "Без них гравитационная компенсация работает некорректно.\n\n"
                + "Задайте данные:\n"
                + "Sunrise WB → Object Templates → " + _tool.getName() + " → Load Data\n"
                + "и перезапустите программу.\n\n"
                + "Или продолжите на свой риск.",
                "Продолжить",
                "Остановить"
            );

            if (choice == 1) {
                throw new RuntimeException(
                    "Остановлено: Load Data не заданы для " + _tool.getName());
            }
        }
    }


    private boolean setupFriSession(ClientCommandMode commandMode) {

        _friConfig = FRIConfiguration.createRemoteConfiguration(_lbr, _selectedNetwork.ip);
        _friConfig.setSendPeriodMilliSec(_sendPeriodMs);
        _friConfig.setReceiveMultiplier(1);

        getLogger().info("Создание FRI-сессии...");
        getLogger().info("Хост: " + _friConfig.getHostName()
            + " | Порт: " + _friConfig.getPortOnRemote()
            + " | Режим: " + commandMode.name()
            + " | Период: " + _friConfig.getSendPeriodMilliSec() + " мс");

        _friSession = new FRISession(_friConfig);
        _friSession.addFRISessionListener(_friListener);

        try {
            getLogger().info("Ожидание FRI-клиента на " + _selectedNetwork.ip
                + " (таймаут " + FRI_CONNECT_TIMEOUT_SEC + " с)...");
            _friSession.await(FRI_CONNECT_TIMEOUT_SEC, TimeUnit.SECONDS);
        } catch (TimeoutException e) {
            getLogger().error("Таймаут FRI! Клиент не ответил за " + FRI_CONNECT_TIMEOUT_SEC + " с.");
            getLogger().error("Убедитесь, что ROS2 FRI-узел запущен на " + _selectedNetwork.ip);
            closeFriSession();
            return false;
        }

        getLogger().info("FRI-соединение установлено!");
        logFriChannelInfo();

        if (commandMode != ClientCommandMode.NO_COMMAND_MODE) {
            _friOverlay = new FRIJointOverlay(_friSession, commandMode);
            getLogger().info("FRIJointOverlay создан для режима: " + commandMode.name());
        } else {
            // In NO_COMMAND_MODE the robot state is streamed but no overlay is needed.
            // В NO_COMMAND_MODE состояние транслируется, но overlay не нужен.
            _friOverlay = null;
        }

        return true;
    }


    private void initFriListener() {
        _friListener = new IFRISessionListener() {

            @Override
            public void onFRIConnectionQualityChanged(FRIChannelInformation info) {
                getLogger().info("FRI quality: " + info.getQuality()
                    + " | jitter=" + info.getJitter() + " мс"
                    + " | latency=" + info.getLatency() + " мс");
            }

            @Override
            public void onFRISessionStateChanged(FRIChannelInformation info) {
                getLogger().info("FRI state: " + info.getFRISessionState()
                    + " | jitter=" + info.getJitter() + " мс"
                    + " | latency=" + info.getLatency() + " мс");
            }
        };
    }


    private void logFriChannelInfo() {
        FRIChannelInformation info = _friSession.getFRIChannelInformation();
        getLogger().info("FRI state   : " + info.getFRISessionState());
        getLogger().info("FRI quality : " + info.getQuality());
        getLogger().info("FRI jitter  : " + info.getJitter() + " мс");
        getLogger().info("FRI latency : " + info.getLatency() + " мс");
    }


    public static void main(final String[] args) {
        ServerFriRos2 app = new ServerFriRos2();
        app.runApplication();
    }
}
