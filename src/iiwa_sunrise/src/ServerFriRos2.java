package ros;

// ════════════════════════════════════════════════════════════════════════════
//
//  ServerFriRos2.java
//  Sunrise OS 1.16  |  FRI 1.16  |  KUKA iiwa 7
//
//  Режимы управления:
//    Position — FRI задаёт целевые углы суставов (PositionControlMode)
//               Сглаживание команд выполняется на стороне ROS2 (FRIClient EMA)
//    Torque   — FRI задаёт добавочные моменты (JointImpedanceControlMode)
//    Monitor  — FRI только читает состояние (NO_COMMAND_MODE)
//               Масса инструмента берётся из Sunrise WB (Load Data)
//               и валидируется через SmartServo.validateForImpedanceMode()
//
//  Сетевые интерфейсы:
//    KONI — 192.170.10.10 (рекомендуется)
//    KLI  — 192.168.21.31
//
// ════════════════════════════════════════════════════════════════════════════

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
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;


public class ServerFriRos2 extends RoboticsAPIApplication {


    // ── КОНСТАНТЫ ────────────────────────────────────────────────────────────

    private static final double[] ZERO_POSITION            = {0, 0, 0, 0, 0, 0, 0};
    private static final double[] MONITOR_WORKING_POSITION = {0, 0, 0, -1.57, 0, 1.57, 0};

    private static final String KONI_IP = "192.170.10.10";
    private static final String KLI_IP  = "192.168.21.31";

    private static final int    FRI_CONNECT_TIMEOUT_SEC = 30;
    private static final double APPROACH_VEL            = 0.30;

    // Monitor режим: нулевая жёсткость = свободное ведение рукой
    private static final double MONITOR_JOINT_STIFFNESS = 0.0;
    private static final double MONITOR_JOINT_DAMPING   = 0.7;


    // ── ПЕРЕЧИСЛЕНИЯ ─────────────────────────────────────────────────────────

    private enum NetworkInterface {
        KONI("KONI (192.170.10.10) - выделенная FRI-сеть", KONI_IP),
        KLI ("KLI  (192.168.21.31) - основная сеть KRC",   KLI_IP);

        final String label;
        final String ip;

        NetworkInterface(String label, String ip) {
            this.label = label;
            this.ip    = ip;
        }
    }

    private enum ControlMode {
        POSITION,
        TORQUE,
        MONITOR
    }


    // ── ПОЛЯ ─────────────────────────────────────────────────────────────────

    private LBR        _lbr;
    private Controller _lbrController;

    /**
     * Инструмент, настроенный в Sunrise WB → Object Templates.
     * Имя должно совпадать с именем в SWB (здесь: "patron").
     * Масса и CoM берутся из Load Data этого объекта.
     */
    @Inject
    @Named("patron")
    private Tool _tool;

    private NetworkInterface _selectedNetwork;
    private ControlMode      _selectedMode;
    private int              _sendPeriodMs;
    private double           _jointStiffness;

    private FRIConfiguration    _friConfig;
    private FRISession          _friSession;
    private FRIJointOverlay     _friOverlay;
    private IFRISessionListener _friListener;


    // ── LIFECYCLE ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {

        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];

        // Прикрепляем инструмент к фланцу — обязательно для корректной
        // гравкомпенсации и validateForImpedanceMode()
        _tool.attachTo(_lbr.getFlange());

        getLogger().info("════════════════════════════════════════════");
        getLogger().info("  ServerFriRos2  |  KUKA iiwa 7 + ROS2 FRI");
        getLogger().info("  Sunrise OS 1.16  |  FRI 1.16");
        getLogger().info("  Робот      : " + _lbr.getName());
        getLogger().info("  Инструмент : " + _tool.getName());
        getLogger().info("════════════════════════════════════════════");

        initFriListener();
        requestUserConfig();
    }

    @Override
    public void run() throws Exception {

        moveToInitialPosition();

        switch (_selectedMode) {
            case POSITION: runPositionMode(); break;
            case TORQUE:   runTorqueMode();   break;
            case MONITOR:  runMonitorMode();  break;
        }

        getLogger().info("Программа завершена.");
    }

    @Override
    public void dispose() {

        if (_friSession != null) {
            getLogger().info("dispose(): Закрытие FRI-сессии...");
            try {
                _friSession.close();
            } catch (Exception e) {
                getLogger().warn("Ошибка при закрытии FRI: " + e.getMessage());
            }
            _friSession = null;
        }

        super.dispose();
    }


    // ── UI — ЗАПРОС КОНФИГУРАЦИИ ─────────────────────────────────────────────

    private void requestUserConfig() {

        // Шаг 1: Сетевой интерфейс
        int netChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Шаг 1 / 3  —  Сетевой интерфейс FRI\n\n"
            + "KONI: выделенная высокоскоростная сеть (рекомендуется)\n"
            + "KLI : основная сеть KRC",
            NetworkInterface.KONI.label,
            NetworkInterface.KLI.label
        );
        _selectedNetwork = (netChoice == 0) ? NetworkInterface.KONI : NetworkInterface.KLI;
        getLogger().info("[Шаг 1] Сеть: " + _selectedNetwork.label);

        // Шаг 2: Режим управления
        int modeChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Шаг 2 / 3  —  Режим управления FRI\n\n"
            + "Position : ROS2 задаёт угловые позиции суставов\n"
            + "Torque   : ROS2 задаёт добавочные моменты\n"
            + "Monitor  : только данные; ведение рукой",
            "Position",
            "Torque",
            "Monitor"
        );

        if (modeChoice == 0) {
            _selectedMode   = ControlMode.POSITION;
            _jointStiffness = 0;

            int pChoice = getApplicationUI().displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Шаг 3 / 3  —  Период отправки [мс]  (Position)\n\n"
                + "Режим: PositionControlMode (жёсткое позиционирование)\n"
                + "Сглаживание команд выполняется на стороне ROS2.\n\n"
                + "10 мс — стабильно\n"
                + " 5 мс — стандарт для 200 Гц ros2_control\n"
                + " 2 мс — быстро, требует низкого джиттера",
                "10 мс",
                " 5 мс",
                " 2 мс"
            );
            _sendPeriodMs = new int[]{10, 5, 2}[pChoice];

        } else if (modeChoice == 1) {
            _selectedMode = ControlMode.TORQUE;

            int pChoice = getApplicationUI().displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Шаг 3а / 4  —  Период отправки [мс]  (Torque)\n\n"
                + "При пропуске пакета Sunrise переходит в PositionHold.\n"
                + "Рекомендуется 1–2 мс при стабильной KONI-сети.",
                " 5 мс",
                " 2 мс  (рекомендуется)",
                " 1 мс  (максимальная частота)"
            );
            _sendPeriodMs = new int[]{5, 2, 1}[pChoice];

            int sChoice = getApplicationUI().displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Шаг 3б / 4  —  Жёсткость суставов [Нм/рад]  (Torque)\n\n"
                + "Высокая: точное следование, меньше отклонение\n"
                + "Низкая : мягкое взаимодействие со средой\n\n"
                + "⚠  1500 Нм/рад — только без людей в рабочей зоне!",
                "1500  (жёсткий / производственный)",
                "1000  (стандарт)",
                " 800  (средний)",
                " 500  (мягкий / взаимодействие)",
                " 300  (очень мягкий)"
            );
            _jointStiffness = new double[]{1500, 1000, 800, 500, 300}[sChoice];

        } else {
            _selectedMode   = ControlMode.MONITOR;
            _sendPeriodMs   = 2;   // 2 мс — запас для non-RT систем без FIFO-планировщика
            _jointStiffness = 0;
            getLogger().info("[Шаг 3] Monitor: период = " + _sendPeriodMs + " мс");
        }

        getLogger().info("════════════════════════════════════════════");
        getLogger().info("  КОНФИГУРАЦИЯ ЗАПУСКА");
        getLogger().info("  Сеть       : " + _selectedNetwork.label);
        getLogger().info("  IP хоста   : " + _selectedNetwork.ip);
        getLogger().info("  Режим      : " + _selectedMode.name());
        getLogger().info("  Период     : " + _sendPeriodMs + " мс");
        getLogger().info("  Инструмент : " + _tool.getName());
        if (_selectedMode == ControlMode.TORQUE) {
            getLogger().info("  Жёсткость : " + _jointStiffness + " Нм/рад");
        }
        getLogger().info("════════════════════════════════════════════");
    }


    // ── ДВИЖЕНИЕ В СТАРТОВУЮ ПОЗИЦИЮ ─────────────────────────────────────────

    private void moveToInitialPosition() {

        if (_selectedMode == ControlMode.MONITOR) {

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


    // ── РЕЖИМ: POSITION ──────────────────────────────────────────────────────

    private void runPositionMode() {
        getLogger().info("═══ Position режим (PositionControlMode) ═══");

        if (!setupFriSession(ClientCommandMode.POSITION)) {
            return;
        }

        PositionControlMode ctrlMode = new PositionControlMode();
        PositionHold posHold = new PositionHold(ctrlMode, -1, TimeUnit.SECONDS);

        getLogger().info("Position режим активен. Ожидаю команды от ROS2...");

        _lbr.move(posHold.addMotionOverlay(_friOverlay));

        _friSession.close();
        _friSession = null;
        getLogger().info("Position режим завершён. FRI закрыт.");
    }


    // ── РЕЖИМ: TORQUE ────────────────────────────────────────────────────────

    private void runTorqueMode() {
        getLogger().info("═══ Torque режим ═══");
        getLogger().info("Жёсткость: " + _jointStiffness + " Нм/рад");

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


    // ── РЕЖИМ: MONITOR ───────────────────────────────────────────────────────

    /**
     * Monitor режим.
     *
     * Фаза A: Валидация Load Data инструмента из Sunrise WB.
     *         Масса берётся из Object Templates → patron → Load Data,
     *         как в TeachKuka.java (SmartServo.validateForImpedanceMode).
     *
     * Фаза B: Диалог подтверждения — оператор запускает ROS2 FRI узел.
     *
     * Фаза C: FRI в NO_COMMAND_MODE — данные суставов идут в ROS2.
     *
     * Фаза D: PositionHold с нулевой жёсткостью — свободное ведение рукой.
     */
    private void runMonitorMode() {
        getLogger().info("═══ Monitor режим ═══");

        // Фаза A: валидация Load Data инструмента из SWB
        validateLoadModel();

        // Фаза B: ждём подтверждения оператора что ROS2 FRI готов
        getApplicationUI().displayModalDialog(
            ApplicationDialogType.INFORMATION,
            "Запустите ROS2 FRI узел на ПК (" + _selectedNetwork.ip + ").\n\n"
            + "Нажмите OK когда ros2_control_node активен.",
            "OK — ROS2 готов"
        );

        // Фаза C: FRI в режиме только чтения
        if (!setupFriSession(ClientCommandMode.NO_COMMAND_MODE)) {
            return;
        }

        // Фаза D: PositionHold с нулевой жёсткостью — свободное ведение рукой
        JointImpedanceControlMode guidingMode = new JointImpedanceControlMode(
            MONITOR_JOINT_STIFFNESS, MONITOR_JOINT_STIFFNESS, MONITOR_JOINT_STIFFNESS,
            MONITOR_JOINT_STIFFNESS, MONITOR_JOINT_STIFFNESS, MONITOR_JOINT_STIFFNESS,
            MONITOR_JOINT_STIFFNESS
        );
        guidingMode.setDampingForAllJoints(MONITOR_JOINT_DAMPING);

        PositionHold posHold = new PositionHold(guidingMode, -1, TimeUnit.SECONDS);

        getLogger().info("Monitor активен:");
        getLogger().info("  • Ведите робота рукой — он не сопротивляется.");
        getLogger().info("  • Данные суставов транслируются в ROS2 каждые "
            + _sendPeriodMs + " мс.");
        getLogger().info("  • Остановите FRI-клиент для завершения.");

        _lbr.move(posHold);

        _friSession.close();
        _friSession = null;
        getLogger().info("Monitor режим завершён. FRI закрыт.");
    }


    // ── ВАЛИДАЦИЯ НАГРУЗКИ ИНСТРУМЕНТА ──────────────────────────────────────

    /**
     * Проверяет Load Data (масса / CoM / инерция) инструмента из Sunrise WB.
     *
     * Аналог validateLoadModel() из TeachKuka.java:
     *   SmartServo.validateForImpedanceMode(_tool) проверяет, что данные
     *   нагрузки заданы корректно для работы с JointImpedanceControlMode.
     *
     * Если валидация не прошла — нужно задать Load Data в:
     *   Sunrise WB → Object Templates → patron → Load Data
     *   (Mass, Centre of Mass, Moment of Inertia)
     */
    private void validateLoadModel() {
        getLogger().info("════════════════════════════════════════════");
        getLogger().info("  ВАЛИДАЦИЯ НАГРУЗКИ ИНСТРУМЕНТА");
        getLogger().info("  Инструмент: " + _tool.getName());
        getLogger().info("════════════════════════════════════════════");

        boolean valid = SmartServo.validateForImpedanceMode(_tool);

        if (valid) {
            getLogger().info("  ✓ Load Data валидны.");
            getLogger().info("    Масса и CoM заданы корректно в Sunrise WB.");
            getLogger().info("    Гравкомпенсация будет работать точно.");
        } else {
            getLogger().warn("  ⚠ Валидация Load Data НЕ прошла!");
            getLogger().warn("    Задайте данные нагрузки в:");
            getLogger().warn("    Sunrise WB → Object Templates → "
                + _tool.getName() + " → Load Data");
            getLogger().warn("    (Mass [кг], Centre of Mass [мм], Inertia [кг·м²])");
            getLogger().warn("    Гравкомпенсация в Monitor режиме может работать некорректно.");

            // Предупреждаем оператора — он решает продолжить или нет
            int choice = getApplicationUI().displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Load Data инструмента '" + _tool.getName() + "' не заданы.\n\n"
                + "Без корректных данных нагрузки гравкомпенсация\n"
                + "будет работать с ошибкой.\n\n"
                + "Задайте данные в Sunrise WB → Object Templates → "
                + _tool.getName() + " → Load Data,\nзатем перезапустите программу.\n\n"
                + "Или продолжите без корректной нагрузки (на свой риск).",
                "Продолжить",
                "Остановить"
            );

            if (choice == 1) {
                throw new RuntimeException(
                    "Остановлено оператором: Load Data не заданы для "
                    + _tool.getName());
            }
        }

        getLogger().info("════════════════════════════════════════════");
    }


    // ── FRI — НАСТРОЙКА СЕССИИ ───────────────────────────────────────────────

    private boolean setupFriSession(ClientCommandMode commandMode) {

        _friConfig = FRIConfiguration.createRemoteConfiguration(_lbr, _selectedNetwork.ip);
        _friConfig.setSendPeriodMilliSec(_sendPeriodMs);
        _friConfig.setReceiveMultiplier(1);

        getLogger().info("Создание FRI-сессии...");
        getLogger().info("  Хост    : " + _friConfig.getHostName()
            + "  порт: " + _friConfig.getPortOnRemote());
        getLogger().info("  Режим   : " + commandMode.name());
        getLogger().info("  Период  : " + _friConfig.getSendPeriodMilliSec() + " мс");

        _friSession = new FRISession(_friConfig);
        _friSession.addFRISessionListener(_friListener);

        try {
            getLogger().info("Ожидание FRI-клиента на " + _selectedNetwork.ip
                + " (таймаут " + FRI_CONNECT_TIMEOUT_SEC + " с)...");
            _friSession.await(FRI_CONNECT_TIMEOUT_SEC, TimeUnit.SECONDS);

        } catch (TimeoutException e) {
            getLogger().error("Таймаут FRI! Клиент не ответил за "
                + FRI_CONNECT_TIMEOUT_SEC + " с.");
            getLogger().error("Убедитесь, что ROS2 FRI-узел запущен на "
                + _selectedNetwork.ip);
            _friSession.close();
            _friSession = null;
            return false;
        }

        getLogger().info("FRI-соединение установлено!");
        logFriChannelInfo();

        if (commandMode != ClientCommandMode.NO_COMMAND_MODE) {
            _friOverlay = new FRIJointOverlay(_friSession, commandMode);
            getLogger().info("FRIJointOverlay создан: " + commandMode.name());
        } else {
            _friOverlay = null;
            getLogger().info("Monitor: FRIJointOverlay не создаётся (только чтение).");
        }

        return true;
    }


    // ── FRI — LISTENER ───────────────────────────────────────────────────────

    private void initFriListener() {
        _friListener = new IFRISessionListener() {

            @Override
            public void onFRIConnectionQualityChanged(FRIChannelInformation info) {
                getLogger().info("[FRI] Качество: " + info.getQuality()
                    + "  jitter=" + info.getJitter() + " мс"
                    + "  latency=" + info.getLatency() + " мс");
            }

            @Override
            public void onFRISessionStateChanged(FRIChannelInformation info) {
                getLogger().info("[FRI] Состояние: " + info.getFRISessionState()
                    + "  jitter=" + info.getJitter() + " мс"
                    + "  latency=" + info.getLatency() + " мс");
            }
        };
    }

    private void logFriChannelInfo() {
        FRIChannelInformation info = _friSession.getFRIChannelInformation();
        getLogger().info("[FRI] Состояние : " + info.getFRISessionState());
        getLogger().info("[FRI] Качество  : " + info.getQuality());
        getLogger().info("[FRI] Jitter    : " + info.getJitter() + " мс");
        getLogger().info("[FRI] Latency   : " + info.getLatency() + " мс");
    }

}
