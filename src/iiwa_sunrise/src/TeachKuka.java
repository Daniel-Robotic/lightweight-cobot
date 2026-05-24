package application;

//══════════════════════════════════════════════════════════════════════
//
//
//TeachKuka.java
//Sunrise OS 1.16 | Servoing 1.16 | ApplicationFramework 1.2
//
//
//Режимы работы:
// 	Режим 1: Захват позиции
//	Режим 2: Запись и воспроизведение траектории
//
//
//══════════════════════════════════════════════════════════════════════

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class TeachKuka extends RoboticsAPIApplication {
	
	@Inject
	private LBR _lbr;
	
	@Inject
	private Controller _lbrController;
	
	@Inject
    @Named("tool1")
    private Tool _gripper;
	
//	==== КОНСТАНТНЫ =====
	private static final double[] HOME_POSITION_RAD = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };

    private static final double[] WORKING_POSITION_RAD = {
        0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0
    };

    private static final double HOME_BLENDING_REL = 0.5;
    private static final double APPROACH_VELOCITY_REL = 0.3;
    private static final double REPLAY_VELOCITY_REL = 0.2;

    private static final double GRAV_COMP_STIFFNESS_TRANSL = 5.0; // N/m
    private static final double GRAV_COMP_STIFFNESS_ROT = 1.0;  // Nm/рад
    private static final double GRAV_COMP_DAMPING = 0.7;

    private static final long GRAV_COMP_UPDATE_INTERVAL_MS = 10L;
    
 // Интервал записи точек (мс). 100 мс = 10 точек/сек
    private static final long RECORD_INTERVAL_MS = 100L;
 // Лимит точек: 3000 * 100 мс = 5 минут записи
    private static final int MAX_TRAJECTORY_POINTS = 3000;
 // Задержка перед стартом воспроизведения — чтобы успеть отойти
    private static final long PRE_REPLAY_DELAY_MS = 2000L;
//  Порог детекции столкновения по внешнему суставному моменту (Нм)
//  4–6 Нм - высокая чувствительность
//  7–10 Нм - средняя (производственная среда)
    private static final double COLLISION_TORQUE_THRESHOLD_NM = 6.0;
 // Период опроса пока путь заблокирован (мс)
    private static final long OBSTACLE_POLL_INTERVAL_MS = 100L;
    
//  ==== СОСТОЯНИЕ ПРИЛОЖЕНИЯ 
    private final List<JointPosition> recordedTrajectory =
            new ArrayList<JointPosition>();
    private final AtomicBoolean stopRecordingFlag =
            new AtomicBoolean(false);
    private final AtomicBoolean stopGravCompFlag =
            new AtomicBoolean(false);
    private volatile IMotionContainer activeMotionContainer = null;
    private volatile ISmartServoRuntime gravCompRuntime = null;
    
    private Thread recordingThread = null;
    private Thread gravCompThread  = null;

	@Override
	public void initialize() {
		_gripper.attachTo(_lbr.getFlange());
		
		getLogger().info("  Программа : TeachKuka.java");
		getLogger().info("  Sunrise OS 1.16 | Servoing 1.16");
        getLogger().info("  Робот     : " + _lbr.getName());
        getLogger().info("  Суставов  : " + _lbr.getJointCount());
        getLogger().info("  Контроллер: " + _lbrController.getName());
        getLogger().info("  Захват: " + _gripper.getName() + " -> прикреплён к фланцу");

	}

	@Override
	public void run() throws Exception {

		moveToHomePosition();
		
		boolean running = true;
		
		while (running) {
			int choice = showMainMenu();
			
			switch (choice) {
				case 0:
					positionSnapshot();
					break;
				case 1:
					trajectoryRecord();
					break;
				case 2:
				default:
					running = false;
					break;
			}
		}
		
		getLogger().info("Выход. Возвращаюсь в Home...");
        moveToHomePosition();
        getLogger().info("Программа завершена.");
	}
	
	@Override
	public void dispose() {
		stopRecordingFlag.set(true);	 
		stopGravCompFlag.set(true);
		stopGravCompThread();
		stopRecordingThread(); 
		cancelActiveMotion();
		super.dispose();
	}
	
	private int showMainMenu() {
		return getApplicationUI().displayModalDialog(
			ApplicationDialogType.QUESTION,
			"Выберите режим работы",
			"Режим 1: Позиция",
			"Режим 2: Траектория",
			"Выход"
		);
	}
	
//	Режим 1
	private void positionSnapshot() throws Exception {
        getLogger().info("Вход в Режим 1: Захват позиции");

        moveToWorkingPosition();
        validateLoadModel();

        startGravCompOnTool();

        getLogger().info("Ведите робота рукой. Он останется там где вы его поставите.");

        boolean inMode = true;
        while (inMode) {
            int choice = getApplicationUI().displayModalDialog(
                ApplicationDialogType.INFORMATION,
                "Выбери действие",
                "Получить позицию",
                "Назад"
            );

            if (choice == 0) {
                logCurrentPosition();
            } else {
                inMode = false;
            }
        }
        
        stopGravComp();
        getLogger().info("Режим 1 завершён. Тормоз активирован.");
    }
	
//	Режим 2
	private void trajectoryRecord() throws Exception {
		getLogger().info("Вход в Режим 2: Запись траектории");
		
		moveToWorkingPosition();
		validateLoadModel();
		
		synchronized (recordedTrajectory) {
			recordedTrajectory.clear();
		}
		
		startGravCompOnTool();
		startRecordingThread();
		
		boolean inMode = true;
		while (inMode) {
			
			int choise = getApplicationUI().displayModalDialog(
					ApplicationDialogType.INFORMATION,
					"Выберите действие",
					
					"Повторить траекторию",
					"Рестарт",
					"Назад"
			);
			
			if (choise == 0) {
				stopRecordingThread();
				    stopGravComp();
	
				    List<JointPosition> snapshot;
				    synchronized (recordedTrajectory) {
				        	snapshot = new ArrayList<JointPosition>(recordedTrajectory);
				    }
	
				    if (snapshot.size() < 2) {
				        	getLogger().warn("Траектория слишком короткая (< 2 точек)."
				        			+ " Переместите робота и попробуйте снова.");
				    } else {
				        	replayTrajectory(snapshot);
				    }
				    getLogger().info("Воспроизведение закончено. Начинаю новую запись...");
				    synchronized (recordedTrajectory) {
				    	recordedTrajectory.clear();
				    }
				    validateLoadModel();
				    startGravCompOnTool();
				    startRecordingThread();
			} else if (choise == 1) {
				stopRecordingThread();
				stopGravComp();
				synchronized (recordedTrajectory) {
                    recordedTrajectory.clear();
                }
				
				getLogger().info("Запись сброшена. Начинаю заново...");
				validateLoadModel();
				startGravCompOnTool();
				startRecordingThread();
			} else {
				stopRecordingThread();
				stopGravComp();
				inMode = false;
			}
		}
		
		getLogger().info("Режим 2 завершён. Тормоз активирован.");
	}
	
//	==== Вспомогательные методоы ====
//	Запись данных
    private void startRecordingThread() {
        stopRecordingFlag.set(false);

        recordingThread = new Thread(new Runnable() {
            @Override
            public void run() {
                recordingLoop();
            }
        }, "TrajectoryRecorder");

        recordingThread.setDaemon(true);
        recordingThread.start();

        getLogger().info("[Запись] Запущена. Интервал: "
            + RECORD_INTERVAL_MS + " мс.");
    }
    
    private void recordingLoop() {
        getLogger().info("[Запись] Поток запущен.");

        while (!stopRecordingFlag.get()) {
            try {
                JointPosition pose = _lbr.getCurrentJointPosition();

                synchronized (recordedTrajectory) {
                    if (recordedTrajectory.size() >= MAX_TRAJECTORY_POINTS) {
                        getLogger().warn("[Запись] Лимит "
                            + MAX_TRAJECTORY_POINTS + " точек достигнут."
                            + " Запись остановлена.");
                        break;
                    }
                    recordedTrajectory.add(pose);
                }

                Thread.sleep(RECORD_INTERVAL_MS);

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            } catch (Exception e) {
                getLogger().error("[Запись] Ошибка: " + e.getMessage());
                break;
            }
        }

        getLogger().info("[Запись] Поток остановлен. Точек: "
            + recordedTrajectory.size());
    }
    
    private void stopRecordingThread() {
        stopRecordingFlag.set(true);
        if (recordingThread != null && recordingThread.isAlive()) {
            recordingThread.interrupt();
            try {
                recordingThread.join(2000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            recordingThread = null;
        }
    }
	
//  Воспроизведение траекторий
    private void replayTrajectory(List<JointPosition> trajectory) throws Exception {
        int total = trajectory.size();

        getLogger().info("===========================================");
        getLogger().info("  ВОСПРОИЗВЕДЕНИЕ  |  Точек: " + total
            + "  |  Скорость: " + (int)(REPLAY_VELOCITY_REL * 100) + "%");
        getLogger().info("  Старт через " + (PRE_REPLAY_DELAY_MS / 1000)
            + " сек. Отойдите от робота!");
        getLogger().info("===========================================");

        Thread.sleep(PRE_REPLAY_DELAY_MS);

        getLogger().info("Перемещаюсь к начальной точке...");
        _lbr.move(
            BasicMotions.ptp(trajectory.get(0))
                        .setJointVelocityRel(REPLAY_VELOCITY_REL)
        );
        getLogger().info("Начальная точка достигнута.");

        SmartServo replayServo = new SmartServo(trajectory.get(0));
        replayServo.setMinimumTrajectoryExecutionTime(8e-3);
        replayServo.setTimeoutAfterGoalReach(3600);
        replayServo.setJointVelocityRel(REPLAY_VELOCITY_REL);

        _gripper.getDefaultMotionFrame()
                .moveAsync(replayServo.setMode(new PositionControlMode()));

        ISmartServoRuntime replayRuntime = replayServo.getRuntime();
        getLogger().info("SmartServo активирован. Воспроизведение...");

        int  pauseCount  = 0;
        long totalWaitMs = 0L;

        for (int i = 1; i < total; i++) {

            if (isObstacleDetected(replayRuntime)) {
                pauseCount++;
                int waitCount = 0;
                getLogger().warn("ПРЕПЯТСТВИЕ на точке " + i + "/" + total
                    + ". Удерживаю позицию...");

                while (isObstacleDetected(replayRuntime)) {
                    replayRuntime.setDestination(_lbr.getCurrentJointPosition());
                    Thread.sleep(OBSTACLE_POLL_INTERVAL_MS);
                    waitCount++;
                    totalWaitMs += OBSTACLE_POLL_INTERVAL_MS;
                    if (waitCount % 30 == 0) {
                        getLogger().warn("  Ожидание: "
                            + (waitCount * OBSTACLE_POLL_INTERVAL_MS / 1000)
                            + " сек. Освободите путь.");
                    }
                }
                getLogger().info("Путь свободен. Продолжаю с точки "
                    + i + "/" + total);
            }

            replayRuntime.setDestination(trajectory.get(i));
            Thread.sleep(RECORD_INTERVAL_MS);
        }

        // stopMotion ПОСЛЕ цикла, не внутри него
        replayRuntime.stopMotion();

        getLogger().info("===========================================");
        getLogger().info("  ГОТОВО  |  Точек: " + total
            + "  |  Пауз: " + pauseCount
            + (pauseCount > 0
                ? "  |  Ожидание: " + (totalWaitMs / 1000.0) + " сек"
                : ""));
        getLogger().info("===========================================");
    }
    
//  Проверка наличия контакта с препятсвием
    private boolean isObstacleDetected(ISmartServoRuntime runtime) {
    		try {
    			
    			double[] tauExt = runtime.getAxisTauExtMsr();
    			if (tauExt == null) {
    				return false;
            }
            for (int j = 0; j < tauExt.length; j++) {
            				if (Math.abs(tauExt[j]) > COLLISION_TORQUE_THRESHOLD_NM) {
            					getLogger().info(String.format(
            							"  [Детекция] J%d: tau_ext=%.2f Нм (порог %.1f Нм)",
                            j + 1, tauExt[j], COLLISION_TORQUE_THRESHOLD_NM));
            					return true;
                }
           }
    		} catch (Exception e) {
    			getLogger().warn("[Детекция] Ошибка: " + e.getMessage());
		}
    		
    		return false;
    }
    
//  Компенсация массы робота
	private void startGravCompOnTool() throws InterruptedException {
        CartesianImpedanceControlMode gravComp = createGravityCompMode();

        SmartServo smartServo = new SmartServo(_lbr.getCurrentJointPosition());
        smartServo.setMinimumTrajectoryExecutionTime(8e-3);
        smartServo.setTimeoutAfterGoalReach(3600);
        smartServo.setJointVelocityRel(0.5);

        activeMotionContainer = _gripper
            .getDefaultMotionFrame()
            .moveAsync(smartServo.setMode(gravComp));
        gravCompRuntime = smartServo.getRuntime();

        stopGravCompFlag.set(false);
        gravCompThread = new Thread(new Runnable() {
            @Override
            public void run() {
                gravCompAnchorLoop();
            }
        }, "GravCompAnchorUpdater");
        gravCompThread.setDaemon(true);
        gravCompThread.start();

        getLogger().info("Гравкомпенсация активна. Якорь следует за роботом.");
    }
	
	private void gravCompAnchorLoop() {
        getLogger().info("[GravComp] Поток обновления якоря запущен.");

        while (!stopGravCompFlag.get()) {
            try {
                ISmartServoRuntime rt = gravCompRuntime;
                if (rt != null) {
                    rt.setDestination(_lbr.getCurrentJointPosition());
                }
                Thread.sleep(GRAV_COMP_UPDATE_INTERVAL_MS);

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            } catch (Exception e) {
                getLogger().warn("[GravComp] Ошибка обновления: " + e.getMessage());
                break;
            }
        }

        getLogger().info("[GravComp] Поток обновления остановлен.");
    }
	
	private void stopGravComp() {
        stopGravCompThread();

        ISmartServoRuntime rt = gravCompRuntime;
        if (rt != null) {
            try {
                rt.stopMotion();
                getLogger().info("SmartServo остановлен. Тормоз активирован.");
            } catch (Exception e) {
                getLogger().warn("Ошибка stopMotion(): " + e.getMessage());
                cancelActiveMotion();
            } finally {
                gravCompRuntime = null;
                activeMotionContainer = null;
            }
        }
    }
	
	private void stopGravCompThread() {
        stopGravCompFlag.set(true);
        if (gravCompThread != null && gravCompThread.isAlive()) {
            gravCompThread.interrupt();
            try {
                gravCompThread.join(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            gravCompThread = null;
        }
    }
	
//  Утилиты
	private void logCurrentPosition() {
		getLogger().info("===========================================");
		
		Frame flangeInWorld = _lbr.getCurrentCartesianPosition(_lbr.getFlange());
		
		double x_mm = flangeInWorld.getX();
		double y_mm = flangeInWorld.getY();
		double z_mm = flangeInWorld.getZ();
		double a_deg = Math.toDegrees(flangeInWorld.getAlphaRad());
		double b_deg = Math.toDegrees(flangeInWorld.getBetaRad());
		double c_deg = Math.toDegrees(flangeInWorld.getGammaRad());
		
		getLogger().info(String.format(" XYZ [мм]:  X=%9.3f  Y=%9.3f  Z=%9.3f", x_mm, y_mm, z_mm));
		getLogger().info(String.format(" ABC [мм]:  [ ° ]: A=%9.3f  B=%9.3f  C=%9.3f", a_deg, b_deg, c_deg));
		
		JointPosition joints = _lbr.getCurrentJointPosition();
		int n = _lbr.getJointCount();
		
		StringBuilder rowDeg = new StringBuilder("  Суставы [ ° ]: ");
		StringBuilder rowRad = new StringBuilder("  Суставы [рад]: ");
		
		for (int i=0; i<n; i++) {
			double rad = joints.get(i);
			rowDeg.append(String.format("J%d=%7.2f  ", i + 1, Math.toDegrees(rad)));
			rowRad.append(String.format("J%d=%7.4f  ", i + 1, rad));
		}
		
		getLogger().info(rowDeg.toString());
		getLogger().info(rowRad.toString());
		
		getLogger().info("===========================================");
	}
	
	private CartesianImpedanceControlMode createGravityCompMode() {
        CartesianImpedanceControlMode mode = new CartesianImpedanceControlMode();
        mode.parametrize(CartDOF.TRANSL)
            .setStiffness(GRAV_COMP_STIFFNESS_TRANSL)
            .setDamping(GRAV_COMP_DAMPING);
        mode.parametrize(CartDOF.ROT)
            .setStiffness(GRAV_COMP_STIFFNESS_ROT)
            .setDamping(GRAV_COMP_DAMPING);
        return mode;
    }
	
	private void validateLoadModel() {
        getLogger().info("Валидация динамической модели нагрузки...");

        boolean valid = SmartServo.validateForImpedanceMode(_gripper);

        if (valid) {
            getLogger().info("Динамическая модель нагрузки валидна. "
                + "Гравкомпенсация будет корректной.");
        } else {
            getLogger().warn("ВНИМАНИЕ: Валидация динамической модели "
                + "не прошла! Проверьте load data захвата в "
                + "Sunrise Workbench -> Object Templates -> "
                + _gripper.getName() + " -> Load data.");
            getLogger().warn("Гравкомпенсация может работать некорректно.");
        }
    }
	
	/**
     * Перемещает в Home (все оси = 0
     */
	private void moveToHomePosition() {
		getLogger().info("Перемещение в Home...");
		_lbr.move(
				BasicMotions.ptp(HOME_POSITION_RAD)
							.setJointVelocityRel(APPROACH_VELOCITY_REL)
		);
		getLogger().info("Home достигнута.");
	}
	
//	Перемещает в рабочую позицию через Home 
	private void moveToWorkingPosition() throws Exception {
		getLogger().info("Перемещение в рабочую позицию ...");
		_lbr.move(
				BasicMotions.batch(
						BasicMotions.ptp(HOME_POSITION_RAD)
									.setBlendingRel(HOME_BLENDING_REL),	
						BasicMotions.ptp(WORKING_POSITION_RAD)
				).setJointVelocityRel(APPROACH_VELOCITY_REL)
		);
		
		getLogger().info("Рабочая позиция достигнута");
	}
	
	private void cancelActiveMotion() {
		if (activeMotionContainer != null) {
			try {
				activeMotionContainer.cancel();
				getLogger().info("Движение отменено. Тормоз активирован.");
			} catch (Exception e) {
				getLogger().warn("Ошибка при cancel(): " + e.getMessage());
			} finally {
				activeMotionContainer = null;
			}
		}
	}
	
	
}