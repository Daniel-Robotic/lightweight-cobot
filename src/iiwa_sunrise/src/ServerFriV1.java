package ros;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.generated.ioAccess.IO_groupIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.sunrise.ISunriseRequestService;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.SSR;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.SSRFactory;
import com.kuka.roboticsAPI.controllerModel.sunrise.connectionLib.Message;
import com.kuka.roboticsAPI.controllerModel.sunrise.positionMastering.PositionMastering;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.OperationMode;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;


public class ServerFriV1 extends RoboticsAPIApplication {
	
//	members
	private LBR _lbr;
	private Controller _lbr_controller;
	
	@Inject
	protected IO_groupIOGroup myMediaFlange;
	
//	Переменные для калибровки
	private final static double sideOffset = Math.toRadians(5);       // offset in radians for side motion
    private static double joggingVelocity = 0.2;                      // relative velocity
    private final static int axisId[] = {0, 1, 2, 3, 4, 5, 6};        // axes to be referenced
    private final static int GMS_REFERENCING_COMMAND = 2;             // safety command for GMS referencing
    private final static int COMMAND_SUCCESSFUL = 1;
    private int positionCounter = 0;
	
//	Переменные для задавния вопроса
	private enum CONTROL_MODE {
		POSITION_CONTROL,
		JOINT_IMPEDANCE_CONTROL,
		CARTESIAN_IMPEDANCE_CONTROL;
	}
	public static String[] getNames(Class<? extends Enum<?>> e) {
	    return Arrays.toString(e.getEnumConstants()).replaceAll("^.|.$", "").split(", ");
	}
	
//	FRI параметры
	private String _client_name;
	private int _send_period;
	private int _calib_id = 0;
	private JointImpedanceControlMode _control_mode = new JointImpedanceControlMode(1000., 1000., 1000., 1000., 1000., 1000., 1000.);
	
	//	TODO: Не реализовано
//	private String[] _control_modes = getNames(CONTROL_MODE.class);
	
	
	
	private FRIConfiguration _fri_configuration;
	private FRISession _fri_session;
	private FRIJointOverlay _fri_overlay;

	
	@Override
	public void initialize() {
		_lbr_controller = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbr_controller.getDevices().toArray()[0];
        
        requestUserConfig();
        configureFRI();
	}	
	
	@Override
	public void run() throws Exception {
		PositionHold posHold = new PositionHold(_control_mode, -1, TimeUnit.SECONDS);
		getLogger().info("Робот готов управлению из ROS2");
		_lbr.move(posHold.addMotionOverlay(_fri_overlay));
		
		_fri_session.close();
	}
	
	// Дополнительный функционал
	
	private void requestUserConfig() {
//		Время отпраки сообщений
		String[] _send_periods = {"1", "2", "5", "10"};
		int id = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, 
													   "Выберите желаемый период отправки по сообщений [мс]", 
													   _send_periods);
		_send_period = Integer.valueOf(_send_periods[id]);
//		_send_period = 10;
		
//		Выбор сервера
		HashMap<String, String> _client_names = new HashMap<String, String>() {{
			put("RNF Server", "192.168.21.1");
			put("Notebook KONI", "192.170.10.10");
			put("Notebook", "192.168.21.31");
		}};
		ArrayList<String> keyList = new ArrayList<String>(_client_names.keySet());
		String[] keyArray = keyList.toArray(new String[0]);
		int id1 = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, 
													"Выберите имя вашего сервера:", 
													keyArray);
		_client_name = _client_names.get(keyArray[id1]);
		
//		Привзяка положения датчика
//		String[] calib_button = {"Нет", "Да"};
//		_calib_id = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, 
//				"Выполнить калибровку осей?", calib_button);
		
//		TODO: Добавить режим выбора контроллера
	}
	
	private void configureFRI(){
		_fri_configuration = FRIConfiguration.createRemoteConfiguration(_lbr, _client_name);
		_fri_configuration.setSendPeriodMilliSec(_send_period);
		_fri_configuration.registerIO(myMediaFlange.getOutput("Out_3"));
		_fri_configuration.registerIO(myMediaFlange.getOutput("Out_4"));
		
		getLogger().info(myMediaFlange.toString());
		
		
		getLogger().info("Создание подключения FRI к серверу: " + _fri_configuration.getHostName() + " по порту " + _fri_configuration.getPortOnRemote());
		getLogger().info("Скорость отправки: " + _fri_configuration.getSendPeriodMilliSec() + "мс.");
		getLogger().info("Скорость получения: " + _fri_configuration.getReceiveMultiplier());
		
		_fri_session = new FRISession(_fri_configuration);
		_fri_overlay = new FRIJointOverlay(_fri_session, ClientCommandMode.POSITION); 
	
		
		try {
			_fri_session.await(10, TimeUnit.SECONDS);
			
			getLogger().info("Перемещение в стартовое положение...");
			moveToInitialPosition();
			
			if (_calib_id == 1) {
				getLogger().info("Выполняется привязка датчика положения...");
				calibration_robot();
			}
			
		} catch(final TimeoutException e) {
			getLogger().error(e.getLocalizedMessage());
			return;
		}
		
		getLogger().info("Соединение FRI успешно установлено");
		
	}
	
	private void moveToInitialPosition() {
		_lbr.move(BasicMotions.batch(
				BasicMotions.ptp(0., 0., 0., 0., 0., 0., 0.)
//				TODO: Вернуть перемещение в базовое положение робота
//				BasicMotions.ptp(0., 0., 0., 0., 0., 0., 0.).setBlendingRel(0.5),
//				BasicMotions.ptp(0.757818236045123,0.698595725843854,-1.32172941537282,-2.04506898499393,0.456059978288825,-0.53869880645079,0.295750371688418)
//				BasicMotions.ptp(0., 0., 0., -toRadians(80), 0., toRadians(90), 0)
		).setJointVelocityRel(0.5));
	}
	
	private double toRadians(double angle) {
		return Math.PI / 180 * angle;
	}
	
	void calibration_robot() {
	    
	    PositionMastering mastering = new PositionMastering(_lbr);

        boolean allAxesMastered = true;
        for (int i = 0; i < axisId.length; ++i)
        {
            boolean isMastered = mastering.getMasteringInfo("NO_TOOL").getMasteringState(i);
            if (!isMastered)
            {
                getLogger().warn("Axis with axisId " + axisId[i] + " is not mastered, therefore it cannot be referenced");
            }
            
            allAxesMastered &= isMastered;
        }
        
       
        if (OperationMode.T1 == _lbr.getOperationMode())
        {
            joggingVelocity = 0.4;
        }
        
        if (allAxesMastered)
        {
            getLogger().info("Perform position and GMS referencing with 5 positions");
            
            getLogger().info("Moving to home position");
            _lbr.move(ptpHome().setJointVelocityRel(joggingVelocity));


            performMotion(new JointPosition(Math.toRadians(0.0),
                                            Math.toRadians(16.18),
                                            Math.toRadians(23.04),
                                            Math.toRadians(37.35),
                                            Math.toRadians(-67.93),
                                            Math.toRadians(38.14),
                                            Math.toRadians(-2.13)));
            
            performMotion(new JointPosition(Math.toRadians(18.51),
                                            Math.toRadians(9.08),
                                            Math.toRadians(-1.90),
                                            Math.toRadians(49.58),
                                            Math.toRadians(-2.92),
                                            Math.toRadians(18.60),
                                            Math.toRadians(-31.18)));

            performMotion(new JointPosition(Math.toRadians(-18.53),
                                            Math.toRadians(-25.76),
                                            Math.toRadians(-47.03),
                                            Math.toRadians(-49.55),
                                            Math.toRadians(30.76),
                                            Math.toRadians(-30.73),
                                            Math.toRadians(20.11)));

            performMotion(new JointPosition(Math.toRadians(-48.66),
                                            Math.toRadians(24.68),
                                            Math.toRadians(-11.52),
                                            Math.toRadians(10.48),
                                            Math.toRadians(-11.38),
                                            Math.toRadians(-20.70),
                                            Math.toRadians(20.87)));

            performMotion(new JointPosition(Math.toRadians(9.01),
                                            Math.toRadians(-35.00),
                                            Math.toRadians(24.72),
                                            Math.toRadians(-82.04),
                                            Math.toRadians(14.65),
                                            Math.toRadians(-29.95),
                                            Math.toRadians(1.57)));
            
            getLogger().info("Moving to home position");
            _lbr.move(ptpHome().setJointVelocityRel(joggingVelocity));
        }
	}
	
	private void performMotion(JointPosition position)
    {
        getLogger().info("Moving to position #" + (++positionCounter));

        PTP mainMotion = new PTP(position).setJointVelocityRel(joggingVelocity);
        _lbr.move(mainMotion);

        getLogger().info("Moving to current position from negative direction");
        JointPosition position1 = new JointPosition(_lbr.getJointCount());
        for (int i = 0; i < _lbr.getJointCount(); ++i)
        {
            position1.set(i, position.get(i) - sideOffset);
        }
        PTP motion1 = new PTP(position1).setJointVelocityRel(joggingVelocity);
        _lbr.move(motion1);
        _lbr.move(mainMotion);

        // Wait a little to reduce robot vibration after stop.
        ThreadUtil.milliSleep(2500);
        
        // Send the command to safety to trigger the measurement
        sendSafetyCommand();

        getLogger().info("Moving to current position from positive direction");
        JointPosition position2 = new JointPosition(_lbr.getJointCount());
        for (int i = 0; i < _lbr.getJointCount(); ++i)
        {
            position2.set(i, position.get(i) + sideOffset);
        }
        PTP motion2 = new PTP(position2).setJointVelocityRel(joggingVelocity);
        _lbr.move(motion2);
        _lbr.move(mainMotion);

        // Wait a little to reduce robot vibration after stop
        ThreadUtil.milliSleep(2500);
        
        // Send the command to safety to trigger the measurement
        sendSafetyCommand();
    }
    
    private void sendSafetyCommand()
    {
        ISunriseRequestService requestService = (ISunriseRequestService) (_lbr_controller.getRequestService());
        SSR ssr = SSRFactory.createSafetyCommandSSR(GMS_REFERENCING_COMMAND);
        Message response = requestService.sendSynchronousSSR(ssr);
        int result = response.getParamInt(0);
        if (COMMAND_SUCCESSFUL != result)
        {
            getLogger().warn("Command did not execute successfully, response = " + result);
        }
    }
	
	@Override
	public void dispose() {
		// close connection
		getLogger().info("Disposing FRI session.");
		_fri_session.close();

		super.dispose();
	}
	
	
	
}
