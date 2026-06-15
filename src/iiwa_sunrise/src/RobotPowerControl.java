package backgroundTask;

import java.io.IOException;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPIBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;
import com.kuka.task.ITaskLogger;

/**
 * Background task: управление питанием контроллера KUKA через кнопки SmartPAD.
 *
 * Панель "System" содержит две safety-critical кнопки:
 *   [0] REBOOT  — запускает D:\Programmes\reboot.cmd
 *   [1] SHUTDOWN — запускает D:\Programmes\shutdown.cmd
 *
 * Защита реализована через setCriticalText():
 *   - 1-е нажатие, smartHMI показывает окно "Critical operation" с текстом предупреждения
 *   - Кнопка деактивируется на ~5 с
 *   - 2-е нажатие в течение 5 с, onKeyEvent(KeyDown) выполняется и скрипт запускается
 *   - Нет нажатия / тап вне окна, окно закрывается, кнопка сбрасывается
 */
public class RobotPowerControl extends RoboticsAPIBackgroundTask {

    @Inject
    private Controller kUKA_Sunrise_Cabinet;

    @Inject
    private ITaskLogger logger;

    private static final String REBOOT_SCRIPT  = "D:\\Programme\\reboot.cmd";
    private static final String SHUTDOWN_SCRIPT = "D:\\Programme\\shutdown.cmd";

    @Override
    public void initialize() {
        // инициализация не требуется
    }

    @Override
    public void run() {
        IUserKeyBar powerBar = getApplicationUI().createUserKeyBar("System");

        // Кнопка REBOOT
        IUserKeyListener rebootListener = new IUserKeyListener() {
            @Override
            public void onKeyEvent(IUserKey key, UserKeyEvent event) {
                if (event == UserKeyEvent.KeyDown) {
                    // Подтверждение через setCriticalText уже получено
                    key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Yellow, UserKeyLEDSize.Small);
                    boolean success = executeScript(REBOOT_SCRIPT);
                    if (!success) {
                        key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Red, UserKeyLEDSize.Small);
                        ThreadUtil.milliSleep(2000);
                    }
                    key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Grey, UserKeyLEDSize.Small);
                }
            }
        };

        //Кнопка SHUTDOWN
        IUserKeyListener shutdownListener = new IUserKeyListener() {
            @Override
            public void onKeyEvent(IUserKey key, UserKeyEvent event) {
                if (event == UserKeyEvent.KeyDown) {
                    key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Yellow, UserKeyLEDSize.Small);
                    boolean success = executeScript(SHUTDOWN_SCRIPT);
                    if (!success) {
                        key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Red, UserKeyLEDSize.Small);
                        ThreadUtil.milliSleep(2000);
                    }
                    key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Grey, UserKeyLEDSize.Small);
                }
            }
        };

        //Регистрация и настройка кнопок
        IUserKey rebootKey = powerBar.addUserKey(0, rebootListener, true);
        rebootKey.setText(UserKeyAlignment.TopLeft, "REBOOT");
        rebootKey.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Grey, UserKeyLEDSize.Small);
        rebootKey.setCriticalText("Controller will REBOOT! Press again to confirm.");

        IUserKey shutdownKey = powerBar.addUserKey(1, shutdownListener, true);
        shutdownKey.setText(UserKeyAlignment.TopLeft, "SHUTDOWN");
        shutdownKey.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Grey, UserKeyLEDSize.Small);
        shutdownKey.setCriticalText("Controller will SHUTDOWN! Press again to confirm.");
        
        powerBar.publish();

        // Держим задачу живой
        while (!Thread.currentThread().isInterrupted()) {
            ThreadUtil.milliSleep(500);
        }
    }

    /**
     * Запускает .cmd-скрипт через cmd.exe.
     * Не ждём завершения (waitFor не вызывается): контроллер уйдёт
     * в reboot/shutdown сам, ожидание привело бы к зависанию задачи.
     *
     * @param scriptPath абсолютный путь к .cmd файлу
     * @return true если процесс запущен успешно, false при IOException
     */
    private boolean executeScript(String scriptPath) {
        try {
            ProcessBuilder pb = new ProcessBuilder("cmd.exe", "/c", scriptPath);
            pb.redirectErrorStream(true);
            pb.start();
            return true;
        } catch (IOException e) {
            logger.error("Failed to execute: " + scriptPath + " | " + e.getMessage());
            return false;
        }
    }
}
