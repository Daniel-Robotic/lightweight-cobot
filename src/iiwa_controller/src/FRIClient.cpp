// ============================================================
// FRIClient.cpp
//
// Ключевые решения:
//  1. В waitForCommand() мы «инициализируем» target_pos_ текущей
//     позицией робота, чтобы при переходе в COMMANDING_ACTIVE
//     не было рывка.
//  2. В command() данные читаются/пишутся под мьютексом —
//     ros2_control::write() работает в другом потоке.
//  3. Момент в режиме TORQUE суммируется с gravity compensation
//     робота (setJointPosition — feedforward, addJointTorque — delta).
// ============================================================
#include "iiwa_controller/FRIClient.h"

#include <cstring>   // std::memcpy
#include <rclcpp/rclcpp.hpp>

namespace iiwa_controller
{

    // Вспомогательная функция
    static const char* friStateName(KUKA::FRI::ESessionState s) {
        switch (s) {
            case KUKA::FRI::IDLE: return "IDLE";
            case KUKA::FRI::MONITORING_WAIT: return "MONITORING_WAIT";
            case KUKA::FRI::MONITORING_READY: return "MONITORING_READY";
            case KUKA::FRI::COMMANDING_WAIT: return "COMMANDING_WAIT";
            case KUKA::FRI::COMMANDING_ACTIVE: return "COMMANDING_ACTIVE";
            default: return "UNKNOWN";
        }
    }

    // Конструктор
    FRIClient::FRIClient(CommandMode mode): cmd_mode_(mode) {
        target_pos_.fill(0.0);
        target_tau_.fill(0.0);
        measured_pos_.fill(0.0);
        measured_tau_.fill(0.0);
    }

    // Вспомогательный приватный метод: обновить measured_pos_ и _tau_
    // !!!вызывать только под data_mutex_!!!
    void FRIClient::updateMeasuredState() {
        // getMeasuredJointPosition() возвращает указатель на массив double[7]
        const double* pos_ptr = robotState().getMeasuredJointPosition();
        const double* tau_ptr = robotState().getMeasuredTorque();
        std::memcpy(measured_pos_.data(), pos_ptr, N_JOINTS * sizeof(double));
        std::memcpy(measured_tau_.data(), tau_ptr, N_JOINTS * sizeof(double));
    }

    // monitor() — MONITORING_WAIT / MONITORING_READY
    // Только читаем состояние, команды не отправляем
    void FRIClient::monitor() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        updateMeasuredState();
    }

    // waitForCommand() — COMMANDING_WAIT
    // FRI требует, чтобы в этом состоянии мы всё равно отправляли
    // команду. Отправляем «эхо» текущей позиции — робот не двигается.
    // Заодно инициализируем target_pos_ измеренной позицией, чтобы
    // при входе в COMMANDING_ACTIVE не было скачка.
    void FRIClient::waitForCommand() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        updateMeasuredState();

        // Инициализируем целевую позицию текущей —
        // ros2_control перезапишет её в следующем цикле write()
        target_pos_ = measured_pos_;

        // Отправляем эхо позиции
        robotCommand().setJointPosition(target_pos_.data());
    }

    // command() — COMMANDING_ACTIVE
    // Основной цикл управления. Вызывается каждые send_period мс.
    void FRIClient::command() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        updateMeasuredState();

        if (cmd_mode_ == CommandMode::POSITION) {
            // Режим управления позицией
            // Просто отправляем целевую позицию, записанную из write()
            robotCommand().setJointPosition(target_pos_.data());
        }
        // CommandMode::TORQUE
        else   {
            // Режим управления моментом
            // FRI требует одновременно задавать позицию
            // и дополнительный момент.
            // target_pos_ используется как feedforward (без движения),
            // target_tau_ желаемый дополнительный момент поверх
            // внутреннего регулятора KUKA.
            robotCommand().setJointPosition(target_pos_.data());
            robotCommand().setTorque(target_tau_.data());
        }
    }

    // onStateChange() — уведомление о смене состояния FRI
    void FRIClient::onStateChange(KUKA::FRI::ESessionState oldState,
                                KUKA::FRI::ESessionState newState) {
        session_state_.store(newState, std::memory_order_relaxed);

        RCLCPP_INFO(
        rclcpp::get_logger("FRIClient"),
        "[FRI] Состояние: %s → %s",
        friStateName(oldState),
        friStateName(newState));

        // При потере сессии очищаем целевые команды для безопасности
        if (newState == KUKA::FRI::IDLE ||
            newState == KUKA::FRI::MONITORING_WAIT) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target_tau_.fill(0.0);
        // target_pos_ оставляем — при переподключении нужно знать
        // последнюю «безопасную» позицию
        RCLCPP_WARN(rclcpp::get_logger("FRIClient"),
                    "[FRI] Команды сброшены (сессия неактивна)");
        }
    }

    // Thread-safe setters/getters (вызываются из ros2_control)
    void FRIClient::setTargetJointPositions(
    const std::array<double, N_JOINTS>& q) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target_pos_ = q;
    }

    void FRIClient::setTargetJointTorques(
    const std::array<double, N_JOINTS>& tau) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target_tau_ = tau;
    }

    std::array<double, FRIClient::N_JOINTS>
    FRIClient::getMeasuredJointPositions() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return measured_pos_;
    }

    std::array<double, FRIClient::N_JOINTS>
    FRIClient::getMeasuredTorque() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return measured_tau_;
    }

    bool FRIClient::isCommandingActive() const {
        return session_state_.load(std::memory_order_relaxed) ==
            KUKA::FRI::COMMANDING_ACTIVE;
    }

    KUKA::FRI::ESessionState FRIClient::getSessionState() const {
        return session_state_.load(std::memory_order_relaxed);
    }

}  