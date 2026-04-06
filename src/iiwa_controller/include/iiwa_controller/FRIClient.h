// ============================================================
// FRIClient.h
// Низкоуровневый клиент FRI (Fast Robot Interface).
// Наследуется от KUKA::FRI::LBRClient и реализует три
// callback-метода, которые вызывает ClientApplication::step():
//   - monitor()        - только чтение состояния
//   - waitForCommand() - переходный режим, эхо позиции
//   - command()        - управление
// ============================================================
#pragma once

#include <array>
#include <mutex>
#include <atomic>

#include "friLBRClient.h"
#include "friClientApplication.h"
#include "friUdpConnection.h"

namespace iiwa_controller {

    /// Режим управления роботом через FRI
    enum class CommandMode {
        POSITION,   // Управление по позиции суставов [рад]
        TORQUE      // Управление по моментум суставов [Нм]
    };

    class FRIClient : public KUKA::FRI::LBRClient {
        public:
            // Константы
            static constexpr size_t N_JOINTS = 7;  // Число суставов

            // Конструктор, деструктор
            explicit FRIClient(CommandMode mode = CommandMode::POSITION);
            ~FRIClient() override = default;

            // Callbacks, которые вызывает ClientApplication::step()
            // Вызывается в состоянии MONITORING
            void monitor() override;

            // Вызывается в COMMANDING_WAIT: робот ждёт команд.
            void waitForCommand() override;

            // Вызывается в COMMANDING_ACTIVE: основной цикл управления
            void command() override;

            // Уведомление о смене состояния FRI сессии
            void onStateChange(KUKA::FRI::ESessionState oldState,
                                KUKA::FRI::ESessionState newState) override;

            // Thread-safe API для ros2_control (вызывается из read/write)
            // Записать целевую позицию из ros2_control (рад)
            void setTargetJointPositions(const std::array<double, N_JOINTS>& q);

            /// Записать целевой момент (Нм); используется только в режиме TORQUE
            void setTargetJointTorques(const std::array<double, N_JOINTS>& tau);

            /// Получить последнюю измеренную позицию суставов (рад)
            std::array<double, N_JOINTS> getMeasuredJointPositions() const;

            /// Получить последний измеренный момент (Нм)
            std::array<double, N_JOINTS> getMeasuredTorque() const;

            /// Проверить, активен ли FRI в режиме COMMANDING_ACTIVE
            bool isCommandingActive() const;

            /// Получить текущее состояние сессии FRI
            KUKA::FRI::ESessionState getSessionState() const;

        private:
            // Режим управления
            CommandMode cmd_mode_;

            // Состояние FRI сессии
            std::atomic<KUKA::FRI::ESessionState> session_state_{
                KUKA::FRI::IDLE};

            // Данные, защищённые мьютексом
            mutable std::mutex data_mutex_;

            std::array<double, N_JOINTS> target_pos_{};    // Целевая позиция [рад]
            std::array<double, N_JOINTS> target_tau_{};    // Целевой момент [Нм]
            std::array<double, N_JOINTS> measured_pos_{};  // Измеренная позиция
            std::array<double, N_JOINTS> measured_tau_{};  // Измеренный момент

            // Вспомогательные методы
            /// Безопасно скопировать измеренную позицию из robotState() в measured_pos_
            void updateMeasuredState();
    };

}