#pragma once

#include <friLBRClient.h>
#include <vector>
#include <array>
#include <cstring>

class FRIClient : public KUKA::FRI::LBRClient {
    
    public:
        FRIClient();

        void monitor() override;
        void waitForCommand() override;
        void command() override;
        void onStateChange(KUKA::FRI::ESessionState oldState,
                           KUKA::FRI::ESessionState newState) override;

        std::array<double, 7> getMeasuredJointPositions() const;
        std::array<double, 7> getMeasuredTorque() const;
        
        void setTargetJointPositions(const std::array<double, 7> target_pos);
    
    private:
        std::array<double, 7> measuredJointPositions_;
        std::array<double, 7> measuredTorque_;
        std::array<double, 7> targetJointPositions_;

};
