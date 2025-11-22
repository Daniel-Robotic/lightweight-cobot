#include "iiwa_controller/FRIClient.h"
#include "rclcpp/rclcpp.hpp"

using namespace KUKA::FRI;

inline const char* to_string(KUKA::FRI::ESessionState s)
{
  using namespace KUKA::FRI;
  switch (s)
  {
    case IDLE:               return "IDLE";
    case MONITORING_WAIT:    return "MONITORING_WAIT";
    case MONITORING_READY:   return "MONITORING_READY";
    case COMMANDING_WAIT:    return "COMMANDING_WAIT";
    case COMMANDING_ACTIVE:  return "COMMANDING_ACTIVE";
    default:                 return "UNKNOWN";
  }
}

FRIClient::FRIClient() {
    targetJointPositions_.fill(0.0);
    measuredJointPositions_.fill(0.0);
    measuredTorque_.fill(0.0);
};

void FRIClient::monitor()
{
  std::memcpy(measuredJointPositions_.data(),
              robotState().getMeasuredJointPosition(),
              7 * sizeof(double));

  std::memcpy(measuredTorque_.data(),
              robotState().getMeasuredTorque(),
              7 * sizeof(double));
}

void FRIClient::setTargetJointPositions(const std::array<double, 7> target_pos) {
    targetJointPositions_ = target_pos;
}

std::array<double, 7> FRIClient::getMeasuredJointPositions() const {
    return measuredJointPositions_;
}

std::array<double, 7> FRIClient::getMeasuredTorque() const {
    return measuredTorque_;
}

void FRIClient::onStateChange(ESessionState oldState, ESessionState newState) {
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("FRIClient"),
            "[FRI Client] FRI state: " << to_string(oldState) << " --> " << to_string(newState));
}


void FRIClient::waitForCommand()
{
    std::memcpy(targetJointPositions_.data(),
              robotState().getMeasuredJointPosition(),
              7 * sizeof(double));
    
    std::memcpy(measuredJointPositions_.data(),
                robotState().getMeasuredJointPosition(),
              7 * sizeof(double));

    std::memcpy(measuredTorque_.data(),
              robotState().getMeasuredTorque(),
              7 * sizeof(double));

  robotCommand().setJointPosition(targetJointPositions_.data());
}

void FRIClient::command() {
    std::memcpy(measuredJointPositions_.data(),
                robotState().getMeasuredJointPosition(),
                7 * sizeof(double));

    robotCommand().setJointPosition(targetJointPositions_.data());
}