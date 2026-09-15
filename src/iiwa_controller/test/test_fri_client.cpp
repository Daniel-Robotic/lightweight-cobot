#include <gtest/gtest.h>
#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>
#include "iiwa_controller/FRIClient.h"
#include "friClientData.h"
#include "pb_frimessages_callbacks.h"

using namespace KUKA::FRI;

class TestApplication : public ClientApplication
{
public:
  using ClientApplication::ClientApplication;
  ClientData * data() {return _data;}
};

class ClientFixture : public iiwa_controller::FRIClient
{
public:
  ClientFixture() : FRIClient(), app(connection, *this), data(app.data())
  {
    auto & m = data->monitoringMsg;
    m.connectionInfo.sendPeriod = 10;
    m.connectionInfo.quality = FRIConnectionQuality_GOOD;
    m.robotInfo.safetyState = SafetyState_NORMAL_OPERATION;
    m.robotInfo.operationMode = OperationMode_AUTOMATIC_MODE;
    m.robotInfo.controlMode = ControlMode_POSITION_CONTROLMODE;
    auto * drives = static_cast<tRepeatedIntArguments *>(m.robotInfo.driveState.arg);
    std::fill_n(drives->value, 7, DriveState_ACTIVE);
    m.ipoData.clientCommandMode = ClientCommandMode_POSITION;
    m.ipoData.overlayType = OverlayType_JOINT;
    m.ipoData.has_trackingPerformance = false;
    m.monitorData.timestamp.sec = 1;
    m.monitorData.timestamp.nanosec = 0;
    fill(m.monitorData.measuredJointPosition, 0.2);
    fill(m.monitorData.commandedJointPosition, 0.3);
    fill(m.monitorData.measuredTorque, 1.0);
    fill(m.monitorData.externalTorque, 0.1);
    fill(m.ipoData.jointPosition, 0.4);
  }
  static void fill(JointValues & values, double value)
  {
    auto * args = static_cast<tRepeatedDoubleArguments *>(values.value.arg);
    std::fill_n(args->value, 7, value);
    args->size = 7;
  }
  void cycle(ESessionState state)
  {
    data->resetCommandMessage();
    data->monitoringMsg.ipoData.has_jointPosition = state >= COMMANDING_WAIT;
    data->monitoringMsg.connectionInfo.sessionState = static_cast<FRISessionState>(state);
    onStateChange(data->lastState, state);
    data->lastState = state;
    data->monitoringMsg.monitorData.timestamp.nanosec += 10000000;
    if (state == COMMANDING_WAIT) waitForCommand();
    else if (state == COMMANDING_ACTIVE) command();
    else monitor();
  }
  double sentPosition()
  {
    return static_cast<tRepeatedDoubleArguments *>(
      data->commandMsg.commandData.jointPosition.value.arg)->value[0];
  }
  UdpConnection connection;
  TestApplication app;
  ClientData * data;
};

TEST(FRIClient, ReportsMeasuredPositionInCommanding)
{
  ClientFixture client;
  client.cycle(COMMANDING_WAIT);
  EXPECT_DOUBLE_EQ(client.sentPosition(), 0.4);
  EXPECT_DOUBLE_EQ(client.getStateSnapshot().measured_pos[0], 0.2);
  client.cycle(COMMANDING_ACTIVE);
  EXPECT_DOUBLE_EQ(client.getStateSnapshot().measured_pos[0], 0.2);
}

TEST(FRIClient, PreservesSdkMonitoringReplyWithoutIpo)
{
  ClientFixture client;
  client.cycle(MONITORING_READY);
  EXPECT_TRUE(client.data->commandMsg.commandData.has_jointPosition);
  EXPECT_DOUBLE_EQ(client.sentPosition(), 0.3);
  EXPECT_FALSE(client.getStateSnapshot().ipo_valid);
}

TEST(FRIClient, RejectsUnsupportedCommandModeBeforeSending)
{
  ClientFixture client;
  client.data->monitoringMsg.ipoData.clientCommandMode = ClientCommandMode_TORQUE;
  EXPECT_THROW(client.cycle(COMMANDING_WAIT), std::runtime_error);
  EXPECT_FALSE(client.data->commandMsg.commandData.has_jointPosition);
}

TEST(FRIClient, PositionCommandsHaveNoEmaDelay)
{
  ClientFixture client;
  client.cycle(COMMANDING_WAIT);
  client.cycle(COMMANDING_ACTIVE);
  iiwa_controller::FRIClient::Joints q;
  q.fill(0.41);
  client.setTargetJointPositions(q);
  client.cycle(COMMANDING_ACTIVE);
  EXPECT_DOUBLE_EQ(client.sentPosition(), 0.41);
  EXPECT_DOUBLE_EQ(client.getStateSnapshot().measured_pos[0], 0.2);
}

TEST(FRIClient, RejectsInvalidCommandsAndBoundsCommandRate)
{
  ClientFixture client;
  iiwa_controller::FRIClient::Joints lower, upper, velocity, q;
  lower.fill(-1.0); upper.fill(1.0); velocity.fill(1.0);
  client.setLimits(lower, upper, velocity);
  client.cycle(COMMANDING_WAIT);
  client.cycle(COMMANDING_ACTIVE);
  q.fill(0.8);
  client.setTargetJointPositions(q);
  client.cycle(COMMANDING_ACTIVE);
  EXPECT_NEAR(client.sentPosition(), 0.41, 1e-12);  // 1 rad/s * 10 ms
  q[0] = 1.1;
  EXPECT_THROW(client.setTargetJointPositions(q), std::invalid_argument);
  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(client.setTargetJointPositions(q), std::invalid_argument);
}

TEST(FRIClient, SafetyStopCannotSendApplicationCommand)
{
  ClientFixture client;
  client.cycle(COMMANDING_WAIT);
  client.data->monitoringMsg.robotInfo.safetyState = SafetyState_SAFETY_STOP_LEVEL_1;
  EXPECT_THROW(client.cycle(COMMANDING_ACTIVE), std::runtime_error);
  EXPECT_FALSE(client.data->commandMsg.commandData.has_jointPosition);
}

TEST(FRIClient, DuplicateTimestampIsNotFreshTelemetry)
{
  ClientFixture client;
  client.cycle(MONITORING_READY);
  EXPECT_THROW(client.monitor(), std::runtime_error);
}

TEST(FRIClient, LeavingCommandingRequiresExplicitRecovery)
{
  ClientFixture client;
  client.cycle(COMMANDING_WAIT);
  client.cycle(COMMANDING_ACTIVE);
  EXPECT_THROW(client.cycle(MONITORING_READY), std::runtime_error);
}

TEST(FRIClient, PositionOverlaySupportsSunriseImpedanceModes)
{
  for (auto mode : {ControlMode_JOINT_IMPEDANCE_CONTROLMODE,
    ControlMode_CARTESIAN_IMPEDANCE_CONTROLMODE})
  {
    ClientFixture client;
    client.data->monitoringMsg.robotInfo.controlMode = mode;
    EXPECT_NO_THROW(client.cycle(COMMANDING_WAIT));
    EXPECT_DOUBLE_EQ(client.sentPosition(), 0.4);
  }
}

TEST(FRIClient, StalledRosCommandStreamStopsBeforeSending)
{
  ClientFixture client;
  client.cycle(COMMANDING_WAIT);
  client.cycle(COMMANDING_ACTIVE);
  std::this_thread::sleep_for(std::chrono::milliseconds(110));
  EXPECT_THROW(client.cycle(COMMANDING_ACTIVE), std::runtime_error);
  EXPECT_FALSE(client.data->commandMsg.commandData.has_jointPosition);
}
