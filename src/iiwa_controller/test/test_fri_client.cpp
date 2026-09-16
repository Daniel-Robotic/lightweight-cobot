#include <gtest/gtest.h>
#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>
#include "iiwa_controller/FRIClient.h"
#include "iiwa_controller/FRICycleGate.hpp"
#include <atomic>
#include <vector>
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
  explicit ClientFixture(double position_smoothing_tau = 0.04)
  : FRIClient(position_smoothing_tau), app(connection, *this), data(app.data())
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

TEST(FRIClient, PositionCommandsAreSmoothedBeforeSending)
{
  ClientFixture client;
  client.cycle(COMMANDING_WAIT);
  client.cycle(COMMANDING_ACTIVE);
  iiwa_controller::FRIClient::Joints q;
  q.fill(0.41);
  client.setTargetJointPositions(q);
  client.cycle(COMMANDING_ACTIVE);
  // tau=40 ms, dt=10 ms: alpha=dt/(tau+dt)=0.2.
  EXPECT_DOUBLE_EQ(client.sentPosition(), 0.402);
  EXPECT_DOUBLE_EQ(client.getStateSnapshot().measured_pos[0], 0.2);
}

TEST(FRIClient, ZeroPositionSmoothingTauForwardsStepImmediately)
{
  ClientFixture client(0.0);
  client.cycle(COMMANDING_WAIT);
  client.cycle(COMMANDING_ACTIVE);
  iiwa_controller::FRIClient::Joints q;
  q.fill(0.41);
  client.setTargetJointPositions(q);
  client.cycle(COMMANDING_ACTIVE);
  EXPECT_DOUBLE_EQ(client.sentPosition(), 0.41);
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

TEST(FRIClient, RejectsMismatchedPeriodAndMissingCommandingPackets)
{
  ClientFixture mismatch;
  mismatch.setExpectedSampleTime(0.005);
  EXPECT_THROW(mismatch.cycle(MONITORING_READY), std::runtime_error);
  ClientFixture gap;
  gap.cycle(COMMANDING_WAIT);
  gap.data->monitoringMsg.monitorData.timestamp.nanosec += 10000000;
  EXPECT_THROW(gap.cycle(COMMANDING_ACTIVE), std::runtime_error);
  EXPECT_FALSE(gap.data->commandMsg.commandData.has_jointPosition);
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

TEST(FRIClient, RejectsOverwritingAnUnconsumedPoint)
{
  ClientFixture client;
  client.cycle(COMMANDING_WAIT);
  client.cycle(COMMANDING_ACTIVE);
  iiwa_controller::FRIClient::Joints q;
  q.fill(0.401);
  client.setTargetJointPositions(q);
  q.fill(0.402);
  EXPECT_THROW(client.setTargetJointPositions(q), std::logic_error);
  client.cycle(COMMANDING_ACTIVE);
  EXPECT_DOUBLE_EQ(client.sentPosition(), 0.4002);
}

TEST(FRIClient, SynchronizedCyclesPreserveEveryPointDespiteSchedulingJitter)
{
  ClientFixture client;
  client.cycle(COMMANDING_WAIT);
  iiwa_controller::FRICycleGate gate;
  gate.reset();
  std::atomic<bool> worker_ok{true};
  std::vector<double> sent;
  std::thread worker([&] {
    for (int frame = 0; frame <= 8; ++frame) {
      client.cycle(COMMANDING_ACTIVE);
      sent.push_back(client.sentPosition());
      if (frame < 8 && !gate.publishAndWait()) {worker_ok = false; break;}
    }
  });
  for (int frame = 1; frame <= 8; ++frame) {
    if (!gate.acquire()) {worker_ok = false; break;}
    // Variable controller execution time cannot cause a point to be overwritten.
    std::this_thread::sleep_for(std::chrono::milliseconds(frame % 3));
    iiwa_controller::FRIClient::Joints q;
    q.fill(0.4 + frame * 0.001);
    client.setTargetJointPositions(q);
    if (!gate.complete()) {worker_ok = false; break;}
  }
  worker.join();
  ASSERT_TRUE(worker_ok);
  ASSERT_EQ(sent.size(), 9u);
  double expected = 0.4;
  for (size_t frame = 1; frame < sent.size(); ++frame) {
    const double target = 0.4 + static_cast<double>(frame) * 0.001;
    expected = 0.2 * target + 0.8 * expected;
    EXPECT_NEAR(sent[frame], expected, 1e-12);
  }
}

TEST(FRICycleGate, DuplicateReadOrWriteCannotAdvanceWorker)
{
  iiwa_controller::FRICycleGate gate;
  gate.reset();
  EXPECT_FALSE(gate.complete());
  std::thread worker([&] {EXPECT_TRUE(gate.publishAndWait());});
  EXPECT_TRUE(gate.acquire());
  EXPECT_FALSE(gate.acquire());
  EXPECT_TRUE(gate.complete());
  EXPECT_FALSE(gate.complete());
  worker.join();
}

TEST(FRICycleGate, StopWakesWorkerAndMissingWriteTimesOut)
{
  iiwa_controller::FRICycleGate gate;
  gate.reset();
  std::thread worker([&] {EXPECT_FALSE(gate.publishAndWait());});
  EXPECT_TRUE(gate.acquire());
  gate.stop();
  worker.join();
  gate.reset();
  EXPECT_FALSE(gate.publishAndWait());
  EXPECT_FALSE(gate.acquire());
  EXPECT_FALSE(gate.complete());
}
