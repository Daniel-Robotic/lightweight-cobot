// Offline protocol peer, not a dynamics or Sunrise safety simulator.
// Both endpoints are hardcoded loopback addresses. Never accepts a robot IP.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>
#include "FRIMessages.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_frimessages_callbacks.h"

int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  std::ofstream csv(argv[1]);
  csv << "sequence,session,receive_seconds,position,delta\n";
  csv.flush();
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in local{}, remote{};
  local.sin_family = remote.sin_family = AF_INET;
  local.sin_port = remote.sin_port = htons(30283);
  inet_pton(AF_INET, "127.0.0.2", &local.sin_addr);
  inet_pton(AF_INET, "127.0.0.1", &remote.sin_addr);
  if (fd < 0 || bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local))) {
    perror("fake FRI loopback bind"); return 2;
  }
  timeval timeout{0, 20000};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  std::array<double, 7> q{}, zero{}, received{};
  std::array<int64_t, 7> drives{};
  drives.fill(DriveState_ACTIVE);
  tRepeatedDoubleArguments positions{7, 7, q.data()}, zeros{7, 7, zero.data()};
  tRepeatedIntArguments drive_values{7, 7, drives.data()};
  FRIMonitoringMessage m{};
  m.header.messageIdentifier = 0x245142;  // Bundled SDK LBRMONITORMESSAGEID.
  m.has_robotInfo = m.has_monitorData = m.has_connectionInfo = true;
  m.robotInfo.has_numberOfJoints = m.robotInfo.has_safetyState = true;
  m.robotInfo.has_operationMode = m.robotInfo.has_controlMode = true;
  m.robotInfo.numberOfJoints = 7;
  m.robotInfo.safetyState = SafetyState_NORMAL_OPERATION;
  m.robotInfo.operationMode = OperationMode_AUTOMATIC_MODE;
  m.robotInfo.controlMode = ControlMode_POSITION_CONTROLMODE;
  m.robotInfo.driveState.funcs.encode = encode_repeatedInt;
  m.robotInfo.driveState.arg = &drive_values;
  m.connectionInfo.quality = FRIConnectionQuality_GOOD;
  m.connectionInfo.has_sendPeriod = m.connectionInfo.has_receiveMultiplier = true;
  m.connectionInfo.sendPeriod = 10;
  m.connectionInfo.receiveMultiplier = 1;
  auto & md = m.monitorData;
  md.has_timestamp = md.has_measuredJointPosition = md.has_commandedJointPosition = true;
  md.has_measuredTorque = md.has_commandedTorque = md.has_externalTorque = true;
  for (auto * values : {&md.measuredJointPosition, &md.commandedJointPosition, &m.ipoData.jointPosition}) {
    values->value.funcs.encode = encode_repeatedDouble;
    values->value.arg = &positions;
  }
  for (auto * values : {&md.measuredTorque, &md.commandedTorque, &md.externalTorque}) {
    values->value.funcs.encode = encode_repeatedDouble;
    values->value.arg = &zeros;
  }
  m.ipoData.has_jointPosition = m.ipoData.has_clientCommandMode = true;
  m.ipoData.has_overlayType = m.ipoData.has_trackingPerformance = true;
  m.ipoData.clientCommandMode = ClientCommandMode_POSITION;
  m.ipoData.overlayType = OverlayType_JOINT;
  m.ipoData.trackingPerformance = 1.0;
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  auto next = start;
  unsigned replies = 0;
  for (unsigned seq = 0; Clock::now() - start < std::chrono::seconds(45); ++seq) {
    m.header.sequenceCounter = seq;
    m.connectionInfo.sessionState = replies < 20 ? FRISessionState_MONITORING_READY :
      replies < 40 ? FRISessionState_COMMANDING_WAIT : FRISessionState_COMMANDING_ACTIVE;
    m.has_ipoData = replies >= 20;
    md.timestamp.sec = 1 + seq / 100;
    md.timestamp.nanosec = (seq % 100) * 10000000;
    uint8_t buffer[2048];
    auto output = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (!pb_encode(&output, FRIMonitoringMessage_fields, &m)) {return 3;}
    sendto(fd, buffer, output.bytes_written, 0, reinterpret_cast<sockaddr *>(&remote), sizeof(remote));
    auto count = recv(fd, buffer, sizeof(buffer), 0);
    if (count > 0) {
      FRICommandMessage command{};
      tRepeatedDoubleArguments target{0, 7, received.data()};
      command.commandData.jointPosition.value.funcs.decode = decode_repeatedDouble;
      command.commandData.jointPosition.value.arg = &target;
      auto input = pb_istream_from_buffer(buffer, count);
      if (!pb_decode(&input, FRICommandMessage_fields, &command) ||
        !command.has_commandData || !command.commandData.has_jointPosition) {return 4;}
      for (double value : received) {if (!std::isfinite(value)) {return 5;}}
      const double delta = received[0] - q[0];
      csv << seq << ',' << m.connectionInfo.sessionState << ',' <<
        std::chrono::duration<double>(Clock::now() - start).count() << ',' << received[0] << ',' << delta << '\n';
      csv.flush();
      q = received;
      m.header.reflectedSequenceCounter = command.header.sequenceCounter;
      ++replies;
    } else if (replies) {
      std::cerr << "Reply timeout after " << replies << " frames\n";
      close(fd); return 6;
    }
    // Before connection, avoid catch-up bursts after receive timeouts.
    next = replies ? next + std::chrono::milliseconds(10) : Clock::now();
    std::this_thread::sleep_until(next);
  }
  close(fd);
  return 0;
}
