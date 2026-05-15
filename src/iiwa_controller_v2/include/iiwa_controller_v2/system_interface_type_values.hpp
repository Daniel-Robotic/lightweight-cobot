#pragma once

namespace iiwa_controller_v2
{

// Per-joint state interface names (beyond standard position/velocity/effort)
constexpr char HW_IF_EXTERNAL_TORQUE[]    = "external_torque";
constexpr char HW_IF_COMMANDED_TORQUE[]   = "commanded_torque";
constexpr char HW_IF_IPO_JOINT_POSITION[] = "ipo_joint_position";

// Auxiliary sensor prefix (used for robot-level telemetry)
constexpr char HW_IF_AUXILIARY_PREFIX[]   = "auxiliary";

// Auxiliary sensor state interface names
constexpr char HW_IF_SAMPLE_TIME[]          = "sample_time";
constexpr char HW_IF_SESSION_STATE[]        = "session_state";
constexpr char HW_IF_CONNECTION_QUALITY[]   = "connection_quality";
constexpr char HW_IF_TIME_STAMP_SEC[]       = "time_stamp_sec";
constexpr char HW_IF_TIME_STAMP_NANO_SEC[]  = "time_stamp_nano_sec";

}  // namespace iiwa_controller_v2
