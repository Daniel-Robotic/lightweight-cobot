#include <gtest/gtest.h>
#include <iiwa_planning/tcp_gizmo_policy.hpp>
using iiwa_planning::GizmoPolicy;
TEST(GizmoPolicy, LeasesAreBarriersAndExpiredCommandsNeverReplay) {
  GizmoPolicy gate;
  EXPECT_TRUE(gate.accept(0, 1., .3));
  EXPECT_TRUE(gate.acquire("planning", 2., 1.));
  EXPECT_EQ(gate.epoch(), 1u);
  EXPECT_FALSE(gate.allowed(1.5, 0., true));
  EXPECT_FALSE(gate.hasTarget());
  EXPECT_TRUE(gate.allowed(3., 0., true));
  EXPECT_FALSE(gate.accept(0, 3., .3));
  EXPECT_TRUE(gate.accept(1, 3., .3));
  EXPECT_TRUE(gate.expired(3.31));
}
TEST(GizmoPolicy, RenewalAndMultipleOwnersHoldThroughPauses) {
  GizmoPolicy gate;
  gate.acquire("a", 2., 0.); gate.acquire("b", 5., 0.);
  gate.acquire("a", 4., 1.);
  EXPECT_EQ(gate.epoch(), 2u);
  gate.release("b");
  EXPECT_FALSE(gate.allowed(4., 100., true));
  EXPECT_TRUE(gate.allowed(5.1, 100., true));
}
TEST(GizmoPolicy, NormalTrajectoryWaitsForSimulationDeadlineAndSettling) {
  GizmoPolicy gate;
  gate.normal(12.);
  EXPECT_FALSE(gate.allowed(100., 11., true));
  EXPECT_FALSE(gate.allowed(100., 12.1, false));
  EXPECT_TRUE(gate.allowed(100., 12.1, true));
  EXPECT_FALSE(gate.hasTarget());
}
TEST(GizmoPolicy, RejectsInvalidLeaseAndBlocksActions) {
  GizmoPolicy gate;
  EXPECT_FALSE(gate.acquire("", 1., 0.));
  EXPECT_FALSE(gate.acquire("x", -1., 0.));
  gate.action(true);
  EXPECT_FALSE(gate.allowed(0., 0., true));
  gate.action(false);
  EXPECT_FALSE(gate.allowed(0., 0., false));
  EXPECT_TRUE(gate.allowed(0., 0., true));
}

TEST(GizmoPolicy, LeaseReleaseRequiresRobotToSettle) {
  GizmoPolicy gate;
  gate.acquire("planning", 2., 0.);
  gate.release("planning");
  EXPECT_FALSE(gate.allowed(1., 1., false));
  EXPECT_TRUE(gate.allowed(1., 1., true));
}

TEST(LowGoalBarrier, DelayedAcceptanceCannotPassHighBarrier) {
  iiwa_planning::LowGoalBarrier barrier;
  EXPECT_TRUE(barrier.begin());
  EXPECT_FALSE(barrier.idle());
  barrier.accepted(true);
  EXPECT_FALSE(barrier.idle());
  // A high-priority request must drain the already accepted low step.
  EXPECT_FALSE(barrier.begin());
  barrier.terminal();
  EXPECT_TRUE(barrier.idle());
}
TEST(LowGoalBarrier, RejectedGoalReleasesBarrierWithoutCancellation) {
  iiwa_planning::LowGoalBarrier barrier;
  EXPECT_TRUE(barrier.begin());
  EXPECT_FALSE(barrier.begin());
  barrier.accepted(false);
  EXPECT_TRUE(barrier.idle());
  EXPECT_TRUE(barrier.begin());
  barrier.accepted(true);
  EXPECT_FALSE(barrier.idle());
  barrier.terminal();
  EXPECT_TRUE(barrier.idle());
}

TEST(GizmoPolicy, ReachedTargetDoesNotRequestServoMotion) {
  EXPECT_FALSE(iiwa_planning::requiresGizmoMotion(0., 0.));
  EXPECT_FALSE(iiwa_planning::requiresGizmoMotion(1e-6, 1e-5));
  EXPECT_TRUE(iiwa_planning::requiresGizmoMotion(1e-3, 0.));
  EXPECT_TRUE(iiwa_planning::requiresGizmoMotion(0., .01));
}
