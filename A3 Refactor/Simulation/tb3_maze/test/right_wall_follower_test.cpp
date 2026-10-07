// Direction and scan-fault regressions for the production maze controller.
#include "tb3_maze/c_right_wall_follower_robot.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadiansPerDegree = kPi / 180.0f;

sensor_msgs::msg::LaserScan BlockedScan(float start = -kPi)
{
    sensor_msgs::msg::LaserScan scan;
    scan.angle_min = start;
    scan.angle_increment = kRadiansPerDegree;
    scan.angle_max = start + 359.0f * scan.angle_increment;
    scan.range_min = 0.01f;
    scan.range_max = 3.5f;
    scan.ranges.assign(360, 0.30f);
    return scan;
}

float BearingDegrees(const sensor_msgs::msg::LaserScan& scan, std::size_t index)
{
    const float angle = scan.angle_min + index * scan.angle_increment;
    return std::atan2(std::sin(angle), std::cos(angle)) / kRadiansPerDegree;
}

void Open(sensor_msgs::msg::LaserScan& scan, float from, float to)
{
    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const float angle = BearingDegrees(scan, i);
        if (angle >= from - 0.01f && angle <= to + 0.01f)
            scan.ranges[i] = std::numeric_limits<float>::infinity();
    }
}

CRightWallFollowerRobot EvaluateScan(const sensor_msgs::msg::LaserScan& scan)
{
    CRightWallFollowerRobot robot;
    const CPose pose{{0.0f, 0.0f}, 0.0f};
    robot.Update(scan, pose);
    return robot;
}

float WaypointBearing(const CRobot& robot)
{
    const auto& point = robot.GetWaypoints().front();
    return std::atan2(point.mY, point.mX) / kRadiansPerDegree;
}

sensor_msgs::msg::LaserScan RightWall(float distance)
{
    auto scan = BlockedScan();
    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const float angle = scan.angle_min + i * scan.angle_increment;
        const float sine = std::sin(angle);
        scan.ranges[i] = sine < -1e-5f ? distance / -sine
                                     : std::numeric_limits<float>::infinity();
        if (scan.ranges[i] > scan.range_max)
            scan.ranges[i] = std::numeric_limits<float>::infinity();
    }
    return scan;
}
}

TEST(RightWallFollower, ChoosesRightOpeningWhenBothSidesAreAvailable)
{
    auto scan = BlockedScan();
    Open(scan, -60.0f, -20.0f);
    Open(scan, 20.0f, 60.0f);
    const auto robot = EvaluateScan(scan);
    ASSERT_FALSE(robot.GetWaypoints().empty());
    EXPECT_LT(WaypointBearing(robot), 0.0f);
    EXPECT_LT(robot.GetAngularVelocity(), 0.0f);
    EXPECT_GT(robot.GetLeftWheelSpeed(), robot.GetRightWheelSpeed());
}

TEST(RightWallFollower, ChoosesRightOpeningForZeroToTwoPiScan)
{
    auto scan = BlockedScan(0.0f);
    Open(scan, -60.0f, -20.0f);
    Open(scan, 20.0f, 60.0f);
    const auto robot = EvaluateScan(scan);
    ASSERT_FALSE(robot.GetWaypoints().empty());
    EXPECT_LT(robot.GetAngularVelocity(), 0.0f);
}

TEST(RightWallFollower, FollowsRightEdgeOfOneWideOpening)
{
    auto scan = BlockedScan();
    Open(scan, -60.0f, 60.0f);
    const auto robot = EvaluateScan(scan);
    ASSERT_FALSE(robot.GetWaypoints().empty());
    EXPECT_LT(WaypointBearing(robot), 0.0f);
    EXPECT_GT(WaypointBearing(robot), -60.0f);
}

TEST(RightWallFollower, KeepsAimInsideNarrowRightOpening)
{
    auto scan = BlockedScan();
    Open(scan, -45.0f, -20.0f);
    const auto robot = EvaluateScan(scan);
    ASSERT_FALSE(robot.GetWaypoints().empty());
    EXPECT_NEAR(WaypointBearing(robot), -32.5f, 1.0f);
}

TEST(RightWallFollower, PrefersRightRearOpeningAtDeadEnd)
{
    auto scan = BlockedScan();
    Open(scan, -160.0f, -100.0f);
    Open(scan, 100.0f, 160.0f);
    const auto robot = EvaluateScan(scan);
    ASSERT_FALSE(robot.GetWaypoints().empty());
    EXPECT_LT(robot.GetWaypoints().front().mX, 0.0f);
    EXPECT_LT(robot.GetWaypoints().front().mY, 0.0f);
    EXPECT_LT(robot.GetAngularVelocity(), 0.0f);
    EXPECT_NEAR(robot.GetLinearVelocity(), 0.0f, 1e-6f);
}

TEST(RightWallFollower, TakesLeftPassageWhenItIsTheOnlyPassage)
{
    auto scan = BlockedScan();
    Open(scan, 20.0f, 60.0f);
    const auto robot = EvaluateScan(scan);
    ASSERT_FALSE(robot.GetWaypoints().empty());
    EXPECT_GT(robot.GetAngularVelocity(), 0.0f);
}

TEST(RightWallFollower, CanTurnLeftWhenOnlyRearLeftIsOpen)
{
    auto scan = BlockedScan();
    Open(scan, 100.0f, 160.0f);
    const auto robot = EvaluateScan(scan);
    ASSERT_FALSE(robot.GetWaypoints().empty());
    EXPECT_GT(robot.GetAngularVelocity(), 0.0f);
}

TEST(RightWallFollower, CorrectsAwayFromCloseRightWallAndTowardsDistantRightWall)
{
    const auto close = EvaluateScan(RightWall(0.15f));
    const auto distant = EvaluateScan(RightWall(0.50f));
    ASSERT_FALSE(close.GetWaypoints().empty());
    ASSERT_FALSE(distant.GetWaypoints().empty());
    EXPECT_GT(close.GetAngularVelocity(), 0.0f);
    EXPECT_LT(distant.GetAngularVelocity(), 0.0f);
}

TEST(RightWallFollower, StopsAndClearsWaypointsWhenAllRangesAreInvalid)
{
    const float invalid[] = {std::numeric_limits<float>::quiet_NaN(), 0.0f,
        -std::numeric_limits<float>::infinity(), -1.0f, 0.005f, 4.0f};
    for (float value : invalid)
    {
        SCOPED_TRACE(value);
        CRightWallFollowerRobot robot;
        auto scan = RightWall(0.30f);
        const CPose pose{{0.0f, 0.0f}, 0.0f};
        robot.Update(scan, pose);
        ASSERT_GT(robot.GetLinearVelocity(), 0.0f);
        std::fill(scan.ranges.begin(), scan.ranges.end(), value);
        robot.Update(scan, pose);
        EXPECT_FLOAT_EQ(robot.GetLeftWheelSpeed(), 0.0f);
        EXPECT_FLOAT_EQ(robot.GetRightWheelSpeed(), 0.0f);
        EXPECT_TRUE(robot.GetWaypoints().empty());
    }
}

TEST(RightWallFollower, DoesNotSearchOnAnInvalidStartupScan)
{
    auto scan = BlockedScan();
    std::fill(scan.ranges.begin(), scan.ranges.end(), std::numeric_limits<float>::quiet_NaN());
    const auto robot = EvaluateScan(scan);
    EXPECT_FLOAT_EQ(robot.GetLinearVelocity(), 0.0f);
    EXPECT_FLOAT_EQ(robot.GetAngularVelocity(), 0.0f);
    EXPECT_TRUE(robot.GetWaypoints().empty());
}

TEST(RightWallFollower, StopsWhenFrontDataIsUnusableDespiteValidSideData)
{
    CRightWallFollowerRobot robot;
    auto scan = RightWall(0.30f);
    const CPose pose{{0.0f, 0.0f}, 0.0f};
    robot.Update(scan, pose);
    ASSERT_GT(robot.GetLinearVelocity(), 0.0f);
    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
        if (std::fabs(BearingDegrees(scan, i)) <= 21.0f)
            scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
    robot.Update(scan, pose);
    EXPECT_FLOAT_EQ(robot.GetLinearVelocity(), 0.0f);
    EXPECT_FLOAT_EQ(robot.GetAngularVelocity(), 0.0f);
    EXPECT_TRUE(robot.GetWaypoints().empty());
}

TEST(RightWallFollower, OneUsableFrontRayDoesNotMaskADeadSector)
{
    auto scan = RightWall(0.30f);
    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
        if (std::fabs(BearingDegrees(scan, i)) <= 21.0f)
            scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
    scan.ranges[180] = std::numeric_limits<float>::infinity();
    const auto robot = EvaluateScan(scan);
    EXPECT_FLOAT_EQ(robot.GetLinearVelocity(), 0.0f);
    EXPECT_FLOAT_EQ(robot.GetAngularVelocity(), 0.0f);
}

TEST(RightWallFollower, StopsOnMissingOrOneSidedFrontCoverage)
{
    for (float start : {-kPi, 0.0f})
    {
        SCOPED_TRACE(start);
        auto scan = BlockedScan(start);
        scan.ranges.resize(120);
        std::fill(scan.ranges.begin(), scan.ranges.end(), std::numeric_limits<float>::infinity());
        const auto robot = EvaluateScan(scan);
        EXPECT_FLOAT_EQ(robot.GetLinearVelocity(), 0.0f);
        EXPECT_FLOAT_EQ(robot.GetAngularVelocity(), 0.0f);
        EXPECT_TRUE(robot.GetWaypoints().empty());
    }
}

TEST(RightWallFollower, StopsAndForgetsTheTargetOnEmptyOrMalformedScans)
{
    for (int variant = 0; variant < 8; ++variant)
    {
        SCOPED_TRACE(variant);
        CRightWallFollowerRobot robot;
        auto scan = RightWall(0.30f);
        const CPose pose{{0.0f, 0.0f}, 0.0f};
        robot.Update(scan, pose);
        ASSERT_GT(robot.GetLinearVelocity(), 0.0f);
        if (variant == 0) scan.ranges.clear();
        if (variant == 1) scan.angle_min = std::numeric_limits<float>::quiet_NaN();
        if (variant == 2) scan.angle_increment = 0.0f;
        if (variant == 3) scan.angle_increment = std::numeric_limits<float>::infinity();
        if (variant == 4) scan.range_min = -1.0f;
        if (variant == 5) scan.range_max = scan.range_min;
        if (variant == 6) scan.range_max = std::numeric_limits<float>::infinity();
        if (variant == 7) scan.range_max = std::numeric_limits<float>::quiet_NaN();
        robot.Update(scan, pose);
        EXPECT_FLOAT_EQ(robot.GetLinearVelocity(), 0.0f);
        EXPECT_FLOAT_EQ(robot.GetAngularVelocity(), 0.0f);
        EXPECT_TRUE(robot.GetWaypoints().empty());
    }
}

TEST(RightWallFollower, TreatsPositiveInfinityAsValidOpenSpace)
{
    auto scan = BlockedScan();
    std::fill(scan.ranges.begin(), scan.ranges.end(), std::numeric_limits<float>::infinity());
    const auto robot = EvaluateScan(scan);
    EXPECT_GT(robot.GetLinearVelocity(), 0.0f);
    EXPECT_LT(robot.GetAngularVelocity(), 0.0f);
    EXPECT_FALSE(robot.GetWaypoints().empty());
}

TEST(RightWallFollower, ToleratesAnIsolatedFrontDropout)
{
    auto scan = RightWall(0.30f);
    scan.ranges[180] = std::numeric_limits<float>::quiet_NaN();
    const auto robot = EvaluateScan(scan);
    EXPECT_GT(robot.GetLinearVelocity(), 0.0f);
    EXPECT_FALSE(robot.GetWaypoints().empty());
}

TEST(RightWallFollower, RecoveryChoosesAFreshWaypointFromCurrentScanAndPose)
{
    CRightWallFollowerRobot robot;
    const CPose initialPose{{0.0f, 0.0f}, 0.0f};
    const auto initialScan = RightWall(0.30f);
    robot.Update(initialScan, initialPose);
    ASSERT_FALSE(robot.GetWaypoints().empty());
    const CPoint oldTarget = robot.GetWaypoints().front();
    auto invalidScan = initialScan;
    std::fill(invalidScan.ranges.begin(), invalidScan.ranges.end(),
              std::numeric_limits<float>::quiet_NaN());
    robot.Update(invalidScan, initialPose);
    EXPECT_TRUE(robot.GetWaypoints().empty());

    const CPose recoveredPose{{0.05f, 0.0f}, 0.0f};
    const auto recoveredScan = RightWall(0.28f);
    CRightWallFollowerRobot fresh;
    fresh.Update(recoveredScan, recoveredPose);
    ASSERT_FALSE(fresh.GetWaypoints().empty());
    const CPoint expected = fresh.GetWaypoints().front();
    // This change is smaller than normal waypoint replacement hysteresis;
    // retaining the pre-fault target would therefore produce the wrong result.
    ASSERT_LT(std::hypot(expected.mX - oldTarget.mX, expected.mY - oldTarget.mY), 0.25f);
    robot.Update(recoveredScan, recoveredPose);
    ASSERT_EQ(robot.GetWaypoints().size(), 1u);
    EXPECT_FLOAT_EQ(robot.GetWaypoints().front().mX, expected.mX);
    EXPECT_FLOAT_EQ(robot.GetWaypoints().front().mY, expected.mY);
    EXPECT_FLOAT_EQ(robot.GetLinearVelocity(), fresh.GetLinearVelocity());
    EXPECT_FLOAT_EQ(robot.GetAngularVelocity(), fresh.GetAngularVelocity());
    EXPECT_GT(robot.GetLinearVelocity(), 0.0f);
}
