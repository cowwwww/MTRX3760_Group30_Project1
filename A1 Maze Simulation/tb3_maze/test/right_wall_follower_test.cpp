// Direction regressions using the production LiDAR, waypoint and wheel code.
#include "tb3_maze/c_robot.h"

#include <gtest/gtest.h>

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
