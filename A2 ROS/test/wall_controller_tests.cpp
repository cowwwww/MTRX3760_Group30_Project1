// Hardware-free contracts for the production scan processor and controller.
#include "project1/WallFollower.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

namespace
{
const double pi = 3.14159265358979323846;
using project1::LaserScan;
using project1::Settings;
using project1::Velocity;
using project1::WallFollower;

void Require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

void Zero(const Velocity& command)
{
    Require(command.linear == 0.0 && command.angular == 0.0, "Expected zero motion");
}

// Ray intersections with a right wall and optional front wall. The wall's
// tangent heading and laser mounting yaw are independent physical inputs.
LaserScan Scene(double distance = 0.30, double front = INFINITY,
                double start = -pi, double increment = pi / 180.0,
                double laserYaw = 0.0, double wallHeading = 0.0)
{
    LaserScan scan;
    scan.angleMin = start;
    scan.angleIncrement = increment;
    scan.rangeMin = 0.12;
    scan.rangeMax = 3.5;
    for (int index = 0; index < 360; ++index)
    {
        const double angle = start + index * increment + laserYaw;
        const double sideDenominator = std::sin(wallHeading - angle);
        double range = sideDenominator > 1e-9 ? distance / sideDenominator : INFINITY;
        if (std::cos(angle) > 1e-9) range = std::min(range, front / std::cos(angle));
        scan.ranges.push_back(static_cast<float>(range <= scan.rangeMax ? range : INFINITY));
    }
    return scan;
}

Velocity CommandFor(const LaserScan& scan)
{
    WallFollower controller;
    controller.UpdateScan(scan, 1.0);
    return controller.Command(1.05);
}

void DefaultSettings() { Settings().Validate(); }

void InvalidSettings()
{
    double Settings::* fields[] = {&Settings::wallDistance, &Settings::forwardSpeed,
        &Settings::turnSpeed, &Settings::searchSpeed, &Settings::searchTurnSpeed,
        &Settings::frontStop, &Settings::frontRelease, &Settings::lostWall,
        &Settings::distanceGain, &Settings::headingGain, &Settings::scanTimeout};
    const double invalid[] = {0.0, -1.0, INFINITY, NAN};
    for (auto field : fields) for (double value : invalid)
    {
        Settings settings;
        settings.*field = value;
        bool rejected = false;
        try { settings.Validate(); } catch (const std::invalid_argument&) { rejected = true; }
        Require(rejected, "Invalid setting accepted");
    }
    Settings settings;
    settings.laserYaw = INFINITY;
    bool rejected = false;
    try { settings.Validate(); } catch (const std::invalid_argument&) { rejected = true; }
    Require(rejected, "Nonfinite yaw accepted");
}

void InvalidOrdering()
{
    for (int variant = 0; variant < 4; ++variant)
    {
        Settings settings;
        if (variant == 0) settings.frontRelease = settings.frontStop;
        if (variant == 1) settings.lostWall = settings.wallDistance;
        if (variant == 2) settings.searchSpeed = settings.forwardSpeed + 0.01;
        if (variant == 3) settings.searchTurnSpeed = settings.turnSpeed + 0.01;
        bool rejected = false;
        try { settings.Validate(); } catch (const std::invalid_argument&) { rejected = true; }
        Require(rejected, "Invalid threshold/speed ordering accepted");
    }
}

void NoScan() { Zero(WallFollower().Command(1.0)); }
void StraightWall()
{
    const auto command = CommandFor(Scene());
    Require(command.linear > 0.09 && std::fabs(command.angular) < 0.01, "Straight tracking incorrect");
}
void CloseWall() { Require(CommandFor(Scene(0.25)).angular > 0.0, "Must turn away from close right wall"); }
void DistantWall() { Require(CommandFor(Scene(0.50)).angular < 0.0, "Must turn towards distant right wall"); }
void WallHeading()
{
    Require(CommandFor(Scene(0.30, INFINITY, -pi, pi / 180.0, 0.0, 0.10)).angular > 0.0,
            "Must align with leftward wall heading");
    Require(CommandFor(Scene(0.30, INFINITY, -pi, pi / 180.0, 0.0, -0.10)).angular < 0.0,
            "Must align with rightward wall heading");
}
void LostWall()
{
    const auto command = CommandFor(Scene(1.0));
    Require(command.linear > 0.0 && command.linear < Settings().forwardSpeed && command.angular < 0.0,
            "Lost right wall must trigger slow right search");
}
void BlockedFrontPriority()
{
    const auto command = CommandFor(Scene(1.0, 0.25));
    Require(command.linear == 0.0 && command.angular > 0.0, "Obstacle avoidance must precede wall search");
}
void CornerHysteresis()
{
    WallFollower controller;
    controller.UpdateScan(Scene(0.30, 0.35), 1.0);
    Require(controller.Command(1.0).linear == 0.0, "Close front must stop forward motion");
    controller.UpdateScan(Scene(0.30, 0.45), 1.1);
    Require(controller.Command(1.1).linear == 0.0 && controller.GetState() == WallFollower::State::TurnLeft,
            "Pivot must persist between stop/release thresholds");
    controller.UpdateScan(Scene(0.30, 0.55), 1.2);
    Require(controller.Command(1.2).linear > 0.0 && controller.GetState() == WallFollower::State::FollowWall,
            "Wall following must resume after front clearance");
}
void StaleScan()
{
    WallFollower controller;
    controller.UpdateScan(Scene(), 1.0);
    Require(controller.Command(1.1).linear > 0.0, "Fresh scan should drive");
    Zero(controller.Command(1.6));
    controller.UpdateScan(Scene(), 2.0);
    Require(controller.Command(2.1).linear > 0.0, "New scan should restore motion");
}
void TimeReversal()
{
    WallFollower controller;
    controller.UpdateScan(Scene(), 1.0);
    Zero(controller.Command(0.9));
    Zero(controller.Command(NAN));
    Zero(controller.Command(INFINITY));
}
void InvalidScanTime()
{
    WallFollower controller;
    controller.UpdateScan(Scene(), NAN);
    Zero(controller.Command(1.0));
}

void SetSector(LaserScan& scan, double centre, double halfWidth, float value)
{
    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const double angle = scan.angleMin + i * scan.angleIncrement;
        const double delta = std::atan2(std::sin(angle - centre), std::cos(angle - centre));
        if (std::fabs(delta) <= halfWidth + 1e-6) scan.ranges[i] = value;
    }
}
void CorruptSector()
{
    WallFollower controller;
    controller.UpdateScan(Scene(), 1.0);
    Require(controller.Command(1.0).linear > 0.0, "Initial scan should drive");
    auto scan = Scene();
    SetSector(scan, -pi / 2.0, pi / 180.0 * 5.0, NAN);
    controller.UpdateScan(scan, 1.1);
    Zero(controller.Command(1.1));
}
void IsolatedDropout()
{
    auto scan = Scene();
    scan.ranges[90] = NAN;
    Require(CommandFor(scan).linear > 0.0, "One missing side ray should not invalidate whole sector");
}
void FrontMinimum()
{
    auto scan = Scene();
    scan.ranges[180] = 0.15f;
    const auto command = CommandFor(scan);
    Require(command.linear == 0.0 && command.angular > 0.0, "Nearest front obstacle must trigger pivot");
}
void SideMedian()
{
    auto scan = Scene();
    scan.ranges[90] = 0.14f;
    const auto command = CommandFor(scan);
    Require(command.linear > 0.09 && std::fabs(command.angular) < 0.02, "Single side outlier should not dominate median");
}
void NoReturns()
{
    auto scan = Scene();
    for (auto& range : scan.ranges) range = INFINITY;
    const auto command = CommandFor(scan);
    Require(command.linear > 0.0 && command.angular < 0.0, "Positive infinity should mean clear space/right search");
    for (auto& range : scan.ranges) range = -INFINITY;
    Zero(CommandFor(scan));
}
void MalformedMetadata()
{
    for (int variant = 0; variant < 7; ++variant)
    {
        auto scan = Scene();
        if (variant == 0) scan.ranges.clear();
        if (variant == 1) scan.angleMin = NAN;
        if (variant == 2) scan.angleIncrement = 0.0;
        if (variant == 3) scan.angleIncrement = INFINITY;
        if (variant == 4) scan.rangeMin = -1.0;
        if (variant == 5) scan.rangeMax = scan.rangeMin;
        if (variant == 6) scan.rangeMax = NAN;
        Zero(CommandFor(scan));
    }
}
void OutOfRange()
{
    for (float invalid : {0.0f, 0.01f, 10.0f})
    {
        auto scan = Scene();
        for (auto& range : scan.ranges) range = invalid;
        Zero(CommandFor(scan));
    }
}
void MissingCoverage()
{
    auto scan = Scene();
    scan.ranges.resize(120); // Includes right but not front or front-right sectors.
    Zero(CommandFor(scan));
}
void ZeroToTwoPi()
{
    const auto command = CommandFor(Scene(0.30, INFINITY, 0.0));
    Require(command.linear > 0.09 && std::fabs(command.angular) < 0.01, "Wrapped scan convention changed steering");
}
void DescendingScan()
{
    const auto command = CommandFor(Scene(0.30, INFINITY, pi, -pi / 180.0));
    Require(command.linear > 0.09 && std::fabs(command.angular) < 0.01, "Descending angular order changed steering");
}
void RotatedLidar()
{
    Settings settings;
    settings.laserYaw = pi / 2.0;
    WallFollower controller(settings);
    controller.UpdateScan(Scene(0.30, INFINITY, -pi, pi / 180.0, pi / 2.0), 1.0);
    const auto command = controller.Command(1.05);
    Require(command.linear > 0.09 && std::fabs(command.angular) < 0.01, "Configured yaw did not restore right-wall geometry");
}
void VelocityBounds()
{
    std::mt19937 random(30);
    std::uniform_real_distribution<double> distance(0.15, 2.0), front(0.15, 2.0), heading(-0.5, 0.5);
    for (int i = 0; i < 1000; ++i)
    {
        const auto command = CommandFor(Scene(distance(random), front(random), -pi, pi / 180.0, 0.0, heading(random)));
        Require(std::isfinite(command.linear) && std::isfinite(command.angular), "Nonfinite velocity");
        Require(command.linear >= 0.0 && command.linear <= Settings().forwardSpeed,
                "Forward speed exceeded configured limit");
        Require(std::fabs(command.angular) <= Settings().turnSpeed, "Turn rate exceeded configured limit");
    }
}

struct Check { const char* name; void (*run)(); };
const Check checks[] = {
    {"default_settings", DefaultSettings}, {"invalid_settings", InvalidSettings},
    {"invalid_ordering", InvalidOrdering}, {"no_scan", NoScan},
    {"straight_wall", StraightWall}, {"close_wall", CloseWall},
    {"distant_wall", DistantWall}, {"wall_heading", WallHeading},
    {"lost_wall", LostWall}, {"blocked_front_priority", BlockedFrontPriority},
    {"corner_hysteresis", CornerHysteresis}, {"stale_scan", StaleScan},
    {"time_reversal", TimeReversal}, {"invalid_scan_time", InvalidScanTime},
    {"corrupt_sector", CorruptSector}, {"isolated_dropout", IsolatedDropout},
    {"front_minimum", FrontMinimum}, {"side_median", SideMedian},
    {"no_returns", NoReturns}, {"malformed_metadata", MalformedMetadata},
    {"out_of_range", OutOfRange}, {"missing_coverage", MissingCoverage},
    {"zero_to_two_pi", ZeroToTwoPi}, {"descending_scan", DescendingScan},
    {"rotated_lidar", RotatedLidar}, {"velocity_bounds", VelocityBounds}
};
}

int main(int argc, char** argv)
{
    if (argc != 2) { std::cerr << "Supply one registered check name\n"; return 2; }
    for (const auto& check : checks) if (std::string(argv[1]) == check.name)
    {
        try { check.run(); std::cout << "PASS " << check.name << '\n'; return 0; }
        catch (const std::exception& error)
        { std::cerr << "FAIL " << check.name << ": " << error.what() << '\n'; return 1; }
    }
    std::cerr << "Unknown check: " << argv[1] << '\n';
    return 2;
}
