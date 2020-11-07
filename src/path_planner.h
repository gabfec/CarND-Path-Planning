#include <vector>
#include "json.hpp"


constexpr auto SPEED_LIMIT_MPH = 49.5;

class PathPlanner
{
public:
    explicit PathPlanner(const char *csv_file);
    ~PathPlanner() = default;

    nlohmann::json ProcessTelemetry(nlohmann::json data);

private:
    struct Waypoints
    {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> s;
        std::vector<double> dx;
        std::vector<double> dy;
    };

    struct Vehicle
    {
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
        inline int lane() {return round((d - 2) / 4);}
        inline double distanceTo(const Vehicle &other) {return fabs(s - other.s);}
        inline bool isInFrontOf(const Vehicle &other) {return s > other.s;}
    };

    Waypoints _waypoints;

    int lane = 1;
    float ref_velocity = 0;

    inline double deg2rad(double x) {return x * M_PI / 180;}
    inline double mph2mps(double speed) {return speed * 0.44704;}

};