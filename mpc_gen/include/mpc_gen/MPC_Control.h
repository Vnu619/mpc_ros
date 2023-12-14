#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include "Eigen-3.3/Eigen/Core"
#include <vector>

// Forward declaration of MPC class
class MPC;

struct Point {
    double x, y, yaw;
};

struct Point2D {
    double x, y;
};

class MPCController {
public:
    MPCController(const ros::NodeHandle& nh, const std::vector<Point>& waypoints);
    void lidar3DCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_2dsub_;
    ros::Subscriber lidar_3dsub_;
    ros::Subscriber odom_sub_;
    ros::Publisher ackermann_pub_;
    double steer_value, throttle_value, speed_value, latency_adjustment_sec;
    std::vector<Point> all_waypoints_;
    size_t current_waypoint_index_;
    std::shared_ptr<MPC> mpc_;
    double x, y;
    std::vector<Point2D> obstacles;
};

#endif // MPC_CONTROLLER_H
