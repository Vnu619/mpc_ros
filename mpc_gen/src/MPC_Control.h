#ifndef MPC_CONTROL_H
#define MPC_CONTROL_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include "Eigen-3.3/Eigen/Core"
#include <vector>
#include "MPC.cpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/crop_box.h>
class MPC;



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
    ros::Publisher cte_pub_;
    double steer_value, throttle_value, speed_value, latency_adjustment_sec, cte_value;
    std::vector<Point> all_waypoints_;
    size_t current_waypoint_index_;
    std::shared_ptr<MPC> mpc_;
    double x, y;
    std::vector<Point2D> obstacles;
};

std::vector<Point2D> processScan(const sensor_msgs::LaserScan& scan, float x, float y);
std::vector<Point> readCSV(const std::string& filename);
double polyeval(Eigen::VectorXd coeffs, double x);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
std::vector<pcl::PointXYZ> process3DLidarData(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);


#endif // MPC_CONTROL_H
