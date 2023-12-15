#include "MPC_Control.h"
#include <ackermann_msgs/AckermannDrive.h>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <fstream>
#include <sstream>
#include <tf/transform_datatypes.h>

MPCController::MPCController(const ros::NodeHandle& nh, const std::vector<Point>& waypoints)
    : nh_(nh), all_waypoints_(waypoints), current_waypoint_index_(0) {
    odom_sub_ = nh_.subscribe("/gem/base_footprint/odom", 1, &MPCController::odomCallback, this);
    ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 1);
    lidar_2dsub_ = nh_.subscribe("/gem/front_laser_points", 10, &MPCController::lidarCallback, this);
    lidar_3dsub_ = nh_.subscribe("/gem/velodyne_points", 1, &MPCController::lidar3DCallback, this);
    cte_pub_ = nh_.advertise<std_msgs::Float64>("/cte", 1);

    mpc_ = std::make_shared<MPC>();
}
void MPCController::lidar3DCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert the sensor_msgs/PointCloud2 data to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *raw_cloud);

    float front_min_x = 0.0f; // Minimum distance in front of the robot to start considering obstacles
    float front_max_x = 10.0f; // Maximum distance in front of the robot to consider
    float lateral_range = 5.0f; // Lateral range to consider on either side of the robot
    float height_threshold = 0.3f; // Minimum height of obstacles to consider
    float max_height = 2.5f; // Maximum height to consider

    // Configure the CropBox filter
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(front_min_x, -lateral_range, height_threshold, 1.0));
    boxFilter.setMax(Eigen::Vector4f(front_max_x, lateral_range, max_height, 1.0));
    boxFilter.setInputCloud(raw_cloud);
    std::cout << "Number of points after filtering: " << raw_cloud->size() << std::endl;
    // Apply the filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    boxFilter.filter(*cloudFiltered);
    std::cout << "Number of points after filtering: " << cloudFiltered->size() << std::endl;
    std::vector<pcl::PointXYZ> obs;
    obs = process3DLidarData(cloudFiltered);

}
void MPCController::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    obstacles = processScan(*scan ,x, y);
    for(int i =0; i<obstacles.size(); i++){
        std::cout<<"x = "<<obstacles[i].x<<"  y = "<<obstacles[i].y<<std::endl;
    }
}


void MPCController::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;

    tf::Quaternion q(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;

    m.getRPY(roll, pitch, yaw);
    double current_velocity = odom->twist.twist.linear.x;
    double current_yaw_velocity = odom->twist.twist.angular.z;

    size_t lookahead = 40; // Number of waypoints to process
    size_t waypoints_count = all_waypoints_.size();
    std::vector<double> ptsx, ptsy;

    for (size_t i = current_waypoint_index_; i < waypoints_count && i < current_waypoint_index_ + lookahead; ++i) {
        double dx = all_waypoints_[i].x - x;
        double dy = all_waypoints_[i].y - y;
        ptsx.push_back(dx * cos(-yaw) - dy * sin(-yaw));
        ptsy.push_back(dx * sin(-yaw) + dy * cos(-yaw));
    
    }

    Eigen::VectorXd ptsx_eigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
    Eigen::VectorXd ptsy_eigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());
    Eigen::VectorXd coeffs_ = polyfit(ptsx_eigen, ptsy_eigen, 3);
    double distance = sqrt(pow(all_waypoints_[current_waypoint_index_ + lookahead - 1].x - x, 2) +
            pow(all_waypoints_[current_waypoint_index_ + lookahead - 1].y - y, 2));
            std::cout<<"distance = "<<distance<<std::endl;
    if ( distance < 5.0) {
        // Update to the next set of waypoints
        current_waypoint_index_ += lookahead;
        std::cout<<"current_waypoint_index_ =  "<<current_waypoint_index_<<std::endl;
    }
    double cte  = polyeval(coeffs_, 0);
    double epsi = -atan(coeffs_[1]);

    const double lat_px = x + current_velocity * latency_adjustment_sec;
    const double lat_py = y;
    const double lat_psi = yaw + current_velocity * (-steer_value) / Lf * latency_adjustment_sec;
    const double lat_v = current_velocity + throttle_value * latency_adjustment_sec;
    const double lat_cte = cte + current_velocity * sin(epsi) * latency_adjustment_sec;
    const double lat_epsi = epsi + current_velocity * (-steer_value) / Lf * latency_adjustment_sec;



    Eigen::VectorXd state(6);

    state << lat_px, lat_py, lat_psi, lat_v, lat_cte, lat_epsi;
    std::vector<double> x_vals = {state[0]};
    std::vector<double> y_vals = {state[1]};
    std::vector<double> psi_vals = {state[2]};
    std::vector<double> v_vals = {state[3]};
    std::vector<double> cte_vals = {state[4]};
    std::vector<double> epsi_vals = {state[5]};
    std::vector<double> delta_vals = {};
    std::vector<double> a_vals = {};


    for (size_t i = 0; i < 5; i++){

    
    auto vars = mpc_->Solve(state, coeffs_,obstacles);
    
        x_vals.push_back(vars[0]);
        y_vals.push_back(vars[1]);
        psi_vals.push_back(vars[2]);
        v_vals.push_back(vars[3]);
        cte_vals.push_back(vars[4]);
        epsi_vals.push_back(vars[5]);
        delta_vals.push_back(vars[6]);
        a_vals.push_back(vars[7]);
        
        steer_value= vars[6];
        speed_value =vars[3];
        throttle_value = vars[7];
        cte_value = vars[4];



        state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
        // std::cout<<"coeffs= "<<coeffs_<<std::endl;
        // std::cout << "x = " << vars[0] << std::endl;
        // std::cout << "y = " << vars[1] << std::endl;
        // std::cout << "psi = " << vars[2] << std::endl;
        // std::cout << "v = " << vars[3] << std::endl;
         std::cout << "cte = " << vars[4] << std::endl;
         std::cout << "epsi = " << vars[5] << std::endl;
         std::cout << "steering = " << vars[6] << std::endl;
         std::cout << "Throttle = " << vars[7] << std::endl;
         std::cout << std::endl;
    }
    std_msgs::Float64 cte_msg;
    cte_msg.data=cte_value;
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    ackermann_msgs::AckermannDrive drive_msg;

    ackermann_msgs::AckermannDriveStamped ackermann_cmd;
    drive_msg.steering_angle = steer_value;
    drive_msg.steering_angle_velocity = 0.0;
    drive_msg.speed = throttle_value;
    drive_msg.jerk = 0.0;
    drive_st_msg.drive = drive_msg;

    ackermann_pub_.publish(drive_msg);
    cte_pub_.publish(cte_msg);

}



std::vector<Point2D> processScan(const sensor_msgs::LaserScan& scan, float x, float y) {
std::vector<Point2D> obstacles;
for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float range = scan.ranges[i];
    if (range < scan.range_min || range > scan.range_max) continue;

    // Convert the laser scan to Cartesian coordinates
    float angle = scan.angle_min + i * scan.angle_increment;
    Point2D point;
    point.x = (range * cos(angle) + x);
    point.y = range * sin(angle) + y;

    // Simple threshold-based obstacle detection
    if (range < 24.0) { 
        obstacles.push_back(point);
    }
}
return obstacles;
}

std::vector<Point> readCSV(const std::string& filename) {
    std::vector<Point> points;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return points;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        Point point;

        std::getline(ss, cell, ',');
        point.x = std::stod(cell);

        std::getline(ss, cell,',');
        point.y = std::stod(cell);

        std::getline(ss, cell,',');
        point.yaw = std::stod(cell);

        points.push_back(point);
    }

    file.close();
    return points;
}

double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}
std::vector<pcl::PointXYZ> process3DLidarData(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::vector<pcl::PointXYZ> obs;
// UnComment the below set of lines to estimate obstacle positions from 3d Lidar.

    // Perform clustering on the filtered cloud

    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // tree->setInputCloud(cloud);

    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // ec.setClusterTolerance(0.02); 
    // ec.setMinClusterSize(50); 
    // ec.setMaxClusterSize(1000); 
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud);
    // ec.extract(cluster_indices);

    // // Extract the centroids
    // for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
    //     for (const auto& idx : it->indices)
    //         cluster->push_back((*cloud)[idx]); 

    //     pcl::PointXYZ centroid;
    //     pcl::computeCentroid(*cluster, centroid);
    //     obs.push_back(centroid);
    //     std::cout<<"obs_x  =  "<<centroid.x<<"  obs_y  =  "<<centroid.y<<std::endl;

    //}

    return obs;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_controller_node");
    ros::NodeHandle nh;
    const std::string filename = "/catkin_ws/src/mpc_ros/mpc_gen/src/wps.csv";
    std::vector<Point> waypoints = readCSV(filename);
    //MPCController mpc_controller;
    // 
    MPCController mpc_controller(nh, waypoints);

    ros::spin();
    return 0;
}