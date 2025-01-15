#ifndef _LIDAR_TO_GRIDMAP_H
#define _LIDAR_TO_GRIDMAP_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>

#include "lidar_to_gridmap/sensor_circle.h"

namespace lidar2gridmap {
class LidarToGridMap {
public:
    LidarToGridMap(ros::NodeHandle & nh, ros::NodeHandle & nh_param);

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
        // 更新机器人位置
        robot_position_ = odom->pose.pose.position;
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    void processSingleLineMode(const pcl::PointCloud<pcl::PointXYZ>& cloud, int robot_grid_x_id, int robot_grid_y_id);

    void processTerrainMode(const pcl::PointCloud<pcl::PointXYZ>& cloud, int robot_grid_x_id, int robot_grid_y_id);

    void updateRay(int x0, int y0, int x1, int y1);

    // 参数
    double resolution_;
    int gridmap_width_;
    double origin_x_;
    double origin_y_;
    double height_threshold_low_;
    double height_threshold_high_;
    double scan_range_min_;
    double scan_range_max_;
    double terrain_threshold_;
    bool use_single_line_mode_ = true;  // 默认使用单线模式
    double robot_position_x_;
    double robot_position_y_;

    sensorCircle sensor_circle_;

    // ROS相关
    ros::NodeHandle& nh_;
    ros::NodeHandle& nh_param_;
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher grid_map_pub_;
    ros::Publisher laserscan_pub_;
    nav_msgs::OccupancyGrid grid_map_;
    geometry_msgs::Point robot_position_;
    sensor_msgs::LaserScan laserscan_;
};
}

#endif // _LIDAR_TO_GRIDMAP_H