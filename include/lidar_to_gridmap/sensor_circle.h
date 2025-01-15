#ifndef _SENSOR_CIRCLE_H
#define _SENSOR_CIRCLE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <cmath>

namespace lidar2gridmap {

class Point2D {
public:
    Point2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}

    void set(double x, double y) {
        this->x = x;
        this->y = y;
    }

    double x;
    double y;
};

class sensorCircle {
public:
    sensorCircle(ros::NodeHandle& nh);

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getPoints(Point2D center, double radius, int num_points);

    void showInRviz(std::string frame_id);

private:
    ros::NodeHandle& nh_;
    ros::Publisher cloud_pub_;

    Point2D center_;               // 圆心坐标
    double radius_;               // 半径
    int num_points_;              // 圆上的点数
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_;  // 智能指针
};

}

#endif // _SENSOR_CIRCLE_H