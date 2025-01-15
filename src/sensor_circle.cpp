#include "lidar_to_gridmap/sensor_circle.h"

namespace lidar2gridmap {

sensorCircle::sensorCircle(ros::NodeHandle& nh): nh_(nh) {
        cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        // 发布点云数据
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sensor_circle_points", 1);
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensorCircle::getPoints(Point2D center, double radius, int num_points) {
        // 设置圆的参数
        center_.x = center.x;  // 圆心 x 坐标
        center_.y = center.y;  // 圆心 y 坐标
        radius_ = radius;    // 半径
        num_points_ = num_points;  // 圆上的点数
        // 生成圆上的点
        cloud_->clear();
        for (int i = 0; i < num_points_; ++i) {
                double angle = 2.0 * M_PI * i / num_points_;  // 角度
                double x = center_.x + radius_ * std::cos(angle);  // x 坐标
                double y = center_.y + radius_ * std::sin(angle);  // y 坐标
                cloud_->push_back(pcl::PointXYZ(x, y, 0.0));  // z 坐标为 0
        }
        return cloud_;
}

void sensorCircle::showInRviz(std::string frame_id){
        // 转换为 ROS 消息
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_, cloud_msg);
        cloud_msg.header.frame_id = frame_id;  // 设置坐标系
        cloud_msg.header.stamp = ros::Time::now();

        // 发布点云数据
        cloud_pub_.publish(cloud_msg);
}
}