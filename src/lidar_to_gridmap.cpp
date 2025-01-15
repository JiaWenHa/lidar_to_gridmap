#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>

class LidarToGridMap {
public:
    LidarToGridMap() : tf_listener_(tf_buffer_) {
        // 初始化参数
        nh_.param("resolution", resolution_, 0.05);
        nh_.param("gridmap_width", gridmap_width_, 400);
        nh_.param("height_threshold_low", height_threshold_low_, 0.2);
        nh_.param("height_threshold_high", height_threshold_high_, 0.8);
        nh_.param("scan_range_min", scan_range_min_, 0.4);
        nh_.param("scan_range_max", scan_range_max_, 12.0);
        nh_.param("terrain_threshold", terrain_threshold_, 0.3);

        origin_x_ = -gridmap_width_ * resolution_ / 2;
        origin_y_ = -gridmap_width_ * resolution_ / 2;


        // 初始化栅格地图
        grid_map_.info.resolution = resolution_;
        grid_map_.info.width = gridmap_width_;
        grid_map_.info.height = gridmap_width_;
        grid_map_.info.origin.position.x = origin_x_;
        grid_map_.info.origin.position.y = origin_y_;
        grid_map_.info.origin.position.z = 0.0;
        grid_map_.info.origin.orientation.w = 1.0;
        grid_map_.data.resize(gridmap_width_ * gridmap_width_, -1);

        // 初始化LaserScan消息
        laserscan_.header.frame_id = "laser_frame"; // 设置激光雷达的坐标系
        laserscan_.angle_min = -M_PI; // 最小角度
        laserscan_.angle_max = M_PI; // 最大角度
        laserscan_.angle_increment = M_PI / 450.0; // 角度分辨率（0.4度）
        laserscan_.time_increment = 0.0; // 时间增量
        laserscan_.scan_time = 0.1; // 扫描时间
        laserscan_.range_min = scan_range_min_; // 最小距离
        laserscan_.range_max = scan_range_max_; // 最大距离

        // 计算激光雷达数据的范围大小
        int num_readings = (laserscan_.angle_max - laserscan_.angle_min) / laserscan_.angle_increment;
        laserscan_.ranges.resize(num_readings, std::numeric_limits<float>::infinity()); // 初始化为无穷大

        // 订阅话题
        pointcloud_sub_ = nh_.subscribe("/registered_scan", 1, &LidarToGridMap::pointcloudCallback, this);
        odom_sub_ = nh_.subscribe("/state_estimation_at_scan", 1, &LidarToGridMap::odomCallback, this);

        // 发布话题
        grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1);
        // 发布单线激光雷达数据
        laserscan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/projected_laserscan", 1);
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
        // 更新机器人位置
        robot_position_ = odom->pose.pose.position;
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        // 将PointCloud2转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*cloud_msg, pcl_cloud);

        // 获取机器人位置
        double robot_x = robot_position_.x;
        double robot_y = robot_position_.y;

        // 将机器人位置转换为栅格地图坐标
        int robot_grid_x = static_cast<int>((robot_x - origin_x_) / resolution_);
        int robot_grid_y = static_cast<int>((robot_y - origin_y_) / resolution_);

        // 处理点云数据
        if (use_single_line_mode_) {
            processSingleLineMode(pcl_cloud, robot_grid_x, robot_grid_y);
        } else {
            processTerrainMode(pcl_cloud, robot_grid_x, robot_grid_y);
        }

        // 发布栅格地图
        grid_map_.header.stamp = ros::Time::now();
        grid_map_.header.frame_id = pcl_cloud.header.frame_id;
        grid_map_pub_.publish(grid_map_);
    }

    void processSingleLineMode(const pcl::PointCloud<pcl::PointXYZ>& cloud, int robot_x, int robot_y) {
        // 清空LaserScan数据
        std::fill(laserscan_.ranges.begin(), laserscan_.ranges.end(), std::numeric_limits<float>::infinity());
        // 清空地图
        std::fill(grid_map_.data.begin(), grid_map_.data.end(), -1);

        laserscan_.header.frame_id = cloud.header.frame_id; // 设置激光雷达的坐标系
        // 遍历点云，过滤高度并投影到2D平面
        for (const auto& point : cloud.points) {
            if (point.z > height_threshold_low_ && point.z < height_threshold_high_) { // 过滤高度低于阈值的点
                // 计算点的极坐标（距离和角度）
                float range = std::hypot(point.x, point.y);
                float angle = std::atan2(point.y, point.x);
                // 计算点的栅格坐标
                int grid_x = static_cast<int>((point.x - origin_x_) / resolution_);
                int grid_y = static_cast<int>((point.y - origin_y_) / resolution_);

                // 更新栅格
                if (grid_x >= 0 && grid_x < gridmap_width_ && grid_y >= 0 && grid_y < gridmap_width_) {
                    int index = grid_y * gridmap_width_ + grid_x;
                    grid_map_.data[index] = 100;  // 设置为占用
                }

                // 更新射线经过的栅格
                updateRay(robot_x, robot_y, grid_x, grid_y);

                // 将角度映射到LaserScan的索引
                if (angle >= laserscan_.angle_min && angle <= laserscan_.angle_max) {
                    int index = static_cast<int>((angle - laserscan_.angle_min) / laserscan_.angle_increment);
                    if (index >= 0 && index < laserscan_.ranges.size()) {
                        // 更新最近的距离
                        if (range < laserscan_.ranges[index]) {
                            laserscan_.ranges[index] = range;
                        }
                    }
                }
            }
        }

        // 发布LaserScan数据
        laserscan_.header.stamp = ros::Time::now();
        laserscan_pub_.publish(laserscan_);
    }

    void processTerrainMode(const pcl::PointCloud<pcl::PointXYZ>& cloud, int robot_x, int robot_y) {
        // 清空地图
        std::fill(grid_map_.data.begin(), grid_map_.data.end(), -1);

        // 创建高度图
        std::map<std::pair<int, int>, std::pair<float, float>> height_map;  // 存储每个栅格的最小和最大高度

        // 处理点云数据
        for (const auto& point : cloud.points) {
            int grid_x = static_cast<int>((point.x - origin_x_) / resolution_);
            int grid_y = static_cast<int>((point.y - origin_y_) / resolution_);

            if (grid_x >= 0 && grid_x < gridmap_width_ && grid_y >= 0 && grid_y < gridmap_width_) {
                auto& heights = height_map[{grid_x, grid_y}];
                heights.first = std::min(heights.first, point.z);
                heights.second = std::max(heights.second, point.z);
            }
        }

        // 更新栅格地图
        for (const auto& [grid, heights] : height_map) {
            auto [grid_x, grid_y] = grid;
            float height_diff = heights.second - heights.first;

            if (height_diff > terrain_threshold_) {
                int index = grid_y * gridmap_width_ + grid_x;
                grid_map_.data[index] = 100;  // 设置为占用
                updateRay(robot_x, robot_y, grid_x, grid_y);
            }
        }
    }

    void updateRay(int x0, int y0, int x1, int y1) {
        // Bresenham算法实现
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true) {
            if (x0 >= 0 && x0 < gridmap_width_ && y0 >= 0 && y0 < gridmap_width_) {
                int index = y0 * gridmap_width_ + x0;
                if (x0 == x1 && y0 == y1) {
                    grid_map_.data[index] = 100;  // 终点设置为占用
                } else if (grid_map_.data[index] != 100) {
                    grid_map_.data[index] = 0;    // 射线经过的栅格设置为空闲
                }
            }

            if (x0 == x1 && y0 == y1) break;

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
    }

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

    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher grid_map_pub_;
    ros::Publisher laserscan_pub_;
    nav_msgs::OccupancyGrid grid_map_;
    geometry_msgs::Point robot_position_;
    sensor_msgs::LaserScan laserscan_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_to_gridmap");
    LidarToGridMap lidar_to_gridmap;
    ros::spin();
    return 0;
}
