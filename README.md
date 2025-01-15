# 全局占用栅格图   
## 实现目标   
1. 接收配准后的16线激光雷达数据和里程计数据，构建全局占用栅格地图，并发布。   
2. 全局占用栅格图的原点怎么定义？   
3. 16线激光雷达话题名为registered_scan，类型为sensor_msgs::PointCloud2
4. 16线激光雷达投影成的单线雷达话题名为projected_laserscan，类型为sensor_msgs::LaserScan
5. 里程计数据话题名为/state_estimation_at_scan，类型为nav_msgs/Odometry
## 方法   
如果算法刚初始化且机器人在起始点位置附近，采用单线雷达的方法更新栅格状态，否则使用地形点云更新栅格状态。   
### **单线雷达点云更新栅格方法**   
先根据障碍物点云的位置，更新栅格占用状态；   
从机器人当前位置进行采样，采样一圈半径为R圆形边界点，用射线法更新栅格占用状态，碰到占用栅格则停止当前射线上的栅格更新，当前位置到占用栅格中间的射线上的栅格状态设置为自由。   
### **地形点云更新栅格状态的方法（有斜坡的时候使用这个方法，对于室内环境可以使用单线雷达点云更新栅格的方法）**   
将点云投影到地面，获取地面栅格中包含的高度最低和最高的点云，如果最高和最低点云的高度差大于threshold值，则表示这个栅格是被占用的。更新对应栅格为占用状态。   
从机器人当前位置进行采样，采样一圈半径为R圆形边界点，用射线法更新栅格占用状态，碰到占用栅格则停止当前射线上的栅格更新，当前位置到占用栅格中间的射线上的栅格状态设置为自由。   

## 参数配置
所有参数可通过launch文件或命令行配置：
- 高度过滤：
  - height_threshold_low: 点云最低高度阈值 (默认: 0.2m)
  - height_threshold_high: 点云最高高度阈值 (默认: 0.8m)
- 距离范围：
  - range_min: 最小有效距离 (默认: 0.4m)
  - range_max: 最大有效距离 (默认: 12.0m)
- 地图参数：
  - resolution: 地图分辨率 (默认: 0.05m/cell)
  - width: 地图宽度 (默认: 400 cells)
  - height: 地图高度 (默认: 400 cells)
  - origin_x: 地图原点X坐标 (默认: -10.0m)
  - origin_y: 地图原点Y坐标 (默认: -10.0m)
- 地形检测：
  - terrain_threshold: 地形高度差阈值 (默认: 0.3m)

## 运行方法
1. 启动节点：
```bash
roslaunch lidar_to_gridmap lidar2gridmap.launch
```
2. 打开RViz查看地图：
```bash
rosrun rviz rviz -d $(rospack find lidar_to_gridmap)/rviz/lidar2gridmap.rviz
```
