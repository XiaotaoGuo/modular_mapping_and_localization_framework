## 坐标系说明

### 前端

- 激光雷达里程计全局坐标系：原点在激光雷达初始位置，姿态和激光雷达初始姿态一致
- 激光雷达里程计载体坐标系：由里程计输出的位姿估计

### 后端

- 全局坐标系：根据 GNSS 信息确定方向，以激光雷达初始位置为原点的东北天坐标系
- 载体里程计坐标系：全局坐标系下的里程计估计的载体位姿
- 优化后的载体位姿：全局坐标系下经过图优化后的载体的位姿全局最优估计

### 可视化

- 校正后的里程计坐标系：从全局优化后的最新一帧和里程计的对应帧可以获得一个相对位姿，通过该位姿可以对后续的里程计位姿估计进行校正（即在优化后的轨迹进行里程计估计）

## 轨迹说明

- 基于增量信息的前端里程计轨迹 -- 可以保证帧间运动平滑性
- 后端位姿优化轨迹            -- 精度更高，不实时
- 后端位姿优化 + 里程计轨迹    --  在优化后的轨迹基础上，利用里程计的帧间位姿估计进行预测
