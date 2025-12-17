# Costmap避障功能说明

## 功能概述

Frenet规划器现已集成基于Costmap的实时避障功能，可以根据环境地图动态过滤掉与障碍物碰撞的轨迹。

## 工作原理

### 1. Costmap订阅
- **话题**: `/costmap` (nav_msgs/OccupancyGrid)
- **数据格式**: 栅格地图，每个栅格值范围 0-100
  - 0: 完全自由空间
  - 1-49: 低代价区域
  - 50-99: 高代价区域（默认视为障碍）
  - 100: 致命障碍
  - -1: 未知区域

### 2. 碰撞检测流程

对于每条候选轨迹：
1. 遍历轨迹上的每个点 (x, y)
2. 以机器人半径为范围进行膨胀检测
3. 检查膨胀范围内的所有costmap栅格
4. 如果任何栅格代价 >= 阈值，则判定为碰撞

### 3. 机器人半径膨胀

```
参数: robot_radius = 0.3m (默认)

检测示意:
    ···
   ·····
  ·······  <- 以轨迹点为中心的圆形区域
   ·····      半径 = robot_radius
    ···
```

## 关键参数配置

在 `config/frenet_params.yaml`:

```yaml
# Costmap避障参数
robot_radius: 0.3                  # 机器人半径 (m)
costmap_obstacle_threshold: 50     # 障碍物阈值 (0-100)
```

### 参数说明

| 参数 | 说明 | 建议值 |
|------|------|--------|
| `robot_radius` | 机器人半径，用于碰撞检测膨胀 | 实际机器人半径 + 安全边界 (0.1-0.2m) |
| `costmap_obstacle_threshold` | Costmap代价阈值 | 50 (保守) / 80 (激进) |

### 不同场景的配置建议

#### 园区低速场景 (当前)
```yaml
robot_radius: 0.3              # 小车 + 安全边界
costmap_obstacle_threshold: 50 # 保守避障
```

#### 狭窄通道
```yaml
robot_radius: 0.25             # 减小安全边界以通过窄道
costmap_obstacle_threshold: 70 # 提高阈值，容忍更高代价
```

#### 开阔区域高速
```yaml
robot_radius: 0.4              # 增加安全边界
costmap_obstacle_threshold: 30 # 更保守的避障
```

## 性能影响

### 计算复杂度
- **无Costmap**: O(N) - N为候选轨迹数量
- **有Costmap**: O(N × M × K²) 
  - N: 候选轨迹数量
  - M: 每条轨迹点数
  - K: robot_radius / resolution

### 优化建议
1. **使用合适的Costmap分辨率**
   ```
   建议: 0.05-0.1m/格
   过细: 计算量大
   过粗: 精度不足
   ```

2. **调整机器人半径**
   ```
   小车宽度: 0.6m
   建议半径: 0.3-0.35m (半径 = 宽度/2 + 安全边界)
   ```

## Costmap数据源

### 常见来源
1. **SLAM建图** (Cartographer, SLAM Toolbox)
2. **激光雷达costmap** (costmap_2d)
3. **深度相机** (DepthImage to LaserScan)
4. **多传感器融合** (layered costmap)

### 示例：从导航栈获取Costmap

如果使用Nav2:
```yaml
# launch文件中重映射
remappings=[
    ('/costmap', '/local_costmap/costmap'),  # Nav2 local costmap
]
```

或使用全局costmap:
```yaml
remappings=[
    ('/costmap', '/global_costmap/costmap'),
]
```

## 测试和验证

### 1. 检查Costmap是否发布
```bash
ros2 topic echo /costmap --once
ros2 topic hz /costmap
```

### 2. 可视化Costmap
在RViz中添加:
- Display Type: Map
- Topic: /costmap

### 3. 监控避障效果
```bash
# 查看规划频率（避障会轻微降低频率）
ros2 topic hz /frenet_path

# 查看候选轨迹数量（在RViz中观察）
```

### 4. 调试信息
启动节点后查看日志:
```
[INFO] Frenet Planner Node Started
[INFO] Planning Frequency: 40.0 Hz (Period: 25 ms)
```

## 避障行为

### 正常场景
```
候选轨迹: 448条
无碰撞轨迹: 448条  ✓
选择最优轨迹
```

### 存在障碍物
```
候选轨迹: 448条
碰撞检测后: 320条有效  ⚠️
从有效轨迹中选择最优
```

### 无可行轨迹
```
候选轨迹: 448条
无碰撞轨迹: 0条  ✗
不发布轨迹，触发紧急停车
```

## 与其他模块集成

### 完整感知-规划流程
```
传感器 → SLAM/Costmap生成 → /costmap
参考路径 → 路径规划 → /local_path_plan
定位 → /odom

↓ 全部输入到Frenet规划器

Frenet规划器 → 避障后的最优轨迹 → /frenet_path
```

## 故障排查

### 问题1: 轨迹全部被过滤
**现象**: 规划器不输出轨迹  
**可能原因**:
- Costmap全是障碍物
- `costmap_obstacle_threshold` 设置过低
- `robot_radius` 设置过大

**解决**:
```yaml
costmap_obstacle_threshold: 70  # 提高阈值
robot_radius: 0.25              # 减小半径
```

### 问题2: 明显的障碍物没有避开
**现象**: 车辆冲向障碍物  
**可能原因**:
- Costmap未收到或更新慢
- 阈值设置过高
- 机器人半径过小

**解决**:
1. 检查costmap话题: `ros2 topic hz /costmap`
2. 降低阈值: `costmap_obstacle_threshold: 30`
3. 增大半径: `robot_radius: 0.4`

### 问题3: 规划频率大幅下降
**现象**: 从43Hz降到15Hz  
**可能原因**: Costmap分辨率过高导致计算量大

**解决**:
- 使用更粗的Costmap分辨率 (0.1m)
- 减少候选轨迹数量
- 降低规划频率期望值

## 总结

✅ **已实现功能**:
- Costmap订阅与解析
- 基于机器人半径的膨胀碰撞检测
- 自动过滤碰撞轨迹
- 可配置的避障参数

🎯 **推荐配置** (园区低速):
```yaml
robot_radius: 0.3
costmap_obstacle_threshold: 50
```

这个配置在保证安全的同时，提供良好的避障性能。
