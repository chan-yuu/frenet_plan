#include "frenet_plan/frenet_planner.hpp"
#include <algorithm>
#include <limits>

namespace frenet_plan {

// ========== QuinticPolynomial ==========
QuinticPolynomial::QuinticPolynomial(double xs, double vxs, double axs,
                                     double xe, double vxe, double axe, double T) {
    a0_ = xs;
    a1_ = vxs;
    a2_ = axs / 2.0;
    
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    
    Eigen::Matrix3d A;
    A << T3, T4, T5,
         3*T2, 4*T3, 5*T4,
         6*T, 12*T2, 20*T3;
    
    Eigen::Vector3d b;
    b << xe - a0_ - a1_*T - a2_*T2,
         vxe - a1_ - 2*a2_*T,
         axe - 2*a2_;
    
    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
    
    a3_ = x(0);
    a4_ = x(1);
    a5_ = x(2);
}

double QuinticPolynomial::calc_point(double t) {
    return a0_ + a1_*t + a2_*t*t + a3_*t*t*t + a4_*t*t*t*t + a5_*t*t*t*t*t;
}

double QuinticPolynomial::calc_first_derivative(double t) {
    return a1_ + 2*a2_*t + 3*a3_*t*t + 4*a4_*t*t*t + 5*a5_*t*t*t*t;
}

double QuinticPolynomial::calc_second_derivative(double t) {
    return 2*a2_ + 6*a3_*t + 12*a4_*t*t + 20*a5_*t*t*t;
}

double QuinticPolynomial::calc_third_derivative(double t) {
    return 6*a3_ + 24*a4_*t + 60*a5_*t*t;
}

// ========== QuarticPolynomial ==========
QuarticPolynomial::QuarticPolynomial(double xs, double vxs, double axs,
                                     double vxe, double axe, double T) {
    a0_ = xs;
    a1_ = vxs;
    a2_ = axs / 2.0;
    
    double T2 = T * T;
    double T3 = T2 * T;
    
    Eigen::Matrix2d A;
    A << 3*T2, 4*T3,
         6*T, 12*T2;
    
    Eigen::Vector2d b;
    b << vxe - a1_ - 2*a2_*T,
         axe - 2*a2_;
    
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
    
    a3_ = x(0);
    a4_ = x(1);
}

double QuarticPolynomial::calc_point(double t) {
    return a0_ + a1_*t + a2_*t*t + a3_*t*t*t + a4_*t*t*t*t;
}

double QuarticPolynomial::calc_first_derivative(double t) {
    return a1_ + 2*a2_*t + 3*a3_*t*t + 4*a4_*t*t*t;
}

double QuarticPolynomial::calc_second_derivative(double t) {
    return 2*a2_ + 6*a3_*t + 12*a4_*t*t;
}

double QuarticPolynomial::calc_third_derivative(double t) {
    return 6*a3_ + 24*a4_*t;
}

// ========== FrenetPlanner ==========
FrenetPlanner::FrenetPlanner() : Node("frenet_planner") {
    // 声明参数
    this->declare_parameter<double>("planning_frequency", 50.0);
    this->declare_parameter<double>("max_speed", 15.0);
    this->declare_parameter<double>("max_accel", 3.0);
    this->declare_parameter<double>("max_curvature", 1.0);
    this->declare_parameter<double>("max_road_width_left", 3.0);
    this->declare_parameter<double>("max_road_width_right", 3.0);
    this->declare_parameter<double>("d_road_sample", 0.5);
    this->declare_parameter<double>("t_sample", 0.5);
    this->declare_parameter<double>("dt", 0.2);
    this->declare_parameter<double>("target_speed", 10.0);
    this->declare_parameter<double>("vehicle_length", 1.0);
    this->declare_parameter<double>("vehicle_width", 0.6);
    this->declare_parameter<int>("costmap_obstacle_threshold", 50);
    this->declare_parameter<double>("k_jerk", 0.1);
    this->declare_parameter<double>("k_time", 0.1);
    this->declare_parameter<double>("k_diff", 1.0);
    this->declare_parameter<double>("k_lat", 1.0);
    this->declare_parameter<double>("k_lon", 1.0);
    
    // 获取参数
    planning_frequency_ = this->get_parameter("planning_frequency").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    max_accel_ = this->get_parameter("max_accel").as_double();
    max_curvature_ = this->get_parameter("max_curvature").as_double();
    max_road_width_left_ = this->get_parameter("max_road_width_left").as_double();
    max_road_width_right_ = this->get_parameter("max_road_width_right").as_double();
    d_road_sample_ = this->get_parameter("d_road_sample").as_double();
    t_sample_ = this->get_parameter("t_sample").as_double();
    dt_ = this->get_parameter("dt").as_double();
    target_speed_ = this->get_parameter("target_speed").as_double();
    vehicle_length_ = this->get_parameter("vehicle_length").as_double();
    vehicle_width_ = this->get_parameter("vehicle_width").as_double();
    costmap_obstacle_threshold_ = this->get_parameter("costmap_obstacle_threshold").as_int();
    k_jerk_ = this->get_parameter("k_jerk").as_double();
    k_time_ = this->get_parameter("k_time").as_double();
    k_diff_ = this->get_parameter("k_diff").as_double();
    k_lat_ = this->get_parameter("k_lat").as_double();
    k_lon_ = this->get_parameter("k_lon").as_double();
    
    // 初始化标志
    path_received_ = false;
    vel_received_ = false;
    odom_received_ = false;
    costmap_received_ = false;
    
    // 订阅
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/local_path_plan", 10,
        std::bind(&FrenetPlanner::pathCallback, this, std::placeholders::_1));
    
    vel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/local_vel_cmd", 10,
        std::bind(&FrenetPlanner::velCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&FrenetPlanner::odomCallback, this, std::placeholders::_1));
    
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10,
        std::bind(&FrenetPlanner::costmapCallback, this, std::placeholders::_1));
    
    // 发布
    frenet_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/frenet_path", 10);
    frenet_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/frenet_vel", 10);
    traj_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/frenet_trajectories", 10);
    
    // 定时器 - 使用配置的规划频率
    int timer_period_ms = static_cast<int>(1000.0 / planning_frequency_);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        std::bind(&FrenetPlanner::planningLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Frenet Planner Node Started");
    RCLCPP_INFO(this->get_logger(), "Planning Frequency: %.1f Hz (Period: %d ms)", planning_frequency_, timer_period_ms);
    std::cout << "d_road_sample_: " << d_road_sample_ << std::endl;
}

void FrenetPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    reference_path_ = msg;
    ref_cartesian_path_ = pathMsgToCartesian(*msg);
    path_received_ = true;
}

void FrenetPlanner::velCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    reference_vel_ = msg;
    vel_received_ = true;
}

void FrenetPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = msg;
    odom_received_ = true;
}

void FrenetPlanner::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    costmap_ = msg;
    costmap_received_ = true;
}

void FrenetPlanner::planningLoop() {
    if (!path_received_ || !odom_received_) {
        return;
    }
    
    if (ref_cartesian_path_.empty()) {
        return;
    }
    
    // 获取当前位置
    CartesianPoint current_point;
    current_point.x = current_odom_->pose.pose.position.x;
    current_point.y = current_odom_->pose.pose.position.y;
    
    // 从四元数获取航向角
    auto q = current_odom_->pose.pose.orientation;
    current_point.theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    current_point.kappa = 0.0;
    
    // 转换到Frenet坐标
    FrenetState current_frenet = getFrenetState(current_point, ref_cartesian_path_);
    
    // 从Odometry获取速度
    double vx = current_odom_->twist.twist.linear.x;
    double vy = current_odom_->twist.twist.linear.y;
    current_frenet.s_d = std::sqrt(vx*vx + vy*vy);
    current_frenet.s_dd = 0.0;
    current_frenet.d_d = 0.0;
    current_frenet.d_dd = 0.0;
    
    // 生成候选轨迹
    std::vector<FrenetPath> candidate_paths = generateTrajectories(current_frenet);
    
    // 找到最优轨迹
    FrenetPath best_path;
    double min_cost = std::numeric_limits<double>::max();
    bool found_valid = false;
    
    for (auto& path : candidate_paths) {
        if (!checkCollision(path)) {
            calculateCost(path);
            if (path.cf < min_cost) {
                min_cost = path.cf;
                best_path = path;
                found_valid = true;
            }
        }
    }
    
    if (found_valid) {
        // 发布最优轨迹
        auto path_msg = frenetPathToMsg(best_path);
        frenet_path_pub_->publish(path_msg);
        
        // 发布速度
        std_msgs::msg::Float64MultiArray vel_msg;
        vel_msg.data = best_path.v;
        frenet_vel_pub_->publish(vel_msg);
    }
    
    // 发布所有候选轨迹用于可视化
    auto markers = createTrajectoryMarkers(candidate_paths);
    traj_markers_pub_->publish(markers);
}

FrenetState FrenetPlanner::getFrenetState(const CartesianPoint& cart_point,
                                         const std::vector<CartesianPoint>& ref_path) {
    FrenetState frenet;
    
    // 找到参考路径上最近的点
    double s = getClosestPoint(ref_path, cart_point.x, cart_point.y);
    
    // 找到对应的索引
    size_t idx = static_cast<size_t>(s / 0.1);  // 假设路径点间距0.1m
    if (idx >= ref_path.size() - 1) {
        idx = ref_path.size() - 2;
    }
    
    const auto& ref_point = ref_path[idx];
    
    // 计算横向偏差
    double dx = cart_point.x - ref_point.x;
    double dy = cart_point.y - ref_point.y;
    
    double cross = std::cos(ref_point.theta) * dy - std::sin(ref_point.theta) * dx;
    frenet.d = cross;
    frenet.s = s;
    
    return frenet;
}

CartesianPoint FrenetPlanner::getCartesianFromFrenet(double s, double d,
                                                     const std::vector<CartesianPoint>& ref_path) {
    CartesianPoint cart;
    
    if (ref_path.empty()) {
        cart.x = cart.y = cart.theta = cart.kappa = 0.0;
        return cart;
    }
    
    // 找到s对应的索引
    size_t idx = static_cast<size_t>(s / 0.1);
    if (idx >= ref_path.size() - 1) {
        idx = ref_path.size() - 2;
    }
    
    const auto& ref_point = ref_path[idx];
    
    // 从Frenet转换到笛卡尔坐标
    cart.x = ref_point.x - d * std::sin(ref_point.theta);
    cart.y = ref_point.y + d * std::cos(ref_point.theta);
    cart.theta = ref_point.theta;
    cart.kappa = ref_point.kappa;
    
    return cart;
}

std::vector<FrenetPath> FrenetPlanner::generateTrajectories(const FrenetState& current_state) {
    std::vector<FrenetPath> paths;
    
    // 采样横向目标位置
    for (double di = -max_road_width_right_; di <= max_road_width_left_; di += d_road_sample_) {
        // 采样时间 (低速场景使用较短的时间范围)
        for (double Ti = 2.0; Ti <= 4.0; Ti += t_sample_) {
            FrenetPath fp;
            
            // 横向五次多项式
            QuinticPolynomial lat_qp(current_state.d, current_state.d_d, current_state.d_dd,
                                     di, 0.0, 0.0, Ti);
            
            // 纵向采样不同的目标速度 (低速场景采样范围更小)
            double v_min = std::max(0.5, target_speed_ - 1.0);
            double v_max = std::min(max_speed_, target_speed_ + 0.5);
            for (double tv = v_min; tv <= v_max; tv += 0.5) {
                FrenetPath fp_copy = fp;
                
                // 纵向四次多项式 (速度为目标)
                QuarticPolynomial lon_qp(current_state.s, current_state.s_d, current_state.s_dd,
                                         tv, 0.0, Ti);
                
                // 采样轨迹
                for (double t = 0.0; t <= Ti; t += dt_) {
                    fp_copy.t.push_back(t);
                    
                    // 横向
                    fp_copy.d.push_back(lat_qp.calc_point(t));
                    fp_copy.d_d.push_back(lat_qp.calc_first_derivative(t));
                    fp_copy.d_dd.push_back(lat_qp.calc_second_derivative(t));
                    fp_copy.d_ddd.push_back(lat_qp.calc_third_derivative(t));
                    
                    // 纵向
                    fp_copy.s.push_back(lon_qp.calc_point(t));
                    fp_copy.s_d.push_back(lon_qp.calc_first_derivative(t));
                    fp_copy.s_dd.push_back(lon_qp.calc_second_derivative(t));
                    fp_copy.s_ddd.push_back(lon_qp.calc_third_derivative(t));
                }
                
                // 转换到笛卡尔坐标
                for (size_t i = 0; i < fp_copy.s.size(); ++i) {
                    CartesianPoint cart = getCartesianFromFrenet(fp_copy.s[i], fp_copy.d[i], 
                                                                ref_cartesian_path_);
                    fp_copy.x.push_back(cart.x);
                    fp_copy.y.push_back(cart.y);
                    fp_copy.yaw.push_back(cart.theta);
                    
                    // 计算速度和加速度
                    double vel = std::sqrt(fp_copy.s_d[i]*fp_copy.s_d[i] + 
                                          fp_copy.d_d[i]*fp_copy.d_d[i]);
                    fp_copy.v.push_back(vel);
                    
                    double accel = std::sqrt(fp_copy.s_dd[i]*fp_copy.s_dd[i] + 
                                            fp_copy.d_dd[i]*fp_copy.d_dd[i]);
                    fp_copy.a.push_back(accel);
                }
                
                // 计算曲率
                for (size_t i = 0; i < fp_copy.x.size() - 1; ++i) {
                    double dx = fp_copy.x[i+1] - fp_copy.x[i];
                    double dy = fp_copy.y[i+1] - fp_copy.y[i];
                    double dyaw = normalizeAngle(fp_copy.yaw[i+1] - fp_copy.yaw[i]);
                    double ds = std::sqrt(dx*dx + dy*dy);
                    fp_copy.curvature.push_back(dyaw / (ds + 1e-6));
                }
                if (!fp_copy.curvature.empty()) {
                    fp_copy.curvature.push_back(fp_copy.curvature.back());
                }
                
                paths.push_back(fp_copy);
            }
        }
    }
    
    return paths;
}

void FrenetPlanner::calculateCost(FrenetPath& path) {
    // Jerk代价
    double jerk_cost = 0.0;
    for (size_t i = 0; i < path.d_ddd.size(); ++i) {
        jerk_cost += path.d_ddd[i] * path.d_ddd[i];
    }
    for (size_t i = 0; i < path.s_ddd.size(); ++i) {
        jerk_cost += path.s_ddd[i] * path.s_ddd[i];
    }
    
    // 时间代价
    double time_cost = path.t.empty() ? 0.0 : path.t.back();
    
    // 横向偏差代价
    double lat_cost = 0.0;
    for (double d : path.d) {
        lat_cost += d * d;
    }
    
    // 速度偏差代价
    double vel_cost = 0.0;
    for (double v : path.v) {
        vel_cost += (target_speed_ - v) * (target_speed_ - v);
    }
    
    path.cd = k_jerk_ * jerk_cost + k_time_ * time_cost + k_lat_ * lat_cost;
    path.cv = k_lon_ * vel_cost;
    path.cf = k_diff_ * path.cd + path.cv;
}

bool FrenetPlanner::checkCollision(const FrenetPath& path) {
    // 检查速度、加速度和曲率约束
    for (double v : path.v) {
        if (v > max_speed_) return true;
    }
    
    for (double a : path.a) {
        if (std::abs(a) > max_accel_) return true;
    }
    
    for (double k : path.curvature) {
        if (std::abs(k) > max_curvature_) return true;
    }
    
    // 检查横向边界
    for (double d : path.d) {
        if (d < -max_road_width_right_ || d > max_road_width_left_) {
            return true;
        }
    }
    
    // 检查costmap碰撞（如果costmap可用）
    if (costmap_received_ && costmap_) {
        for (size_t i = 0; i < path.x.size(); ++i) {
            if (checkCostmapCollision(path.x[i], path.y[i])) {
                return true;
            }
        }
    }
    
    return false;
}

double FrenetPlanner::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double FrenetPlanner::getClosestPoint(const std::vector<CartesianPoint>& path,
                                     double x, double y) {
    double min_dist = std::numeric_limits<double>::max();
    int min_idx = 0;
    
    for (size_t i = 0; i < path.size(); ++i) {
        double dx = path[i].x - x;
        double dy = path[i].y - y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
        }
    }
    
    // 返回累积距离s
    double s = 0.0;
    for (int i = 0; i < min_idx; ++i) {
        double dx = path[i+1].x - path[i].x;
        double dy = path[i+1].y - path[i].y;
        s += std::sqrt(dx*dx + dy*dy);
    }
    
    return s;
}

std::vector<CartesianPoint> FrenetPlanner::pathMsgToCartesian(const nav_msgs::msg::Path& path_msg) {
    std::vector<CartesianPoint> cart_path;
    
    for (size_t i = 0; i < path_msg.poses.size(); ++i) {
        CartesianPoint pt;
        pt.x = path_msg.poses[i].pose.position.x;
        pt.y = path_msg.poses[i].pose.position.y;
        
        auto q = path_msg.poses[i].pose.orientation;
        pt.theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        pt.kappa = 0.0;
        
        cart_path.push_back(pt);
    }
    
    return cart_path;
}

nav_msgs::msg::Path FrenetPlanner::frenetPathToMsg(const FrenetPath& fpath) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";
    
    for (size_t i = 0; i < fpath.x.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = fpath.x[i];
        pose.pose.position.y = fpath.y[i];
        pose.pose.position.z = 0.0;
        
        // 从yaw生成四元数
        double yaw = fpath.yaw[i];
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        
        path_msg.poses.push_back(pose);
    }
    
    return path_msg;
}

visualization_msgs::msg::MarkerArray FrenetPlanner::createTrajectoryMarkers(
    const std::vector<FrenetPath>& trajectories) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    for (size_t i = 0; i < trajectories.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "map";
        marker.ns = "frenet_trajectories";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.scale.x = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 0.5;
        
        for (size_t j = 0; j < trajectories[i].x.size(); ++j) {
            geometry_msgs::msg::Point pt;
            pt.x = trajectories[i].x[j];
            pt.y = trajectories[i].y[j];
            pt.z = 0.0;
            marker.points.push_back(pt);
        }
        
        marker_array.markers.push_back(marker);
    }
    
    return marker_array;
}

bool FrenetPlanner::checkCostmapCollision(double x, double y) {
    if (!costmap_) return false;
    
    // 检查矩形车辆范围内的所有点
    double resolution = costmap_->info.resolution;
    double half_length = vehicle_length_ / 2.0;
    double half_width = vehicle_width_ / 2.0;
    
    int check_cells_x = static_cast<int>(std::ceil(half_length / resolution));
    int check_cells_y = static_cast<int>(std::ceil(half_width / resolution));
    
    // 遍历车辆矩形覆盖的栅格
    for (int dx = -check_cells_x; dx <= check_cells_x; ++dx) {
        for (int dy = -check_cells_y; dy <= check_cells_y; ++dy) {
            double offset_x = dx * resolution;
            double offset_y = dy * resolution;
            
            // 检查是否在矩形范围内
            if (std::abs(offset_x) > half_length || std::abs(offset_y) > half_width) {
                continue;
            }
            
            double check_x = x + offset_x;
            double check_y = y + offset_y;
            
            int cost = getCostmapCost(check_x, check_y);
            if (cost >= costmap_obstacle_threshold_ || cost < 0) {
                return true;  // 碰撞
            }
        }
    }
    
    return false;
}

int FrenetPlanner::getCostmapCost(double x, double y) {
    if (!costmap_) return -1;
    
    // 世界坐标转换到栅格坐标
    double origin_x = costmap_->info.origin.position.x;
    double origin_y = costmap_->info.origin.position.y;
    double resolution = costmap_->info.resolution;
    
    int mx = static_cast<int>((x - origin_x) / resolution);
    int my = static_cast<int>((y - origin_y) / resolution);
    
    // 检查边界
    if (mx < 0 || my < 0 || 
        mx >= static_cast<int>(costmap_->info.width) || 
        my >= static_cast<int>(costmap_->info.height)) {
        return -1;  // 超出地图范围
    }
    
    // 获取代价值
    int index = my * costmap_->info.width + mx;
    if (index >= 0 && index < static_cast<int>(costmap_->data.size())) {
        return static_cast<int>(costmap_->data[index]);
    }
    
    return -1;
}

} // namespace frenet_plan
