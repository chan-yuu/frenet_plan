#ifndef FRENET_PLANNER_HPP_
#define FRENET_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <memory>

namespace frenet_plan {

struct FrenetState {
    double s;      // 纵向位置
    double s_d;    // 纵向速度
    double s_dd;   // 纵向加速度
    double d;      // 横向位置
    double d_d;    // 横向速度
    double d_dd;   // 横向加速度
};

struct CartesianPoint {
    double x;
    double y;
    double theta;
    double kappa;
};

struct FrenetPath {
    std::vector<double> t;
    std::vector<double> d;
    std::vector<double> d_d;
    std::vector<double> d_dd;
    std::vector<double> d_ddd;
    std::vector<double> s;
    std::vector<double> s_d;
    std::vector<double> s_dd;
    std::vector<double> s_ddd;
    
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    std::vector<double> v;
    std::vector<double> a;
    std::vector<double> curvature;
    
    double cd = 0.0;  // 横向代价
    double cv = 0.0;  // 速度代价
    double cf = 0.0;  // 总代价
};

class QuinticPolynomial {
public:
    QuinticPolynomial(double xs, double vxs, double axs, 
                      double xe, double vxe, double axe, double T);
    
    double calc_point(double t);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);

private:
    double a0_, a1_, a2_, a3_, a4_, a5_;
};

class QuarticPolynomial {
public:
    QuarticPolynomial(double xs, double vxs, double axs, 
                      double vxe, double axe, double T);
    
    double calc_point(double t);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);

private:
    double a0_, a1_, a2_, a3_, a4_;
};

class FrenetPlanner : public rclcpp::Node {
public:
    FrenetPlanner();
    ~FrenetPlanner() = default;

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void velCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    void planningLoop();
    
    // Frenet坐标转换
    FrenetState getFrenetState(const CartesianPoint& cart_point, 
                               const std::vector<CartesianPoint>& ref_path);
    CartesianPoint getCartesianFromFrenet(double s, double d, 
                                          const std::vector<CartesianPoint>& ref_path);
    
    // 轨迹生成
    std::vector<FrenetPath> generateTrajectories(const FrenetState& current_state);
    
    // 轨迹评估
    void calculateCost(FrenetPath& path);
    bool checkCollision(const FrenetPath& path);
    bool checkCostmapCollision(double x, double y);
    int getCostmapCost(double x, double y);
    
    // 辅助函数
    double normalizeAngle(double angle);
    double getClosestPoint(const std::vector<CartesianPoint>& path, 
                          double x, double y);
    std::vector<CartesianPoint> pathMsgToCartesian(const nav_msgs::msg::Path& path_msg);
    nav_msgs::msg::Path frenetPathToMsg(const FrenetPath& fpath);
    visualization_msgs::msg::MarkerArray createTrajectoryMarkers(
        const std::vector<FrenetPath>& trajectories);
    
    // ROS2接口
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frenet_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr frenet_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_markers_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 数据存储
    nav_msgs::msg::Path::SharedPtr reference_path_;
    std_msgs::msg::Float64MultiArray::SharedPtr reference_vel_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    
    std::vector<CartesianPoint> ref_cartesian_path_;
    
    // 参数
    double planning_frequency_;
    double max_speed_;
    double max_accel_;
    double max_curvature_;
    double max_road_width_left_;
    double max_road_width_right_;
    double d_road_sample_;
    double t_sample_;
    double dt_;
    double target_speed_;
    double vehicle_length_;
    double vehicle_width_;
    int costmap_obstacle_threshold_;
    
    // 代价权重
    double k_jerk_;
    double k_time_;
    double k_diff_;
    double k_lat_;
    double k_lon_;
    
    bool path_received_;
    bool vel_received_;
    bool odom_received_;
    bool costmap_received_;
};

} // namespace frenet_plan

#endif // FRENET_PLANNER_HPP_
