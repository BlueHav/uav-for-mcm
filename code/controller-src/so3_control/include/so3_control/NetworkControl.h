#ifndef NETWORK_CONTROL_H_
#define NETWORK_CONTROL_H_

#include <Eigen/Eigen>
#include <math.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <chrono>
#include <sstream>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/SetTakeoffLand.h>
#include <so3_control/SO3Control.h>
#include <so3_control/HGDO.h>
#include <so3_control/ADO.h>
#include <uav_utils/converters.h>
#include <string>
#include <iostream>
#include <filesystem>
#include <regex>
#include <fstream>
#include <thread>
#include <mutex>
#include <algorithm>

#define ONE_G 9.81

// 干扰观测器: 由参数 disturbance_observer_type 选 ADO 或 HGDO, 接口与 HGDO 相同
class DisturbanceObserver
{
public:
    DisturbanceObserver(double control_dt = 0.02, bool use_ado = true)
        : use_ado_(use_ado), hgdo_(control_dt), ado_(control_dt){};

    void HGDO_ext_force_ob(const Eigen::Vector3d &U_input, const Eigen::Vector3d &vel, Eigen::Vector3d &dis)
    {
        if (use_ado_)
            ado_.ADO_ext_force_ob(U_input, vel, dis);
        else
            hgdo_.HGDO_ext_force_ob(U_input, vel, dis);
    }

private:
    bool use_ado_;
    HGDO hgdo_;
    ADO ado_;
};

class NetworkControl
{
public:
    NetworkControl();
    ~NetworkControl() = default;

    // Two-phase init: needs shared_from_this()-free access to *this, so it is called after construction.
    void init();

private:
    using PositionCommand = quadrotor_msgs::PositionCommand;
    using SO3Command = quadrotor_msgs::SO3Command;
    using SetTakeoffLand = quadrotor_msgs::SetTakeoffLand;

    ros::Publisher so3_command_pub_;
    ros::Subscriber position_cmd_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber respawn_sub_;
    ros::Subscriber arm_sub_;
    bool armed_ = false;
    void arm_callback(const std_msgs::Bool::ConstPtr msg);
    ros::WallTimer takeoff_land_control_timer_;
    ros::Publisher controller_fault_pub_;
    using SteadyClock = std::chrono::steady_clock;
    SteadyClock::time_point last_command_wall_ = SteadyClock::now();
    ros::Time last_command_stamp_{0};
    bool watchdog_fault_ = false;
    double command_timeout_ = 0.3;
    Eigen::Vector3d stop_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d stop_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d stop_acc_ = Eigen::Vector3d::Zero();
    void publishControllerFault(bool active, const std::string& reason);
    void latchCommandTimeout();
    void watchdogStopTick();
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_{"~"};
    std::mutex mutex_;

    double mass_ = 0.98;
    double control_dt_ = 0.02;
    double hover_thrust_ = 0.4;
    double kx_xy, kx_z, kv_xy, kv_z;

    double cur_yaw_ = 0;
    Eigen::Vector3d cur_pos_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d cur_vel_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d cur_acc_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Quaterniond cur_att_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d dis_acc_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d last_des_acc_ = Eigen::Vector3d(0, 0, 0);
    double last_thrust_ = 0;

    Eigen::Vector3d des_pos_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d des_vel_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d des_acc_ = Eigen::Vector3d(0, 0, 0);
    double des_yaw_ = 0;
    double des_yaw_dot_ = 0;

    bool is_simulation_ = false;
    bool state_init_ = false;
    bool ref_valid_ = false;
    bool ctrl_valid_ = false;
    bool position_cmd_init_ = false;
    bool takeoff_cmd_init_ = false;
    bool use_disturbance_observer_ = false;
    bool use_ado_ = true;
    bool record_log_ = false;
    // respawn 后的"跟随"窗口: 期望位置每帧贴住当前位置, 窗口结束时最后一次贴合即新的悬停点
    ros::Time respawn_hold_end_{0};
    const double respawn_hold_time_ = 0.3;

    SO3Control so3_controller_;
    DisturbanceObserver disturbance_observer_;

    std::ofstream logger;
    std::string logger_file_name;

    void initLogRecorder();

    void recordLog(Eigen::Vector3d &cur_v, Eigen::Vector3d &cur_a, Eigen::Vector3d &des_a, Eigen::Vector3d &dis_a, double cur_yaw, double des_yaw);

    Eigen::Vector3d publishHoverSO3Command(Eigen::Vector3d des_pos, Eigen::Vector3d des_vel, Eigen::Vector3d des_acc, double des_yaw, double des_yaw_dot);

    void get_Q_from_ACC(const Eigen::Vector3d &ref_acc, double ref_yaw, Eigen::Quaterniond &quat_des, Eigen::Vector3d &force_des);

    void pub_SO3_command(Eigen::Vector3d ref_acc, double ref_yaw, double cur_yaw);

    void limite_acc(Eigen::Vector3d &acc);

    void network_cmd_callback(const PositionCommand::ConstPtr cmd);

    void odom_callback(const nav_msgs::Odometry::ConstPtr odom);

    void imu_callback(const sensor_msgs::Imu::ConstPtr imu);

    void respawn_callback(const std_msgs::Bool::ConstPtr msg);

    bool in_respawn_hold() { return ros::Time::now() < respawn_hold_end_; }

    void timerCallback();

};

#endif

