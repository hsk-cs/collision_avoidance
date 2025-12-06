/**
 * 集成避障功能的无人机控制器
 * 功能：起飞 → 悬停 → 选择模式 → 前进（避障/位置控制）→ 降落
 * 控制模式：位置控制（默认） / 速度+避障控制（可选）
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>

using namespace std;

// ==================== 常量定义 ====================
#define ALTITUDE 1.2f           // 默认飞行高度
#define CONTROL_RATE 20.0       // 控制频率

// ==================== 全局变量声明 ====================
// MAVROS状态与位置
mavros_msgs::State current_state;
nav_msgs::Odometry local_pos;
mavros_msgs::PositionTarget setpoint_raw;

// 起飞初始位置
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;

// TF变换
tf::Quaternion quat;
double roll, pitch, yaw;

// 避障相关变量
sensor_msgs::LaserScan laser_data;
Eigen::Quaterniond q_fcu;
Eigen::Vector3d euler_fcu; // 无人机欧拉角 [roll, pitch, yaw]

// 避障参数
float target_x = 5.0;               // 目标点X坐标
float target_y = 0.0;               // 目标点Y坐标
float R_outside = 2.0;              // 外警戒半径
float R_inside = 1.0;               // 内警戒半径
float p_xy = 0.5;                   // 位置跟踪P增益
float vel_track_max = 0.5;          // 跟踪速度限幅
float p_R = 0.5;                    // 外圈排斥力增益
float p_r = 1.0;                    // 内圈排斥力增益
float vel_collision_max = 0.5;      // 避障速度限幅
float vel_sp_max = 1.0;             // 总速度限幅
int range_min = 0;                  // 激光雷达有效范围起始索引
int range_max = 359;                // 激光雷达有效范围结束索引
bool enable_avoidance = false;      // 是否启用避障功能
bool use_avoidance_in_mission = false; // 任务中是否使用避障

// 避障计算中间变量
float distance_c = 10.0;            // 最近障碍物距离
float angle_c = 0.0;                // 最近障碍物角度(度)
float distance_cx = 0.0;            // 障碍物在机体系X分量
float distance_cy = 0.0;            // 障碍物在机体系Y分量
float vel_track[2] = {0, 0};        // 跟踪速度
float vel_collision[2] = {0, 0};    // 避障速度
float vel_sp_body[2] = {0, 0};      // 机体系总速度
float vel_sp_ENU[2] = {0, 0};       // ENU系总速度

// 任务控制变量
int mission_num = 0;                // 任务状态
float if_debug = 0;                 // 调试模式
float err_max = 0.2;                // 位置控制误差容限
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
ros::Time precision_land_last_time;
ros::Time mission_last_time;

// ==================== 函数声明 ====================
// 回调函数
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &scan);

// 任务控制函数
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool precision_land();

// 避障相关函数
void cal_min_distance();
void collision_avoidance(float target_x, float target_y);
float satfunc(float data, float Max);
void rotation_yaw(float yaw_angle, float input[2], float output[2]);
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q);

// 工具函数
void print_param();

// ==================== 函数定义 ====================

/************************************************************************
 * 状态回调函数
 *************************************************************************/
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

/************************************************************************
 * 位置回调函数
 *************************************************************************/
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    
    // 提取欧拉角（用于原有任务逻辑）
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    // 提取四元数（用于避障逻辑）
    q_fcu = Eigen::Quaterniond(
        local_pos.pose.pose.orientation.w,
        local_pos.pose.pose.orientation.x,
        local_pos.pose.pose.orientation.y,
        local_pos.pose.pose.orientation.z
    );
    
    // 转换为欧拉角
    euler_fcu = quaternion_to_euler(q_fcu);
    
    // 记录起飞初始位置
    if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
        ROS_INFO("起飞点已记录: (%.2f, %.2f, %.2f), 偏航: %.2f度", 
                init_position_x_take_off, init_position_y_take_off, 
                init_position_z_take_off, init_yaw_take_off * 180.0 / M_PI);
    }
}

/************************************************************************
 * 激光雷达回调函数
 *************************************************************************/
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    if (!enable_avoidance) return;
    
    sensor_msgs::LaserScan laser_tmp = *scan;
    laser_data = *scan;
    int count = laser_data.ranges.size();
    
    // 处理无效数据
    for(int i = 0; i < count; i++)
    {
        if(isinf(laser_tmp.ranges[i]) || isnan(laser_tmp.ranges[i]))
        {
            if(i == 0) laser_tmp.ranges[i] = laser_tmp.ranges[count-1];
            else laser_tmp.ranges[i] = laser_tmp.ranges[i-1];
        }
    }
    
    // 调整坐标系（前向为0度）
    for(int i = 0; i < count; i++)
    {
        int adjusted_idx = (i + 180) % 360;
        laser_data.ranges[i] = laser_tmp.ranges[adjusted_idx];
    }
    
    // 计算最近障碍物
    cal_min_distance();
}

/************************************************************************
 * 计算最近障碍物
 *************************************************************************/
void cal_min_distance()
{
    if(laser_data.ranges.empty()) return;
    
    distance_c = laser_data.ranges[range_min];
    angle_c = range_min;
    
    for(int i = range_min; i <= range_max; i++)
    {
        if(laser_data.ranges[i] < distance_c)
        {
            distance_c = laser_data.ranges[i];
            angle_c = i;
        }
    }
}

/************************************************************************
 * 四元数转欧拉角
 *************************************************************************/
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
    return q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX顺序
}

/************************************************************************
 * 饱和函数
 *************************************************************************/
float satfunc(float data, float Max)
{
    if(fabs(data) > Max)
        return (data > 0) ? Max : -Max;
    return data;
}

/************************************************************************
 * 坐标系旋转（机体系到ENU系）
 *************************************************************************/
void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}

/************************************************************************
 * 避障算法核心
 *************************************************************************/
void collision_avoidance(float target_x, float target_y)
{
    // 1. 计算目标追踪速度
    vel_track[0] = p_xy * (target_x - local_pos.pose.pose.position.x);
    vel_track[1] = p_xy * (target_y - local_pos.pose.pose.position.y);
    
    // 速度限幅
    for(int i = 0; i < 2; i++)
    {
        vel_track[i] = satfunc(vel_track[i], vel_track_max);
    }
    
    // 2. 计算障碍排斥速度
    vel_collision[0] = 0;
    vel_collision[1] = 0;
    
    if(distance_c < R_outside)
    {
        // 障碍物在机体系中的坐标
        float angle_rad = (angle_c - 180) * M_PI / 180.0;
        distance_cx = distance_c * cos(angle_rad);
        distance_cy = distance_c * sin(angle_rad);
        
        float F_c = 0;
        if(distance_c > R_inside && distance_c <= R_outside)
        {
            // 外圈：轻度排斥
            F_c = p_R * (R_outside - distance_c);
        }
        else if(distance_c <= R_inside)
        {
            // 内圈：强烈排斥
            F_c = p_R * (R_outside - R_inside) + p_r * (R_inside - distance_c);
        }
        
        // 计算排斥速度（方向远离障碍物）
        if(distance_c > 0)
        {
            vel_collision[0] = -F_c * distance_cx / distance_c;
            vel_collision[1] = -F_c * distance_cy / distance_c;
        }
        
        // 避障速度限幅
        for(int i = 0; i < 2; i++)
        {
            vel_collision[i] = satfunc(vel_collision[i], vel_collision_max);
        }
    }
    
    // 3. 合成总速度（机体系）
    vel_sp_body[0] = vel_track[0] + vel_collision[0];
    vel_sp_body[1] = vel_track[1] + vel_collision[1];
    
    // 总速度限幅
    for(int i = 0; i < 2; i++)
    {
        vel_sp_body[i] = satfunc(vel_sp_body[i], vel_sp_max);
    }
    
    // 4. 转换到ENU坐标系
    rotation_yaw(euler_fcu[2], vel_sp_body, vel_sp_ENU);
}

/************************************************************************
 * 位置巡航控制
 *************************************************************************/
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
    if(mission_pos_cruise_flag == false)
    {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
    }
    
    // 设置控制模式：位置+偏航控制
    setpoint_raw.type_mask = 0b111111000111; // 控制位置和偏航
    setpoint_raw.coordinate_frame = 1; // MAV_FRAME_LOCAL_NED
    setpoint_raw.position.x = x + init_position_x_take_off;
    setpoint_raw.position.y = y + init_position_y_take_off;
    setpoint_raw.position.z = z + init_position_z_take_off;
    setpoint_raw.yaw = target_yaw + init_yaw_take_off;
    
    ROS_DEBUG_THROTTLE(1.0, "位置控制: 当前(%.2f,%.2f,%.2f) -> 目标(%.2f,%.2f,%.2f)", 
                      local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, 
                      local_pos.pose.pose.position.z, setpoint_raw.position.x, 
                      setpoint_raw.position.y, setpoint_raw.position.z);
    
    // 检查是否到达目标点
    if(fabs(local_pos.pose.pose.position.x - setpoint_raw.position.x) < error_max &&
       fabs(local_pos.pose.pose.position.y - setpoint_raw.position.y) < error_max &&
       fabs(local_pos.pose.pose.position.z - setpoint_raw.position.z) < error_max &&
       fabs(yaw - setpoint_raw.yaw) < 0.1)
    {
        mission_pos_cruise_flag = false;
        return true;
    }
    return false;
}

/************************************************************************
 * 精确降落
 *************************************************************************/
bool precision_land()
{
    if(!precision_land_init_position_flag)
    {
        precision_land_init_position_x = local_pos.pose.pose.position.x;
        precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
        precision_land_init_position_flag = true;
        ROS_INFO("开始精确降落，保持位置: (%.2f, %.2f)", 
                precision_land_init_position_x, precision_land_init_position_y);
    }
    
    // 设置降落目标
    setpoint_raw.position.x = precision_land_init_position_x;
    setpoint_raw.position.y = precision_land_init_position_y;
    setpoint_raw.position.z = -0.15; // 地面高度
    
    // 使用位置控制模式
    setpoint_raw.type_mask = 0b110111111000;
    setpoint_raw.coordinate_frame = 1;
    
    // 检查是否完成降落（5秒后判断）
    if(ros::Time::now() - precision_land_last_time > ros::Duration(5.0))
    {
        ROS_INFO("精确降落完成");
        precision_land_init_position_flag = false;
        return true;
    }
    return false;
}

/************************************************************************
 * 参数打印函数
 *************************************************************************/
void print_param()
{
    cout << "========== 控制参数 ==========" << endl;
    cout << "飞行高度: " << ALTITUDE << " m" << endl;
    cout << "位置误差容限: " << err_max << " m" << endl;
    cout << "调试模式: " << (if_debug == 1 ? "自动" : "遥控") << endl;
    
    if(enable_avoidance)
    {
        cout << "========== 避障参数 ==========" << endl;
        cout << "目标点: (" << target_x << ", " << target_y << ") m" << endl;
        cout << "外警戒半径: " << R_outside << " m" << endl;
        cout << "内警戒半径: " << R_inside << " m" << endl;
        cout << "位置P增益: " << p_xy << endl;
        cout << "跟踪速度限幅: " << vel_track_max << " m/s" << endl;
        cout << "外圈排斥增益: " << p_R << endl;
        cout << "内圈排斥增益: " << p_r << endl;
        cout << "避障速度限幅: " << vel_collision_max << " m/s" << endl;
        cout << "总速度限幅: " << vel_sp_max << " m/s" << endl;
    }
    else
    {
        cout << "避障功能: 已禁用" << endl;
    }
    cout << "===============================" << endl;
}

/************************************************************************
 * 主函数
 *************************************************************************/
int main(int argc, char **argv)
{
    // 防止中文输出乱码
    setlocale(LC_ALL, "");
    
    // 初始化ROS节点
    ros::init(argc, argv, "template");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    // ==================== 参数读取 ====================
    // 基础控制参数
    nh_private.param<float>("err_max", err_max, 0.2);
    nh_private.param<float>("if_debug", if_debug, 0);
    
    // 避障参数
    nh_private.param<bool>("enable_avoidance", enable_avoidance, false);
    nh_private.param<float>("target_x", target_x, 5.0);
    nh_private.param<float>("target_y", target_y, 0.0);
    nh_private.param<float>("R_outside", R_outside, 2.0);
    nh_private.param<float>("R_inside", R_inside, 1.0);
    nh_private.param<float>("p_xy", p_xy, 0.5);
    nh_private.param<float>("vel_track_max", vel_track_max, 0.5);
    nh_private.param<float>("p_R", p_R, 0.5);
    nh_private.param<float>("p_r", p_r, 1.0);
    nh_private.param<float>("vel_collision_max", vel_collision_max, 0.5);
    nh_private.param<float>("vel_sp_max", vel_sp_max, 1.0);
    nh_private.param<int>("range_min", range_min, 0);
    nh_private.param<int>("range_max", range_max, 359);
    
    // ==================== 订阅者 ====================
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, lidar_cb);
    
    // ==================== 发布者 ====================
    ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
    
    // ==================== 服务客户端 ====================
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
    
    // 设置控制频率
    ros::Rate rate(CONTROL_RATE);
    
    // 打印参数
    print_param();
    
    // ==================== 用户确认 ====================
    int choice = 0;
    cout << "输入 1 继续，其他退出: ";
    cin >> choice;
    if (choice != 1) return 0;
    
    ros::spinOnce();
    rate.sleep();
    
    // ==================== 等待连接飞控 ====================
    ROS_INFO("等待连接飞控...");
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("飞控连接成功!");
    
    // ==================== 设置初始位置 ====================
    setpoint_raw.type_mask = 0b110111111000; // 控制位置和偏航
    setpoint_raw.coordinate_frame = 1; // MAV_FRAME_LOCAL_NED
    setpoint_raw.position.x = 0;
    setpoint_raw.position.y = 0;
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0;
    
    // 发送初始设定点（解锁前必需）
    ROS_INFO("发送初始设定点...");
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    cout << "ok" << endl;
    
    // ==================== 起飞准备 ====================
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();
    mission_last_time = ros::Time::now();
    
    // ==================== 起飞阶段 ====================
    while (ros::ok())
    {
        // 切换到OFFBOARD模式
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if(if_debug == 1) // 自动模式
            {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
            }
            else // 等待遥控器切换
            {
                ROS_INFO_THROTTLE(1.0, "等待遥控器切换至OFFBOARD模式");
            }
            last_request = ros::Time::now();
        }
        // 解锁电机
        else if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        
        // 到达起飞高度后进入任务模式
        if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2)
        {
            if (ros::Time::now() - last_request > ros::Duration(1.0))
            {
                mission_num = 1; // 进入悬停状态
                last_request = ros::Time::now();
                ROS_INFO("到达起飞高度，准备执行任务");
                
                // 询问是否启用避障
                if(enable_avoidance)
                {
                    int avoidance_choice = 0;
                    cout << "是否启用避障前进? 1启用, 0仅位置控制: ";
                    cin >> avoidance_choice;
                    use_avoidance_in_mission = (avoidance_choice == 1);
                    
                    if(use_avoidance_in_mission)
                        ROS_INFO("避障模式已启用");
                    else
                        ROS_INFO("将使用标准位置控制");
                }
                
                break; // 退出起飞循环，进入任务循环
            }
        }
        
        // 起飞控制
        mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    
    // ==================== 任务执行阶段 ====================
    while (ros::ok())
    {
        ROS_WARN("mission_num = %d", mission_num);
        
        switch (mission_num)
        {
            // 状态1: 起飞点悬停
            case 1:
                if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
                {
                    mission_num = 2; // 进入前进状态
                    last_request = ros::Time::now();
                    ROS_INFO("悬停完成，准备前进");
                }
                else if(ros::Time::now() - last_request >= ros::Duration(3.0))
                {
                    mission_num = 2; // 超时保护
                    last_request = ros::Time::now();
                    ROS_WARN("悬停超时，继续前进");
                }
                break;
            
            // 状态2: 前进（分避障和位置控制两种模式）
            case 2:
                if(use_avoidance_in_mission && enable_avoidance)
                {
                    // 模式A：避障速度控制
                    collision_avoidance(target_x, target_y);
                    
                    // 切换到速度控制模式
                    setpoint_raw.type_mask = 0b110111111000; // 控制速度，保持高度
                    setpoint_raw.velocity.x = vel_sp_ENU[0];
                    setpoint_raw.velocity.y = vel_sp_ENU[1];
                    setpoint_raw.position.z = ALTITUDE;
                    setpoint_raw.yaw = 0;
                    
                    // 显示避障信息
                    ROS_INFO_THROTTLE(0.5, "避障控制 | 速度: (%.2f, %.2f) m/s | 障碍物: %.2f m @ %.0f°", 
                                     vel_sp_ENU[0], vel_sp_ENU[1], distance_c, angle_c);
                    
                    // 检查是否到达目标点
                    float dx = target_x - local_pos.pose.pose.position.x;
                    float dy = target_y - local_pos.pose.pose.position.y;
                    float distance_to_target = sqrt(dx*dx + dy*dy);
                    
                    if(distance_to_target < 0.3)
                    {
                        mission_num = 3; // 进入降落
                        last_request = ros::Time::now();
                        ROS_INFO("到达目标点附近，准备降落");
                    }
                    else if(ros::Time::now() - last_request > ros::Duration(30.0))
                    {
                        mission_num = 3; // 超时保护
                        ROS_WARN("前进超时，强制进入降落");
                    }
                }
                else
                {
                    // 模式B：标准位置控制
                    if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0.0, err_max))
                    {
                        mission_num = 3; // 进入降落
                        last_request = ros::Time::now();
                        ROS_INFO("到达目标点，准备降落");
                    }
                }
                break;
            
            // 状态3: 降落
            case 3:
                if(precision_land())
                {
                    mission_num = -1; // 任务完成
                    ROS_INFO("任务完成!");
                }
                break;
            
            default:
                break;
        }
        
        // 发布控制指令
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        
        // 任务完成检查
        if(mission_num == -1) 
        {
            ROS_INFO("程序退出");
            ros::Duration(2.0).sleep(); // 等待降落稳定
            break;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
