// 整合所有头文件内容到 template.cpp
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <stdlib.h>
#include <math_utils.h>

using namespace std;

// 来自 template.h 的常量定义
#define ALTITUDE 1.2f

// 来自 template.h 的全局变量
mavros_msgs::PositionTarget setpoint_raw;
int mission_num = 0;

struct mission_point {
    float x;
    float y;
} MissionPoint[7] = {
    {0, 0},
    {0, 0},
    {0, 0},
    {8, 0},
    {16, 0},
    {24, 0},
    {32, 0}
};

// 来自 template.h 的变量声明
mavros_msgs::State current_state;
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;

// 来自 collision_avoidance.h 的变量声明
sensor_msgs::LaserScan Laser;
geometry_msgs::PoseStamped pos_drone;
Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;
float target_x = 8.0;
float target_y = 0.0;
int range_min = 0;
int range_max = 359;
float last_time = 0;
float fly_height = ALTITUDE;  // 使用template.h中的常量

float R_outside = 2.0, R_inside = 1.0;
float p_R = 1.0;
float p_r = 2.0;

float distance_c = 0.0, angle_c = 0.0;
float distance_cx = 0.0, distance_cy = 0.0;
float vel_collision[2] = {0.0, 0.0};
float vel_collision_max = 1.0;

float p_xy = 0.5;
float vel_track[2] = {0.0, 0.0};
float vel_track_max = 1.0;
int flag_land = 0;

std_msgs::Bool flag_collision_avoidance;
float vel_sp_body[2] = {0.0, 0.0};
float vel_sp_ENU[2] = {0.0, 0.0};
float vel_sp_max = 1.0;

// 来自 template.h 的函数声明
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
double xy_d(int mission_point_id);
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool precision_land();

// 来自 collision_avoidance.h 的函数声明
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan);
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void cal_min_distance();
float satfunc(float data, float Max);
void collision_avoidance(float target_x, float target_y);
void rotation_yaw(float yaw_angle, float input[2], float output[2]);
void printf();
void printf_param();

// 来自 template.cpp 的变量
float if_debug = 0;
float err_max = 0.2;

// ================ 函数实现 ================

// 来自 template.h 的函数实现
void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (flag_init_position == false && (local_pos.pose.pose.position.z != 0)) {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
    }
}

double xy_d(int mission_point_id) {
    return sqrt( pow((local_pos.pose.pose.position.x - MissionPoint[mission_point_id].x), 2)
                + pow((local_pos.pose.pose.position.y - MissionPoint[mission_point_id].y), 2) );
}

float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max) {
    if (mission_pos_cruise_flag == false) {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
    }
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = x + init_position_x_take_off;
    setpoint_raw.position.y = y + init_position_y_take_off;
    setpoint_raw.position.z = z + init_position_z_take_off;
    setpoint_raw.yaw = target_yaw;
    ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", 
             local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, 
             local_pos.pose.pose.position.z, target_yaw * 180.0 / M_PI, 
             x + init_position_x_take_off, y + init_position_y_take_off, 
             z + init_position_z_take_off, target_yaw * 180.0 / M_PI);
    
    if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && 
        fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && 
        fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max && 
        fabs(yaw - target_yaw) < 0.1) {
        ROS_INFO("到达目标点，巡航点任务完成");
        mission_pos_cruise_flag = false;
        return true;
    }
    return false;
}

float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
ros::Time precision_land_last_time;
bool precision_land() {
    if (!precision_land_init_position_flag) {
        precision_land_init_position_x = local_pos.pose.pose.position.x;
        precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
        precision_land_init_position_flag = true;
    }
    setpoint_raw.position.x = precision_land_init_position_x;
    setpoint_raw.position.y = precision_land_init_position_y;
    setpoint_raw.position.z = -0.15;
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    
    if (ros::Time::now() - precision_land_last_time > ros::Duration(5.0)) {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false;
        return true;
    }
    return false;
}

// 来自 collision_avoidance.h 的函数实现
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan) {
    sensor_msgs::LaserScan Laser_tmp;
    Laser_tmp = *scan;
    Laser = *scan;
    int count = Laser.ranges.size();

    // 剔除inf的情况
    for (int i = 0; i < count; i++) {
        int a = isinf(Laser_tmp.ranges[i]);
        if (a == 1) {
            if (i == 0) {
                Laser_tmp.ranges[i] = Laser_tmp.ranges[count-1];
            } else {
                Laser_tmp.ranges[i] = Laser_tmp.ranges[i-1];
            }
        }
    }
    
    for (int i = 0; i < count; i++) {
        if (i+180 > 359) {
            Laser.ranges[i] = Laser_tmp.ranges[i-180];
        } else {
            Laser.ranges[i] = Laser_tmp.ranges[i+180];
        }
    }
    
    cal_min_distance();
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    pos_drone = *msg;
    Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, 
                                 msg->pose.orientation.y, msg->pose.orientation.z);
    q_fcu = q_fcu_enu;
    Euler_fcu = quaternion_to_euler(q_fcu);
}

void cal_min_distance() {
    distance_c = Laser.ranges[range_min];
    angle_c = 0;
    for (int i = range_min; i <= range_max; i++) {
        if (Laser.ranges[i] < distance_c) {
            distance_c = Laser.ranges[i];
            angle_c = i;
        }
    }
}

float satfunc(float data, float Max) {
    if (abs(data) > Max) return (data > 0) ? Max : -Max;
    else return data;
}

void rotation_yaw(float yaw_angle, float input[2], float output[2]) {
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}

void collision_avoidance(float target_x, float target_y) {
    // 2. 根据最小距离判断：是否启用避障策略
    if (distance_c >= R_outside) {
        flag_collision_avoidance.data = false;
    } else {
        flag_collision_avoidance.data = true;
    }

    // 3. 计算追踪速度
    vel_track[0] = p_xy * (target_x - pos_drone.pose.position.x);
    vel_track[1] = p_xy * (target_y - pos_drone.pose.position.y);

    // 速度限幅
    for (int i = 0; i < 2; i++) {
        vel_track[i] = satfunc(vel_track[i], vel_track_max);
    }
    
    vel_collision[0] = 0;
    vel_collision[1] = 0;

    // 4. 避障策略
    if (flag_collision_avoidance.data == true) {
        distance_cx = distance_c * cos(angle_c/180*3.1415926);
        distance_cy = distance_c * sin(angle_c/180*3.1415926);

        float F_c = 0;

        if (distance_c > R_outside) {
            // 对速度不做限制
            // vel_collision保持不变
            cout << " Forward Outside " << endl;
        } else if (distance_c > R_inside && distance_c <= R_outside) {
            // 小幅度抑制移动速度
            F_c = p_R * (R_outside - distance_c);
        } else if (distance_c <= R_inside) {
            // 大幅度抑制移动速度
            F_c = p_R * (R_outside - R_inside) + p_r * (R_inside - distance_c);
        }

        vel_collision[0] = vel_collision[0] - F_c * distance_cx / distance_c;
        vel_collision[1] = vel_collision[1] - F_c * distance_cy / distance_c;
        
        // 避障速度限幅
        for (int i = 0; i < 2; i++) {
            vel_collision[i] = satfunc(vel_collision[i], vel_collision_max);
        }
    }

    vel_sp_body[0] = vel_track[0] + vel_collision[0];
    vel_sp_body[1] = vel_track[1] + vel_collision[1];

    for (int i = 0; i < 2; i++) {
        vel_sp_body[i] = satfunc(vel_sp_body[i], vel_sp_max);
    }
    
    rotation_yaw(Euler_fcu[2], vel_sp_body, vel_sp_ENU);

    // 设置setpoint_raw消息
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + */ 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.velocity.x = vel_sp_ENU[0];
    setpoint_raw.velocity.y = vel_sp_ENU[1];
    setpoint_raw.yaw = 0;  // 保持当前航向
}

void printf() {
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>collision_avoidance<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "Minimum_distance : " << endl;
    cout << "Distance : " << distance_c << " [m] " << endl;
    cout << "Angle :    " << angle_c << " [du] " << endl;
    cout << "distance_cx :    " << distance_cx << " [m] " << endl;
    cout << "distance_cy :    " << distance_cy << " [m] " << endl;
    
    if (flag_collision_avoidance.data == true) {
        cout << "Collision avoidance Enabled " << endl;
    } else {
        cout << "Collision avoidance Disabled " << endl;
    }
    
    cout << "vel_track_x : " << vel_track[0] << " [m/s] " << endl;
    cout << "vel_track_y : " << vel_track[1] << " [m/s] " << endl;
    cout << "vel_collision_x : " << vel_collision[0] << " [m/s] " << endl;
    cout << "vel_collision_y : " << vel_collision[1] << " [m/s] " << endl;
    cout << "vel_sp_x : " << vel_sp_ENU[0] << " [m/s] " << endl;
    cout << "vel_sp_y : " << vel_sp_ENU[1] << " [m/s] " << endl;
}

void printf_param() {
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "target_x : " << target_x << endl;
    cout << "target_y : " << target_y << endl;
    cout << "R_outside : " << R_outside << endl;
    cout << "R_inside : " << R_inside << endl;
    cout << "p_xy : " << p_xy << endl;
    cout << "vel_track_max : " << vel_track_max << endl;
    cout << "p_R : " << p_R << endl;
    cout << "p_r : " << p_r << endl;
    cout << "vel_collision_max : " << vel_collision_max << endl;
    cout << "vel_sp_max : " << vel_sp_max << endl;
    cout << "range_min : " << range_min << endl;
    cout << "range_max : " << range_max << endl;
    cout << "fly height: " << fly_height << endl;
}

// 来自 template.cpp 的辅助函数
void print_param() {
    std::cout << "=== 控制参数 ===" << std::endl;
    std::cout << "err_max: " << err_max << std::endl;
    std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
    std::cout << "if_debug: " << if_debug << std::endl;
    if (if_debug == 1) cout << "自动offboard" << std::endl;
    else cout << "遥控器offboard" << std::endl;
}

// 主函数
int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "template");
    ros::NodeHandle nh;

    // 订阅
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1000, lidar_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

    // 发布
    ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

    // 服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

    ros::Rate rate(20);

    // 参数读取
    nh.param<float>("err_max", err_max, 0);
    nh.param<float>("if_debug", if_debug, 0);
    
    // 读取避障参数
    nh.param<float>("target_x", target_x, 8.0);
    nh.param<float>("target_y", target_y, 0.0);
    nh.param<float>("R_outside", R_outside, 2.0);
    nh.param<float>("R_inside", R_inside, 1.0);
    nh.param<float>("p_xy", p_xy, 0.5);
    nh.param<float>("vel_track_max", vel_track_max, 1.0);
    nh.param<float>("p_R", p_R, 1.0);
    nh.param<float>("p_r", p_r, 2.0);
    nh.param<float>("vel_collision_max", vel_collision_max, 1.0);
    nh.param<float>("vel_sp_max", vel_sp_max, 1.0);
    nh.param<int>("range_min", range_min, 0);
    nh.param<int>("range_max", range_max, 359);
    nh.param<float>("fly_height", fly_height, ALTITUDE);
    
    print_param();
    printf_param();

    int choice = 0;
    std::cout << "1 to go on , else to quit" << std::endl;
    std::cin >> choice;
    if (choice != 1) return 0;
    
    ros::spinOnce();
    rate.sleep();

    // 等待连接到飞控
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // 设置初始期望位置
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = 0;
    setpoint_raw.position.y = 0;
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0;

    // 发送初始设定点
    for (int i = 100; ros::ok() && i > 0; --i) {
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    
    std::cout << "ok" << std::endl;

    // 切换到OFFBOARD模式和解锁
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0))) {
            if (if_debug == 1) {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                }
            } else {
                ROS_INFO("Waiting for OFFBOARD mode");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // 当无人机到达起飞点高度后，悬停1秒后进入任务模式
        if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2) {
            if (ros::Time::now() - last_request > ros::Duration(1.0)) {
                mission_num = 1;
                last_request = ros::Time::now();
                break;
            }
        }

        mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    // 主任务循环
    while (ros::ok()) {
        if (!mission_num) {
            ROS_WARN("请输入要执行的任务编号：");
            std::cin >> mission_num;
        }
        
        ROS_WARN("mission_num = %d", mission_num);
        
        switch (mission_num) {
            case 1:  // 起飞
                if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max)) {
                    mission_num = 0;
                    last_request = ros::Time::now();
                } else if (ros::Time::now() - last_request >= ros::Duration(3.0)) {
                    mission_num = 0;
                    last_request = ros::Time::now();
                }
                break;

            case 2:  // collision_avoidance
                ros::spinOnce();
                collision_avoidance(target_x, target_y);
                printf();
                {
                    float abs_distance = sqrt(pow((pos_drone.pose.position.x - target_x), 2) + 
                                              pow((pos_drone.pose.position.y - target_y), 2));
                    if (abs_distance < err_max) {
                        mission_num = 0;
                        last_request = ros::Time::now();
                    }
                }
                break;

            case 6:  // 降落
                if (precision_land()) {
                    mission_num = -1;
                    last_request = ros::Time::now();
                }
                break;

            default:
                break;
        }

        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();


        if (mission_num == -1) {
            exit(0);
        }
    }

    return 0;
}