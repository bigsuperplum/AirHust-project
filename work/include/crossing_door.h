#include <ros/ros.h>

//topic 头文件
#include <iostream>
#include <px4_command/command.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <stdlib.h>

//#include <darknet_ros_msgs/BoundingBoxes.h> //目标检测
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <math_utils.h>


using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
enum Command
{
    Move_ENU,
    Move_Body,
    Hold,
    Takeoff,
    Land,
    Arm,
    Disarm,
    Failsafe_land,
    Idle
};

#define RAD2DEG(x) ((x)*180./M_PI)
#define Height 480
#define Width 640
//--------------------------------------------输入--------------------------------------------------
float door_x;
float door_y;
//--------------------------------------------穿门避障用-------------------------------------------------
float door_center_x[2];                                         //前两道门的x坐标
float door_center_y[2];                                         //前两道门的y坐标
bool reach_door_flag[2];                                        //到达前两道门的标志
float fly_height;                                               //设定的飞行高度
Eigen::Quaterniond q_fcu;                                       //飞机姿态四元数
Eigen::Vector3d Euler_fcu;                                      //飞机姿态欧拉角

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void finddoorcentor(int i);                                                          //通过激光雷达找到门口中心

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


    //读取参数表中的参数



    nh.param<float>("door_x", door_x, 1.0);
    nh.param<float>("door_y", door_y, 1.7);

    //获取设定的起飞高度
    nh.getParam("/px4_pos_controller/Takeoff_height",fly_height);

    //打印现实检查参数
    printf_param();


    //初值
    vel_track[0]= 0;
    vel_track[1]= 0;

    vel_collision[0]= 0;
    vel_collision[1]= 0;

    vel_sp_body[0]= 0;
    vel_sp_body[1]= 0;

    vel_sp_ENU[0]= 0;
    vel_sp_ENU[1]= 0;

    //四向最小距离 初值
    flag_land = 0;


    //输出指令初始化
    int comid = 1;


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        //1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
        ros::spinOnce();
        /**************************dyx****************************************/ //change: just cross one door
        //策略：穿门原理，当穿完最后一道门对墙面人像图片进行识别并计算图片中心xy坐标，导航过去。
        /*
        if(!reach_door_flag[0]) finddoorcentor(0);
        else if(reach_door_flag[0]&&!reach_door_flag[1])
        {
           if(detect_num&&abs_distance1<0.3)
           {
              cout<<"detectnum "<<detect_num<<endl;
              reach_door_flag[1]=true;
           }
              else finddoorcentor(1);
              abs_distance1 = sqrt((pos_drone.pose.position.x - door_center_x[1]) * (pos_drone.pose.position.x - door_center_x[1]) + (pos_drone.pose.position.y - door_center_y[1]) * (pos_drone.pose.position.y - door_center_y[1]));
        }
        else if(reach_door_flag[0]&&reach_door_flag[1]) detect_nav();
        */
        if(!reach_door_flag[0]) //finddoorcentor(0);
        {
           collision_avoidance(door_x+0.1,door_y);
           float abs_distance;
           abs_distance = sqrt((pos_drone.pose.position.x - door_x - 0.1) * (pos_drone.pose.position.x -door_x - 0.1) + (pos_drone.pose.position.y - door_y) * (pos_drone.pose.position.y - door_y));
           if(abs_distance < 0.3)
           {
              reach_door_flag[0] = true;
           }
        }
        // else if(reach_door_flag[0])
        // {
        //    collision_avoidance(final_x,final_y);
        //    float abs_distance;
        //    abs_distance = sqrt((pos_drone.pose.position.x - final_x) * (pos_drone.pose.position.x -final_x) + (pos_drone.pose.position.y - final_y) * (pos_drone.pose.position.y - final_y));
        //    if(abs_distance < 0.3 )
        //    {
        //       flag_land = 1;
        //    }
        // }


        /**************************dyx****************************************/
        //弃用Body下的指令
        //5. 发布Command指令给position_controller.cpp
/*      Command_now.command = Move_Body;     //机体系下移动
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
        Command_now.vel_sp[0] =  vel_sp_body[0];
        Command_now.vel_sp[1] =  vel_sp_body[1];  //ENU frame
        Command_now.pos_sp[2] =  0;
        Command_now.yaw_sp = 0 ;
*/
        //启用ENU下的指令
    //     Command_now.command = Move_ENU;     //机体系下移动
    //     Command_now.comid = comid;
    //     comid++;
    //     Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
    //     Command_now.vel_sp[0] =  vel_sp_ENU[0];
    //     Command_now.vel_sp[1] =  vel_sp_ENU[1];  //ENU frame
    //     Command_now.pos_sp[2] =  fly_height;
    //     Command_now.yaw_sp = 0 ;


    //     if(flag_land == 1) Command_now.command = Land;

    //     command_pub.publish(Command_now);

    //     //打印
    //     printf();

    //     rate.sleep();

    // }
//思路：门相对于墙的激光数据会有很大的突变，在激光数据里找到这个突变范围，再转换到ENU坐标系下即可求出门的中心
void finddoorcenter(int i)
{
    //1.if no center , findcenter set target
    //2.use collision_avoidance
    //3.judge whether reach target

    //1.
    float a,b,c;
    double l;
    cout << "********************" << endl;
    if (!door_center_x[i])
    {
        a = Laser.ranges[0];
        b = Laser.ranges[90];//why 89? why not 90?
        c = Laser.ranges[270];
        int theta1 = atan(b/a)/3.1415926*180;
        int theta2 = atan(c/a)/3.1415926*180;
        cout << "theta1: " << theta1 << endl;
        cout << "theta2: " << theta2 << endl;
        std::vector<int> door_angle;
        door_angle.reserve(theta1 + theta2);
        for (int k = theta1; k > 0; k--) {
            float angle = k;
            l = a / cos(angle / 180 * 3.1415926);
            float dl = abs(l - Laser.ranges[k]);
            if (dl > 1) 
                door_angle.push_back(k);
            //cout<<"k: "<<k<<" l: "<<l<<" Laser: "<<Laser.ranges[k]<<" dl: "<<dl<<endl;
        }
        for(int k = 0; k <= theta2; k++) {
            float angle = k;
            l = a / cos(angle / 180 * 3.1415926);
            float dl = abs(l - Laser.ranges[359 - k]);
            if (dl > 1) 
                door_angle.push_back(359 - k);
            //cout<<"k: "<<359-k<<" l: "<<l<<" Laser: "<<Laser.ranges[359-k]<<" dl: "<<dl<<endl;
        }
        cout << "door angle num: " << door_angle.size() << endl;
        cout << "first:" << door_angle.front() << "last one: " << door_angle.back() << endl;
        int the1 = door_angle.front();
        int the2 = door_angle.back();
        float angle1, angle2;
        float x1, x2, y1, y2;
        x1 = a;
        x2 = a;
        if (the1 > 270)
        {
            angle1 = 359 - the1;
            y1 = -a * tan(angle1 / 180 * 3.1415926);
        }
        else
        {
            angle1 = the1;
            y1 = a * tan(angle1 / 180 * 3.1415926);
        }
        if (the2 > 270)
        {
            angle2 = 359 - the2;
            y2 = -a * tan(angle2 / 180 * 3.1415926);
        }
        else
        {
            angle2 = the2;
            y2 = a * tan(angle2 / 180 * 3.1415926);
        }


        cout << "x1 y1: " << x1 << " " << y1 << endl;
        cout << "x2 y2: " << x2 << " " << y2 << endl;

        door_center_x[i] = (x1 + x2) / 2 + pos_drone.pose.position.x;
        door_center_y[i] = (y1 + y2) / 2 + pos_drone.pose.position.y;
        cout << "door position: " << door_center_x[i] << " " << door_center_y[i] << endl;
    }
    collision_avoidance(door_center_x[i]+0.1,door_center_y[i]);
    float abs_distance;
    abs_distance = sqrt(pow((pos_drone.pose.position.x - door_center_x[i]-0.1), 2) + pow((pos_drone.pose.position.y - door_center_y[i]), 2));
    //cout<<"abs_distance: "<<abs_distance<<endl;
    cout << "door position: " << door_center_x[i] << " " << door_center_y[i] << endl;
    if(abs_distance < 0.3)
    {
        reach_door_flag[i] = true;
    }
}