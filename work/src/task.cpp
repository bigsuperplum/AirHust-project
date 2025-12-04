#include <template.h>
#include <collision_avoidance.h>
#include <crossing_door.h>

// 全局变量定义
// int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;
void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  if(if_debug == 1) cout << "自动offboard" << std::endl;
  else cout << "遥控器offboard" << std::endl;
}




int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  // 初始化ROS节点
  ros::init(argc, argv, "template");
  ros::NodeHandle nh;

  //【避障】Lidar数据
  ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1000, lidar_cb);

  //【避障】无人机当前位置 坐标系 NED系
  ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
//  ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 100, vel_cb);
  

  // 订阅mavros相关话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
// ros::Publisher mavros_setpoint_vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel", 100);

  // 创建服务客户端
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取
  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("if_debug", if_debug, 0);

  //读取参数表中的参数
  nh.param<float>("target_x", target_x, 1.0); //dyx
  nh.param<float>("target_y", target_y, 0.0); //dyx

  nh.param<float>("R_outside", R_outside, 2);
  nh.param<float>("R_inside", R_inside, 1);

  nh.param<float>("p_xy", p_xy, 0.5);

  nh.param<float>("vel_track_max", vel_track_max, 0.5);

  nh.param<float>("p_R", p_R, 0.0);
  nh.param<float>("p_r", p_r, 0.0);

  nh.param<float>("vel_collision_max", vel_collision_max, 0.0);
  nh.param<float>("vel_sp_max", vel_sp_max, 0.0);

  nh.param<float>("fly_height", fly_height, 0.5);

  print_param();

  
  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1) return 0;
  ros::spinOnce();
  rate.sleep();
  
  // 等待连接到飞控
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  //设置无人机的期望位置
 
  setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
  setpoint_raw.coordinate_frame = 1;
  setpoint_raw.position.x = 0;
  setpoint_raw.position.y = 0;
  setpoint_raw.position.z = ALTITUDE;
  setpoint_raw.yaw = 0;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout<<"ok"<<std::endl;

  // 定义客户端变量，设置为offboard模式
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 定义客户端变量，请求无人机解锁
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 记录当前时间，并赋值给变量last_request
  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
      if(if_debug == 1)
      {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
      }
      else
      {
        ROS_INFO("Waiting for OFFBOARD mode");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
    if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.3)
    {
      if (ros::Time::now() - last_request > ros::Duration(1.0))
      {
        // mission_num = 1;
 	      last_request = ros::Time::now();
        break;
      }
    }

    mission_pos_cruise(0, 0, ALTITUDE, 0, err_max); 
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  
  float abs_distance = 0;

  while (ros::ok())
  {
    if (!mission_num){
      ROS_WARN("请输入要执行的任务编号：");
      std::cin >> mission_num;
    }
    ROS_WARN("mission_num = %d", mission_num);
    switch (mission_num)
    {
      // mission1: 起飞
      case 1:
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
        {
          mission_num = 0;
          last_request = ros::Time::now();
        }
	    else if(ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num = 0;
          last_request = ros::Time::now();
        }
        break;

      // mission2: collision_avoidance
      case 2:
        //回调一次 更新传感器状态
        //1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
        ros::spinOnce();
        collision_avoidance(target_x,target_y);
        printf();
        abs_distance = sqrt(pow((pos_drone.pose.position.x - target_x), 2) + pow((pos_drone.pose.position.y - target_y), 2));
        if (abs_distance < err_max)
        {
          mission_num = 0; // 任务结束
          last_request = ros::Time::now();
        }
        break;

      // mission3: crossing_door
      case 3:
        ros::spinOnce();
        abs_distance = sqrt(pow((pos_drone.pose.position.x - 10.0), 2) + pow((pos_drone.pose.position.y - 0.0), 2));
        if (abs_distance >= err_max)
          collision_avoidance(10.0,0.0);
        else {
          if ((!door_center_x[0]))
            finddoorcenter(0);
          collision_avoidance(door_center_x[0] + 0.1,door_center_y[0]);
          abs_distance = sqrt(pow((pos_drone.pose.position.x - (door_center_x[0] + 0.1)), 2) + pow((pos_drone.pose.position.y - door_center_y[0]), 2));
          if (abs_distance < err_max) {
            mission_num = 0; // 任务结束
            last_request = ros::Time::now();
          }
        }
        break;


      //降落
      case 6:
        if(precision_land())
        {
          mission_num = -1; // 任务结束
          last_request = ros::Time::now();
        }
        break;
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    // mavros_setpoint_vel_pub.publish(setpoint_vel);
    ros::spinOnce();
    rate.sleep();
    
    if(mission_num == -1) 
    {
      exit(0);
    }
  }
  return 0;
}


