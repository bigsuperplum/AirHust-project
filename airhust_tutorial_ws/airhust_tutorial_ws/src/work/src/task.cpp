#include <template.h>
#include <collision_avoidance.h>
#include <crossing_door.h>
// #include <vision.h>
#include <ring_detection.h>



// void detectAndCrossRing() {
//     // 1. 检测环中心
//     RingDetectionResult ring = findRingCenter();
    
//     if (!ring.found) {
//         // 尝试简易方法
//         ring = findRingCenterSimple();
//     }
    
//     if (ring.found) {
//         // 2. 飞向环中心
//         collision_avoidance(ring.center_x, ring.center_y);
        
//         // 3. 检查是否到达
//         float distance_to_ring = sqrt(
//             pow(pos_drone.pose.position.x - ring.center_x, 2) + 
//             pow(pos_drone.pose.position.y - ring.center_y, 2)
//         );
        
//         if (distance_to_ring < 0.3) {
//             cout << "已到达环中心，开始穿过..." << endl;
            
//             // 4. 穿过环（继续向前飞）
//             // 穿过环后目标点（在环中心前方一段距离）
//             float exit_x = ring.center_x + (ring.center_x - pos_drone.pose.position.x) * 2.0;
//             float exit_y = ring.center_y + (ring.center_y - pos_drone.pose.position.y) * 2.0;
            
//             collision_avoidance(exit_x, exit_y);
//         }
//     } else {
//         cout << "未检测到环，使用预设目标" << endl;
//         // 使用预设目标点
//     }
// }








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

  // // === 新增：视觉话题 ===
  // target_pub = nh.advertise<std_msgs::String>("/target", 10);
  // image_sub = nh.subscribe("/image_raw", 1, imageCallback);

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取
  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("if_debug", if_debug, 0);

  //读取参数表中的参数
  // nh.param<float>("target_x", target_x, 1.0); //dyx
  // nh.param<float>("target_y", target_y, 0.0); //dyx

  nh.param<float>("R_outside", R_outside, 2);
  nh.param<float>("R_inside", R_inside, 1);

  nh.param<float>("p_xy", p_xy, 0.5);

  nh.param<float>("vel_track_max", vel_track_max, 0.5);

  nh.param<float>("p_R", p_R, 0.0);
  nh.param<float>("p_r", p_r, 0.0);

  nh.param<float>("vel_collision_max", vel_collision_max, 0.0);
  nh.param<float>("vel_sp_max", vel_sp_max, 0.0);

  nh.param<float>("fly_height", fly_height, 0.5);

  nh.param<int>("range_min", range_min, 0);
  nh.param<int>("range_max", range_max, 359);

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
        mission_num = 2;
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
    ROS_WARN("mission_num = %d", mission_num);
    switch (mission_num)
    {
      // mission1: 识别颜色
      // case 1:
      //     if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
      //     {
      //         // 到达起飞点，悬停进行颜色识别
      //         if (!should_hover)
      //         {
      //             should_hover = true;
      //             hover_start_time = ros::Time::now();
      //             hover_x = local_pos.pose.pose.position.x;
      //             hover_y = local_pos.pose.pose.position.y;
      //             hover_z = local_pos.pose.pose.position.z;
      //             hover_yaw = yaw;
      //             image_processed = false;     // 重置图像处理标志
      //             image_received = false;      // 重置图像接收标志
      //             color_recognized = false;    // 重置颜色识别标志
      //             ROS_INFO("开始悬停进行起飞地标颜色识别...");
      //         }
      //         // 悬停3秒进行颜色识别
      //         if ((ros::Time::now() - hover_start_time).toSec() >= 3.0)
      //         {
      //             should_hover = false;
      //             if (color_recognized)
      //             {
      //                 ROS_INFO("起飞地标颜色识别完成: %s", landmark_color.c_str());
      //             }
      //             else
      //             {
      //                 ROS_WARN("起飞地标颜色识别失败或超时");
      //             }
      //             mission_num = 2;  // 进入下一个任务
      //             last_request = ros::Time::now();
      //         }
      //     }
      //     break;

      // mission2: collision_avoidance
      case 2:
        //回调一次 更新传感器状态
        //1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
        target_x = 8.0;
        target_y = 2.0;
        ros::spinOnce();
        collision_avoidance(target_x,target_y);
        printf();
        abs_distance = sqrt(pow((pos_drone.pose.position.x - target_x), 2) + pow((pos_drone.pose.position.y - target_y), 2));
        if (abs_distance < err_max)
        {
          mission_num = 3; // 任务结束
          last_request = ros::Time::now();
        }
        break;
          case 3:
        ros::spinOnce();
        
          target_x = 8.0;
          target_y = -1.0;
          abs_distance = sqrt(pow((pos_drone.pose.position.x - target_x), 2) + pow((pos_drone.pose.position.y - target_y), 2));
          collision_avoidance(target_x,target_y);
          if (abs_distance < err_max)
          {
            mission_num = 4; // 任务结束
            last_request = ros::Time::now();
          }
        break;
        

      // mission3: crossing_door
      case 4:
        ros::spinOnce();
        if ((!door_center_x[0])){
          target_x = 10.0;
          target_y = 0.3;
          abs_distance = sqrt(pow((pos_drone.pose.position.x - target_x), 2) + pow((pos_drone.pose.position.y - target_y), 2));
          collision_avoidance(target_x,target_y);
          if (abs_distance < 0.1)
            finddoorcenter(0);
        }
        else {
          cout << "door position: " << door_center_x[0] << " " << door_center_y[0] << endl;
          target_x = door_center_x[0] + 3.0;
          target_y = door_center_y[0];
          collision_avoidance(target_x,target_y);
          abs_distance = sqrt(pow((pos_drone.pose.position.x - (door_center_x[0] + 3.0)), 2) + pow((pos_drone.pose.position.y - door_center_y[0]), 2));
          if (abs_distance < err_max) {
            mission_num = 5; // 任务结束
            last_request = ros::Time::now();
          }
        }
        break;

      // mission3: crossing_ring（穿环）
      case 5:
        target_x = 16.5;
        target_y = 2.4;
        collision_avoidance(target_x,target_y);
        //mission_pos_cruise(16.5, 2.4, 1.36, 0.0, err_max);
        abs_distance = sqrt(pow((pos_drone.pose.position.x - target_x), 2) + pow((pos_drone.pose.position.y - target_y), 2));
        if (abs_distance < err_max) {
          mission_num = 6; // 任务结束
          last_request = ros::Time::now();
        }
        break; 
      case 6:
        target_x = 20.0;
        target_y = 2.4;
        collision_avoidance(target_x,target_y);
        mission_pos_cruise(20.0, 2.4, 1.36, 0.0, err_max);
        abs_distance = sqrt(pow((pos_drone.pose.position.x - target_x), 2) + pow((pos_drone.pose.position.y - target_y), 2));
        if (abs_distance < err_max) {
          mission_num = 7; // 任务结束
          last_request = ros::Time::now();
        }
        break; 
         case 7:
        target_x = 20.0;
        target_y = 0.2;
        //collision_avoidance(target_x,target_y);
        mission_pos_cruise(20.0, 0.2, 1.29, 0.0, err_max);
        abs_distance = sqrt(pow((pos_drone.pose.position.x - target_x), 2) + pow((pos_drone.pose.position.y - target_y), 2));
        if (abs_distance < err_max) {
          mission_num = 8; // 任务结束
          last_request = ros::Time::now();
        }
        break; 
         case 8:
        target_x = 23.0;
        target_y = 0.2;
        //collision_avoidance(target_x,target_y);
                mission_pos_cruise(23.0, 0.2, 1.29, 0.0, err_max);

        abs_distance = sqrt(pow((pos_drone.pose.position.x - target_x), 2) + pow((pos_drone.pose.position.y - target_y), 2));
        if (abs_distance < err_max) {
          mission_num = 9; // 任务结束
          last_request = ros::Time::now();
        }
        break; 
         case 9:
        target_x = 25.0;
        target_y = 0.2;
        //collision_avoidance(target_x,target_y);
                mission_pos_cruise(25.0, 0.2, 1.29, 0.0, err_max);

        abs_distance = sqrt(pow((pos_drone.pose.position.x - target_x), 2) + pow((pos_drone.pose.position.y - target_y), 2));
        if (abs_distance < err_max) {
          ros::Duration du(10.0);
          du.sleep();
          mission_num = 10; // 任务结束
          last_request = ros::Time::now();
        }
        break; 
    //   case 8:
    //     target_x = 28.0;
    //     target_y = -1.0;
    //     collision_avoidance(target_x,target_y);
    //     if (mission_pos_cruise(28.0, -1.0, ALTITUDE, 0.0, err_max)) {
    //         last_request = ros::Time::now();
    //         should_hover = true;
    //         hover_start_time = ros::Time::now();
    //         hover_x = local_pos.pose.pose.position.x;
    //         hover_y = local_pos.pose.pose.position.y;
    //         hover_z = local_pos.pose.pose.position.z;
    //         hover_yaw = yaw;
    //         image_processed = false;
    //         image_received = false;
    //         mission_num = 9;
    //     }
    //     break;
    //   case 9:  // 字母识别任务
    //     if (mission_pos_cruise(28.0, 1.0, ALTITUDE, 0.0, err_max)) {
    //         last_request = ros::Time::now();
    //         should_hover = true;
    //         hover_start_time = ros::Time::now();
    //         hover_x = local_pos.pose.pose.position.x;
    //         hover_y = local_pos.pose.pose.position.y;
    //         hover_z = local_pos.pose.pose.position.z;
    //         hover_yaw = yaw;
    //         image_processed = false;
    //         image_received = false;
    //         mission_num = 10;
    //     }
    //     break;
    //   case 10:  // 二维码识别任务
    //     if (mission_pos_cruise(28.0, 3.0, ALTITUDE, 0.0, err_max)) {
    //         last_request = ros::Time::now();
    //         should_hover = true;
    //         hover_start_time = ros::Time::now();
    //         hover_x = local_pos.pose.pose.position.x;
    //         hover_y = local_pos.pose.pose.position.y;
    //         hover_z = local_pos.pose.pose.position.z;
    //         hover_yaw = yaw;
    //         image_processed = false;
    //         image_received = false;
    //         mission_num = 11;
    //     }
    //     break;
    //   case 11:  // 数字识别任务（保留原始任务）
    //     // 悬停15秒进行数字识别
    //     if (!should_hover)
    //     {
    //         should_hover = true;
    //         hover_start_time = ros::Time::now();
    //         hover_x = local_pos.pose.pose.position.x;
    //         hover_y = local_pos.pose.pose.position.y;
    //         hover_z = local_pos.pose.pose.position.z;
    //         hover_yaw = yaw;
    //         image_processed = false;
    //         image_received = false;
    //     }
    //     if ((ros::Time::now() - hover_start_time).toSec() >= hover_duration) {
    //         mission_num = 12; // 数字识别完成后进入第一个降落地标
    //         should_hover = false;
    //         last_request = ros::Time::now();
    //     }
    //     break;
    //   case 12:  // 第一个降落地标 (35, 3)
    //     if (mission_pos_cruise(35.0, 3.0, ALTITUDE, 0.0, err_max))
    //     {
    //         // 到达第一个降落地标，悬停进行颜色识别
    //         if (!should_hover)
    //         {
    //             should_hover = true;
    //             hover_start_time = ros::Time::now();
    //             hover_x = local_pos.pose.pose.position.x;
    //             hover_y = local_pos.pose.pose.position.y;
    //             hover_z = local_pos.pose.pose.position.z;
    //             hover_yaw = yaw;
    //             image_processed = false;
    //             image_received = false;
    //             ROS_INFO("到达第一个降落地标，开始悬停进行颜色识别...");
            
    //         // 悬停3秒进行颜色识别
    //         if ((ros::Time::now() - hover_start_time).toSec() >= 3.0)
    //         {
    //             should_hover = false;
    //             if (!landing_colors[0].empty())
    //             {
    //                 ROS_INFO("第一个降落地标颜色识别完成: %s", landing_colors[0].c_str());
    //             }
    //             else
    //             {
    //                 ROS_WARN("第一个降落地标颜色识别失败或超时");
    //             }
    //             mission_num = 13;
    //             last_request = ros::Time::now();
    //         }
    //     }
    //     break;
    //   case 13:  // 第二个降落地标 (35, 1)
    //     if (mission_pos_cruise(35.0, 1.0, ALTITUDE, 0.0, err_max))
    //     {
    //         // 到达第二个降落地标，悬停进行颜色识别
    //         if (!should_hover)
    //         {
    //             should_hover = true;
    //             hover_start_time = ros::Time::now();
    //             hover_x = local_pos.pose.pose.position.x;
    //             hover_y = local_pos.pose.pose.position.y;
    //             hover_z = local_pos.pose.pose.position.z;
    //             hover_yaw = yaw;
    //             image_processed = false;
    //             image_received = false;
    //             ROS_INFO("到达第二个降落地标，开始悬停进行颜色识别...");
            
    //         // 悬停3秒进行颜色识别
    //         if ((ros::Time::now() - hover_start_time).toSec() >= 3.0)
    //         {
    //             should_hover = false;
    //             if (!landing_colors[1].empty())
    //             {
    //                 ROS_INFO("第二个降落地标颜色识别完成: %s", landing_colors[1].c_str());
    //             }
    //             else
    //             {
    //                 ROS_WARN("第二个降落地标颜色识别失败或超时");
    //             }
    //             mission_num = 14;
    //             last_request = ros::Time::now();
    //         }
    //     }
    //     break;
    //   case 14:  // 第三个降落地标 (35, -1)
    //     if (mission_pos_cruise(35.0, -1.0, ALTITUDE, 0.0, err_max))
    //     {
    //         // 到达第三个降落地标，悬停进行颜色识别
    //         if (!should_hover)
    //         {
    //             should_hover = true;
    //             hover_start_time = ros::Time::now();
    //             hover_x = local_pos.pose.pose.position.x;
    //             hover_y = local_pos.pose.pose.position.y;
    //             hover_z = local_pos.pose.pose.position.z;
    //             hover_yaw = yaw;
    //             image_processed = false;
    //             image_received = false;
    //             ROS_INFO("到达第三个降落地标，开始悬停进行颜色识别...");
            
    //         // 悬停3秒进行颜色识别
    //         if ((ros::Time::now() - hover_start_time).toSec() >= 3.0)
    //         {
    //             should_hover = false;
    //             if (!landing_colors[2].empty())
    //             {
    //                 ROS_INFO("第三个降落地标颜色识别完成: %s", landing_colors[2].c_str());
    //             }
    //             else
    //             {
    //                 ROS_WARN("第三个降落地标颜色识别失败或超时");
    //             }
    //             mission_num = 15;  // 进入颜色比较阶段
    //             last_request = ros::Time::now();
    //         }
    //     }
    //     break;xpected ‘}’ at end of input
  

    //   case 15:  // 颜色比较和决策
    //     // 比较三个降落地标颜色与起飞地标颜色
    //     color_compare_result = compareLandmarkColors()
    //     if (color_compare_result >= 0 && color_compare_result < 3) {
    //         // 找到匹配的降落地标，飞往该点
    //         float target_x, target_y;
    //         if (color_compare_result == 0) {
    //             target_x = 35.0; target_y = 3.0;
    //         }
    //         else if (color_compare_result == 1) {
    //             target_x = 35.0; target_y = 1.0;
    //         }
    //         else {
    //             target_x = 35.0; target_y = -1.0;
            
    //         ROS_INFO("飞往匹配的降落地标: (%.1f, %.1f)", target_x, target_y)
    //         if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0.0, err_max)) {
    //             mission_num = 16;  // 到达匹配点，准备降落
    //             last_request = ros::Time::now();
    //         }
    //     }
    //     else {
    //         // 没有找到匹配，选择最后一个点作为默认降落点
    //         ROS_WARN("未找到颜色匹配的降落地标，使用默认降落点(35, -1)");
    //         if (mission_pos_cruise(35.0, -1.0, ALTITUDE, 0.0, err_max)) {
    //             mission_num = 16;  // 到达默认点，准备降落
    //             last_request = ros::Time::now();
    //         }
    //     }
    //     break;
    case 10:  // 降落
        // 执行原来的降落逻辑
        mavros_msgs::SetMode land_mode;  // 这里添加分号修复编译错误
       land_mode.request.custom_mode = "AUTO.LAND";
      if (set_mode_client.call(land_mode) && land_mode.response.mode_sent) {
           ROS_INFO("切换到自动降落模式");
      }
      if (!current_state.armed) {
          ROS_INFO("无人机已降落并自动解锁");
          mission_num = -1;
    }
      break;
    }
   // switch语句结
    //   if (should_hover) {
    //     if ((ros::Time::now() - hover_start_time).toSec() < hover_duration) {
    //         setpoint_raw.type_mask = 0b111111111000;
    //         setpoint_raw.coordinate_frame = 1;
    //         setpoint_raw.position.x = hover_x;
    //         setpoint_raw.position.y = hover_y;
    //         setpoint_raw.position.z = hover_z;
    //         setpoint_raw.yaw = hover_yaw;
    //     }
    //     else {
    //         should_hover = false;
    //     }
    // }


      //降落
      // case 6:
      //   if(precision_land())
      //   {
      //     mission_num = -1; // 任务结束
      //     last_request = ros::Time::now();
      //   }
      //   break;
    
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


