#include <template.h>

// 全局变量定义
int mission_num = 0;//任务阶段编号
float if_debug = 0;//调试模式标志
float err_max = 0.2;//位置误差阈值，判断是否到达目标点
void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  if(if_debug == 1) cout << "自动offboard" << std::endl;
  else cout << "遥控器offboard" << std::endl;
}//打印无人机控制参数,用于调试和参数确认


int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  // 初始化ROS节点
  ros::init(argc, argv, "template");
  ros::NodeHandle nh;

  // 订阅mavros相关话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);//无人机状态
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);//无人机本地位置

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);//无人机期望位置

  // 创建服务客户端
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");//无人机解锁 / 上锁服务
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");//切换飞行模式
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");//发送自定义指令

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取

  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("if_debug", if_debug, 0);
  print_param();
  //从 ROS 参数服务器读取err_max和if_debug（若无参数则用默认值），并打印参数。
  
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
 
  setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;//指定哪些控制维度需要被忽略,每个数值对应一个控制维度的掩码位，通过累加数值可以组合忽略多个维度。
  setpoint_raw.coordinate_frame = 1;//指定无人机控制指令的参考坐标系,数值为1即本地东北天坐标系，MAVROS 默认转换为 ENU
  setpoint_raw.position.x = 0;
  setpoint_raw.position.y = 0;
  setpoint_raw.position.z = ALTITUDE;
  setpoint_raw.yaw = 0;//偏航角:无人机机头朝向与坐标系 X 轴（东方向）的夹角

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }//预发送 100 个期望位置指令（MAVROS 要求进入 OFFBOARD 模式前需持续发送指令
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
          ROS_INFO("Offboard enabled");//切换 OFFBOARD 模式
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
    // 当无人机到达起飞点高度后，悬停10秒后进入任务模式，提高视觉效果
    if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2)
    {
      if (ros::Time::now() - last_request > ros::Duration(10.0))
      {
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
  

  while (ros::ok())
  {
    ROS_WARN("mission_num = %d", mission_num);
    
    switch (mission_num)
    {
      // mission1: 起飞
      case 1:
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
        {
          mission_num = 2;
          last_request = ros::Time::now();
        }
	    else if(ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num = 2;
          last_request = ros::Time::now();
        }
        break;

      //世界系前进
      case 2:
        if (mission_pos_cruise(0, -1, ALTITUDE, 0.0 , err_max))
        {
          mission_num = 3;
          last_request = ros::Time::now();
        }
        break;
      case 3:
        if (mission_pos_cruise(4.0, -1, ALTITUDE, 0.0 , err_max))
        {
          mission_num = 4;
          last_request = ros::Time::now();
        }
        break;
      case 4:
        if (mission_pos_cruise(4.0, 1.5, ALTITUDE, 0.0 , err_max))
        {
          mission_num = 5;
          last_request = ros::Time::now();
        }
        break;
      case 5:
        if (mission_pos_cruise(4.0, 1.6, ALTITUDE, 0.0 , err_max))
        {
          mission_num = 6;
          last_request = ros::Time::now();
        }
        break;
       case 6:
        if (mission_pos_cruise(12.0, 1.6, ALTITUDE, 0.0 , err_max))
        {
          mission_num = 7;
          last_request = ros::Time::now();
        }
        break;
       case 7:
        if (mission_pos_cruise(12.0, 3.0, ALTITUDE, 0.0 , err_max))
        {
          mission_num = 8;
          last_request = ros::Time::now();
        }
        break;
        case 8:
        if (mission_pos_cruise(17.0, 3.0, ALTITUDE, 0.0 , err_max))
        {
          mission_num = 9;
          last_request = ros::Time::now();
        }
        break;
        case 9:
        if (mission_pos_cruise(17.0, 2.4, ALTITUDE, 0.0 , err_max))
        {
          mission_num = 10;
          last_request = ros::Time::now();
        }
        break;
        case 10:
        if (mission_pos_cruise(20.0, 2.4, ALTITUDE, 0.0 , err_max))
        {
          mission_num = 11;
          last_request = ros::Time::now();
        }
        break;
        case 11:
        if (mission_pos_cruise(20.0, 0.35, 1.3, 0.0 , err_max))
        {
          mission_num = 12;
          last_request = ros::Time::now();
        }
        break;
        case 12:
        if (mission_pos_cruise(31.0, 0.35, 1.3, 0.0 , err_max))
        {
          mission_num = 13;
          last_request = ros::Time::now();
        }
        break;
        case 13:
        if (mission_pos_cruise(31.0, 1.0, ALTITUDE, 0.0 , err_max))
        {
          mission_num = 14;
          last_request = ros::Time::now();
        }
        break;
        case 14:
        if (mission_pos_cruise(35.0, 1.0, ALTITUDE, 0.0 , err_max))
        {
          mission_num = 15;
          last_request = ros::Time::now();
        }
        break;

      //降落
      case 15:
        if(mission_pos_cruise(35.0, 1.0, 0.0, 0.0 , err_max))
        {
          mission_num = -1; // 任务结束
          last_request = ros::Time::now();
        }
        break;
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);//更新setpoint_raw为目标位置 (x, y, z) 和偏航角 yaw
    ros::spinOnce();
    rate.sleep();
    
    if(mission_num == -1) 
    {
      exit(0);
    }
  }
  return 0;
}
//相同的代码有时候撞上有时候不撞上个人猜测有可能是惯性的原因
