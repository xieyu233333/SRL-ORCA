//-----------------------------------------------------------------------------
//	Version 1.0
//-----------------------------------------------------------------------------
//	File Name: six_robotmove.cpp
//-----------------------------------------------------------------------------
//	CopyRight (C) 2023, TigherLab.USTC
//-----------------------------------------------------------------------------
//	Modifier	Date		Detail
//
//	qinjianmin		20230509	create
//-----------------------------------------------------------------------------
//	Note:   Interaction 4机器人ORCA运动--基础程序--cpp文件--用于HSRLN算法搭建动态环境
//-----------------------------------------------------------------------------


#include <angles/angles.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <collvoid_srvs/GetObstacles.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include "collvoid_local_planner/common.h"
#include "collvoid_local_planner/hsrln_four_robotmove.h"
#include "collvoid_local_planner/Vector2.h"
#include <collvoid_local_planner/Agent.h>
#include <collvoid_local_planner/GetCollvoidTwist.h>
#include<gazebo_msgs/ModelState.h>
#include <tf/tf.h>

#include <std_msgs/Int32MultiArray.h>//用于IDSI_Rank数据发布--230702
#include <std_msgs/Float32MultiArray.h>//用于IDSI_Rank数据发布--230710
//#include <functional>  // 包含 std::bind 的头文件--chatgpt--230703

//using namespace std_msgs; // 包含--chatgpt给的报错解决--230703
using namespace collvoid;

// //超参数
// //机器人激光雷达探测长度数据：36个值，从机器人正前方x轴方向，开始逆时针数？？
// float _lidarlegth_data[40] = { 0.2,0.2,0.2,0.2,0.2,  0.2,0.2,0.2,0.2,0.2,  0.2,0.2,0.2,0.2,0.2,  0.2,0.2,0.2,0.2,0.2,
// 0.2,0.2,0.2,0.2,0.2,  0.2,0.186,0.168,0.158,0.152,  0.15,0.152,0.158,0.168,0.186,  0.2,0.2,0.2,0.2,0.2 };

// //机器人激光雷达--放大倍数
// float _lidarlagre = 2.1;

// { 0.3,0.3,0.3,0.3,0.3,  0.3,0.3,0.3,0.3,0.3,  0.3,0.3,0.3,0.3,0.3,  0.3,0.3,0.3,0.3,0.3,
// 0.3,0.3,0.3,0.291,0.243,  0.212,0.193,0.180,0.173,0.171,  0.171,0.173,0.180,0.193,0.212,  0.243,0.291,0.3,0.3,0.3 };

//  0.3,0.3,0.3,0.3,0.3,  0.3,0.3,0.3,0.3,0.3,  0.3,0.3,0.3,0.3,0.3,  0.3,0.3,0.3,0.3,0.3,
// 0.3,0.3,0.3,0.291,0.243,  0.212,0.193,0.180,0.173,0.171,  0.171,0.173,0.180,0.193,0.212,  0.243,0.291,0.3,0.3,0.3

//所有参数由6改为4----------------------------------------------2024.01.30

// //超参数
// //机器人初始坐标 行--x,y 列--机器人编号
// float _beginloc_list[2][6] = {{ 0, 1, 2, 3, 4, 5 },
//                               { 0, 1, 2, 3, 4, 5 }};
//机器人初始坐标 行--x,y 列--机器人编号
float _beginloc_list[4][2] = {{ 2.5, 0 },
                                { 1.5, 3 },
                                { -1, 2.5 },
                                { -3, 0.5 },
                                // { -2, -2 },
                                // { 2.5, -1 }
                                };

                                // {{ 0, 0 },
                                // { 1, 1 },
                                // { 2, 2 },
                                // { 3, 3 },
                                // { 4, 4 },
                                // { 5, 5 }};

                                // {{ 5, 0 },
                                // { 3, 6 },
                                // { -2, 5 },
                                // { -6, 1 },
                                // { -4, -4 },
                                // { 5, -2 }};

// //超参数
// //机器人目标 行--x,y 列--机器人编号
// float _target_list[2][6] = {{ 5, 6, 7, 8, 9, 10 },
//                               { 0, 1, 2, 3, 4, 5 }};
//                               //超参数
//机器人目标 行--x,y 列--机器人编号
float _target_list[4][2] = {{ -3, -0.5 },
                            { -2.5, -1 },
                            { 2.5, -0.5 },
                            { 2.5, -1.5 },
                            // { 3, 0.5 },
                            // { 0, -2 }
                            };

                            // {{ 5, 0 },
                            // { 6, 1 },
                            // { 7, 2 },
                            // { 8, 3 },
                            // { 9, 4 },
                            // { 10, 5 }};

                            // {{ -6, -1 },
                            // { -5, -2 },
                            // { 5, -1 },
                            // { 5, -3 },
                            // { 6, 1 },
                            // { 0, -4 }};


//速度列表 横--速度列表
//x向--5
float _speed_list[4] = {0.06, 0.10, 0.14 ,0.18 };//,0.22, 0.25

//超参数
//机器人初始方向角
float _heading_list[4] = { 0, 0, 0, 3.14 };//, 3.14, 3.14

//超参数
//机器人根据到目标夹角的旋转的超参数
//正负代表方向正方向，顺时针还是逆时针是
float _w_p = 1.0 * 0.05;

//超参数
//机器人到目标的范围，小于它即为到达目标
float _distance_target = 0.2;

//超参数
//延时指数--用于位置回调函数odom_callback延时
int _Delay_index[4] = { 50,50,50,50 };//,50, 50



int main(int argc, char **argv) {
    ros::init(argc, argv, "interact");
    //各个函数值初始化
    init();

    //出发位置计算
    // local_compute();

    ros::Rate rate(50);

    //ROS_INFO("Debug----------10,");

    while (ros::ok()) {
            //ROS_INFO("Debug----------11,");
        local_compute();
        //Print_IDSI();-----------------------------------------------为hsrln算法注释掉，24.01.31
            //ROS_INFO("Debug----------12,");
        ros::spinOnce();
            //ROS_INFO("Debug----------13,");
        rate.sleep();
    }
}

//所有参数由6改为4----------------------------------------------2024.01.30
//各个函数值初始化
void init()
{
    ros::NodeHandle nh("~");

    for (int i = 0; i < 4; i++) {
        agent.push_back(AgentPtr(new Agent));//---------------------------------------------疑问待定，是否要修改？？hsrln算法，
    }

    //ROS_INFO("Debug----------0,");

    //发布初始化
    //仿照写一个发布者？--发布机器人的线速度和角速度
    //消息类型为geometry_msgs::Point
    //还有待细细调整-------------------------------------------------------------------------
    //cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/cmd_vel", 4 * 4);
    for (int i = 0; i < 4; i++)
    {
        //sac_cmd_vel_sub_[i] = nh.subscribe<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/sac_cmd_vel", 4 * 4, boost::bind(&OrcaLocalPlanner::sac_cmd_vel_callback, this, _1, i));
        //此处改为/cmd_vel-直接发送数据，或/sac_cmd_vel--经过orca处理发送数据
        sac_cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/tb3_" + std::to_string(i+2) + "/sac_cmd_vel", 4 * 4);
    }

    // sac_cmd_vel_pub_2 = nh.advertise<geometry_msgs::Twist>("/tb3_2/sac_cmd_vel", 4 * 4);
    // sac_cmd_vel_pub_3 = nh.advertise<geometry_msgs::Twist>("/tb3_3/sac_cmd_vel", 4 * 4);
    // sac_cmd_vel_pub_4 = nh.advertise<geometry_msgs::Twist>("/tb3_4/sac_cmd_vel", 4 * 4);
    // sac_cmd_vel_pub_5 = nh.advertise<geometry_msgs::Twist>("/tb3_5/sac_cmd_vel", 4 * 4);



    //ROS_INFO("Debug----------1,");

    //发布初始化
    //仿照写一个发布者？--发布重置目标位置
    //消息类型为gazebo_msgs::ModelState
    //还有待细细调整-------------------------------------------------------------------------
    //cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/cmd_vel", 4 * 4);
    for (int i = 0; i < 4; i++)
    {
        //sac_cmd_vel_sub_[i] = nh.subscribe<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/sac_cmd_vel", 4 * 4, boost::bind(&OrcaLocalPlanner::sac_cmd_vel_callback, this, _1, i));
        set_model_pub_[i] = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 4 * 4);
    }

    // set_model_pub_2 = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 4 * 4);
    // set_model_pub_3 = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 4 * 4);
    // set_model_pub_4 = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 4 * 4);
    // set_model_pub_5 = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 4 * 4);



    //ROS_INFO("Debug----------2,");

    // //订阅激光雷达数据？？
    // //scan_sub_[i] = nh.subscribe<sensor_msgs::LaserScan>("/tb3_" + std::to_string(i ) + "/scan", 1, boost::bind(&OrcaLocalPlanner::scan_callback, this, _1, i));
    // for (int i = 0; i < 4; i++)
    // {
    //     scan_sub_[i] = nh.subscribe<sensor_msgs::LaserScan>("/tb3_" + std::to_string(i+2) + "/scan", 1, boost::bind(&scan_callback, _1, i+2));
    // }

    // scan_sub_2 = nh.subscribe<sensor_msgs::LaserScan>("/tb3_2/scan", 1, boost::bind(&scan_callback, _1, 2));
    // scan_sub_3 = nh.subscribe<sensor_msgs::LaserScan>("/tb3_3/scan", 1, boost::bind(&scan_callback, _1, 3));
    // scan_sub_4 = nh.subscribe<sensor_msgs::LaserScan>("/tb3_4/scan", 1, boost::bind(&scan_callback, _1, 4));
    // scan_sub_5 = nh.subscribe<sensor_msgs::LaserScan>("/tb3_5/scan", 1, boost::bind(&scan_callback, _1, 5));



    //ROS_INFO("Debug----------3,");

    //订阅者--订阅机器人位置信息？？
    for (int i = 0; i < 4; i++)
    {
        odom_sub_[i] = nh.subscribe<nav_msgs::Odometry>("/tb3_" + std::to_string(i+2) + "/odom", 4 * 4, boost::bind(&odom_callback, _1, i));
    }

    // odom_sub_2 = nh.subscribe<nav_msgs::Odometry>("/tb3_2/odom", 4 * 4, boost::bind(&odom_callback, _1, 2));
    // odom_sub_3 = nh.subscribe<nav_msgs::Odometry>("/tb3_3/odom", 4 * 4, boost::bind(&odom_callback, _1, 3));
    // odom_sub_4 = nh.subscribe<nav_msgs::Odometry>("/tb3_4/odom", 4 * 4, boost::bind(&odom_callback, _1, 4));
    // odom_sub_5 = nh.subscribe<nav_msgs::Odometry>("/tb3_5/odom", 4 * 4, boost::bind(&odom_callback, _1, 5));


    // //订阅者--订阅机器人位置信息？？------------------------------------------------------------还在调整中，此处有问题
    // for (int i = 0; i < 4; i++)
    // {
    //     IDSI_Rank[i] = nh.subscribe<std_msgs::Float32MultiArray>("/tb3_" + std::to_string(i+2) +"/idsi_topic", 10, boost::bind(&IDSI_callback, _1, i+2));
    // }

    //ROS_INFO("Debug----------4,");

    //初始化--机器人相关参数--赋初值，float
    for (int i = 0; i < 4; i++)
    {
        //机器人初始坐标 行--x,y 列--机器人编号
        begin_inf[i][0] = _beginloc_list[i][0];
        begin_inf[i][1] = _beginloc_list[i][1];

        // //速度列表 横--速度列表--多余？？230620
        // begin_inf[i][2] = _speed_list[i];

        //速度列表 横--速度列表       
        begin_inf[i][2] = _speed_list[i];
      
        //机器人初始方向角
        begin_inf[i][3] = _heading_list[i];

        //机器人目标 行--x,y 列--机器人编号
        target_inf[i][0] = _target_list[i][0];
        target_inf[i][1] = _target_list[i][1];
 
        //机器人到达数组--初始均为false
        arrival_bool[i] = false;
    }

    //ROS_INFO("Debug----------5,");

    //程序开始判别--bool-为ture
    scriptbegin_bool = true;

    // //初始化--赋初值，float
    // //交互数据收集 横--横向编号， 竖--纵向编号,内容为交互数据值
    // //x向--20  y向--10
    // //public float begin_inf[20][20];
	// for (int i = 0; i < 20; i++)
	// {
	// 	    for (int j = 0; j < 10; j++)
    //         {
    //             //全部赋初始值为--0
    //             interact_data[i][j] = 0;
    //         }
    // }

    //ROS_INFO("Debug----------6,");

    // //初始化--赋初值，float
    // //距离数据收集 横--横向编号， 竖--纵向编号,内容为出发点到终点距离值
    // //x向--20  y向--10
    // //public float begin_inf[20][20];
	// for (int i = 0; i < 20; i++)
	// {
	// 	    for (int j = 0; j < 10; j++)
    //         {
    //             //全部赋初始值为--0
    //             dis_data[i][j] = 0;
    //         }
    // }

    //ROS_INFO("Debug----------7,");

    // //初始化--赋初值，float
    // //真实数据收集 横--横向编号， 竖--纵向编号,内容为交互数据值
    // //x向--20  y向--10
    // //public float begin_inf[20][20];
	// for (int i = 0; i < 20; i++)
	// {
	// 	    for (int j = 0; j < 10; j++)
    //         {
    //             //全部赋初始值为--0
    //             real_data[i][j] = 0;
    //         }
    // }

    // //初始化--赋初值，int
    // //idsi数据 横--机器人编号， 竖--纵向编号,内容为动态障碍物排序
    // //x向--6--机器人编号  y向--6

	// for (int i = 0; i < 4; i++)
	// {
	// 	for (int j = 0; j < 4; j++)
    //     {
    //         idsi_data[i][j] = -1;
    //     }
    // }


    //ROS_INFO("Debug----------8,");

//暂时注释掉，可能有用--230621
    // //初始化--赋初值，float
    // //机器人激光雷达探测长度数据：36个值，从正前方顺时针开始数？？
    // //public float lidar_legth[36];
    // //改为40个值，以免雷达数组溢出
    // for (int i = 0; i < 2; i++)
    // {
    //     for (int j = 0; j < 40; j++)
    //     {
    //         lidar_legth[i][j] = 0;                
    //     }
    // }

    //ROS_INFO("Debug----------9,");

    //ROS_INFO("C++ Interact begins, index_x is :%i, index_y is :%i", _index_x,_index_y);
    ROS_INFO("Begin Locate, robot-0 is, ,robot-1 is,");//%f,%f.", begin_inf[0][0],begin_inf[0][1],begin_inf[1][0],begin_inf[1][1]
}

//出发位置计算
void local_compute()
{
    // //定义2个机器人的线速度角速度--发布者的twist
    // geometry_msgs::Twist sac_cmd_vel_0;
    // geometry_msgs::Twist sac_cmd_vel_1;

            //ROS_INFO("Debug----------11----0,");

    //定义2个机器人的重置位置--发布者的ModelState
    gazebo_msgs::ModelState restart_location_0;
    gazebo_msgs::ModelState restart_location_1;
    gazebo_msgs::ModelState restart_location_2;
    gazebo_msgs::ModelState restart_location_3;
    // gazebo_msgs::ModelState restart_location_4;
    // gazebo_msgs::ModelState restart_location_5;

            //ROS_INFO("Debug----------11----1,");

    //程序开始判别--bool
    if(scriptbegin_bool == true)
    {
        //如果两机器人--均到达终点,
        if( (arrival_bool[0] == true) && (arrival_bool[1] == true)&& (arrival_bool[2] == true)&& (arrival_bool[3] == true)) //&& (arrival_bool[0] == true)&& (arrival_bool[1] == true) 
        {
            //计算重新开始位置--机器人-0
            restart_location_0.model_name = "tb3_" + std::to_string(0+2);
 
            restart_location_0.pose.position.x = begin_inf[0][0];//begin_inf[0][0] 2.0
            restart_location_0.pose.position.y = begin_inf[0][1];//begin_inf[0][1]
            restart_location_0.pose.position.z = 0;

            //方向角转换为四元数
            auto q_0 = tf::createQuaternionFromYaw(begin_inf[0][3]);

            restart_location_0.pose.orientation.x = q_0.x();
            restart_location_0.pose.orientation.y = q_0.y();
            restart_location_0.pose.orientation.z = q_0.z();
            restart_location_0.pose.orientation.w = q_0.w();            

            //ROS_INFO("Debug----------11----2,");

            //计算重新开始位置--机器人-1
            restart_location_1.model_name = "tb3_" + std::to_string(1+2);
 
            restart_location_1.pose.position.x = begin_inf[1][0];//begin_inf[1][0] -1.0
            restart_location_1.pose.position.y = begin_inf[1][1];//begin_inf[1][1] -1.0
            restart_location_1.pose.position.z = 0;

            //方向角转换为四元数
            auto q_1 = tf::createQuaternionFromYaw(begin_inf[1][3]);

            restart_location_1.pose.orientation.x = q_1.x();
            restart_location_1.pose.orientation.y = q_1.y();
            restart_location_1.pose.orientation.z = q_1.z();
            restart_location_1.pose.orientation.w = q_1.w();

            //ROS_INFO("Debug----------11----3,");

            //计算重新开始位置--机器人-2
            restart_location_2.model_name = "tb3_" + std::to_string(2+2);
 
            restart_location_2.pose.position.x = begin_inf[2][0];//begin_inf[1][0] -1.0
            restart_location_2.pose.position.y = begin_inf[2][1];//begin_inf[1][1] -1.0
            restart_location_2.pose.position.z = 0;

            //方向角转换为四元数
            auto q_2 = tf::createQuaternionFromYaw(begin_inf[2][3]);

            restart_location_2.pose.orientation.x = q_2.x();
            restart_location_2.pose.orientation.y = q_2.y();
            restart_location_2.pose.orientation.z = q_2.z();
            restart_location_2.pose.orientation.w = q_2.w();

            //ROS_INFO("Debug----------11----4,");

            //计算重新开始位置--机器人-3
            restart_location_3.model_name = "tb3_" + std::to_string(3+2);
 
            restart_location_3.pose.position.x = begin_inf[3][0];//begin_inf[1][0] -1.0
            restart_location_3.pose.position.y = begin_inf[3][1];//begin_inf[1][1] -1.0
            restart_location_3.pose.position.z = 0;

            //方向角转换为四元数
            auto q_3 = tf::createQuaternionFromYaw(begin_inf[3][3]);

            restart_location_3.pose.orientation.x = q_3.x();
            restart_location_3.pose.orientation.y = q_3.y();
            restart_location_3.pose.orientation.z = q_3.z();
            restart_location_3.pose.orientation.w = q_3.w();

            ROS_INFO("Debug----------11----5,");

            // //计算重新开始位置--机器人-4
            // restart_location_4.model_name = "tb3_" + std::to_string(4);
 
            // restart_location_4.pose.position.x = begin_inf[4][0];//begin_inf[1][0] -1.0
            // restart_location_4.pose.position.y = begin_inf[4][1];//begin_inf[1][1] -1.0
            // restart_location_4.pose.position.z = 0;

            // //方向角转换为四元数
            // auto q_4 = tf::createQuaternionFromYaw(begin_inf[4][3]);

            // restart_location_4.pose.orientation.x = q_4.x();
            // restart_location_4.pose.orientation.y = q_4.y();
            // restart_location_4.pose.orientation.z = q_4.z();
            // restart_location_4.pose.orientation.w = q_4.w();

            // //ROS_INFO("Debug----------11----6,");

            // //计算重新开始位置--机器人-5
            // restart_location_5.model_name = "tb3_" + std::to_string(5);
 
            // restart_location_5.pose.position.x = begin_inf[5][0];//begin_inf[1][0] -1.0
            // restart_location_5.pose.position.y = begin_inf[5][1];//begin_inf[1][1] -1.0
            // restart_location_5.pose.position.z = 0;

            // //方向角转换为四元数
            // auto q_5 = tf::createQuaternionFromYaw(begin_inf[5][3]);

            // restart_location_5.pose.orientation.x = q_5.x();
            // restart_location_5.pose.orientation.y = q_5.y();
            // restart_location_5.pose.orientation.z = q_5.z();
            // restart_location_5.pose.orientation.w = q_5.w();

            // ROS_INFO("Debug----------11----7,");

            //仿照写一个发布者？
            //发布6个机器人的重置位置--发布者的ModelState
            //还有待细细调整-------------------------------------------------------------------------
            //cmd_vel_pub_[n].publish(cmd_vel);
            // set_model_pub_[0].publish(restart_location_0);
            // set_model_pub_[1].publish(restart_location_1);
            set_model_pub_[0].publish(restart_location_0);
            set_model_pub_[1].publish(restart_location_1);
            set_model_pub_[2].publish(restart_location_2);
            set_model_pub_[3].publish(restart_location_3);

            // set_model_pub_0.publish(restart_location_0);
            // set_model_pub_1.publish(restart_location_1);
            // set_model_pub_2.publish(restart_location_2);
            // set_model_pub_3.publish(restart_location_3);
            // set_model_pub_4.publish(restart_location_4);
            // set_model_pub_5.publish(restart_location_5);

            //ROS_INFO("Debug----------11----8,");

            //延时指数--用于位置回调函数odom_callback延时
            for (int i = 0; i < 4; i++)
            {
                _Delay_index[i] = 50;                
            }

            //ROS_INFO("Debug----------11----9,");

            //如果两机器人均到达终点--重新开始，bool为false
            for (int i = 0; i < 4; i++)
            {
                arrival_bool[i] = false;
            }

        }
        else//如果两机器人--没有都到达终点
        {
                
            //ROS_INFO("Debug----------11----10,");         

            //为运动机器人--计算运行真实速度
            for (int i = 0; i < 4; i++)
            {
                //定义机器人-0 的位置
                Vec2d _Robot_pos;

                _Robot_pos.x_ = agent[i]->position_.x();
                _Robot_pos.y_ = agent[i]->position_.y();

                //定义机器人-0 的目标
                Vec2d _Robot_goal;

                _Robot_goal.x_ = target_inf[i][0];
                _Robot_goal.y_ = target_inf[i][1];

                //定义机器人-0 的速度角速度向量
                Vec2d _Robot_vel;
            //ROS_INFO("Debug----------11----11,");    
                //计算速度
                //compute_vel(Vec2d pos, double angle, Vec2d goal, double angular_fb) 
                _Robot_vel = compute_vel(_Robot_pos, agent[i]->heading_, _Robot_goal, 0);

                //ROS_INFO("Robot = %i Speed, x = %f, y = %f,", i, _Robot_vel.x_, _Robot_vel.y_);
                //定义机器人的线速度角速度--发布者的twist
                geometry_msgs::Twist sac_cmd_vel;

            //ROS_INFO("Debug----------11----12,");    

                //机器人-0
                //线速度
                sac_cmd_vel.linear.x = _Robot_vel.x_;

                //默认为0--因为是2d平面
                sac_cmd_vel.linear.y = 0.0;//默认为0

                //角速度，机器人-0角速度默认为0
                //sac_cmd_vel_0.angular.z = 0.0;//默认为0
                sac_cmd_vel.angular.z = _Robot_vel.y_;

            //ROS_INFO("Debug----------11----13,");    
                //发布机器人的线速度和角速度
                //还有待细细调整-------------------------------------------------------------------------
                //cmd_vel_pub_[n].publish(cmd_vel);
                sac_cmd_vel_pub_[i].publish(sac_cmd_vel);//-----------------------------------------------------------------------------------------------------------
                // if(i==2)//------------------------hsrln算法，240131
                // {
                //     sac_cmd_vel_pub_2.publish(sac_cmd_vel);
                // }
                // else if(i==3)
                // {
                //     sac_cmd_vel_pub_3.publish(sac_cmd_vel);
                // }
                // else if(i==4)
                // {
                //     sac_cmd_vel_pub_4.publish(sac_cmd_vel);
                // }
                // else if(i==5)
                // {
                //     sac_cmd_vel_pub_5.publish(sac_cmd_vel);
                // }



            //ROS_INFO("Debug----------11----14,");    
                // ROS_INFO("Runing!!!,Robot = %i, speed= %f, ang_speed = %f, target_inf = %f,%f,positon = %f,%f",

                }


            // //交互数据计算--更新对应的横纵坐标的交互数据
            // interactdata_update( _index_x, _index_y );
            //ROS_INFO("Debug----------11----15,");    
            //检测目标距离
            for (int i = 0; i < 4; i++)
            {
                //机器人当前到目标距离
                float _dis_to_target = sqrt( pow( agent[i]->position_.x() - target_inf[i][0], 2.0) + pow(agent[i]->position_.y() - target_inf[i][1], 2.0) );

             //ROS_INFO("Debug----------11----16,");    

                //判断到目标距离是否满足条件，同时满足回调函数延时条件
                if(( _dis_to_target < _distance_target )&&( _Delay_index[i] < 0 ))
                {

                    //机器人-i，的速度设置为0/////////////////////////////////////////////////////////////////////////////////////////////
                    begin_inf[i][2] = 0;

                    // ROS_INFO("Arrived target!!!, Relay_index is %i, Robot = %i,real-time position is X = %f, Y = %f, target location is X = %f, Y = %f",
                    //  _Delay_index[i], i,agent[i]->position_.x(),agent[i]->position_.y(),target_inf[i][0],target_inf[i][1]);//velsend_bool[i]

                    //则该机器人到达终点--重新开始，bool为ture
                    arrival_bool[i] = true;
                }

                //ROS_INFO("Debug----------11----17,");    
            }
        }
    }
    else
    {
        //程序结束，debug
        //ROS_INFO("Over!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }




}



//暂时注释掉，可能有用--230621
// //打印--计算两点距离
// float distance_data( float _data_1_x, float _data_1_y, float _data_2_x, float _data_2_y )
// {
//     //先平方，再开方，计算距离
//     float _dis = sqrt( pow( _data_2_x - _data_1_x, 2.0) + pow(_data_2_y - _data_1_y, 2.0) );

//     return _dis; 
// }

// //订阅者--激光雷达--回调函数
// void scan_callback(const sensor_msgs::LaserScanConstPtr scan, int n) {
//     double angle = 0;
//     //    double angle = agent[n]->heading_;
//     //obs_lines_[n].clear();
//     // ROS_INFO("scan_callback");
//     // ROS_INFO("scan:%d", scan.get()->header.stamp.nsec);

//     // auto msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/tb3_" + std::to_string(n) + "/odom", ros::Duration(5));
//     // odom_callback(msg, n);

//     visualization_msgs::Marker obs_marker;

//     obs_marker.header.stamp       = ros::Time::now();
//     obs_marker.header.frame_id    = "map";

//     obs_marker.ns = "obs";
//     obs_marker.id = 0;
//     obs_marker.type = visualization_msgs::Marker::SPHERE_LIST;
//     obs_marker.action = visualization_msgs::Marker::ADD;
//     obs_marker.scale.x = 0.01;
//     obs_marker.scale.y = 0.01;
//     obs_marker.scale.z = 0.01;
//     obs_marker.pose.orientation.x = 0.0;
//     obs_marker.pose.orientation.y = 0.0;
//     obs_marker.pose.orientation.z = 0.0;
//     obs_marker.pose.orientation.w = 1.0;
//     obs_marker.color.a = 0.8;
//     obs_marker.color.r = 1.0;
//     obs_marker.color.g = 0.0;
//     obs_marker.color.b = 0.0;
    
//     int step = (10.0 / 180.0) * M_PI / scan.get()->angle_increment;

//     for (int i = 0, j = 0; i < scan.get()->ranges.size(); i+=step, j++) {
//         double dis = scan.get()->ranges[i];
//         // ROS_INFO("%f", dis);
//         Vector2 obs(dis * cos(angle), dis * sin(angle));
//         // obs += agent[n]->position_;
//         //computeObstacleLine(obs, n);

//         //给激光雷达赋值
//         lidar_legth[n][j] = dis;

//         //ROS_INFO("Lidar-data, Line = %i, data = %f", j,dis);//lidar_legth[j]
//         //
//         angle += scan.get()->angle_increment * step;

//         if (angle > M_PI) {
//             angle -= 2 * M_PI;
//         } else if (angle < -M_PI) {
//             angle += 2 * M_PI;
//         }
//         geometry_msgs::Point tmp;
//         tmp.x = obs.x();
//         tmp.y = obs.y();
//         tmp.z = 0.0;
//         obs_marker.points.push_back(tmp);
//     }
//     //无用
//     //obs_pub_[n].publish(obs_marker);
// }

//订阅者--机器人位置--回调函数
void odom_callback(const nav_msgs::OdometryConstPtr odom, int n) {
    // ROS_INFO("odom:%d", odom.get()->header.stamp.nsec);
    agent[n]->position_ = collvoid::Vector2(odom.get()->pose.pose.position.x, odom.get()->pose.pose.position.y);
    agent[n]->velocity_ = rotateVectorByAngle(odom.get()->twist.twist.linear.x,
                                                odom.get()->twist.twist.linear.y, (agent[n]->heading_));
    agent[n]->heading_ = tf::getYaw(odom.get()->pose.pose.orientation);
    // ROS_INFO("%d, velocity_:%6.2f,%6.2f", n, agent[n]->velocity_.x(), agent[n]->velocity_.y());
    //agent[n]->footprint_ = rotateFootprint(minkowski_footprint_, agent[n]->heading_);

    //回调函数延时指数--减1
    _Delay_index[n]--;
}

// //用于IDSI_Rank数据发布--回调函数--230710
// void IDSI_callback(const std_msgs::Float32MultiArray::ConstPtr idsi, int index)
// {
//   // 获取收到的数据
//   std::vector<float> data = idsi->data;

//   // 打印接收到的数组
//   ROS_INFO("Received Array %d:", index);
//   for (int i = 0; i < data.size(); ++i)
//   {
//     idsi_data[index][i] = data[i];//idsi数据赋值
//     ROS_INFO("%f", data[i]);
//   }
// }


// // 打印IDSI--接收到的数组
// void Print_IDSI()
// {
//   for (int i = 0; i < 4; ++i)
//   {
//     // 打印接收到的数组
//     ROS_INFO("Received Robot = %i, RobIDSI-Array  = [%f, %f, %f, %f ]", i, idsi_data[i][0], idsi_data[i][1], idsi_data[i][2], idsi_data[i][3] );//, idsi_data[i][4], idsi_data[i][5]
//   } 
// }

//机器人线速度角速度--计算函数
inline Vec2d compute_vel(Vec2d pos, double angle, Vec2d goal, double angular_fb) {
    Vec2d delta_pos = goal - pos;
    double dis = delta_pos.mod();
    double delta_angle;
    double ang = delta_pos.ang();
    if (fabs(angle - ang) > M_PI) {
        if (angle > ang) {
            delta_angle = -(M_PI - angle + ang + M_PI);
        } else {
            delta_angle = M_PI - ang + angle + M_PI;
        }
    } else {
        delta_angle = angle - ang;
    }

    //比例系数
    double vel = 1.0;
    
    //线速度的最大最小范围
    double linear = LIMIT_R(dis * vel, 0, 0.25);
    if (fabs(delta_angle) > M_PI * 1.0 / 4) {
        linear = 0.0;
    }
    static double last_delta_angle = 0.0;
    double angular = -LIMIT_R(delta_angle * 10.0 + angular_fb * 0.0, -2.82, 2.82);//delta_angle * 10.0是角速度的比例系数
    last_delta_angle = delta_angle;
    if (dis < 0.1) {
        angular = 0.0;
    }
    return Vec2d(linear, angular);
}

