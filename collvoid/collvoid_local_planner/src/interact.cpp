//-----------------------------------------------------------------------------
//	Version 1.0
//-----------------------------------------------------------------------------
//	File Name: interact.cpp
//-----------------------------------------------------------------------------
//	CopyRight (C) 2023, TigherLab.USTC
//-----------------------------------------------------------------------------
//	Modifier	Date		Detail
//
//	qinjianmin		20230325	create
//-----------------------------------------------------------------------------
//	Note:   Interaction 交互数据程序--cpp文件
//-----------------------------------------------------------------------------


#include <angles/angles.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <collvoid_srvs/GetObstacles.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include "collvoid_local_planner/common.h"
#include "collvoid_local_planner/interact.h"
#include "collvoid_local_planner/Vector2.h"
#include <collvoid_local_planner/Agent.h>
#include <collvoid_local_planner/GetCollvoidTwist.h>
#include<gazebo_msgs/ModelState.h>
#include <tf/tf.h>
using namespace collvoid;

//超参数
//机器人激光雷达探测长度数据：36个值，从机器人正前方x轴方向，开始逆时针数？？
float _lidarlegth_data[40] = { 0.3,0.3,0.3,0.3,0.3,  0.3,0.3,0.3,0.3,0.3,  0.3,0.3,0.3,0.3,0.3,  0.3,0.3,0.3,0.3,0.3,
0.3,0.3,0.3,0.291,0.243,  0.212,0.193,0.180,0.173,0.171,  0.171,0.173,0.180,0.193,0.212,  0.243,0.291,0.3,0.3,0.3 };


// { 0.2,0.2,0.2,0.2,0.2,  0.2,0.2,0.2,0.2,0.2,  0.2,0.2,0.2,0.2,0.2,  0.2,0.2,0.2,0.2,0.2,
// 0.2,0.2,0.2,0.2,0.2,  0.2,0.186,0.168,0.158,0.152,  0.15,0.152,0.158,0.168,0.186,  0.2,0.2,0.2,0.2,0.2 };

//机器人激光雷达--放大倍数
float _lidarlagre = 1.4;//2.1 1.8 1.4




//  0.3,0.3,0.3,0.3,0.3,  0.3,0.3,0.3,0.3,0.3,  0.3,0.3,0.3,0.3,0.3,  0.3,0.3,0.3,0.3,0.3,
// 0.3,0.3,0.3,0.291,0.243,  0.212,0.193,0.180,0.173,0.171,  0.171,0.173,0.180,0.193,0.212,  0.243,0.291,0.3,0.3,0.3

//超参数
//机器人横纵向每格子移动数据，每格1m？？？
float _steplegth_x = 0.2;//0.1
float _steplegth_y = 0.2;//0.1

float _steplegth_loc = 1.0;//0.1 0.6 1.0

//超参数
//机器人初始坐标
float _beginloc_R_0_x = 0;//0.1
float _beginloc_R_0_y = 0;//0.1

float _beginloc_R_1_x = 2.9;//1.0
float _beginloc_R_1_y = 0.9;//0.5

//超参数
//机器人初始速度
float _beginspeed_R_0 = 0.08;//0.1
float _beginspeed_R_1 = 0.08;//0.1

//速度列表 横--速度列表
//x向--5
float _speed_list[6] = {0.10, 0.14, 0.18 ,0.22, 0.25, 0.10};

//超参数
//机器人初始方向角
float _beginheading_R_0 = 0;//0.1
float _beginheading_R_1 = 3.141592653;//0.1

//超参数
//机器人根据到目标夹角的旋转的超参数
//正负代表方向正方向，顺时针还是逆时针是
float _w_p = 1.0 * 0.05;

//超参数
//机器人到目标的范围，小于它即为到达目标
float _distance_target = 0.2;

//超参数
//延时指数--用于位置回调函数odom_callback延时
int _Delay_index[2] = {50,50};

//临时距离存储数组，和_index_target长度对应
float _dislist[3] = {0, 0, 0};


int main(int argc, char **argv) {
    ros::init(argc, argv, "interact");
    //各个函数值初始化
    init();

    //出发位置计算
    // local_compute();

    ros::Rate rate(50);

    while (ros::ok()) {
        local_compute();
        ros::spinOnce();
        rate.sleep();
    }
}


//各个函数值初始化
void init()
{
    ros::NodeHandle nh("~");

    for (int i = 0; i < 2; i++) {
        agent.push_back(AgentPtr(new Agent));
    }

    //发布初始化
    //仿照写一个发布者？--发布机器人的线速度和角速度
    //消息类型为geometry_msgs::Point
    //还有待细细调整-------------------------------------------------------------------------
    //cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/cmd_vel", 4 * 4);
    for (int i = 0; i < 2; i++)
    {
        //sac_cmd_vel_sub_[i] = nh.subscribe<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/sac_cmd_vel", 4 * 4, boost::bind(&OrcaLocalPlanner::sac_cmd_vel_callback, this, _1, i));
        //此处改为/cmd_vel-直接发送数据，或/sac_cmd_vel--经过orca处理发送数据
        sac_cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/sac_cmd_vel", 4 * 4);
    }

    //发布初始化
    //仿照写一个发布者？--发布重置目标位置
    //消息类型为gazebo_msgs::ModelState
    //还有待细细调整-------------------------------------------------------------------------
    //cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/cmd_vel", 4 * 4);
    for (int i = 0; i < 2; i++)
    {
        //sac_cmd_vel_sub_[i] = nh.subscribe<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/sac_cmd_vel", 4 * 4, boost::bind(&OrcaLocalPlanner::sac_cmd_vel_callback, this, _1, i));
        set_model_pub_[i] = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 4 * 4);
    }

    //订阅激光雷达数据？？
    //还有待细细调整-------------------------------------------------------------------------
    //scan_sub_[i] = nh.subscribe<sensor_msgs::LaserScan>("/tb3_" + std::to_string(i ) + "/scan", 1, boost::bind(&OrcaLocalPlanner::scan_callback, this, _1, i));
    for (int i = 0; i < 2; i++)
    {
        scan_sub_[i] = nh.subscribe<sensor_msgs::LaserScan>("/tb3_" + std::to_string(i) + "/scan", 1, boost::bind(&scan_callback, _1, i));
    }

    //订阅者--订阅机器人位置信息？？
    for (int i = 0; i < 2; i++)
    {
        odom_sub_[i] = nh.subscribe<nav_msgs::Odometry>("/tb3_" + std::to_string(i) + "/odom", 4 * 4, boost::bind(&odom_callback, _1, i));
    }


    //超参数 int
    //机器人横纵向格子编号--初始化为0
    _index_x = 0;
    _index_y = 0;

    //机器人速度--编号--初始化为0
    _index_speed = 0;

    //机器人速度--编号--初始化为0
    _index_target = 0;


    //初始化--赋初值，float
    //机器人数据数组：横--序号，0-1  竖--位置坐标x,y,速度Vx,Vy,还需要加上角度数据，如何表示初始角度？？
    //public float begin_inf[2][4];

    //机器人-0，的初始位置为原点？？/////////////////////////////////////////////////////////////////////////////////////////////
    begin_inf[0][0] = _beginloc_R_0_x;//0
    begin_inf[0][1] = _beginloc_R_0_y;

    //机器人-0，的初始速度/////////////////////////////////////////////////////////////////////////////////////////////
    begin_inf[0][2] = _beginspeed_R_0;//0.10

    //机器人-0，的初始方向角--弧度制
    begin_inf[0][3] = _beginheading_R_0;

    //机器人-1，的初始位置为(-1f,1f)？？/////////////////////////////////////////////////////////////////////////////////////////////
    begin_inf[1][0] = _beginloc_R_1_x;//-0.5;-1
    begin_inf[1][1] = _beginloc_R_1_y;

    //机器人-1，的初始速度/////////////////////////////////////////////////////////////////////////////////////////////
    begin_inf[1][2] = _speed_list[0];//0.05 _beginspeed_R_1

    //机器人-1，的初始方向角--弧度制
    begin_inf[1][3] = _beginheading_R_1;

    //机器人目的地--数组float ：横--序号，0-1  竖--位置坐标x,y,速度Vx,Vy,还需要加上角度数据，如何表示初始角度？？
    //机器人-0
    target_inf[0][0] = 2.0;
    target_inf[0][1] = 0;

    //机器人-1
    target_inf[1][0] = -begin_inf[1][0];//-begin_inf[1][0]
    target_inf[1][1] = -begin_inf[1][1] + _steplegth_loc * (_index_target -1 );//-begin_inf[1][1];//-begin_inf[1][1]

    ROS_INFO("Begin Target!!!Robot_1, X = %f, Y = %f,Robot_1, X = %f, Y = %f", target_inf[0][0], target_inf[0][1], target_inf[1][0], target_inf[1][1]);

    //初始化--赋初值,bool
    //机器人到达数组--初始均为false
    //public bool arrival_bool[2];
    arrival_bool[0] = false;
    arrival_bool[1] = false;

    // //初始化--赋初值,bool
    // //机器人速度到达--数组--初始为false
    // speedlist_bool[0]= false;
    // speedlist_bool[1]= false;
    
    //初始化--赋初值,bool
    //机器人发布速度信息--判别数组--初始为true
    //velsend_bool[0] = true;
    //velsend_bool[1] = true;

    //程序开始判别--bool-为ture
    scriptbegin_bool = true;

    //初始化--赋初值，float
    //交互数据收集 横--横向编号， 竖--纵向编号,内容为交互数据值
    //x向--20  y向--10
    //public float begin_inf[20][20];
	for (int i = 0; i < 20; i++)
	{
		    for (int j = 0; j < 10; j++)
            {
                //全部赋初始值为--0
                interact_data[i][j] = 0;
            }
    }


    //初始化--赋初值，float
    //距离数据收集 横--横向编号， 竖--纵向编号,内容为出发点到终点距离值
    //x向--20  y向--10
    //public float begin_inf[20][20];
	for (int i = 0; i < 20; i++)
	{
		    for (int j = 0; j < 10; j++)
            {
                //全部赋初始值为--0
                dis_data[i][j] = 0;
            }
    }


    //初始化--赋初值，float
    //真实数据收集 横--横向编号， 竖--纵向编号,内容为交互数据值
    //x向--20  y向--10
    //public float begin_inf[20][20];
	for (int i = 0; i < 20; i++)
	{
		    for (int j = 0; j < 10; j++)
            {
                //全部赋初始值为--0
                real_data[i][j] = 0;
            }
    }

    //初始化--赋初值，float
    //机器人激光雷达探测长度数据：36个值，从正前方顺时针开始数？？
    //public float lidar_legth[36];
    //改为40个值，以免雷达数组溢出
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 40; j++)
        {
            lidar_legth[i][j] = 0;                
        }
    }


    //距离数据收集 横--横向编号， 竖--纵向编号,内容为出发点到终点距离值
    //第一位置组的--第一个数据
    //distance_data( float _data_1_x, float _data_1_y, float _data_2_x, float _data_2_y )
    _dislist[0] = distance_data( begin_inf[1][0], begin_inf[1][1], target_inf[1][0], target_inf[1][1] );


    ROS_INFO("C++ Interact begins, index_x is :%i, index_y is :%i, _index_speed is :%i, _index_target is :%i", _index_x,_index_y,_index_speed,_index_target);
    ROS_INFO("Begin Locate, robot-0 is, %f,%f,robot-1 is,%f,%f.", begin_inf[0][0],begin_inf[0][1],begin_inf[1][0],begin_inf[1][1]);
}

//出发位置计算
void local_compute()
{
    // //定义2个机器人的线速度角速度--发布者的twist
    // geometry_msgs::Twist sac_cmd_vel_0;
    // geometry_msgs::Twist sac_cmd_vel_1;

    //定义2个机器人的重置位置--发布者的ModelState
    gazebo_msgs::ModelState restart_location_0;
    gazebo_msgs::ModelState restart_location_1;


    //程序开始判别--bool
    if(scriptbegin_bool == true)
    {
        //如果两机器人--均到达终点,
        if( (arrival_bool[0] == true) && (arrival_bool[1] == true) )
        {

            //重置新开始点？？
            //机器人横纵向格子点编号--增加
            //第一行第一列开始，先横行增加，完成一行，再进行下一行
            if(_index_x < 20)
            {
                if( _index_y < 10)
                {
                    if( _index_speed < 5)
                    {
                        if( _index_target < 2 )
                        {
                            //机器人目标--编号--增加1
                            _index_target++;
                            
                            //重新设定，机器人-1，的目标位置
                            target_inf[1][0] = -begin_inf[1][0];
                            target_inf[1][1] = -begin_inf[1][1] + _steplegth_loc * (_index_target -1 );//-begin_inf[1][1]

                            //距离数据收集 横--横向编号， 竖--纵向编号,内容为出发点到终点距离值
                            //distance_data( float _data_1_x, float _data_1_y, float _data_2_x, float _data_2_y )
                            _dislist[_index_target] = distance_data( begin_inf[1][0], begin_inf[1][1], target_inf[1][0], target_inf[1][1] );  
                        }
                        else
                        {
                            //机器人速度--编号--增加1
                            _index_speed++;

                            //--2机器人，重置速度为初始速度/////////////////////////////////////////////////////////////////////////////////////////////
                            begin_inf[0][2] = _beginspeed_R_0;//0.10

                            begin_inf[1][2] =  _speed_list[_index_speed] ;//0.10_beginspeed_R_1    

                            //机器人目标--编号--初始化为0
                            _index_target = 0; 

                            //重新设定，机器人-1，的目标位置
                            target_inf[1][0] = -begin_inf[1][0];
                            target_inf[1][1] = -begin_inf[1][1] + _steplegth_loc * (_index_target -1 );//-begin_inf[1][1]

                            //距离数据收集 横--横向编号， 竖--纵向编号,内容为出发点到终点距离值
                            //distance_data( float _data_1_x, float _data_1_y, float _data_2_x, float _data_2_y )
                            _dislist[_index_target] = distance_data( begin_inf[1][0], begin_inf[1][1], target_inf[1][0], target_inf[1][1] );  

                        }

                    }
                    else
                    {

                        //机器人速度--编号--重置为0
                        _index_speed = 0;

                        //--2机器人，重置速度为初始速度/////////////////////////////////////////////////////////////////////////////////////////////
                        begin_inf[0][2] = _beginspeed_R_0;//0.10

                        begin_inf[1][2] = _speed_list[_index_speed] ;//0.10_beginspeed_R_1  




                        //距离数据收集 横--横向编号， 竖--纵向编号,内容为出发点到终点距离值
                        //distance_data( float _data_1_x, float _data_1_y, float _data_2_x, float _data_2_y )
                        //_dislist[0]列表所有数据求和，再乘以_index_speed的数量
                        dis_data[_index_x][_index_y] = 5 *( _dislist[0] +_dislist[1] +_dislist[2] ) ;
                        //ROS_INFO("Compute!!!  dis_data = A, index_x is :%i, index_y is :%i, dislist[0,1,2] is :%f, %f, %f", _index_x,_index_y,_dislist[0],_dislist[1],_dislist[2] );
 
                        _index_y++;
                        //机器人-1，的初始位置，y轴增加1格
                        begin_inf[1][1] = begin_inf[1][1] - _steplegth_y;

                    }


                }
                else//换行
                {

                    //先打印上一行的交互数据
                    //打印--交互数据--一行的数组
                    //interactdata_update( int _line_x )
                    interactdata_print(_index_x);

                    _index_y = 0;
                    _index_x++;

                    //机器人-1，的初始位置，y轴重置为初始坐标
                    begin_inf[1][1] = _beginloc_R_1_y;

                    //机器人-1，的初始位置，x轴增加1格
                    begin_inf[1][0] = begin_inf[1][0] - _steplegth_x;

                    // //因为换行，所以要算_index_y = 0
                    // //距离数据收集 横--横向编号， 竖--纵向编号,内容为出发点到终点距离值
                    // //distance_data( float _data_1_x, float _data_1_y, float _data_2_x, float _data_2_y )
                    // //_dislist[0]列表所有数据求和，再乘以_index_speed的数量
                    // dis_data[_index_x][_index_y] = 5 *( _dislist[0] +_dislist[1] +_dislist[2] ) ;   
                    // ROS_INFO("Compute!!!  dis_data= B, index_x is :%i, index_y is :%i, dislist[0,1,2] is :%f, %f, %f", _index_x,_index_y,_dislist[0],_dislist[1],_dislist[2] );
                }
            }
            else
            {
                //程序开始判别--bool--全部结束
                scriptbegin_bool = false;
            }



            // ROS_INFO("scan:%d", scan.get()->header.stamp.nsec);

            /////////////////////////////////////////////////////////////////////////////////////////////////////////一会恢复--23.05.08
            //ROS_INFO("Reborth index_x is :%i, index_y is :%i, index_speed is :%i, _index_target is :%i", _index_x,_index_y,_index_speed,_index_target );
            //ROS_INFO("New Locate, robot-0 is, %f,%f,robot-1 is,%f,%f.", begin_inf[0][0],begin_inf[0][1],begin_inf[1][0],begin_inf[1][1]);
            //ROS_INFO("New Target, robot-0 is, %f,%f,robot-1 is,%f,%f.", target_inf[0][0],target_inf[0][1],target_inf[1][0],target_inf[1][1]);


            //计算重新开始位置--机器人-0
            restart_location_0.model_name = "tb3_" + std::to_string(0);
 
            restart_location_0.pose.position.x = begin_inf[0][0];//begin_inf[0][0] 2.0
            restart_location_0.pose.position.y = begin_inf[0][1];//begin_inf[0][1]
            restart_location_0.pose.position.z = 0;

            //方向角转换为四元数
            auto q_0 = tf::createQuaternionFromYaw(begin_inf[0][3]);

            restart_location_0.pose.orientation.x = q_0.x();
            restart_location_0.pose.orientation.y = q_0.y();
            restart_location_0.pose.orientation.z = q_0.z();
            restart_location_0.pose.orientation.w = q_0.w();


            //计算重新开始位置--机器人-1
            restart_location_1.model_name = "tb3_" + std::to_string(1);
 
            restart_location_1.pose.position.x = begin_inf[1][0];//begin_inf[1][0] -1.0
            restart_location_1.pose.position.y = begin_inf[1][1];//begin_inf[1][1] -1.0
            restart_location_1.pose.position.z = 0;

            //方向角转换为四元数
            auto q_1 = tf::createQuaternionFromYaw(begin_inf[1][3]);

            restart_location_1.pose.orientation.x = q_1.x();
            restart_location_1.pose.orientation.y = q_1.y();
            restart_location_1.pose.orientation.z = q_1.z();
            restart_location_1.pose.orientation.w = q_1.w();

            //仿照写一个发布者？
            //发布2个机器人的重置位置--发布者的ModelState
            //还有待细细调整-------------------------------------------------------------------------
            //cmd_vel_pub_[n].publish(cmd_vel);
            set_model_pub_[0].publish(restart_location_0);
            set_model_pub_[1].publish(restart_location_1);


            // //--2机器人，重置速度为初始速度/////////////////////////////////////////////////////////////////////////////////////////////
            // begin_inf[0][2] = _beginspeed_R_0;//0.10

            // begin_inf[1][2] = _beginspeed_R_1;//0.10

            //延时指数--用于位置回调函数odom_callback延时
            _Delay_index[0] = 50;
            _Delay_index[1] = 50;

            /////////////////////////////////////////////////////////////////////////////////////////////////////////一会恢复--23.05.08
            //ROS_INFO("Reborth success!!!speed = Robot-0 is :%f,Robot-1 is :%f", begin_inf[0][2],begin_inf[1][2]);//"Reborth index_x is :%i, index_y is :%i", _index_x,_index_y
            //机器人发布速度信息--判别数组--完成重置后，重新开始发数据
            //velsend_bool[0] = true;
            //velsend_bool[1] = true;

            //如果两机器人均到达终点--重新开始，bool为false
            arrival_bool[0] = false;
            arrival_bool[1] = false;


        }
        else//如果两机器人--没有都到达终点
        {
            //两机器人重合的区域，要去除
            if( (4 <=_index_y)&&(_index_y <= 5)&&(14 <= _index_x)&&(_index_x <= 15) )  
            {

                //交互数据计算--重合区域交互为空集--更新对应的横纵坐标的交互数据
                interact_data[_index_x][_index_y] = 0;

                ROS_INFO("Jump this area!!!!, index_x is :%i, index_y is :%i", _index_x,_index_y);

                //如果在重合区域--直接跳过重新开始，bool为ture
                arrival_bool[0] = true;
                arrival_bool[1] = true;

            }
            else//不在重合区域，正常运行，发布目标位置，收集交互数据
            {
                //由目标位置，换算线速度角速度

                // cmd_vel.linear.x = std::min(vstar, vMaxAng());
                // cmd_vel.linear.y = 0.0;
                //cmd_vel.angular.z = sign(dif_ang) * std::min(std::abs(dif_ang / time_to_holo_), max_vel_th_);

                //线速度角速度换算-----------------------------------------------

                //角速度，机器人-0角速度计算
                //目标点和y轴的夹角，顺时针为正？？
                // float _angle_robot_0 = asin( ( 1*(target_inf[0][1] - agent[0]->position_.y()) - 0*(target_inf[0][0] - agent[0]->position_.x()) )
                // /sqrt( pow(target_inf[0][0] - agent[0]->position_.x(), 2.0) + pow(target_inf[0][1] - agent[0]->position_.y(), 2.0) ) );
                
                //为运动机器人--计算运行真实速度
                for (int i = 0; i < 2; i++)
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

                    //计算速度
                    //compute_vel(Vec2d pos, double angle, Vec2d goal, double angular_fb, float max_speed) 

                    //ROS_INFO("Print!!!!!  linear =, max_speed = %f", begin_inf[i][2] );//linear ,max_speed
                    _Robot_vel = compute_vel(_Robot_pos, agent[i]->heading_, _Robot_goal, 0, begin_inf[i][2] );//begin_inf[0][2]为最大速度

                    //定义机器人的线速度角速度--发布者的twist
                    geometry_msgs::Twist sac_cmd_vel;

                    //机器人-0
                    //线速度
                    sac_cmd_vel.linear.x = _Robot_vel.x_;

                    //默认为0--因为是2d平面
                    sac_cmd_vel.linear.y = 0.0;//默认为0

                    //角速度，机器人-0角速度默认为0
                    //sac_cmd_vel_0.angular.z = 0.0;//默认为0
                    sac_cmd_vel.angular.z = _Robot_vel.y_;


                    //发布机器人的线速度和角速度
                    //还有待细细调整-------------------------------------------------------------------------
                    //cmd_vel_pub_[n].publish(cmd_vel);
                    sac_cmd_vel_pub_[i].publish(sac_cmd_vel);

                    // ROS_INFO("Runing!!!,Robot = %i, speed= %f, ang_speed = %f, target_inf = %f,%f,positon = %f,%f",
                    // i,sac_cmd_vel.linear.x, sac_cmd_vel.angular.z, target_inf[i][0],target_inf[i][1],agent[i]->position_.x(),agent[i]->position_.y()
                    // );//"Arrived target, Robot = %i", i
                }


                //交互数据计算--更新对应的横纵坐标的交互数据
                interactdata_update( _index_x, _index_y );

                //检测目标距离
                for (int i = 0; i < 2; i++)
                {
                    //机器人当前到目标距离
                    float _dis_to_target = sqrt( pow( agent[i]->position_.x() - target_inf[i][0], 2.0) + pow(agent[i]->position_.y() - target_inf[i][1], 2.0) );
  
                    //判断到目标距离是否满足条件，同时满足回调函数延时条件
                    if(( _dis_to_target < _distance_target )&&( _Delay_index[i] < 0 ))
                    {

                        //机器人-i，的速度设置为0///////////////////////////////////////////////////////////////////////此处有修改，去除设置为0设定，23.5.11
                        //begin_inf[i][2] = 0;

                        // ROS_INFO("Arrived target!!!, Relay_index is %i, Robot = %i,real-time position is X = %f, Y = %f, target location is X = %f, Y = %f",
                        //  _Delay_index[i], i,agent[i]->position_.x(),agent[i]->position_.y(),target_inf[i][0],target_inf[i][1]);//velsend_bool[i]

                        //则该机器人到达终点--重新开始，bool为ture
                        arrival_bool[i] = true;

                        //机器人发布速度信息--判别数组--停发数据
                        //velsend_bool[i] = false;
                    }
                }
            }
        }
    }
    else
    {
        //程序结束，debug
        //ROS_INFO("Over!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }




}


//交互数据计算
void interactdata_update( int _localindex_x, int _localindex_y )
{

    //初始化--赋初值，float
    //机器人激光雷达探测长度数据：36个值，从正前方顺时针开始数？？
    //public float lidar_legth[36];
//--------------------------------------------------------------------------------------------
    for (int i = 0; i < 40; i++)
    {
        //读取机器人-1，的雷达数据
        if(lidar_legth[0][i] < _lidarlegth_data[i] * _lidarlagre )//激光数据---乘以放大倍数
        {
            //交互数据收集 横--横向编号， 竖--纵向编号,内容为交互数据值
            //横向--10  纵向--20      
            interact_data[_localindex_x][_localindex_y]++;

            //  ROS_INFO("interact_data X= %i,X= %i,data = %f.",_localindex_x, _localindex_y,
            //  interact_data[_localindex_x][_localindex_y]);
        }
    }

}

//打印--交互数据--一行的数组
void interactdata_print( int _line_x )
{
    // int input[] = { 1, 2, 3, 4, 5 };
    // size_t n = sizeof(input)/sizeof(input[0]);
    
    //求数组长度--x维度
    size_t n = sizeof(interact_data[_line_x])/sizeof(interact_data[_line_x][0]);

    //赋值一个空的字符串
    std::string content = "";

        //赋值一个空的字符串
    std::string content_1 = "";

    //循环遍历数组元素
    for (size_t i = 0; i < n; i++) {

        //真实数据计算
        real_data[_line_x][i] = interact_data[_line_x][i] / dis_data[_line_x][i];

        content += std::to_string(interact_data[_line_x][i]);//interact_data[_line_x][i]
        content.push_back(',');

        content_1 += std::to_string(dis_data[_line_x][i]);//interact_data[_line_x][i]
        content_1.push_back(',');

        //std::cout << input[i] << ' ';
    }
    //去掉最后一个元素“,”
    content.pop_back();

        //去掉最后一个元素“,”
    content_1.pop_back();

    //打印该行的--交互数据
    ROS_INFO("Get!!!!!  Line = %i, interact_data = [%s]", _line_x , content.c_str());
    ROS_INFO("Get!!!!!  Line = %i, dis_data = [%s]", _line_x , content_1.c_str());
    //return 0;
}

//打印--计算两点距离
float distance_data( float _data_1_x, float _data_1_y, float _data_2_x, float _data_2_y )
{
    //先平方，再开方，计算距离
    float _dis = sqrt( pow( _data_2_x - _data_1_x, 2.0) + pow(_data_2_y - _data_1_y, 2.0) );

    return _dis;
  
}

//订阅者--激光雷达--回调函数
void scan_callback(const sensor_msgs::LaserScanConstPtr scan, int n) {
    double angle = 0;
    //    double angle = agent[n]->heading_;
    //obs_lines_[n].clear();
    // ROS_INFO("scan_callback");
    // ROS_INFO("scan:%d", scan.get()->header.stamp.nsec);

    // auto msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/tb3_" + std::to_string(n) + "/odom", ros::Duration(5));
    // odom_callback(msg, n);

    visualization_msgs::Marker obs_marker;

    obs_marker.header.stamp       = ros::Time::now();
    obs_marker.header.frame_id    = "map";

    obs_marker.ns = "obs";
    obs_marker.id = 0;
    obs_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    obs_marker.action = visualization_msgs::Marker::ADD;
    obs_marker.scale.x = 0.01;
    obs_marker.scale.y = 0.01;
    obs_marker.scale.z = 0.01;
    obs_marker.pose.orientation.x = 0.0;
    obs_marker.pose.orientation.y = 0.0;
    obs_marker.pose.orientation.z = 0.0;
    obs_marker.pose.orientation.w = 1.0;
    obs_marker.color.a = 0.8;
    obs_marker.color.r = 1.0;
    obs_marker.color.g = 0.0;
    obs_marker.color.b = 0.0;
    
    int step = (10.0 / 180.0) * M_PI / scan.get()->angle_increment;

    for (int i = 0, j = 0; i < scan.get()->ranges.size(); i+=step, j++) {
        double dis = scan.get()->ranges[i];
        // ROS_INFO("%f", dis);
        Vector2 obs(dis * cos(angle), dis * sin(angle));
        // obs += agent[n]->position_;
        //computeObstacleLine(obs, n);

        //给激光雷达赋值
        lidar_legth[n][j] = dis;

        //ROS_INFO("Lidar-data, Line = %i, data = %f", j,dis);//lidar_legth[j]
        //
        angle += scan.get()->angle_increment * step;

        if (angle > M_PI) {
            angle -= 2 * M_PI;
        } else if (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        geometry_msgs::Point tmp;
        tmp.x = obs.x();
        tmp.y = obs.y();
        tmp.z = 0.0;
        obs_marker.points.push_back(tmp);
    }
    //无用
    //obs_pub_[n].publish(obs_marker);
}

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

//机器人线速度角速度--计算函数
inline Vec2d compute_vel(Vec2d pos, double angle, Vec2d goal, double angular_fb, float max_speed) {
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
    double linear = LIMIT_R(dis * vel, 0, max_speed );//0.25

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

