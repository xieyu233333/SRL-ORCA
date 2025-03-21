//-----------------------------------------------------------------------------
//	Version 1.0
//-----------------------------------------------------------------------------
//	File Name: Visual_Field_1.cpp
//-----------------------------------------------------------------------------
//	CopyRight (C) 2023, TigherLab.USTC
//-----------------------------------------------------------------------------
//	Modifier	Date		Detail
//
//	qinjianmin		20230307	create
//-----------------------------------------------------------------------------
//	Note:   Social Visual_Field 社会力观测视野程序
//-----------------------------------------------------------------------------

//interact.cpp拷贝--23.05.13
#include <angles/angles.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <collvoid_srvs/GetObstacles.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include "collvoid_local_planner/common.h"
#include "collvoid_local_planner/visual_field.h"//需要根据程序h文件更改--230623
#include "collvoid_local_planner/Vector2.h"
#include <collvoid_local_planner/Agent.h>
#include <collvoid_local_planner/GetCollvoidTwist.h>
#include<gazebo_msgs/ModelState.h>
#include <tf/tf.h>

#include <std_msgs/Int32MultiArray.h>//用于IDSI_Rank数据发布--230702
#include <std_msgs/Float32MultiArray.h>//用于IDSI_Rank数据发布--230710
using namespace collvoid;

//原本--无用？？？
// #include <iostream>
#include <vector>
// #include <cmath>
// #include "visual_field.h"
// using namespace std;
using std::vector;

//社会力模型观测--各向异性系数 anisotropic intensity
float Anis_in = 0.4;

////机器人观测半径--超参数
float RA_str = 3.0;/////需要根据实际场景调整------------------------------------------------------------待定


//时间段，观测频率--超参数
float Time_Interval = 2.0;//_time_long，2

//超参数
//延时指数--用于位置回调函数odom_callback延时
int _Delay_index[6] = { 50,50,50,50,50, 50 };

// //log临时标记--230630
// int __log_bool = false;


///////////////////////////////////////////////////////////////////////////从此开始----IDSI模块超参数------230704

//超参数
//安全机器人雷达数据：横--序号，0-9  竖--一定角度范围内，的机器人激光雷达数据，建议所有机器人的全部统一
//S_AHV的雷达探测长度-----------------------------------------------------------暂定1.0，待调整
//若实物机器人，雷达分布，按前进方向为0线，逆时针旋转分布。则0-5线，34-39线为危险
float SafeyLidar_Data[10] = { 0.45, 0.41, 0.35, 0.32, 0.25,   0.25, 0.32, 0.35, 0.41, 0.45 };

//最小Pr值
float MinPr_c = 0.05;

//单向观测，Interact_Pr值缩小倍数
float MinPr_e = 0.1;

//IDSI相关超参数
//以下参数--参照清华大学王建强的论文
float SPE_K = 0.5;

//障碍物和机器人有相同的SPE_R
float SPE_R = 1.0;

float SPE_k1 = 1.5;//不能为1.0，否则分母为0

float SPE_k3 = 16.0;//45.0,160太大？？机器人和汽车速度差异太大，改为16？？

float SPE_a = 0.1;//英文论文是0.06是否过小？？？，硕士论文0.1

//需要自己设定
float SPE_k4 = 1.2;//暂定1.5-2.0之间较好？？根据公式？？王建强论文中是 k2 = 1.2,230710实验，2.0偏大，调小到0.2？？

//超参数---IDSI使用的激光雷达的线数，暂定10
int Num_Lidar = 10;

////超参数---基础优先级的初始值
float _data_PrBase[10] = { 1.00, 1.01, 1.02, 1.03, 1.04,   1.05, 1.06, 1.07, 1.08, 1.09 };


int main(int argc, char **argv) {
    ros::init(argc, argv, "visual_field");///////////////////////////////////////////////////////////////////////////////////////是否正确？
    //各个函数值初始化
    init();

	//idsi相关接口的初始化
	idsi_init();

	//设置更新频率，赫兹？？是否设置为=2
    ros::Rate rate(Time_Interval);///////////////////////////////////////////////////////////////////////////////////////是否正确？--赫兹

    while (ros::ok()) {
        //local_compute();

		Robot_Data_Update();

		//更新机器人部分雷达观测数据--不需要？？
		idsi_Lidar_Update();

		//计算机器人IDSI数据------------------------------正在逐步完善中！！！！！！！！！！！！！！！！！！！！！
		IDSI_Compute();

		//输出机器人观测函数--向ros发送最新IDSI数据
		Output_Observe_index();

		Print_IDSI();//打印测试数据------------------测试用

        ros::spinOnce();
        rate.sleep();
    }



}


//各个函数值初始化
void init()
{
		//数据初始化--开始
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ros::NodeHandle nh("~");

    for (int i = 0; i < 6; i++) {
        agent.push_back(AgentPtr(new Agent));
    }

    //发布初始化
    //仿照写一个发布者？--发布机器人的线速度和角速度---------------------------------还要问家鑫，怎么发布一组bool数据？？？
    //消息类型为geometry_msgs::Point
    //cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/cmd_vel", 4 * 4);
    for (int i = 0; i < 6; i++)
    {
        //sac_cmd_vel_sub_[i] = nh.subscribe<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/sac_cmd_vel", 4 * 4, boost::bind(&OrcaLocalPlanner::sac_cmd_vel_callback, this, _1, i));
        //此处改为/cmd_vel-直接发送数据，或/sac_cmd_vel--经过orca处理发送数据
        sac_cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/sac_cmd_vel", 4 * 4);
    }

    // //发布初始化
    // //仿照写一个发布者？--发布重置目标位置
    // //消息类型为gazebo_msgs::ModelState
    // //cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/cmd_vel", 4 * 4);
    // for (int i = 0; i < 6; i++)
    // {
    //     //sac_cmd_vel_sub_[i] = nh.subscribe<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/sac_cmd_vel", 4 * 4, boost::bind(&OrcaLocalPlanner::sac_cmd_vel_callback, this, _1, i));
    //     set_model_pub_[i] = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 4 * 4);
    // }

    //订阅激光雷达数据？？
    //scan_sub_[i] = nh.subscribe<sensor_msgs::LaserScan>("/tb3_" + std::to_string(i ) + "/scan", 1, boost::bind(&OrcaLocalPlanner::scan_callback, this, _1, i));
    for (int i = 0; i < 6; i++)
    {
        scan_sub_[i] = nh.subscribe<sensor_msgs::LaserScan>("/tb3_" + std::to_string(i) + "/scan", 1, boost::bind(&scan_callback, _1, i));
    }

	// 发布者--用于IDSI_Rank数据发布--230710-----------------------------------------------------------------------------
	for (int i = 0; i < 6; i++)
	{
		IDSI_Rank[i] = nh.advertise<std_msgs::Float32MultiArray>("/tb3_" + std::to_string(i) +"/idsi_topic", 10);		
	}


    //订阅者--订阅机器人位置信息？？
    for (int i = 0; i < 6; i++)
    {
        odom_sub_[i] = nh.subscribe<nav_msgs::Odometry>("/tb3_" + std::to_string(i) + "/odom", 4 * 4, boost::bind(&odom_callback, _1, i));
    }


	//时间段，在该段时间内,float
	//Time_Interval = _time_long;//赋初值---------------------------------------------------------是否应和频率统一？？    ros::Rate rate(2);

	//机器人的真实数量,int
	Num_Robot = 6;//赋初值---------------------------------------------------------

	// //是否需要？？？？？
	// //机器人序号：0-9,int
	// Robot_index[10] = {0,1,2,3,4,5,6,7,8,9};//赋初值---------------------------------------------------------

	//机器人数据数组：横--序号，0-9  竖--位置坐标x,y,速度Vx,Vy
	//横--序号，0-9
	for (int i = 0; i < 10; i++)
	{
		//竖--位置坐标x, y, 速度Vx, Vy
		for (int j = 0; j < 5; j++)
		{
			Robot_Data[i][j] = 0;//赋初值
		}
	}

	////机器人观测半径float
	//Robot_Range[10] = { 2.0f, 2.0f, 2.0f, 2.0f, 2.0f,    2.0f, 2.0f, 2.0f, 2.0f, 2.0f };//赋初值---------------------------------------------------------


	//机器人观测结果数组：横--机器人序号，0-9  竖--动态障碍物序号（障碍物也即为其他机器人）,0-9，true为真--观测的到,bool
	//横--序号，0-9
	for (int i = 0; i < 10; i++)
	{
		//竖--动态障碍物序号（障碍物也即为其他机器人）, 0-9
		for (int j = 0; j < 10; j++)
		{
			Observe_index[i][j] = false;//赋初值为false，最开始无法相互观测
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

	ROS_INFO("Visual_field Begin !!!!!!!!");//,%f,%f., begin_inf[0][0],begin_inf[0][1],begin_inf[1][0],begin_inf[1][1]

	//数据初始化--结束
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

//更新机器人数据函数--从ros发送数据中读取
void Robot_Data_Update()
{
	for (int i = 0; i< Num_Robot; i++)
	{
		//从ros发送数据中读取
		Robot_Data[i][0] = agent[i]->position_.x();
		Robot_Data[i][1] = agent[i]->position_.y();
		Robot_Data[i][2] = agent[i]->velocity_.x();
		Robot_Data[i][3] = agent[i]->velocity_.y();
		Robot_Data[i][4] = agent[i]->heading_;//是否正确？？？？后面不对，agent[i]->velocity_.x()
		//sin(agent[i]->heading_)是否正确？？？？-------------------------------------------------------应该有问题？速度方向和机器人朝向其实不同，应为朝向
		//右手定则，机器人前方向为x轴（食指），上方为z轴（大拇指），左方为y轴（中指），来换算agent[i]->heading_，heading方向计算为逆时针

		// ROS_INFO("Data!!!!!  Robot = %i, visual_data = [%f, %f, %f, %f ]", i, Robot_Data[i][0], Robot_Data[i][1], 
		// 		Robot_Data[i][2], Robot_Data[i][3]);//d代表整形，f代表浮点数
	}
}

//输出机器人观测函数--向ros发送最新IDSI数据
void Output_Observe_index()
{
	//对于第i个机器人
	for (int i = 0; i < Num_Robot; i++)
	{
		//对于第j个障碍物
		for (int j = 0; j < Num_Robot; j++)
		{
			if (i == j )
			{
				//机器人不需要对自己观测，自己不是障碍物
				Observe_index[i][j] = false;
			}
			else
			{
				//ROS_INFO("Data!!!!!  Robot = %i, j = %i,", i, j );

				//第i个机器人，对于第j个障碍物的观测状态
			    //float A_str, float R_Px, float R_Py, float R_Vx, float R_Vy, float O_Px, float O_Py
				// if( i==0 && j==1 )
				// {
				Observe_index[i][j] = IsIn_Range(RA_str, Robot_Data[i][0], Robot_Data[i][1], Robot_Data[i][4], Robot_Data[j][0], Robot_Data[j][1]);	
				//Robot_Data[i][4]方向角改为--编号为4		
				//}

			// //log临时控制器--230630
			// if( i==0 && j==2 )
			// {
			// 	__log_bool = true;
			// }

			// __log_bool = false;
			
			}

        	// if(i == 0)
			// {
			// 	// ROS_INFO("Data!!!!!  Robot = %i,RA_str =%f, visual_data = [%f, %f, %f, %f, %f, %f ]", i, RA_str, Robot_Data[i][0], Robot_Data[i][1], 
			// 	// Robot_Data[i][2], Robot_Data[i][3], Robot_Data[j][0], Robot_Data[j][1]  );
			// }

		}

        // if(i == 0)
		// {

		//用于IDSI_Rank数据发布--230710--------------------------------------------------------------待细细调整
		std_msgs::Float32MultiArray IDSI_msg;
		IDSI_msg.data.resize(Num_Robot);  // 设置数据大小为6 Num_Robot

		// 设置6个int数据
		for (int k = 0; k < Num_Robot; k++)
		{
    		//IDSI_msg.data[k] = Observe_index[i][k];
			IDSI_msg.data[k] = IDSI_T[i][k];
		}

		IDSI_Rank[i].publish(IDSI_msg);

		// ROS_INFO("Get!!!!!  Robot = %i, bool = [%d, %d, %d, %d, %d, %d, ]", i, Observe_index[i][0], Observe_index[i][1], Observe_index[i][2], 
		// 
		//暂时取消log，-----230718																				Observe_index[i][3], Observe_index[i][4], Observe_index[i][5]  );	
		// ROS_INFO("Get!!!!!  Robot = %i, IDSI_T = [%f, %f, %f, %f, %f, %f, ]", i, 
		// IDSI_T[i][0], IDSI_T[i][1], IDSI_T[i][2], IDSI_T[i][3], IDSI_T[i][4], IDSI_T[i][5] );				
		//}
	}
}

//判断动态障碍物是否在视野范围内 true--在范围内
//A_str观测半径，R_Px代表Robot的x坐标位置，O_Px代表Robot的x坐标位置，以此类推
bool IsIn_Range(float _A_str, float _R_Px, float _R_Py, float _head, float _O_Px, float _O_Py )//float _R_Vx, float _R_Vy, 
{

	// ROS_INFO("Input Data!!!!! _A_str = %f,  _R_Px = %f, _R_Py = %f,!!! __head = %f,  _O_Px =%f, _O_Py = %f ", _A_str, _R_Px, _R_Py, _head,  _O_Px, _O_Py );	
		

	vector<float> Vector_V(2);//机器人的前进向量--正前方
	vector<float> Vector_R(2);//障碍物O相对于机器人R的方向向量

	//此处是否要赋0 ???
	float _R_str = 0;//机器人在Vector_R方向，的最大观测距离
	float _w_str = 0;//Vector_V和Vector_R的夹角

	float _Dis_RO = 0;//障碍物O相对于机器人R的距离

	Vector_V[0] = cos(_head);
	Vector_V[1] = sin(_head);

	Vector_R[0] = _O_Px - _R_Px;
	Vector_R[1] = _O_Py - _R_Py;

	_Dis_RO = sqrt(Vector_R[0]* Vector_R[0]+ Vector_R[1]* Vector_R[1]);//障碍物O相对于机器人R的距离,先平方和再开方

	//Vector_V的方向角，Y方向值除以X方向的值，再用反三角函数actan
	float _VR_heading = atan2( Vector_R[1] , Vector_R[0] );

	// //XX_dis代表：Vector_PV和Vector_R的叉积，再除以两向量的模长
	// float _XX_dis = (Vector_V[0] * Vector_R[0] - Vector_V[1] * Vector_R[1]) / ( sqrt( pow(Vector_V[0], 2.0) + pow(Vector_V[1], 2.0) ) 
	// 				* sqrt( pow(Vector_R[0], 2.0) + pow(Vector_R[1], 2.0) ) )  ;

	//求两个heading方向角度的差
	//求Vector_R相对于正方向Vector_V的夹角
	_w_str = _VR_heading - _head;

	//机器人在Vector_R方向，的最大观测距离,//cos(_w_str)在0-3.14的弧度制下要取绝对值
	_R_str = _A_str * ( Anis_in + 0.5*( 1- Anis_in )*(1 + cos(_w_str) ) );

	//判断 Dis_RO <= R_str，障碍物O是否在机器人R观测范围内
	if ( _Dis_RO <= _R_str )
	{
		// if(	__log_bool == true)
		// {
		// 	ROS_INFO("IsIn_Range!!!!! _A_str = %f,  _XX_dis = %f, _w_str = %f,!!! _Dis_RO = %f, _R_str =%f, Range = 1 ", _A_str, _XX_dis, _w_str, _Dis_RO, _R_str );	
		// }
		//ROS_INFO("IsIn_Range!!!!! _A_str = %f,  _VR_heading = %f, _w_str = %f,!!! _Dis_RO = %f, _R_str =%f, Range = 1 ", _A_str, _VR_heading, _w_str, _Dis_RO, _R_str );	
		
		return true;
	}
	else
	{
		// if(	__log_bool == true)
		// {
		// 	ROS_INFO("IsIn_Range!!!!! _A_str = %f,  _XX_dis = %f, _w_str = %f,!!! _Dis_RO = %f, _R_str =%f, Range = 0 ", _A_str, _XX_dis, _w_str, _Dis_RO, _R_str);
		// }
		//ROS_INFO("IsIn_Range!!!!! _A_str = %f,  _VR_heading = %f, _w_str = %f,!!! _Dis_RO = %f, _R_str =%f, Range = 0 ", _A_str, _VR_heading, _w_str, _Dis_RO, _R_str);

		return false;
	}
	
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

///////////////////////////////////////////////////////////////////////////从此处开始-----IDSI模块程序------230704

//idsi相关接口的初始化
void idsi_init()
{
	//数据初始化--开始
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//真实雷达数据初始化--赋为0
	//对于第i个机器人
    for (int i = 0; i < 10; i++)//Num_Robot
	{
		//遍历激光雷达数据
		for (int j = 0; j < 40; j++)
		{
			//第i个机器人，第j个激光数据---激光雷达数据由机器人正前方为中轴，最左侧为0线，顺时针依次增大，根据实物雷达情况待定---230705
			//从ros发送数据中读取
			idsi_Lidar_Data[i][j] = 0;//ros
		}
	}

	//基础优先级赋初值------------------------------------------------暂定0.2，待调整
    for (int i = 0; i < 10; i++)//Num_Robot
	{
		Base_Pr[i] = _data_PrBase[i]; 
	}

	//实时优先级赋初值--------------------------------------------暂时全部统一和Base_Pr[10]一样？？
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		RealTime_Pr[i] = 0.2;
	}

	//机器人0-9 实时优先级
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			Interact_Pr[i][j] = 0.2;//实时优先级赋初值---------------------------------------------------------赋值为0？？还是和基础优先级一样？？
		}
	}


	//机器人safety potential energy--安全势能-数组,float
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			SPE_V[i][j] = 0;//---------------------------------------------------------赋初始值为0f??？ 是否可行？？？
		}
	}


	//机器人--交互场功率-数组：横--机器人序号，0-9,float
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			Power_I[i][j] = 0;//---------------------------------------------------------赋初始值为0f??？ 是否可行？？？
		}
	}


	//机器人interactive driving safety index--交互驾驶安全指数-数组：横--机器人序号，0-9,float
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			IDSI_T[i][j] = 0;//---------------------------------------------------------赋初始值为0f??？ 是否可行？？？
		}
	}

	//机器人观测范围障碍物数量，根据Observe_index[10][10]检索而来
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		Observe_Num[i] = 0;//---------------------------------------------------------赋初始值为0??？ 是否可行？？？是否有用？？		
	}


	//IDSI顺序数组----根据IDSI对障碍物威胁程度进行排序
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			Rank_IDSI_Index[i][j] = -1;//---------------------------------------------------------赋初始值为 -1 ??？ 是否可行？？？
		}
	}


	//数据初始化--结束
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
}


//更新机器人优先级--从ros发送数据中读取
void idsi_Lidar_Update()
{
	//对于第i个机器人
	for (int i = 0; i < Num_Robot; i++)
	{
		//遍历激光雷达数据
		//此处暂时按照前进方向为0线开始，顺时针排列，可能需要根据实物机器人的激光雷达标定，调整排序--待定-----230705
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		for (int j = 0 + 0; j < 40 + 0; j++)
		{
			if(j < Num_Lidar)//待调整
			{
				//第i个机器人，第j个激光数据
				//从ros发送数据中读取
				idsi_Lidar_Data[i][j] = lidar_legth[i][j];//ros;
			}
		}
	}
}



//计算机器人IDSI数据
void IDSI_Compute()
{
	//更新机器人实时优先级
	RealTime_Pr_Update();

	//更新机器人交互优先级
	Interact_Pr_Update();

	//逐个计算机器人IDSI----interactive driving safety index--交互驾驶安全指数--
	//对于第i个机器人
	for (int i = 0; i < Num_Robot; i++)
	{
		//对于第j个障碍物
		for (int j = 0; j < Num_Robot; j++)
		{
			//如果第i个机器人，能够观测到第j个障碍物
			if (Observe_index[i][j] == true)
			{
				//机器人interactive driving safety index--交互驾驶安全指数--赋值
				//int Index_R, int Index_O, float _R_RTPr, float _O_RTPr, float _R_Px, float _R_Py, float _R_Vx, float _R_Vy, float _O_Px, float _O_Py, float _O_Vx, float _O_Vy
				//暂时注销，后面恢复-------------------------------23.0706
				IDSI_T[i][j] = IDSI_Update(i, j, Interact_Pr[i][j], Interact_Pr[j][i], Robot_Data[i][0], Robot_Data[i][1], Robot_Data[i][2], Robot_Data[i][3], Robot_Data[j][0], Robot_Data[j][1], Robot_Data[j][2], Robot_Data[j][3]);
			
			}
			else
			{
				//暂时注销，后面恢复-------------------------------23.0706
				IDSI_T[i][j] = 0;
			}

		}
	}

	//IDSI排序----根据IDSI对障碍物威胁程度进行排序
	//暂时注销，后面恢复---------------------------目前可以不用排序？？----23.0709
	//IDSI_Sort();	
}



//更新机器人实时优先级
void RealTime_Pr_Update()
{
	//对于第i个机器人
	for (int i = 0; i < Num_Robot; i++)
	{
		//不安全线数量
		int UnsafeLidar_Num = 0;

		//遍历激光雷达数据--判断危险线数
		for (int j = 0; j < Num_Lidar; j++)
		{
			//如果j<5,半边循环
			if(j<5)
			{

			}
			//如果j>34,半边循环
			if(j>34)
			{
				//激光雷达数据小于安全数据--判断
				if (idsi_Lidar_Data[i][j] < SafeyLidar_Data[j-30])
				{
					UnsafeLidar_Num++;
				}
			}
		}


		//根据不安全线数量，计算优先级Pr，论文公式4
		RealTime_Pr[i] =  ( Base_Pr[i] - MinPr_c ) * ( 1 - (float)UnsafeLidar_Num / (float)Num_Lidar ) + MinPr_c;//注意整数除法，只保留整数，数据类型需要先转换---230706
		//RealTime_Pr[i] = ( Base_Pr[i] - MinPr_c ) * ( 1 - (float)UnsafeLidar_Num / (float)Num_Lidar ) + MinPr_c ;//
		//RealTime_Pr[i] = 3.14;

		//暂时取消log，-----230718
		// ROS_INFO("RealTime_Pr_Update() is running !!!!!, Base_Pr[i] = %f, MinPr_c = %f, Num_Lidar = %i, UnsafeLidar_Num  = %i, RealTime_Pr[i] = %f", 
		// Base_Pr[i], MinPr_c, Num_Lidar, UnsafeLidar_Num, RealTime_Pr[i] );
	}

}

//更新机器人交互优先级
void Interact_Pr_Update()
{
	//对于第i个机器人
	for (int i = 0; i < Num_Robot; i++)
	{
		//对于第j个障碍物
		for (int j = 0; j < Num_Robot; j++)
		{
			//如果第i个机器人，能够观测到第j个障碍物
			if (Observe_index[i][j] == true)
			{
				//如果第j个障碍物，能够反向观测到第i个机器人
				if (Observe_index[j][i] == true)
				{
					//可互相观测，实时优先级=优先级RealTime_Pr
					Interact_Pr[i][j] = RealTime_Pr[i];
				}
				else
				{
					//单向观测，实时优先级=优先级RealTime_Pr
					Interact_Pr[i][j] = MinPr_e * Base_Pr[i];
					//ROS_INFO("One Shot- Observation!!!!!!!!!!!!!!!!!!!!!!!!!" );
				}
			}
			else
			{
				//如果第j个障碍物，可以反向观测到第i个机器人
				if (Observe_index[j][i] == true)
				{
					//i对j不可观测，但j可以观测到i ---i保持自己的，实时优先级=优先级RealTime_Pr ？？？
					Interact_Pr[i][j] = RealTime_Pr[i];
				}
				else
				{
					//双向无法观测，应当怎么样处理？？？？---设定为，实时优先级=优先级RealTime_Pr ？？？实事求是地处理，保障避障安全？？？
					Interact_Pr[i][j] = RealTime_Pr[i];
				}
			}

			if(	Observe_index[i][j] == true && Observe_index[j][i] == false )
			{
				    ROS_INFO("One Shot- Observation--------------------------- Robot-i = %i, Robot-j = %i, ", i, j );
			}
		}
	}

}





//更新机器人观测范围障碍物数量，根据Observe_index[10][10]检索而来
void ObserveNum_Update()
{
	////机器人观测范围障碍物数量，根据Observe_index[10][10]检索而来
	////先清零上一组数据
	//Observe_Num = { 0,0,0,0,0,  0,0,0,0,0 };

	//对于第i个机器人
	for (int i = 0; i < Num_Robot; i++)
	{
		//观测数量计数
		int _Obs_num = 0;

		//对于第j个障碍物
		for (int j = 0; j < Num_Robot; j++)
		{
			//如果可观测
			if (Observe_index[i][j] == true )
			{
				_Obs_num++;
			}
		}

		//赋值给观测数量
		Observe_Num[i] = _Obs_num;
	}
}


//更新机器人----interactive driving safety index--交互驾驶安全指数
//Index_R表示机器人的编号（代表i），R_IAPr表示机器人的交互优先级（代表j）
//R_Px代表Robot的x坐标位置，O_Px代表Robot的x坐标位置，以此类推
float IDSI_Update(int Index_R, int Index_O, float _R_IAPr, float _O_IAPr, float _R_Px, float _R_Py, float _R_Vx, float _R_Vy,  float _O_Px, float _O_Py, float _O_Vx, float _O_Vy)
{

	float RL_ji = 0;

	//相对位置半径R_ji的模长
	RL_ji = sqrt( pow( _R_Px - _O_Px, 2.0) + pow(_R_Py - _O_Py, 2.0) );//障碍物j相对于机器人i的距离,先平方和再开方


	float VL_i = 0;

	//R机器人i速度VL_i的模长
	VL_i = sqrt(pow(_R_Vx, 2.0) + pow(_R_Vy, 2.0));//先平方和再开方



	float VL_j = 0;

	//O障碍物j速度VL_j的模长
	VL_j = sqrt(pow(_O_Vx, 2.0f) + pow(_O_Vy, 2.0));//先平方和再开方

	float O_Qj = 0;

	//O_Qj,表示O障碍物j的速度方向V_j和相对位置半径R_ji的夹角，论文公式8
	//O_Qj = [ (_R_Px - _O_Px)*_O_Vx + (_R_Py - _O_Py)*_O_Vy ]/( RL_ji*VL_j );
	O_Qj = acos( ( (_R_Px - _O_Px) * _O_Vx + (_R_Py - _O_Py) * _O_Vy )/( RL_ji * VL_j ) );



	float SPE_Vji_aa = 0;

	//动能场的SPE的后半部分，为了简化计算，论文公式8
	SPE_Vji_aa = pow( SPE_k3 - VL_j * cos(O_Qj), 1.0 - SPE_k1 ) / (SPE_k3 - VL_j);

	float SPE_Vji = 0;

	//动能场的SPE，论文公式8
	//SPE_Vji = (SPE_K* SPE_R*SPE_R * _R_IAPr*_O_IAPr * SPE_k3) * pow(SPE_Vji_aa, 1.0 / SPE_k1) / [ (SPE_k1 - 1.0)*pow(RL_ji, SPE_k1 - 1.0) ];
	SPE_Vji = (SPE_K* SPE_R* SPE_R * _R_IAPr* _O_IAPr * SPE_k3) * pow(SPE_Vji_aa, 1.0 / SPE_k1) / ( (SPE_k1 - 1.0) * pow( RL_ji, SPE_k1 - 1.0 ) );

	//传参到，机器人safety potential energy--安全势能-数组
	//j对i的动能SPE
	SPE_V[Index_R][Index_O] = SPE_Vji;

	//暂时取消log，-----230718
	// ROS_INFO("IDSI_Update() is running-------, Robot-i =%i, Robot-j =%i, O_Qj = %f, VL_jr = %f, RL_ji = %f, SPE_Vji_aa = %f, SPE_Vji = %f", 
	// 	Index_R, Index_O, O_Qj, VL_j, RL_ji, SPE_Vji_aa, SPE_Vji );

	float P_Iji_aa = 0;

	//机器人--交互场功率P_I的后半部分，为了简化计算，论文公式13
	//Rji和（Vj-Vi）的点乘
	P_Iji_aa = (_R_Px - _O_Px) * (_O_Vx - _R_Vx) + (_R_Py - _O_Py) * (_O_Vy - _R_Vy);

	float P_Iji = 0;

	//机器人--交互场功率P_I，论文公式13
	P_Iji = (SPE_K* SPE_R*SPE_R * _R_IAPr*_O_IAPr) * P_Iji_aa / pow(RL_ji, SPE_k4 + 1.0);

	//传参到，机器人--交互场功率-数组
	Power_I[Index_R][Index_O] = P_Iji;

	

	float IDSI_ji = 0;

	//机器人interactive driving safety index--交互驾驶安全指数,，论文公式10
	//IDSI_ji = -1.0 * SPE_a * SPE_Vji + (1.0 - SPE_a) * P_Iji;
	IDSI_ji = 100.0 * ( 1.0 * SPE_a * SPE_Vji + (1.0 - SPE_a) * P_Iji );

	//检测排除nan和inf数据，防止DRL无法训练
	if(std::isnan(IDSI_ji))
	{
		IDSI_ji = 0.1;
	}
	else if (std::isinf(IDSI_ji)) 
	{
        if (IDSI_ji > 0) {
            IDSI_ji = 40.0;
        } else {
            IDSI_ji = -40.0;
        }
	}

	//输出
	return IDSI_ji;
}












// 打印IDSI--接收到的数组
void Print_IDSI()
{
//     // 打印接收到的数组--实时优先级数据--Base_Pr[i]
//     ROS_INFO("Base_Pr[i]  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f,] ", 
//     Base_Pr[0], Base_Pr[1], Base_Pr[2], Base_Pr[3], Base_Pr[4], 
//     Base_Pr[5], Base_Pr[6], Base_Pr[7], Base_Pr[8], Base_Pr[9] );

//     // 打印接收到的数组--实时优先级数据--RealTime_Pr[i]
//     ROS_INFO("RealTime_Pr[i]  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f,] ", 
//     RealTime_Pr[0], RealTime_Pr[1], RealTime_Pr[2], RealTime_Pr[3], RealTime_Pr[4], 
//     RealTime_Pr[5], RealTime_Pr[6], RealTime_Pr[7], RealTime_Pr[8], RealTime_Pr[9] );

//   for (int i = 0; i < 6; ++i)
//   {
  
//     // // 打印发送的数组
//     ROS_INFO("Robot = %i, Observe_index[i][k] = [%i, %i, %i, %i, %i,  %i, %i, %i, %i, %i,] ", 
//     i, Observe_index[i][0], Observe_index[i][1], Observe_index[i][2], Observe_index[i][3], Observe_index[i][4], 
//      Observe_index[i][5], Observe_index[i][6], Observe_index[i][7], Observe_index[i][8], Observe_index[i][9] );	
//   }

//   for (int i = 0; i < 6; ++i)
//   {
// 	//可互相观测，实时优先级=优先级RealTime_Pr
//     ROS_INFO("Robot = %i!!!!!, Interact_Pr[i][j]  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f, ] ", 
//     i, Interact_Pr[i][0], Interact_Pr[i][1], Interact_Pr[i][2], Interact_Pr[i][3], Interact_Pr[i][4], 
// 	 Interact_Pr[i][5], Interact_Pr[i][6], Interact_Pr[i][7], Interact_Pr[i][8], Interact_Pr[i][9]);		
//   }

//   for (int i = 0; i < 6; ++i)
//   {
// 	//SPE_V[i][j]数据
//     ROS_INFO("Robot = %i!!!!!, SPE_V[i][j]  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f, ] ", 
//     i, SPE_V[i][0], SPE_V[i][1], SPE_V[i][2], SPE_V[i][3], SPE_V[i][4], 
// 	 SPE_V[i][5], SPE_V[i][6], SPE_V[i][7], SPE_V[i][8], SPE_V[i][9]);	
//   }

//     for (int i = 0; i < 6; ++i)
//   {
// 	//Power_I[i][j]交互数据
//     ROS_INFO("Robot = %i!!!!!, Power_I[i][j]  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f, ] ", 
//     i, Power_I[i][0], Power_I[i][1], Power_I[i][2], Power_I[i][3], Power_I[i][4], 
// 	 Power_I[i][5], Power_I[i][6], Power_I[i][7], Power_I[i][8], Power_I[i][9]);	
//   }

  for (int i = 0; i < 6; ++i)
  {
		//IDSI_T[i][j]交互数据
    ROS_INFO("Robot = %i!!!!!, IDSI_T[i][j]  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f, ] ", 
    i, IDSI_T[i][0], IDSI_T[i][1], IDSI_T[i][2], IDSI_T[i][3], IDSI_T[i][4], 
	 IDSI_T[i][5], IDSI_T[i][6], IDSI_T[i][7], IDSI_T[i][8], IDSI_T[i][9]);	
  }


//   for (int i = 0; i < 6; ++i)
//   {
//     // 打印接收到的数组--真实激光雷达
//     ROS_INFO("Received Robot = %i, Real-Lidardata  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f, ] ", 
//     i, lidar_legth[i][0], lidar_legth[i][1], lidar_legth[i][2], lidar_legth[i][3], lidar_legth[i][4], 
// 	 lidar_legth[i][5], lidar_legth[i][6], lidar_legth[i][7], lidar_legth[i][8], lidar_legth[i][9]);	 


// 	// 打印接收到的数组--IDSI雷达数据
//     ROS_INFO("Received Robot = %i, IDSI-Lidardata  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f,] ", 
//     i, idsi_Lidar_Data[i][0], idsi_Lidar_Data[i][1], idsi_Lidar_Data[i][2], idsi_Lidar_Data[i][3], idsi_Lidar_Data[i][4], 
//      idsi_Lidar_Data[i][5], idsi_Lidar_Data[i][6], idsi_Lidar_Data[i][7], idsi_Lidar_Data[i][8], idsi_Lidar_Data[i][9]);

//   } 
}
///////////////////////////////////////////////////////////////////////////从此处结束----IDSI模块程序------230704