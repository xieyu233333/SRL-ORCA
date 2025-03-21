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
//	Note:   Social Visual_Field ������۲���Ұ����
//-----------------------------------------------------------------------------

//interact.cpp����--23.05.13
#include <angles/angles.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <collvoid_srvs/GetObstacles.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include "collvoid_local_planner/common.h"
#include "collvoid_local_planner/visual_field.h"//��Ҫ���ݳ���h�ļ�����--230623
#include "collvoid_local_planner/Vector2.h"
#include <collvoid_local_planner/Agent.h>
#include <collvoid_local_planner/GetCollvoidTwist.h>
#include<gazebo_msgs/ModelState.h>
#include <tf/tf.h>

#include <std_msgs/Int32MultiArray.h>//����IDSI_Rank���ݷ���--230702
#include <std_msgs/Float32MultiArray.h>//����IDSI_Rank���ݷ���--230710
using namespace collvoid;

//ԭ��--���ã�����
// #include <iostream>
#include <vector>
// #include <cmath>
// #include "visual_field.h"
// using namespace std;
using std::vector;

//�����ģ�͹۲�--��������ϵ�� anisotropic intensity
float Anis_in = 0.4;

////�����˹۲�뾶--������
float RA_str = 3.0;/////��Ҫ����ʵ�ʳ�������------------------------------------------------------------����


//ʱ��Σ��۲�Ƶ��--������
float Time_Interval = 2.0;//_time_long��2

//������
//��ʱָ��--����λ�ûص�����odom_callback��ʱ
int _Delay_index[6] = { 50,50,50,50,50, 50 };

// //log��ʱ���--230630
// int __log_bool = false;


///////////////////////////////////////////////////////////////////////////�Ӵ˿�ʼ----IDSIģ�鳬����------230704

//������
//��ȫ�������״����ݣ���--��ţ�0-9  ��--һ���Ƕȷ�Χ�ڣ��Ļ����˼����״����ݣ��������л����˵�ȫ��ͳһ
//S_AHV���״�̽�ⳤ��-----------------------------------------------------------�ݶ�1.0��������
//��ʵ������ˣ��״�ֲ�����ǰ������Ϊ0�ߣ���ʱ����ת�ֲ�����0-5�ߣ�34-39��ΪΣ��
float SafeyLidar_Data[10] = { 0.45, 0.41, 0.35, 0.32, 0.25,   0.25, 0.32, 0.35, 0.41, 0.45 };

//��СPrֵ
float MinPr_c = 0.05;

//����۲⣬Interact_Prֵ��С����
float MinPr_e = 0.1;

//IDSI��س�����
//���²���--�����廪��ѧ����ǿ������
float SPE_K = 0.5;

//�ϰ���ͻ���������ͬ��SPE_R
float SPE_R = 1.0;

float SPE_k1 = 1.5;//����Ϊ1.0�������ĸΪ0

float SPE_k3 = 16.0;//45.0,160̫�󣿣������˺������ٶȲ���̫�󣬸�Ϊ16����

float SPE_a = 0.1;//Ӣ��������0.06�Ƿ��С��������˶ʿ����0.1

//��Ҫ�Լ��趨
float SPE_k4 = 1.2;//�ݶ�1.5-2.0֮��Ϻã������ݹ�ʽ��������ǿ�������� k2 = 1.2,230710ʵ�飬2.0ƫ�󣬵�С��0.2����

//������---IDSIʹ�õļ����״���������ݶ�10
int Num_Lidar = 10;

////������---�������ȼ��ĳ�ʼֵ
float _data_PrBase[10] = { 1.00, 1.01, 1.02, 1.03, 1.04,   1.05, 1.06, 1.07, 1.08, 1.09 };


int main(int argc, char **argv) {
    ros::init(argc, argv, "visual_field");///////////////////////////////////////////////////////////////////////////////////////�Ƿ���ȷ��
    //��������ֵ��ʼ��
    init();

	//idsi��ؽӿڵĳ�ʼ��
	idsi_init();

	//���ø���Ƶ�ʣ����ȣ����Ƿ�����Ϊ=2
    ros::Rate rate(Time_Interval);///////////////////////////////////////////////////////////////////////////////////////�Ƿ���ȷ��--����

    while (ros::ok()) {
        //local_compute();

		Robot_Data_Update();

		//���»����˲����״�۲�����--����Ҫ����
		idsi_Lidar_Update();

		//���������IDSI����------------------------------�����������У�����������������������������������������
		IDSI_Compute();

		//��������˹۲⺯��--��ros��������IDSI����
		Output_Observe_index();

		Print_IDSI();//��ӡ��������------------------������

        ros::spinOnce();
        rate.sleep();
    }



}


//��������ֵ��ʼ��
void init()
{
		//���ݳ�ʼ��--��ʼ
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ros::NodeHandle nh("~");

    for (int i = 0; i < 6; i++) {
        agent.push_back(AgentPtr(new Agent));
    }

    //������ʼ��
    //����дһ�������ߣ�--���������˵����ٶȺͽ��ٶ�---------------------------------��Ҫ�ʼ��Σ���ô����һ��bool���ݣ�����
    //��Ϣ����Ϊgeometry_msgs::Point
    //cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/cmd_vel", 4 * 4);
    for (int i = 0; i < 6; i++)
    {
        //sac_cmd_vel_sub_[i] = nh.subscribe<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/sac_cmd_vel", 4 * 4, boost::bind(&OrcaLocalPlanner::sac_cmd_vel_callback, this, _1, i));
        //�˴���Ϊ/cmd_vel-ֱ�ӷ������ݣ���/sac_cmd_vel--����orca����������
        sac_cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/sac_cmd_vel", 4 * 4);
    }

    // //������ʼ��
    // //����дһ�������ߣ�--��������Ŀ��λ��
    // //��Ϣ����Ϊgazebo_msgs::ModelState
    // //cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/cmd_vel", 4 * 4);
    // for (int i = 0; i < 6; i++)
    // {
    //     //sac_cmd_vel_sub_[i] = nh.subscribe<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/sac_cmd_vel", 4 * 4, boost::bind(&OrcaLocalPlanner::sac_cmd_vel_callback, this, _1, i));
    //     set_model_pub_[i] = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 4 * 4);
    // }

    //���ļ����״����ݣ���
    //scan_sub_[i] = nh.subscribe<sensor_msgs::LaserScan>("/tb3_" + std::to_string(i ) + "/scan", 1, boost::bind(&OrcaLocalPlanner::scan_callback, this, _1, i));
    for (int i = 0; i < 6; i++)
    {
        scan_sub_[i] = nh.subscribe<sensor_msgs::LaserScan>("/tb3_" + std::to_string(i) + "/scan", 1, boost::bind(&scan_callback, _1, i));
    }

	// ������--����IDSI_Rank���ݷ���--230710-----------------------------------------------------------------------------
	for (int i = 0; i < 6; i++)
	{
		IDSI_Rank[i] = nh.advertise<std_msgs::Float32MultiArray>("/tb3_" + std::to_string(i) +"/idsi_topic", 10);		
	}


    //������--���Ļ�����λ����Ϣ����
    for (int i = 0; i < 6; i++)
    {
        odom_sub_[i] = nh.subscribe<nav_msgs::Odometry>("/tb3_" + std::to_string(i) + "/odom", 4 * 4, boost::bind(&odom_callback, _1, i));
    }


	//ʱ��Σ��ڸö�ʱ����,float
	//Time_Interval = _time_long;//����ֵ---------------------------------------------------------�Ƿ�Ӧ��Ƶ��ͳһ����    ros::Rate rate(2);

	//�����˵���ʵ����,int
	Num_Robot = 6;//����ֵ---------------------------------------------------------

	// //�Ƿ���Ҫ����������
	// //��������ţ�0-9,int
	// Robot_index[10] = {0,1,2,3,4,5,6,7,8,9};//����ֵ---------------------------------------------------------

	//�������������飺��--��ţ�0-9  ��--λ������x,y,�ٶ�Vx,Vy
	//��--��ţ�0-9
	for (int i = 0; i < 10; i++)
	{
		//��--λ������x, y, �ٶ�Vx, Vy
		for (int j = 0; j < 5; j++)
		{
			Robot_Data[i][j] = 0;//����ֵ
		}
	}

	////�����˹۲�뾶float
	//Robot_Range[10] = { 2.0f, 2.0f, 2.0f, 2.0f, 2.0f,    2.0f, 2.0f, 2.0f, 2.0f, 2.0f };//����ֵ---------------------------------------------------------


	//�����˹۲������飺��--��������ţ�0-9  ��--��̬�ϰ�����ţ��ϰ���Ҳ��Ϊ���������ˣ�,0-9��trueΪ��--�۲�ĵ�,bool
	//��--��ţ�0-9
	for (int i = 0; i < 10; i++)
	{
		//��--��̬�ϰ�����ţ��ϰ���Ҳ��Ϊ���������ˣ�, 0-9
		for (int j = 0; j < 10; j++)
		{
			Observe_index[i][j] = false;//����ֵΪfalse���ʼ�޷��໥�۲�
		}
	}

    //��ʼ��--����ֵ��float
    //�����˼����״�̽�ⳤ�����ݣ�36��ֵ������ǰ��˳ʱ�뿪ʼ������
    //public float lidar_legth[36];
    //��Ϊ40��ֵ�������״��������
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 40; j++)
        {
            lidar_legth[i][j] = 0;                
        }
    }

	ROS_INFO("Visual_field Begin !!!!!!!!");//,%f,%f., begin_inf[0][0],begin_inf[0][1],begin_inf[1][0],begin_inf[1][1]

	//���ݳ�ʼ��--����
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

//���»��������ݺ���--��ros���������ж�ȡ
void Robot_Data_Update()
{
	for (int i = 0; i< Num_Robot; i++)
	{
		//��ros���������ж�ȡ
		Robot_Data[i][0] = agent[i]->position_.x();
		Robot_Data[i][1] = agent[i]->position_.y();
		Robot_Data[i][2] = agent[i]->velocity_.x();
		Robot_Data[i][3] = agent[i]->velocity_.y();
		Robot_Data[i][4] = agent[i]->heading_;//�Ƿ���ȷ�����������治�ԣ�agent[i]->velocity_.x()
		//sin(agent[i]->heading_)�Ƿ���ȷ��������-------------------------------------------------------Ӧ�������⣿�ٶȷ���ͻ����˳�����ʵ��ͬ��ӦΪ����
		//���ֶ��򣬻�����ǰ����Ϊx�ᣨʳָ�����Ϸ�Ϊz�ᣨ��Ĵָ������Ϊy�ᣨ��ָ����������agent[i]->heading_��heading�������Ϊ��ʱ��

		// ROS_INFO("Data!!!!!  Robot = %i, visual_data = [%f, %f, %f, %f ]", i, Robot_Data[i][0], Robot_Data[i][1], 
		// 		Robot_Data[i][2], Robot_Data[i][3]);//d�������Σ�f��������
	}
}

//��������˹۲⺯��--��ros��������IDSI����
void Output_Observe_index()
{
	//���ڵ�i��������
	for (int i = 0; i < Num_Robot; i++)
	{
		//���ڵ�j���ϰ���
		for (int j = 0; j < Num_Robot; j++)
		{
			if (i == j )
			{
				//�����˲���Ҫ���Լ��۲⣬�Լ������ϰ���
				Observe_index[i][j] = false;
			}
			else
			{
				//ROS_INFO("Data!!!!!  Robot = %i, j = %i,", i, j );

				//��i�������ˣ����ڵ�j���ϰ���Ĺ۲�״̬
			    //float A_str, float R_Px, float R_Py, float R_Vx, float R_Vy, float O_Px, float O_Py
				// if( i==0 && j==1 )
				// {
				Observe_index[i][j] = IsIn_Range(RA_str, Robot_Data[i][0], Robot_Data[i][1], Robot_Data[i][4], Robot_Data[j][0], Robot_Data[j][1]);	
				//Robot_Data[i][4]����Ǹ�Ϊ--���Ϊ4		
				//}

			// //log��ʱ������--230630
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

		//����IDSI_Rank���ݷ���--230710--------------------------------------------------------------��ϸϸ����
		std_msgs::Float32MultiArray IDSI_msg;
		IDSI_msg.data.resize(Num_Robot);  // �������ݴ�СΪ6 Num_Robot

		// ����6��int����
		for (int k = 0; k < Num_Robot; k++)
		{
    		//IDSI_msg.data[k] = Observe_index[i][k];
			IDSI_msg.data[k] = IDSI_T[i][k];
		}

		IDSI_Rank[i].publish(IDSI_msg);

		// ROS_INFO("Get!!!!!  Robot = %i, bool = [%d, %d, %d, %d, %d, %d, ]", i, Observe_index[i][0], Observe_index[i][1], Observe_index[i][2], 
		// 
		//��ʱȡ��log��-----230718																				Observe_index[i][3], Observe_index[i][4], Observe_index[i][5]  );	
		// ROS_INFO("Get!!!!!  Robot = %i, IDSI_T = [%f, %f, %f, %f, %f, %f, ]", i, 
		// IDSI_T[i][0], IDSI_T[i][1], IDSI_T[i][2], IDSI_T[i][3], IDSI_T[i][4], IDSI_T[i][5] );				
		//}
	}
}

//�ж϶�̬�ϰ����Ƿ�����Ұ��Χ�� true--�ڷ�Χ��
//A_str�۲�뾶��R_Px����Robot��x����λ�ã�O_Px����Robot��x����λ�ã��Դ�����
bool IsIn_Range(float _A_str, float _R_Px, float _R_Py, float _head, float _O_Px, float _O_Py )//float _R_Vx, float _R_Vy, 
{

	// ROS_INFO("Input Data!!!!! _A_str = %f,  _R_Px = %f, _R_Py = %f,!!! __head = %f,  _O_Px =%f, _O_Py = %f ", _A_str, _R_Px, _R_Py, _head,  _O_Px, _O_Py );	
		

	vector<float> Vector_V(2);//�����˵�ǰ������--��ǰ��
	vector<float> Vector_R(2);//�ϰ���O����ڻ�����R�ķ�������

	//�˴��Ƿ�Ҫ��0 ???
	float _R_str = 0;//��������Vector_R���򣬵����۲����
	float _w_str = 0;//Vector_V��Vector_R�ļн�

	float _Dis_RO = 0;//�ϰ���O����ڻ�����R�ľ���

	Vector_V[0] = cos(_head);
	Vector_V[1] = sin(_head);

	Vector_R[0] = _O_Px - _R_Px;
	Vector_R[1] = _O_Py - _R_Py;

	_Dis_RO = sqrt(Vector_R[0]* Vector_R[0]+ Vector_R[1]* Vector_R[1]);//�ϰ���O����ڻ�����R�ľ���,��ƽ�����ٿ���

	//Vector_V�ķ���ǣ�Y����ֵ����X�����ֵ�����÷����Ǻ���actan
	float _VR_heading = atan2( Vector_R[1] , Vector_R[0] );

	// //XX_dis����Vector_PV��Vector_R�Ĳ�����ٳ�����������ģ��
	// float _XX_dis = (Vector_V[0] * Vector_R[0] - Vector_V[1] * Vector_R[1]) / ( sqrt( pow(Vector_V[0], 2.0) + pow(Vector_V[1], 2.0) ) 
	// 				* sqrt( pow(Vector_R[0], 2.0) + pow(Vector_R[1], 2.0) ) )  ;

	//������heading����ǶȵĲ�
	//��Vector_R�����������Vector_V�ļн�
	_w_str = _VR_heading - _head;

	//��������Vector_R���򣬵����۲����,//cos(_w_str)��0-3.14�Ļ�������Ҫȡ����ֵ
	_R_str = _A_str * ( Anis_in + 0.5*( 1- Anis_in )*(1 + cos(_w_str) ) );

	//�ж� Dis_RO <= R_str���ϰ���O�Ƿ��ڻ�����R�۲ⷶΧ��
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

//������--������λ��--�ص�����
void odom_callback(const nav_msgs::OdometryConstPtr odom, int n) {
    // ROS_INFO("odom:%d", odom.get()->header.stamp.nsec);
    agent[n]->position_ = collvoid::Vector2(odom.get()->pose.pose.position.x, odom.get()->pose.pose.position.y);
    agent[n]->velocity_ = rotateVectorByAngle(odom.get()->twist.twist.linear.x,
                                                odom.get()->twist.twist.linear.y, (agent[n]->heading_));
    agent[n]->heading_ = tf::getYaw(odom.get()->pose.pose.orientation);
    // ROS_INFO("%d, velocity_:%6.2f,%6.2f", n, agent[n]->velocity_.x(), agent[n]->velocity_.y());
    //agent[n]->footprint_ = rotateFootprint(minkowski_footprint_, agent[n]->heading_);

    //�ص�������ʱָ��--��1
    _Delay_index[n]--;
}

//������--�����״�--�ص�����
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

        //�������״︳ֵ
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
    //����
    //obs_pub_[n].publish(obs_marker);
}

///////////////////////////////////////////////////////////////////////////�Ӵ˴���ʼ-----IDSIģ�����------230704

//idsi��ؽӿڵĳ�ʼ��
void idsi_init()
{
	//���ݳ�ʼ��--��ʼ
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//��ʵ�״����ݳ�ʼ��--��Ϊ0
	//���ڵ�i��������
    for (int i = 0; i < 10; i++)//Num_Robot
	{
		//���������״�����
		for (int j = 0; j < 40; j++)
		{
			//��i�������ˣ���j����������---�����״������ɻ�������ǰ��Ϊ���ᣬ�����Ϊ0�ߣ�˳ʱ���������󣬸���ʵ���״��������---230705
			//��ros���������ж�ȡ
			idsi_Lidar_Data[i][j] = 0;//ros
		}
	}

	//�������ȼ�����ֵ------------------------------------------------�ݶ�0.2��������
    for (int i = 0; i < 10; i++)//Num_Robot
	{
		Base_Pr[i] = _data_PrBase[i]; 
	}

	//ʵʱ���ȼ�����ֵ--------------------------------------------��ʱȫ��ͳһ��Base_Pr[10]һ������
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		RealTime_Pr[i] = 0.2;
	}

	//������0-9 ʵʱ���ȼ�
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			Interact_Pr[i][j] = 0.2;//ʵʱ���ȼ�����ֵ---------------------------------------------------------��ֵΪ0�������Ǻͻ������ȼ�һ������
		}
	}


	//������safety potential energy--��ȫ����-����,float
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			SPE_V[i][j] = 0;//---------------------------------------------------------����ʼֵΪ0f??�� �Ƿ���У�����
		}
	}


	//������--����������-���飺��--��������ţ�0-9,float
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			Power_I[i][j] = 0;//---------------------------------------------------------����ʼֵΪ0f??�� �Ƿ���У�����
		}
	}


	//������interactive driving safety index--������ʻ��ȫָ��-���飺��--��������ţ�0-9,float
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			IDSI_T[i][j] = 0;//---------------------------------------------------------����ʼֵΪ0f??�� �Ƿ���У�����
		}
	}

	//�����˹۲ⷶΧ�ϰ�������������Observe_index[10][10]��������
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		Observe_Num[i] = 0;//---------------------------------------------------------����ʼֵΪ0??�� �Ƿ���У������Ƿ����ã���		
	}


	//IDSI˳������----����IDSI���ϰ�����в�̶Ƚ�������
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			Rank_IDSI_Index[i][j] = -1;//---------------------------------------------------------����ʼֵΪ -1 ??�� �Ƿ���У�����
		}
	}


	//���ݳ�ʼ��--����
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
}


//���»��������ȼ�--��ros���������ж�ȡ
void idsi_Lidar_Update()
{
	//���ڵ�i��������
	for (int i = 0; i < Num_Robot; i++)
	{
		//���������״�����
		//�˴���ʱ����ǰ������Ϊ0�߿�ʼ��˳ʱ�����У�������Ҫ����ʵ������˵ļ����״�궨����������--����-----230705
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		for (int j = 0 + 0; j < 40 + 0; j++)
		{
			if(j < Num_Lidar)//������
			{
				//��i�������ˣ���j����������
				//��ros���������ж�ȡ
				idsi_Lidar_Data[i][j] = lidar_legth[i][j];//ros;
			}
		}
	}
}



//���������IDSI����
void IDSI_Compute()
{
	//���»�����ʵʱ���ȼ�
	RealTime_Pr_Update();

	//���»����˽������ȼ�
	Interact_Pr_Update();

	//������������IDSI----interactive driving safety index--������ʻ��ȫָ��--
	//���ڵ�i��������
	for (int i = 0; i < Num_Robot; i++)
	{
		//���ڵ�j���ϰ���
		for (int j = 0; j < Num_Robot; j++)
		{
			//�����i�������ˣ��ܹ��۲⵽��j���ϰ���
			if (Observe_index[i][j] == true)
			{
				//������interactive driving safety index--������ʻ��ȫָ��--��ֵ
				//int Index_R, int Index_O, float _R_RTPr, float _O_RTPr, float _R_Px, float _R_Py, float _R_Vx, float _R_Vy, float _O_Px, float _O_Py, float _O_Vx, float _O_Vy
				//��ʱע��������ָ�-------------------------------23.0706
				IDSI_T[i][j] = IDSI_Update(i, j, Interact_Pr[i][j], Interact_Pr[j][i], Robot_Data[i][0], Robot_Data[i][1], Robot_Data[i][2], Robot_Data[i][3], Robot_Data[j][0], Robot_Data[j][1], Robot_Data[j][2], Robot_Data[j][3]);
			
			}
			else
			{
				//��ʱע��������ָ�-------------------------------23.0706
				IDSI_T[i][j] = 0;
			}

		}
	}

	//IDSI����----����IDSI���ϰ�����в�̶Ƚ�������
	//��ʱע��������ָ�---------------------------Ŀǰ���Բ������򣿣�----23.0709
	//IDSI_Sort();	
}



//���»�����ʵʱ���ȼ�
void RealTime_Pr_Update()
{
	//���ڵ�i��������
	for (int i = 0; i < Num_Robot; i++)
	{
		//����ȫ������
		int UnsafeLidar_Num = 0;

		//���������״�����--�ж�Σ������
		for (int j = 0; j < Num_Lidar; j++)
		{
			//���j<5,���ѭ��
			if(j<5)
			{

			}
			//���j>34,���ѭ��
			if(j>34)
			{
				//�����״�����С�ڰ�ȫ����--�ж�
				if (idsi_Lidar_Data[i][j] < SafeyLidar_Data[j-30])
				{
					UnsafeLidar_Num++;
				}
			}
		}


		//���ݲ���ȫ���������������ȼ�Pr�����Ĺ�ʽ4
		RealTime_Pr[i] =  ( Base_Pr[i] - MinPr_c ) * ( 1 - (float)UnsafeLidar_Num / (float)Num_Lidar ) + MinPr_c;//ע������������ֻ��������������������Ҫ��ת��---230706
		//RealTime_Pr[i] = ( Base_Pr[i] - MinPr_c ) * ( 1 - (float)UnsafeLidar_Num / (float)Num_Lidar ) + MinPr_c ;//
		//RealTime_Pr[i] = 3.14;

		//��ʱȡ��log��-----230718
		// ROS_INFO("RealTime_Pr_Update() is running !!!!!, Base_Pr[i] = %f, MinPr_c = %f, Num_Lidar = %i, UnsafeLidar_Num  = %i, RealTime_Pr[i] = %f", 
		// Base_Pr[i], MinPr_c, Num_Lidar, UnsafeLidar_Num, RealTime_Pr[i] );
	}

}

//���»����˽������ȼ�
void Interact_Pr_Update()
{
	//���ڵ�i��������
	for (int i = 0; i < Num_Robot; i++)
	{
		//���ڵ�j���ϰ���
		for (int j = 0; j < Num_Robot; j++)
		{
			//�����i�������ˣ��ܹ��۲⵽��j���ϰ���
			if (Observe_index[i][j] == true)
			{
				//�����j���ϰ���ܹ�����۲⵽��i��������
				if (Observe_index[j][i] == true)
				{
					//�ɻ���۲⣬ʵʱ���ȼ�=���ȼ�RealTime_Pr
					Interact_Pr[i][j] = RealTime_Pr[i];
				}
				else
				{
					//����۲⣬ʵʱ���ȼ�=���ȼ�RealTime_Pr
					Interact_Pr[i][j] = MinPr_e * Base_Pr[i];
					//ROS_INFO("One Shot- Observation!!!!!!!!!!!!!!!!!!!!!!!!!" );
				}
			}
			else
			{
				//�����j���ϰ�����Է���۲⵽��i��������
				if (Observe_index[j][i] == true)
				{
					//i��j���ɹ۲⣬��j���Թ۲⵽i ---i�����Լ��ģ�ʵʱ���ȼ�=���ȼ�RealTime_Pr ������
					Interact_Pr[i][j] = RealTime_Pr[i];
				}
				else
				{
					//˫���޷��۲⣬Ӧ����ô������������---�趨Ϊ��ʵʱ���ȼ�=���ȼ�RealTime_Pr ������ʵ�����ǵش������ϱ��ϰ�ȫ������
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





//���»����˹۲ⷶΧ�ϰ�������������Observe_index[10][10]��������
void ObserveNum_Update()
{
	////�����˹۲ⷶΧ�ϰ�������������Observe_index[10][10]��������
	////��������һ������
	//Observe_Num = { 0,0,0,0,0,  0,0,0,0,0 };

	//���ڵ�i��������
	for (int i = 0; i < Num_Robot; i++)
	{
		//�۲���������
		int _Obs_num = 0;

		//���ڵ�j���ϰ���
		for (int j = 0; j < Num_Robot; j++)
		{
			//����ɹ۲�
			if (Observe_index[i][j] == true )
			{
				_Obs_num++;
			}
		}

		//��ֵ���۲�����
		Observe_Num[i] = _Obs_num;
	}
}


//���»�����----interactive driving safety index--������ʻ��ȫָ��
//Index_R��ʾ�����˵ı�ţ�����i����R_IAPr��ʾ�����˵Ľ������ȼ�������j��
//R_Px����Robot��x����λ�ã�O_Px����Robot��x����λ�ã��Դ�����
float IDSI_Update(int Index_R, int Index_O, float _R_IAPr, float _O_IAPr, float _R_Px, float _R_Py, float _R_Vx, float _R_Vy,  float _O_Px, float _O_Py, float _O_Vx, float _O_Vy)
{

	float RL_ji = 0;

	//���λ�ð뾶R_ji��ģ��
	RL_ji = sqrt( pow( _R_Px - _O_Px, 2.0) + pow(_R_Py - _O_Py, 2.0) );//�ϰ���j����ڻ�����i�ľ���,��ƽ�����ٿ���


	float VL_i = 0;

	//R������i�ٶ�VL_i��ģ��
	VL_i = sqrt(pow(_R_Vx, 2.0) + pow(_R_Vy, 2.0));//��ƽ�����ٿ���



	float VL_j = 0;

	//O�ϰ���j�ٶ�VL_j��ģ��
	VL_j = sqrt(pow(_O_Vx, 2.0f) + pow(_O_Vy, 2.0));//��ƽ�����ٿ���

	float O_Qj = 0;

	//O_Qj,��ʾO�ϰ���j���ٶȷ���V_j�����λ�ð뾶R_ji�ļнǣ����Ĺ�ʽ8
	//O_Qj = [ (_R_Px - _O_Px)*_O_Vx + (_R_Py - _O_Py)*_O_Vy ]/( RL_ji*VL_j );
	O_Qj = acos( ( (_R_Px - _O_Px) * _O_Vx + (_R_Py - _O_Py) * _O_Vy )/( RL_ji * VL_j ) );



	float SPE_Vji_aa = 0;

	//���ܳ���SPE�ĺ�벿�֣�Ϊ�˼򻯼��㣬���Ĺ�ʽ8
	SPE_Vji_aa = pow( SPE_k3 - VL_j * cos(O_Qj), 1.0 - SPE_k1 ) / (SPE_k3 - VL_j);

	float SPE_Vji = 0;

	//���ܳ���SPE�����Ĺ�ʽ8
	//SPE_Vji = (SPE_K* SPE_R*SPE_R * _R_IAPr*_O_IAPr * SPE_k3) * pow(SPE_Vji_aa, 1.0 / SPE_k1) / [ (SPE_k1 - 1.0)*pow(RL_ji, SPE_k1 - 1.0) ];
	SPE_Vji = (SPE_K* SPE_R* SPE_R * _R_IAPr* _O_IAPr * SPE_k3) * pow(SPE_Vji_aa, 1.0 / SPE_k1) / ( (SPE_k1 - 1.0) * pow( RL_ji, SPE_k1 - 1.0 ) );

	//���ε���������safety potential energy--��ȫ����-����
	//j��i�Ķ���SPE
	SPE_V[Index_R][Index_O] = SPE_Vji;

	//��ʱȡ��log��-----230718
	// ROS_INFO("IDSI_Update() is running-------, Robot-i =%i, Robot-j =%i, O_Qj = %f, VL_jr = %f, RL_ji = %f, SPE_Vji_aa = %f, SPE_Vji = %f", 
	// 	Index_R, Index_O, O_Qj, VL_j, RL_ji, SPE_Vji_aa, SPE_Vji );

	float P_Iji_aa = 0;

	//������--����������P_I�ĺ�벿�֣�Ϊ�˼򻯼��㣬���Ĺ�ʽ13
	//Rji�ͣ�Vj-Vi���ĵ��
	P_Iji_aa = (_R_Px - _O_Px) * (_O_Vx - _R_Vx) + (_R_Py - _O_Py) * (_O_Vy - _R_Vy);

	float P_Iji = 0;

	//������--����������P_I�����Ĺ�ʽ13
	P_Iji = (SPE_K* SPE_R*SPE_R * _R_IAPr*_O_IAPr) * P_Iji_aa / pow(RL_ji, SPE_k4 + 1.0);

	//���ε���������--����������-����
	Power_I[Index_R][Index_O] = P_Iji;

	

	float IDSI_ji = 0;

	//������interactive driving safety index--������ʻ��ȫָ��,�����Ĺ�ʽ10
	//IDSI_ji = -1.0 * SPE_a * SPE_Vji + (1.0 - SPE_a) * P_Iji;
	IDSI_ji = 100.0 * ( 1.0 * SPE_a * SPE_Vji + (1.0 - SPE_a) * P_Iji );

	//����ų�nan��inf���ݣ���ֹDRL�޷�ѵ��
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

	//���
	return IDSI_ji;
}












// ��ӡIDSI--���յ�������
void Print_IDSI()
{
//     // ��ӡ���յ�������--ʵʱ���ȼ�����--Base_Pr[i]
//     ROS_INFO("Base_Pr[i]  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f,] ", 
//     Base_Pr[0], Base_Pr[1], Base_Pr[2], Base_Pr[3], Base_Pr[4], 
//     Base_Pr[5], Base_Pr[6], Base_Pr[7], Base_Pr[8], Base_Pr[9] );

//     // ��ӡ���յ�������--ʵʱ���ȼ�����--RealTime_Pr[i]
//     ROS_INFO("RealTime_Pr[i]  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f,] ", 
//     RealTime_Pr[0], RealTime_Pr[1], RealTime_Pr[2], RealTime_Pr[3], RealTime_Pr[4], 
//     RealTime_Pr[5], RealTime_Pr[6], RealTime_Pr[7], RealTime_Pr[8], RealTime_Pr[9] );

//   for (int i = 0; i < 6; ++i)
//   {
  
//     // // ��ӡ���͵�����
//     ROS_INFO("Robot = %i, Observe_index[i][k] = [%i, %i, %i, %i, %i,  %i, %i, %i, %i, %i,] ", 
//     i, Observe_index[i][0], Observe_index[i][1], Observe_index[i][2], Observe_index[i][3], Observe_index[i][4], 
//      Observe_index[i][5], Observe_index[i][6], Observe_index[i][7], Observe_index[i][8], Observe_index[i][9] );	
//   }

//   for (int i = 0; i < 6; ++i)
//   {
// 	//�ɻ���۲⣬ʵʱ���ȼ�=���ȼ�RealTime_Pr
//     ROS_INFO("Robot = %i!!!!!, Interact_Pr[i][j]  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f, ] ", 
//     i, Interact_Pr[i][0], Interact_Pr[i][1], Interact_Pr[i][2], Interact_Pr[i][3], Interact_Pr[i][4], 
// 	 Interact_Pr[i][5], Interact_Pr[i][6], Interact_Pr[i][7], Interact_Pr[i][8], Interact_Pr[i][9]);		
//   }

//   for (int i = 0; i < 6; ++i)
//   {
// 	//SPE_V[i][j]����
//     ROS_INFO("Robot = %i!!!!!, SPE_V[i][j]  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f, ] ", 
//     i, SPE_V[i][0], SPE_V[i][1], SPE_V[i][2], SPE_V[i][3], SPE_V[i][4], 
// 	 SPE_V[i][5], SPE_V[i][6], SPE_V[i][7], SPE_V[i][8], SPE_V[i][9]);	
//   }

//     for (int i = 0; i < 6; ++i)
//   {
// 	//Power_I[i][j]��������
//     ROS_INFO("Robot = %i!!!!!, Power_I[i][j]  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f, ] ", 
//     i, Power_I[i][0], Power_I[i][1], Power_I[i][2], Power_I[i][3], Power_I[i][4], 
// 	 Power_I[i][5], Power_I[i][6], Power_I[i][7], Power_I[i][8], Power_I[i][9]);	
//   }

  for (int i = 0; i < 6; ++i)
  {
		//IDSI_T[i][j]��������
    ROS_INFO("Robot = %i!!!!!, IDSI_T[i][j]  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f, ] ", 
    i, IDSI_T[i][0], IDSI_T[i][1], IDSI_T[i][2], IDSI_T[i][3], IDSI_T[i][4], 
	 IDSI_T[i][5], IDSI_T[i][6], IDSI_T[i][7], IDSI_T[i][8], IDSI_T[i][9]);	
  }


//   for (int i = 0; i < 6; ++i)
//   {
//     // ��ӡ���յ�������--��ʵ�����״�
//     ROS_INFO("Received Robot = %i, Real-Lidardata  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f, ] ", 
//     i, lidar_legth[i][0], lidar_legth[i][1], lidar_legth[i][2], lidar_legth[i][3], lidar_legth[i][4], 
// 	 lidar_legth[i][5], lidar_legth[i][6], lidar_legth[i][7], lidar_legth[i][8], lidar_legth[i][9]);	 


// 	// ��ӡ���յ�������--IDSI�״�����
//     ROS_INFO("Received Robot = %i, IDSI-Lidardata  = [%f, %f, %f, %f, %f,  %f, %f, %f, %f, %f,] ", 
//     i, idsi_Lidar_Data[i][0], idsi_Lidar_Data[i][1], idsi_Lidar_Data[i][2], idsi_Lidar_Data[i][3], idsi_Lidar_Data[i][4], 
//      idsi_Lidar_Data[i][5], idsi_Lidar_Data[i][6], idsi_Lidar_Data[i][7], idsi_Lidar_Data[i][8], idsi_Lidar_Data[i][9]);

//   } 
}
///////////////////////////////////////////////////////////////////////////�Ӵ˴�����----IDSIģ�����------230704