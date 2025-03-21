//-----------------------------------------------------------------------------
//	Version 1.0
//-----------------------------------------------------------------------------
//	File Name: Visual_Field.h
//-----------------------------------------------------------------------------
//	CopyRight (C) 2023, TigherLab.USTC
//-----------------------------------------------------------------------------
//	Modifier	Date		Detail
//
//	qinjianmin		20230307	create
//-----------------------------------------------------------------------------
//	Note:   Social Visual_Field ������۲���Ұ����--ͷ�ļ�
//-----------------------------------------------------------------------------


#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <collvoid_msgs/PoseArrayWeighted.h>
#include <collvoid_msgs/AggregatedPoseTwist.h>

#include <geometry_msgs/PolygonStamped.h>

#include <collvoid_msgs/PoseTwistWithCovariance.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include "collvoid_local_planner/Vector2.h"

#include <collvoid_local_planner/Agent.h>
#include <collvoid_local_planner/GetCollvoidTwist.h>
#include<gazebo_msgs/ModelState.h>

#include <std_msgs/Int32MultiArray.h>//����IDSI_Rank���ݷ���--230702
#include <std_msgs/Float32MultiArray.h>//����IDSI_Rank���ݷ���--230710

using namespace collvoid;
using std::vector;//Ϊ����vector���������ӣ���cpp������--230628

struct Obstacle {
        std::vector<Vector2> points;
        ros::Time last_seen;
    };

#define LIMIT(x,min,max) ((x) = ((x) < (min)  ? (min) : ((x) > (max) ? (max) : (x) )))
#define LIMIT_R(x,min,max) ((x) < (min)  ? (min) : ((x) > (max) ? (max) : (x)))

template <typename T>
class Vec2 {
public:
    T x_, y_;
    
    Vec2(): x_(0.0), y_(0.0) {}
    Vec2(T x, T y): x_(x), y_(y) {}
    Vec2<T> operator+(const Vec2<T> &v) const {return Vec2<T>(x_ + v.x_, y_ + v.y_);}
    Vec2<T> operator-(const Vec2<T> &v) const {return Vec2<T>(x_ - v.x_, y_ - v.y_);}
    Vec2<T> operator*(const T v) const {return Vec2<T>(x_ * v, y_ * v);}
    T operator*(const Vec2<T> v) const {return x_ * v.x_ + y_ * v.y_;}
    Vec2<T> operator/(const T v) const {return Vec2<T>(x_ / v, y_ / v);}
    bool operator==(const Vec2 &v) const {return x_ == v.x_ && y_ == v.y_;}
    double mod() const {return sqrt(x_ * x_ + y_ * y_);}
    double ang() const {return atan2(y_, x_);}
};

#define Vec2d Vec2<double>
#define Vec2f Vec2<float>
#define Vec2i Vec2<int>


// //���ƹ۲�ʱ��ε�boolֵ
// bool T_timer;

//ʱ��Σ��ڸö�ʱ����
//float Time_Interval;

//�����˵���ʵ����
int Num_Robot;

// //�Ƿ���Ҫ����������
// //��������ţ�0-9
// int Robot_index[10];

//�������������飺��--��ţ�0-9  ��--λ������x,y,�ٶ�Vx,Vy,�����heading
float Robot_Data[10][5];

// //����IDSI_Rank���ݷ���--230702
// int IDSI_Rank[6];

////�Ƿ���Ҫ����������
////�����˹۲�뾶
//public float Robot_Range[10];

//�����˹۲������飺��--��������ţ�0-9  ��--��̬�ϰ�����ţ��ϰ���Ҳ��Ϊ���������ˣ�,0-9��trueΪ��--�۲�ĵ�
bool Observe_index[10][10];

//�����˼����״�̽�ⳤ�����ݣ�40��ֵ������ǰ��˳ʱ�뿪ʼ������---ʵ�ʵļ����״�����
float lidar_legth[6][40];//ԭΪ40������--����������ӦΪ6��

//���»��������ݺ���--��ros���������ж�ȡ
void Robot_Data_Update();

//���»����˹۲⺯��--��ros���������ж�ȡ
void Output_Observe_index();

//�ж϶�̬�ϰ����Ƿ�����Ұ��Χ�� true--�ڷ�Χ��
bool IsIn_Range(float _A_str, float _R_Px, float _R_Py, float _head, float _O_Px, float _O_Py );

// //��ʱ��
// bool Timer();


//����Ϊ��interact.h--�и��Ƶ�����

//������--�����״�--�ص�����
void scan_callback(const sensor_msgs::LaserScanConstPtr scan, int n);

//������--������λ��--�ص�����
void odom_callback(const nav_msgs::OdometryConstPtr odom, int n);


//���ļ����״��
ros::Subscriber scan_sub_[6];

//����дһ�������ߣ�
ros::Publisher sac_cmd_vel_pub_[6];///////////////////////////////////////////////////////////////////////////////////////������Ϣros��ʽ�д�ȷ�ϣ���������//���÷�����

//����дһ�������ߣ�----����IDSI_Rank���ݷ���--230702
ros::Publisher IDSI_Rank[6];//----------------------------------------------------���ȴ���

// //����дһ�������ߣ�
// ros::Publisher set_model_pub_[6];

//������--���Ļ�����λ����Ϣ����
ros::Subscriber odom_sub_[6];

std::vector<boost::shared_ptr<Agent>> agent;



//����Ϊ���ȼ���Pr�Ĳ���//////////////////////////////////////////////////////////////////////////////////////////////////////


//������0-9 �������ȼ�
float Base_Pr[10];

//������0-9 ���ȼ�
float RealTime_Pr[10];//Pr[10];

//������0-9 ʵʱ���ȼ�
float Interact_Pr[10][10];

// //�����״����ʵ����--��ros����
// int Num_Lidar = 40;//�ݶ�Ϊ40�����ܻ���Ҫ�޸�


// //��ȫ�������״����ݣ���--��ţ�0-9  ��--һ���Ƕȷ�Χ�ڣ��Ļ����˼����״����ݣ��������л����˵�ȫ��ͳһ
// float SafeyLidar_Data[10];


//��ʵ�������״����ݣ���--��ţ�0-9  ��--һ���Ƕȷ�Χ�ڣ��Ļ����˼����״�����,��ʱȡ10
//��i�������ˣ���j����������---�����״������ɻ�������ǰ��Ϊ���ᣬ�����Ϊ0�ߣ�˳ʱ���������󣬸���ʵ���״��������---230705
float idsi_Lidar_Data[10][10];


//��������ֵ��ʼ��
void init();

//idsi��ؽӿڵĳ�ʼ��
void idsi_init();

//���»������״�����--��ros���������ж�ȡ
void idsi_Lidar_Update();


//���»�����ʵʱ���ȼ�
void RealTime_Pr_Update();//��Pr_Update()���Ķ���


//���»����˽������ȼ�
void Interact_Pr_Update();//��RealTime_Pr_Update()���Ķ���

//���������IDSI����
void IDSI_Compute();


//����ΪIDSI�Ĳ���//////////////////////////////////////////////////////////////////////////////////////////////////////

//������safety potential energy--��ȫ����-���飺��--��������ţ�0-9  ��--��̬�ϰ�����ţ��ϰ���Ҳ��Ϊ���������ˣ�,0-9����ֵΪ��ȫ����
float SPE_V[10][10];

//������--����������-���飺��--��������ţ�0-9  ��--��̬�ϰ�����ţ��ϰ���Ҳ��Ϊ���������ˣ�,0-9����ֵΪ����������
float Power_I[10][10];

//������interactive driving safety index--������ʻ��ȫָ��-���飺��--��������ţ�0-9  ��--��̬�ϰ�����ţ��ϰ���Ҳ��Ϊ���������ˣ�,0-9����ֵΪ������ʻ��ȫָ��
float IDSI_T[10][10];

//�����˹۲ⷶΧ�ϰ�������������Observe_index[10][10]��������
//public bool Observe_index[10][10];
int Observe_Num[10];

//IDSI˳������----����IDSI���ϰ�����в�̶Ƚ�������
int Rank_IDSI_Index[10][10];


//���»����˹۲ⷶΧ�ϰ�������������Observe_index[10][10]��������
void ObserveNum_Update();

//���»�����----interactive driving safety index--������ʻ��ȫָ��

float IDSI_Update(int Index_R, int Index_O, float _R_IAPr, float _O_IAPr, 
float _R_Px, float _R_Py, float _R_Vx, float _R_Vy,  float _O_Px, float _O_Py, float _O_Vx, float _O_Vy);


//IDSI����----����IDSI���ϰ�����в�̶Ƚ�������
void IDSI_Sort();

//IDSI--˳��������ݻ���������
void IDSI_RankOut();

//ð������-�ϸ� �Ӵ�С����
vector <int> bubbleSort(vector<int>& nums, vector<int>& index);


// ��ӡIDSI--���յ�������
void Print_IDSI();






