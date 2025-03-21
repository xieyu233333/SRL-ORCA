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
//	Note:   Social Visual_Field 社会力观测视野程序--头文件
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

#include <std_msgs/Int32MultiArray.h>//用于IDSI_Rank数据发布--230702
#include <std_msgs/Float32MultiArray.h>//用于IDSI_Rank数据发布--230710

using namespace collvoid;
using std::vector;//为定义vector向量而增加，由cpp复制来--230628

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


// //控制观测时间段的bool值
// bool T_timer;

//时间段，在该段时间内
//float Time_Interval;

//机器人的真实数量
int Num_Robot;

// //是否需要？？？？？
// //机器人序号：0-9
// int Robot_index[10];

//机器人数据数组：横--序号，0-9  竖--位置坐标x,y,速度Vx,Vy,朝向角heading
float Robot_Data[10][5];

// //用于IDSI_Rank数据发布--230702
// int IDSI_Rank[6];

////是否需要？？？？？
////机器人观测半径
//public float Robot_Range[10];

//机器人观测结果数组：横--机器人序号，0-9  竖--动态障碍物序号（障碍物也即为其他机器人）,0-9，true为真--观测的到
bool Observe_index[10][10];

//机器人激光雷达探测长度数据：40个值，从正前方顺时针开始数？？---实际的激光雷达数据
float lidar_legth[6][40];//原为40测试中--机器人数量应为6个

//更新机器人数据函数--从ros发送数据中读取
void Robot_Data_Update();

//更新机器人观测函数--从ros发送数据中读取
void Output_Observe_index();

//判断动态障碍物是否在视野范围内 true--在范围内
bool IsIn_Range(float _A_str, float _R_Px, float _R_Py, float _head, float _O_Px, float _O_Py );

// //计时器
// bool Timer();


//以下为从interact.h--中复制的内容

//订阅者--激光雷达--回调函数
void scan_callback(const sensor_msgs::LaserScanConstPtr scan, int n);

//订阅者--机器人位置--回调函数
void odom_callback(const nav_msgs::OdometryConstPtr odom, int n);


//订阅激光雷达？？
ros::Subscriber scan_sub_[6];

//仿照写一个发布者？
ros::Publisher sac_cmd_vel_pub_[6];///////////////////////////////////////////////////////////////////////////////////////发布信息ros格式有待确认！！！！！//不用发布者

//仿照写一个发布者？----用于IDSI_Rank数据发布--230702
ros::Publisher IDSI_Rank[6];//----------------------------------------------------长度待定

// //仿照写一个发布者？
// ros::Publisher set_model_pub_[6];

//订阅者--订阅机器人位置信息？？
ros::Subscriber odom_sub_[6];

std::vector<boost::shared_ptr<Agent>> agent;



//以下为优先级类Pr的参数//////////////////////////////////////////////////////////////////////////////////////////////////////


//机器人0-9 基础优先级
float Base_Pr[10];

//机器人0-9 优先级
float RealTime_Pr[10];//Pr[10];

//机器人0-9 实时优先级
float Interact_Pr[10][10];

// //激光雷达的真实数量--由ros输入
// int Num_Lidar = 40;//暂定为40，可能还需要修改


// //安全机器人雷达数据：横--序号，0-9  竖--一定角度范围内，的机器人激光雷达数据，建议所有机器人的全部统一
// float SafeyLidar_Data[10];


//真实机器人雷达数据：横--序号，0-9  竖--一定角度范围内，的机器人激光雷达数据,暂时取10
//第i个机器人，第j个激光数据---激光雷达数据由机器人正前方为中轴，最左侧为0线，顺时针依次增大，根据实物雷达情况待定---230705
float idsi_Lidar_Data[10][10];


//各个函数值初始化
void init();

//idsi相关接口的初始化
void idsi_init();

//更新机器人雷达数据--从ros发送数据中读取
void idsi_Lidar_Update();


//更新机器人实时优先级
void RealTime_Pr_Update();//由Pr_Update()更改而来


//更新机器人交互优先级
void Interact_Pr_Update();//由RealTime_Pr_Update()更改而来

//计算机器人IDSI数据
void IDSI_Compute();


//以下为IDSI的参数//////////////////////////////////////////////////////////////////////////////////////////////////////

//机器人safety potential energy--安全势能-数组：横--机器人序号，0-9  竖--动态障碍物序号（障碍物也即为其他机器人）,0-9，数值为安全势能
float SPE_V[10][10];

//机器人--交互场功率-数组：横--机器人序号，0-9  竖--动态障碍物序号（障碍物也即为其他机器人）,0-9，数值为交互场功率
float Power_I[10][10];

//机器人interactive driving safety index--交互驾驶安全指数-数组：横--机器人序号，0-9  竖--动态障碍物序号（障碍物也即为其他机器人）,0-9，数值为交互驾驶安全指数
float IDSI_T[10][10];

//机器人观测范围障碍物数量，根据Observe_index[10][10]检索而来
//public bool Observe_index[10][10];
int Observe_Num[10];

//IDSI顺序数组----根据IDSI对障碍物威胁程度进行排序
int Rank_IDSI_Index[10][10];


//更新机器人观测范围障碍物数量，根据Observe_index[10][10]检索而来
void ObserveNum_Update();

//更新机器人----interactive driving safety index--交互驾驶安全指数

float IDSI_Update(int Index_R, int Index_O, float _R_IAPr, float _O_IAPr, 
float _R_Px, float _R_Py, float _R_Vx, float _R_Vy,  float _O_Px, float _O_Py, float _O_Vx, float _O_Vy);


//IDSI排序----根据IDSI对障碍物威胁程度进行排序
void IDSI_Sort();

//IDSI--顺序输出数据机器人数据
void IDSI_RankOut();

//冒泡排序-上浮 从大到小排序
vector <int> bubbleSort(vector<int>& nums, vector<int>& index);


// 打印IDSI--接收到的数组
void Print_IDSI();






