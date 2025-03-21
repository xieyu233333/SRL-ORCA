//-----------------------------------------------------------------------------
//	Version 1.0
//-----------------------------------------------------------------------------
//	File Name: interact.h
//-----------------------------------------------------------------------------
//	CopyRight (C) 2023, TigherLab.USTC
//-----------------------------------------------------------------------------
//	Modifier	Date		Detail
//
//	qinjianmin		20230325	create
//-----------------------------------------------------------------------------
//	Note:   Interaction 交互数据程序--头文件
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

using namespace collvoid;

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

//机器人数据数组：横--序号，0-1  竖--位置坐标x,y,速度Vx,Vy,还需要加上角度数据，如何表示初始角度？？
float begin_inf[2][4];

//机器人目的地--数组：横--序号，0-1  竖--位置坐标x,y,速度Vx,Vy,还需要加上角度数据，如何表示初始角度？？
float target_inf[2][2];

//机器人速度到达--数组--初始为false
bool arrival_bool[2];

// //机器人速度列表数组--初始为false
// bool speedlist_bool[2];

//机器人发布速度信息--判别数组--初始为true
//bool velsend_bool[2];

//程序开始判别--bool
bool scriptbegin_bool;

//交互数据收集 横--横向编号， 竖--纵向编号,内容为交互数据值
//x向--20  y向--10
float interact_data[20][10];

//距离数据收集 横--横向编号， 竖--纵向编号,内容为出发点到终点距离值
//x向--20  y向--10
float dis_data[20][10];

//真实数据收集 横--横向编号， 竖--纵向编号,内容为交互数据值
//x向--20  y向--10
float real_data[20][10];

//机器人激光雷达探测长度数据：40个值，从正前方顺时针开始数？？
float lidar_legth[2][40];


//超参数
//机器人横纵向格子编号？？？速度编号
int _index_x;
int _index_y;

//机器人速度--编号--初始化为0
int _index_speed;

//机器人目标--编号--初始化为0
int _index_target;

//各个函数值初始化
void init();

//出发位置计算
void local_compute();


//交互数据计算
void interactdata_update( int _localindex_x, int _localindex_y);

//订阅者--激光雷达--回调函数
void scan_callback(const sensor_msgs::LaserScanConstPtr scan, int n);

//订阅者--机器人位置--回调函数
void odom_callback(const nav_msgs::OdometryConstPtr odom, int n);

//机器人线速度角速度--计算函数
inline Vec2d compute_vel(Vec2d pos, double angle, Vec2d goal, double angular_fb, float max_speed);

//打印--交互数据--一行的数组
void interactdata_print( int _line_x );

//打印--计算两点距离
float distance_data( float _data_1_x, float _data_1_y, float _data_2_x, float _data_2_y );

// //计时器
// bool Timer();

//订阅激光雷达？？
//还有待细细调整-------------------------------------------------------------------------
ros::Subscriber scan_sub_[2];

//仿照写一个发布者？
//还有待细细调整-------------------------------------------------------------------------
ros::Publisher sac_cmd_vel_pub_[2];

//仿照写一个发布者？
//还有待细细调整-------------------------------------------------------------------------
ros::Publisher set_model_pub_[2];

//订阅者--订阅机器人位置信息？？
ros::Subscriber odom_sub_[2];

std::vector<boost::shared_ptr<Agent>> agent;

// struct Obstacle {
//         std::vector<Vector2> points;
//         ros::Time last_seen;
//     };

// class OrcaLocalPlanner{
// public:
//     OrcaLocalPlanner();

//     ~OrcaLocalPlanner();

//     void init();
//     void compute_vel(int n);
//     void sac_cmd_vel_callback(const geometry_msgs::TwistConstPtr cmd_vel, int n);
//     void odom_callback(const nav_msgs::OdometryConstPtr odom, int n);
//     void rigid_callback(const geometry_msgs::PoseStampedConstPtr odom, int n);
//     void scan_callback(const sensor_msgs::LaserScanConstPtr scan, int n);
//     bool compareNeighborsPositions(const AgentPtr &agent1, const AgentPtr &agent2, int n);
//     bool compareVectorPosition(const collvoid::Vector2 &v1, const collvoid::Vector2 &v2, int n);
//     void addNHConstraints(double min_dist, Vector2 pref_velocity, int n);
//     void setFootprint(geometry_msgs::PolygonStamped footprint);
//     void setMinkowskiFootprintVector2(geometry_msgs::PolygonStamped minkowski_footprint);
//     void computeObstacleLine(Vector2 &obst, int n);
//     double vMaxAng();

//     ros::Subscriber sac_cmd_vel_sub_[4];
//     ros::Subscriber odom_sub_[4];
//     ros::Subscriber rigid_sub_[4];
//     ros::Subscriber scan_sub_[4];
//     ros::Publisher cmd_vel_pub_[4];
//     ros::Publisher obs_pub_[4];
//     geometry_msgs::Twist sac_cmd_vel_[4];
//     geometry_msgs::Twist new_cmd_vel_[4];
//     std::vector<boost::shared_ptr<Agent>> agent;
//     double min_vel_x_, max_vel_x_, min_vel_y_, max_vel_y_, max_vel_th_, min_vel_th_, min_vel_th_inplace_;
//     double last_twist_ang_[4];
//     double time_to_holo_;
//     double min_error_holo_;
//     double max_error_holo_;
//     double footprint_radius_;
//     double cur_loc_unc_radius_;
//     geometry_msgs::PolygonStamped footprint_msg_;
//     std::vector<std::pair<collvoid::Vector2, collvoid::Vector2> > footprint_lines_;
//     bool has_polygon_footprint_;
//     std::vector<Vector2> minkowski_footprint_;
//     std::vector<Line> obs_lines_[4];
// };
