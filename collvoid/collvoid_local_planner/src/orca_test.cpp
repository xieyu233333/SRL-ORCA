/*
 * Copyright (c) 2012, Daniel Claes, Maastricht University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Maastricht University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <angles/angles.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <collvoid_srvs/GetObstacles.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include "collvoid_local_planner/orca_test.h"

#include "collvoid_local_planner/orca.h"
#include "collvoid_local_planner/common.h"

#include <iostream>

using namespace std;

OrcaLocalPlanner::OrcaLocalPlanner() {

}

OrcaLocalPlanner::~OrcaLocalPlanner() {

}

void OrcaLocalPlanner::init(bool sim) {
    ros::NodeHandle nh("~");
    sim_ = sim;//代表什么？？？
    for (int i = 0; i < 6; i++) {
        agent.push_back(AgentPtr(new Agent));
    }
    if (!sim) {
        for (int i = 0; i < 6; i++) {
            sac_cmd_vel_sub_[i] = nh.subscribe<geometry_msgs::Twist>("/burger" + (i > 0 ? std::to_string(i + 1) : "") + "/sac_cmd_vel", 4 * 4, boost::bind(&OrcaLocalPlanner::sac_cmd_vel_callback, this, _1, i));
        }
        for (int i = 0; i < 6; i++) {
            odom_sub_[i] = nh.subscribe<nav_msgs::Odometry>("/burger" + (i > 0 ? std::to_string(i + 1) : "") + "/odom", 1, boost::bind(&OrcaLocalPlanner::odom_callback, this, _1, i));
        }
        for (int i = 0; i < 6; i++) {
            rigid_sub_[i] = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/Rigid" + std::to_string(i+1) + "/pose", 1, boost::bind(&OrcaLocalPlanner::rigid_callback, this, _1, i));
        }
        for (int i = 0; i < 6; i++) {
            scan_sub_[i] = nh.subscribe<sensor_msgs::LaserScan>("/burger" + (i > 0 ? std::to_string(i + 1) : "") + "/scan", 1, boost::bind(&OrcaLocalPlanner::scan_callback, this, _1, i));
        }
        for (int i = 0; i < 6; i++) {
            cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/burger" + (i > 0 ? std::to_string(i + 1) : "") + "/cmd_vel", 4 * 4);
        }
        for (int i = 0; i < 6; i++) {
            obs_pub_[i] = nh.advertise<visualization_msgs::Marker>("/burger" + (i > 0 ? std::to_string(i + 1) : "") + "/obs", 4 * 4);
        }
    }
    else {
        cerr << "??";
        for (int i = 0; i < 6; i++) {
            sac_cmd_vel_sub_[i] = nh.subscribe<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/sac_cmd_vel", 4 * 4, boost::bind(&OrcaLocalPlanner::sac_cmd_vel_callback, this, _1, i));
        }
        for (int i = 0; i < 6; i++) {
            odom_sub_[i] = nh.subscribe<nav_msgs::Odometry>("/tb3_" + std::to_string(i ) + "/odom", 1, boost::bind(&OrcaLocalPlanner::odom_callback, this, _1, i));
        }
        for (int i = 0; i < 6; i++) {
            scan_sub_[i] = nh.subscribe<sensor_msgs::LaserScan>("/tb3_" + std::to_string(i ) + "/scan", 1, boost::bind(&OrcaLocalPlanner::scan_callback, this, _1, i));
        }
        for (int i = 0; i < 6; i++) {
            cmd_vel_pub_[i] = nh.advertise<geometry_msgs::Twist>("/tb3_" + std::to_string(i) + "/cmd_vel", 4 * 4);
        }
        for (int i = 0; i < 6; i++) {
            obs_pub_[i] = nh.advertise<visualization_msgs::Marker>("/tb3_" + std::to_string(i) + "/obs", 4 * 4);
        }
    }
    agent[0]->agent_neighbors_.push_back(agent[1]);
    agent[0]->agent_neighbors_.push_back(agent[2]);
    agent[0]->agent_neighbors_.push_back(agent[3]);
    agent[0]->agent_neighbors_.push_back(agent[4]);
    agent[0]->agent_neighbors_.push_back(agent[5]);
    agent[1]->agent_neighbors_.push_back(agent[0]);
    agent[1]->agent_neighbors_.push_back(agent[2]);
    agent[1]->agent_neighbors_.push_back(agent[3]);
    agent[1]->agent_neighbors_.push_back(agent[4]);
    agent[1]->agent_neighbors_.push_back(agent[5]);
    agent[2]->agent_neighbors_.push_back(agent[0]);
    agent[2]->agent_neighbors_.push_back(agent[1]);
    agent[2]->agent_neighbors_.push_back(agent[3]);
    agent[2]->agent_neighbors_.push_back(agent[4]);
    agent[2]->agent_neighbors_.push_back(agent[5]);
    agent[3]->agent_neighbors_.push_back(agent[0]);
    agent[3]->agent_neighbors_.push_back(agent[1]);
    agent[3]->agent_neighbors_.push_back(agent[2]);
    agent[3]->agent_neighbors_.push_back(agent[4]);
    agent[3]->agent_neighbors_.push_back(agent[5]);
    agent[4]->agent_neighbors_.push_back(agent[0]);
    agent[4]->agent_neighbors_.push_back(agent[1]);
    agent[4]->agent_neighbors_.push_back(agent[2]);
    agent[4]->agent_neighbors_.push_back(agent[3]);
    agent[4]->agent_neighbors_.push_back(agent[5]);
    agent[5]->agent_neighbors_.push_back(agent[0]);
    agent[5]->agent_neighbors_.push_back(agent[1]);
    agent[5]->agent_neighbors_.push_back(agent[2]);
    agent[5]->agent_neighbors_.push_back(agent[3]);
    agent[5]->agent_neighbors_.push_back(agent[4]);
    min_vel_x_ = -0.05;
    max_vel_x_ = 0.3;
    min_vel_y_ = 0.0;
    max_vel_y_ = 0.0;
    max_vel_th_ = 2.0;
    min_vel_th_ = -2.0;
    min_vel_th_inplace_ = 0.5;
    time_to_holo_ = 0.03;//0.066--231011,可调节的重要超参数，时间反应？？越小反应越快？？0.03，效果还不错
    min_error_holo_ = 0.02;
    max_error_holo_ = 0.1;
    footprint_radius_ = 0.01;//0.11--231011,可调节的重要超参数，感知半径？？对反应影响好像不大？？
    cur_loc_unc_radius_ = 0.0;
    double radius_ = footprint_radius_;
    //only coverting round footprints for now
    geometry_msgs::PolygonStamped footprint;
    geometry_msgs::Point32 p;
    double angle = 0;
    double step = 2 * M_PI / 16;
    while (angle < 2 * M_PI) {
        geometry_msgs::Point32 pt;
        pt.x = radius_ * cos(angle);
        pt.y = radius_ * sin(angle);
        pt.z = 0.0;
        footprint.polygon.points.push_back(pt);
        angle += step;
    }
    setFootprint(footprint);
    for (int i = 0; i < 6; i++) {
        last_twist_ang_[i] = 0;
    }
    for (int i = 0; i < 6; i++) {
        agent[i]->trunc_time_ = 1.0;
        agent[i]->left_pref_ = -0.05;
        agent[i]->timestep_ = 0.0;
        agent[i]->sim_period_ = 0.066;
        agent[i]->cur_allowed_error_ = 0.0;
    }
}

void OrcaLocalPlanner::scan_callback(const sensor_msgs::LaserScanConstPtr scan, int n) {
    double angle = agent[n]->heading_;
    obs_lines_[n].clear();
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

    for (int i = 0; i < scan.get()->ranges.size(); i++) {
        double dis = scan.get()->ranges[i];
        // ROS_INFO("%f", dis);
        Vector2 obs(dis * cos(angle), dis * sin(angle));
        obs += agent[n]->position_;
        computeObstacleLine(obs, n);
        angle += scan.get()->angle_increment;
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
    obs_pub_[n].publish(obs_marker);
}

void OrcaLocalPlanner::computeObstacleLine(Vector2 &obst, int n) {
    Line line;
    Vector2 relative_position = obst - agent[n]->position_;
    double dist_to_footprint;
    double dist = collvoid::abs(agent[n]->position_ - obst);
    dist_to_footprint = footprint_radius_;
    dist = dist - dist_to_footprint - 0.03;
    //if (dist < (double)max_vel_with_obstacles_){
    //  dist *= dist;
    //}
    //    line.point = normalize(relative_position) * (dist - dist_to_footprint - 0.03);
    line.point = normalize(relative_position) * (dist);
    line.dir = Vector2(-normalize(relative_position).y(), normalize(relative_position).x());
    obs_lines_[n].push_back(line);
}

void OrcaLocalPlanner::setFootprint(geometry_msgs::PolygonStamped footprint) {
    if (footprint.polygon.points.size() < 2) {
        ROS_ERROR("The footprint specified has less than two nodes");
        return;
    }
    footprint_msg_ = footprint;
    setMinkowskiFootprintVector2(footprint_msg_);

    footprint_lines_.clear();
    geometry_msgs::Point32 p = footprint_msg_.polygon.points[0];
    collvoid::Vector2 first = collvoid::Vector2(p.x, p.y);
    collvoid::Vector2 old = collvoid::Vector2(p.x, p.y);
    //add linesegments for footprint
    for (size_t i = 0; i < footprint_msg_.polygon.points.size(); i++) {
        p = footprint_msg_.polygon.points[i];
        collvoid::Vector2 point = collvoid::Vector2(p.x, p.y);
        footprint_lines_.push_back(std::make_pair(old, point));
        old = point;
    }
    //add last segment
    footprint_lines_.push_back(std::make_pair(old, first));
    has_polygon_footprint_ = true;
}

void OrcaLocalPlanner::setMinkowskiFootprintVector2(geometry_msgs::PolygonStamped minkowski_footprint) {
    minkowski_footprint_.clear();
    ROS_INFO("minkowski_footprint.polygon.points: %d", minkowski_footprint.polygon.points.size());//%d"
    BOOST_FOREACH(geometry_msgs::Point32 p, minkowski_footprint.polygon.points) {
                    minkowski_footprint_.push_back(Vector2(p.x, p.y));
                }
}

void OrcaLocalPlanner::sac_cmd_vel_callback(const geometry_msgs::TwistConstPtr cmd_vel, int n) {
    // ROS_INFO("%d", n);
    sac_cmd_vel_[n] = *cmd_vel.get();
    compute_vel(n);
}

void OrcaLocalPlanner::odom_callback(const nav_msgs::OdometryConstPtr odom, int n) {
    // ROS_INFO("odom:%d", odom.get()->header.stamp.nsec);
    if (sim_)
        agent[n]->position_ = collvoid::Vector2(odom.get()->pose.pose.position.x, odom.get()->pose.pose.position.y);
    agent[n]->velocity_ = rotateVectorByAngle(odom.get()->twist.twist.linear.x,
                                                odom.get()->twist.twist.linear.y, (agent[n]->heading_));
    if (sim_)
        agent[n]->heading_ = tf::getYaw(odom.get()->pose.pose.orientation);
    // ROS_INFO("%d, velocity_:%6.2f,%6.2f", n, agent[n]->velocity_.x(), agent[n]->velocity_.y());
    agent[n]->footprint_ = rotateFootprint(minkowski_footprint_, agent[n]->heading_);
}

void OrcaLocalPlanner::rigid_callback(const geometry_msgs::PoseStampedConstPtr odom, int n) {
    agent[n]->position_ = collvoid::Vector2(-odom.get()->pose.position.x, -odom.get()->pose.position.y);
    agent[n]->heading_ = tf::getYaw(odom.get()->pose.orientation);
    agent[n]->heading_ += M_PI / 2.0;
    if (agent[n]->heading_ > M_PI) {
        agent[n]->heading_ -= 2 * M_PI;
    }
    // ROS_INFO("%d, position_:%6.2f,%6.2f heading:%6.2f", n, agent[n]->position_.x(), agent[n]->position_.y(), agent[n]->heading_);
    agent[n]->footprint_ = rotateFootprint(minkowski_footprint_, agent[n]->heading_);
}

bool OrcaLocalPlanner::compareNeighborsPositions(const AgentPtr &agent1, const AgentPtr &agent2, int n) {
    return compareVectorPosition(agent1->position_, agent2->position_, n);
}

bool OrcaLocalPlanner::compareVectorPosition(const collvoid::Vector2 &v1, const collvoid::Vector2 &v2, int n) {
    return collvoid::absSqr(agent[n]->position_ - v1) <= collvoid::absSqr(agent[n]->position_ - v2);
}

void OrcaLocalPlanner::compute_vel(int n) {
    geometry_msgs::Twist cmd_vel;
    Vector2 pref_velocity = Vector2(0.0, 0.0);
    agent[n]->new_velocity_ = Vector2(0.0, 0.0);

    Vector2 position = agent[n]->position_;
    double x_vel = sac_cmd_vel_[n].linear.x * 1.25;
    double z_angular = sac_cmd_vel_[n].angular.z;
    // position = Vector2(0.0, 0.0);
    // x_vel = 0.02;
    // z_angular = M_PI / 20;
    Vector2 raw_position = position;
    double heading = agent[n]->heading_;
    // heading = 0;
    double raw_heading = heading;
    const double dt = 0.01;
    for (double t = 0; t < 0.066; t += dt) {
        heading += z_angular * dt;
        if (heading > M_PI) {
            heading -= 2 * M_PI;
        } else if (heading < -M_PI) {
            heading += 2 * M_PI;
        }
        position.x() += x_vel * cos(heading) * dt;
        position.y() += x_vel * sin(heading) * dt;
    }
    pref_velocity = (position - raw_position) / 0.066;
    // ROS_INFO("%d: %f,%f -> %f,%f (%f,%f)", n, x_vel, z_angular, pref_velocity.x(), pref_velocity.y(), position.x(), position.y());

    // switch (n) {
    // case 0:
    //     pref_velocity = Vector2(0.2, -0.2);
    //     // pref_velocity = Vector2(0.0, 0.0);
    //     break;
    // case 1:
    //     pref_velocity = Vector2(-0.2, -0.2);
    //     // pref_velocity = Vector2(0.0, 0.0);
    //     break;
    // case 2:
    //     pref_velocity = Vector2(-0.2, 0.2);
    //     // pref_velocity = Vector2(0.0, 0.0);
    //     break;
    // case 3:
    //     pref_velocity = Vector2(0.2, 0.2);
    //     // pref_velocity = Vector2(0.0, 0.0);
    //     break;
    // }
    Vector2 new_velocity_;

    if (fabs(pref_velocity.x()) > 0.04 || fabs(pref_velocity.x()) > 0.04) {

        if (fabs(pref_velocity.x()) > 0.01 || fabs(pref_velocity.x()) > 0.01) {
            agent[n]->controlled_ = true;
        } else {
            agent[n]->controlled_ = false;
        }

        std::sort(agent[n]->agent_neighbors_.begin(), agent[n]->agent_neighbors_.end(),
                    boost::bind(&OrcaLocalPlanner::compareNeighborsPositions, this, _1, _2, n));
        //reset
        agent[n]->additional_orca_lines_.clear();

        //get closest agent/obstacle
        double min_dist_neigh = DBL_MAX;
        if (agent[n]->agent_neighbors_.size() > 0)
            min_dist_neigh = collvoid::abs(agent[n]->agent_neighbors_[0]->position_ - agent[n]->position_);

        double min_dist = std::min(min_dist_neigh, DBL_MAX);

        //incorporate NH constraints
        agent[n]->max_speed_x_ = max_vel_x_;

        // ROS_INFO("addNHConstraints");
        addNHConstraints(min_dist, pref_velocity, n);

        for (int i = 0 ; i < obs_lines_[n].size(); i++) {
            agent[n]->additional_orca_lines_.push_back(obs_lines_[n][i]);
        }

        // ROS_INFO("%d", n);
        agent[n]->computeOrcaVelocity(pref_velocity, true);

        new_velocity_ = agent[n]->new_velocity_;
        // new_velocity_ = pref_velocity;

        double speed_ang = atan2(new_velocity_.y(), new_velocity_.x());
        // ROS_INFO("speed_ang:%f, raw_heading:%f", speed_ang, raw_heading);
        double dif_ang = angles::shortest_angular_distance(raw_heading, speed_ang);

        double vel = collvoid::abs(new_velocity_);
        double vstar;

        if (std::abs(dif_ang) > EPSILON)
            vstar = calcVstar(vel, dif_ang);
        else
            vstar = max_vel_x_;

        cmd_vel.linear.x = std::min(vstar, vMaxAng());
        cmd_vel.linear.y = 0.0;

        // ROS_INFO("dif_ang %f", dif_ang);
        if (std::abs(dif_ang) > 3.0 * M_PI / 4.0) {
            if (last_twist_ang_[n] != 0.0)
            {
                cmd_vel.angular.z = sign(last_twist_ang_[n]) *
                                std::min(std::abs(dif_ang / time_to_holo_), max_vel_th_);
            }
            else {
                cmd_vel.angular.z = sign(dif_ang) *
                                    std::min(std::abs(dif_ang / time_to_holo_), max_vel_th_);

            }
            last_twist_ang_[n] = cmd_vel.angular.z;
        }
        else {
            cmd_vel.angular.z = sign(dif_ang) * std::min(std::abs(dif_ang / time_to_holo_), max_vel_th_);
            last_twist_ang_[n] = 0.;
        }

        // cmd_vel.linear.x = sac_cmd_vel_[n].linear.x;
        // cmd_vel.angular.z = sac_cmd_vel_[n].angular.z;
        cmd_vel_pub_[n].publish(cmd_vel);
    } else {
        cmd_vel_pub_[n].publish(sac_cmd_vel_[n]);
    }
    // ROS_INFO("agent %d pref_vel:%f, %f vel:%f, %f", n, pref_velocity.x(), pref_velocity.y(), new_velocity_.x(), new_velocity_.y());
    // ROS_INFO("agent %d sac_cmd:%f, %f cmd:%f, %f", n, sac_cmd_vel_[n].linear.x, sac_cmd_vel_[n].angular.z, cmd_vel.linear.x, cmd_vel.angular.z);
}

void OrcaLocalPlanner::addNHConstraints(double min_dist, Vector2 pref_velocity, int n) {
    double min_error = min_error_holo_;
    double max_error = max_error_holo_;
    double error = max_error;
    double v_max_ang = vMaxAng();

    //ROS_ERROR("v_max_ang %.2f", v_max_ang);

    if (min_dist < 2.0 * footprint_radius_ + cur_loc_unc_radius_) {
        error = (max_error - min_error) / (collvoid::sqr(2 * (footprint_radius_ + cur_loc_unc_radius_))) *
                collvoid::sqr(min_dist) + min_error; // how much error do i allow?
        //ROS_DEBUG("Error = %f", error);
        if (min_dist < 0) {
            error = min_error;
            // ROS_DEBUG("%s I think I am in collision", me_->getId().c_str());
        }
    }
    agent[n]->cur_allowed_error_ = 1.0 / 3.0 * agent[n]->cur_allowed_error_ + 2.0 / 3.0 * error;
    //ROS_ERROR("error = %f", cur_allowed_error_);
    double speed_ang = atan2(pref_velocity.y(), pref_velocity.x());
    double dif_ang = angles::shortest_angular_distance(agent[n]->heading_, speed_ang);
    if (std::abs(dif_ang) > M_PI / 2.0) { // || cur_allowed_error_ < 2.0 * min_error) {
        double max_track_speed = calculateMaxTrackSpeedAngle(time_to_holo_, M_PI / 2.0, agent[n]->cur_allowed_error_,
                                                                max_vel_x_, max_vel_th_, v_max_ang);
        if (max_track_speed <= 2 * min_error) {
            max_track_speed = 2 * min_error;
        }
        //ROS_INFO("Max Track speed %f", max_track_speed);
        addMovementConstraintsDiffSimple(max_track_speed, agent[n]->heading_, agent[n]->additional_orca_lines_);
    }
    else {
        addMovementConstraintsDiff(agent[n]->cur_allowed_error_, time_to_holo_, max_vel_x_, max_vel_th_, agent[n]->heading_, v_max_ang,
                                    agent[n]->additional_orca_lines_);
    }
    agent[n]->max_speed_x_ = vMaxAng();
}

double OrcaLocalPlanner::vMaxAng() {
    return max_vel_x_;
}

#define LIMIT(x,min,max) 	((x) = ((x) < (min)  ? (min) : ((x) > (max) ? (max) : (x) )))
#define LIMIT_R(x,min,max) 	((x) < (min)  ? (min) : ((x) > (max) ? (max) : (x)))

template <typename T>
class Vec2 {
public:
    T x_, y_;
    
    Vec2(): x_(0.0), y_(0.0) {}
    Vec2(const T x, const T y): x_(x), y_(y) {}
    Vec2<T> operator+(const Vec2<T> &v) const {return Vec2<T>(x_ + v.x_, y_ + v.y_);}
    Vec2<T> operator-(const Vec2<T> &v) const {return Vec2<T>(x_ - v.x_, y_ - v.y_);}
    Vec2<T> operator*(const T v) const {return Vec2<T>(x_ * v, y_ * v);}
    friend Vec2<T> operator*(const T v, const Vec2<T> a) {return Vec2<T>(a.x_ * v, a.y_ * v);}
    T operator*(const Vec2<T> v) const {return x_ * v.x_ + y_ * v.y_;}
    Vec2<T> operator/(const T v) const {return Vec2<T>(x_ / v, y_ / v);}
    bool operator==(const Vec2 &v) const {return x_ == v.x_ && y_ == v.y_;}
    double mod() const {return sqrt(x_ * x_ + y_ * y_);}
    double square_norm() const {return x_ * x_ + y_ * y_;}
    double ang() const {return atan2(y_, x_);}
};

#define Vec2d Vec2<double>
#define Vec2f Vec2<float>
#define Vec2i Vec2<int>

inline Vec2d compute_vel(Vec2d pos, double angle, Vec2d goal) {
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
    
    double linear = LIMIT_R(dis * 0.6, -0.28, 0.28);
    if (fabs(delta_angle) > M_PI * 1.0 / 4) {
        linear = 0.0;
    }
    static double last_delta_angle = 0.0;
    double angular = LIMIT_R(-delta_angle * 1.2, -2, 2);
    last_delta_angle = delta_angle;
    if (dis < 0.1) {
        angular = 0.0;
    }
    return Vec2d(linear, angular);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "OrcaLocalPlanner");
    //ros::NodeHandle nh;
    OrcaLocalPlanner planner;
    planner.init(argv[1][0] == '1');
    ros::Rate rate(50);
    
    ROS_INFO("OrcaLocalPlanner initialized");
    // ros::spin();
    for (int i = 0; i < 50 * 1; i++) {
        rate.sleep();
    }

    double goal[][2] = {
        // {-0.2, 1.11},
        // {0.15, 1.55},
        // {-1.5, -0.2},
        // {-1.0, 0.2},
        // {0.0, -1.54},
        // {1.51, -0.22}

        // {-1.5, -0.17},
        // {-1.2, 0.21},
        // {1.5, 0.13},
        // {1.12, 0.18},
        // {0.2, -1.3},
        // {0.0, 1.5}

        {-1.0433609619140625, -0.2237098541259765},
        {-2.4360283203125, -0.27485211181640623},
        {0.873778564453125, -1.773697021484375},
        {0.45964913940429686, -1.451390869140625},
        {-0.01739626121520996, -1.196630615234375},
        {0.22241242218017578, 0.577993225097656}
    };
    /*
    [-1.5, -0.17],
            [-1.2, 0.21],
            [0.2, -1.3],
            [1.12, 0.18],
            [1.5, 0.13],
            [0.0, 1.5]
    */
    int cnt = 0;
    while (ros::ok()) {
        // if (cnt++ % 3 == 0) {
            // for (int i = 0; i < 6; i++) {
            //     auto cv = compute_vel(
            //         Vec2d(planner.agent[i]->position_.x(), planner.agent[i]->position_.y()),
            //         planner.agent[i]->heading_,
            //         Vec2d(goal[i][0], goal[i][1])
            //     );
            //     geometry_msgs::Twist tmp;
            //     tmp.linear.x = cv.x_;
            //     tmp.angular.z = cv.y_;
            //     // tmp.linear.x = 0;//cv.x_;
            //     // tmp.angular.z = 0;//cv.y_;
            //     // cout << cv.x_ << " " << cv.y_ << endl;
            //     planner.sac_cmd_vel_[i] = tmp;
            //     planner.compute_vel(i);
            // }
        // }
        // planner.compute_vel(2);
        ros::spinOnce();
        rate.sleep();
    }

}


