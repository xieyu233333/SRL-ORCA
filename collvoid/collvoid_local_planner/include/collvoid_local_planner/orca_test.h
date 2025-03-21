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


#ifndef ORCA_TEST_H
#define ORCA_TEST_H

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

#include <collvoid_local_planner/Agent.h>
#include <collvoid_local_planner/GetCollvoidTwist.h>

using namespace collvoid;

struct Obstacle {
        std::vector<Vector2> points;
        ros::Time last_seen;
    };

class OrcaLocalPlanner{
public:
    OrcaLocalPlanner();

    ~OrcaLocalPlanner();

    void init(bool sim);
    void compute_vel(int n);
    void sac_cmd_vel_callback(const geometry_msgs::TwistConstPtr cmd_vel, int n);
    void odom_callback(const nav_msgs::OdometryConstPtr odom, int n);
    void rigid_callback(const geometry_msgs::PoseStampedConstPtr odom, int n);
    void scan_callback(const sensor_msgs::LaserScanConstPtr scan, int n);
    bool compareNeighborsPositions(const AgentPtr &agent1, const AgentPtr &agent2, int n);
    bool compareVectorPosition(const collvoid::Vector2 &v1, const collvoid::Vector2 &v2, int n);
    void addNHConstraints(double min_dist, Vector2 pref_velocity, int n);
    void setFootprint(geometry_msgs::PolygonStamped footprint);
    void setMinkowskiFootprintVector2(geometry_msgs::PolygonStamped minkowski_footprint);
    void computeObstacleLine(Vector2 &obst, int n);
    double vMaxAng();

    ros::Subscriber sac_cmd_vel_sub_[6];
    ros::Subscriber odom_sub_[6];
    ros::Subscriber rigid_sub_[6];
    ros::Subscriber scan_sub_[6];
    ros::Publisher cmd_vel_pub_[6];
    ros::Publisher obs_pub_[6];
    geometry_msgs::Twist sac_cmd_vel_[6];
    geometry_msgs::Twist new_cmd_vel_[6];
    std::vector<boost::shared_ptr<Agent>> agent;
    double min_vel_x_, max_vel_x_, min_vel_y_, max_vel_y_, max_vel_th_, min_vel_th_, min_vel_th_inplace_;
    double last_twist_ang_[6];
    double time_to_holo_;
    double min_error_holo_;
    double max_error_holo_;
    double footprint_radius_;
    double cur_loc_unc_radius_;
    bool sim_;
    geometry_msgs::PolygonStamped footprint_msg_;
    std::vector<std::pair<collvoid::Vector2, collvoid::Vector2> > footprint_lines_;
    bool has_polygon_footprint_;
    std::vector<Vector2> minkowski_footprint_;
    std::vector<Line> obs_lines_[6];
};

#endif
