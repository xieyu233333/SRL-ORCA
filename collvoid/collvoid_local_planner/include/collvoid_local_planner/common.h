#ifndef _COMMON_H
#define _COMMON_H

#include <tf/transform_listener.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

inline tf::Stamped<tf::Pose> geoposestamped_to_tfposestamped(const geometry_msgs::PoseStamped& geo_pose) {
    tf::Stamped<tf::Pose> tf_pose;
    tf_pose.setOrigin(tf::Vector3(geo_pose.pose.position.x, 
                                        geo_pose.pose.position.y, 
                                        geo_pose.pose.position.z));
    tf_pose.setRotation(tf::Quaternion(geo_pose.pose.orientation.x, 
                                            geo_pose.pose.orientation.y, 
                                            geo_pose.pose.orientation.z,
                                            geo_pose.pose.orientation.w));
    tf_pose.frame_id_ = geo_pose.header.frame_id;
    tf_pose.stamp_ = geo_pose.header.stamp;
    return tf_pose;
}

inline geometry_msgs::PoseStamped tfposestamped_to_geoposestamped(const tf::Stamped<tf::Pose>& tf_pose) {
    geometry_msgs::PoseStamped geo_pose;
    geo_pose.pose.position.x = tf_pose.getOrigin().x();
    geo_pose.pose.position.y = tf_pose.getOrigin().y();
    geo_pose.pose.position.z = tf_pose.getOrigin().z();
    geo_pose.pose.orientation.x = tf_pose.getRotation().x();
    geo_pose.pose.orientation.y = tf_pose.getRotation().y();
    geo_pose.pose.orientation.z = tf_pose.getRotation().z();
    geo_pose.pose.orientation.w = tf_pose.getRotation().w();
    geo_pose.header.frame_id = tf_pose.frame_id_;
    geo_pose.header.stamp = tf_pose.stamp_;
    return geo_pose;
}

inline void transform(const tf2_ros::Buffer &tf, 
    const tf::Stamped<tf::Pose> &in, 
    tf::Stamped<tf::Pose> &out, 
    const std::string &target_frame, 
    ros::Duration timeout = ((ros::Duration)((0.0)))) {
    auto in_ = tfposestamped_to_geoposestamped(in);
    geometry_msgs::PoseStamped out_;
    tf.transform(in_, out_, target_frame);
    out = geoposestamped_to_tfposestamped(out_);
}

#endif