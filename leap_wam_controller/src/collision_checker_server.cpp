#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <leap_wam_controller/LeapGoalPose.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <trajectory_msgs/JointTrajectory.h>

bool check_for_collision(leap_wam_controller::LeapGoalPose::Request &req,
                         leap_wam_controller::LeapGoalPose::Response &res)
{
    ROS_INFO("Entered function");
    geometry_msgs::Pose target_pose = req.pose;
    std::cout << target_pose << std::endl;
    // double target_joint_poses[7];
    // for (int i=0; i < 7; i++)
    // {
    //     target_joint_poses[i] = req.positions[i];
    // }

    // std::cout << *target_joint_poses << std::endl;
    // This part is done by motion planning //
    
    // moveit::planning_interface::MoveGroup group("arm");
    // ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    // ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // group.setPoseTarget(target_pose);
    // ROS_INFO("Pose is set");
    // ROS_INFO("Pose Position x: %f, y:%f, z:%f Orientation x:%f, y:%f, z:%f, w:%f", target_pose.position.x,
    //   target_pose.position.y, target_pose.position.z, target_pose.orientation.x,
    //   target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
    // moveit::planning_interface::MoveGroup::Plan my_plan;
    // ROS_INFO("1");
    // bool success = group.plan(my_plan);
    // ROS_INFO("Planned");
    // res.result = success;
    // ROS_INFO("Planning: %s",success?"":"FAILED");

    // This part is done by checking state
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    ROS_INFO("[LEAP WAM Coll Checker] Model loaded");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    planning_scene::PlanningScene planning_scene(kinematic_model);

    robot_state::RobotState kinematic_state(kinematic_model);
    const robot_model::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup("arm");
    // kinematic_state.setToRandomPositions();
    bool is_state_set = kinematic_state.setFromIK(joint_model_group, target_pose, 5, 0.1);
    // kinematic_state.setJointGroupPositions(joint_model_group, target_joint_poses);
    std::cout << kinematic_state << std::endl;
    // std::cout << is_state_set << std::endl;

    // ROS_INFO("[LEAP WAM Coll Checker] State set.");
    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result, kinematic_state);
    res.result = collision_result.collision;
    ROS_INFO("[LEAP WAM Coll Checker] Result is set and sent.");
        // ROS_INFO("[LEAP WAM Coll Checker] Requested State is " << (collision_result.collision ? "in" : "not in") << " collision");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leap_wam_collision_checker_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("leap_wam_collision_checker_server", check_for_collision);
    ROS_INFO("[LEAP WAM] Collision checker is ready");
    ros::spin();

    return 0;
}
