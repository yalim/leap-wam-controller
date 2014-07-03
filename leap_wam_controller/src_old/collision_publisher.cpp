#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Bool.h>

/*TODO: Change the name 'wam' if necessary*/

bool userCallback(const robot_state::RobotState &kinematic_state, bool verbose)
{
  const double* joint_values = kinematic_state.getJointPositions("r_shoulder_pan_joint");
  return (joint_values[0] > 0.0);
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "leap_wam_collision_publisher");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene.getCurrentState();

    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    for(it2 = collision_result.contacts.begin();
        it2 != collision_result.contacts.end();
        ++it2)
    {
      acm.setEntry(it2->first.first, it2->first.second, true);
    }
    ros::Rate loop_rate(10);
    while (ros::ok())
    {

        collision_result.clear();
        planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);

        // TODO: Publish (collision_result.collision : True: Collision False: Not Collision)
        ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("leap_wam/collision_result",1);
        std_msgs::Bool col_result;
        col_result.data = collision_result.collision;
        ROS_INFO("[LEAP_WAM COLL CHECKER]: %s",col_result.data ? "True" : "False");

        chatter_pub.publish(col_result);
        ros::spin();
        loop_rate.sleep();
    }

    return 0;
}