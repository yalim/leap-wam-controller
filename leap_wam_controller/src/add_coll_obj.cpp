#include <ros/ros.h>

#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "addCylinder");

  ros::NodeHandle nh;

  ros::Publisher object_in_map_pub_;
  object_in_map_pub_  = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

  ros::Duration(2.0).sleep();// This delay is so critical, otherwise the first published object may not be added in the collision_space by the environment_server

  //add the cylinder into the collision space
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "iri_wam_base_link";

  /* The id of the object is used to identify it. */
  collision_object.id = "box1";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.44;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  object_in_map_pub_.publish(collision_object);

  ROS_INFO("Should have published");

  ros::Duration(2.0).sleep();

  ros::shutdown();
}