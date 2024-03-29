#include <ros/ros.h>

// Moveit
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometric_shapes/shape_operations.h>

const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
 
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.5;
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  openGripper(grasps[0].pre_grasp_posture);

  closedGripper(grasps[0].grasp_posture);

  move_group.setSupportSurfaceName("table1");
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{

  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, tau / 4);  // A quarter turn about the z-axis
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  openGripper(place_location[0].post_place_posture);

  group.setSupportSurfaceName("table2");
  group.place("object", place_location);
}

void setupEnvironmentObjects(moveit::planning_interface::PlanningSceneInterface& psi)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4; 

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation = collision_objects[0].ADD;


  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4; 

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  collision_objects[2].id = "object";
  collision_objects[2].header.frame_id = "panda_link0";

  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2; 

  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;

  collision_objects[2].operation = collision_objects[2].ADD;

  psi.addCollisionObjects(collision_objects);

}


void setupEnvironmentObjects2(moveit::planning_interface::PlanningSceneInterface& psi)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  Eigen::Vector3d p(1.0, 1.0, 1.0);
  // shapes::Mesh* msh = shapes::createMeshFromResource("package://manipulator_sim/meshes/pod_lowres.stl",p);
  shapes::Mesh* msh = shapes::createMeshFromResource("package://manipulator_sim/meshes/mailbox.dae",p);

  shapes::ShapeMsg msh_msg;
  shapes::constructMsgFromShape(msh, msh_msg);
  shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(msh_msg);

  collision_objects[0].id = "table3";
  collision_objects[0].header.frame_id = "panda_link0";

  collision_objects[0].meshes.resize(1);
  collision_objects[0].meshes[0] = mesh;

  collision_objects[0].mesh_poses.resize(1);
  collision_objects[0].mesh_poses[0].position.x = 1.0;
  collision_objects[0].mesh_poses[0].position.y = 1.0;
  collision_objects[0].mesh_poses[0].position.z = 0.0;

  collision_objects[0].operation = collision_objects[0].ADD;

  psi.addCollisionObjects(collision_objects);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Pick-Place Initializing");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group_interface("panda_arm");
  group_interface.setPlanningTime(45.0);

  // For visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  //add collision objects
  setupEnvironmentObjects2(planning_scene_interface);
  
  // Prompt to start
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start");

  pick(group_interface);  
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
  place(group_interface);

  ros::waitForShutdown();
  return 0;
}