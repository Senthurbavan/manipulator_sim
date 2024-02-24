#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_planning");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group = 
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO("Planning Frame: %s, EE Link: %s", move_group_interface.getPlanningFrame().c_str(),
                                                move_group_interface.getEndEffectorLink().c_str());

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    setupEnvironmentObjects(planning_scene_interface);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start");

    // Start the Planning
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = 0.5;
    goal_pose.position.y = 0.0;
    goal_pose.position.z = 0.78;

    tf2::Quaternion q;
    q.setRPY(0, M_PI, 3.0*M_PI_4);
    geometry_msgs::Quaternion qm = tf2::toMsg(q);
    goal_pose.orientation = qm;

    move_group_interface.setPoseTarget(goal_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;

    moveit::core::MoveItErrorCode res = move_group_interface.plan(plan1);
    ROS_INFO("Planning %s", res == moveit::core::MoveItErrorCode::SUCCESS ? "Success" : "Fail");
    ROS_INFO("Going to execute the trajectory...");

    res = move_group_interface.execute(plan1);
    ROS_INFO("Trajectory Execution %s", res == moveit::core::MoveItErrorCode::SUCCESS ? "Success" : "Fail");

    ros::waitForShutdown();
    return 0;
}