#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometric_shapes/shape_operations.h>
#include <gazebo_msgs/SpawnModel.h>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include <moveit_visual_tools/moveit_visual_tools.h>

void setupEnvironmentObjects(moveit::planning_interface::PlanningSceneInterface& psi)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.3;
  collision_objects[0].primitives[0].dimensions[1] = 0.5;
  collision_objects[0].primitives[0].dimensions[2] = 0.4; 

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation = collision_objects[0].ADD;

  collision_objects[1].id = "object";
  collision_objects[1].header.frame_id = "panda_link0";

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.3;
  collision_objects[1].primitives[0].dimensions[1] = 0.02;
  collision_objects[1].primitives[0].dimensions[2] = 0.2; 

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.5;
  collision_objects[1].primitive_poses[0].position.y = 0;
  collision_objects[1].primitive_poses[0].position.z = 0.5;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  psi.addCollisionObjects(collision_objects);
}


void setupEnvironmentObjects2(moveit::planning_interface::PlanningSceneInterface& psi)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  Eigen::Vector3d p(1.0, 1.0, 1.0);
  // shapes::Mesh* msh = shapes::createMeshFromResource("package://manipulator_sim/meshes/pod_lowres.stl",p);
  shapes::Mesh* msh = shapes::createMeshFromResource("file:///home/senthu/Gazebo_models/gazebo_models/cinder_block_2/meshes/cinder_block.dae",p);

  ///home/senthu/Gazebo_models/gazebo_models/cinder_block_2/meshes

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

void setupPlanningSceneObject(moveit::planning_interface::PlanningSceneInterface& psi,
                              std::string file_path,
                              geometry_msgs::Pose pose,
                              std::string reference_frame, 
                              std::string name,
                              double scale)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  Eigen::Vector3d p(scale, scale, scale);
  shapes::Mesh* msh = shapes::createMeshFromResource(file_path, p);

  shapes::ShapeMsg msh_msg;
  shapes::constructMsgFromShape(msh, msh_msg);
  shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(msh_msg);

  collision_objects[0].id = name;
  collision_objects[0].header.frame_id = reference_frame;

  collision_objects[0].meshes.resize(1);
  collision_objects[0].meshes[0] = mesh;

  collision_objects[0].mesh_poses.resize(1);
  collision_objects[0].mesh_poses[0] = pose;

  collision_objects[0].operation = collision_objects[0].ADD;

  psi.addCollisionObjects(collision_objects);
}

void spawnModelGazebo(ros::ServiceClient& sc, 
                      std::string file_path,
                      geometry_msgs::Pose pose,
                      std::string reference_frame, 
                      std::string name)
{
  gazebo_msgs::SpawnModel srv;
  srv.request.initial_pose = pose;
  srv.request.reference_frame = reference_frame;
  srv.request.model_name = name;
  
  std::ifstream file(file_path);
  if(!file.is_open())
  {
    ROS_WARN("spawnModelGazebo: could not read the model: %s", file_path.c_str());
    return;
  }
  std::string fileContent((std::istreambuf_iterator<char>(file)),
                            std::istreambuf_iterator<char>());
  file.close();

  srv.request.model_xml = fileContent;
  
  ROS_INFO("Going to call spawn service in Gazebo with model: %s", file_path.c_str());

  if(sc.call(srv))
  {
    ROS_INFO("Spawning Success");
  }else
  {
    ROS_INFO("Spawning failed");
  }
  
  return;
}

void setupGazeboEnvironment(moveit::planning_interface::PlanningSceneInterface& psi, 
                            ros::ServiceClient& sc, std::string reference_frame)
{
  std::string dir_path = std::string(getenv("HOME")) + "/Gazebo_models/gazebo_models";

  // Object 1
  std::string file_dir_path = dir_path + "/cinder_block_2";

  std::string file_sdf_path = file_dir_path + "/model.sdf";
  std::string file_mesh_path = "file://" + file_dir_path + "/meshes/cinder_block.dae";

  geometry_msgs::Pose obj_pose;
  obj_pose.orientation.w = 1.0;
  obj_pose.position.x = 2.0;
  obj_pose.position.y = 2.0;
  obj_pose.position.z = 0.0;

  spawnModelGazebo(sc, file_sdf_path, obj_pose, reference_frame, "model1");
  setupPlanningSceneObject(psi, file_mesh_path, obj_pose, reference_frame, "model1", 1.0);

  obj_pose.orientation.w = 1.0;
  obj_pose.position.x = 1.0;
  obj_pose.position.y = 2.0;
  obj_pose.position.z = 0.0;

  spawnModelGazebo(sc, file_sdf_path, obj_pose, reference_frame, "model2");
  setupPlanningSceneObject(psi, file_mesh_path, obj_pose, reference_frame, "model2", 1.0);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_planning");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    double goal_x, goal_y, goal_z;
    pnh.param("goal_x", goal_x, 0.5);
    pnh.param("goal_y", goal_y, 0.0);
    pnh.param("goal_z", goal_z, 0.78);

    ROS_INFO("goalx: %f, goaly: %f, goalz: %f", goal_x, goal_y, goal_z);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient spawnModelGazeboClient = nh.serviceClient<gazebo_msgs::SpawnModel>(
                                                                "/gazebo/spawn_sdf_model");

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

    setupGazeboEnvironment(planning_scene_interface, spawnModelGazeboClient, 
                            move_group_interface.getPlanningFrame());

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start");

    // Start the Planning
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = goal_x;
    goal_pose.position.y = goal_y;
    goal_pose.position.z = goal_z;

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