#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometric_shapes/shape_operations.h>
#include <gazebo_msgs/SpawnModel.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include <fstream>

static const std::string PLANNING_GROUP = "panda_arm";


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
  obj_pose.position.x = 0.2;
  obj_pose.position.y = 0.0;
  obj_pose.position.z = 0.0;

  spawnModelGazebo(sc, file_sdf_path, obj_pose, reference_frame, "cinder_block");
  setupPlanningSceneObject(psi, file_mesh_path, obj_pose, reference_frame, "cinder_block", 1.0);

  // Object 2
  file_dir_path = dir_path + "/cafe_table";
  file_sdf_path = file_dir_path + "/model.sdf";
  file_mesh_path = "file://" + file_dir_path + "/meshes/cafe_table.dae";

  obj_pose.orientation.w = 1.0;
  obj_pose.position.x = 0.85;
  obj_pose.position.y = 0.0;
  obj_pose.position.z = 0.0;

  spawnModelGazebo(sc, file_sdf_path, obj_pose, reference_frame, "cafe_table");
  setupPlanningSceneObject(psi, file_mesh_path, obj_pose, reference_frame, "cafe_table", 0.0254);

}

/*********************************************
********** Currently Used Functions **********
**********************************************/


void setupPlanningScene1(moveit::planning_interface::PlanningSceneInterface& psi, 
                              std::vector<geometry_msgs::Pose>& waypoints)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.5;
  collision_objects[0].primitives[0].dimensions[1] = 0.75;
  collision_objects[0].primitives[0].dimensions[2] = 0.4; 

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.525;
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
  collision_objects[1].primitives[0].dimensions[2] = 0.3; 

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.525;
  collision_objects[1].primitive_poses[0].position.y = 0;
  collision_objects[1].primitive_poses[0].position.z = 0.5;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  psi.addCollisionObjects(collision_objects);

  // std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = 0.4;
  goal_pose.position.y = -0.375;
  goal_pose.position.z = 0.55;

  tf2::Quaternion q;
  q.setRPY(0, M_PI, 3.0*M_PI_4);
  geometry_msgs::Quaternion qm = tf2::toMsg(q);
  goal_pose.orientation = qm;
  waypoints.push_back(goal_pose);

  goal_pose.position.x += 0.125;
  goal_pose.position.y += 0.575;
  waypoints.push_back(goal_pose);

  return;
}

void setupPlanningScene2(moveit::planning_interface::PlanningSceneInterface& psi, 
                              std::vector<geometry_msgs::Pose>& waypoints)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  collision_objects[0].id = "box_front";
  collision_objects[0].header.frame_id = "panda_link0";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.02;
  collision_objects[0].primitives[0].dimensions[1] = 0.5;
  collision_objects[0].primitives[0].dimensions[2] = 0.3; 

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.4;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.15;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation = collision_objects[0].ADD;


  collision_objects[1].id = "box_left";
  collision_objects[1].header.frame_id = "panda_link0";

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.36; //0.4 - 2*0.02
  collision_objects[1].primitives[0].dimensions[1] = 0.02;
  collision_objects[1].primitives[0].dimensions[2] = 0.3; 

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.59;
  collision_objects[1].primitive_poses[0].position.y = -0.24;
  collision_objects[1].primitive_poses[0].position.z = 0.15;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  collision_objects[2].id = "box_right";
  collision_objects[2].header.frame_id = "panda_link0";

  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.36; //0.4 - 2*0.02
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.3; 

  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.59;
  collision_objects[2].primitive_poses[0].position.y = 0.24;
  collision_objects[2].primitive_poses[0].position.z = 0.15;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;

  collision_objects[2].operation = collision_objects[2].ADD;

  collision_objects[3].id = "shelf_top";
  collision_objects[3].header.frame_id = "panda_link0";

  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.5;
  collision_objects[3].primitives[0].dimensions[1] = 0.3;
  collision_objects[3].primitives[0].dimensions[2] = 0.02; 

  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.3;
  collision_objects[3].primitive_poses[0].position.y = 0.75;
  collision_objects[3].primitive_poses[0].position.z = 0.5;
  collision_objects[3].primitive_poses[0].orientation.w = 1.0;

  collision_objects[3].operation = collision_objects[3].ADD;

  psi.clear();
  psi.addCollisionObjects(collision_objects);

  // std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = 0.6;
  goal_pose.position.y = -0.05;
  goal_pose.position.z = 0.1;

  tf2::Quaternion q;
  q.setRPY(0, M_PI, 3.0*M_PI_4);
  geometry_msgs::Quaternion qm = tf2::toMsg(q);
  goal_pose.orientation = qm;
  waypoints.push_back(goal_pose);

  goal_pose.position.x = 0.286;
  goal_pose.position.y = 0.66;
  goal_pose.position.z = 0.117;
  goal_pose.orientation.w = 0.383;
  goal_pose.orientation.x = -0.333;
  goal_pose.orientation.y = 0.676;
  goal_pose.orientation.z = 0.535;
  waypoints.push_back(goal_pose);

  return;
}

void printPlannerParams(moveit::planning_interface::MoveGroupInterface& mgi)
{
  ROS_INFO(" ");
  ROS_INFO("======Printing params========");

  ROS_INFO("Planning pipeline: %s", mgi.getPlanningPipelineId().c_str());
  ROS_INFO("Planner: %s", mgi.getPlannerId().c_str());
  ROS_INFO("Goal Position Tolerance: %f", mgi.getGoalPositionTolerance());
  ROS_INFO("Goal Orientation Tolerance: %f", mgi.getGoalOrientationTolerance());
  ROS_INFO("Planning Time: %f", mgi.getPlanningTime());

  ROS_INFO("%s Params", mgi.getPlannerId().c_str());
  std::map<std::string, std::string> planner_params = mgi.getPlannerParams(mgi.getPlannerId(), 
                                                                           PLANNING_GROUP);
  for(const auto& param : planner_params)
  {
    ROS_INFO("-%s: %s", param.first.c_str(), param.second.c_str());
  }
  
  ROS_INFO("=============================");
  ROS_INFO(" ");
}

void setPlannerParams(moveit::planning_interface::MoveGroupInterface& mgi)
{
  std::string planner_id; 
  std::map<std::string, std::string> planner_params;
  
  // Global Parameters
  mgi.setPlanningPipelineId("ompl"); //pipeline options: "ompl", "chomp", "pilz_industrial_motion_planner"
  mgi.setNumPlanningAttempts(4); // default: 1
  mgi.setMaxVelocityScalingFactor(0.1); // default: 0.1 values: 0.0 - 1.0
  mgi.setMaxAccelerationScalingFactor(0.1); // default: 0.1 values: 0.0 - 1.0
  mgi.setGoalPositionTolerance(0.0001); // default: 1e-4
  mgi.setGoalOrientationTolerance(0.001); // default: 1e-3
  mgi.setPlanningTime(5.0); // in seconds
  planner_params["longest_valid_segment_fraction"] = "0.005";
  planner_params["projection_evaluator"]           = "joints(panda_joint1,panda_joint2)";

  //Planner Specific Parameters

  // // LBKPIECE
  // planner_id = "LBKPIECE";
  // planner_params["border_fraction"]         = "0.9";
  // planner_params["min_valid_path_fraction"] = "0.5";
  // planner_params["range"]                   = "1";

  // //RRT
  // planner_id = "RRT";
  // planner_params["goal_bias"] = "0.005";
  // planner_params["range"]     = "1";

  //RRTConnect
  planner_id = "RRTConnect";
  planner_params["range"] = "0";
  
  mgi.setPlannerId(planner_id);
  mgi.setPlannerParams(planner_id, PLANNING_GROUP, planner_params, false);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_planning");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient spawnModelGazeboClient = nh.serviceClient<gazebo_msgs::SpawnModel>(
                                                                "/gazebo/spawn_sdf_model");

    // static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group = 
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO("Planning Frame: %s, EE Link: %s", move_group_interface.getPlanningFrame().c_str(),
                                                move_group_interface.getEndEffectorLink().c_str());

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");


    std::vector<geometry_msgs::Pose> waypoints;
    
    // setupPlanningScene2(planning_scene_interface, waypoints);
    setupGazeboEnvironment(planning_scene_interface, spawnModelGazeboClient, "panda_link0");

    printPlannerParams(move_group_interface);
    setPlannerParams(move_group_interface);
    printPlannerParams(move_group_interface);

    // Visualize the waypoints
    visual_tools.deleteAllMarkers();
    for(geometry_msgs::Pose point : waypoints)
    {
      visual_tools.publishSphere(point, rvt::YELLOW, rvt::XLARGE);
    }
    visual_tools.trigger();
    visual_tools.loadRemoteControl();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start");

    bool success = true;
    
    // Plan and Execute 
    for(int i=0; i<waypoints.size(); i++)
    {

      moveit::core::RobotState current_state(*move_group_interface.getCurrentState());
      move_group_interface.setStartState(current_state);

      geometry_msgs::Pose pose = waypoints[i];
      move_group_interface.setPoseTarget(pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan1;

      moveit::core::MoveItErrorCode res = move_group_interface.plan(plan1);
      success = res == moveit::core::MoveItErrorCode::SUCCESS ? true : false;
      if(success)
      {
        ROS_INFO("Planning to waypoint %d is Success with planning time: %f seconds", i, plan1.planning_time_);
      }
      else{
        ROS_INFO("Planning to waypoint %d Failed", i);
        break;
      }

      visual_tools.publishTrajectoryLine(plan1.trajectory_, joint_model_group);
      visual_tools.trigger();

      ROS_INFO("Going to execute the trajectory %d...", i);
      res = move_group_interface.execute(plan1);
      success = res == moveit::core::MoveItErrorCode::SUCCESS ? true : false;
      if(success)
      {
        ROS_INFO("Trajectory execution to waypoint %d is Success", i);
      }
      else{
        ROS_INFO("Trajectory execution to waypoint %d Failed", i);
        break;
      }
    }
    printPlannerParams(move_group_interface);    
    if(success)
    {
      ROS_INFO("Final goal reached !!!");
    }else
    {
      ROS_INFO("Failed to reach the final goal!!!");
    }

    ros::waitForShutdown();
    return 0;
}