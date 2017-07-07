#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

std::vector<moveit_msgs::CollisionObject> defineFrame(moveit::planning_interface::MoveGroup &group)
{
    //Add the Collision Frame to the world
    moveit_msgs::CollisionObject ceiling;
  	moveit_msgs::CollisionObject pillar1;
  	moveit_msgs::CollisionObject pillar2;
  	moveit_msgs::CollisionObject pillar3;
  	moveit_msgs::CollisionObject pillar4;
  	moveit_msgs::CollisionObject wall1;
  	moveit_msgs::CollisionObject wall2;
  	moveit_msgs::CollisionObject wall3;
  	moveit_msgs::CollisionObject wall4;
  	moveit_msgs::CollisionObject workspace;
  	
    ceiling.header.frame_id = group.getPlanningFrame();
  	pillar1.header.frame_id = group.getPlanningFrame();
  	pillar2.header.frame_id = group.getPlanningFrame();
  	pillar3.header.frame_id = group.getPlanningFrame();
  	pillar4.header.frame_id = group.getPlanningFrame();
  	wall1.header.frame_id = group.getPlanningFrame();
  	wall2.header.frame_id = group.getPlanningFrame();
  	wall3.header.frame_id = group.getPlanningFrame();
  	wall4.header.frame_id = group.getPlanningFrame();
  	workspace.header.frame_id = group.getPlanningFrame();

  	// The id of the object is used to identify it.
  	ceiling.id = "Ceiling";
  	pillar1.id = "Pillar1";
  	pillar2.id = "Pillar2";
  	pillar3.id = "Pillar3";
  	pillar4.id = "Pillar4";
    wall1.id = "Wall1";
    wall2.id = "Wall2";
    wall3.id = "Wall3";
    wall4.id = "Wall4";
  	workspace.id = "Workspace";

  	// Define ceiling/floor size 
  	shape_msgs::SolidPrimitive floors;
 	floors.type = floors.BOX;
  	floors.dimensions.resize(3);
  	floors.dimensions[0] = 2.0;
  	floors.dimensions[1] = 2.0;
  	floors.dimensions[2] = 0.01;

  	// Define pillar size
  	shape_msgs::SolidPrimitive pillars;
 	pillars.type = pillars.BOX;
  	pillars.dimensions.resize(3);
  	pillars.dimensions[0] = 0.04;
  	pillars.dimensions[1] = 0.04;
  	pillars.dimensions[2] = 1.43;

    // Define Top/Bottom Walls
  	shape_msgs::SolidPrimitive wallUT;
 	wallUT.type = wallUT.BOX;
  	wallUT.dimensions.resize(3);
  	wallUT.dimensions[0] = 2.0;
  	wallUT.dimensions[1] = 0.01;
  	wallUT.dimensions[2] = 0.055;

    // Define Left/Right Walls
  	shape_msgs::SolidPrimitive wallLR;
 	wallLR.type = wallLR.BOX;
  	wallLR.dimensions.resize(3);
  	wallLR.dimensions[0] = 0.01;
  	wallLR.dimensions[1] = 2.0;
  	wallLR.dimensions[2] = 0.055;

  	// A pose for the ceiling
  	geometry_msgs::Pose ceiling_pose;
  	ceiling_pose.position.z = 0.01;
  	
	// A pose for the workspace
  	geometry_msgs::Pose workspace_pose;
  	workspace_pose.position.z = -1.425;
  	
	// Pillar Poses
  	geometry_msgs::Pose pillar1_pose;
  	pillar1_pose.position.x = 0.98;
  	pillar1_pose.position.y = 0.98;
  	pillar1_pose.position.z = -0.7075;
  	geometry_msgs::Pose pillar2_pose;
  	pillar2_pose.position.x = -0.98;
  	pillar2_pose.position.y = 0.98;
  	pillar2_pose.position.z = -0.7075;
  	geometry_msgs::Pose pillar3_pose;
  	pillar3_pose.position.x = 0.98;
  	pillar3_pose.position.y = -0.98;
  	pillar3_pose.position.z = -0.7075;
  	geometry_msgs::Pose pillar4_pose;
  	pillar4_pose.position.x = -0.98;
  	pillar4_pose.position.y = -0.98;
  	pillar4_pose.position.z = -0.7075;

    // Wall Poses
    geometry_msgs::Pose wall1_pose;
    wall1_pose.position.x = 0;
    wall1_pose.position.y = 0.995;
    wall1_pose.position.z = -1.3975;
    geometry_msgs::Pose wall2_pose;
    wall2_pose.position.x = 0;
    wall2_pose.position.y = -0.995;
    wall2_pose.position.z = -1.3975;
    geometry_msgs::Pose wall3_pose;
    wall3_pose.position.x = 0.995;
    wall3_pose.position.y = 0;
    wall3_pose.position.z = -1.3975;
    geometry_msgs::Pose wall4_pose;
    wall4_pose.position.x = -0.995;
    wall4_pose.position.y = 0;
    wall4_pose.position.z = -1.3975;


    // Add pose and size and define add bit //
    ceiling.primitives.push_back(floors);
  	ceiling.primitive_poses.push_back(ceiling_pose);
  	ceiling.operation = ceiling.ADD;
    workspace.primitives.push_back(floors);
  	workspace.primitive_poses.push_back(workspace_pose);
  	workspace.operation = workspace.ADD;
    pillar1.primitives.push_back(pillars);
  	pillar1.primitive_poses.push_back(pillar1_pose);
  	pillar1.operation = pillar1.ADD;
    pillar2.primitives.push_back(pillars);
  	pillar2.primitive_poses.push_back(pillar2_pose);
  	pillar2.operation = pillar2.ADD;
    pillar3.primitives.push_back(pillars);
  	pillar3.primitive_poses.push_back(pillar3_pose);
  	pillar3.operation = pillar3.ADD;
    pillar4.primitives.push_back(pillars);
  	pillar4.primitive_poses.push_back(pillar4_pose);
  	pillar4.operation = pillar4.ADD;
    wall1.primitives.push_back(wallUT);
  	wall1.primitive_poses.push_back(wall1_pose);
  	wall1.operation = wall1.ADD;
    wall2.primitives.push_back(wallUT);
  	wall2.primitive_poses.push_back(wall2_pose);
  	wall2.operation = wall2.ADD;
    wall3.primitives.push_back(wallLR);
  	wall3.primitive_poses.push_back(wall3_pose);
  	wall3.operation = wall3.ADD;
    wall4.primitives.push_back(wallLR);
  	wall4.primitive_poses.push_back(wall4_pose);
  	wall4.operation = wall4.ADD;
  	
    std::vector<moveit_msgs::CollisionObject> collision_objects;  
  	collision_objects.push_back(ceiling);  
  	collision_objects.push_back(workspace);  
  	collision_objects.push_back(pillar1);  
  	collision_objects.push_back(pillar2);  
  	collision_objects.push_back(pillar3);  
  	collision_objects.push_back(pillar4);  
  	collision_objects.push_back(wall1);  
  	collision_objects.push_back(wall2);  
  	collision_objects.push_back(wall3);  
  	collision_objects.push_back(wall4);  

    return collision_objects;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "addCollisionFrame");
  	ros::NodeHandle node_handle;
  	ros::AsyncSpinner spinner(1);
  	spinner.start();

  	sleep(5.0);

    //URDF defines 'Arm' as the planning group of the UR10
    //Gripper as the planning group of the Robotiq
  	moveit::planning_interface::MoveGroup group("Arm");
  	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //Define Collision Objects
    std::vector<moveit_msgs::CollisionObject> collision_objects = defineFrame(group);
  	ROS_INFO("Adding Collision Frame into the world");  

    //NECESSARY OTHERWISE OBJECTS WILL NOT BE ADDED
  	sleep(2.0);
  	// Now, let's add the collision object into the world
  	planning_scene_interface.addCollisionObjects(collision_objects);
  
  	// Sleep so we have time to see the object in RViz
  	sleep(2.0);
  	ROS_INFO("Collision Frame Defined! You can now plan around the frame.");  

  	ros::shutdown();
  	return 0;
}
