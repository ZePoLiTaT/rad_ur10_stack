#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

std::vector<moveit_msgs::CollisionObject> defineFrame(moveit::planning_interface::MoveGroup &group)
{
    int invert_z = -1;

    //Add the Collision Frame to the world
  	moveit_msgs::CollisionObject box;
  
  	box.header.frame_id = group.getPlanningFrame();

  	// The id of the object is used to identify it.
  	box.id = "tray";

    // Define ceiling/floor size 
    shape_msgs::SolidPrimitive floors;
    floors.type = floors.BOX;
    floors.dimensions.resize(3);
    floors.dimensions[0] = 1.50;
    floors.dimensions[1] = 1.50;
    floors.dimensions[2] = 0.30;

	// A pose for the box
  	geometry_msgs::Pose box_pose;
  	box_pose.position.z = -1.370 * invert_z - floors.dimensions[2]/2.0;

    box.primitives.push_back(floors);
  	box.primitive_poses.push_back(box_pose);
  	box.operation = box.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;  
  	collision_objects.push_back(box);  


    return collision_objects;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "addTray");
  	ros::NodeHandle node_handle;
  	ros::AsyncSpinner spinner(1);
  	spinner.start();

  	sleep(5.0);

    //URDF defines 'Arm' as the planning group of the UR10
    //Gripper as the planning group of the Robotiq
  	//moveit::planning_interface::MoveGroup group("Arm");
    moveit::planning_interface::MoveGroup group("manipulator");
  	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //Define Collision Objects
    std::vector<moveit_msgs::CollisionObject> collision_objects = defineFrame(group);
  	ROS_INFO("Adding Tray Shape into the world");  

    //NECESSARY OTHERWISE OBJECTS WILL NOT BE ADDED
  	sleep(2.0);
  	// Now, let's add the collision object into the world
  	planning_scene_interface.addCollisionObjects(collision_objects);
  
  	// Sleep so we have time to see the object in RViz
  	sleep(2.0);
  	ROS_INFO("Collision Frame Defined! You can now plan around the tray.");  

  	ros::shutdown();
  	return 0;
}
