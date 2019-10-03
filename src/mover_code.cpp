#include"std_msgs/String.h"
#include "ros/ros.h"

#include <sstream>

#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Model.h"

// MoveIt header files
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

// Transformation header files
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"



// Global Variables 
std::vector<osrf_gear::Order> order_vector; 
osrf_gear::LogicalCameraImage logcams;

void orderCallback(const osrf_gear::Order& msg)
{
  ROS_DEBUG("I heard the order: [%s]", msg.order_id.c_str());
  order_vector.push_back(msg);
}

void cameraCallback(const osrf_gear::LogicalCameraImage& msg)
{
  // !!! USES INCORRECT CAMERA SOMETIMES
  for(int camobs = 0; camobs < msg.models.size(); camobs++)
  {
    ROS_DEBUG("I heard the camera: [%s]", msg.models[camobs].type.c_str());
  }
  if(msg.models.size() > 4)
    logcams = msg;
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "mover");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  std_srvs::Trigger begin_comp;

  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition"); 

  ros::Subscriber sub = n.subscribe("/ariac/orders", 1000, orderCallback);

  ros::Subscriber camera = n.subscribe("/ariac/logical_camera", 1000, cameraCallback);

  ros::AsyncSpinner a = ros::AsyncSpinner(2); 

  a.start(); 

  // Start Competition
  begin_client.call(begin_comp);

  ROS_INFO("Competition service returned: %s", begin_comp.response.message.c_str());

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer); 
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  
  // Retrieve the transformation
  geometry_msgs::TransformStamped tfStamped;
  try{
//    tfStamped = tfBuffer.lookupTransform(move_group.getPlanningFrame(),"logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
    tfStamped = tfBuffer.lookupTransform("world","logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
    ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),tfStamped.child_frame_id.c_str());
  }
  catch(tf2::TransformException &ex)
  {
    ROS_ERROR("A %s", ex.what());
  }
//  tfBuffer.lookupTransform("logical_camera_frame", "world", ros::Time(0.0));

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();

    if(!order_vector.empty())
    {
      osrf_gear::Order curOrd;
      curOrd = order_vector.at(0);

      // Go through all kits
      for(int index = 0; index < curOrd.kits.size(); index++)
      {
        // Go through all objects 
        for(int obs = 0; obs < curOrd.kits[index].objects.size(); obs++)
        {
          ROS_DEBUG("I heard: [%s]", curOrd.kits[index].objects[obs].type.c_str());
      
          // Set up request for finding storage loc
          ros::ServiceClient client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
          osrf_gear::GetMaterialLocations srv;
          srv.request.material_type = curOrd.kits[index].objects[obs].type;

          // Request Storage Loc
          if (client.call(srv))
          {
            ROS_INFO("Object Found: %s", curOrd.kits[index].objects[obs].type.c_str());
            ROS_INFO("Location of Object: %s", srv.response.storage_units[0].unit_id.c_str());

            int itemIndex; 
            for(int camobs = 0; camobs < logcams.models.size(); camobs++)
            {
              ROS_INFO("%s", logcams.models[camobs].type.c_str());
              ROS_INFO(curOrd.kits[index].objects[obs].type.c_str());

              //!!! BROKEN COMPARISON!!!
              if(!strcmp(logcams.models[camobs].type.c_str(), curOrd.kits[index].objects[obs].type.c_str()))
              {
                itemIndex = camobs;
                ROS_INFO("FOUND MATCHING ITEM");
                break;
              }
            }
            ROS_INFO_STREAM("Pose: " << logcams.models[itemIndex].pose);

            geometry_msgs::TransformStamped transformStamped;

            geometry_msgs::PoseStamped part_pose, goal_pose;
            part_pose.pose = logcams.models[itemIndex].pose;
            tf2::doTransform(part_pose, goal_pose, transformStamped);

            // Add height to the goal pose.
            goal_pose.pose.position.z += 0.10; 
            // 10 cm above the part
            // Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...more on quaternions later in the semester).
            goal_pose.pose.orientation.w = 0.707;
            goal_pose.pose.orientation.x = 0.0;
            goal_pose.pose.orientation.y = 0.707;
            goal_pose.pose.orientation.z = 0.0;

            //Set the desired pose for the arm in the arm controller.
            move_group.setPoseTarget(goal_pose);
            
            moveit::planning_interface::MoveGroupInterface::Plan the_plan;
            
            // Create a plan based on the settings (all default settings now) in the_plan.
            bool success = (move_group.plan(the_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO("THERE");
            if(!success){
              ROS_INFO("Plan Failed... Trying Again");
              success = (move_group.plan(the_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            }
            ROS_INFO("Planning Successful!");

            // In the event that the plan was created, execute it.
            move_group.execute(the_plan);
          }

        }
      }
      
      // Remove item from vector 
      order_vector.erase(order_vector.begin(), order_vector.begin() + 1);
    }
  }

  return 0;
}
