#include"std_msgs/String.h"
#include "ros/ros.h"

#include <sstream>

#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Model.h"

// Global Variables 
std::vector<osrf_gear::Order> order_vector; 
osrf_gear::LogicalCameraImage logcams;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void orderCallback(const osrf_gear::Order& msg)
{
  ROS_DEBUG("I heard: [%s]", msg.order_id.c_str());
  order_vector.push_back(msg);
}

void cameraCallback(const osrf_gear::LogicalCameraImage& msg)
{
  for(int camobs = 0; camobs < msg.models.size(); camobs++)
  {
    ROS_INFO("I heard a: [%s]", msg.models[camobs].type.c_str());
  }
  ROS_INFO("JEJEJEJ");
  
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

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Subscriber sub = n.subscribe("/ariac/orders", 1000, orderCallback);

  ros::Subscriber camera = n.subscribe("/ariac/logical_camera", 1000, cameraCallback);

  // Start Competition
  begin_client.call(begin_comp);

  ROS_INFO("Competition service returned: %s", begin_comp.response.message.c_str());

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
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
              ROS_INFO(logcams.models[camobs].type.c_str());
              ROS_INFO(curOrd.kits[index].objects[obs].type.c_str());
              if(logcams.models[camobs].type.c_str() == curOrd.kits[index].objects[obs].type.c_str())
              {
                itemIndex = camobs;
                ROS_INFO("FOUND MATCHING ITEM");
                break;
              }
            }
          }

        }
      }
      
      // Remove item from vector 
      order_vector.erase(order_vector.begin(), order_vector.begin() + 1);
    }
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
