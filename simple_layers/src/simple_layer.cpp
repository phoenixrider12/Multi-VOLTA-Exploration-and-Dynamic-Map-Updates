#include "simple_layers/simple_layer.h"
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32MultiArray.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {}

float oldX = 0;
float oldY = 0;
float oldLength = 0;
float oldWidth = 0;
float newX = 0;
float newY = 0;
float newLength = 0;
float newWidth = 0;

void SimpleLayer::formationCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){

  ROS_INFO("Receiving points from callback");

    newX = msg->data[0];
    newY = msg->data[1];
    newLength = msg->data[2];
    newWidth = msg->data[3];

    ROS_INFO("Callback: %f %f %f %f", newX, newY, newLength, newWidth);
}

void SimpleLayer::onInitialize()
{
//   ros::NodeHandle nh("~/" + name_);
  current_ = true;
  sub = nh.subscribe("/objectron", 1000, &SimpleLayer::formationCallback, this);

  ROS_INFO("Initialize: %f %f %f %f", newX, newY, newLength, newWidth);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

}

void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

float distance = 0;
int updated = 1;
ros::Time start_time;
ros::Time time;

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
    ROS_INFO("Updating costs");
    unsigned int mx;
    unsigned int my;

      ROS_INFO("Update Costs: %f %f %f %f", newX, newY, newLength, newWidth);
      distance = sqrt(pow(newX - oldX, 2) + pow(newY - oldY, 2));
      ROS_INFO("Distance: %f", distance);

      if (distance > 0.5)
      {
         if (updated == 1)
         {
             start_time = ros::Time::now();
         }
         time = ros::Time::now();

         if (time - start_time > ros::Duration(60))
         {
            for (float i = (newX - newLength/2); i < (newX + newLength/2); i += 0.001)
            {
                for (float j = (newY - newWidth/2); j < (newY + newWidth/2); j += 0.001)
                {
                    if(master_grid.worldToMap(i, j, mx, my))
                    {
                        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
                    }
                }
            }

            for (float i = (oldX - oldLength/2); i < (oldX + oldLength/2); i += 0.001)
            {
                for (float j = (oldY - oldWidth/2); j < (oldY + oldWidth/2); j += 0.001)
                {
                    if(master_grid.worldToMap(i, j, mx, my))
                    {
                        master_grid.setCost(mx, my, FREE_SPACE);
                    }
                }
            }

            oldX = newX;
            oldY = newY;
            oldLength = newLength;
            oldWidth = newWidth;
	        updated = 1;
         }

         else
         {
             time = ros::Time::now();
             updated = 0;
         }
      }

      else
      {
          for (float i = (oldX - oldLength/2); i < (oldX + oldLength/2); i += 0.001)
          {
              for (float j = (oldY - oldWidth/2); j < (oldY + oldWidth/2); j += 0.001)
              {
                  if(master_grid.worldToMap(i, j, mx, my))
                  {
                      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
                  }
              }
          }
      }
}

}
