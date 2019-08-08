#include <pluginlib/class_list_macros.h>
#include "pub_sub_nodelet_example/PubSubNodelet.h"

ros::Rate r = ros::Rate(25);

namespace pub_sub_nodelet_example
{
  void PubSubNodelet::onInit()
  {
    // Init nodehandle
    nh = getNodeHandle();

    // Bind subscription topic
    sub = nh.subscribe("chatter", 1, &PubSubNodelet::msgCB, this);
    pub = nh.advertise<std_msgs::String>("chatter", 1);

    NODELET_INFO("NODELET READY!");

    while (ros::ok())
    {
      std::ostringstream string_count;
      string_count << count;
      message.data = "Rawr " + string_count.str();

      pub.publish(message);
      ros::spinOnce();
      r.sleep();

      count++;
    }
  }

  void PubSubNodelet::msgCB(const std_msgs::StringConstPtr& msg)
  {
    NODELET_INFO_STREAM(msg->data);
  }
}

// Export the nodelet as a plugin
PLUGINLIB_EXPORT_CLASS(pub_sub_nodelet_example::PubSubNodelet, nodelet::Nodelet)
