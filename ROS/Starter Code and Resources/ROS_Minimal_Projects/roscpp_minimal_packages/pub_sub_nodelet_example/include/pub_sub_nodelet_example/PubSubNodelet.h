#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>

namespace pub_sub_nodelet_example
{
  class PubSubNodelet : public nodelet::Nodelet
  {
  protected:
    ros::NodeHandle nh;

    ros::Publisher pub;
    ros::Subscriber sub;

    std_msgs::String message;
    unsigned int count = 0;

  public:
    virtual void onInit();
    void msgCB(const std_msgs::StringConstPtr& msg);
  };
}
