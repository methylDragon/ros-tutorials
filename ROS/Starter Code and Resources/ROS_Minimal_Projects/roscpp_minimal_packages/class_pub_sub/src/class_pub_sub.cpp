#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

class ClassPubSub
{
protected:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Rate r;

  std_msgs::String message;
  unsigned int count = 0;

public:
  // Constructor
  ClassPubSub()
  : r(25)
  {
    sub = nh.subscribe("chatter", 1, &ClassPubSub::msgCB, this);
    pub = nh.advertise<std_msgs::String>("chatter", 1);

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

  // We pass a pointer here to prevent copying
  // The access operator has to change, of course
  void msgCB(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO_STREAM(msg->data);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "class_pub_sub");
  ClassPubSub();
}
