#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure_example/ExampleConfig.h>

void callback(dynamic_reconfigure_example::ExampleConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %d %f %s %s %d",
            config.int_param,
            config.double_param,
            config.str_param.c_str(),
            config.bool_param ? "True" : "False",
            config.size);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_reconfigure_example");

  dynamic_reconfigure::Server<dynamic_reconfigure_example::ExampleConfig> server;
  dynamic_reconfigure::Server<dynamic_reconfigure_example::ExampleConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
