#include <ros/ros.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure_example/ExampleConfig.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_reconfigure_client");

  ros::NodeHandle nh;

  // Remember to input the name of the server that the
  // dynamically reconfigurable node is hosting!
  ros::ServiceClient client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/dynamic_reconfigure_example/set_parameters");

  // Create service message
  dynamic_reconfigure::Reconfigure service;

  // Create dependent messages
  dynamic_reconfigure::IntParameter int_param;
  int_param.name = "int_param";
  int_param.value = 0;

  dynamic_reconfigure::DoubleParameter double_param;
  double_param.name = "double_param";
  double_param.value = 1.0;

  dynamic_reconfigure::StrParameter str_param;
  str_param.name = "str_param";
  str_param.value = "Rawr";

  dynamic_reconfigure::BoolParameter bool_param;
  bool_param.name = "bool_param";
  bool_param.value = true;

  dynamic_reconfigure::IntParameter size;
  size.name = "size";
  size.value = 0;

  // Populate request
  service.request.config.ints.push_back(int_param);
  service.request.config.doubles.push_back(double_param);
  service.request.config.strs.push_back(str_param);
  service.request.config.bools.push_back(bool_param);
  service.request.config.ints.push_back(size);

  // Service callback function
  // client.call(service) calls the client!
  if (client.call(service))
  {
    ROS_INFO("Reconfigure call succeeded!");
  }
  else
  {
    ROS_INFO("Reconfigure call failed!");
    return 1;
  }

  return 0;
}
