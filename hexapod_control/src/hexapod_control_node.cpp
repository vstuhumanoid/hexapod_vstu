#include "hexapod_controller.hpp"

HexapodController *controller;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hexapod_controller");
  ros::NodeHandle nh;

  controller = new HexapodController(nh);

  while(ros::ok())
  {
    controller->FrundExchange();
    ros::spinOnce();
  }

  return 0;
}