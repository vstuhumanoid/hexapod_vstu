#ifndef HEXAPOD_CONTROLLER_H
#define HEXAPOD_CONTROLLER_H

#include <experimental/optional>
#include <string>
#include <atomic>
#include <thread>

#include "asio.hpp"

using asio::ip::udp;

// Server side implementation of UDP client-server model
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <map>
#include <sstream>
#include <string>
#include <vector>


#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "hexapod_msgs/FrundControl.h"

#include "frund_gateway.hpp"

struct JointParams 
{
  int number;
  bool reverce;
};

class HexapodController
{
private:
    FrundGateway *gateway;
    std::thread *thread;

    ros::NodeHandle &_nh;
    std::map<std::string, int> jointNumbers;
    std::map<std::string, bool> jointInversion;
    std::map<std::string, ros::Publisher> pubJoints;
    ros::Subscriber subWalkCommands;

    std_msgs::Float64 position;
    
    static void WalkParamsCallback(const hexapod_msgs::FrundControl::ConstPtr& msg);

    void init_rc_control();
    void read_params();
    void gateway_init(int port);
    
public:
    HexapodController(ros::NodeHandle& nh);
    ~HexapodController();
    void FrundExchange();
};

#endif