#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "hexapod_msgs/FrundControl.h"

// Server side implementation of UDP client-server model
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

const std::string ns = "/hexapod/";
#define PORT 55556

#define JOINTS_COUNT 18
#define ITEMS_PER_JOINT 9

/* struct JointParams {
  int number;
  bool reverce;
}; */

//std::map<std::string, JointParams> jointParams;

/*std::map<std::string, JointParams> jointParams = {
    {"j_c1_lf", {1, false}},     {"j_c1_lm", {7, false}},
    {"j_c1_lr", {13, false}},    {"j_c1_rf", {4, false}},
    {"j_c1_rm", {10, false}},    {"j_c1_rr", {16, false}},
    {"j_thigh_lf", {2, false}},  {"j_thigh_lm", {8, false}},
    {"j_thigh_lr", {14, false}}, {"j_thigh_rf", {5, true}},
    {"j_thigh_rm", {11, true}},  {"j_thigh_rr", {17, true}},
    {"j_tibia_lf", {3, false}},  {"j_tibia_lm", {9, false}},
    {"j_tibia_lr", {15, false}}, {"j_tibia_rf", {6, true}},
    {"j_tibia_rm", {12, true}},  {"j_tibia_rr", {18, true}},
};*/

/*std::map<std::string, JointParams> jointParams = {
    {"j_c1_lf", {1, false}},     {"j_c1_lm", {4, false}},
    {"j_c1_lr", {7, false}},     {"j_c1_rf", {16, true}},
    {"j_c1_rm", {13, true}},     {"j_c1_rr", {10, true}},
    {"j_thigh_lf", {2, false}},  {"j_thigh_lm", {5, false}},
    {"j_thigh_lr", {8, false}},  {"j_thigh_rf", {17, false}},
    {"j_thigh_rm", {14, false}}, {"j_thigh_rr", {11, false}},
    {"j_tibia_lf", {3, false}},  {"j_tibia_lm", {6, false}},
    {"j_tibia_lr", {9, false}},  {"j_tibia_rf", {18, false}},
    {"j_tibia_rm", {15, false}}, {"j_tibia_rr", {12, false}},
};*/

int sock;
double input[300], output[300], par[10], t = 10;
struct sockaddr_in server_addr, client_addr;
int recvSize = 300;
socklen_t slen;

std::map<int, double> positions;

void gateway_init(int);

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "hexapod_controller");
  ros::NodeHandle n;

  std::map<std::string, int> jointNumbers;
  std::map<std::string, bool> jointInversion;

  if(!n.getParam("hexapod_joints_numbers_map", jointNumbers))
  {
    ROS_ERROR("hexapod_joint_numbers_map param not found");
  }

  if(!n.getParam("hexapod_joints_inversion_map", jointInversion))
  {
    ROS_ERROR("hexapod_joint_inversion_map param not found");
  }

  std::map<std::string, ros::Publisher> pub_joints;

  for (auto it = jointNumbers.begin(); it != jointNumbers.end(); it++) 
  {
    const std::string topic = ns + it->first + "_position_controller/command";
    pub_joints[it->first] = n.advertise<std_msgs::Float64>(topic, 1000);
  }

  std_msgs::Float64 position;

  int frund_port;

  if(!n.getParam("frund_port", frund_port))
  {
    ROS_ERROR("frund_port param not found");
  }

  gateway_init(frund_port);

  // Обнуляем параметры
  memset(&par, 0, 10 * sizeof(double));

  while (ros::ok()) 
  {
    int recv = recvfrom(sock, (char *)input, recvSize * sizeof(double), 0,
                        (struct sockaddr *)&client_addr, &slen);
    if (recv < 0) 
    {
      printf("recvfrom() failed\n");
      exit(EXIT_FAILURE);
    }

    printf("\nClient : %d", recv);

    for (int i = 0; i < JOINTS_COUNT; i++) 
    {
      int jointNumber = input[i * ITEMS_PER_JOINT + 0];
      double time = input[i * ITEMS_PER_JOINT + 1];
      double jointPosition = input[i * ITEMS_PER_JOINT + 2];

      double pCoeff = input[i * ITEMS_PER_JOINT + 3];
      double iCoeff = input[i * ITEMS_PER_JOINT + 4];
      double dCoeff = input[i * ITEMS_PER_JOINT + 5];

      double pCoeffMult = input[i * ITEMS_PER_JOINT + 6];
      double iCoeffMult = input[i * ITEMS_PER_JOINT + 7];
      double dCoeffMult = input[i * ITEMS_PER_JOINT + 8];

      positions[jointNumber] = jointPosition;
    }

    for (auto it = jointNumbers.begin(); it != jointNumbers.end(); it++) 
    {
      position.data = jointInversion[it->first] ?
        -positions[it->second] : 
        positions[it->second];

      pub_joints[it->first].publish(position);
    }

    memcpy((void *)par, (const void *)(output + 200), 10 * sizeof(double));

    sendto(sock, (const char *)output, recvSize * sizeof(double), 0,
           (const struct sockaddr *)&client_addr, slen);

    ros::spinOnce();
    // loop_rate.sleep();
  }
}

void gateway_init(int port) 
{
  // Creating socket file descriptor
  if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) 
  {
    perror("socket creation failed");
    exit(EXIT_FAILURE);
  }

  memset(&server_addr, 0, sizeof(server_addr));
  memset(&client_addr, 0, sizeof(client_addr));

  // Filling server information
  server_addr.sin_family = AF_INET; // IPv4
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(port);

  // Bind the socket with the server address
  if (bind(sock, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) 
  {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  printf("Server started...\n");

  slen = sizeof(client_addr); // len is value/result
}