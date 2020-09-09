#include "hexapod_controller.hpp"

using namespace std;

const std::string ns = "/hexapod/";
const std::string walk_params_ns = "walk_params/";

#define JOINTS_COUNT 18
#define ITEMS_PER_JOINT 2

// Параметры движения для управления с пульта
std::map<std::string, int> walkParams = 
{
  {"walk_forward_back", 0}, // попугаи движения вперед(+) / назад(-)
  {"walk_left_right", 1}, // попугаи движения влево(+) / вправо(-)
  {"walk_up_down", 2}, // попугаи движения вверх(+) / вниз(-)
  {"roll", 3}, // крен ( наклон лево(+) / право(-) )
  {"pitch", 4}, // тангаж ( наклон нос(-) / корма(+) )
  {"yaw", 5}, // рысканье (поворот лево(-) / право(+) )
  {"step_lenght", 6}, // длинна шага
  {"step_height", 7}, // высота шага
  {"walk_param", 8}, // параметр походки ???
  {"feet_y", 9}, // y стоп
  {"feet_x", 11}, // x стоп
  {"walk_type", 10} // тип походки (1, 2, 3, 4)
};

const std::string controller_name = "<-- Hexapod Controller-- >";

int sock;
float input[600];
double output[300];
struct sockaddr_in server_addr, client_addr;
int recvSize = 600;
socklen_t slen;

std::map<int, double> positions;
std_msgs::Float64 position;
int frund_port;

void HexapodController::read_params()
{
  if(!_nh.getParam("hexapod_joints_numbers_map", jointNumbers))
  {
    ROS_ERROR("hexapod_joint_numbers_map param not found");
  }

  if(!_nh.getParam("hexapod_joints_inversion_map", jointInversion))
  {
    ROS_ERROR("hexapod_joint_inversion_map param not found");
  }

  if(!_nh.getParam("frund_port", frund_port))
  {
    ROS_ERROR("frund_port param not found");
  }
}

void HexapodController::init_rc_control()
{
  subWalkCommands = _nh.subscribe(ns + walk_params_ns, 1000, WalkParamsCallback);
}

HexapodController::HexapodController(ros::NodeHandle& nh) : 
    _nh(nh)
{
  read_params();

  for (auto it = jointNumbers.begin(); it != jointNumbers.end(); it++) 
  {
    const std::string topic = ns + it->first + "_position_controller/command";
    pubJoints[it->first] = nh.advertise<std_msgs::Float64>(topic, 1000);
  }

  init_rc_control();

  printf("Gateway starting...\n");
  //gateway = new FrundGateway(frund_port);
  //thread = new std::thread(&FrundGateway::HandleConnections, gateway, frund_port);

  gateway_init(frund_port);
}

HexapodController::~HexapodController()
{
  close(sock);
  //thread->join();
  //printf("Gateway joined\n");
}

void HexapodController::WalkParamsCallback(const hexapod_msgs::FrundControl::ConstPtr& msg)
{
  cout << "Walk params:" << endl;

  cout << "\tAccel:angualar:" << endl;
  cout << "\t\tx: " << msg->accel.angular.x << endl;
  cout << "\t\ty: " << msg->accel.angular.y << endl;
  cout << "\t\tz: " << msg->accel.angular.z << endl;

  cout << "\tAccel:linear:" << endl;
  cout << "\t\tx: " << msg->accel.linear.x << endl;
  cout << "\t\ty: " << msg->accel.linear.y << endl;
  cout << "\t\tz: " << msg->accel.linear.z << endl;

  cout << "\tGate settings:" << endl;
  cout << "\t\tstep length: " << msg->gait_settings.step_length << endl;
  cout << "\t\tstep height: " << msg->gait_settings.step_height << endl;
  cout << "\t\tsupport movement: " << msg->gait_settings.support_movement << endl;
  cout << "\t\tfoot body ratio: " << msg->gait_settings.foot_body_ratio << endl;
  cout << "\t\tgait type: " << int(msg->gait_settings.gait_type) << endl;
  
  cout << endl;

  // Запись в массив
  output[0] = (msg->accel.linear.x);
  output[1] = (msg->accel.linear.y);
  output[2] = (msg->accel.linear.z);

  output[3] = (msg->accel.angular.x);
  output[4] = (msg->accel.angular.y);
  output[5] = (msg->accel.angular.z);
}

void HexapodController::FrundExchange()
{
  gateway->ReadBuffer(input);

  int recv = recvfrom(sock, (char *)input, recvSize * sizeof(float), 0,
                        (struct sockaddr *)&client_addr, &slen);
  if (recv < 0)
  {
    printf("recvfrom() failed\n");
    exit(EXIT_FAILURE);
  }

  cout << "frund :" << recv << endl;;

  float time = input[0];

  cout << "time = " << time << endl;

  for(int i = 0; i < JOINTS_COUNT; i++)
  {
    int jointNumber = input[i * ITEMS_PER_JOINT + 1];
    float jointPosition = input[i * ITEMS_PER_JOINT + 2];

    //cout << "joint[" << jointNumber << "] = " << jointPosition << endl;

    positions[jointNumber] = jointPosition;
  }

  for (auto it = jointNumbers.begin(); it != jointNumbers.end(); it++)
  {
    position.data = jointInversion[it->first] ? 
    -positions[it->second] : 
    positions[it->second];

    pubJoints[it->first].publish(position);
  }

  gateway->WriteBuffer(output);
  sendto(sock, (const char *)output, 300 * sizeof(double), 0,
          (const struct sockaddr *)&client_addr, slen);
}

void HexapodController::gateway_init(int port) 
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