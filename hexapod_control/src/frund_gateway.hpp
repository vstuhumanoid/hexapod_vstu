#ifndef FRUND_GATEWAY_HPP
#define FRUND_GATEWAY_HPP

#include "asio.hpp"

#define PACKET_SIZE 300

class FrundGateway
{
public:
  FrundGateway() {}
  FrundGateway(int port);
  void HandleConnections(int port);
  void WriteBuffer(double* buffer);
  void ReadBuffer(float* buffer);
private:
  int _port;
  asio::io_service _service;
  
  float recv_buffer[PACKET_SIZE * 2];
  double send_buffer[PACKET_SIZE];
};

#endif // FRUND_GATEWAY_HPP