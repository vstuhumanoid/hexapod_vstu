#include "frund_gateway.hpp"

FrundGateway::FrundGateway(int port)
{
  _port = port;
  printf("Gateway created.\n");
}

void FrundGateway::HandleConnections(int port)
{
  asio::ip::udp::socket sock(
    _service,
    asio::ip::udp::endpoint(asio::ip::udp::v4(), port)
  );
  
  printf("Handler started.\n");

  while (true) 
	{
		asio::ip::udp::endpoint sender_ep;
		int bytes = sock.receive_from(asio::buffer(recv_buffer), sender_ep);
		
    printf("\nClient : %d", bytes);

		sock.send_to(asio::buffer(send_buffer), sender_ep);
	}
}

void FrundGateway::WriteBuffer(double *buffer)
{
  memcpy(send_buffer, buffer, PACKET_SIZE); 
}

void FrundGateway::ReadBuffer(float *buffer)
{
  memcpy(buffer, recv_buffer, PACKET_SIZE * 2);
}