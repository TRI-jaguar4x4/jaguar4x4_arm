#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <string>
#include <stdexcept>

#include "jaguar4x4_arm/Communication.h"

int Communication::ipValidator(std::string& ip)
{
  unsigned int n1,n2,n3,n4;
  if ( sscanf(ip.c_str(), "%u.%u.%u.%u", &n1,&n2,&n3,&n4) != 4 ) {
    return -1;
  }
  
  if ((n1 != 0) && (n1 <= 255) && (n2 <= 255) && (n3 <= 255) && (n4 <= 255) )
  {
    return 0;
  }

  return -1;
}

void Communication::connect(std::string& ip, uint16_t port)
{
  if (ipValidator(ip)) {
    throw std::runtime_error(ip + " not a valid IP address");
  }

  struct sockaddr_in _addr;
  memset(&_addr, 0, sizeof(_addr));
  _addr.sin_family = AF_INET;
  _addr.sin_port = htons(port);

  if ( inet_aton(ip.c_str(), &_addr.sin_addr) == 0)
  {
    throw std::runtime_error(ip + " not a valid inet IP address");
  }

  //TCP
  if ((fd_ = socket(PF_INET, SOCK_STREAM,IPPROTO_TCP)) < 0)
  {
    throw std::runtime_error("Failed to create socket");
  }

  //TCP, setup connection here
  if (::connect(fd_, (struct sockaddr *) &_addr,sizeof(_addr)) < 0) {
    throw std::runtime_error("Failed to connect with robot. IP address " + ip + " Port: " + std::to_string(port));
   }
}

void Communication::sendCommand(std::string& cmd)
{
}

std::string Communication::recvMessage(std::string& boundary)
{
  return "string";
}
