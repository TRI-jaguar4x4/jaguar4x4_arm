#include <stdio.h>

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
}

void Communication::sendCommand()
{
}

std::string Communication::recvMessage(std::string& boundary)
{
  return "string";
}
