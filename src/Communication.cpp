#include <arpa/inet.h>
#include <errno.h>
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
  bool done = false;
  const char* cmd_progress = cmd.c_str();
  size_t len_to_send = cmd.length();
  while (!done) {
    int retval = send(fd_, cmd_progress, len_to_send, 0 /* blocking */);
    if (retval < 0) {

      if (retval == EINTR) {
	continue;
      }
      throw std::runtime_error("Failed to send command: " + cmd + std::string(::strerror(errno)));
    }

    len_to_send -= retval;
    if (len_to_send) {
      cmd_progress += retval;
      continue;
    }
    done = true;
  }
}

std::string Communication::recvMessage(std::string& boundary, int timeout_msec)
{
  fd_set readfds;

  std::string rcv_str;
  char rcv_chunk[1024];
  
  FD_ZERO(&readfds);
  FD_SET(fd_, &readfds);

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = timeout_msec*1000;

  bool done=false;
  while (!done) {
    int retval = select(fd_ + 1, &readfds, NULL, NULL, &tv);
    if (retval < 0) {
      if (retval == EINTR) {
	continue;
      }
      throw std::runtime_error("Failed to receive command: " + std::string(::strerror(errno)));
    } else if (retval == 0) {
      throw std::runtime_error("Failed to receive command, timeout expired");
    }
    if (!FD_ISSET(fd_, &readfds)) {
      throw std::runtime_error("Failed to receive command, bad fd");
    }

    int rcv_retval = recv(fd_, rcv_chunk, sizeof(rcv_chunk)-1, 0);
    if (rcv_retval < 0) {
      if (rcv_retval == EINTR) {
	continue;
      }
      throw std::runtime_error("Failed to receive chunk: " + std::string(::strerror(errno)));
    }
    rcv_chunk[rcv_retval] = '\0';
    rcv_str.append(rcv_chunk);

    // if we found a boundary, we're done
    if (rcv_str.find(boundary) != std::string::npos) {
      done = true;
    }
    // TODO: This could return the entire command + the beginning
    // of a new one... deal with that somehow.
  }
  
  return rcv_str;
}
