#include <iostream>
#include <regex>

#include "jaguar4x4_arm/ArmReceive.h"

// Terminology from http://www.cplusplus.com/reference/string/string/compare/
static bool startsWith(const std::string& compared, const std::string& comparing)
{
  if (compared.length() < comparing.length()) {
    return false;
  }
  return compared.compare(0, comparing.length(), comparing) == 0;
}

static void dumpHex(const std::string& msg)
{
  static const char* const lut = "0123456789ABCDEF";
  std::string output;
  output.reserve(2 * msg.length());
  for (size_t i = 0; i < msg.length(); i++) {
    output.push_back(lut[msg[i] >> 4]);
    output.push_back(lut[msg[i] & 15]);
  }
  std::cerr << output << "\n";
}

ArmReceive::ArmReceive(AbstractCommunication* comm) : comm_(comm)
{
}

void ArmReceive::getAndParseMessage()
{
  std::string msg = comm_->recvMessage("\r", 1000);
  std::smatch sm;

  // nothing before the "\r"
  if (msg.empty()) {
    return;
  }
  
  dumpHex(msg);

  if (startsWith(msg,"A=")) {
    if (std::regex_match(msg, sm, std::regex("A=(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for A " << sm.size() << "\n";
    }
  } else if (startsWith(msg,"C=")) {
    if (std::regex_match(msg, sm, std::regex("C=(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for C " << sm.size() << "\n";
    }
  } else if (startsWith(msg,"P=")) {
    if (std::regex_match(msg, sm, std::regex("P=(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for P " << sm.size() << "\n";
    }
  } else if (startsWith(msg,"S=")) {
    if (std::regex_match(msg, sm, std::regex("S=(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for S " << sm.size() << "\n";
    }
  } else if (startsWith(msg,"T=")) {
    if (std::regex_match(msg, sm, std::regex("T=(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for T " << sm.size() << "\n";
    }
  } else if (startsWith(msg,"V=")) {
    if (std::regex_match(msg, sm, std::regex("V=(-?[0-9-]*?):(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for V " << sm.size() << "\n";
    }
  } else if (startsWith(msg,"+")) {
    // valid command accepted
  } else if (startsWith(msg,"AI=")) {
    if (std::regex_match(msg, sm, std::regex("AI=(-?[0-9-]*?):(-?[0-9-]*?):(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for AI " << sm.size() << "\n";
    }
  } else if (startsWith(msg,"FF=")) {
    if (std::regex_match(msg, sm, std::regex("FF=(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for FF " << sm.size() << "\n";
    }
  } else if (startsWith(msg,"MMOD=")) {
    if (std::regex_match(msg, sm, std::regex("MMOD=(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for MMOD " << sm.size() << "\n";
    }
  } else {
    std::cerr << "UNKNOWN MESSAGE TYPE '"<< msg << "'\n";
  }
}
