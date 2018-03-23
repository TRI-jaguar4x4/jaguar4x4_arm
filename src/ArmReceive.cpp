// Copyright 2018 Toyota Research Institute.  All rights reserved.
#include <iostream>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "jaguar4x4_arm/ArmReceive.h"

// Terminology from http://www.cplusplus.com/reference/string/string/compare/
static bool startsWith(const std::string& compared,
                       const std::string& comparing)
{
  if (compared.length() < comparing.length()) {
    return false;
  }
  return compared.compare(0, comparing.length(), comparing) == 0;
}

static double adToTemperature(uint16_t adValue)
{
  static double resTable[25] = {114660,84510,62927,47077,35563,27119,20860,16204,12683,10000,7942,6327,5074,4103,3336,2724,2237,1846,1530,1275,1068,899.3,760.7,645.2,549.4};
  static double tempTable[25] = { -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
  static double FULLAD = 4095;

  //for new temperature sensor
  double tempM = 0;
  double k = (adValue / FULLAD);
  double resValue = 0;
  if (k != 0)
  {
    resValue = (10000 / k -10000);      //AD value to resistor
  }
  else
  {
    resValue = resTable[0];
  }

  int index = -1;
  if (resValue >= resTable[0])       //too lower
  {
    tempM = -20;
  }
  else if (resValue <= resTable[24])
  {
    tempM = 100;
  }
  else
  {
    for (int i = 0; i < 24; i++)
    {
      if ((resValue <= resTable[i]) && (resValue >= resTable[i + 1]))
      {
        index = i;
        break;
      }
    }
    if (index >= 0)
    {
      tempM = tempTable[index] + (resValue - resTable[index]) / (resTable[index + 1] - resTable[index]) * (tempTable[index + 1] - tempTable[index]);
    }
    else
    {
      tempM = 0;
    }
  }

  return tempM;
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

MotorTempMsg::MotorTempMsg(uint16_t temp1, uint16_t temp2)
  : AbstractArmMsg(AbstractArmMsg::MessageType::motor_temperature),
    motor_temp_adc_1_(temp1), motor_temp_adc_2_(temp2)
{
  motor_temp_1_ = adToTemperature(temp1);
  motor_temp_2_ = adToTemperature(temp2);
}

ArmReceive::ArmReceive(AbstractCommunication* comm) : comm_(comm)
{
}

static double str_to_d(const std::string& in) {
  std::stringstream ss;
  ss.imbue(std::locale::classic());
  ss << in;

  double out;
  ss >> out;

  if (ss.fail() || !ss.eof()) {
    throw std::runtime_error("failed str_to_d conversion");
  }

  return out;
}

std::vector<std::unique_ptr<AbstractArmMsg>> ArmReceive::getAndParseMessage()
{
  std::string msg = comm_->recvMessage("\r", 100);
  std::smatch sm;
  std::vector<std::unique_ptr<AbstractArmMsg>> arm_msgs;

  // nothing before the "\r"
  if (msg.empty()) {
    return arm_msgs;
  }

  //  dumpHex(msg);

  if (startsWith(msg,"A=")) {
    if (std::regex_match(msg, sm, std::regex("A=(-?[0-9-]*?):(-?[0-9-]*?)$")))
    {
      arm_msgs.emplace_back(std::make_unique<MotorAmpMsg>(str_to_d(sm[1])/10.0,
                                                          str_to_d(sm[2])/10.0));
    } else {
      std::cerr << "BOO, A didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"C=")) {
    if (std::regex_match(msg, sm, std::regex("C=(-?[0-9-]*?):(-?[0-9-]*?)$")))
    {
      arm_msgs.emplace_back(std::make_unique<MotorEncPosMsg>(std::stol(sm[1]),
                                                             std::stol(sm[2])));
    } else {
      std::cerr << "BOO, C didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"P=")) {
    if (std::regex_match(msg, sm, std::regex("P=(-?[0-9-]*?):(-?[0-9-]*?)$")))
    {
      arm_msgs.emplace_back(std::make_unique<MotorPowerMsg>(std::stol(sm[1]),
                                                            std::stol(sm[2])));
    } else {
      std::cerr << "BOO, P didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"S=")) {
    if (std::regex_match(msg, sm, std::regex("S=(-?[0-9-]*?):(-?[0-9-]*?)$")))
    {
      arm_msgs.emplace_back(std::make_unique<MotorEncVelMsg>(std::stol(sm[1]),
                                                             std::stol(sm[2])));
    } else {
      std::cerr << "BOO, S didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"T=")) {
    if (std::regex_match(msg, sm, std::regex("T=(-?[0-9-]*?):(-?[0-9-]*?)$")))
    {
      arm_msgs.emplace_back(std::make_unique<MotorBoardTempMsg>(str_to_d(sm[1]),
                                                                str_to_d(sm[2])));
    } else {
      std::cerr << "BOO, T didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"V=")) {
    if (std::regex_match(
          msg, sm, std::regex("V=(-?[0-9-]*?):(-?[0-9-]*?):(-?[0-9-]*?)$")))
    {
      arm_msgs.emplace_back(std::make_unique<MotorVoltageMsg>(
                              str_to_d(sm[1])/10.0,
                              str_to_d(sm[2])/10.0,
                              str_to_d(sm[3])/1000.0));
    } else {
      std::cerr << "BOO, V didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"+")) {
    // valid command received
    arm_msgs.emplace_back(std::make_unique<MotorCmdAcceptedMsg>());
  } else if (startsWith(msg,"-")) {
    // INvalid command received
    arm_msgs.emplace_back(std::make_unique<MotorCmdRejectedMsg>());
  } else if (startsWith(msg,"AI=")) {
    if (std::regex_match(
          msg, sm,
          std::regex("AI=(-?[0-9-]*?):(-?[0-9-]*?):(-?[0-9-]*?):(-?[0-9-]*?)$")))
    {
      arm_msgs.emplace_back(std::make_unique<MotorTempMsg>(std::stoul(sm[3]),
                                                           std::stoul(sm[4])));
    } else {
      std::cerr << "BOO, AI didn't parse ";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"FF=")) {
    if (std::regex_match(msg, sm, std::regex("FF=(-?[0-9-]*?)$")))
    {
      arm_msgs.emplace_back(std::make_unique<MotorFlagsMsg>(std::stoul(sm[1])));
    } else {
      std::cerr << "BOO, FF didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"MMOD=")) {
    if (std::regex_match(msg, sm,
                         std::regex("MMOD=(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      arm_msgs.emplace_back(std::make_unique<MotorModeMsg>(std::stoul(sm[1]),
                                                           std::stoul(sm[2])));
    } else {
      std::cerr << "BOO, MMOD didn't parse\n";
      dumpHex(msg);
    }
  } else {
    std::cerr << "UNKNOWN MESSAGE TYPE '"<< msg << "'\n";
  }

  return arm_msgs;
}
