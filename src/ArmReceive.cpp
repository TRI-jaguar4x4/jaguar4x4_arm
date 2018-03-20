#include <iostream>
#include <regex>
#include <string>
#include <utility>

#include "jaguar4x4_arm/ArmReceive.h"

// Terminology from http://www.cplusplus.com/reference/string/string/compare/
static bool startsWith(const std::string& compared, const std::string& comparing)
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

MotorTempMsg::MotorTempMsg(uint16_t temp1, uint16_t temp2) : AbstractArmMsg(AbstractArmMsg::MessageType::motor_temperature), motor_temp_adc_1_(temp1), motor_temp_adc_2_(temp2)
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

void ArmReceive::getAndParseMessage()
{
  std::string msg = comm_->recvMessage("\r", 100);
  std::smatch sm;

  // nothing before the "\r"
  if (msg.empty()) {
    return;
  }
  
  //  dumpHex(msg);

  if (startsWith(msg,"A=")) {
    if (std::regex_match(msg, sm, std::regex("A=(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << sm[1] << " " << sm[2] << " ";
      MotorAmpMsg amp_message(str_to_d(sm[1]),str_to_d(sm[2]));
      std::cerr << "motor_amperage: " << amp_message.motor_amp_1_ << " " << amp_message.motor_amp_2_ << "\n";
    } else {
      std::cerr << "BOO, A didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"C=")) {
    if (std::regex_match(msg, sm, std::regex("C=(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for C " << sm.size() << "\n";
    } else {
      std::cerr << "BOO, C didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"P=")) {
    if (std::regex_match(msg, sm, std::regex("P=(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for P " << sm.size() << "\n";
    } else {
      std::cerr << "BOO, P didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"S=")) {
    if (std::regex_match(msg, sm, std::regex("S=(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for S " << sm.size() << "\n";
    } else {
      std::cerr << "BOO, S didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"T=")) {
    if (std::regex_match(msg, sm, std::regex("T=(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for T " << sm.size() << "\n";
    } else {
      std::cerr << "BOO, T didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"V=")) {
    if (std::regex_match(msg, sm, std::regex("V=(-?[0-9-]*?):(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for V " << sm.size() << " contents[0]: " << sm[0] << "\n";
    } else {
      std::cerr << "BOO, V didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"+")) {
    // valid command accepted
  } else if (startsWith(msg,"-")) {
    // INvalid command accepted
  } else if (startsWith(msg,"AI=")) {
    if (std::regex_match(msg, sm, std::regex("AI=(-?[0-9-]*?):(-?[0-9-]*?):(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      MotorTempMsg motor_temp(std::stoul(sm[3]), std::stoul(sm[4]));
      std::cerr << "motor_temperature: " << motor_temp.motor_temp_adc_1_ << " " << motor_temp.motor_temp_1_ << " " << motor_temp.motor_temp_adc_2_ << " " << motor_temp.motor_temp_2_ << " " << "\n";
    } else {
      std::cerr << "BOO, AI didn't parse ";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"FF=")) {
    if (std::regex_match(msg, sm, std::regex("FF=(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for FF " << sm.size() << "\n";
    } else {
      std::cerr << "BOO, FF didn't parse\n";
      dumpHex(msg);
    }
  } else if (startsWith(msg,"MMOD=")) {
    if (std::regex_match(msg, sm, std::regex("MMOD=(-?[0-9-]*?):(-?[0-9-]*?)$"))) {
      //      std::cerr << "size of sm for MMOD " << sm.size() << "\n";
    } else {
      std::cerr << "BOO, MMOD didn't parse\n";
      dumpHex(msg);
    }
  } else {
    std::cerr << "UNKNOWN MESSAGE TYPE '"<< msg << "'\n";
  }
}
