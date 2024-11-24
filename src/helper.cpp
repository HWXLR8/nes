#include <helper.hpp>

#include <iomanip>
#include <iostream>
#include <sstream>

std::string hex2str(int num, int padding) {
  std::stringstream ss;
  ss << std::setw(padding) << std::setfill('0') << std::uppercase << std::hex << num;
  return ss.str();
}

void printHex(uint16_t n) {
    std::cout << "0x" << std::hex << std::uppercase << n << std::endl;
}
