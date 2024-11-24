#include <iostream>

#include <CPU6502.hpp>

int main() {
  CPU6502* CPU = new CPU6502();
  std::cout << "CPU instance successfully created" << std::endl;
  delete CPU;
  return 0;
}
