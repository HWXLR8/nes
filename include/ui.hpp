#pragma once

#include <map>
#include <string>
#include <vector>

#include <cartridge.hpp>
#include <CPU6502.hpp>

class UI {
public:
  UI(CPU6502* cpu, Cartridge* cart);
  void nextInstruction();

private:
  CPU6502* cpu_;
  Cartridge* cart_;
  int next_instruction_ = 0;
  std::vector<std::string> program_context_;
  std::map<uint16_t, std::string> program_;

  void parseProgram(uint16_t init_addr);
};
