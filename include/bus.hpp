#pragma once

#include <cstdint>

#include <array>

#include <cartridge.hpp>

class CPU6502;
class PPU2C02;

class Bus {
public:
  CPU6502* cpu_;
  PPU2C02* ppu_;
  Cartridge* cart_;
  std::array<uint8_t, 2048> cpu_ram_;

  Bus(CPU6502* cpu, PPU2C02* ppu, Cartridge* cart_);
  void cpuWrite(uint16_t addr, uint8_t data);
  uint8_t cpuRead(uint16_t addr);
};
