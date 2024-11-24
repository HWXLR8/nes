#pragma once

#include <cstdint>

class PPU2C02 {
public:
  PPU2C02();
  uint8_t nametables_[2][1024];
  uint8_t palette[32];

  // main bus, these names are retarded
  uint8_t cpuRead(uint16_t addr);
  void cpuWrite(uint16_t addr, uint8_t data);

  // ppu bus
  uint8_t ppuRead(uint16_t addr);
  void ppuWrite(uint16_t addr, uint8_t data);
};
