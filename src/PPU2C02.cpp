#include <PPU2C02.hpp>

PPU2C02::PPU2C02() {

}

uint8_t PPU2C02::cpuRead(uint16_t addr) {
  uint8_t data = 0x00;

  switch (addr) {
  // control
  case 0x0000:
    break;

  // mask
  case 0x0001:
    break;

  // status
  case 0x0002:
    break;

  // OAM address
  case 0x0003:
    break;

  // OAM data
  case 0x0004:
    break;

  // scroll
  case 0x0005:
    break;

  // ppu address
  case 0x0006:
    break;

  // ppu data
  case 0x0007:
    break;
  }

  return data;
}

void PPU2C02::cpuWrite(uint16_t addr, uint8_t data) {
  switch (addr) {
  // control
  case 0x0000:
    break;

  // mask
  case 0x0001:
    break;

  // status
  case 0x0002:
    break;

  // OAM address
  case 0x0003:
    break;

  // OAM data
  case 0x0004:
    break;

  // scroll
  case 0x0005:
    break;

  // ppu address
  case 0x0006:
    break;

  // ppu data
  case 0x0007:
    break;
  }
}

uint8_t PPU2C02::ppuRead(uint16_t addr) {
  uint8_t data = 0x00;
  addr &= 0x3FFF;
  return data;
}

void PPU2C02::ppuWrite(uint16_t addr, uint8_t data) {
  addr &= 0x3FFF;
}
