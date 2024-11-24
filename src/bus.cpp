#include <bus.hpp>

#include <CPU6502.hpp>
#include <PPU2C02.hpp>

Bus::Bus(CPU6502* cpu, PPU2C02* ppu, Cartridge* cart) {
  cpu_ = cpu;
  ppu_ = ppu;
  cart_ = cart;

  // initialize ram to 0
  for (auto& i : cpu_ram_) {
    i = 0x00;
  }
}

void Bus::cpuWrite(uint16_t addr, uint8_t data) {
  if (addr >= 0x0000 && addr <= 0x1FFF) { // ram
    cpu_ram_[addr & 0x07FF] = data;
  } else if (addr >= 0x2000 && addr <= 0x3FFF) { // ppu
    ppu_->cpuWrite(addr & 0x0007, data);
  }
}

uint8_t Bus::cpuRead(uint16_t addr) {
  uint8_t data = 0x00;
  if (addr >= 0x0000 && addr <= 0x1FFF) { // ram
    data = cpu_ram_[addr & 0x07FF];
  } else if (addr >= 0x2000 && addr <= 0x0FFF) { // ppu
    data = ppu_->cpuRead(addr & 0x0007);
  } else if (addr >= 0x8000 && addr < 0x10000) { // cart
    data = cart_->read(addr);
  }
  return data;
}
