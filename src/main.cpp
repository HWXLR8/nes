#include <iostream>
#include <vector>

#include <cartridge.hpp>
#include <CPU6502.hpp>
#include <PPU2C02.hpp>
#include <ui.hpp>

int main() {
  CPU6502* cpu = new CPU6502();
  PPU2C02* ppu = new PPU2C02();
  Cartridge* cart = new Cartridge("rom/nestest.nes");
  Bus* bus = new Bus(cpu, ppu, cart);
  UI* ui = new UI(cpu, cart);

  cpu->connectBus(bus);
  cpu->reset();
  cpu->setPC(0xC000);

  ui->nextInstruction();

  delete cpu;
  return 0;
}
