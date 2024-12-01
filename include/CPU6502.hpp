#pragma once

#include <cstdint>

#include <map>
#include <string>
#include <vector>

#include <bus.hpp>
#include <cartridge.hpp>

enum REGISTER {
  A,
  X,
  Y,
  S,
  PC,
  P,
};

struct INSTRUCTION {
  // instruction name
  std::string name;
  // pointer to instruction opcode
  uint8_t(CPU6502::*opcode)(void) = nullptr;
  // pointer to instruction addressing mode
  uint8_t(CPU6502::*address_mode)(void) = nullptr;
  // number of cycles required for execution
  uint8_t cycles = 0;
};

class CPU6502 {
public:
  enum STATUS_FLAG {
    C = (1 << 0), // carry bit
    Z = (1 << 1), // zero
    I = (1 << 2), // disable interrupts
    D = (1 << 3), // decimal mode
    B = (1 << 4), // break
    U = (1 << 5), // unused
    V = (1 << 6), // overflow
    N = (1 << 7), // negative
  };

  CPU6502();
  void connectBus(Bus* n);
  // all registers will be treated as 16 bit integers for the sake of
  // simplifying this function
  uint16_t getRegister(REGISTER r);
  void setPC(uint16_t addr);
  int getCycle();
  bool isIdle();

  // addressing modes
  // returns 1 if additional clock cycle is required, 0 otherwise
  uint8_t IMP(); // implied
  uint8_t IMM(); // immediate
  uint8_t ZP0(); // zero page
  uint8_t ZPX(); // zero page X offset
  uint8_t ZPY(); // zero page Y offset
  uint8_t REL(); // relative
  uint8_t ABS(); // absolute
  uint8_t ABX(); // absolute X offset
  uint8_t ABY(); // absolute Y offset
  uint8_t IND(); // indirect
  uint8_t IZX(); // indirect zero page X offset
  uint8_t IZY(); // indirect zero page Y offset

  // opcodes
  // returns 1 if additional clock cycle is required, 0 otherwise
  uint8_t ADC();
  uint8_t AND();
  uint8_t ASL();
  uint8_t BCC();
  uint8_t BCS();
  uint8_t BEQ();
  uint8_t BIT();
  uint8_t BMI();
  uint8_t BNE();
  uint8_t BPL();
  uint8_t BRK();
  uint8_t BVC();
  uint8_t BVS();
  uint8_t CLC();
  uint8_t CLD();
  uint8_t CLI();
  uint8_t CLV();
  uint8_t CMP();
  uint8_t CPX();
  uint8_t CPY();
  uint8_t DEC();
  uint8_t DEX();
  uint8_t DEY();
  uint8_t EOR();
  uint8_t INC();
  uint8_t INX();
  uint8_t INY();
  uint8_t JMP();
  uint8_t JSR();
  uint8_t LDA();
  uint8_t LDX();
  uint8_t LDY();
  uint8_t LSR();
  uint8_t NOP();
  uint8_t ORA();
  uint8_t PHA();
  uint8_t PHP();
  uint8_t PLA();
  uint8_t PLP();
  uint8_t ROL();
  uint8_t ROR();
  uint8_t RTI();
  uint8_t RTS();
  uint8_t SBC();
  uint8_t SEC();
  uint8_t SED();
  uint8_t SEI();
  uint8_t STA();
  uint8_t STX();
  uint8_t STY();
  uint8_t TAX();
  uint8_t TAY();
  uint8_t TSX();
  uint8_t TXA();
  uint8_t TXS();
  uint8_t TYA();
  // undocumented
  uint8_t LAX();
  uint8_t SAX();
  uint8_t DCP();
  uint8_t ISB(); // aka ISC
  uint8_t SLO();
  uint8_t RLA();
  // illegal
  uint8_t ILL();

  int clock();
  void reset();
  void irq(); // interrupt request
  void nmi(); // non-maskable interrupt

  uint8_t read(uint16_t addr);
  void write(uint16_t addr, uint8_t data);

  INSTRUCTION opcodeLookup(uint8_t opcode);

private:
  Bus* bus_ = nullptr;
  // registers
  uint8_t a_ = 0x00; // accumulator
  uint8_t x_ = 0x00; // x register
  uint8_t y_ = 0x00; // y register
  uint8_t s_ = 0x00; // stack pointer - offset from 0x0100 (stack
			// address in memory)
  uint16_t pc_ = 0x0000; // program counter - stores the address of
			 // the next program byte the cpu needs to
			 // read
  uint8_t p_ = 0x00; // processor status register


  // depending on addressing mode, we will want to fetch from
  // different parts of memory
  uint16_t addr_abs_ = 0x0000;
  // branch instructions can only jump a certain distance, they jump
  // to a relative address
  uint16_t addr_rel_ = 0x0000;
  // current opcode
  uint8_t opcode_ = 0x00;
  // number of cycles left in the duration of the current instruction
  uint8_t cycles_left_ = 0;
  // total number of elapsed cycles
  int cycles_ = 0;
  uint8_t data_ = 0x00;
  std::vector<INSTRUCTION> instructions_;

  uint8_t fetch_data();
  bool getFlag(STATUS_FLAG flag);
  void setFlag(STATUS_FLAG flag, bool condition);
};
