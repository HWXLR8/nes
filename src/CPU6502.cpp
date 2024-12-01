#include <CPU6502.hpp>

#include <filesystem>
#include <fstream>

CPU6502::CPU6502() {
  instructions_ = {
    { "BRK", &CPU6502::BRK, &CPU6502::IMM, 7 },
    { "ORA", &CPU6502::ORA, &CPU6502::IZX, 6 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "SLO", &CPU6502::SLO, &CPU6502::IZX, 8 },
    { "???", &CPU6502::NOP, &CPU6502::ZP0, 3 },
    { "ORA", &CPU6502::ORA, &CPU6502::ZP0, 3 },
    { "ASL", &CPU6502::ASL, &CPU6502::ZP0, 5 },
    { "SLO", &CPU6502::SLO, &CPU6502::ZP0, 5 },
    { "PHP", &CPU6502::PHP, &CPU6502::IMP, 3 },
    { "ORA", &CPU6502::ORA, &CPU6502::IMM, 2 },
    { "ASL", &CPU6502::ASL, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "???", &CPU6502::NOP, &CPU6502::ABS, 4 },
    { "ORA", &CPU6502::ORA, &CPU6502::ABS, 4 },
    { "ASL", &CPU6502::ASL, &CPU6502::ABS, 6 },
    { "SLO", &CPU6502::SLO, &CPU6502::ABS, 6 },
    { "BPL", &CPU6502::BPL, &CPU6502::REL, 2 },
    { "ORA", &CPU6502::ORA, &CPU6502::IZY, 5 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "SLO", &CPU6502::SLO, &CPU6502::IZY, 8 },
    { "???", &CPU6502::NOP, &CPU6502::ZPX, 4 },
    { "ORA", &CPU6502::ORA, &CPU6502::ZPX, 4 },
    { "ASL", &CPU6502::ASL, &CPU6502::ZPX, 6 },
    { "SLO", &CPU6502::SLO, &CPU6502::ZPX, 6 },
    { "CLC", &CPU6502::CLC, &CPU6502::IMP, 2 },
    { "ORA", &CPU6502::ORA, &CPU6502::ABY, 4 },
    { "???", &CPU6502::NOP, &CPU6502::IMP, 2 },
    { "SLO", &CPU6502::SLO, &CPU6502::ABY, 7 },
    { "???", &CPU6502::NOP, &CPU6502::ABX, 4 },
    { "ORA", &CPU6502::ORA, &CPU6502::ABX, 4 },
    { "ASL", &CPU6502::ASL, &CPU6502::ABX, 7 },
    { "SLO", &CPU6502::SLO, &CPU6502::ABX, 7 },
    { "JSR", &CPU6502::JSR, &CPU6502::ABS, 6 },
    { "AND", &CPU6502::AND, &CPU6502::IZX, 6 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "RLA", &CPU6502::RLA, &CPU6502::IZX, 8 },
    { "BIT", &CPU6502::BIT, &CPU6502::ZP0, 3 },
    { "AND", &CPU6502::AND, &CPU6502::ZP0, 3 },
    { "ROL", &CPU6502::ROL, &CPU6502::ZP0, 5 },
    { "RLA", &CPU6502::RLA, &CPU6502::ZP0, 5 },
    { "PLP", &CPU6502::PLP, &CPU6502::IMP, 4 },
    { "AND", &CPU6502::AND, &CPU6502::IMM, 2 },
    { "ROL", &CPU6502::ROL, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "BIT", &CPU6502::BIT, &CPU6502::ABS, 4 },
    { "AND", &CPU6502::AND, &CPU6502::ABS, 4 },
    { "ROL", &CPU6502::ROL, &CPU6502::ABS, 6 },
    { "RLA", &CPU6502::RLA, &CPU6502::ABS, 6 },
    { "BMI", &CPU6502::BMI, &CPU6502::REL, 2 },
    { "AND", &CPU6502::AND, &CPU6502::IZY, 5 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "RLA", &CPU6502::RLA, &CPU6502::IZY, 8 },
    { "???", &CPU6502::NOP, &CPU6502::ZPX, 4 },
    { "AND", &CPU6502::AND, &CPU6502::ZPX, 4 },
    { "ROL", &CPU6502::ROL, &CPU6502::ZPX, 6 },
    { "RLA", &CPU6502::RLA, &CPU6502::ZPX, 6 },
    { "SEC", &CPU6502::SEC, &CPU6502::IMP, 2 },
    { "AND", &CPU6502::AND, &CPU6502::ABY, 4 },
    { "???", &CPU6502::NOP, &CPU6502::IMP, 2 },
    { "RLA", &CPU6502::RLA, &CPU6502::ABY, 7 },
    { "???", &CPU6502::NOP, &CPU6502::ABX, 4 },
    { "AND", &CPU6502::AND, &CPU6502::ABX, 4 },
    { "ROL", &CPU6502::ROL, &CPU6502::ABX, 7 },
    { "RLA", &CPU6502::RLA, &CPU6502::ABX, 7 },
    { "RTI", &CPU6502::RTI, &CPU6502::IMP, 6 },
    { "EOR", &CPU6502::EOR, &CPU6502::IZX, 6 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 8 },
    { "???", &CPU6502::NOP, &CPU6502::ZP0, 3 },
    { "EOR", &CPU6502::EOR, &CPU6502::ZP0, 3 },
    { "LSR", &CPU6502::LSR, &CPU6502::ZP0, 5 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 5 },
    { "PHA", &CPU6502::PHA, &CPU6502::IMP, 3 },
    { "EOR", &CPU6502::EOR, &CPU6502::IMM, 2 },
    { "LSR", &CPU6502::LSR, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "JMP", &CPU6502::JMP, &CPU6502::ABS, 3 },
    { "EOR", &CPU6502::EOR, &CPU6502::ABS, 4 },
    { "LSR", &CPU6502::LSR, &CPU6502::ABS, 6 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 6 },
    { "BVC", &CPU6502::BVC, &CPU6502::REL, 2 },
    { "EOR", &CPU6502::EOR, &CPU6502::IZY, 5 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 8 },
    { "???", &CPU6502::NOP, &CPU6502::ZPX, 4 },
    { "EOR", &CPU6502::EOR, &CPU6502::ZPX, 4 },
    { "LSR", &CPU6502::LSR, &CPU6502::ZPX, 6 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 6 },
    { "CLI", &CPU6502::CLI, &CPU6502::IMP, 2 },
    { "EOR", &CPU6502::EOR, &CPU6502::ABY, 4 },
    { "???", &CPU6502::NOP, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 7 },
    { "???", &CPU6502::NOP, &CPU6502::ABX, 4 },
    { "EOR", &CPU6502::EOR, &CPU6502::ABX, 4 },
    { "LSR", &CPU6502::LSR, &CPU6502::ABX, 7 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 7 },
    { "RTS", &CPU6502::RTS, &CPU6502::IMP, 6 },
    { "ADC", &CPU6502::ADC, &CPU6502::IZX, 6 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 8 },
    { "???", &CPU6502::NOP, &CPU6502::ZP0, 3 },
    { "ADC", &CPU6502::ADC, &CPU6502::ZP0, 3 },
    { "ROR", &CPU6502::ROR, &CPU6502::ZP0, 5 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 5 },
    { "PLA", &CPU6502::PLA, &CPU6502::IMP, 4 },
    { "ADC", &CPU6502::ADC, &CPU6502::IMM, 2 },
    { "ROR", &CPU6502::ROR, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "JMP", &CPU6502::JMP, &CPU6502::IND, 5 },
    { "ADC", &CPU6502::ADC, &CPU6502::ABS, 4 },
    { "ROR", &CPU6502::ROR, &CPU6502::ABS, 6 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 6 },
    { "BVS", &CPU6502::BVS, &CPU6502::REL, 2 },
    { "ADC", &CPU6502::ADC, &CPU6502::IZY, 5 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 8 },
    { "???", &CPU6502::NOP, &CPU6502::ZPX, 4 },
    { "ADC", &CPU6502::ADC, &CPU6502::ZPX, 4 },
    { "ROR", &CPU6502::ROR, &CPU6502::ZPX, 6 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 6 },
    { "SEI", &CPU6502::SEI, &CPU6502::IMP, 2 },
    { "ADC", &CPU6502::ADC, &CPU6502::ABY, 4 },
    { "???", &CPU6502::NOP, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 7 },
    { "???", &CPU6502::NOP, &CPU6502::ABX, 4 },
    { "ADC", &CPU6502::ADC, &CPU6502::ABX, 4 },
    { "ROR", &CPU6502::ROR, &CPU6502::ABX, 7 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 7 },
    { "???", &CPU6502::NOP, &CPU6502::IMM, 2 },
    { "STA", &CPU6502::STA, &CPU6502::IZX, 6 },
    { "???", &CPU6502::NOP, &CPU6502::IMP, 2 },
    { "SAX", &CPU6502::SAX, &CPU6502::IZX, 6 },
    { "STY", &CPU6502::STY, &CPU6502::ZP0, 3 },
    { "STA", &CPU6502::STA, &CPU6502::ZP0, 3 },
    { "STX", &CPU6502::STX, &CPU6502::ZP0, 3 },
    { "SAX", &CPU6502::SAX, &CPU6502::ZP0, 3 },
    { "DEY", &CPU6502::DEY, &CPU6502::IMP, 2 },
    { "???", &CPU6502::NOP, &CPU6502::IMP, 2 },
    { "TXA", &CPU6502::TXA, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "STY", &CPU6502::STY, &CPU6502::ABS, 4 },
    { "STA", &CPU6502::STA, &CPU6502::ABS, 4 },
    { "STX", &CPU6502::STX, &CPU6502::ABS, 4 },
    { "SAX", &CPU6502::SAX, &CPU6502::ABS, 4 },
    { "BCC", &CPU6502::BCC, &CPU6502::REL, 2 },
    { "STA", &CPU6502::STA, &CPU6502::IZY, 6 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 6 },
    { "STY", &CPU6502::STY, &CPU6502::ZPX, 4 },
    { "STA", &CPU6502::STA, &CPU6502::ZPX, 4 },
    { "STX", &CPU6502::STX, &CPU6502::ZPY, 4 },
    { "SAX", &CPU6502::SAX, &CPU6502::ZPY, 4 },
    { "TYA", &CPU6502::TYA, &CPU6502::IMP, 2 },
    { "STA", &CPU6502::STA, &CPU6502::ABY, 5 },
    { "TXS", &CPU6502::TXS, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 5 },
    { "???", &CPU6502::NOP, &CPU6502::IMP, 5 },
    { "STA", &CPU6502::STA, &CPU6502::ABX, 5 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 5 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 5 },
    { "LDY", &CPU6502::LDY, &CPU6502::IMM, 2 },
    { "LDA", &CPU6502::LDA, &CPU6502::IZX, 6 },
    { "LDX", &CPU6502::LDX, &CPU6502::IMM, 2 },
    { "LAX", &CPU6502::LAX, &CPU6502::IZX, 6 },
    { "LDY", &CPU6502::LDY, &CPU6502::ZP0, 3 },
    { "LDA", &CPU6502::LDA, &CPU6502::ZP0, 3 },
    { "LDX", &CPU6502::LDX, &CPU6502::ZP0, 3 },
    { "LAX", &CPU6502::LAX, &CPU6502::ZP0, 3 },
    { "TAY", &CPU6502::TAY, &CPU6502::IMP, 2 },
    { "LDA", &CPU6502::LDA, &CPU6502::IMM, 2 },
    { "TAX", &CPU6502::TAX, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "LDY", &CPU6502::LDY, &CPU6502::ABS, 4 },
    { "LDA", &CPU6502::LDA, &CPU6502::ABS, 4 },
    { "LDX", &CPU6502::LDX, &CPU6502::ABS, 4 },
    { "LAX", &CPU6502::LAX, &CPU6502::ABS, 4 },
    { "BCS", &CPU6502::BCS, &CPU6502::REL, 2 },
    { "LDA", &CPU6502::LDA, &CPU6502::IZY, 5 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "LAX", &CPU6502::LAX, &CPU6502::IZY, 5 },
    { "LDY", &CPU6502::LDY, &CPU6502::ZPX, 4 },
    { "LDA", &CPU6502::LDA, &CPU6502::ZPX, 4 },
    { "LDX", &CPU6502::LDX, &CPU6502::ZPY, 4 },
    { "LAX", &CPU6502::LAX, &CPU6502::ZPY, 4 },
    { "CLV", &CPU6502::CLV, &CPU6502::IMP, 2 },
    { "LDA", &CPU6502::LDA, &CPU6502::ABY, 4 },
    { "TSX", &CPU6502::TSX, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 4 },
    { "LDY", &CPU6502::LDY, &CPU6502::ABX, 4 },
    { "LDA", &CPU6502::LDA, &CPU6502::ABX, 4 },
    { "LDX", &CPU6502::LDX, &CPU6502::ABY, 4 },
    { "LAX", &CPU6502::LAX, &CPU6502::ABY, 4 },
    { "CPY", &CPU6502::CPY, &CPU6502::IMM, 2 },
    { "CMP", &CPU6502::CMP, &CPU6502::IZX, 6 },
    { "???", &CPU6502::NOP, &CPU6502::IMP, 2 },
    { "DCP", &CPU6502::DCP, &CPU6502::IZX, 8 },
    { "CPY", &CPU6502::CPY, &CPU6502::ZP0, 3 },
    { "CMP", &CPU6502::CMP, &CPU6502::ZP0, 3 },
    { "DEC", &CPU6502::DEC, &CPU6502::ZP0, 5 },
    { "DCP", &CPU6502::DCP, &CPU6502::ZP0, 5 },
    { "INY", &CPU6502::INY, &CPU6502::IMP, 2 },
    { "CMP", &CPU6502::CMP, &CPU6502::IMM, 2 },
    { "DEX", &CPU6502::DEX, &CPU6502::IMP, 2 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "CPY", &CPU6502::CPY, &CPU6502::ABS, 4 },
    { "CMP", &CPU6502::CMP, &CPU6502::ABS, 4 },
    { "DEC", &CPU6502::DEC, &CPU6502::ABS, 6 },
    { "DCP", &CPU6502::DCP, &CPU6502::ABS, 6 },
    { "BNE", &CPU6502::BNE, &CPU6502::REL, 2 },
    { "CMP", &CPU6502::CMP, &CPU6502::IZY, 5 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "DCP", &CPU6502::DCP, &CPU6502::IZY, 8 },
    { "???", &CPU6502::NOP, &CPU6502::ZPX, 4 },
    { "CMP", &CPU6502::CMP, &CPU6502::ZPX, 4 },
    { "DEC", &CPU6502::DEC, &CPU6502::ZPX, 6 },
    { "DCP", &CPU6502::DCP, &CPU6502::ZPX, 6 },
    { "CLD", &CPU6502::CLD, &CPU6502::IMP, 2 },
    { "CMP", &CPU6502::CMP, &CPU6502::ABY, 4 },
    { "NOP", &CPU6502::NOP, &CPU6502::IMP, 2 },
    { "DCP", &CPU6502::DCP, &CPU6502::ABY, 7 },
    { "???", &CPU6502::NOP, &CPU6502::ABX, 4 },
    { "CMP", &CPU6502::CMP, &CPU6502::ABX, 4 },
    { "DEC", &CPU6502::DEC, &CPU6502::ABX, 7 },
    { "DCP", &CPU6502::DCP, &CPU6502::ABX, 7 },
    { "CPX", &CPU6502::CPX, &CPU6502::IMM, 2 },
    { "SBC", &CPU6502::SBC, &CPU6502::IZX, 6 },
    { "???", &CPU6502::NOP, &CPU6502::IMP, 2 },
    { "ISB", &CPU6502::ISB, &CPU6502::IZX, 8 },
    { "CPX", &CPU6502::CPX, &CPU6502::ZP0, 3 },
    { "SBC", &CPU6502::SBC, &CPU6502::ZP0, 3 },
    { "INC", &CPU6502::INC, &CPU6502::ZP0, 5 },
    { "ISB", &CPU6502::ISB, &CPU6502::ZP0, 5 },
    { "INX", &CPU6502::INX, &CPU6502::IMP, 2 },
    { "SBC", &CPU6502::SBC, &CPU6502::IMM, 2 },
    { "NOP", &CPU6502::NOP, &CPU6502::IMP, 2 },
    { "SBC", &CPU6502::SBC, &CPU6502::IMM, 2 },
    { "CPX", &CPU6502::CPX, &CPU6502::ABS, 4 },
    { "SBC", &CPU6502::SBC, &CPU6502::ABS, 4 },
    { "INC", &CPU6502::INC, &CPU6502::ABS, 6 },
    { "ISB", &CPU6502::ISB, &CPU6502::ABS, 6 },
    { "BEQ", &CPU6502::BEQ, &CPU6502::REL, 2 },
    { "SBC", &CPU6502::SBC, &CPU6502::IZY, 5 },
    { "???", &CPU6502::ILL, &CPU6502::IMP, 2 },
    { "ISB", &CPU6502::ISB, &CPU6502::IZY, 8 },
    { "???", &CPU6502::NOP, &CPU6502::ZPX, 4 },
    { "SBC", &CPU6502::SBC, &CPU6502::ZPX, 4 },
    { "INC", &CPU6502::INC, &CPU6502::ZPX, 6 },
    { "ISB", &CPU6502::ISB, &CPU6502::ZPX, 6 },
    { "SED", &CPU6502::SED, &CPU6502::IMP, 2 },
    { "SBC", &CPU6502::SBC, &CPU6502::ABY, 4 },
    { "NOP", &CPU6502::NOP, &CPU6502::IMP, 2 },
    { "ISB", &CPU6502::ISB, &CPU6502::ABY, 7 },
    { "???", &CPU6502::NOP, &CPU6502::ABX, 4 },
    { "SBC", &CPU6502::SBC, &CPU6502::ABX, 4 },
    { "INC", &CPU6502::INC, &CPU6502::ABX, 7 },
    { "ISB", &CPU6502::ISB, &CPU6502::ABX, 7 },
  };
}

void CPU6502::connectBus(Bus* n) {
  bus_ = n;
}

uint16_t CPU6502::getRegister(REGISTER r) {
  switch(r) {
  case A:
    return a_;

  case X:
    return x_;

  case Y:
    return y_;

  case S:
    return s_;

  case PC:
    return pc_;

  case P:
    return p_;
  }
  return -1;
}

uint8_t CPU6502::read(uint16_t addr) {
  return bus_->cpuRead(addr);
}

void CPU6502::write(uint16_t addr, uint8_t data) {
  bus_->cpuWrite(addr, data);
}

int CPU6502::clock() {
  if (cycles_left_ == 0) {
    opcode_ = read(pc_);
    pc_++;
    // check how many cycles the next instruction is going to take
    cycles_left_ = instructions_[opcode_].cycles;
    // call the fuction required for next instruction's address mode
    uint8_t addr_additional_cycles = (this->*instructions_[opcode_].address_mode)();
    // call the function that runs the next instructions operation
    uint8_t opcode_additional_cycles = (this->*instructions_[opcode_].opcode)();

    cycles_left_ += (addr_additional_cycles & opcode_additional_cycles);
  }

  cycles_left_--;
  cycles_++;
  return cycles_left_;
}

void CPU6502::setFlag(STATUS_FLAG flag, bool condition) {
  if (condition) {
    p_ |= flag;
  } else {
    p_ &= ~flag;
  }
}

bool CPU6502::getFlag(STATUS_FLAG flag) {
  return (p_ & flag) > 0;
}

// addressing modes

// implied addressing mode - the data is implied by the operation.
// CLC - clear carry flag, implies that this operates on the status register
// RTS - return from subroutine, implies that the return address will be taken from the stack
uint8_t CPU6502::IMP() {
  // we will fetch data from the accumulator since we may need it for instructions like PHA
  data_ = a_;
  return 0;
}

// immediate addressing mode - data is taken from the byte following the opcode
// LDA #$05 - load accumulator with memory
// ORA #$80 - OR memory with accumulator
uint8_t CPU6502::IMM() {
  // we want to fetch memory from the next byte
  addr_abs_ = pc_++;
  return 0;
}

// zero page addressing - an 8 bit address is provided within the
// zero page. since the argument is only 1 byte, the cpu does not need
// an additional cycle to fetch the page offset
// LDX $13 - load X register with the contents at memory address 0x13
// AND $07 - AND with accumulator with the contents stored at memory address 0x07
uint8_t CPU6502::ZP0() {
  addr_abs_ = read(pc_);
  pc_++;
  // we only want to read the low byte bc page 0
  addr_abs_ &= 0x00FF;
  return 0;
}

// zero page addressing with X register offset - an 8 bit address is
// provided, to which the contents of the X register is added (without
// carry)
uint8_t CPU6502::ZPX() {
  addr_abs_ = read(pc_) + x_;
  pc_++;
  // we only want to read the low byte bc page 0
  addr_abs_ &= 0x00FF;
  return 0;
}

// zero page addressing with Y register offset - an 8 bit address is
// provided, to which the contents of the Y register is added (without
// carry)
uint8_t CPU6502::ZPY() {
  addr_abs_ = read(pc_) + y_;
  pc_++;
  // we only want to read the low byte bc page 0
  addr_abs_ &= 0x00FF;
  return 0;
}

// absolute addressing - data is read from an absolute address specified as a 16-bit number
// LDA $06d3 - load accumulotor with the contents at address 0x06D3
// STX $0200 - store the contents of X register at address 0x0200
// JMP $5913 - jump to address 0x5913
uint8_t CPU6502::ABS() {
  uint16_t low_byte = read(pc_);
  pc_++;
  uint16_t high_byte = read(pc_) << 8;
  pc_++;

  addr_abs_ = (high_byte | low_byte);
  return 0;
}

// absolute addressing with X offset
uint8_t CPU6502::ABX() {
  uint16_t low_byte = read(pc_);
  pc_++;
  uint16_t high_byte = read(pc_) << 8;
  pc_++;

  addr_abs_ = (high_byte | low_byte);
  addr_abs_ += x_;

  // if the high byte has changed, we have changed pages, and may need
  // an additional clock cycle
  if ((addr_abs_ & 0xFF00) != high_byte) {
    return 1;
  }
  return 0;
}

// absolute addressing mode with Y offset
uint8_t CPU6502::ABY() {
  uint16_t low_byte = read(pc_);
  pc_++;
  uint16_t high_byte = read(pc_) << 8;
  pc_++;

  addr_abs_ = (high_byte | low_byte);
  addr_abs_ += y_;

  // if the high byte has changed, we have changed pages and need an
  // additional clock cycle
  if ((addr_abs_ & 0xFF00) != high_byte) {
    return 1;
  }
  return 0;
}

// indirect addressing - a 16 bit absolute address is supplied which
// points to the address of the data
uint8_t CPU6502::IND() {
  uint16_t ptr_low_byte = read(pc_);
  pc_++;
  uint16_t ptr_high_byte = read(pc_) << 8;
  pc_++;

  uint16_t ptr = (ptr_high_byte | ptr_low_byte);

  // in the case where ptr_low_byte != 0xFF
  uint16_t deref_addr_low_byte = read(ptr);
  uint16_t deref_addr_high_byte = read(ptr + 1) << 8;

  // if the low byte of the pointer is 0xFF, then we need to cross a
  // page boundary to read the dereferenced address. Due to a bug in
  // the CPU this does not occur, the high byte does not change. In
  // other words, for an address of the form (xxFF), the most
  // significant byte of the dereferenced address will be read from
  // xx00 instead of xx+1.
  if (ptr_low_byte == 0x00FF) {
    uint16_t buggy_deref_addr_high_byte = read(ptr & 0xFF00) << 8;
    addr_abs_ = (buggy_deref_addr_high_byte | deref_addr_low_byte);
  } else {
    addr_abs_ = (deref_addr_high_byte | deref_addr_low_byte);
  }

  return 0;
}

// indirect addressing of zero page with X offset - an 8 bit address
// is supplied (somewhere in the zero page) which is added (without
// carry) to the X register. The resulting address is used as a
// pointer to the data being accessed
uint8_t CPU6502::IZX() {
  uint16_t supplied_addr = read(pc_);
  pc_++;
  uint16_t ptr = supplied_addr + x_;
  // since we are addressing in the zero page, and adding X without
  // carry, we scrap everything above 0xFF
  uint16_t deref_low_byte = read(ptr & 0x00FF);
  uint16_t deref_high_byte = read((ptr + 1) & 0x00FF);

  addr_abs_ = (deref_high_byte << 8 | deref_low_byte);
  return 0;
}

// indirect addressing of zero page with Y offset - for some retarded
// fucking reason this works differently than IZX. Instaed of adding Y
// to the supplied 8 bit address, we add the value of the Y register
// to the address that the supplied address points to.
uint8_t CPU6502::IZY() {
  uint16_t ptr = read(pc_);
  pc_++;

  uint16_t deref_addr_low_byte = read(ptr);
  uint16_t deref_addr_high_byte = read((ptr + 1) & 0x00FF) << 8;

  addr_abs_ = (deref_addr_high_byte | deref_addr_low_byte);
  addr_abs_ += y_;

  // if we crossed a page boundary, we must add an additional cycle
  if ((addr_abs_ & 0xFF00) != deref_addr_high_byte) {
    return 1;
  }
  return 0;
}

// relative addressing - an 8 bit signed offset is provided to be
// added to the program counter to find the effective address. this is
// limited to (-128, +127).
uint8_t CPU6502::REL() {
  addr_rel_ = read(pc_);
  pc_++;

  // if the sign is positive, set the high byte to 0xFF
  if (addr_rel_ & 0x80) {
    addr_rel_ |= 0xFF00;
  }
  return 0;
}

uint8_t CPU6502::fetch_data() {
  if (instructions_[opcode_].address_mode != &CPU6502::IMP) {
    data_ = read(addr_abs_);
  }
  return data_;
}

// instructions

// ADd memory to the accumulator with Carry
uint8_t CPU6502::ADC() {
  fetch_data();
  // we are casting everything to 16 bit ints so we can capture the carry bit (bit 8)
  uint16_t result = (uint16_t)a_ + (uint16_t)data_ + (uint16_t)getFlag(C);
  setFlag(C, result > 255);
  setFlag(Z, (result & 0x00FF) == 0);
  setFlag(N, result & 0x0080);
  // some dirty logic to determine if overflow has occurred
  setFlag(V, (~((uint16_t)a_ ^ (uint16_t)data_) & ((uint16_t)a_ ^ (uint16_t)result)) & 0x0080);
  a_ = result & 0x00FF;
  return 1;
}

// AND data with the contents of the accumulator, then store the
// result in the accumulator
uint8_t CPU6502::AND() {
  fetch_data();
  a_ = data_ & a_;
  setFlag(Z, a_ == 0x00);
  setFlag(N, a_ & 0x80);
  return 1;
}

// Arithmetic Shift Left (A << 1) or (M << 1)
// shift either the accumulator or data in memory left 1 bit. bit 7 is
// shifted into C. set N if bit 6 (pre shift) is on. set Z if result
// is 0.
uint8_t CPU6502::ASL() {
  // if the addressing mode is implied, we are operating on the accumulator
  if (instructions_[opcode_].address_mode == &CPU6502::IMP) {
    setFlag(C, a_ & 0x80);
    setFlag(N, a_ & 0x40);
    a_ <<= 1;
    setFlag(Z, a_ == 0);

  } else { // otherwise operating on memory
    fetch_data();
    setFlag(C, data_ & 0x80);
    setFlag(N, data_ & 0x40);
    data_ <<= 1;
    setFlag(Z, data_ == 0);
    write(addr_abs_, data_);
  }
  return 0;
}

// Branch if Carry Clear - branch if C == 0
uint8_t CPU6502::BCC() {
  if (getFlag(C)) {
    return 0;
  }

  // branches automatically add 1 to cycle count
  cycles_left_++;
  uint16_t addr = pc_ + addr_rel_;

  // add another cycle if we cross a page boundary
  if ((addr & 0xFF00) != (pc_ & 0xFF00)) {
    cycles_left_++;
  }

  pc_ = addr;
  return 0;
}

// Branch on Carry Set - branch if C == 1
uint8_t CPU6502::BCS() {
  if (!getFlag(C)) {
    return 0;
  }
  // branches automatically add 1 to cycle count
  cycles_left_++;
  uint16_t addr = pc_ + addr_rel_;

  // add another cycle if we cross a page boundary
  if ((addr & 0xFF00) != (pc_ & 0xFF00)) {
    cycles_left_++;
  }

  pc_ = addr;
  return 0;
}

// Branch if Equal - branch if Z == 1
uint8_t CPU6502::BEQ() {
  if (!getFlag(Z)) {
    return 0;
  }
  // branches automatically add 1 to cycle count
  cycles_left_++;
  uint16_t addr = pc_ + addr_rel_;

  // add another cycle if we cross a page boundary
  if ((addr & 0xFF00) != (pc_ & 0xFF00)) {
    cycles_left_++;
  }

  pc_ = addr;
  return 0;
}

// test BITs
// perform an AND betwwen a memory location and the accumulator, but
// does not store the result in the accumulator. N flag is set to the
// value of bit 7 of the memory being tested. V flag is set to bit 6
// of the memory being tested. If the result is 0, set Z flag.
uint8_t CPU6502::BIT() {
  fetch_data();
  setFlag(N, data_ & 0x80);
  setFlag(V, data_ & 0x40);
  setFlag(Z, (data_ & a_) == 0);
  return 0;
}

// Branch if Negative - branch if N == 1
uint8_t CPU6502::BMI() {
  if (!getFlag(N)) {
    return 0;
  }
  // branches automatically add 1 to cycle count
  cycles_left_++;
  uint16_t addr = pc_ + addr_rel_;

  // add another cycle if we cross a page boundary
  if ((addr & 0xFF00) != (pc_ & 0xFF00)) {
    cycles_left_++;
  }

  pc_ = addr;
  return 0;
}

// Branch if Not Equal - branch if Z == 0
uint8_t CPU6502::BNE() {
  if (getFlag(Z)) {
    return 0;
  }
  // branches automatically add 1 to cycle count
  cycles_left_++;
  uint16_t addr = pc_ + addr_rel_;

  // add another cycle if we cross a page boundary
  if ((addr & 0xFF00) != (pc_ & 0xFF00)) {
    cycles_left_++;
  }

  pc_ = addr;
  return 0;
}

// Branch if Positive - branch if N == 0
uint8_t CPU6502::BPL() {
  if (getFlag(N)) {
    return 0;
  }
  // branches automatically add 1 to cycle count
  cycles_left_++;
  uint16_t addr = pc_ + addr_rel_;

  // add another cycle if we cross a page boundary
  if ((addr & 0xFF00) != (pc_ & 0xFF00)) {
    cycles_left_++;
  }

  pc_ = addr;
  return 0;
}

// BReaK
// 6502.org says this runs in IMP, but other sources say IMM, not sure
// which is correct. The PC is incremented by 1 so that an RTI will go
// to the address of BRK +2 (also mentioned in the MCS6500
// Microcomputer Family Programminng Manual §9.11 pp. 144), so that
// BRK may be used to replace two-byte instrucsions for debugging and
// the subsequent RTI will be correct
uint8_t CPU6502::BRK() {
  setFlag(I, true);
  // store the PC of the second byte to the stack
  pc_++;
  uint8_t pc_high_byte = pc_ >> 8;
  uint8_t pc_low_byte = pc_ & 0x00FF;
  write(0x0100 + s_, pc_high_byte);
  s_--;
  write(0x0100 + s_, pc_low_byte);
  s_--;

  // write p_ to stack
  setFlag(U, true);
  setFlag(B, true);
  write(0x0100 + s_, p_);
  s_--;
  setFlag(B, false);

  // read new PC from 0xFFFA
  uint16_t new_pc_low_byte = read(0xFFFE);
  uint16_t new_pc_high_byte = read(0xFFFF) << 8;
  pc_ = new_pc_low_byte | new_pc_high_byte;

  return 0;
}

// Branch on oVerflow clear - branch if V == 0
uint8_t CPU6502::BVC() {
  if (getFlag(V)) {
    return 0;
  }
  // branches automatically add 1 to cycle count
  cycles_left_++;
  uint16_t addr = pc_ + addr_rel_;

  // add another cycle if we cross a page boundary
  if ((addr & 0xFF00) != (pc_ & 0xFF00)) {
    cycles_left_++;
  }

  pc_ = addr;
  return 0;
}

// Branch on oVerflow Set - branch if V == 1
uint8_t CPU6502::BVS() {
  if (!getFlag(V)) {
    return 0;
  }
  // branches automatically add 1 to cycle count
  cycles_left_++;
  uint16_t addr = pc_ + addr_rel_;

  // add another cycle if we cross a page boundary
  if ((addr & 0xFF00) != (pc_ & 0xFF00)) {
    cycles_left_++;
  }

  pc_ = addr;
  return 0;
}

// CLear the Carry bit
uint8_t CPU6502::CLC() {
  setFlag(C, false);
  return 0;
}

// CLear the Decimal bit
uint8_t CPU6502::CLD() {
  setFlag(D, false);
  return 0;
}

// CLear the Interrupt bit
uint8_t CPU6502::CLI() {
  setFlag(I, false);
  return 0;
}

// CLear the oVerflow bit
uint8_t CPU6502::CLV() {
  setFlag(V, false);
  return 0;
}

// CoMPare memory and accumulator (A - M)
// subtract the contents of memory from the contents of the
// accumulator. Z flag is set on equal comparison. N flag is set to
// the value of bit 7 of the result. C flag is set when the value in
// memory is less than or equal to the accumulator. This instruction
// does not affect the accumulator.
uint8_t CPU6502::CMP() {
  fetch_data();
  uint16_t result = (uint16_t)a_ - (uint16_t)data_;
  setFlag(Z, a_ == data_);
  setFlag(N, result & 0x80);
  setFlag(C, data_ <= a_);
  return 0;
}

// ComPare X register to memory (X - M)
// the result of this instruction is not stored anywhere and affect no
// registers. the carry flag is set if X => M. the N flag is set if
// the result contains bit 7. Z flag is set if X and M are equal.
uint8_t CPU6502::CPX() {
  fetch_data();
  uint16_t result = (uint16_t)x_ - (uint16_t)data_;
  setFlag(C, x_ >= data_);
  setFlag(N, result & 0x80);
  setFlag(Z, x_ == data_);
  return 0;
}

// ComPare Y register to memory (Y - M)
// the result of this instruction is not stored anywhere and affect no
// registers. the carry flag is set if Y => M. the N flag is set if
// the result contains bit 7. Z flag is set if Y and M are equal.
uint8_t CPU6502::CPY() {
  fetch_data();
  uint16_t result = (uint16_t)y_ - (uint16_t)data_;
  setFlag(C, y_ >= data_);
  setFlag(N, result & 0x80);
  setFlag(Z, y_ == data_);
  return 0;
}

// DECrement memory by 1. (M - 1 -> M)
// this instruction does not affect any registers, or C or V flags. if
// result contains bit 7, N is set. if the result of the decrement is
// 0, Z is set.
uint8_t CPU6502::DEC() {
  fetch_data();
  uint8_t result = data_ - 1;
  setFlag(N, result & 0x80);
  setFlag(Z, result == 0);
  write(addr_abs_, result);
  return 0;
}

// DEcrement X regiseter by 1. (X - 1 -> X)
// subtract 1 from the X register then store the result in X. does not
// affect C or V. set N if result has bit 7. set Z if result is 0.
uint8_t CPU6502::DEX() {
  x_--;
  setFlag(N, x_ & 0x80);
  setFlag(Z, x_ == 0);
  return 0;
}

// DEcrement Y regiseter by 1. (Y - 1 -> Y)
// subtract 1 from the Y register then store the result in Y. does not
// affect C or V. set N if result has bit 7. set Z if result is 0.
uint8_t CPU6502::DEY() {
  y_--;
  setFlag(N, y_ & 0x80);
  setFlag(Z, y_ == 0);
  return 0;
}

// Exclusive OR memory with accumulator (A XOR M -> A)
// perform a bit by bit xor between A and M and store the result in
// A. set Z if result is 0, set N if bit 7 is on.
uint8_t CPU6502::EOR() {
  fetch_data();
  a_ ^= data_;
  setFlag(Z, a_ == 0);
  setFlag(N, a_ & 0x80);
  return 1;
}

// INCrement memory by 1 (M + 1 -> M)
// does not affect any registers, C or V flags. set N if bit 7 is
// on. set Z if result is 0.
uint8_t CPU6502::INC() {
  fetch_data();
  data_++;
  write(addr_abs_, data_);
  setFlag(N, data_ & 0x80);
  setFlag(Z, data_ == 0);
  return 0;
}

// INcrement X register by 1 (X + 1 -> X)
// does not affect C or V. set N if bit 7 is on. set Z if result is 0.
uint8_t CPU6502::INX() {
  x_++;
  setFlag(N, x_ & 0x80);
  setFlag(Z, x_ == 0);
  return 0;
}

// INcrement Y register by 1 (Y + 1 -> Y)
// does not affect C or V. set N if bit 7 is on. set Z if result is 0.
uint8_t CPU6502::INY() {
  y_++;
  setFlag(N, y_ & 0x80);
  setFlag(Z, y_ == 0);
  return 0;
}

// JuMP to new location
// (PC + 1) -> PCL
// (PC + 1) -> PCH
// reads new program counter, byte by byte, from the address following
// the opcode. affects no flags
uint8_t CPU6502::JMP() {
  pc_ = addr_abs_;
  return 0;
}

// Jump to SubRoutine
// stores the PC pointing to the last byte of this instruction on the
// stack to allow the user to return to perform the next instrucstion
// after the subroutine is complete. affects no flags, causes S to be
// decremented by 2 and substitutes new values into the PC.
uint8_t CPU6502::JSR() {
  pc_--; // the last byte of this instruction
  uint8_t pcl = pc_ & 0x00FF;
  uint8_t pch = (pc_ & 0xFF00) >> 8;
  write(0x0100 + s_, pch);
  s_--;
  write(0x0100 + s_, pcl);
  s_--;
  pc_ = addr_abs_;
  return 0;
}

// LoaD Accumulator with memory (M -> A)
// does not affect C or V. set Z if A is 0, set N if A had bit 7 on.
uint8_t CPU6502::LDA() {
  a_ = fetch_data();
  setFlag(Z, a_ == 0);
  setFlag(N, a_ & 0x80);
  return 1;
}

// LoaD X register with memory (M -> X)
// does not affect C or V. set Z if A is 0, set N if A had bit 7 on.
uint8_t CPU6502::LDX() {
  x_ = fetch_data();
  setFlag(Z, x_ == 0);
  setFlag(N, x_ & 0x80);
  return 1;
}

// LoaD Y register with memory (M -> Y)
// does not affect C or V. set Z if A is 0, set N if A had bit 7 on.
uint8_t CPU6502::LDY() {
  y_ = fetch_data();
  setFlag(Z, y_ == 0);
  setFlag(N, y_ & 0x80);
  return 1;
}

// Logical Shift Right (A >> 1) or (M >> 1)
// shift either the accumulator or a a specified memory location 1 bit
// to the right. the rightmost bit is shifted into C. N is always
// reset. Z is set if result is 0.
uint8_t CPU6502::LSR() {
  // if the addressing mode is implied, we are operating on the accumulator
  if (instructions_[opcode_].address_mode == &CPU6502::IMP) {
    setFlag(C, a_ & 0x1);
    a_ >>= 1;
    setFlag(Z, a_ == 0);
  } else { // otherwise operating on memory
    fetch_data();
    setFlag(C, data_ & 0x1);
    data_ >>= 1;
    setFlag(Z, data_ == 0);
    write(addr_abs_, data_);
  }
  setFlag(N, false);
  return 0;
}

// No Operation
uint8_t CPU6502::NOP() {
  switch(opcode_) {
  case 0x1C:
  case 0x3C:
  case 0x5C:
  case 0x7C:
  case 0xDC:
  case 0xFC:
    return 1;
  }
  return 0;
}

// OR memory with Accumulator (A | M -> A)
// set Z if result is 0. set N if bit 7 is on
uint8_t CPU6502::ORA() {
  fetch_data();
  a_ |= data_;
  setFlag(Z, a_ == 0);
  setFlag(N, a_ & 0x80);
  return 1;
}

// PusH Accumulator on to stack
// transfer accumulator to the next location on the stack
uint8_t CPU6502::PHA() {
  write(0x0100 + s_, a_);
  s_--;
  return 0;
}

// PusH Processor status on stack
// affects no registers or flags
uint8_t CPU6502::PHP() {
  // for some reason these flags are supposed to be set. this is not
  // mentioned in the mos programming manual, but NESDEV says it is
  // so.
  bool b = getFlag(B);
  bool u = getFlag(U);
  setFlag(B, true);
  setFlag(U, true);
  write(0x0100 + s_, p_);
  s_--;
  setFlag(B, b);
  setFlag(U, u);
  return 0;
}

// PuLl Accumulator from stack
uint8_t CPU6502::PLA() {
  s_++;
  a_ = read(0x0100 + s_);
  setFlag(Z, a_ == 0);
  setFlag(N, a_ & 0x80);
  return 0;
}

// PuLl Processor status from stack
uint8_t CPU6502::PLP() {
  s_++;
  p_ = read(0x0100 + s_);
  setFlag(B, false);
  setFlag(U, true);
  return 0;
}

// ROtate Left
// shift either the accumulator or addressed memory left 1 bit, with
// original bit 7 being shifted into bit 0, also storing original bit 7
// in C. set N equal to original bit 6. set Z if result is 0.
uint8_t CPU6502::ROL() {
  // operating on accumulator
  if (instructions_[opcode_].address_mode == &CPU6502::IMP) {
    uint8_t c = getFlag(C);
    uint8_t new_c = (a_ & 0x80) >> 7; // get the carry bit
    setFlag(C, new_c); // set the carry flag
    setFlag(N, a_ & 0x40);
    setFlag(Z, a_ == 0);
    a_ <<= 1;
    a_ |= c;
  } else { // operating on memory
    fetch_data();
    uint8_t c = getFlag(C);
    uint8_t new_c = (data_ & 0x80) >> 7;
    setFlag(C, new_c);
    setFlag(N, data_ & 0x40);
    setFlag(Z, data_ == 0);
    data_ <<= 1;
    data_ |= c;
    write(addr_abs_, data_);
  }
  return 0;
}

// ROtate Right
// shift either the accumulator or addressed memory right 1 bit, with
// original bit 0 being shifted into bit 7, also storing original bit 0
// in C. set N equal to original bit 6. set Z if result is 0.
uint8_t CPU6502::ROR() {
  // operating on accumulator
  if (instructions_[opcode_].address_mode == &CPU6502::IMP) {
    uint8_t carry = getFlag(C);
    setFlag(C, a_ & 0x01);
    a_ >>= 1;

    // set bit 7 to C
    if (carry) {
      a_ |= (1 << 7);
    } else {
      a_ &= ~(1 << 7);
    }

    setFlag(N, a_ & 0x80);
    setFlag(Z, a_ == 0);
  } else { // operating on memory
    fetch_data();
    uint8_t carry = getFlag(C);
    setFlag(C, data_ & 0x01);
    data_ >>= 1;

    // set bit 7 to C
    if (carry) {
      data_ |= (1 << 7);
    } else {
      data_ &= ~(1 << 7);
    }

    setFlag(N, data_ & 0x80);
    setFlag(Z, data_ == 0);
    write(addr_abs_, data_);
  }
  return 0;
}

// ReTurn from Interrupt
uint8_t CPU6502::RTI() {
  // read the status register from the stack
  s_++;
  p_ = read(0x0100 + s_);
  setFlag(B, false);
  setFlag(U, true);

  // read the program counter from the stack
  s_++;
  uint16_t pc_low_byte = read(0x0100 + s_);
  s_++;
  uint16_t pc_high_byte = read(0x0100 + s_) << 8;
  pc_ = pc_high_byte | pc_low_byte;

  return 0;
}

// ReTurn from Subroutine
// load PC from stack, increment PC so it points to the instruction
// following JSR.
uint8_t CPU6502::RTS() {
  s_++;
  uint16_t pc_low = read(0x0100 + s_);
  s_++;
  uint16_t pc_high = read(0x0100 + s_) << 8;
  pc_ = pc_high | pc_low;
  pc_++;
  return 0;
}

// SuBtract memory from the accumulator with Carry
uint8_t CPU6502::SBC() {
  fetch_data();

  uint16_t inverted_data = ((uint16_t)data_) ^ 0x00FF;

  // we are casting everything to 16 bit ints so we can capture the carry bit (bit 8)
  uint16_t result = (uint16_t)a_ + (uint16_t)inverted_data + (uint16_t)getFlag(C);
  setFlag(C, result > 255);
  setFlag(Z, (result & 0x00FF) == 0);
  setFlag(N, result & 0x0080);
  // some dirty logic to determine if overflow has occurred
  setFlag(V, (~((uint16_t)a_ ^ (uint16_t)inverted_data) & ((uint16_t)a_ ^ (uint16_t)result)) & 0x0080);
  a_ = result & 0x00FF;
  return 1;
}

// SEt Carry flag
uint8_t CPU6502::SEC() {
  setFlag(C, true);
  return 0;
}

// SEt Decimal flag
uint8_t CPU6502::SED() {
  setFlag(D, true);
  return 0;
}

// SEt Interrupt flag
uint8_t CPU6502::SEI() {
  setFlag(I, true);
  return 0;
}

// STore Accumulator in memory (A -> M)
uint8_t CPU6502::STA() {
  write(addr_abs_, a_);
  return 0;
}

// STore X register in memory (X -> M)
// transfer value of X into addressed memory location. no flags or
// registers are affected.
uint8_t CPU6502::STX() {
  write(addr_abs_, x_);
  return 0;
}

// STore Y register in memory (Y -> M)
// transfer value of Y into addressed memory location. no flags or
// registers are affected.
uint8_t CPU6502::STY() {
  write(addr_abs_, y_);
  return 0;
}

// Transfer Accumulator to X register (A -> X)
// set N if bit 7 is on. set Z if result is 0.
uint8_t CPU6502::TAX() {
  x_ = a_;
  setFlag(N, x_ & 0x80);
  setFlag(Z, x_ == 0);
  return 0;
}

// Transfer Accumulator to Y register (A -> Y)
// set N if bit 7 is on. set Z if result is 0.
uint8_t CPU6502::TAY() {
  y_ = a_;
  setFlag(N, y_ & 0x80);
  setFlag(Z, y_ == 0);
  return 0;
}

// Transfer Stack pointer to X register (S -> X)
// set N if bit 7 is on. set Z if result is 0
uint8_t CPU6502::TSX() {
  x_ = s_;
  setFlag(N, x_ & 0x80);
  setFlag(Z, x_ == 0);
  return 0;
}

// Transfer X register to Accumulator (X -> A)
// set N if bit 7 is on. set Z if result is 0
uint8_t CPU6502::TXA() {
  a_ = x_;
  setFlag(N, a_ & 0x80);
  setFlag(Z, a_ == 0);
  return 0;
}

// Transfer X register to Stack pointer (X -> S)
uint8_t CPU6502::TXS() {
  s_ = x_;
  return 0;
}

// Transfer Y register to Accumulator (Y -> A)
// set N if bit 7 is on, set Z if result is 0
uint8_t CPU6502::TYA() {
  a_ = y_;
  setFlag(N, a_ & 0x80);
  setFlag(Z, a_ == 0);
  return 0;
}

// ILLegal opcode
uint8_t CPU6502::ILL() {
  return 0;
}

///////////////////////////////
// UNDOCUMENTED INSTRUCTIONS //
///////////////////////////////

// Load memory into Accumulator and X register
// does not affect C or V. set Z if data is 0, set N if data had bit 7 on.
uint8_t CPU6502::LAX() {
  fetch_data();
  x_ = data_;
  a_ = data_;
  setFlag(Z, data_ == 0);
  setFlag(N, data_ & 0x80);
  return 1;
}

uint8_t CPU6502::SAX() {
  write(addr_abs_, a_ & x_);
  return 0;
}

// DEC followed by CMP
uint8_t CPU6502::DCP() {
  fetch_data();
  data_--;
  write(addr_abs_, data_);
  uint16_t result = (uint16_t)a_ - (uint16_t)data_;
  setFlag(Z, a_ == data_);
  setFlag(N, result & 0x80);
  setFlag(C, data_ <= a_);
  return 0;
}

// INC followed by SBC
uint8_t CPU6502::ISB() {
  fetch_data();
  // INC
  data_++;
  write(addr_abs_, data_);

  // SBC
  uint16_t inverted_data = ((uint16_t)data_) ^ 0x00FF;
  // we are casting everything to 16 bit ints so we can capture the carry bit (bit 8)
  uint16_t result = (uint16_t)a_ + (uint16_t)inverted_data + (uint16_t)getFlag(C);
  setFlag(C, result > 255);
  setFlag(Z, (result & 0x00FF) == 0);
  setFlag(N, result & 0x0080);
  // some dirty logic to determine if overflow has occurred
  setFlag(V, (~((uint16_t)a_ ^ (uint16_t)inverted_data) & ((uint16_t)a_ ^ (uint16_t)result)) & 0x0080);
  a_ = result & 0x00FF;
  return 0;
}

// ASL followed by ORA
uint8_t CPU6502::SLO() {
  ASL();
  ORA();
  return 0;
}

// ROL followed by AND
uint8_t CPU6502::RLA() {
  ROL();
  AND();
  return 0;
}

////////////////
// INTERRUPTS //
////////////////

void CPU6502::reset() {
  a_ = 0;
  x_ = 0;
  y_ = 0;
  s_ = 0xFD;
  // U and I are set to 1
  p_ = 0x00 | U;
  p_ |= I;

  // we always read the program counter reset value from 0xFFFC
  uint16_t addr = 0xFFFC;
  uint16_t low_byte = read(addr);
  uint16_t high_byte = read(addr + 1) << 8;

  pc_ = high_byte | low_byte;

  addr_rel_ = 0x0000;
  addr_abs_ = 0x0000;
  data_ = 0x00;

  cycles_left_ = 8;
}

void CPU6502::irq() {
  if (getFlag(I)) {
    return;
  }
  // store the program counter on the stack
  uint8_t pc_low_byte = (pc_ & 0x00FF);
  uint8_t pc_high_byte = (pc_ & 0xFF00) >> 8;
  write(0x0100 + s_, pc_low_byte);
  s_--;
  write(0x0100 + s_, pc_high_byte);
  s_--;

  // store the status register on the stack
  setFlag(B, 0);
  setFlag(U, 1);
  setFlag(I, 1);
  write(0x0100 + s_, p_);
  s_--;

  // we always read the new program counter from 0xFFFE
  uint16_t addr = 0xFFFE;
  uint16_t new_pc_low_byte = read(addr);
  uint16_t new_pc_high_byte = read(addr + 1) << 8;
  pc_ = new_pc_high_byte | new_pc_low_byte;

  cycles_left_ = 7;
}

void CPU6502::nmi() {
   // store the program counter on the stack
  uint8_t pc_low_byte = (pc_ & 0x00FF);
  uint8_t pc_high_byte = (pc_ & 0xFF00) >> 8;
  write(0x0100 + s_, pc_low_byte);
  s_--;
  write(0x0100 + s_, pc_high_byte);
  s_--;

  // store the status register on the stack
  setFlag(B, 0);
  setFlag(U, 1);
  setFlag(I, 1);
  write(0x0100 + s_, p_);
  s_--;

  // we always read the new program counter from 0xFFFA
  uint16_t addr = 0xFFFA;
  uint16_t new_pc_low_byte = read(addr);
  uint16_t new_pc_high_byte = read(addr + 1) << 8;
  pc_ = new_pc_high_byte | new_pc_low_byte;

  cycles_left_ = 8;
}

void CPU6502::setPC(uint16_t addr) {
  pc_ = addr;
}

INSTRUCTION CPU6502::opcodeLookup(uint8_t opcode) {
  return instructions_[opcode];
}

int CPU6502::getCycle() {
  return cycles_;
}

bool CPU6502::isIdle() {
  return cycles_left_ == 0;
}
