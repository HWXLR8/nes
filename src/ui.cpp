#include <ui.hpp>

#include <iomanip>
#include <ios>
#include <iostream>

#include <helper.hpp>

UI::UI(CPU6502* cpu, Cartridge* cart) {
  cpu_ = cpu;
  cart_ = cart;

  parseProgram(0x8000);
  parseProgram(0xC000);

  next_instruction_ = 0;
}

void UI::nextInstruction() {
  // abort if the there is still an instruction in progress
  if (!cpu_->isIdle()) {
    return;
  }
  // abort if the program counter has not changed
  if (cpu_->getRegister(PC) == next_instruction_) {
    return;
  }

  next_instruction_ = cpu_->getRegister(PC);
  program_context_.clear();

  int c = 0;
  for (auto& [addr, line] : program_) {
    if (addr == next_instruction_ || c > 0 && c < 10) {
      program_context_.push_back(line);
      c++;
    }
  }

  // we subtract 1 from the current cycle since we are on the "idle"
  // cycle, ie. 1 cycle _after_ the instruction is complete
  std::cout << std::left << std::setw(32) << std::setfill(' ') << program_context_[0]
	    << "A:" << std::uppercase << std::hex << std::right << std::setw(2) << std::setfill('0') << cpu_->getRegister(A) << " "
	    << "X:" << std::uppercase << std::hex << std::setw(2) << cpu_->getRegister(X) << " "
	    << "Y:" << std::uppercase << std::hex << std::setw(2) << cpu_->getRegister(Y) << " "
	    << "P:" << std::uppercase << std::hex << std::setw(2) << cpu_->getRegister(P) << " "
	    << "SP:" << std::uppercase << std::hex << std::setw(2) << cpu_->getRegister(S) << " "
    	    << "CYC:" << std::dec << std::setw(1) << cpu_->getCycle() - 1 << " "
	    << std::endl;
}

// parse the program (vector of uint8_t) and return it in a human
// readable from as a map of program address -> opcodes w/ arguments
// (uint16_t -> string). I am not proud of this
void UI::parseProgram(uint16_t init_addr) {
  // string representation of the instruction
  std::string line;
  // maximum number of leading zeros for hex numbers
  int hex_padding = 4;
  // bytes remaining to be read for the given instruction
  int bytes_left = 0;
  // program counter
  uint16_t pc = init_addr;
  // the address of the actual opcode in memory
  uint16_t op_addr = init_addr;
  // the argument supplied to the instruction
  uint16_t arg = 0x0000;
  // the size of the argument in bits
  int arg_size;
  // instruction struct inferred from opcode
  INSTRUCTION i;
  // are we using a relative addressing mode?
  bool relative_addr = false;

  for (uint8_t byte : cart_->getPRG()) {
    // we have read the opcode, but still require more bytes for the
    // complete instruction
    if (bytes_left != 0) {
      // read the lower byte of 16 bit arg
      if (arg_size == 16 && bytes_left == 2) {
	arg |= byte;
	// read the upper byte of 16 bit arg
      } else if (arg_size == 16 && bytes_left == 1) {
	arg |= (byte << 8);
	// read the argument for 8 bit arg
      } else {
	arg |= byte;
      }
      bytes_left--;
      // reached the end of the instruction and argument
      if (bytes_left == 0) {
	// relative addresses use the argument as an offset to the PC
	if (relative_addr) {
	  arg += pc + 1;
	}
	line += hex2str(arg, hex_padding);
	program_.insert({op_addr, line});
	arg = 0x0000;
      }
      pc++;
      // if we have exceeded 16K, we're done
      if (pc >= init_addr + 0x4000 - 1) {
	return;
      }
      // skip loop interation until we have read the full instruction
      // with arguments
      continue;
    }

    line = "";
    i = cpu_->opcodeLookup(byte);

    // no argument
    if (i.address_mode == &CPU6502::IMP) {
      relative_addr = false;
      op_addr = pc;
      program_.insert({op_addr, i.name});

      // 1 byte argument
    } else if (i.address_mode == &CPU6502::IMM ||
	       i.address_mode == &CPU6502::ZP0 ||
	       i.address_mode == &CPU6502::ZPX ||
	       i.address_mode == &CPU6502::ZPY ||
	       i.address_mode == &CPU6502::REL ||
	       i.address_mode == &CPU6502::IZX ||
	       i.address_mode == &CPU6502::IZY) {
      arg_size = 8;
      hex_padding = 4;
      bytes_left = 1;
      line += i.name + " ";
      if (i.address_mode == &CPU6502::IMM) {
	line += "#";
	hex_padding = 2;
      }
      // relative addressing mode means we need to treat the argument
      // differently
      if (i.address_mode == &CPU6502::REL) {
	relative_addr = true;
      } else {
	relative_addr = false;
      }
      line += "$";
      op_addr = pc;

      // 2 byte argument
    } else if (i.address_mode == &CPU6502::ABS ||
	       i.address_mode == &CPU6502::ABX ||
	       i.address_mode == &CPU6502::ABY ||
	       i.address_mode == &CPU6502::IND) {
      relative_addr = false;
      arg_size = 16;
      hex_padding = 4;
      bytes_left = 2;
      line += i.name + " $";
      op_addr = pc;
    }

    pc++;
  }
  return;
}
