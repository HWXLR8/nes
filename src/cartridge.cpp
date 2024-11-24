#include <cartridge.hpp>

#include <filesystem>
#include <fstream>

Cartridge::Cartridge(std::string path) {
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("no such file or directory: " + path);
  }
  std::ifstream file(path, std::ios::binary);
  unsigned char byte;
  uint16_t offset = 0x0004;
  file.seekg(offset);
  file.read(reinterpret_cast<char*>(&byte), 1);

  // number of PRG-ROM banks
  PRG_bank_count_ = static_cast<uint8_t>(byte);

  // PRG-ROM size in bytes
  PRG_size_ = PRG_bank_count_ * 16384;

  // jump past header
  file.seekg(0x0010);

  // read PRG ROM
  for (int c = 1; c <= PRG_size_; c++) {
    file.read(reinterpret_cast<char*>(&byte), 1);
    PRG_ROM_.push_back(static_cast<uint8_t>(byte));
  }

  // copy lower bank into upper bank if only 1 16K PRG ROM is present
  if (PRG_bank_count_ == 1) {
    file.seekg(0x0010);
    for (int c = 1; c <= PRG_size_; c++) {
      file.read(reinterpret_cast<char*>(&byte), 1);
      PRG_ROM_.push_back(static_cast<uint8_t>(byte));
    }
  }
}

std::vector<uint8_t> Cartridge::getPRG() {
  return PRG_ROM_;
}

int Cartridge::getPRGBankCount() {
  return PRG_bank_count_;
}

uint8_t Cartridge::read(uint16_t addr) {
  addr -= 0x8000;
  return PRG_ROM_[addr];
}
