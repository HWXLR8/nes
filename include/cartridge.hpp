#pragma once

#include <string>
#include <vector>

#include <cstdint>

class Cartridge {
public:
  Cartridge(std::string path);
  std::vector<uint8_t> getPRG();
  int getPRGBankCount();
  uint8_t read(uint16_t addr);

private:
  uint8_t mapper_id_;
  uint8_t PRG_bank_count_;
  int PRG_size_;
  uint8_t CHR_BANKS_;
  std::vector<uint8_t> PRG_ROM_;
  std::vector<uint8_t> CHR_ROM_;
};
