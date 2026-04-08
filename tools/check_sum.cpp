#include "check_sum.hpp"

namespace tools 
{
//definition
bool verify_check_sum16(uint8_t * pchMessage, uint32_t dwLength)
{
  if (dwLength <= 2) {
    return false;
  }
  uint16_t cSUm = 0;
  for (uint32_t i = 0; i < dwLength - 2; i++) {
    cSUm += pchMessage[i];
  }

  uint16_t fSum = (static_cast<uint16_t>(pchMessage[dwLength - 2]) & 0xFF) |
                         (static_cast<uint16_t>(pchMessage[dwLength - 1]) << 8);

  return cSUm == fSum;
}

void append_check_sum(uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t cSUm = 0;
  for (uint32_t i = 0; i < dwLength - 3; i++) {
    cSUm += pchMessage[i];
  }
  // 因为电控校验需要先校验tail，因此校验码放在倒数第二、三个字节
  pchMessage[dwLength - 3] = (uint8_t)(cSUm & 0xFF);
  pchMessage[dwLength - 2] = (uint8_t)((cSUm >> 8) & 0xFF);
}
}
