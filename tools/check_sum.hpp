#ifndef TOOLS__CHECK_SUM_HPP
#define TOOLS__CHECK_SUM_HPP

#include <cstdint>

namespace tools 
{
bool verify_check_sum16(uint8_t * pchMessage, uint32_t dwLength);

void append_check_sum(uint8_t * pchMessage, uint32_t dwLength);
}

#endif  // TOOLS__CHECK_SUM_HPP

