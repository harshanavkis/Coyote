#ifndef MMIO_HANDLER_HPP
#define MMIO_HANDLER_HPP

#include <cstdint>

#include "cThread.hpp"

// Registers, corresponding to registers defined the vFPGA
enum class JigsawRegisters: uint32_t {
    DMA_CMD_REG = 0,
    DMA_SRC_ADDR_REG = 1,
    DMA_DST_ADDR_REG = 2,
    DMA_LEN_REG = 3,
    DMA_STATUS_REG = 4,
    START_COMPUTATION_REG = 5,
    CYCLES_PER_COMPUTATION_REG = 6,
    COYOTE_PID_REG = 7,
    COYOTE_DMA_TX_LEN_REG = 8
};

void edu_mmio_write(coyote::cThread &coyote_thread, char *data, uint64_t offset);
void edu_mmio_read(coyote::cThread &coyote_thread, char *data, uint64_t offset);

#endif // MMIO_HANDERL_HPP
