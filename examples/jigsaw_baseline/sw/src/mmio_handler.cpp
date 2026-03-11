#include "mmio_handler.hpp"

#include <cstring>
#include <cstdint>
#include <cstdio>

void edu_mmio_read(coyote::cThread &coyote_thread, char *data, uint64_t offset)
{
    uint32_t reg = offset / 8;

    uint64_t val;

    switch (reg) {
        case static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG):
            val = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::DMA_SRC_ADDR_REG):
            val = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_SRC_ADDR_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::DMA_DST_ADDR_REG):
            val = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_DST_ADDR_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::DMA_LEN_REG):
            val = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_LEN_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG):
            val = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::START_COMPUTATION_REG):
            val = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::START_COMPUTATION_REG));
            break;
        default:
            val = 0;
            break;
    }

    std::printf("edu_mmio_read with register %lu, got value %x\n", reg, val);

    std::memcpy(data, &val, sizeof(val));
}

void edu_mmio_write(coyote::cThread &coyote_thread, char *data, uint64_t offset)
{
    uint32_t reg = offset / 8;
    
    uint64_t val;
    std::memcpy(&val, data, sizeof(val));

    std::printf("edu_mmio_write with register %lu, val: %x\n", reg, val);

    switch (reg) {
        case static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG):
            coyote_thread.setCSR(val, static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::DMA_SRC_ADDR_REG):
            coyote_thread.setCSR(val, static_cast<uint32_t>(JigsawRegisters::DMA_SRC_ADDR_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::DMA_DST_ADDR_REG):
            coyote_thread.setCSR(val, static_cast<uint32_t>(JigsawRegisters::DMA_DST_ADDR_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::DMA_LEN_REG):
            coyote_thread.setCSR(val, static_cast<uint32_t>(JigsawRegisters::DMA_LEN_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG):
            coyote_thread.setCSR(val, static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::START_COMPUTATION_REG):
            coyote_thread.setCSR(val, static_cast<uint32_t>(JigsawRegisters::START_COMPUTATION_REG));
            break;
        default:
            break;
    }
}

