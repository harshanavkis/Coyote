#include "mmio_handler.hpp"

#include <cstring>
#include <cstdint>
#include <cstdio>

void edu_mmio_read(coyote::cThread &coyote_thread, char *data, uint64_t offset)
{
    uint32_t reg = offset / sizeof(uint64_t);
    uint64_t val = 0;

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
        case static_cast<uint32_t>(JigsawRegisters::DMA_H2D_LEN_REG):
            val = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_H2D_LEN_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG):
            val = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::START_COMPUTATION_REG):
            val = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::START_COMPUTATION_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::CYCLES_PER_COMPUTATION_REG):
            val = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::CYCLES_PER_COMPUTATION_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::COYOTE_PID_REG):
            val = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::COYOTE_PID_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::COYOTE_DMA_TX_LEN_REG):
            val = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::COYOTE_DMA_TX_LEN_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::DMA_D2H_LEN_REG):
            val = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_D2H_LEN_REG));
            break;
        default:
            break;
    }

    std::printf("edu_mmio_read offset 0x%lx -> csr %u, got value 0x%lx\n", offset, reg, val);

    std::memcpy(data, &val, sizeof(val));
}

void edu_mmio_write(coyote::cThread &coyote_thread, char *data, uint64_t offset)
{
    uint32_t reg = offset / sizeof(uint64_t);
    
    uint64_t val;
    std::memcpy(&val, data, sizeof(val));

    std::printf("edu_mmio_write offset 0x%lx -> csr %u, val: 0x%lx\n", offset, reg, val);

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
        case static_cast<uint32_t>(JigsawRegisters::DMA_H2D_LEN_REG):
            coyote_thread.setCSR(val, static_cast<uint32_t>(JigsawRegisters::DMA_H2D_LEN_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG):
            coyote_thread.setCSR(val, static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::START_COMPUTATION_REG):
            coyote_thread.setCSR(val, static_cast<uint32_t>(JigsawRegisters::START_COMPUTATION_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::CYCLES_PER_COMPUTATION_REG):
            coyote_thread.setCSR(val, static_cast<uint32_t>(JigsawRegisters::CYCLES_PER_COMPUTATION_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::COYOTE_PID_REG):
            coyote_thread.setCSR(coyote_thread.getCtid(), static_cast<uint32_t>(JigsawRegisters::COYOTE_PID_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::COYOTE_DMA_TX_LEN_REG):
            coyote_thread.setCSR(val, static_cast<uint32_t>(JigsawRegisters::COYOTE_DMA_TX_LEN_REG));
            break;
        case static_cast<uint32_t>(JigsawRegisters::DMA_D2H_LEN_REG):
            coyote_thread.setCSR(val, static_cast<uint32_t>(JigsawRegisters::DMA_D2H_LEN_REG));
            break;
        default:
            break;
    }
}
