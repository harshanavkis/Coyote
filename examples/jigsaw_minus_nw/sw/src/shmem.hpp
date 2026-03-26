#ifndef SHMEM_HPP
#define SHMEM_HPP

#include <coyote/cThread.hpp> // cThread class

/* FPGA macros */
// Constants
#define CLOCK_PERIOD_NS 4
#define DEFAULT_VFPGA_ID 0

#define N_LATENCY_REPS 1
#define N_THROUGHPUT_REPS 32

// Registers for jigsaw_host_controller based on jigsaw_minus_nw_axi_ctrl_parser
enum class JigsawHostControlRegisters : uint32_t {
    MMIO_VADDR_REG = 0,
    MMIO_CTRL_REG = 1,
    MMIO_WRITE_STATUS_REG = 2,
    MMIO_READ_STATUS_REG = 3,
    COYOTE_PID_REG = 4,
    MMIO_OP_REG = 5,
    MMIO_ADDR_REG = 6,
    MMIO_DATA_REG = 7,
    MMIO_READ_DATA_REG = 8
};



/* Some macros for the shared memory and its structure 
 */

#define SHMEM_FILE "/dev/shm/ivshmem"  // Adjust this path as needed
#define SHMEM_SIZE (1 << 21)  // 2 MB, adjust as needed
#define READ_DOORBELL_OFFSET 0
#define WRITE_DOORBELL_OFFSET 1
#define DOORBELL_SIZE 1  // 1 byte for each doorbell
#define TOTAL_DOORBELL_SIZE (DOORBELL_SIZE * 2)
#define MMIO_REGION_OFFSET (24)
#define DMA_REGION_OFFSET (1 << 12) // 4K aligned
#define DMA_SIZE (SHMEM_SIZE - DMA_REGION_OFFSET)

/* Offsets in the shared memory with special values */
#define OFFSET_PROXY_SHMEM (256)

void *init_shared_memory();

void *run_shmem_app(coyote::cThread &coyote_thread);

/**
 * @brief Operation code for read requests
 */
#define OP_READ 0

/**
 * @brief Operation code for write requests
 */
#define OP_WRITE 1

/**
 * @brief Structure representing the message header
 */
struct mmio_message_header
{
    uint8_t operation; /** Operation type */
    uint64_t address;  /** Memory address for the operation */
    uint64_t length;   /** Length of data to read or write */
    uint64_t value;    /** Value in case of optype write **/
} __attribute__((packed));

#endif // SHMEM_HPP
