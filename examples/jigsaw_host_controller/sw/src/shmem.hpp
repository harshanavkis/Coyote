#ifndef SHMEM_HPP
#define SHMEM_HPP

#include <coyote/cThread.hpp>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
#define CLOCK_PERIOD_NS      4
#define DEFAULT_VFPGA_ID     0
#define RDMA_BUFFER_SIZE     (1024 * 1024)  // 1 MiB


// ---------------------------------------------------------------------------
// AXI-Lite register map — jigsaw_hc_axi_ctrl_parser
// ---------------------------------------------------------------------------
enum class HCReg : uint32_t {
    MMIO_VADDR          = 0,
    MMIO_CTRL           = 1,
    MMIO_WRITE_STATUS   = 2,
    MMIO_READ_STATUS    = 3,
    COYOTE_PID          = 4,
    REMOTE_VADDR        = 5,
};


/* Some macros for the shared memory and its structure 
 */

#define SHMEM_FILE "/dev/shm/ivshmem"  // Adjust this path as needed
#define SHMEM_SIZE (1 << 20)  // 1 MB, adjust as needed
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
