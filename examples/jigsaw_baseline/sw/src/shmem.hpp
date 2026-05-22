#ifndef SHMEM_HPP
#define SHMEM_HPP

#include <coyote/cThread.hpp> // cThread class

/* Some macros for the shared memory and its structure 
 */

// hugetlbfs-backed so the daemon's userMap region uses 2 MiB pages instead
// of 4 KiB tmpfs pages. Without this, the vFPGA takes ~1024 page faults per
// 4 MiB H2D, which serializes behind the host_controller's MMIO arbitration
// and stalls the kernel's DMA_STATUS poll loop. Path must match the QEMU
// memory-backend-file mem-path in jigsaw-overall/scripts/run/vm.sh.
#define SHMEM_FILE "/dev/hugepages/ivshmem"
// Sized to fit JIGSAW_TRACE_MAX_BYTES (~14 MiB) + 4 KiB DMA offset. Must
// stay in lockstep with SHMEM_SIZE in spdm-linux/include/misc/qemu_ivshmem.h
// and the QEMU ivshmem -object size in jigsaw-overall/scripts/run/vm.sh.
#define SHMEM_SIZE (1 << 24)  // 16 MiB
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
