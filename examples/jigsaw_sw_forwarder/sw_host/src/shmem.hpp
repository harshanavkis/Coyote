#ifndef JIGSAW_SW_FORWARDER_SHMEM_HPP
#define JIGSAW_SW_FORWARDER_SHMEM_HPP

#include <cstdint>

/*
 * ivshmem protocol shared with the guest — identical layout and semantics to
 * jigsaw_host_controller/sw (doorbells, message slot, proxy pointer), so the
 * VM stack (QEMU disagg-fake-pci + guest kernel) runs unchanged against the
 * software forwarder.
 */

// hugetlbfs-backed so the guest's DMA region uses 2 MiB pages. Path must
// match the QEMU memory-backend-file mem-path in
// jigsaw-overall/scripts/run/vm.sh.
#define SHMEM_FILE "/dev/hugepages/ivshmem"
// Must stay in lockstep with SHMEM_SIZE in spdm-linux
// include/misc/qemu_ivshmem.h and the QEMU ivshmem -object size.
#define SHMEM_SIZE (1 << 24)  // 16 MiB
#define READ_DOORBELL_OFFSET 0
#define WRITE_DOORBELL_OFFSET 1
#define DOORBELL_SIZE 1  // 1 byte for each doorbell
#define TOTAL_DOORBELL_SIZE (DOORBELL_SIZE * 2)
#define MMIO_REGION_OFFSET (24)
#define DMA_REGION_OFFSET (1 << 12)  // 4K aligned

/* Offsets in the shared memory with special values */
#define OFFSET_PROXY_SHMEM (256)

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

void *init_shared_memory();

// Doorbell helpers for the forwarder main loop
void shmem_arm_write_doorbell();
void shmem_wait_write_doorbell();
void shmem_read_header(struct mmio_message_header *header);
void shmem_complete_read(uint64_t value);

#endif // JIGSAW_SW_FORWARDER_SHMEM_HPP
