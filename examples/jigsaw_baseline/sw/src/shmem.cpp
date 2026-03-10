#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cstdint>

#include "shmem.hpp"
#include "mmio_handler.hpp"

#include "cThread.hpp"

static void *shmem = NULL;
static volatile uint8_t *read_doorbell = NULL;
static volatile uint8_t *write_doorbell = NULL;

static int create_or_open_shmem_file() {
    int fd = open(SHMEM_FILE, O_RDWR | O_CREAT, 0666);
    if (fd < 0) {
        perror("Failed to open or create shared memory file");
        return -1;
    }

    struct stat st;
    if (fstat(fd, &st) < 0) {
        perror("Failed to get file status");
        close(fd);
        return -1;
    }

    if (st.st_size != SHMEM_SIZE) {
        if (ftruncate(fd, SHMEM_SIZE) < 0) {
            perror("Failed to set file size");
            close(fd);
            return -1;
        }
        printf("Created shared memory file with size %d bytes\n", SHMEM_SIZE);
    } else {
        printf("Opened existing shared memory file with correct size\n");
    }

    return fd;
}

void *init_shared_memory() {
    int fd = create_or_open_shmem_file();
    if (fd < 0) {
        return nullptr;
    }

    shmem = mmap(NULL, SHMEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shmem == MAP_FAILED) {
        perror("Failed to mmap shared memory");
        close(fd);
        return nullptr;
    }

    read_doorbell = (volatile uint8_t *)shmem + READ_DOORBELL_OFFSET;
    write_doorbell = (volatile uint8_t *)shmem + WRITE_DOORBELL_OFFSET;
    close(fd);

    // Initialize doorbells to 0
    *read_doorbell = 0;
    *write_doorbell = 0;

    // write proxyShmem address into shmem
    *reinterpret_cast<uint64_t *>((reinterpret_cast<char *>(shmem) + OFFSET_PROXY_SHMEM)) = reinterpret_cast<uint64_t>(shmem) + DMA_REGION_OFFSET;

    msync(shmem, TOTAL_DOORBELL_SIZE, MS_SYNC);

    return shmem;
}

static void wait_for_write_doorbell_set() {
    while (__atomic_load_n(write_doorbell, __ATOMIC_ACQUIRE) == 0) { }
}

static void wait_for_read_doorbell_clear() {
    while (__atomic_load_n(read_doorbell, __ATOMIC_ACQUIRE) != 0) { }
}

static int ivshmem_mmio_region_read(void *buf) {
    uint8_t type;

    wait_for_write_doorbell_set();

    memcpy(&type, reinterpret_cast<char *>(shmem) + MMIO_REGION_OFFSET, 1);
    memcpy(buf, reinterpret_cast<char *>(shmem) + MMIO_REGION_OFFSET + 1, sizeof(struct mmio_message_header));

    __atomic_store_n(write_doorbell, 0, __ATOMIC_RELEASE);

    return 0;
}

static int ivshmem_mmio_region_write(void *buf, size_t count) {

    // Op-type can only be OP_READ
    uint8_t type = OP_READ;

    wait_for_read_doorbell_clear();

    memcpy(reinterpret_cast<char *>(shmem) + MMIO_REGION_OFFSET, &type, 1);
    memcpy(reinterpret_cast<char *>(shmem) + MMIO_REGION_OFFSET + 1, buf, count);

    __atomic_store_n(read_doorbell, 1, __ATOMIC_RELEASE);

    return 0;
}

void *run_shmem_app(coyote::cThread &coyote_thread) {

    printf("connection.c: In shmem app\n");

    printf("SHMEM application started. Waiting for messages...\n");

    char data[8];
    loff_t offset;

    while (1) {
        mmio_message_header header;
        if (ivshmem_mmio_region_read(&header) != 0) {
            perror("Failed to read message");
            continue;
        }

#ifdef CONFIG_DISAGG_DEBUG_MMIO
        printf("Received message: operation=%u, address=0x%lx, length=%u\n",
                header.operation, header.address, header.length);
#endif

        switch (header.operation) {

            case OP_READ:

                offset = header.address;
                edu_mmio_read(coyote_thread, data, offset);

                if (ivshmem_mmio_region_write(data, sizeof(data)) != 0) {
                    perror("Failed to write response");
                    continue;
                }
                continue;

            case OP_WRITE:

                memcpy(data, &header.value, sizeof(header.value));

                offset = header.address;
                edu_mmio_write(coyote_thread, data, offset);

                continue;

            default:
                fprintf(stderr, "Unknown operation: %d\n", header.operation);
                continue;
        }

        printf("Response sent. Waiting for next message...\n");
    }

    munmap(shmem, SHMEM_SIZE);
}

