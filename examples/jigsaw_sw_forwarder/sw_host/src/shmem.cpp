#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include <cstdio>
#include <cstring>
#include <cstdint>

#include "shmem.hpp"

static void *shmem = NULL;
static volatile uint8_t *read_doorbell = NULL;
static volatile uint8_t *write_doorbell = NULL;

static int create_or_open_shmem_file()
{
    int fd = open(SHMEM_FILE, O_RDWR | O_CREAT, 0666);
    if (fd < 0)
    {
        perror("Failed to open or create shared memory file");
        return -1;
    }

    struct stat st;
    if (fstat(fd, &st) < 0)
    {
        perror("Failed to get file status");
        close(fd);
        return -1;
    }

    if (st.st_size != SHMEM_SIZE)
    {
        if (ftruncate(fd, SHMEM_SIZE) < 0)
        {
            perror("Failed to set file size");
            close(fd);
            return -1;
        }
        printf("Created shared memory file with size %d bytes\n", SHMEM_SIZE);
    }
    else
    {
        printf("Opened existing shared memory file with correct size\n");
    }

    return fd;
}

void *init_shared_memory()
{
    int fd = create_or_open_shmem_file();
    if (fd < 0)
    {
        return nullptr;
    }

    shmem = mmap(NULL, SHMEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shmem == MAP_FAILED)
    {
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

void shmem_arm_write_doorbell()
{
    __atomic_store_n(write_doorbell, 0, __ATOMIC_RELEASE);
}

void shmem_wait_write_doorbell()
{
    while (__atomic_load_n(write_doorbell, __ATOMIC_ACQUIRE) == 0)
    {
    }
}

void shmem_read_header(struct mmio_message_header *header)
{
    memcpy(header, reinterpret_cast<char *>(shmem) + MMIO_REGION_OFFSET,
           sizeof(struct mmio_message_header));
}

void shmem_complete_read(uint64_t value)
{
    *(uint64_t *)((char *)shmem + 16) = value;
    __atomic_store_n(read_doorbell, 1, __ATOMIC_RELEASE);
}
