#ifndef RDMA_SERVER_HPP
#define RDMA_SERVER_HPP

#include <stdlib.h>
#include <infiniband/verbs.h>

//#define CONFIG_DISAGG_DEBUG_DMA_SEC
//#define CONFIG_DISAGG_DEBUG_MMIO_SEC

#define NUM_RECV_BUFS 8
#define BUFS_SIZE 92
#define DMA_SIZE (1024*64) 

struct disagg_regions_rdma *init_rdma(const char *serverIP, const char *port);

/* 
 * Send the @buf of @size to remote 
 * @buf has to be a region in struct disagg_regions_rdma.send_buf
 */
int rdma_send(void *buf, size_t size);

/*
 * Receives a message from remote
 * @returns pointer to received message
 */
void *rdma_recv(void);

// Reads @count bytes from the remote region at @raddr to the local dma_buf (field in struct disagg_regions_rdma)
int rdma_read(uint64_t raddr, size_t count);

// Writes @count bytes of the local dma_buf (field in struct disagg_regions_rdma) at @raddr to the remote region
int rdma_write(uint64_t raddr, size_t count);

struct disagg_regions_rdma {
    // source for rdma_post_send
	void *send_buf;
	struct ibv_mr *mr_send_buf;

    /*
     * Destinations for rdma_post_recv
     * At every time only one half of the buffers is actually posted as recv.
     * Every buffer in the first half has a "buddy" in the second half.
     * Only ever one buddy of a pair is posted as recv.
     * When a buffer is in a completed request his buddy is posted for recv.
     * The buffer itself is returned as a result.
     * This prevents overriding of memory regions if the result is not yet processed.
     */
	struct recv_bufs {
		void *buf;
		struct ibv_mr *mr;
	} recv_bufs[NUM_RECV_BUFS];

    /* 
     * Source/destination for rdma_write/rdma_read
     */
	void *dma_buf; 
	struct ibv_mr *mr_dma_buf;

	// Used when writing to remote DMA region
    uint64_t raddr;
	uint32_t rkey;
};

extern struct disagg_regions_rdma regions_rdma;

#endif // RDMA_SERVER_HPP
