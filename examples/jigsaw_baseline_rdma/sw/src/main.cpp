// Includes
#include <chrono>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <getopt.h>
#include <boost/program_options.hpp>

#include <coyote/cThread.hpp>
#include "message.hpp"
#include "rdma_server.hpp"

// Constants
#define CLOCK_PERIOD_NS 4
#define DEFAULT_VFPGA_ID 0

#define N_LATENCY_REPS 1
#define N_THROUGHPUT_REPS 32

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

static struct disagg_regions_rdma *regions; 

void print_help(char **argv)
{
    printf("usage: %s\n", argv[0]);
    printf("\t--localAddress [IP address of local interface]           or -a\n");
    printf("\t--localPort [Local port to use]                          or -p\n");
}

void setup(int argc, char **argv)
{
    /** Setup RDMA **/
	int op, ret;
	char *localAddr = NULL;
	char *localPort = NULL;

	/*** Read command line arguments ***/
	struct option long_opts[] = {
		{ "localAddress", 1, NULL, 'a' },
		{ "localPort", 1, NULL, 'p' },
		{ NULL, 0, NULL, 0 }
	};

	while ((op = getopt_long(argc, argv, "a:p:", long_opts, NULL)) != -1) {
		switch (op) {
			case 'a':
				localAddr = optarg;
				break;
			case 'p':
				localPort = optarg;
				break;
			default:
                print_help(argv);
				exit(EXIT_FAILURE);
		}
	}

	if (!localAddr || !localPort) {
		printf("Both IP address and port have to be specified\n");
        print_help(argv);
		exit(EXIT_FAILURE);
	}

	regions = init_rdma(localAddr, localPort);
	if (regions == NULL) {
        fprintf(stderr, "init_rdma failed\n");
		exit(EXIT_FAILURE);
    }
}

void device_d2h(coyote::cThread &coyote_thread, void *dst, size_t size)
{
    coyote_thread.setCSR(reinterpret_cast<uint64_t>(dst), static_cast<uint32_t>(JigsawRegisters::DMA_DST_ADDR_REG));
    coyote_thread.setCSR(static_cast<uint64_t>(size), static_cast<uint32_t>(JigsawRegisters::DMA_LEN_REG));
    coyote_thread.setCSR(coyote_thread.getCtid(), static_cast<uint32_t>(JigsawRegisters::COYOTE_PID_REG));

    // Start DMA transfer
    coyote_thread.setCSR(static_cast<uint64_t>(3), static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG));

    while (coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG)) != 1) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(CLOCK_PERIOD_NS));
    }
}

void device_h2d(coyote::cThread &coyote_thread, void *src, size_t size)
{
    coyote_thread.setCSR(reinterpret_cast<uint64_t>(src), static_cast<uint32_t>(JigsawRegisters::DMA_SRC_ADDR_REG));
    coyote_thread.setCSR(static_cast<uint64_t>(size), static_cast<uint32_t>(JigsawRegisters::DMA_LEN_REG));
    coyote_thread.setCSR(coyote_thread.getCtid(), static_cast<uint32_t>(JigsawRegisters::COYOTE_PID_REG));

    // Start DMA transfer
    coyote_thread.setCSR(static_cast<uint64_t>(1), static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG));

    while (coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG)) != 1) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(CLOCK_PERIOD_NS));
    }
}

int main(int argc, char *argv[])
{
    struct msg *recv_msg = NULL;
    struct msg *send_msg = NULL;

    setup(argc, argv);

    // Create Coyote thread and allocate memory for the transfer
    coyote::cThread coyote_thread(DEFAULT_VFPGA_ID, getpid());

    coyote_thread.userMap(reinterpret_cast<char *>(regions->dma_buf), DMA_SIZE);

    send_msg = static_cast<msg *>(regions->send_buf);

    while(1) {
        recv_msg = static_cast<msg *>(rdma_recv());

        if (recv_msg == NULL)
            return EXIT_FAILURE;

        if (recv_msg->type != 0) {
            fprintf(stderr, "Received wrong message type\n");
            return EXIT_FAILURE;
        }

        if (recv_msg->direction == 0) {
            device_d2h(coyote_thread, regions->dma_buf, recv_msg->size);
            if (rdma_write(regions->raddr, recv_msg->size) < 0) {
                fprintf(stderr, "rdam_write failed\n");
                return EXIT_FAILURE;
            }
        } else {
            if (rdma_read(regions->raddr, recv_msg->size) < 0) {
                fprintf(stderr, "rdam_read failed\n");
                return EXIT_FAILURE;
            }
            device_h2d(coyote_thread, regions->dma_buf, recv_msg->size);
        }

        send_msg->type = 1;
        if (rdma_send(send_msg, sizeof(*send_msg)) != 0) {
            fprintf(stderr, "rdma_send failed\n");
            return EXIT_FAILURE;
        }

    }

    return EXIT_SUCCESS;
}

