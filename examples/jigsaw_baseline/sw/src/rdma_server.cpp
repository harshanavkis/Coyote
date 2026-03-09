// The initial structure of this file was taken from: https://github.com/linux-rdma/rdma-core/blob/master/librdmacm/examples/rdma_server.c
// But it was modified to work for this project
/*
 * Copyright (c) 2005-2009 Intel Corporation.  All rights reserved.
 *
 * This software is available to you under the OpenIB.org BSD license
 * below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AWV
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <netdb.h>
#include <rdma/rdma_cma.h>
#include <rdma/rdma_verbs.h>

#include "rdma_server.hpp"

static struct rdma_cm_id *listen_id, *id;
static int send_flags;

// Contains the buffers and registered memory regions and rkey of remote
struct disagg_regions_rdma regions_rdma;

int rdma_write(uint64_t raddr, size_t count)
{
#ifdef CONFIG_DISAGG_DEBUG_DMA_SEC
	printf("rdma_write: count: %lu, address to read from %lx\n", count, raddr);
#endif
	int ret;
	struct ibv_wc wc;

	ret = rdma_post_write(id, NULL, regions_rdma.dma_buf, count, regions_rdma.mr_dma_buf, 0, raddr, regions_rdma.rkey);

	if (ret != 0) {
		perror("rdma_post_write failed\n");
		goto out;
	}

	while ((ret = rdma_get_send_comp(id, &wc)) == 0);
	if (ret < 0)
		perror("rdma_get_send_comp");

out:
	return ret;
}

int rdma_read(uint64_t raddr, size_t count)
{
#ifdef CONFIG_DISAGG_DEBUG_DMA_SEC
	printf("rdma_read: count: %lu, address to read from %lx\n", count, raddr);
#endif
	int ret;
	struct ibv_wc wc;

	ret = rdma_post_read(id, NULL, regions_rdma.dma_buf, count, regions_rdma.mr_dma_buf, 0, raddr, regions_rdma.rkey);

	if (ret != 0) {
		perror("rdma_post_read failed\n");
		goto out;
	}

	while ((ret = rdma_get_send_comp(id, &wc)) == 0);
	if (ret < 0)
		perror("rdma_get_send_comp");

out:
	return ret;
}

int rdma_send(void *buf, size_t size)
{
	int ret; 
	struct ibv_wc wc;

	ret = rdma_post_send(id, NULL, buf, size, regions_rdma.mr_send_buf, send_flags);
	if (ret) {
		perror("rdma_post_send");
		goto out;
	}

	while ((ret = rdma_get_send_comp(id, &wc)) == 0);
	if (ret < 0)
		perror("rdma_get_send_comp");
	else
		ret = 0;

out:
	return ret;
}

// Retrieves a completed recv work request and returns the corresponding buffer
// The buffer's buddy is posted for recv
// Return NULL for failure
void *rdma_recv(void)
{
	struct ibv_wc wc;
	uint64_t i;
	int ret;
	uint64_t buddy;

	while ((ret = rdma_get_recv_comp(id, &wc)) == 0);
	if (ret < 0) {
		perror("rdma_get_recv_comp");
		return NULL;
	}

	i = wc.wr_id;

	if (i >= NUM_RECV_BUFS) {
		printf("invalid recv buffer id\n");
		return NULL;
	}

	// Get buddy index
	if (i < NUM_RECV_BUFS / 2) {
		buddy = i + (NUM_RECV_BUFS / 2);
	} else {
		buddy = i - (NUM_RECV_BUFS / 2);
	}

    // Post buddy for recv
	ret = rdma_post_recv(id, (void *) buddy, regions_rdma.recv_bufs[buddy].buf, BUFS_SIZE, regions_rdma.recv_bufs[buddy].mr);
	if (ret) {
		perror("rdma_post_recv");
		return NULL;
	}

	return regions_rdma.recv_bufs[i].buf;
}

static void deregister_mregions(void)
{
	rdma_dereg_mr(regions_rdma.mr_dma_buf);
	free(regions_rdma.dma_buf);

	for (int i = 0; i < NUM_RECV_BUFS; ++i) {
		rdma_dereg_mr(regions_rdma.recv_bufs[i].mr);
		free(regions_rdma.recv_bufs[i].buf);
	}

	if ((send_flags & IBV_SEND_INLINE) == 0)
		rdma_dereg_mr(regions_rdma.mr_send_buf);
	free(regions_rdma.send_buf);
}

// Allocates regions and
// Registers regions in struct regions_dma with the RDMA side
static int register_mregions(void)
{
	struct ibv_mr *res_mr;
	void *res_buf;
	int i;

	// First, the enc_buf, input for rdma_post_send
	res_buf = malloc(BUFS_SIZE); 
	if (!res_buf) {
		printf("Error malloc\n");
		goto out;
	}
	regions_rdma.send_buf = res_buf;
	if ((send_flags & IBV_SEND_INLINE) == 0) {
		res_mr = rdma_reg_msgs(id, res_buf, BUFS_SIZE);
		if (!res_mr) {
			perror("rdma_reg_msgs");
			goto free_buf1;
		}
		regions_rdma.mr_send_buf = res_mr;
	} else {
		regions_rdma.mr_send_buf = NULL;
	}

	// Now, the recv buffers, input for rdma_post_recv
    i = 0;
	for (; i < NUM_RECV_BUFS; ++i) {
		res_buf = malloc(BUFS_SIZE);
		if (!res_buf) {
			printf("Error malloc\n");
			goto free_dereg;
		}
		regions_rdma.recv_bufs[i].buf = res_buf;

		res_mr = rdma_reg_msgs(id, res_buf, BUFS_SIZE);
		if (!res_mr) {
			perror("rdma_reg_msgs");
			free(res_buf);
			goto free_dereg;
		}
		regions_rdma.recv_bufs[i].mr = res_mr;
	}

	// The dma buffer
	res_buf = malloc(DMA_SIZE);
	if (!res_buf) {
		printf("Error malloc\n");
		goto free_dereg;
	}
	regions_rdma.dma_buf = res_buf;
	res_mr = rdma_reg_msgs(id, res_buf, DMA_SIZE);

	if (!res_mr) {
		perror("rdma_reg_msgs for remote DMA");
		goto free_shmem;
	}
	regions_rdma.mr_dma_buf = res_mr;

	return 0;

free_shmem:
	free(regions_rdma.dma_buf);
free_dereg:
	--i;
	for (; i >= 0; --i) {
		rdma_dereg_mr(regions_rdma.recv_bufs[i].mr);
		free(regions_rdma.recv_bufs[i].buf);
	}

	if ((send_flags & IBV_SEND_INLINE) == 0)
		rdma_dereg_mr(regions_rdma.mr_send_buf);
free_buf1:
	free(regions_rdma.send_buf);
out:
	return 1;
}

struct disagg_regions_rdma *
init_rdma(const char *serverIP, const char *port)
{
	struct rdma_addrinfo hints, *res;
	struct ibv_qp_init_attr init_attr;
	struct ibv_qp_attr qp_attr;
	int ret;
	void *recv_msg;

	printf("rdma_server: start\n");

	// Uses serverIP and port to get address informations
	memset(&hints, 0, sizeof hints);
	hints.ai_flags = RAI_PASSIVE;
	hints.ai_port_space = RDMA_PS_TCP;
	ret = rdma_getaddrinfo(serverIP, port, &hints, &res);
	if (ret) {
		printf("rdma_getaddrinfo: %s\n", gai_strerror(ret));
		return NULL;
	}

	// Configures attributes
	memset(&init_attr, 0, sizeof init_attr);
	init_attr.cap.max_send_wr = init_attr.cap.max_recv_wr = NUM_RECV_BUFS / 2;
	init_attr.cap.max_send_sge = init_attr.cap.max_recv_sge = 1;
	init_attr.cap.max_inline_data = BUFS_SIZE;
	init_attr.sq_sig_all = 1;
	init_attr.qp_type = IBV_QPT_RC;
	ret = rdma_create_ep(&listen_id, res, NULL, &init_attr);
	if (ret) {
		perror("rdma_create_ep");
		goto out_free_addrinfo;
	}

	// Listens for connection requests
	ret = rdma_listen(listen_id, 0);
	if (ret) {
		perror("rdma_listen");
		goto out_destroy_listen_ep;
	}

	// Retreives connection request
	ret = rdma_get_request(listen_id, &id);
	if (ret) {
		perror("rdma_get_request");
		goto out_destroy_listen_ep;
	}

	// Checks attribute values of connection
	memset(&qp_attr, 0, sizeof qp_attr);
	memset(&init_attr, 0, sizeof init_attr);
	ret = ibv_query_qp(id->qp, &qp_attr, IBV_QP_CAP,
			&init_attr);
	if (ret) {
		perror("ibv_query_qp");
		goto out_destroy_accept_ep;
	}
	if (init_attr.cap.max_inline_data >= BUFS_SIZE)
		send_flags = IBV_SEND_INLINE;
	else
		printf("rdma_server: device doesn't support IBV_SEND_INLINE, "
				"using sge sends\n");

	// Registers the send/recv memory regions with RDMA
	ret = register_mregions();
	if (ret) {
		ret = -1;
		printf("server error end\n");
		goto out_destroy_accept_ep;
	}

	// Post first half of receive buffers
	for (uint64_t i = 0; i < NUM_RECV_BUFS / 2; ++i) {
		ret = rdma_post_recv(id, (void *) i, regions_rdma.recv_bufs[i].buf, BUFS_SIZE, regions_rdma.recv_bufs[i].mr);
		if (ret) {
			perror("rdma_post_recv");
			goto out_dereg;
		}
	}

	// Accept the connection request
	ret = rdma_accept(id, NULL);
	if (ret) {
		perror("rdma_accept");
		goto out_dereg;
	}

	// Retreive first completed recv request (the meta information about the remote DMA area)
	recv_msg = rdma_recv();
	if (!recv_msg) {
		goto out_disconnect;
	}

    // First message is the remote_address
    regions_rdma.raddr = *(uint64_t *)recv_msg;
    
	recv_msg = rdma_recv();
	if (!recv_msg) {
		goto out_disconnect;
	}

	// Second message is the meta information (i.e. rkey) about the shmem DMA region
	regions_rdma.rkey = *(uint32_t *)recv_msg;

	printf("raddr: 0x%x, rkey: 0x%x\n", regions_rdma.raddr, regions_rdma.rkey);

	return &regions_rdma;

out_disconnect:
	rdma_disconnect(id);
out_dereg:
	deregister_mregions();
out_destroy_accept_ep:
	rdma_destroy_ep(id);
out_destroy_listen_ep:
	rdma_destroy_ep(listen_id);
out_free_addrinfo:
	rdma_freeaddrinfo(res);
	return NULL;
}

