#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <malloc.h>
#include <time.h>
#include <sys/time.h>
#include <chrono>
#include <cstring>

#include "cProcess.hpp"

using namespace fpga;

constexpr auto const ddefSize = 64 * 64 * 8;
constexpr auto const odefSize = 64 * 64 * 8 * 2;

int main(int argc, char *argv[])
{
    uint32_t d_data_size = ddefSize;
    uint32_t o_data_size = odefSize;

    uint64_t *dMem, *oMem, *fMem;
    uint64_t n_input_pages, n_output_pages;

    n_input_pages = d_data_size / hugePageSize + ((d_data_size % hugePageSize > 0) ? 1 : 0);
    n_output_pages = o_data_size / hugePageSize + ((o_data_size % hugePageSize > 0) ? 1 : 0);

    cProcess cproc(0, getpid());

    uint64_t fpga_data = 345679821;

    // dMem = (uint64_t *)cproc.getMem({CoyoteAlloc::HOST_2M, (uint32_t)n_input_pages});
    // oMem = (uint64_t *)cproc.getMem({CoyoteAlloc::HUGE_2M, (uint32_t)n_output_pages});
    fMem = (uint64_t *)cproc.getMem({CoyoteAlloc::HUGE_2M, (uint32_t)n_output_pages});

    // memcpy(dMem, &fpga_data, 8);
    // memcpy(oMem, &fpga_data, 8);
    memcpy(fMem, &fpga_data, 8);

    // std::cout << "Host memory data mapped at: " << dMem << std::endl;
    // std::cout << "Host memory data mapped at: " << oMem << std::endl;
    std::cout << "FPGA memory data mapped at: " << fMem << std::endl;

    // std::cout << "dMem before:" << *((uint64_t *)dMem) << std::endl;
    // std::cout << "oMem before:" << *((uint64_t *)oMem) << std::endl;
    std::cout << "fMem before:" << *((uint64_t *)fMem) << std::endl;

    /* Using host memory */
    // cproc.ioSwDbg();
    // cproc.ioSwitch(IODevs::HOST_MEM);
    // cproc.ioSwDbg();
    // cproc.invoke({CoyoteOper::TRANSFER, (void *)fMem, (void *)fMem, 64 * 64 * 8 * 2, 64 * 64 * 8});
    // // std::cout << "In host mem: dMem after:" << *((uint64_t *)dMem) << std::endl;
    // std::cout << "In host mem: fMem after:" << *((uint64_t *)fMem) << std::endl;

    /*****************************************************************************/

    /* Using FPGA DRAM */
    cproc.ioSwDbg();
    cproc.ioSwitch(IODevs::FPGA_DRAM);
    cproc.ioSwDbg();

    /* First config: offload and sync to same memory */
    /* Works and only fMem is updated immediately*/
    std::cout << "Offload and sync to same region" << std::endl;
    cproc.invoke({CoyoteOper::OFFLOAD, (void *)fMem, 64 * 64 * 8 * 2, true, true, 0, false});
    std::cout << "In FPGA mem: fMem after OFFLOAD:" << *((uint64_t *)fMem) << std::endl;
    cproc.invoke({CoyoteOper::READ, (void *)fMem, 64 * 64 * 8 * 2, true, true, 0, false});
    std::cout << "In FPGA mem: fMem after READ:" << *((uint64_t *)fMem) << std::endl;
    // For matmul change to 64*64*8
    cproc.invoke({CoyoteOper::WRITE, (void *)fMem, 64 * 64 * 8 * 2, true, true, 0, false});
    std::cout << "In FPGA mem: fMem after WRITE:" << *((uint64_t *)fMem) << std::endl;
    // For matmul change to 64*64*8
    cproc.invoke({CoyoteOper::SYNC, (void *)fMem, 64 * 64 * 8 * 2, true, true, 0, false});
    std::cout << "In FPGA mem: fMem after SYNC:" << *((uint64_t *)fMem) << std::endl;
    // std::cout << "In FPGA mem: oMem after:" << *((uint64_t *)oMem) << std::endl;
    std::cout << "In FPGA mem: fMem after:" << *((uint64_t *)fMem) << std::endl;

    /* Second config: offload and sync to different regions */
    /* TODO: Works but oMem seems to be updated in the next run */
    // std::cout << "Offload and sync to different regions" << std::endl;
    // cproc.invoke({CoyoteOper::OFFLOAD, (void *)fMem, o_data_size, true, true, 0, false});
    // std::cout << "In FPGA mem: fMem after OFFLOAD:" << *((uint64_t *)fMem) << std::endl;
    // cproc.invoke({CoyoteOper::READ, (void *)fMem, o_data_size, true, false, 0, false});
    // std::cout << "In FPGA mem: fMem after READ:" << *((uint64_t *)fMem) << std::endl;
    // cproc.invoke({CoyoteOper::WRITE, (void *)fMem, o_data_size, true, true, 0, false});
    // std::cout << "In FPGA mem: fMem after WRITE:" << *((uint64_t *)fMem) << std::endl;
    // cproc.invoke({CoyoteOper::SYNC, (void *)fMem, (void *)oMem, o_data_size, o_data_size, true, true, 0, false});
    // std::cout << "In FPGA mem: fMem after SYNC:" << *((uint64_t *)fMem) << std::endl;

    // std::cout << "In FPGA mem: oMem after:" << *((uint64_t *)oMem) << std::endl;
    // std::cout << "In FPGA mem: fMem after:" << *((uint64_t *)fMem) << std::endl;
    /**********************************************************************************/

    cproc.printDebug();

    // cproc.ioSwDbg();
    // cproc.ioSwitch(IODevs::HOST_MEM);
    // cproc.ioSwDbg();
    // cproc.invoke({CoyoteOper::TRANSFER, (void *)dMem, o_data_size});
    // std::cout << "In host mem: dMem after:" << *((uint64_t *)dMem) << std::endl;
}
