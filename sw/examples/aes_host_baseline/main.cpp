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
#include <boost/program_options.hpp>

// #include "userUtils.hpp"
#include "cProcess.hpp"

using namespace fpga;

// Default size in bytes
constexpr auto const defSize = 64 * 1024;

// Read size
constexpr auto const rdSize = 4 * 1024;

// Deafult IO device
constexpr auto const ioDev = IODevs::HOST_MEM;

// AES data
constexpr auto const keyLow = 0xabf7158809cf4f3c;
constexpr auto const keyHigh = 0x2b7e151628aed2a6;
constexpr auto const plainLow = 0xe93d7e117393172a;
constexpr auto const plainHigh = 0x6bc1bee22e409f96;
constexpr auto const cipherLow = 0xa89ecaf32466ef97;
constexpr auto const cipherHigh = 0x3ad77bb40d7a3660;

int main(int argc, char *argv[])
{
    // Read arguments
    boost::program_options::options_description programDescription("Options:");
    programDescription.add_options()("size,s", boost::program_options::value<uint32_t>(), "Data size");
    programDescription.add_options()("iodev,d", boost::program_options::value<uint32_t>(), "IO Device to read data from");

    boost::program_options::variables_map commandLineArgs;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, programDescription), commandLineArgs);
    boost::program_options::notify(commandLineArgs);

    uint32_t size = defSize;
    IODevs io_dev = ioDev;
    uint64_t n_pages;
    if (commandLineArgs.count("size") > 0)
        size = commandLineArgs["size"].as<uint32_t>();

    cProcess cproc(0, getpid());

    if (commandLineArgs.count("iodev") > 0)
    {
        io_dev = cproc.userInIOSwtch(commandLineArgs["iodev"].as<uint32_t>());
        if (io_dev == IODevs::ERROR_DEV)
        {
            std::cout << "User entered an invalid IO configuration" << std::endl;
            return (EXIT_FAILURE);
        }
        std::cout << "User selected IO device: " << static_cast<uint32_t>(io_dev) << std::endl;
    }

    n_pages = size / hugePageSize + ((size % hugePageSize > 0) ? 1 : 0);

    // Allocate test data and result data
    void *tMem = (uint64_t *)cproc.getMem({CoyoteAlloc::HOST_2M, (uint32_t)n_pages});
    void *rMem = (uint64_t *)cproc.getMem({CoyoteAlloc::HOST_2M, (uint32_t)n_pages});

    for (int i = 0; i < size / 8; i++)
    {
        ((uint64_t *)tMem)[i] = i % 2 ? plainHigh : plainLow;
    }

    // IO device: use host memory
    cproc.ioSwitch(io_dev);
    cproc.setCSR(keyLow, 0);
    cproc.setCSR(keyHigh, 1);

    cproc.invoke({CoyoteOper::TRANSFER, (void *)tMem, size});

    // Check the results
    bool k = true;
    for (int i = 0; i < size / 8; i++)
    {
        if (i % 2 ? ((uint64_t *)tMem)[i] != cipherHigh : ((uint64_t *)tMem)[i] != cipherLow)
        {
            k = false;
            break;
        }
    }

    std::cout << (k ? "Success: cipher text matches test vectors!" : "Error: found cipher text that doesn't match the test vector") << std::endl;
}
