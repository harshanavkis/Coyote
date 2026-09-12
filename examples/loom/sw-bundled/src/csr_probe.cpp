#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <coyote/cThread.hpp>
#include "loom.hpp"
static uint64_t R(coyote::cThread &t, int w) { return loom::csr_read(t, 8 * w); }
static void dbg(coyote::cThread &t, const char *tag) {
    printf("%-14s dbg:", tag);
    for (int i = 0; i < 10; i++) printf(" %lu", (unsigned long) R(t, 32 + i));
    printf("\n");
}
int main(int argc, char **argv) {
    coyote::cThread t(0, getpid(), argc > 1 ? atoi(argv[1]) : 0);
    loom::csr_write(t, loom::TX_PACE, 8);
    loom::csr_write(t, loom::TBL_LEN, 0x1234);
    dbg(t, "before");
    loom::csr_write(t, loom::TBL_IDX, 2);
    dbg(t, "after IDX=2");
    // read word 6 repeatedly, interleaved with reads of other addresses
    printf("w6: %lx", (unsigned long) R(t, 6));
    printf("  w6: %lx", (unsigned long) R(t, 6));
    (void) R(t, 15); printf("  [r15] w6: %lx", (unsigned long) R(t, 6));
    (void) R(t, 9);  printf("  [r9] w6: %lx", (unsigned long) R(t, 6));
    (void) R(t, 6);  printf("  [r6] w6: %lx\n", (unsigned long) R(t, 6));
    printf("w4: %lx", (unsigned long) R(t, 4));
    (void) R(t, 15); printf("  [r15] w4: %lx\n", (unsigned long) R(t, 4));
    // is it stable over time?
    usleep(100000);
    printf("100ms later  w4: %lx  w6: %lx\n", (unsigned long) R(t, 4), (unsigned long) R(t, 6));
    loom::csr_write(t, loom::TX_PACE, 0);
    return 0;
}
