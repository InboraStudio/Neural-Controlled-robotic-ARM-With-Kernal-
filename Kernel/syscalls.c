/******************************************************************************
 * Minimal System Calls Stubs
 * Prevents linker errors when using GCC without standard library
 * These functions should never be called in bare-metal code
 *****************************************************************************/

#include <stdint.h>

/* Stub for _sbrk (heap allocation - not used) */
void* _sbrk(int incr) {
    extern uint8_t __heap_start;
    static uint8_t *heap_end = &__heap_start;
    uint8_t *prev_heap_end = heap_end;
    heap_end += incr;
    return prev_heap_end;
}

/* Stub for _exit (never called in embedded) */
void _exit(int status) {
    (void)status;
    while(1) { }  /* Infinite loop */
}

/* Stub for _kill */
int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    return -1;
}

/* Stub for _getpid */
int _getpid(void) {
    return 1;
}

/* Stub for _write (can implement for USART debugging) */
int _write(int file, char *ptr, int len) {
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

/* Stub for _close */
int _close(int file) {
    (void)file;
    return -1;
}

/* Stub for _fstat */
int _fstat(int file, void *st) {
    (void)file;
    (void)st;
    return -1;
}

/* Stub for _isatty */
int _isatty(int file) {
    (void)file;
    return 1;
}

/* Stub for _lseek */
int _lseek(int file, int ptr, int dir) {
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

/* Stub for _read */
int _read(int file, char *ptr, int len) {
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}
