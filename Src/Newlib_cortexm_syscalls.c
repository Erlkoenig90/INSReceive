/*
Copyright (c) 2015, Niklas Gürtler
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*
Including this file in a project compiled with GCC-ARM-Embedded and the newlib allows usage of malloc and thereby printf(). Adjust _exit, and _write_r to your needs and add the include to the microcontroller main header file.
*/

#include <stddef.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#undef errno

#include <reent.h>

#include <stm32f407xx.h>

// This macro can be used to add function attributes to all  functions in this file if neccessary.
#define SYSCALL_FUN_ATTR

// Use this definition if you encounter linker errors like: `_fstat_r' referenced in section ... defined in discarded section `.text' of ./syscalls.o (symbol from plugin) when compiling with LTO
//#define SYSCALL_FUN_ATTR __attribute__ ((used))


ssize_t _write_r(struct _reent * reent, int fd, const void *buf, size_t cnt) SYSCALL_FUN_ATTR;
void _exit(int status) SYSCALL_FUN_ATTR;
void *_sbrk_r(struct _reent * reent, ptrdiff_t incr) SYSCALL_FUN_ATTR;
int _close_r(struct _reent * reent, int fd) SYSCALL_FUN_ATTR;
int _fstat_r(struct _reent * reent, int fd, struct stat *pstat) SYSCALL_FUN_ATTR;
int _isatty_r(struct _reent * reent, int file) SYSCALL_FUN_ATTR;
off_t _lseek_r(struct _reent * reent, int fd, off_t pos, int whence) SYSCALL_FUN_ATTR;
ssize_t _read_r(struct _reent * reent, int fd, void *buf, size_t cnt) SYSCALL_FUN_ATTR;
int _kill(int pid, int sig) SYSCALL_FUN_ATTR;
int _getpid(void) SYSCALL_FUN_ATTR;


// Prints string to stdout
ssize_t _write_r(struct _reent * reent, int fd, const void *buf, size_t cnt) {
	if (fd != 1 && fd != 2) {
		((struct _reent*) reent)->_errno = EBADF;
		return -1;
	}
	
	// Prints the string to SWO output. May be change to e.g. UART output.

	if ((ITM->TCR & 1) == 0)
		return cnt;
	if ((ITM->TER & 1) == 0)
		return cnt;

	for (size_t i = 0; i < cnt; ++i) {
		while ((ITM->PORT[0].u8 & 1) == 0);
		ITM->PORT[0].u8 = ((uint8_t*) (buf))[i];
	}
	return cnt;
}

void _exit(int status) {
	((void) status);
	// Stop controller in case of fatal error
	__disable_irq ();
	while (1) {
		__asm__ volatile ("bkpt");
	}
	 __builtin_unreachable ();
}

// These should be defined in the linker script.
extern char _Min_Heap_Size;
extern char _end;
static char* heapEnd = &_end;

// Allocates more heap space
void *_sbrk_r(struct _reent * reent, ptrdiff_t incr) {
	char* prev = heapEnd;

	if (heapEnd + incr > &_end + ((size_t) &_Min_Heap_Size)) {
		((struct _reent*) reent)->_errno = ENOMEM;
		return NULL;
	}

	heapEnd += incr;

	return prev;
}


int _close_r(struct _reent * reent, int fd) {
	((void) reent);
	((void) fd);
	((struct _reent*) reent)->_errno = EBADF;
	return -1;
}

int _fstat_r(struct _reent * reent, int fd, struct stat *pstat) {
	((void) reent); ((void) fd);
	pstat->st_mode = S_IFCHR;
	return 0;
}

int _isatty_r(struct _reent * reent, int file) {
	((void) reent); ((void) file);
	return 1;
}

off_t _lseek_r(struct _reent * reent, int fd, off_t pos, int whence) {
	((void) reent); ((void) fd); ((void) pos); ((void) whence);
	return 0;
}

ssize_t _read_r(struct _reent * reent, int fd, void *buf, size_t cnt) {
	((void) reent); ((void) fd); ((void) buf); ((void) cnt);
	return 0;
}

int _kill(int pid, int sig) {
	((void) pid); ((void) sig);
	return -1;
}

int _getpid(void) {
	return 1;
}
