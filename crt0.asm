// Rowley C Compiler, runtime support.
//
// Copyright (c) 2001, 2002, 2003, 2008 Rowley Associates Limited.
//
// This file may be distributed under the terms of the License Agreement
// provided with this software.
//
// THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Preprocessor Definitions:
//
// __NOVECTORS - if defined .vectors section isn't allocated
//
// FULL_LIBRARY - if defined argc, argv are setup by debug_getargs, exit symbol is defined and executes
//                on return from main, exit calls atexit functions and then debug_exit
//
// ARGSSPACE - ram size in bytes to contain the debug_getargs

#ifndef ARGSSPACE
#define ARGSSPACE 128
#endif

#ifdef FULL_LIBRARY
        .public _exit
#endif

#include <msp430.h>

; Create sections
        .data
        .bss

; Go to code section.
        .psect  "ISR"
        .keep

; Executed upon reset
__reset proc

; Turn off watchdog.  You can enable it in main() if required.
        ;mov.w #WDTPW+WDTHOLD, &WDTCTL
; Kick Watchdog
        mov.w #WDTPW+WDTCNTCL+WDTSSEL, &WDTCTL

; Set up stack.
        mov.w   #___RAM_Address+___RAM_Size, sp

; Copy from initialised data section to data section.
        LINKIF  SIZEOF(IDATA0)
        mov.w   #SFB(IDATA0), r15
        mov.w   #data_init_begin, r14
        mov.w   #data_init_end-data_init_begin, r13
        callx   #_memcpy
        ENDLINKIF

; Kick Watchdog
        mov.w #WDTPW+WDTCNTCL+WDTSSEL, &WDTCTL

; Zero the bss.  Ensure the stack is not allocated in the bss!
        LINKIF  SIZEOF(UDATA0)
        mov.w   #SFB(UDATA0), r15
        mov.w   #0, r14
        mov.w   #SFE(UDATA0)-SFB(UDATA0), r13
        callx   #_memset
        ENDLINKIF

; Kick Watchdog
        mov.w #WDTPW+WDTCNTCL+WDTSSEL, &WDTCTL

#ifdef FULL_LIBRARY
        mov.w   #ARGSSPACE, r15
        mov.w   #args, r14
        callx   #_debug_getargs
        mov.w   #args, r14
#else
        mov.w   #0, r15
        mov.w   #0, r14
#endif
; Call user entry point void main(void).
        callx   #_main
#ifdef FULL_LIBRARY
        endproc
_exit   proc
        mov     r15, r5
        callx   #__execute_at_exit_fns
        mov     r5, r15
        callx   #_debug_exit
#endif
; If main() returns, kick off again.
        jmp     __reset
        endproc

; Heap data structures; removed by the linker if the heap isn't used.
        .break   
        .data
        align   WORD
___heap_start__::
        DW      0
        DW      heap_size
        DS      heap_size-4    

#ifndef __NOVECTORS
; Reset vector
        .vectors
        .keep
        org     RESET_VECTOR
        dw      __reset
#endif

; Initialise the IDATA0 section by duplicating the contents into the
; CONST section and copying them on startup.
        .const
data_init_begin:
        .init  "IDATA0"
data_init_end:

#ifdef FULL_LIBRARY
        .bss
args:
        .space ARGSSPACE
#endif

; Define symbols for runtime support routines.  The 16-bit and 32-bit
; multipliers do not have the same addresses over all MSP430 variants,
; so define these as symbols to be fixed up at link time.

#ifdef MPY_
        .public  ___crt_mpy
        .public  ___crt_mpys
        .public  ___crt_mac
        .public  ___crt_macs
        .public  ___crt_op2
        .public  ___crt_reslo
        .public  ___crt_reshi
        .public  ___crt_sumext
___crt_mpy    equ MPY_
___crt_mpys   equ MPYS_
___crt_mac    equ MAC_
___crt_macs   equ MACS_
___crt_op2    equ OP2_
___crt_reslo  equ RESLO_
___crt_reshi  equ RESHI_
___crt_sumext equ SUMEXT_
#endif

#ifdef MPY32L_
        .public  ___crt_mpy32l
        .public  ___crt_mpy32h
        .public  ___crt_mac32l
        .public  ___crt_mac32h
        .public  ___crt_mpys32l
        .public  ___crt_mpys32h
        .public  ___crt_op2l
        .public  ___crt_op2h
        .public  ___crt_res0
        .public  ___crt_res1
        .public  ___crt_res2
        .public  ___crt_res3
___crt_mpy32l  equ MPY32L_
___crt_mpy32h  equ MPY32H_
___crt_mac32l  equ MAC32L_
___crt_mac32h  equ MAC32H_
___crt_mpys32l equ MPYS32L_
___crt_mpys32h equ MPYS32H_
___crt_op2l    equ OP2L_
___crt_op2h    equ OP2H_
___crt_res0    equ RES0_
___crt_res1    equ RES1_
___crt_res2    equ RES2_
___crt_res3    equ RES3_
#endif
