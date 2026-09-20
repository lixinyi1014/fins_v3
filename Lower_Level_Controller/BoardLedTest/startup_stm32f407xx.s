        .syntax unified
        .cpu cortex-m4
        .thumb
        .eabi_attribute 24, 1

        .section .isr_vector, "a", %progbits
        .align 2
        .global g_pfnVectors
g_pfnVectors:
        .word   0x20020000
        .word   Reset_Handler
        .word   Default_Handler
        .word   Default_Handler
        .word   Default_Handler
        .word   Default_Handler
        .word   Default_Handler
        .word   Default_Handler
        .word   Default_Handler
        .word   Default_Handler
        .word   Default_Handler
        .word   Default_Handler
        .word   Default_Handler
        .word   Default_Handler
        .word   Default_Handler
        .word   Default_Handler
        /* External IRQs are not enabled by this test image. */
        .rept   96
        .word   Default_Handler
        .endr

        .section .text.Reset_Handler, "ax", %progbits
        .align 2
        .global Reset_Handler
        .type Reset_Handler, %function
Reset_Handler:
        ldr     sp, =0x20020000
        bl      main
1:      b       1b
        .size Reset_Handler, .-Reset_Handler

        .section .text.Default_Handler, "ax", %progbits
        .align 2
        .global Default_Handler
        .type Default_Handler, %function
Default_Handler:
2:      b       2b
        .size Default_Handler, .-Default_Handler
