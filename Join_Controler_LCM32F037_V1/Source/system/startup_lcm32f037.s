/**
 * @file startup_lcm32f037.s
 * @brief Startup file for LCM32F037 Cortex-M0 MCU
 *        Compatible with ARM Compiler 6 (armclang)
 */

    .syntax unified
    .arch armv6-m
    .eabi_attribute 24, 1
    .eabi_attribute 25, 1

    .section .stack, "aw", %nobits
    .align 3
    .globl __StackTop
    .globl __StackLimit
__StackLimit:
    .space 0x00000400   /* 1 KB Stack */
__StackTop:

    .section .heap, "aw", %nobits
    .align 3
    .globl __HeapBase
    .globl __HeapLimit
__HeapBase:
    .space 0x00000100   /* 256 B Heap (library only) */
__HeapLimit:

    .section RESET, "a", %progbits
    .align 3
    .globl __Vectors
    .globl __Vectors_End
    .globl __Vectors_Size
__Vectors:
    .word __StackTop                /* Top of Stack */
    .word Reset_Handler             /* Reset Handler */
    .word NMI_Handler               /* NMI Handler */
    .word HardFault_Handler         /* Hard Fault Handler */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word SVC_Handler               /* SVCall Handler */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word PendSV_Handler            /* PendSV Handler */
    .word SysTick_Handler           /* SysTick Handler */

    /* External Interrupts (LCM32F037 Specific) */
    .word Default_Handler           /*  0: WWDG */
    .word Default_Handler           /*  1: LVD */
    .word Default_Handler           /*  2: WT */
    .word Default_Handler           /*  3: EFLS */
    .word Default_Handler           /*  4: CHIPCTRL */
    .word Default_Handler           /*  5: EXTI 0..1 */
    .word Default_Handler           /*  6: EXTI 2..3 */
    .word Default_Handler           /*  7: EXTI 4..15 */
    .word Default_Handler           /*  8: LCD/ACMP 0..2 */
    .word Default_Handler           /*  9: DMAC CH0 */
    .word DMAC_CH1_2_Handler        /* 10: DMAC CH1..2 (UART DMA TX Complete) */
    .word Default_Handler           /* 11: DMAC CH3..5 */
    .word Default_Handler           /* 12: ADC */
    .word Default_Handler           /* 13: TIM1 Non-CC */
    .word TIM1_CC_Handler           /* 14: TIM1 CC (40kHz FOC loop) */
    .word Default_Handler           /* 15: TIM2 / ACMP */
    .word Default_Handler           /* 16: TIM3 */
    .word TIM2_Handler              /* 17: TIM2 return-delay timer (original IRQ17 -> 0x08003341) */
    .word Default_Handler           /* 18: TIM7 */
    .word Default_Handler           /* 19: TIM14 */
    .word Default_Handler           /* 20: TIM15 */
    .word Default_Handler           /* 21: TIM16 */
    .word Default_Handler           /* 22: TIM17 */
    .word Default_Handler           /* 23: I2C0 */
    .word Default_Handler           /* 24: Reserved */
    .word Default_Handler           /* 25: SPI0 */
    .word Default_Handler           /* 26: SPI2 */
    .word Default_Handler           /* 27: UART0 */
    .word UART1_Handler             /* 28: UART1 */
    .word Default_Handler           /* 29: Reserved */
    .word Default_Handler           /* 30: Reserved */
    .word Default_Handler           /* 31: DIV */
__Vectors_End:
    .equ __Vectors_Size, __Vectors_End - __Vectors

    .text
    .thumb
    .align 2
    .globl Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    ldr   r0, =LM_SystemInit
    blx   r0
    ldr   r0, =__main
    bx    r0
    .size Reset_Handler, . - Reset_Handler

    /* Stack/heap layout for the C library (avoids semihosting bkpt #0xAB,
     * which locks up the core when no debugger is attached).
     * Returns: R0 = heap base, R1 = stack base (top), R2 = heap limit, R3 = stack limit */
    .globl __use_two_region_memory
__use_two_region_memory:
    .globl __user_initial_stackheap
    .type __user_initial_stackheap, %function
__user_initial_stackheap:
    ldr   r0, =__HeapBase
    ldr   r1, =__StackTop
    ldr   r2, =__HeapLimit
    ldr   r3, =__StackLimit
    bx    lr
    .ltorg
    .size __user_initial_stackheap, . - __user_initial_stackheap

    .globl Default_Handler
    .type Default_Handler, %function
Default_Handler:
    b     Default_Handler
    .size Default_Handler, . - Default_Handler

    .weak NMI_Handler
    .thumb_set NMI_Handler, Default_Handler

    .weak HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler

    .weak SVC_Handler
    .thumb_set SVC_Handler, Default_Handler

    .weak PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler

    .weak SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler

    .weak DMAC_CH1_2_Handler
    .thumb_set DMAC_CH1_2_Handler, Default_Handler

    .weak TIM1_CC_Handler
    .thumb_set TIM1_CC_Handler, Default_Handler

    .weak TIM2_Handler
    .thumb_set TIM2_Handler, Default_Handler

    .weak UART1_Handler
    .thumb_set UART1_Handler, Default_Handler

    .end
