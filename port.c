#include <stdbool.h>

#include "stm32f10x.h"
#include "cmsis_os.h"       /* Only for os_pthread definition. */

#define PORT_THREAD_STACK_MAGIC

#define PORT_CONTEXT_OFFSET_xPSR            ( 0xF )
#define PORT_CONTEXT_OFFSET_RA              ( 0xE )
#define PORT_CONTEXT_OFFSET_LR              ( 0xD )
#define PORT_CONTEXT_OFFSET_R12             ( 0xC )
#define PORT_CONTEXT_OFFSET_R3              ( 0xB )
#define PORT_CONTEXT_OFFSET_R2              ( 0xA )
#define PORT_CONTEXT_OFFSET_R1              ( 0x9 )
#define PORT_CONTEXT_OFFSET_R0              ( 0x8 )
#define PORT_CONTEXT_OFFSET_R11             ( 0x7 )
#define PORT_CONTEXT_OFFSET_R10             ( 0x6 )
#define PORT_CONTEXT_OFFSET_R9              ( 0x5 )
#define PORT_CONTEXT_OFFSET_R8              ( 0x4 )
#define PORT_CONTEXT_OFFSET_R7              ( 0x3 )
#define PORT_CONTEXT_OFFSET_R6              ( 0x2 )
#define PORT_CONTEXT_OFFSET_R5              ( 0x1 )
#define PORT_CONTEXT_OFFSET_R4              ( 0x0 )

/* Set bit is for Thumb execution mode. */
#define PORT_CONTEXT_INIVAL_xPSR            ( 0x01000000 )
#define PORT_CONTEXT_INIVAL_LR              ( 0xFFFFFFFF )



void *port_prepare_stack (const size_t stacksize, const os_pthread pthread, const void * const argument)
{
    void * const sp = malloc(stacksize);

    /* Reserve 4 byte aligned space on thread stack for full context. */
    uint32_t * const frameptr           = (uint32_t *)(((uintptr_t)sp - 0x40) & ~0x3);

    frameptr[PORT_CONTEXT_OFFSET_xPSR]  = (uint32_t)PORT_CONTEXT_INIVAL_xPSR;
    frameptr[PORT_CONTEXT_OFFSET_LR]    = (uint32_t)PORT_CONTEXT_INIVAL_LR;
    frameptr[PORT_CONTEXT_OFFSET_RA]    = (uint32_t)pthread;
    frameptr[PORT_CONTEXT_OFFSET_R0]    = (uint32_t)argument;

    return (void *)frameptr;
}

void port_start_kernel(register void *, register void *, register void *)   __attribute__ ( (naked) );
void port_start_kernel(register void *stack_pointer, register void *jump_addr, register void *arg)
{
    /* According to some manuals stack_pointer should be in r0, jump_addr r1 and arg in r2. */
    __asm volatile  (
                    "   msr     psp, r0                 \n"
                    "   mov     r0, #2                  \n"     /* second LSB in CONTROL defines stack that we will use */
                    "   msr     control, r0             \n"     /* switch stack pointer from reset's MainSP to ProcessSP */
                    "   isb                             \n"     /* synchronize changes */

                    "   ldr     r0, =0xE000ED08         \n"     /* reload msp to it after-reset value */
                    "   ldr     r0, [r0]                \n"
                    "   ldr     r0, [r0]                \n"
                    "   msr     msp, r0                 \n"

                    "   ldr     lr, =0xFFFFFFFF         \n"     /* reset LR */

                    "   mov     r0, #0                  \n"
                    "   msr     basepri, r0             \n"

                    "   mov     r0, r2                  \n"     /* arg to r0 */
                    "   cpsie   i                       \n"     /* enable interrupts */
                    "   b       r1                      \n"     /* jump to task code */
                    );
}

#define PORT_ISRNUM_THREAD      ( 0x0 )
bool port_isr_active (void)
{
    return ((bool)(__get_IPSR() != PORT_ISRNUM_THREAD));
}

void port_resched (void)
{
    /* Call the PendSV exception. */
    SCB->ICSR       |=  SCB_ICSR_PENDSVSET_Msk;
}

void context_switch (register uint32_t, register void *) __attribute__ ( (naked) );
void context_switch (register uint32_t running_tid, register void *base)
{
    __asm volatile  (
                    "   ldr     r1, =tcb_table_ptr      \n"     /* Must be eq. to TCB_ATTR(sp, running_tid) */
                    "   ldr     r1, [r1]                \n"

                    "   ldr     r0, =running_tid        \n"
                    "   ldr     r0, [r0]                \n"
                    
                    "   ldr     r2, [r1, r0, LSL 2]     \n"
                    "   stmdb   r2!, {r4-r11}           \n"
                    "   str     r2, [r1, r0, LSL 2]     \n"

                    "   push    {lr}                    \n"
                    "   bl      kernel_context_switch   \n"
                    "   pop     {lr}                    \n"

                    "   ldr     r2, [r1, r0, LSL 2]     \n"
                    "   ldmia   r2!, {r4-r11}           \n"
                    "   msr     psp, r2                 \n"
                    "   bx      lr                      \n"
                    );
}

void port_thread_exit (void)    __attribute__ ( (naked) );
void port_thread_exit (void)
{
    __asm volatile  (
                    "   ldr     r1, =tcb_table_ptr      \n"     /* Must be eq. to TCB_ATTR(sp, running_tid) */
                    "   ldr     r1, [r1]                \n"

                    "   ldr     r0, =running_tid        \n"
                    "   ldr     r0, [r0]                \n"
                    
                    "   ldr     r2, [r1, r0, LSL 2]     \n"
                    "   msr     psp, r2                 \n"
                    "   mov     r0, #2                  \n"
                    "   msr     control, r0             \n"
                    "   isb                             \n"

                    "   ldr     r0, [SP, #60]           \n"
                    "   msr     xPSR, r0                \n"

                    "   pop     {r4-r11}                \n"
                    "   pop     {r0-r3, r12, lr}        \n"
                    "   pop     {pc}                    \n"     /* PC and LR cannot be in popped together. */
                    );
}
