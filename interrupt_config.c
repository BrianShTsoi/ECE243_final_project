#include "address_map_arm.h"

// Function Declarations
void set_A9_IRQ_stack();
void config_GIC();
void config_PS2();
void enable_A9_interrupts();

/* Set up/initialize all contollers and conditions for interrupts */
void interrupt_set_up() {
    set_A9_IRQ_stack(); // initialize the stack pointer for IRQ mode
    config_GIC(); // configure the general interrupt controller
    config_PS2(); // configure PS/2 to generate interrupts for mouse
    enable_A9_interrupts(); // enable interrupts
}

/* Initialize stack pointer register for IRQ mode */
void set_A9_IRQ_stack(void) {
    int stack, mode;
    stack = A9_ONCHIP_END - 7; // top of A9 onchip memory, aligned to 8 bytes
    // change processor to IRQ mode with interrupts disabled
    mode = INT_DISABLE | IRQ_MODE;
    asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
    // set banked stack pointer 
    asm("mov sp, %[ps]" : : [ps] "r"(stack));
    // go back to SVC mode before executing subroutine return! 
    mode = INT_DISABLE | SVC_MODE;
    asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
}

/* Configure the Generic Interrupt Controller (GIC) */
void config_GIC(void) {
    int address; // used to calculate register addresses
    
    // configure PS/2 interrupt
    *((int *)0xFFFED108) = 0x00008000;
    // potentially set bits of ICDICFRn for PS/2 to 0'b10 for edge-triggered behaviour

    // set Interrupt Priority Mask Register (ICCPMR), enable interrupts of all
    // priorities
    address = MPCORE_GIC_CPUIF + ICCPMR;
    *((int *)address) = 0xFFFF;
    // set CPU Interface Control Register (ICCICR), enable signaling of
    // interrupts
    address = MPCORE_GIC_CPUIF + ICCICR;
    *((int *)address) = ENABLE;
    // configure the Distributor Control Register (ICDDCR) to send pending
    // interrupts to CPUs
    address = MPCORE_GIC_DIST + ICDDCR;
    *((int *)address) = ENABLE;
}

/* Setup the PS/2 interrupts in the FPGA */
void config_PS2() {
    volatile int * PS2_ptr = (int *)PS2_BASE;
    *(HPS_GPIO1_ptr + 0x1) = 0x1;
}

/* Turn on interrupts in the ARM processor */
void enable_A9_interrupts(void) {
    int status = SVC_MODE | INT_ENABLE;
    asm("msr cpsr, %[ps]" : : [ps] "r"(status));
}
