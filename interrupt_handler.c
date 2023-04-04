#include "address_map_arm.h"

void PS2_ISR(void);

/* Define the IRQ exception handler */
void __atribute__((interrupt)) __cs3_isr_irq(void) {
    int address = MPCORE_GIC_CPUIF + ICCIAR;
    int int_ID = *((int *)address);

    if (int_ID == HPS_TIMER0_IRQ) // check if interrupt is from the HPS timer
        PS2_ISR();

    return;
}

void PS2_ISR() {
    // Read from Data in loop while Ravail > 0
    // Clear interrupt in RI
}