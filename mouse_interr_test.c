#include "address_map_arm.h"

/* VGA colors */
#define WHITE 0xFFFF
#define YELLOW 0xFFE0
#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define GREY 0xC618
#define PINK 0xFC18
#define ORANGE 0xFC00
#define BLACK 0x0000

/* Screen size */
#define RESOLUTION_X 320
#define RESOLUTION_Y 240

/* Graphics constants */
#define BOX_LEN 10

struct Box {
    int x; 
    int y;

    int dx;
    int dy;

    int prev_x;
    int prev_y;
    int prior_x;
    int prior_y;

    short int color;
};

/* Function Declarations */
void disable_A9_interrupts(void);
void set_A9_IRQ_stack(void);
void config_GIC(void);
void config_PS2(void);
void enable_A9_interrupts(void);
void PS2_ISR(void);
int twoCompToInt(char value, char sign, char overflow);
void HEX_PS2(char, char, char);
void config_interrupt(int, int);
void set_up_pixel_buf_ctrl();
void clear_screen();
void wait_for_vsync();
void plot_pixel(int x, int y, short int color);
void erase_box(struct Box box);
void draw_box(struct Box box);
void move_box(struct Box* box);
struct Box prior_box(struct Box box);
struct Box construct_box(int x, int y, int dx, int dy, short int color);

/* Byte packets for mouse */
char byte1 = 0, byte2 = 0, byte3 = 0;
int initialized = 0;

/* Display variables */
volatile int g_pixel_back_buffer; // global variable
volatile int * const G_PIXEL_BUF_CTRL_PTR = (int *) PIXEL_BUF_CTRL_BASE;

const int MAX_BOX_X = RESOLUTION_X - BOX_LEN;
const int MAX_BOX_Y =  RESOLUTION_Y - BOX_LEN;
struct Box cursor;
struct Box object;

int main(void) {
    disable_A9_interrupts(); // disable interrupts in the A9 processor
    set_A9_IRQ_stack(); // initialize the stack pointer for IRQ mode
    config_GIC(); // configure the general interrupt controller
    config_PS2(); // configure pushbutton KEYs to generate interrupts
    enable_A9_interrupts(); // enable interrupts in the A9 processor

    // Reset mouse
    volatile int * PS2_ptr = (int *)PS2_BASE;
    *(PS2_ptr) = 0xFF;

    cursor = construct_box(MAX_BOX_X/2, MAX_BOX_Y/2, 0, 0, WHITE);
    object = construct_box(MAX_BOX_X/4, MAX_BOX_Y/4, 0, 0, BLUE);

    // Set up display
    set_up_pixel_buf_ctrl();

    while (1) {
        erase_box(object);
        draw_box(object);
        move_box(&object);
        erase_box(cursor);
        draw_box(cursor);
        move_box(&cursor);

        wait_for_vsync(); // swap front and back buffers on VGA vertical sync
        g_pixel_back_buffer = *(G_PIXEL_BUF_CTRL_PTR + 1); // new back buffer
    }
}

/* setup the KEY interrupts in the FPGA */
void config_PS2() {
    volatile int * PS2_ptr = (int *)PS2_BASE;
    *(PS2_ptr + 0x1) = 0x1;
}

// Define the IRQ exception handler
void __attribute__((interrupt)) __cs3_isr_irq(void) {
    // Read the ICCIAR from the CPU Interface in the GIC
    int address = MPCORE_GIC_CPUIF + ICCIAR;
    int interrupt_ID = *((int *)address);

    if (interrupt_ID == 79) // check if interrupt is from PS2
        PS2_ISR();
    else
    while (1); // if unexpected, then stay here
    // Write to the End of Interrupt Register (ICCEOIR)
    *((int *)0xFFFEC110) = interrupt_ID;
}
// Define the remaining exception handlers
void __attribute__((interrupt)) __cs3_reset(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_undef(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_swi(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_pabort(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_dabort(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_fiq(void) {
    while (1);
}

/*
* Turn off interrupts in the ARM processor
*/
void disable_A9_interrupts(void) {
    int status = 0b11010011;
    asm("msr cpsr, %[ps]" : : [ps] "r"(status));
}

/*
* Initialize the banked stack pointer register for IRQ mode
*/
void set_A9_IRQ_stack(void) {
    int stack, mode;
    stack = A9_ONCHIP_END - 7; // top of A9 onchip memory, aligned to 8 bytes
    /* change processor to IRQ mode with interrupts disabled */
    mode = 0b11010010;
    asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
    /* set banked stack pointer */
    asm("mov sp, %[ps]" : : [ps] "r"(stack));
    /* go back to SVC mode before executing subroutine return! */
    mode = 0b11010011;
    asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
}

/*
* Turn on interrupts in the ARM processor
*/
void enable_A9_interrupts(void) {
    int status = 0b01010011;
    asm("msr cpsr, %[ps]" : : [ps] "r"(status));
}

/*
* Configure the Generic Interrupt Controller (GIC)
*/
void config_GIC(void) {
    config_interrupt (79, 1); // configure the FPGA KEYs interrupt (73)
    // Set Interrupt Priority Mask Register (ICCPMR). Enable interrupts of all
    // priorities
    *((int *) 0xFFFEC104) = 0xFFFF;
    // Set CPU Interface Control Register (ICCICR). Enable signaling of
    // interrupts
    *((int *) 0xFFFEC100) = 1;
    // Configure the Distributor Control Register (ICDDCR) to send pending
    // interrupts to CPUs
    *((int *) 0xFFFED000) = 1;
}

/*
* Configure Set Enable Registers (ICDISERn) and Interrupt Processor Target
* Registers (ICDIPTRn). The default (reset) values are used for other registers
* in the GIC.
*/
void config_interrupt(int N, int CPU_target) {
    int reg_offset, index, value, address;
    /* Configure the Interrupt Set-Enable Registers (ICDISERn).
    * reg_offset = (integer_div(N / 32) * 4
    * value = 1 << (N mod 32) */
    reg_offset = (N >> 3) & 0xFFFFFFFC;
    index = N & 0x1F;
    value = 0x1 << index;
    address = 0xFFFED100 + reg_offset;
    /* Now that we know the register address and value, set the appropriate bit */
    *(int *)address |= value;
    /* Configure the Interrupt Processor Targets Register (ICDIPTRn)
    * reg_offset = integer_div(N / 4) * 4
    * index = N mod 4 */
    reg_offset = (N & 0xFFFFFFFC);
    index = N & 0x3;
    address = 0xFFFED800 + reg_offset + index;
    /* Now that we know the register address and value, write to (only) the
    * appropriate byte */
    *(char *)address = (char)CPU_target;
}

void PS2_ISR(void) {
    volatile int * PS2_ptr = (int *)PS2_BASE;
    int PS2_data, RVALID;

    PS2_data = *(PS2_ptr); // read the Data register in the PS/2 port
    RVALID = PS2_data & 0x8000; // extract the RVALID field

    if (RVALID) {
        /* shift the next data byte into the display */
        byte1 = byte2;
        byte2 = byte3;
        byte3 = PS2_data & 0xFFFF;
        HEX_PS2(byte1, byte2, byte3);

        if ((byte2 == (char)0xAA) && (byte3 == (char)0x00))
        // mouse inserted; initialize sending of data
        *(PS2_ptr) = 0xF4;

        if ((byte2 == (char)0xFA) && (byte3 == (char)0xAA))
        initialized = 1;
        else if (initialized) {
            char x_sign = byte1 >> 4;
            x_sign &= 0x1;
            char y_sign = byte1 >> 5;
            y_sign &= 0x1;

            char x_overflow = byte1 >> 6;
            x_overflow &= 0x1;
            char y_overflow = byte1 >> 7;
            y_overflow &= 0x1;

            cursor.dx = twoCompToInt(byte2, x_sign, x_overflow);
            cursor.dy = twoCompToInt(byte3, y_sign, y_overflow); 
			
			char left_click = byte1 & 0x1;
			if (left_click == (char)0x01) {
				cursor.color = GREEN;
				if ((cursor.x + BOX_LEN >= object.x && cursor.x <= object.x + BOX_LEN) && 
					(cursor.y + BOX_LEN >= object.y && cursor.y <= object.y + BOX_LEN)) {
						object.dx = cursor.dx;
						object.dy = cursor.dy;
				}
			} else {	
				cursor.color = WHITE;
				object.dx = 0;
				object.dy = 0;
			}
        }
    }
}

int twoCompToInt(char value, char sign, char overflow) {
    int num = 0;

    // Max dx/dy for overflow 
    if (overflow == (char)0x01) {
        if (sign == (char)0x01) {
            num = -25;
        } else {
            num = 25;
        }
    }
    
    if (sign & 0x01) {
        num += (int)((unsigned char)value | 0xFFFFFF00);
    } else {
        num += (int)value;
    }

    return num;
}

void HEX_PS2(char b1, char b2, char b3) {
    volatile int * HEX3_HEX0_ptr = (int *)HEX3_HEX0_BASE;
    volatile int * HEX5_HEX4_ptr = (int *)HEX5_HEX4_BASE;

    /* SEVEN_SEGMENT_DECODE_TABLE gives the on/off settings for all segments in
    * a single 7-seg display in the DE1-SoC Computer, for the hex digits 0 - F
    */
    unsigned char seven_seg_decode_table[] = {
        0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
        0x7F, 0x67, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};
    unsigned char hex_segs[] = {0, 0, 0, 0, 0, 0, 0, 0};
    unsigned int shift_buffer, nibble;
    unsigned char code;
    int i;
    shift_buffer = (b1 << 16) | (b2 << 8) | b3;

    for (i = 0; i < 6; ++i) {
        nibble = shift_buffer & 0x0000000F; // character is in rightmost nibble
        code = seven_seg_decode_table[nibble];
        hex_segs[i] = code;
        shift_buffer = shift_buffer >> 4;
    }

    /* drive the hex displays */
    *(HEX3_HEX0_ptr) = *(int *)(hex_segs);
    *(HEX5_HEX4_ptr) = *(int *)(hex_segs + 4);
}

void set_up_pixel_buf_ctrl() {
    /* set front pixel buffer to start of FPGA On-chip memory */
    *(G_PIXEL_BUF_CTRL_PTR + 1) = FPGA_ONCHIP_BASE; // first store the address in the back buffer
    /* initialize a pointer to the pixel buffer, used by drawing functions */
    g_pixel_back_buffer = *(G_PIXEL_BUF_CTRL_PTR + 1);
    clear_screen(); // g_pixel_back_buffer points to the pixel buffer
    /* now, swap the front/back buffers, to set the front buffer location */
    wait_for_vsync();

    /* set back pixel buffer to start of SDRAM memory */
    *(G_PIXEL_BUF_CTRL_PTR + 1) = SDRAM_BASE;
    g_pixel_back_buffer = *(G_PIXEL_BUF_CTRL_PTR + 1); // we draw on the back buffer
    clear_screen(); // g_pixel_back_buffer points to the pixel buffer
}

void clear_screen() {
    int i;
    int j;
    for (i = 0; i < RESOLUTION_X; i++) {
        for (j = 0; j < RESOLUTION_Y; j++) {
            plot_pixel(i, j, BLACK); // 0 is black
        }
    }
}

void wait_for_vsync() {
    register int status;

    *G_PIXEL_BUF_CTRL_PTR = 1;

    status = *(G_PIXEL_BUF_CTRL_PTR + 3);
    while ((status & 0x01) != 0) {
        status = *(G_PIXEL_BUF_CTRL_PTR + 3);
    }
}

void plot_pixel(int x, int y, short int color) {
    *(short int *)(g_pixel_back_buffer + (y << 10) + (x << 1)) = color;
}

struct Box prior_box(struct Box box) {
    return construct_box(box.prior_x, box.prior_y, 0, 0, BLACK);
}

void erase_box(struct Box box) {
    if (box.prior_x != -1 && box.prior_y != -1)
        draw_box(prior_box(box));
}

void draw_box(struct Box box) {
    int i, j;
    for (i = box.x; i < box.x + BOX_LEN; i++) {
        for (j = box.y; j < box.y + BOX_LEN; j++) {
            plot_pixel(i, j, box.color);
        }
    }
}

void move_box(struct Box* box) {
    box->prior_x = box->prev_x;
    box->prior_y = box->prev_y;
    box->prev_x = box->x;
    box->prev_y = box->y;

    box->x += box->dx;
    box->y += box->dy;

    if (box->x > MAX_BOX_X) {
        box->x = MAX_BOX_X;
    } else if (box->x < 0) {
        box->x = 0;
    }

    if (box->y > MAX_BOX_Y) {
        box->y = MAX_BOX_Y;
    } else if (box->y < 0) {
        box->y = 0;
    }

    box->dx = 0;
    box->dy = 0;
}

struct Box construct_box(int x, int y, int dx, int dy, short int color) {
    struct Box box;
    box.x = x;
    box.y = y;

    box.dx = dx;
    box.dy = dy;

    box.prev_x = -1;
    box.prev_y = -1;
    box.prior_x = -1;
    box.prior_y = -1;

    box.color = color;
    return box;
}
