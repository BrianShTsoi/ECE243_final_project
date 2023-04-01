/* This files provides address values that exist in the system */

#define SDRAM_BASE            0xC0000000
#define FPGA_ONCHIP_BASE      0xC8000000
#define FPGA_CHAR_BASE        0xC9000000

/* Cyclone V FPGA devices */
#define LEDR_BASE             0xFF200000
#define HEX3_HEX0_BASE        0xFF200020
#define HEX5_HEX4_BASE        0xFF200030
#define SW_BASE               0xFF200040
#define KEY_BASE              0xFF200050
#define TIMER_BASE            0xFF202000
#define PIXEL_BUF_CTRL_BASE   0xFF203020
#define CHAR_BUF_CTRL_BASE    0xFF203030

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

#define ABS(x) (((x) > 0) ? (x) : -(x))

/* Screen size. */
#define RESOLUTION_X 320
#define RESOLUTION_Y 240

/* Constants for animation */
#define BOX_LEN 2
#define NUM_BOXES 8

#define FALSE 0
#define TRUE 1

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

struct Box {
    int x; 
    int y; 
    int dx; 
    int dy; 
    short int color;
};

const int MAX_BOX_X = RESOLUTION_X - BOX_LEN;
const int MAX_BOX_Y =  RESOLUTION_Y - BOX_LEN;
const short int COLORS[10] = {WHITE, YELLOW, RED, GREEN, BLUE, 
                                CYAN, MAGENTA, GREY, PINK, ORANGE};

// Begin part3.c code for Lab 7
void plot_pixel(int x, int y, short int color);
void clear_screen();
void draw_line(int x0, int y0, int x1, int y1, short int line_color);
void swap(int* num0, int* num1);
void wait_for_vsync();

struct Box prev_box(struct Box box);
void erase_box(struct Box box);
void erase_boxes(struct Box boxes[NUM_BOXES]);
void draw_box(struct Box box);
void draw_boxes(struct Box boxes[NUM_BOXES]);
void move_box(struct Box* box);
void move_boxes(struct Box boxes[NUM_BOXES]);
void setup_box(struct Box* box);
void setup_boxes(struct Box boxes[NUM_BOXES]);
void draw_box_line(struct Box box0, struct Box box1);
void erase_lines(struct Box boxes[NUM_BOXES]);
void draw_lines(struct Box boxes[NUM_BOXES]);


volatile int pixel_buffer_start; // global variable

int main(void) {
    volatile int * pixel_ctrl_ptr = (int *)0xFF203020;
    // declare other variables(not shown)
    // struct Box box = {300, 200, 1, 1, WHITE};
    struct Box boxes[NUM_BOXES];
    setup_boxes(boxes);
    // initialize location and direction of rectangles(not shown)

    /* set front pixel buffer to start of FPGA On-chip memory */
    *(pixel_ctrl_ptr + 1) = 0xC8000000; // first store the address in the 
                                        // back buffer
    /* now, swap the front/back buffers, to set the front buffer location */
    wait_for_vsync();
    /* initialize a pointer to the pixel buffer, used by drawing functions */
    pixel_buffer_start = *pixel_ctrl_ptr;
    clear_screen(); // pixel_buffer_start points to the pixel buffer
    /* set back pixel buffer to start of SDRAM memory */
    *(pixel_ctrl_ptr + 1) = 0xC0000000;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // we draw on the back buffer
    clear_screen(); // pixel_buffer_start points to the pixel buffer

    while (1)
    {
        /* Erase any boxes and lines that were drawn in the last iteration */
        // erase_box(box);
        // draw_box(box);
        // move_box(&box);
        erase_boxes(boxes);
        erase_lines(boxes);
        draw_boxes(boxes);
        draw_lines(boxes);
        move_boxes(boxes);

        // code for drawing the boxes and lines (not shown)
        // code for updating the locations of boxes (not shown)

        wait_for_vsync(); // swap front and back buffers on VGA vertical sync
        pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
    }
}

void plot_pixel(int x, int y, short int color)
{
    *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = color;
}

void clear_screen() 
{
    int i;
    int j;
    for (i = 0; i < RESOLUTION_X; i++) {
        for (j = 0; j < RESOLUTION_Y; j++) {
            plot_pixel(i, j, 0); // 0 is black
        }
    }
}

void draw_line(int x0, int y0, int x1, int y1, short int line_color) {
    int is_steep = ABS(y1 - y0) > ABS(x1 - x0);
    if (is_steep) {
        swap(&x0, &y0);
        swap(&x1, &y1);
    }
    if (x0 > x1) {
        swap(&x0, &x1);
        swap(&y0, &y1);
    }

    int dx = x1 - x0;
    int dy = ABS(y1 - y0);
    int error = -(dx / 2);
    int y = y0;
    int y_step = 1;
    if (y0 > y1) {
        y_step = -1;
    }

    int x;
    for (x = x0; x < x1; x++) {
        if (is_steep)
            plot_pixel(y, x, line_color);
        else
            plot_pixel(x, y, line_color);
        error = error + dy;
        if (error > 0) {
            y = y + y_step;
            error -= dx;
        }
    }

}

void swap(int* num0, int* num1) {
    int temp = *num0;
    *num0 = *num1;
    *num1 = temp;
}

void wait_for_vsync() {
    volatile int* pixel_ctrl_ptr = 0xFF203020;
    register int status;

    *pixel_ctrl_ptr = 1;

    status = *(pixel_ctrl_ptr + 3);
    while ((status & 0x01) != 0) {
        status = *(pixel_ctrl_ptr + 3);
    }
}

struct Box prev_box(struct Box box) {
    struct Box prev_box;
    prev_box.x = box.x - 2 * box.dx;
    prev_box.y = box.y - 2 * box.dy;
    prev_box.dx = box.dx;
    prev_box.dy = box.dy;
    prev_box.color = 0;
    // Check if invalid, then "mirror"
    if (prev_box.x < 0) {
        prev_box.x -= 2 * (prev_box.x);
        prev_box.dx *= -1;
    } else if (prev_box.x > MAX_BOX_X) {
        prev_box.x -= 2 * (prev_box.x - MAX_BOX_X);
        prev_box.dx *= -1;
    }

    if (prev_box.y < 0) {
        prev_box.y -= 2 * (prev_box.y);
        prev_box.dy *= -1;
    } else if (prev_box.y > MAX_BOX_Y) {
        prev_box.y -= 2 * (prev_box.y - MAX_BOX_Y);
        prev_box.dy *= -1;
    }

    return prev_box;
}

void erase_box(struct Box box) {
    draw_box(prev_box(box));
}

void erase_boxes(struct Box boxes[NUM_BOXES]) {
    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        erase_box(boxes[i]);
    }
}

void draw_box(struct Box box) {
    int i, j;
    for (i = box.x; i < box.x + BOX_LEN; i++) {
        for (j = box.y; j < box.y + BOX_LEN; j++) {
            plot_pixel(i, j, box.color);
        }
    }
}

void draw_boxes(struct Box boxes[NUM_BOXES]) {
    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        draw_box(boxes[i]);
    }
}

void move_box(struct Box* box) {
    box->x += box->dx;
    box->y += box->dy;
    if (box->x == 0 || box->x == MAX_BOX_X) {
        box->dx *= -1;
    }
    if (box->y == 0 || box->y == MAX_BOX_Y) {
        box->dy *= -1;
    }
}

void move_boxes(struct Box boxes[NUM_BOXES]) {
    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        move_box(&boxes[i]);
    }
}

void setup_box(struct Box* box) {
    box->x = rand() % MAX_BOX_X;
    box->y = rand() % MAX_BOX_Y;
    box->dx = (rand() % 2 * 2) - 1;
    box->dy = (rand() % 2 * 2) - 1;
    box->color = COLORS[rand() % 10];

    // Check for dx and dy
    if (box->x == 0) {
        box->dx = 1;
    } else if (box->x == MAX_BOX_X) {
        box->dx = -1;
    }
    if (box->y == 0) {
        box->dy = 1;
    } else if (box->y == MAX_BOX_Y) {
        box->dy = -1;
    }
}

void setup_boxes(struct Box boxes[NUM_BOXES]) {
    srand(time(NULL));
    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        setup_box(&boxes[i]);
    }
}

void draw_box_line(struct Box box0, struct Box box1) {
    draw_line(box0.x, box0.y, box1.x, box1.y, box0.color);
}

void erase_lines(struct Box boxes[NUM_BOXES]) {
    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        draw_box_line(prev_box(boxes[i]), prev_box(boxes[(i + 1) % NUM_BOXES]));
    }
}

void draw_lines(struct Box boxes[NUM_BOXES]) {
    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        draw_box_line(boxes[i], boxes[(i + 1) % NUM_BOXES]);
    }
}