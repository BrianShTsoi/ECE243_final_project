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
#define BLACK 0x0000

#define ABS(x) (((x) > 0) ? (x) : -(x))

/* Screen size. */
#define RESOLUTION_X 320
#define RESOLUTION_Y 240

/* Constants for animation */
#define BOX_LEN 10
#define NUM_BOXES 6

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

    int prev_x;
    int prev_y;
    int prior_x;
    int prior_y;

    short int color;
};

const int MAX_BOX_X = RESOLUTION_X - BOX_LEN;
const int MAX_BOX_Y =  RESOLUTION_Y - BOX_LEN;
const short int COLORS[10] = {WHITE, YELLOW, RED, GREEN, BLUE, 
                                CYAN, MAGENTA, GREY, PINK, ORANGE};

void plot_pixel(int x, int y, short int color);
void clear_screen();
void draw_line(int x0, int y0, int x1, int y1, short int line_color);
void swap(int* num0, int* num1);
void wait_for_vsync();

struct Box prior_box(struct Box box);
void erase_box(struct Box box);
void erase_boxes(struct Box boxes[NUM_BOXES]);
void draw_box(struct Box box);
void draw_boxes(struct Box boxes[NUM_BOXES]);
void move_box(struct Box* box);
void move_boxes(struct Box boxes[NUM_BOXES]);

struct Box construct_box(int x, int y, int dx, int dy, short int color);
void set_up_random_box(struct Box* box);
void set_up_random_boxes(struct Box boxes[NUM_BOXES]);
void set_up_still_boxes(struct Box boxes[NUM_BOXES]);
void set_up_pixel_buf_ctrl();

void draw_box_line(struct Box box0, struct Box box1);
void erase_lines(struct Box boxes[NUM_BOXES]);
void draw_lines(struct Box boxes[NUM_BOXES]);

volatile int g_pixel_back_buffer; // global variable
volatile int * const G_PIXEL_BUF_CTRL_PTR = (int *) PIXEL_BUF_CTRL_BASE;

int main(void) {
    struct Box boxes[NUM_BOXES];
    // set_up_random_boxes(boxes);
    set_up_still_boxes(boxes);
    set_up_pixel_buf_ctrl();

    while (1) {
        erase_boxes(boxes);
        // erase_lines(boxes);
        draw_boxes(boxes);
        // draw_lines(boxes);
        move_boxes(boxes);

        wait_for_vsync(); // swap front and back buffers on VGA vertical sync
        g_pixel_back_buffer = *(G_PIXEL_BUF_CTRL_PTR + 1); // new back buffer
    }
}

void plot_pixel(int x, int y, short int color) {
    *(short int *)(g_pixel_back_buffer + (y << 10) + (x << 1)) = color;
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
    register int status;

    *G_PIXEL_BUF_CTRL_PTR = 1;

    status = *(G_PIXEL_BUF_CTRL_PTR + 3);
    while ((status & 0x01) != 0) {
        status = *(G_PIXEL_BUF_CTRL_PTR + 3);
    }
}

struct Box prior_box(struct Box box) {
    return construct_box(box.prior_x, box.prior_y, 0, 0, BLACK);
}

void erase_box(struct Box box) {
    if (box.prior_x != -1 && box.prior_y != -1)
        draw_box(prior_box(box));
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
    box->prior_x = box->prev_x;
    box->prior_y = box->prev_y;
    box->prev_x = box->x;
    box->prev_y = box->y;

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

void set_up_random_box(struct Box* box) {
    int x = rand() % MAX_BOX_X;
    int y = rand() % MAX_BOX_Y;
    int dx = (rand() % 2 * 2) - 1;
    int dy = (rand() % 2 * 2) - 1;
    short int color = COLORS[rand() % 10];

    if (x == 0) {
        dx = 1;
    } else if (x == MAX_BOX_X) {
        dx = -1;
    }

    if (y == 0) {
        dy = 1;
    } else if (y == MAX_BOX_Y) {
        dy = -1;
    }

    *box = construct_box(x, y, dx, dy, color);
}

void set_up_random_boxes(struct Box boxes[NUM_BOXES]) {
    srand(time(NULL));
    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        set_up_random_box(&boxes[i]);
    }
}

void set_up_still_boxes(struct Box boxes[NUM_BOXES]) {
    int coord_spacing = BOX_LEN * 3;
    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        short int color = COLORS[rand() % 10];
        boxes[i] = construct_box(i*coord_spacing, i*coord_spacing, 0, 0, color);
    }
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

void draw_box_line(struct Box box0, struct Box box1) {
    draw_line(box0.x, box0.y, box1.x, box1.y, box0.color);
}

void erase_lines(struct Box boxes[NUM_BOXES]) {
    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        if (boxes[i].prior_x != -1 && boxes[i].prior_y != -1)
            draw_box_line(prior_box(boxes[i]), prior_box(boxes[(i + 1) % NUM_BOXES]));
    }
}

void draw_lines(struct Box boxes[NUM_BOXES]) {
    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        draw_box_line(boxes[i], boxes[(i + 1) % NUM_BOXES]);
    }
}