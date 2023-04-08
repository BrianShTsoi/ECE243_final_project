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
#define INIT_GRID_SIZE 8
#define NUM_BOX_EDGES 3

#define TOLERANCE 1e-9

#define FALSE 0
#define TRUE 1

#define _USE_MATH_DEFINES

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <assert.h>

struct Edge {
    int b0;
    int b1;
};

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

    struct Edge edges[NUM_BOXES];
    int num_edges;
};

const int MAX_BOX_X = RESOLUTION_X - BOX_LEN;
const int MAX_BOX_Y =  RESOLUTION_Y - BOX_LEN;
const short int COLORS[10] = {WHITE, YELLOW, RED, GREEN, BLUE, 
                                CYAN, MAGENTA, GREY, PINK, ORANGE};

void plot_pixel(int x, int y, short int color);
void clear_screen();
void draw_line(int x0, int y0, int x1, int y1, short int line_color);
void swap(int* num0, int* num1);
void swap_double(double* num0, double* num1);
void wait_for_vsync();

struct Box prior_box(struct Box box);
void erase_box(struct Box box);
void erase_boxes(struct Box boxes[NUM_BOXES]);
void draw_box(struct Box box);
void draw_boxes(struct Box boxes[NUM_BOXES]);
void move_box(struct Box* box);
void move_boxes(struct Box boxes[NUM_BOXES]);

struct Box construct_box(int x, int y, int dx, int dy, short int color);
void set_up_moving_box(struct Box* box);
void set_up_moving_boxes(struct Box boxes[NUM_BOXES]);
int coord_exist(struct Box boxes[NUM_BOXES], int num_existing_box, int x, int y);
int points_colinear(int x0, int y0, int x1, int y1, int x2, int y2);
int box_colinear(struct Box boxes[NUM_BOXES], int num_existing_box, int x, int y);
void set_up_still_boxes(struct Box boxes[NUM_BOXES]);
void set_up_pixel_buf_ctrl();

void draw_box_line(struct Box box0, struct Box box1, short int color);

void set_up_cyclic_edges(struct Box boxes[NUM_BOXES]);
int compare_edges(struct Edge edge0, struct Edge edge1);
int edge_exist(struct Box boxes[NUM_BOXES], struct Edge edge);

int lines_intersect(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3);
int edges_intersect(struct Box boxes[NUM_BOXES], struct Edge edge0, struct Edge edge1);
int new_edge_intersect(struct Box boxes[NUM_BOXES], struct Edge edge);
struct Edge discover_new_edge(struct Box boxes[NUM_BOXES], int b0);
int set_up_random_edge(struct Box boxes[NUM_BOXES], int b0);
void set_up_random_edges(struct Box boxes[NUM_BOXES]);
void position_boxes(struct Box boxes[NUM_BOXES]);

void draw_edge(struct Box boxes[NUM_BOXES], struct Edge edge);
void draw_edges(struct Box boxes[NUM_BOXES]);
void erase_edge(struct Box boxes[NUM_BOXES], struct Edge edge);
void erase_edges(struct Box boxes[NUM_BOXES]);

void print_edges_info(struct Box boxes[NUM_BOXES]);
void print_boxes_info(struct Box boxes[NUM_BOXES]);
void draw_loop(struct Box boxes[NUM_BOXES]);

volatile int g_pixel_back_buffer; // global variable
volatile int * const G_PIXEL_BUF_CTRL_PTR = (int *) PIXEL_BUF_CTRL_BASE;

int main(void) {
    srand(time(NULL));
    // for repeat testing
    while (1) {

        // srand(time(NULL));
        set_up_pixel_buf_ctrl();

        struct Box boxes[NUM_BOXES];
        // set_up_moving_boxes(boxes);
        set_up_still_boxes(boxes);
        // set_up_cyclic_edges(boxes);
        set_up_random_edges(boxes);

        position_boxes(boxes);

        int i;
        for (i = 0; i < 10; i++) {
            draw_loop(boxes);
        }

    // for repeat testing
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

void swap_double(double* num0, double* num1) {
    double temp = *num0;
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

    box.num_edges = 0;
    return box;
}

void set_up_moving_box(struct Box* box) {
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

void set_up_moving_boxes(struct Box boxes[NUM_BOXES]) {
    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        set_up_moving_box(&boxes[i]);
    }
}

int coord_exist(struct Box boxes[NUM_BOXES], int num_existing_box, int x, int y) {
    int i;
    for (i = 0; i < num_existing_box; i++) {
        if (boxes[i].x == x && boxes[i].y == y) {
            return TRUE;
        }
    }
    return FALSE;
}

int points_colinear(int x0, int y0, int x1, int y1, int x2, int y2) {
    int area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
    if (area == 0) {
        return TRUE;
    }
    return FALSE;
}

int box_colinear(struct Box boxes[NUM_BOXES], int num_existing_box, int x, int y) {
    if (num_existing_box == 0 || num_existing_box == 1) {
        return FALSE;
    }

    int i, j;
    for (i = 0; i < num_existing_box; i++) {
        for (j = i + 1; j < num_existing_box; j++) {
            if (i == j) continue;

            if (points_colinear(boxes[i].x, boxes[i].y, boxes[j].x, boxes[j].y, x, y)) {
                return TRUE;
            }
        }
    }
    return FALSE;
}

void set_up_still_boxes(struct Box boxes[NUM_BOXES]) {
    int x_spacing = MAX_BOX_X / (INIT_GRID_SIZE + 1);
    int y_spacing = MAX_BOX_Y / (INIT_GRID_SIZE + 1);

    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        int x, y;
        do {
            x = ((rand() % INIT_GRID_SIZE) + 1) * x_spacing;
            y = ((rand() % INIT_GRID_SIZE) + 1) * y_spacing;
            // x = (rand() % MAX_BOX_X);
            // y = (rand() % MAX_BOX_Y);

        } while (coord_exist(boxes, i, x, y) || box_colinear(boxes, i, x, y));

        short int color = COLORS[i % 10];
        boxes[i] = construct_box(x, y, 0, 0, color);
    }
    draw_loop(boxes);
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

void draw_box_line(struct Box box0, struct Box box1, short int color) {
    draw_line(box0.x, box0.y, box1.x, box1.y, color);
}

void set_up_cyclic_edges(struct Box boxes[NUM_BOXES]) {
    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        int a = i;
        int b = (i + 1) % NUM_BOXES;
        struct Edge edge = {a, b};
        boxes[a].edges[boxes[a].num_edges] = edge;
        boxes[a].num_edges++;
        boxes[b].edges[boxes[b].num_edges] = edge;
        boxes[b].num_edges++;
    }
}

int compare_edges(struct Edge edge0, struct Edge edge1) {
    return (edge0.b0 == edge1.b0 && edge0.b1 == edge1.b1);
}

int edge_exist(struct Box boxes[NUM_BOXES], struct Edge edge) {
    int i, j;
    for (i = 0; i < NUM_BOXES; i++) {
        for (j = 0; j < boxes[i].num_edges; j++) {
            if (compare_edges(boxes[i].edges[j], edge)) return TRUE;
        }
    }

    return FALSE; 
}

int lines_intersect(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3) {
    // "perpendicular lines", will cause division by zero when finding slope
    if (ABS(x1 - x0) < TOLERANCE ) {
        if (ABS(y3 - y2) < TOLERANCE) {
            return (x1 > fmin(x2, x3) && x1 < fmax(x2, x3) &&
                    y3 > fmin(y0, y1) && y3 < fmax(y0, y1));
        }
        else {
            swap_double(&x0, &y0);
            swap_double(&x1, &y1);
            swap_double(&x2, &y2);
            swap_double(&x3, &y3);
        }
    }
    if (ABS(x3 - x2) < TOLERANCE) {
        if (ABS(y1 - y0) < TOLERANCE) {
            return (x3 > fmin(x0, x1) && x3 < fmax(x0, x1) &&
                    y1 > fmin(y2, y3) && y1 < fmax(y2, y3));
        }
        else {
            swap_double(&x0, &y0);
            swap_double(&x1, &y1);
            swap_double(&x2, &y2);
            swap_double(&x3, &y3);
        }
    }

    // equation of line y = mx + b
    double m = (y1 - y0) / (x1 - x0); 
    double b = y0 - m * x0;
    // equation of line y = nx + c
    double n = (y3 - y2) / (x3 - x2);
    double c = y2 - n * x2;
    if (m == n) {
        return FALSE;
    }

    double x_intersect = (c - b) / (m - n);
    if (x_intersect < fmin(x0, x1) || x_intersect > fmax(x0, x1) ||
        x_intersect < fmin(x2, x3) || x_intersect > fmax(x2, x3)) {
        return FALSE;
    }
    return TRUE;
}

int edges_intersect(struct Box boxes[NUM_BOXES], struct Edge edge0, struct Edge edge1) {
    int x0 = boxes[edge0.b0].x;
    int y0 = boxes[edge0.b0].y;

    int x1 = boxes[edge0.b1].x;
    int y1 = boxes[edge0.b1].y;

    int x2 = boxes[edge1.b0].x;
    int y2 = boxes[edge1.b0].y;

    int x3 = boxes[edge1.b1].x;
    int y3 = boxes[edge1.b1].y;

    if (x0 == x2 && y0 == y2) return FALSE;
    if (x1 == x2 && y1 == y2) return FALSE;
    if (x0 == x3 && y0 == y3) return FALSE;
    if (x1 == x3 && y1 == y3) return FALSE;

    return lines_intersect((double) x0, (double) y0, (double) x1, (double) y1, 
                           (double) x2, (double) y2, (double) x3, (double) y3);
}

int new_edge_intersect(struct Box boxes[NUM_BOXES], struct Edge edge) {
    int i, j;
    for (i = 0; i < NUM_BOXES; i++) {
        for (j = 0; j < boxes[i].num_edges; j++) {
            if (edges_intersect(boxes, boxes[i].edges[j], edge)) {
                return TRUE;
            }
        }
    }
    return FALSE;
}

struct Edge most_qualified_edge(struct Box boxes[NUM_BOXES], int b0, struct Edge edges[NUM_BOXES], int num_edges) {
    struct Edge edge;
    int min_num_box_edges = NUM_BOXES;
    int i;
    for (i = 0; i < num_edges; i++) {
        int b1 = edges[i].b0 == b0 ? edges[i].b1 : edges[i].b0;

        if (min_num_box_edges >= boxes[b1].num_edges) {
            min_num_box_edges = boxes[b1].num_edges;
            edge.b0 = edges[i].b0;
            edge.b1 = edges[i].b1;
        }
    }
    return edge;
}

struct Edge discover_new_edge(struct Box boxes[NUM_BOXES], int b0) {
    int num_valid_new_edges = 0;;
    struct Edge valid_new_edges[NUM_BOXES];

    int b1;
    for (b1 = 0; b1 < NUM_BOXES; b1++) {
        if (b0 == b1) continue;

        struct Edge new_edge;
        new_edge.b0 = b0 < b1 ? b0 : b1;
        new_edge.b1 = b0 < b1 ? b1 : b0;

        if (!edge_exist(boxes, new_edge) && !new_edge_intersect(boxes, new_edge)) {
            valid_new_edges[num_valid_new_edges].b0 = new_edge.b0;
            valid_new_edges[num_valid_new_edges].b1 = new_edge.b1;
            num_valid_new_edges++;
        }
    }
    // return valid_new_edges[rand() % num_valid_new_edges];
    if (num_valid_new_edges > 0) {
        return most_qualified_edge(boxes, b0, valid_new_edges, num_valid_new_edges);
    }
    else {
        struct Edge failed_edge = {-1, -1};
        return failed_edge;
    }
}

int set_up_random_edge(struct Box boxes[NUM_BOXES], int b0) {
    struct Edge new_edge = discover_new_edge(boxes, b0);
    if (new_edge.b0 == -1 && new_edge.b1 == -1) {
        return FALSE;
    }
    b0 = new_edge.b0;
    int b1 = new_edge.b1;
    boxes[b0].edges[boxes[b0].num_edges] = new_edge;
    boxes[b0].num_edges++;
    boxes[b1].edges[boxes[b1].num_edges] = new_edge;
    boxes[b1].num_edges++;

    draw_loop(boxes);
    return TRUE;
}

void set_up_random_edges(struct Box boxes[NUM_BOXES]) {
    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        while (boxes[i].num_edges < NUM_BOX_EDGES) {
            if (!set_up_random_edge(boxes, i)) {
                break;
            }
        }
    }
}

void position_boxes(struct Box boxes[NUM_BOXES]) {
    int center_x = MAX_BOX_X / 2;
    int center_y = MAX_BOX_Y / 2;
    int radius = MAX_BOX_Y / 3;
    double angle_rad = 2 * M_PI / NUM_BOXES;

    int i;
    for (i = 0; i < NUM_BOXES; i++) {
        int x_new = center_x + radius * cos(i * angle_rad);
        int y_new = center_y + radius * sin(i * angle_rad);

        move_box(&boxes[i]);
        boxes[i].x = x_new;
        boxes[i].y = y_new;
    }
}

void draw_edge(struct Box boxes[NUM_BOXES], struct Edge edge) {
    draw_box_line(boxes[edge.b0], boxes[edge.b1], WHITE);
}

void draw_edges(struct Box boxes[NUM_BOXES]) {
    int i, j;
    for (i = 0; i < NUM_BOXES; i++) {
        for (j = 0; j < boxes[i].num_edges; j++) {
            if (i == boxes[i].edges[j].b0) {
                draw_edge(boxes, boxes[i].edges[j]);
            }
        }
    }
}

void erase_edge(struct Box boxes[NUM_BOXES], struct Edge edge) {
    draw_box_line(prior_box(boxes[edge.b0]), prior_box(boxes[edge.b1]), BLACK);
}

void erase_edges(struct Box boxes[NUM_BOXES]) {
    int i, j;
    for (i = 0; i < NUM_BOXES; i++) {
        for (j = 0; j < boxes[i].num_edges; j++) {
            if (i == boxes[i].edges[j].b0) {
                erase_edge(boxes, boxes[i].edges[j]);
            }
        }
    }
}

void print_edges_info(struct Box boxes[NUM_BOXES]) {
    printf("PRINTING EDGE INFO................\r\n");
    int i, j;
    for (i = 0; i < NUM_BOXES; i++) {
        for (j = 0; j < boxes[i].num_edges; j++) {
            if (i == boxes[i].edges[j].b0) {
                printf("Edge: %d, %d\r\n", boxes[i].edges[j].b0, boxes[i].edges[j].b1);
            }
        }
    }
    printf("----------------------------------------------------------------\r\n");
}

void print_boxes_info(struct Box boxes[NUM_BOXES]) {
    printf("PRINTING BOX INFO................\r\n");
    int i, j;
    for (i = 0; i < NUM_BOXES; i++) {
        printf("box %d \n\r", i);
        printf("x: %d, y: %d\n\r", boxes[i].x, boxes[i].y);
        printf("Num of Edges: %d\n\r", boxes[i].num_edges);

        for (j = 0; j < boxes[i].num_edges; j++) {
            printf("edge %d \n\r", j);
            printf("%d, %d\n\r", boxes[i].edges[j].b0, boxes[i].edges[j].b1);
        }
        printf("----------------------------------------\r\n");

    }
    printf("----------------------------------------------------------------\r\n");
}

void draw_loop(struct Box boxes[NUM_BOXES]) {
    int i = 2;
    while (i--) {
        erase_boxes(boxes);
        erase_edges(boxes);
        draw_boxes(boxes);
        draw_edges(boxes);
        move_boxes(boxes);

        wait_for_vsync(); // swap front and back buffers on VGA vertical sync
        g_pixel_back_buffer = *(G_PIXEL_BUF_CTRL_PTR + 1); // new back buffer
    }
}