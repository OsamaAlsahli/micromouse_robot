#pragma once
#include <stdint.h>
#include <mbed.h>

struct Pose;

struct Node{
  bool open [4];
  bool explored [4];
  int link [4];
  int parent;
  int parent_dir;
};

constexpr int max_nodes = 50;
extern Node nodes[max_nodes];

extern int chosen_dir;
extern int target_dir;
extern int depart_node;
extern int depart_dir;

static constexpr int cell_size_mm; = 25;
static constexpr int robot_width_mm = 180;
static constexpr int robot_half_mm = 90;
static constexpr int safety_margin_mm = 30;
static constexpr int inflate_radius_mm = robot_half + safety_margin;

extern int grid_w;
extern int grid_h;
extern float grid_origin_x_mm;
extern float grid_origin_y_mm;
extern int16_t *grid_logodds;

void grid_init(int maze_w_mm, int maze_h_mm, int cell_size_mm);
bool world_to_grid(float wx_mm, wy_mm, int *out_gx, int *out_gy);
static inline size_t grid_index(int gx, int gy);
static inline void update_cell_free_idx(size_t idx);
static inline void update_cell_occ_idx(size_t idx);
static void free_line(int x0, int y0, int x1, int y1);
static void inflate_at_cell(int cx, int cy, int inflateCells);
void grid_update(float robotX_mm, float robotY_mm, float robotYaw_rad, float sensor_dx_mm, float sensor_dy_mm, float sensor_ang_rad, float range_mm, float max_range_mm);
bool grid_front_path_clear(float robotX_mm, float robotY_mm, float robotYaw_rad, float dist_mm, float half_w_mm);
void grid_save_coll(float world_x_mm, float world_y_mm);

void init_node(int id);
void map_init();
void walls(const Pose &r);
void new_node();
