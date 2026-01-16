#include "RoboHW.h"
#include "RoboNav.h"
#include "RoboMap.h"
#include <cmath>

Node nodes[max_nodes];

int node_cnt = 0;
int target_dir = -1;
int depart_node = -1;
int depart_dir = -1;


int cell_size_mm;
int robot_width_mm;
int robot_half_mm;
int safety_margin_mm;
int inflate_radius_mm = robot_half + safety_margin;

int grid_w = 0;
int grid_h = 0;
float grid_origin_x_mm = 0.0f;
float grid_origin_y_mm = 0.0f;
int16_t *grid_logodds = nullptr;

const int occ_inc = 3;
const int free_dec = 1;
const int lo_min = -100;
const int lo_max = 100;
const int occ_th = 40;

void grid_init(int maze_w_mm, int maze_h_mm, int cell_size_mm){
  grid_w = (int)ceil((float)maze_w_mm / (float)cell_size_mm) + 6;
  grid_h = (int)ceil((float)maze_h_mm / (float)cell_size_mm) + 6;
  grid_origin_x_mm = -(grid_w * (float)cell_size_mm) / 2.0f;
  grid_origin_y_mm = -(grid_h * (float)cell_size_mm) / 2.0f;
  size_t total_cells = (size_t)grid_w * (size_t)grid_h;
  if(grid_logodds){
    free(grid_logodds);
    grid_logodds = nullptr;
  }

  grid_logodds = (int16_t*) calloc(total_cells, sizeof(int16_t));
  if(!grid_logodds){
    Serial.println("Error: grid allocation failed");
    grid_w = grid_h = 0;
    return;}
}

bool world_to_grid(float wx_mm, wy_mm, int *out_gx, int *out_gy){
  float rx = (wx_mm - grid_origin_x_mm) / (float)cell_size_mm;
  float ry = (wy_mm - grid_origin_y_mm) / (float)cell_size_mm;
  int gx = (int)floorf(rx + 0.5f);
  int gy (int)floorf(ry  + 0.5f);
  if(gx < 0 || gy < 0 || gx >= grid_w || gy >= grid_h){
    return false;
  }
  *out_gx = gx;
  *out_gy = gy;
  return true;
}

static inline size_t grid_index(int gx, int gy){
  return (size_t)gy * (size_t)grid_w + (size_t)gx;
}

static inline void update_cell_free_idx(size_t idx){
  int v = (int)grid_logodds[idx] - free_dec;
  if(v < lo_min){
    v = lo_min;
  }
  grid_logodds[idx] = (int16_t)v;
}

static inline void update_cell_occ_idx(size_t idx){
  int v = (int)grid_logodds[idx] + occ_inc;
  if(v > lo_max){
    v = lo_max;
  }
  grid_logodds[idx] = (int16_t)v;
}

static void free_line(int x0, int y0, int x1, int y1){
  int dx = abs(x1 - x0);
  int sx = (x0 < x1) ? 1 : -1;
  int dy = -abs(y1 - y0);
  int sy = (y0 < y1) ? 1 : -1
  int err = dx + dy;
  int x = x0, y = y0;
  while(true){
    if(x == x1 && y == y1){
      break;

      size_t idx = grid_index(x,y);
      update_cell_free_idx(idx);
      int e2 = 2 * err;
      if(e2 >= dy){
        err += dy;
        x +=sx;
      }
      if(e2 <= dx){
        err += dx;
        y +=sy;
      }
    }
  }
}

static void inflate_at_cell(int cx, int cy, int inflateCells){
  if(cx < 0 || cy < 0){
    return;
  }
  int minx = cx - inflateCells;
  if(minx < 0){
    minx = 0;
  }
  int maxx = cx + inflateCells;
  if(maxx >= grid_w){
    maxx = grid_w - 1;
  }
  int miny = cy - inflateCells;
  if(miny < 0){
    miny = 0;
  }
  int maxy = cy + inflateCells;
  if(maxy >= grid_h){
    maxy = grid_h - 1;
  }
  int r2 = inflateCells * inflateCells;
  for(int x = minx; x <= maxx; ++x){
    for(int y = miny; y <= maxy; ++y){
      int dx = x - cx;
      int dy = y - cy;
      if(dx*dx + dy*dy <= r2){
        size_t idx = grid_index(x, y);
        if(grid_logodds[idx] < occ_th){
          grid_logodds[idx] = (int16_t)occ_th;
        }
      }
    }
  }
}



void grid_update(float robotX_mm, float robotY_mm, float robotYaw_rad, float sensor_dx_mm, float sensor_dy_mm, float sensor_ang_rad, float range_mm, float max_range_mm){
  float sx = robotX_mm + cosf(robotYaw_rad) * sensor_dx_mm-sinf(robotYaw_rad) * sensor_dy_mm;
  float sy = robotY_mm + sinf(robotYaw_rad) * sensor_dx_mm + cosf(robotYaw_rad) * sensor_dy_mm;
  float beam = robotYaw_rad + sensor_ang_rad;
  bool end_hit = (range_mm < mx_range_mm);
  float ex = sx + rnge_mm * cosf(beam);
  float ey = sy + range_mm * sinf(beam);

  if(!end_hit){
    ex = sx + max_range_mm * cosf(beam);
    ey = sy + max_range_mm * sinf(beam);
  }
  int sxg, syg, exg, eyg;
  if(!world_to_grid(sx, sy, &sxg, &syg)){
    return;
  }
  if(!world_to_grid(ex, ey, &exg, &eyg)){
    if(exg < 0){
      exg = 0
    }else if(exg >= grid_w){
      exg = grid_w - 1;
    }
  }
    if(eyg < 0){
      eyg = 0
    }else if(eyg >= grid_h){
      eyg = grid_h - 1;
    }
  }

  free_line(sxg, syg, exg, eyg);

  if(end_hit){
    size_t idx_end = grid_index(exg, eyg);
    update_cll_occ_idx(idx_end);
    int inflateCells = (int)ceil((float)inflate_radius_mm / (float)cell_size_mm);
    inflate_at_cell(exg, eygm inflateCells);
  }
}

bool grid_front_path_clear(float robotX_mm, float robotY_mm, float robotYaw_rad, float dist_mm, float half_w_mm)

void grid_save_coll(float world_x_mm, float world_y_mm)


void walls(Pose &r){
  for(int i = 0; i <= 3; i++){
  nodes[crnt_node].open[i] = false;
  }
  bool front = read_us() > front_sensor_th;
  bool left = read_l_ir() > side_sensor_th;
  bool right = read_r_ir() > side_sensor_th;
  if(r.o == up){
    nodes[crnt_node].open[0] = front;
    nodes[crnt_node].open[1] = right;
    nodes[crnt_node].open[3] = left;
  }else if(r.o == rt){
    nodes[crnt_node].open[0] = left;
    nodes[crnt_node].open[1] = front;
    nodes[crnt_node].open[2] = right;
  }else if(r.o == dn){
    nodes[crnt_node].open[1] = left;
    nodes[crnt_node].open[2] = front;
    nodes[crnt_node].open[3] = right;
  }else if(r.o == lt){
    nodes[crnt_node].open[0] = right;
    nodes[crnt_node].open[2] = left;
    nodes[crnt_node].open[3] = front;
  }
  if(nodes[crnt_node].parent != -1){
    nodes[crnt_node].open[nodes[crnt_node].parent_dir] = true;
  }
}

void init_node(int id){
  for(int i = 0; i <= 3; i++){
    nodes[id].open[i] = false;
    nodes[id].explored[i] = false;
    nodes[id].link[i] = -1;
  }
  nodes[id].parent = -1;
  nodes[id].parent_dir = -1;
}

void new_node(){
  if (node_cnt == 1 && crnt_node == 0 && chosen_dir == -1){
    return;
  }
  if(chosen_dir != nodes[crnt_node].parent_dir){
    depart_node = crnt_node;
    crnt_node = node_cnt;
    node_cnt++;
    init_node(crnt_node);
    nodes[depart_node].link[chosen_dir] = crnt_node;
    nodes[crnt_node].parent = depart_node;

    nodes[crnt_node].parent_dir = (chosen_dir +2)%4;
    nodes[crnt_node].explored[nodes[crnt_node].parent_dir] = true;
  }else{
    crnt_node= nodes[crnt_node].parent;
  }
}

void map_init(){
  init_node(0);
  node_cnt = 1;
  crnt_node = 0;
}
