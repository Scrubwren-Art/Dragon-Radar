#ifndef GRID_SCREEN_H
#define GRID_SCREEN_H
#include "lvgl.h"
#include <stdbool.h>

/* Radar target definition */
typedef struct
{
  float x;        /* X position in world space (-50 to 50) */
  float y;        /* Y position in world space (-50 to 50) */
  float distance; /* Distance factor 0.0-1.0 (0=close/large, 1=far/small) */
} radar_target_t;

void grid_screen_show(void);
void grid_screen_update(void);
void grid_screen_start_scan(void);
bool grid_screen_is_scan_complete(void);
void grid_screen_zoom_increment(void);

#endif
