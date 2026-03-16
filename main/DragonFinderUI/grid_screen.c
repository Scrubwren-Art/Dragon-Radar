#include "grid_screen.h"
#include "QMI8658.h"
#include "Buzzer.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define ROWS 10
#define COLS 10
#define NUM_TARGETS 8
#define MAX_DISTANCE 50.0f
#define DOT_SIZE_MIN 3
#define DOT_SIZE_MAX 15

/* Static variables for grid state */
static lv_obj_t *grid_screen = NULL;
static float current_heading = 0.0f;
static uint32_t last_update_time = 0;
static int visible_dot_count = 0;		 /* Number of dots to show in scan mode */
static uint32_t scan_start_time = 0; /* When scan effect started */
static bool in_scan_mode = true;		 /* True during initial scan */
static uint32_t buzzer_on_time = 0;	 /* When buzzer was turned on for dot appearance */

/* Radar targets - positions will be randomized on init */
static radar_target_t radar_targets[NUM_TARGETS];

/* Helper: Convert radians to degrees */
static inline float rad_to_deg(float rad)
{
	return rad * 180.0f / 3.14159265f;
}

/* Helper: Convert degrees to radians */
static inline float deg_to_rad(float deg)
{
	return deg * 3.14159265f / 180.0f;
}

/* Helper: Rotate 2D point by angle in degrees */
static void rotate_point(float *x, float *y, float angle_deg)
{
	float rad = deg_to_rad(angle_deg);
	float cos_a = cosf(rad);
	float sin_a = sinf(rad);

	float x_new = (*x) * cos_a - (*y) * sin_a;
	float y_new = (*x) * sin_a + (*y) * cos_a;

	*x = x_new;
	*y = y_new;
}

/* Helper: Get random float between min and max */
static float random_float(float min, float max)
{
	return min + (float)rand() / RAND_MAX * (max - min);
}

/* Initialize radar targets with random positions */
static void init_random_targets(void)
{
	for (int i = 0; i < NUM_TARGETS; ++i)
	{
		radar_targets[i].x = random_float(-MAX_DISTANCE, MAX_DISTANCE);
		radar_targets[i].y = random_float(-MAX_DISTANCE, MAX_DISTANCE);
		radar_targets[i].distance = random_float(0.2f, 0.9f);
	}
}

/* Helper: Draw red filled equilateral triangle at center point */
static void draw_center_triangle(lv_draw_ctx_t *draw_ctx, int cx, int cy, int size)
{
	/* Equilateral triangle with base = size */
	float height = size * 0.866f; /* sqrt(3)/2 ≈ 0.866 */

	int x1 = cx; /* Top */
	int y1 = (int)(cy - height / 2);

	int x2 = (int)(cx - size / 2); /* Bottom-left */
	int y2 = (int)(cy + height / 2);

	int x3 = (int)(cx + size / 2); /* Bottom-right */
	int y3 = (int)(cy + height / 2);

	/* Draw filled triangle using scanlines */
	lv_draw_line_dsc_t fill_dsc;
	lv_draw_line_dsc_init(&fill_dsc);
	fill_dsc.color = lv_color_hex(0xFF0000); /* Red */
	fill_dsc.width = 1;

	/* Scanline fill from top to bottom */
	for (int y = y1; y <= y2; y++)
	{
		/* Calculate left and right X positions for this scanline */
		/* Left side: interpolate from top (x1,y1) to bottom-left (x2,y2) */
		float t_left = (y2 > y1) ? (float)(y - y1) / (float)(y2 - y1) : 0.0f;
		int left_x = (int)(x1 + t_left * (x2 - x1));

		/* Right side: interpolate from top (x1,y1) to bottom-right (x3,y3) */
		float t_right = (y3 > y1) ? (float)(y - y1) / (float)(y3 - y1) : 0.0f;
		int right_x = (int)(x1 + t_right * (x3 - x1));

		/* Draw horizontal fill line at this Y */
		lv_point_t p1 = {(lv_coord_t)left_x, (lv_coord_t)y};
		lv_point_t p2 = {(lv_coord_t)right_x, (lv_coord_t)y};
		lv_draw_line(draw_ctx, &fill_dsc, &p1, &p2);
	}

	/* Draw outline with thicker lines */
	lv_draw_line_dsc_t outline_dsc;
	lv_draw_line_dsc_init(&outline_dsc);
	outline_dsc.color = lv_color_hex(0xFF0000); /* Red */
	outline_dsc.width = 2;

	lv_point_t p1 = {(lv_coord_t)x1, (lv_coord_t)y1};
	lv_point_t p2 = {(lv_coord_t)x2, (lv_coord_t)y2};
	lv_point_t p3 = {(lv_coord_t)x3, (lv_coord_t)y3};

	lv_draw_line(draw_ctx, &outline_dsc, &p1, &p2);
	lv_draw_line(draw_ctx, &outline_dsc, &p2, &p3);
	lv_draw_line(draw_ctx, &outline_dsc, &p3, &p1);
}

/* Helper: Draw a fuzzy yellow dot at world position */
static void draw_fuzzy_dot(lv_draw_ctx_t *draw_ctx, int px, int py,
													 float distance_from_center)
{
	int scr_w = lv_disp_get_hor_res(NULL);
	int scr_h = lv_disp_get_ver_res(NULL);

	/* Calculate dot size and color based on distance from center of display
	 * Center (0,0) relative = largest/whitest, edges = smallest/yellowest */
	int center_x = scr_w / 2;
	int center_y = scr_h / 2;

	/* Distance from pixel position to center */
	int dx = px - center_x;
	int dy = py - center_y;
	float dist_to_center = sqrtf((float)(dx * dx + dy * dy));

	/* Max distance is diagonal from center to corner */
	float max_dist = sqrtf((float)(center_x * center_x + center_y * center_y)) - 10.0f; /* Subtract small margin to avoid zero size at corners */

	/* Normalize distance to 0-1 range (0 at center, 1 at corners) */
	float normalized_dist = dist_to_center / max_dist;
	if (normalized_dist > 1.0f)
		normalized_dist = 1.0f;

	/* Size: larger at center (0), smaller at edges (1) */
	int dot_size = DOT_SIZE_MAX - (int)(normalized_dist * (float)(DOT_SIZE_MAX - DOT_SIZE_MIN));
	if (dot_size < DOT_SIZE_MIN)
		dot_size = DOT_SIZE_MIN;

	/* Color based on center distance: closer to center (0.0) = whiter, farther (1.0) = more yellow */
	uint8_t brightness = (uint8_t)(255.0f * (1.0f - normalized_dist));

	/* Draw circle using rect with circular radius */
	lv_draw_rect_dsc_t rect_dsc;
	lv_draw_rect_dsc_init(&rect_dsc);
	rect_dsc.radius = LV_RADIUS_CIRCLE; /* Make it circular */
	rect_dsc.bg_opa = LV_OPA_COVER;
	rect_dsc.bg_color = lv_color_hex(0xFFFF00 | brightness); /* Yellow to white gradient */
	rect_dsc.border_width = 0;

	/* Much enhanced shadow for extreme fuzziness effect */
	rect_dsc.shadow_width = 30; /* Large blur radius */
	rect_dsc.shadow_color = lv_color_hex(0xFFFF00 | brightness);
	rect_dsc.shadow_opa = LV_OPA_80; /* Very opaque shadow */
	rect_dsc.shadow_ofs_x = 0;
	rect_dsc.shadow_ofs_y = 0;

	/* Define rectangular area (will be drawn as circle due to radius) */
	lv_area_t area;
	area.x1 = px - dot_size / 2;
	area.y1 = py - dot_size / 2;
	area.x2 = px + dot_size / 2;
	area.y2 = py + dot_size / 2;

	lv_draw_rect(draw_ctx, &rect_dsc, &area);
}

/* Draw callback for the grid and radar dots */
static void draw_grid_and_radar_cb(lv_event_t *e)
{
	lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);
	lv_obj_t *obj = lv_event_get_target(e);

	int scr_w = lv_obj_get_width(obj);
	int scr_h = lv_obj_get_height(obj);

	/* Draw grid lines */
	lv_draw_line_dsc_t line_dsc;
	lv_draw_line_dsc_init(&line_dsc);
	line_dsc.color = lv_color_hex(0x000000);
	line_dsc.width = 2;

	/* Vertical lines */
	for (int i = 0; i <= COLS; ++i)
	{
		lv_point_t p1 = {(lv_coord_t)((i * scr_w) / COLS), 0};
		lv_point_t p2 = {(lv_coord_t)((i * scr_w) / COLS), (lv_coord_t)(scr_h - 1)};
		lv_draw_line(draw_ctx, &line_dsc, &p1, &p2);
	}

	/* Horizontal lines */
	for (int j = 0; j <= ROWS; ++j)
	{
		lv_point_t p1 = {0, (lv_coord_t)((j * scr_h) / ROWS)};
		lv_point_t p2 = {(lv_coord_t)(scr_w - 1), (lv_coord_t)((j * scr_h) / ROWS)};
		lv_draw_line(draw_ctx, &line_dsc, &p1, &p2);
	}

	/* Draw radar dots - sort by distance so larger dots render on top */
	/* Create array of indices sorted by distance (farthest first) */
	int dot_order[NUM_TARGETS];
	for (int i = 0; i < NUM_TARGETS; ++i)
	{
		dot_order[i] = i;
	}

	/* Simple bubble sort: farthest first (larger distance value) */
	for (int i = 0; i < NUM_TARGETS - 1; ++i)
	{
		for (int j = i + 1; j < NUM_TARGETS; ++j)
		{
			if (radar_targets[dot_order[i]].distance < radar_targets[dot_order[j]].distance)
			{
				int temp = dot_order[i];
				dot_order[i] = dot_order[j];
				dot_order[j] = temp;
			}
		}
	}

	/* Render dots from farthest to closest (so close dots appear on top) */
	for (int idx = 0; idx < NUM_TARGETS; ++idx)
	{
		int i = dot_order[idx];

		/* In scan mode, only render dots up to visible_dot_count */
		if (in_scan_mode && idx >= visible_dot_count)
		{
			break;
		}

		float x = radar_targets[i].x;
		float y = radar_targets[i].y;
		float distance = radar_targets[i].distance;

		/* Rotate point by current heading */
		rotate_point(&x, &y, current_heading);

		/* Map from world coordinates (-50 to +50) to display pixels
		 * Center at screen center, scale to fit
		 * Approximately 1/5 of screen width/height is the visible range */
		float scale = (float)scr_w / (MAX_DISTANCE * 2.0f) * 0.8f;
		int px = scr_w / 2 + (int)(x * scale);
		int py = scr_h / 2 - (int)(y * scale); /* Negative because Y grows downward */

		/* Clamp to screen bounds */
		if (px < 0)
			px = 0;
		if (px >= scr_w)
			px = scr_w - 1;
		if (py < 0)
			py = 0;
		if (py >= scr_h)
			py = scr_h - 1;

		/* Draw the fuzzy yellow dot at exact pixel coordinates */
		draw_fuzzy_dot(draw_ctx, px, py, distance);
	}

	/* Draw red triangle at center */
	draw_center_triangle(draw_ctx, scr_w / 2, scr_h / 2, DOT_SIZE_MAX);
}

/* Initialize the grid screen with radar */
void grid_screen_show(void)
{
	/* Seed random number generator using accelerometer data for true entropy
	 * Accel values will be different each boot due to sensor noise and positioning */
	uint32_t seed = (uint32_t)(Accel.x * 10000) ^ (uint32_t)(Accel.y * 10000) ^ (uint32_t)(Accel.z * 10000);
	srand(seed);

	/* Initialize random radar targets */
	init_random_targets();

	/* Create a fresh screen */
	grid_screen = lv_obj_create(NULL);

	/* Set dark green background */
	lv_obj_set_style_bg_color(grid_screen, lv_color_hex(0x003300), 0);
	lv_obj_set_style_bg_opa(grid_screen, LV_OPA_COVER, 0);

	/* Attach draw callback to render grid and radar */
	lv_obj_add_event_cb(grid_screen, draw_grid_and_radar_cb, LV_EVENT_DRAW_MAIN, NULL);

	/* Make it the active screen */
	lv_scr_load(grid_screen);

	/* Mark first update time */
	last_update_time = 0;
}

/* Update heading based on gyro data */
void grid_screen_update(void)
{
	/* Get current time in milliseconds */
	uint32_t current_time = lv_tick_get();

	/* Calculate time delta in milliseconds first */
	int32_t time_delta_ms = (int32_t)(current_time - last_update_time);

	/* Turn off buzzer if it was turned on and has been on for more than 100ms */
	if (buzzer_on_time > 0 && (current_time - buzzer_on_time) > 10)
	{
		Buzzer_Off();
		buzzer_on_time = 0;
	}

	/* Update on subsequent calls (skip first call to avoid huge jumps) */
	if (last_update_time > 0 && time_delta_ms > 0 && time_delta_ms < 1000)
	{
		/* Convert milliseconds to seconds */
		float delta_time = (float)time_delta_ms / 1000.0f;

		/* Update heading from gyro Z-axis (angular velocity in degrees per second)
		 * Multiplied by 2.0 to make rotation more visible */
		current_heading += Gyro.z * delta_time * 2.0f;

		/* Normalize heading to 0-360 degrees */
		while (current_heading >= 360.0f)
			current_heading -= 360.0f;
		while (current_heading < 0.0f)
			current_heading += 360.0f;
	}

	/* Handle scan effect timing */
	if (in_scan_mode)
	{
		/* If scan just started (first call), scan_start_time may be 0, so check for that */
		if (scan_start_time == 0)
		{
			scan_start_time = current_time;
			visible_dot_count = 0; 
		}
		else
		{
			uint32_t elapsed = current_time - scan_start_time;
			/* Show one dot every 250ms */
			int new_dot_count = (elapsed / (250 + (rand() % 150))) + 1;
			if (new_dot_count > NUM_TARGETS)
			{
				new_dot_count = NUM_TARGETS;
				in_scan_mode = false; /* Scan complete */
			}

			// Do a beep when each new dot appears (optional, can be commented out if not desired)
			if (new_dot_count > visible_dot_count)
			{
				Buzzer_On();
				buzzer_on_time = current_time; /* Will be turned off in main loop after short duration */
			}

			visible_dot_count = new_dot_count;
		}
	}

	/* Invalidate screen to trigger redraw */
	lv_obj_invalidate(grid_screen);

	/* Update last update time */
	last_update_time = current_time;
}

/* Start the scan effect (used at boot and when display wakes) */
void grid_screen_start_scan(void)
{
	visible_dot_count = 0;
	scan_start_time = 0; /* Will be set on first update */
	in_scan_mode = true;
}

/* Check if scan is complete */
bool grid_screen_is_scan_complete(void)
{
	return !in_scan_mode;
}
