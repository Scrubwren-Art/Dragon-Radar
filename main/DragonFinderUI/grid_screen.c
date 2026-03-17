#include "grid_screen.h"
#include "QMI8658.h"
#include "Buzzer.h"
#include "BAT_Driver.h"
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define ROWS 10
#define COLS 10
#define NUM_TARGETS 7
#define MAX_DISTANCE 50.0f
#define DOT_SIZE_MIN 8
#define DOT_SIZE_MAX 15

/* Static variables for grid state */
static lv_obj_t *grid_screen = NULL;
static lv_obj_t *battery_label = NULL;
static lv_obj_t *battery_container = NULL;
static int smoothed_battery_pct = -1;
static float current_heading = 0.0f;
static uint32_t last_update_time = 0;
static int visible_dot_count = 0;		 /* Number of dots to show in scan mode */
static uint32_t scan_start_time = 0; /* When scan effect started */
static bool in_scan_mode = true;		 /* True during initial scan */
static uint32_t buzzer_on_time = 0;	 /* When buzzer was turned on for dot appearance */

/* Radar targets - positions will be randomized on init */
static radar_target_t radar_targets[NUM_TARGETS];

/* Zoom state for radar */
static int zoom_level = 0; /* 0=default, 1=25%, 2=50%, 3=75% */
static uint32_t zoom_beep_time = 0; /* When zoom beep was triggered */

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

	int x2 = (int)(cx - size * 0.75f); /* Bottom-left */
	int y2 = (int)(cy + height / 2);

	int x3 = (int)(cx + size * 0.75f); /* Bottom-right */
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

	/* Color: bright orange closer to center, bright yellow at edges */
	lv_color_t dot_color;
	if (normalized_dist < 0.5f)
	{
		/* Center: bright orange */
		dot_color = lv_color_hex(0xFF9000);
	}
	else
	{
		/* Edges: bright yellow (avoids blue tint) */
		dot_color = lv_color_hex(0xFFEAA00);
	}

	/* Draw circle using rect with circular radius */
	lv_draw_rect_dsc_t rect_dsc;
	lv_draw_rect_dsc_init(&rect_dsc);
	rect_dsc.radius = LV_RADIUS_CIRCLE; /* Make it circular */
	rect_dsc.bg_opa = LV_OPA_COVER;
	rect_dsc.bg_color = dot_color;
	rect_dsc.border_width = 0;

	/* Enhanced glow/shadow effect with higher opacity for brightness */
	rect_dsc.shadow_width = 40; /* Larger blur radius for more glow */
	rect_dsc.shadow_color = dot_color;
	rect_dsc.shadow_opa = LV_OPA_90; /* More opaque shadow for brighter glow */
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

/* Helper: Draw a dotted line from one point to another with animation */
static void draw_dotted_line(lv_draw_ctx_t *draw_ctx, int x1, int y1, int x2, int y2, lv_color_t color)
{
	int dx = x2 - x1;
	int dy = y2 - y1;
	float distance = sqrtf((float)(dx * dx + dy * dy));
	
	if (distance < 1.0f)
		return;
	
	/* Normalize direction vector */
	float dir_x = dx / distance;
	float dir_y = dy / distance;
	
	/* Get current time for animation (pulsing effect) */
	uint32_t current_time = lv_tick_get();
	float animation_offset = ((current_time / 250) % 14) / 14.0f * distance; /* Complete cycle every ~3500ms */
	
	/* Dotted line: 8 pixels on, 6 pixels off */
	float pos = -animation_offset;
	lv_draw_line_dsc_t line_dsc;
	lv_draw_line_dsc_init(&line_dsc);
	line_dsc.color = color;
	line_dsc.width = 2;
	
	while (pos < distance)
	{
		float seg_start = pos;
		float seg_end = pos + 8.0f;
		if (seg_end > distance)
			seg_end = distance;
		
		if (seg_start < 0.0f)
			seg_start = 0.0f;
		
		if (seg_start < seg_end)
		{
			int px1 = (int)(x1 + dir_x * seg_start);
			int py1 = (int)(y1 + dir_y * seg_start);
			int px2 = (int)(x1 + dir_x * seg_end);
			int py2 = (int)(y1 + dir_y * seg_end);
			
			lv_point_t p1 = {(lv_coord_t)px1, (lv_coord_t)py1};
			lv_point_t p2 = {(lv_coord_t)px2, (lv_coord_t)py2};
			lv_draw_line(draw_ctx, &line_dsc, &p1, &p2);
		}
		
		pos += 14.0f; /* 8 on + 6 off */
	}
}

/* Draw callback for the grid and radar dots */
static void draw_grid_and_radar_cb(lv_event_t *e)
{
	lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);
	lv_obj_t *obj = lv_event_get_target(e);

	int scr_w = lv_obj_get_width(obj);
	int scr_h = lv_obj_get_height(obj);

	/* Calculate zoom-dependent grid spacing */
	float zoom_multiplier = 1.0f + (zoom_level * 0.25f);
	float base_spacing_x = (float)scr_w / COLS;
	float base_spacing_y = (float)scr_h / ROWS;
	float zoomed_spacing_x = base_spacing_x * zoom_multiplier;
	float zoomed_spacing_y = base_spacing_y * zoom_multiplier;

	/* Calculate offset to keep grid centered when zoomed */
	float offset_x = (scr_w - (COLS * zoomed_spacing_x)) / 2.0f;
	float offset_y = (scr_h - (ROWS * zoomed_spacing_y)) / 2.0f;

	/* Draw grid lines */
	lv_draw_line_dsc_t line_dsc;
	lv_draw_line_dsc_init(&line_dsc);
	line_dsc.color = lv_color_hex(0x000000);
	line_dsc.width = 2;

	/* Vertical lines */
	for (int i = 0; i <= COLS; ++i)
	{
		float x_pos = offset_x + (i * zoomed_spacing_x);
		lv_point_t p1 = {(lv_coord_t)x_pos, 0};
		lv_point_t p2 = {(lv_coord_t)x_pos, (lv_coord_t)(scr_h - 1)};
		lv_draw_line(draw_ctx, &line_dsc, &p1, &p2);
	}

	/* Horizontal lines */
	for (int j = 0; j <= ROWS; ++j)
	{
		float y_pos = offset_y + (j * zoomed_spacing_y);
		lv_point_t p1 = {0, (lv_coord_t)y_pos};
		lv_point_t p2 = {(lv_coord_t)(scr_w - 1), (lv_coord_t)y_pos};
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

	/* Track the closest dot for drawing a line from center */
	int closest_dot_idx = -1;
	float closest_distance = FLT_MAX;
	int closest_px = 0, closest_py = 0;
	lv_color_t closest_color = lv_color_hex(0xFFDD00);

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
		float zoom_multiplier = 1.0f + (zoom_level * 0.25f); /* 1.0, 1.25, 1.5, 1.75 */
		float scale = (float)scr_w / (MAX_DISTANCE * 2.0f) * 0.8f * zoom_multiplier;
		int px = scr_w / 2 + (int)(x * scale);
		int py = scr_h / 2 - (int)(y * scale); /* Negative because Y grows downward */

		/* Track closest dot (by world-space distance to origin) */
		float world_dist_sq = radar_targets[i].x * radar_targets[i].x + radar_targets[i].y * radar_targets[i].y;
		if (world_dist_sq < closest_distance)
		{
			closest_distance = world_dist_sq;
			closest_dot_idx = i;
			closest_px = px;
			closest_py = py;
			/* Determine color based on normalized_dist from center */
			float normalized_dist = sqrtf((float)(px - scr_w/2) * (px - scr_w/2) + (float)(py - scr_h/2) * (py - scr_h/2)) / 
									 sqrtf((float)(scr_w/2) * (scr_w/2) + (float)(scr_h/2) * (scr_h/2));
			if (normalized_dist > 1.0f) normalized_dist = 1.0f;
			if (normalized_dist < 0.5f)
				closest_color = lv_color_hex(0xFFAA00);
			else
				closest_color = lv_color_hex(0xFFDD00);
		}

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

	/* Draw dotted line from center to closest dot (only after scan completes) */
	if (closest_dot_idx >= 0 && !in_scan_mode)
	{
		draw_dotted_line(draw_ctx, scr_w / 2, scr_h / 2, closest_px, closest_py, closest_color);
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

	/* Create battery container with background color and border */
	battery_container = lv_obj_create(grid_screen);
	lv_obj_set_size(battery_container, 70, 28);
	lv_obj_set_pos(battery_container, (lv_coord_t)((lv_disp_get_hor_res(NULL) - 70) / 2), 10);
	lv_obj_set_style_bg_color(battery_container, lv_color_hex(0x003300), 0);
	lv_obj_set_style_bg_opa(battery_container, LV_OPA_COVER, 0);
	lv_obj_set_style_border_color(battery_container, lv_color_hex(0x000000), 0);
	lv_obj_set_style_border_width(battery_container, 2, 0);
	lv_obj_set_style_radius(battery_container, 0, 0);
	lv_obj_set_style_outline_width(battery_container, 0, 0);
	lv_obj_set_style_shadow_width(battery_container, 0, 0);
	lv_obj_set_style_pad_all(battery_container, 0, 0);
	lv_obj_clear_flag(battery_container, LV_OBJ_FLAG_SCROLLABLE);

	/* Create battery percentage label inside container */
	battery_label = lv_label_create(battery_container);
	lv_label_set_text(battery_label, "--%");
	lv_obj_set_style_text_color(battery_label, lv_color_hex(0x000000), 0);
	lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_16, 0);
	lv_obj_center(battery_label);

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

	/* Turn off zoom beep buzzer after 150ms for a deeper sound */
	if (zoom_beep_time > 0 && (current_time - zoom_beep_time) > 150)
	{
		Buzzer_Off();
		zoom_beep_time = 0;
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
				/*Buzzer_On();
				buzzer_on_time = current_time; Will be turned off in main loop after short duration */
			}

			visible_dot_count = new_dot_count;
		}
	}

	/* Update battery percentage display */
	if (battery_label != NULL)
	{
		float voltage = BAT_Get_Volts();
		static char bat_text[12];
		
		/* Show "Charging" if voltage >= 4.15V (charging voltage) */
		if (voltage >= 4.15f)
		{
			snprintf(bat_text, sizeof(bat_text), "Charging");
			lv_obj_set_style_bg_color(battery_container, lv_color_hex(0x003300), 0);
			smoothed_battery_pct = -1;
		}
		else
		{
			float min_voltage = 3.0f;
			float max_voltage = 3.7f;
			int target_percentage = (int)((voltage - min_voltage) / (max_voltage - min_voltage) * 100.0f);
			if (target_percentage < 0) target_percentage = 0;
			if (target_percentage > 100) target_percentage = 100;

			/* Interpolate towards target percentage (smoothing) */
			if (smoothed_battery_pct < 0)
				smoothed_battery_pct = target_percentage;
			else
				smoothed_battery_pct = smoothed_battery_pct + (target_percentage - smoothed_battery_pct) / 4;

			int display_pct = smoothed_battery_pct;
			snprintf(bat_text, sizeof(bat_text), "%d%%", display_pct);

			/* Set box color based on percentage */
			lv_color_t box_color;
			if (display_pct >= 50)
				box_color = lv_color_hex(0x003300); /* Green */
			else if (display_pct >= 20)
				box_color = lv_color_hex(0xFFA500); /* Orange */
			else
				box_color = lv_color_hex(0xFF0000); /* Red */
			lv_obj_set_style_bg_color(battery_container, box_color, 0);
		}
		lv_label_set_text(battery_label, bat_text);
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

/* Increment zoom level and cycle through: 0->1->2->3->0 */
void grid_screen_zoom_increment(void)
{
	zoom_level = (zoom_level + 1) % 4;
	printf("Zoom level: %d (%.2fx)\n", zoom_level, 1.0f + (zoom_level * 0.25f));
	/* Play deep buzzer beep for zoom */
	Buzzer_On();
	zoom_beep_time = lv_tick_get();
	lv_obj_invalidate(grid_screen); /* Redraw with new zoom */
}
