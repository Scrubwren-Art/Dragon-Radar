#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "TCA9554PWR.h"
#include "PCF85063.h"
#include "QMI8658.h"
#include "ST7701S.h"
#include "SD_MMC.h"
#include "LVGL_Driver.h"
#include "LVGL_Example.h"
#include "grid_screen.h"
#include "Button_Driver.h"
#include "Wireless.h"
#include "BAT_Driver.h"

/* Display power state */
static bool display_active = true;
static uint32_t last_activity_time = 0;
static PressEvent last_button_state = NONE_PRESS;

extern PressEvent BOOT_KEY_State; /* From Button_Driver.h */

void Driver_Loop(void *parameter)
{
    while (1)
    {
        QMI8658_Loop();
        RTC_Loop();
        BAT_Get_Volts();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}
void Driver_Init(void)
{
    Flash_Searching();
    BAT_Init();
    I2C_Init();
    PCF85063_Init();
    QMI8658_Init();
    EXIO_Init(); // Example Initialize EXIO
    xTaskCreatePinnedToCore(
        Driver_Loop,
        "Other Driver task",
        4096,
        NULL,
        3,
        NULL,
        0);
}
void app_main(void)
{
    button_Init();
    // Wireless_Init();  /* Disabled: WiFi/BLE not needed */
    Driver_Init();
    LCD_Init();
    // SD_Init();  /* Disabled: SD card not needed */
    LVGL_Init();
    /********************* Demo *********************/
    grid_screen_show();
    /* Start radar scan effect at boot */
    grid_screen_start_scan();
    grid_screen_update(); /* Initialize scan state */
    last_activity_time = lv_tick_get();

    // lv_demo_widgets();
    // lv_demo_keypad_encoder();
    // lv_demo_benchmark();
    // lv_demo_stress();
    // lv_demo_music();

    // Simulated_Touch_Init();  /* Disabled: touch simulation not needed */
    while (1)
    {
        uint32_t current_time = lv_tick_get();

        /* Handle display timeout (30 seconds of inactivity) */
        if (display_active)
        {
            uint32_t idle_time = current_time - last_activity_time;
            if (idle_time > 30000)
            {                     /* 30 seconds */
                Set_Backlight(0); /* Turn off backlight */
                display_active = false;
                printf("Display OFF\n");
            }
        }

        /* Handle button events */
        if (BOOT_KEY_State != last_button_state && BOOT_KEY_State != NONE_PRESS)
        {
            if (BOOT_KEY_State == LONG_PRESS_START)
            {
                printf("Long press detected\n");
                if (!display_active)
                {
                    /* Wake display and restart scan */
                    display_active = true;
                    Set_Backlight(100);       /* Turn on full brightness */
                    grid_screen_start_scan(); /* Restart scan effect */
                    grid_screen_update();     /* Initialize scan state */
                    last_activity_time = current_time;
                    printf("Display ON - restarted scan\n");
                }
                else
                {
                    Set_Backlight(0); /* Turn off backlight */
                    display_active = false;
                    printf("Display OFF\n");
                }
            }
            else if (BOOT_KEY_State == SINGLE_CLICK)
            {
                printf("Single click detected - zooming radar\n");
                if (display_active)
                {
                    grid_screen_zoom_increment();
                }
            }
        }
        last_button_state = BOOT_KEY_State;

        /* Update radar if display is active or dissolving */
        if (display_active)
        {
            // Update radar heading from gyro rotation
            grid_screen_update();
        }

        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
    }
}
