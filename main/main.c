#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
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
    Wireless_Init();  /* Enable BLE for beacon tracking */
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

        /* Check for wake-up from sleep (only check once immediately after wake) */
        static bool wakeup_pending = false;
        static bool was_sleeping = false;
        
        if (was_sleeping) {
            /* We're waking up - get the wake-up cause immediately */
            esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
            if (wakeup_cause == ESP_SLEEP_WAKEUP_GPIO) {
                wakeup_pending = true;
            }
            was_sleeping = false;
        }

        /* Handle wake-up if pending */
        if (wakeup_pending) {
            wakeup_pending = false;
            display_active = true;
            Set_Backlight(100);
            grid_screen_start_scan();
            grid_screen_update();
            BLE_Enable();
            last_activity_time = lv_tick_get();
        }

        /* Handle display timeout (30 seconds of inactivity) - enter sleep mode */
        if (display_active && !was_sleeping)
        {
            uint32_t idle_time = current_time - last_activity_time;
            if (idle_time > 120000)
            {                     /* 120 seconds */
                /* Enter light sleep mode to save power */
                Set_Backlight(0);      /* Turn off backlight */
                BLE_Disable();         /* Disable BLE to save power */
                
                /* Configure GPIO wake-up on button press (GPIO 0 = low) */
                gpio_wakeup_enable(BOOT_KEY_PIN, GPIO_INTR_LOW_LEVEL);
                esp_sleep_enable_gpio_wakeup();
                
                display_active = false;
                was_sleeping = true;
                
                /* Enter light sleep - will wake on GPIO 0 low (button press) */
                esp_light_sleep_start();
                
                /* After wake-up: loop continues, was_sleeping will be checked next iteration */
            }
        }

        /* Handle button events */
        if (BOOT_KEY_State != last_button_state && BOOT_KEY_State != NONE_PRESS)
        {
            if (BOOT_KEY_State == LONG_PRESS_START)
            {
                if (!display_active && !was_sleeping)
                {
                    /* Turn on display */
                    display_active = true;
                    Set_Backlight(100);       /* Turn on full brightness */
                    grid_screen_start_scan();   /* Restart scan effect */
                    grid_screen_update();      /* Initialize scan state */
                    BLE_Enable();             /* Re-enable BLE */
                    last_activity_time = lv_tick_get();
                }
                else if (display_active)
                {
                    /* Turn off display - enter sleep mode */
                    Set_Backlight(0);
                    display_active = false;
                    BLE_Disable();
                    was_sleeping = true;
                    
                    /* Configure GPIO wake-up */
                    gpio_wakeup_enable(BOOT_KEY_PIN, GPIO_INTR_LOW_LEVEL);
                    esp_sleep_enable_gpio_wakeup();
                    
                    /* Enter light sleep */
                    esp_light_sleep_start();
                }
            }
            else if (BOOT_KEY_State == SINGLE_CLICK)
            {
                if (display_active)
                {
                    grid_screen_zoom_increment();
                }
            }
        }
        last_button_state = BOOT_KEY_State;

        /* Update radar if display is active */
        if (display_active)
        {
            grid_screen_update();
        }

        /* Update LVGL */
        if (display_active)
        {
            lv_timer_handler();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
