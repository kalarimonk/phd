#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sh1107.h"
#include "driver/i2s.h"

#define TAG "SH1107"

//I2C Pins
#define I2C_SDA_GPIO 21
#define I2C_SCL_GPIO 22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// I2S Pins
#define I2S_WS       23  // Word Select (LRCL)
#define I2S_SCK      18  // Bit Clock (BCLK)
#define I2S_SD       19  // Data Line (DOUT)

// Ball dimensions
#define RADIUS 4

// Audio "physics"
#define BLOW_FORCE 4
#define BLOW_THRESHOLD 200

// Free fall "physics"  
#define GRAVITY 9.8  // Simulated gravity value
#define FALL_SPEED 0.85  // Speed of the free fall (controls how fast time progresses)
#define HORIZONTAL_SPEED 2  // Speed at which the ball moves horizontally
#define ANIMATION_DELAY 10 

static SH1107_t dev; 

void setup_microphone() {
  const i2s_config_t i2s_config = {
    .mode = I2S_MODE_MASTER | I2S_MODE_RX,
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = 26,    // IIS_SCLK
    .ws_io_num = 32,     // IIS_LCLK
    .data_out_num = -1,  // IIS_DSIN
    .data_in_num = 33,   // IIS_DOUT
  };

  ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pin_config));
  ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_NUM_0));

}

void setup_oled(void) {
    i2c_master_init(&dev, I2C_SDA_GPIO, I2C_SCL_GPIO, 0);
    sh1107_init(&dev, 64, 128);
	sh1107_contrast(&dev, 0xff);
    sh1107_clear_screen(&dev, false);
	sh1107_direction(&dev, DIRECTION0);
}

void sh1107_draw_pixel(SH1107_t* dev, int x, int y)
{
    if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) {
        // Pixel is out of bounds, do nothing
        return;
    }
    int page_num = x / 8 ;
    int bit_position = x % 8;
    dev->_page[page_num]._segs[y] |= 1 << bit_position;  
}

void sh1107_clear_pixel(SH1107_t* dev, int x, int y)
{
    if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) {
        // Pixel is out of bounds, do nothing
        return;
    }
    int page_num = x / 8 ;
    int bit_position = x % 8;
    dev->_page[page_num]._segs[y] &= ~(1 << bit_position);  
}

void draw_ball(SH1107_t* dev, int centerX, int centerY, int radius) {
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            // Check if the point (x, y) lies inside the circle equation (x^2 + y^2 <= r^2)
            if ((x * x + y * y) <= (radius * radius)) {
                sh1107_draw_pixel(dev, centerX + x, centerY + y);
            }
        }
    }
}

void clear_ball(SH1107_t* dev, int centerX, int centerY, int radius) {
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            // Check if the point (x, y) lies inside the circle equation (x^2 + y^2 <= r^2)
            if ((x * x + y * y) <= (radius * radius)) {
                sh1107_clear_pixel(dev, centerX + x, centerY + y);
            }
        }
    }
}

void move_ball(SH1107_t* dev, int *centerX, int centerY, int radius, char command){
    clear_ball(dev, *centerX, centerY, radius);
    sh1107_show_buffer(dev);

    if(command == 'L'){
        *centerX = *centerX + (2*radius);
    } else {
        *centerX = *centerX - (2*radius);
    }
    draw_ball(dev, *centerX, centerY, radius); 
    sh1107_show_buffer(dev);
}

void free_fall_ball(SH1107_t* dev, int* currentX, int currentY, int targetX, int radius) {
    // Initialize time and initial position
    float time = 0;
    float deltaX = (float)(targetX - *currentX) / (SCREEN_WIDTH / HORIZONTAL_SPEED);

    while (*currentX != targetX) {
        // Clear the ball's previous position
        clear_ball(dev, *currentX, currentY, radius);

        // Update horizontal position towards the target
        if (*currentX != targetX) {
            *currentX += (deltaX > 0 ? HORIZONTAL_SPEED : -HORIZONTAL_SPEED);
            if (abs(*currentX - targetX) < HORIZONTAL_SPEED) *currentX = targetX;
        }

        // Draw the ball at the new position
        draw_ball(dev, *currentX, currentY, radius);
        sh1107_show_buffer(dev);

        // Update the time step
        time += FALL_SPEED;

        // Delay for the animation
        vTaskDelay(pdMS_TO_TICKS(ANIMATION_DELAY));
    }
}

// A little buggy from down here

uint8_t map (int16_t x) {
    return (uint8_t)((x + 32768) * 128 / 65535);
}

int check_mic_input() {
    int16_t i2s_data[64];  // Buffer to store I2S data
    size_t bytes_read;
    int mic_amplitude = 0;

    // Read the microphone input (pseudocode)
    // if (i2s_read(I2S_NUM_0, i2s_data, 64*sizeof(int16_t), &r_bytes, 1000) == ESP_OK)
    i2s_read(I2S_NUM_0, (void*)i2s_data, sizeof(i2s_data), &bytes_read, 1000);

    // Process the I2S input to get amplitude (blow detection)
    for (int i = 0; i < 64; i++) {
        mic_amplitude += abs(i2s_data[i]);  // Sum of absolute values to get signal intensity
    }

    printf("Blow detected: Amplitude = %d\n", mic_amplitude);
    // Return true if blowing is detected based on the amplitude threshold
    return mic_amplitude > BLOW_THRESHOLD;
}

// Main function to simulate the free fall with resistance from blowing
void simulate_ball(SH1107_t* dev, int* currentX, int currentY, int targetX, int radius) {
    // Initialize time, and current position
    float time = 0;

    // Start the ball at the top of the screen
    *currentX = 0;

    while (*currentX != targetX) {
        // Clear the ball's previous position
        clear_ball(dev, *currentX, currentY, radius);

        // Check if user is blowing
        bool is_blowing = check_mic_input();
        // int blow_amp = check_mic_input();

        // Update vertical position based on blowing status
        if (is_blowing) {
            // Apply upward force if blowing is detected
            *currentX -= BLOW_FORCE;
            if (*currentX < 0) *currentX = 0;  // Make sure the ball doesn't go above the screen
        } else {
            // Continue free fall if no blowing is detected
            free_fall_ball(dev, currentX, currentY, targetX, RADIUS);
        }

        // Draw the ball at the new position
        draw_ball(dev, *currentX, currentY, RADIUS);
        sh1107_show_buffer(dev);

        // Update the time step (only increment time if not blowing)
        if (!is_blowing) {
            time += FALL_SPEED;
        }

        // Delay for the animation
        vTaskDelay(pdMS_TO_TICKS(ANIMATION_DELAY));
    }
}

void app_main(void) {

    // Setup the peripherals
    setup_oled();
    setup_microphone();


    int centerX = 4;
    int centerY = 32;
    
    draw_ball(&dev, centerX, centerY, RADIUS);
    sh1107_show_buffer(&dev);

    simulate_ball(&dev, &centerX, centerY, SCREEN_WIDTH, RADIUS);

    // draw_ball(&dev, centerX, centerY, radius);
    // sh1107_show_buffer(&dev);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);

    // move_ball(&dev, &centerX, centerY, radius, 'L');
    // vTaskDelay(5000 / portTICK_PERIOD_MS);  
    
    free_fall_ball(&dev, &centerX, centerY, 128, RADIUS);
    
    // move_ball(&dev, &centerX, centerY, radius, 'L');
    // vTaskDelay(5000 / portTICK_PERIOD_MS);     

    // esp_restart();
}