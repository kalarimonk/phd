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

// I²S configuration
#define I2S_NUM         I2S_NUM_0 // I²S port number (0 or 1)
#define I2S_SCK_PIN     26        // Serial clock pin (SCK)
#define I2S_WS_PIN      32        // Word select pin (LRCLK/WS)
#define I2S_SD_PIN      33        // Serial data pin (SD)

// Ball dimensions
#define RADIUS 4

// Audio "physics"
#define BLOW_FORCE 4
#define BLOW_THRESHOLD 65

// Free fall "physics"  
#define GRAVITY 9.8  // Simulated gravity value
#define FALL_SPEED 0.85  // Speed of the free fall (controls how fast time progresses)
#define HORIZONTAL_SPEED 2  // Speed at which the ball moves horizontally
#define ANIMATION_DELAY 10 

// Sample rate and buffer configuration
#define SAMPLE_RATE     16000     // Sample rate in Hz (16kHz)
#define I2S_BUFF_LEN    256      // Size of I²S buffer

volatile float rms_signal_intensity = 0;

static SH1107_t dev; 

void setup_microphone() {
  const i2s_config_t i2s_config = {
    .mode = I2S_MODE_MASTER | I2S_MODE_RX,
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = I2S_BUFF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK_PIN,    // IIS_SCLK
    .ws_io_num = I2S_WS_PIN,     // IIS_LCLK
    .data_out_num = I2S_PIN_NO_CHANGE,  // IIS_DSIN
    .data_in_num = I2S_SD_PIN,   // IIS_DOUT
  };

  ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pin_config));
  ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_NUM_0));

}

void setup_oled(void) {
    i2c_master_init(&dev, I2C_SDA_GPIO, I2C_SCL_GPIO, -1);
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
    float deltaX = (float)(targetX - *currentX) / (SCREEN_WIDTH / HORIZONTAL_SPEED);

    while (*currentX != targetX) {
        // Clear the ball's previous position
        clear_ball(dev, *currentX, currentY, radius);

        printf("\nRMS Value: %.2f, Threshold: %d", rms_signal_intensity, BLOW_THRESHOLD);

        if(rms_signal_intensity > BLOW_THRESHOLD){
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

        // Update horizontal position towards the target
        if (*currentX != targetX) {
            *currentX += (deltaX > 0 ? HORIZONTAL_SPEED : -HORIZONTAL_SPEED);
            if (abs(*currentX - targetX) < HORIZONTAL_SPEED) *currentX = targetX;
        }

        // Draw the ball at the new position
        draw_ball(dev, *currentX, currentY, radius);
        sh1107_show_buffer(dev);

        // Delay for the animation
        vTaskDelay(pdMS_TO_TICKS(ANIMATION_DELAY));
    }
}

// Function to map raw data to a range (e.g., 0-128)
int32_t mapRawData(int16_t sample) {
    int16_t mappedValue = (sample + 32768) * 128 / 65534; // Normalize to range 0-128
    return mappedValue;
}

float calculate_rms(int16_t *buffer, size_t sample_count) {
    int32_t sum_of_squares = 0;

    // Sum the squares of each sample
    for (size_t i = 0; i < sample_count; i++) {
        int32_t sample = buffer[i] >> 8; // Extract 24-bit sample
        sum_of_squares += (int32_t)sample * (int32_t)sample;
    }

    // Calculate the mean of squares
    float mean_of_squares = (float)sum_of_squares / sample_count;

    // Return the square root of the mean of squares (RMS value)
    return sqrtf(mean_of_squares);
}

// Task for reading I²S data
void i2s_read_task(void *param) {
    // Buffer to store the raw I²S data
    int16_t i2sBuffer[I2S_BUFF_LEN];
    size_t bytesRead;
    size_t samples_read;

    while (1) {
        // Read data from the I²S input
        i2s_read(I2S_NUM_0, i2sBuffer, I2S_BUFF_LEN*sizeof(int16_t), &bytesRead, portMAX_DELAY);
        samples_read = bytesRead/sizeof(int16_t);

        float rms_value = calculate_rms(i2sBuffer, samples_read);
        rms_signal_intensity = rms_value;
        // printf("\nRMS Value: %.2f", rms_value);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void oled_update_task(void *param) {
    
    
    // Infinite loop to update the OLED display based on the signal intensity
    while (1) {
        int centerX = 4;
        int centerY = 32;

        sh1107_clear_screen(&dev, false);
        free_fall_ball(&dev, &centerX, centerY, 128, 4);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {

    // Setup the peripherals
    setup_oled();
    setup_microphone();

    xTaskCreatePinnedToCore(&i2s_read_task,"i2s_read_task",4096,NULL,1,NULL,0);
    xTaskCreatePinnedToCore(&oled_update_task,"oled_update_task",4096,NULL,1,NULL,1);
}