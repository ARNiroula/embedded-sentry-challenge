// StdLib
#include <cstdint>
#include <cstring>
#include <limits>

// Mbed and STM32 Drivers
#include "mbed.h"
#include "drivers/stm32f429i_discovery_lcd.h"
#include "drivers/LCD_DISCO_F429ZI.h"
#include "stm32f4xx.h"

// Local Files
#include "macro.hpp"
#include "dtw_distance.hpp"
#include "utils.hpp"

// Gyroscope registers
#define L3GD20_CTRL_REG1        0x20
#define L3GD20_CTRL_REG4        0x23
#define L3GD20_OUT_X_L          0x28
#define L3GD20_OUT_X_H          0x29
#define L3GD20_OUT_Y_L          0x2A
#define L3GD20_OUT_Y_H          0x2B
#define L3GD20_OUT_Z_L          0x2C
#define L3GD20_OUT_Z_H          0x2D

// Gyroscope scaling factor
#define GYRO_SCALE_FACTOR       (17.5f * 0.017453292519943295769236907684886f / 1000.0f)

// Recording duration in ms
#define RECORDING_DURATION      2000

// Tolerance for gesture comparison
#define GESTURE_TOLERANCE    80

// Press duration for recording (in ms)
#define LONG_PRESS_DURATION     2000

// Samples per second and total samples
#define SAMPLES_PER_SECOND 50
#define NUM_SAMPLES (RECORDING_DURATION / 1000 * SAMPLES_PER_SECOND)

// --- Register Addresses and Configuration Values ---
#define CTRL_REG1 0x20               // Control register 1 address
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1 //0110 1111  // Configuration: ODR=100Hz, Enable X/Y/Z axes, power on
#define CTRL_REG4 0x23               // Control register 4 address
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0  // Configuration: High-resolution, 2000dps sensitivity

// SPI communication completion flag
#define SPI_FLAG 1

// Address of the gyroscope's X-axis output lower byte register
#define OUT_X_L 0x28

// Scaling factor for converting raw sensor data in dps (deg/s) to angular velocity in rps (rad/s)
// Combines sensitivity scaling and conversion from degrees to radians
#define DEG_TO_RAD (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)

// EventFlags object to synchronize asynchronous SPI transfers
EventFlags flags;

// --- SPI Transfer Callback Function ---
// Called automatically when an SPI transfer completes
void spi_cb(int event) {
    flags.set(SPI_FLAG);  // Set the SPI_FLAG to signal that transfer is complete
}

using namespace std::chrono;
using namespace std;
Timer t;

// LCD
LCD_DISCO_F429ZI lcd;

void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

void draw_snowman() {
    uint16_t screen_width = lcd.GetXSize(); // Use GetXSize for width
    uint16_t screen_height = lcd.GetYSize(); // Use GetYSize for height
    uint16_t center_x = screen_width / 2;
    for (int i = 220; i > 0; i--) {
        uint16_t base_y = screen_height - i;

        lcd.Clear(LCD_COLOR_RED);
        // Draw the body (three circles: bottom, middle, and head)
        lcd.SetTextColor(LCD_COLOR_WHITE);
        lcd.FillCircle(center_x, base_y - 30, 15); // Bottom circle
        lcd.FillCircle(center_x, base_y - 50, 10); // Middle circle
        lcd.FillCircle(center_x, base_y - 65, 7);  // Head

        // Draw the eyes (small black dots on the head)
        lcd.SetTextColor(LCD_COLOR_BLACK);
        lcd.FillCircle(center_x - 3, base_y - 67, 1); // Left eye
        lcd.FillCircle(center_x + 3, base_y - 67, 1); // Right eye

        // Draw the fixed, mini nose (small orange rectangle)
        lcd.SetTextColor(LCD_COLOR_ORANGE);
        lcd.FillRect(center_x - 1, base_y - 65, 3, 2); // Small rectangle for nose

        // Draw the mouth (small black dots)
        lcd.FillCircle(center_x - 4, base_y - 61, 1); // Left dot
        lcd.FillCircle(center_x, base_y - 60, 1);     // Center dot
        lcd.FillCircle(center_x + 4, base_y - 61, 1); // Right dot

        // Draw the arms (two lines)
        lcd.SetTextColor(LCD_COLOR_BROWN);
        if (i % 2 == 0) {
            lcd.DrawLine(center_x - 15, base_y - 50, center_x - 25, base_y - 40); // Left arm
            lcd.DrawLine(center_x + 15, base_y - 50, center_x + 25, base_y - 40); // Right arm
        } else {
            lcd.DrawLine(center_x - 15, base_y - 50, center_x - 30, base_y - 50); // Left arm
            lcd.DrawLine(center_x + 15, base_y - 50, center_x + 30, base_y - 50); // Right arm
        }

        // Draw the hat (black rectangle)
        lcd.SetTextColor(LCD_COLOR_BLACK);
        lcd.FillRect(center_x - 7, base_y - 75, 14, 3); // Hat brim
        lcd.FillRect(center_x - 5, base_y - 85, 10, 10); // Hat top
        delay_ms(50);
    }
    lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"HO HO HO ", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"MERRY CHRISTMAS!!!", CENTER_MODE);
}

// BUTTON for recording and entering key
DigitalIn button(PA_0);

// LED for indicating success
DigitalOut led(LED1);
I2C_HandleTypeDef hi2c;

// Variables to store key gestures
float key_vals[MAX_ARRAY_2D_SIZE][3];
float gyro_vals[MAX_ARRAY_2D_SIZE][3];

bool key_recorded = false;

void read_gyro(
    int row,
    uint8_t write_buf[32],
    uint8_t read_buf[32],
    SPI *spi,
    EventFlags *flags,
    float arr[MAX_ARRAY_2D_SIZE][3]) {
    uint16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;
    // Function prototypes
    // Prepare to read gyroscope output starting at OUT_X_L
    // - write_buf[0]: register address with read (0x80) and auto-increment (0x40) bits set
    write_buf[0] = OUT_X_L | 0x80 | 0x40; // Read mode + auto-increment

    // Perform SPI transfer to read 6 bytes (X, Y, Z axis data)
    // - write_buf[1:6] contains dummy data for clocking
    // - read_buf[1:6] will store received data
    spi->transfer(write_buf, 7, read_buf, 7, spi_cb);
    flags->wait_all(SPI_FLAG);  // Wait until the transfer completes

    // --- Extract and Convert Raw Data ---
    // Combine high and low bytes
    raw_gx = (((uint16_t)read_buf[2]) << 8) | read_buf[1];
    raw_gy = (((uint16_t)read_buf[4]) << 8) | read_buf[3];
    raw_gz = (((uint16_t)read_buf[6]) << 8) | read_buf[5];

    gx = raw_gx * DEG_TO_RAD;
    gy = raw_gy * DEG_TO_RAD;
    gz = raw_gz * DEG_TO_RAD;

    arr[row][0] = gx;
    arr[row][1] = gy;
    arr[row][2] = gz;
    delay_ms(1000 / SAMPLES_PER_SECOND);
}

// Main program
int main() {
    // --- SPI Initialization ---
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

    // Buffers for SPI data transfer:
    // - write_buf: stores data to send to the gyroscope
    // - read_buf: stores data received from the gyroscope
    uint8_t write_buf[32], read_buf[32];

    // Configure SPI interface:
    // - 8-bit data size
    // - Mode 3 (CPOL = 1, CPHA = 1): idle clock high, data sampled on falling edge
    spi.format(8, 3);

    // Set SPI communication frequency to 1 MHz
    spi.frequency(1'000'000);

    // --- Gyroscope Initialization ---
    // Configure Control Register 1 (CTRL_REG1)
    // - write_buf[0]: address of the register to write (CTRL_REG1)
    // - write_buf[1]: configuration value to enable gyroscope and axes
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);  // Initiate SPI transfer
    flags.wait_all(SPI_FLAG);  // Wait until the transfer completes

    // Configure Control Register 4 (CTRL_REG4)
    // - write_buf[0]: address of the register to write (CTRL_REG4)
    // - write_buf[1]: configuration value to set sensitivity and high-resolution mode
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);  // Initiate SPI transfer
    flags.wait_all(SPI_FLAG);  // Wait until the transfer completes

    // Initialize the gyroscope and LCD
    lcd.DisplayOn();
    lcd.Clear(LCD_COLOR_WHITE);

    // Initialize Key and Entered Gesture value
    memset(key_vals, 0, sizeof key_vals);
    memset(gyro_vals, 0, sizeof gyro_vals);
    // Main loop
    // lcd.Clear(LCD_COLOR_WHITE);
    while (1) {

        if (button.read() != 1) {
            continue;
        }
        float dtw_distance = std::numeric_limits<float>::infinity();
        // Check button press
        uint32_t press_time = HAL_GetTick();

        // Wait for button release
        while (button.read() == 1) {}

        // Determine if it was a short press or long press
        if ((HAL_GetTick() - press_time) >= LONG_PRESS_DURATION) {
            // Long press - Record key gesture
            memset(key_vals, 0, sizeof key_vals);
            lcd.Clear(LCD_COLOR_BLUE);
            lcd.DisplayStringAt(
                0,
                LINE(7),
                (uint8_t *) "RECORDING!!",
                CENTER_MODE
            );
            delay_ms(1000);  // Give user a chance to get ready

            for (int i = 0; i < MAX_ARRAY_2D_SIZE; i++) {
                read_gyro(
                    i,
                    write_buf,
                    read_buf,
                    &spi,
                    &flags,
                    key_vals
                    );
                delay_ms(1000 / SAMPLES_PER_SECOND);
            }
            standard_scaler(key_vals);

            key_recorded = true;

            lcd.Clear(LCD_COLOR_WHITE);
            lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"RECORDING COMPLETE", CENTER_MODE);
            delay_ms(1000);
        } else {
            // Short press - Enter key
            if (!key_recorded) {
                lcd.Clear(LCD_COLOR_RED);
                lcd.DisplayStringAt(
                    0,
                    LINE(7), 
                (uint8_t *) "NO KEY RECORDED!!",
                CENTER_MODE
                );
                delay_ms(1000);
                continue;
            }

            lcd.Clear(LCD_COLOR_BLUE);
            lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"PSST: ", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"ENTER GESTURE", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"FOR SECRET MESSAGE", CENTER_MODE);
            delay_ms(2000);  // Give user a chance to get ready
            memset(gyro_vals, 0, sizeof gyro_vals);
            for (int i = 0; i < MAX_ARRAY_2D_SIZE; i++) {
                read_gyro(
                    i,
                    write_buf,
                    read_buf,
                    &spi,
                    &flags,
                    gyro_vals 
                    );
                delay_ms(1000 / SAMPLES_PER_SECOND);
            }

            standard_scaler(gyro_vals);
            // Compare entered key with recorded key
            bool success = false;

            dtw_distance = dtw_distance_only(
                key_vals,
                MAX_ARRAY_2D_SIZE,
                3,
                gyro_vals,
                MAX_ARRAY_2D_SIZE,
                3,
                2);

            lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"PROCESSING....", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"PLEASE WAIT", CENTER_MODE);

            printf("Similarity Score => %f\n\n", dtw_distance);
            printf("**********************");
            if (dtw_distance <= GESTURE_TOLERANCE) {
                    success = true;
            }

            if (success) {

                // Successful unlock
                printf("SUCCESSSSS!!!!!!!!!!!!!!!!!!!!\n\n\n\n");
                draw_snowman();
                // lcd.Clear(LCD_COLOR_GREEN);
                led = 1;  // Turn on LED
                delay_ms(2000);  // Display success for 2 seconds
                led = 0;  // Turn off LED
            } else {
                // Unsuccessful unlock
                printf("FAILUREE!!!!!!!!!!!!!!!!!!!!\n\n\n\n");
                lcd.Clear(LCD_COLOR_RED);
                lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"UNLOCK FAILED", CENTER_MODE);
                lcd.DisplayStringAt(0, LINE(8), (uint8_t *)":-(", CENTER_MODE);
                delay_ms(2000);  // Display failure for 2 seconds
            }
        }
    }
}