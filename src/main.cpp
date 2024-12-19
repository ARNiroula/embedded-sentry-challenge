#include <cstdint>
#include <cstring>
#include <limits>

#include "mbed.h"
#include "drivers/LCD_DISCO_F429ZI.h"
#include "dtw_distance.hpp"
#include "stm32f4xx.h"
#include "dtw_distance.hpp"

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
#define GESTURE_TOLERANCE      400

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

// BUTTON for recording and entering key
DigitalIn button(PA_0);

// LED for indicating success
DigitalOut led(LED1);
I2C_HandleTypeDef hi2c;

// Variables to store key gestures
float key_vals[MAX_ARRY_2D_SIZE][3];
float gyro_vals[MAX_ARRY_2D_SIZE][3];

bool key_recorded = false;

void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

void read_gyro(
    int row,
    uint8_t write_buf[32],
    uint8_t read_buf[32],
    SPI *spi,
    EventFlags *flags,
    float arr[MAX_ARRY_2D_SIZE][3]) {
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
    arr[row][1] = gz;
    delay_ms(1000 / SAMPLES_PER_SECOND);
}

// Main program
int main() {
    // --- SPI Initialization ---
    // SPI(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel)
    // PF_9 = MOSI, PF_8 = MISO, PF_7 = SCLK, PC_1 = Chip Select (CS)
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

    // --- Continuous Gyroscope Data Reading ---
    // bool has_entered = false;
    // float gx, gy, gz;  // Variables to store converted angular velocity values


    // Initialize the gyroscope and LCD
    memset(key_vals, 0, sizeof key_vals);
    memset(gyro_vals, 0, sizeof gyro_vals);

    lcd.DisplayOn();
    lcd.Clear(LCD_COLOR_WHITE);

    // Initialize hi2c
    hi2c.Instance = I2C1;
    hi2c.Init.ClockSpeed = 400000;
    hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c.Init.OwnAddress1 = 0;
    hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c.Init.OwnAddress2 = 0;
    hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    // Main loop
    // uint8_t data[6];    
    while (1) {
        float dtw_distance = std::numeric_limits<float>::infinity();
        // uint16_t raw_gx, raw_gy, raw_gz;   // Variables to store raw data

        if (button.read() != 1) {
            continue;
        }
        // Check button press
        uint32_t press_time = HAL_GetTick();

        // Wait for button release
        while (button.read() == 1) {}

        // Determine if it was a short press or long press
        if ((HAL_GetTick() - press_time) >= LONG_PRESS_DURATION) {
            // Long press - Record key gesture
            lcd.Clear(LCD_COLOR_BLUE);
            lcd.DisplayStringAt(
                0, LINE(4), (uint8_t *)"RECORDING!!", CENTER_MODE
            );
            delay_ms(1000);  // Give user a chance to get ready

            for (int i = 0; i < MAX_ARRY_2D_SIZE; i++) {
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

            key_recorded = true;

            lcd.Clear(LCD_COLOR_WHITE);
            lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"RECORDING COMPLETE", CENTER_MODE);
            delay_ms(1000);
        } else {
            // Short press - Enter key
            if (!key_recorded) {
                lcd.Clear(LCD_COLOR_RED);
                lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"NO KEY RECORDED", CENTER_MODE);
                delay_ms(1000);
                continue;
            }

            lcd.Clear(LCD_COLOR_BLUE);
            lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"ENTER KEY", CENTER_MODE);
            delay_ms(1000);  // Give user a chance to get ready

            for (int i = 0; i < MAX_ARRY_2D_SIZE; i++) {
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

            // Compare entered key with recorded key
            bool success = false;
            // for (int i = 0; i < MAX_ARRY_2D_SIZE; i++) {
            //     printf("%f\t%f\t%f\n", key_vals[i][0], key_vals[i][1], key_vals[i][2]);
            // }
            // printf("**********************\n");
            // for (int i = 0; i < MAX_ARRY_2D_SIZE; i++) {
            //     printf("%f\t%f\t%f\n", gyro_vals[i][0], gyro_vals[i][1],gyro_vals[i][2]);
            // }
            dtw_distance = dtw_distance_only(
                key_vals,
                MAX_ARRY_2D_SIZE,
                3,
                gyro_vals,
                MAX_ARRY_2D_SIZE,
                3,
                2);
            printf("%f\n\n", dtw_distance);
            printf("**********************");
            if (dtw_distance <= GESTURE_TOLERANCE) {
                    success = true;
            }

            if (success) {

                // Successful unlock
                printf("SUCCESSSSS!!!!!!!!!!!!!!!!!!!!\n\n\n\n");
                lcd.Clear(LCD_COLOR_GREEN);
                lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"UNLOCK SUCCESSFUL", CENTER_MODE);
                led = 1;  // Turn on LED
                delay_ms(2000);  // Display success for 2 seconds
                led = 0;  // Turn off LED
            } else {
                // Unsuccessful unlock
                printf("FAILUREE!!!!!!!!!!!!!!!!!!!!\n\n\n\n");
                lcd.Clear(LCD_COLOR_RED);
                lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"UNLOCK FAILED", CENTER_MODE);
                delay_ms(2000);  // Display failure for 2 seconds
            }
        }
    }
}