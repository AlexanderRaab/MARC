
/*  Includes the setup and Raspberry Pico specific methods.
    Author:     Alexander Raab
    E-Mail:     alex.raab@liwest.at
    Date:       17.06.2023
    Version:    1.1
    Licence:    TODO

    This header is used to setup and interface the Raspberry Pico, specifically IO ports and communication.
*/

#pragma once

#include "driver_config.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include "pico/binary_info.h"
#include "quadrature_encoder.pio.h"
#include "pico_uart_transports.h"
#include "pico/mutex.h"

// PIO
PIO pio_enc_L;
PIO pio_enc_R;

// Range sensor time buffer
u_int32_t range_sensor_time[8] = {0};               // Used to store the current micro seconds for the range measurement

// Mutexes for multicore functionality
struct mutex mtx_cmd_vel;
struct mutex mtx_odom_data;

void i2c_write_to_device(i2c_inst_t *i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *src, size_t len);
void i2c_read_from_device(i2c_inst_t *i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *tar, size_t len);
void range_sensor_callback(uint gpio, uint32_t events);

/* Initializes a gpio pin and sets it to input or output

    @param pin PIN number
    @param dir GPIO_IN or GPIO_OUT
*/ 
void setup_gpio_pin(uint pin, bool dir)
{
    gpio_init(pin);
    gpio_set_dir(pin, dir);
}

/* Used to initializes all GPIO pins.

*/
void setup_gpio()
{
    // General
    setup_gpio_pin(LED_BUILTIN, GPIO_OUT);
    setup_gpio_pin(PIN_MOT_ON, GPIO_OUT);

    // Motors & encoders
    setup_gpio_pin(PIN_MOT_L_FOR, GPIO_OUT);
    setup_gpio_pin(PIN_MOT_L_BAC, GPIO_OUT);
    gpio_set_function(PIN_MOT_L_PWM, GPIO_FUNC_PWM);

    setup_gpio_pin(PIN_MOT_R_FOR, GPIO_OUT);
    setup_gpio_pin(PIN_MOT_R_BAC, GPIO_OUT);
    gpio_set_function(PIN_MOT_R_PWM, GPIO_FUNC_PWM);

    // I2C
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C0_SCL);
    gpio_pull_up(PIN_I2C0_SDA);
    bi_decl(bi_2pins_with_func(PIN_I2C0_SDA, PIN_I2C0_SCL, GPIO_FUNC_I2C));

    // Range sensors
    setup_gpio_pin(PIN_TRIG_1, GPIO_OUT);
    setup_gpio_pin(PIN_ECHO_1, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_ECHO_1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &range_sensor_callback);
    setup_gpio_pin(PIN_ECHO_2, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_ECHO_2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &range_sensor_callback);
    setup_gpio_pin(PIN_ECHO_3, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_ECHO_3, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &range_sensor_callback);
    setup_gpio_pin(PIN_ECHO_4, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_ECHO_4, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &range_sensor_callback);
    setup_gpio_pin(PIN_ECHO_5, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_ECHO_5, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &range_sensor_callback);
    setup_gpio_pin(PIN_ECHO_6, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_ECHO_6, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &range_sensor_callback);
    setup_gpio_pin(PIN_ECHO_7, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_ECHO_7, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &range_sensor_callback);
    setup_gpio_pin(PIN_ECHO_8, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_ECHO_8, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &range_sensor_callback);
}

/* Used to initializes the PIO for the encoders.

*/
void setup_encoder_pio()
{
    pio_enc_L = pio0;
    pio_enc_R = pio1;

    uint offset_l = pio_add_program(pio_enc_L, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio_enc_L, PIO_ENC_L_SM, offset_l, PIN_ENC_L_A, 0);

    uint offset_r = pio_add_program(pio_enc_R, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio_enc_R, PIO_ENC_R_SM, offset_r, PIN_ENC_R_A, 0);
}

/*  Used to initialize the hardware PWMs for the motor control.

*/
void setup_hardware_pwm()
{
    uint slice_m_l = pwm_gpio_to_slice_num(PIN_MOT_L_PWM);
    uint slice_m_r = pwm_gpio_to_slice_num(PIN_MOT_R_PWM);

    pwm_set_wrap(slice_m_l, MAX_MOT_PWM);
    pwm_set_wrap(slice_m_r, MAX_MOT_PWM);

    pwm_set_enabled(slice_m_l, true);
    pwm_set_enabled(slice_m_r, true);
}

/*  Used to initialize the LSM6DS33 IMU over I2C.

*/
void setup_LSM6DS33()
{
    uint8_t val = LSM_CONFIG_CTRL1;
    i2c_write_to_device(I2C_INSTANCE, LSM_ADDR, LSM_CTRL1_XL, &val, sizeof(val));

    val = LSM_CONFIG_CTRL2;
    i2c_write_to_device(I2C_INSTANCE, LSM_ADDR, LSM_CTRL2_G, &val, sizeof(val));

    val = LSM_CONFIG_CTRL3;
    i2c_write_to_device(I2C_INSTANCE, LSM_ADDR, LSM_CTRL3_C, &val, sizeof(val));
}

/* Used to initialize multicore functionality

*/
void setup_multicore()
{
    multicore_launch_core1(core1_main);

    mutex_init(&mtx_cmd_vel);
    mutex_init(&mtx_odom_data);

    //irq_set_enabled(1, true);
    //irq_set_enabled(0, false);
}

/* Performs all setup routines for the IO and communication.

*/
void pico_setup()
{
    // Initialize GPIO
    setup_gpio();

    // Initialize encoder PIO
    setup_encoder_pio();

    // Initialize PWMs
    setup_hardware_pwm();

    // Initialize I2C
    i2c_init(I2C_INSTANCE, I2C_BAUDRATE);

    // Initialize 6-DOF IMU
    setup_LSM6DS33();

    // Initialize and start multicore functionalities
    setup_multicore();
}


/* Sets the PWM value and direction of the DC motors.

    @param ctrl_left PWM value for left motor - must be between +-MAX_MOT_PWM
    @param ctrl_right PWM value for right motor - must be between +-MAX_MOT_PWM
*/
void set_motor_pwm(int ctrl_left, int ctrl_right)
{   
    // Left motor
    if(ctrl_left < MIN_MOT_PWM && ctrl_left > -MIN_MOT_PWM)
    {
        pwm_set_gpio_level(PIN_MOT_L_PWM, 0);
        gpio_put(PIN_MOT_L_FOR, 0);
        gpio_put(PIN_MOT_L_BAC, 0);
    }
    else if(ctrl_left > 0)
    {
        pwm_set_gpio_level(PIN_MOT_L_PWM, ctrl_left);
        gpio_put(PIN_MOT_L_FOR, 1);
        gpio_put(PIN_MOT_L_BAC, 0);
    }
    else 
    {
        pwm_set_gpio_level(PIN_MOT_L_PWM, -ctrl_left);
        gpio_put(PIN_MOT_L_FOR, 0);
        gpio_put(PIN_MOT_L_BAC, 1);
    }

    if(ctrl_right < MIN_MOT_PWM && ctrl_right > -MIN_MOT_PWM)
    {
        pwm_set_gpio_level(PIN_MOT_R_PWM, 0);
        gpio_put(PIN_MOT_R_FOR, 0);
        gpio_put(PIN_MOT_R_BAC, 0);
    }
    else if(ctrl_right > 0)
    {
        pwm_set_gpio_level(PIN_MOT_R_PWM, ctrl_right);
        gpio_put(PIN_MOT_R_FOR, 1);
        gpio_put(PIN_MOT_R_BAC, 0);
    }
    else 
    {
        pwm_set_gpio_level(PIN_MOT_R_PWM, -ctrl_right);
        gpio_put(PIN_MOT_R_FOR, 0);
        gpio_put(PIN_MOT_R_BAC, 1);
    }
}

/* Write a number of bytes to a subaddress of a I2C device

    @param i2c i2c instance
    @param dev_addr byte address of the device
    @param reg_addr byte address of the first
    @param src pointer to the data storage
    @param len length of data to read in bytes
*/
void i2c_write_to_device(i2c_inst_t *i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *src, size_t len)
{
    uint8_t msg[len + 1];
    msg[0] = reg_addr;
    memcpy(&msg[1], src, len);

    i2c_write_timeout_per_char_us(i2c, dev_addr, &msg, len+1, false, I2C_TIMEOUT_US);
}

/* Reads a number of bytes from a subaddress of a I2C device

    @param i2c i2c instance
    @param dev_addr byte address of the device
    @param reg_addr byte address of the first
    @param tar pointer where to store the read data
    @param len length of data to read in bytes
*/
void i2c_read_from_device(i2c_inst_t *i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *tar, size_t len)
{
    uint8_t reg = reg_addr;
    i2c_write_timeout_per_char_us(i2c, dev_addr, &reg, 1, false, I2C_TIMEOUT_US);
    i2c_read_timeout_per_char_us(i2c, dev_addr, tar, len, false, I2C_TIMEOUT_US);
}


/* Reads IMU data and parses acceleration and angular velocity.

    @param lin_acc *float[3] pointer to store linear acceleration data
    @param ang_vel *float[3] pointer to store angular velocity data
*/
void read_LSM6DS33(float *lin_acc, float *ang_vel)
{
    uint8_t buffer[12];
    i2c_read_from_device(I2C_INSTANCE, LSM_ADDR, LSM_OUTX_L_G, &buffer, 12);

    for(int i = 0; i < 3; i++)
    {
        ang_vel[i] = (int16_t)(buffer[2*i+1] << 8 | buffer[2*i]) * CONV_DPS_TO_RPS * LSM_GYRO_SCALE;
        lin_acc[i] = (int16_t)(buffer[2*i+7] << 8 | buffer[2*i+6]) * CONV_G_TO_MPS2 * LSM_ACCE_SCALE;  
    }
}

/* Reads the temperature from IMU.

    @param lin_acc float pointer to store temperature 
*/
void read_LSM6DS33_temp(float *temp)
{
    uint8_t buffer[2];
    i2c_read_from_device(I2C_INSTANCE, LSM_ADDR, LSM_OUT_TEMP_L , &buffer, 2);
    int16_t temp_raw =  buffer[1] << 8 | buffer[0];
    *temp = temp_raw / LSM_TEMP_SCALE + 25.0;
}

/* Callback for interrupts from the echo pins of the HC-SR04 sensors.
    Used to store the measurement start and parse the range value when receiving an echo.

    @param gpio GPIO source of the interrupt
    @param events GPIO event of the interrupt

*/
void range_sensor_callback(uint gpio, uint32_t events)
{
    uint32_t time = time_us_32();
    uint pin;

    switch (gpio)
    {
    case PIN_ECHO_1:
        pin = 0;
        break;
    case PIN_ECHO_2:
        pin = 1;
        break;
    case PIN_ECHO_3:
        pin = 2;
        break;
    case PIN_ECHO_4:
        pin = 3;
        break;
    case PIN_ECHO_5:
        pin = 4;
        break;
    case PIN_ECHO_6:
        pin = 5;
        break;
    case PIN_ECHO_7:
        pin = 6;
        break;
    default:
        pin = 7;
    }

    if(events == GPIO_IRQ_EDGE_RISE)
    {
        range_sensor_time[pin] = time;
    }
    else
    {
        ultrasonic_range_results[pin] = (float)(time - range_sensor_time[pin]) * SOUND_SPEED_2 ;
    }
}

/* Initializes a measurement of the HC-SR04 by sending a 10us pulse to the trigger pin.

    @param pin Trigger pin of the HC-SR04
*/
void range_sensor_int_sample(uint pin)
{
    gpio_put(pin, 1);
    sleep_us(10);
    gpio_put(pin, 0);
}