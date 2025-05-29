/**
 * @file tsl2591.c
 * @brief Driver for the TSL2591 light sensor
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include "pico/stdlib.h"
 #include "hardware/i2c.h"
 #include "tsl2591.h"
 
 // Static variables
 static i2c_inst_t *i2c_instance;
 static tsl2591_gain_t current_gain = TSL2591_GAIN_1X;
 static tsl2591_integration_time_t current_integration_time = TSL2591_INTEGRATIONTIME_100;
 
 /**
  * @brief Write a byte to the TSL2591 register
  * 
  * @param reg Register address
  * @param value Value to write
  * @return true if successful, false otherwise
  */
 static bool tsl2591_write_register(uint8_t reg, uint8_t value) {
     uint8_t buf[2];
     buf[0] = TSL2591_COMMAND_BIT | reg;
     buf[1] = value;
     
     int result = i2c_write_blocking(i2c_instance, TSL2591_I2C_ADDR, buf, 2, false);
     return (result == 2);
 }
 
 /**
  * @brief Read a byte from the TSL2591 register
  * 
  * @param reg Register address
  * @param value Pointer to store the read value
  * @return true if successful, false otherwise
  */
 static bool tsl2591_read_register(uint8_t reg, uint8_t *value) {
     uint8_t cmd = TSL2591_COMMAND_BIT | reg;
     
     int result = i2c_write_blocking(i2c_instance, TSL2591_I2C_ADDR, &cmd, 1, true);
     if (result != 1) {
         return false;
     }
     
     result = i2c_read_blocking(i2c_instance, TSL2591_I2C_ADDR, value, 1, false);
     return (result == 1);
 }
 
 /**
  * @brief Initialize the TSL2591 sensor
  * 
  * @param i2c I2C instance to use
  * @return true if initialization successful, false otherwise
  */
 bool tsl2591_init(i2c_inst_t *i2c) {
     i2c_instance = i2c;
     
     // Check if the sensor is responding
     uint8_t id;
     if (!tsl2591_read_register(TSL2591_REGISTER_ID, &id)) {
         printf("TSL2591: Failed to read sensor ID\n");
         return false;
     }
     
     // ID should be 0x50
     if (id != 0x50) {
         printf("TSL2591: Invalid sensor ID (0x%02X)\n", id);
         return false;
     }
     
     // Power on the sensor
     if (!tsl2591_enable()) {
         printf("TSL2591: Failed to enable sensor\n");
         return false;
     }
     
     // Set lower gain and shorter integration time for more appropriate readings
     tsl2591_set_gain(TSL2591_GAIN_1X);
     tsl2591_set_integration_time(TSL2591_INTEGRATIONTIME_100);
     
     // Power off the sensor (until needed)
     tsl2591_disable();
     
     printf("TSL2591: Initialized successfully\n");
     return true;
 }
 
 /**
  * @brief Set the gain for the TSL2591 sensor
  * 
  * @param gain Gain setting
  * @return true if successful, false otherwise
  */
 bool tsl2591_set_gain(tsl2591_gain_t gain) {
     // Read current register value
     uint8_t value;
     if (!tsl2591_read_register(TSL2591_REGISTER_CONTROL, &value)) {
         return false;
     }
     
     // Update gain bits (bits 5:4)
     value &= 0xCF; // Clear gain bits
     value |= gain;
     
     // Write back the updated value
     if (!tsl2591_write_register(TSL2591_REGISTER_CONTROL, value)) {
         return false;
     }
     
     current_gain = gain;
     
     const char* gain_strings[] = {
         "1x", "25x", "428x", "9876x"
     };
     printf("TSL2591: Gain set to %s\n", gain_strings[gain >> 4]);
     
     return true;
 }
 
 /**
  * @brief Set the integration time for the TSL2591 sensor
  * 
  * @param integration_time Integration time setting
  * @return true if successful, false otherwise
  */
 bool tsl2591_set_integration_time(tsl2591_integration_time_t integration_time) {
     // Read current register value
     uint8_t value;
     if (!tsl2591_read_register(TSL2591_REGISTER_CONTROL, &value)) {
         return false;
     }
     
     // Update integration time bits (bits 2:0)
     value &= 0xF8; // Clear integration time bits
     value |= integration_time;
     
     // Write back the updated value
     if (!tsl2591_write_register(TSL2591_REGISTER_CONTROL, value)) {
         return false;
     }
     
     current_integration_time = integration_time;
     
     int int_time_ms = (integration_time + 1) * 100;
     printf("TSL2591: Integration time set to %d ms\n", int_time_ms);
     
     return true;
 }
 
 /**
  * @brief Enable the TSL2591 sensor
  * 
  * @return true if successful, false otherwise
  */
 bool tsl2591_enable(void) {
     // Enable sensor with AEN (bit 1)
     uint8_t value = TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN;
     return tsl2591_write_register(TSL2591_REGISTER_ENABLE, value);
 }
 
 /**
  * @brief Disable the TSL2591 sensor
  * 
  * @return true if successful, false otherwise
  */
 bool tsl2591_disable(void) {
     // Power off the sensor
     return tsl2591_write_register(TSL2591_REGISTER_ENABLE, 0);
 }
 
 /**
  * @brief Read the full luminosity (visible + IR) from the sensor
  * 
  * @param channel0 Pointer to store full spectrum reading
  * @param channel1 Pointer to store infrared reading
  * @return true if successful, false otherwise
  */
 bool tsl2591_get_full_luminosity(uint16_t *channel0, uint16_t *channel1) {
     // Enable the sensor
     if (!tsl2591_enable()) {
         return false;
     }
     
     // Wait for integration time to complete
     int delay_ms = (current_integration_time + 1) * 100;
     sleep_ms(delay_ms + 50); // Add 50ms buffer
     
     // Read channel 0 (full spectrum) data
     uint8_t ch0_low, ch0_high;
     if (!tsl2591_read_register(TSL2591_REGISTER_CHAN0_L, &ch0_low) ||
         !tsl2591_read_register(TSL2591_REGISTER_CHAN0_H, &ch0_high)) {
         return false;
     }
     
     // Read channel 1 (infrared) data
     uint8_t ch1_low, ch1_high;
     if (!tsl2591_read_register(TSL2591_REGISTER_CHAN1_L, &ch1_low) ||
         !tsl2591_read_register(TSL2591_REGISTER_CHAN1_H, &ch1_high)) {
         return false;
     }
     
     // Disable the sensor
     tsl2591_disable();
     
     // Combine high and low bytes
     *channel0 = (ch0_high << 8) | ch0_low;
     *channel1 = (ch1_high << 8) | ch1_low;
     
     return true;
 }
 
 /**
  * @brief Get visible light reading
  * 
  * @return uint16_t Visible light reading (full - IR)
  */
 uint16_t tsl2591_get_visible_light(void) {
     uint16_t full, ir;
     
     if (tsl2591_get_full_luminosity(&full, &ir)) {
         return full - ir;
     }
     
     return 0;
 }
 
 /**
  * @brief Calculate lux based on sensor readings
  * 
  * @param ch0 Full spectrum reading
  * @param ch1 Infrared reading
  * @return float Lux value
  */
 float tsl2591_calculate_lux(uint16_t ch0, uint16_t ch1) {
     // Check if either channel is saturated
     if ((ch0 == 0xFFFF) || (ch1 == 0xFFFF)) {
         return -1.0;
     }

     // Calculate the actual integration time in milliseconds (not seconds)
     float atime = 100.0f * (current_integration_time + 1);

     // Calculate the actual gain multiplier
     float again;
     switch (current_gain) {
         case TSL2591_GAIN_1X:    again = 1.0f;    break;
         case TSL2591_GAIN_25X:   again = 25.0f;   break;
         case TSL2591_GAIN_428X:  again = 428.0f;  break;
         case TSL2591_GAIN_9876X: again = 9876.0f; break;
         default:                 again = 1.0f;    break;
     }

     // Calculate CPL (counts per lux) using datasheet formula
     float cpl = (atime * again) / 408.0f;  // 408.0 is the recommended datasheet value

     // Prevent division by zero
     if (cpl < 0.001f) {
         cpl = 0.001f;
     }

     // Calculate lux using TSL2591 datasheet formula
     float lux1 = ((float)ch0 - (1.64f * (float)ch1)) / cpl;
     float lux2 = ((0.59f * (float)ch0) - (0.86f * (float)ch1)) / cpl;
     float lux = lux1 > lux2 ? lux1 : lux2;

     // Debug output to help diagnose readings
     printf("DEBUG: ch0=%u, ch1=%u, atime=%.1f, again=%.1f, cpl=%.3f, raw_lux=%.2f\n", 
            ch0, ch1, atime, again, cpl, lux);

     // Reasonable bounds for indoor lighting
     if (lux > 1000.0f) {
         // If reading is too high, adjust gain
         if (current_gain > TSL2591_GAIN_1X) {
             tsl2591_set_gain(current_gain >> 1);  // Decrease gain
             printf("Light too bright, decreasing gain\n");
         }
         lux = 1000.0f;
     } else if (lux < 1.0f && current_gain < TSL2591_GAIN_9876X) {
         // If reading is too low, adjust gain
         tsl2591_set_gain(current_gain << 1);  // Increase gain
         printf("Light too dim, increasing gain\n");
     }

     return (lux < 0) ? 0 : lux;
 }
