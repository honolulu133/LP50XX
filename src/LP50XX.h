/**
 * @file LP50XX.h
 * @author rneurink (ruben.neurink@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2021-07-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __LP50XX_H
#define __LP50XX_H

#include <Arduino.h>
#include <Wire.h>

#define DEFAULT_ADDRESS 0x14
#define BROADCAST_ADDRESS 0x0C

enum LP50XX_TYPE {
    LP5009 = 0x00,
    LP5012 = 0x01,
};

enum LED_Configuration {
    RGB,
    GRB,
    BGR,
    RBG,
    GBR,
    BRG,
};

/**
 * @brief LED bank configuration
 * Enable bank control for the specified LED banks.
 * When a channel is configured in LED bank-control mode, the related color mixing and intensity control
 * is governed by the bank control registers (BANK_A_COLOR, BANK_B_COLOR, BANK_C_COLOR, and BANK_BRIGHTNESS)
 * regardless of the inputs on its own color-mixing and intensity-control registers
 * 
 * The LED banks are defined as follows:
 * - LED_0_Bank: OUT0, OUT1, OUT2
 * - LED_1_Bank: OUT3, OUT4, OUT5
 * - LED_2_Bank: OUT6, OUT7, OUT8
 * - LED_3_Bank: OUT9, OUT10, OUT11 (only on the LP5012)
 */
enum LP50XX_LED_Bank {
    LED0_Module = 1,
    LED1_Module = 2,
    LED2_Module = 4,
    LED3_Module = 8
};

enum LP50XX_Configuration {
    //DEVICE_CONFIG0
    LED_GLOBAL_ON = 0 << 0,
    LED_GLOBAL_OFF = 1 << 0,
    //DEVICE_CONFIG1
    MAX_CURRENT_25mA = 0 << 1,
    MAX_CURRENT_35mA = 1 << 1,
    PWM_DITHERING_OFF = 0 << 2,
    PWM_DITHERING_ON = 1 << 2,
    AUTO_INC_OFF = 0 << 3,
    AUTO_INC_ON = 1 << 3,
    POWER_SAVE_OFF = 0 << 4,
    POWER_SAVE_ON = 1 << 4,
    LOG_SCALE_OFF = 0 << 5,
    LOG_SCALE_ON = 1 << 5
};

/**
 * @brief Structure to hold the configuration of the LP50XX
 * 
 * This structure contains the following fields:
 * @param Log_scale 1 = Logarithmic scale dimming curve enabled, 0 = Linear scale dimming curve enabled
 * @param Power_save 1 = Automatic power-saving mode enabled, 0 = Automatic power-saving mode not enabled
 * @param Auto_inc 1 = Automatic increment mode enabled, 0 = Automatic increment mode not enabled. The auto-increment feature allows writing or reading several consecutive registers within one transmission.
 * @param PWM_dithering 1 = PWM dithering mode enabled, 0 = PWM dithering mode not enabled
 * @param Max_current_option 1 = Output maximum current IMAX = 35 mA., 0 = Output maximum current IMAX = 25.5 mA.
 * @param LED_Global_off 1 = Shut down all LEDs, 0 = Normal operation 
 */
struct st_LP50XX_Configuration {
    bool Log_scale; // Bit5 Log scale
    bool Power_save; // Bit4 Power save
    bool Auto_inc; // Bit3 Auto increment
    bool PWM_dithering; // Bit2 PWM dithering
    bool Max_current_option; // Bit1 Max current option
    bool LED_Global_off; // Bit0 LED global off
};

enum EAddressType {
    Normal,
    Broadcast
};

// Register definitions
#define DEVICE_CONFIG0 0x00     // Chip_EN
#define DEVICE_CONFIG1 0x01     // Configurations for Log_scale, Power_save, Auto_inc, PWM_dithering, Max_current_option and LED_Global_off
#define LED_CONFIG0 0x02        // Contains the BANK configurations
#define BANK_BRIGHTNESS 0x03    // Contains the BANK brightness level 
#define BANK_A_COLOR 0x04       // Contains the BANK A color value
#define BANK_B_COLOR 0x05       // Contains the BANK B color value
#define BANK_C_COLOR 0x06       // Contains the BANK C color value
#define LED0_BRIGHTNESS 0x07    // Contains the brightness level for LED 0
#define LED1_BRIGHTNESS 0x08    // Contains the brightness level for LED 1
#define LED2_BRIGHTNESS 0x09    // Contains the brightness level for LED 2
#define LED3_BRIGHTNESS 0x0A    // Contains the brightness level for LED 3 (only on the LP5012)
#define OUT0_COLOR 0x0B         // Contains the color value for output 0
#define OUT1_COLOR 0x0C         // Contains the color value for output 1
#define OUT2_COLOR 0x0D         // Contains the color value for output 2
#define OUT3_COLOR 0x0E         // Contains the color value for output 3
#define OUT4_COLOR 0x0F         // Contains the color value for output 4
#define OUT5_COLOR 0x10         // Contains the color value for output 5
#define OUT6_COLOR 0x11         // Contains the color value for output 6
#define OUT7_COLOR 0x12         // Contains the color value for output 7
#define OUT8_COLOR 0x13         // Contains the color value for output 8
#define OUT9_COLOR 0x14         // Contains the color value for output 9 (only on the LP5012)
#define OUT10_COLOR 0x15        // Contains the color value for output 10 (only on the LP5012)
#define OUT11_COLOR 0x16        // Contains the color value for output 11 (only on the LP5012)
#define RESET_REGISTERS 0x17    // Resets all the registers to their default values

/**
 * @brief Class to communicate with the LP5009 or LP5012
 */
class LP50XX 
{
    public:
        LP50XX(void); // Constructor
        LP50XX(LED_Configuration ledConfiguration); // Constructor with a specific led configuration
        LP50XX(uint8_t enablePin); // Constructor with enable pin
        LP50XX(LED_Configuration ledConfiguration, uint8_t enablePin); // Constructor with a specific led configuration and an enable pin

        LP50XX(LP50XX_TYPE type); // Constructor with a specific type
        LP50XX(LP50XX_TYPE type, LED_Configuration ledConfiguration); // Constructor with a specific type and a specific led configuration
        LP50XX(LP50XX_TYPE type, uint8_t enablePin); // Constructor with a specific type and an enable pin
        LP50XX(LP50XX_TYPE type, LED_Configuration ledConfiguration, uint8_t enablePin); // Constructor with a specific type, a specific led configuration and an enable pin

        /**
         * Initialisation functions
         */
        void Begin(uint8_t i2c_address = DEFAULT_ADDRESS); // Initialize the driver
        void Reset();
        void ResetRegisters(EAddressType addressType = EAddressType::Normal);

        /**
         * Configuration functions
         */
        /**
         * @brief Configures the device according to the configuration param
         * 
         * @note A configuration can be `Configure(LED_GLOBAL_ON | MAX_CURRENT_25mA | PWM_DITHERING_ON | AUTO_INC_ON | POWER_SAVE_ON | LOG_SCALE_ON);`
         * 
         * @param configuration The configuration of the device, this can be made by bitwise OR ('|') the enum @ref LP50XX_Configuration
         * @param addressType the I2C address type to write to 
         */
        void Configure(uint8_t configuration, EAddressType addressType = EAddressType::Normal);

        /**
         * @brief Sets the PWM scaling used by the device
         * 
         * @param scaling The scaling of the device. @ref LOG_SCALE_OFF @ref LOG_SCALE_ON
         */
        void SetScaling(uint8_t scaling);

        /**
         * @brief Sets the power saving mode of the device
         * 
         * @param powerSave The power saving mode. @ref POWER_SAVE_OFF @ref POWER_SAVE_ON
         */
        void SetPowerSaving(uint8_t powerSave);
        
        /**
         * @brief Sets the auto increment mode of the device
         * 
         * @param autoInc The auto increment mode. @ref AUTO_INC_OFF @ref AUTO_INC_ON
         */
        void SetAutoIncrement(uint8_t autoInc);
        
        /**
         * @brief Sets the PWM dithering of the device
         * 
         * @param dithering The dithering mode. @ref PWM_DITHERING_OFF @ref PWM_DITHERING_ON
         */
        void SetPWMDithering(uint8_t dithering);
        
        /**
         * @brief Sets the max current option of the device
         * 
         * @param option The max current option. @ref MAX_CURRENT_25mA @ref MAX_CURRENT_35mA
         */
        void SetMaxCurrentOption(uint8_t option);
        
        /**
         * @brief Sets the global LED off mode of the device
         * 
         * @param value The desired setting. @ref LED_GLOBAL_OFF @ref LED_GLOBAL_ON
         */
        void SetGlobalLedOff(uint8_t value);

        /** 
         * Set the configuration of the LP50XX as a structure
         * 
         * @param configuration The configuration to set. See @ref LP50XX_Configuration
         * @param addressType the I2C address type to write to
         */
        void SetConfiguration(st_LP50XX_Configuration configuration, EAddressType addressType = EAddressType::Normal);

        /**
         * Get the configuration of the LP50XX as a structure
         * 
         * @return The configuration of the LP50XX
         */
        st_LP50XX_Configuration GetConfiguration();

        /**
         * @brief Sets the enable pin of the device. This pin is used to enable the device in @ref Begin
         * 
         * @param enablePin 
         */
        void SetEnablePin(uint8_t enablePin);

        /**
         * @brief Sets the LED configuration acording the @ref LED_Configuration enum
         * 
         * @param ledConfiguration 
         */
        void SetLEDConfiguration(LED_Configuration ledConfiguration);

        /**
         * @brief Sets the I2C address
         * 
         * @param address 
         */
        void SetI2CAddress(uint8_t address);

        /**
         * Bank control functions
         */

        /**
         * Set the BANK control for specific led-modules
         * 
         * @param leds The LEDs to include in BANK control. See @ref LP50XX_LED_Bank
         * @param addressType the I2C address type to write to
         * 
         * @note Code example could be `SetBankControl(LED0_Module | LED1_Module | LED2_Module | LED3_Module);`
         */
        void SetBankControl(uint8_t ledBanks, EAddressType addressType = EAddressType::Normal);

        /**
         * Set the brightness level of all banks
         * Sets the brightness level of all led-modules (LED0 - LED3) connected to the bank.
         * 
         * @param brightness The brightness level from 0 to 0xFF
         * @param addressType the I2C address type to write to
         */
        void SetBankBrightness(uint8_t brightness, EAddressType addressType = EAddressType::Normal);


        /**
         * Set the brightness level of a specific bank
         * Bank A: OUT0, OUT3, OUT6, OUT9
         * Bank B: OUT1, OUT4, OUT7, OUT10
         * Bank C: OUT2, OUT5, OUT8, OUT11
         * 
         * @param value The brightness level from 0 to 0xFF
         * @param addressType the I2C address type to write to
         */
        void SetBankColorA(uint8_t value, EAddressType addressType = EAddressType::Normal);
        void SetBankColorB(uint8_t value, EAddressType addressType = EAddressType::Normal);
        void SetBankColorC(uint8_t value, EAddressType addressType = EAddressType::Normal);

        /**
         * Set the BANK color according to the set LED configuration @ref SetLEDConfiguration
         * 
         * @param red The red color value from 0 to 0xFF
         * @param green The green color value from 0 to 0xFF
         * @param blue The blue color value from 0 to 0xFF
         * @param addressType the I2C address type to write to
         */
        void SetBankColor(uint8_t red, uint8_t green, uint8_t blue, EAddressType addressType = EAddressType::Normal);

        /**
         * Output control functions
         */
        /**
         * @brief Sets the brightness level of a single LED module (3 outputs)
         * 
         * @param led The led module to set. 0..3
         * @param brighness The brightness level from 0 to 0xFF
         * @param addressType the I2C address type to write to
         */
        void SetLEDBrightness(uint8_t led, uint8_t brightness, EAddressType addressType = EAddressType::Normal);


        /**
         * Set the color level of a single output
         * 
         * @param output The output to set. 0..11
         * @param value The color value from 0 to 0xFF
         * @param addressType the I2C address type to write to
         */
        void SetOutputColor(uint8_t output, uint8_t value, EAddressType addressType = EAddressType::Normal);

        /**
         * Set the LED color according to the set LED configuration @ref SetLEDConfiguration
         * Only sets the led modules with bank control enabled
         * 
         * @param led The led to set. 0..3
         * @param red The red color value from 0 to 0xFF
         * @param green The green color value from 0 to 0xFF
         * @param blue The blue color value from 0 to 0xFF
         * @param addressType the I2C address type to write to
         */
        void SetLEDColor(uint8_t led, uint8_t red, uint8_t green, uint8_t blue, EAddressType addressType = EAddressType::Normal);

        /**
         * Set the brightness of three consecutive LEDs for each value in the array.
         * Every three LEDs are connected to the same brightness register, so:
         * brightness[0] controls LED0, LED1, LED2
         * brightness[1] controls LED3, LED4, LED5
         * brightness[2] controls LED6, LED7, LED8
         * brightness[3] controls LED9, LED10, LED11 (LP5012 only)
         * 
         * @note Keeping 0xFF (default) results in 100% dimming duty cycle. With this setting, users can just
         * configure the color mixing register by channel to achieve the target dimming effect in a
         * single-color LED application.
         * 
         * @param brightness Array of brightness settings. Use [4] for LP5012, [3] for LP5009
         * @param addressType the I2C address type to write to
         */
        void SetLedBrightness(uint8_t* brightness, EAddressType addressType = EAddressType::Normal);

        /**
         * Set the individual brightness of every output.
         *          * 
         * @param output Array of brightness settings. Use [12] for LP5012, [9] for LP5009
         * @param addressType the I2C address type to write to
         */
        void SetOutputColor(uint8_t* output, EAddressType addressType = EAddressType::Normal);

        /**
         * Low level functions
         */
        /**
         * @brief Writes a value to a specified register. @warning only use if you know what you're doing
         * 
         * @param reg The register to write to
         * @param value The value to write to the register
         * @param addressType the I2C address type to write to
         */
        void WriteRegister(uint8_t reg, uint8_t value, EAddressType addressType = EAddressType::Normal);

        /**
         * @brief Reads a value from a specified register.
         * 
         * @param reg The register to read from
         * @param value a reference to a @ref uint8_t value
         */
        void ReadRegister(uint8_t reg, uint8_t *value);

    protected:

    private:
        uint8_t     _i2c_address;
        uint8_t     _i2c_address_broadcast = BROADCAST_ADDRESS;
        uint8_t     _enable_pin = 0xFF;
        LED_Configuration     _led_configuration = RGB;
        LP50XX_TYPE _type = LP5009;

        uint8_t getAddress(EAddressType addressType);
};

#endif