/**
 * @file LP50XX.cpp
 * @author rneurink (ruben.neurink@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2021-07-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "LP50XX.h"
#include "I2C_coms.h"

/*----------------------- Initialisation functions --------------------------*/

/**
 * @brief This function instantiates the class object
 */
LP50XX::LP50XX() {

}

/**
 * @brief This function instantiates the class object with a specific LED configuration
 * 
 * @param ledConfiguration The LED configuration in which the leds are attached to the outputs. See @ref LED_Configuration
 */
LP50XX::LP50XX(LED_Configuration ledConfiguration) {
    _led_configuration = ledConfiguration;

    LP50XX();
}

/**
 * @brief This function instantiates the class object with an enable pin
 * 
 * @param enablePin the pin that is connected to the EN pin of the LP5009 or LP5012
 */
LP50XX::LP50XX(uint8_t enablePin) {
    pinMode(enablePin, OUTPUT);
    _enable_pin = enablePin;

    LP50XX();
}

/**
 * @brief This function instantiates the class object with a specific LED configuration and with an enable pin
 * 
 * @param ledConfiguration The LED configuration in which the leds are attached to the outputs. See @ref LED_Configuration
 * @param enablePin the pin that is connected to the EN pin of the LP5009 or LP5012
 */
LP50XX::LP50XX(LED_Configuration ledConfiguration, uint8_t enablePin) {
    _led_configuration = ledConfiguration;
    
    pinMode(enablePin, OUTPUT);
    _enable_pin = enablePin;

    LP50XX();
}

/**
 * @brief This function instantiates the class object with a specific type
 *          
 * @param type The type of the device. See @ref LP50XX_TYPE
 */
LP50XX::LP50XX(LP50XX_TYPE type)
: _type(type) {

    LP50XX();
}
/**
 * @brief This function instantiates the class object with
 * a specific type and a specific LED configuration
 *  
 * @param type The type of the device. See @ref LP50XX_TYPE
 * @param ledConfiguration The LED configuration in which the leds are attached to the outputs. See @ref LED_Configuration
 */
LP50XX::LP50XX(LP50XX_TYPE type, LED_Configuration ledConfiguration)
: _type(type), _led_configuration(ledConfiguration) {

    LP50XX();
}

/**
 * @brief This function instantiates the class object with
 * a specific type and an enable pin
 *  @param type The type of the device. See @ref LP50XX_TYPE
 * @param enablePin the pin that is connected to the EN pin of the LP5009 or LP5012
 */ 
LP50XX::LP50XX(LP50XX_TYPE type, uint8_t enablePin)
: _type(type), _enable_pin(enablePin) {

    pinMode(_enable_pin, OUTPUT);
    LP50XX();
}

/**
 * @brief This function instantiates the class object with
 * a specific type, a specific LED configuration and an enable pin
 * 
 * @param type The type of the device. See @ref LP50XX_TYPE
 * @param ledConfiguration The LED configuration in which the leds are attached to the outputs. See @ref LED_Configuration
 * @param enablePin the pin that is connected to the EN pin of the LP5009 or LP5012
 */
LP50XX::LP50XX(LP50XX_TYPE type, LED_Configuration ledConfiguration, uint8_t enablePin)
: _type(type), _led_configuration(ledConfiguration), _enable_pin(enablePin) {

    pinMode(_enable_pin, OUTPUT);
    LP50XX();
}

/**
 * @brief Initializes the I2C bus and the LP5009 or LP5012
 * 
 * @param i2cAddress The I2C address of the device
 */
void LP50XX::Begin(uint8_t i2cAddress) {
    i2c_init();
    _i2c_address = i2cAddress;

    if (_enable_pin != 0xFF) {
        digitalWrite(_enable_pin, HIGH);
    }

    // 500 us delay after enabling the device before I2C access is available
    delayMicroseconds(500);

    // Enable the Chip_EN bit to start up the device
    i2c_write_byte(_i2c_address, DEVICE_CONFIG0, 1 << 6);
}

/**
 * @brief Resets the device by using the enable pin if available and resetting the registers
 * 
 */
void LP50XX::Reset() {
    if (_enable_pin != 0xFF) {
        digitalWrite(_enable_pin, LOW);
        delay(10);
        digitalWrite(_enable_pin, HIGH);
        // 500 us delay after enabling the device before I2C access is available
        delayMicroseconds(500);
    }
    
    ResetRegisters();

    // Enable the Chip_EN bit to start up the device
    i2c_write_byte(_i2c_address, DEVICE_CONFIG0, 1 << 6);
}

/**
 * @brief Resets the registers to their original values
 * 
 * @param addressType the I2C address type to write to 
 */
void LP50XX::ResetRegisters(EAddressType addressType) {
    i2c_write_byte(getAddress(addressType), RESET_REGISTERS, 0xFF);
}


/*----------------------- Configuration functions ---------------------------*/
void LP50XX::Configure(uint8_t configuration, EAddressType addressType) {
    i2c_write_byte(getAddress(addressType), DEVICE_CONFIG1, configuration & 0x3F);
}

void LP50XX::SetScaling(uint8_t scaling) {
    uint8_t buff;
    i2c_read_byte(_i2c_address, DEVICE_CONFIG1, &buff);

    scaling = scaling & 1 << 5;
    if (scaling >> 5 & 1) {
        buff |= 1 << 5;
    } else {
        buff &= ~(1 << 5);
    }
    i2c_write_byte(_i2c_address, DEVICE_CONFIG1, buff);
}

void LP50XX::SetPowerSaving(uint8_t powerSave) {
    uint8_t buff;
    i2c_read_byte(_i2c_address, DEVICE_CONFIG1, &buff);

    powerSave = powerSave & 1 << 4;
    if (powerSave >> 4 & 1) {
        buff |= 1 << 4;
    } else {
        buff &= ~(1 << 4);
    }
    i2c_write_byte(_i2c_address, DEVICE_CONFIG1, buff);
}

void LP50XX::SetAutoIncrement(uint8_t autoInc) {
    uint8_t buff;
    i2c_read_byte(_i2c_address, DEVICE_CONFIG1, &buff);

    autoInc = autoInc & 1 << 3;
    if (autoInc >> 3 & 1) {
        buff |= 1 << 3;
    } else {
        buff &= ~(1 << 3);
    }
    i2c_write_byte(_i2c_address, DEVICE_CONFIG1, buff);
}

void LP50XX::SetPWMDithering(uint8_t dithering) {
    uint8_t buff;
    i2c_read_byte(_i2c_address, DEVICE_CONFIG1, &buff);

    dithering = dithering & 1 << 2;
    if (dithering >> 2 & 1) {
        buff |= 1 << 2;
    } else {
        buff &= ~(1 << 2);
    }
    i2c_write_byte(_i2c_address, DEVICE_CONFIG1, buff);
}

void LP50XX::SetMaxCurrentOption(uint8_t option) {
    uint8_t buff;
    i2c_read_byte(_i2c_address, DEVICE_CONFIG1, &buff);

    option = option & 1 << 1;
    if (option >> 1 & 1) {
        buff |= 1 << 1;
    } else {
        buff &= ~(1 << 1);
    }
    i2c_write_byte(_i2c_address, DEVICE_CONFIG1, buff);
}

void LP50XX::SetGlobalLedOff(uint8_t value) {
    uint8_t buff;
    i2c_read_byte(_i2c_address, DEVICE_CONFIG1, &buff);

    value = value & 1 << 0;
    if (value >> 0 & 1) {
        buff |= 1 << 0;
    } else {
        buff &= ~(1 << 0);
    }
    i2c_write_byte(_i2c_address, DEVICE_CONFIG1, buff);
}

void SetConfiguration(st_LP50XX_Configuration configuration, EAddressType addressType = EAddressType::Normal) {
    uint8_t buff = 0;
    buff |= (configuration.Log_scale << 5);
    buff |= (configuration.Power_save << 4);
    buff |= (configuration.Auto_inc << 3);
    buff |= (configuration.PWM_dithering << 2);
    buff |= (configuration.Max_current_option << 1);
    buff |= (configuration.LED_Global_off << 0);

    i2c_write_byte(getAddress(addressType), DEVICE_CONFIG1, buff);
}

st_LP50XX_Configuration LP50XX::GetConfiguration() {
    uint8_t buff;
    i2c_read_byte(_i2c_address, DEVICE_CONFIG1, &buff);

    st_LP50XX_Configuration configuration;
    configuration.Log_scale = (buff >> 5) & 1;
    configuration.Power_save = (buff >> 4) & 1;
    configuration.Auto_inc = (buff >> 3) & 1;
    configuration.PWM_dithering = (buff >> 2) & 1;
    configuration.Max_current_option = (buff >> 1) & 1;
    configuration.LED_Global_off = (buff >> 0) & 1;

    return configuration;
}

void LP50XX::SetEnablePin(uint8_t enablePin) {
    pinMode(enablePin, OUTPUT);
    _enable_pin = enablePin;
}

void LP50XX::SetLEDConfiguration(LED_Configuration ledConfiguration) {
    _led_configuration = ledConfiguration;
}

void LP50XX::SetI2CAddress(uint8_t address) {
    _i2c_address = address;
}


/*----------------------- Bank control functions ----------------------------*/

void LP50XX::SetBankControl(uint8_t ledBanks, EAddressType addressType) {
    i2c_write_byte(getAddress(addressType), LED_CONFIG0, ledBanks);
}

void LP50XX::SetBankBrightness(uint8_t brightness, EAddressType addressType) {
    i2c_write_byte(getAddress(addressType), BANK_BRIGHTNESS, brightness);
}

void LP50XX::SetBankColorA(uint8_t value, EAddressType addressType) {
    i2c_write_byte(getAddress(addressType), BANK_A_COLOR, value);
}

void LP50XX::SetBankColorB(uint8_t value, EAddressType addressType) {
    i2c_write_byte(getAddress(addressType), BANK_B_COLOR, value);
}

void LP50XX::SetBankColorC(uint8_t value, EAddressType addressType) {
    i2c_write_byte(getAddress(addressType), BANK_C_COLOR, value);
}

void LP50XX::SetBankColor(uint8_t red, uint8_t green, uint8_t blue, EAddressType addressType) {
    SetAutoIncrement(AUTO_INC_ON);

    uint8_t buff[3];
    switch (_led_configuration)
    {
    case RGB:
        buff[0] = red;
        buff[1] = green;
        buff[2] = blue;
        break;
    case GRB:
        buff[0] = green;
        buff[1] = red;
        buff[2] = blue;
        break;
    case BGR:
        buff[0] = blue;
        buff[1] = green;
        buff[2] = red;
        break;
    case RBG:
        buff[0] = red;
        buff[1] = blue;
        buff[2] = green;
        break;
    case GBR:
        buff[0] = green;
        buff[1] = blue;
        buff[2] = red;
        break;
    case BRG:
        buff[0] = blue;
        buff[1] = red;
        buff[2] = green;
        break;
    }

    i2c_write_multi(getAddress(addressType), BANK_A_COLOR, buff, 3);
}


/*----------------------- Output control functions --------------------------*/

void LP50XX::SetLEDBrightness(uint8_t led, uint8_t brightness, EAddressType addressType) {
    i2c_write_byte(getAddress(addressType), LED0_BRIGHTNESS + led, brightness);
}

void LP50XX::SetOutputColor(uint8_t output, uint8_t value, EAddressType addressType) {
    i2c_write_byte(getAddress(addressType), OUT0_COLOR + output, value);
}

void LP50XX::SetLEDColor(uint8_t led, uint8_t red, uint8_t green, uint8_t blue, EAddressType addressType) {
    SetAutoIncrement(AUTO_INC_ON);

    uint8_t buff[3];
    switch (_led_configuration)
    {
    case RGB:
        buff[0] = red;
        buff[1] = green;
        buff[2] = blue;
        break;
    case GRB:
        buff[0] = green;
        buff[1] = red;
        buff[2] = blue;
        break;
    case BGR:
        buff[0] = blue;
        buff[1] = green;
        buff[2] = red;
        break;
    case RBG:
        buff[0] = red;
        buff[1] = blue;
        buff[2] = green;
        break;
    case GBR:
        buff[0] = green;
        buff[1] = blue;
        buff[2] = red;
        break;
    case BRG:
        buff[0] = blue;
        buff[1] = red;
        buff[2] = green;
        break;
    }

    i2c_write_multi(getAddress(addressType), OUT0_COLOR + (led * 3), buff, 3);
}

void SetLedBrightness(uint8_t* brightness, EAddressType addressType) {
    if (brightness == nullptr) {
        return;
    }

    uint8_t length = 3;
    if (_type == LP5012) {
        length = 4;
    }

    i2c_write_multi(getAddress(addressType), LED0_BRIGHTNESS, brightness, length);
}

 void SetOutputColor(uint8_t* output, EAddressType addressType) {
    if (output == nullptr) {
        return;
    }

    uint8_t length = 9;
    if (_type == LP5012) {
        length = 12;
    }

    i2c_write_multi(getAddress(addressType), OUT0_COLOR, output, length);
}

/*----------------------- Low level functions -------------------------------*/

void LP50XX::WriteRegister(uint8_t reg, uint8_t value, EAddressType addressType) {
    i2c_write_byte(getAddress(addressType), reg, value);
}

void LP50XX::ReadRegister(uint8_t reg, uint8_t *value) {
    i2c_read_byte(_i2c_address, reg, value);
}

/*------------------------- Helper functions --------------------------------*/

/*
 *  PRIVATE
 */ 

/**
 * @brief Resolves the EAddressType into an address
 * 
 * @param addressType the I2C address type to write to. This translates to the stored addresses in the class
 * @return uint8_t the I2C address translated from the addressType
 */
uint8_t LP50XX::getAddress(EAddressType addressType) {
    uint8_t i2c_address;
    switch (addressType)
    {
    case EAddressType::Broadcast:
        i2c_address = _i2c_address_broadcast;
        break;
    case EAddressType::Normal:
    default:
    i2c_address = _i2c_address;
        break;
    }
    return i2c_address;
}
