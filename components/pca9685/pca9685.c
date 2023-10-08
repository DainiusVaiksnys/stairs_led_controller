#include "pca9685.h"
#include <driver/i2c.h>
#include <rom/ets_sys.h>

// REGISTER ADDRESSES
#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1                                                          \
  0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

#define ACK_CHECK_EN    0x1     /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS   0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL         0x0     /*!< I2C ack value */
#define NACK_VAL        0x1     /*!< I2C nack value */

#define I2C_TIMEOUT_MS 1000

#define min(a,b) (b<a?b:a)

i2c_port_t _i2c_port = I2C_NUM_0;
uint32_t _oscillator_freq = FREQUENCY_OSCILLATOR;

esp_err_t read8(uint8_t device, uint8_t addr, uint8_t *value);
esp_err_t readTwoBytes(uint8_t device, uint8_t addr, uint8_t *byte1, uint8_t *byte2);
esp_err_t write8(uint8_t device, uint8_t addr, uint8_t d);
esp_err_t writeTwoWords(uint8_t device, uint8_t addr, uint16_t word1, uint16_t word2);

/*!
 *  @brief  Setups the I2C interface and hardware
 *  @param  prescale
 *          Sets External Clock (Optional)
 *  @return true if successful, otherwise false
 */
boolean pca9685_beginPrescale(uint8_t address, uint16_t freqHz, uint8_t prescale) {
    pca9685_reset(address);

    if (prescale) {
        pca9685_setExtClk(address, prescale);
    } else {
        // set a default frequency
        pca9685_setPWMFreq(address, freqHz);
    }

    return true;
}

boolean pca9685_begin(uint8_t address, uint16_t freqHz) {
    return pca9685_beginPrescale(address, freqHz, 0);
}

/*!
 *  @brief  Sends a reset command to the PCA9685 chip over I2C
 */
void pca9685_reset(uint8_t address) {
    ESP_ERROR_CHECK(write8(address, PCA9685_MODE1, MODE1_RESTART));
    ets_delay_us(10000);
}

/*!
 *  @brief  Puts board into sleep mode
 */
void pca9685_sleep(uint8_t address) {
    uint8_t awake;
    ESP_ERROR_CHECK(read8(address, PCA9685_MODE1, &awake));

    uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
    ESP_ERROR_CHECK(write8(address, PCA9685_MODE1, sleep));
    ets_delay_us(5000); // wait until cycle ends for sleep to be active
}

/*!
 *  @brief  Wakes board from sleep
 */
void pca9685_wakeup(uint8_t address) {
    uint8_t sleep;
    ESP_ERROR_CHECK(read8(address, PCA9685_MODE1, &sleep));

    uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
    ESP_ERROR_CHECK(write8(address, PCA9685_MODE1, wakeup));
}

/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *  @param  prescale
 *          Configures the prescale value to be used by the external clock
 */
void pca9685_setExtClk(uint8_t address, uint8_t prescale) {
    uint8_t oldmode;
    ESP_ERROR_CHECK(read8(address, PCA9685_MODE1, &oldmode));

    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
    ESP_ERROR_CHECK(write8(address, PCA9685_MODE1, newmode)); // go to sleep, turn off internal oscillator

    // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
    // use the external clock.
    ESP_ERROR_CHECK(write8(address, PCA9685_MODE1, (newmode |= MODE1_EXTCLK)));

    ESP_ERROR_CHECK(write8(address, PCA9685_PRESCALE, prescale)); // set the prescaler

    ets_delay_us(5000);
    // clear the SLEEP bit to start
    ESP_ERROR_CHECK(write8(address, PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI));

#ifdef ENABLE_DEBUG_OUTPUT
    ESP_ERROR_CHECK(read8(address, PCA9685_MODE1, &newmode));
    ESP_LOGI(TAG, "Mode now %X", newmode);
#endif
}

/*!
 *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
 *  @param  freq Floating point frequency that we will attempt to match
 */
void pca9685_setPWMFreq(uint8_t address, uint16_t freqHz) {
#ifdef ENABLE_DEBUG_OUTPUT
    ESP_LOGI(TAG, "Attempting to set freq %i Hz", freqHz);
#endif
    // Range output modulation frequency is dependant on oscillator
    if (freqHz < 1)
        freqHz = 1;
    if (freqHz > 1526)
        freqHz = 1526; // Datasheet limit is 1526=25MHz/(4*4096)

    float prescaleval = ((_oscillator_freq / ((float)freqHz * 4096.0f)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN)
        prescaleval = PCA9685_PRESCALE_MIN;
    if (prescaleval > PCA9685_PRESCALE_MAX)
        prescaleval = PCA9685_PRESCALE_MAX;
    uint8_t prescale = (uint8_t)prescaleval;

#ifdef ENABLE_DEBUG_OUTPUT
    ESP_LOGI(TAG, "Final pre-scale: %i", prescale);
#endif

    uint8_t oldmode;
    ESP_ERROR_CHECK(read8(address, PCA9685_MODE1, &oldmode));

    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
    ESP_ERROR_CHECK(write8(address, PCA9685_MODE1, newmode));   // go to sleep
    ESP_ERROR_CHECK(write8(address, PCA9685_PRESCALE, prescale)); // set the prescaler
    ESP_ERROR_CHECK(write8(address, PCA9685_MODE1, oldmode));
    ets_delay_us(5000);
    // This sets the MODE1 register to turn on auto increment.
    ESP_ERROR_CHECK(write8(address, PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI));

#ifdef ENABLE_DEBUG_OUTPUT
    ESP_ERROR_CHECK(read8(address, PCA9685_MODE1, &newmode));
    ESP_LOGI(TAG, "Mode now %X", newmode);
#endif
}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either
 *  open drain or push pull / totempole.
 *  Warning: LEDs with integrated zener diodes should
 *  only be driven in open drain mode.
 *  @param  totempole Totempole if true, open drain if false.
 */
void pca9685_setOutputMode(uint8_t address, boolean totempole) {
    uint8_t oldmode;
    ESP_ERROR_CHECK(read8(address, PCA9685_MODE2, &oldmode));

    uint8_t newmode;
    if (totempole) {
        newmode = oldmode | MODE2_OUTDRV;
    } else {
        newmode = oldmode & ~MODE2_OUTDRV;
    }
    ESP_ERROR_CHECK(write8(address, PCA9685_MODE2, newmode));
#ifdef ENABLE_DEBUG_OUTPUT
    ESP_LOGI(TAG, "Setting output mode: %s by setting MODE2 to %i", totempole ? "totempole" : "open drain", newmode);
#endif
}

/*!
 *  @brief  Reads set Prescale from PCA9685
 *  @return prescale value
 */
uint8_t pca9685_readPrescale(uint8_t address) {
    uint8_t prescale;
    ESP_ERROR_CHECK(read8(address, PCA9685_PRESCALE, &prescale));

    return prescale;
}

/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  off If true, returns PWM OFF value, otherwise PWM ON
 *  @return requested PWM output value
 */
uint16_t pca9685_getPWM(uint8_t address, uint8_t num, boolean off) {
    uint8_t byte1, byte2;
    uint8_t reg = PCA9685_LED0_ON_L + 4 * num;
    if (off)
        reg += 2;

    ESP_ERROR_CHECK(readTwoBytes(address, reg, &byte1, &byte2));

    return (uint16_t)byte1 | (uint16_t)byte2 << 8;
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 */
void pca9685_setPWM(uint8_t address, uint8_t num, uint16_t on, uint16_t off) {
#ifdef ENABLE_DEBUG_OUTPUT
    ESP_LOGI(TAG, "Setting PWM %i: %i->%i", num, on, off);
#endif
    ESP_ERROR_CHECK(writeTwoWords(address, PCA9685_LED0_ON_L + 4 * num, on, off));
}

/*!
 *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
 * on/off tick placement and properly handles a zero value as completely off and
 * 4095 as completely on.  Optional invert parameter supports inverting the
 * pulse for sinking to ground.
 *   @param  num One of the PWM output pins, from 0 to 15
 *   @param  val The number of ticks out of 4096 to be active, should be a value
 * from 0 to 4095 inclusive.
 *   @param  invert If true, inverts the output, defaults to 'false'
 */
void pca9685_setPin(uint8_t address, uint8_t num, uint16_t val, boolean invert) {
    // Clamp value between 0 and 4095 inclusive.
    val = min(val, (uint16_t)4095);
    if (invert) {
        if (val == 0) {
        // Special value for signal fully on.
        pca9685_setPWM(address, num, 4096, 0);
        } else if (val == 4095) {
        // Special value for signal fully off.
        pca9685_setPWM(address, num, 0, 4096);
        } else {
        pca9685_setPWM(address, num, 0, 4095 - val);
        }
    } else {
        if (val == 4095) {
        // Special value for signal fully on.
        pca9685_setPWM(address, num, 4096, 0);
        } else if (val == 0) {
        // Special value for signal fully off.
        pca9685_setPWM(address, num, 0, 4096);
        } else {
        pca9685_setPWM(address, num, 0, val);
        }
  }
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
 * microseconds, output is not precise
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  Microseconds The number of Microseconds to turn the PWM output ON
 */
void pca9685_writeMicroseconds(uint8_t address, uint8_t num, uint16_t Microseconds) {
#ifdef ENABLE_DEBUG_OUTPUT
    ESP_LOGI(TAG, "Setting PWM Via Microseconds on output %i: %i", num, Microseconds);
#endif

    double pulse = Microseconds;
    double pulselength;
    pulselength = 1000000; // 1,000,000 us per second

    // Read prescale
    uint16_t prescale = pca9685_readPrescale(address);

#ifdef ENABLE_DEBUG_OUTPUT
    ESP_LOGI(TAG, "%i PCA9685 chip prescale", prescale);
#endif

    // Calculate the pulse for PWM based on Equation 1 from the datasheet section
    // 7.3.5
    prescale += 1;
    pulselength *= prescale;
    pulselength /= _oscillator_freq;

#ifdef ENABLE_DEBUG_OUTPUT
    ESP_LOGI(TAG, "%i us per bit", pulselength);
#endif

  pulse /= pulselength;

#ifdef ENABLE_DEBUG_OUTPUT
    ESP_LOGI(TAG, "%i pulse for PWM", pulse);
#endif

    pca9685_setPWM(address, num, 0, pulse);
}

/*!
 *  @brief  Getter for the internally tracked oscillator used for freq
 * calculations
 *  @returns The frequency the PCA9685 thinks it is running at (it cannot
 * introspect)
 */
uint32_t pca9685_getOscillatorFrequency(void) {
  return _oscillator_freq;
}

/*!
 *  @brief Setter for the internally tracked oscillator used for freq
 * calculations
 *  @param freq The frequency the PCA9685 should use for frequency calculations
 */
void pca9685_setOscillatorFrequency(uint32_t freq) {
  _oscillator_freq = freq;
}

/******************* Low level I2C interface */
esp_err_t read8(uint8_t device, uint8_t addr, uint8_t *byte) {
  return readTwoBytes(device, addr, byte, NULL);
}

esp_err_t readTwoBytes(uint8_t device, uint8_t addr, uint8_t *byte1, uint8_t *byte2) {
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_i2c_port, cmd, I2C_TIMEOUT_MS/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, byte1, ACK_VAL);
    if (byte2) {
        i2c_master_read_byte(cmd, byte2, NACK_VAL);
    }
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(_i2c_port, cmd, I2C_TIMEOUT_MS/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

esp_err_t write8(uint8_t device, uint8_t addr, uint8_t d) {
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, d, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_i2c_port, cmd, I2C_TIMEOUT_MS/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t writeTwoWords(uint8_t device, uint8_t addr, uint16_t word1, uint16_t word2) {
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, word1 & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, word1 >> 8, NACK_VAL);
    i2c_master_write_byte(cmd, word2 & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, word2 >> 8, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_i2c_port, cmd, I2C_TIMEOUT_MS/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}