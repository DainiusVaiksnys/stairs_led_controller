#include "esp_err.h"

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef long unsigned int uint32_t;

typedef unsigned char boolean;

boolean pca9685_begin(uint8_t address, uint16_t freqHz);
boolean pca9685_beginPrescale(uint8_t address, uint16_t freqHz, uint8_t prescale);
void pca9685_reset(uint8_t address);
void pca9685_sleep(uint8_t address);
void pca9685_wakeup(uint8_t address);
void pca9685_setExtClk(uint8_t address, uint8_t prescale);
void pca9685_setPWMFreq(uint8_t address, uint16_t freqHz);
void pca9685_setOutputMode(uint8_t address, boolean totempole);
uint16_t pca9685_getPWM(uint8_t address, uint8_t num, boolean off);
void pca9685_setPWM(uint8_t address, uint8_t num, uint16_t on, uint16_t off);
void pca9685_setPin(uint8_t address, uint8_t num, uint16_t val, boolean invert);
uint8_t pca9685_readPrescale(uint8_t address);
void pca9685_writeMicroseconds(uint8_t address, uint8_t num, uint16_t Microseconds);

void pca9685_setOscillatorFrequency(uint32_t freq);
uint32_t pca9685_getOscillatorFrequency(void);