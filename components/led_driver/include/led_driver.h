void led_driver_init(unsigned short freqHz);
void led_driver_cleanup();
void led_driver_map(unsigned short int ledIndex, unsigned short int pwmChannel);
void led_driver_setNumLeds(unsigned short int num);
void led_driver_startInfiniteAnimation();