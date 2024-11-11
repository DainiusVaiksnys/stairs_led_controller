#define SERVER_MAX_STAIRS 255

enum Mode {
    MODE_OFF,
    MODE_ON,
    MODE_MOTION,
    MODE_BINDING,
    MODE_ANIMATION_WAVES,
    MODE_SOUND
};

struct Config {
    unsigned short int numLeds;
    unsigned short int maxBrightness;
    unsigned char stepsPerMinute;
    Mode userMode;
    unsigned char channelMap[SERVER_MAX_STAIRS];
};