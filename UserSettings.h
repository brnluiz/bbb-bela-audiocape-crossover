#ifndef USER_SETTINGS_H
#define USER_SETTINGS_H

#define BEAGLERT_FREQ       44100.00
#define BUTTERWORTH_ORDER   2
#define LINKWITZ_ORDER      2
#define FILTER_NUM          2

struct UserSettings{
    float frequency;
    bool butterworth;
    bool linkwitz;
    bool bassboost;
};

#endif