#ifndef BUTTERWORTH_FILTER_PARAMETERS
#define BUTTERWORTH_FILTER_PARAMETERS

struct FilterParameters {
    float a[8];
    float b[8];

    float x[8];
    float y[8];

    unsigned short order;
};

#endif