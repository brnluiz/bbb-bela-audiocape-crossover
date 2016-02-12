#ifndef BUTTERWORTH_FILTER_PARAMETERS
#define BUTTERWORTH_FILTER_PARAMETERS

struct FilterParameters {
    float a0;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;

    float x[FILTER_ORDER];
    float y[FILTER_ORDER];
};

#endif