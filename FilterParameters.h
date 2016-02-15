#ifndef FILTER_PARAMETERS
#define FILTER_PARAMETERS

#define MAX_FILTER_ORDER 8

struct FilterParameters {
    int order;

    float a[MAX_FILTER_ORDER];
    float b[MAX_FILTER_ORDER];

    float x[MAX_FILTER_ORDER];
    float y[MAX_FILTER_ORDER];
};

#endif