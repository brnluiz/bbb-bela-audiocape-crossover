/*
 * assignment1_crossover
 * RTDSP 2016
 *
 * First assignment for ECS732 RTDSP, to implement a 2-way audio crossover
 * using the BeagleBone Black.
 *
 * Andrew McPherson and Victor Zappi
 * Modified by Becky Stewart
 * Queen Mary, University of London
 */

#include <BeagleRT.h>
#include <cmath>
#include <Utilities.h>
#include <rtdk.h>

/* TASK: declare any global variables you need here */

// setup() is called once before the audio rendering starts.
// Use it to perform any initialisation and allocation which is dependent
// on the period size or sample rate.
//
// userData holds an opaque pointer to a data structure that was passed
// in from the call to initAudio().
//
// Return true on success; returning false halts the program.
#define BEAGLERT_FREQ 44100.00
#define FILTER_ORDER  2

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
FilterParameters gLowPass, gHighPass;

const float c = 2 * BEAGLERT_FREQ;
const float d = c * 1.4142;

float gCrossoverFrequency;
float w0;

bool setup(BeagleRTContext *context, void *userData)
{
	// Retrieve a parameter passed in from the initAudio() call
	if(userData != 0)
		gCrossoverFrequency = *(float *)userData;
	w0 = 2 * M_PI * gCrossoverFrequency;

	/* TASK:
	 * Calculate the filter coefficients based on the given
	 * crossover frequency.
	 *
	 * Initialise any previous state (clearing buffers etc.)
	 * to prepare for calls to render()
	 */
	gLowPass.a0 = pow(c,2) + d*w0 + pow(w0,2);
	gLowPass.a1 = (2*pow(w0,2) - 2*pow(c,2)) / gLowPass.a0;
	gLowPass.a2 = (pow(c,2) - d*w0 + pow(w0,2)) / gLowPass.a0;
	gLowPass.b0 = (pow(w0,2)) / gLowPass.a0;
	gLowPass.b1 = (2*pow(w0,2)) / gLowPass.a0;
	gLowPass.b2 = (pow(w0,2)) / gLowPass.a0;
	gLowPass.a0 = 1;

	gHighPass.a0 = pow(c,2) + d*w0 + pow(w0,2);
	gHighPass.a1 = (2*pow(w0,2) - 2*pow(c,2)) / gHighPass.a0;
	gHighPass.a2 = (pow(c,2) - d*w0 + pow(w0,2)) / gHighPass.a0;
	gHighPass.b0 = (pow(c,2)) / gHighPass.a0;
	gHighPass.b1 = (2*pow(c,2)) / gHighPass.a0;
	gHighPass.b2 = (pow(c,2)) / gHighPass.a0;
	gHighPass.a0 = 1;

	for(int i = 0; i < FILTER_ORDER; i++) {
		gLowPass.x[i] = 0;
		gLowPass.y[i] = 0;
		gHighPass.x[i] = 0;
		gHighPass.y[i] = 0;
	}

	return true;
}

// render() is called regularly at the highest priority by the audio engine.
// Input and output are given from the audio hardware and the other
// ADCs and DACs (if available). If only audio is available, numMatrixFrames
// will be 0.

void render(BeagleRTContext *context, void *userData)
{
	for(unsigned int n = 0; n < context->audioFrames; n++) {
		// Get the input
		float sample = (context->audioIn[n*context->audioChannels] + context->audioIn[n*context->audioChannels+1]) * 0.5;

		// Implement the low pass filter
		float lowPassOut = gLowPass.b2 * gLowPass.x[1] + gLowPass.b1 * gLowPass.x[0] + gLowPass.b0 * sample 
		- (gLowPass.a2 * gLowPass.y[1] + gLowPass.a1 * gLowPass.y[0]);

		// Implement the high pass filter
		float highPassOut = gHighPass.b2 * gHighPass.x[1] + gHighPass.b1 * gHighPass.x[0] + gHighPass.b0 * sample 
		- (gHighPass.a2 * gHighPass.y[1] + gHighPass.a1 * gHighPass.y[0]);

		// Save the samples the old samples
		gLowPass.x[1] = gLowPass.x[0];
		gLowPass.x[0] = sample;
		gLowPass.y[1] = gLowPass.y[0];
		gLowPass.y[0] = lowPassOut;

		// Save the samples the old samples
		gHighPass.x[1] = gHighPass.x[0];
		gHighPass.x[0] = sample;
		gHighPass.y[1] = gHighPass.y[0];
		gHighPass.y[0] = highPassOut;

		context->audioOut[n * context->audioChannels + 0] = lowPassOut; // Left channel
		context->audioOut[n * context->audioChannels + 1] = highPassOut; // Right channel
	}
}

// cleanup_render() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in initialise_render().

void cleanup(BeagleRTContext *context, void *userData)
{
	/* TASK:
	 * If you allocate any memory, be sure to release it here.
	 * You may or may not need anything in this function, depending
	 * on your implementation.
	 */
}
