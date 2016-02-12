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
#include "UserSettings.h"
#include "ButterworthFilterParameters.h"

/* TASK: declare any global variables you need here */

// setup() is called once before the audio rendering starts.
// Use it to perform any initialisation and allocation which is dependent
// on the period size or sample rate.
//
// userData holds an opaque pointer to a data structure that was passed
// in from the call to initAudio().
//
// Return true on success; returning false halts the program.

FilterParameters gLowPass, gHighPass;

struct OutputCrossaudio {
	float high;
	float low;
};
OutputCrossaudio output;
UserSettings settings;

bool setup(BeagleRTContext *context, void *userData)
{
	const float c = 2 * BEAGLERT_FREQ;
	const float d = c * 1.4142;

	// Retrieve a parameter passed in from the initAudio() call
	if(userData != 0) {
		settings = *(UserSettings *)userData;
	}
	float w = 2 * M_PI * settings.frequency;

	/* TASK:
	 * Calculate the filter coefficients based on the given
	 * crossover frequency.
	 *
	 * Initialise any previous state (clearing buffers etc.)
	 * to prepare for calls to render()
	 */
	if(settings.linkwitz) {
		gLowPass.a[0] = pow(c,4)+2*pow(c,2)*d*w+2*pow(c,2)*pow(w,2)+pow(d,2)*pow(w,2)+2*d*pow(w,3)+pow(w,4);
		gLowPass.a[1] = (-(4*pow(c,4))-(4*pow(c,2)*d*w)+(4*pow(w,4))+(4*d*pow(w,3))) / gLowPass.a[0];
		gLowPass.a[2] = (+(6*pow(c,4))+(6*pow(w,4))-(4*pow(c,2)*pow(w,2))-(2*pow(d,2)*pow(w,2))) / gLowPass.a[0];
		gLowPass.a[3] = (-(4*pow(c,4))-(4*d*pow(w,3))+(4*pow(w,4))+(4*pow(c,2)*d*w)) / gLowPass.a[0];
		gLowPass.a[4] = (pow(c,4)-(2*pow(c,2)*d*w)-(2*d*pow(w,3))+(pow(d,2)*pow(w,2))+pow(w,4)+(2*pow(c,2)*pow(w,2))) / gLowPass.a[0];
		
		gLowPass.b[0] = (1*pow(w,4)) / gLowPass.a[0];
		gLowPass.b[1] = (4*pow(w,2)) / gLowPass.a[0];
		gLowPass.b[2] = (6*pow(w,2)) / gLowPass.a[0];
		gLowPass.b[3] = (4*pow(w,2)) / gLowPass.a[0];
		gLowPass.b[4] = (1*pow(w,4)) / gLowPass.a[0];


		gHighPass.a[0] = gLowPass.a[0];
		gHighPass.a[1] = gLowPass.a[1];
		gHighPass.a[2] = gLowPass.a[2];
		gHighPass.a[3] = gLowPass.a[3];
		gHighPass.a[4] = gLowPass.a[4];

		gHighPass.b[0] = (1*pow(c,4)) / gLowPass.a[0];
		gHighPass.b[1] = (4*pow(c,2)) / gLowPass.a[0];
		gHighPass.b[2] = (6*pow(c,2)) / gLowPass.a[0];
		gHighPass.b[3] = (4*pow(c,2)) / gLowPass.a[0];
		gHighPass.b[4] = (1*pow(c,4)) / gLowPass.a[0];

		gLowPass.a[0] = 1;
		gHighPass.a[0] = 1;
	} else {
		gLowPass.a[0] = pow(c,2) + d*w + pow(w,2);
		gLowPass.a[1] = (2*pow(w,2) - 2*pow(c,2)) / gLowPass.a[0];
		gLowPass.a[2] = (pow(c,2) - d*w + pow(w,2)) / gLowPass.a[0];
		gLowPass.b[0] = (pow(w,2)) / gLowPass.a[0];
		gLowPass.b[1] = (2*pow(w,2)) / gLowPass.a[0];
		gLowPass.b[2] = (pow(w,2)) / gLowPass.a[0];

		gHighPass.a[0] = gLowPass.a[0];
		gHighPass.a[1] = gLowPass.a[1];
		gHighPass.a[2] = gLowPass.a[2];
		gHighPass.b[0] = (pow(c,2)) / gLowPass.a[0];
		gHighPass.b[1] = (2*pow(c,2)) / gLowPass.a[0];
		gHighPass.b[2] = (pow(c,2)) / gLowPass.a[0];
		
		gLowPass.a[0] = 1;
		gHighPass.a[0] = 1;
	}

	for(int i = 0; i < 8; i++) {
		gLowPass.x[i] = 0;
		gLowPass.y[i] = 0;
		gHighPass.x[i] = 0;
		gHighPass.y[i] = 0;
	}

	return true;
}

OutputCrossaudio crossoverButterworth(float sample, FilterParameters &lowPass, FilterParameters &highPass) {
	// Implement the low pass filter
	float lowPassOut = lowPass.b[2] * lowPass.x[1] + lowPass.b[1] * lowPass.x[0] + lowPass.b[0] * sample 
	- (lowPass.a[2] * lowPass.y[1] + lowPass.a[1] * lowPass.y[0]);

	// Implement the high pass filter
	float highPassOut = highPass.b[2] * highPass.x[1] + highPass.b[1] * highPass.x[0] + highPass.b[0] * sample 
	- (highPass.a[2] * highPass.y[1] + highPass.a[1] * highPass.y[0]);

	// Save the samples the old samples
	lowPass.x[1] = lowPass.x[0];
	lowPass.x[0] = sample;
	lowPass.y[1] = lowPass.y[0];
	lowPass.y[0] = lowPassOut;

	// Save the samples the old samples
	highPass.x[1] = highPass.x[0];
	highPass.x[0] = sample;
	highPass.y[1] = highPass.y[0];
	highPass.y[0] = highPassOut;

	OutputCrossaudio output;
	output.low = lowPassOut;
	output.high = highPassOut;

	return output;
}

OutputCrossaudio crossoverLinkwitz(float sample, FilterParameters &lowPass, FilterParameters &highPass) {
	// Implement the low pass filter
	float lowPassOut = lowPass.b[4] * lowPass.x[3] + lowPass.b[3] * lowPass.x[2] + lowPass.b[2] * lowPass.x[1] + lowPass.b[1] * lowPass.x[0] + lowPass.b[0] * sample 
	- ( + lowPass.a[4] * lowPass.y[3] + lowPass.a[3] * lowPass.y[2] + lowPass.a[2] * lowPass.y[1] + lowPass.a[1] * lowPass.y[0]);

	// Implement the high pass filter
	float highPassOut = highPass.b[4] * highPass.x[3] + highPass.b[3] * highPass.x[2] + highPass.b[2] * highPass.x[1] + highPass.b[1] * highPass.x[0] + highPass.b[0] * sample 
	- ( + highPass.a[4] * highPass.y[3] + highPass.a[3] * highPass.y[2] + highPass.a[2] * highPass.y[1] + highPass.a[1] * highPass.y[0]);

	// Save the samples the old samples
	lowPass.x[4] = lowPass.x[3];
	lowPass.x[3] = lowPass.x[2];
	lowPass.x[2] = lowPass.x[1];
	lowPass.x[1] = lowPass.x[0];
	lowPass.x[0] = sample;

	lowPass.y[4] = lowPass.y[3];
	lowPass.y[3] = lowPass.y[2];
	lowPass.y[2] = lowPass.y[1];
	lowPass.y[1] = lowPass.y[0];
	lowPass.y[0] = lowPassOut;

	// Save the samples the old samples
	highPass.x[4] = highPass.x[3];
	highPass.x[3] = highPass.x[2];
	highPass.x[2] = highPass.x[1];
	highPass.x[1] = highPass.x[0];
	highPass.x[0] = sample;

	highPass.y[4] = highPass.y[3];
	highPass.y[3] = highPass.y[2];
	highPass.y[2] = highPass.y[1];
	highPass.y[1] = highPass.y[0];
	highPass.y[0] = highPassOut;

	OutputCrossaudio output;
	output.low = lowPassOut;
	output.high = highPassOut;

	return output;
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
		
		if (settings.linkwitz) {
			OutputCrossaudio stage1 = crossoverLinkwitz(sample, gLowPass, gHighPass);

			output.low = stage1.low;
			output.low = stage1.high;
		} else {
			OutputCrossaudio stage1 = crossoverButterworth(sample, gLowPass, gHighPass);
			output.low = stage1.low;
			output.high = stage1.high;
		}

		// Audio output
		context->audioOut[n * context->audioChannels + 0] = output.low; // Left channel
		context->audioOut[n * context->audioChannels + 1] = output.high; // Right channel
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
