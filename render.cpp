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

FilterParameters gLowPass[FILTER_NUM], gHighPass[FILTER_NUM];

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


	for (int i = 0; i < FILTER_NUM; i++) {
		gLowPass[i].a0 = pow(c,2) + d*w + pow(w,2);
		gLowPass[i].a1 = (2*pow(w,2) - 2*pow(c,2)) / gLowPass[i].a0;
		gLowPass[i].a2 = (pow(c,2) - d*w + pow(w,2)) / gLowPass[i].a0;
		gLowPass[i].b0 = (pow(w,2)) / gLowPass[i].a0;
		gLowPass[i].b1 = (2*pow(w,2)) / gLowPass[i].a0;
		gLowPass[i].b2 = (pow(w,2)) / gLowPass[i].a0;
		gLowPass[i].a0 = 1;

		gHighPass[i].a0 = pow(c,2) + d*w + pow(w,2);
		gHighPass[i].a1 = (2*pow(w,2) - 2*pow(c,2)) / gHighPass[i].a0;
		gHighPass[i].a2 = (pow(c,2) - d*w + pow(w,2)) / gHighPass[i].a0;
		gHighPass[i].b0 = (pow(c,2)) / gHighPass[i].a0;
		gHighPass[i].b1 = (2*pow(c,2)) / gHighPass[i].a0;
		gHighPass[i].b2 = (pow(c,2)) / gHighPass[i].a0;
		gHighPass[i].a0 = 1;
	}

	for(int i = 0; i < FILTER_ORDER; i++) {
		for(int j = 0; j < FILTER_NUM; j++) {
			gLowPass[j].x[i] = 0;
			gLowPass[j].y[i] = 0;
			gHighPass[j].x[i] = 0;
			gHighPass[j].y[i] = 0;
		}
	}

	return true;
}

OutputCrossaudio filterButterworth(float sample, FilterParameters &lowPass, FilterParameters &highPass) {
	// Implement the low pass filter
	float lowPassOut = lowPass.b2 * lowPass.x[1] + lowPass.b1 * lowPass.x[0] + lowPass.b0 * sample 
	- (lowPass.a2 * lowPass.y[1] + lowPass.a1 * lowPass.y[0]);

	// Implement the high pass filter
	float highPassOut = highPass.b2 * highPass.x[1] + highPass.b1 * highPass.x[0] + highPass.b0 * sample 
	- (highPass.a2 * highPass.y[1] + highPass.a1 * highPass.y[0]);

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
			OutputCrossaudio stage1 = filterButterworth(sample, gLowPass[0], gHighPass[0]);
			sample = stage1.high;
			OutputCrossaudio stage2 = filterButterworth(sample, gLowPass[1], gHighPass[1]);

			output.low = stage1.low;
			output.high = stage2.high;
		} else {
			OutputCrossaudio stage1 = filterButterworth(sample, gLowPass[0], gHighPass[0]);
			output.low = stage1.low;
			output.high = stage1.high;
		}


		// Audio output
		// Butterworth
		// context->audioOut[n * context->audioChannels + 0] = stage1.low; // Left channel
		// context->audioOut[n * context->audioChannels + 1] = stage1.high; // Right channel
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
