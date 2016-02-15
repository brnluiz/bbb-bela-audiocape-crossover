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
#include "FilterParameters.h"

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

	gHighPass.order = 2;
	gLowPass.order  = 2;
	gLowPass.order  = 2;

	gLowPass.a[0] = pow(c,2) + d*w + pow(w,2);
	gLowPass.a[1] = (2*pow(w,2) - 2*pow(c,2));
	gLowPass.a[2] = (pow(c,2) - d*w + pow(w,2));		
	gLowPass.b[0] = (pow(w,2));
	gLowPass.b[1] = (2*pow(w,2));
	gLowPass.b[2] = (pow(w,2));

	gHighPass.a[0] = gLowPass.a[0];
	gHighPass.a[1] = gLowPass.a[1];
	gHighPass.a[2] = gLowPass.a[2];
	gHighPass.b[0] = (pow(c,2));
	gHighPass.b[1] = -(2*pow(c,2));
	gHighPass.b[2] = (pow(c,2));

	if(settings.linkwitz) {
		FilterParameters bLowPass, bHighPass;
		bLowPass  = gLowPass;
		bHighPass = gHighPass;

		gHighPass.order = 4;
		gLowPass.order  = 4;

		gLowPass.a[0] = pow(bLowPass.a[0] ,2);
		gLowPass.a[1] = (2*bLowPass.a[0]*bLowPass.a[1]);
		gLowPass.a[2] = (2*bLowPass.a[0]*bLowPass.a[2] + pow(bLowPass.a[1],2));
		gLowPass.a[3] = (2*bLowPass.a[1]*bLowPass.a[2]);
		gLowPass.a[4] = (pow(bLowPass.a[2],2));
		
		gLowPass.b[0] = (pow(bLowPass.b[0],2));
		gLowPass.b[1] = (2*bLowPass.b[0]*bLowPass.b[1]);
		gLowPass.b[2] = (2*bLowPass.b[0]*bLowPass.b[2] + pow(bLowPass.b[1],2));
		gLowPass.b[3] = (2*bLowPass.b[1]*bLowPass.b[2]);
		gLowPass.b[4] = (pow(bLowPass.b[2],2));

		gHighPass.a[0] = pow(bHighPass.a[0], 2);
		gHighPass.a[1] = (2*bHighPass.a[0]*bHighPass.a[1]);
		gHighPass.a[2] = (2*bHighPass.a[0]*bHighPass.a[2] + pow(bHighPass.a[1],2));
		gHighPass.a[3] = (2*bHighPass.a[1]*bHighPass.a[2]);
		gHighPass.a[4] = (pow(bHighPass.a[2],2));
		
		gHighPass.b[0] = (pow(bHighPass.b[0],2));
		gHighPass.b[1] = (2*bHighPass.b[0]*bHighPass.b[1]);
		gHighPass.b[2] = (2*bHighPass.b[0]*bHighPass.b[2] + pow(bHighPass.b[1],2));
		gHighPass.b[3] = (2*bHighPass.b[1]*bHighPass.b[2]);
		gHighPass.b[4] = (pow(bHighPass.b[2],2));
	}

	for(int i = 0; i < MAX_FILTER_ORDER; i++) {
		gLowPass.x[i]  = 0;
		gLowPass.y[i]  = 0;
		gHighPass.x[i] = 0;
		gHighPass.y[i] = 0;
		
		gLowPass.b[i]  = gLowPass.b[i]/gLowPass.a[0];
		gHighPass.b[i] = gHighPass.b[i]/gHighPass.a[0];
		
		if(i != 0) {
			gLowPass.a[i]  = gLowPass.a[i]/gLowPass.a[0];
			gHighPass.a[i] = gHighPass.a[i]/gHighPass.a[0];
		}
	}
	gLowPass.a[0]  = 1;
	gHighPass.a[0] = 1;

	return true;
}

float filter(float sample, FilterParameters &filter) {
	float out = 0;
	// Create the filter output

	for(int i = 0; i <= filter.order; i++) {
		if (i > 0) {
			out += filter.b[i] * filter.x[i-1];
			out -= filter.a[i] * filter.y[i-1];
		} else {
			out += filter.b[0] * sample;
		}
	}

	// Save data for later
	for (int i = 1; i < filter.order; i++) {
		filter.x[i] = filter.x[i-1];
		filter.y[i] = filter.y[i-1];
	}
	filter.x[0] = sample;
	filter.y[0] = out;

	return out;
}

OutputCrossaudio crossover(float sample, FilterParameters &lowPass, FilterParameters &highPass) {
	float lowPassOut  = filter(sample, lowPass);
	float highPassOut = filter(sample, highPass);

	OutputCrossaudio output;
	output.low = lowPassOut;
	output.high = highPassOut;

	return output;
}

OutputCrossaudio butterworthCross(float sample, FilterParameters &lowPass, FilterParameters &highPass) {
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

OutputCrossaudio linkwitzCross(float sample, FilterParameters &lowPass, FilterParameters &highPass) {
	// Implement the low pass filter
	float lowPassOut = lowPass.b[4] * lowPass.x[3] + lowPass.b[3] * lowPass.x[2] + lowPass.b[2] * lowPass.x[1] + lowPass.b[1] * lowPass.x[0] + lowPass.b[0] * sample 
	- (lowPass.a[4] * lowPass.y[3] + lowPass.a[3] * lowPass.y[2] + lowPass.a[2] * lowPass.y[1] + lowPass.a[1] * lowPass.y[0]);

	// Implement the high pass filter
	float highPassOut = highPass.b[4] * highPass.x[3] + highPass.b[3] * highPass.x[2] + highPass.b[2] * highPass.x[1] + highPass.b[1] * highPass.x[0] + highPass.b[0] * sample 
	- (highPass.a[4] * highPass.y[3] + highPass.a[3] * highPass.y[2] + highPass.a[2] * highPass.y[1] + highPass.a[1] * highPass.y[0]);

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

		// Do the crossover based on the filter details
		OutputCrossaudio out;
		if(settings.butterworth) {
			out = butterworthCross(sample, gLowPass, gHighPass);
		} else if(settings.linkwitz) {
			out = linkwitzCross(sample, gLowPass, gHighPass);
		}
		// OutputCrossaudio out = crossover(sample, gLowPass, gHighPass);

		// Audio output
		context->audioOut[n * context->audioChannels + 0] = out.low; // Left channel
		context->audioOut[n * context->audioChannels + 1] = out.high; // Right channel
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
