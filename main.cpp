/*
 * assignment1_crossover
 * RTDSP 2015
 * Student: Bruno Luiz da Silva
 * ID: 150724708
 * First assignment for ECS732 RTDSP, to implement a 2-way audio crossover
 * using the BeagleBone Black.
 *
 * Andrew McPherson and Victor Zappi
 * Queen Mary, University of London
 */

#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <libgen.h>
#include <signal.h>
#include <getopt.h>
#include <BeagleRT.h>
#include "UserSettings.h"

using namespace std;

UserSettings userSettings;

// Handle Ctrl-C by requesting that the audio rendering stop
void interrupt_handler(int var)
{
	gShouldStop = true;
}

// Print usage information
void usage(const char * processName)
{
	cerr << "Usage: " << processName << " [options]" << endl;

	BeagleRT_usage();

	cerr << "   --help [-h]:                Print this menu\n";
}

int main(int argc, char *argv[])
{
	BeagleRTInitSettings settings;	// Standard audio settings
	
	userSettings.frequency = 1000.0;
	userSettings.butterworth = true;
	userSettings.linkwitz = false;
	userSettings.bassboost = false;

	struct option customOptions[] =
	{
		{"help", 0, NULL, 'h'},
		{"frequency", 1, NULL, 'f'},
		{"linkwitz", 1, NULL, 'l'},
		{"bassboost", 1, NULL, 'b'},
		{NULL, 0, NULL, 0}
	};

	// Set default settings
	BeagleRT_defaultSettings(&settings);

	// Parse command-line arguments
	while (1) {
		int c;
		if ((c = BeagleRT_getopt_long(argc, argv, "hf:lb", customOptions, &settings)) < 0)
				break;
		switch (c) {
		case 'h':
				usage(basename(argv[0]));
				exit(0);
        case 'f':
        		userSettings.frequency = atof(optarg);
        		if(userSettings.frequency < 20.0)
        			userSettings.frequency = 20.0;
        		if(userSettings.frequency > 5000.0)
        			userSettings.frequency = 5000.0;
        		break;
        case 'l':
        		userSettings.butterworth = false;
        		userSettings.linkwitz = true;
        		break;
        case 'b':
        		userSettings.bassboost = true;
        		break;        		
		case '?':
		default:
				usage(basename(argv[0]));
				exit(1);
		}
	}

	// Initialise the PRU audio device
	if(BeagleRT_initAudio(&settings, &userSettings) != 0) {
		cout << "Error: unable to initialise audio" << endl;
		return -1;
	}

	// Start the audio device running
	if(BeagleRT_startAudio()) {
		cout << "Error: unable to start real-time audio" << endl;
		return -1;
	}

	// Set up interrupt handler to catch Control-C
	signal(SIGINT, interrupt_handler);
	signal(SIGTERM, interrupt_handler);

	// Run until told to stop
	while(!gShouldStop) {
		usleep(100000);
	}

	// Stop the audio device
	BeagleRT_stopAudio();

	// Clean up any resources allocated for audio
	BeagleRT_cleanupAudio();

	// All done!
	return 0;
}
