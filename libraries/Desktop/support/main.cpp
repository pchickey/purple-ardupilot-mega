#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sched.h>
#include <wiring.h>
#include <getopt.h>
#include <signal.h>
#include <string.h>
#include "desktop.h"

void setup(void);
void loop(void);

// the state of the desktop simulation
struct desktop_info desktop_state;

static void usage(void)
{
	printf("Options:\n");
	printf("\t-s          enable CLI slider switch\n");
	printf("\t-w          wipe eeprom and dataflash\n");
}

int main(int argc, char * const argv[])
{
	int opt;
	// default state
	desktop_state.slider = false;
	gettimeofday(&desktop_state.sketch_start_time, NULL);

	while ((opt = getopt(argc, argv, "swh")) != -1) {
		switch (opt) {
		case 's':
			desktop_state.slider = true;
			break;
		case 'w':
			unlink("eeprom.bin");
			unlink("dataflash.bin");
			break;
		default:
			usage();
			exit(1);
		}
	}

	if (strcmp(SKETCH, "ArduCopter") == 0) {
		desktop_state.quadcopter = true;
	}

	sitl_setup();
	setup();

	while (true) {
		struct timeval tv;
		fd_set fds;
		int fd_high = 0;

		FD_ZERO(&fds);
		loop();

		desktop_serial_select_setup(&fds, &fd_high);
		tv.tv_sec = 0;
		tv.tv_usec = 100;

		select(fd_high+1, &fds, NULL, NULL, &tv);
	}
	return 0;
}
