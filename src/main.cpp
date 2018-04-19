
#include "tracker\TrackerSystem.h"

////////////////////////////
// MAIN

int main(int argc, char* argv[]) {
	TrackerSystem ts;
	ts.Init();
	ts.Loop();

	std::cout << std::endl << "waiting on a keypress to terminate.";
	getchar();
}