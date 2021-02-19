#include <iostream>
#include <thread>

#include "recorder.h"
#include "random_trajectory_generator.h"

int main(int argc, char **argv) {
    
    // Recorder runs asynchronously to main thread with its own airsim client
    // This will launch the thread.
    Recorder recorder;
    std::thread recorder_thread(&Recorder::record, &recorder);
    
    // Run trajectory generator (sending commands for moving) on main thread
    RandomTrajectoryGenerator generator;
    generator.takeoff();
    generator.generate();
    generator.land();

    recorder.finish();
    recorder_thread.join();

    return EXIT_SUCCESS;
}
