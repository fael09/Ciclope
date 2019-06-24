// Set (CMAKE_CXX_FLAGS "-lwiringPi")
#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <string> 
#include <stdlib.h>
#include <sstream>
#include <thread>
#include <mutex>
#include <atomic>  
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
 int func() {
    return 1;
}

 int main(){
    std::packaged_task<int()> task{func}; // Create task using func.
    auto future = task.get_future();      // Get the future object.

    std::thread t{std::move(task)};       // std::packaged_task is move-only.

    /* Do something while thread is running... */

    t.join();

    std::cout << "Value: " << future.get() << std::endl; // Get result atomically.
 }