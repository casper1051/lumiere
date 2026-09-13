#include "debug.h"
#include <iostream>
#include <string>

void log(std::string message) {
    if (debug_enabled) std::cout << " [ Debug ] " << message << std::endl << std::flush;
}

void log(std::string sender, std::string message) {
    if (debug_enabled) std::cout << " [ " << sender << " ] " << message << std::endl << std::flush;
}
