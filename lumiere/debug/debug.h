#ifndef LUMIERE_DEBUG
#define LUMIERE_DEBUG
#include <string>

inline bool debug_enabled = true;

void log(std::string message);
void log(std::string sender, std::string message);

#endif
