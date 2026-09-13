#ifndef LUMIERE_MOVEMENT
#define LUMIERE_MOVEMENT
#include "../platform/platform_def.h"

void add_movement_packet(MovementType type, int speed, MovementDurationType duration_type, int distance=0);

void run_movement_worker();

#endif
