#include "constants.h"

namespace CONFIG {
    const int RIGHT_MOT_ID = 2;
    const int RIGHT_MOT_RATIO = 8;

    const int LEFT_MOT_ID = 1;
    const int LEFT_MOT_RATIO = 10;

    const double PITCH_KP = 1.2;
    const double PITCH_KI = 0.0;
    const double PITCH_KD = 1;

    const double POS_KP = 0.0;
    const double POS_KI = 0.0;
    const double POS_KD = 0.0;

    const double YAW_KP = 1;
    const double YAW_KI = 0;
    const double YAW_KD = 0;

    const double MAX_ALPHA = 256;
    const double MAX_VEL = 32;

    const double CTRL_RATE = 1000;

    const double MAX_HOMING_TILT_ANGLE = 1.5;
}