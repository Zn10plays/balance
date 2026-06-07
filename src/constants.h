// Constants.h
#pragma once

namespace CONFIG {
    extern const int RIGHT_MOT_ID;
    extern const int RIGHT_MOT_RATIO;

    extern const int LEFT_MOT_ID;
    extern const int LEFT_MOT_RATIO;

    // pitch pid gains
    extern const double PITCH_KP;
    extern const double PITCH_KI;
    extern const double PITCH_KD;

    extern const double POS_KP;
    extern const double POS_KI;
    extern const double POS_KD;

    // yaw pid gains
    extern const double YAW_KP;
    extern const double YAW_KI;
    extern const double YAW_KD;

    // dynamic limits
    extern const double MAX_ALPHA;
    extern const double MAX_VEL;

    // frequency
    extern const double CTRL_RATE; // in hz 

    // max tilt angle
    extern const double MAX_HOMING_TILT_ANGLE;
}
