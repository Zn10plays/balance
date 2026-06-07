#include <cmath>
#include <moteus/moteus.h>
#include <hat/pi3hat.h>

void quaternionToEuler(const mjbots::pi3hat::Quaternion& q, mjbots::pi3hat::Euler& ret_obj) {
    double roll, pitch, yaw;

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1.0) {
        // Use 90 degrees if out of range safely
        pitch = std::copysign(M_PI / 2.0, sinp); 
    } else {
        pitch = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    // Convert radians to degrees
    roll  = roll  * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw   = yaw   * 180.0 / M_PI;

    ret_obj.yaw = yaw;
    ret_obj.pitch = pitch;
    ret_obj.roll = roll;
}
