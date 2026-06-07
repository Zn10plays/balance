#include <moteus/moteus.h>
#include <hat/pi3hat_moteus_transport.h>
#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <algorithm> // For std::clamp

#include "constants.h"
#include "angles.cpp"
#include "PID.cpp"

namespace moteus = mjbots::moteus;

// Helper to constrain values
double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

int main() {
    // 1. Configure the pi3hat transport options
    using Transport = mjbots::pi3hat::Pi3HatMoteusTransport;
    Transport::Options toptions;
    
    toptions.servo_map[1] = 5;
    toptions.servo_map[2] = 3;
    toptions.attitude_rate_hz = CONFIG::CTRL_RATE;
    toptions.mounting_deg.pitch = 0;
    toptions.mounting_deg.yaw = 0;
    toptions.mounting_deg.roll = 0;
    
    auto transport = std::make_shared<Transport>(toptions);

    mjbots::pi3hat::Attitude attitude;
    mjbots::pi3hat::Euler angles;
    int loop_delay_ms = (1.0 / CONFIG::CTRL_RATE) * 1000;

    // INNER LOOP: Keeps the robot upright at a given Target Pitch
    PID pitch_controller(
        CONFIG::PITCH_KP,
        CONFIG::PITCH_KI,
        CONFIG::PITCH_KD,
        loop_delay_ms
    );

    // OUTER LOOP: Keeps the robot stationary at X=0
    // (Note: You will need to add POS_KP, POS_KI, POS_KD to your constants.h)
    PID position_controller(
        CONFIG::POS_KP, 
        CONFIG::POS_KI, 
        CONFIG::POS_KD, 
        loop_delay_ms
    );

    // Moteus Controller Setup
    moteus::Controller leftMotor([&transport]() {
        moteus::Controller::Options options;
        options.id = CONFIG::LEFT_MOT_ID;
        options.query_format.position = moteus::kFloat;
        options.position_format.velocity_limit = mjbots::moteus::kFloat;
        options.transport = transport;
        return options;
    }());

    moteus::Controller rightMotor([&transport]() {
        moteus::Controller::Options options;
        options.id = CONFIG::RIGHT_MOT_ID;
        options.query_format.position = moteus::kFloat;
        options.position_format.velocity_limit = mjbots::moteus::kFloat;
        options.transport = transport;
        return options;
    }());

    // Velocity commands (Position is set to NaN to run in pure velocity mode)
    moteus::PositionMode::Command leftCommand;
    leftCommand.position = std::numeric_limits<double>::quiet_NaN();
    leftCommand.velocity_limit = CONFIG::MAX_VEL * CONFIG::LEFT_MOT_RATIO / CONFIG::RIGHT_MOT_RATIO;

    moteus::PositionMode::Command rightCommand;
    rightCommand.position = std::numeric_limits<double>::quiet_NaN();
    rightCommand.velocity_limit = CONFIG::MAX_VEL;

    std::vector<moteus::CanFdFrame> commands_to_send;
    std::vector<moteus::CanFdFrame> replies_to_receive;

    std::cout << "Starting tracking loop... with delay "  << loop_delay_ms << "ms" << std::endl;

    leftMotor.SetStop();
    rightMotor.SetStop();

    // High-frequency control loop
    auto start_time = std::chrono::steady_clock::now();
    auto next_loop_time = start_time;

    // Variables for tracking displacement
    double robot_x_displacement = 0.0;

    double left_wheel_pos = 0.0;
    double right_wheel_pos = 0.0;
    
    double home_position_offset = 0.0;
    double current_position = 0;
    
    bool first_loop = true;

    while (true) {
        commands_to_send.clear();
        replies_to_receive.clear();

        // get current state
        moteus::BlockingCallback cbk;
        transport->Cycle(
            commands_to_send.data(), 
            commands_to_send.size(),
            &replies_to_receive, 
            &attitude, nullptr, nullptr,
            cbk.callback()
        );
        cbk.Wait();

        // 2. PARSE SENSORS
        quaternionToEuler(attitude.attitude, angles);

        if (first_loop) {
            home_position_offset = 0; // Set current spot as "Home = 0"
            current_position = 0;
        }

        robot_x_displacement = current_position - home_position_offset;

        // 3. SAFETY CHECK: If robot falls over, kill the motors
        if (std::abs(angles.pitch) > 60.0) {
            commands_to_send.push_back(leftMotor.MakeStop());
            commands_to_send.push_back(rightMotor.MakeStop());

            first_loop = true;
        }

        // ==============================================================================
        // 4. CASCADED PID LOGIC
        // ==============================================================================

        // OUTER LOOP (Position Controller)
        // Goal: Keep displacement at 0.0. 
        // Output: A "Target Pitch" to lean the robot toward home.
        double target_pitch = position_controller.calculate(0, robot_x_displacement);

        // Clamp max tilt
        target_pitch = clamp(target_pitch, -CONFIG::MAX_HOMING_TILT_ANGLE, CONFIG::MAX_HOMING_TILT_ANGLE);

        // INNER LOOP (Pitch Controller)
        double target_velocity = -pitch_controller.calculate(target_pitch, angles.pitch);

        // ==============================================================================

        // 5. COMMAND MOTORS
        leftCommand.velocity = target_velocity * CONFIG::LEFT_MOT_RATIO;
        rightCommand.velocity = target_velocity * CONFIG::RIGHT_MOT_RATIO;

        commands_to_send.push_back(leftMotor.MakePosition(leftCommand));
        commands_to_send.push_back(rightMotor.MakePosition(rightCommand));

        moteus::BlockingCallback cbk1;
        transport->Cycle(
            commands_to_send.data(), 
            commands_to_send.size(),
            &replies_to_receive, 
            &attitude, nullptr, nullptr,
            cbk1.callback()
        );
        cbk1.Wait();

        // Parse Moteus replies to get exactly how far the wheels have turned
        for (const auto& frame : replies_to_receive) {
            auto result = moteus::Query::Parse(frame.data, frame.size);
            if (frame.source == CONFIG::LEFT_MOT_ID) {
                left_wheel_pos = result.position; // Position in revolutions
            } else if (frame.source == CONFIG::RIGHT_MOT_ID) {
                right_wheel_pos = result.position; // Position in revolutions
            }
        }

        
        // Calculate average linear displacement (taking gear ratios/directions into account)
        // Note: You may need to invert one of these depending on your motor mounting orientation
        current_position = ((left_wheel_pos) * CONFIG::LEFT_MOT_RATIO + 
        (right_wheel_pos) * CONFIG::RIGHT_MOT_RATIO) / 2.0;
        
        if (first_loop) {
            home_position_offset = current_position;
            first_loop = false;
        }


        // 6. TERMINAL OUTPUT
        std::cout << "\r" << std::fixed << std::setprecision(2)
          << "  pitch: " << std::setw(6) << angles.pitch
          << "  tgt_pitch: " << std::setw(6) << target_pitch
          << "  disp_x: " << std::setw(6) << robot_x_displacement
          << "  vel_cmd: " << std::setw(6) << target_velocity
          << (first_loop ? "  control: Inactive" : "  control: Active")
          << "  " << std::flush;

        // 7. SLEEP UNTIL NEXT CYCLE
        next_loop_time += std::chrono::milliseconds(loop_delay_ms);
        std::this_thread::sleep_until(next_loop_time);
    }

    return 0;
}