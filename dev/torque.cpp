#include <moteus/moteus.h>
#include <hat/pi3hat_moteus_transport.h>
#include <iostream>
#include <unistd.h>
#include "constants.h"
#include "angles.cpp"
#include "PID.cpp"
#include <iomanip>
#include <cmath>

namespace moteus = mjbots::moteus;

int main() {
// 1. Configure the pi3hat transport options
    using Transport = mjbots::pi3hat::Pi3HatMoteusTransport;
    Transport::Options toptions;
    
    // Map motor ID 1 to JC3 (bus 3), and motor ID 2 to JC5 (bus 5)
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

    PID pitch_controller = PID(
        CONFIG::PITCH_KP,
        CONFIG::PITCH_KI,
        CONFIG::PITCH_KD,
        loop_delay_ms
    );

    PID yaw_controller = PID(
        CONFIG::YAW_KP,
        CONFIG::YAW_KI,
        CONFIG::YAW_KD,
        loop_delay_ms
    );

    // this doesnt
    moteus::Controller leftMotor([&transport]() {
        moteus::Controller::Options options;
        options.id = CONFIG::LEFT_MOT_ID;
        // options.position_format.kp_scale = mjbots::moteus::kFloat;
        // options.position_format.kd_scale = mjbots::moteus::kFloat;
        // options.position_format.ilimit_scale = mjbots::moteus::kFloat;
        // options.position_format.feedforward_torque = mjbots::moteus::kFloat;
        options.position_format.accel_limit = mjbots::moteus::kFloat;
        options.position_format.velocity_limit = mjbots::moteus::kFloat;
        options.transport = transport;
        return options;
    }());

    moteus::Controller rightMotor([&transport]() {
        moteus::Controller::Options options;
        options.id = CONFIG::RIGHT_MOT_ID;
        // options.position_format.kp_scale = mjbots::moteus::kFloat;
        // options.position_format.kd_scale = mjbots::moteus::kFloat;
        // options.position_format.ilimit_scale = mjbots::moteus::kFloat;
        // options.position_format.feedforward_torque = mjbots::moteus::kFloat;
        options.position_format.accel_limit = mjbots::moteus::kFloat;
        options.position_format.velocity_limit = mjbots::moteus::kFloat;
        options.transport = transport;
        return options;
    }());

    moteus::PositionMode::Command leftCommand;
    leftCommand.position = std::numeric_limits<double>::quiet_NaN();
    leftCommand.velocity = 1000;
    leftCommand.velocity_limit = CONFIG::MAX_VEL;
    

    moteus::PositionMode::Command rightCommand;
    rightCommand.position = std::numeric_limits<double>::quiet_NaN();
    leftCommand.velocity = 1000;
    rightCommand.velocity_limit = CONFIG::MAX_VEL;

    std::vector<moteus::CanFdFrame> commands_to_send;
    std::vector<moteus::CanFdFrame> replies_to_receive;

    std::cout << "Starting tracking loop... with delay "  << loop_delay_ms << std::endl;

    leftMotor.SetStop();
    rightMotor.SetStop();

    // 4. High-frequency control loop (Feeds the watchdog timer)
    auto start_time = std::chrono::steady_clock::now();
    auto next_loop_time = start_time;

    double commanded_alpha;
    while (true) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();

        commands_to_send.clear();
        replies_to_receive.clear();

        // if (elapsed_seconds > 12) {
        //     break;
        // }

        moteus::BlockingCallback cbk;
        transport->Cycle(
            commands_to_send.data(), 
            commands_to_send.size(),
            &replies_to_receive, 
            &attitude, nullptr, nullptr,
            cbk.callback()
        );

        quaternionToEuler(attitude.attitude, angles);

        cbk.Wait();

        if (true) {

            if (std::abs(angles.pitch) > 60) {
                commands_to_send.push_back(leftMotor.MakeStop());
                commands_to_send.push_back(rightMotor.MakeStop());
            }

            commanded_alpha = pitch_controller.calculate(0, angles.pitch);

            leftCommand.accel_limit = std::abs(commanded_alpha * CONFIG::LEFT_MOT_RATIO);
            leftCommand.velocity = commanded_alpha < 0 ? 1000 : -1000;

            rightCommand.accel_limit = std::abs(commanded_alpha * CONFIG::RIGHT_MOT_RATIO);
            rightCommand.velocity = commanded_alpha < 0 ? 1000 : -1000;

            commands_to_send.push_back(leftMotor.MakePosition(leftCommand));
            commands_to_send.push_back(rightMotor.MakePosition(rightCommand));
        }
        
        moteus::BlockingCallback cbk1;
        transport->Cycle(
            commands_to_send.data(), 
            commands_to_send.size(),
            &replies_to_receive, 
            &attitude, nullptr, nullptr,
            cbk1.callback()
        );

        cbk1.Wait();

        quaternionToEuler(attitude.attitude, angles);

        std::cout << "\r"
          << std::fixed << std::setprecision(2)
          << "yaw: "   << std::setw(8) << angles.yaw
          << "  pitch: " << std::setw(8) << angles.pitch
          << "  roll: "  << std::setw(8) << angles.roll
          << "  main: " << std::setw(5) << commanded_alpha
          << "  left: "  << std::setw(5) << leftCommand.accel_limit
          << "  right: "  << std::setw(5) << rightCommand.accel_limit
          << "  " // trailing spaces to clear any leftover characters
          << std::flush;

        
        next_loop_time += std::chrono::milliseconds(loop_delay_ms);
        std::this_thread::sleep_until(next_loop_time);
    }

    std::cout << "\n\nTime limit reached. Stopping motors..." << std::endl;

    return 0;
}