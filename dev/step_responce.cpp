#include <string>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <fstream>
#include <iomanip>

// Moteus and pi3hat C++ headers
#include "moteus/moteus.h"
#include "hat/pi3hat_moteus_transport.h"

struct LogData {
    double timestamp;
    double target_torque;
    double m1_position;
    double m1_velocity;
    double m1_torque;
    double m2_position;
    double m2_velocity;
    double m2_torque;
};

int main(int argc, char* argv[]) {
    // 1. Configure the pi3hat transport
    using Transport = mjbots::pi3hat::Pi3HatMoteusTransport;
    Transport::Options toptions;
    
    toptions.servo_map[1] = 5; 
    toptions.servo_map[2] = 3; 
    
    auto transport = std::make_shared<Transport>(toptions);

    // 2. Initialize the moteus controllers
    mjbots::moteus::Controller::Options coptions1, coptions2;
    coptions1.id = 1; coptions1.transport = transport;
    // coptions1.position_format.kp_scale = mjbots::moteus::kFloat;
    // coptions1.position_format.kd_scale = mjbots::moteus::kFloat;
    // coptions1.position_format.ilimit_scale = mjbots::moteus::kFloat;
    // coptions1.position_format.feedforward_torque = mjbots::moteus::kFloat;
    coptions1.position_format.accel_limit = mjbots::moteus::kFloat;
    coptions1.position_format.velocity_limit = mjbots::moteus::kFloat;

    coptions2.id = 2; coptions2.transport = transport;
    // coptions2.position_format.kp_scale = mjbots::moteus::kFloat;
    // coptions2.position_format.kd_scale = mjbots::moteus::kFloat;
    // coptions2.position_format.ilimit_scale = mjbots::moteus::kFloat;
    // coptions2.position_format.feedforward_torque = mjbots::moteus::kFloat;
    coptions2.position_format.accel_limit = mjbots::moteus::kFloat;
    coptions2.position_format.velocity_limit = mjbots::moteus::kFloat;
    
    mjbots::moteus::Controller controller1(coptions1);
    mjbots::moteus::Controller controller2(coptions2);

    // 3. Setup Commands for PURE TORQUE mode
    mjbots::moteus::PositionMode::Command cmd;
    mjbots::moteus::PositionMode::Command cmd1;
    mjbots::moteus::PositionMode::Command cmd2;

    cmd.position = std::numeric_limits<double>::quiet_NaN();
    cmd.velocity = 0.0;

    // To command pure torque, we disable the PID loop by zeroing the gains
    // and setting the target position to NaN.
    cmd1.position = std::numeric_limits<double>::quiet_NaN();
    cmd1.velocity = 1000;

    cmd1.accel_limit = 30;
    cmd1.velocity_limit = 30;

    // cmd1.kp_scale = 0.0; // Disable proportional position gain
    // cmd1.kd_scale = 0.0; // Disable derivative velocity gain
    // cmd1.feedforward_torque = 0.1;

    // 4. Pre-allocate memory for logging
    const int LOOP_RATE_HZ = 1000;
    const int TEST_DURATION_SEC = 3; // Reduced to 3 seconds for safety on torque tests
    std::vector<LogData> data_log;
    data_log.reserve(LOOP_RATE_HZ * TEST_DURATION_SEC);

    std::cout << "Starting Torque Step Test at 500Hz..." << std::endl;
    std::cout << "WARNING: Motors will accelerate continuously if unloaded!" << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(2)); // Give the user 2 seconds to read the warning

    
    // Stop to clear faults
    controller1.SetStop();
    controller2.SetStop();

    auto start_time = std::chrono::steady_clock::now();
    auto next_loop_time = start_time;

    // std::this_thread::sleep_for(std::chrono::milliseconds(50));


    // 5. High-frequency Data Collection Loop
    while (true) {
        auto now = std::chrono::steady_clock::now();
        double elapsed_seconds = std::chrono::duration<double>(now - start_time).count();

        // Exit condition
        if (elapsed_seconds >= TEST_DURATION_SEC) {
            break;
        }

        // STEP INPUT LOGIC:
        // Hold 0.0 Nm for the first 0.5 seconds, then jump to 0.2 Nm
        // Modify '0.2' to your desired torque in Newton-meters (Nm)
        double target_torque = (elapsed_seconds < 0.5) ? 0.0 : 15;
        mjbots::moteus::PositionMode::Command desired_cmd = (elapsed_seconds < 0.5) ? cmd : cmd1;

        // Send commands
        auto result1 = controller1.SetPosition(desired_cmd);
        auto result2 = controller2.SetPosition(desired_cmd);

        // Log the results
        if (result1 && result2) {
            data_log.push_back({
                elapsed_seconds,
                target_torque,
                result1->values.position,
                result1->values.velocity,
                result1->values.torque,
                result2->values.position,
                result2->values.velocity,
                result2->values.torque
            });
        }

        // Wait precisely for the next 2ms tick (500 Hz)
        next_loop_time += std::chrono::microseconds(1000000 / LOOP_RATE_HZ);
        std::this_thread::sleep_until(next_loop_time);
    }

    // 6. Test complete, IMMEDIATELY de-energize motors
    controller1.SetStop();
    controller2.SetStop();
    std::cout << "Test complete. Motors safely stopped." << std::endl;

    // 7. Write the recorded data to a CSV file
    std::string filename = "torque_response.csv";
    std::cout << "Writing " << data_log.size() << " points to " << filename << "..." << std::endl;

    std::ofstream outfile(filename);
    if (outfile.is_open()) {
        // Write CSV Header
        outfile << "Time(s),alpha(rots/s^2),M1_Pos,M1_Vel,M1_Torque,M2_Pos,M2_Vel,M2_Torque\n";
        
        // Ensure standard formatting
        outfile << std::fixed << std::setprecision(6);

        // Write all recorded rows
        for (const auto& row : data_log) {
            outfile << row.timestamp << ","
                    << row.target_torque << ","
                    << row.m1_position << ","
                    << row.m1_velocity << ","
                    << row.m1_torque << ","
                    << row.m2_position << ","
                    << row.m2_velocity << ","
                    << row.m2_torque << "\n";
        }
        outfile.close();
        std::cout << "File saved successfully." << std::endl;
    } else {
        std::cerr << "Error: Could not open " << filename << " for writing." << std::endl;
    }

    return 0;
}