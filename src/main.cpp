#include <moteus/moteus.h>
#include "constants.h"
#include <iostream>
#include <unistd.h>

namespace moteus = mjbots::moteus;

int main() {
    std::cout << "Initing the project!" << std::endl;

    auto transport = moteus::Controller::MakeSingletonTransport({});
    moteus::Controller leftMotor([&]() {
        moteus::Controller::Options options;
        options.transport = transport;
        options.id = config::LEFT_MOT_ID;
        return options;
    }());
    moteus::Controller rightMotor([&]() {
        moteus::Controller::Options options;
        options.transport = transport;
        options.id = config::RIGHT_MOT_ID;
        return options;
    }());

    moteus::PositionMode::Command leftMove;
    leftMove.position = 1;
    moteus::PositionMode::Command rightMove;
    rightMove.position = -1;

    // home command
    moteus::PositionMode::Command moveHome;
    moveHome.position = 0;

    while (true) {
        leftMotor.SetPosition(leftMove);
        rightMotor.SetPosition(rightMove);
        usleep(1000000);

        leftMotor.SetPosition(moveHome);
        rightMotor.SetPosition(moveHome);

        usleep(1000000);
    }

    return 0;
}