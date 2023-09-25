#include <iostream>
#include <string>
#include <cstdlib>
#include <sstream>

void moveArmToPosition(const std::string& command) {
    std::string positionCommand, speedStr;
    std::istringstream iss(command);
    long speed;
    iss >> positionCommand >> speedStr;
    if(!speedStr.empty()) {
        speed = std::atoi(speedStr.c_str());
        std::cout << "Moving to position: " << positionCommand << ", with speed " << speed << " ms." << std::endl;
    } else {
        std::cout << "Moving to position: " << positionCommand << std::endl;

    }

}

void moveJointToDegree(const std::string& command) {
    int jointNumber, degrees;
    std::string interval;
    std::istringstream iss(command);
    std::cout << command << std::endl;
    iss >> jointNumber >> degrees >> interval;
    if(!interval.empty()) {
        std::cout << "Moving joint " << jointNumber << " to " << degrees << " degrees with interval " << interval << " ms." << std::endl;
    } else {
        std::cout << "Moving joint " << jointNumber << " to " << degrees << " degrees" << std::endl;
    }
}

int main() {
    std::cout << "Robot interface, Kars en Mitchel 2023" << std::endl;
   std::cout << "\nSelect an option:\n";
        std::cout << "1. Move the arm to a fixed position (P/R/I/U followed by an optional speed in ms)(Type '1', followed by a enter, after that type the command.)\n";
        std::cout << "2. Move a specific joint to a certain degree (joint number, degrees, and interval)(Type '2',  followed by a enter, after that type the command.)\n";
        std::cout << "3. Emergency stop(Type '3')\n";
        std::cout << "4. Exit (Type '4')\n";
    while (true) {
    
        std::string userInput;
        std::cin >> userInput;

        if (userInput == "1") {
            std::string command;
            std::cout << "Enter position command: " << std::endl;
            std::cin.ignore();
            std::getline(std::cin, command);
            moveArmToPosition(command);
            //place feedback here
        } else if (userInput == "2") {
            std::string command;
            std::cout << "Enter joint command: " << std::endl;
            std::cin.ignore();
            std::getline(std::cin, command);
            moveJointToDegree(command);
            //place feedback here
        } else if (userInput == "3") {
            std::cout << "Emergency stop initiated." << std::endl;
            //place feedback here
        } else if (userInput == "4") {
            std::cout << "Exiting program." << std::endl;
            break;
        }
    }

    return 0;
}
