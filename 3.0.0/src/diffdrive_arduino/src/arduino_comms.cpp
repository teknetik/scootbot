#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>

void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "INIT SERIAL");
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    // serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

        try 
    {
        serial_conn_.open();
    }
    catch(const serial::IOException& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "Unable to open port: %s", e.what());
        return;
    }
    
    if(serial_conn_.isOpen())
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Successfully connected to the serial port.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "Failed to open the serial port.");
    }

}


void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{   
    std::string motor1 = sendMsg("1,getp\r");
    std::string motor2 = sendMsg("2,getp\r");
    
    // Check the length of the strings and log an error if they are less than 4 characters.
    if (motor1.length() < 4 || motor2.length() < 4) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), 
                     "Received strings are too short. motor1: %s, motor2: %s", 
                     motor1.c_str(), motor2.c_str());
        return;
    }

    std::string stripped_motor1 = motor1.substr(3);
    std::string stripped_motor2 = motor2.substr(3);
    
    RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), 
                "Stripped strings are. stripped_motor1: %s, stripped_motor2: %s", 
                stripped_motor1.c_str(), stripped_motor2.c_str());
    
    val_1 = std::atoi(stripped_motor1.c_str());
    val_2 = std::atoi(stripped_motor2.c_str());
}


void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}



void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    std::stringstream ss;
    // LEFT MOTOR
    ss << "1,S" << val_1 << "\r";
    sendMsg(ss.str());

    ss.str(std::string()); // Clear stringstream
    ss.clear(); // Clear any error flags

    //RIGHT MOTOR
    ss << "2,S" << val_2 << "\r";
    sendMsg(ss.str());
}

void ArduinoComms::start()
{
    std::stringstream ss;
    
    ss << "1,START\r";
    sendMsg(ss.str());
    ss.str(std::string()); // Clear stringstream
    ss.clear(); // Clear any error flags
    ss << "2,START\r";
    sendMsg(ss.str());
}

//std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
std::string ArduinoComms::sendMsg(const std::string &msg_to_send)
{
    serial_conn_.write(msg_to_send);

    std::string response = serial_conn_.readline();


    RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), 
                "Sent: %s \n", msg_to_send.c_str());
    RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), 
                "Received: %s \n", response.c_str());

    

    return response;
}