#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>


void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}


void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{   
    std::string motor1 = sendMsg("1,getp\r");
    std::string motor2 = sendMsg("2,getp\r");

    std::string stripped_motor1 = motor1.substr(3);
    std::string stripped_motor2 = motor2.substr(3);
    
    val_1 = std::atoi(stripped_motor1.c_str());
    val_2 = std::atoi(stripped_motor2.c_str());
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


void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

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