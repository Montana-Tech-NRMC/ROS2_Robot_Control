#include "hardware/bus_pirate.hpp"
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

extern "C" {
    #include <linux/i2c-dev.h>
    #include <linux/i2c.h>
    #include <i2c/smbus.h>
    #include <netinet/in.h>
    #include <sys/socket.h>
    #include <arpa/inet.h>
}

int BusPirate::startServer() {
    return 0;
}

int angle;
int command;

BusPirate::BusPirate() : Node("bus_pirate") {
    devices_ = {DEVICE_A, DEVICE_B};

    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open i2c bus");
        exit(1);
    }
    setTarget(DEVICE_B);

    setDesiredPositionA(angle);

    RCLCPP_INFO(this->get_logger(), "set desired position to %d", angle);

    unsigned int despos = getDesiredPositionA();

    RCLCPP_INFO(this->get_logger(), "des pos read %d", despos);

    RCLCPP_INFO(this->get_logger(), "command sent %x", command);

    i2cWriteByte(0xCC, command);

/*
    unsigned int val = i2cReadWord(DESIRED_SPEED_A);

    RCLCPP_INFO(this->get_logger(), "read val %d", val);
    int position = getMotorPositionA();

    RCLCPP_INFO(this->get_logger(), "read position %d", position);

*/
}

void BusPirate::setMotorSpeedA(__u16 speed) {
    i2cWriteWord(DES_SPEED_A_L, speed);
}

void BusPirate::setMotorSpeedB(__u16 speed) {
    i2cWriteWord(DES_SPEED_B_L, speed);
}

void BusPirate::setDesiredPositionA(__u16 angle) {
    i2cWriteWord(DESIRED_POSITION_A, angle);
    //return i2cReadWord(DESIRED_POSITION_A);
}

__u16 BusPirate::getDesiredPositionA() {
    return i2cReadWord(DESIRED_POSITION_A);
}

__u16 BusPirate::getMotorPositionA() {
    return i2cReadWord(POSITION_A_L);
}


BusPirate::~BusPirate() {
    close(fd);
}

void BusPirate::pollDevices() {
    int current = i2cReadWord(ADC_A_L);
    RCLCPP_INFO(this->get_logger(), "(%d)", current); 
}

int BusPirate::checkStatus() {
    return 1;
}

int BusPirate::setTarget(__u8 slave_addr) {
    if (ioctl(fd, I2C_SLAVE, slave_addr) < 0) {
        return 1;
    }
    return 0;
}

int BusPirate::i2cReadByte(__u8 reg) {
    int res = i2c_smbus_read_byte_data(fd, reg);
    return res;
}

int BusPirate::i2cReadWord(__u8 reg) {
    int res = i2c_smbus_read_word_data(fd, reg);
    return res;
}

int BusPirate::i2cWriteWord(__u8 reg, __u16 data) {
    return i2c_smbus_write_word_data(fd, reg, data);
}

int BusPirate::i2cWriteByte(__u8 reg, __u8 data) {
    int res = i2c_smbus_write_byte_data(fd, reg, data);
    return res;
}

int main(int argc, char** argv) {
    if (argc > 2) {
        std::stringstream command_ss(argv[1]);
        std::stringstream angle_ss(argv[2]);
        angle_ss >> angle;
        command_ss >> command;
    } else if (argc > 1) {
        std::stringstream command_ss(argv[1]);
        command_ss >> command;
        angle = 0;
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BusPirate>();
    std::cout << angle << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
}