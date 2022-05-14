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

int angle;
int command;

BusPirate::BusPirate() : Node("bus_pirate") {
    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open i2c bus");
        exit(1);
    }

    subscriber_ = this->create_subscription<nrmc_robot_interfaces::msg::ClientCmd>("client_cmd", 10,
                            std::bind(&BusPirate::callbackClientCmd, this, std::placeholders::_1));
    
    pidSetSubscriber_ = this->create_subscription<nrmc_robot_interfaces::msg::PIDSet>("pid_set", 10,
                            std::bind(&BusPirate::callbackPIDSet, this, std::placeholders::_1));
}

void BusPirate::callbackPIDSet(const nrmc_robot_interfaces::msg::PIDSet::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "%d %d %d", msg->i_div, msg->p_div, msg->d_div);
    if(this->target != ARM) {
        setTarget(ARM);
    }

    i2cWriteByte(PID_GAIN_MULT, (unsigned char) msg->p_mult);
    i2cWriteByte(PID_GAIN_DIV,  (unsigned char) msg->p_div);
    i2cWriteByte(PID_INT_MULT,  (unsigned char) msg->i_mult);
    i2cWriteByte(PID_INT_DIV,   (unsigned char) msg->i_div);
    i2cWriteByte(PID_DIF_MULT,  (unsigned char) msg->d_mult);
    i2cWriteByte(PID_DIF_DIV,   (unsigned char) msg->d_div);
    i2cWriteByte(0xCC, 5);
}

void BusPirate::callbackClientCmd(const nrmc_robot_interfaces::msg::ClientCmd::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "%d %x %d", msg->angle, msg->command, msg->max_speed);
    if(this->target != ARM) {
        setTarget(ARM);
    }
    setDesiredPositionA(msg->angle);
    setMotorSpeedA(msg->max_speed);
    i2cWriteByte(0xCC, msg->command);

    int desiredPos = getDesiredPositionA();
    int setSpeed = getMotorSpeedA();

    RCLCPP_INFO(this->get_logger(), "Read speed: %d, angle: %d", setSpeed, desiredPos);
}

void BusPirate::setMotorSpeedA(__u16 speed) {
    i2cWriteWord(DES_SPEED_A_L, speed);
}

void BusPirate::setMotorSpeedB(__u16 speed) {
    i2cWriteWord(DES_SPEED_B_L, speed);
}

void BusPirate::setDesiredPositionA(__u16 angle) {
    i2cWriteWord(DESIRED_POSITION_A, angle);
}

__u16 BusPirate::getDesiredPositionA() {
    return i2cReadWord(DESIRED_POSITION_A);
}

__u16 BusPirate::getMotorPositionA() {
    return i2cReadWord(POSITION_A_L);
}

__u16 BusPirate::getMotorSpeedA() {
    return i2cReadWord(DESIRED_SPEED_A);
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
    rclcpp::spin(node);
    rclcpp::shutdown();
}