#ifndef BUS_PIRATE
#define BUS_PIRATE

#define DEVICE_A 0x48
#define DEVICE_B 0x44

//Register Map
//Read Write Block
#define CMD_REG            0xCC
#define MODE_REG           0x00
#define MAX_CURRENT_L      0x01
#define MAX_CURRENT_H      0x02
#define CURRENT_DURATION_L 0x03
#define CURRENT_DURATION_H 0x04
#define DES_SPEED_A_L      0x05
#define DES_SPEED_A_H      0x06
#define DES_SPEED_B_L      0x07
#define DES_SPEED_B_H      0x08
#define DES_POS_A_L        0x09
#define DES_POS_A_H        0x0A
#define DES_POS_B_L        0x0B
#define DES_POS_B_H        0x0C

#define READONLY           0x20
/// Read only Registers
#define POSITION_A_L       0x20
#define POSITION_A_H       0x21
#define POSITION_B_L       0x22
#define POSITION_B_H       0x23
#define ADC_A_L            0x24
#define ADC_A_H            0x25
#define ADC_B_L            0x26
#define ADC_B_H            0x27
#define ADC_C_L            0x28
#define ADC_C_H            0x29
#define SWITCH_STATES      0x2A
#define REMEX_STATE        0x2F

#define DESIRED_SPEED_A DES_SPEED_A_L
#define DESIRED_SPEED_B DES_SPEED_B_L
#define DESIRED_POSITION_A DES_POS_A_L
#define DESIRED_POSITION_B DES_POS_B_L
#define CURRENT_DURATION CURRENT_DURATION_L 
#define MAX_CURRENT MAX_CURRENT_L 
#define POSITION_A POSITION_A_L
#define POSITION_B POSITION_B_L
#define ADC_A ADC_A_L
#define ADC_B ADC_B_L 
#define ADC_C ADC_C_L

/* Global includes */
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>

/* Local includes */
#include "rclcpp/rclcpp.hpp"

class BusPirate: public rclcpp::Node
{
public:
    BusPirate();

    ~BusPirate();

private:

    int startServer();

    void setMotorSpeedA(__u16 speed);

    void setMotorSpeedB(__u16 speed);

    void setDesiredPositionA(__u16 angle);

    __u16 getDesiredPositionA();

    __u8 getRemexStatus();

    __u8 getRemexSwitchStates();

    __u8 getRemexMode();

    __u16 readADCA();

    __u16 readADCB();

    __u16 readADCC();

    __u16 getMotorPositionA();

    __u16 getMotorPositionB();

    void pollDevices();

    int setTarget(__u8 slave_addr);

    // deprecate
    int i2cReadByte(__u8 reg);

    int i2cReadWord(__u8 reg);

    int checkStatus();

    int i2cWriteWord(__u8 reg, __u16 data);

    int i2cWriteByte(__u8 reg, __u8 data);

    int fd;

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<int> devices_;
};

#endif