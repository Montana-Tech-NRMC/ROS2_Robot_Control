#ifndef BUS_PIRATE
#define BUS_PIRATE

#define DEVICE_A 0x48
#define DEVICE_B 0x44

//Register Map
//Read Write Block
#define CMD_REG            0xCC
#define MODE_REG           0x00
#define MAX_CURRENT_H      0x01
#define MAX_CURRENT_L      0x02
#define CURRENT_DURATION_H 0x03
#define CURRENT_DURATION_L 0x04
#define DES_SPEED_A_H      0x05
#define DES_SPEED_A_L      0x06
#define DES_SPEED_B_H      0x07
#define DES_SPEED_B_L      0x08
#define DES_POS_A_H        0x09
#define DES_POS_A_L        0x0A
#define DES_POS_B_H        0x0B
#define DES_POS_B_L        0x0C

#define READONLY           0x20
/// Read only Registers
#define POSITION_A_H       0x20
#define POSITION_A_L       0x21
#define POSITION_B_H       0x22
#define POSITION_B_L       0x23
#define ADC_A_H            0x24
#define ADC_A_L            0x25
#define ADC_B_H            0x26
#define ADC_B_L            0x27
#define ADC_C_H            0x28
#define ADC_C_L            0x29
#define SWITCH_STATES      0x2A
#define REMEX_STATE        0x2F


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
    void pollDevices();

    int i2cReadByte(__u8 reg);

    int i2cReadWord(__u8 reg);

    int checkStatus();

    int i2cWriteByte(__u8 reg, __u8 data);

    int setTarget(__u8 slave_addr);

    int fd;

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<int> devices_;
};

#endif