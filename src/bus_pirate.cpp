#include "hardware/bus_pirate.hpp"
#include <vector>

extern "C" {
    #include <linux/i2c-dev.h>
    #include <linux/i2c.h>
    #include <i2c/smbus.h>
}

BusPirate::BusPirate() : Node("bus_pirate") {
    devices_ = {DEVICE_A, DEVICE_B};

    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open i2c bus");
        exit(1);
    }

    setTarget(DEVICE_B);

    i2cWriteByte(0x00, 0x44);
    
    i2cWriteByte(0x01, 0x55);

    int val = i2cReadWord(0x00);

    RCLCPP_INFO(this->get_logger(), "read val %x", val);

    //timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(BusPirate::pollDevices, this));
}

BusPirate::~BusPirate() {
    close(fd);
}

void BusPirate::pollDevices() {
    for (int dev : devices_) {
        setTarget(dev);
    }
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

int BusPirate::i2cWriteByte(__u8 reg, __u8 data) {
    int res = i2c_smbus_write_byte_data(fd, reg, data);
    return res;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BusPirate>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}