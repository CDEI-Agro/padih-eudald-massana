#ifndef CANALYSTII_H
#define CANALYSTII_H
#include "controlcan.hpp"

// CANalyst-ii is can-usb device class, which offers
// start_device, close_device, init_can_interface,
// receive_can_frame, send_can_frame methods. By
// using the provided libcontrol.so 
// Simple usage:
// CANalystii can_device;
// can_device.start_device();
// can_device.init_can_interface();
// can_device.receive_can_frame();//listening from CAN bus
// can_device.send_can_frame();//send to CAN bus

namespace canalystii{
class CANalystii{
public:
    CANalystii(int vci_device_type=4, int vci_device_ind=0);
    ~CANalystii();
    // start device with default parameters, and return status
    bool open_can_device( int channel_index);
    // Initialize can device
    bool init_can_device(VCI_INIT_CONFIG can_config);
    // start can device
    bool start_can_device(int channel_index);
    // send message to can device
    void send_can_message(int channel_index, double left_wheel_vel, double right_wheel_vel);
    // receive can message
    void receive_can_message(int channel_index);

private:
    unsigned int vci_device_type_ = 4;
    unsigned int vci_device_ind_ = 0;
};

}

#endif