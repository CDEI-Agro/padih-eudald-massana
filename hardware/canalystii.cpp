#include <iostream>
#include <cstring>       // For memset
#include "controlcan.hpp"  // Include the ControlCAN API
#include "canalystii.hpp"
#include "rclcpp/rclcpp.hpp"


namespace canalystii{


// Constructor: Initialize member variables
CANalystii::CANalystii(int vci_device_type, int vci_device_ind)
    : vci_device_type_(vci_device_type),
      vci_device_ind_(vci_device_ind) {
}

// Destructor: Cleanup resources if necessary
CANalystii::~CANalystii() {
    // You can add cleanup logic here, such as closing the device
        VCI_CloseDevice(vci_device_type_, vci_device_ind_);
        std::cout << "Device is being closed." << std::endl;
        // You can call a method to stop or close the device if necessary
    
}

// Function to initialize and start the CAN device
bool CANalystii::open_can_device(int channel_index) {
    // Step 1: Initialize CAN device
    if (VCI_OpenDevice(vci_device_type_, vci_device_ind_, 0) != 1) {
        return false;
    }
    return true;
}


bool CANalystii::init_can_device(VCI_INIT_CONFIG can_config){
        // Initialize the CAN device
    if (VCI_InitCAN(vci_device_type_, vci_device_ind_, 0, &can_config) != 1) { // Assuming channelIndex is 0
        VCI_CloseDevice(vci_device_type_, vci_device_ind_);
        return false;  // Return false on failure
    }
    return true;  // Return true if initialization is successful
}

bool CANalystii::start_can_device(int channel_index){
    if (VCI_StartCAN(vci_device_type_, vci_device_ind_, channel_index) != 1) {
        VCI_CloseDevice(vci_device_type_, vci_device_ind_);
        return false;
        }
    return true;
}

// Function to send CAN messages
void CANalystii::send_can_message(int channel_index, double left_wheel_vel, double right_wheel_vel) {

    // For debugginh

    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveAgribotHardware"), "left and right wheel velocities  %f %f", left_wheel_vel, right_wheel_vel);
    
    VCI_CAN_OBJ sendMsg;
    memset(&sendMsg, 0, sizeof(VCI_CAN_OBJ));

    sendMsg.ID = 0x123;  // CAN ID
    sendMsg.SendType = 0; // Normal send mode
    sendMsg.RemoteFlag = 0; // Data frame (not remote)
    sendMsg.ExternFlag = 0; // Standard frame
    sendMsg.DataLen = 8;   // 8 bytes of data

    // Example data to send (fill with whatever data you want to send)
    sendMsg.Data[0] = 0x11;
    sendMsg.Data[1] = 0x22;
    sendMsg.Data[2] = 0x33;
    sendMsg.Data[3] = 0x44;
    sendMsg.Data[4] = 0x55;
    sendMsg.Data[5] = 0x66;
    sendMsg.Data[6] = 0x77;
    sendMsg.Data[7] = 0x88;

    int result = VCI_Transmit(vci_device_type_, vci_device_ind_, channel_index, &sendMsg, 1);

    // For debugging
    // if (result == 1) {
    //     RCLCPP_INFO(rclcpp::get_logger("DiffDriveAgribotHardware"), "Message written");

    // } else {
    //     RCLCPP_INFO(rclcpp::get_logger("DiffDriveAgribotHardware"), "Message Failed");
    // }
}

// Function to receive CAN messages
void CANalystii::receive_can_message(int channelIndex) {
    VCI_CAN_OBJ recvMsg[2500];  // Array to store incoming messages
    int result = VCI_Receive(vci_device_type_, vci_device_ind_, channelIndex, recvMsg, 2500, 100);

    if (result > 0) {
        for (int i = 0; i < result; ++i) {
            std::cout << "Received CAN message with ID: 0x" << std::hex << recvMsg[i].ID << std::endl;
            std::cout << "Data: ";
            for (int j = 0; j < recvMsg[i].DataLen; ++j) {
                std::cout << "0x" << std::hex << (int)recvMsg[i].Data[j] << " ";
            }
            std::cout << std::endl;
        }
    } else if (result == 0) {
        std::cout << "No messages received." << std::endl;
    } else {
        std::cerr << "Failed to receive CAN message." << std::endl;
    }
}

}