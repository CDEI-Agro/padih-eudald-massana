#ifndef PLC_COMMUNICATION_HPP_
#define PLC_COMMUNICATION_HPP_

#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <array>
#include <thread>
#include <chrono>
#include <bitset>
#include "wheel.hpp"

namespace plc_communication
{
    typedef struct
    {
        uint8_t feedback_mode;          // 0: joint feedback 1: plataform-twist feedback
        float stamp;                    // Seconds since startup
        float platform_vel_X;           // Vehicle velocity X axis [m/s]
        float platform_vel_y;           // Vehicle velocity Y axis [m/s]
        float platform_vel_z;           // Vehicle velocity Z axis [rad/s]
        float encoder_vel_l;            // Left wheel encoder velocity [rad/s]
        float encoder_vel_r;            // Right wheel encoder velocity [rad/s]
        float encoder_vel_t;            // Turret encoder velocity [rad/s]
        float platform_pos_angular;     // Vehicle absolute angular position [rad]
        float encoder_ticks_r;          // Encoder ticks for right motor
        float encoder_ticks_l;          // Encoder ticks for left motor
        float encoder_ticks_turret;     // Encoder ticks for turret motor
        uint16_t status_word_l;         // Left wheel status word (CiA DSP-402)
        uint16_t status_word_r;         // Right wheel status word (CiA DSP-402)
        uint16_t status_word_t;         // Turret status word (CiA DSP-402)
        uint8_t shutdown_feed_back;     // 0: SD order not received, 1: SD order received

    } Data_PLC_to_PC; // Data struct to receive from the PLC

    typedef struct
    {
        uint8_t command_mode;           // 0: Joint command 1: Platform-twist command
        uint8_t interpolation;          // 0: Disabled 1: Enabled
        uint32_t sequence;              // Sequence index. Starts at 0, then increases
        float platform_cmd_vel_x;       // Velocity command X axis [m/s]
        float platform_cmd_vel_y;       // Velocity command Y axis [m/s]
        float platform_cmd_vel_z;       // Velocity command Z axis [rad/s]
        float encoder_cmd_vel_l;        // Left wheel rotational velocity command [rad/s]
        float encoder_cmd_vel_r;        // Right wheel rotational velocity command [rad/s]
        float encoder_cmd_vel_t;        // Turret rotational velocity command [rad/s]
        uint16_t control_word_l;        // Left wheel control word (CiA DSP-402)
        uint16_t control_word_r;        // Right wheel control word (CiA DSP-402)
        uint16_t control_word_t;        // Turret control word (CiA DSP-402)
        uint8_t shutdown_order;         // 0: Regular operation, 1: Request shutdown

    } Data_PC_to_PLC; // Data struct to send to the PLC

    /// @brief The class to handle communications between PLC and PC
    class PLCCommunication
    {
    public:
        PLCCommunication();
        ~PLCCommunication();

        /// @brief It reads few necessary communication params and save them internally.
        /// @param plc_port Port of the PLC
        /// @param plc_ip IP of the PLC
        /// @param pc_port Port of the PC
        /// @param pc_ip IP of the PC
        void init(int plc_port, std::string plc_ip, int pc_port, std::string pc_ip);

        /// @return If there exist a successful connection to PLC.
        bool connected() const;

        /// @brief establish connection to PLC and set param connected_ to true if successful.
        void connect();

        /// @brief Drop connection to PLC and set param connected_ to false if successful.
        void disconnect();

        /// @brief Initialize all three motor drivers using the enable state machine.
        void initialize_drivers();

        /// @brief Deactivate/disable drivers
        void deactivate_drivers();

        /// @brief Shutdown PLC
        void shutdown_drivers();

        /// @brief Send write commands to motors
        void send_motor_commands(double cmd_left_wheel, double cmd_right_wheel, double cmd_turret);

        motor_states::MotorStates read_motor_states();

    private:
        // Parameters for the PLC communication
        int PLC_PORT_;
        int PC_PORT_;
        std::string PLC_IP_;
        std::string PC_IP_;
        int delay_time_; // When sending data to PLC, the minimum number of milliseconds between frames

        // Variables
        bool connected_; //bool to check if socket is connected to the PLC
        int socket_fd_;
        uint32_t sequence_counter_; // This is the sequence number of the message to send. Always increasing.
        struct sockaddr_in PLC_addr_;
        struct sockaddr_in client_addr_;
        socklen_t clientaddr_len_ = sizeof(client_addr_);

        // Functions
        void send_data_to_plc(Data_PC_to_PLC &data);
        void receive_data_from_plc(Data_PLC_to_PC &received_data);
        void drivers_state_machine(uint16_t &control_word, const uint16_t &status_word, int &step);
    };

} // End of plc_communication namespace

#endif