#include "include/diffdrive_agribot/plc_communication.hpp"

namespace plc_communication
{

    PLCCommunication::PLCCommunication()
    {
    }

    PLCCommunication::~PLCCommunication()
    {
    }

    void PLCCommunication::init(int plc_port, std::string plc_ip, int pc_port, std::string pc_ip)
    {
        PLC_PORT_ = plc_port;
        PLC_IP_ = plc_ip;
        PC_PORT_ = pc_port;
        PC_IP_ = pc_ip;
        connected_ = false;
        delay_time_ = 4;
        sequence_counter_ = 0;
    }

    bool PLCCommunication::connected() const
    {
        return connected_;
    }

    void PLCCommunication::connect()
    {
        // Create a UDP socket
        if ((socket_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
        {
            perror("Failed to create socket");
            std::exit(EXIT_FAILURE);
        }

        // Set up PLC address information
        PLC_addr_.sin_family = AF_INET;
        PLC_addr_.sin_port = htons(PLC_PORT_);
        PLC_addr_.sin_addr.s_addr = inet_addr(PLC_IP_.c_str());

        // Set up local address information
        struct sockaddr_in local_addr;
        std::memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(PC_PORT_);
        local_addr.sin_addr.s_addr = inet_addr(PC_IP_.c_str());

        // Bind the socket to the specified local address
        if (bind(socket_fd_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0)
        {
            perror("Failed to bind socket");
            close(socket_fd_);
            std::exit(EXIT_FAILURE);
        }

        connected_ = true;
    }

    void PLCCommunication::disconnect()
    {
        close(socket_fd_);
        connected_ = false;
    }

    void PLCCommunication::initialize_drivers()
    {

        for (int driver = 0; driver < 3; driver++){

            // Create empty struct, where received data from PLC will be stored
            Data_PLC_to_PC rcv_data;
            receive_data_from_plc(rcv_data);

            // Add delay
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_time_));

            uint16_t control_word; // initialize control word
            uint16_t status_word; // initialize status word  

            int step = 0;
            int iter = 0, max_iter = 50;
        
            while (step != 5 and iter < max_iter){
                
                // initialize new struct to send data each time
                // if outside the while loop it will send different control words to other motors
                Data_PC_to_PLC data_to_send; 
                send_data_to_plc(data_to_send); // send all zeros to plc
                std::this_thread::sleep_for(std::chrono::milliseconds(delay_time_)); //wait for some time (usually 4ms)
                receive_data_from_plc(rcv_data); //receive data from plc
                
                // Read status word depending on which driver is being dealt with
                switch (driver){
                    case 0:
                        status_word = rcv_data.status_word_l;
                        break;
                    case 1:
                        status_word = rcv_data.status_word_r;
                        break;
                    case 2:
                        status_word = rcv_data.status_word_t;
                        break;
                }
                drivers_state_machine(control_word, status_word, step); //run state machine step to enable drivers
                
                // Send control word depending on which driver is being dealt with
                switch (driver){
                    case 0:
                        data_to_send.control_word_l = control_word;
                        break;
                    case 1:
                        data_to_send.control_word_r = control_word;
                        break;
                    case 2:
                        data_to_send.control_word_t = control_word;
                        break;
                }                
                
                iter++; //increment iter until max iterations. Don't want to get stuck in
                        // an infinite loop
            }
        }

    }

    void PLCCommunication::deactivate_drivers()
    {

    }

    void PLCCommunication::shutdown_drivers()
    {
        Data_PC_to_PLC shutdown_order_data;
        shutdown_order_data.shutdown_order = 1; 
        send_data_to_plc(shutdown_order_data);
        // TODO:  should return shutdown feedback: success or failure 
    }

    void PLCCommunication::send_motor_commands(double cmd_left_wheel, double cmd_right_wheel, double cmd_turret)
    {
        Data_PC_to_PLC motor_command_data;

        motor_command_data.command_mode = 0; // 0: joint commands, 1: platform commands
        motor_command_data.interpolation = 0;
        motor_command_data.shutdown_order = 0;

        // Control words (I think it is 15, which is 0xF, but I am not sure)
        motor_command_data.control_word_l = 15;
        motor_command_data.control_word_r = 15;
        motor_command_data.control_word_t = 15;

        // Platform commands (not used)
        motor_command_data.platform_cmd_vel_x = 0;
        motor_command_data.platform_cmd_vel_y = 0;
        motor_command_data.platform_cmd_vel_z = 0;

        // Joint commands
        motor_command_data.encoder_cmd_vel_l = cmd_left_wheel;
        motor_command_data.encoder_cmd_vel_r = cmd_right_wheel;
        motor_command_data.encoder_cmd_vel_t = cmd_turret;

        // Send data
        send_data_to_plc(motor_command_data);
    }

    motor_states::MotorStates PLCCommunication::read_motor_states()
    {
        Data_PLC_to_PC rcv_data;
        receive_data_from_plc(rcv_data);
        motor_states::MotorStates motor_states;
        motor_states.motor_vel_l = rcv_data.encoder_vel_l;
        motor_states.motor_vel_r = rcv_data.encoder_vel_r;
        motor_states.turret_vel = rcv_data.encoder_vel_t;
        motor_states.encoder_ticks_r = rcv_data.encoder_ticks_r;
        motor_states.encoder_ticks_l = rcv_data.encoder_ticks_l;
        motor_states.encoder_ticks_turret = rcv_data.encoder_ticks_turret;



        return motor_states;
    }

    ///////////////////////////////////
    // Private functions
    ///////////////////////////////////

    // Function to send data to the PLC
    void PLCCommunication::send_data_to_plc(Data_PC_to_PLC &data)
    {
        data.sequence = sequence_counter_;
        char bytes_buffer[37];
        std::memcpy(bytes_buffer + 0, &data.command_mode, 1);
        std::memcpy(bytes_buffer + 1, &data.interpolation, 1);
        std::memcpy(bytes_buffer + 2, &data.sequence, 4);
        std::memcpy(bytes_buffer + 6, &data.platform_cmd_vel_x, 4);
        std::memcpy(bytes_buffer + 10, &data.platform_cmd_vel_y, 4);
        std::memcpy(bytes_buffer + 14, &data.platform_cmd_vel_z, 4);
        std::memcpy(bytes_buffer + 18, &data.encoder_cmd_vel_r, 4);
        std::memcpy(bytes_buffer + 22, &data.encoder_cmd_vel_l, 4);
        std::memcpy(bytes_buffer + 26, &data.encoder_cmd_vel_t, 4);
        std::memcpy(bytes_buffer + 30, &data.control_word_r, 2);
        std::memcpy(bytes_buffer + 32, &data.control_word_l, 2);
        std::memcpy(bytes_buffer + 34, &data.control_word_t, 2);
        std::memcpy(bytes_buffer + 36, &data.shutdown_order, 1);
        
        ssize_t bytes_sent = sendto(socket_fd_, bytes_buffer, sizeof(bytes_buffer), 0, (struct sockaddr *)&PLC_addr_, sizeof(PLC_addr_));

        if (bytes_sent == -1){
         perror("Error sending data");   
        }
        sequence_counter_ ++; //increase sequence counter by 1 after every send call
    }

    // Function to receive data from the PLC
    void PLCCommunication::receive_data_from_plc(Data_PLC_to_PC &received_data)
    {

        char bytes_buffer[53]; // initiazlize a character array: the first index returns a pointer to the memory location 
        ssize_t bytes_read = recvfrom(socket_fd_, bytes_buffer, sizeof(bytes_buffer), 0, (struct sockaddr *)&client_addr_, &clientaddr_len_);

        // copy buffer data to our struct
        /*
        format:
            memcpy(destination , source index, number of bytes to copy )
        */
        std::memcpy(&received_data.feedback_mode, bytes_buffer + 0, 1);
        std::memcpy(&received_data.stamp, bytes_buffer + 2, 4);
        std::memcpy(&received_data.platform_vel_X, bytes_buffer + 6, 4);
        std::memcpy(&received_data.platform_vel_y, bytes_buffer + 10, 4);
        std::memcpy(&received_data.platform_vel_z, bytes_buffer + 14, 4);
        std::memcpy(&received_data.encoder_vel_l, bytes_buffer + 18, 4);
        std::memcpy(&received_data.encoder_vel_r, bytes_buffer + 22, 4);
        std::memcpy(&received_data.encoder_vel_t, bytes_buffer + 26, 4);
        std::memcpy(&received_data.platform_pos_angular, bytes_buffer + 30, 4);
        std::memcpy(&received_data.encoder_ticks_l, bytes_buffer + 34, 4);
        std::memcpy(&received_data.encoder_ticks_r, bytes_buffer + 38, 4);
        std::memcpy(&received_data.encoder_ticks_turret, bytes_buffer + 42, 4);
        std::memcpy(&received_data.status_word_l, bytes_buffer + 46, 2);
        std::memcpy(&received_data.status_word_r, bytes_buffer + 48, 2);
        std::memcpy(&received_data.status_word_t, bytes_buffer + 50, 2);
        std::memcpy(&received_data.shutdown_feed_back, bytes_buffer + 52, 1);

        if (bytes_read < 0)
        {
            perror("Error receiving data");
        }
    }

    void PLCCommunication::drivers_state_machine(uint16_t &control_word, const uint16_t &status_word, int &step)
    {
        // Convert to bitset
        std::bitset<16> status_word_bits(status_word);
        std::bitset<16> control_word_bits;

        switch (step)
        {
        case 0: // Initial step of procedure: checking fault

            control_word = 0;

            if (status_word_bits[3]) // Check bit 3 for fault
            {
                step = 1; // There is fault, so reset fault
            }
            else
            {
                step = 2; // No fault, starting enable procedure
            }
            break;

        case 1: // Driver in fault

            control_word_bits.set(7, true); // Bit 7 active
            control_word = static_cast<uint16_t>(control_word_bits.to_ulong());

            if (!status_word_bits[3]) // Check bit 3 for fault
            {
                step = 2; // Fault has been reset, starting enable procedure
            }
            break;

        case 2: // Starting enable procedure

            control_word = 6;

            if (status_word_bits[0]) // Check bit 0
            {
                step = 3; // Ready to Switch ON
            }
            break;

        case 3: // Switch ON

            control_word = 7;

            if (status_word_bits[1]) // Check bit 1
            {
                step = 4; // Switched ON
            }
            break;

        case 4: // Operation Enable

            control_word = 15; // It is the same as 0xF;

            if (status_word_bits[2]) // Check bit 2
            {
                step = 5; // Operation enable al driver correcte
            }
            break;
        }
    }

} // End namespace plc_communication