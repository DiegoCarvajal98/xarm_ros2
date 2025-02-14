#ifndef XARM_CONTROL_ESP_COMMS_HPP
#define XARM_CONTROL_ESP_COMMS_HPP

#include <memory>
#include <iostream>
#include <sstream>
#include <libserial/SerialPort.h>

class EspComms
{
public:
    EspComms() = default;

    /**
     * Connects to serial device
     * @param serial_device String with the serial device to be connected
     */
    void connect(std::string &serial_device)
    {
        //   timeout_ms_ = timeout_ms;
        serial_conn_.Open("/dev/ttyUSB0");
        serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_230400);

        return;
    }

    /**
     * Disconnect serial device
     */
    void disconnect()
    {
        serial_conn_.Close();
    }

    /**
     * Checks if the serial comunication is open
     * @returns True if connection is open or False is connection is closed
     */
    bool connected() const
    {
        return serial_conn_.IsOpen();
    }

    /**
     * Method to send messages through serial communication
     * @param msg_to_send: Message to be sent
     * @param print_output: If set to True, prints on the command line the
     * message sent and the message received.
     * @returns String with the response received from the controller
     */
    std::string send_msg(const std::string &msg_to_send, bool print_output = false)
    {
        serial_conn_.FlushIOBuffers();
        serial_conn_.Write(msg_to_send);

        // std::string command = "";
        std::string response = "";
        try
        {
            serial_conn_.ReadLine(response, '\r', 1000);
        }
        catch (const LibSerial::ReadTimeout &)
        {
            std::cerr << "The ReadByte() call has timed out." << std::endl;
        }

        if (print_output)
        {
            std::cout << "Sent: " << msg_to_send << " Received: " << response << std::endl;
        }

        return response;
    }

    void send_empty_msg()
    {
        std::string response = send_msg("\r");
    }

    /**
     * Read encoder values over Serial
     * @param val1: Pointer to store encoder 1 value ()
     * @param val2: Pointer to store encoder 2 value
     */
    bool read_encoder_values(double &pos1, double &pos2, double &pos3,
                             double &pos4, double &pos5, double &pos6)
    {
        std::string token = send_msg("e\r");

        double pos_arr[6] = {0.0};

        int split_ind = 0;
        std::string r_pose;
        std::string delimiter = " ";

        try
        {
            for (auto i = 0; i < 6; i++)
            {
                // Ensure delimiter is found to prevent invalid substr
                if (token.empty())
                {
                    std::cerr << "[ERROR] Received empty token!" << std::endl;
                    return false;
                }

                if (i < 5)
                    split_ind = token.find(delimiter);
                else
                    split_ind = token.find("\r");

                r_pose = token.substr(0, split_ind);
                token = token.substr(split_ind + 1);

                // Ensure that r_pose only contains numbers
                if (r_pose.find_first_not_of("0123456789.-") != std::string::npos)
                {
                    std::cerr << "[ERROR] Non-numeric data received: " << r_pose << std::endl;
                    return false;
                }

                // std::cout << r_pose << std::endl;
                pos_arr[i] = stod(r_pose);
            }

            pos1 = pos_arr[0];
            pos2 = pos_arr[1];
            pos3 = pos_arr[2];
            pos4 = pos_arr[3];
            pos5 = pos_arr[4];
            pos6 = pos_arr[5];
        }
        catch (const std::exception &e)
        {
            std::cerr << "[ERROR] Exemption during encoder parsing: " << e.what() << '\n';
        }

        return true;
    }

    /**
     * Send motor commands over Serial communication
     * @param msg: Array of motor command values
     */
    void set_values(int msg[])
    {
        std::stringstream ss;
        ss << "m";

        for (int i = 0; i < 7; i++)
        {
            ss << " " << msg[i];
        }

        ss << "\r";

        std::string token = send_msg(ss.str());

        // if (token == "ServoNotConnected\r")
        // {
        //     std::cerr << "Servomotors not connected (write)" << std::endl;
        // }
    }

private:
    int32_t timeout_ms_;
    LibSerial::SerialPort serial_conn_;
    int zero_pos[6] = {1680, 1100, 1200, 1175, 1225, 1175};
};

#endif // XARM_CONTROL_ESP_COMMS_HPP