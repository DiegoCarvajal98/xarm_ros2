#include <memory>
#include <libserial/SerialPort.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("test_serial")
    {
        connect();

        if (connected())
            RCLCPP_INFO(this->get_logger(), "Connected");

        state = 0;

        subscription_ = this->create_subscription<std_msgs::msg::String>("control_pos", 10,
                                                                         std::bind(&MinimalSubscriber::subscription_callback, this, _1));
    }

    void subscription_callback(std_msgs::msg::String::SharedPtr msg)
    {
        std::string data = msg->data;

        int pos[6];
        int idx;

        std::string ss;

        for (int i = 0; i < 7; i++)
        {
            idx = data.find(" ");
            pos[i] = stoi(data.substr(0, idx));
            data = data.substr(idx + 1);

            if (i < 6)
                pos[i] = pos[i] * 100;
        }

        pos[5] = zero_pos[5] - pos[5] * 1.25;
        pos[4] = zero_pos[4] + pos[4];
        pos[3] = zero_pos[3] + pos[3];
        pos[2] = zero_pos[2] + pos[2];
        pos[1] = zero_pos[1] + pos[1];
        pos[0] = zero_pos[0] + pos[0];

        read_encoder_values();

        set_values(pos);
    }

    void connect()
    {
        //   timeout_ms_ = timeout_ms;
        serial_conn_.Open("/dev/ttyUSB0");
        serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_230400);

        return;
    }

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
            // serial_conn_.ReadLine(command, '\r', 1000);
            // RCLCPP_INFO(this->get_logger(), command.c_str());
            serial_conn_.ReadLine(response, '\r', 1000);
            // RCLCPP_INFO(this->get_logger(), response.c_str());
        }
        catch (const LibSerial::ReadTimeout &)
        {
            RCLCPP_ERROR(this->get_logger(), "The ReadByte() call has timed out.");
        }

        if (print_output)
        {
            // RCLCPP_INFO(this->get_logger(), response.c_str());
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
    void read_encoder_values()
    {
        std::string token = send_msg("e\r");

        int split_ind = 0;
        std::string r_pose;
        std::string delimiter = " ";

        int pos = 0.0;

        // RCLCPP_INFO(this->get_logger(), token.c_str());

        if (token != "ServoNotConnected\r")
        {
            for (int i = 0; i < 6; i++)
            {
                split_ind = token.find(delimiter);
                r_pose = token.substr(0, split_ind);
                token = token.substr(split_ind + 1);
                pos = stoi(r_pose);
                RCLCPP_INFO(this->get_logger(), "%d", pos);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Servomotors not connected");
        }
    }

    /**
     * Send motor commands over Serial communication
     * @param msg: Array of motor command values
     */
    void set_values(int msg[])
    {
        std::stringstream ss;
        ss << "m";

        for (int i = 0; i < 6; i++)
        {
            ss << " " << msg[i];
        }

        ss << "\r";

        std::string token = send_msg(ss.str());

        // RCLCPP_INFO(this->get_logger(), "%s", token);

        if (token == "ServoNotConnected\r")
        {
            RCLCPP_ERROR(this->get_logger(), "Servomotors not connected");

            return;
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    int32_t timeout_ms_;
    LibSerial::SerialPort serial_conn_;
    int state;
    int zero_pos[6] = {11750, 12250, 11750, 12000, 11000, 16800};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}