#include <memory>
#include <libserial/SerialPort.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("test_serial")
    {
        connect();

        if (connected()) RCLCPP_INFO(this->get_logger(), "Connected");

        state = 0;

        timer = this->create_wall_timer(std::chrono::seconds(1),
                                        std::bind(&MinimalSubscriber::timer_callback, this));
    }

    void timer_callback()
    {
        // RCLCPP_INFO(this->get_logger(), "received message");

        if (state == 0)
        {
            set_values("475 1100 1200 1175 1225 1175");
            RCLCPP_INFO(this->get_logger(), "Position 1");
            state++;
        }
        else if (state == 1)
        {
            read_encoder_values();
            // RCLCPP_INFO(this->get_logger(), "Led OFF");
            state++;
        }
        else if (state == 2)
        {
            set_values("1680 1300 1300 1300 1300 2075");
            RCLCPP_INFO(this->get_logger(), "Position 2");
            state++;
        }
        else if (state == 3)
        {
            read_encoder_values();
            // RCLCPP_INFO(this->get_logger(), "Led OFF");
            state = 0;
        }
    }

    void connect()
    {
        //   timeout_ms_ = timeout_ms;
        serial_conn_.Open("/dev/ttyUSB0");
        serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

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
            serial_conn_.ReadLine(response, '\r', 1000);
        }
        catch (const LibSerial::ReadTimeout &)
        {
            RCLCPP_ERROR(this->get_logger(), "The ReadByte() call has timed out.");
        }

        if (print_output)
        {
            RCLCPP_INFO(this->get_logger(), response.c_str());
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
        std::string token = send_msg("e\r", true);

        int split_ind = 0;
        std::string r_pose;
        std::string delimiter = " ";

        for (int i = 0; i < 6; i++){
            split_ind = token.find(delimiter);
            r_pose = token.substr(0,split_ind);
            token = token.substr(split_ind + 1);
            RCLCPP_INFO(this->get_logger(), r_pose.c_str());
        }
    }

    /**
     * Send motor commands over Serial communication
     * @param val_1: Velocity command for motor 1
     * @param val_2: Velocity command for motor 2
     */
    void set_values(std::string msg)
    {
        std::stringstream ss;
        ss << "m" << " " << msg << "\r";

        send_msg(ss.str(), true);
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    int32_t timeout_ms_;
    LibSerial::SerialPort serial_conn_;
    rclcpp::TimerBase::SharedPtr timer;
    int state;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}