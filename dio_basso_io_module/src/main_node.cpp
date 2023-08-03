#include "rclcpp/rclcpp.hpp"
#include "gpio_interfaces/msg/digital.hpp"
#include "gpio_interfaces/msg/io_state.hpp"

#include <thread>
#include <mutex>

#include <fcntl.h>
#include "modbus/modbus.h"

class DIOBassoModbus: public rclcpp::Node
{
    public:
        DIOBassoModbus(): Node("dio_basso_modbus")
        {
            this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
            this->declare_parameter<int>("baudrate", 115200);
            this->declare_parameter<int>("id", 1);
            this->declare_parameter<double>("rate", 10.0);
            this->declare_parameter<uint16_t>("io_mode", 2);

            auto port_name = this->get_parameter("port_name").get_parameter_value().get<std::string>();
            auto baudrate = this->get_parameter("baudrate").get_parameter_value().get<long int>();
            auto id = this->get_parameter("id").get_parameter_value().get<int>();
            io_mode_ = this->get_parameter("io_mode").get_parameter_value().get<uint16_t>();

            RCLCPP_INFO(this->get_logger(), "Port: [%s] and baudrate %ld", port_name.c_str(), baudrate);

            modbus_ = modbus_new_rtu(port_name.c_str(), baudrate, 'N', 8, 1);
            assert(modbus_ != NULL);
            modbus_set_slave(modbus_, id);
            if(modbus_connect(modbus_) == -1)
            {
                modbus_free(modbus_);
                assert(false);
            }

            switch(io_mode_)
            {
                case 0:
                case 2:
                    sub_digital_out_ = this->create_subscription<gpio_interfaces::msg::Digital>(
                        "set_digital_output", 10, std::bind(&DIOBassoModbus::callback_digital_output, this, std::placeholders::_1));
                    break;
                default:
                    break;
            }
            pub_io_state_ = this->create_publisher<gpio_interfaces::msg::IOState>("dio_state", 10);

            uint16_t send_registers[1] = {io_mode_};
            modbus_write_registers(modbus_, 0, 1, send_registers);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            auto period = std::chrono::duration<double>(1.0 / this->get_parameter("rate").as_double());
            timer_ = this->create_wall_timer(period, std::bind(&DIOBassoModbus::timer_callback, this));
            RCLCPP_INFO(this->get_logger(), "Initialized...");

        }
        ~DIOBassoModbus()
        {
            modbus_free(modbus_);
        }

    private:
        void timer_callback()
        {
            // Read all digital state
            uint16_t read_registers[4] = {0, };

            m_.lock();
            modbus_read_input_registers(modbus_, 0, 4, read_registers);
            m_.unlock();

            auto msg = gpio_interfaces::msg::IOState();
            switch(io_mode_)
            {
                case 0:
                    msg.digital_in_state.resize(2, gpio_interfaces::msg::Digital());
                    msg.digital_out_state.resize(2, gpio_interfaces::msg::Digital());

                    msg.digital_in_state[0].pin = 1;
                    msg.digital_in_state[0].state = read_registers[0] == 0 ? false : true;
                    msg.digital_in_state[1].pin = 2;
                    msg.digital_in_state[1].state = read_registers[1] == 0 ? false : true;
                    msg.digital_out_state[0].pin = 3;
                    msg.digital_out_state[0].state = read_registers[2] == 0 ? false : true;
                    msg.digital_out_state[1].pin = 4;
                    msg.digital_out_state[1].state = read_registers[3] == 0 ? false : true;
                    break;
                case 1:
                    msg.digital_in_state.resize(4, gpio_interfaces::msg::Digital());

                    msg.digital_in_state[0].pin = 1;
                    msg.digital_in_state[0].state = read_registers[0] == 0 ? false : true;
                    msg.digital_in_state[1].pin = 2;
                    msg.digital_in_state[1].state = read_registers[1] == 0 ? false : true;
                    msg.digital_in_state[2].pin = 3;
                    msg.digital_in_state[2].state = read_registers[2] == 0 ? false : true;
                    msg.digital_in_state[3].pin = 4;
                    msg.digital_in_state[3].state = read_registers[3] == 0 ? false : true;
                    break;
                case 2:
                    msg.digital_out_state.resize(4, gpio_interfaces::msg::Digital());

                    msg.digital_out_state[0].pin = 1;
                    msg.digital_out_state[0].state = read_registers[0] == 0 ? false : true;
                    msg.digital_out_state[1].pin = 2;
                    msg.digital_out_state[1].state = read_registers[1] == 0 ? false : true;
                    msg.digital_out_state[2].pin = 3;
                    msg.digital_out_state[2].state = read_registers[2] == 0 ? false : true;
                    msg.digital_out_state[3].pin = 4;
                    msg.digital_out_state[3].state = read_registers[3] == 0 ? false : true;
                    break;
            }
            msg.header.stamp = this->get_clock()->now();
            pub_io_state_->publish(msg);
        }

        void callback_digital_output(const gpio_interfaces::msg::Digital::SharedPtr msg)
        {
            if(io_mode_ == 0 && msg->pin < 3)
            {
                return;
            }
            else if (io_mode_ == 1)
            {
                return;
            }

            int address = 2;
            switch(msg->pin)
            {
                case 1:
                    address = 2;
                    break;
                case 2:
                    address = 6;
                    break;
                case 3:
                    address = 10;
                    break;
                case 4:
                    address = 14;
                    break;
            }
            m_.lock();
            modbus_write_register(modbus_, address, (uint16_t)(msg->state ? 1 : 0));
            m_.unlock();
        }

    private:
        rclcpp::Publisher<gpio_interfaces::msg::IOState>::SharedPtr pub_io_state_;
        rclcpp::Subscription<gpio_interfaces::msg::Digital>::SharedPtr sub_digital_out_;
        uint16_t io_mode_;
        modbus_t *modbus_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::mutex m_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DIOBassoModbus>());
    rclcpp::shutdown();
    return 0;
}