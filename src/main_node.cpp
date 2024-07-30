#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "realtime_tools/realtime_publisher.h"
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <angles/angles.h>

using realtime_tools::RealtimePublisher;

class IMUTARSBSeriesNode: public rclcpp::Node
{
    public:
        IMUTARSBSeriesNode(): Node("imu_tars_b_node")
        {
            this->declare_parameter<std::string>("interface_name", "can0");
            this->declare_parameter<std::string>("prefix", "");
            this->declare_parameter<std::string>("frame_id", "imu_link");
            this->declare_parameter<uint8_t>("id", 226); // 0xE2 (Default ID)

            auto interface_name = this->get_parameter("interface_name").get_parameter_value().get<std::string>();
            prefix_ = this->get_parameter("prefix").get_parameter_value().get<std::string>();
            frame_id_ = this->get_parameter("frame_id").get_parameter_value().get<std::string>();
            imu_id_ = this->get_parameter("id").get_parameter_value().get<uint8_t>();

            RCLCPP_INFO(this->get_logger(), "Connect IMU via interface: [%s], CAN Id: [%d] prefix: [%s] and frame_id: [%s]",
                interface_name.c_str(), imu_id_, prefix_.c_str(), frame_id_.c_str());

            // SocketCAN Initialize
            struct ifreq ifr;
            struct sockaddr_can addr;

            memset(&ifr, 0, sizeof(ifr));
            memset(&addr, 0, sizeof(addr));

            can_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
            strcpy(ifr.ifr_name, interface_name.c_str());
            ioctl(can_sock_, SIOCGIFINDEX, &ifr);

            timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 2000;  // 2ms
            setsockopt(can_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            addr.can_ifindex = ifr.ifr_ifindex;
            addr.can_family = PF_CAN;

            if (bind(can_sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "SocketCAN Bind Error.");
                assert(true);
            }
            RCLCPP_INFO(this->get_logger(), "SocketCAN Initialized...");

            // ROS Related
            auto pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", rclcpp::SystemDefaultsQoS());
            rt_pub_imu_ = std::make_shared<RealtimePublisher<sensor_msgs::msg::Imu>>(pub_imu);


            // Create Thread...
            thread_recv_data_ = std::make_shared<std::thread>(std::bind(&IMUTARSBSeriesNode::thread_recv_data_func, this));
            RCLCPP_INFO(this->get_logger(), "Initialized...");
        }

    private:
        void thread_recv_data_func(void)
        {
            RCLCPP_INFO(this->get_logger(), "Wait for first CAN frame...");
            struct can_frame recv_frame;
            while(rclcpp::ok())
            {
                auto ret = ::read(can_sock_, &recv_frame, sizeof(struct can_frame));
                if(ret <= 0)
                    continue;

                if((recv_frame.can_id & 0xFF) == imu_id_)
                {
                    if(((recv_frame.can_id >> 8) & 0xFF) == 0x2D)
                    {
                        RCLCPP_INFO(this->get_logger(), "PGN 61485 Received.");
                        break;
                    }
                }

                rclcpp::sleep_for(std::chrono::milliseconds(2));
            }

            RCLCPP_INFO(this->get_logger(), "Start receive thread...");

            double pitch_angle = 0.0;
            double roll_angle = 0.0;
            double yaw_angle = 0.0;
            double gyro_x = 0.0;
            double gyro_y = 0.0;
            double gyro_z = 0.0;
            double linear_x = 0.0;
            double linear_y = 0.0;
            double linear_z = 0.0;

            while(rclcpp::ok())
            {
                auto ret = ::read(can_sock_, &recv_frame, sizeof(struct can_frame));
                if(ret <= 0)
                    continue;

                if((recv_frame.can_id & 0xFF) == imu_id_)
                {
                    RCLCPP_DEBUG(this->get_logger(), "%X", (recv_frame.can_id >> 8) & 0xFF);

                    if(((recv_frame.can_id >> 8) & 0xFF) == 0x29) // PGN 61481 (0xF029) PITCH AND ROLL BROADCAST DATA
                    {
                        uint32_t pitch_raw = (recv_frame.data[2] << 16) + (recv_frame.data[1] << 8) + recv_frame.data[0];
                        double pitch_deg = (pitch_raw - 8192000) / 32768.0;
                        pitch_angle = angles::from_degrees(pitch_deg);

                        uint32_t roll_raw = (recv_frame.data[5] << 16) + (recv_frame.data[4] << 8) + recv_frame.data[3];
                        double roll_deg = (roll_raw - 8192000) / 32768.0;
                        roll_angle = angles::from_degrees(roll_deg);
                    }
                    else if(((recv_frame.can_id >> 8) & 0xFF) == 0x2A) // PGN 61482 (0xF02A) ANGULAR RATE BROADCAST DATA
                    {
                        uint16_t pitch_raw = (recv_frame.data[1] << 8) + recv_frame.data[0];
                        double pitch_deg = (pitch_raw - 32000) / 128.0;
                        gyro_y = angles::from_degrees(pitch_deg);

                        uint16_t roll_raw = (recv_frame.data[3] << 8) + recv_frame.data[2];
                        double roll_deg = (roll_raw - 32000) / 128.0;
                        gyro_x = angles::from_degrees(roll_deg);

                        uint16_t yaw_raw = (recv_frame.data[5] << 8) + recv_frame.data[4];
                        double yaw_deg = (yaw_raw - 32000) / 128.0;
                        gyro_z = angles::from_degrees(yaw_deg);

                    }
                    else if(((recv_frame.can_id >> 8) & 0xFF) == 0x2D) // PGN 61485 (0xF02D) ACCELERATION BROADCAST DATA
                    {
                        uint16_t lateral = (recv_frame.data[1] << 8) + recv_frame.data[0];
                        linear_y = (lateral - 32000) / 100.0;

                        uint16_t longitudinal = (recv_frame.data[3] << 8) + recv_frame.data[2];
                        linear_x = (longitudinal - 32000) / 100.0;

                        uint16_t vertical = (recv_frame.data[5] << 8) + recv_frame.data[4];
                        linear_z = (vertical - 32000) / 100.0;

                        // Publish IMU Topic
                        rt_pub_imu_->trylock();

                        rt_pub_imu_->msg_.header.frame_id = prefix_ + "/" + frame_id_;
                        rt_pub_imu_->msg_.header.stamp = this->now();

                        tf2::Quaternion q;
                        q.setRPY(roll_angle, pitch_angle, yaw_angle);

                        rt_pub_imu_->msg_.orientation.x = q.getX();
                        rt_pub_imu_->msg_.orientation.y = q.getY();
                        rt_pub_imu_->msg_.orientation.z = q.getZ();
                        rt_pub_imu_->msg_.orientation.w = q.getW();

                        rt_pub_imu_->msg_.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

                        rt_pub_imu_->msg_.angular_velocity.x = gyro_x;
                        rt_pub_imu_->msg_.angular_velocity.y = gyro_y;
                        rt_pub_imu_->msg_.angular_velocity.z = gyro_z;

                        rt_pub_imu_->msg_.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

                        rt_pub_imu_->msg_.linear_acceleration.x = linear_x;
                        rt_pub_imu_->msg_.linear_acceleration.y = linear_y;
                        rt_pub_imu_->msg_.linear_acceleration.z = linear_z;

                        rt_pub_imu_->msg_.linear_acceleration_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

                        rt_pub_imu_->unlockAndPublish();
                    }
                }
            }
        }

    private:
        // std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> pub_imu_;
        std::shared_ptr<RealtimePublisher<sensor_msgs::msg::Imu>> rt_pub_imu_;
        std::shared_ptr<std::thread> thread_recv_data_;

        int can_sock_;
        std::string prefix_;
        std::string frame_id_;
        uint8_t imu_id_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUTARSBSeriesNode>());
    rclcpp::shutdown();
    return 0;
}