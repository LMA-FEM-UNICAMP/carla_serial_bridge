
#include <chrono>
#include <functional>
#include <memory>
#include <cstring>

#include "rclcpp/rclcpp.hpp"

#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "carla_serial_bridge/serial_com.hpp"

// Driving mode
#define EMERGENCY 0
#define AUTOWARE 1
#define MANUAL 4

#define PORT "/dev/ttyUSB0"
#define BAUDRATE B921600
#define RX_POOLING_RATE 20ms

typedef union{
  float f;
  unsigned char bytes[4];
} float_bytes;

typedef struct{
  float_bytes fLongSpeed;
  float_bytes fLatSpeed;
  float_bytes fHeadingRate;
  float_bytes fSteeringStatus;
} vehicle_status;

using namespace std::chrono_literals;

using std::placeholders::_1;

class CarlaSerialBridge : public rclcpp::Node{
    public:

    CarlaSerialBridge() : Node("carla_serial_brigde", "microautoware"){

        vehicle_control_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/carla/ego_vehicle/ackermann_cmd", 1);

        vehicle_manual_control_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 1); // This 10 is QoS?

        velocity_status_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>("/microautoware/vehicle/status/velocity_status", 
                                                                                                1, std::bind(&CarlaSerialBridge::velocity_status_sub_callback, this, _1));

        steering_status_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>("/microautoware/vehicle/status/steering_status", 
                                                                                                1, std::bind(&CarlaSerialBridge::steering_status_sub_callback, this, _1));

        clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", 1, std::bind(&CarlaSerialBridge::clock_sub_callback, this, _1));

        timer_ = this->create_wall_timer(RX_POOLING_RATE, std::bind(&CarlaSerialBridge::timer_callback, this));

        bVelocityData = 0;
        bSteeringData = 0;

    }

    private:

    void send_to_microautoware(){

        char tx_msg[23];
        sprintf(tx_msg, "#A%c%c%c%cB%c%c%c%cC%c%c%c%cD%c%c%c%c$",
                vehicle_status_tx.fLongSpeed.bytes[0], vehicle_status_tx.fLongSpeed.bytes[1], vehicle_status_tx.fLongSpeed.bytes[2], vehicle_status_tx.fLongSpeed.bytes[3], 
                vehicle_status_tx.fLatSpeed.bytes[0], vehicle_status_tx.fLatSpeed.bytes[1], vehicle_status_tx.fLatSpeed.bytes[2], vehicle_status_tx.fLatSpeed.bytes[3],
                vehicle_status_tx.fHeadingRate.bytes[0], vehicle_status_tx.fHeadingRate.bytes[1], vehicle_status_tx.fHeadingRate.bytes[2], vehicle_status_tx.fHeadingRate.bytes[3],
                vehicle_status_tx.fSteeringStatus.bytes[0], vehicle_status_tx.fSteeringStatus.bytes[1], vehicle_status_tx.fSteeringStatus.bytes[2], vehicle_status_tx.fSteeringStatus.bytes[3]);
        
        serial_com_link.writeSerialPort(tx_msg); 

        bVelocityData = 0;
        bSteeringData = 0;
        ucControlMode = 0;
    }

    void publish_to_carla(){

        if((AUTOWARE == ucControlMode) || (EMERGENCY == ucControlMode)){

            auto vehicle_control_msg_ = ackermann_msgs::msg::AckermannDrive();

            vehicle_control_msg_.steering_angle = xSteeringAngle_Control.f;
            vehicle_control_msg_.steering_angle_velocity = xSteeringAngleVelocity_Control.f;
            vehicle_control_msg_.speed = xSpeed_Control.f;
            vehicle_control_msg_.acceleration = xAcceleration_Control.f;
            vehicle_control_msg_.jerk = xJerk_Control.f;
        
            vehicle_control_pub_->publish(vehicle_control_msg_);

        }
        else if(MANUAL == ucControlMode){

            auto vehicle_control_msg_ = carla_msgs::msg::CarlaEgoVehicleControl();

            vehicle_control_msg_.throttle = xSpeed_Control.f;
            vehicle_control_msg_.steer = xSteeringAngle_Control.f;
            vehicle_control_msg_.brake = xAcceleration_Control.f;
            vehicle_control_msg_.hand_brake = 0;
            vehicle_control_msg_.reverse = 0;
            vehicle_control_msg_.manual_gear_shift = 0;
            vehicle_control_msg_.gear = 1;

            vehicle_control_msg_.header.stamp = clock;
        
            vehicle_manual_control_pub_->publish(vehicle_control_msg_);
        }
    }

    void clock_sub_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg){
        clock = msg->clock;
    }

    void steering_status_sub_callback(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg) {
        vehicle_status_tx.fSteeringStatus.f = msg->steering_tire_angle;

        bSteeringData = 1;

        if(bSteeringData && bVelocityData){
            send_to_microautoware();
        }
    }

    void velocity_status_sub_callback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
        vehicle_status_tx.fHeadingRate.f = msg->heading_rate;
        vehicle_status_tx.fLongSpeed.f = msg->longitudinal_velocity;
        vehicle_status_tx.fLatSpeed.f = msg->lateral_velocity;

        bVelocityData = 1;

        if(bSteeringData && bVelocityData){
            send_to_microautoware();
        }
    }

    void timer_callback(){

        char sm_state = 0;
        
        char * rx_msg; 
        
        rx_msg = serial_com_link.readSerialPort();

        for(int i = 0; i < 64; i++){
            
            switch (sm_state){

                case 0:  
                    if(rx_msg[i] == '#'){   
                        sm_state = 1;
                    }
                    break;
                
                case 1:
                    switch (rx_msg[i]){
                        case 'M':
                            sm_state = 20;                 
                            break;

                        case 'S':
                            sm_state = 30;                 
                            break;

                        case 'W':
                            sm_state = 40;                 
                            break;

                        case 'V':
                            sm_state = 50;                 
                            break;

                        case 'A':
                            sm_state = 60;                 
                            break;

                        case 'J':
                            sm_state = 70;                   
                            break; 

                        case '$':
                            sm_state = 0;   
                            this->publish_to_carla();
                            std::cout << "S: " << xSteeringAngle_Control.f << std::endl
                                      << "W: " << xSteeringAngleVelocity_Control.f << std::endl
                                      << "V: " << xSpeed_Control.f << std::endl
                                      << "A: " << xAcceleration_Control.f << std::endl
                                      << "J: " << xJerk_Control.f << std::endl
                                      << "--------------------------" << std::endl;     
                            break;
                        
                        default:
                            sm_state = 0;       
                            break;
                    }
                    break;        
                
                case 20:
                    ucControlMode = rx_msg[i];
                    sm_state = 1;                 
                    break;     
                
                case 30:
                    xSteeringAngle_Control.bytes[0] = rx_msg[i];
                    sm_state = 31;                 
                    break;        
                
                case 31:
                    xSteeringAngle_Control.bytes[1] = rx_msg[i];
                    sm_state = 32;                 
                    break;
                
                
                case 32:
                    xSteeringAngle_Control.bytes[2] = rx_msg[i];
                    sm_state = 33;                 
                    break;        
                
                case 33:
                    xSteeringAngle_Control.bytes[3] = rx_msg[i];
                    sm_state = 1;                 
                    break;        
                
                case 40:
                    xSteeringAngleVelocity_Control.bytes[0] = rx_msg[i];
                    sm_state = 41;                 
                    break;        
                
                case 41:
                    xSteeringAngleVelocity_Control.bytes[1] = rx_msg[i];
                    sm_state = 42;                 
                    break;        
                
                case 42:
                    xSteeringAngleVelocity_Control.bytes[2] = rx_msg[i];
                    sm_state = 43;                 
                    break;        
                
                case 43:
                    xSteeringAngleVelocity_Control.bytes[3] = rx_msg[i];
                    sm_state = 1;                 
                    break;        
                
                case 50:
                    xSpeed_Control.bytes[0] = rx_msg[i];
                    sm_state = 51;                 
                    break;        
                
                case 51:
                    xSpeed_Control.bytes[1] = rx_msg[i];
                    sm_state = 52;                 
                    break;        
                
                case 52:
                    xSpeed_Control.bytes[2] = rx_msg[i];
                    sm_state = 53;                 
                    break;        
                
                case 53:
                    xSpeed_Control.bytes[3] = rx_msg[i];
                    sm_state = 1;                 
                    break;        
                
                case 60:
                    xAcceleration_Control.bytes[0] = rx_msg[i];
                    sm_state = 61;                 
                    break;        
                
                case 61:
                    xAcceleration_Control.bytes[1] = rx_msg[i];
                    sm_state = 62;                 
                    break;        
                
                case 62:
                    xAcceleration_Control.bytes[2] = rx_msg[i];
                    sm_state = 63;                 
                    break;        
                
                case 63:
                    xAcceleration_Control.bytes[3] = rx_msg[i];
                    sm_state = 1;                 
                    break;      
                
                case 70:
                    xJerk_Control.bytes[0] = rx_msg[i];
                    sm_state = 71;                 
                    break;        
                
                case 71:
                    xJerk_Control.bytes[1] = rx_msg[i];
                    sm_state = 72;                 
                    break;        
                
                case 72:
                    xJerk_Control.bytes[2] = rx_msg[i];
                    sm_state = 73;                 
                    break;        
                
                case 73:
                    xJerk_Control.bytes[3] = rx_msg[i];
                    sm_state = 1;                 
                    break;
                
                default:
                    sm_state = 0;       
                    break;
            }   
            
        }
      
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr vehicle_control_pub_;

    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr vehicle_manual_control_pub_;

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;

    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_sub_;

    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_sub_;

    __uint32_t loop_rate;
    __uint32_t baudrate;
    char * port_name;
    

    float_bytes xSteeringAngle_Control;
    float_bytes xSteeringAngleVelocity_Control;
    float_bytes xSpeed_Control;
    float_bytes xAcceleration_Control;
    float_bytes xJerk_Control;

    unsigned char ucControlMode;

    bool bSteeringData;
    bool bVelocityData;

    
    vehicle_status vehicle_status_tx;

    rclcpp::Time clock;

    SerialCom serial_com_link{PORT, BAUDRATE};
};


int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarlaSerialBridge>());
    rclcpp::shutdown();
    return 0;
}
