#ifndef SERVO_CONTROLLER_HPP_
#define SERVO_CONTROLLER_HPP_

#include "soem_ros2/soem.h"

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#define EC_TIMEOUTMON 2000
#define NUMOFSERVO_DRIVE 2 

#define Control_Word 0x6040
#define Status_Word 0x6041
#define mod_op 0x6060
#define sync_ch_2 0x1C12
#define sync_ch_3 0x1C13
#define Tx_PDO1 0x1A00
#define Tx_PDO2 0x1A01
#define Tx_PDO3 0x1A02
#define Tx_PDO4 0x1A03
#define Rx_PDO1 0x1600
#define Rx_PDO2 0x1601
#define Rx_PDO3 0x1602
#define Rx_PDO4 0x1603

double torque_command[8] = {0,0,0,0,0,0,0,0};
double goal_pos[2] = {0, 0};

bool init_flag 			= false;
bool run_flag 			= false;
bool homing_flag 		= false;
bool check_param 		= false;
int check_param_result 	= 0;
int play_count = 0;

const int storage_size = 500;

char IOmap[4096];
int expectedWKC;
boolean needlf;
volatile int wkc;

int run=1;
int sys_ready=0;

int started[NUMOFSERVO_DRIVE]={0}, ServoState=0;
uint8 servo_ready=0, servo_prestate=0;
int32_t zeropos[NUMOFSERVO_DRIVE]={0};
double gt=0;

double sine_amp=11500, f=0.2, period;
int recv_fail_cnt=0;

uint8 currentgroup = 0;

//variables for pdo re-mapping (sdo write)
int os;
uint32_t ob;
uint16_t ob2;
uint8_t  ob3;
boolean inOP;

using namespace std::chrono_literals;

class ServoController : public rclcpp::Node
{
public:
    ServoController() : Node("ServoController")
    {
		RCLCPP_INFO(this->get_logger(), "[HEROEHS Linear Actuator Controller] is started");

        std::string interface = "eth0";    // 추후 yaml 파일에서 읽어오도록 수정

        ana1_pub        = this->create_publisher<std_msgs::msg::Float64>("analog1", 1);
  		ana2_pub        = this->create_publisher<std_msgs::msg::Float64>("analog2", 1);
		joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
		
		if (setup_ethercat(interface.c_str()))
		{
			RCLCPP_INFO(this->get_logger(), "Initialization succeeded");
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Initialization failed");
		}

		timer_statecheck 	= this->create_wall_timer(1000ms, std::bind(&ServoController::StateCheckCallback, this)); 
		timer_datacycle 	= this->create_wall_timer(1ms, std::bind(&ServoController::DataCycleCallback, this));
		joy_sub 			= this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&ServoController::JoyCallback, this, std::placeholders::_1));
		torque_command_sub 	= this->create_subscription<std_msgs::msg::Float64MultiArray>("torque_command", 1, std::bind(&ServoController::TorqueCommandCallback, this, std::placeholders::_1));
	}

    ~ServoController()
    {
		pdo_transfer_active = false;
		RCLCPP_INFO(this->get_logger(), "stop ethercat");
		/* request INIT state for all slaves */
		RCLCPP_INFO(this->get_logger(), "Request init state for all slaves");
		ec_slave[0].state = EC_STATE_INIT;
		ec_writestate(0);
		/* stop SOEM, close socket */
		ec_close();
    }

private:

    void StateCheckCallback();
    void DataCycleCallback();
	void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
	void TorqueCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr torque_cmd_msg);

	bool setup_ethercat(const char*);

	rclcpp::TimerBase::SharedPtr timer_statecheck;
	rclcpp::TimerBase::SharedPtr timer_datacycle;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ana1_pub;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ana2_pub;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;

	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_command_sub;

	volatile int expectedWKC;
	volatile int wkc;
	bool pdo_transfer_active = false;
};

#endif /* SERVO_CONTROLLER_HPP_ */
