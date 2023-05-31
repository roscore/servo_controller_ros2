#include <servo_controller.hpp>

using namespace std::chrono_literals;

void ServoController::StateCheckCallback()
{    
	int slave;
	uint8 currentgroup = 0;

	if (pdo_transfer_active && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
	{
		/* one ore more slaves are not responding */
		ec_group[currentgroup].docheckstate = FALSE;
		ec_readstate();
		for (slave = 1; slave <= ec_slavecount; slave++)
		{
			if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
			{
				ec_group[currentgroup].docheckstate = TRUE;
				if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
				{
					RCLCPP_ERROR(this->get_logger(), "ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.", slave);
					ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
					ec_writestate(slave);
				}
				else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
				{
					RCLCPP_WARN(this->get_logger(), "slave %d is in SAFE_OP, change to OPERATIONAL.", slave);
					ec_slave[slave].state = EC_STATE_OPERATIONAL;
					ec_writestate(slave);
				}
				else if (ec_slave[slave].state > 0)
				{
					if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
					{
						ec_slave[slave].islost = FALSE;
						RCLCPP_INFO(this->get_logger(), "MESSAGE : slave %d reconfigured", slave);
					}
				}
				else if (!ec_slave[slave].islost)
				{
					/* re-check state */
					ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
					if (!ec_slave[slave].state)
					{
						ec_slave[slave].islost = TRUE;
						RCLCPP_ERROR(this->get_logger(), "slave %d lost", slave);
					}
				}
			}
			if (ec_slave[slave].islost)
			{
				if (!ec_slave[slave].state)
				{
					if (ec_recover_slave(slave, EC_TIMEOUTMON))
					{
						ec_slave[slave].islost = FALSE;
						RCLCPP_INFO(this->get_logger(), "MESSAGE : slave %d recovered", slave);
					}
				}
				else
				{
					ec_slave[slave].islost = FALSE;
					RCLCPP_INFO(this->get_logger(), "MESSAGE : slave %d found", slave);
				}
			}
		}
		if (!ec_group[currentgroup].docheckstate)
		{
			RCLCPP_INFO(this->get_logger(), "OK : all slaves resumed OPERATIONAL.");
		}
	}

	// Extract current thread
	//auto curr_thread = std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id()));
	//auto info_message = "\n<<statecheck THREAD " + curr_thread + ">> ";
	//RCLCPP_INFO(this->get_logger(), info_message);
};

void ServoController::DataCycleCallback()
{
	if (pdo_transfer_active)
	{
		ec_send_processdata();
		wkc = ec_receive_processdata(EC_TIMEOUTRET);

		std_msgs::msg::Float64 msg;
		float analog;
		uint16_t analog_aid;
		uint8_t * shared_mem_ptr = ec_slave[3].inputs;
		for (int j=0; j<2; j++) 
		{
			analog_aid = *(shared_mem_ptr + 3 + j*4);
			analog_aid = analog_aid << 8; // opschuiven naar eerste 8 bits
			analog_aid = analog_aid + *(shared_mem_ptr + 2 + j*4); // LS 8 bits erbij
			if ((analog_aid >> 15) & 1) // check MSB als 1 dan negatief getal en 'goed' zetten middels two-complements
			{
				analog = (float) ( (analog_aid ^ ((2<<15)-1)) + 1) / -32767.0 * 10.0;
				// gebruik ^ ((2<<15)-1) om inverse te krijgen, want ~analog_aid werkt niet
			}
			else
			{
				analog = (float) (analog_aid) / 32767.0 * 10.0;
			}
			msg.data = analog;
			if (j==0)
			{
				ana1_pub->publish(msg);
			}
			else
			{
				ana2_pub->publish(msg);
			}
		}
	}

	// Extract current thread
	//auto curr_thread = std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id()));
	//auto info_message = "\n<<datacycle THREAD " + curr_thread + ">> ";
	//RCLCPP_INFO(this->get_logger(), info_message);
};

static int ServoWrite8 (uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
   int wkc;

   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                      EC_TIMEOUTRXM);
   return wkc;
}

static int ServoWrite16 (uint16 slave, uint16 index, uint8 subindex, uint16 value)
{
   int wkc;

   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                      EC_TIMEOUTRXM);
   return wkc;
}

static int ServoWrite32 (uint16 slave, uint16 index, uint8 subindex, uint32 value)
{
   int wkc;

   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                      EC_TIMEOUTRXM);
   return wkc;
}

static int ServoSetup(uint16 slave)
{
  int wkc = 0;

  printf ("HERoEHS Servo Drive Setup\n");

  wkc += ServoWrite8 (slave, 0x1C12, 0, 0);
  wkc += ServoWrite8 (slave, 0x1C13, 0, 0);

  wkc += ServoWrite8  (slave, 0x1A00, 0, 0);
  wkc += ServoWrite32 (slave, 0x1A00, 1, 0x60410010);
  wkc += ServoWrite32 (slave, 0x1A00, 2, 0x60640020);
  wkc += ServoWrite8  (slave, 0x1A00, 0, 2);

  wkc += ServoWrite8  (slave, 0x1600, 0, 0);
  wkc += ServoWrite32 (slave, 0x1600, 1, 0x60400010);
  wkc += ServoWrite32 (slave, 0x1600, 2, 0x607A0020);
  wkc += ServoWrite8  (slave, 0x1600, 0, 2);

  wkc += ServoWrite16 (slave, 0x1C12, 1, 0x1600);
  wkc += ServoWrite16 (slave, 0x1C12, 0, 1);

  wkc += ServoWrite16 (slave, 0x1C13, 1, 0x1A00);
  wkc += ServoWrite16 (slave, 0x1C13, 0, 1);

  /* Explicitly set flags that are (probably) invalid in EEPROM */
  //  ec_slave[slave].SM[2].SMflags = 0x10024;

  /* Explicitly disable sync managers that are activated by EEPROM */
  //  ec_slave[slave].SM[4].StartAddr = 0;
  //  ec_slave[slave].SM[5].StartAddr = 0;

  /* Set a slave name */
  strncpy (ec_slave[slave].name, "SERVO", EC_MAXNAME);

  printf("ServoSetup: %d\n", wkc);


  if (wkc != 14)
  {
    printf ("Servo Setup Failed\n");
    return -1;
  }

  return 0;
}

void CheckMotorInitStatus(void)
{
	for(int iter = 0; iter < NUMOFSERVO_DRIVE; iter++)
	{
		uint16_t kind_of_motor;
		os=sizeof(kind_of_motor); kind_of_motor = 0x00;
		ec_SDOread(iter+1, 0x6402,0x00, FALSE, &os, &kind_of_motor, EC_TIMEOUTRXM); //read status of driver

		if(kind_of_motor == 2)  check_param_result++;
		else                    printf("%d Moter Status is wrong!!! \n", iter+1);
	}
}

void Homing(void)
{
	int i, oloop, iloop, k, wkc_count;	

	for(int k=0; k < NUMOFSERVO_DRIVE; ++k)
	{
		os=sizeof(ob2); ob2 = 0x06;	//Shutdown
		wkc_count=ec_SDOwrite(k+1, 0x6040,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave operation mode
		printf("Control Word: %d\n", ob2);

		os=sizeof(ob2); ob2 = 0x0f;	//Set Enable Operation
		wkc_count=ec_SDOwrite(k+1, 0x6040,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave operation mode
		printf("Control Word: %d\n", ob2);


		os=sizeof(ob2); ob2 = 0x0B;	//Set HM mode origin -> 0x06
		wkc_count=ec_SDOwrite(k+1, 0x6060,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave operation mode
		printf("Control Word: %d\n", ob2);


		os=sizeof(ob2); ob2 = 0x1f;	//Start HM
		wkc_count=ec_SDOwrite(k+1, 0x6040,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave operation mode
		printf("Control Word: %d\n", ob2);
	}

	init_flag = true;

}

boolean ecat_init(void)
{
	std::string lan_port_name_;
	lan_port_name_ = "eth0";
	int n = lan_port_name_.length();

	char ecat_ifname[n + 1];

	strcpy(ecat_ifname, lan_port_name_.c_str());

	int i, oloop, iloop, k, wkc_count;	
	needlf = FALSE;
	inOP = FALSE;

	printf("Starting simple test\n");
	
	if (ec_init(ecat_ifname))
	{
		printf("ec_init on %s succeeded.\n", ecat_ifname); //ifname
		/* find and auto-config slaves */

		if ( ec_config_init(FALSE) > 0 )
		{
			printf("%d slaves found and configured.\n",ec_slavecount);

			//ServoSetup(1);

			//PDO re-mapping****************************************************************************************************
			// for (k=0; k<NUMOFSERVO_DRIVE; ++k)
			// {
			//   if (( ec_slavecount >= 1 ) && strcmp(ec_slave[k+1].name, "iServo_EtherCAT(LAN9252)_V1") == 0) //change name for other drives
			//   {
			//     printf("Re mapping for HERoEHS Linear Actuator...\n");
			//     os=sizeof(ob2); ob2 = 0x1600;	//RxPDO, check MAXPOS ESI
			//     //0x1c12 is Index of Sync Manager 2 PDO Assignment (output RxPDO), CA (Complete Access) must be TRUE
			//     //wkc_count=ec_SDOwrite(k+1, 0x1c12,01,TRUE,os, &ob2,EC_TIMEOUTRXM);	//change slave position (k+1) if needed
			//     wkc_count=ec_SDOwrite(k+1, 0x1c12,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM);	//change slave position (k+1) if needed

			//     if (wkc_count==0)
			//     {
			//       printf("RxPDO assignment error\n");
			//       return FALSE;
			//     }

			//     os=sizeof(ob2); ob2 = 0x1a00;	//TxPDO, check MAXPOS ESI
			//     //0x1c13 is Index of Sync Manager 3 PDO Assignment (input TxPDO), CA (Complete Access) must be TRUE
			//     //wkc_count=ec_SDOwrite(k+1, 0x1c13,01,TRUE,os, &ob2,EC_TIMEOUTRXM); //change slave position (k+1) if needed
			//     wkc_count=ec_SDOwrite(k+1, 0x1c13,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave position (k+1) if needed
			//     if (wkc_count==0)
			//     {
			//       printf("TxPDO assignment error\n");
			//       return FALSE;
			//     }

			//     os=sizeof(ob2); ob2 = 0x02;	//TxPDO, check MAXPOS ESI
			//     wkc_count=ec_SDOwrite(k+1, 0x1600,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave position (k+1) if needed
			//     os=sizeof(ob2); ob2 = 0x6040;	//TxPDO, check MAXPOS ESI
			//     wkc_count=ec_SDOwrite(k+1, 0x1600,0x01,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave position (k+1) if needed
			//     os=sizeof(ob2); ob2 = 0x607A;	//TxPDO, check MAXPOS ESI
			//     wkc_count=ec_SDOwrite(k+1, 0x1600,0x02,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave position (k+1) if needed
				
			//   }
			// }

			//PDO re-mapping****************************************************************************************************

			ec_config_map(&IOmap);
			
			printf("Slaves mapped, state to SAFE_OP.\n");
			/* wait for all slaves to reach SAFE_OP state */
			ec_statecheck(0, EC_STATE_INIT,  EC_TIMEOUTSTATE * 4); ///////////////////////////////////////////

			oloop = ec_slave[0].Obytes;
			if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
			if (oloop > 8) oloop = 8;
			iloop = ec_slave[0].Ibytes;
			if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
			if (iloop > 8) iloop = 8;

			printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

			printf("Request operational state for all slaves\n");
			expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
			printf("Calculated workcounter %d\n", expectedWKC);
			ec_slave[0].state = EC_STATE_OPERATIONAL;
			/* send one valid process data to make outputs in slaves happy*/
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);
			/* request OP state for all slaves */

			//ec_writestate(0);
			//ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE); //wait for OP

			if (ec_slave[0].state == EC_STATE_OPERATIONAL )
			{
				printf("Operational state reached for all slaves.\n");
				wkc_count = 0;

				// for (k=0; k<NUMOFSERVO_DRIVE; ++k)
				// {
				//   maxpos_drive_pt[k].ptOutParam=(MAXPOS_DRIVE_RxPDO_t*)  		ec_slave[k+1].outputs;
				//   maxpos_drive_pt[k].ptInParam= (MAXPOS_DRIVE_TxPDO_t*)  		ec_slave[k+1].inputs;
				//   maxpos_drive_pt[k].ptOutParam->ModeOfOperation=OP_MODE_CYCLIC_SYNC_POSITION;
				// }
				inOP = TRUE;
			}
			else
			{
				printf("Not all slaves reached operational state.\n");
				// ec_readstate();
				// for(i = 1; i<=ec_slavecount ; i++)
				// {
				//   if(ec_slave[i].state != EC_STATE_OPERATIONAL)
				//   {
				//     printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
				//     i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
				//   }
				// }
				// for (i=0; i<NUMOFSERVO_DRIVE; ++i)
				//   ec_dcsync01(i+1, FALSE, 0, 0, 0); // SYNC0,1 
			}
		}
		else
		{
			printf("No slaves found!\n");
			inOP=FALSE;
		}
	}
	else
	{
		printf("No socket connection on %s\nExcecute as root\n", ecat_ifname);
		return FALSE;
	}

	return inOP;
}

void ServoController::TorqueCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
	torque_command[0] = round(msg->data.at(0) / 75559 * 1000 * 1000 / 4.86);
	torque_command[1] = round(msg->data.at(1) / 75559 * 1000 * 1000 / 4.86);
	torque_command[2] = round(msg->data.at(2) / 75559 * 1000 * 1000 / 4.86);
	torque_command[3] = round(msg->data.at(3) / 75559 * 1000 * 1000 / 4.86);
	torque_command[4] = round(msg->data.at(4) / 75559 * 1000 * 1000 / 4.86);
	torque_command[5] = round(msg->data.at(5) / 75559 * 1000 * 1000 / 4.86);
	torque_command[6] = round(msg->data.at(6) / 75559 * 1000 * 1000 / 4.86);
	torque_command[7] = round(msg->data.at(7) / 75559 * 1000 * 1000 / 4.86);
}

void ServoController::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
	if(msg->buttons[3] == true && msg->buttons[0] == false && msg->buttons[1] == false)
	{
		RCLCPP_INFO(this->get_logger(), "set the base mode");
		play_count = 4;
	}
	else if(msg->buttons[1] == true && msg->buttons[3] == false && msg->buttons[0] == false)
	{
		RCLCPP_INFO(this->get_logger(), "set the script mode");
		play_count = 1;
	}
	else if(msg->buttons[0] == true && msg->buttons[1] == false && msg->buttons[3] == false)
	{
		RCLCPP_INFO(this->get_logger(), "set the controller mode");
		play_count = 3;
	}

	goal_pos[1] = 340000 + (static_cast<int>(msg->axes[1]) * -330000);
	goal_pos[0] = 260000 + (static_cast<int>(msg->axes[4]) * -250000);
}

bool ServoController::setup_ethercat(const char* ifname)
{
  int i, chk;
  char IOmap[4096];

  /* initialise SOEM, bind socket to ifname */
  if (ec_init(ifname))
  {
    RCLCPP_INFO(this->get_logger(), "ec_init on %s succeeded.", ifname);
    /* find and auto-config slaves */
																																													
    if (ec_config_init(FALSE) > 0)
    {
      RCLCPP_INFO(this->get_logger(), "%d slaves found and configured.", ec_slavecount);

      ec_config_map(&IOmap);

      ec_configdc();

      RCLCPP_INFO(this->get_logger(), "Slaves mapped, state to SAFE_OP.");
      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

      RCLCPP_INFO(this->get_logger(), "segments : %d : %d %d %d %d", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1],
               ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

      RCLCPP_INFO(this->get_logger(), "Request operational state for all slaves");
      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
      RCLCPP_INFO(this->get_logger(), "Calculated workcounter %d", expectedWKC);
      ec_slave[0].state = EC_STATE_OPERATIONAL;
      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      /* request OP state for all slaves */
      ec_writestate(0);
      chk = 40;
      /* wait for all slaves to reach OP state */

      do
      {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
      } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
      if (ec_slave[0].state == EC_STATE_OPERATIONAL)
      {
        RCLCPP_INFO(this->get_logger(), "Operational state reached for all slaves.");
        pdo_transfer_active = true;
        /* Check if slaves are found in the expected order */
        return true;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Not all slaves reached operational state.");
        ec_readstate();
        for (i = 1; i <= ec_slavecount; i++)
        {
          if (ec_slave[i].state != EC_STATE_OPERATIONAL)
          {
            RCLCPP_WARN(this->get_logger(), "Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s", i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                     ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
          }
        }
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "No slaves found!");
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "No socket connection on %s. \n", ifname);
  }
  return false;
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto enode = std::make_shared<ServoController>();

	if(init_flag == false)
	{
		CheckMotorInitStatus();
		init_flag = true;
	}
	else
	{
		//if(check_param_result == NUMOFSERVO_DRIVE)	check_param = true;
		//else check_param = false;
		check_param = true;
	}
	
	check_param = true; // for test

	if(check_param == true)
	{
		RCLCPP_WARN(enode->get_logger(), "Homing will Start in 5 seconds");
		rclcpp::sleep_for(5s);

		RCLCPP_WARN(enode->get_logger(), "Homing Start");
		Homing();
		rclcpp::sleep_for(20s);
		//homing_flag = true; //homing 여부 확인 플래그
		RCLCPP_WARN(enode->get_logger(), "Homing End");

		RCLCPP_WARN(enode->get_logger(), "Linear Actuator Moving in 5 seconds");
		rclcpp::sleep_for(5s);

		run_flag = true;

		play_count = 0;
		int count = 0;
		//linear_actuator_data.clear();
	}

	rclcpp::sleep_for(5s);
	RCLCPP_WARN(enode->get_logger(), "Start Motor Control");
	run_flag = true;
	rclcpp::sleep_for(5s);

	while(run_flag == true && rclcpp::ok())
	{
		for(int iter = 0; iter < NUMOFSERVO_DRIVE; iter++)
		{
			uint16_t mode_of_driver;
			uint profile_velocity;
			int target_position;
			int target_position_read;
			int actual_position;
			int demand_position;
			uint16_t op_cmd;
			int8_t op_mode;

			if(play_count == 0)
			{
				os=sizeof(op_mode); op_mode = 1;	//Set Profile Position Mode
				ec_SDOwrite(iter+1, 0x6060,0x00,FALSE,os, &op_mode,EC_TIMEOUTRXM); //change slave operation mode
				printf("Set Profile Position Mode: %d\n", op_mode);

				uint16_t status_of_driver;
				os=sizeof(status_of_driver); status_of_driver = 0x00;
				ec_SDOread(iter+1, 0x6061,0x00, FALSE, &os, &status_of_driver, EC_TIMEOUTRXM); //read status of driver
				printf("Status of driver: %d\n", status_of_driver);
				rclcpp::sleep_for(1s);
				RCLCPP_INFO(enode->get_logger(),"start Position control mode");
			}
			else if(play_count == 1)
			{
				os=sizeof(mode_of_driver); mode_of_driver = 0x00;
				ec_SDOread(iter+1, 0x6041,0x00, FALSE, &os, &mode_of_driver, EC_TIMEOUTRXM); //read status of driver
				printf("Mode of driver: %d\n", mode_of_driver);

				os=sizeof(op_cmd); op_cmd = 0x0f;	//Enable
				ec_SDOwrite(iter+1, 0x6040,0x00,FALSE,os, &op_cmd,EC_TIMEOUTRXM); //change slave operation mode
				printf("set OP mod");

				os=sizeof(profile_velocity); profile_velocity = 0x3E8;	// pre state
				ec_SDOwrite(iter+1, 0x6081,0x00,FALSE,os, &profile_velocity,EC_TIMEOUTRXM); //change slave operation mode
				printf("profile veloicty: %d\n", profile_velocity);

				os=sizeof(target_position);
				if((iter+1 == 1) || (iter+1 == 3) || (iter+1 == 7) || (iter+1 == 12) || (iter+1 == 13) || (iter+1 == 18) || (iter+1 == 19) || (iter+1 == 24))
				{
					target_position = 50000;
				}
				else if((iter+1 == 2) || (iter+1 == 4) || (iter+1 == 8) || (iter+1 == 11) || (iter+1 == 14) || (iter+1 == 17) || (iter+1 == 20) || (iter+1 == 23))
				{
					target_position = 50000;
				}

				ec_SDOwrite(iter+1, 0x607A,0x00, FALSE, os, &target_position, EC_TIMEOUTRXM); //read status of driver
				printf("Target Position(write) of driver: %d\n", target_position);

				os=sizeof(op_cmd); op_cmd = 0x3f;	// pre state
				ec_SDOwrite(iter+1, 0x6040,0x00,FALSE,os, &op_cmd,EC_TIMEOUTRXM); //change slave operation mode
				printf("set PP mod");

				os=sizeof(target_position_read); target_position_read = 0x00;
				ec_SDOread(iter+1, 0x607A,0x00, FALSE, &os, &target_position_read, EC_TIMEOUTRXM); //read status of driver
				printf("Target Position(read) of driver: %d\n", target_position_read);

				os=sizeof(actual_position); actual_position = 0x00;
				ec_SDOread(iter+1, 0x6064,0x00, FALSE, &os, &actual_position, EC_TIMEOUTRXM); //read status of driver
				printf("Actual Position of driver: %d\n", actual_position);

				os=sizeof(demand_position); demand_position = 0x00;
				ec_SDOread(iter+1, 0x6062,0x00, FALSE, &os, &demand_position, EC_TIMEOUTRXM); //read status of driver
				printf("Demand Position of driver: %d\n", demand_position);

				RCLCPP_INFO(enode->get_logger(),"moving to target position");
			}
			else if(play_count == 2)
			{
				os=sizeof(mode_of_driver); mode_of_driver = 0x00;
				ec_SDOread(iter+1, 0x6041,0x00, FALSE, &os, &mode_of_driver, EC_TIMEOUTRXM); //read status of driver
				printf("Mode of driver: %d\n", mode_of_driver);

				os=sizeof(op_cmd); op_cmd = 0x0f;	//Enable
				ec_SDOwrite(iter+1, 0x6040,0x00,FALSE,os, &op_cmd,EC_TIMEOUTRXM); //change slave operation mode
				printf("set OP mod");

				os=sizeof(profile_velocity); profile_velocity = 0x3E8;	// pre state
				ec_SDOwrite(iter+1, 0x6081,0x00,FALSE,os, &profile_velocity,EC_TIMEOUTRXM); //change slave operation mode
				printf("profile veloicty: %d\n", profile_velocity);

				os=sizeof(target_position);
				if((iter+1 == 1) || (iter+1 == 3) || (iter+1 == 7) || (iter+1 == 12) || (iter+1 == 13) || (iter+1 == 18) || (iter+1 == 19) || (iter+1 == 24))
				{
					target_position = 500000;
				}
				else if((iter+1 == 2) || (iter+1 == 4) || (iter+1 == 8) || (iter+1 == 11) || (iter+1 == 14) || (iter+1 == 17) || (iter+1 == 20) || (iter+1 == 23))
				{
					target_position = 500000;
				}

				ec_SDOwrite(iter+1, 0x607A,0x00, FALSE, os, &target_position, EC_TIMEOUTRXM); //read status of driver
				printf("Target Position(write) of driver: %d\n", target_position);

				os=sizeof(op_cmd); op_cmd = 0x3f;	// pre state
				ec_SDOwrite(iter+1, 0x6040,0x00,FALSE,os, &op_cmd,EC_TIMEOUTRXM); //change slave operation mode
				printf("set PP mod");

				os=sizeof(target_position_read); target_position_read = 0x00;
				ec_SDOread(iter+1, 0x607A,0x00, FALSE, &os, &target_position_read, EC_TIMEOUTRXM); //read status of driver
				printf("Target Position(read) of driver: %d\n", target_position_read);

				os=sizeof(actual_position); actual_position = 0x00;
				ec_SDOread(iter+1, 0x6064,0x00, FALSE, &os, &actual_position, EC_TIMEOUTRXM); //read status of driver
				printf("Actual Position of driver: %d\n", actual_position);

				os=sizeof(demand_position); demand_position = 0x00;
				ec_SDOread(iter+1, 0x6062,0x00, FALSE, &os, &demand_position, EC_TIMEOUTRXM); //read status of driver
				printf("Demand Position of driver: %d\n", demand_position);

				RCLCPP_INFO(enode->get_logger(),"moving to target position");
			}
			else if(play_count == 3)
			{
				os=sizeof(mode_of_driver); mode_of_driver = 0x00;
				ec_SDOread(iter+1, 0x6041,0x00, FALSE, &os, &mode_of_driver, EC_TIMEOUTRXM); //read status of driver
				printf("Mode of driver: %d\n", mode_of_driver);

				os=sizeof(op_cmd); op_cmd = 0x0f;	//Enable
				ec_SDOwrite(iter+1, 0x6040,0x00,FALSE,os, &op_cmd,EC_TIMEOUTRXM); //change slave operation mode
				printf("set OP mod");

				os=sizeof(profile_velocity); profile_velocity = 0x3E8;	// pre state
				ec_SDOwrite(iter+1, 0x6081,0x00,FALSE,os, &profile_velocity,EC_TIMEOUTRXM); //change slave operation mode
				printf("profile veloicty: %d\n", profile_velocity);

				os=sizeof(target_position);
				if((iter+1 == 1) || (iter+1 == 3))
				{
					target_position = goal_pos[0];
				}
				else if((iter+1 == 2) || (iter+1 == 4))
				{
					target_position = goal_pos[1];
				}

				ec_SDOwrite(iter+1, 0x607A,0x00, FALSE, os, &target_position, EC_TIMEOUTRXM); //read status of driver
				printf("Target Position(write) of driver: %d\n", target_position);

				os=sizeof(op_cmd); op_cmd = 0x3f;	// pre state
				ec_SDOwrite(iter+1, 0x6040,0x00,FALSE,os, &op_cmd,EC_TIMEOUTRXM); //change slave operation mode
				printf("set PP mod");

				os=sizeof(target_position_read); target_position_read = 0x00;
				ec_SDOread(iter+1, 0x607A,0x00, FALSE, &os, &target_position_read, EC_TIMEOUTRXM); //read status of driver
				printf("Target Position(read) of driver: %d\n", target_position_read);

				os=sizeof(actual_position); actual_position = 0x00;
				ec_SDOread(iter+1, 0x6064,0x00, FALSE, &os, &actual_position, EC_TIMEOUTRXM); //read status of driver
				printf("Actual Position of driver: %d\n", actual_position);

				os=sizeof(demand_position); demand_position = 0x00;
				ec_SDOread(iter+1, 0x6062,0x00, FALSE, &os, &demand_position, EC_TIMEOUTRXM); //read status of driver
				printf("Demand Position of driver: %d\n", demand_position);

				RCLCPP_INFO(enode->get_logger(),"moving to target position");
			}
			else if(play_count == 4)
			{
				os=sizeof(mode_of_driver); mode_of_driver = 0x00;
				ec_SDOread(iter+1, 0x6041,0x00, FALSE, &os, &mode_of_driver, EC_TIMEOUTRXM); //read status of driver
				printf("Mode of driver: %d\n", mode_of_driver);

				os=sizeof(op_cmd); op_cmd = 0x0f;	//Enable
				ec_SDOwrite(iter+1, 0x6040,0x00,FALSE,os, &op_cmd,EC_TIMEOUTRXM); //change slave operation mode
				printf("set OP mod");

				os=sizeof(profile_velocity); profile_velocity = 0x1f4;	// pre state
				ec_SDOwrite(iter+1, 0x6081,0x00,FALSE,os, &profile_velocity,EC_TIMEOUTRXM); //change slave operation mode
				printf("profile veloicty: %d\n", profile_velocity);

				os=sizeof(target_position);
				if((iter+1 == 1) || (iter+1 == 3) || (iter+1 == 7) || (iter+1 == 12) || (iter+1 == 13) || (iter+1 == 18) || (iter+1 == 19) || (iter+1 == 24))
				{
					target_position = 200000;
				}
				else if((iter+1 == 2) || (iter+1 == 4) || (iter+1 == 8) || (iter+1 == 11) || (iter+1 == 14) || (iter+1 == 17) || (iter+1 == 20) || (iter+1 == 23))
				{
					target_position = 200000;
				}

				ec_SDOwrite(iter+1, 0x607A,0x00, FALSE, os, &target_position, EC_TIMEOUTRXM); //read status of driver
				printf("Target Position(write) of driver: %d\n", target_position);

				os=sizeof(op_cmd); op_cmd = 0x3f;	// pre state
				ec_SDOwrite(iter+1, 0x6040,0x00,FALSE,os, &op_cmd,EC_TIMEOUTRXM); //change slave operation mode
				printf("set PP mod");

				os=sizeof(target_position_read); target_position_read = 0x00;
				ec_SDOread(iter+1, 0x607A,0x00, FALSE, &os, &target_position_read, EC_TIMEOUTRXM); //read status of driver
				printf("Target Position(read) of driver: %d\n", target_position_read);

				os=sizeof(actual_position); actual_position = 0x00;
				ec_SDOread(iter+1, 0x6064,0x00, FALSE, &os, &actual_position, EC_TIMEOUTRXM); //read status of driver
				printf("Actual Position of driver: %d\n", actual_position);

				os=sizeof(demand_position); demand_position = 0x00;
				ec_SDOread(iter+1, 0x6062,0x00, FALSE, &os, &demand_position, EC_TIMEOUTRXM); //read status of driver
				printf("Demand Position of driver: %d\n", demand_position);

				RCLCPP_INFO(enode->get_logger(),"moving to target position");
			}
			/*else
			{
			count ++;
			RCLCPP_INFO(enode->get_logger(),"count : %d", count);  
			
			os=sizeof(op_mode); op_mode = 4;	//Set Profile Torque Mode
			ec_SDOwrite(iter+1, 0x6060,0x00,FALSE,os, &op_mode,EC_TIMEOUTRXM); //change slave operation mode
			printf("Set Profile Torque Mode: %d\n", op_mode);

			os=sizeof(mode_of_driver); mode_of_driver = 0x00;
			ec_SDOread(iter+1, 0x6041,0x00, FALSE, &os, &mode_of_driver, EC_TIMEOUTRXM); //read status of driver
			//printf("Mode of driver: %d\n", mode_of_driver);

			uint target_torque = 0;
			os=sizeof(target_torque);
			target_torque = torque_command[iter];
			ec_SDOwrite(iter+1, 0x6071,0x00, FALSE, os, &target_torque, EC_TIMEOUTRXM); //read status of driver
			printf("Target torque(write) of driver: %d\n", target_torque);

			os=sizeof(op_cmd); op_cmd = 0xf;	// pre state
			ec_SDOwrite(iter+1, 0x6040,0x00,FALSE,os, &op_cmd,EC_TIMEOUTRXM); //change slave operation mode
			printf("set PT mod\n");

			int16_t actual_torque;
			os=sizeof(actual_torque); actual_torque=0;
			ec_SDOread(1, 0x6077, 0x00, FALSE, &os, &actual_torque, EC_TIMEOUTRXM);
			printf("Actual Torque of driver: %d\n", actual_torque);

			uint target_torque_read = 0;
			os=sizeof(target_torque_read); target_torque_read = 0x00;
			ec_SDOread(iter+1, 0x6071,0x00, FALSE, &os, &target_torque_read, EC_TIMEOUTRXM); //read status of driver
			//printf("Target Position(read) of driver: %d\n", target_torque_read);

			os=sizeof(actual_position); actual_position = 0x00;
			ec_SDOread(iter+1, 0x6064,0x00, FALSE, &os, &actual_position, EC_TIMEOUTRXM); //read status of driver

				RCLCPP_WARN(enode->get_logger(),"calc actual position!!!");
				linear_actuator_length.data.push_back(float(actual_position));
				RCLCPP_WARN(enode->get_logger(),"pub actual position!!!");
				
			os=sizeof(demand_position); demand_position = 0x00;
			ec_SDOread(iter+1, 0x6062,0x00, FALSE, &os, &demand_position, EC_TIMEOUTRXM); //read status of driver
			//printf("Demand Position of driver: %d\n", demand_position);
			}*/

		}
		
		//linear_actuator_length_pub.publish(linear_actuator_length);
		//linear_actuator_length.data.clear();

		if(play_count == 0)
		{
			RCLCPP_INFO(enode->get_logger(), "start Position control mode");
			rclcpp::sleep_for(3s);

			play_count = 4;
		}
		else if(play_count == 1)
		{
			RCLCPP_WARN(enode->get_logger(),"moving ...");
			rclcpp::sleep_for(1s);
			play_count = 2;
		}
		else if(play_count == 2)
		{
			RCLCPP_WARN(enode->get_logger(),"moving ...");
			rclcpp::sleep_for(1s);
			play_count = 1;
		}
		else if(play_count == 3)
		{
			//test
		}
		else if(play_count == 4)
		{
			RCLCPP_WARN(enode->get_logger(),"Start Base Mode in 5 seconds");
			rclcpp::sleep_for(5s);
			play_count = 1;
		}
		//rclcpp::spin(enode);
	}

	rclcpp::shutdown();
	return 0;
}
