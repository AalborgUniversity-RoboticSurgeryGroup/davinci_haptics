#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/Float64.h>
#include <vector>

#define ROLL_LIMIT 2.5
#define PITCH_LIMIT 1.2
#define JAW_LIMIT 1.5

#define Kt_p4 0.0439

#define FREQ 200

using namespace std;

class System
{
public:
	vector<double> current;
	vector<double> position;
	vector<double> velocity;

	vector<double> setpoint;

	bool ready;
	vector<double> Kt;
	vector<double> Gr;

	void SystemCallback(sensor_msgs::JointState system);
	void check_limits_position(vector<double> limit);

	System(vector<double> torque_constant, vector<double> gear_ratio);
	~System();
private:

};
System::System(vector<double> torque_constant, vector<double> gear_ratio)
{
	ready = false;
	Kt = torque_constant;
	Kt.resize(torque_constant.size());
	Gr = gear_ratio;
	Gr.resize(gear_ratio.size());

}
System::~System()
{

}
void System::SystemCallback(sensor_msgs::JointState system)
{
	int length = system.name.size();
	current.resize(length);
	position.resize(length);
	velocity.resize(length);

	if (length > 0)
	{
		for(int i=0; i<length;i++)
		{
			current[i] = system.effort[i];
			position[i] = system.position[i];
			velocity[i] = system.velocity[i];
		}
		ready = true;
	}
	else
	{
		ready = false;
	}
}
void System::check_limits_position(vector<double> limit)
{

}

class Joystick
{
public:

	Joystick();
	~Joystick();
	void JoystickCallback(sensor_msgs::JointState joint_states);
	void check_limits(void);

	double roll,pitch,jaw_left,jaw_right;
	vector<double> position;
	vector<double> current;
	vector<double> Kt;
	vector<double> Gr;

	bool ready;
private:
	
};
Joystick::Joystick()
{
	roll = 0;
	pitch = 0;
	jaw_left = 0;
	jaw_right = 0;

	ready = false;

	Kt.push_back(0.00841);
	Kt.push_back(0.0109);
	Kt.push_back(0.0109);	//roll
	Kt.push_back(0.0109);	//pitch
	Kt.resize(4);

	Gr.push_back(5.1*2);
	Gr.push_back(19);
	Gr.push_back(4.4);	//roll
	Gr.push_back(19);
	Gr.resize(4);
}
Joystick::~Joystick()
{

}
void Joystick::JoystickCallback(sensor_msgs::JointState joint_states)
{

	int l = joint_states.name.size();
	position.resize(l);
	current.resize(l);

	for(int i=0; i<l;i++)
	{
		position[i] = joint_states.position[i];
		current[i] = joint_states.effort[i];
	}
	if (l == 4)
	{
		ready = true;
	}
	else
	{
		ready = false;
	}


}
void Joystick::check_limits(void)
{
	if (position[2] > ROLL_LIMIT)
	{
		roll = ROLL_LIMIT;
	}
	else if (position[2] < -ROLL_LIMIT)
	{
		roll = -ROLL_LIMIT;
	}
	else
	{
		roll = position[2];
	}

	if (position[3] > PITCH_LIMIT)
	{
		pitch = PITCH_LIMIT;
	}
	else if (position[3] < -PITCH_LIMIT)
	{
		pitch = -PITCH_LIMIT;
	}
	else
	{
		pitch = position[3];
	}

	if (-position[0] + position[1] > JAW_LIMIT)
	{
		jaw_left = JAW_LIMIT;
	}
	else if (-position[0] + position[1]  < -JAW_LIMIT)
	{
		jaw_left = -JAW_LIMIT;
	}
	else
	{
		jaw_left = -position[0] + position[1] ;
	}

	if (-position[0] - position[1]> JAW_LIMIT)
	{
		jaw_right = JAW_LIMIT;
	}
	else if (-position[0] - position[1] < -JAW_LIMIT)
	{
		jaw_right = -JAW_LIMIT;
	}
	else
	{
		jaw_right = -position[0] - position[1];
	}

}



class Haptic_controller
{
public:

	Haptic_controller();
	~Haptic_controller();

	void calculate_torque(vector<double> current, vector<double> torque_constant, vector<double> gear_ratio);
	void calculate_current_sp(vector<double> current, vector<double> torque_constant, vector<double> gear_ratio);
	void print(vector<double> vec);

	vector<double> T;
	vector<double> I_sp;

private:

};
Haptic_controller::Haptic_controller()
{

}
Haptic_controller::~Haptic_controller()
{

}
void Haptic_controller::calculate_torque(vector<double> current, vector<double> torque_constant, vector<double> gear_ratio)
{
	T.resize(current.size());
	for (int i=0;i<current.size();i++)
	{
		T[i] = current[i]*torque_constant[i]*gear_ratio[i];
	}

}
void Haptic_controller::calculate_current_sp(vector<double> torque, vector<double> torque_constant, vector<double> gear_ratio)
{
	I_sp.resize(4);

	I_sp[0] = 0;//(torque[2]+torque[3])/torque_constant[0]/gear_ratio[0];	// pinch 	(Motor1)
 	I_sp[1] = 0;//(torque[2]-torque[3])/torque_constant[1]/gear_ratio[1];	// yaw		(Motor2)
	I_sp[2] = torque[5]/torque_constant[2]/gear_ratio[2];	// Roll		(Motor3)
	I_sp[3] = 0;//torque[4]/torque_constant[3]/gear_ratio[3];	// Pitch	(Motor4)

	
}
void Haptic_controller::print(vector<double> vec)
{
	for(int i=0;i<vec.size();i++)
	{
		printf("%lf \t",vec.at(i));
	}
	printf("\n");
}

int main(int argc, char **argv)
{
	vector<double> kT (7,Kt_p4);
	vector<double> gR (7);
	gR.at(0)=0;
	gR.at(1)=0;
	gR.at(2)=10.0;
	gR.at(3)=10.0;
	gR.at(4)=11.0;
	gR.at(5)=7.5;
	gR.at(6)=0;

	System P4 (kT,gR);

	Joystick joystick;

	ros::init(argc, argv, "haptic_controller");
	ros::NodeHandle n;
	ros::Subscriber Joystick_sub = n.subscribe("davinci_joystick/joint_states",1,&Joystick::JoystickCallback, &joystick);
	ros::Subscriber P4_sub = n.subscribe("/davinci/joint_states", 1, &System::SystemCallback, &P4);

	ros::Publisher instr_roll_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_roll_controller/command",1);
	ros::Publisher instr_pitch_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_pitch_controller/command",1);
	ros::Publisher instr_yawl_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_jaw_left_controller/command",1);
	ros::Publisher instr_yawr_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_jaw_right_controller/command",1);
	ros::Publisher joystick_sp_pub = n.advertise<std_msgs::Float64MultiArray>("davinci_joystick/I_sp",1);

	ros::Rate rate(FREQ);

	std_msgs::Float64 roll_setpoint;
	std_msgs::Float64 pitch_setpoint;
	std_msgs::Float64 jaw_left_setpoint;
	std_msgs::Float64 jaw_right_setpoint;

	std_msgs::Float64MultiArray joystick_sp;
	joystick_sp.data.resize(4);


	Haptic_controller C;

	while (ros::ok())
	{
		if (joystick.ready)
		{
			joystick.check_limits();
		}
		

		if (P4.ready)
		{
			C.calculate_torque(P4.current,P4.Kt,P4.Gr);
			vector<double>Torque_robot =C.T;
			C.calculate_current_sp(C.T,joystick.Kt,joystick.Gr);
		}

		if(joystick.ready)
		{
			roll_setpoint.data = joystick.roll;
			pitch_setpoint.data = joystick.pitch;
			jaw_left_setpoint.data = joystick.jaw_left;
			jaw_right_setpoint.data = joystick.jaw_right;
		}

		if(P4.ready)
		{
			joystick_sp.data[0] = 0;
			joystick_sp.data[1] = 0;
			joystick_sp.data[2] = C.I_sp[2];
			joystick_sp.data[3] = C.I_sp[3];
		}

		instr_yawl_pub.publish(jaw_left_setpoint);
		instr_yawr_pub.publish(jaw_right_setpoint);
		instr_roll_pub.publish(roll_setpoint);
		instr_pitch_pub.publish(pitch_setpoint);
		joystick_sp_pub.publish(joystick_sp);

		ros::spinOnce();
		rate.sleep();
	}

}
