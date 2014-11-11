#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <vector>

#define ROLL_LIMIT 1.5
#define PITCH_LIMIT 1.2
#define JAW_LIMIT 1.5

#define FREQ 500

using namespace std;

class Davinci
{
public:
	vector<double> current;
	vector<double> position;
	bool init;

	void StateCallback(sensor_msgs::JointState arm);
	Davinci();
	~Davinci();
private:

};

Davinci::Davinci()
{
	init = false;
}
Davinci::~Davinci()
{
}
void Davinci::StateCallback(sensor_msgs::JointState arm)
{
	int l = arm.name.size();
	current.resize(l);
	position.resize(l);

	for(int i=0; i<l;i++)
	{
		current[i] = arm.effort[i];
		position[i] = arm.position[i];
	}
	if (l > 0)
	{
		init = true;
	}
	else
	{
		init = false;
	}
	// p4_hand_pitch, p4_hand_roll, p4_intr_jaw_left, p4 instr_jaw_right, p4_instr_pitch, p4_instr_roll, p4_instr_slide

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

	bool init;
private:
	
};
Joystick::Joystick()
{
	roll = 0;
	pitch = 0;
	jaw_left = 0;
	jaw_right = 0;

	init = false;
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
		init = true;
	}
	else
	{
		init = false;
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


int main(int argc, char **argv)
{
	Davinci P4;
	Joystick joystick;

	ros::init(argc, argv, "haptic_controller");
	ros::NodeHandle n;
	ros::Subscriber Joystick_sub = n.subscribe("davinci_joystick/joint_states",1,&Joystick::JoystickCallback, &joystick);
	ros::Subscriber P4_sub = n.subscribe("/davinci/joint_states", 1, &Davinci::StateCallback, &P4);

	ros::Publisher instr_roll_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_roll_controller/command",1);
	ros::Publisher instr_pitch_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_pitch_controller/command",1);
	ros::Publisher instr_yawl_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_jaw_left_controller/command",1);
	ros::Publisher instr_yawr_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_jaw_right_controller/command",1);

	ros::Rate rate(FREQ);

	std_msgs::Float64 roll_setpoint;
	std_msgs::Float64 pitch_setpoint;
	std_msgs::Float64 jaw_left_setpoint;
	std_msgs::Float64 jaw_right_setpoint;

	while (ros::ok())
	{
		if (joystick.init)
		{
			joystick.check_limits();
		}

		roll_setpoint.data = joystick.roll;
		pitch_setpoint.data = joystick.pitch;
		jaw_left_setpoint.data = joystick.jaw_left;
		jaw_right_setpoint.data = joystick.jaw_right;

		instr_yawl_pub.publish(jaw_left_setpoint);
		instr_yawr_pub.publish(jaw_right_setpoint);
		instr_roll_pub.publish(roll_setpoint);
		instr_pitch_pub.publish(pitch_setpoint);

		ros::spinOnce();
		rate.sleep();
	}

}
