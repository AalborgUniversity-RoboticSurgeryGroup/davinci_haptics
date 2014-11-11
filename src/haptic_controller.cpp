#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <vector>

#define ROLL_LIMIT 2.5
#define PITCH_LIMIT 1.2
#define JAW_LIMIT 1.5

using namespace std;

class Davinci
{
public:
	double I_roll;
	double I_jaw_left;
	double I_jaw_right;
	double I_pitch;


	void StateCallback(sensor_msgs::JointState arm);
	Davinci();
	~Davinci();
private:

};

Davinci::Davinci()
{
	I_roll = 0.0;
	I_jaw_left = 0.0;
	I_jaw_right = 0.0;
	I_pitch = 0.0;
}
Davinci::~Davinci()
{
}
void Davinci::StateCallback(sensor_msgs::JointState arm)
{
	I_jaw_left = arm.effort[2];
	I_jaw_right = arm.effort[3];
	I_pitch = arm.effort[4];
	I_roll = arm.effort[5];
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

private:
	vector<double> position;
	vector<double> current;
};
Joystick::Joystick()
{
	roll = 0;
	pitch = 0;
	jaw_left = 0;
	jaw_right = 0;
}
Joystick::~Joystick()
{

}
void Joystick::JoystickCallback(sensor_msgs::JointState joint_states)
{
	position.clear();
	current.clear();

	for(int i=0; i<joint_states.name.size();i++)
	{
		position.push_back(joint_states.position[i]);
		current.push_back(joint_states.effort[i]);
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
		roll = -position[2];
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

	ros::Rate rate(100);

	std_msgs::Float64 roll_setpoint;
	std_msgs::Float64 pitch_setpoint;
	std_msgs::Float64 jaw_left_setpoint;
	std_msgs::Float64 jaw_right_setpoint;

	while (ros::ok())
	{

		joystick.check_limits();

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
