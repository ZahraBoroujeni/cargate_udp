#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <cargate_udp/AuxDevicesData.h>
#include <autonomos_comm_transport/udp.h>

#include "CargatePacket.h"



namespace aa
{
namespace modules
{
namespace io
{
namespace cargate
{

class CarGate
{
private:
    // the node handle
    ros::NodeHandle nh_;

    // ode handle in the private namespace
    ros::NodeHandle priv_nh_;

    // subscribers

    // wanted steering angle position
    ros::Subscriber mWantedSteeringAngleIn_;
    // normalized wanted steering angle


public:

	explicit CarGate(ros::NodeHandle nh, int argc,char** argv);
	virtual ~CarGate();
	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

protected:

	std::string mTargetAddress;
	uint32_t mTargetPort;
	float mMaxThrottle;
	float mMinThrottle;
	float mMaxBrake;
	float mMinBrake;
	float mMaxSteeringAngle;
	tools::cargate::CargatePacket mPacket;
    TimeStamp mLastStamp;
	bool mDisabled;
	int mUdpSocket;
	struct sockaddr_in myaddr;

	//autonomos::util::transport::UDP mUdpSocket;

	ros::Subscriber mWantedSteeringAngleIn;				// wanted steering angle position //float
	ros::Subscriber mNormalizedWantedSteeringAngleIn;	// normalized wanted steering angle //float

	/** Activation request */
	ros::Subscriber  mActivationRequestIn; //bool

	/** *0,75..4,00 V: Normalbetrieb, 4,40 V: Kickdown*/
	ros::Subscriber  mDesiredThrottleVoltageIn; //timedouble
	/** 0..127,5 bar*/
	ros::Subscriber  mDesiredBrakePressureIn;  //timedouble
	/**
	 * -1.0 .. +1.0
	 *  Only used if mDesiredThrottleVoltageIn or mDesiredBrakePressureIn are not connected.
	 *  Negative value brakes, postive value gives gas.
	 */
	ros::Subscriber  mDesiredSpeedIn; //timedouble

	/**  1=Park, 2=Reverse, 4=Neutral, 8=Drive*/
	ros::Subscriber mDesiredGearPositionIn;

	/** sets desired headlight state: 0=off, 1=park, 2=on */
	ros::Subscriber mDesiredHeadlightIn;
	/** sets desired wiper state: 0=off, 1=intermittent, 2=low, 3=high */
	ros::Subscriber mDesiredWiperIn;
	/** sets desired siren state: 0=off, 1=on*/
	ros::Subscriber mDesiredSirenIn;
	/** sets desired turn signal state: 0=off, 1=left, 2=right, 3=hazards */
	ros::Subscriber mDesiredTurnSignalIn;

	ros::Subscriber  mDesiredAuxDevicesIn; //AuxDevicesData

		 // callback functions
	void WantedSteeringAngleIn(const std_msgs::Float32 & msg);
	void NormalizedWantedSteeringAngleIn(const std_msgs::Float32 & msg);
	void ActivationRequestIn(const std_msgs::Bool & msg);
	void DesiredThrottleVoltageIn(const std_msgs::Float32 & msg);
	void DesiredBrakePressureIn(const std_msgs::Float32 & msg);
	void DesiredSpeedIn(const std_msgs::Float32 & msg);
	void DesiredGearPositionIn(const std_msgs::Float32 & msg);
	void DesiredAuxDevicesIn(const cargate_udp::AuxDevicesData & aux);
	void DesiredHeadlightIn(const std_msgs::Float32 & msg);
	void DesiredWiperIn(const std_msgs::Float32 & msg);
	void DesiredSirenIn(const std_msgs::Float32 & msg);
	void DesiredTurnSignalIn(const std_msgs::Float32 & msg);
	 
};

}
}
}
}
