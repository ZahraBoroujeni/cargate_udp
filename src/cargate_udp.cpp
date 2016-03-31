#include "cargate_udp.h"
#include <autonomos_comm_transport/udp.h>
#include <ros/ros.h>


namespace aa
{
namespace modules
{
namespace io
{
namespace cargate
{

using namespace modules::io::cargate;

//REGISTERTASKCONTEXT2(CarGate, "PeriodicActivity");

// static TaskStatus theTaskStatus;
CarGate::CarGate(ros::NodeHandle nh, int argc,char** argv) : nh_(nh), priv_nh_("~"),mPacket(0) //mUdpSocket(0, "", 11001, "192.168.1.20",true,false)
{
	ROS_INFO("try to make a UDP port ...");
	priv_nh_.param<std::string>("TargetAddress", mTargetAddress, "192.168.1.20");
    //priv_nh_.param<uint32_t>("TargetPort", mTargetPort, 11001);
    priv_nh_.param<float>("MaxThrottle", mMaxThrottle, 4.0);
    priv_nh_.param<float>("MinThrottle", mMinThrottle, 0.75);
    priv_nh_.param<float>("MaxBrake", mMaxBrake, 127.5);
    priv_nh_.param<float>("MinBrake", mMinBrake, 0.0);
    priv_nh_.param<float>("MaxSteeringAngle", mMaxSteeringAngle, 530.0);
    
    mTargetPort=11001;

	
	// initialize subscriber
    mWantedSteeringAngleIn = nh_.subscribe(nh_.resolveName("mWantedSteeringAngleIn"), 10, &CarGate::WantedSteeringAngleIn,this);
    mNormalizedWantedSteeringAngleIn = nh_.subscribe(nh_.resolveName("mNormalizedWantedSteeringAngleIn"), 10, &CarGate::NormalizedWantedSteeringAngleIn,this);
    mActivationRequestIn = nh_.subscribe(nh_.resolveName("mActivationRequestIn"), 10, &CarGate::ActivationRequestIn,this);
    mDesiredThrottleVoltageIn = nh_.subscribe(nh_.resolveName("mDesiredThrottleVoltageIn"), 10, &CarGate::DesiredThrottleVoltageIn,this);
    
    mDesiredBrakePressureIn = nh_.subscribe(nh_.resolveName("mDesiredBrakePressureIn"), 10, &CarGate::DesiredBrakePressureIn,this);
    mDesiredSpeedIn = nh_.subscribe(nh_.resolveName("mDesiredSpeedIn"), 10, &CarGate::DesiredSpeedIn,this);
    mDesiredGearPositionIn = nh_.subscribe(nh_.resolveName("mDesiredGearPositionIn"), 10, &CarGate::DesiredGearPositionIn,this);
    mDesiredHeadlightIn = nh_.subscribe(nh_.resolveName("mDesiredHeadlightIn"), 10, &CarGate::DesiredHeadlightIn,this);
    mDesiredWiperIn = nh_.subscribe(nh_.resolveName("mDesiredWiperIn"), 10, &CarGate::DesiredWiperIn,this);
    mDesiredSirenIn = nh_.subscribe(nh_.resolveName("mDesiredSirenIn"), 10, &CarGate::DesiredSirenIn,this);
    mDesiredTurnSignalIn = nh_.subscribe(nh_.resolveName("mDesiredTurnSignalIn"), 10, &CarGate::DesiredTurnSignalIn,this);
    mDesiredAuxDevicesIn = nh_.subscribe(nh_.resolveName("mDesiredAuxDevicesIn"), 10, &CarGate::DesiredAuxDevicesIn,this);
   // mUdpSocket=createUdpSocket(mTargetPort);
    
   
    bool start_=startHook();


}

CarGate::~CarGate()
{
}

void CarGate::WantedSteeringAngleIn(const std_msgs::Float32 & msg)
{
	mPacket.normalisedWantedAngle = msg.data / (-mMaxSteeringAngle);
	mDisabled=false;
}


void CarGate::NormalizedWantedSteeringAngleIn(const std_msgs::Float32 & msg)
{
	mPacket.normalisedWantedAngle = msg.data;
	mDisabled=false;
}

void CarGate::ActivationRequestIn(const std_msgs::Bool & msg)
{
	mPacket.activationRequest = msg.data;
	if (!mPacket.activationRequest) {
		//mUdpSocket.send(&mPacket, sizeof(mPacket), 0, boost::asio::ip::address::from_string(mTargetAddress), mTargetPort);
		return;
	}
	mDisabled=false;
}

void CarGate::DesiredThrottleVoltageIn(const std_msgs::Float32 & msg)
{
	mPacket.throttleVoltage = msg.data;
	mDisabled=false;
}

void CarGate::DesiredBrakePressureIn(const std_msgs::Float32 & msg)
{
	mPacket.brakePressure = msg.data;
	mDisabled=false;
}


void CarGate::DesiredSpeedIn(const std_msgs::Float32 & msg)
{
	// 	mPacket.throttleVoltage = normalisedSpeedToThrottle(older.data, mMinThrottle.get(), mMaxThrottle.get());
	// 	mPacket.brakePressure = normalisedSpeedToBrake(older.data, mMinBrake.get(), mMaxBrake.get());
	mDisabled=false;
}

void CarGate::DesiredGearPositionIn(const std_msgs::Float32 & msg)
{
	mPacket.gearPosition =	msg.data;
	mDisabled=false;
}


void CarGate::DesiredAuxDevicesIn(const cargate_udp::AuxDevicesData & aux)
{
	mPacket.headlight = aux.headlightState.data;
	mPacket.wiper = aux.wiperState.data;
	mPacket.siren = aux.sirenState.data;
	mPacket.turnSignal = aux.turnsignalState.data;
	mDisabled=false;
}


void CarGate::DesiredHeadlightIn(const std_msgs::Float32 & msg)
{
	mPacket.headlight =	msg.data;
	mDisabled=false;
}
void CarGate::DesiredWiperIn(const std_msgs::Float32 & msg)
{
	mPacket.wiper =	msg.data;
	mDisabled=false;
}
void CarGate::DesiredSirenIn(const std_msgs::Float32 & msg)
{
	mPacket.siren =	msg.data;
	mDisabled=false;
}
void CarGate::DesiredTurnSignalIn(const std_msgs::Float32 & msg)
{
	mPacket.turnSignal =	msg.data;
	mDisabled=false;
}

// int CarGate::createUdpSocket(boost::uint16_t port)
// {
// 	SOCKET  s = socket(PF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);

// 	struct sockaddr_in addr;

// 	memset(&addr, 0, sizeof(addr));

// 	addr.sin_family = AF_INET;
// 	addr.sin_port = htons(port);
// 	addr.sin_addr.s_addr = INADDR_ANY;

// 	if (0 != ::bind(s, (sockaddr *)&addr, sizeof(addr))) {
// 		close(s);
// 		return -1;
// 	}

// 	return s;
// }

bool CarGate::startHook()
{
	mDisabled = false;
	myaddr = { 0 };
	myaddr.sin_family      = AF_INET;
	myaddr.sin_port        = htons(mTargetPort);
    inet_aton(mTargetAddress.c_str(), &(myaddr.sin_addr));
    mUdpSocket = socket(PF_INET, SOCK_DGRAM, 0);
	if (bind(mUdpSocket, (struct sockaddr*)&myaddr, sizeof(myaddr))!=0) {
   		ROS_ERROR("oh no, could not create UDP port");
   		return  false;
	}
	ROS_INFO("success!");
	connect(mUdpSocket, (struct sockaddr*)&myaddr, sizeof(myaddr));
	return true;
}

void CarGate::updateHook()
{
	bool const disable=false;

	if (disable != mDisabled) {
#if defined(VERBOSE)
		std::cout << "Disabled switched to " << disable << std::endl;
#endif
		mDisabled = disable;
	}

	if (disable) {
		return;
	}

	++mPacket.sequence;
	//mLastStamp = older;
	
    sendto(mUdpSocket,&mPacket, sizeof(mPacket), 0,(struct sockaddr*)&myaddr, sizeof(myaddr));



//	std::cout<<"sending packet with normalisedWantedAngle="<<mPacket.normalisedWantedAngle<<", throotleVoltage = "<<mPacket.throttleVoltage<<", brakePressure = "<<mPacket.brakePressure<<" in packet no. "<<mPacket.sequence<<" to "<<mTargetAddress.rvalue()<<":"<<mTargetPort.get()<<std::endl;
	//int written =mUdpSocket.write(&mPacket, sizeof(mPacket),&sa,100);

	//mUdpSocket.sendto(&mPacket, sizeof(mPacket), 0, boost::asio::ip::address::from_string(mTargetAddress.rvalue()), mTargetPort.get());

	mDisabled=true;
}

void CarGate::stopHook()
{
	++mPacket.sequence;
	mPacket.activationRequest = false;
	shutdown(mUdpSocket, SHUT_RD);
	//int written =mUdpSocket.write(&mPacket, sizeof(mPacket),1000);

}

}
}
}
}

