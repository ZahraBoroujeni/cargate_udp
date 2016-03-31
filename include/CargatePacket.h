#pragma once
#include <boost/cstdint.hpp>

namespace aa
{
namespace tools
{

namespace cargate
{

class CargatePacket
{
public:
	CargatePacket(unsigned int seq)
		: version(0)
		, sequence(seq)
		, activationRequest(false)
		, normalisedWantedAngle(0)
		, throttleVoltage(0)
		, brakePressure(0)
		, gearPosition(4)
		, headlight(0)
		, wiper(0)
		, siren(0)
		, turnSignal(0)
	{}

	CargatePacket(unsigned int seq,
				  float _normalisedWantedAngle,
				  float _throttleVoltage,
				  float _brakePressure,
				  int _gearPosition,
				  int _headlight,
				  int _wiper,
				  int _siren,
				  int _turnSignal
				 )
		: version(0)
		, sequence(seq)
		, activationRequest(true)
		, normalisedWantedAngle(_normalisedWantedAngle)
		, throttleVoltage(_throttleVoltage)
		, brakePressure(_brakePressure)
		, gearPosition(_gearPosition)
		, headlight(_headlight)
		, wiper(_wiper)
		, siren(_siren)
		, turnSignal(_turnSignal)
	{}

	boost::uint32_t version;
	boost::uint32_t sequence;
	bool activationRequest;
	float normalisedWantedAngle;
	float throttleVoltage;	/** *0,75..4,00 V: Normalbetrieb, 4,40 V: Kickdown*/
	float brakePressure;		/** 0..127,5 bar*/
	boost::int32_t gearPosition;		/**  1=Park, 2=Reverse, 4=Neutral, 8=Drive*/
	boost::int32_t headlight;			/** sets  headlight state: 0=off, 1=park, 2=on */
	boost::int32_t wiper;				/** sets  wiper state: 0=off, 1=intermittent, 2=low, 3=high */
	boost::int32_t siren;				/** sets  siren state: 0=off, 1=on*/
	boost::int32_t turnSignal;			/** sets  turn signal state: 0=off, 1=left, 2=right, 3=hazards */
};


}



}




}



