#pragma once
#ifndef Q_MOC_RUN
#include "eCAN.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h> 
#include <stdlib.h>
#include <unistd.h>             
#endif

using namespace std;

struct sOdometry {	//130321 Lk
	double t;
	double WheelSpeed_FL;	// m/s
	double WheelSpeed_FR;	// m/s
	double WheelSpeed_RL;	// m/s
	double WheelSpeed_RR;	// m/s
};

struct sObdVelocity{ //130524 Lk
	double t;
	double Speed;
};

struct sMcuBrkAcc{
	int BrkCtlBit;
	int AccCtlBit;
	double ArmAngle;
	double AccVoltage;
};

struct sMcuSAS{
	int SteerCtlBit;
	double SteeringAngle;
};

struct sAPC_CMD_State {
	int APC_CmdState;
	int APC_GearCmdState;
	int CmdStateAliveCounter;
};

struct sVEH_STATE{
	double 	LatAccel;	// [m/s^2]
	double 	LongAccel;	// [m/s^2]
	double 	YawRate;	// []
	double 	SteerAngle;	// [Deg]
	int 	GearState;	// []
};

struct sWHL_SPD{
	double 	WhlSpd_RR;	// [km/h]
	double 	WhlSpd_RL;	// [km/h]
	double 	WhlSpd_FR;	// [km/h]
	double 	WhlSpd_FL;	// [km/h]
};


class CAN_MCU
{
public:
	CAN_MCU(int can_I);
	~CAN_MCU();

	bool Get_APC_CMD_State();	// add
	bool Get_VEH_STATE();		// add
	bool Get_WHL_SPD();		// add

	// bool GetOdometrydata();
	// bool GetMcuBrkAcc();
	// bool GetMcuSAS();

public:
	eCAN *_canport;
	
	// sOdometry _mcuOdometry;		
	// sMcuBrkAcc _mcuBrkAcc;
	// sMcuSAS _mcuSAS;

	sAPC_CMD_State _APC_CMD_State;
	sVEH_STATE _VEH_STATE;
	sWHL_SPD _WHL_SPD;
};