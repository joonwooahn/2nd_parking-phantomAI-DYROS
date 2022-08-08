#ifndef Q_MOC_RUN
#include "../inc/CAN_MCU.h"
#endif
#include <math.h>
 
#define FALSE 0
#define TRUE 1
#define COUT false

#define UINT8 uint8_t
#define UINT16 uint16_t

CAN_MCU::CAN_MCU(int can_I)  	
{
	_canport = new eCAN(can_I);

}

CAN_MCU::~CAN_MCU()
{
	delete _canport;
	_canport = NULL;
}


bool CAN_MCU::Get_APC_CMD_State()
{
	UINT8 *pData;
	UINT16 Temp;
	int bit;	

	if (_canport->APC_CMD_State.updateFlag == TRUE) {			
		pData = _canport->APC_CMD_State.abData;

		int dataLen = 0, decimal = 0, factor = 0;

//////// APC_CmdState 0|4@1+ (1, 0)
		dataLen = 4;	factor = 1;
		uint8_t bitArrAPC_CmdState[dataLen] = {0,};
		bitArrAPC_CmdState[0] = pData[0] >> 3 & 1;
		bitArrAPC_CmdState[1] = pData[0] >> 2 & 1;
		bitArrAPC_CmdState[2] = pData[0] >> 1 & 1;
		bitArrAPC_CmdState[3] = pData[0] >> 0 & 1;
		
		decimal = 0;
		for(int i = 0 ; i < dataLen ; i++) 
			decimal = (decimal << 1) + bitArrAPC_CmdState[i];

		_APC_CMD_State.APC_CmdState = (int)decimal*factor;
		if (COUT) printf("_APC_CMD_State.APC_CmdState: %d \n", _APC_CMD_State.APC_CmdState);
////////

//////// APC_GearCmdState 4|4@1+ (1, 0)
		dataLen = 4;	factor = 1;	
		uint8_t bitArrAPC_GearCmdState[dataLen] = {0,};
		bitArrAPC_GearCmdState[0] = pData[0] >> 7 & 1;
		bitArrAPC_GearCmdState[1] = pData[0] >> 6 & 1;
		bitArrAPC_GearCmdState[2] = pData[0] >> 5 & 1;
		bitArrAPC_GearCmdState[3] = pData[0] >> 4 & 1;
		
		decimal = 0;
		for(int i = 0 ; i < dataLen ; i++) 
			decimal = (decimal << 1) + bitArrAPC_GearCmdState[i];
			
		_APC_CMD_State.APC_GearCmdState = (int)decimal*factor;
		if (COUT) printf("_APC_CMD_State.APC_GearCmdState: %d \n", _APC_CMD_State.APC_GearCmdState);
////////

//////// CmdStateAliveCounter 4|4@1+ (1, 0)
		dataLen = 4;	factor = 1;	
		uint8_t bitArrCmdStateAliveCounter[dataLen] = {0,};
		bitArrCmdStateAliveCounter[0] = pData[1] >> 3 & 1;
		bitArrCmdStateAliveCounter[1] = pData[1] >> 2 & 1;
		bitArrCmdStateAliveCounter[2] = pData[1] >> 1 & 1;
		bitArrCmdStateAliveCounter[3] = pData[1] >> 0 & 1;
		
		decimal = 0;
		for(int i = 0 ; i < dataLen ; i++) 
			decimal = (decimal << 1) + bitArrCmdStateAliveCounter[i];
			
		_APC_CMD_State.CmdStateAliveCounter = (int)decimal*factor;
		if (COUT) printf("_APC_CMD_State.CmdStateAliveCounter: %d \n", _APC_CMD_State.CmdStateAliveCounter);
////////

		_canport->APC_CMD_State.updateFlag = FALSE;
		return true;
	} else {	
		return false;
	}
}

bool CAN_MCU::Get_VEH_STATE ()
{
	UINT8 *pData;
	UINT16 Temp;
	int bit;	

	if (_canport->VEH_STATE.updateFlag == TRUE) {	
		
		pData = _canport->VEH_STATE.abData;	//UINT8

		// init
		int dataLen = 0, sign = 0, decimal = 0, signed_decimal = 0;	
		double signVal = 0.0, factor = 0.0;

//////// LatAccel 0|11@1- (0.01, 0) m/s^2 // Name 0|dataLen@does_this_value_has_sign- (factor, 0)
		dataLen = 11;	sign = 1;	factor = 0.01;	
		uint8_t bitArrLatAccel[dataLen] = {0,};
		// for (int i = dataLen-1; i >= 0; --i) {
		// 	if (i >= 8)	{
		// 		bitArrLatAccel[dataLen-1-i] = pData[1] >> (i-8) & 1;
		// 		printf("%d", bitArrLatAccel[dataLen-1-i]);
		// 	}
		// 	else {
		// 		bitArrLatAccel[dataLen-1-i] = pData[0] >> i & 1;
		// 		printf(",%d", bitArrLatAccel[dataLen-1-i]);
		// 	}
		// }
		// operator '>>' means 'right-shift'
		bitArrLatAccel[0] = pData[1] >> 2 & 1;
		bitArrLatAccel[1] = pData[1] >> 1 & 1;
		bitArrLatAccel[2] = pData[1] >> 0 & 1;
		bitArrLatAccel[3] = pData[0] >> 7 & 1;
		bitArrLatAccel[4] = pData[0] >> 6 & 1;
		bitArrLatAccel[5] = pData[0] >> 5 & 1;
		bitArrLatAccel[6] = pData[0] >> 4 & 1;
		bitArrLatAccel[7] = pData[0] >> 3 & 1;
		bitArrLatAccel[8] = pData[0] >> 2 & 1;
		bitArrLatAccel[9] = pData[0] >> 1 & 1;
		bitArrLatAccel[10]= pData[0] >> 0 & 1;

		// printf("%d", bitArrLatAccel[0]);

		if (sign)   signVal = bitArrLatAccel[0] == 0 ? +1.0 : -1.0;// if the value has signed
		else		signVal = 1.0; // if not, bitrrLatAccel[0] is included in 'the value'

		signed_decimal = (int)signVal * bitArrLatAccel[0] * pow(2, dataLen-1); // signed_bit
		decimal = 0;
		for(int i = sign ; i < dataLen ; i++) // if 'decimal' is printed out, it will be showed as a decimal value
			decimal = (decimal << 1) + bitArrLatAccel[i];
		if (sign && signVal < 0) decimal = signed_decimal+decimal; //if bitArrLatAccell[0] == 0 then 'decimal is DECIMAL'

		_VEH_STATE.LatAccel = (double)decimal*factor;
		if (COUT) printf("_VEH_STATE.LatAccel: %f m/s^2 \n", _VEH_STATE.LatAccel);
////////

//////// LongAccel 11|11@1+ (0.01, 0) m/s^2
		dataLen = 11;	sign = 0;	factor = 0.01;	
		uint8_t bitArrLongAccel[dataLen] = {0,};
		bitArrLongAccel[0] = pData[2] >> 5 & 1;
		bitArrLongAccel[1] = pData[2] >> 4 & 1;
		bitArrLongAccel[2] = pData[2] >> 3 & 1;
		bitArrLongAccel[3] = pData[2] >> 2 & 1;
		bitArrLongAccel[4] = pData[2] >> 1 & 1;
		bitArrLongAccel[5] = pData[2] >> 0 & 1;
		bitArrLongAccel[6] = pData[1] >> 7 & 1;
		bitArrLongAccel[7] = pData[1] >> 6 & 1;
		bitArrLongAccel[8] = pData[1] >> 5 & 1;
		bitArrLongAccel[9] = pData[1] >> 4 & 1;
		bitArrLongAccel[10]= pData[1] >> 3 & 1;

		decimal = 0;
		for(int i = sign ; i < dataLen ; i++) 
			decimal = (decimal << 1) + bitArrLongAccel[i];
		
		_VEH_STATE.LongAccel = (double)decimal*factor;
		if (COUT) printf("_VEH_STATE.LongAccel: %f m/s^2 \n", _VEH_STATE.LongAccel);
////////

//////// YawRate 22|13@1- (0.01, 0) m/s^2
		dataLen = 13;	sign = 1;	factor = 0.01;	
		uint8_t bitArrYawRate[dataLen] = {0,};
		bitArrYawRate[0] = pData[4] >> 2 & 1;
		bitArrYawRate[1] = pData[4] >> 1 & 1;
		bitArrYawRate[2] = pData[4] >> 0 & 1;
		bitArrYawRate[3] = pData[3] >> 7 & 1;
		bitArrYawRate[4] = pData[3] >> 6 & 1;
		bitArrYawRate[5] = pData[3] >> 5 & 1;
		bitArrYawRate[6] = pData[3] >> 4 & 1;
		bitArrYawRate[7] = pData[3] >> 3 & 1;
		bitArrYawRate[8] = pData[3] >> 2 & 1;
		bitArrYawRate[9] = pData[3] >> 1 & 1;
		bitArrYawRate[10]= pData[3] >> 0 & 1;
		bitArrYawRate[11]= pData[2] >> 7 & 1;
		bitArrYawRate[12]= pData[2] >> 6 & 1;
			
		if (sign)   signVal = bitArrYawRate[0] == 0 ? +1.0 : -1.0;
		else		signVal = 1.0;
		
		signed_decimal = (int)signVal*bitArrYawRate[0]*pow(2, dataLen-1);
		decimal = 0;
		for(int i = sign ; i < dataLen ; i++)  
			decimal = (decimal << 1) + bitArrYawRate[i];
		if (sign && signVal < 0) decimal = signed_decimal+decimal;

		_VEH_STATE.YawRate = (double)decimal*factor;
		if (COUT) printf("_VEH_STATE.YawRate: %f \n", _VEH_STATE.YawRate);
////////

//////// GearState 35|4@1+ (1.0, 0) m/s^2
		dataLen = 4;	sign = 0;	factor = 1.0;	
		uint8_t bitArrGearState[dataLen] = {0,};
		bitArrGearState[0] = pData[4] >> 6 & 1;
		bitArrGearState[1] = pData[4] >> 5 & 1;
		bitArrGearState[2] = pData[4] >> 4 & 1;
		bitArrGearState[3] = pData[4] >> 3 & 1;
		
		decimal = 0;
		for(int i = sign ; i < dataLen ; i++)
			decimal = (decimal << 1) + bitArrGearState[i];

		_VEH_STATE.GearState = (int)decimal*factor;
		if (COUT) printf("_VEH_STATE.GearState: %d \n", _VEH_STATE.GearState);
////////

//////// SteerAngle 40|16@1- (0.1, 0) Deg
		dataLen = 16;	sign = 1;	factor = 0.1;	
		uint8_t bitArrSteerAngle[dataLen] = {0,};
		bitArrSteerAngle[0] = pData[6] >> 7 & 1;
		bitArrSteerAngle[1] = pData[6] >> 6 & 1;
		bitArrSteerAngle[2] = pData[6] >> 5 & 1;
		bitArrSteerAngle[3] = pData[6] >> 4 & 1;
		bitArrSteerAngle[4] = pData[6] >> 3 & 1;
		bitArrSteerAngle[5] = pData[6] >> 2 & 1;
		bitArrSteerAngle[6] = pData[6] >> 1 & 1;
		bitArrSteerAngle[7] = pData[6] >> 0 & 1;
		bitArrSteerAngle[8] = pData[5] >> 7 & 1;
		bitArrSteerAngle[9] = pData[5] >> 6 & 1;
		bitArrSteerAngle[10]= pData[5] >> 5 & 1;
		bitArrSteerAngle[11]= pData[5] >> 4 & 1;
		bitArrSteerAngle[12]= pData[5] >> 3 & 1;
		bitArrSteerAngle[13]= pData[5] >> 2 & 1;
		bitArrSteerAngle[14]= pData[5] >> 1 & 1;
		bitArrSteerAngle[15]= pData[5] >> 0 & 1;
		
		if (sign)   signVal = bitArrSteerAngle[0] == 0 ? +1.0 : -1.0;
		else		signVal = 1.0;

		signed_decimal = (int)signVal*bitArrSteerAngle[0]*pow(2, dataLen-1);
		// printf("signed_decimal: %d\n", signed_decimal);
		// printf("%d", bitArrSteerAngle[0]);
		decimal = 0;
		for(int i = sign ; i < dataLen ; i++) // {
			decimal = (decimal << 1) + bitArrSteerAngle[i];
			// printf("%d", bitArrSteerAngle[i]);
		// }
		// printf("\n decimal: %d \n", decimal);
		if (sign && signVal < 0) decimal = signed_decimal+decimal;

		_VEH_STATE.SteerAngle = (double)decimal*factor;
		if (COUT) printf("_VEH_STATE.SteerAngle: %f Deg \n", _VEH_STATE.SteerAngle);
////////

		_canport->VEH_STATE.updateFlag = FALSE;
		if (COUT) printf("\n");
		return true;// if the value is being as input
	} else {	
		return false;
	}
}


bool CAN_MCU::Get_WHL_SPD ()
{
	UINT8 *pData;
	UINT16 Temp;
	int bit;	

	if (_canport->WHL_SPD.updateFlag == TRUE) {			
		pData = _canport->WHL_SPD.abData;

		int dataLen = 0, decimal = 0; 
		double factor = 0.0;

//////// SG_ WhlSpd_FL : 0|14@1+ (0.01,0)
		dataLen = 14;	factor = 0.01;
		uint8_t bitArrWhlSpd_FL[dataLen] = {0,};
		bitArrWhlSpd_FL[0] =  pData[1] >> 5 & 1;
		bitArrWhlSpd_FL[1] =  pData[1] >> 4 & 1;
		bitArrWhlSpd_FL[2] =  pData[1] >> 3 & 1;
		bitArrWhlSpd_FL[3] =  pData[1] >> 2 & 1;
		bitArrWhlSpd_FL[4] =  pData[1] >> 1 & 1;
		bitArrWhlSpd_FL[5] =  pData[1] >> 0 & 1;
		bitArrWhlSpd_FL[6] =  pData[0] >> 7 & 1;
		bitArrWhlSpd_FL[7] =  pData[0] >> 6 & 1;
		bitArrWhlSpd_FL[8] =  pData[0] >> 5 & 1;
		bitArrWhlSpd_FL[9] =  pData[0] >> 4 & 1;
		bitArrWhlSpd_FL[10] = pData[0] >> 3 & 1;
		bitArrWhlSpd_FL[11] = pData[0] >> 2 & 1;
		bitArrWhlSpd_FL[12] = pData[0] >> 1 & 1;
		bitArrWhlSpd_FL[13] = pData[0] >> 0 & 1;

		decimal = 0;
		for(int i = 0 ; i < dataLen ; i++) 
			decimal = (decimal << 1) + bitArrWhlSpd_FL[i];

		_WHL_SPD.WhlSpd_FL = decimal*factor;
		if (COUT) 
			printf("_WHL_SPD.WhlSpd_FL: %2f \n", _WHL_SPD.WhlSpd_FL);
////////

//////// SG_ WhlSpd_FR : 16|14@1+ (0.01,0)
		dataLen = 14;	factor = 0.01;
		uint8_t bitArrWhlSpd_FR[dataLen] = {0,};
		bitArrWhlSpd_FR[0] =  pData[3] >> 5 & 1;
		bitArrWhlSpd_FR[1] =  pData[3] >> 4 & 1;
		bitArrWhlSpd_FR[2] =  pData[3] >> 3 & 1;
		bitArrWhlSpd_FR[3] =  pData[3] >> 2 & 1;
		bitArrWhlSpd_FR[4] =  pData[3] >> 1 & 1;
		bitArrWhlSpd_FR[5] =  pData[3] >> 0 & 1;
		bitArrWhlSpd_FR[6] =  pData[2] >> 7 & 1;
		bitArrWhlSpd_FR[7] =  pData[2] >> 6 & 1;
		bitArrWhlSpd_FR[8] =  pData[2] >> 5 & 1;
		bitArrWhlSpd_FR[9] =  pData[2] >> 4 & 1;
		bitArrWhlSpd_FR[10] = pData[2] >> 3 & 1;
		bitArrWhlSpd_FR[11] = pData[2] >> 2 & 1;
		bitArrWhlSpd_FR[12] = pData[2] >> 1 & 1;
		bitArrWhlSpd_FR[13] = pData[2] >> 0 & 1;

		decimal = 0;
		for(int i = 0 ; i < dataLen ; i++) 
			decimal = (decimal << 1) + bitArrWhlSpd_FR[i];

		_WHL_SPD.WhlSpd_FR = decimal*factor;
		if (COUT) 
			printf("_WHL_SPD.WhlSpd_FR: %2f \n", _WHL_SPD.WhlSpd_FR);
////////

//////// SG_ WhlSpd_RL : 32|14@1+ (0.01,0)
		dataLen = 14;	factor = 0.01;
		uint8_t bitArrWhlSpd_RL[dataLen] = {0,};
		bitArrWhlSpd_RL[0] =  pData[5] >> 5 & 1;
		bitArrWhlSpd_RL[1] =  pData[5] >> 4 & 1;
		bitArrWhlSpd_RL[2] =  pData[5] >> 3 & 1;
		bitArrWhlSpd_RL[3] =  pData[5] >> 2 & 1;
		bitArrWhlSpd_RL[4] =  pData[5] >> 1 & 1;
		bitArrWhlSpd_RL[5] =  pData[5] >> 0 & 1;
		bitArrWhlSpd_RL[6] =  pData[4] >> 7 & 1;
		bitArrWhlSpd_RL[7] =  pData[4] >> 6 & 1;
		bitArrWhlSpd_RL[8] =  pData[4] >> 5 & 1;
		bitArrWhlSpd_RL[9] =  pData[4] >> 4 & 1;
		bitArrWhlSpd_RL[10] = pData[4] >> 3 & 1;
		bitArrWhlSpd_RL[11] = pData[4] >> 2 & 1;
		bitArrWhlSpd_RL[12] = pData[4] >> 1 & 1;
		bitArrWhlSpd_RL[13] = pData[4] >> 0 & 1;

		decimal = 0;
		for(int i = 0 ; i < dataLen ; i++) 
			decimal = (decimal << 1) + bitArrWhlSpd_RL[i];

		_WHL_SPD.WhlSpd_RL = decimal*factor;
		if (COUT) 
			printf("_WHL_SPD.WhlSpd_RL: %2f \n", _WHL_SPD.WhlSpd_RL);
////////

//////// SG_ WhlSpd_RR : 48|14@1+ (0.01,0)
		dataLen = 14;	factor = 0.01;
		uint8_t bitArrWhlSpd_RR[dataLen] = {0,};
		bitArrWhlSpd_RR[0] =  pData[7] >> 5 & 1;
		bitArrWhlSpd_RR[1] =  pData[7] >> 4 & 1;
		bitArrWhlSpd_RR[2] =  pData[7] >> 3 & 1;
		bitArrWhlSpd_RR[3] =  pData[7] >> 2 & 1;
		bitArrWhlSpd_RR[4] =  pData[7] >> 1 & 1;
		bitArrWhlSpd_RR[5] =  pData[7] >> 0 & 1;
		bitArrWhlSpd_RR[6] =  pData[6] >> 7 & 1;
		bitArrWhlSpd_RR[7] =  pData[6] >> 6 & 1;
		bitArrWhlSpd_RR[8] =  pData[6] >> 5 & 1;
		bitArrWhlSpd_RR[9] =  pData[6] >> 4 & 1;
		bitArrWhlSpd_RR[10] = pData[6] >> 3 & 1;
		bitArrWhlSpd_RR[11] = pData[6] >> 2 & 1;
		bitArrWhlSpd_RR[12] = pData[6] >> 1 & 1;
		bitArrWhlSpd_RR[13] = pData[6] >> 0 & 1;

		decimal = 0;
		for(int i = 0 ; i < dataLen ; i++) 
			decimal = (decimal << 1) + bitArrWhlSpd_RR[i];

		_WHL_SPD.WhlSpd_RR = decimal*factor;
		if (COUT) 
			printf("_WHL_SPD.WhlSpd_RR: %2f \n", _WHL_SPD.WhlSpd_RR);
////////

		_canport->WHL_SPD.updateFlag = FALSE;
		return true;
	} else {	
		return false;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////

// bool CAN_MCU::GetOdometrydata ()	
// {
// 	UINT8 *pData;
// 	UINT16 Temp;
	
// 	if (_canport->CAR_WHL_SPD_RAW.updateFlag == TRUE){	
// 		pData = _canport->CAR_WHL_SPD_RAW.abData;
// 		Temp = pData[1]<<8;
// 		Temp |= pData[0];			
// 		_mcuOdometry.WheelSpeed_RR	= Temp * 0.01*1000./(60.*60.);	
	
// 		Temp = pData[3]<<8;
// 		Temp |= pData[2];
// 		_mcuOdometry.WheelSpeed_RL	= Temp * 0.01*1000./(60.*60.); 

// 		Temp = pData[5]<<8;
// 		Temp |= pData[4];			
// 		_mcuOdometry.WheelSpeed_FR	= Temp * 0.01*1000./(60.*60.);	
	
// 		Temp = pData[7]<<8;
// 		Temp |= pData[6];
// 		_mcuOdometry.WheelSpeed_FL	= Temp * 0.01*1000./(60.*60.); 

// 		_canport->CAR_WHL_SPD_RAW.updateFlag = FALSE;
// 		return true;
// 	}else
// 		return false;
// }

// bool CAN_MCU::GetMcuBrkAcc ()
// {
// 	UINT8 *pData;
// 	UINT16 Temp;
// 	int bit;	

// 	if (_canport->CAR_MONITOR_BRK_ACC.updateFlag == TRUE){	
		
// 		pData = _canport->CAR_MONITOR_BRK_ACC.abData;
	
// 		bit = (0x1 & pData[0]);
// 		_mcuBrkAcc.BrkCtlBit = bit;

// 		bit = (0x1 & (pData[0] >> 1));
// 		_mcuBrkAcc.AccCtlBit = bit;

// 		Temp = pData[1];
// 		_mcuBrkAcc.ArmAngle = Temp;

// 		Temp = pData[2];
// 		_mcuBrkAcc.AccVoltage = Temp;

// 		_canport->CAR_MONITOR_BRK_ACC.updateFlag = FALSE;
// 		return true;
// 	}else{	
// 		return false;
// 	}
// }

// bool CAN_MCU::GetMcuSAS ()
// {
// 	UINT8 *pData;
// 	UINT16 Temp;
// 	int bit;	

// 	if (_canport->CAR_MONITOR_STEER.updateFlag == TRUE){	
		
// 		pData = _canport->CAR_MONITOR_STEER.abData;
	
// 		bit = (0x1 & pData[0]);
// 		_mcuSAS.SteerCtlBit = bit;

// 		Temp = pData[2]<<8;
// 		Temp |= pData[1];	

// 		_mcuSAS.SteeringAngle = Temp * 0.1;
	
// 		if ( _mcuSAS.SteeringAngle >= 3276.8 )
// 			_mcuSAS.SteeringAngle -= 6553.5;

// 		_canport->CAR_MONITOR_STEER.updateFlag = FALSE;
// 		return true;
// 	}else{	
// 		return false;
// 	}
// }