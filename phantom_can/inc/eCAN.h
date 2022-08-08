#ifndef Q_MOC_RUN
// #include "../inc/ECI10A.h"
#include "../inc/ECI113.h"
#include "../inc/ECI_pshpack1.h"
#include "../inc/ECI_poppack.h"
#include <stdint.h>
#endif


#define UINT8 uint8_t
#define UINT16 uint16_t
#define UINT32 uint32_t

struct sCanData{
	UINT32	dwTime;
	UINT8	abData[8];
	bool	updateFlag;
	DWORD	dwMsgId;	// 추가.
	char	text[256];	// 추가.
};


class eCAN
{
public:
	ECI_CTRL_HDL hCanCtl[1];

	sCanData APC_CMD_State;	// add
	sCanData VEH_STATE;		// add
	sCanData WHL_SPD;		// add

	// sCanData CAR_MONITOR_BRK_ACC;
	// sCanData CAR_MONITOR_STEER;
	// sCanData CAR_MONITOR_WHL_SPD;
	// sCanData CAR_WHL_SPD_RAW;


	eCAN(int can_I);
	~eCAN();

	void GetCanMsg(sCanData *TargetData, ECI_CTRL_MESSAGE *SourceData);
	void ReadCanBuffer(void);
	void EciPrintHwInfo(const ECI_HW_INFO* pstcHwInfo);
	void TransmitCanMsg(UINT8 *msgData, UINT32 id);	
	void EciPrintCtrlCapabilities(const ECI_CTRL_CAPABILITIES* pstcCtrlCaps);

	ECI_RESULT EciGetNthCtrlOfClass(const ECI_HW_INFO* pstcHwInfo, e_CTRLCLASS eCtrlClass, DWORD dwRelCtrlIndex, DWORD* pdwCtrIndex);

private:
	
};