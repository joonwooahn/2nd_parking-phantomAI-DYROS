#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
// #include <QtWidgets/QMainWindow> /// jw delete
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include "../../../build/phantom_can/ui_phantom_can_gui.h"
#include "CAN_MCU.h"
#endif

class QNode : public QThread {
	Q_OBJECT
public:
	QNode(int argc, char** argv, Ui::MainWindow* ui, CAN_MCU* _pCanMCU, QObject *parent = 0);

	void run();
	void stop();
	void GetMotionMsg(float& sas, float& acc, float& brk);
	// ros::Subscriber steerSub, velocityCtrlSub;
	ros::Subscriber Sub_ControlCommand, Sub_localization;
	ros::Publisher Pub_vehicle_state;	//, Pub_FutureTrajectory;

	// void VelModeTransmit(int mode, float brkAng, float accAng);
	// void SteerModeTransmit(int mode, float steerAng);
	// void CallbackSteer(const std_msgs::Float32MultiArray::ConstPtr& msg);
	// void CallbackVelocity(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void CallbackLocalizationData(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void CallbackCtrlCmd(const std_msgs::Float32MultiArray::ConstPtr& msg);
private:
	int init_argc;
	char** init_argv;
	bool threadStop;
	Ui::MainWindow* _ui;
	
	CAN_MCU* m_pCanMCU;
public:
	double m_currentSteer, m_curr_vel;
	double m_acc;
	double m_vel;
	double m_brk;
	double m_steer;
	int m_gear;
	double m_err;
	double m_accGain;
	double m_car2switchDist;
	int m_leftSwitchingCnt;
	
	bool m_steerMsgUpdated;
	bool m_velocityCtrlMsgUpdated;
	bool m_gearCtrlMsgUpdated;
	
	bool m_steerMode;
	bool m_velMode;	
	bool m_autonomousMode;

	bool m_AutoParkingOn, m_APC_Enable, m_StopRequest, m_GearShiftEnable, m_finishFLAG, m_manual_state_control;
	int m_APC_CmdState, m_gear_state, m_ApcTestMode;

	double m_WhlSpd_RR, m_WhlSpd_RL, m_WhlSpd_FR, m_WhlSpd_FL;

	void phantom_APC_CMD(bool AutoParkingOn, bool APC_Enable, bool StopRequest, double AccelCmd, double StrAngCmd, bool GearShiftEnable, int GearShiftCmd, int ApcTestMode, int CmdStateAliveCounter);
};