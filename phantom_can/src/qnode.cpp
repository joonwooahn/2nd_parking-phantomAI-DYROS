#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../inc/qnode.h" 
#endif

#define COUT false
#define SIGN(x) ((x >= 0) ? 1.0 : -1.0)
#define KMpH2MpS 0.277777
#define Mps2KMpH 3.6
#define DISTANCE(x1, y1, x2, y2) sqrt((x1 - x2)*(x1 - x2) +(y1 - y2)*(y1 - y2))
#define _RAD2DEG 180 / M_PI
#define _DEG2RAD M_PI / 180

// Constructor
QNode::QNode(int argc, char** argv, Ui::MainWindow* ui,CAN_MCU* _pCanMCU, QObject *parent)
    : QThread(parent), init_argc(argc), init_argv(argv),m_pCanMCU(_pCanMCU), _ui(ui)
{
  m_autonomousMode = false;
  m_velocityCtrlMsgUpdated = false;
  m_steerMsgUpdated = false;

  threadStop = false;
  ros::init(init_argc, init_argv, "CAN_MCU");
  ros::NodeHandle node;

  m_finishFLAG = false;
  m_currentSteer = m_curr_vel = 0.0;
	m_acc = 0.0;    //double
	m_vel = 0.0;    //double
	m_brk = 0.0;    //double
	m_steer = 0.0;  //double
	m_gear = 0;  //int   
	m_err = 0.0;    //double
  m_gear_state = 0;
  m_accGain = 0.01;
  m_car2switchDist = 0.0;
  m_leftSwitchingCnt = 0;

  m_AutoParkingOn = false;
  m_APC_Enable = false;
  m_StopRequest = false;
  m_GearShiftEnable = false;
  m_ApcTestMode = 0;  // 0: full, 1: steer, 2: accel only

  m_manual_state_control = false;

  // steerSub = n.subscribe("SteerAngleData", 10, &QNode::CallbackSteer, this);
  // velocityCtrlSub = n.subscribe("VelocityCtrlData", 10, &QNode::CallbackVelocity, this);
  // ControlCommandSub = n.subscribe("/PhantomCAN_APC_CMD", 1, &QNode::CallbackCtrlCmd, this);
  Sub_localization = node.subscribe("/LocalizationData", 1, &QNode::CallbackLocalizationData, this);
  Sub_ControlCommand = node.subscribe("/Control_Command", 1, &QNode::CallbackCtrlCmd, this);
  Pub_vehicle_state = node.advertise<std_msgs::Float32MultiArray>("/CAN/vehicle_state", 1);
}

void QNode::stop() {
  threadStop = true;
}

void QNode::CallbackLocalizationData(const std_msgs::Float32MultiArray::ConstPtr& msg){
  m_curr_vel = msg->data.at(3);    // to [m/s]   
}

void QNode::phantom_APC_CMD(bool AutoParkingOn, bool APC_Enable, bool StopRequest, double AccelCmd, double StrAngCmd, bool GearShiftEnable, int GearShiftCmd, int ApcTestMode, int CmdStateAliveCounter) {
    uint8_t CanDataBuf[8];
    uint8_t* pCanData = CanDataBuf;
	
    char bitArr[8][8];

    for (int i = 0; i < 8; i++)   
      for (int j = 0; j < 8; j++) 
        bitArr[i][j] = 0;
    
    // AutoParkingOn 0|1@1+ (1, 0)
    if (AutoParkingOn)  bitArr[0][0] = 1;
    else                bitArr[0][0] = 0;
    // APC_Enable 1|1@1+ (1, 0)
    if (APC_Enable)   bitArr[0][1] = 1;
    else              bitArr[0][1] = 0;
    // StopRequest 2|1@1+ (1, 0)
    if (StopRequest)  bitArr[0][2] = 1;
    else              bitArr[0][2] = 0;

    int dataLen = 0, sign = 0, signVal = 0;
    double factor = 0.0;
    
//// AccelCmd 3|11@1- (0.01, 0) 540 --> 54000
    sign = 1; dataLen = 11 - sign; factor = 100.0; // (this factor is '')
    if (sign) signVal = AccelCmd >= 0 ? 0 : 1;
    else      signVal = 0;
    
    if (AccelCmd < 0.0) AccelCmd = pow(2, dataLen) - round(abs(AccelCmd*factor));
    else                AccelCmd = round((abs)(AccelCmd*factor));
 
    int bitArrAccelCmd[dataLen] = {0,};
    for (int i = 0; AccelCmd > 0; i++) { // from Decimal to Bit
      bitArrAccelCmd[i] = (int)AccelCmd % 2; // remainder
      AccelCmd = (int)AccelCmd / 2; // 
    }
    
    if (COUT) {
      printf("\n AccelCmd: %d", (uint8_t)signVal);
      for (int i = dataLen-1; i >= 0; i--)
        printf("%d", bitArrAccelCmd[i]);
    }

    if (m_velMode && (m_ApcTestMode==2 || m_ApcTestMode==0)) {
      bitArr[0][3] = bitArrAccelCmd[0];
      bitArr[0][4] = bitArrAccelCmd[1];
      bitArr[0][5] = bitArrAccelCmd[2];
      bitArr[0][6] = bitArrAccelCmd[3];
      bitArr[0][7] = bitArrAccelCmd[4];
      bitArr[1][0] = bitArrAccelCmd[5];
      bitArr[1][1] = bitArrAccelCmd[6];
      bitArr[1][2] = bitArrAccelCmd[7];
      bitArr[1][3] = bitArrAccelCmd[8];
      bitArr[1][4] = bitArrAccelCmd[9];
      bitArr[1][5] = (uint8_t)signVal;
    }
////

//// StrAngCmd 16|16@1- (0.1, 0) 540 --> 5400
    sign = 1; dataLen = 16 - sign;  factor = 10.0;
    if (sign) signVal = StrAngCmd >= 0 ? 0 : 1;
    else      signVal = 0;
    
    if (StrAngCmd < 0.0)  StrAngCmd = pow(2, dataLen) - round(abs(StrAngCmd*factor)); //
    else                  StrAngCmd = round((abs)(StrAngCmd*factor));
 
    int bitArrStrAngCmd[dataLen] = {0,};
    for (int i = 0; StrAngCmd > 0; i++){
      bitArrStrAngCmd[i] = (int)StrAngCmd % 2;
      StrAngCmd = (int)StrAngCmd/2;
    }

    if (COUT) {
      printf("\nStrAngCmd: %f / %d", StrAngCmd, (uint8_t)signVal);
      for (int i = dataLen-1; i >= 0; i--)
        printf("%d", bitArrStrAngCmd[i]);
    }
 
    if (m_steerMode && (m_ApcTestMode==1 || m_ApcTestMode==0)) {
      bitArr[2][0] = bitArrStrAngCmd[0];
      bitArr[2][1] = bitArrStrAngCmd[1];
      bitArr[2][2] = bitArrStrAngCmd[2];
      bitArr[2][3] = bitArrStrAngCmd[3];
      bitArr[2][4] = bitArrStrAngCmd[4];
      bitArr[2][5] = bitArrStrAngCmd[5];
      bitArr[2][6] = bitArrStrAngCmd[6];
      bitArr[2][7] = bitArrStrAngCmd[7];
      bitArr[3][0] = bitArrStrAngCmd[8];
      bitArr[3][1] = bitArrStrAngCmd[9];
      bitArr[3][2] = bitArrStrAngCmd[10];
      bitArr[3][3] = bitArrStrAngCmd[11];
      bitArr[3][4] = bitArrStrAngCmd[12];
      bitArr[3][5] = bitArrStrAngCmd[13];
      bitArr[3][6] = bitArrStrAngCmd[14];
      bitArr[3][7] = (uint8_t)signVal;
    }
////

//// GearShiftEnable 32|1@1+ (1, 0)
    if (GearShiftEnable)  bitArr[4][0] = 1;
    else                  bitArr[4][0] = 0;  

//// GearShiftCmd 33|4@1+ (1, 0)
    sign = 0; dataLen = 4; factor = 1;
    
    int bitArrGearShiftCmd[dataLen] = {0,};
    // GearShiftCmd *= factor;
 
    for (int i = 0; GearShiftCmd > 0; i++){
      bitArrGearShiftCmd[i] = (int)GearShiftCmd % 2;
      GearShiftCmd = (int)GearShiftCmd/2;
    }

    // if (m_GearShiftEnable && m_APC_CmdState==5) {
    bitArr[4][1] = bitArrGearShiftCmd[0];
    bitArr[4][2] = bitArrGearShiftCmd[1];
    bitArr[4][3] = bitArrGearShiftCmd[2];
    bitArr[4][4] = bitArrGearShiftCmd[3];
    // }
////

//// ApcTestMode 37|3@1+ (1,0)
    sign = 0; dataLen = 3; factor = 1;
    
    int bitArrApcTestMode[dataLen] = {0,};
    // ApcTestMode *= factor;
 
    for (int i = 0; ApcTestMode > 0; i++){
      bitArrApcTestMode[i] = (int)ApcTestMode % 2;
      ApcTestMode = (int)ApcTestMode/2;
    }

    if (COUT) {
      printf("\nApcTestMode: ");
      for (int i = dataLen-1; i >= 0; i--)
        printf("%d", bitArrApcTestMode[i]);
    }

    bitArr[4][5] = bitArrApcTestMode[0];
    bitArr[4][6] = bitArrApcTestMode[1];
    bitArr[4][7] = bitArrApcTestMode[2];
////

//// CmdStateAliveCounter 40|4@1+ (1, 0)
    sign = 0; dataLen = 4; factor = 1;
    
    int bitArrCmdStateAliveCounter[dataLen] = {0,};
    // CmdStateAliveCounter *= factor;
 
    for (int i = 0; CmdStateAliveCounter > 0; i++){
      bitArrCmdStateAliveCounter[i] = (int)CmdStateAliveCounter % 2;
      CmdStateAliveCounter = (int)CmdStateAliveCounter/2;
    }

    if (COUT) {
      printf("\nbitArrCmdStateAliveCounter: ");
      for (int i = dataLen-1; i >= 0; i--)
        printf("%d", bitArrCmdStateAliveCounter[i]);
    }

    bitArr[5][0] = bitArrCmdStateAliveCounter[0];
    bitArr[5][1] = bitArrCmdStateAliveCounter[1];
    bitArr[5][2] = bitArrCmdStateAliveCounter[2];
    bitArr[5][3] = bitArrCmdStateAliveCounter[3];
////

    if (COUT) {
      cout << endl;
      for (int i = 0; i < 8; i++) {
        cout << i << " ";
        for (int j = 7; j >= 0; j--) {
          printf("%d",  bitArr[i][j]);
        } cout << endl;
      }cout << endl;
    }

    for (int i = 0; i < 8; i++) {
      int decimal = 0;
        for (int j = 8-1; j >= 0; j--) {
          decimal <<= 1;
          decimal += bitArr[i][j];
      }
      pCanData[i] = decimal;
      // printf("%d ", pCanData[i]);
    } 
    // printf("\n");

    m_pCanMCU->_canport->TransmitCanMsg(pCanData, 0x180);
}

void QNode::CallbackCtrlCmd(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  // m_AutoParkingOn = msg->data.at(0);
  // m_APC_Enable =    msg->data.at(1);
  // m_StopRequest =   msg->data.at(2);
  m_steer = -msg->data.at(0);
  m_steerMsgUpdated = true;

  m_vel = msg->data.at(1);
  m_velocityCtrlMsgUpdated = true;

  if (m_gear != msg->data.at(2)) { // if 'gear_switch' is needed,
    if (msg->data.at(2) == 1) //1: forward / -1: backward
      m_gear = 5;  // D
    else if (msg->data.at(2) == -1)
      m_gear = 7;  // R
    else 
      m_gear = 0;  // P
    m_gearCtrlMsgUpdated = true;
  }
  m_acc = msg->data.at(3)*m_accGain;  // acc
  // cout << msg->data.at(3) << endl;
  
  m_err = msg->data.at(4);  // cross-track error

  if (msg->data.at(5) == 1) m_finishFLAG = true; 
  else                        m_finishFLAG = false;

  if (m_finishFLAG)
      m_gear = 0;  // P

  m_car2switchDist   = msg->data.at(6);
  m_leftSwitchingCnt = msg->data.at(7);
  // cout << msg->data.at(0) << " " << msg->data.at(1) << " " <<msg->data.at(2)<< " " <<msg->data.at(3) << " " <<msg->data.at(4) << " " <<msg->data.at(5) << endl;
}	

void QNode::run() {
  ros::Rate loop_rate(10);  // 10hz 100ms, // control loop
    // ros::Rate loop_rate(20);  // 20hz 50ms
  // ros::Rate loop_rate(50);  // 50hz 20ms
  
  int cnttttt = 0;
 
  while (ros::ok())
  {
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(m_WhlSpd_RL);
    msg.data.push_back(m_WhlSpd_RR);
    msg.data.push_back(m_WhlSpd_FL);
    msg.data.push_back(m_WhlSpd_FR); 
    msg.data.push_back(m_currentSteer);
    Pub_vehicle_state.publish(msg);

    // cout << "cmd: "<< m_steer  << ", curr: " << m_currentSteer << endl;
    if (abs(m_steer - m_currentSteer) > 90.0)
      m_steer = m_currentSteer + SIGN((m_steer - m_currentSteer))*45.0;
  // cout << "------------ cmd: "<< m_steer  << ", curr: " << m_currentSteer << endl;
    
    if (cnttttt > 10) cnttttt = 0;

    if(threadStop == true)  break;
      
    if(m_pCanMCU) {
      // if (!m_manual_state_control) 
      {
          if (m_gear_state == m_gear) { // same gear 
              m_GearShiftEnable = false;
              if (m_APC_CmdState==3 || m_APC_CmdState==4) {  //APC CONTROL STOP || APC_CONTROL_NORMAL
                  m_StopRequest = false;
              }
          }
          else {  //gear change
            if (m_APC_CmdState==2) {  // APC_READY 
                m_StopRequest = true;
                m_GearShiftEnable = false;
                // for (int i = 0; i < 30 ; i++) 
                //   cout << "m_APC_CmdState: "<< m_APC_CmdState << ", m_StopRequest: " << m_StopRequest << ", m_GearShiftEnable: " << m_GearShiftEnable << ", m_gear: " << m_gear << ", m_gear_state: " << m_gear_state << endl;
            }
            else if (m_APC_CmdState==3) {  // APC CONTROL STOP
                m_StopRequest = true;
                m_GearShiftEnable = true;
                // for (int i = 0; i < 30 ; i++) 
                //   cout << "m_APC_CmdState: "<< m_APC_CmdState << ", m_StopRequest: " << m_StopRequest << ", m_GearShiftEnable: " << m_GearShiftEnable << ", m_gear: " << m_gear << ", m_gear_state: " << m_gear_state << endl;
            }
            else if (m_APC_CmdState==4) {  // APC_CONTROL_NORMAL
                m_StopRequest = true;
                m_GearShiftEnable = false;
                // for (int i = 0; i < 30 ; i++) 
                //   cout << "m_APC_CmdState: "<< m_APC_CmdState << ", m_StopRequest: " << m_StopRequest << ", m_GearShiftEnable: " << m_GearShiftEnable << ", m_gear: " << m_gear << ", m_gear_state: " << m_gear_state << endl;
            }
            else if (m_APC_CmdState==5) { // APC_CONTROL_GEARSHIFT
                m_GearShiftEnable = true;
                // for (int i = 0; i < 30 ; i++) 
                //   cout << "m_APC_CmdState: "<< m_APC_CmdState << ", m_StopRequest: " << m_StopRequest << ", m_GearShiftEnable: " << m_GearShiftEnable << ", m_gear: " << m_gear << ", m_gear_state: " << m_gear_state << endl;
            }
          }

        // if (m_APC_CmdState != 4) {
        //     m_acc = 0.0;
        //     m_steer = 0.0;
        // }

        if (m_finishFLAG) {
          m_gear = 0;
          m_acc = 0.0;
          m_steer = 0.0;
          
          if (m_gear_state == m_gear)  {
            m_GearShiftEnable = false;
            m_StopRequest = true; m_StopRequest = true; m_StopRequest = true;
            m_AutoParkingOn = false;
            m_APC_Enable = false;
          }
        }
      }
      phantom_APC_CMD( // inputs to control through CAN
      // bool AutoParkingOn, bool APC_Enable, bool StopRequest,
              m_AutoParkingOn,    m_APC_Enable,   m_StopRequest,
      // double AccelCmd, double StrAngCmd,
              m_acc,              m_steer, 
      // bool GearShiftEnable, int GearShiftCmd,
              m_GearShiftEnable,           m_gear,
      // int ApcTestMode (0: full, 1: steer, 2: accel only), int CmdStateAliveCounter
              m_ApcTestMode,                                  cnttttt++);
    }
    // cout << m_steer << ", " << m_acc << endl;

    ros::spinOnce();
    loop_rate.sleep();
  }

  qDebug("qnode Thread End");
}
