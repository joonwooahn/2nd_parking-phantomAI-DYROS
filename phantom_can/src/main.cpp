#ifndef Q_MOC_RUN
// #include <QtGui> /// jw delete
#include <QApplication>
#include <stdio.h>
#include "../inc/main.h"
#include <QTimer> /// jw add
#endif

CanReceiveThread::CanReceiveThread(CAN_MCU* _pCanMCU, QObject * parent)
    : QThread(parent)
    , m_pCanMCU(_pCanMCU)
{
  threadStop = false;
}

void CanReceiveThread::stop()
{
  threadStop = true;

}

void CanReceiveThread::run()
{
  qDebug("can Thread Start");
  while(!threadStop)
  {
    m_pCanMCU->_canport->ReadCanBuffer(); // each thread, the can_buffer is read

    m_pCanMCU->Get_APC_CMD_State();   // add
    m_pCanMCU->Get_VEH_STATE();       // add
    m_pCanMCU->Get_WHL_SPD();       // add

  //   m_pCanMCU->GetOdometrydata();
  //   m_pCanMCU->GetMcuBrkAcc();
  //   m_pCanMCU->GetMcuSAS();
  }

  threadStop = false;
  qDebug("can Thread End");
}


/////////////////////////////////////////////////////////////////////////////////

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , _argc(argc), _argv(argv)
{
  nMode = 6;
	
  ui.setupUi(this);
	
  connect( ui.DrivingMode1, SIGNAL(clicked()), this, SLOT(OnDrivingMode()) );
  connect( ui.DrivingMode2, SIGNAL(clicked()), this, SLOT(OnDrivingMode()) );
  connect( ui.DrivingMode3, SIGNAL(clicked()), this, SLOT(OnDrivingMode()) );
  connect( ui.DrivingMode4, SIGNAL(clicked()), this, SLOT(OnDrivingMode()) );
  connect( ui.DrivingMode5, SIGNAL(clicked()), this, SLOT(OnDrivingMode()) );
  connect( ui.DrivingMode6, SIGNAL(clicked()), this, SLOT(OnDrivingMode()) );
  connect( ui.DrivingMode_GearTest, SIGNAL(clicked()), this, SLOT(OnDrivingMode()) );
  ui.DrivingMode1->setEnabled(false);
  ui.DrivingMode2->setEnabled(false);
  ui.DrivingMode3->setEnabled(false);
  ui.DrivingMode4->setEnabled(false);
  ui.DrivingMode5->setEnabled(false);
  ui.DrivingMode6->setEnabled(false);
  ui.DrivingMode_GearTest->setEnabled(false);
  ui.DrivingMode_Gear->setEnabled(false);

    ui.checkBox_manual->setEnabled(false);
    ui.checkStopRequest->setEnabled(false);
    ui.checkAutoParkingOn->setEnabled(false);
    ui.checkAPC_Enable->setEnabled(false);
    ui.checkGearShiftEnable->setEnabled(false);

    ui.accBox->setEnabled(false);
    ui.accGainBox->setEnabled(false);
    ui.steerBox->setEnabled(false);

  connect( ui.DrivingMode_Gear, SIGNAL(clicked()), this, SLOT(OncheckBoxGear()) );

  m_pCanMCU = NULL;

  canThread = NULL;
	
  qnode = NULL;
	
  timer = new QTimer(this);
  connect( timer, SIGNAL(timeout()), this, SLOT(OnTimer()) );
  //connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  //connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
}

MainWindow::~MainWindow() {}

void MainWindow::OncheckBoxGear() {
  if (ui.DrivingMode_Gear->isChecked()) 
      qnode->m_GearShiftEnable = true;
  else  qnode->m_GearShiftEnable = false;
}

void MainWindow::OnDrivingMode() {
  if(m_pCanMCU) {
    if( ui.DrivingMode1->isChecked() ) {
    // fully autonomous mode
      qnode->m_velMode = true;
      qnode->m_steerMode = true;
      qnode->m_GearShiftEnable = true;
      qnode->m_autonomousMode = true;
      qnode->m_ApcTestMode = 0;

      ui.accBox->setEnabled(false);
      ui.steerBox->setEnabled(false);
      ui.accGainBox->setEnabled(true);
    }
    else if(ui.DrivingMode2->isChecked() ) {
      // semi auto - only steer
      qnode->m_velMode = false;
      qnode->m_steerMode = true;
      qnode->m_GearShiftEnable = false;
      qnode->m_autonomousMode = true;
      qnode->m_ApcTestMode = 1;

      ui.accBox->setEnabled(false);
      ui.steerBox->setEnabled(false);
    }
    else if(ui.DrivingMode3->isChecked() ) {
      // semi auto - only vel
      qnode->m_velMode = true;
      qnode->m_steerMode = false;
      qnode->m_GearShiftEnable = false;
      qnode->m_autonomousMode = true;
      qnode->m_ApcTestMode = 2;

      ui.accBox->setEnabled(false);
      ui.steerBox->setEnabled(false);
    }
    else if(ui.DrivingMode4->isChecked() ) {
      // manual velocity
      qnode->m_velMode = false;
      qnode->m_steerMode = true;
      qnode->m_GearShiftEnable = false;
      qnode->m_autonomousMode = false;
      ui.accBox->setEnabled(true);
      ui.steerBox->setEnabled(false);
      qnode->m_ApcTestMode = 2;
    }
    else if(ui.DrivingMode5->isChecked() ) {
      // manual steer
      qnode->m_velMode = true;
      qnode->m_steerMode = false;
      qnode->m_GearShiftEnable = false;
      qnode->m_autonomousMode = false;
      ui.steerBox->setValue(qnode->m_currentSteer);
      ui.accBox->setEnabled(false);
      ui.steerBox->setEnabled(true);
      qnode->m_ApcTestMode = 1;
    }
    else if(ui.DrivingMode6->isChecked() ) {
      // driver
      qnode->m_velMode = false;
      qnode->m_steerMode = false;
      qnode->m_GearShiftEnable = false;
      qnode->m_autonomousMode = false;
      ui.accBox->setEnabled(false);
      ui.steerBox->setEnabled(false);
      qnode->m_ApcTestMode = 0;
    }
    else if(ui.DrivingMode_GearTest->isChecked() ) {
      // driver
      qnode->m_velMode = false;
      qnode->m_steerMode = false;
      qnode->m_GearShiftEnable = true;
      qnode->m_autonomousMode = false;
      ui.accBox->setEnabled(false);
      ui.steerBox->setEnabled(false);
    }
    else {
      qnode->m_velMode = false;
      qnode->m_steerMode = false;
      qnode->m_autonomousMode = false;
    }
    // printf("---------------------\n");
  }
}

void MainWindow::OnTimer()
{
  qnode->m_WhlSpd_RR = m_pCanMCU->_WHL_SPD.WhlSpd_RR;
  qnode->m_WhlSpd_RL = m_pCanMCU->_WHL_SPD.WhlSpd_RL;
  
  qnode->m_WhlSpd_FR = m_pCanMCU->_WHL_SPD.WhlSpd_FR;
  qnode->m_WhlSpd_FL = m_pCanMCU->_WHL_SPD.WhlSpd_FL;

  // cout << m_pCanMCU->_WHL_SPD.WhlSpd_RR << " " << m_pCanMCU->_WHL_SPD.WhlSpd_RL << endl;
  if (ui.checkBox_manual->isChecked()) {
    if(ui.checkStopRequest->isChecked())  qnode->m_StopRequest = true;
    else                                  qnode->m_StopRequest = false;
    if(ui.checkAutoParkingOn->isChecked())qnode->m_AutoParkingOn = true;
    else                                  qnode->m_AutoParkingOn = false;
    if(ui.checkAPC_Enable->isChecked())   qnode->m_APC_Enable = true;
    else                                  qnode->m_APC_Enable = false;
    if(ui.checkGearShiftEnable->isChecked())  qnode->m_GearShiftEnable = true;
    else                                      qnode->m_GearShiftEnable = false;
    qnode->m_manual_state_control = true;
  }
  else  qnode->m_manual_state_control = false;

  if (qnode->m_StopRequest)   ui.checkStopRequest->setChecked(1);
  else                        ui.checkStopRequest->setChecked(0);
  if (qnode->m_AutoParkingOn) ui.checkAutoParkingOn->setChecked(1);
  else                        ui.checkAutoParkingOn->setChecked(0);
  if (qnode->m_APC_Enable)    ui.checkAPC_Enable->setChecked(1);
  else                        ui.checkAPC_Enable->setChecked(0);
  if (qnode->m_GearShiftEnable)      ui.checkGearShiftEnable->setChecked(1);
  else                        ui.checkGearShiftEnable->setChecked(0);
  
  qnode->m_APC_CmdState = m_pCanMCU->_APC_CMD_State.APC_CmdState;

  char text[2048];
  
  if (qnode->m_steerMsgUpdated) {
    sprintf(text, "%.1f", qnode->m_steer );
    ui.lineEdit_steer->setText(text);
    sprintf(text, "%.2f", qnode->m_vel );
    ui.lineEdit_vel->setText(text);

    sprintf(text, "%.2f", qnode->m_curr_vel );
    ui.lineEdit_CurrVel->setText(text);
    
    sprintf(text, "%.5f", qnode->m_acc );
    ui.lineEdit_acc->setText(text);
    sprintf(text, "%.2f", qnode->m_err );
    ui.lineEdit_err->setText(text);


    if (!qnode->m_finishFLAG) {
      sprintf(text, "%.2f", qnode->m_car2switchDist);
      ui.lineEdit_switchingPtDist->setText(text);
    }
    else {
      sprintf(text, "%s", "FINISH!!!");
      ui.lineEdit_switchingPtDist->setText(text);
    }

    sprintf(text, "%.d", qnode->m_leftSwitchingCnt );
    ui.lineEdit_switchingCNT->setText(text);
  }
  
  if (qnode->m_gearCtrlMsgUpdated) {
    // cout << qnode->m_gear << endl;
    if (qnode->m_gear == 5)
      sprintf(text, "%c", 'D' );
    else if (qnode->m_gear == 6)
      sprintf(text, "%c", 'N' );
    else if (qnode->m_gear == 7)
      sprintf(text, "%c", 'R' );
    else 
      sprintf(text, "%c", 'P' );
    ui.lineEdit_gear->setText(text);
  }

  // 
  int idx = (int)(m_pCanMCU->_APC_CMD_State.APC_CmdState);
  switch (idx) {
    case 0: sprintf(text, "%s", "APC OFF"); break;
    case 1: sprintf(text, "%s", "APC ON");  break;
    case 2: sprintf(text, "%s", "APC READY"); break;
    case 3: sprintf(text, "%s", "APC CONTROL STOP");  break;
    case 4: sprintf(text, "%s", "APC CONTROL NORMAL");  break;
    case 5: sprintf(text, "%s", "APC CONTROL GEARSHIFT"); break;
    case 7: sprintf(text, "%s", "APC FAIL");  break;
    default: sprintf(text, "-");
  }
  ui.lineEdit_APC_CMD_State->setText(text);

  sprintf(text, "%.2f", m_pCanMCU->_VEH_STATE.LatAccel);
  ui.lineEdit_LatAccel->setText(text);
  sprintf(text, "%.2f", m_pCanMCU->_VEH_STATE.LongAccel);
  ui.lineEdit_LongAccel->setText(text);
  sprintf(text, "%.2f", m_pCanMCU->_VEH_STATE.YawRate);
  ui.lineEdit_YawRate->setText(text);

  idx = (int)(m_pCanMCU->_VEH_STATE.GearState);
  qnode->m_gear_state = idx;
  switch (idx) {
    case 0: sprintf(text, "%c", 'P'); break;
    case 5: sprintf(text, "%c", 'D'); break;
    case 6: sprintf(text, "%c", 'N'); break;
    case 7: sprintf(text, "%c", 'R'); break;
    default: sprintf(text, "-");
  }
  ui.lineEdit_GearState->setText(text);

  sprintf(text, "%.1f", m_pCanMCU->_VEH_STATE.SteerAngle);
  qnode->m_currentSteer = m_pCanMCU->_VEH_STATE.SteerAngle;
  ui.lineEdit_SteerAngle->setText(text);
}
///////////////////////////////////////////////////////////////////

void MainWindow::on_pushButton_Z_clicked()
{
  qnode->m_velMode = false;
  qnode->m_steerMode = true;
  qnode->m_GearShiftEnable = false;
  ui.steerBox->setValue(0);
  if(ui.DrivingMode5->isChecked()) {
    qnode->m_steer = 0.0;
  }
}

void MainWindow::on_pushButton_LF_clicked()
{
  qnode->m_velMode = false;
  qnode->m_steerMode = true;
  qnode->m_GearShiftEnable = false;
  ui.steerBox->setValue(-539.0);
  if(ui.DrivingMode5->isChecked()) {
    qnode->m_steer = -539.0;
  }
}

void MainWindow::on_pushButton_RF_clicked()
{
  qnode->m_velMode = false;
  qnode->m_steerMode = true;
  qnode->m_GearShiftEnable = false;
  ui.steerBox->setValue(539);
  if(ui.DrivingMode5->isChecked()) {
    qnode->m_steer = 539.0;
  }
}

void MainWindow::on_pushButton_gearP_clicked()
{
  qnode->m_velMode = false;
  qnode->m_steerMode = false;
  qnode->m_GearShiftEnable = true;
  qnode->m_gear = 0;
}

void MainWindow::on_pushButton_gearN_clicked()
{
  qnode->m_velMode = false;
  qnode->m_steerMode = false;
  qnode->m_GearShiftEnable = true;
  qnode->m_gear = 6;
}

void MainWindow::on_pushButton_gearR_clicked()
{
  qnode->m_velMode = false;
  qnode->m_steerMode = false;
  qnode->m_GearShiftEnable = true;
  qnode->m_gear = 7;
}

void MainWindow::on_pushButton_gearD_clicked()
{
  qnode->m_velMode = false;
  qnode->m_steerMode = false;
  qnode->m_GearShiftEnable = true;
  qnode->m_gear = 5;
}

void MainWindow::on_accBox_valueChanged()
{
  qnode->m_velMode = true;
  qnode->m_steerMode = false;
  qnode->m_GearShiftEnable = false;
  if(ui.DrivingMode4->isChecked()) {
    qnode->m_acc = ui.accBox->value();
  }
}

void MainWindow::on_steerBox_valueChanged()
{
  qnode->m_velMode = false;
  qnode->m_steerMode = true;
  qnode->m_GearShiftEnable = false;
  if(ui.DrivingMode5->isChecked()) {
    qnode->m_steer =ui.steerBox->value();
  }
}

void MainWindow::on_accGainBox_valueChanged(){
  qnode->m_accGain = ui.accGainBox->value();
}

void MainWindow::on_startButton_clicked()
{
  if(!m_pCanMCU) {
    ui.startButton->setText("Stop");

    m_pCanMCU = new CAN_MCU(ui.canPort1->value());
    printf("can port[%d]\n", ui.canPort1->value());

    canThread = new CanReceiveThread(m_pCanMCU);
    canThread->start();

    // initialize Q-node
    qnode = new QNode(_argc, _argv, &ui, m_pCanMCU);
    qnode->start();

    //timer->start(20); // ontimer. ms.
    timer->start(10); // ontimer. ms.
		
    /////////////////////////////////////////////////
    ui.DrivingMode1->setEnabled(true);
    ui.DrivingMode2->setEnabled(true);
    ui.DrivingMode3->setEnabled(true);
    ui.DrivingMode4->setEnabled(true);
    ui.DrivingMode5->setEnabled(true);
    ui.DrivingMode6->setEnabled(true);
    ui.DrivingMode_GearTest->setEnabled(true);
    ui.DrivingMode_Gear->setEnabled(true);

    ui.accGainBox->setEnabled(true);

    ui.checkBox_manual->setEnabled(true);
    ui.checkStopRequest->setEnabled(true);
    ui.checkAutoParkingOn->setEnabled(true);
    ui.checkAPC_Enable->setEnabled(true);
    ui.checkGearShiftEnable->setEnabled(true);

    qnode->m_finishFLAG = false;
    qnode->m_acc = 0.0;    //double
    qnode->m_vel = 0.0;    //double
    qnode->m_brk = 0.0;    //double
    qnode->m_steer = 0.0;  //double
    qnode->m_gear = 0;  //int   
    qnode->m_err = 0.0;    //double
    qnode->m_car2switchDist = 0.0; //double 
    qnode->m_leftSwitchingCnt = 0; //int 
    qnode->m_gear_state = 0;
    qnode->m_curr_vel = 0.0;
    qnode->m_accGain = 0.01;

    qnode->m_AutoParkingOn = false;
    qnode->m_APC_Enable = false;
    qnode->m_StopRequest = false;
    qnode->m_GearShiftEnable = false;
    qnode->m_ApcTestMode = 0;  // 0: full, 1: steer, 2: accel only

    qnode->m_manual_state_control = false;

    OnDrivingMode();
  }
  else {  // terminate
    ui.DrivingMode6->setChecked(true);
    OnDrivingMode();

    ui.startButton->setText("Start");

    timer->stop();

    canThread->stop();
    canThread->wait();
    qnode->stop();
    qnode->wait();
    delete canThread;
    canThread = NULL;
		
    delete qnode;
    qnode = NULL;

    delete m_pCanMCU;
    m_pCanMCU = NULL;

    printf("thread clear\n");
    ////////////////////////////////////////////////
    ui.DrivingMode1->setEnabled(false);
    ui.DrivingMode2->setEnabled(false);
    ui.DrivingMode3->setEnabled(false);
    ui.DrivingMode4->setEnabled(false);
    ui.DrivingMode5->setEnabled(false);
    ui.DrivingMode6->setEnabled(false);
    ui.DrivingMode_GearTest->setEnabled(false);
    ui.DrivingMode_Gear->setEnabled(false);    

    ui.checkBox_manual->setEnabled(false);
    ui.checkStopRequest->setEnabled(false);
    ui.checkAutoParkingOn->setEnabled(false);
    ui.checkAPC_Enable->setEnabled(false);
    ui.checkGearShiftEnable->setEnabled(false);

    ui.accBox->setEnabled(false);
    ui.steerBox->setEnabled(false);
    ui.accGainBox->setEnabled(false);

    // m_finishFLAG = false;
	  // qnode->m_acc = 0.0;    //double
    // qnode->m_vel = 0.0;    //double
    // qnode->m_brk = 0.0;    //double
    // qnode->m_steer = 0.0;  //double
    // qnode->m_gear = 0;  //int   
    // qnode->m_err = 0.0;    //double
    // qnode->m_gear_state = 0;
    // qnode->m_curr_vel = 0.0;
    // qnode->m_accGain = 0.01;

    // qnode->m_AutoParkingOn = false;
    // qnode->m_APC_Enable = false;
    // qnode->m_StopRequest = false;
    // qnode->m_GearShiftEnable = false;
    // qnode->m_ApcTestMode = 0;  // 0: full, 1: steer, 2: accel only

    // qnode->m_manual_state_control = false;

  }
}

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  MainWindow w(argc, argv);
  w.show();

  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

  return app.exec();
}
