#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include "stdint.h"
#include "main.h" //only for custom faults - remove
#include "state_machine.h"
#include "encoder_speed_pos_fdbk.h"

#define ONE_SHOT_REGEN_FIXED_IQ 10000  //lifted make 5000, on ground < 17000
#define REGEN_TURNOFF_RPM 150
#define RPM_CLOSED_LOOP_THRESHOLD_CC 200
#define RPM_CLOSED_LOOP_THRESHOLD2 300


//Vars for load observer and Accel FF
#define RAMPUP_RPM_PER_SEC 300.0f // 250 is 1m/s2
#define ESTIMATED_J_KGM2 0.222f //0.222f //0.222f on ground, 0.0005 when lifted   // to check
#define KT  0.266f // 20Nm @ 75 rms = 0.266 Nm/Arms
#define ARMS_TO_S16_CONVERSION_FACTOR 252.05f // 130RMS/183.33PK = 32767. so EACH RMS = 32767/130 = 252 s16 units . Max rms we re allowed is 75 = 75*252 ~19K s16 units 
#define MAX_ACCEL_FF_S16 8300 // calculated for 250 RAMPUP_RPM_PER_SEC, and J = 0.222 ( 5505) , 375 RAMPUP_RPM_PER_SEC is 8257 for same J

#define S16_TO_NM_CONVERSION_FACTOR  0.00105534f //KT/ARMS_TO_S16_CONVERSION_FACTOR
#define NM_TO_S16_CONVERSION_FACTOR 947.56f
#define MAX_COMPENSATION_TORQUE_NM 3.0f

#define OBSERVER_BANDWIDTH_P 100.0f  // 'p' in rad/s. Start at 100, max 200.
#define OBSERVER_COMP_GAIN_K 0.5f    // 'k'. Start at 0.5, raise towards 1.0 if stable.

#define PI_F 3.14159265359f

typedef enum {
  CC_IDLE,
  CC_RUNNING_CL,
  CC_RUNNING_OL,
  CC_RUNNING_OL_ERROR,
  CC_ERROR,
  CC_FINISH,
}cc_state_t;


typedef enum {
  CC_RAMPOFF,
  CC_RAMPUP,
  CC_RAMPDOWN,
}cc_rampState;

typedef enum {
  CC, //continuous control
  SS, //single shot
  LCC, //laptop continous 
  LSS, //laptop single shot
  NO_RUN, 
}runType;

typedef struct{

  runType runType;
  int8_t podDirection;
  int16_t targetRPM;
  uint16_t brakeDistance;
  int8_t motorDirection; 
  uint8_t loggingOn;
  
  uint16_t indexID;
  cc_state_t cc_state ; //continuous control state
  cc_rampState cc_ramp ;
  int16_t currentRPM;
  uint16_t currentAbsRpm;
  uint16_t iqRef;
  int8_t brakeState;
  uint8_t engageBrake;
  uint16_t CL_DeltaRPMThreshold;

  float travelledDist;
  
  uint32_t faultTick;
  uint8_t encoderFault;
  float distAtFault;

  int16_t phaseVoltageRMS;
  int16_t phaseCurrentRMS;
  
  int16_t DC_VOLTAGE;
  State_t MCSDK_STATE;

  uint8_t MCSDK_PreFault;
  uint8_t MCSDK_CurrentFault;
  CustomErrors CustomFaults;
  int16_t customFaultErrorVal;
  uint8_t neverStarting;
  
  uint8_t startingDeadTimeCrossed;
}systemState;


typedef struct {
  uint8_t timerOnBool;
  uint8_t PCM_timer_thresh;
  volatile uint8_t PCM_timer;
}continousControlTimer;

typedef struct{
  uint8_t brakeOneTime;
  uint8_t brakeCounter;
  uint32_t brakeTime1;
}brakeCtrl;

typedef struct {
  uint8_t logStopMsgSent;
  uint32_t startTickForDelayedStop;
  
}logController;

typedef struct {
  float finalAbsTarget;
  float finalTargetRadsec;
  float rampupTime_secF; //calculated time to rampup based on our RAMPUP_RPM_PER_SEC 
  float accel_Radsec2F; // acceleration in rad/sec2
  float accelTorqueEstimated;
  float accelARMS;
  
  int16_t accelFF_PK_s16;
  uint16_t rampupTime_ms; 
  
  int16_t rawIQ;
  int16_t IQwFF;
  
  uint8_t ffActive;
  
}accelControl;

typedef struct {
  uint8_t active;
  float localPolarity;
  float prevTorque_Nm;
  float currentVelocityRadsec;
  float compensationTorque_Nm;
  int16_t compensationTorque_Nm_s16;
  
  // --- NEW: Luenberger Observer States ---
  float theta_hat;  // Estimated position (rad)
  float w_hat;      // Estimated velocity (rad/s) - Ultra clean!
  float TL_hat;     // Estimated load torque (Nm)
  float e_rad;  // error btw actual and estimated position.
  // --- NEW: Luenberger Gains ---
  float l1;         // Position correction gain
  float l2;         // Velocity correction gain
  float l3;         // Torque correction gain
  
  int16_t rawIQ;
  int16_t IQwaFF;
  int16_t IQw_aFF_wLO;
  
}loadObserver;



uint8_t CheckPodState(systemState *ss);
void updateTMCMState(systemState *ss);
uint8_t TMCM_SpeedLoop_TurnOff(void);
uint16_t Calculate_CLDeltaRPMThreshold(uint16_t targetRpm);
void applyRegen(int currentDirection);

uint8_t WaitmSecForStopping(logController *lc,uint16_t msec);

//accel variables
void calculateAccelerationVars(systemState *ss, accelControl *a);
void zeroAccelerationFFVars(accelControl *a);

void ZeroLOVars(loadObserver *lo);
float getCurrentVelocity_RadSec(ENCODER_Handle_t *pHandle);
float getCurrentPosition_Rad(ENCODER_Handle_t *pHandle);
void leunbergObserverExec(loadObserver *lo);


#endif
