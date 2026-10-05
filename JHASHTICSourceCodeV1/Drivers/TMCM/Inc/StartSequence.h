#ifndef START_SEQ_H_
#define START_SEQ_H_

#include "StateMachine.h"

//ON GROUND(800,2000,3000,300,5500,5,50,600)
//LIFTED (500,800,1000,300,1000,5,50,200)

#define T_RAMP_DONE 800 //time to ramp up the Torque
#define T_STALL_CHECK 2000 // time before which we should see some movement
#define T_HANDOVER_MAX 3000 // time in which if we dont see required speed we turn off
#define T_RAMPUP_OFFSET 300 // how much delta ms from the calcualted rampup time in which the target rampup must be complete. Its 

#define START_TORQUE_IQ 5500//command to start the torque
#define RPM_STALL_MIN 5 // RPM below which we consider it a stall
#define RPM_HANDOVER 50 
#define RPM_STEADYSTATE 600

enum StartState { IDLE_WAITING, START_SEQUENCE, SPEED_LOOP_SYNC,RAMPING_TO_TARGET,WAIT_PCM,TRANSITION_DONE };
typedef enum { NO_ERROR, START_STALL,SPEED_HANDOVER_TIMEOUT, PCM_HANDOVER_TIMEOUT }ErrorState;

typedef struct{
  uint8_t currentState;
  ErrorState errorState;
  
  uint16_t globalStartSeqTimer;
  
  uint8_t PCM_startCommand;
  uint8_t speedRampStarted;
 
}startSeq;

extern startSeq ssq;

void initializeStartSeqParams( startSeq *ssq_);
ErrorState ExecStartSeq(startSeq *ssq_,systemState *ss);
ErrorState ExecStartSeqWithAccelFF(startSeq *ssq_,systemState *ss, accelControl *a);

void resetStartSeqParams( startSeq *ssq);
void resetStartSeqErrorState( startSeq *ssq);


#endif
