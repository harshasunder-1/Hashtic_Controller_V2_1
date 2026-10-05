//State Machine

#include "StateMachine.h"
#include "state_machine.h" //this is mcsdk state machine
#include "motorcontrol.h"


extern MCI_Handle_t * pMCI[NBR_OF_MOTORS];

uint8_t CheckPodState(systemState *ss){
  
    uint8_t IsPodStopped = 0;
    uint8_t isMotorStateRun = 0;
    uint8_t isMotorEncoderNOK = 0;
    uint8_t isMotorBrakeDisengaged = 0;
    
    //if we get a command when the vehicle is moving, dont allow it. Infact stop the pod.
    uint16_t absRPM = 0;
    absRPM = ss->currentRPM;
    if (ss->currentRPM < 0){absRPM = -(ss->currentRPM);}
                
    // whether pod is moving or not, it is already in a run state. so we dont use another start command, go to stop
    if ((ss->MCSDK_STATE == START) || (ss->MCSDK_STATE == START_RUN)){  
      isMotorStateRun = 1;
    }
    if (ss->cc_state != CC_IDLE){  // only allow motor to run if state is IDLE, which can only happen if precharge is pressed before this.
      isMotorStateRun = 1;
    }
    
    if(absRPM < 5){IsPodStopped = 1;} // only allow start if pod is stopped.
    if(ss->CustomFaults != NO_FAULTS){isMotorEncoderNOK = 1;}
    if(ss->brakeState == 0){isMotorBrakeDisengaged = 1;}
    
    if (IsPodStopped==1 && isMotorStateRun==0 && isMotorEncoderNOK==0 && isMotorBrakeDisengaged == 1){
      return 1;
    }else{
      return 0;
    }
}

void updateTMCMState(systemState *ss){  
  ss->DC_VOLTAGE = VBS_GetAvBusVoltage_V(&RealBusVoltageSensorParamsM1._Super);
  ss->currentRPM= 6*MC_GetMecSpeedAverageMotor1();
  
  uint16_t absRPM = 0;
  absRPM = ss->currentRPM;
  if (ss->currentRPM < 0){absRPM = -(ss->currentRPM);}
  
  ss->currentAbsRpm = absRPM;
  ss->MCSDK_PreFault =  MC_GetOccurredFaultsMotor1();
  ss->MCSDK_CurrentFault = MC_GetCurrentFaultsMotor1();
  ss->MCSDK_STATE = MC_GetSTMStateMotor1();
  ss->phaseVoltageRMS = (int16_t)((MC_GetPhaseVoltageAmplitudeMotor1())/1.414f);
  ss->phaseCurrentRMS = (int16_t)((MC_GetPhaseCurrentAmplitudeMotor1())/1.414f);
}


uint8_t TMCM_SpeedLoop_TurnOff(void){   
    MC_StopMotor1();      
    MC_ProgramSpeedRampMotor1(0, 0); // asynchronous
    MC_Clear_IqdrefMotor1();
    pMCI[0]->pSTC->TargetFinal = 0;
    pMCI[0]->pSTC->SpeedRefUnitExt = 0;
    pMCI[0]->pSTC->TorqueRef = 0;
    return 1;
}

uint16_t Calculate_CLDeltaRPMThreshold(uint16_t targetRpm){  //closed loop threshold scales with RPM.
  uint16_t out = 0;
  if (targetRpm <=300){
      out = targetRpm/2;
    }else{
      out  = RPM_CLOSED_LOOP_THRESHOLD_CC;
    }
  return out;
}
          

//MotorControl Functions
void applyRegen(int currentDirection){
  if(currentDirection == 1){  //change this later..make it cleaner, direction from outside?
      MC_ProgramTorqueRampMotor1(-ONE_SHOT_REGEN_FIXED_IQ,300 ); 
    }else{
      MC_ProgramTorqueRampMotor1(ONE_SHOT_REGEN_FIXED_IQ,300 );
    }
}
     
uint8_t WaitmSecForStopping(logController *lc,uint16_t msec){
  // setting the lc->startTickForDelayedStop to zero resets the counter.
  
  if (lc->startTickForDelayedStop == 0){
    lc->startTickForDelayedStop = HAL_GetTick();
    if (lc->startTickForDelayedStop == 0){
      lc->startTickForDelayedStop = 1;
    }
  }
  else{
    if (HAL_GetTick() - lc->startTickForDelayedStop >= msec){
      lc->startTickForDelayedStop =0; // reset the counter after we ve successfully passed the time.
      return 1;
    }
  }
  return 0;
}

void zeroAccelerationFFVars(accelControl *a){
  a->finalAbsTarget = 0;
  a->finalTargetRadsec = 0;
  a->rampupTime_secF = 0;
  a->rampupTime_ms = 0;
  a->accel_Radsec2F = 0;
  a->accelTorqueEstimated = 0;
  a->accelARMS = 0;
  a->accelFF_PK_s16 = 0;
}

void calculateAccelerationVars(systemState *ss, accelControl *a){
  a->finalAbsTarget = ss->targetRPM; //target will alays be positive. direction come seperately. Only for acceleration. deceleration doesnt us PI.
  
  a->rampupTime_secF = (float)a->finalAbsTarget/RAMPUP_RPM_PER_SEC; //time taken in sec to reach the target at our chosen ramp up rate
  if (a->rampupTime_secF < 1){a->rampupTime_secF = 1;}
  
  a->rampupTime_ms = (uint16_t)(a->rampupTime_secF * 1000); //same time in ms we can use to put in to the speed loop command
  
  a->finalTargetRadsec = a->finalAbsTarget * 0.104719755f; //2*PI / 60
  a->accel_Radsec2F = a->finalTargetRadsec/a->rampupTime_secF; //acceleration in rad/sec2
    
  //torque = J * acceleration 
  a->accelTorqueEstimated = a->accel_Radsec2F * ESTIMATED_J_KGM2;
  a->accelARMS = a->accelTorqueEstimated/KT;
 
  // this is the accel FF, it has a sign baed on the direction.
  // Convert to s16 using 32-bit int to prevent overflow before clamping
  int32_t ff_calculated = (int32_t)(a->accelARMS * ARMS_TO_S16_CONVERSION_FACTOR * (float)ss->motorDirection);
  
  if (ff_calculated > MAX_ACCEL_FF_S16){
    ff_calculated = MAX_ACCEL_FF_S16;
  }else if (ff_calculated < -MAX_ACCEL_FF_S16){
    ff_calculated = -MAX_ACCEL_FF_S16;
  }
  a->accelFF_PK_s16 = (int16_t)ff_calculated ;
}