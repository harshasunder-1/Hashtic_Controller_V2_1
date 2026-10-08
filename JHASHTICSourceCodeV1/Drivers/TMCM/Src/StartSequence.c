#include "stdint.h"
#include "StartSequence.h"
#include "motorcontrol.h"
#include "Configuration.h"

extern FOCVars_t FOCVars[1];

void initializeStartSeqParams( startSeq *ssq_){
  ssq_->currentState = IDLE_WAITING;  
  ssq_->errorState = NO_ERROR;
  
  ssq_->globalStartSeqTimer = 0;
  ssq_->speedRampStarted = 0;
  ssq_->PCM_startCommand = 0;
  ssq_->logicalTorque = 0;
}

void resetStartSeqParams( startSeq *ssq_){
  ssq_->currentState = IDLE_WAITING;  
  ssq_->globalStartSeqTimer = 0;
  ssq_->PCM_startCommand = 0;
  ssq_->speedRampStarted = 0;
  ssq_->logicalTorque = 0;
}

void resetStartSeqErrorState( startSeq *ssq_){
  ssq_->errorState = NO_ERROR;
}


int16_t kickTorque=0,scaledKickTorque=0;
ErrorState ExecStartSeq(startSeq *ssq_,systemState *ss){
  
  switch (ssq_->currentState) {
    
    case IDLE_WAITING:
        if (ssq_->PCM_startCommand == 1) { 
            //MC_ProgramTorqueRampMotor1(0, 0); // asynchronous
            //Whenever we apply either a torque ramp or speed ramp, inside the calcCurrRef function we will 
            // see signForCW and apply the correct Iq. But here we dont need to do anything.
            int16_t startingTorque = START_TORQUE_IQ * ss->motorDirection;
            MC_ProgramTorqueRampMotor1(startingTorque,T_RAMP_DONE);
            MC_StartMotor1();
            ssq_->globalStartSeqTimer = 0;
            ssq_->currentState = START_SEQUENCE;
        }
        break;

    case START_SEQUENCE:
      
        // 1. SUCCESS CONDITION (With Noise Debouncing)
        if (ss->currentAbsRpm >= RPM_HANDOVER) {

          // Bumpless transfer with clamping
            PID_SetUpperOutputLimit(&PIDSpeedHandle_M1, IQMAX);
            PID_SetLowerOutputLimit(&PIDSpeedHandle_M1, -IQMAX);
            
            uint16_t kiDivisor= PID_GetKIDivisor(&PIDSpeedHandle_M1);
            
            //Here current torque is electrical torque with a sign, but the the integral term in the PI loop needs to be 'logical' torque.
            //ie: it has to be +ve current for +ve speed. multiplying like below changes electrical torque back to logical torque
            //if needed because the winding is opposite.
            kickTorque = FOCVars[0].Iqdref.q; // this has a -ve sign for FWD motion if winding is -1
            scaledKickTorque = kickTorque * 8 / 10; // 80% scaling 
            int32_t logicalTorque = (int32_t)scaledKickTorque * c.signForCWRotation; // here the torque becomes positive if moving FWD, whatever the winding direction.
          
            int32_t integralTerm = 0 ;            
            integralTerm = kiDivisor * logicalTorque; 
        
            PID_SetIntegralTerm(&PIDSpeedHandle_M1,integralTerm);
            
           // MC_StopMotor1(); // uncomment and test only the starting torque
           
            /*---NEW----*/
            //set the internal ramp of the ST speed loop code to the current RPM.
            int16_t currentSpeed_STunits = ss->currentAbsRpm /6 * ss->motorDirection;
            MC_ProgramSpeedRampMotor1(currentSpeed_STunits, 1 );
            
            ssq_->currentState = SPEED_LOOP_SYNC;
        }     
            /*--OLD--
            MC_ProgramSpeedRampMotor1(ss->targetRPM /6 * ss->motorDirection, 8000 );
            //sendPCM_MSG(currentRPM, currentPosition);
            ssq_->currentState = TRANSITION_DONE;
            break; */
           

        // 2. EARLY STALL CHECK
        if (ssq_->globalStartSeqTimer >= T_STALL_CHECK) {
            if (ss->currentAbsRpm < RPM_STALL_MIN) {
                ssq_->errorState = START_STALL;
                MC_StopMotor1();
                ssq_->currentState = IDLE_WAITING; 
                break;
            }
        }

        // 3. FINAL TIMEOUT
        if (ssq_->globalStartSeqTimer >= T_HANDOVER_MAX) {
            ssq_->errorState = SPEED_HANDOVER_TIMEOUT;
            MC_StopMotor1();
            ssq_->currentState = IDLE_WAITING;
            break;
        }
        break;
        
    case SPEED_LOOP_SYNC:
        // Wait for the medium frequency interrupt to execute the 0-ms ramp
        if (MC_GetCommandStateMotor1() == MCI_COMMAND_EXECUTED_SUCCESFULLY) {
            
            // The STC internal reference is now synced to the actual physical speed. 
            // Now we can safely command the final 6-second ramp to the target speed.
            int16_t finalTargetSpeed = (ss->targetRPM / 6) * ss->motorDirection;
            MC_ProgramSpeedRampMotor1(finalTargetSpeed, 6000);
            
            ssq_->currentState = TRANSITION_DONE;
            break;
        }

        // Safety: Ensure we don't get stuck here forever if the interrupt hangs
        if (ssq_->globalStartSeqTimer >= T_HANDOVER_MAX) {
            ssq_->errorState = SPEED_HANDOVER_TIMEOUT;
            MC_StopMotor1();
            ssq_->currentState = IDLE_WAITING;
            break;
        }
        break;

    /*case WAIT_PCM:
        // GLOBAL ABORT check here as well
        if (!PCM_Start_Enable) {
            stopMotor();
            currentState = IDLE;
            break;
        }

        // Timeout check for PCM spatial limit
        if (currentPosition >= 5 && PCM_Handover == 0) {
            error = PCM_HANDOVER_TIMEOUT;
            stopMotor();
            currentState = IDLE;
            break;
        }
        
        // Successful Handover
        if (PCM_Handover == 1) {
            currentState = IDLE;
        }
        break;
*/
  } // closes switch statements
  
  return ssq_->errorState;
}



ErrorState ExecStartSeqWithAccelFF(startSeq *ssq_,systemState *ss, accelControl *a,loadObserver *lo){
   
  switch (ssq_->currentState) {
    
    case IDLE_WAITING:
        if (ssq_->PCM_startCommand == 1) { 
            //MC_ProgramTorqueRampMotor1(0, 0); // asynchronous
            //Whenever we apply either a torque ramp or speed ramp, inside the calcCurrRef function we will 
            // see signForCW and apply the correct Iq. But here we dont need to do anything.
            int16_t startingTorque = START_TORQUE_IQ * ss->motorDirection;
            MC_ProgramTorqueRampMotor1(startingTorque,T_RAMP_DONE);
            MC_StartMotor1();
            ssq_->globalStartSeqTimer = 0;
            ssq_->currentState = START_SEQUENCE;
        }
        break;

    case START_SEQUENCE:
      
        // 1. SUCCESS CONDITION (With Noise Debouncing)
        if (ss->currentAbsRpm >= RPM_HANDOVER) {

          // Bumpless transfer with clamping
            PID_SetUpperOutputLimit(&PIDSpeedHandle_M1, IQMAX);
            PID_SetLowerOutputLimit(&PIDSpeedHandle_M1, -IQMAX);
            
            uint16_t kiDivisor= PID_GetKIDivisor(&PIDSpeedHandle_M1);
            
            //Here current torque is electrical torque with a sign, but the the integral term in the PI loop needs to be 'logical' torque.
            //ie: it has to be +ve current for +ve speed. multiplying like below changes electrical torque back to logical torque
            //if needed because the winding is opposite.
            kickTorque = FOCVars[0].Iqdref.q; // this has a -ve sign for FWD motion if winding is -1
            scaledKickTorque = kickTorque * 8 / 10; // 80% scaling 
            int32_t logicalTorque = (int32_t)scaledKickTorque * c.signForCWRotation; // here the torque becomes positive if moving FWD, whatever the winding direction.
            ssq_->logicalTorque = logicalTorque ; //save for usage in the next sequence.
            
            //accelerationFF term comes with the direction correctly set.            
            int32_t integralTerm = 0 ;            
            // acceleration ff subtraction for bumpless transfer
            integralTerm = kiDivisor * logicalTorque;
        
            // Directional clamping (prevent negative windup in forward direction)
            if (logicalTorque > 0 && integralTerm < 0) {
                integralTerm = 0;
            } else if (logicalTorque < 0 && integralTerm > 0) {
                integralTerm = 0;
            }
           
            PID_SetIntegralTerm(&PIDSpeedHandle_M1,integralTerm);
            
           // MC_StopMotor1(); // uncomment and test only the starting torque
           
            /*---NEW----*/
            //set the internal ramp of the ST speed loop code to the current RPM.
            int16_t currentSpeed_STunits = (int16_t)((ss->currentAbsRpm * ss->motorDirection)/6);
            MC_ProgramSpeedRampMotor1(currentSpeed_STunits, 1 );
            
            ssq_->currentState = SPEED_LOOP_SYNC;
        }               

        // 2. EARLY STALL CHECK
        if (ssq_->globalStartSeqTimer >= T_STALL_CHECK) {
            if (ss->currentAbsRpm < RPM_STALL_MIN) {
                ssq_->errorState = START_STALL;
                MC_StopMotor1();
                ssq_->currentState = IDLE_WAITING; 
                break;
            }
        }

        // 3. FINAL TIMEOUT
        if (ssq_->globalStartSeqTimer >= T_HANDOVER_MAX) {
            ssq_->errorState = SPEED_HANDOVER_TIMEOUT;
            MC_StopMotor1();
            ssq_->currentState = IDLE_WAITING;
            break;
        }
        break;
        
    case SPEED_LOOP_SYNC:
        // Wait for the medium frequency interrupt to execute the 0-ms ramp
        if (MC_GetCommandStateMotor1() == MCI_COMMAND_EXECUTED_SUCCESFULLY) {
                 
            // The STC internal reference is now synced to the actual physical speed. 
            // Now we can safely command the final rampup to the target speed.
            int16_t finalTargetSpeed = (int16_t)((ss->targetRPM * ss->motorDirection)/6);
            MC_ProgramSpeedRampMotor1(finalTargetSpeed, a->rampupTime_ms);
            
            // Subtract accelFF from the integrator right as we turn it on
            int32_t current_integral = PIDSpeedHandle_M1.wIntegralTerm;
            int32_t offset = PID_GetKIDivisor(&PIDSpeedHandle_M1) * a->accelFF_PK_s16;
            PID_SetIntegralTerm(&PIDSpeedHandle_M1, current_integral - offset);
                 
            a->ffActive = 1;
            
            //seed DOB values with starting values
            // 1. Set current and previous velocity to actual measured handoff speed
            float currentVelLocal_radsec = (((float)ss->currentAbsRpm * (float)ss->motorDirection) / 60.0f) * (2.0f * (float)M_PI);
            
            // 2. Set previous applied torque to actual breakaway torque applied
            // Seed the shadow motor with actual physical states at the moment of handoff
            lo->theta_hat = getCurrentPosition_Rad(&ENCODER_M1) * (float)lo->localPolarity;
            lo->w_hat     = currentVelLocal_radsec; // From your existing handoff logic
            lo->TL_hat    = 0.0f;
            lo->prevTorque_Nm = ssq_->logicalTorque * S16_TO_NM_CONVERSION_FACTOR;

            // 3. Assume zero initial disturbance error at handoff instant (or pre-charge with expected Coulomb friction)
            lo->compensationTorque_Nm = 0.0f;

            // 4. Enable the observer           
            lo->active = 0;
            
            //reset the global timer so that we wait till the full ramp is over.
            ssq_->globalStartSeqTimer = 0;
            
            ssq_->speedRampStarted = 1;
            ssq_->currentState = RAMPING_TO_TARGET;
            break;
        }

        // Safety: Ensure we don't get stuck here forever if the interrupt hangs
        if (ssq_->globalStartSeqTimer >= T_HANDOVER_MAX) {
            ssq_->errorState = SPEED_HANDOVER_TIMEOUT;
            MC_StopMotor1();
            ssq_->currentState = IDLE_WAITING;
            break;
        }
        break;
        
  case RAMPING_TO_TARGET:
        // Check if ST MCSDK speed ramp is complete
        if (MC_HasRampCompletedMotor1()) { 
            //DISABLE FEEDFORWARD once steady speed is reached
            a->ffActive = 0; 
            a->accelFF_PK_s16 = 0; // Clear value for safety
            ssq_->currentState = TRANSITION_DONE;
            break;
        }
        
        // Software timer fallback check (rampupTime_ms)
        if (ssq_->globalStartSeqTimer >= (a->rampupTime_ms + T_RAMPUP_OFFSET)) {
            a->ffActive = 0;
            a->accelFF_PK_s16 = 0;
            ssq_->errorState = SPEED_RAMP_FAIL;
            MC_StopMotor1();
            ssq_->currentState = IDLE_WAITING;
            break;
        }
        break;
    

  } // closes switch statements
  
  return ssq_->errorState;
}
