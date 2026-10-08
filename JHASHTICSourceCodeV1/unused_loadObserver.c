/*disturbance calculations- calculations always done in a frame of reference where +ve torque produces + acceleration, and hence +ve velocity..
     Whwnever we multiply the read velocity values and multiply it with the localPolarity var, we are making this happen, whatever the mounting
     or winding.This frame of ref, is called motorSpace. also known as logical space. The torque calculated by STC calc torque is already in this frame of reference. */
    float wRaw = getCurrentVelocity_RadSec(&ENCODER_M1);
    lo.currentVelocityRadsec = wRaw * lo.localPolarity; 
    
    //70% old value, 30 new value
    lo.filteredVelocityRadSec = (lo.currentVelocityRadsec * LPF_ALPHA_VELOCITY) + (lo.prevVelocityRadsec * (1.0f - LPF_ALPHA_VELOCITY));
    
    calculateDisturbanceFF(&lo); // this fn calculates the compensation torque in the local frame,
    
   if(lo.active){
      if (lo.compensationTorque_Nm > MAX_COMPENSATION_TORQUE_NM){lo.compensationTorque_Nm = MAX_COMPENSATION_TORQUE_NM;}
      if (lo.compensationTorque_Nm < -MAX_COMPENSATION_TORQUE_NM){lo.compensationTorque_Nm = -MAX_COMPENSATION_TORQUE_NM;}
      lo.compensationTorque_Nm_s16 = (int16_t)(lo.compensationTorque_Nm * NM_TO_S16_CONVERSION_FACTOR); 
      //totalIq_MotorSpace += (int32_t)lo.compensationTorque_Nm_s16;
      
      lo.IQw_aFF_wLO = totalIq_MotorSpace + lo.compensationTorque_Nm_s16;
    }
    
    
        //update load observer torque command.
   lo.prevTorque_Nm = FOCVars[bMotor].hTeref * S16_TO_NM_CONVERSION_FACTOR ;
