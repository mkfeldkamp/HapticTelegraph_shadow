 // const int PWMstartAneg = -30, PWMstartApos = 40;
  // // for negative commands, clip at max and min
  // if (u < 0) {
  //   if (u < -MaxPWM) {
  //     Serial.println("% Warning: Clipping motor output to " + String(-MaxPWM));
  //     u = MaxPWM;
  //   } else if (PWMstartAneg < u) u = -PWMstartAneg;
  //   else {
  //     analogWrite(MotorA2, abs(u));
  //     analogWrite(MotorA1, 0);
  //   }
  // } else {  // 0 < u  (positive), do the same as above but swapped
  //   if (MaxPWM < u) {
  //     Serial.println("% Warning: Clipping motor output to " + String(MaxPWM));
  //     u = MaxPWM;
  //   } else if (u < PWMstartApos) u = PWMstartApos;
  //   else {
  //     analogWrite(MotorA1, abs(u));
  //     analogWrite(MotorA2, 0);
  //   }
  // }