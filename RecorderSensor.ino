float alphaYawForRec = 0;
float alphaPitchForRec = 0;

//for yaw
float get_x_in_xy_rotation()
{
  float xx = DCM_Matrix[0][0];    //DCM_Matrix_Inverse[0][0];
  float xy = DCM_Matrix[0][1];    //DCM_Matrix_Inverse[1][0];
  float xz = DCM_Matrix[0][2];    //DCM_Matrix_Inverse[2][0];

  float alpha_temp = TO_DEG(atan(xx / xy));
  float alpha;

  if (xx > 0 && xy < 0)
    alpha = alpha_temp + 180;
  else if (xx < 0 && xy < 0)
    alpha = alpha_temp - 180;
  else
    alpha = alpha_temp;

  //make it at 0-360
  if (alpha < 0)
    alpha += 360;

  //  Serial.print("alpha is "); Serial.print(alpha);
  //  Serial.print(",   "); Serial.print(xx > 0 ? '+' : '-');
  //  Serial.print(",   "); Serial.println(xy > 0 ? '+' : '-');

  return alpha;
}

//for pitch
float get_x_in_xz_rotation()
{
  float xx = DCM_Matrix[0][0];    //DCM_Matrix_Inverse[0][0];
  float xy = DCM_Matrix[0][1];    //DCM_Matrix_Inverse[1][0];
  float xz = DCM_Matrix[0][2];    //DCM_Matrix_Inverse[2][0];

  float alpha_temp = TO_DEG(atan(xx / xz));
  float alpha;

  if (xx > 0 && xz < 0)
    alpha = alpha_temp + 180;
  else if (xx < 0 && xz < 0)
    alpha = alpha_temp - 180;
  else
    alpha = alpha_temp;

  //make it at 0-360
  if (alpha < 0)
    alpha += 360;
  //  Serial.print("alpha is "); Serial.print(alpha);
  //  Serial.print(",   "); Serial.print(xx > 0 ? '+' : '-');
  //  Serial.print(",   "); Serial.println(xz > 0 ? '+' : '-');

  return alpha;
}

void check_recoder_status()
{
  alphaYawForRec = get_x_in_xy_rotation();
  alphaPitchForRec = get_x_in_xz_rotation();

  if (output_recorder_status_on)
    output_recorder_status();
}


void output_recorder_status()
{
  Serial.print("$");  //mark
  int angle = shortForFloat(alphaYawForRec);
  printHolder(angle);
  Serial.print(abs(angle));

  angle = shortForFloat(alphaPitchForRec);
  printHolder(angle);
  Serial.println(abs(angle));
}



