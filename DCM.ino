/* This file is part of the Nano AHRS Firmware */

// DCM

/**************************************************/
//to correct numerical errors to make DCM back to orthogonal matrix
void Normalize(void)
{
  float error = 0;
  float temporary[3][3];
  float renorm = 0;

  error = -Vector_Dot_Product(&DCM_Matrix[0][0], &DCM_Matrix[1][0]) * .5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19   -error/2*Y
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19   -error/2*X

  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19   X-error/2*Y
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19   Y-error/2*X

  Vector_Cross_Product(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b //eq.20

  //The equation is on the Tylor's expansion with approximation to avoid square roots and divisions.
  renorm = .5 * (3 - Vector_Dot_Product(&temporary[0][0], &temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

  renorm = .5 * (3 - Vector_Dot_Product(&temporary[1][0], &temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

  renorm = .5 * (3 - Vector_Dot_Product(&temporary[2][0], &temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void Drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift.
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;


  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0] * Accel_Vector[0] + Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2 * abs(1 - Accel_magnitude), 0, 1); //

  Vector_Cross_Product(&errorRollPitch[0], &Accel_Vector[0], &DCM_Matrix[2][0]); //adjust the ground of reference

  //p control
  Vector_Scale(&Omega_P[0], &errorRollPitch[0], Kp_ROLLPITCH * Accel_weight); //param1=p2*p3
  //i control
  Vector_Scale(&Scaled_Omega_I[0], &errorRollPitch[0], Ki_ROLLPITCH * Accel_weight);

  Vector_Add(Omega_I, Omega_I, Scaled_Omega_I); //Omega_I is i control value, which is accumulated for integration , lufei

//  Serial.print("weight="); Serial.println(Accel_weight);
//  Serial.print("Omega_P="); Serial.print(Omega_P[0]); Serial.print(", "); Serial.print(Omega_P[1]); Serial.print(", "); Serial.println(Omega_P[2]);
//  Serial.print("Omega_I="); Serial.print(Omega_I[0]); Serial.print(", "); Serial.print(Omega_I[1]); Serial.print(", "); Serial.println(Omega_I[2]);

  //lufei: disable compass fixing, because it is easy be interfered.
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
  /*
    mag_heading_x = cos(MAG_Heading);
    mag_heading_y = sin(MAG_Heading);
    errorCourse = (DCM_Matrix[0][0] * mag_heading_y) - (DCM_Matrix[1][0] * mag_heading_x); //Calculating YAW error
    Vector_Scale(errorYaw, &DCM_Matrix[2][0], errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

    Vector_Scale(&Scaled_Omega_P[0], &errorYaw[0], Kp_YAW ); //.01proportional of YAW.
    Vector_Add(Omega_P, Omega_P, Scaled_Omega_P); //Adding  Proportional.

    Vector_Scale(&Scaled_Omega_I[0], &errorYaw[0], Ki_YAW); //.00001Integrator
    Vector_Add(Omega_I, Omega_I, Scaled_Omega_I); //adding integrator to the Omega_I
  */
}

void Matrix_update(void)
{
  //  Serial.print("[Gyro] ");
  //  Serial.print(TO_DEG(Gyro_Vector[0]));
  //  Serial.print(",       ");
  //  Serial.print(TO_DEG(Gyro_Vector[1]));
  //  Serial.print(",       ");
  //  Serial.println(TO_DEG(Gyro_Vector[2]));

  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

  //  Serial.print("[Omega] ");
  //  Serial.print(TO_DEG(Omega_Vector[0]));
  //  Serial.print(",       ");
  //  Serial.print(TO_DEG(Omega_Vector[1]));
  //  Serial.print(",       ");
  //  Serial.println(TO_DEG(Omega_Vector[2]));


#if DEBUG__NO_DRIFT_CORRECTION == true // Do not use drift correction
  Update_Matrix[0][0] = 0;
  Update_Matrix[0][1] = -G_Dt * Gyro_Vector[2]; //-z
  Update_Matrix[0][2] = G_Dt * Gyro_Vector[1]; //y
  Update_Matrix[1][0] = G_Dt * Gyro_Vector[2]; //z
  Update_Matrix[1][1] = 0;
  Update_Matrix[1][2] = -G_Dt * Gyro_Vector[0];
  Update_Matrix[2][0] = -G_Dt * Gyro_Vector[1];
  Update_Matrix[2][1] = G_Dt * Gyro_Vector[0];
  Update_Matrix[2][2] = 0;
#else // Use drift correction, we are here by default. Lufei: right here
  Update_Matrix[0][0] = 0;
  Update_Matrix[0][1] = -G_Dt * Omega_Vector[2]; //-z
  Update_Matrix[0][2] = G_Dt * Omega_Vector[1]; //y
  Update_Matrix[1][0] = G_Dt * Omega_Vector[2]; //z
  Update_Matrix[1][1] = 0;
  Update_Matrix[1][2] = -G_Dt * Omega_Vector[0]; //-x
  Update_Matrix[2][0] = -G_Dt * Omega_Vector[1]; //-y
  Update_Matrix[2][1] = G_Dt * Omega_Vector[0]; //x
  Update_Matrix[2][2] = 0;
#endif

  Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); //a*b=c, Eq.17

  for (int x = 0; x < 3; x++) //Matrix Addition (update)
  {
    for (int y = 0; y < 3; y++)
    {
      DCM_Matrix[x][y] += Temporary_Matrix[x][y];
    }
  }
}


void Euler_angles(void)
{
  pitch = -asin(DCM_Matrix[2][0]);
  roll = atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);
  yaw = atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]);
}


