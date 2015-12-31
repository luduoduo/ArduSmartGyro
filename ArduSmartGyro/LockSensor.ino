#define HIGH 1
#define LOW 0

enum {
  POSITIVE = 0,
  NEGATIVE,
  ACTIVE
};
enum {
  DOOR_OPEN = -1,	//Door is open
  LOCK_OPEN = 0,	//Door is close but lock is open
  LOCK_CLOSE1,	//in fact, all of status>0 are close
  LOCK_CLOSE2,
  LOCK_CLOSE3
};

//rotate from u to l, true means clockwise, false means counter clockwise
bool gCLOCKWISE = true;
float gHALFPOINT = -180.0f;

//[2] is earlier postion than [1]
char stack_name[3] = {'P', 'N', 'A'};

//[0] is current, [1] is previous
//P->A->N for clockWise is true and lock is open
int area_stack[3] = {1, 2, 0};

//默认状态是门关闭+锁打开
int lock_status = LOCK_OPEN;
int door_status = LOCK_OPEN;

void output_area_stack()
{
  Serial.print("history <");
  Serial.print(gCLOCKWISE ? " CW> " : "CCW> ");
  Serial.print("u="); Serial.print(threshold_unlock);
  Serial.print(", l="); Serial.print(threshold_lock);
  Serial.print(", h="); Serial.print(gHALFPOINT); Serial.print(" : ");
  Serial.print(stack_name[area_stack[2]]); Serial.print("[2] -> ");
  Serial.print(stack_name[area_stack[1]]); Serial.print("[1] -> ");
  Serial.print(stack_name[area_stack[0]]); Serial.print("[0]    ");
  Serial.print(lock_status);
  Serial.print("  (");
  Serial.print(alphaForLock);
  Serial.println(")");
}

//toLock means if unlock or lock has just been updated
void init_lock_sensor(bool toLock)
{
  //by default, lock is open, init status is P->A->N, meaning of unlock
  if (toLock)
  {
    //toLock is true means we are marking 'l' and make the lock is close (N->A->P)
    lock_status = LOCK_CLOSE1;
    area_stack[0] = POSITIVE;
    area_stack[1] = ACTIVE;
    area_stack[2] = NEGATIVE;
  }
  else
  {
    lock_status = LOCK_OPEN;
    area_stack[0] = NEGATIVE;
    area_stack[1] = ACTIVE;
    area_stack[2] = POSITIVE;
  }

  float diff = threshold_lock - threshold_unlock;
  if (diff <= -180.0)
  {
    gCLOCKWISE = true;
    gHALFPOINT = (threshold_lock + threshold_unlock) / 2;
  }
  else if (diff > -180.0 && diff < 0)
  {
    gCLOCKWISE = false;
    gHALFPOINT = (threshold_lock + threshold_unlock) / 2 + 180;
    if (gHALFPOINT > 180)
      gHALFPOINT -= 360;
  }
  else if (diff > 0 && diff < 180.0)
  {
    gCLOCKWISE = true;
    gHALFPOINT = (threshold_lock + threshold_unlock) / 2 + 180;
    if (gHALFPOINT > 180)
      gHALFPOINT -= 360;
  }
  else //diff>180.0
  {
    gCLOCKWISE = false;
    gHALFPOINT = (threshold_lock + threshold_unlock) / 2;
  }

  if (gCLOCKWISE && gHALFPOINT == 180)
    gHALFPOINT = -180;
  else if (!gCLOCKWISE && gHALFPOINT == -180)
    gHALFPOINT = 180;

  //to avoid incoming wrong changing report when next angle arrives
  //freeze can clear history
  freeze_lock_status();
  //  output_area_stack();
}

//determine if the angle is in the area of from the start to the end
//there are different process according to closkwise
bool determineArea(float startAngle, float endAngle, float angle)
{
  bool result = false;
  if (gCLOCKWISE)
  {
    if (startAngle < endAngle)
    {
      if ( startAngle < angle  && angle < endAngle)
        result = true;
    }
    else
    {
      if (angle > startAngle || angle < endAngle)
        result = true;
    }
  }
  else
  {
    if (startAngle < endAngle)
    {
      if ( angle < startAngle || angle > endAngle)
        result = true;

    }
    else
    {
      if (endAngle < angle && angle < startAngle)
        result = true;
    }
  }
  return result;
}


//optimized inverse matrix, Z in ground grame and XY in plane frame
float get_z_in_xy_rotation()
{
  //z=(0,0,1) in ground frame is vertical, we watch it in plane frame.
  //The column DCM_Inverse[*][2] is equal to DCM_I * Z(0,0,1), meaning Zp (plane/body frame) comming from Zg,
  //meaning also the row DCM_Matrix[2][*]
  float zx = DCM_Matrix[2][0];    //DCM_Matrix_Inverse[0][2];
  float zy = DCM_Matrix[2][1];    //DCM_Matrix_Inverse[1][2];
  float zz = DCM_Matrix[2][2];    //DCM_Matrix_Inverse[2][2];

  float alpha_temp;
  float alpha;

  //lufei: temp to avoid nan
  if (abs(zy) < 0.001)
    alpha_temp = 90;
  else
    alpha_temp = TO_DEG(atan(zx / zy));

  //  Serial.print("zx="); Serial.print(zx);
  //  Serial.print(", zy="); Serial.print(zy);
  //  Serial.print(", zz="); Serial.print(zz);
  //  Serial.print(", alpha="); Serial.println(alpha_temp);

  if (zx > 0 && zy < 0)
    alpha = alpha_temp + 180;
  else if (zx < 0 && zy < 0)
    alpha = alpha_temp - 180;
  else
    alpha = alpha_temp;

  //  Serial.print("alpha is "); Serial.print(alpha);
  //  Serial.print(",   "); Serial.print(zx>0?'+':'-');
  //  Serial.print(",   "); Serial.println(zy>0?'+':'-');

  return alpha;
}


void check_lock_sensor()
{

  alphaForLock = get_z_in_xy_rotation();

  //  Serial.print("a="); Serial.println(alphaForLock);

  //根据pitch判断门状态
  //  update_doorStatus(TO_DEG(pitch));

  //根据yaw判断区域
  update_area(alphaForLock);

  //检查锁的状态
  bool isChanged = check_lock_status();
  if (isChanged)
    freeze_lock_status();

  if (output_lock_status_on)
  {
    output_lock_status();
    //    Serial.print("lock is "); Serial.print(lock_status);
    //    Serial.print("    alpha is "); Serial.println(alpha);
  }

  //disable LED alarm
  //  if (lock_status <= LOCK_OPEN)
  //  {
  //    digitalWrite(STATUS_LED_PIN, HIGH);
  //  }
  //  else
  //  {
  //    if (lock_status > LOCK_OPEN)
  //      digitalWrite(STATUS_LED_PIN, LOW);
  //    if (lock_status > LOCK_CLOSE1)
  //      digitalWrite(STATUS_LED_PIN, LOW);
  //    if (lock_status > LOCK_CLOSE2)
  //      digitalWrite(STATUS_LED_PIN, LOW);
  //  }
}




//void update_doorStatus(float degree)
//{
//  door_status = LOCK_OPEN;	//that is DOOR_CLOSE
//}

void update_area(float degree)
{
  int current_area = 0;

  if (determineArea(gHALFPOINT, threshold_unlock, degree))
    current_area = NEGATIVE;
  else if (determineArea(threshold_unlock, threshold_lock, degree))
    current_area = ACTIVE;
  else
    current_area = POSITIVE;

  //  Serial.print("HIT: "); Serial.print(stack_name[current_area]);Serial.print(",  ");Serial.println(degree);

  //the same area
  if (current_area == area_stack[0])
    return;

  area_stack[2] = area_stack[1];
  area_stack[1] = area_stack[0];
  area_stack[0] = current_area;

//  output_area_stack();
}

//根据三段历史来判断状态是否发生变化
bool check_lock_status()
{
  if (area_stack[1] != ACTIVE)
    return false;	//keep status

  //N->A->P lock++, P->A->N lock--
  if (area_stack[0] == POSITIVE && area_stack[2] == NEGATIVE)
    lock_status++;
  else if (area_stack[0] == NEGATIVE && area_stack[2] == POSITIVE)
    lock_status--;
  else  //NAN, PAP, or others considered as no change
    return false;
  return true;
}


//If we do not freeze (clear) status, the changing event will be sent repeatedly
void freeze_lock_status()
{
  //clear history
  area_stack[1] = area_stack[0];
  area_stack[2] = area_stack[0];
  Serial.println("freeze_lock_status");
}

//for exmaple: %-1659%0 means angle=-16.59 degree and status=0
void output_lock_status()
{
  Serial.print("%");  //mark
  int angle = shortForFloat(alphaForLock);
  printHolder(angle);
  Serial.print(abs(angle));
  Serial.print("%");  //seperator
  Serial.println(lock_status >= 0 ? lock_status : 0);
}

