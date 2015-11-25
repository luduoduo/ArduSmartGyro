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
char stack_name[3] = {'P', 'N', 'A'};

//[0] is current, [1] is previous
int area_stack[3];

//默认状态是门关闭+锁打开
int lock_status = LOCK_OPEN;
int door_status = LOCK_OPEN;

float alphaForLock = 0;

//user must reset status @ N area
void init_lock_sensor()
{
  //by default, lock is open, init status is P->A->N, meaning of unlock
  lock_status = LOCK_OPEN + 1; //update_area will +1 later
  area_stack[0] = NEGATIVE;
  area_stack[1] = ACTIVE;
  area_stack[2] = POSITIVE;
}

float get_z_in_xy_rotation()
{
  //z=(0,0,1) in body frame, here to compute (xx,xy,xz) which is in original fixed frame
  
  float zx = DCM_Matrix_Inverse[0][2];
  float zy = DCM_Matrix_Inverse[1][2];
  float zz = DCM_Matrix_Inverse[2][2];
//  Serial.print("z="); Serial.print(zx); Serial.print(", "); Serial.print(zy); Serial.print(", "); Serial.println(zz);

//  float xx = DCM_Matrix_Inverse[0][0];
//  float xy = DCM_Matrix_Inverse[1][0];
//  float xz = DCM_Matrix_Inverse[2][0];
//  Serial.print("xxxx="); Serial.print(xx); Serial.print(", "); Serial.print(xy); Serial.print(", "); Serial.print(xz); Serial.print("   AngleX="); Serial.println(TO_DEG(atan(xz / xy)));
  // float zr = sqrt(zx*zx + zy*zy + zz*zz);
  float alpha_temp;
  float alpha;

  //lufei: temp to avoid nan
  if (abs(zy) < 0.001)
    alpha_temp = 90;
  else
    alpha_temp = TO_DEG(atan(zx / zy));

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

  if (lock_status <= LOCK_OPEN)
  {
    digitalWrite(STATUS_LED_PIN, HIGH);
  }
  else
  {
    if (lock_status > LOCK_OPEN)
      digitalWrite(STATUS_LED_PIN, LOW);
    if (lock_status > LOCK_CLOSE1)
      digitalWrite(STATUS_LED_PIN, LOW);
    if (lock_status > LOCK_CLOSE2)
      digitalWrite(STATUS_LED_PIN, LOW);
  }
}




void update_doorStatus(float degree)
{
  door_status = LOCK_OPEN;	//that is DOOR_CLOSE
}

void update_area(float degree)
{
  int current_area = 0;
  if (threshold_lock > threshold_unlock)
  {
    if (degree > threshold_lock)
      current_area = POSITIVE;
    else if (degree < threshold_unlock)
      current_area = NEGATIVE;
    else
      current_area = ACTIVE;
  }
  else
  {
    if (degree > threshold_unlock)
      current_area = NEGATIVE;
    else if (degree < threshold_lock)
      current_area = POSITIVE;
    else
      current_area = ACTIVE;
  }

  //the same area
  if (current_area == area_stack[0])
    return;

  area_stack[2] = area_stack[1];
  area_stack[1] = area_stack[0];
  area_stack[0] = current_area;
}

//根据三段历史来判断状态
bool check_lock_status()
{
  if (area_stack[1] != ACTIVE)
    return false;	//keep status

  //N->A->P lock++, P->A->N lock--
  if (area_stack[0] == POSITIVE && area_stack[2] == NEGATIVE)
    lock_status++;
  else if (area_stack[0] == NEGATIVE && area_stack[2] == POSITIVE)
    lock_status--;


  //  Serial.print("history: ");
  //  Serial.print(stack_name[area_stack[2]]);
  //  Serial.print(" -> ");
  //  Serial.print(stack_name[area_stack[1]]);
  //  Serial.print(" -> ");
  //  Serial.print(stack_name[area_stack[0]]);
  //  Serial.print("      ");
  //  Serial.println(lock_status);

  return true;
}

void freeze_lock_status()
{
  //clear history
  area_stack[1] = area_stack[0];
  area_stack[2] = area_stack[0];
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

