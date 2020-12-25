~~~c
#include <Pixy2.h>
#include <PIDLoop.h>
#include <ZumoMotors.h>

// this limits how fast Zumo travels forward (400 is max possible for Zumo)
#define MAX_TRANSLATE_VELOCITY  250

Pixy2 pixy;
ZumoMotors motors;
PIDLoop panLoop(350, 0, 600, true); //픽시 서보
PIDLoop tiltLoop(500, 0, 700, true);  //픽시 서보
PIDLoop rotateLoop(300, 600, 300, false);  //픽시 좌 우 
PIDLoop translateLoop(400, 800, 300, false);  //픽시 위 아래

#define trigPin 10
#define echoPin 9
#define rtrigPin 13
#define rechoPin 12

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  // 모터 객체 초기화
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  
  // 픽시 객체 초기화
  pixy.init();
  // user color connected components program
  pixy.changeProg("color_connected_components");
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(rtrigPin, OUTPUT);
  pinMode(rechoPin, INPUT);
}

// 최소 30프레임(1/2초) 동안 존재했던 가장 큰 블록을 가져간다.
// index를 반환하고 그렇지 않으면 -1을 반환한다.
int16_t acquireBlock()
{
  if (pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_age>30)
    return pixy.ccc.blocks[0].m_index;
  return -1;
}

// 지정된 index로 블럭을 찾는다.
// 프레임 -- acquireBlock()에 고정시킨 객체
// 현재 프레임에 없는 경우 NULL 반환

int mm;

Block *trackBlock(uint8_t index)
{
  uint8_t i;
  //uint8_t mm;

  for (i=0; i<pixy.ccc.numBlocks; i++)
  {
    if (index==pixy.ccc.blocks[i].m_index)
      mm = pixy.ccc.blocks[i].m_signature;
      return &pixy.ccc.blocks[i];
  }
  return NULL;
}

#define LEFT_DIR 4
#define LEFT_WHEEL 5
#define RIGHT_DIR 7
#define RIGHT_WHEEL 6 

void DriveInit() {
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_WHEEL, OUTPUT);
  pinMode(LEFT_WHEEL, OUTPUT);
  } 
  
// 바퀴 정지
void Stop() {
    analogWrite(LEFT_WHEEL, 0);
    analogWrite(RIGHT_WHEEL, 0);
    }

//바퀴 동작
void Drive(int32_t left, int32_t right) {
  digitalWrite(LEFT_DIR, (left > 0) ? 1 : 0);   //바퀴 앞 뒤 변경
  digitalWrite(RIGHT_DIR, (right > 0) ? 1 : 0);
  analogWrite(LEFT_WHEEL, abs(left));           //abs 절대값, 바퀴 작동
  analogWrite(RIGHT_WHEEL, abs(right));
  }

void loop() {  
  static int16_t index = -1;
  int32_t panOffset, tiltOffset, headingOffset, left, right;
  Block *block=NULL;
  pixy.ccc.getBlocks();

  if (index==-1) // 물체를 찾는중
  {
    Serial.println("Searching for block...");
    index = acquireBlock();
    if (index>=0)
      Serial.println("Found block!");
 }
  // 블럭 저장
  if (index>=0)
     block = trackBlock(index);

  // 물체 추적
  if (block) {
    // calculate pan and tilt errors
    panOffset = (int32_t)pixy.frameWidth/2 - (int32_t)block->m_x;
    tiltOffset = (int32_t)block->m_y - (int32_t)pixy.frameHeight/2;  

    // 서보모터값 업데이트
    panLoop.update(panOffset);
    tiltLoop.update(tiltOffset);

    // 서보 작동
    pixy.setServos(panLoop.m_command, tiltLoop.m_command);

    // 서보모터
    panOffset += panLoop.m_command - PIXY_RCS_CENTER_POS;
    tiltOffset += tiltLoop.m_command - PIXY_RCS_CENTER_POS - PIXY_RCS_CENTER_POS/2 + PIXY_RCS_CENTER_POS/8;

    rotateLoop.update(panOffset);   // 픽시 좌 우
    translateLoop.update(-tiltOffset);   // 픽시 위 아래

    // 최대속도 유지
    if (translateLoop.m_command>MAX_TRANSLATE_VELOCITY)
      translateLoop.m_command = MAX_TRANSLATE_VELOCITY;

    // 회전속도 기준 바퀴속도 
    // left right 바퀴 입력값
    //left = -rotateLoop.m_command + translateLoop.m_command;
    //right = rotateLoop.m_command + translateLoop.m_command;
    //Serial.println(rotateLoop.m_command);    //rotateloop test 좌 우
    //Serial.println(translateLoop.m_command);   //translateloop test 위 아래
    left = -rotateLoop.m_command;  // 좌
    right = rotateLoop.m_command;  // 우

   left = left / 2;   //값 조정
   right = right / 2;
  /**if (left > 120){
    left = 120;
   }
   if (right > 120) {
    right = 120;
   }   **/

// 초음파를 보낸다. 다 보내면 echo가 HIGH 상태로 대기하게 된다.
    digitalWrite(trigPin, LOW);
    digitalWrite(echoPin, LOW);
    digitalWrite(trigPin, HIGH);
    digitalWrite(trigPin, LOW);
  
  // echoPin 이 HIGH를 유지한 시간을 저장 한다.
    unsigned long duration = pulseIn(echoPin, HIGH); 
  // HIGH 였을 때 시간(초음파가 보냈다가 다시 들어온 시간)을 가지고 거리를 계산 한다.
  float distance = ((float)(340 * duration) / 10000) / 2;  
  
  Serial.print(distance);
  Serial.println("cm");

//////////////////////////////////////////////
/////////////////////////////////////////////

    digitalWrite(rtrigPin, LOW);
    digitalWrite(rechoPin, LOW);
    digitalWrite(rtrigPin, HIGH);
    digitalWrite(rtrigPin, LOW);
  
  // echoPin 이 HIGH를 유지한 시간을 저장 한다.
    unsigned long rduration = pulseIn(rechoPin, HIGH); 
  // HIGH 였을 때 시간(초음파가 보냈다가 다시 들어온 시간)을 가지고 거리를 계산 한다.
  float rdistance = ((float)(340 * rduration) / 10000) / 2;  
  
  Serial.print(rdistance);
  Serial.println("cm");
  
  /**
   float duration, distance;
  // 초음파를 보낸다. 다 보내면 echo가 HIGH 상태로 대기하게 된다.
  digitalWrite(trigPin, HIGH);
  //delay(10);
  
  digitalWrite(trigPin, LOW);
  // echoPin 이 HIGH를 유지한 시간을 저장 한다.
  duration = pulseIn(echoPin, HIGH); 
  // HIGH 였을 때 시간(초음파가 보냈다가 다시 들어온 시간)을 가지고 거리를 계산 한다.
  distance = ((float)(340 * duration) / 10000) / 2;  
  
  Serial.print(distance);
  Serial.println("cm");
**/
   
   // 픽시에 저장된 첫 번째 색
   if (mm == 1) {

////// 좌측 바퀴쪽 초음파센서
    while ( distance < 30 ){
      Drive(200, 100);
    digitalWrite(trigPin, LOW);
    digitalWrite(echoPin, LOW);    
    digitalWrite(trigPin, HIGH);    
    digitalWrite(trigPin, LOW);
  
  // echoPin 이 HIGH를 유지한 시간을 저장 한다.
    unsigned long duration = pulseIn(echoPin, HIGH); 
  // HIGH 였을 때 시간(초음파가 보냈다가 다시 들어온 시간)을 가지고 거리를 계산 한다.
  float distance = ((float)(340 * duration) / 10000) / 2;  
      
      Serial.println("noooo");
      if (distance > 30 )
        break;
    }

    //////////////////////
    ////////////////////// 우측바퀴쪽 초음파센서

  while ( rdistance < 30 ){
    Drive(100, 200);    
    digitalWrite(rtrigPin, LOW);
    digitalWrite(rechoPin, LOW);    
    digitalWrite(rtrigPin, HIGH);    
    digitalWrite(rtrigPin, LOW);

  // echoPin 이 HIGH를 유지한 시간을 저장 한다.
    unsigned long rduration = pulseIn(rechoPin, HIGH); 
  // HIGH 였을 때 시간(초음파가 보냈다가 다시 들어온 시간)을 가지고 거리를 계산 한다.
  float rdistance = ((float)(340 * rduration) / 10000) / 2;  
      
      Serial.println("noooo");
      if (rdistance > 30 )
        break;
    }

  ////////////////////
  ////////////////////
     
    if (block->m_height >= 180) {  // 물체와 거리가 180이상 가까워 지면 정지
    //Drive(-200, -200);
    Stop();
    Serial.println(block->m_height);
    delay(500);
   }
   else if (block -> m_height <= 50){  // 물체와 거리가 50 이하로 멀어지면 따라가기
    if (left < 0){
      int aa = left + 200;
      Drive(aa, 200);
    }
    //Drive(150, 150);
    //Serial.print("left go");
    else if (right < 0) {
      int bb = right + 200;
      Drive(200, bb);
     }
     else {
      Drive(200, 200);
      }
   }
   else {
    if ( left < 0)
      Drive(0, right);
    if ( right < 0)
      Drive(left, 0);
    //Drive(left, right);
    Serial.println(left);
    Serial.println(right);
    }
  }
   /**
   else if (mm == 2) {
    if (block -> m_height > 150 && block -> m_height < 200) {   // 물체와 거리가 150 이상 200 이하일때 정지
      Stop();
    }
   else if (block->m_height >= 180) {  // 물체와 거리가 180이상 가까워 지면 정지
    //Drive(-200, -200);
    Stop();
    Serial.println(block->m_height);
    delay(500);
   }
   else if (block -> m_height <= 50){  // 물체와 거리가 50 이하로 멀어지면 따라가기
    Drive(150, 150);
    Serial.print("gooo");
   }
   else {
    if ( left < 0)
      Drive(0, right);
    if ( right < 0)
      Drive(left, 0);
    //Drive(left, right);
    Serial.println(left);
    Serial.println(right);
    }
   } **/
  } 
  else {
    rotateLoop.reset();
    translateLoop.reset();
    Stop();
    index = -1;
  }
   }
/**
  void cho_loop() {
  float duration, distance;
  
  // 초음파를 보낸다. 다 보내면 echo가 HIGH 상태로 대기하게 된다.
  digitalWrite(trigPin, HIGH);
  delay(10);
  digitalWrite(trigPin, LOW);
  
  // echoPin 이 HIGH를 유지한 시간을 저장 한다.
  duration = pulseIn(echoPin, HIGH); 
  // HIGH 였을 때 시간(초음파가 보냈다가 다시 들어온 시간)을 가지고 거리를 계산 한다.
  distance = ((float)(340 * duration) / 10000) / 2;  
  
  Serial.print(distance);
  Serial.println("cm");
  // 수정한 값을 출력
  
}
**/
