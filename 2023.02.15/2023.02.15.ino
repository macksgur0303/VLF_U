#include <NewPing.h>
#include <Servo.h>
#include <MsTimer2.h>

#define encodPinA1 2
#define encodPinB1 3
#define MOTOR_DIR 4
#define MOTOR_PWM 5
#define A0pin A0
#define SIpin 22
#define CLKpin 24
#define RC_SERVO_PIN 9
#define MaxDistance  350
#define LEFT_STEER_ANGLE  -35  // 실험으로 구할것-45
#define NEURAL_ANGLE 91
#define RIGHT_STEER_ANGLE  30  // 실험으로 구할35
#define NPIXELS 128

Servo Steeringservo;
int Steering_Angle =  NEURAL_ANGLE;

NewPing F_sensor(F_Sonar, F_Sonar, MaxDistance);
double F_Sonar_distance = 0.0;
NewPing L_sensor(L_Sonar, L_Sonar, MaxDistance);
double L_Sonar_distance = 0.0;
NewPing R_sensor(R_Sonar, R_Sonar, MaxDistance);
double R_Sonar_distance = 0.0;

void setup() {
  Serial.begin(9600);
  pinMode(L_InPin_2, OUTPUT);
  pinMode(L_InPin_1, OUTPUT);
  pinMode(R_InPin_2, OUTPUT);
  pinMode(R_InPin_1, OUTPUT);
  read_sonar_sensor();
  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(NEURAL_ANGLE); 
}

void motor_control(int direction, int speed)
{
  digitalWrite(MOTOR_DIR.direction);
  analogWrite(MOTOR_PWM, speed);
}

void steering_control(int steer_angle)
{
  Steeringservo.write(NEURAL_ANGLE+ steer_angle);
}

void read_sonar_sensor(void) //초음파센서 측정
{
    R_Sonar_distance = R_sensor.ping_cm()*10.0;
    L_Sonar_distance = L_sensor.ping_cm()*10.0;
    F_Sonar_distance = F_sensor.ping_cm()*10.0;
    
    if(L_Sonar_distance == 0)
    {
      L_Sonar_distance = MaxDistance * 20;
    }

    if(F_Sonar_distance == 0)
    {
      F_Sonar_distance =  MaxDistance * 20;
    }

    if(R_Sonar_distance == 0)
    {
      R_Sonar_distance =  MaxDistance * 20;
    }
}

/////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////

void keep_sensor_going(void) // 초음파 거리에 따른 주행
{
  int re = read_line_sensor();
  
  if(re == 100)
  {

    if(F_Sonar_distance <= 950 && F_Sonar_distance >= 500 && R_Sonar_distance > 1000 ) // 미로 우회전
    {
      motor_control(1,84*2,10*2);
      delay(1800);
      motor_control(1,84*2,100*2);
      delay(1000);
    }

    else
    {
      if(R_Sonar_distance + L_Sonar_distance <= 1170 && R_Sonar_distance - 70 <= L_Sonar_distance && R_Sonar_distance + 70 >= L_Sonar_distance) // 좌 우 거리 같을때 진행
      {
        motor_control(1,84*2,100*2);
      
      }
    
      else if( L_Sonar_distance < R_Sonar_distance) // 우회전
      {
        motor_control(1,84*2,70*2);
        if(L_Sonar_distance < 300) // 비상용 우회전
        {
         motor_control(1,84*2,0);
        }  
      }

      else if( L_Sonar_distance > R_Sonar_distance ) // 좌회전
      {
        motor_control(1,57*2,110*2);
      
        if(R_Sonar_distance < 300) // 비상용 좌회전
        {
          motor_control(1,0,100*2);
        }   
      }

      else
      {
        motor_control(1,90*2,100*2);
      }
    }
    /*
    if(F_Sonar_distance <= 700 && R_Sonar_distance > 900) // 미로 우회전
    {
      motor_control(1,60*2,10*2);
      delay(2000);
    }
    */

    
  } 
}

///////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////

void serial_print(void)
{
  Serial.print(L_sensor.ping_cm());
  Serial.print("  ");
  Serial.print(F_sensor.ping_cm());
  Serial.print("  ");
  Serial.println(R_sensor.ping_cm());
  Serial.print(L_Sonar_distance);
  Serial.print("  ");
  Serial.print(F_Sonar_distance);
  Serial.print("  ");
  Serial.println(R_Sonar_distance);
  
}
void loop() 
{
  read_sonar_sensor();
  steering_control(LEFT_STEER_ANGLE);
  steering_control(RIGHT_STEER_ANGLE);
  keep_sensor_going();
  serial_print();
}
