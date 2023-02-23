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
#define NEURAL_ANGLE1 -1
#define RIGHT_STEER_ANGLE  30  // 실험으로 구할35
#define NPIXELS 128
#define threshold_value 60
#define sensor_num 3

byte Pixel[NPIXELS];

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];
int flag_line_adaptation;
int sensor_values[sensor_num];
int sensor_index = 0;


Servo Steeringservo;
int Steering_Angle =  NEURAL_ANGLE;

NewPing F_sensor(48, 49, MaxDistance);
double F_Sonar_distance = 0.0;
NewPing L_sensor(52, 53, MaxDistance);
double L_Sonar_distance = 0.0;
NewPing R_sensor(50, 51, MaxDistance);
double R_Sonar_distance = 0.0;

void setup() {
  Serial.begin(9600); 
  read_sonar_sensor();
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i] = 1023; // 0;
    MIN_LineSensor_Data[i] = 0; // 1023;
  }

  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);
  pinMode(A0pin, INPUT);

  digitalWrite(SIpin, LOW);
  digitalWrite(CLKpin, LOW);

#if FARADOC

  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  
#endif

  flag_line_adaptation = 0;
  
  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(NEURAL_ANGLE);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);


}

void motor_control(int direction, int speed)
{
  digitalWrite(MOTOR_DIR, direction);
  analogWrite(MOTOR_PWM, speed);
}

void steering_control(int steer_angle)
{
  if(steer_angle >= RIGHT_STEER_ANGLE)
  {
    steer_angle = RIGHT_STEER_ANGLE;
  }

  if(steer_angle <= LEFT_STEER_ANGLE)
  {
    steer_angle = LEFT_STEER_ANGLE;
  }
  
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

int filltering_sonar(int Sonar_distance)
{
  float sum;
  for (int j = 0; j < sensor_num; j++)
  {
    sensor_values[sensor_index] = Sonar_distance;
    sensor_index = (sensor_index + 1) % sensor_num; 
    sum = 0.0;

    for (int i = 0; i < sensor_num; i++) 
    {
      sum += sensor_values[i];
    }
  }
  float Filtered_Sonar_distance = sum / sensor_num;

  return Filtered_Sonar_distance;
}
/////////////////////////////////////////////////////////

void line_adapation(void)
{
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    if (LineSensor_Data[i] >= MAX_LineSensor_Data[i])
    {
      MAX_LineSensor_Data[i] = LineSensor_Data[i];
    }

    if (LineSensor_Data[i] <= MIN_LineSensor_Data[i])
    {
      MIN_LineSensor_Data[i] = LineSensor_Data[i];
    }
  }
}

void read_line_sensor(void)
{
  int i;

  delayMicroseconds (1);
  delay(10);
  
  digitalWrite(CLKpin, LOW);
  digitalWrite(SIpin, HIGH);
  digitalWrite(CLKpin, HIGH);
  digitalWrite(SIpin, LOW);

  delayMicroseconds (1);

  for (i = 0; i < NPIXELS; i++)
  {
    Pixel[i] = analogRead (A0pin) / 4;
    digitalWrite (CLKpin, LOW);
    delayMicroseconds (1);
    digitalWrite (CLKpin, HIGH);
  }

  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data_Adaption[i] = map(Pixel[i], MIN_LineSensor_Data[i], MAX_LineSensor_Data[i], 0, 256);
  }
}

void threshold(void)
{
  int i;
  
  for (i = 0; i < NPIXELS; i++)
  {
    if ((byte)Pixel[i] >= threshold_value)
    {
      LineSensor_Data_Adaption[i] = 255;
    }

    else
    {
      LineSensor_Data_Adaption[i] = 0;
    }
  }
}

void steering_by_camera(void)
{
  int i;
  long sum = 0;
  long x_sum = 0;
  int steer_data = 0;

  for (i = 0; i < NPIXELS; i++)
  {
    sum += LineSensor_Data_Adaption[i];
    x_sum += LineSensor_Data_Adaption[i]*i;
  }
  steer_data = (x_sum/sum) - NPIXELS/2;
  if (sum == 0)
  {
    keep_sensor_going();
  }
  
  else
  {
    steering_control(steer_data);
    motor_control(1,230);
  }
}

///////////////////////////////////////////////////////////

void keep_sensor_going(void) // 초음파 거리에 따른 주행
{
  int Filtered_F_Sonar_distance = filltering_sonar(F_Sonar_distance);
  int Filtered_R_Sonar_distance = filltering_sonar(R_Sonar_distance);
  int Filtered_L_Sonar_distance = filltering_sonar(L_Sonar_distance);

  if(Filtered_F_Sonar_distance <= 950 && Filtered_R_Sonar_distance > 3000) // 미로 우회전
  {
    steering_control(NEURAL_ANGLE1 + 50);
    motor_control(1,230);
    delay(1200);
    steering_control(NEURAL_ANGLE1);
    motor_control(1,230);
    delay(1000);
  }
  else
  {
    if(Filtered_R_Sonar_distance + Filtered_L_Sonar_distance <= 1170 && Filtered_R_Sonar_distance - 50 <= Filtered_L_Sonar_distance && Filtered_R_Sonar_distance + 50 >= Filtered_L_Sonar_distance) // 좌 우 거리 같을때 진행
    {
      steering_control(NEURAL_ANGLE1);
      motor_control(1,230);  
    }
  
    else if(Filtered_L_Sonar_distance < Filtered_R_Sonar_distance) // 우회전
    {
      steering_control(NEURAL_ANGLE1 + 15);
      motor_control(1,230);
      if(L_Sonar_distance < 300) // 비상용 우회전
      {
        steering_control(NEURAL_ANGLE1 + 30);
        motor_control(1,230);
      }  
    }
    else if(Filtered_L_Sonar_distance > Filtered_R_Sonar_distance ) // 좌회전
    {
      steering_control(NEURAL_ANGLE1 - 15);
      motor_control(1,230);
      
      if(Filtered_R_Sonar_distance < 300) // 비상용 좌회전
      {
        steering_control(NEURAL_ANGLE1 - 30);
        motor_control(1,230);
      }   
    }
    else
    {
      steering_control(NEURAL_ANGLE1);
      motor_control(1,230);
    }
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
  int Filtered_F_Sonar_distance = filltering_sonar(F_Sonar_distance);
  int Filtered_R_Sonar_distance = filltering_sonar(R_Sonar_distance);
  int Filtered_L_Sonar_distance = filltering_sonar(L_Sonar_distance);
  
  Serial.print(Filtered_L_Sonar_distance);
  Serial.print("  ");
  Serial.print(Filtered_F_Sonar_distance);
  Serial.print("  ");
  Serial.println(Filtered_R_Sonar_distance);

  delay(1000);
  
}
void loop() 
{
  read_sonar_sensor();
  read_line_sensor();
  line_adapation();
  threshold();
  steering_by_camera();
  serial_print();
}
