#include <Motor.h>
#include <PID_v1.h>
#include <Arduino.h>

#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

double m1_x = 0.7071;
double m1_y = 0.7071;
double m1_r = 0.5;

double m2_x = 0.7071;
double m2_y = -0.7071;
double m2_r = 0.5;

double m3_x = -0.7071;
double m3_y = -0.7071;
double m3_r = 0.5;

double m4_x = -0.7071;
double m4_y = 0.7071;
double m4_r = 0.5;

double max_coef = abs(m1_x) + abs(m1_y) + abs(m1_r);
//print("before ", max_coef)

int interval = 5;
int CPR = 720;
class squareBase
{
public:
  Motor *m1, *m2, *m3, *m4;
  bool initial = true;
  double initialAngle = 0;
  UniversalEncoder *Enc1, *Enc2;
  double xKp = 0, xKi = 0, xKd = 0;
  double xSetpoint = 0, xOutput = 0, xInput = 0;
  double yKp = 0, yKi = 0, yKd = 0;
  double ySetpoint = 0, yOutput = 0, yInput = 0;
  double rKp = 0, rKi = 0, rKd = 0;
  double rSetpoint = 0, rOutput = 0, rInput = 0;
  double prevRfR = 0;
  unsigned long count = 0;
  double fx, fy, fr;
  double dFx = 0, dFy = 0, dFr = 0;
  int PWM = 0;
  double rFx, rFy, rFr;
  double M1, M2, M3, M4;
  double m_1, m_2, m_3, m_4;
  unsigned long prevTime = 0;
  double filterR = 15;
  double prevReading = 0;
  double diff = 0;
  double headingDegrees, heading, xyHeading;
  double offset = 0;
  double linearOffset = 0.1;
  double loadOffset = 0.5; // BMM150 bmm = BMM150();
  PID *fxPID = new PID(&xInput, &xOutput, &xSetpoint, xKp, xKi, xKd, DIRECT);
  PID *fyPID = new PID(&yInput, &yOutput, &ySetpoint, yKp, yKi, yKd, DIRECT);
  PID *frPID = new PID(&rInput, &rOutput, &rSetpoint, rKp, rKi, rKd, DIRECT);

  squareBase()
  {
    m1 = new Motor();
    m2 = new Motor();
    m3 = new Motor();
    m4 = new Motor();
  }

  squareBase(Motor *mt1, Motor *mt2, Motor *mt3, Motor *mt4, UniversalEncoder *en1, UniversalEncoder *en2)
  {
    m1 = mt1;
    m2 = mt2;
    m3 = mt3;
    m4 = mt4;

    Enc1 = en1;
    Enc2 = en2;

    fxPID->SetMode(AUTOMATIC);
    fxPID->SetSampleTime(1);

    fyPID->SetMode(AUTOMATIC);
    fyPID->SetSampleTime(1);

    frPID->SetMode(AUTOMATIC);
    frPID->SetSampleTime(1);

    fxPID->SetOutputLimits(-255, 255);
    fyPID->SetOutputLimits(-255, 255);
    frPID->SetOutputLimits(-255, 255);

    // setmpu6050();
    m1_x = m1_x / max_coef;
    m2_x = m2_x / max_coef;
    m3_x = m3_x / max_coef;
    m4_x = m4_x / max_coef;

    m1_y = m1_y / max_coef;
    m2_y = m2_y / max_coef;
    m3_y = m3_y / max_coef;
    m4_y = m4_y / max_coef;

    m1_r = m1_r / max_coef;
    m2_r = m2_r / max_coef;
    m3_r = m3_r / max_coef;
    m4_r = m4_r / max_coef;
    Serial.println("cons");
  }
  void setmpu6050()
  {
    Wire.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
  }
  void setTuningsX(double kp, double ki, double kd)
  {
    xKp = kp;
    xKi = ki;
    xKd = kd;
    fxPID->SetTunings(kp, ki, kd);
  }
  void setTuningsY(double kp, double ki, double kd)
  {
    yKp = kp;
    yKi = ki;
    yKd = kd;
    fyPID->SetTunings(kp, ki, kd);
  }
  void setTuningsR(double kp, double ki, double kd)
  {
    rKp = kp;
    rKi = ki;
    rKd = kd;
    frPID->SetTunings(kp, ki, kd);
  }
  void setRobotSpeeds(float x_speed, float y_speed, float r_speed)
  {

    fx = x_speed;
    fy = y_speed;
    fr = r_speed;
    //	Serial.println(String("m1:")+fx+String("m2: ")+fy+String("m3: ")+fr);
    //setPoint();
    applyRobotSpeeds(fx, fy, fr);
  }
  void setPoint()
  {
    xSetpoint = fx;
    ySetpoint = fy;
    rSetpoint = fr;
  }

  void applyRobotSpeeds(float x_speed, float y_speed, float r_speed)
  {

    m_1 = (m1_x * x_speed + m1_y * y_speed + m1_r * r_speed);
    m_2 = (m2_x * x_speed + m2_y * y_speed + m2_r * r_speed);
    m_3 = (m3_x * x_speed + m3_y * y_speed + m3_r * r_speed);
    m_4 = (m4_x * x_speed + m4_y * y_speed + m4_r * r_speed);
    //Serial.println(String("m1:")+m_1+String("m2: ")+m_2+String("m3: ")+m_3+String("m4: ")+m_4);
    mapSpeed(m_1 * loadOffset, m_2 * loadOffset, m_3 * loadOffset, m_4 * loadOffset);
  }
  void mapSpeed(double m_1, double m_2, double m_3, double m_4)
  {
    M1 = map(m_1, -370, 370, -255, 255);
    M2 = map(m_2, -370, 370, -255, 255);
    M3 = map(m_3, -370, 370, -255, 255);
    M4 = map(m_4, -370, 370, -255, 255);
    setSpeed(M1, M2, M3, M4);
  }
  void setSpeed(double s1, double s2, double s3, double s4)
  {
    m1->setPWM(s1);
    m2->setPWM(s2);
    m3->setPWM(s3);
    m4->setPWM(s4);
    // Serial.println(String("m1:")+s1+String("m2: ")+s2+String("m3: ")+s3+String("m4: ")+s4);
  }
  double calculateFx()
  {
    return (((double)Enc1->getReadings() / CPR) * ((double)1000 * 60 / interval) * 0.58 * linearOffset);
  }

  double calculateFy()
  {
    return (((double)Enc2->getReadings() / CPR) * ((double)1000 * 60 / interval) * 0.58 * linearOffset);
  }
  // double getHeading()
  // {

  //   bmm150_mag_data value;
  //   bmm.read_mag_data();
  //   value.x = bmm.raw_mag_data.raw_datax;
  //   value.y = bmm.raw_mag_data.raw_datay;
  //   //  value.z = bmm.raw_mag_data.raw_dataz;
  //   xyHeading = atan2(value.x, value.y);
  //   //  float zxHeading = atan2(value.z, value.x);
  //   heading = xyHeading;
  //   if (heading < 0)
  //     heading += 2 * PI;
  //   if (heading > 2 * PI)
  //     heading -= 2 * PI;
  //   prevReading = headingDegrees;
  //   headingDegrees = heading * 180 / M_PI;
  //   diff = headingDegrees - prevReading;
  //   if (abs(diff) > 270)
  //   {
  //     if (diff > 0)
  //     {
  //       offset -= 360;
  //     }
  //     else
  //     {
  //       offset += 360;
  //     }
  //   }
  //   return headingDegrees + offset - initialAngle;
  // }
  // double calculateFr(bool filter = true)
  // {
  //   double reading = getHeading();
  //   while (filter && abs(prevRfR - reading) >filterR)
  //   {
  //     Serial.print("ERRRRRRROOOR**********"+String(prevRfR));

  //   prevRfR = reading;
  //     reading = getHeading();

  //     Serial.println("New*********"+String(reading));
  //   }
  //   return reading;
  // }
  // void calibrate()
  // {
  //   //Serial.print(String("Inputbmm")+heading);

  //   for (int i = 0; i < 500; i++)
  //   {

  //     if(i%0 == 0){
  //       Serial.println(i);
  //     }
  //     initialAngle += (calculateFr(false) + initialAngle);
  //   }
  //   initialAngle /= 500;
  //   initial = false;
  //   Serial.println("Initial Angle Is " + String(initialAngle));
  //   prevRfR = 0;
  // }
  double calculateFr()
  {
    mpu6050.update();

    return (mpu6050.getAngleZ() * (-1));
  }
  void calculateDerivedFx()
  {
    dFx = fx + xOutput;
  }

  void calculateDerivedFy()
  {

    dFy = fy + yOutput;
  }

  void calculateDerivedFr()
  {
    dFr = fr + rOutput;
  }
  void compute()
  {
    if (millis() - prevTime > interval)
    {
      count += 1;
      
        rFr = calculateFr();
        rFx = calculateFx();
        rFy = calculateFy();
      

      // rFr = calculateFr();
      // rFx = calculateFx();
      // rFy = calculateFy();
      //Serial.println("Compute");
      // Serial.println(rFr);
      xInput = (rFx - count * fx);
      yInput = (rFy - count * fy);
      rInput = (rFr -fr);
      // Serial.println(String("x i/p") + xInput + String("yi/p: ") + yInput + String("ri/p: ") + rInput);
      // Serial.println();
      fxPID->Compute();
      fyPID->Compute();
      frPID->Compute();
      // Serial.println(String("xo/p:")+xOutput+String("yo/p: ")+yOutput+String("ro/p: ")+rOutput);
      calculateDerivedFx();
      calculateDerivedFy();
      calculateDerivedFr();

      applyRobotSpeeds(dFx, dFy, dFr);
      prevTime = millis();
    }
  }
};