#define motorStartLed 12
#define motorCW 11
#define motorCCW 10
#define EncA 2
#define EncB 4


// ===== SERIAL COMMUNICATION =====
#define BUFF_LEN 6
uint8_t buff[BUFF_LEN];


// ===== MOTOR =====
int isStart = 0;
float setSpeed = 0;


// ===== PID =====
class PID {
  float Kp = 1;
  float Ki = 0.005;
  float Kd = 0.5;
  float error_prev = 0;
  float error_sum = 0;
  unsigned long time_prev = 0;

 public:
  void setGain(float Kp, float Ki, float Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
  }
  void setKp(float Kp) {
    this->Kp = Kp;
  }
  void setKi(float Ki) {
    this->Ki = Ki;
  }
  void setKd(float Kd) {
    this->Kd = Kd;
  }
  float pid_control(float error) {
    float time_curr = millis();
    float dt = time_curr - this->time_prev;
    this->time_prev = time_curr;
    
    float P = Kp * error;
    float I = Ki * this->error_sum * dt;
    float D = Kd * (error - this->error_prev) / dt;

    this->error_prev = error;
    this->error_sum += error;

    return P + I + D;
  }
}pid;


// ===== ENCODER =====
volatile long int encoder_pos = 0;
long int prev_encoder_pos = 0;
unsigned long curr_time;
unsigned long prev_time = 0;
float motorSpeed = 0;

void readEncoder() {
  if (digitalRead(EncA) != digitalRead(EncB)) {
    encoder_pos++;  //CW
  } 
  else {
    encoder_pos--;  //CCW
  }
  
  curr_time = millis();

  motorSpeed = (((float)encoder_pos - (float)prev_encoder_pos) * 60.0 * 1000000.0 / (24 * (curr_time - prev_time))) / 1000;

  prev_encoder_pos = encoder_pos;
  prev_time = curr_time;
}


// FLOAT CONVERTER CLASS
class ConverterFloat {
 public:
  union Conveter {
    float floatValue;
    uint8_t byteValue[4];  
  }conv_val;
  
  float toFloat32(uint8_t *buff, uint8_t offsetValue)   {
    conv_val.byteValue[0] = buff[0 + offsetValue];
    conv_val.byteValue[1] = buff[1 + offsetValue];
    conv_val.byteValue[2] = buff[2 + offsetValue];
    conv_val.byteValue[3] = buff[3 + offsetValue];
  
    return conv_val.floatValue;
  }
  
  void putFloatByteArray(uint8_t *buff, float FloatValue, uint8_t offsetValue) {
    conv_val.floatValue = FloatValue;
    buff[0 + offsetValue] = conv_val.byteValue[0];
    buff[1 + offsetValue] = conv_val.byteValue[1];
    buff[2 + offsetValue] = conv_val.byteValue[2];
    buff[3 + offsetValue] = conv_val.byteValue[3];
  }
}float_conv;


void getSerialData() {
  if (Serial.available()) {
    buff[0] = Serial.read();  // reading data length (byte)
    delay(5);

    for (uint8_t i = 1; i < buff[0]; ++i) {
      buff[i] = Serial.read();
      delay(5);
    }

    if (buff[1] == 1) {  
      float fval = float_conv.toFloat32(buff, 2);
      pid.setKp(fval);

      float_conv.putFloatByteArray(buff, fval, 2);
      Serial.write(buff, 6);
    }
    else if (buff[1] == 2) {
      float fval = float_conv.toFloat32(buff, 2);
      pid.setKi(fval);

      float_conv.putFloatByteArray(buff, fval, 2);
      Serial.write(buff, 6);      
    }
    else if (buff[1] == 3) {
      float fval = float_conv.toFloat32(buff, 2);
      pid.setKd(fval);

      float_conv.putFloatByteArray(buff, fval, 2);
      Serial.write(buff, 6);      
    }
    else if (buff[1] == 4) {
      float fval = float_conv.toFloat32(buff, 2);
      setSpeed = fval;
  
      float_conv.putFloatByteArray(buff, fval, 2);
      Serial.write(buff, 6);      
    }
    else if (buff[1] == 5) {
      startStopMotor();    
    }
    else {
      printf("Unknown process!");
    }
  }
}


void startStopMotor() {
  isStart = !isStart;
}


void setup() 
{
  pinMode(motorCW, OUTPUT);
  pinMode(motorCCW, OUTPUT);
  pinMode(EncA, INPUT);             
  pinMode(EncB, INPUT);
  pinMode(motorStartLed, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(EncA), readEncoder, RISING);
    
  Serial.begin (9600);
  Serial.println("start");           
}


void loop() {
  if (Serial.available()) {
    getSerialData();
  }

  float error = setSpeed - motorSpeed;
  float controlSpeed = pid.pid_control(error);
  delay(1);

  if (controlSpeed > 166) { controlSpeed=165; }
  if (controlSpeed < 0) { controlSpeed=0; }

  float motorValue = 0;

  if (isStart) {   
    motorValue = map(controlSpeed, 0, 166, 0, 255);
    digitalWrite(motorStartLed, HIGH);
  }
  else {
    motorValue = 0;
    digitalWrite(motorStartLed, LOW);
  }

  analogWrite(motorCW, motorValue);
  digitalWrite(motorCCW, LOW);
}
