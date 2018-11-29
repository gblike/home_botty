
int STATUSLED = 13;

enum Direction { STOP = 0, FORWARD = 1, BACKWARD = 2};

class HBridge { 
  private:
    int a;
    int b;
  
  public:
  
  HBridge(int pin1, int pin2) : a(pin1), b(pin2) {
    pinMode(a, OUTPUT);
    pinMode(b, OUTPUT);
  }

  void changeDirection(Direction direction) {
      switch(direction) {
      case STOP:
        stop();
        break;
      case FORWARD:
        forward();
        break;
      case BACKWARD:
        backward();
        break;
      }
  }
  
  void stop() {
    Serial.println("H-Bridge stop");
    digitalWrite(a, LOW);
    digitalWrite(b, LOW);
  }
  
  void forward() {
    Serial.println("H-Bridge forward");
    digitalWrite(a, HIGH);
    digitalWrite(b, LOW);
  }
  
  void backward() {
    Serial.println("H-Bridge backward");
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
  }
};

class Motor {
  private:
    int output;
    HBridge controller;
    unsigned char dutyCycle;
  
  public:
    Motor(int outputPin, int bridgeInputA, int bridgeInputB) : output(outputPin), controller(HBridge(bridgeInputA, bridgeInputB)), dutyCycle(0){
      pinMode(output, OUTPUT);
    }

    void stop(){
      Serial.print("Motor stop: Pin #");
      Serial.print(output);
      setDutyCycle(0);
      controller.changeDirection(STOP);
    }
    
    void setPowerRatio(unsigned char ratio, Direction direction) {
      Serial.print("Changing direction: Pin #");
      Serial.print(output);
      Serial.print(" Speed ");
      Serial.print(ratio);
      Serial.print(" Direction ");
      Serial.println(direction);
      
      setDutyCycle(ratio);
      controller.changeDirection(direction);
    }
    
    float getPowerRatio() {
      return getDutyCycle();
    }
    
    void setDutyCycle(unsigned char cycle) {
      dutyCycle = cycle;
      analogWrite(output, dutyCycle);
    }

    int getDutyCycle() {
      return dutyCycle;
    }   
};


Motor *leftMotor;
Motor *rightMotor;

void setup() {
  pinMode(STATUSLED, OUTPUT);

  Serial.begin(9600);

  rightMotor  = new Motor(7, 9, 10);
  leftMotor = new Motor(8, 11, 12);
  Serial.println("Initialized.");
}

unsigned long lastCommandTime = 1;
unsigned long lastIdle = 0;

void loop() {
  digitalWrite(STATUSLED, ((millis() / 1000 % 2) == 0 ? HIGH : LOW));

  if(Serial && Serial.available() >= 3) {
      char motorId = Serial.read();
      
      Direction direction;
       switch(Serial.read()) {
        case 'S':
          direction = STOP;
          break;
        case 'F':
          direction = FORWARD;
          break;
        case 'B':
          direction = BACKWARD;
          break;
        default:
          direction = STOP;
      }
      
      unsigned char speed = Serial.read();
      Motor *motor; 
      if(motorId == 'R') {
        motor = rightMotor;
      } else {
        motor = leftMotor;
      }
      Serial.println("Updating motors.");
      motor->setPowerRatio(speed, direction);
      lastCommandTime = millis();
  } else if(millis() - lastCommandTime > 1500) {
    if(lastIdle != lastCommandTime) {
      Serial.println("Idling motors.");
      leftMotor->stop();
      rightMotor->stop();
      lastIdle = lastCommandTime;
    }
  }
}

  // TODO implement auto off if no recent command has been given
// TODO set the PWM clock speed for 20KHz

