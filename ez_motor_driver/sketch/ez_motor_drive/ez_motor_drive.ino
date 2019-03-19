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
const byte NUM_PARAMS = 3;

Direction readDirection(String direction) {
    if(direction.equalsIgnoreCase("STOP")) {
      return STOP;
    } else if(direction.equalsIgnoreCase("FORWARD")) {
      return FORWARD;
    } else if(direction.equalsIgnoreCase("BACKWARD")) {
      return BACKWARD;
    } else {
      return STOP;
    }
}

Motor* readMotor(String motorId) {
    if(motorId.equalsIgnoreCase("Right")) {
      return rightMotor;
    } else {
      return leftMotor;
    }
}

byte readSpeed(String rawSpeed) {
  return rawSpeed.toInt();
}

void serialEvent() {
    String message = Serial.readStringUntil('\n');
    int pos = 0;
    byte parameterPos = 0;
    String token;
    String commandParams[NUM_PARAMS];
    while (parameterPos <= NUM_PARAMS && (pos = message.indexOf(' ')) != -1 || message.length() > 0) {
        if(pos != -1) {
          commandParams[parameterPos] = message.substring(0, pos);
          message.remove(0, pos + 1);
        } else {
          commandParams[parameterPos] = message;
          message = "";
        }
        parameterPos++;
    }

    if(parameterPos != NUM_PARAMS) {
      Serial.println("Invalid command: Use 'motor direction speed'");
    } else {
      Direction direction;
      Motor *motor;
      byte speed;
      for(byte i = 0; i < NUM_PARAMS; i++) {
        String param = commandParams[i];
        switch(i) {
          case 0:
            motor = readMotor(param);
            break;
          case 1:
            direction = readDirection(param);
            break;
          case 2:
            speed = readSpeed(param);
            break;
        }
      }
      Serial.println("Updating motors.");
      motor->setPowerRatio(speed, direction);     
    }
    lastCommandTime = millis();
}

void loop() {
  digitalWrite(STATUSLED, ((millis() / 1000 % 2) == 0 ? HIGH : LOW));

  if(millis() - lastCommandTime > 1500) {
    if(lastIdle != lastCommandTime) {
      if(Serial) {
        Serial.println("Idling motors.");
      }
      
      leftMotor->stop();
      rightMotor->stop();
      lastIdle = lastCommandTime;
    }
  }
}

  // TODO implement auto off if no recent command has been given
// TODO set the PWM clock speed for 20KHz
