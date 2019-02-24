// This program demonstrates capability to
// quickly tune Kp, Ki, and Kd constants
// in PID control, without need
// to recompile code in between.

// Instructions to run
// 1) Open in Arduino IDE
// 2) Upload code, open Serial Monitor

// Example usages
// PURPOSE                COMMAND OVER SERIAL
// set Kp = 0.0156        wkp.015
// set Kd = -6.102        wkd-6.102
// increment Kp += qq     kp+
// decrement Kp -= qq     kd-
// set qq = 0.005         wqq.005
// print Kp, Kd, qq       r
// "run motors"           m

float Kp;
float Kd;
//float Ki;
float qq = 0.1; //value to increment Kp and Kd by

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("setup complete");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if(c =='r'){readTuning();}

    if (c == 'm'){Serial.println("motor running");}
    
    if (c == 'k'){
      Serial.flush();
      delay(10);
      tuningRoutine();
    }

    if (c == 'w'){
      Serial.flush();
      delay(10);
      writeRoutine();
    }
    
  }
}

void tuningRoutine(){  
  if(Serial.available()){
    Serial.print("tuning mode active...");                
    char param = Serial.read();
    char cmd = Serial.read();
    switch(param){
      case 'p':
        if(cmd=='+'){Kp+=qq; Serial.print("Kp increased to "); Serial.println(Kp,8);}
        if(cmd=='-'){Kp-=qq; Serial.print("Kp decreased to "); Serial.println(Kp,8);}
        break;
      case 'd':
        if(cmd=='+'){Kd+=qq; Serial.print("Kd increased to "); Serial.println(Kd,8);}
        if(cmd=='-'){Kd-=qq; Serial.print("Kd decreased to "); Serial.println(Kd,8);}
        break;
    }
  }
}

void writeRoutine(){  
  if(Serial.available()){
    Serial.print("pid parameter writing mode active...");
    char param1 = Serial.read();
    char param2 = Serial.read();
    String s1 = Serial.readString();

    if(param1=='k'&&param2=='p'){Kp = s1.toFloat(); Serial.println(Kp,8);}
    if(param1=='k'&&param2=='d'){Kd = s1.toFloat(); Serial.println(Kd,8);}
    if(param1=='q'&&param2=='q'){qq = s1.toFloat(); Serial.println(qq,8);}
  }
}

void readTuning(){
  Serial.print("Kp:");    Serial.print(Kp,8);
  Serial.print("   Kd:"); Serial.print(Kd,8);
  Serial.print("   qq:"); Serial.println(qq,8);
}
