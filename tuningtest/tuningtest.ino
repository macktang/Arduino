float Kp;
float Kd;
//float Ki;
float q = 0.1; //value to increment Kp and Kd by

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("hello");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if(c =='r'){readTuning();}

    if (c == 'm'){Serial.println("motor running");}
    
    if (c == 't'){
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
        if(cmd=='+'){Kp+=q; Serial.print("Kp increased to "); Serial.println(Kp,8);}
        if(cmd=='-'){Kp-=q; Serial.print("Kp decreased to "); Serial.println(Kp,8);}
        break;
      case 'd':
        if(cmd=='+'){Kd+=q; Serial.print("Kd increased to "); Serial.println(Kd,8);}
        if(cmd=='-'){Kd-=q; Serial.print("Kd decreased to "); Serial.println(Kd,8);}
        break;
    }
  }
}

void writeRoutine(){  
  if(Serial.available()){
    Serial.print("pid parameter writing mode active...");                
    char param = Serial.read();
    String s1 = Serial.readString();

    switch(param){
      case 'p':
        Kp = s1.toFloat();
        Serial.print("Kp set to: "); Serial.println(Kp,8);
        break;
      case 'd':
        Kd = s1.toFloat();
        Serial.print("Kd set to: "); Serial.println(Kd,8);
        break;
      case 'q':
        q = s1.toFloat();
        Serial.print("q (increment value) set to: "); Serial.println(q,8);
        break;
    }
  }
}

void readTuning(){
  Serial.print("Kp:");    Serial.print(Kp,8);
  Serial.print("   q:"); Serial.print(q,8);
  Serial.print("   Kd:"); Serial.println(Kd,8);
}
