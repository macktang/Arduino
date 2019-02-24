float Kp;
float Kd;
//float Ki;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("hello");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if(c =='r'){
      readTuning();
    }

    if (c == 'm'){
      Serial.println("motor running");
    }
    
    if (c == 't'){
      Serial.flush();
      delay(10);

      tuningRoutine();

    }
  }
}

void tuningRoutine()
{  
  if(Serial.available())
    {
      Serial.print("tuning mode active...");                
      char param = Serial.read();
      char cmd = Serial.read();
      switch(param){
        case 'p':
          if(cmd=='+'){Kp+=1; Serial.print("Kp increased to "); Serial.println(Kp);}
          if(cmd=='-'){Kp-=1; Serial.print("Kp decreased to "); Serial.println(Kp);}
          break;
        case 'd':
          if(cmd='+'){Kd+=1; Serial.print("Kd incresed to "); Serial.println(Kd);}
          if(cmd=='-'){Kd-=1; Serial.print("Kd decreased to "); Serial.println(Kd);}
          break;
      }
    }
}

void readTuning()
{
  Serial.print("Kp:");    Serial.print(Kp,8);
//  Serial.print("   Ki:"); Serial.print(Ki,8);
  Serial.print("   Kd:"); Serial.println(Kd,8);
}
