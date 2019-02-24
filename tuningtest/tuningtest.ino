float Kp;
float Kd;
float Ki;

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
  // put your main code here, to run repeatedly:
//  getTuning();

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
      }
    }
}

void readTuning()
{
//  Serial.println();
  Serial.print("Kp:");       Serial.print(Kp,8);
  Serial.print("   Ki:");       Serial.print(Ki,8);
  Serial.print("   Kd:");       Serial.println(Kd,8);
//  delay(2000);
//  Serial.println("motor off");
  
}

//int getTuning() 
//{
//  if(!Serial.available())      return 0;
//  delay(10);                  
//  char param = Serial.read();                            // get parameter byte
//  if(!Serial.available())    return 0;
//  char cmd = Serial.read();                              // get command byte
//  Serial.flush();
//  switch (param) 
//  {
//    case 'p':
//      if(cmd=='+')  Kp+=1;
//      if(cmd=='-')    Kp-=1;
//      break;
//    case 'i':
//      if(cmd=='+')    Ki+=0.5;
//      if(cmd=='-')    Ki-=0.5;
//      break;
//    case 'd':
//      if(cmd=='+')    Kd+=2;
//      if(cmd=='-')    Kd-=1;
//      break;
//    default: 
//      Serial.print("?");          Serial.print(param);
//      Serial.print("?");          Serial.println(cmd);
//    }
//
//    readTuning();
//  
//  }
