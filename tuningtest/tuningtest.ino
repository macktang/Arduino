float Kp;
float Kd;
float Ki;
float K;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("hello");

}

void loop() {
  // put your main code here, to run repeatedly:
  getTuning();

}

void useTuning()
{
  Serial.println("running motor");
  Serial.println();
  Serial.print("K:");           Serial.print(K,8);
  Serial.print("   Kp:");       Serial.print(Kp,8);
  Serial.print("   Ki:");       Serial.print(Ki,8);
  Serial.print("   Kd:");       Serial.println(Kd,8);
  delay(2000);
  Serial.println("motor off");
  
}

int getTuning() 
{
  if(!Serial.available())      return 0;
  delay(10);                  
  char param = Serial.read();                            // get parameter byte
  if(!Serial.available())    return 0;
  char cmd = Serial.read();                              // get command byte
  Serial.flush();
  switch (param) 
  {
    case 'p':
      if(cmd=='+')  Kp+=1;
      if(cmd=='-')    Kp-=1;
      break;
    case 'i':
      if(cmd=='+')    Ki+=0.5;
      if(cmd=='-')    Ki-=0.5;
      break;
    case 'd':
      if(cmd=='+')    Kd+=2;
      if(cmd=='-')    Kd-=1;
      break;
    case 'k':
      if(cmd=='+')     K +=1;
      if(cmd=='-')    K -=0.5;
      break;
    default: 
      Serial.print("?");          Serial.print(param);
      Serial.print("?");          Serial.println(cmd);
    }

    useTuning();
//    
//  Serial.println();
//  Serial.print("K:");           Serial.print(K);
//  Serial.print("   Kp:");       Serial.print(Kp);
//  Serial.print("   Ki:");      Serial.print(Ki);
//  Serial.print("   Kd:");       Serial.println(Kd);
  
  }
