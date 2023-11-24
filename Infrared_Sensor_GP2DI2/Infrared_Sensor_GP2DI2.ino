/*SHARP GP2Y0A21YK0F IR distance sensor with Arduino and SharpIR library example code. More info: https://www.makerguides.com */

// Include the library:
//#include SharpIR-master

// Wait a minute until take the value of the if sensor, we will take 5 values and compute the mean in the variable if_reference
// If this value change up or down 50 the alarm will detected intrusion

float GP2D12; //Used to be a char
float a,b; // Used to be a char
byte if_pin = A0;
float wall;
bool first = true;
int range = 10;

void setup() {
  Serial.begin(9600); //
}

void loop(){

  if (first){
    delay (10000);
    int mean;
    mean = if_mean();
    Serial.print("Mean value: ");
    Serial.println(mean);

    a = mean / 10;
    b = mean % 10;
    wall = a*10 + b;

    if(wall>10 && wall<80){
      Serial.print(a,DEC); 
      Serial.print(b,DEC);
      Serial.println("cm"); 
    }
  }

  int val;
  GP2D12 = read_gp2d12_range(if_pin);
  Serial.print("Value: ");
  Serial.println(GP2D12);
  a=GP2D12/10;
  b=GP2D12%10;
  val=a*10+b;

  if (val>10&&val<80){
    Serial.print(a,DEC);//
    Serial.print(b,DEC);//
    Serial.println("cm");//
    if (val > (wall+range) || val < (wall+range)){
      Serial.println ("Intruder detected");
    } 
  }

  else{
    Serial.println("Outside range");//
  }

  delay(3000);
}

float if_mean(){
  int sum;
  for (i=0; i<5; i++){
    sum = sum + read_gp2d12_range(if_pin);
  }
  delay(1000); //Wait 1 sec before next reading
  return (sum/5);
}


float read_gp2d12_range(byte pin){
  int tmp;
  tmp = analogRead(pin);
  Serial.print("tmp value: ");
  Serial.println(tmp);
  int res;
  res = (6787.0 /((float)tmp - 3.0)) - 4.0;
  Serial.print("Result value: ");
  Serial.println(res);
  if (tmp < 3)return -1;
  return res;
}
