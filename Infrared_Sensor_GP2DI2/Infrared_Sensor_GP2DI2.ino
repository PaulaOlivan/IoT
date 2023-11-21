//Not tested due to breakage of the input output cable
char GP2D12;
char a, b;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int val;
  GP2D12 = read_gp2d12_range(0);
  a = GP2D12 / 10;
  b = GP2D12;
  val = a * 10 + b;
  if (val > 10 && val < 80) {
    Serial.print("Distancia: ");
    Serial.print(a, DEC);
    Serial.println(" cm");
  } else {
    Serial.println("Fuera de rango");
  }
  delay(50);
}

float read_gp2d12_range(byte pin) {
  int tmp;
  tmp = analogRead(pin);
  if (tmp < 3) return -1;
  return (6787.0 / ((float)tmp - 3.0)) - 4.0;
}
