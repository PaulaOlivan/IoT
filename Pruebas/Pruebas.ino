const int pinBuzzer = 9; // Pin para el buzzer

void setup() {
  pinMode(pinBuzzer, OUTPUT);
}

void loop() {
  // Tono bajo por 1 segundo
  tone(pinBuzzer, 1, 10);
  delay(1000);
  noTone(pinBuzzer);

  // Espera 5 segundos con el buzzer apagado
  delay(5000);
}