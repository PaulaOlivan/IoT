const int ledPin = A5; // A5 works like a digital pin

void setup() {
  pinMode(ledPin, OUTPUT); // Configurar el pin como salida
}

void loop() {
  digitalWrite(ledPin, HIGH); // Encender el LED
  delay(1000); // Esperar un segundo
  digitalWrite(ledPin, LOW); // Apagar el LED
  delay(1000); // Esperar un segundo
}
