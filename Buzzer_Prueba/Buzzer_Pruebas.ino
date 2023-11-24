// Definición de pines
const int buzzerPin = 9; // Pin al que está conectado el buzzer
int intruso = 3; // Variable para controlar los tonos del buzzer

void setup() {
  pinMode(buzzerPin, OUTPUT); // Configurar el pin del buzzer como salida
}

void loop() {
  if (intruso != 0) {
    // Si intruso es diferente de 0, cambiará el tono del buzzer según su valor
    if (intruso == 1) {
      Serial.println("Intruso detectado con nivel 1");
      tone(buzzerPin, 500); // Frecuencia para el tono más bajo
    } 
    else if (intruso == 2) {
      Serial.println("Intruso detectado con nivel 2");
      tone(buzzerPin, 1000); // Frecuencia para el tono medio
    } 
    else if (intruso == 3) {
      Serial.println("Intruso detectado con nivel 3");
      tone(buzzerPin, 2000); // Frecuencia para el tono más alto
    }
    delay(1000); // Mantener el tono durante 1 segundo
    noTone(buzzerPin); // Detener el tono
    intruso = 0; // Restablecer la variable intruso a 0
  }
  Serial.println("No se detectan intrusos");
  delay(2000);
}
