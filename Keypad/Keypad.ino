const int keyVals[16] = {195, 248, 515, 642, 160, 221, 496, 629, 267, 320, 563, 682, 109, 166, 468, 605};
const char keys[16] = {'1', '2', '3', 'A', '4', '5', '6', 'B', '7', '8', '9', 'C', '*', '0', '#', 'D'};
int range = 2; // Tolerancia arriba o abajo del valor numérico
int lastButton = -1; // Último botón presionado
unsigned long lastPressTime = 0; // Último momento en el que se presionó un botón
bool buttonReleased = true; // Indica si el botón fue liberado

void setup() {
  Serial.begin(9600); // Inicializa la comunicación serial
}

void loop() {
  unsigned long currentTime = millis(); // Obtener el tiempo actual

  int keyIn = analogRead(A1); // Lee la entrada del teclado
  int detectedButton = detectButton(keyIn); // Encuentra el botón presionado

  if (detectedButton != -1) {
    if (buttonReleased) {
      Serial.print("Nuevo botón pulsado: ");
      Serial.println(keys[detectedButton]); // Muestra el botón por el monitor serial
      lastButton = detectedButton;
      lastPressTime = currentTime; // Actualiza el tiempo del último botón pulsado
      buttonReleased = false; // Deshabilita la detección hasta que se libere el botón
      delay(100); // Se agrega un pequeño tiempo de parada para evitar pulsos espurios
    }
  } 
  else if (detectedButton == -1) {
    buttonReleased = true; // Habilita la detección si no se detecta ningún botón presionado
    //Serial.println("Se ha dejado de pulsar el teclado");
  }
}

int detectButton(int inputValue) {
  for (int i = 0; i < 16; i++) {
    if (inputValue >= keyVals[i] - range && inputValue <= keyVals[i] + range) {
      return i; // Devuelve el índice del botón detectado
    }
  }
  return -1; // Si no se detecta ningún botón, devuelve -1
}

void obtenerVoltajeMedioTecla() {
  Serial.println("Obteniendo voltaje medio de cada tecla...");

  for (int i = 0; i < 100; i++) {
    int sum = 0;
    for (int j = 0; j < 100; j++) {
      sum += analogRead(A1); // Leer valor analógico
      delay(10);
    }
    float averageVoltage = (float)sum / 100.0; // Calcular media
    Serial.print("Tecla: ");
    Serial.print(i);
    Serial.print(" - Voltaje medio: ");
    Serial.println(averageVoltage);
  }
}