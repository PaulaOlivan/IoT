const int keyVals[16] = {195, 248, 515, 642, 160, 221, 496, 629, 267, 320, 563, 682, 109, 166, 468, 605};
const char keys[16] = {'1', '2', '3', 'A', '4', '5', '6', 'B', '7', '8', '9', 'C', '*', '0', '#', 'D'};
int range = 5; // Tolerancia arriba o abajo del valor numérico
int lastButton = -1; // Último botón presionado
int prevButton = -1; // Botón previo presionado

void setup() {
  Serial.begin(9600); // Inicializa la comunicación serial
  //obtenerVoltajeMedioTecla();
}

void loop() {
  int keyIn = analogRead(A1); // Lee la entrada del teclado

  int detectedButton = detectButton(keyIn); // Encuentra el botón presionado

  if (detectedButton != -1) {
    if (detectedButton != lastButton || detectedButton != prevButton) {
      Serial.print("Nuevo botón pulsado: ");
      Serial.println(keys[detectedButton]); // Muestra el botón por el monitor serial
      prevButton = lastButton;
      lastButton = detectedButton;
    }
  }
  delay(100); // Espera un breve momento antes de la siguiente lectura
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