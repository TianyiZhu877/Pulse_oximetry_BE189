const int analogPin = A0;               // Analog input pin
const unsigned long SAMPLE_INTERVAL_MS = 16;  // Sampling interval in milliseconds

void setup() {
  Serial.begin(115200);                   // Start serial communication
  pinMode(7, OUTPUT); 
  pinMode(6, OUTPUT); 
  digitalWrite(7, HIGH);
  digitalWrite(6, LOW);
}

void loop() {
  static unsigned long lastSampleTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = currentTime;

    int rawValue = analogRead(analogPin);           // Read analog input
    float voltage = rawValue * (5.0 / 1023.0);       // Convert to voltage
    Serial.println(1023-rawValue);                        // Print voltage to Serial Plotter
  }
}
