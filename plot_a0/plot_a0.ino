const int analogPin = A0;               // Analog input pin
const unsigned long SAMPLE_INTERVAL_MS = 50;  // Sampling interval in milliseconds

void setup() {
  Serial.begin(9600);                   // Start serial communication
  pinMode(3, OUTPUT); 
  pinMode(2, OUTPUT); 
  digitalWrite(3, HIGH);
  digitalWrite(2, LOW);
}

void loop() {
  static unsigned long lastSampleTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = currentTime;

    int rawValue = analogRead(analogPin);           // Read analog input
    float voltage = rawValue * (5.0 / 1023.0);       // Convert to voltage
    Serial.println(voltage);                        // Print voltage to Serial Plotter
  }
}
