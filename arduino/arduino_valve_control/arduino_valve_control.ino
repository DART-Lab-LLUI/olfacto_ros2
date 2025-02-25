const int valvePins[] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39}; // Using pins 22-39

void setup() {
  // Setup serial communication
  Serial.begin(9600);
  
  // Setup each pin as an output and ensure it starts HIGH (relays off)
  for (int i = 0; i < 18; i++) {
    pinMode(valvePins[i], OUTPUT);
    digitalWrite(valvePins[i], HIGH);  // Ensure all valves start closed
  }
}

void loop() {
  // Check if there is any incoming serial data
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read until newline character

    if (command.startsWith("VALVE_")) {
      int underscore1 = command.indexOf('_', 6);
      int valveNumber = command.substring(6, underscore1).toInt();
      String state = command.substring(underscore1 + 1);
      
      if (valveNumber >= 1 && valveNumber <= 18) {
        int valveIndex = valveNumber - 1; // Convert to 0-based index
        
        if (state == "ON") {
          digitalWrite(valvePins[valveIndex], LOW); // Turn ON valve
        } else if (state == "OFF") {
          digitalWrite(valvePins[valveIndex], HIGH); // Turn OFF valve
        }
      }
    }
  }
}
