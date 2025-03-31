// Pins for valves 1-16 (connected to pins 22-37)
const int valvePins[] = {
  22, 23, 24, 25, 26, 27, 28, 29, 
  30, 31, 32, 33, 34, 35, 36, 37,
  2, 3, 4, 5  // Valves 17-20 (new additions)
};

void setup() {
  Serial.begin(9600);
  
  // Initialize all valve pins as output and set them HIGH (valves off)
  for (int i = 0; i < 20; i++) {
    pinMode(valvePins[i], OUTPUT);
    digitalWrite(valvePins[i], HIGH);
  }
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');

    if (command.startsWith("VALVE_")) {
      int underscore1 = command.indexOf('_', 6);
      int valveNumber = command.substring(6, underscore1).toInt();
      String state = command.substring(underscore1 + 1);
      
      if (valveNumber >= 1 && valveNumber <= 20) {
        int valveIndex = valveNumber - 1;
        
        if (state == "ON") {
          digitalWrite(valvePins[valveIndex], LOW);
        } else if (state == "OFF") {
          digitalWrite(valvePins[valveIndex], HIGH);
        }
      }
    }
  }
}
