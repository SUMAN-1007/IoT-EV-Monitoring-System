#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C LCD (change address as needed)

// ----- Pin Configuration -----
const int cell1Pin   = A0;    // Battery Cell 1 voltage input
const int cell2Pin   = A1;    // Battery Cell 2 voltage input
const int tempPin    = A2;    // Temperature sensor input (TMP36/LM35)
const int redLED     = 9;     // Fault indicator LED
const int greenLED   = 8;     // Normal status LED
const int relayPin   = 4;     // EV Relay output (simulate EV power)
const int buttonPin  = 7;     // Push-button for toggling EV state
const int buzzerPin  = 10;    // Buzzer output pin

// ----- EV State and Debounce Variables -----
bool evOn = false;              
int buttonState = LOW;          // Debounced button state
int lastButtonReading = LOW;    // Last raw button reading
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// ----- Slide Mechanism Variables -----
unsigned long lastSlideChangeTime = 0;
const unsigned long slideInterval = 2000; // Change slide every 2 seconds
int currentSlide = 0; // 0: EV Status, 1: Temperature, 2: Battery Cell 1, 3: Battery Cell 2

// ----- Sensor Thresholds (Safety Ranges) -----
const float TEMP_MAX = 50.0;           // Temperature over this is unsafe
const float TEMP_MIN = 10.0;           // Temperature under this is unsafe
const float CELL_LOW_THRESHOLD = 3.0;  // Voltage below this is unsafe
const float CELL_HIGH_THRESHOLD = 4.2; // Voltage above this is unsafe

// ----- Moving Average Filter Parameters -----
const int SAMPLE_SIZE = 10;   // Number of samples to average

// --- Filter Arrays and Variables for Battery Cell 1 ---
float cell1Buffer[SAMPLE_SIZE];
int cell1Index = 0;
float cell1Sum = 0.0;

// --- Filter Arrays and Variables for Battery Cell 2 ---
float cell2Buffer[SAMPLE_SIZE];
int cell2Index = 0;
float cell2Sum = 0.0;

// --- Filter Arrays and Variables for Temperature ---
float tempBuffer[SAMPLE_SIZE];
int tempIndex = 0;
float tempSum = 0.0;

// ----- Auto Shutdown Flag -----
// When a critical fault occurs, this flag is set so that the system remains in shutdown mode.
bool faultShutdown = false;

void setup() {
  lcd.init();
  lcd.backlight();
  
  // Set pin modes for outputs and inputs.
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(buttonPin, INPUT);      // Using external pull-down resistor wiring
  pinMode(buzzerPin, OUTPUT);
  
  // Initialize outputs off.
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(relayPin, LOW);
  digitalWrite(buzzerPin, LOW);
  
  Serial.begin(9600);
  
  // Initialize filter buffers to zeros.
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    cell1Buffer[i] = 0.0;
    cell2Buffer[i] = 0.0;
    tempBuffer[i] = 0.0;
  }
  cell1Index = 0; cell1Sum = 0.0;
  cell2Index = 0; cell2Sum = 0.0;
  tempIndex = 0;  tempSum = 0.0;
  
  displayEvOff(); // Show "EV OFF" on startup.
}

// ----- Moving Average Filter Functions -----
// Update the circular buffer and return the filtered (smoothed) value.

float getFilteredCell1Voltage() {
  float newReading = analogRead(cell1Pin) * (5.0 / 1023.0);
  cell1Sum -= cell1Buffer[cell1Index];
  cell1Buffer[cell1Index] = newReading;
  cell1Sum += newReading;
  cell1Index = (cell1Index + 1) % SAMPLE_SIZE;
  return cell1Sum / SAMPLE_SIZE;
}

float getFilteredCell2Voltage() {
  float newReading = analogRead(cell2Pin) * (5.0 / 1023.0);
  cell2Sum -= cell2Buffer[cell2Index];
  cell2Buffer[cell2Index] = newReading;
  cell2Sum += newReading;
  cell2Index = (cell2Index + 1) % SAMPLE_SIZE;
  return cell2Sum / SAMPLE_SIZE;
}

float getFilteredTemperature() {
  float newReading = analogRead(tempPin) * (5.0 / 1023.0) * 100.0;
  tempSum -= tempBuffer[tempIndex];
  tempBuffer[tempIndex] = newReading;
  tempSum += newReading;
  tempIndex = (tempIndex + 1) % SAMPLE_SIZE;
  return tempSum / SAMPLE_SIZE;
}

void loop() {
  // ----- Button Handling (Debounce) to Toggle EV State -----
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {  // On button press, toggle EV state.
        evOn = !evOn;
        Serial.print("Button pressed, EV now: ");
        Serial.println(evOn ? "ON" : "OFF");
        
        // When manually turning off, clear fault flag.
        if (!evOn) {
          faultShutdown = false;
          displayEvOff();
          digitalWrite(relayPin, LOW);
          digitalWrite(redLED, LOW);
          digitalWrite(greenLED, LOW);
          digitalWrite(buzzerPin, LOW);
        }
        
        // Wait for button release.
        while (digitalRead(buttonPin) == HIGH) {
          delay(10);
        }
      }
    }
  }
  lastButtonReading = reading;
  
  // ----- If EV is OFF (manually or by auto shutdown), maintain display and exit loop -----
  if (!evOn) {
    if (faultShutdown) {
      // If auto shutdown occurred, show fault message.
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("FAULT - EV OFF");
      // The fault condition detail will already be shown (see below).
    } else {
      displayEvOff();
    }
    delay(200);
    return;
  }
  
  // ----- EV is ON: Activate Relay -----
  digitalWrite(relayPin, HIGH);
  
  // ----- Get Filtered Sensor Readings -----
  float cell1Voltage = getFilteredCell1Voltage();
  float cell2Voltage = getFilteredCell2Voltage();
  float temperature  = getFilteredTemperature();
  
  // ----- Determine Status Text per Sensor -----
  String tempStatus  = (temperature > TEMP_MAX) ? "HIGH" : (temperature < TEMP_MIN) ? "LOW" : "NORM";
  String cell1Status = (cell1Voltage < CELL_LOW_THRESHOLD) ? "LOW" : (cell1Voltage > CELL_HIGH_THRESHOLD) ? "HIGH" : "NORM";
  String cell2Status = (cell2Voltage < CELL_LOW_THRESHOLD) ? "LOW" : (cell2Voltage > CELL_HIGH_THRESHOLD) ? "HIGH" : "NORM";
  
  // ----- Fault Detection & Auto Shutdown with Fault Details -----
  bool fault = (temperature > TEMP_MAX || temperature < TEMP_MIN ||
                cell1Voltage < CELL_LOW_THRESHOLD || cell1Voltage > CELL_HIGH_THRESHOLD ||
                cell2Voltage < CELL_LOW_THRESHOLD || cell2Voltage > CELL_HIGH_THRESHOLD);
  
  if (fault) {
    // Determine which sensor is at fault (check in order of priority)
    String faultCondition = "";
    if (temperature > TEMP_MAX)
      faultCondition = "Temp High";
    else if (temperature < TEMP_MIN)
      faultCondition = "Temp Low";
    else if (cell1Voltage < CELL_LOW_THRESHOLD)
      faultCondition = "C1 Low";
    else if (cell1Voltage > CELL_HIGH_THRESHOLD)
      faultCondition = "C1 High";
    else if (cell2Voltage < CELL_LOW_THRESHOLD)
      faultCondition = "C2 Low";
    else if (cell2Voltage > CELL_HIGH_THRESHOLD)
      faultCondition = "C2 High";
    
    // Auto shutdown.
    faultShutdown = true;          // Lock system in fault state.
    evOn = false;                  // Automatically shut off EV.
    digitalWrite(relayPin, LOW);   // Deactivate relay.
    digitalWrite(buzzerPin, HIGH); // Activate buzzer alert.
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
    
    // Display fault message and condition on LCD.
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FAULT - EV OFF");
    lcd.setCursor(0, 1);
    lcd.print(faultCondition);
    
    Serial.print("FAULT - EV SHUTDOWN: ");
    Serial.println(faultCondition);
    delay(1000);  // Hold fault message before re-looping.
    return;
  } else {
    // Normal operation: clear buzzer and set LED status.
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, HIGH);
    digitalWrite(buzzerPin, LOW);
  }
  
  // ----- LCD Slide Update (Normal Operation) -----
  unsigned long currentTime = millis();
  if (currentTime - lastSlideChangeTime >= slideInterval) {
    currentSlide = (currentSlide + 1) % 4; // Cycle slides 0 to 3.
    lastSlideChangeTime = currentTime;
    lcd.clear();
  }
  
  switch (currentSlide) {
    case 0:  // Slide: EV Status
      lcd.setCursor(0, 0);
      lcd.print("EV STATUS:");
      lcd.setCursor(0, 1);
      lcd.print("ON");
      break;
    case 1:  // Slide: Temperature
      lcd.setCursor(0, 0);
      lcd.print("Temp:");
      lcd.print(temperature, 1);
      lcd.print((char)223); // Degree symbol.
      lcd.print("C");
      lcd.setCursor(0, 1);
      lcd.print(tempStatus);
      break;
    case 2:  // Slide: Battery Cell 1
      lcd.setCursor(0, 0);
      lcd.print("C1:");
      lcd.print(cell1Voltage, 2);
      lcd.print("V");
      lcd.setCursor(0, 1);
      lcd.print(cell1Status);
      break;
    case 3:  // Slide: Battery Cell 2
      lcd.setCursor(0, 0);
      lcd.print("C2:");
      lcd.print(cell2Voltage, 2);
      lcd.print("V");
      lcd.setCursor(0, 1);
      lcd.print(cell2Status);
      break;
  }
  
  // ----- Serial Monitor Logging for Debugging -----
  Serial.print("EV: ON, Temp: ");
  Serial.print(temperature, 1);
  Serial.print("C (");
  Serial.print(tempStatus);
  Serial.print("), C1: ");
  Serial.print(cell1Voltage, 2);
  Serial.print("V (");
  Serial.print(cell1Status);
  Serial.print("), C2: ");
  Serial.print(cell2Voltage, 2);
  Serial.print("V (");
  Serial.print(cell2Status);
  Serial.println(")");
  
  delay(100);
}

// ----- Utility Function to Display "EV OFF" on LCD -----
void displayEvOff() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EV OFF");
  lcd.setCursor(0, 1);
  lcd.print("Press Btn");
  delay(500);
}
