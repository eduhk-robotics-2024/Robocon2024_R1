#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <VescUart.h>

/** Initiate VescUart class for two VESCs */
VescUart VESC1;
VescUart VESC2;

/** Initialize the LCD with I2C address 0x27 and 20x4 dimensions */
LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  /** Setup Serial port to display data */
  Serial.begin(9600);

  /** Setup UART ports (Serial1 and Serial2 on Arduino Mega) */
  Serial1.begin(115200);
  Serial2.begin(115200);

  while (!Serial) {
    ; // Wait for Serial to be ready
  }

  /** Define which hardware serial ports to use as UART */
  VESC1.setSerialPort(&Serial1);
  VESC2.setSerialPort(&Serial2);

  /** Initialize the LCD */
  lcd.init();
  lcd.backlight();
  lcd.print("Setup complete.");
  delay(2000);
  lcd.clear();

  Serial.println("Setup complete. Enter two RPM values separated by a space to control the motors.");
}

void loop() {
  static int lastInputValue1 = 0;
  static int lastInputValue2 = 0;

  /** Check if data is available in the Serial Monitor */
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n'); // Read the input until a newline character
    int spaceIndex = inputString.indexOf(' '); // Find the index of the space character

    if (spaceIndex != -1) {
      // Separate the input into two parts
      String inputString1 = inputString.substring(0, spaceIndex);
      String inputString2 = inputString.substring(spaceIndex + 1);

      int inputValue1 = inputString1.toInt(); // Convert the first part to an integer
      int inputValue2 = inputString2.toInt(); // Convert the second part to an integer

      if ((inputValue1 != 0 || inputString1 == "0") && (inputValue2 != 0 || inputString2 == "0")) {
        // Ensure both inputs are valid
        Serial.print("Setting RPM1 to: ");
        Serial.println(inputValue1);
        Serial.print("Setting RPM2 to: ");
        Serial.println(inputValue2);

        // Update the last known values
        lastInputValue1 = inputValue1;
        lastInputValue2 = inputValue2;

        // Send RPM command to both VESCs
        VESC1.setRPM(inputValue1);
        VESC2.setRPM(inputValue2);
      } else {
        Serial.println("Invalid input. Please enter two numeric values separated by a space.");
      }
    } else {
      Serial.println("Invalid input format. Please enter two numeric values separated by a space.");
    }
  }

  // Continuously send RPM commands to keep the motors spinning
  if (lastInputValue1 != 0 || lastInputValue2 != 0) {
    VESC1.setRPM(lastInputValue1);
    VESC2.setRPM(lastInputValue2);
  }

  /** Call the function getVescValues() to acquire data from VESC1 */
  if (VESC1.getVescValues()) {
    Serial.print("VESC1 RPM: ");
    Serial.println(VESC1.data.rpm);
    Serial.print("VESC1 Input Voltage: ");
    Serial.println(VESC1.data.inpVoltage);
    Serial.print("VESC1 Amp Hours: ");
    Serial.println(VESC1.data.ampHours);
    Serial.print("VESC1 Tachometer: ");
    Serial.println(VESC1.data.tachometerAbs);

    // Display data on LCD
    lcd.setCursor(0, 0);
    lcd.print("VESC1 RPM: ");
    lcd.print(VESC1.data.rpm);
    lcd.setCursor(0, 1);
    lcd.print("VESC1 Voltage: ");
    lcd.print(VESC1.data.inpVoltage);
  } else {
    Serial.println("Failed to get data from VESC1!");
    lcd.setCursor(0, 0);
    lcd.print("VESC1 Error");
  }

  /** Call the function getVescValues() to acquire data from VESC2 */
  if (VESC2.getVescValues()) {
    Serial.print("VESC2 RPM: ");
    Serial.println(VESC2.data.rpm);
    Serial.print("VESC2 Input Voltage: ");
    Serial.println(VESC2.data.inpVoltage);
    Serial.print("VESC2 Amp Hours: ");
    Serial.println(VESC2.data.ampHours);
    Serial.print("VESC2 Tachometer: ");
    Serial.println(VESC2.data.tachometerAbs);

    // Display data on LCD
    lcd.setCursor(0, 2);
    lcd.print("VESC2 RPM: ");
    lcd.print(VESC2.data.rpm);
    lcd.setCursor(0, 3);
    lcd.print("VESC2 Voltage: ");
    lcd.print(VESC2.data.inpVoltage);
  } else {
    Serial.println("Failed to get data from VESC2!");
    lcd.setCursor(0, 2);
    lcd.print("VESC2 Error");
  }

  delay(100);
}
