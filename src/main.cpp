#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define TFT_DC 2
#define TFT_CS 4
#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino resetpin)



//ESP32 DEVKIT V1:
/*
const int buttonDistancePin = 13;  // Wybór odległości
const int buttonStartStopPin = 12; // Przycisk START
const int relayDirectionPin = 14;  // Przekaźnik kierunku jazdy (przód/tył)
const int relaySpeedPin = 27;      // Przekaźnik prędkości (szybko/wolno)
const int relayControlPin = 26;    // Przekaźnik kontrolujący włączanie/wyłączanie
const int sensorPin = 25;          // Czujnik szczelinowy
const int endstopPin = 33;         // Krańcówka
const int debugPin = 5;            // aktywacja trybu debug

const int pulsePerRotation = 22;      // Impulsy na obrót
const int distancePerPulse = 38 / 22; // cm na obrót
const int slowingThreshold = 50;      // 0.5 metra (50 cm) przed celem

// PRZEKAŹNIK SYMULUJĄCY KRAŃCÓWKĘ:
const int simulationPin = 32;
*/

//ESP32 RELAY BOARD 4CHANNEL:

const int buttonDistancePin = 13;  // Wybór odległości
const int buttonStartStopPin = 12; // Przycisk START
const int relayDirectionPin = 32;  // Przekaźnik kierunku jazdy (przód/tył)
const int relaySpeedPin = 33;      // Przekaźnik prędkości (szybko/wolno)
const int relayControlPin = 25;    // Przekaźnik kontrolujący włączanie/wyłączanie
const int sensorPin = 27;          // Czujnik szczelinowy
const int endstopPin = 14;         // Krańcówka
const int debugPin = 5;            // aktywacja trybu debug

const int pulsePerRotation = 22;      // Impulsy na obrót
const int distancePerPulse = 38 / 22; // cm na obrót
const int slowingThreshold = 50;      // 0.5 metra (50 cm) przed celem

// PRZEKAŹNIK SYMULUJĄCY KRAŃCÓWKĘ:
const int simulationPin = 26;

int targetDistance = 0;
int currentPosition = 0;
bool isMoving = false;
bool isSlowingDown = false;
int selectedDistance = 5;     // Wybrany zakres odległości (5/10/15/25 m)
bool forwardDirection = true; // Zmienna do kontroli kierunku

volatile unsigned long motorTimer = 0;
volatile unsigned long pulseCheck = 0;
const int timeLimiter = 15000;

// Deklaracje funkcji
void startMotor();
void stopMotor();
bool isAtEndstop();
bool isDirectionRelayForward();
void homingMotor();
void pulseCounter(); // Zmieniona nazwa funkcji obsługującej przerwanie
void distanceDisplay();
// void distanceSelect();
void displayCenteredText(String text);
void homingSymbol();
void arrowUp();
void arrowDown();
void arrowUpBlink();
void arrowDownBlink();

bool endstopState = isAtEndstop();
bool directionRelayState = isDirectionRelayForward();

Adafruit_SSD1306 tft(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // uncomment if oled
// Adafruit_GC9A01A tft(TFT_CS, TFT_DC); //comment if oled

void setup()
{
  Serial.begin(9600);
  Serial.println("Welcome V1.0");

  pinMode(buttonDistancePin, INPUT_PULLUP);
  pinMode(buttonStartStopPin, INPUT_PULLUP);
  pinMode(relayDirectionPin, OUTPUT);
  pinMode(relaySpeedPin, OUTPUT);
  pinMode(relayControlPin, OUTPUT);
  pinMode(sensorPin, INPUT_PULLUP);
  pinMode(endstopPin, INPUT_PULLUP);
  pinMode(debugPin, INPUT_PULLUP);

  // Inicjowanie przekaźników jako wyłączonych
  digitalWrite(relayDirectionPin, HIGH);
  digitalWrite(relaySpeedPin, HIGH);
  digitalWrite(relayControlPin, HIGH);

  endstopState = isAtEndstop();
  directionRelayState = isDirectionRelayForward();

  if (!tft.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  // Display Text
  tft.clearDisplay();
  tft.setTextSize(1);
  tft.setTextColor(WHITE);
  tft.setCursor(0, 0);
  tft.println("WELCOME V1.0");
  tft.display();
  delay(2000);

  /*
    tft.begin ();
    tft.setRotation (2);
    tft.setTextSize(2);
    tft.fillScreen (0x0000);
    delay (200);
    tft.fillScreen (0xF800);
    delay (200);
    tft.fillScreen (0x07E0);
    delay (200);
    tft.fillScreen (0x001F);
    delay (200);
    tft.fillScreen (0x0000);
    delay (200);
    */

  // displayCenteredText("Welcome");
  // tft.fillScreen (0x0000);

  // debug
  Serial.println("ControlPoint(1)");

  /*Serial.println(endstopState ? "Ative" : "Not active");
  Serial.print("isMoving: ");
  Serial.println(isMoving ? "True" : "False");
  Serial.print("isSlowingDown: ");
  Serial.println(isSlowingDown ? "True" : "False");
  Serial.print("Selected distance: ");
  Serial.println(selectedDistance);
  Serial.print("Forward direction: ");
  Serial.println(forwardDirection ? "True" : "False");
  Serial.print("Relay direction: ");
  Serial.println(directionRelayState ? "Forward" : "Reverse");
  Serial.print("Relay speed: ");
  Serial.println(relaySpeedPin ? "LOW" : "HIGH");
  Serial.print("Relay control: ");
  Serial.println(relayControlPin ? "OFF" : "ON");
  Serial.println();
  */
  // end debug

  if (!endstopState)
  {
    homingMotor();
  }
  else
  {
    Serial.println("Select distance and press start...");
    // tft.clearDisplay();
    // tft.setCursor(0, 0);
    // tft.println("Select distance and press start...");
    // tft.display();
    // distanceSelect();
    //  tu musi drukować zmiena distance
    tft.clearDisplay();
    tft.display();
    distanceDisplay();
  }
}

void loop()
{
  endstopState = isAtEndstop();
  directionRelayState = isDirectionRelayForward();

  // distanceSelect();
  int distanceButtonState = digitalRead(buttonDistancePin);
  if (distanceButtonState == LOW && endstopState)
  {
    // Distance button is pressed, change the selected distance
    selectedDistance = (selectedDistance + 5) % 30;
    /*
      tft.clearDisplay();
      tft.setCursor(0, 0);
      tft.print("Selected Distance: ");
      tft.print(selectedDistance);
      tft.println("m");
      tft.println();
      tft.display();
    */
    distanceDisplay();
    delay(500); // Debounce
    endstopState = isAtEndstop();
    directionRelayState = isDirectionRelayForward();

    // debug

    Serial.println("ControlPoint(2)");
    /*
    Serial.println(endstopState ? "Ative" : "No active");
    Serial.print("isMoving: ");
    Serial.println(isMoving ? "True" : "False");
    Serial.print("isSlowingDown: ");
    Serial.println(isSlowingDown ? "True" : "False");
    Serial.print("Selected distance: ");
    Serial.println(selectedDistance);
    Serial.print("Forward direction: ");
    Serial.println(forwardDirection ? "True" : "False");
    Serial.print("Relay direction: ");
    Serial.println(directionRelayState ? "Forward" : "Reverse");
    Serial.print("Relay speed: ");
    Serial.println(relaySpeedPin ? "LOW" : "HIGH");
    Serial.print("Relay control: ");
    Serial.println(relayControlPin ? "OFF" : "ON");
    Serial.println();
    // end debug

   */
  }

  int startStopState = digitalRead(buttonStartStopPin);

  if (startStopState == LOW)
  {
    if (!isMoving)
    {
      if (selectedDistance == 0)
      {
        tft.clearDisplay();
        tft.setCursor(0, 0);
        tft.setTextSize(2);
        tft.println("Select");
        tft.println("distance");
        tft.println("first (1).");
        tft.println();
        tft.display();
        tft.setTextSize(8);
        delay(1000);
        distanceDisplay();
        // delay(500);
        return;
      }

      isMoving = true;

      // debug
      Serial.println("ControlPoint(3)");
      /*
      Serial.println(endstopState ? "Ative" : "No active");
      Serial.print("isMoving: ");
      Serial.println(isMoving ? "True" : "False");
      Serial.print("isSlowingDown: ");
      Serial.println(isSlowingDown ? "True" : "False");
      Serial.print("Selected distance: ");
      Serial.println(selectedDistance);
      Serial.print("Forward direction: ");
      Serial.println(forwardDirection ? "True" : "False");
      Serial.print("Relay direction: ");
      Serial.println(directionRelayState ? "Forward" : "Reverse");
      Serial.print("Relay speed: ");
      Serial.println(relaySpeedPin ? "LOW" : "HIGH");
      Serial.print("Relay control: ");
      Serial.println(relayControlPin ? "OFF" : "ON");
      Serial.println();
      // end debug
      */

      startMotor();
      endstopState = isAtEndstop();
      directionRelayState = isDirectionRelayForward();
    }
    else
    {
      // isMoving = false;
      stopMotor();
      // isSlowingDown = false; //chyba zbędne
      endstopState = isAtEndstop();
      directionRelayState = isDirectionRelayForward();

      // debug
      Serial.println("ControlPoint(4)");
      /*
      Serial.println(endstopState ? "Ative" : "No active");
      Serial.print("isMoving: ");
      Serial.println(isMoving ? "True" : "False");
      Serial.print("isSlowingDown: ");
      Serial.println(isSlowingDown ? "True" : "False");
      Serial.print("Selected distance: ");
      Serial.println(selectedDistance);
      Serial.print("Forward direction: ");
      Serial.println(forwardDirection ? "True" : "False");
      Serial.print("Relay direction: ");
      Serial.println(directionRelayState ? "Forward" : "Reverse");
      Serial.print("Relay speed: ");
      Serial.println(relaySpeedPin ? "LOW" : "HIGH");
      Serial.print("Relay control: ");
      Serial.println(relayControlPin ? "OFF" : "ON");
      Serial.println();
      // end debug
      */
    }
    delay(500); // Debounce
  }
}
/*
void startMotor() {
  if (isAtEndstop()) {
    Serial.println("Select a distance first2.");
    Serial.println();
    isMoving = false;
    return;
  }

  currentPosition = 0.0;
  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, RISING); // Zmieniona nazwa przerwania
  digitalWrite(relayDirectionPin, forwardDirection ? LOW : HIGH);
  float targetPosition = selectedDistance * 100.0;
  digitalWrite(relaySpeedPin, isSlowingDown ? HIGH : LOW);
  digitalWrite(relayControlPin, LOW); // Motor ON = relay control ON/LOW

  while (currentPosition < targetPosition && isMoving) {
    Serial.print("\r            ");
    Serial.print("\r");
    Serial.print("Current position: ");
    Serial.print(currentPosition);
    Serial.print(" | Remain distance: ");
    Serial.print(targetPosition - currentPosition);

    if (currentPosition >= (targetPosition - slowingThreshold * 100.0)) {
      isSlowingDown = true; // Zastosowanie isSlowingDown do przełączania prędkości
      digitalWrite(relaySpeedPin, HIGH);
      Serial.println("Motor on LOW speed");
    }
  }

  stopMotor();
  isSlowingDown = false;
  isMoving = false;
  forwardDirection = !forwardDirection;
  digitalWrite(relayDirectionPin, forwardDirection ? LOW : HIGH);
}

*/

void startMotor()
{
  if (endstopState) //&& targetDistance == 0) //<--- TO DODAĆ NA DOCELOWYM OBIEKCIE I SPRAWDZIĆ CZY BĘDZIE RUSZAĆ Z KRAŃCÓWKI
  {
    tft.clearDisplay();
    tft.setCursor(0, 0);
    tft.setTextSize(2);
    tft.println("Select");
    tft.println("distance");
    tft.println("first (2).");
    tft.println();
    tft.display();
    tft.setTextSize(8);
    delay(1000);
    distanceDisplay();
    isMoving = false;
    // delay(500);//debounce
    return;
  }

  if (forwardDirection) // && endstopState) //<----TO DODAĆ NA DOCELOWYM OBIEKCIE I SPRAWDZIĆ CZY BĘDZIE RUSZAĆ Z KRAŃCÓWKI
  {
    currentPosition = 0;

    // Attach interrupt for the sensor
    attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, RISING); // Zmieniona nazwa przerwania

    int targetPosition = selectedDistance * 100;

    digitalWrite(relayDirectionPin, forwardDirection ? LOW : HIGH);
    digitalWrite(relaySpeedPin, isSlowingDown ? HIGH : LOW); // High speed = relay speed ON/LOW; Low speed = relay speed OFF/HIGH
    digitalWrite(relayControlPin, LOW);                      // Motor ON = relay control ON/LOW
    motorTimer = millis();
    pulseCheck = currentPosition;

    while (currentPosition < targetPosition && isMoving)
    {
      arrowUpBlink();
      distanceDisplay();
      // tft.clearDisplay();
      // tft.display();
      // tft.setCursor(0, 0);
      Serial.print("\r            ");
      Serial.print("\r");
      Serial.print("Current position: ");
      Serial.print(currentPosition);
      Serial.print(" | Remain distance: ");
      Serial.print(targetPosition - currentPosition);
      // tft.display();

      // Serial.print(millis());
      Serial.print(" | ");
      Serial.print("MotorTime: ");
      Serial.print((millis() - motorTimer) / 1000);
      Serial.print("s");
      Serial.print(" | ");
      Serial.print(currentPosition - pulseCheck);
      Serial.println();

      if (/*currentPosition - pulseCheck < 1 ||*/ millis() - motorTimer > timeLimiter)
      {
        digitalWrite(relayControlPin, HIGH);
        digitalWrite(relaySpeedPin, HIGH);
        digitalWrite(relayDirectionPin, HIGH);
        tft.clearDisplay();
        tft.setCursor(5, 0);
        tft.setTextSize(8);
        tft.println("E1");
        tft.display();
        isMoving = false;
        Serial.println("E1: MOTOR STUCK OR ENCODER BROKEN!");
        Serial.print("pulsecheck: ");
        Serial.println(pulseCheck);
        Serial.print("currentPosition: ");
        Serial.println(currentPosition);
        Serial.print("IS moving: ");
        Serial.println(isMoving ? "True" : "False");
        delay(5000);
        tft.clearDisplay();
        tft.display();
        homingMotor();

        return;
      }

      if (!isSlowingDown && currentPosition >= (targetPosition - slowingThreshold))
      {
        isSlowingDown = true;
        digitalWrite(relaySpeedPin, HIGH);
        // tft.clearDisplay();
        // tft.setCursor(0, 0);
        Serial.println(" Motor on LOW speed");
        // tft.display();
      }
    }
  }
  else if (!forwardDirection) // && !endstopState)  //<----TO DODAĆ NA DOCELOWYM OBIEKCIE I SPRAWDZIĆ CZY BĘDZIE RUSZAĆ Z KRAŃCÓWKI
  {
    currentPosition = selectedDistance * 100;

    // Attach interrupt for the sensor
    attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, RISING); // Zmieniona nazwa przerwania

    int targetPosition = 0;

    digitalWrite(relayDirectionPin, forwardDirection ? LOW : HIGH);
    digitalWrite(relaySpeedPin, isSlowingDown ? HIGH : LOW); // High speed = relay speed ON/LOW; Low speed = relay speed OFF/HIGH
    digitalWrite(relayControlPin, LOW);                      // Motor ON = relay control ON/LOW

    motorTimer = millis();
    pulseCheck = currentPosition;

    while (currentPosition > targetPosition && isMoving)
    {
      arrowDownBlink();
      distanceDisplay();
      // tft.clearDisplay();
      // tft.display();
      // tft.setCursor(0, 0);
      Serial.print("\r            ");
      Serial.print("\r");
      Serial.print("Current position: ");
      Serial.print(currentPosition);
      Serial.print(" | Remain distance: ");
      Serial.print(currentPosition - targetPosition);
      // tft.display();
      // Serial.print(millis());
      Serial.print(" | ");
      Serial.print((millis() - motorTimer) / 1000);
      Serial.print("s");
      Serial.print(" | ");
      Serial.print(pulseCheck - currentPosition);
      Serial.println();

      if (/*currentPosition - pulseCheck < 1 ||*/ millis() - motorTimer > timeLimiter)
      {
        digitalWrite(relayControlPin, HIGH);
        digitalWrite(relaySpeedPin, HIGH);
        digitalWrite(relayDirectionPin, HIGH);
        tft.clearDisplay();
        tft.setCursor(5, 0);
        tft.setTextSize(8);
        tft.println("E1");
        tft.display();
        isMoving = false;
        Serial.println("E1: MOTOR STUCK OR ENCODER BROKEN!");
        Serial.print("pulsecheck: ");
        Serial.println(pulseCheck);
        Serial.print("currentPosition: ");
        Serial.println(currentPosition);
        Serial.print("IS moving: ");
        Serial.println(isMoving ? "True" : "False");
        delay(5000);
        tft.clearDisplay();
        tft.display();
        homingMotor();

        return;
      }

      if (!isSlowingDown && currentPosition <= slowingThreshold)
      {
        isSlowingDown = true;
        digitalWrite(relaySpeedPin, HIGH);
        // tft.clearDisplay();
        // tft.setCursor(0, 0);
        Serial.println(" Motor on LOW speed");
        // tft.display();
      }
    }
  }

  stopMotor();
  isSlowingDown = false;
  isMoving = false;
  forwardDirection = !forwardDirection;
  distanceDisplay();

  if (forwardDirection)
  {
    arrowUp();
  }
  else
  {
    arrowDown();
  }
  // digitalWrite(relayDirectionPin, forwardDirection ? LOW : HIGH);
}

void stopMotor()
{
  // tft.clearDisplay();
  // tft.setCursor(0, 0);
  Serial.println(" Stopping motor");
  distanceDisplay();
  // tft.display();
  digitalWrite(relayControlPin, HIGH);
  digitalWrite(relaySpeedPin, HIGH);
  digitalWrite(relayDirectionPin, HIGH);

  // distanceSelect();

  /*
  forwardDirection = !forwardDirection;
  digitalWrite(relayDirectionPin, forwardDirection ? LOW : HIGH);
  Serial.print("direction relay - ");
  Serial.println(relayDirectionPin ? "Forward" : "Reverse");
  */
  detachInterrupt(sensorPin);
}

bool isAtEndstop()
{
  return digitalRead(endstopPin) == LOW;
}

bool isDirectionRelayForward()
{
  return digitalRead(relayDirectionPin) == LOW;
}

void homingMotor()
{
  if (isAtEndstop())
  {
    currentPosition = 0;
    Serial.println("Motor at endstop, select distance and press start");
    return;
  }

  homingSymbol();

  while (digitalRead(buttonStartStopPin) == HIGH)
  {
  }

  forwardDirection = false; // Reverse
  isSlowingDown = true;     // Slow speed
  // startMotor();
  digitalWrite(relayDirectionPin, forwardDirection ? LOW : HIGH); // HIGH = relay OFF/Reverse, LOW = relay ON/Forward
  digitalWrite(relaySpeedPin, isSlowingDown ? HIGH : LOW);        // HIGH = relay OFF/Slow speed, LOW = relay ON/High speed
  digitalWrite(relayControlPin, LOW);                             // HIGH = relay OFF/Motor OFF, LOW = relay ON/Motor ON
  motorTimer = millis();
  pulseCheck = currentPosition;
  // arrowDownBlink();
  endstopState = isAtEndstop();
  directionRelayState = isDirectionRelayForward();
  Serial.println("Motor run reverse on LOW speed");

  // debug
  Serial.println("ControlPoint(5)");
  /*
  Serial.println(endstopState ? "Ative" : "No active");
  Serial.print("isMoving: ");
  Serial.println(isMoving ? "True" : "False");
  Serial.print("isSlowingDown: ");
  Serial.println(isSlowingDown ? "True" : "False");
  Serial.print("Selected distance: ");
  Serial.println(selectedDistance);
  Serial.print("Forward direction: ");
  Serial.println(forwardDirection ? "True" : "False");
  Serial.print("Relay direction: ");
  Serial.println(directionRelayState ? "Forward" : "Reverse");
  Serial.print("Relay speed: ");
  Serial.println(relaySpeedPin ? "LOW" : "HIGH");
  Serial.print("Relay control: ");
  Serial.println(relayControlPin ? "OFF" : "ON");
  Serial.println();
  // end debug
  */

  while (!isAtEndstop())
  {
    arrowDownBlink();

    if (/*currentPosition - pulseCheck < 1 ||*/ millis() - motorTimer > timeLimiter)
    {
      digitalWrite(relayControlPin, HIGH);
      digitalWrite(relaySpeedPin, HIGH);
      digitalWrite(relayDirectionPin, HIGH);
      tft.clearDisplay();
      tft.setCursor(5, 0);
      tft.setTextSize(8);
      tft.println("E1");
      tft.display();
      isMoving = false;
      Serial.println("E1: MOTOR STUCK OR ENCODER BROKEN!");
      Serial.print("pulsecheck: ");
      Serial.println(pulseCheck);
      Serial.print("currentPosition: ");
      Serial.println(currentPosition);
      Serial.print("IS moving: ");
      Serial.println(isMoving ? "True" : "False");
      delay(5000);
      tft.clearDisplay();
      tft.display();
      homingMotor();

      return;
    }
  }

  currentPosition = 0;
  stopMotor();
  forwardDirection = true;
  isSlowingDown = false;
  // digitalWrite(relayDirectionPin, forwardDirection ? LOW : HIGH); // HIGH = relay OFF/Reverse, LOW = relay ON/Forward
  // digitalWrite(relaySpeedPin, isSlowingDown ? HIGH : LOW);        // HIGH = relay OFF/Slow speed, LOW = relay ON/High speed
  // digitalWrite(relayControlPin, HIGH);                            // HIGH = relay OFF/Motor OFF, LOW = relay ON/Motor ON

  digitalWrite(relayDirectionPin, HIGH); // HIGH = relay OFF/Reverse, LOW = relay ON/Forward
  digitalWrite(relaySpeedPin, HIGH);     // HIGH = relay OFF/Slow speed, LOW = relay ON/High speed
  digitalWrite(relayControlPin, HIGH);   // HIGH = relay OFF/Motor OFF, LOW = relay ON/Motor ON
  endstopState = isAtEndstop();
  directionRelayState = isDirectionRelayForward();
  tft.clearDisplay();
  tft.display();
  distanceDisplay();
  Serial.println("Homing Motor Complete");
  Serial.println("Select the distance and press start..");
  // tft.display();

  // debug
  Serial.println("ControlPoint(6)");
  /*
  Serial.println(endstopState ? "Ative" : "No active");
  Serial.print("isMoving: ");
  Serial.println(isMoving ? "True" : "False");
  Serial.print("isSlowingDown: ");
  Serial.println(isSlowingDown ? "True" : "False");
  Serial.print("Selected distance: ");
  Serial.println(selectedDistance);
  Serial.print("Forward direction: ");
  Serial.println(forwardDirection ? "True" : "False");
  Serial.print("Relay direction: ");
  Serial.println(directionRelayState ? "Forward" : "Reverse");
  Serial.print("Relay speed: ");
  Serial.println(relaySpeedPin ? "LOW" : "HIGH");
  Serial.print("Relay control: ");
  Serial.println(relayControlPin ? "OFF" : "ON");
  Serial.println();
  // end debug
  */
}

void pulseCounter()
{
  if (isMoving)
  {
    if (forwardDirection)
    {
      currentPosition += distancePerPulse;
    }
    else
    {
      currentPosition -= distancePerPulse;
    }
  }
}

void displayCenteredText(String text)
{
  int16_t x1;
  int16_t y1;
  uint16_t width;
  uint16_t height;

  tft.getTextBounds(text, 0, 0, &x1, &y1, &width, &height);

  // display on horizontal and vertical center
  tft.setCursor((SCREEN_WIDTH - width) / 2, (SCREEN_HEIGHT - height) / 2);
  tft.println(text); // text to display
  tft.display();
}

void arrowUpBlink()
{
  // tft.clearDisplay();
  // tft.display();
  // tft.setCursor(10, 0);
  // tft.setTextSize(8);
  // tft.println("5"); // tu musi drukować zmiena distance
  tft.fillTriangle(112, 40, 96, 60, 128, 60, WHITE);
  tft.display();
  delay(100);
  tft.fillTriangle(112, 20, 96, 40, 128, 40, WHITE);
  tft.display();
  delay(100);
  tft.fillTriangle(112, 0, 96, 20, 128, 20, WHITE);
  tft.display();
  delay(100);
  tft.fillTriangle(112, 40, 96, 60, 128, 60, BLACK);
  tft.display();
  delay(100);
  tft.fillTriangle(112, 20, 96, 40, 128, 40, BLACK);
  tft.display();
  delay(100);
  tft.fillTriangle(112, 0, 96, 20, 128, 20, BLACK);
  tft.display();
  delay(400);
}

void arrowDownBlink()
{
  // tft.clearDisplay();
  // tft.display();
  // tft.setCursor(10, 0);
  // tft.setTextSize(8);
  // tft.println("5");
  tft.fillTriangle(96, 0, 128, 0, 112, 20, WHITE);
  tft.display();
  delay(100);
  tft.fillTriangle(96, 20, 128, 20, 112, 40, WHITE);
  tft.display();
  delay(100);
  tft.fillTriangle(96, 40, 128, 40, 112, 60, WHITE);
  tft.display();
  delay(100);
  tft.fillTriangle(96, 0, 128, 0, 112, 20, BLACK);
  tft.display();
  delay(100);
  tft.fillTriangle(96, 20, 128, 20, 112, 40, BLACK);
  tft.display();
  delay(100);
  tft.fillTriangle(96, 40, 128, 40, 112, 60, BLACK);
  tft.display();
  delay(100);
}

void arrowUp()
{
  // tft.clearDisplay();
  // tft.display();
  tft.fillTriangle(112, 40, 96, 60, 128, 60, WHITE);
  tft.fillTriangle(112, 20, 96, 40, 128, 40, WHITE);
  tft.fillTriangle(112, 0, 96, 20, 128, 20, WHITE);
  tft.setCursor(5, 0);
  tft.setTextSize(8);
  tft.println(selectedDistance);
  tft.display();
}
void arrowDown()
{
  // tft.clearDisplay();
  // tft.display();
  tft.fillTriangle(96, 0, 128, 0, 112, 20, WHITE);
  tft.fillTriangle(96, 20, 128, 20, 112, 40, WHITE);
  tft.fillTriangle(96, 40, 128, 40, 112, 60, WHITE);
  tft.setCursor(5, 0);
  tft.setTextSize(8);
  tft.println(selectedDistance);
  tft.display();
}
void homingSymbol()
{
  tft.clearDisplay();
  tft.display();
  tft.fillRect(0, 0, 128, 64, WHITE);
  tft.setCursor(50, 5);
  tft.setTextSize(6);
  tft.setTextColor(BLACK, WHITE);
  tft.println("H");
  tft.setTextSize(2);
  tft.setCursor(35, 50);
  tft.println("START");
  tft.display();
  // delay(1000);
  // tft.fillRect(20,50, 100, 100, WHITE);
  // tft.setCursor(35,50);
  // tft.println("START");
  // tft.display();
  // delay(1000);
  tft.setTextColor(WHITE);
  // tft.display();
}

void distanceDisplay()
{
  tft.clearDisplay();
  // tft.display();
  if (forwardDirection)
  {
    arrowUp();
  }
  else
  {
    arrowDown();
  }
  tft.setCursor(5, 0);
  tft.setTextSize(8);
  tft.println(selectedDistance);
  tft.display();
}
