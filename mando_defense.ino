#include <SPI.h>
#include <Wire.h>

#include <SparkFunLSM9DS1.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN    6
#define LED_COUNT  169
#define BRIGHTNESS 50 // (max = 255)

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

// Accelerometer setup
LSM9DS1 imu;

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

short accelerometerLow = -33;
short accelerometerHigh = 33;
double accelerometerRange = abs(accelerometerLow) + abs(accelerometerHigh);
short boardIsUnderControl = 5;

short leftLowDegreeBoundary = 315;//45;
short leftHighDegreeBoundary = 225;//135;

short leftLowLEDBoundary = map(leftLowDegreeBoundary, 0, 360, 1, LED_COUNT);
short leftHighLEDBoundary = map(leftHighDegreeBoundary, 0, 360, 1, LED_COUNT);

short rightLowDegreeBoundary = 45;//315;
short rightHighDegreeBoundary = 135;//225;

short rightLowLEDBoundary = map(rightLowDegreeBoundary, 0, 360, 1, LED_COUNT);
short rightHighLEDBoundary = map(rightHighDegreeBoundary, 0, 360, 1, LED_COUNT);

short leftFlankDegree = 260;
short rightFlankDegree = 100;

short leftFlankLED = map(leftFlankDegree, 0, 360, 1, LED_COUNT);
short rightFlankLED = map(rightFlankDegree, 0, 360, 1, LED_COUNT);

short prevLeftPos = 0;
short prevRightPos = 0;

short defaultBallPosition = floor(LED_COUNT / 2);
double ballPosition = defaultBallPosition;

short defaultBallDirection = 1;
short ballDirection = defaultBallDirection;

double defaultBallUpdateRate = 0.4;
double ballUpdateRate = defaultBallUpdateRate;

double alpha = 0.3;
double filteredPosition = 0.0;
double previousPosition = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.println("Begin setup");
  
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(BRIGHTNESS);  
  
  Serial.println("LED setup complete");

  delay(100);
  
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.

  Wire.begin();
  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
  }

  Serial.println("Done setup.");
}
 
void loop() {

  if (imu.accelAvailable()) {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  } 
  double reading = imu.calcAccel(imu.ay);
  //Serial.println(reading);
  reading = reading *100;
  filteredPosition = (1-alpha)*filteredPosition + (alpha*reading);

  //double boardControl = (abs(filteredPosition - previousPosition) * 100) / accelerometerRange;  
  //Serial.println(boardControl);
  
  previousPosition = filteredPosition;

  // map from accelerometer values to degrees
  short right_degrees = map(filteredPosition, accelerometerLow, accelerometerHigh, rightLowDegreeBoundary, rightHighDegreeBoundary);
  short right_pos = map(right_degrees, rightLowDegreeBoundary, rightHighDegreeBoundary, rightLowLEDBoundary, rightHighLEDBoundary);
  short left_pos = map(right_degrees +180, leftLowDegreeBoundary, leftHighDegreeBoundary, leftLowLEDBoundary, leftHighLEDBoundary);
  
  updateCenterBoard();
  updateLeftBoard(left_pos);
  updateRightBoard(right_pos);

  // update the ball position
  strip.setPixelColor(floor(ballPosition), strip.Color(0,  0,   0)); 

  if (left_pos <= ballPosition) {
    ballPosition = left_pos - 1;
    ballDirection = -1;
    ballUpdateRate = ballUpdateRate * 1.05;
  } else if (right_pos >= ballPosition){  
    ballPosition = right_pos + 1;
    ballDirection = 1;
    ballUpdateRate = ballUpdateRate * 1.05;
  } else {
    ballPosition = ballPosition + (ballDirection * ballUpdateRate);
  }

  if (ballUpdateRate > 3) {
    ballUpdateRate = defaultBallUpdateRate;
  }

  strip.setPixelColor(leftFlankLED, strip.Color(0, 255, 0));
  strip.setPixelColor(rightFlankLED, strip.Color(0, 255, 0));
  
  strip.setPixelColor(ballPosition, strip.Color(0, 0, 255)); 
  strip.show();
  
  delay(25);
}

void updateCenterBoard() {
  // set the bottom lights 
  for (int i = LED_COUNT; i >= leftLowLEDBoundary; i--) {
    strip.setPixelColor(i, strip.Color(153, 255, 255)); 
  }
  for (int i = 0; i <= rightLowLEDBoundary; i++) {
    strip.setPixelColor(i, strip.Color(153, 255, 255)); 
  }
}

void updateRightBoard(short right_pos){
  for (int i = rightLowLEDBoundary; i <= right_pos; i++) {
    if (i < rightHighLEDBoundary) {
      strip.setPixelColor(i, strip.Color(153,  255,   255)); 
    }
  }
  if (right_pos < rightHighLEDBoundary) {
    for (int i = right_pos+1; i <= rightHighLEDBoundary; i++) {
      strip.setPixelColor(i, strip.Color(0,  0,   0)); 
    }
  }
}

void updateLeftBoard(short left_pos) {
  for (int i = leftLowLEDBoundary; i >= left_pos; i--) {
    if (i > leftHighLEDBoundary) {
      strip.setPixelColor(i, strip.Color(153, 255, 255)); 
    }
  }
  if (left_pos > leftHighLEDBoundary) {
    for (int i = left_pos-1; i >= leftHighLEDBoundary; i--) {
      strip.setPixelColor(i, strip.Color(0,  0,   0)); 
    }
  }
}
