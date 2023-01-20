#include <FastLED.h>
#include <Wire.h>

#include <string.h>

#define LED_PIN 5 //data pin for LED. 
#define NUM_LEDS 300 //Numebr of LEDS being run on the link
#define BRIGHTNESS 20 //Maximum Brightness for LED
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

#define UPDATES_PER_SECOND 100

#define SPTR_SIZE 20
char *sPtr[SPTR_SIZE];

char *tokenRemainder;

int RGBNum[3];


char *command = "S|#FFA500-#FFA500-#0000FF-#0000FF";  // Initial command to be run

CRGB color[4];

const char delinizerOne[] = "|"; //Token item one

const char delinizerTwo[] = "-"; //Token item two

int speed = 3; // General speed overall

int separateAll(String str, char **p, int size, char *token) {
  Serial.println(str);
  int n;
  char s[160];
  strcpy(s, str.c_str());

  *p++ = strtok(s, token);
  for (n = 1; NULL != (*p++ = strtok(NULL, " ")); n++)
    if (size == n)
      break;

  return n;
}

char StrContains(char *str, char *sfind) {
  char found = 0;
  char index = 0;
  char len;

  len = strlen(str);

  if (strlen(sfind) > len) {
    return 0;
  }
  while (index < len) {
    if (str[index] == sfind[found]) {
      found++;
      if (strlen(sfind) == found) {
        return 1;
      }
    } else {
      found = 0;
    }
    index++;
  }

  return 0;
}

char *separate(char *str, char **p, int size, char *token) {
  char *msg = strtok(str, token);
  Serial.println(msg);
  tokenRemainder = strtok(NULL, " ");
  return msg;
}

CRGBPalette16 currentPalette;
TBlendType currentBlending;

void startup(uint8_t color) {
  fill_solid(currentPalette, 16, CRGB::Black);  //turning all colors black
  for (int i = 0; i < 4; i++) {
    currentPalette[0] = color;
  }
  for (int i = NUM_LEDS; i > 0; i--) {
    FillLEDsFromPaletteColors(i);
    FastLED.show();
    FastLED.delay(1000 / UPDATES_PER_SECOND);
  }
}

void setup() {
  delay(3000);         // power-up safety delay
  Serial.begin(9600);  // begin Serial Input on baud 9600
  Wire.begin(4);       // begin wire transmission between I2C as slave #4
  Wire.onReceive(setLEDS);
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  //currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;

  startup(CRGB::Blue);
  FastLED.clear();
  delay(1000);
  fill_solid(currentPalette, 16, CRGB::Blue);
  FastLED.show();
  Serial.println(F("Command:"));
  Serial.println(command);
  Serial.println(F("Running LEDS..."));
  runLEDS();
  FastLED.show();
}


void loop() {
  static uint8_t startIndex = 0;
  startIndex = startIndex + 1; /* motion speed */


  FillLEDsFromPaletteColors(startIndex);

  FastLED.show();
  FastLED.delay(1000 / UPDATES_PER_SECOND);
}

void setLEDS() {
  char *newCommand = Wire.read();
  char command = newCommand;
  runLEDS();
}

void changeSpeed(char *setting) {
  separate(setting, sPtr, SPTR_SIZE, ":");
  speed = atoi(separate(tokenRemainder, sPtr, SPTR_SIZE, ":"));
  Serial.println(speed);
}

void runLEDS() {
  // split(command, 2);
  // char f = subStrings[0];
  // Serial.println(f);
  // char p = subStrings[1];
  // Serial.println(p);

  // int N = separate(command, sPtr, SPTR_SIZE, &strData, delinizerOne);
  separateAll(command, sPtr, SPTR_SIZE, delinizerOne);
  // for (int n = 0; n < SPTR_SIZE; n++) {
  //   Serial.println (sPtr [n]);
  // }
  // for (int n = 0; n < N; n++) {
  //   Serial.println(sPtr[n]);
  // }
  Serial.println("Function:");
  Serial.println(sPtr[0]);
  Serial.println("Param:");
  Serial.println(sPtr[1]);
  identifyFunction(sPtr[0], sPtr[1]);
}


void identifyFunction(char *f, char *p) {
  //Serial.println(f);
  if (strcmp(f, "C") == 0) {
    Serial.println("Clearing...");
    clear();
  } else if (strcmp(f, "S") == 0) {
    Serial.println("Solid...");
    solid(p);
  } else if (strcmp(f, "B") == 0) {
    Serial.println("Blink...");
    blink(p);
  } else if (strcmp(f, "T") == 0) {
    Serial.println("Track...");
    track(p);
  } else if (strcmp(f, "R") == 0) {
    Serial.println("Rainbow...");
    rainbow(p);
  }
}


void clear() {
  fill_solid(currentPalette, 16, CRGB::Black);
}

int getRGB(String hexvalue) {
  //Serial.println(hexvalue);
  //String hexvalue = value;
  hexvalue.toUpperCase();

  char c[7];
  hexvalue.toCharArray(c, 8);
  int red = hexcolorToInt(c[1], c[2]);
  int green = hexcolorToInt(c[3], c[4]);
  int blue = hexcolorToInt(c[5], c[6]);
  RGBNum[0] = red;
  RGBNum[1] = green;
  RGBNum[2] = blue;
}

int hexcolorToInt(char upper, char lower) {
  int uVal = (int)upper;
  int lVal = (int)lower;
  uVal = uVal > 64 ? uVal - 55 : uVal - 48;
  uVal = uVal << 4;
  lVal = lVal > 64 ? lVal - 55 : lVal - 48;
  // Serial.println(uVal+lVal);
  return uVal + lVal;
}


void getColors(char *codes) {
  //Serial.println(codes);
  //String lol = String(codes);
  sPtr[0] = separate(codes, sPtr, SPTR_SIZE, delinizerTwo);
  sPtr[1] = separate(tokenRemainder, sPtr, SPTR_SIZE, delinizerTwo);
  sPtr[2] = separate(tokenRemainder, sPtr, SPTR_SIZE, delinizerTwo);
  sPtr[3] = separate(tokenRemainder, sPtr, SPTR_SIZE, delinizerTwo);
  char *hex[4] = { sPtr[0], sPtr[1], sPtr[2], sPtr[3] };
  for (int i = 0; i < 4; i++) {
    if (strlen(hex[i]) > 3) {
      getRGB(String(hex[i]));
      color[i] = CRGB(RGBNum[0], RGBNum[1], RGBNum[2]);
    } else {
      Serial.println(i);
      Serial.println("No Value");
      if (i > 0) {
        color[i] = CRGB(RGBNum[0], RGBNum[1], RGBNum[2]);
      } else {
        color[i] = CRGB(0, 0, 0);
      }
    }
  }
}

void solid(char *p) {
  Serial.println(p);
  clear();
  getColors(p);

  for (int h = 0; h < 4; h++) {
    for (int n = 1; n < 5; n++) {
      currentPalette[(h * 4) + n] = color[h];
    }
  }
  speed = 3;
}

void blink(char *p) {
  clear();
  getColors(p);

  fill_solid(currentPalette, 16, CRGB::Black);
  // and set every fourth one to white.
  for (int i = 0; i < 4; i++) {
    currentPalette[i * 4] = color[i];
  }
  speed = 0;
}

void track(char *p) {
  clear();
  getColors(p);

  fill_solid(currentPalette, 16, CRGB::Black);
  // and set every fourth one to white.
  for (int i = 0; i < 4; i++) {
    currentPalette[i * 4] = color[i];
  }
  speed = 2;
}

void rainbow(char *p) {
  clear();
  getColors(p);
  currentPalette = RainbowStripeColors_p;
  currentBlending = LINEARBLEND;
  speed = 3;
}

void FillLEDsFromPaletteColors(uint8_t colorIndex) {
  uint8_t brightness = 255;

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = ColorFromPalette(currentPalette, colorIndex, brightness, currentBlending);
    colorIndex += speed;
  }
}
