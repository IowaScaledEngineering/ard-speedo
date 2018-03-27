#include <Wire.h>
#include <LiquidCrystal.h>

#define EAST_DETECTOR_ADDR  0x38
#define WEST_DETECTOR_ADDR  0x39

#define DETECT_THRESHOLD 0x300

#define DETECTOR_TIMEOUT_MS 60000

#define DETECTOR_SPACING_INCHES  12.0

//#define SCALE_FACTOR  160
#define SCALE_FACTOR  87.1

#define DET_IDLE    0
#define START_EAST  1
#define CALC_EAST   2
#define START_WEST  3
#define CALC_WEST   4
#define WAIT        5
#define TIMEOUT     6

#define BACKLIGHT_PIN        10

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void writeByte(uint8_t addr, uint8_t cmd, uint16_t writeVal)
{
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  Wire.write(writeVal);
  Wire.endTransmission();
}

void readWord(uint8_t addr, uint8_t cmd, uint16_t* data)
{
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  Wire.requestFrom(addr, (uint8_t)2);
  byte b1 = Wire.read();
  byte b2 = Wire.read();
  Wire.endTransmission();

  *data = b1;
  *data |= ((uint16_t)b2 << 8);
}

void initializeTMD26711(uint8_t addr)
{
	// Initialize TMD26711 (bit 0x80 set to indicate command)
	writeByte(addr, 0x80|0x00, 0x00);   // Start with everything disabled
	writeByte(addr, 0x80|0x01, 0xFF);   // Minimum ATIME
	writeByte(addr, 0x80|0x02, 0xFF);   // Maximum integration time
	writeByte(addr, 0x80|0x03, 0xFF);   // Minimum wait time

	// Note: IRQ not currently used
	writeByte(addr, 0x80|0x08, 0x00);   // Set interrupt low threshold to 0x0000
	writeByte(addr, 0x80|0x09, 0x00);
	writeByte(addr, 0x80|0x0A, 0x00);   // Set interrupt low threshold to 0x0300
	writeByte(addr, 0x80|0x0B, 0x03);
	writeByte(addr, 0x80|0x0C, 0x10);   // Single out-of-range cycle triggers interrupt

	writeByte(addr, 0x80|0x0D, 0x00);   // Long wait disabled
	writeByte(addr, 0x80|0x0E, 8);      // Pulse count
	writeByte(addr, 0x80|0x0F, 0x20);   // 100% LED drive strength, Use channel 1 diode (ch 1 seems less sensitive to fluorescent light)

	writeByte(addr, 0x80|0x00, 0x27);   // Power ON, Enable proximity, Enable proximity interrupt (not used currently)
}

uint8_t readDetector(uint8_t addr)
{
	uint16_t proximity;
	readWord(addr, 0x80|0x20|0x18, &proximity);  // Read data register (0x80 = command, 0x20 = auto-increment)

	if(WEST_DETECTOR_ADDR == addr)
		lcd.setCursor(0,1);
	else
		lcd.setCursor(15,1);

	if(proximity > DETECT_THRESHOLD)
	{
		lcd.print('*');
		return(1);
	}
	else
	{
		lcd.print('-');
		return(0);
	}
}

byte detectorState = DET_IDLE;

void setup()
{
	Wire.begin();
	Serial.begin(115200);
	lcd.begin(16,2);
	pinMode(BACKLIGHT_PIN, OUTPUT);
	digitalWrite(BACKLIGHT_PIN, 1);

	lcd.clear();

	initializeTMD26711(EAST_DETECTOR_ADDR);
	initializeTMD26711(WEST_DETECTOR_ADDR);
	
	// Read once to initialize
	readDetector(EAST_DETECTOR_ADDR);
	readDetector(WEST_DETECTOR_ADDR);

	detectorState = DET_IDLE;
}

unsigned long startTime = 0;
unsigned long endTime = 0;
unsigned long decimph = 0;
byte dir = 0;


void updateDisplay(unsigned int decimph, int8_t dir)
{
	lcd.setCursor(3,0);

	if (decimph > 1200)
		lcd.print("***.*");
	else
	{
		uint8_t digit = (decimph/1000)%10;
		if(digit)
			lcd.print(digit);
		else
			lcd.print(' ');
		digit = (decimph/100)%10;
		if(digit)
			lcd.print(digit);
		else
			lcd.print(' ');
		digit = (decimph/10)%10;
		lcd.print(digit);
		lcd.print('.');
		digit = (decimph/1)%10;
		lcd.print(digit);
	}
	lcd.print(" MPH ");
}

void loop()
{
	uint8_t eastDetector = readDetector(EAST_DETECTOR_ADDR);
	uint8_t westDetector = readDetector(WEST_DETECTOR_ADDR);
	
	Serial.print(westDetector);
	Serial.print(" ");
	Serial.print(eastDetector);
	Serial.print('\n');
	
	switch(detectorState)
	{
		case DET_IDLE:
			startTime = millis();
			if (eastDetector)
			{
				detectorState = START_EAST;
			}
			else if (westDetector)
			{
				detectorState = START_WEST;
			}

			break;

		case START_EAST:
			endTime = millis();
			lcd.setCursor(6,1);
			lcd.print("<---");
			if (westDetector)
				detectorState = CALC_EAST;

			if (endTime > startTime + DETECTOR_TIMEOUT_MS)
				detectorState = TIMEOUT;
			break;

		case START_WEST:
			endTime = millis();
			lcd.setCursor(6,1);
			lcd.print("--->");
			if (eastDetector)
				detectorState = CALC_WEST;

			if (endTime > startTime + DETECTOR_TIMEOUT_MS)
				detectorState = TIMEOUT;
			break;

		case CALC_EAST:
		case CALC_WEST:
			// Calculation of model speed
			// Scale distance - DETECTOR_SPACING_INCHES * SCALE_FACTOR / (63360 in/mile)
			// Time = millis * 3,600,000 milliseconds / hour
			// Speed = distance / time
			// Speed = (DETECTOR_SPACING_INCHES * SCALE_FACTOR / 63360) / (time / 3600000)
			// Speed = (DETECTOR_SPACING_INCHES * SCALE_FACTOR * 625 / time * 11)

			decimph = (DETECTOR_SPACING_INCHES * SCALE_FACTOR * 625 * 10) / ((endTime - startTime) * 11);

			dir = detectorState;

			startTime = millis();
			detectorState = WAIT;
			break;

		case WAIT:
			updateDisplay(decimph, dir);
			if ( (eastDetector) || (westDetector) )
				startTime = millis();

			if (millis() > startTime + 5000)
				detectorState = TIMEOUT;
			break;

		case TIMEOUT:
			detectorState = DET_IDLE;
			decimph = 0;
			dir = 0;
			lcd.setCursor(6,1);
			lcd.print("    ");
			updateDisplay(0, 0);
			break;
	}
}

