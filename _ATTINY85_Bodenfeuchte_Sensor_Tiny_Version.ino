/************************************************************************
*  433MHz - Bodenfeuchte-Sensor [FHEM]
*  Thanks to TantaJu@https://forum.fhem.de/index.php/topic,57460.0.html
*  used original Mysensor-Sketch as template
*  juergs, 16.10.2016, initial version.
*  juergs, 22.10.2016, updated.
*  *********************************************************************
*  ATMEL ATTINY 25/45/85 / ARDUINO
*
*                      +-\/-+
*     Ain0 (D 5) PB5  1|    |8  Vcc
*     Ain3 (D 3) PB3  2|    |7  PB2 (D 2) Ain1 *INT2*
*     Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1 *INT1*
*                GND  4|    |5  PB0 (D 0) pwm0 *INT0*
*                      +----+
*
*  Install: ATtiny-Models in Arduino IDE:
*  http://highlowtech.org/?p=1695
*
*  ATTINY:
*    https://cpldcpu.wordpress.com/2014/04/25/the-nanite-85/
*    https://thewanderingengineer.com/2014/08/11/pin-change-interrupts-on-attiny85/
*    http://thegaragelab.com/a-software-uart-for-the-attiny85/
*    https://github.com/thegaragelab/tinytemplate
*    https://github.com/thegaragelab/microboot
*    http://thegaragelab.com/microboot-a-simple-bootloader/
*
*
* The LaCrosse-protocol (TX2)seems to be:
*
*     Bits 0-3: header
*     Bits 4-11: device ID, changes when replacing the batteries. Unlike in the post linked above, bit 11 does not appear to be a checksum.
*     Bits 12-15: either 1111 for automatic transmission (once every 60 seconds) or 1011 for manual transmission (using the button in the battery compartment). Manual transmission does not update the weather station.
*     Bits 16-27: encode the temperature. The system of encoding decimal digits seems to be ditched in favor of a more elegant one: apply a NOT (change 1 to 0 and 0 to 1), convert to base 10, divide by 10 (into a float), subtract 50, and the result is the temperature in C.
*     Bits 28-35: encode the relative humidity. Apply a NOT, convert to base 10, and the result is the relative humidity in %.
*     Bits 36-43: appear to encode a checksum (though I plan to double-check if this is not the dew point, also reported by the weather station).

*     Example:
*     HHHH 1000 0010 1111 1101 0010 1111 1101 0011 1010 0100
*     encoding T=22.0C and RH=44%
*
*****************************************************************************************************************************************************
*/
/*
#include "LaCrosse.h"
#include "Narcoleptic.h"
#include "OneWire.h"
*/

#include <attiny_bodenfeuchte_sensor\LaCrosse.h>
#include <attiny_bodenfeuchte_sensor\Narcoleptic.h>
#include <attiny_bodenfeuchte_sensor\OneWire.h>

#define SN "Bodenfeuchte-Sensor-433-Version"
#define SV "1.1 vom 22.10.2016"

//--- conditionals
//--- zum aktivieren Kommentierung entfernen 
#define USE_WITH_NANO            
//#define USE_SEPARATE_BATTERIE_ID 
//#define USE_WITH_DALLAS_SENSOR          
//#define USE_WITH_LED
#define USE_SERIAL_OUTPUT_NANO
//#define USE_SERIAL_OUTPUT_ATTINY

#ifdef USE_WITH_NANO 
#define DALLAS_SENSOR_PIN         10    //   PIN5 = PB0 = D0 - Achtung: MOSI, Jumper zum Programmieren entfernen.
#define BODENFEUCHTE_SENSOR_PIN   2     //
#define BODENFEUCHTE_POWER_PIN    11    //   
#define TX_433_PIN                12    //   PIN6 = PB1 = D1 - Achtung: MISO.   PIN_SEND in LaCrosse.cpp
#else
#define DALLAS_SENSOR_PIN         0     //   PIN5 = PB0 = D0 - Achtung: MOSI, Jumper zum Programmieren entfernen.
#define BODENFEUCHTE_SENSOR_PIN   3     //
#define BODENFEUCHTE_POWER_PIN    4     //   
#define TX_433_PIN                1     //   PIN6 = PB1 = D1 - Achtung: MISO.   PIN_SEND in LaCrosse.cpp
#endif 

#define SENSORID_BODENFEUCHTE     100     //   Temperatur + Bodenfeuchte (int)
#define SENSORID_BATTERIE         101     //   VCC + Bodenfeuchte (float)

#define OW_ROMCODE_SIZE           8

#ifdef USE_WITH_DALLAS_SENSOR
//--- die 18B20-Instanz setzen 
OneWire  ds(DALLAS_SENSOR_PIN);     // on arduino port pin 2 (a 4.7K resistor is necessary, between Vcc and DQ-Pin   1=GND 2=DQ 3=Vcc )
#endif

									//--- use with leds? 
#define LED_ONTIME       100  // Number of cycles for LEDs to stay on, when only temporary

									//-- how many AA Cells do you want? Set 15 per cell, so 1*AA = 15, 2*AA=30
#define MAXBATTERY        15  // Maximum voltage of Battery in 100 mV, for percentage calculation
									/*
									* Nothing to normally configure beyond this line, hardware specific configs follow
									*/
#define POWERAMV          4   // Output line to provide power for the AMV
#define INPUTFREQ         2   // IRQ input for frequency count. Must be 2 or 3

#define CHILD_ID          0   // Child-Number for Humidity
#define CHILD_TEMP        1   // Child-Number for Temperature, if attached

#define BATTPOWER         1   // ADC Input for Battery-Power, comment out if no Step-Up is used

#define MAIN_PERIOD       62 //998   // How long to count pulses, for a fixed frequency this should be 998 ms (for a total runtime of 1 sec with 1 MHz oscillator)
#define SETTLE_TIME       1000   // Waiting time in ms between powering up AMV and stable frequency

#ifdef WITH_LED
#define LED_WS          5   // White LED, normal operation
#define LED_RO          6   // Red LED, Error
#define LED_MAIN        4   // Mains for LEDs, must be set LOW for LEDs to work
#endif 

									//--- globals
unsigned int            led_startup;
float                   controller_VCC = 0.0;
long                    vcc_reading = 0;
/* Internals to follow, nothing to adjust */
volatile unsigned int   pulsecount = 0;   // Counter for pulses
unsigned int            average;                 // IIR floating average filter
byte                    led_temporary = 1;                // Are LEDs jumpered to stay on=0, otherwise 1 
unsigned int            loop_counter = 0;   // Verzögerungs-Timeout-Berechnung
											//--- TX-output
volatile float          bodenfeuchte = 0.0;
volatile float          batteriespannung = 0.0;

//--- Temperatursensor-Instanz
OneWire  dallas(DALLAS_SENSOR_PIN);

//---------------------------------------------------------------------
//---------------------------------------------------------------------
//--- Prototypes
float DoBodenFeuchteMeasurement(void);
float ReadSingleOneWireSensor(OneWire ds);
//---------------------------------------------------------------------
//---------------------------------------------------------------------
//--- Helpers 
//---------------------------------------------------------------------
long readVcc()
{
	//--- read 1.1V reference against AVcc
	//--- set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
	ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	ADMUX = _BV(MUX3) | _BV(MUX2);
#else
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

	delay(2); // Wait for Vref to settle

	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA, ADSC)); // measuring

	uint8_t low = ADCL; // must read ADCL first - it then locks ADCH  
	uint8_t high = ADCH; // unlocks both

	long result = (high << 8) | low;

	/***************************************************************************************
	*        internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
	*                         = 1.1 * 5126 / 5258 => 1.09 ==> 1.09*1023*1000 = 1097049
	****************************************************************************************/

	result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

								//result = 1097049L / result; // korrigierter Wert bei 3V3 muss fuer jeden Controller bestimmt werden, obiger Wert scheint allgemeiner zu sein.

	return result; // Vcc in millivolts
}
//---------------------------------------------------------------------
void setup()
{
	//--- setup code here

#ifdef USE_WITH_NANO
#ifdef USE_SERIAL_OUTPUT_NANO
	Serial.begin(57600);
	delay(2000);
	Serial.println("===============================================");
	Serial.println("Start Bodenfeuchte-Sensor.");
	Serial.print("Sensor-ID (Bodenfeuchte): ");
	Serial.println(SENSORID_BODENFEUCHTE);
	Serial.print("Sensor-ID (Batterie): ");
	Serial.println(SENSORID_BATTERIE);
	Serial.println("===============================================");
#endif 
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
#endif 

	//--- make power-line for AMV output and low
	pinMode(POWERAMV, OUTPUT);
	digitalWrite(POWERAMV, LOW);

	//--- preset SensorId & TX instance 
	LaCrosse.bSensorId = SENSORID_BODENFEUCHTE;
	LaCrosse.setTxPinMode(OUTPUT);
}
//---------------------------------------------------------------------
void loop()
{
	// put your main code here, to run repeatedly:
#ifdef USE_WITH_NANO
	/*    Serial.print("Temp.: ");
	Serial.print(( (float) temp_mittel/10.0) + temp_offset, 1);
	Serial.print(" Druck: ");
	Serial.print(((float)druck_mittel/100.0)+druck_offset, 2);
	Serial.print(" Druck[corr]: ");
	Serial.println((float) luftdruck, 2);
	*/
#endif

	delay(1000); // ms, remove on productive

#ifdef USE_WITH_NANO
	digitalWrite(13, HIGH);
#endif 

	//--- Betriebsspannung auslesen  
	vcc_reading = readVcc();

	//float controllerVCC = 1.1 * 1023 / vcc_reading; 
	float controllerVCC = vcc_reading / 1000.0;

	//--- Bodenfeuchte auslesen 
	bodenfeuchte = DoBodenFeuchteMeasurement();

	float tx_temp = ReadSingleOneWireSensor(dallas);

#ifdef USE_WITH_NANO 
#ifdef USE_SERIAL_OUTPUT_NANO
	Serial.print("Sensor-ID (Batterie): ");
	Serial.println(SENSORID_BATTERIE);
	Serial.print("Dallas-Sensor: ");
	Serial.println(tx_temp);
#endif 
#endif 

	// *** Bodenfeuchte-Zählerstand senden 
	//     transfer measured values through LaCrosse-TX2-instance
	LaCrosse.bSensorId = SENSORID_BODENFEUCHTE;
	LaCrosse.t = tx_temp;         //--- alias temperature;  
	LaCrosse.h = bodenfeuchte;    //--- alias humidity;  
	LaCrosse.sendTemperature();
	LaCrosse.sleep(1);        /* 1 second, no power-reduction! see impact on powersave */
	LaCrosse.sendHumidity();
	LaCrosse.sleep(1);        /* 1 second, no power-reduction! see impact on powersave */

							  // *** Batteriespannung senden 
#ifdef USE_SEPARATE_BATTERIE_ID 
	LaCrosse.bSensorId = SENSORID_BATTERIE;
#endif 
	LaCrosse.t = float(bodenfeuchte) / 1000;
	LaCrosse.sendTemperature();
	LaCrosse.sleep(1);        /* 1 second, no power-reduction! see impact on powersave */
	LaCrosse.h = controllerVCC;        //--- alias humidity;
	LaCrosse.sendHumidity();

#ifdef USE_WITH_NANO
	digitalWrite(13, LOW);
#endif

	LaCrosse.sleep(1);        /* 1 second, no power-reduction! */

#ifdef USE_WITH_NANO
	digitalWrite(13, HIGH);

#ifdef USE_SERIAL_OUTPUT_NANO
	Serial.print("VCC (int) = ");
	Serial.println(vcc_reading, DEC);
	Serial.print("VCC (float) = ");
	Serial.println(controllerVCC);
	Serial.println("===============================================");
	Serial.print("Sensor-ID (Bodenfeuchte): ");
	Serial.println(SENSORID_BODENFEUCHTE);
	Serial.println("Bodenfeuchte (int) = ");
	Serial.print("Bodenfeuchte (int) = ");
	Serial.println(bodenfeuchte, 0);
	Serial.print("Bodenfeuchte (float) = ");
	Serial.println(float(bodenfeuchte) / 1000);
#endif 

	digitalWrite(13, LOW);

	delay(1000);     //--- as test repetition
					 //delay(60000);  // ms

#else

	Narcoleptic.delay_minutes(3);

#endif 
}

//---------------------------------------------------------------------
void myinthandler(void)   // interrupt handler
{
	pulsecount++;
	//Serial.print("PulseCount:") ;
	//Serial.println(pulsecount);  
}
//---------------------------------------------------------------------
float DoBodenFeuchteMeasurement(void)
{
	//--- [1] startUp Multivibrator
	pinMode(INPUTFREQ, INPUT);
	pinMode(POWERAMV, OUTPUT);
	digitalWrite(POWERAMV, HIGH);   //--- power up sensor circuit  

									//--- [2] wait for settle
	delay(SETTLE_TIME);

	//--- [3] Prepare measurement
	pulsecount = 0;
#if INPUTFREQ == 2
	attachInterrupt(0, myinthandler, FALLING); // IRQ D2 low to high
#elif INPUTFREQ == 3
	attachInterrupt(1, myinthandler, FALLING); // IRQ D3 low to high
#else
#error "Frequency input must be D2 or D3"
#endif

	//--- [4] wait for measurement to finalize, wait for interrupt actions
	delay(MAIN_PERIOD);

	//--- [5] store actual counter value
	//---     register counts per period (frequency) and calculate IIR
	unsigned long _pulses = pulsecount;
	pulsecount = 0;

	//--- simple IIR, floating average
	average += _pulses;
	if (average != _pulses)    //--- during startup both are equal
		average >>= 1;

	//---[6] stop measuring and AMV
	digitalWrite(POWERAMV, LOW);
#if INPUTFREQ == 2
	detachInterrupt(0);
#elif INPUTFREQ == 3
	detachInterrupt(1);
#else
#error "Frequency input must be D2 or D3"
#endif

	pinMode(INPUTFREQ, INPUT);

	//--- [7] as result set reading  for TX 
	return ((float)average);
}
//-------------------------------------------------------------------------
float ReadSingleOneWireSensor(OneWire ds)
{
	//--- h + t variables holding measurement results 
	//--- we do not have a humidity sensor, preset a fictive value
	float dallas_temperatur = 12.3;

	//--- 18B20 stuff
	byte i;
	byte present = 0;
	byte type_s;
	byte data[12];
	byte addr[8];
	float celsius, fahrenheit;

	if (!ds.search(addr))
	{
		ds.reset_search();
		delay(250);
		return dallas_temperatur;
	}

	if (OneWire::crc8(addr, 7) != addr[7])
	{
		//Serial.println("CRC is not valid!");
		//--- blink Led instead? 
		return dallas_temperatur;
	}

	//--- the first ROM byte indicates which chip
	switch (addr[0])
	{
	case 0x10:
		//Serial.println("  Chip = DS18S20");  // or old DS1820
		type_s = 1;
		break;
	case 0x28:
		// Serial.println("  Chip = DS18B20");
		type_s = 0;
		break;
	case 0x22:
		// Serial.println("  Chip = DS1822");
		type_s = 0;
		break;
	default:
		// Serial.println("Device is not a DS18x20 family device.");
		return dallas_temperatur;
	}

	ds.reset();

	ds.select(addr);

	ds.write(0x44);        // start conversion, use alternatively ds.write(0x44,1) with parasite power on at the end

	delay(1000);     // maybe 750ms is enough, maybe not
					 // we might do a ds.depower() here, but the reset will take care of it.

	present = ds.reset();

	ds.select(addr);

	ds.write(0xBE);         //--- read scratchpad

	for (i = 0; i < 9; i++)
	{
		//--- we need 9 bytes
		data[i] = ds.read();
	}

	//--- Convert the data to actual temperature
	//--- because the result is a 16 bit signed integer, it should
	//--- be stored to an "int16_t" type, which is always 16 bits
	//--- even when compiled on a 32 bit processor.
	int16_t raw = (data[1] << 8) | data[0];
	if (type_s)
	{
		raw = raw << 3;     //--- 9 bit resolution default
		if (data[7] == 0x10)
		{
			//--- "count remain" gives full 12 bit resolution
			raw = (raw & 0xFFF0) + 12 - data[6];
		};
	}
	else
	{
		byte cfg = (data[4] & 0x60);
		// at lower res, the low bits are undefined, so let's zero them
		if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
		else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
		else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
											  //// default is 12 bit resolution, 750 ms conversion time
	};

	celsius = (float)raw / 16.0;
	fahrenheit = celsius * 1.8 + 32.0;

	//--- t (float) is sended value  by LaCrosse,
	// use alternatively LaCrosse.setSensorId(id) 
	dallas_temperatur = celsius;

	//---- Check if any reads failed and exit early (to try again).  
	if (isnan(dallas_temperatur))
	{
		// led blink?
		//--- signalize error condition 
		dallas_temperatur = -99.0;
		return dallas_temperatur;
	};
}
//-------------------------------------------------------------------------
//---------------------------------------------------------------------
// <eof>
//---------------------------------------------------------------------

