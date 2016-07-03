#include <SPI.h>
#include "ads12xx.h"

/* Arduino UNO */
#if 1
/* Pin configuration: */
int  START = 8;
int  CS = 9;
int  DRDY = 3;
#else
/* Use free pins when LCD shield is used. */
int  START = A1;
int  CS = A2;
int  DRDY = 2;
#endif

//Define which ADC to use in the ads12xx.h file

ads12xx ADS;

void setup()
{
	Serial.begin(9600);
	while (!Serial) {
		
	}
	Serial.println("Serial online");
	ADS.begin(CS, START, DRDY);  //initialize ADS as object of the ads12xx class

	ADS.Reset();
	
	delay(10);

	Serial.println("Commands for testing:");
	Serial.println("'r' to read Register");
	Serial.println("'w' to write Register");
	Serial.println("'R' to get a Conversion Result");
	Serial.println("'x' to SDATAC, 'd' for SDATA");
	Serial.println("'o' to write Pre Predefinde Registers");
	Serial.println("'f' to write a command");
	Serial.println("'c' to select calibration methode");
	Serial.println("'t' to chose test");
	Serial.println("'h' for this help");
}


void loop() {

	if (Serial.available()) {
		char cin = Serial.read();
		char  check = 'y';
		uint8_t cmd;
		uint8_t cin1;
		switch (cin) {
		case 'r':
			Serial.println("Which Register to read?");
			while (!Serial.available());
			Serial.print("Register Value for: ");
			cin1 = Serial.parseInt();
			Serial.println(cin1);
			Serial.println(ADS.GetRegisterValue(cin1));
			break;
		case 'w':
			Serial.println("Which Register to write?");
			while (!Serial.available());
			cin1 = Serial.parseInt();
			Serial.println("Which Value to write?");
			while (!Serial.available());
			ADS.SetRegisterValue(cin1, Serial.parseInt());
			break;
		case 'R':
			Serial.println("Conversion Result");
			Serial.println(ADS.GetConversion());
			break;
		case 'x':
			Serial.println("Stop SDATAC");
			ADS.Reset();
			break;
		case 'o':
			Serial.println("Writing predefind Registers");
#ifdef ADS1256
			ADS.SetRegisterValue(MUX, P_AIN0 | N_AINCOM);
			ADS.SetRegisterValue(DRATE, DR_1000);
#endif
#ifdef ADS1248
			ADS.SetRegisterValue(SYS0, DOR3_2000 | PGA2_0);

			ADS.SetRegisterValue(IDAC0, DRDY_ONLY | IMAG2_OFF
				//	| IDAC0_ID
				);
			ADS.SetRegisterValue(IDAC1, I1DIR_OFF);
			ADS.SetRegisterValue(MUX1, BCS1_1 | MUX_SP2_AIN0 | MUX_SN2_AIN2);
#endif
			Serial.println("Writing sucessfull");
			break;
		case 'd':
			while (check == 'y') {
				if (Serial.available()) {
					check = Serial.read();

				}
				uint32_t data = ADS.GetConversion();
				int timer1 = micros();
				if (long minus = data >> 23 == 1) {
					long data = data - 16777216;
				}
				Serial.println(data);

#ifdef ADS1256
				//	double voltage = (4.9986 / 8388608)*data;
				//	Serial.println(voltage);
#endif
			}
			break;
		case 'f':
			Serial.println("Which command to write");
			while (!Serial.available());
			cmd = Serial.parseInt();
			Serial.print(cmd, HEX);
			ADS.SendCMD(cmd);
			break;
		case 'c':
			Serial.println("Which Calibration to run?");
#ifdef ADS1256			
			Serial.println("'1' for SELFCAL");
#endif
#ifdef ADS1248
			Serial.println("'1' for SELFOCAL");
#endif
			Serial.println("'2' for SYSOGCAL\n'3' for SYSGCAL");
			while (!Serial.available());
			cmd = Serial.parseInt();
			switch (cmd)
			{
#ifdef ADS1256
			case 1:
				Serial.println("Preforming Self Gain and Offset Callibration");
				ADS.SendCMD(SELFCAL);
				delay(5);
				Serial.print("OFC0: ");
				Serial.println(ADS.GetRegisterValue(OFC0));
				Serial.print("OFC1: ");
				Serial.println(ADS.GetRegisterValue(OFC1));
				Serial.print("OFC2: ");
				Serial.println(ADS.GetRegisterValue(OFC2));
				Serial.print("FSC0: ");
				Serial.println(ADS.GetRegisterValue(FSC0));
				Serial.print("FSC1: ");
				Serial.println(ADS.GetRegisterValue(FSC1));
				Serial.print("FSC2: ");
				Serial.println(ADS.GetRegisterValue(FSC2));
				break;
#endif
#ifdef ADS1248
			case 1:
				Serial.println("Preforming Self Offset Callibration");
				ADS.SendCMD(SELFOCAL);
				delay(5);
				Serial.print("OFC0: ");
				Serial.println(ADS.GetRegisterValue(OFC0));
				Serial.print("OFC1: ");
				Serial.println(ADS.GetRegisterValue(OFC1));
				Serial.print("OFC2: ");
				break;
#endif
			case 2:
				Serial.println("Preforming System Offset Callibration");
				ADS.SendCMD(SYSOCAL);
				delay(5);
				Serial.print("OFC0: ");
				Serial.println(ADS.GetRegisterValue(OFC0));
				Serial.print("OFC1: ");
				Serial.println(ADS.GetRegisterValue(OFC1));
				Serial.print("OFC2: ");
				Serial.println(ADS.GetRegisterValue(OFC2));
				break;
			case 3:
				Serial.println("Preforming System Gain Callibration");
				ADS.SendCMD(SYSGCAL);
				delay(5);
				Serial.print("FSC0: ");
				Serial.println(ADS.GetRegisterValue(FSC0));
				Serial.print("FSC1: ");
				Serial.println(ADS.GetRegisterValue(FSC1));
				Serial.print("FSC2: ");
				Serial.println(ADS.GetRegisterValue(FSC2));
				break;
			default:
				break;
			}
			break;
		case 'h':
			Serial.println("Commands for testing:");
			Serial.println("'r' to read Register");
			Serial.println("'w' to write Register");
			Serial.println("'R' to get a Conversion Result");
			Serial.println("'x' to SDATAC, 'd' for SDATA");
			Serial.println("'o' to write Pre Predefinde Registers");
			Serial.println("'f' to write a command");
			Serial.println("'c' to select calibration methode");
			Serial.println("'t' to chose test");
			Serial.println("'h' for this help");
			break;
#ifdef ADS1248
		case 't':
			Serial.println("Chose which test to run");
			Serial.println("'1' for Internal Temperature\n'2' for Supply Voltage Measurement\n'3' for External Voltage Reference\n'4' for Voltage Measurement\n'5' for Thermocouple Measurement");
			while (!Serial.available());
			cmd = Serial.parseInt();
			switch (cmd)
			{
			case 1:
				test_intTemp();
				break;
			case 2:
				test_supVoltage();
				break;
			case 3:
				test_extrefVoltage();
				break;
			case 4:
				test_Voltage();
				break;
			case 5:
				test_Thermo();
				break;
			}

#endif

		default:
			break;
		}
	}
}

#ifdef ADS1248

/*
Theses are test functions for the ADS1248 system monitor as well
as some examples for 4-wire and thermocouple measurment (Warning these are not well tested).

Run them atleast 3 times to get proper values.

--------------------------------------------------------------

This function gets temperature from the internal diode
*/
void test_intTemp() {
	ADS.SetRegisterValue(MUX1, MUXCAL2_TEMP | VREFCON1_ON | REFSELT1_ON);

	unsigned long temp_val = ADS.GetConversion();
	Serial.println(temp_val, DEC);
	double temp_voltage = (2.048 / 8388608)*temp_val;	//only 23bit data here.
	Serial.println(temp_voltage, DEC);
	double temp = ((temp_voltage - 0.118) / (405 * 0.000001)) + 25;	//see Datasheet S.30
	Serial.print("Internal Temperature is ");
	Serial.print(temp);
	Serial.println(" Grad Celsius!");
}

/*
This function measures the supplied voltages
*/
void test_supVoltage() {
	ADS.SetRegisterValue(MUX1, MUXCAL2_DVDD | VREFCON1_ON | REFSELT1_ON);

	unsigned long volt_val = ADS.GetConversion();
	//Serial.println(volt_val, DEC);
	double voltage = (2.048 / 16777216)*volt_val;
	voltage *= 4 * 2.048;
	Serial.print("DVDD Voltage: ");
	Serial.print(voltage);
	Serial.println("V");

	ADS.SetRegisterValue(MUX1, MUXCAL2_AVDD | VREFCON1_ON | REFSELT1_ON);

	unsigned long volt1_val = ADS.GetConversion();
	//Serial.println(volt1_val, DEC);
	double voltage1 = (2.048 / 16777216)*volt1_val;
	voltage1 *= 4 * 2.048;
	Serial.print("AVDD Voltage: ");
	Serial.print(voltage1);
	Serial.println("V");
}

/*
This function measures the voltage of the external voltage reference 1
You have to connect the AIN0 to the specific REF0 or REF1 output
*/
void test_extrefVoltage() {
	ADS.SetRegisterValue(MUX1, REFSELT1_ON | VREFCON1_ON | MUXCAL2_REF0);	  //ADS Reference on Intern, Internal Reference on, System Montitor on REF1
	ADS.SetRegisterValue(IDAC0, IMAG2_1500);			 //	IDAC at 1,5mA current
	ADS.SetRegisterValue(IDAC1, I1DIR_AIN0 | I2DIR_OFF);			 // IDAC1 Currentoutput on AIN0, IDAC2 off
	ADS.SetRegisterValue(SYS0, DOR3_5);

	unsigned long volt_val = ADS.GetConversion();
	//Serial.println(volt_val, DEC);
	double voltage = (2.048 / (16777216))*volt_val;
	voltage *= 4 * 2.048;
	Serial.print("External V_Ref: ");
	Serial.print(voltage, DEC);
	Serial.println("V");


}

double convertR2T(double R)
{
  double T;
  double R0;
  double A =  3.9083e-3;
  double B = -5.775e-7; 
  double C = -4.183e-12;

  /* Auto detect RTD type */
  R0 = 100; /* PT100 */
  if (R > 500) {
    R0 = 1000; /* PT1000 */
  }

  T = -R0*A+sqrt(((R0*A)*(R0*A))-4*R0*B*(R0-R));
  T = T / (2*R0*B);
  
  return T;
}

double convertU2R(double U, double I)
{
  double R = U / I;

  return R;
}

double convertConversion2U(double conversion, double I, double Rref)
{
  double Uref = 2 * Rref * I;
  double pga = 1;
  double U;
  
  U = ((conversion * Uref) / pga) / 8388607.0;

  return U;
}

/*
Example for a 2-wire measurement (PT100 or PT1000 probe)
See README.md for setup
*/
void test_Voltage() {
  double I = 0.001; /* IDAC 1mA */
  double Rref = 1100; /* 1.1kOhm percision resistor */
  
	ADS.SetRegisterValue(MUX0, MUX_SN2_AIN0 | MUX_SP2_AIN1);
	ADS.SetRegisterValue(MUX1, REFSELT1_REF1 | VREFCON1_ON);	  //ADS Reference on REF1, Internal Reference on
	ADS.SetRegisterValue(IDAC0, IMAG2_1000);      // IDAC at 1mA current
	ADS.SetRegisterValue(IDAC1, I1DIR_AIN0 | I2DIR_AIN1);
	ADS.SetRegisterValue(SYS0, PGA2_0 | DOR3_10);

	unsigned long conversion = ADS.GetConversion();
	Serial.println(conversion, DEC);

  double U = convertConversion2U(conversion, I, Rref);
  Serial.print(U, DEC);
  Serial.println("V");
  
  double R = convertU2R(U, I);	
	Serial.print("Resistance:");
	Serial.print(R, DEC);
	Serial.println("Ohm");

  double T = convertR2T(R);
  Serial.print(T, DEC);
  Serial.println("C");
}

/*
Untested function for Thermocouple measurment
*/
void test_Thermo() {
	ADS.SetRegisterValue(MUX0, MUX_SP2_AIN0 | MUX_SN2_AIN1);
	ADS.SetRegisterValue(MUX1, REFSELT1_ON | VREFCON1_ON);	  //ADS Reference on Intern, Internal Reference on
	ADS.SetRegisterValue(VBIAS, VBIAS_0);
	ADS.SetRegisterValue(SYS0, PGA2_128);		   // 2000 sps vollkommen un�tz rauschen �berwiegt

	long volt_val = ADS.GetConversion();
	if (long minus = volt_val >> 23 == 1) {
		long volt_valneg = volt_val - 16777216;
		Serial.println(volt_valneg, DEC);
	}
	else
	{
		Serial.println(volt_val, DEC);
	}

	//Serial.println(minus, BIN);

	double voltage = (2.048 / (16777216 * 2))*(volt_val / 32);
	Serial.print("Thermocouple: ");
	Serial.println(voltage, DEC);
}
#endif
