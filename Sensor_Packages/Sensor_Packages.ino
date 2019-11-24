/* Author: Michael Barbosa - Aguayo
 * Date: 10/21/2019
 * Project: EECS 159 Senior Design
 * 
 * Description:
 *    This file contains sample code for reading a temperature
 *    sensor from an Arduino using a ZTP-115M temerature sensor.
 *    Just look at the comments to see what the code does.
 *    
 *    Everything is outputted through the Serial Monitor.
 *    
 * EDITS:
 *    
 */




// Pin Variables

int tempPin_1 = 19;   // The analog pin ZTE-155's Vout (sense) pin is connected here
int tempPin_2 = 20;   // The resolution is 10mV / degrees centigrade with a 
int tempPin_3 = 21;   // 500mV offset to allow for negative temeratures
int tempPin_4 = 22;   // this pin will serve as the BRAKE temp sensor
int wSpeed    = 17;   // wheel speed pin variable


/*
 * setup() - this function runs once when you turn your Arduino on
 * We initialize the serial connection with the computer
 */
void setup()
{
  Serial.begin(9600); // Start the serial connection with the computer
                      // To view the result open the serial monitor
}

void loop()
{
  // getting the voltage reading from the temp sensor
  int volt_1 = analogRead(tempPin_1);
  int volt_2 = analogRead(tempPin_2);
  int volt_3 = analogRead(tempPin_3);
  int volt_4 = analogRead(tempPin_4);

  // Converting that reading to a voltage, for 3.3v arduino use 3.3
  //float voltage = volt_1 * 5.0;
  //voltage /= 1024.0;

  // Print out the voltage
  Serial.print(volt_1); Serial.println(" volts");
  Serial.print(volt_2); Serial.println(" volts");
  Serial.print(volt_3); Serial.println(" volts");
  Serial.print(volt_4); Serial.println(" volts");

  
  // Now print out the tempurature
  //float temperatureC = (voltage - 0.5) *100;  // Converting from 10mV per degree wit 500mV offset

  // to degrees  (( voltage - 500mV) times 100)

  // Now convert to fahrenheit
  //float temperatureF = (temperatureC *9.0 / 5.9) + 32.0;

  //Serial.print(temperatureF); Serial.println(" Degrees F");

  delay(1000);  // Waiting for one second.
}
