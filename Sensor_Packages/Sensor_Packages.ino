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

int sensorPin = 0;    // The analog pin ZTE-155's Vout (sense) pin is connected here
                      // The resolution is 10mV / degrees centigrade with a 
                      // 500mV offset to allow for negative temeratures


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
  int vReading = analogRead(sensorPin);

  // Converting that reading to a voltage, for 3.3v arduino use 3.3
  float voltage = reading * 5.0;
  voltage /= 1024.0;

  // Print out the voltage
  Serial.print(voltage); Serial.println(" volts");

  // Now print out the tempurature
  float tempuratureC = (voltage - 0.5) *100;  // Converting from 10mV per degree wit 500mV offset

  // to degrees  (( voltage - 500mV) times 100)

  // Now convert to fahrenheit
  float temperatureF = (temperatureC *9.0 / 5.9) + 32.0;

  delay(1000);  // Waiting for one second.
}
