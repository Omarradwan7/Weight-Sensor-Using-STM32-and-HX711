# Weight-Sensor-Using-STM32-and-HX711

Our aim for this project is to read the weight from the load cell and then convert the reading using the HX711 module. After that we display the readings on Tera Term and then we transmit the readings using TTL to be displayed on the Mobile Phone.

We had to experiment with the weight sensor datasheet’s Gain values, choosing the best fit that provided us with stable weight readings. From 3 options (32, 64, 128), we felt as though having 128 as our Gain enabled us to generate the most accurate/stable readings possible from the load cell. This is not to say that 128 will always be the most suitable, we are merely stating what worked well with us.

Delay function: A simple reusable (applicable to limitless values) delay function that produces a parametrized delay whenever needed.

PowerUp function: An initializing function that wakes up the HX711 module and provides it with starting pin values (zeros).

SetGain function: A flexible function that enables the user to quickly cycle betweeen potential gain values to determine the best suitability.

Initialize function: An encapsulating function that groups the previous two functions together and initializes the HX711 weight sensor.

GetValue function: A crucial function that serves the purpose of retrieving serialized bit values from the HX711 sensor (32 bits).

IsReady function: This function checks whether or not the DOUT pin of the HX711 is set to LOW (read).

Main function: Simply obtain the calibrated weight reading and print it.
