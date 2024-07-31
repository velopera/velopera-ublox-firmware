# velopera-ublox-firmware

The velopera-ublox-firmware is based on one main application and four components that do their corresponding job. The following diagram visualises the relationship between components and main application.


CAN Interface

CAN interface based on “can_velopera” and “can_data_handler” components. The “can_data_handler” component is an abstract class which abstracts CAN data handling processes between main application and CAN data handlers of different types of bikes. In this example “can_data_handler” component abstracts “can_velopera” CAN data handler.

The “can_velopera” CAN data handler, handles the filtering process of Velopera CAN frames.  

In this example Two Wire Automotive Interface (TWAI) API of Espressif used to implement the CAN data transmission.

I2C Interface

In this example I2C interface communicates with the mc6470 magnetometer sensor. The I2C driver API of Espressif used to implement the communication process with the mc6470 magnetometer sensor. 

The mc6470 component  is used to handle the initialization and compass data fetching processes.

UART Transceiver

The example project uses INT-COM pins of the board to communicate with the nRF9160 module. The “UART Transceiver” application transmits the CAN and Compass data as a JSON object via UART. E.G.
 

In this example UART driver API of Espressif used to implement the UART application.

GATT Server

The example project provides a GATT Server advertisement feature. The “gatts” component handles the implementation of the GATT Server. The GATT Server advertises the bike name (Velopera).

In this example GATTS API of Espressif used to implement the “gatts” component.

The following sequence diagram visualises the most significant chain of events during operation of the example project:





