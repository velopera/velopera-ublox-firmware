# velopera-ublox-firmware

# VELOpera U-Blox Firmware

The u-blox Nina is based on an Espressif core, the development environment is based upon Espressif IDF, integrated into Visual Studio Code.

Project URL: https://github.com/velopera/velopera-ublox-firmware

## Software dependencies
Install Visual Studio Code
https://code.visualstudio.com/
Install Espressif IDF VS Code Extension
https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md

After installing the required development environment and downloading the example project, we are ready to build and run the example project.
## How to build and run the example project
 To build and run the example project follow these steps:
  1. Open Visual Studio Code and open Espressif IDF Extension’s welcome page.
  2. Hit the “Import project” button on the welcome page and browse the directory of the example project.
  3. Open imported folder in the Visual Studio Code.
  4. Click the “ESP-IDF Build, Flash and Monitor” button on the “Status Bar”.


## Project Description
 
The velopera-ublox-firmware is based on one main application and four components that do their corresponding job. The following diagram visualises the relationship between components and main application.

![image](https://github.com/user-attachments/assets/e5638d00-f2a3-4643-9346-daa324d37806)


### CAN Interface

CAN interface based on “can_velopera” and “can_data_handler” components. The “can_data_handler” component is an abstract class which abstracts CAN data handling processes between main application and CAN data handlers of different types of bikes. In this example “can_data_handler” component abstracts “can_velopera” CAN data handler.

The “can_velopera” CAN data handler, handles the filtering process of Velopera CAN frames.  

In this example Two Wire Automotive Interface (TWAI) API of Espressif used to implement the CAN data transmission.

### I2C Interface

In this example I2C interface communicates with the mc6470 magnetometer sensor. The I2C driver API of Espressif used to implement the communication process with the mc6470 magnetometer sensor. 

The mc6470 component  is used to handle the initialization and compass data fetching processes.

UART Transceiver

The example project uses INT-COM pins of the board to communicate with the nRF9160 module. The “UART Transceiver” application transmits the CAN and Compass data as a JSON object via UART. E.G.
 

In this example UART driver API of Espressif used to implement the UART application.

### GATT Server

The example project provides a GATT Server advertisement feature. The “gatts” component handles the implementation of the GATT Server. The GATT Server advertises the bike name (Velopera).

In this example GATTS API of Espressif used to implement the “gatts” component.

The following sequence diagram visualises the most significant chain of events during operation of the example project:

![image](https://github.com/user-attachments/assets/efd807e0-f28d-4801-8b2e-a3e1c65aa25c)


## Testing

After programming the example project to the uBlox Nina module, click the “ESP-IDF Monitor Device” button on the status bar to open the serial monitor to see the debug output of the example project.

## Useful Links

#### Espressif IDF documentation
https://idf.espressif.com/
#### uBlox Nina W10x Module page 
https://www.u-blox.com/en/product/nina-w10-series-open-cpu
#### System integration manual - chapter 2.5
https://content.u-blox.com/sites/default/files/NINA-W1_SIM_UBX-17005730.pdf
#### ESP ultimate guide documentation
https://www.espressif.com/sites/default/files/documentation/ESP32-C3WirelessAdventure.pdf


---

*This work has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 101070599 (SecOPERA)*
