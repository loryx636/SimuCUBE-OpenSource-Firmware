# SimuCUBE-OpenSource-Firmware
This is the repository for the opensource development of the SimuCUBE firmware. 

See the Wiki page for details. 

https://github.com/SimuCUBE/SimuCUBE-OpenSource-Firmware/wiki

## License

Usage of the firmware files is governed by the Granite Devices End User License Agreement, posted at https://granitedevices.com/wiki/Granite_Devices_Software_End-user_License_Agreement

All contributors must agree to the Granite Devices Contributor License Agreement, posted at https://granitedevices.com/wiki/Granite_Devices_Contributor_License_Agreement

All Granite Devices legal documents, such as the EULA, can be found from https://granitedevices.com/legal

## How to submit your code

Implement your code and features. In you pull request, write following:

    I accept the Granite Devices Contributor License Agreement

Pull requests without this text will not be included in the firmware.

## Programming connector documentation

We use Segger J-link devices, and their connector pinout is published at https://www.segger.com/products/debug-probes/j-link/technology/interface-description/
Simucube has an SWD connection available near the big electrolytic capacitor on the PCB. Its pinout is:
| Pin        | Signal           |
| ------------- |:-------------:|
| 1      | +3.3V |
| 2      | SW_CLK      |
| 3 | GND     |
| 4 | SWD_IO      |
| 5 | NRST      |
| 6 | SWO      |


Pin 1 is the closest one to the big white regenerative braking resistor.
Connecting the first 4 pins is the minimal requirement.
