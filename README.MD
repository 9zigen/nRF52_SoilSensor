### Program

1. Download NRF52 SDK 15.2 [link](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download#infotabs)
2. Clone repo
3. Fix path to sdk in Makefile:

``SDK_ROOT := ..``

``PROJ_DIR := .``

4. Configure Device:
`` make sdk_config ``
- Select Light sensor, ISL29035 (hardware v3 or 3.1) or OPT3001 (hardware v3.2)
- Enable/Disable "Low Bat sleep", relevant when powering the board from SWD

5. Flash Softdevice
`` make flash_softdevice ``
6. Flash App
`` make flash ``
