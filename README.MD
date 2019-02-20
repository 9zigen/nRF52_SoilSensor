###Program

1. Download NRF52 SDK 15.2 [link](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download#infotabs)
2. Clone repo
3. Fix path to sdk in Makefile:

``SDK_ROOT := ..``

``PROJ_DIR := .``

4. Flash Softdevice
`` make flash_softdevice ``
5. Flash App
`` make flash ``