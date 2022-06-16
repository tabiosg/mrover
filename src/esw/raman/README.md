Scripts that forward Raman data using socat 
======================================================================================
### About
The Raman spectrometer is used during the science mission
to detect kerogens and carotenoids.
The CCD is a sensor and is the portion of the Raman setup that collects light,
working together with the Raman laser. The CCD is connected to the STM32F401RE Nucleo. 
The Nucleo is connected to the Jetson via USB. 
The Jetson script forwards data from the Nucleo
directly to the base station using socat such that the data
can be accessed on the base station via the Raman gui (pyCCDGUI).

### Hardware
- NVIDIA Jetson Xavier
- STM32401RE
- TCD1304 Linear CCD
- Raman laser

### Overview
jetson_raman.sh should run on the jetson.
science_raman.sh should run on the science laptop.
One must make sure that the 99-usb-serial.rules is copied to /etc/udev/rules.d/99-usb-serial.rules too.

## TODO
- [ ] Make sure that these files are in the proper place
- [ ] Make sure scripts run on startup/launch
- [ ] Make sure scripts keep running if Raman nucleo gets plugged in afterward (should not error)
- [ ] Test with ROS
- [ ] MAJOR - I have no idea how 99-usb-serial.rules works. Apparently it needs to be copied to /etc/udev/rules.d/99-usb-serial.rules somehow on startup/launch. Also, it would probably make sense if placed in a different folder.