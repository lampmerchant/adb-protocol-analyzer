# ADB Protocol Analyzer

A protocol analyzer for the Apple Desktop Bus (ADB).

## HOWTO

### Requirements

* PIC
   * Microchip MPASM (bundled with MPLAB)
   * A PIC12F1840 microcontroller
   * A device to program the PIC with
* Python
   * Python 3.x
   * PySerial

### Steps

* Build the code using Microchip MPASM and download the result into a PIC12F1840.
* Connect the Tx line to a UART on a PC.
* Run the Python code.

### Example

```
$ python3 -i analyzer.py
>>> import serial
>>> AdbAnalyzer(serial.Serial(port='/dev/ttyS0', baudrate=115200, timeout=1)).run()
```

## Serial Protocol

* Uses 8-N-1 at 115200 baud.
* Host sends the ADB command bytes as received.
   * For a Talk (0bXXXX11XX) or Listen (0bXXXX10XX) command, this is followed with:
      * Payload length (1 byte, should be 2-8 but this is not enforced)
      * Payload
* Reset conditions are treated the same as SendReset commands.
