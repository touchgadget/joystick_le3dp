# Logitech Extreme 3D Pro flight joystick

Demo Arduino program that reads axes and buttons over USB and prints them on
the serial console. It works with the Adafruit Feather RP2040 with USB Type A
Host board.

Sample output. The first line is a hex dump of the binary HID report.
The second line decodes the binary data to useful values.

```
LE3DP report(7): 0xFC 0xF1 0x87 0x80 0x00 0x99 0x00
X:508,Y:508,hat:8,twist:128,slider:153,buttons_a:0x0,buttons_b:0x0
```

Additional information to decode the values.
```
N=North, S=South, W=West, E=East

X : (W)0..511..1023(E)
Y : (N)0..511..1023(S)
hat : 0=N,1=NE,2=E,3=SE,4=S,5=SW,6=W,7=NW,8=center
twist: 0=counter clock wise, 128=center, 255=clock wise
slider: 0=N..255=S
```
