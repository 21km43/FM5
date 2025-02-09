# F機復元PJ（計測系統）

## Hardware

* [M5Stack CoreS3 SE](https://www.switch-science.com/products/9690)
* [M5Stack Core2用ポート拡張モジュール](https://www.switch-science.com/products/8308)
* [M5Stack用GNSSモジュール 気圧/IMU/地磁気センサ付き（NEO-M9N/BMP280/BMI270/BMM150）](https://www.switch-science.com/products/9276?_pos=2&_sid=b1554245d&_ss=r)
* microSD 16GB

## Software

* [PlatformIO](https://platformio.org)
* [VSCode](https://code.visualstudio.com)

## CoreS3 SE Pinout

* Port A ... G1, G2 (Air Speed)
* Port B ... G8, G9 (Altitude)
* Port C ... G18, G17 (LoRa)
* Port D ... G14, G10 (G14 = Toggle RS485 inout for altitude sensor, G10 = Propeller Rotation Speed)
* Port E ... G13, G5 (Control board)

GNSS

* PPS ... G0 (Not used)
* TX ... G6
* RX ... G7
