# SimpleGyro

simple gyroscope with digital low pass filter

![Demo_gif](https://github.com/kw81634dr/SimpleGyro/blob/main/images/OLED_Demo.gif)

## Display Mode

+ ![Arrow_img](https://github.com/kw81634dr/SimpleGyro/blob/main/images/Arrow.jpg)
+ ![Bubble_img](https://github.com/kw81634dr/SimpleGyro/blob/main/images/bubble_level.jpg)
+ ![Line_img](https://github.com/kw81634dr/SimpleGyro/blob/main/images/line.jpg)

## Hardware

+ MCU(Esp12(esp8266))
+ Gyro module(MPU6050)
+ OLED(SSD1306, I2C)

## Wiring

| ESP12 PIN | GPIO# | Description                |
|-----------|-------|----------------------------|
| D1        | 5     | SCL                        |
| D2        | 4     | SDA                        |
| D3        | 0     | Calibration trigger Button |
| D5        | 14    | LowPass Filter Switch      |
| D7        | 13    | Display Mode switch button |

## Develop Enviroment

+ Arduino

## Dependent libraries

>backup in `library_backup` directory

`I2cdev`, `MPU6050`, `ThingPulse OLED SSD1306`

---
Update: Sep.28, 2018
