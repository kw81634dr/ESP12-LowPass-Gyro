# SimpleGyro

simple gyroscope with digital low pass filter

![Demo_gif](https://github.com/kw81634dr/SimpleGyro/blob/main/images/OLED_Demo.gif)

## Display Mode

![DIsplayMode_img](https://github.com/kw81634dr/SimpleGyro/blob/main/images/DisplayMode.png)

from left to right: Arrow, Bubble, Line

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

backup in `library_backup` directory
`I2cdev`, `MPU6050`, `ThingPulse OLED SSD1306`

## IIR Low pass filter

[Source: *Low pass filter algorithm origin*](https://dsp.stackexchange.com/questions/41854/low-pass-filter-algorithm-origin)

```c
y += beta * (x - y);
```

>It can be converted to an algebraic relation between the samples of the filter output sequence *𝑦[𝑛]* and the filter input *𝑥[𝑛]* as:
`𝑦[𝑛+1]−(1−b)𝑦[𝑛]=b𝑥[𝑛]`which is also equivalent to`𝑦[𝑛]−(1−b)𝑦[𝑛−1]=b𝑥[𝑛−1]`

![Freq_response_fig](https://i.stack.imgur.com/fnQMX.png)

---
Update: Sep.28, 2018
