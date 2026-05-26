# 🖱️ AIR MOUSE
### Gesture-Controlled Mouse Using STM32F4 and IMU-Flex Sensor Interface

---

## 📸 Project Image
![Air Mouse Setup](./your-image-her.jpg)

---


## 📌 Overview

AIR MOUSE is an embedded systems project that lets you control your computer's mouse cursor using hand gestures — no surface needed. An STM32F407G microcontroller reads motion data from an MPU-6050 IMU and two flex sensors worn on the fingers. The orientation data (roll & pitch) drives cursor movement, while finger bends trigger left and right clicks. All data is transmitted wirelessly to a PC via a Bluetooth HC-05/06 module, where a Python script translates it into real mouse actions.

---

## 🧰 Hardware Components

- STM32F407G-DISC1 Discovery Board
- MPU-6050 IMU (Accelerometer + Gyroscope)
- 2× Flex Sensors
- HC-05/06 Bluetooth Module
- 2× 10kΩ Resistors (pull-up for I2C)
- 2× 10kΩ Resistors (voltage divider for flex sensors)
- Breadboard & Jumper Wires

---

## 🔌 Wiring / Connections

### MPU-6050 → STM32
| MPU-6050 Pin | STM32 Pin | Notes |
|---|---|---|
| SCL | PB8 | 10kΩ pull-up to 3.3V |
| SDA | PB9 | 10kΩ pull-up to 3.3V |
| VCC | 3.3V | — |
| GND | GND | — |
| AD0 | GND | Sets I2C address to 0x68 |

### Flex Sensor 1 (Left Click) → STM32
| Connection | Pin |
|---|---|
| One end | 5V |
| Middle wiper | PA1 |
| Other end | 10kΩ → GND |

### Flex Sensor 2 (Right Click) → STM32
| Connection | Pin |
|---|---|
| One end | 5V |
| Middle wiper | PA5 |
| Other end | 10kΩ → GND |

### Bluetooth HC-05/06 → STM32
| HC-05/06 Pin | STM32 Pin |
|---|---|
| VCC | 5V |
| GND | GND |
| RXD | PA2 |
| TXD | PA3 |

---

## ⚙️ How It Works

1. The STM32 reads raw accelerometer and gyroscope data from the MPU-6050 over I2C.
2. A complementary filter (96% gyro + 4% accelerometer) computes stable **roll** and **pitch** angles.
3. Flex sensor ADC readings on PA1 and PA5 detect finger bends for **left click** and **right click**.
4. The STM32 transmits data over UART to the HC-05/06 Bluetooth module in the format:
   ```
   roll,pitch,flex1,flex2\n
   LCLICK\n
   RCLICK\n
   ```
5. The Python script (`mouse_control.py`) receives this over a serial COM port and maps it to mouse movements and clicks using `pyautogui`.

---

## 🖥️ Software Setup (PC Side)

### Requirements
```bash
pip install pyserial pyautogui
```

### Configuration
Open `mouse_control.py` and update the COM port to match your Bluetooth adapter:
```python
PORT = "COM10"   # Change this to your port (e.g., COM3, /dev/rfcomm0)
BAUD = 9600
```

### Run
```bash
python mouse_control.py
```

## 🔧 Firmware Details (`main.c`)

| Peripheral | Configuration |
|---|---|
| Clock | 84 MHz (PLL from HSE) |
| I2C1 (PB8/PB9) | MPU-6050 communication |
| ADC1 (PA1, PA5) | Flex sensor readings |
| USART2 (PA2/PA3) | Bluetooth serial output |
| LED PD12 | Left click indicator |
| LED PD13 | Right click indicator |

---

## 📝 Notes

- The IMU is **calibrated on startup** — keep the device still for the first few seconds after powering on.
- Cursor sensitivity and deadzone can be tuned in `mouse_control.py` via `SENSITIVITY`, `SCALE`, `DEADZONE_MIN`, and `DEADZONE_MAX`.
- To enable high-speed Bluetooth (115200 baud), set `ENABLE_HIGH_SPEED_BT 1` in `main.c` (requires HC-05 AT mode configuration first).
