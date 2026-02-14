# 🌱 ESP32 1–16 Zone Smart Irrigation Controller

**ESP32-based irrigation controller** designed to manage **1–16 solenoid zones**, with automatic **tank / mains water source selection**, **live weather integration**, and a modern **local web interface**:

👉 **[http://espirrigation.local](http://espirrigation.local)**

Built for **reliability, flexibility, and real-world garden setups** — from small residential systems to large multi-zone installations.

---

## ✨ Features

* **1–16 irrigation zones (Optional)**
* **Automatic Tank ↔ Mains water source selection (Optional)**
* **Rain & wind-aware scheduling (Optional)**
* **Modern web UI**
* **Weather Display**
* **Optional TFT / OLED / LCD displays**
* **ESP32/ESP32-S3 recommended (KC868-A6/A8 also supported)**

---

## ⏱ Zones & Scheduling

* **1–16 zones**
* **Two start times per zone**

  * Optional second start with separate duration
* **7-day scheduling**
* **Minute + second precision**

  * Sequential (default)
  * All at Once (power supply permitting)
* **Editable zone names**

  * Stored directly on ESP32

---

## 🖥 Supported Hardware Variants

### ESP32 + 240×320 SPI TFT

Full-colour device interface showing:

* System status
* Active zones
* Rain / wind delays
* Water source state

### ESP32 + I²C OLED

Compact, low-pin-count display ideal for small enclosures.
Select screen type in setup. 

### ESP8266 + I²C LCD

Lightweight verion using **16×2 LCD**.

---

## 🌦 Weather Integration

* Goto: https://open-meteo.com/en/docs to get lat and long.
* Enter your Latitude and longitude in Setup.
* Save.  
<img width="1030" height="173" alt="image" src="https://github.com/user-attachments/assets/a4094761-555c-4059-bfe3-a7f00c617894" />

### Live Weather Data

* Temperature
* Feels like
* Humidity
* Wind speed
* Rain mm 1hr/24hr
* High/Low
* Pressure
* SunUp/Down
* Condition (Rain / Clear / Thunderstorm ect.)

### Smart Delays

* Rain delay (sensor or weather-based)
* Wind delay (configurable m/s threshold)
* Rain cooldown and 24-hour rainfall limits
* Rolling rainfall totals (1h / 24h)

---

## 📊 Dashboard Features

* **Tank level (%)**

  * Auto: Tank / Auto: Mains / Forced
* **Live weather snapshot**
* **Next Water**

  * Zone, start time, duration, ETA
* **Active delay status**

  * Rain / Wind cause
* **Zone cards**

  * Progress bars
  * Manual On / Off control

---

## 🔌 Hardware & I/O

* **KC868-A6/8 support**

  * PCF8574 @ `0x24` (relays)
  * PCF8574 @ `0x22` (inputs)
* **Automatic I²C Relay detection (for KC868)**

  * Falls back to GPIO mode for ESP32/ESP32-s3 if I²C Relay expanders/KC868 not found
* **Fully configurable pins**

  * Zones, tank, mains, sensors, polarity
  * Changes applied after reboot
* **Displays**

  * SPI TFT or I²C OLED (optional)
* **Backlight control**

  * Photoresistor + 100k resistor
  * Auto-off when enclosure door is closed

---

## 🌐 Networking & UX

* **WiFiManager captive portal**

  * SSID: `ESPIrrigationAP`
* **mDNS**

  * [http://espirrigation.local/](http://espirrigation.local/)
* **OTA updates**

  * Hostname: `ESP32-Irrigation`
* **Event logging**

  * CSV format
  * Includes weather snapshot per event
  * Downloadable via web UI

---

## ⚙ Behaviour & Safety Logic

Watering is **cancelled and logged** if blocked by:

* Rain delay
* Wind delay
* Master off
* Cooldown period

Manual zone activation **respects the same rules**.

### Rain Delay Behaviour 

* scheduled watering cancelled if triggered while condition says raining, "cooldown" meaning period after eg. If rainfall is 5mm all watering is cancelled for 48 hrs.


### Wind Delay Behaviour

* Scheduled watering waits while wind exceeds threshold and resumes once wind drops below limit.

---

## 📦 Requirements

* Reliable **Wi-Fi connection**
* **ESP32 board** (ESP32/ESP32-S3 or KC868-A6/8 recommended)
* **TFT 240x320 w/ BL pin (for screen sleep) or OLED Screen 
* **1–16 relay module** (if not using KC868)
* **Tank level sensor**

  * 0–3.3 V analog output
* **Solenoid power supply**

  * ~10 W per solenoid
  * 12 V DC or 12/24 V AC

---

## 🧰 Typical Materials

* KC868-A6 **or** ESP32 dev board + relay module
* 1–16 irrigation solenoids
* 7-core irrigation cable
* Tank level sensor
* External solenoid power supply

---

## 🔧 Typical Wiring

* Tie all solenoid returns to supply **GND / COM**
* Feed **12/24 V** into each relay **COM**
* Solenoid hot lead → **Relay N.O.**
* Relays 1–4 → Zones 1–4
* Relay 5 → Mains valve
* Relay 6 → Tank valve
* Tank sensor → **IO36 (A1)** *(≤ 3.3 V!)*
* Rain sensor → **IO27** *(configurable)*

---

## 🚀 Flashing & Setup

### Arduino IDE Setup

Add ESP32 boards:

```text
https://dl.espressif.com/dl/package_esp32_index.json
```

Install **ESP32 by Espressif Systems**

Select board:

* **ESP32 Dev Module** (KC868-A6 compatible)
* **ESP32S3 Dev Module** 

Set partition scheme:

```text
Large APP (4MB)
```

---

### KC868-A Library

Download Kincony PCF8574 library:
[https://www.kincony.com/forum/attachment.php?aid=1697](https://www.kincony.com/forum/attachment.php?aid=1697)

---

## 📡 First-Run Wi-Fi

1. Connect to **ESPIrrigationAP**
2. Or browse to: [http://192.168.4.1]
3. Enter Wi-Fi credentials
4. Device reboots and joins your network

---

## 🌍 Web Endpoints

| Path                   | Description       |
| ---------------------- | ----------------- |
| `/`                    | Dashboard         |
| `/setup`               | Configuration     |
| `/status`              | JSON status       |
| `/events`              | Event log         |
| `/tank`                | Tank calibration  |
| `/download/events.csv` | Event CSV         |
| `/i2c-test`            | Relay test        |
| `/stopall`             | Stop all zones    |
| `/valve/on/<z>`        | Start zone        |
| `/valve/off/<z>`       | Stop zone         |
| `/reboot`              | Reboot controller |

---

## 📸 Screenshots

-Main Page

<img width="641" height="919" alt="image" src="https://github.com/user-attachments/assets/fa40d71c-d27f-44aa-9ebd-2128cacf388c" />

-Schedules 

<img width="846" height="687" alt="image" src="https://github.com/user-attachments/assets/6393f64d-017c-4672-887b-5ffccd7a34d4" />

-Setup Page

<img width="765" height="775" alt="image" src="https://github.com/user-attachments/assets/c02c13c2-0407-45e4-9973-79321f7280ac" />


-KC868-A6 Wiring

<img width="791" height="754" alt="Screenshot 2025-07-25 233726" src="https://github.com/user-attachments/assets/f601fb06-e70c-4fc6-a7f3-5fdf410b4e73" />

-Esp32 w/TFT 8 Relay Module 

<img width="610" height="354" alt="image" src="https://github.com/user-attachments/assets/03cd3421-9d22-4272-9b58-5f9a08d18479" />

<img width="403" height="249" alt="image" src="https://github.com/user-attachments/assets/63f053da-0656-4761-a37d-9579fe1d955f" />
