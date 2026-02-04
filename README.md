# 🌱 ESP32 1–16 Zone Smart Irrigation Controller

**ESP32-based irrigation controller** designed to manage **1–16 solenoid zones**, with automatic **tank / mains water source selection**, **live weather integration**, and a modern **local web interface**:

👉 **[http://espirrigation.local](http://espirrigation.local)**

Built for **reliability, flexibility, and real-world garden setups** — from small residential systems to large multi-zone installations.

---

## ✨ Highlights

* **1–16 irrigation zones**
* **Automatic Tank ↔ Mains water source selection**
* **Rain & wind-aware scheduling**
* **Modern web UI**
* **Optional TFT / OLED / LCD displays**
* **KC868-A6 board support with GPIO fallback**
* **ESP32-S3 recommended (KC868-A6 also supported)**

---

## 🖥 Supported Hardware Variants

### ESP32 + 240×320 SPI TFT

Full-colour on-device interface showing:

* System status
* Active zones
* Rain / wind delays
* Water source state

### ESP32 + I²C OLED

Compact, low-pin-count display ideal for small enclosures.

### ESP8266 + I²C LCD

Lightweight option using common **16×2 or 20×4 LCDs**.

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

## ⏱ Zones & Scheduling

* **1–16 zones**
* **Two start times per zone**

  * Optional second start with separate duration
* **7-day scheduling**
* **Minute + second precision**
* **Per-zone duration**
* **Overlapping modes**

  * Sequential (default)
  * Concurrent (power supply permitting)
* **Editable zone names**

  * Stored directly on ESP32

---

## 🔌 Hardware & I/O

* **KC868-A support**

  * PCF8574 @ `0x24` (relays)
  * PCF8574 @ `0x22` (inputs)
* **Automatic I²C detection**

  * Falls back to GPIO mode if expanders not found
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

Manual zone activation **respects the same safety rules**.

### Wind Delay Behaviour

* Scheduled watering waits while wind exceeds threshold
* Automatically resumes once wind drops below limit

---

## 📦 Requirements

* Reliable **Wi-Fi connection**
* **ESP32 board** (ESP32-S3 or KC868-A recommended)
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

<img width="870" height="939" alt="image" src="https://github.com/user-attachments/assets/067be7c3-5128-4438-ae53-19e2e26ac221" />

-Schedules 

<img width="839" height="722" alt="image" src="https://github.com/user-attachments/assets/d144b3e7-fb4d-49d1-aa05-a41db55fa3db" />

-Setup Page

<img width="931" height="878" alt="image" src="https://github.com/user-attachments/assets/f0803af9-1f69-41b2-a368-bd27c94870fd" />


-GPIO/Timezone/MQTT Page

<img width="663" height="925" alt="image" src="https://github.com/user-attachments/assets/3f734430-b14d-4861-992b-66c234913597" />
<img width="666" height="259" alt="image" src="https://github.com/user-attachments/assets/61ba1308-9258-4ffe-a550-2ea1b438b1ee" />

-KC868-A6 Wiring

<img width="791" height="754" alt="Screenshot 2025-07-25 233726" src="https://github.com/user-attachments/assets/f601fb06-e70c-4fc6-a7f3-5fdf410b4e73" />

-Esp32 w/TFT 8 Relay Module 

<img width="610" height="354" alt="image" src="https://github.com/user-attachments/assets/03cd3421-9d22-4272-9b58-5f9a08d18479" />

<img width="403" height="249" alt="image" src="https://github.com/user-attachments/assets/63f053da-0656-4761-a37d-9579fe1d955f" />
