ESP32 1–16 Zone Smart Irrigation Controller

A feature-rich, ESP32-based irrigation controller designed to manage 1–16 solenoid zones, with automatic tank / mains water source selection, live weather integration, and a modern local web interface at:

👉 http://espirrigation.local

Built for reliability, flexibility, and real-world garden setups — from small residential systems to multi-zone installations.

✨ Key Highlights

1–16 irrigation zones

Automatic Tank ↔ Mains selection

Live OpenWeather integration

Rain & wind-aware scheduling

Web UI + optional TFT / OLED / LCD displays

KC868-A board support with GPIO fallback*

ESP32-S3 recommended (ESP8266 also supported)

🖥 Supported Hardware Variants
ESP32 + 240×320 SPI TFT

Full-colour on-device interface showing:

System status

Active zones

Rain / wind delays

Water source state

ESP32 + I²C OLED

Compact, low-pin-count display ideal for small enclosures.

ESP8266 + I²C LCD

Lightweight option using common 16×2 or 20×4 LCDs.

All variants share the same firmware features and web UI.

🌦 Weather Integration

Uses the free OpenWeatherMap API:

🔑 Sign up for an API key:
https://home.openweathermap.org/users/sign_up

Live weather data

Temperature

Humidity

Wind speed

Conditions (rain, drizzle, thunderstorm)

Smart delays

Rain delay (sensor or weather-based)

Wind delay (configurable m/s threshold)

Rain cooldown & 24-hour rainfall limits

Rolling rainfall totals (1h / 24h)

📊 Dashboard Features

Tank level (%)

Auto: Tank / Auto: Mains / Forced mode

Live weather snapshot

Next Water

Zone, start time, duration, ETA

Active delay status

Rain / Wind cause

Zone cards

Progress bars

Manual On / Off control

⏱ Zones & Scheduling

1–16 zones

Two start times per zone

Optional second start with separate duration

7-day scheduling

Minute + second precision

Per-zone duration

Overlapping mode

Run sequentially or

Run concurrently (power supply permitting)

Editable zone names

Stored directly on ESP32

🔌 Hardware & I/O

KC868-A support*

PCF8574 @ 0x24 (relays)

PCF8574 @ 0x22 (inputs)

Automatic I²C detection

Falls back to GPIO mode if expanders not found

Configurable pins

Zones, tank, mains, sensors, polarity

Changes applied after reboot

Displays

SPI TFT or I²C OLED (optional)

Backlight control

Photoresistor + 100k resistor (auto-off when enclosure closed)

🌐 Networking & UX

WiFiManager captive portal

SSID: ESPIrrigationAP

mDNS

http://espirrigation.local/

OTA updates

Hostname: ESP32-Irrigation

Event logging

CSV format

Includes weather snapshot per event

Downloadable via web UI

⚙ Behaviour & Safety Logic

Watering is cancelled (and logged) if blocked by:

Rain delay

Wind delay

Master off

Cooldown period

Manual zone activation respects all safety rules

Wind delay behaviour

Scheduled watering waits

Automatically resumes once wind drops below threshold

📦 Requirements

Wi-Fi connectivity

ESP32 board (ESP32-S3 or KC868-A* recommended)

1–16 relay module (if not using KC868)

Tank level sensor

0–3.3 V analog output

Solenoid power supply

~10 W per solenoid

12 V DC or 12/24 V AC

OpenWeatherMap API key

🧰 Typical Materials

KC868-A6 or ESP32 dev board + relay module

1–16 irrigation solenoids

7-core irrigation cable

Tank level sensor

External solenoid power supply

🔧 Typical Wiring

Tie all solenoid returns to supply GND / COM

Feed 12/24 V into each relay COM

Solenoid hot lead → Relay N.O.

Relays 1–4 → Zones 1–4

Relay 5 → Mains valve

Relay 6 → Tank valve

Tank sensor → IO36 (A1) (≤ 3.3 V!)

Rain sensor → IO27 (configurable)

🚀 Flashing & Setup
Arduino IDE Setup

Add ESP32 boards:

https://dl.espressif.com/dl/package_esp32_index.json


Install ESP32 by Espressif Systems

Select board:

ESP32 Dev Module (KC868-A6 compatible)

Partition Scheme

Large APP (4MB)

KC868-A* Library

Download Kincony PCF8574 library:
https://www.kincony.com/forum/attachment.php?aid=1697

📡 First-Run Wi-Fi

Connect to ESPIrrigationAP

Or browse to: http://192.168.4.1

Enter Wi-Fi credentials

Device reboots and joins your network

🌍 Web Endpoints
Path	Description
/	Dashboard
/setup	Configuration
/status	JSON status
/events	Event log
/tank	Tank calibration
/download/events.csv	Event CSV
/i2c-test	Relay test
/stopall	Stop all zones
/valve/on/<z>	Start zone
/valve/off/<z>	Stop zone
/reboot	Reboot

📸 Screenshots
-Main Page

<img width="870" height="939" alt="image" src="https://github.com/user-attachments/assets/067be7c3-5128-4438-ae53-19e2e26ac221" />

-Schedules 

<img width="839" height="722" alt="image" src="https://github.com/user-attachments/assets/d144b3e7-fb4d-49d1-aa05-a41db55fa3db" />

-Setup Page

<img width="662" height="928" alt="image" src="https://github.com/user-attachments/assets/da1fbe08-07a3-42e0-abed-b22fc94b2055" />

-GPIO/Timezone/MQTT Page

<img width="663" height="925" alt="image" src="https://github.com/user-attachments/assets/3f734430-b14d-4861-992b-66c234913597" />
<img width="666" height="259" alt="image" src="https://github.com/user-attachments/assets/61ba1308-9258-4ffe-a550-2ea1b438b1ee" />

-KC868-A6 Wiring

<img width="791" height="754" alt="Screenshot 2025-07-25 233726" src="https://github.com/user-attachments/assets/f601fb06-e70c-4fc6-a7f3-5fdf410b4e73" />

-Wemos R32 w/ 6 Relay module 24vac

![PXL_20251222_121735203~2](https://github.com/user-attachments/assets/d0dc6ca8-cdca-4920-9006-1908e681a619)
<img width="403" height="249" alt="image" src="https://github.com/user-attachments/assets/63f053da-0656-4761-a37d-9579fe1d955f" />




