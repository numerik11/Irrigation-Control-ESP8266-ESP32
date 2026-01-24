## ESP32 / ESP8266 1-16 Zone Irrigation Controller

ESP32 or ESP8266–based irrigation controllers designed to drive up to 16 solenoid zones.

The system supports automatic tank/mains water source selection, live weather integration with rain and wind delays, and manual zone control, all managed through a local web interface at espirrigation.local.

1. ESP32 with 170×320 SPI TFT display — full-colour device interface for system status, rain/wind delays and active zones

2. ESP32 with I²C OLED display — compact, low-pin-count display for smaller enclosures

3. ESP8266 with I²C LCD — common, compact, low-pin-count display for smaller enclosures

All versions support two programmable start times per day, 7-day scheduling, and minute/second-level run times, suitable for both small gardens and multi-zone irrigation setups.

---

Free OpenWeatherMap API key → https://home.openweathermap.org/users/sign_up

---

## Features

- **Dashboard**
  - Tank level (%) with `Auto:Mains` / `Auto:Tank` / `Force` state
  - Live weather (OpenWeather Current): temperature, humidity, wind, condition
  - **Next Water**: next scheduled run (zone, start, ETA, duration)
  - Delay for rain & wind with cause
  - Zone cards with progress and manual On/Off

- **Zones & Schedules**
  - 4-zone mode (Zones 1–4) **+** Mains/Tank master valves on relays 5 & 6 (For KC868-A6)  
  - 1-16 zone mode (If using GPIO and Relay module, A6 Can be used for 6 zone)
  - Two start times per zone (optional Start 2), per-day enable, minute precision
  - Per-zone duration (minutes + seconds)
  - Overlapping starts will run one at a time or together choose in setup (running together be sure you have sufficent power-supply for multiple solenoid coils). 
  - Editable zone names stored in esp32

- **Delays & Sensors**
  - Rain sources: physical sensor (invert option) + weather conditions (Rain/Drizzle/Thunderstorm or rain amount)
  - Wind delay: configurable threshold (m/s)
  - Rain cooldown and 24h threshold (mm)
  - Rolling sum actual rainfall stats (1h / 24h)

- **Hardware & I/O**
  - KC868-A6 support (PCF8574 @ 0x24 relays, 0x22 inputs)
  - Automatic I²C health + debounce → GPIO fallback for generic ESP32 boards
  - All zone/mains/tank, high/low and pins configurable in Setup (Reboot after chaging pins)
  - OLED status screens (Home / Rain Delay)

- **Networking & UX**
  - WiFiManager captive portal: `ESPIrrigationAP` (first boot/failure)
  - mDNS: `http://espirrigation.local/`
  - Event logger to CSV (weather snapshot per event, downloadable)

---

## Requirements

- **Board:** Any ESP32 Module (KC868-A6 recommended)
- ** 6-relay module ** (If mot using KC868-A6)
- **Tank level Sensor - analog pin:** IO36 (A1) (≤ 3.3V ADC)
- **Rain sensor:** IO27 (optional)
- **Power:** Matches your solenoids (e.g., 12V DC or 12/24V AC)
- **Weather API:** Free OpenWeather API key → https://home.openweathermap.org/users/sign_up

---

## Materials

- KC868-A6 (recommended) or ESP32 dev board and a 6-relay module  
- 6 irrigation solenoids (12V DC or 24V AC to match power Source)  
- 7-core irrigation cable to the solenoid pit/box  
- Tank level sensor with 0–3.3V output to ESP32 ADC
- Power Source for solenoids (AC12v/24v,DC12/24v) 

---

## Wiring (Typical)

- Tie all solenoid returns to supply GND/COM.  
- Feed 12/24V into each relay COM; solenoid hot lead to N.O.  
- Relays 1–4 → Solenoids 1–4  
- Relay 5 → **Mains** solenoid, Relay 6 → **Tank** solenoid (4-zone master valves)  
- Tank level sensor → IO36 (A1). **Do not exceed 3.3V.**  
- Rain sensor → IO27 (configurable)

---

## Flashing the Controller

1. **Install ESP32 Boards (Arduino IDE)**  
   *File → Preferences → Additional Boards URLs:*
   Paste:
   `https://dl.espressif.com/dl/package_esp32_index.json`
   Then Goto:
   *Tools → Board → Boards Manager… →* install **ESP32 by Espressif Systems**.

3. **Select a Board**  
   *Tools → Board →* **ESP32 Dev Module** (works for KC868-A6)  
   Suggested: Flash 80 MHz, Upload 115200–921600.
   
   - Important!! Set Partition Scheme.

!!!!Tools → Partition Scheme: Large APP (4MB).!!!!
   
4. **(KC868-A6) PCF8574 Library**  
   Download Kincony PCF8574 Library .zip: <https://www.kincony.com/forum/attachment.php?aid=1697>  
   *Sketch → Include Library → Add .ZIP Library…*

5. **Upload**  
   Open the esp32irrigation.ino sketch and click **Upload**.

6. **First-Run Wi-Fi**  
   Serial Monitor @ 115200. Connect to **ESPIrrigationAP** (captive portal),  
   or browse to **http://192.168.4.1** and set your Wifi SSID + password.

7. **Access & Configure**  
   Goto   http://espirrigation.local/ or assigned local IP to accsess home page, OLED shows assigned IP on startup.  
   Goto - **Setup**: enter OpenWeather API Key + City ID, set timezone, use zones (4 or 6), wind enable?/rain enable?, GPIO pins.
   Goto - **Home**: set days, start times, durations per zone.

---

## Behaviour (Scheduling & Delays)

- Starts during **rain**, **pause**, **master off**, or **rain cooldown** → **CANCELLED** (logged)  
- Manual “On” commands also respect the same rules and cancel when blocked  
- “Next Water” still reflects the next eligible schedule (no queueing)
- Wind delay, when enabled, scheduled watering is automatically postponed until wind speed falls below the configured threshold, then starts automaticly.
  
---

## Useful Endpoints

- `/` — Dashboard  
- `/setup` — Setup page (API keys, zones, GPIO, sensors, delays, MQTT, timezone)  
- `/status` — JSON snapshot (device time/TZ, Next Water, zones, tank, weather)  
- `/events` — Event log (table)  
- `/tank` — Tank calibration (Set Empty/Full)  
- `/download/config.txt` — Download raw config  
- `/download/schedule.txt` — Download schedule  
- `/download/events.csv` — Download event log CSV  
- `/i2c-test` — I²C relay pulse test  
- `/api/time` — Local/UTC time probe  
- `/whereami` — IP/SSID/RSSI/Mode  
- `/reboot` — Reboot controller  
- `/stopall` — Stop all running zones  
- `/valve/on/<z>` — Manual start zone `<z>` (0-based)  
- `/valve/off/<z>` — Manual stop zone `<z>` (0-based)

---

## Notes

- **Tank ADC:** default IO36 (A1). 
- **mDNS:** `http://espirrigation.local/` after joining your Wi-Fi.  
- **OTA:** enabled (hostname `ESP32-Irrigation`).  
- **Fallback:** If I²C expanders from KC868 aren’t detected, GPIO fallback is enabled and pins for other ESP32 boards from Setup are used.
- **Any issues, questions or feedback please send me a message.

## Links

- KC868-A6: <https://www.kincony.com/esp32-6-channel-relay-module-kc868-a6.html>  
- OpenWeatherMap: <https://openweathermap.org>

---

## Screenshots

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

