# ESP32 Smart Display + FAN Controller

**OLED + ESP32 + MQTT (HA) + DS18B20 + FAN (MOSFET, hysteresis) + Solar power**

> A complete IoT project on **ESP32**: **SSD1306 128×64** OLED display, **Home Assistant** integration via **MQTT**, local **DS18B20** temperature measurement, automatic fan control with hysteresis, and **solar power** with **TP4056** Li-Ion charging. Designed as a **portfolio project** to showcase skills in electronics, microcontroller programming, and IoT integration.

---

## 📌 Project Goal

The project was built to strengthen my knowledge in electronics and ESP32 programming. I created a self-contained device in an enclosure with an OLED and all necessary peripherals. The display shows temperature, humidity, and pressure from **Home Assistant** (via MQTT). Internal temperature is monitored with **DS18B20**, while a cooling fan is driven by the ESP32 through an **N-MOSFET** with hysteresis to prevent overheating.

---

## ✨ Key Features

- **OLED UI (SSD1306 128×64, I²C)** – 3 rows, 2 columns (see below).
- **MQTT → OLED**: temperature, humidity, pressure (`esp32/weather/*`).
- **DS18B20 → MQTT**: publishes measured temperature (retained).
- **FAN ON/OFF with hysteresis** driven by MOSFET (low-side, GPIO23).
- **Fan modes**: `AUTO` / `ON` / `OFF` (via MQTT).
- **Configurable thresholds** (`on_c` / `off_c`) via MQTT (retained state).
- **NTP** – timestamp of last MQTT update (HH:MM).
- **LWT (Last Will & Testament)** – topic `esp32/availability` reports `online` / `offline`.
- **Solar powered**: 6 V panel → Schottky diode → step-up → step-down → **TP4056** → dual 18650 cells (design rationale below).

---

## 🖥️ OLED UI

```
Row 1:  <MQTT Temp °C>               <MQTT Humidity %>
Row 2:  <MQTT Pressure hPa>          FAN <ON|OFF>
Row 3:  <DS18B20 Temp (int) °C>      <Last MQTT update HH:MM>
```

---

## 🔌 MQTT Topics

**Inbound (HA → ESP32)**

* `esp32/weather/temperature`  → e.g. `23.4`
* `esp32/weather/humidity`     → e.g. `55`
* `esp32/weather/pressure`     → e.g. `1015`

**Outbound (ESP32 → HA, retained)**

* `esp32/fan/temperature`  → DS18B20 °C (e.g. `23.75`)
* `esp32/fan/state`        → `ON` | `OFF`

**Control (HA → ESP32) + retained config**

* `esp32/fan/cmd`              → `AUTO` | `ON` | `OFF`
* `esp32/fan/mode`             → retained (`AUTO`/`ON`/`OFF`)
* `esp32/fan/config/on_c/set`  → float °C (command)
* `esp32/fan/config/off_c/set` → float °C (command)
* `esp32/fan/config/on_c`      → retained float °C (state)
* `esp32/fan/config/off_c`     → retained float °C (state)

**Availability (LWT)**

* `esp32/availability` → `online` | `offline`

---

## 🧠 Fan Control Logic (Hysteresis)

* Defaults: **ON ≥ 30.0 °C**, **OFF ≤ 25.0 °C** (compile‑time enforced).
* `AUTO` mode uses hysteresis based on **DS18B20** readings. `ON` / `OFF` force the state regardless of temperature.
* DS18B20 validation: rejects **–127 °C** (disconnected) and **85 °C** (power‑on default).

---

## 🔧 Hardware

### Power Supply Stage
- **Solar panel 1 W / 6 V**
- **Schottky diode (1N5819)** — prevents back-current at night
- **Step-UP converter XL6009** — boosts unstable panel output (~7 V)
- **Step-DOWN converter LM2596S** — regulated **5 V** for TP4056 & logic
- **TP4056 USB-C** — Li-Ion charger with protection
- **2× Lithium-ion rechargeable cell (parallel)** in holder

### Regulation & Distribution
- **Step-UP XL6009** — **5 V** rail (fan supply)
- **Step-DOWN LM2596S** — **5 V → 3.3 V** (ESP32, DS18B20, OLED)

### Control & Processing
- **ESP32 DevKit (ESP-WROOM, CP2102, USB-C)**
- **OLED 0.96" SSD1306, 128×64, I²C**
- **DS18B20** + **4.7 kΩ** pull-up to 3.3 V

### Cooling Stage (MOSFET low-side)
- **N-MOSFET IRLZ44N** (gate at **GPIO23**, HIGH = ON)
  - **Gate series resistor** **100–220 Ω** (limits switching spikes)
  - **Gate pull-down** **47–100 kΩ** → Source (safe OFF; faster gate discharge)
  - **Flyback Schottky diode** **1N5819**: **cathode (+)** to fan supply, **anode (−)** to MOSFET **drain**
  - **Electrolytic capacitor** **220 µF** close to the fan (suppresses inrush/step loads)
- **5 V cooling fan**

### Passives
- **Electrolytic capacitors 220 µF** (power stabilization)
- Additional resistors, wires, connectors

> **Why step-up *and* step-down between solar panel and TP4056?** A 6 V/1 W solar panel produces a variable voltage. Boosting to a higher, stable headroom and then regulating down to **5 V** keeps the TP4056 input stable, avoiding intermittent charging or excess heating.


---

## 🪛 ESP32 Pinout

* **OLED I²C**: SDA **GPIO21**, SCL **GPIO22**, address **0x3C**
* **DS18B20**: DATA **GPIO4** + 4.7 kΩ to 3.3 V
* **FAN (MOSFET gate)**: **GPIO23** (HIGH = ON)

---

## 🗂️ Repo Structure

```
📦 esp32-smart-display-fan
├─ src/                  # ESP32 firmware (Arduino)
├─ hardware/             # schematics, PCB, photos
├─ docs/                 # documentation, diagrams
└─ README.md             # this file
```

---

## ⚙️ Configuration (Wi‑Fi / MQTT)

In the code, edit constants under **USER CONFIGURATION**:

```cpp
constexpr const char* WIFI_SSID     = "<your-ssid>";
constexpr const char* WIFI_PASSWORD = "<your-pass>";
constexpr const char* MQTT_SERVER   = "<broker-ip>";
constexpr uint16_t    MQTT_PORT     = <port>;
constexpr const char* MQTT_USER     = "<user>";
constexpr const char* MQTT_PASSWORD = "<pass>";
```

Default timezone: **GMT+2** (`TIMEZONE_OFFSET = 7200`).

---

## ▶️ Build & Upload

### Arduino IDE (ESP32 core)

1. Install libraries: `Adafruit_GFX`, `Adafruit_SSD1306`, `PubSubClient`, `OneWire`, `DallasTemperature`, `NTPClient`.
2. Select board **ESP32 Dev Module** and the correct COM port.
3. Configure Wi-Fi / MQTT constants in the sketch and upload.

### PlatformIO (optional)

Use this minimal `platformio.ini`:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

monitor_speed = 115200

lib_deps =
  adafruit/Adafruit GFX Library
  adafruit/Adafruit SSD1306
  knolleary/PubSubClient
  paulstoffregen/OneWire
  milesburton/DallasTemperature
  arduino-libraries/NTPClient
```

---

## 🧩 Home Assistant Integration (configuration.yaml example)

```yaml
mqtt:
  sensor:
    - name: "Weather Temperature"
      state_topic: "esp32/weather/temperature"
      unit_of_measurement: "°C"
    - name: "Weather Humidity"
      state_topic: "esp32/weather/humidity"
      unit_of_measurement: "%"
    - name: "Weather Pressure"
      state_topic: "esp32/weather/pressure"
      unit_of_measurement: "hPa"
    - name: "ESP32 Fan Temperature"
      state_topic: "esp32/fan/temperature"
      unit_of_measurement: "°C"
  binary_sensor:
    - name: "ESP32 Fan State"
      state_topic: "esp32/fan/state"
      payload_on: "ON"
      payload_off: "OFF"
  switch:
    - name: "ESP32 Fan Mode ON"
      command_topic: "esp32/fan/cmd"
      payload_on: "ON"
      payload_off: "AUTO"
```

Control thresholds: publish floats to `esp32/fan/config/on_c/set` and `.../off_c/set`.

---

## 🧪 Reliability & Safety

* **LWT**: broker sets `offline` on `esp32/availability` if ESP32 disconnects.
* **DS18B20 validation**: invalid values (–127 / 85 °C) discarded; only last valid is published.
* **OLED redraw watchdog**: ensures refresh at least every 5 minutes.

---

## 📷 Gallery

Photos in (`docs/images/`).

---

## 📜 License

**MIT License**

---

## 👤 Author

Rafał Gromulski
Programming • IoT • Embedded systems • Electronics. Contact: rgromulski@gmail.com / [LinkedIn](https://www.linkedin.com/in/rafałgromulski) / [GitHub](https://github.com/RafalGromulski).

---

### Appendix A — Detailed BOM (Summary)

* **Solar panel 1 W / 6 V** (136×110×3 mm)
* **Schottky diode 1N5819** (1 A / 40 V)
* **Step‑up converter XL6009** (3–30 V in, 5-35 V out)
* **Step‑down converter LM2596S** (3.2-35 V in, 1.5-35 V out)
* **TP4056 USB‑C** (Li-Ion charger, 5V input, 1A, 4.2V cutoff, protection)
* **Samsung INR18650‑35E Li-Ion cell**
* **2X18650P battery holder**
* **ESP32 DevKit (ESP‑WROOM, CP2102, USB‑C)**
* **OLED 0.96" SSD1306 (I²C, 128×64, blue)**
* **DS18B20** (THT TO‑92)
* **N‑MOSFET IRLZ44N**
* **Cooling fan 30mm 5V (3010)**
* **Resistors Set (0.25W 1%)**
* **Electrolytic capacitor 220 µF**
* **Wiring / connectors**

> Detailed specs and reasoning included in parts documentation and comments in code.
