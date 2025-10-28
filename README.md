# 🚀 PRT Venturi Test

This repository contains the **electronics and software** developed to determine whether a **Venturi** is **cavitating** during testing.  

---

## Overview

The system measures and logs data from multiple sensors — including pressure, flow, and temperature — to analyze the fluid dynamics inside a Venturi tube.  
It is designed to be reliable, high-performance, and adaptable to a variety of testing configurations.

### Key Features
- **Arduino GIGA R1** as the main controller  
- **4–20 mA pressure sensors** for accurate differential measurements  
- **Hall-effect and ultrasonic flow meters**  
- **data logging** to SD card (CSV format)
- **Integrated Python GUI** for real-time data visualization and control

---

## Repository Structure

```
PRT-Venturi-Test/
│
├── /Arduino/               # Main firmware for data acquisition and logging
├── /GUI/                   # Python-based interface for live monitoring and plotting
├── /Hardware/              # Schematics, Fritzing diagrams, and wiring layouts
├── /Tests/                 # Example CSV logs and collected test data
└── README.md               # Project documentation
```

---

## Hardware Setup

| Component | Description | Notes |
|------------|--------------|-------|
| **Arduino GIGA R1** | Central control unit | Handles sensor acquisition and data logging |
| **Pressure Sensors (×4)** | 4–20 mA analog sensors | Measure pressure across the venturi |
| **Flow Meters** | Hall-effect & Ultrasonic | Measure instantaneous and cumulative flow |

---

## Software Features

- Automatic test file creation (`Test1.csv`, `Test2.csv`, …)
- High-speed serial communication (2 Mbit/s)
- Real-time monitoring and plotting via **Python GUI**

---

## Installation

### Arduino Firmware
1. Clone this repository:
   ```bash
   git clone https://github.com/Dev-is-here22/PRT-Venturi-Test.git
   ```

2. Open the `/Arduino/` sketch in **Arduino IDE** or **PlatformIO**.

3. Set the board to **Arduino GIGA R1**.

4. Verify that the following libraries are available (usually included by default):
   - `SD`
   - `SPI`

5. Connect the hardware according to the provided schematic and upload the sketch.

---

### Python GUI

The `/Python_GUI/` directory contains a simple graphical interface for:
- Real-time visualization of pressures, flow rates, and temperature  
- Serial communication with the Arduino  
- Logging data directly to disk  
- Quick post-test plotting and analysis  

To run the GUI:

```bash
cd Python_GUI
pip install -r requirements.txt
python venturi_gui.py
```

---

## Usage

1. Power the system and insert an SD card.  
2. Start the test — a new CSV file will be automatically created.  
3. Optionally, launch the **Python GUI** to monitor data live.  
4. After the test, retrieve logged data from the SD card or GUI output folder.  
5. Analyze the results using Python, Excel, or any preferred tool.


---

## Troubleshooting

| Issue | Possible Cause | Solution |
|--------|----------------|-----------|
| Incorrect pressure readings | Drift or offset | Re-run sensor calibration (`CALIBRATION_LOOPS`) |
| Serial output missing | Baud rate mismatch | Ensure monitor is set to 2,000,000 baud |

---

## Contributing

This repository is intended for **internal collaboration** within the **PoliTo Rocket Team**.  
Members of the **Propulsion** and **Electronics** departments are encouraged to:

- Open issues for bugs or improvement suggestions  
- Submit pull requests with updated code or documentation  
- Upload test results in `/Tests/` using clear and consistent naming conventions

---

## Future Development

- Enhanced live data visualization features in the Python GUI  
- Improved real-time flow smoothing algorithms  

