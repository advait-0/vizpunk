
<p align="center">
  <img src="assets/logo.png" alt="Vizpunk Logo" width="400">
</p>

# AIO Oscilloscope, Function Generator, and Power Supply

## Overview
Vizpunk(Visualizer - PSU - Function Generator) is an **all-in-one (AIO) electronics testing and debugging tool** designed for engineers, students, and hobbyists. 
It integrates three essential instruments into a single **compact and cost-effective** system:

- **Oscilloscope** – Capture and analyze signals in real time.
- **Function Generator** – Generate sine, square, and triangular waveforms for circuit testing.
- **Power Supply** – Provide a stable source for powering electronic circuits.

This system runs on an **ESP32-based web server**, allowing users to **control and monitor** the device wirelessly from any browser—no software installation needed!

## Features
✅ Wireless control via a built-in web interface  
✅ Real-time waveform visualization  
✅ Adjustable frequency and waveform settings  
✅ Power supply control with voltage and current monitoring  
✅ Compact and low-cost design  
✅ Open-source and customizable  

## Setup Instructions
### Installation
1. Clone this repository:
   ```sh
   git clone https://github.com/yourusername/vizpunk.git
   cd vizpunk
   ```

2. Install the latest version of ESP-IDF using their [official guide.](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html#manual-installation)
3. Build and flash the project using `idf.py flash monitor`
4. Connect to the SSID `Vizpunk` on your smartphone or laptop, the captive portal should redirect you to the vizpunk home page or go to [vizpunk.com](http://vizpunk.com).
5. Get your work done with our easy-to-use GUI.

## Third-Party Acknowledgments

This project uses uPlot, a fast and lightweight charting library.
uPlot is licensed under the MIT License. See the [uPlot repository](https://github.com/leeoniya/uPlot) for more details.