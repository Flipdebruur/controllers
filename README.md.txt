# README

## Dependencies

- Python 3.8 or higher  
- pyserial (`pip install pyserial`)  
- Webots R2023b or later  
- MicroPython firmware (v1.19.1 or later) for ESP32  

## How to Reproduce the Experiment

1. **ESP32 Setup**  
   - Flash the ESP32 with MicroPython firmware.  
   - Change map and coordinates to whatever you are using.
   - Upload `main.py` to the ESP32 using Thonny or another tool.  
   - Connect a button to GPIO 0 (for safe mode) and a LED to GPIO 2 (for visual feedback).  
   - Ensure UART connections are correct

2. **Webots Simulation**  
   - Open your Webots world with a robot (e.g. e-puck) that has proximity sensors and wheel encoders.  
   - Set the controller of the robot to `LAB7pathfollowerctrl`.  
   - Make sure the serial port in `LAB7pathfollowerctrl.py` matches your ESP32's COM port.


3. **Running the System**  
   - Start the ESP32 first; it waits for Webots to connect.  
   - Then run the Webots simulation.  
   - The robot will receive goals from the ESP32, follow the path, and report obstacles when detected.  
   - The ESP32 recalculates the path and sends new waypoints when necessary.

