# AURORA - AI-driven Utility Regulation for Occupancy-aware Resource Allocation

This project is an attempt to tackle **energy wastage in indoor spaces** using a combination of **computer vision, IoT, and cloud communication**.

Instead of keeping lights, fans, or cooling systems running unnecessarily, the system observes how a space is actually being used and makes decisions accordingly.

At a high level, a vision model detects and counts people in real time. This occupancy data is sent to the cloud using MQTT, where it is consumed by an IoT node and a live dashboard. The system then intelligently decides when devices should be turned ON or OFF, factoring in both human presence and environmental conditions such as temperature.

The goal is simple:  
**use energy only when it is genuinely needed.**

---

## System Overview

- **Computer Vision (YOLO)**  
  Detects and counts people in real time using a camera feed.

- **Cloud Communication (EMQX + MQTT)**  
  Enables reliable, low-latency data transfer across different networks.

- **IoT Control (ESP32)**  
  Receives occupancy data and controls relays connected to electrical devices.

- **Live Dashboard (JavaScript)**  
  Visualizes occupancy, temperature trends, relay states, and estimated energy savings.

---

## Contribution

Most existing automation systems rely on static schedules or simple motion sensors.  
This system instead reacts to **actual human behavior**, reducing unnecessary power consumption and enabling smarter energy usage.

By combining AI-based perception with IoT control, the project demonstrates a practical and scalable approach to **sustainable energy management** in classrooms, offices, and shared indoor spaces.

---

## Key Features

- Real-time vision-based occupancy detection  
- Cloud-based MQTT communication (network-agnostic)  
- Intelligent relay control using occupancy and temperature logic  
- Live analytics dashboard with energy usage estimation  
- Modular and scalable architecture  

---

## Tech Stack

- Python (YOLO, OpenCV)
- ESP32 (WiFi, MQTT, relay control)
- EMQX Cloud (MQTT broker)
- JavaScript (Dashboard & analytics)
- HTML / CSS (UI)

---

## Use Cases

- Smart classrooms
- Office energy optimization
- Shared indoor spaces
- Green building prototypes
- Hackathons and academic projects

---

##  Note

This project is built as a **proof of concept** for intelligent, green energy–aware automation.  
It is designed to be extendable with additional sensors, predictive models, and multi-room control logic.
