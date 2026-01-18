## Hardware

  - Seeed Studio XIAO ESP32C6
  - Seeed Studio Grove Base for XIAO
  - 4x 4-pin Grove Connector cables
  - 3x XKC-Y25-NPN Non-Contact Liquid Level Sensor
  - MIGRO Electrical Weatherproof Outdoor Junction Box
  - MEAN WELL APV-12-5 5v power supply
  - SunLee 2 Pole 40 Amp contactor 120v coil
  - 3 Pin 15A 125V AC Panel Mount Outlet plug
  - Right Angle USB C to 2 Pin Bare Wire plug
  - 20AWG 4 conductor cable
  - 14AWG silicone hookup wire
  - Spade crimp connectors

## ESPHome instructions

 - Pull project files
 - Install CLI [ESPHome](https://esphome.io/guides/installing_esphome)
 - create a `secrets.yaml` file in the project root with the following contents:
```
wifi_ssid: "YOUR_2.4GHZ_WIFI_SSID"
wifi_password: "YOUR_2.4GHZ_WIFI_SSID_PASSWORD"
home_assistant_key: "HOME_ASSISTANT_API_PRE_SHARED_KEY"
```
 - cd into project folder and run `esphome run config.yaml` to build and upload

## Hardware & Installation

Hardware overview:
![hardware overview](https://raw.githubusercontent.com/aartrost/esphome-sewage-pump-controller/main/pictures/hardware.jpeg)

High voltage box installed:
![High voltage box](https://raw.githubusercontent.com/aartrost/esphome-sewage-pump-controller/main/pictures/high-voltage-box-installed.jpeg)

Completed project:
![High voltage box](https://raw.githubusercontent.com/aartrost/esphome-sewage-pump-controller/main/pictures/final-installation.jpeg)

## Home assistant

![Home Assistant screenshot](https://raw.githubusercontent.com/aartrost/esphome-sewage-pump-controller/main/pictures/home-assistant.png)
