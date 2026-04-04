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