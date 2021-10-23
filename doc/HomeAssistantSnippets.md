
I use the ESP32 bluetooth presence scans to provide presence information to HomeAssistant.

Here's an example of a Bluetooth presence sensor for a _ESP32-MQTT-Bluetooth-Monitor_ device with Identity ```ESP32-bluetooth-monitor```.
The device that the confidence is reported for has alias ```s10```. When the Bluetooth scanner node goes down, this is also reflected in the presence sensor.
This snippet belongs in the ```sensor:``` section of the HomeAssistant configuration.
```yaml
- platform: mqtt
  state_topic: 'monitor/ESP32-bluetooth-monitor/s10'
  value_template: '{{ value_json.confidence }}'
  unit_of_measurement: '%'
  availability:
    - topic: "monitor/ESP32-bluetooth-monitor/status"
  payload_available: "online"
  payload_not_available: "offline"
  name: 'ESP32 S10 bluetooth presence confidence'
```


The following snippet is an automation used to trigger _Arrival_ / _Departure_ scans based on door open / close triggers.
For my household where 4 devices are in the Known Devices list, precense is generally detected within about __2.5 seconds__ of the door being opened!

```yaml
- id: 'bt_presence_scan_on_door_open_close'
  alias: Bluetooth Presence scan on Door open/close
  description: Scan for bluetooth arrival / departure on door open / close events
  trigger:
  # Any door sensor to trigger the scan for:
  - platform: state
    entity_id: binary_sensor.voordeur_contact
  - platform: state
    entity_id: binary_sensor.achterdeur_contact
  condition: []
  action:
  # Either scan for departure or arrivals
  - choose:
    - conditions:
      - condition: template
        value_template: "{{ trigger.to_state.state == 'off' }}"
      # Door close: delayed scan for departing devices
      sequence:
      - delay: 00:02:00
      - service: mqtt.publish
        data:
          topic: monitor/scan/DEPART
    # Door open: scan for new devices
    default:
    - service: mqtt.publish
      data:
        topic: monitor/scan/ARRIVE
  mode: parallel
  max: 3
  max_exceeded: silent
```

I've newly added passive ble iBeacon scan support which will also support rssi value reporting. Below is an example to get to that data using an MQTT sensor in HomeAssistant. This example uses an _ESP32-MQTT-Bluetooth-Monitor_ device with Identity ```ESP32-bluetooth-monitor```.
The device that the confidence is reported for has alias ```s10-iBeacon```. 
This snippet belongs in the ```sensor:``` section of the HomeAssistant configuration.
```yaml
- platform: mqtt
  state_topic: 'monitor/ESP32-bluetooth-monitor/s10-iBeacon'
  value_template: '{{ value_json.rssi }}'
  unit_of_measurement: 'dBm'
  availability:
    - topic: "monitor/ESP32-bluetooth-monitor/status"
  payload_available: "online"
  payload_not_available: "offline"
  name: 'ESP32 S10 iBeacon RSSI'
```

