## 🚗 Intelligent Communication System for CyberCar - Implementation

This project demonstrates the implementation of a distributed firmware update system for an intelligent CyberCar, composed of three main components:

* **MPU Board** (Laptop/SBC like Jetson Nano or Raspberry Pi)
* **Domain Controller (DC)** (STM32H7)
* **MCU Node** (STM32F4)

The system uses MQTT (via LWIP), TCP/IP stack, and CAN-bus to ensure reliable and modular communication.

---

### 🖥️ MPU Board Implementation

The MPU acts as the central broker and entry point of the system.

* **Role**: Receives precompiled firmware and publishes it over MQTT.
* **Technologies**: Python 3, Eclipse Mosquitto (MQTT broker), paho-mqtt
* **Network Setup**:

  * MQTT Broker: `192.168.10.231:1883`
  * Topic used: `/firmware`

#### Python Script (Essential Flow)

```python
import paho.mqtt.client as mqtt

# Configuration
BROKER_ADDRESS = "192.168.10.231"
TOPIC_FIRMWARE = "firmware"
FIRMWARE_PATH = "F446_APP.bin"

client = mqtt.Client()
client.connect(BROKER_ADDRESS)

with open(FIRMWARE_PATH, "rb") as f:
    firmware = f.read()
    client.publish(TOPIC_FIRMWARE, payload=firmware, qos=1)
```

---

### 🌐 Domain Controller Implementation (STM32H7)

The DC subscribes to the MQTT topic and transfers firmware over CAN to the MCU node.

* **MQTT Client**: Implemented via LWIP stack in C (STM32CubeIDE)
* **CAN Interface**: FDCAN at 1Mbps
* **Workflow**:

  1. Connects to MQTT broker
  2. Subscribes to `/firmware`
  3. Buffers incoming firmware data
  4. Sends firmware via CAN using ID chunking:

     * `0x10`: Start
     * `0x11`: Data (8-byte chunks)
     * `0x12`: End

#### Key C Code Snippet

```c
void mqtt_incoming_data_cb(...) {
  if (receiving_firmware)
    memcpy(&firmware_buffer[offset], data, len);
}

void send_firmware_over_can() {
  // Send start
  send_can_frame(0x10, &start_byte);

  // Send data in 8-byte chunks with ID 0x11
  for (...) {
    send_can_frame(0x11, chunk);
  }

  // Send end
  send_can_frame(0x12, &end_byte);
}
```

---

### 🧳 MCU Node Implementation (STM32F4)

This node receives firmware via CAN and writes it to flash memory.

* **CAN Peripheral**: Normal mode at 1Mbps
* **Flash Memory Target Address**: `0x08020000`
* **Firmware Size Supported**: Up to 256KB

#### Workflow

1. Upon receiving CAN ID `0x10`, begin firmware write mode.
2. Buffer and write incoming data (ID `0x11`) to flash.
3. Finalize and optionally jump to the new application after ID `0x12`.

#### Key C Callback

```c
void HAL_CAN_RxFifo0MsgPendingCallback(...) {
  if (RxHeader.StdId == 0x10) {
    Flash_Erase();
  } else if (RxHeader.StdId == 0x11) {
    Flash_Write(...);
  } else if (RxHeader.StdId == 0x12) {
    Flash_Finalize();
    JumpToApplication();
  }
}
```

---

### 🔄 Future Improvements

* Implement secure OTA authentication and encryption.
* Dynamic topic mapping for multiple domain controllers.
* Chunk index and CRC validation for robustness.
* Full bootloader integration for the MCU node.

