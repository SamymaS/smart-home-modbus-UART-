# Projet Modbus RTU entre ESP32 et Raspberry Pi

Ce projet met en place une communication **Modbus RTU** entre un **ESP32** (esclave) et un **Raspberry Pi** (maître) via les broches UART (GPIO 4 et GPIO 5).

---

## 1. Présentation

L'ESP32 simule un capteur de température. La valeur de température est stockée dans un registre Modbus (Holding Register).  
Le Raspberry Pi interroge régulièrement l'ESP32 pour récupérer cette température et l'affiche.  
La communication se fait en Modbus RTU via une liaison UART directe.

---

## 2. Matériel nécessaire

- ESP32 DevKit
- Raspberry Pi (modèle 3, 4 ou supérieur recommandé)
- Câbles Dupont (femelle-femelle)
- Alimentation indépendante pour chaque carte
- (Optionnel) Adaptateur RS485 si besoin de communication longue distance

---

## 3. Schéma de câblage

```
ESP32 (UART1)       Raspberry Pi (UART0 ou autre)
---------------------------------------------------
GND  ---------------- GND (Pin 6)
TX (GPIO 5) -------- RX (GPIO 15 - GPIO RXD0)
RX (GPIO 4) -------- TX (GPIO 14 - GPIO TXD0)
```


---

## 4. Paramètres de communication Modbus RTU

| Paramètre        | Valeur   |
|------------------|----------|
| **Vitesse**      | 9600 bauds |
| **Parité**       | Aucun    |
| **Bits de données** | 8    |
| **Bits d'arrêt** | 1        |
| **Identifiant Esclave** | 1 |

---

## 5. Structure des registres Modbus

| Registre | Adresse | Type             | Description                                |
|----------|---------|------------------|--------------------------------------------|
| 0        | 0       | Holding Register | Température simulée (en dixième de degré) |

---

## 6. Installation et configuration du Raspberry Pi

### Activation de l'UART sur Raspberry Pi

1. Modifier `/boot/config.txt` :
    ```bash
    sudo nano /boot/config.txt
    ```
    Ajouter :
    ```
    enable_uart=1
    ```
2. Désactiver la console série :
    ```bash
    sudo raspi-config
    ```
    - Interfacing Options > Serial Port > Désactiver la console
3. Redémarrer :
    ```bash
    sudo reboot
    ```

### Permissions sur le port série

```bash
ls -l /dev/serial*
sudo usermod -aG dialout $USER
```

---

## 7. Code côté ESP32

Le code utilise les librairies :
- Modbus (https://github.com/epsilonrt/modbus-arduino)
- ModbusSerial (https://github.com/epsilonrt/modbus-serial)
```
#include <Arduino.h>
#include <Modbus.h>
#include <ModbusSerial.h>

#define BAUDRATE 9600
#define TX_PIN 5
#define RX_PIN 4
#define SLAVE_ID 1

const int TEMP_REGISTER = 0;

ModbusSerial modbus(Serial1, SLAVE_ID, -1);

long lastUpdate = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Modbus RTU - Esclave");

    Serial1.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);

    modbus.config(BAUDRATE);
    modbus.addHreg(TEMP_REGISTER);

    modbus.setAdditionalServerData("ESP32 Sensor Node");
}

void loop() {
    modbus.task();

    if (millis() - lastUpdate > 2000) {
        lastUpdate = millis();
        int temperature = random(200, 300);
        modbus.Hreg(TEMP_REGISTER, temperature);
        Serial.printf("Température mise à jour : %.1f°C\\n", temperature / 10.0);
    }
}
```
---

## 7. Code côté Raspberry Pi
Installation des dépendances
```
python3 -m venv venv
source venv/bin/activate
pip install pymodbus==3.2.2
```

Exemple Raspberry Pi
```
import time
from pymodbus.client import ModbusSerialClient

PORT = "/dev/serial0"
BAUDRATE = 9600
SLAVE_ID = 1

def read_temperature(client):
    result = client.read_holding_registers(address=0, count=1, slave=SLAVE_ID)
    if result.isError():
        print("❌ Erreur de lecture :", result)
        return None
    temperature = result.registers[0] / 10.0
    print(f"🌡️ Température actuelle : {temperature:.1f} °C")
    return temperature

def main():
    client = ModbusSerialClient(
        method='rtu',
        port=PORT,
        baudrate=BAUDRATE,
        stopbits=1,
        parity='N',
        bytesize=8,
        timeout=1
    )

    if not client.connect():
        print("❌ Impossible de se connecter au port Modbus.")
        return

    try:
        while True:
            read_temperature(client)
            time.sleep(2)
    except KeyboardInterrupt:
        print("🚪 Arrêt demandé.")
    finally:
        client.close()

if __name__ == "__main__":
    main()

```

---

## 9. Test et vérifications
Moniteur série ESP32
```
ESP32 Modbus RTU - Esclave
Température mise à jour : 23.5°C
Température mise à jour : 24.1°C
```

Test de lecture Raspberry Pi
```
python read_home.py
```
attendu
```
🌡️ Température actuelle : 23.5 °C
🌡️ Température actuelle : 24.1 °C
```
---

## 10. Commandes utiles
Lister les ports
```
ls -l /dev/serial*
```

Ajouter l'utilisateur
```
sudo usermod -aG dialout $USER
```

AVérifier les processus utilisant le port série
```
sudo lsof /dev/serial0
```

---

## 11. Evolutions possibles
- Ajout de registres supplémentaires (humidités, vitesses, etc.)
- Intégration RS485
- Tableau de bord web
- Intégration Prometheus/Grafana

---

## 12. Références
- Modbus-Serial (https://github.com/epsilonrt/modbus-serial)
- Modbus-Arduino (https://github.com/epsilonrt/modbus-arduino)
- Pymodbus (https://pymodbus.readthedocs.io/en/latest/)
