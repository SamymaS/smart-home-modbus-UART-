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
