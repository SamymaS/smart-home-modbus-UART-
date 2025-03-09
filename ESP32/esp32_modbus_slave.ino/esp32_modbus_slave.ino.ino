#include <Arduino.h>
#include <Modbus.h>
#include <ModbusSerial.h>

// Définir les broches UART
#define RXPIN 4
#define TXPIN 5

// Configuration de la communication
#define BAUD 9600
#define SLAVE_ID 1

// Registres Modbus
#define REG_TEMP 0  // Adresse du registre pour la température

// Objet Modbus
ModbusSerial mb(Serial1, SLAVE_ID, -1);  // Pas de pin TX enable (pas RS485)

// Variable de température simulée
uint16_t temperature = 200;  // Exemple 20.0°C (x10 pour conserver une précision)

void setup() {
    Serial.begin(115200);
    Serial.println("🚀 Démarrage de l'ESP32 en Modbus RTU");

    // Configuration de Serial1 sur les GPIO définis
    Serial1.begin(BAUD, SERIAL_8E1, RXPIN, TXPIN);

    // Initialisation Modbus
    mb.config(BAUD);
    mb.setAdditionalServerData("ESP32-Temp-Sensor");

    // Création du registre pour la température
    mb.addIreg(REG_TEMP, temperature);

    Serial.println("✅ Modbus RTU prêt - En attente de requêtes");
}

void loop() {
    // Mise à jour de la température (simulation simple, tu peux la remplacer par une vraie lecture de capteur)
    temperature = 200 + random(-10, 10);  // Entre 19.0°C et 21.0°C

    // Mise à jour du registre avec la nouvelle température
    mb.Ireg(REG_TEMP, temperature);

    // Traitement Modbus (répond aux requêtes du maître)
    mb.task();

}
