#include <Arduino.h>
#include <Modbus.h>
#include <ModbusSerial.h>

// Configuration des pins pour la communication série
#define RX_PIN     4   // GPIO 4 = RX
#define TX_PIN     5   // GPIO 5 = TX

// Définir le baudrate et les paramètres Modbus
#define BAUD      9600UL        // Baudrate à 9600
#define SLAVE_ID  1             // ID Modbus de l'esclave
#define TXEN_PIN  -1            // Pas de gestion de DE/RE car pas de RS485 (laisser à -1)

#define REG_TEMP        0       // Registre 0 : Température (lecture)
#define REG_FAN_SPEED   1       // Registre 1 : Vitesse ventilateur (lecture/écriture)

// Création de l'instance ModbusSerial
HardwareSerial mySerial(1);               // Utilisation de Serial1 pour la communication Modbus
ModbusSerial modbus(mySerial, SLAVE_ID, TXEN_PIN);  // Initialisation avec Serial1, ID esclave, pas de TXEN

// Variables pour les registres
uint16_t holdingRegs[2] = {250, 1};  // Température fictive et vitesse initiale

// Timer pour simuler une mise à jour de température
long lastUpdate = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("🔌 Démarrage de l'ESP32 Modbus RTU");

    // Configuration de Serial1 avec les pins définis et le bon baudrate
    mySerial.begin(BAUD, SERIAL_8E1, RX_PIN, TX_PIN);

    // Configuration du ModbusSerial (uniquement le baudrate car Serial1 est déjà configuré)
    modbus.config(BAUD);

    // Ajoute une description facultative pour le serveur Modbus (utile pour la fonction Report Server ID)
    modbus.setAdditionalServerData("ESP32_ModbusSlave");

    // Ajout des registres (température = lecture seule, vitesse = lecture/écriture)
    modbus.addHreg(REG_TEMP);          // Registre pour la température (lecture seule côté maître)
    modbus.addHreg(REG_FAN_SPEED);     // Registre pour la vitesse ventilateur (modifiable par le maître)

    // Initialisation des registres avec des valeurs par défaut
    modbus.Hreg(REG_TEMP, holdingRegs[REG_TEMP]);
    modbus.Hreg(REG_FAN_SPEED, holdingRegs[REG_FAN_SPEED]);

    Serial.println("✅ Modbus RTU prêt - En attente de requêtes");
}

void loop() {
    // Simulation d'une mise à jour de la température toutes les 2 secondes
    if (millis() - lastUpdate >= 2000) {
        lastUpdate = millis();

        holdingRegs[REG_TEMP] = random(200, 300);  // Température fictive entre 20.0 et 30.0 °C (x10 pour avoir 1 décimale)
        modbus.Hreg(REG_TEMP, holdingRegs[REG_TEMP]);
    }

    // Vérifie si la vitesse a été modifiée par le maître
    uint16_t newSpeed = modbus.Hreg(REG_FAN_SPEED);
    if (newSpeed != holdingRegs[REG_FAN_SPEED]) {
        holdingRegs[REG_FAN_SPEED] = newSpeed;
        Serial.print("🔄 Nouvelle vitesse reçue : ");
        Serial.println(holdingRegs[REG_FAN_SPEED]);
    }

    // Appelle la fonction Modbus pour gérer les requêtes entrantes
    modbus.task();
}
