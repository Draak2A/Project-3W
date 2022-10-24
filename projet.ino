#include <EEPROM.h>   // Utilisation mémoire EEPROM
#include <ChainableLED.h>   // Librairie LED
#include <Adafruit_BME280.h>    // Librairie capteur BME280
#include <SoftwareSerial.h>   // Librairie pour le GPS
#include <Wire.h>
#include <RTClib.h>   // Librairie pour la clock
#include <SPI.h>
#include "SdFat.h"   // Librairie pour le lecteur SD

RTC_DS1307 rtc;       // Déclaration de la clock
Adafruit_BME280 bme;    // Initialisation du capteur BME280

ChainableLED leds(4, 5, 1);    // Définition des broches de la LED
SoftwareSerial SoftSerial(5, 6); // Pin GPS

const byte btnV = 2;   // Initialisation bouton sur port 2 et 3 pour utilisation d'interruption
const byte btnR = 3;
volatile byte btn_timer = 0;   // Timer temps d'appui bouton      // Variable en volatile par sécurité pour les interruptions
volatile byte led_timer = 0;    // Timer LED
unsigned long config_timeout;    // Temps d'inactivité dans mode config
byte station_mode;    // Mode station
const byte sensor_nb = 4;    // Nombre de capteurs hors gps
byte multiplicator;   // Multiplicateur pour le temps d'attente entre deux mesures
bool gps = 1;

// Récupération des variables dans mémoire EEPROM
// Lors de première utilisation aller dans mode config et taper commande DEFAULT pour enregistrer les valeurs
int value;
#define LOG_INTERVALL EEPROM.get(0, value)
#define FILE_MAX_SIZE EEPROM.get(2, value)
#define TIMEOUT EEPROM.read(4)
#define LUMIN EEPROM.read(6)
#define LUMIN_LOW EEPROM.get(7, value)
#define LUMIN_HIGH EEPROM.get(9, value)
#define TEMP_AIR EEPROM.read(11)
#define MIN_TEMP_AIR EEPROM.get(12, value)
#define MAX_TEMP_AIR EEPROM.get(14, value)
#define HYGR EEPROM.read(16)
#define HYGR_MINT EEPROM.get(17, value)
#define HYGR_MAXT EEPROM.get(19, value)
#define PRESSURE EEPROM.read(21)
#define PRESSURE_MIN EEPROM.get(22, value)
#define PRESSURE_MAX EEPROM.get(24, value)

float sensor_data[sensor_nb];   // Tableau des données des capteurs
String gps_data;    // Variable pour stocker les données GPS
bool error[6] = {0, 0, 0, 0, 0, 0};   // Tableau des erreurs


// Initialisation des interruptions
void setup_interrupt() {
  attachInterrupt(digitalPinToInterrupt(btnV), interrupt, CHANGE);   // Interruption bouton vert
  attachInterrupt(digitalPinToInterrupt(btnR), interrupt, CHANGE);   // Interruption bouton rouge
  cli();    // Désactivation des interruptions
  TCCR1A = 0;   // Tinitialisation TIMER1
  TCCR1B = 0;
  TCNT1  = 0;   // Initialisation du compteur à 0
  OCR1A = 15624;    // = (16*10^6) / (1*1024) - 1 pour interruption à 1Hz (doit être < à 65536)
  TCCR1B = 0b00001101;    // Démarre le timer en mode CTC (bit 3) et avec un prescaler de 1024 (bit 0 et 2)
  TIMSK1 = 0b00000000;    // Ne lance pas la comparaison des valeurs pour l'interruption

  TCCR2B = 0b00000111;    // Tinitialisation TIMER2
  TCNT2 = 98;   // Initialisation du compteur à 98
  TIMSK2 = 0b00000001;    // Active l'overflow interrupt
  TIFR2 = 0b00000000;   // Reset overflow interrupt flag
  sei();    // Activation des interruptions
}

void interrupt() {
  if (digitalRead(btnR) == 0 || digitalRead(btnV) == 0) {
    TIMSK1 = 0b00000010;  // Lance la comparaison des valeurs pour l'interruption
  }
  else {
    TIMSK1 = 0b00000000;
    btn_timer = 0;
  }
}

ISR(TIMER1_COMPA_vect){
  btn_timer++;    // Incrémente le timer toutes les secondes quand un bouton est appuyé
  if (btn_timer == 5) {    // Si le bouton est appuyé pendant 5 secondes
    btn_timer++;    // Incrémente le timer pour ne pas rentrer de nouveau dans la condition
    if ((digitalRead(btnR) == 0 && station_mode == 3) || digitalRead(btnR) == 0 && station_mode == 2) {
      station_mode = 1;
      Serial.println("Mode standard");
    }
    else if (digitalRead(btnR) == 0 && station_mode == 1) {
      station_mode = 3;
      Serial.println("Mode maintenance");
    }
    else if (digitalRead(btnV) == 0 && station_mode == 1) {
      station_mode = 2;
      Serial.println("Mode economique");
    }
  }
}

ISR(TIMER2_OVF_vect) {
  led_timer++;
  if (led_timer > 101){   // Quand le timer dépasse 101 soit 3 secondes d'appui, on réinitialise le timer. Le timer est de 3 secondes pour permettre les différents éclairages de la LED
    led_timer = 0;
  }
  changeLed();
  TCNT2 = 131;   // Reset le compteur
  TIFR2 = 0b00000000;   // Reset overflow interrupt flag
}

void changeLed() {
  if (error[0]){    // Erreur d'accès à un capteur
    if (led_timer < 50){
      leds.setColorRGB(0, 0, 255, 0);   // Led = vert
    } 
    else{
      leds.setColorRGB(0, 255, 0, 0);   // Led = rouge
    }
  }
  else if (error[1]){    // Données incohérente
    if (led_timer < 66){
      leds.setColorRGB(0, 0, 255, 0); // Led = vert
    } 
    else{
      leds.setColorRGB(0, 255, 0, 0); // Led = rouge
    }
  }
  else if (error[2]){    // Erreur GPS
    if (led_timer < 50){
      leds.setColorRGB(0, 255, 0, 0); // Led = rouge
    } 
    else{
      leds.setColorRGB(0, 255, 255, 0);; // Led = jaune
    }
  }
  else if (error[3]){    // Erreur clock
    if (led_timer < 50){
      leds.setColorRGB(0, 255, 0, 0); // Led = rouge
    } 
    else{
      leds.setColorRGB(0, 0, 0, 255); // Led = bleu
    }
  }
  else if (error[4]){    // SD pleine
    if (led_timer < 50){
      leds.setColorRGB(0, 255, 255, 255); // Led = blanc
    } 
    else{
      leds.setColorRGB(0, 255, 0, 0); // Led = rouge
    }
  }
  else if (error[5]){    // Erreur SD
    if (led_timer < 66){
      leds.setColorRGB(0, 255, 255, 255); // Led = blanc
    } 
    else{
      leds.setColorRGB(0, 255, 0, 0); // Led = rouge
    }
  }
  else if (station_mode == 1) {   // Mode standard
    leds.setColorRGB(0, 0, 255, 0); // Led = vert
  }
  else if (station_mode == 2) {   // Mode économique
    leds.setColorRGB(0, 0, 0, 255); // Led = bleu
  }
  else if (station_mode == 3) {   // Mode maintenance
    leds.setColorRGB(0, 255, 128, 0); // Led = orange
  }
  else {    // Mode configuration
    leds.setColorRGB(0, 255, 255, 0); // Led = jaune
  }
}

// Fonction permettant de remettre les valeurs par défaut pour les variables dans la mémoire EEPROM
void defaultValue() {
  EEPROM.write(0, 10);
  EEPROM.put(2, 4096);
  EEPROM.write(4, 30);
  EEPROM.write(6, 1);
  EEPROM.write(7, 255);
  EEPROM.put(9, 768);
  EEPROM.write(11, 1);
  EEPROM.put(12, -10);
  EEPROM.write(14, 60);
  EEPROM.write(16, 1);
  EEPROM.write(17, 0);
  EEPROM.write(19, 50);
  EEPROM.write(21, 1);
  EEPROM.put(22, 850);
  EEPROM.put(24, 1080);
  EEPROM.write(255, 1);   
}

// Fonction vérifiant si la commande entrée dans la console est valide et récupère l'addresse de la variable à modifier
void getEEPROMAddress(byte *var_address, String *command_var) {
if (*command_var == "LOG_INTERVALL=") {
    *var_address = 0;
  }
  else if (*command_var == "FILE_MAX_SIZE=") {
    *var_address = 2;
  }
  else if (*command_var == "TIMEOUT=") {
    *var_address = 4;
  }
  else if (*command_var == "LUMIN=") {
    *var_address = 6;
  }
  else if (*command_var == "LUMIN_LOW=") {
    *var_address = 7;
  }
  else if (*command_var == "LUMIN_HIGH=") {
    *var_address = 9;
  }
  else if (*command_var == "TEMP_AIR=") {
    *var_address = 11;
  }
  else if (*command_var == "MIN_TEMP_AIR=") {
    *var_address = 12;
  }
  else if (*command_var == "MAX_TEMP_AIR=") {
    *var_address = 14;
  }
  else if (*command_var == "HYGR=") {
    *var_address = 16;
  }
  else if (*command_var == "HYGR_MINT=") {
    *var_address = 17;
  }
  else if (*command_var == "HYGR_MAXT=") {
    *var_address = 19;
  }
  else if (*command_var == "PRESSURE=") {
    *var_address = 21;
  }
  else if (*command_var == "PRESSURE_MIN=") {
    *var_address = 22;
  }
  else if (*command_var == "PRESSURE_MAX=") {
    *var_address = 24;
  }
  else if (*command_var == "CLOCK=") {
    *var_address = 200;
  }
  else if (*command_var == "DATE=") {
    *var_address = 201;
  }
}

// Récupération de la commande entrée dans la console
void commandEntry() {
  if (Serial.available() > 0) {
    config_timeout = millis();    // Réinitialisation du timer d'inactivité quand une commande est entrée
    String command = Serial.readString();   // Récupération de la commande entrée dans la console
    Serial.println(command);
    if (command == "VERSION") {   // Affichage de la version du programme
      Serial.println("Version 1.0");
    }
    else if (command == "RESET") {    // Remise à zéro des variables stockées dans la mémoire EEPROM
      defaultValue();
      Serial.println("Valeurs par défaut chargées");
      return ;
    }
    else if (command == "EXIT") {   // Sortie du mode configuration
      station_mode = 1;
      return ;
    }
    else {
      String command_var = "";
      byte var_address = 255;   // Utilisation de l'adresse 255 pour indiquer que la commande n'est pas valide
      for (int i=0; i<command.length(); i++) {
        command_var += command[i];
        if (command[i] == '=') {    // Récupération de la commande et de l'addresse de la variable à modifier
          getEEPROMAddress(&var_address, &command_var);
          command_var = "";   // Vidage de la variable contenant le nom de la commande pour stocker ensuite la nouvelle valeur à sauvegarder
        }
      }
      if (var_address != 255) {
        DateTime now = rtc.now();
        if (var_address == 200) {
          String hours = String(command_var[0])+String(command_var[1]);     // Récupération de l'heure entrée dans la console
          String minutes = String(command_var[3])+String(command_var[4]);
          String seconds = String(command_var[6])+String(command_var[7]);
          rtc.adjust(DateTime(now.year(), now.month(), now.day(), hours.toInt(), minutes.toInt(), seconds.toInt()));    // Modification de l'heure
        }
        else if (var_address == 201) {
          String month, day, year;
          month = String(command_var[0])+String(command_var[1]);      // Récupération de la date entrée dans la console
          day = String(command_var[3])+String(command_var[4]);
          year = String(command_var[6])+String(command_var[7])+String(command_var[8])+String(command_var[9]);
          rtc.adjust(DateTime(year.toInt(), month.toInt(), day.toInt(), now.hour(), now.minute(), now.second()));   // Modification de la date
        }
        else {
          int command_value = command_var.toInt();    // Convertion de la valeur entrée dans la console en entier
          EEPROM.write(var_address, command_value);   // Sauvegarde de la nouvelle valeur dans la mémoire EEPROM
          Serial.println("Valeur modifiée");
          return ;
        }
      }
      else {
        Serial.println("Commande inconnue");
      }
    }
  }
}

void getMeasures() {
  unsigned long bme_timeout = millis();
  while ((millis() - bme_timeout) < TIMEOUT * 1000) {
    if (!bme.begin()) {    // Vérification de la présence du capteur BME280
      error[0] = 1;   // Si le capteur n'est pas détecté, on indique une erreur
    }
    else {
      error[0] = 0;   // Si le capteur est détecté, on indique qu'il n'y a pas d'erreur
      break;
    }
  }
  if (LUMIN == 1) {   // Si la mesure de la luminosité est activée
    sensor_data[0] = analogRead(A0);   // Récupération de la valeur de la lumière
  }
  if (TEMP_AIR == 1) {    // Si la mesure de la température est activée
    sensor_data[1] = bme.readTemperature();    // Récupération de la température
  }
  if (HYGR == 1) {    // Si la mesure de l'humidité est activée
    sensor_data[2] = bme.readHumidity();   // Récupération de l'humidité
  }
  if (PRESSURE == 1) {    // Si la mesure de la pression est activée
    sensor_data[3] = bme.readPressure() / 100;   // Récupération de la pression (conversion en hPa)
  }
}

void getErrors() {
  if (error[0] == 0) {   // Si il n'y a pas d'erreur sur le capteur
    if (sensor_data[1] < MIN_TEMP_AIR || sensor_data[1] > MAX_TEMP_AIR) {   // Vérification si la température est dans le domaine de définition
      error[1] = 1;
    }
    if (sensor_data[1] < HYGR_MINT || sensor_data[1] > HYGR_MAXT) {   // Vérification la température est correcte pour prendre la pression en considération
      sensor_data[2] = NULL;
    }
    if (sensor_data[3] < PRESSURE_MIN || sensor_data[3] > PRESSURE_MAX) {   // Vérification si la pression est dans le domaine de définition
      error[1] = 1;
    }
  }
}

void getGpsData() {
  if (SoftSerial.available()) {   // Vérification de la communication avec le module GPS
    while(true) {
      gps_data = SoftSerial.readStringUntil('\n');    // Récupération ligne par ligne des données GPS
      if (gps_data.startsWith("$GPGGA", 0)) {   // Vérification si la ligne commence par $GPGGA. Si c'est le cas, nous gardons la ligne
        break;
      }
    }
  }
  else {
    error[2] = 1;   // Si il n'y a pas de communication avec le module GPS, on indique une erreur
  }
}

void saveData() {
  SdFat SD;
  File myFile;
  static byte i;    // Variable statique pour ne pas réinitialiser la variable à chaque appel de la fonction
  unsigned long comp_timeout = millis();
  while ((millis() - comp_timeout) < TIMEOUT * 1000) {    // Vérification du fonctionnement de la clock
    if (!rtc.isrunning()) {
      error[3] = 1;
    }
    else {
      error[3] = 0;
      break;
    }
  }
  comp_timeout = millis();
  while ((millis() - comp_timeout) < TIMEOUT * 1000) {    // Vérification de la présence de la carte SD
    if (!SD.begin(4)) {
      error[5] = 1;
    }
    else {
      error[5] = 0;
      break;
    }
  }
  DateTime now = rtc.now();
  String date = now.toString("YYMMDD");   // Récupération de la date
  String current_file = date + "_0.LOG";    // Nom du fichier dans lequel il faut sauvegarder les données
  if (!SD.exists(current_file)) {   // Vérification si le fichier existe
    i = 0;    // Si le fichier n'existe pas, on initialise la variable i à 0
  }
  myFile = SD.open(current_file, FILE_WRITE);
  if (myFile.size() > FILE_MAX_SIZE - 80) {   // Vérification si le fichier ne dépasse pas la taille maximale ou qu'il ne va pas dépasser la taille maximale après l'écriture des données
    i++;    // Si le fichier dépasse la taille maximale, on incrémente la variable i
    String new_file = date + "_" + String(i) + ".LOG";    // Nom du nouveau fichier
    if (i > 9) {
      String new_file = date + String(i) + ".LOG";    // Si on arrive au fichier de révision n°10 on enlève "_" pour que le nom du fichier ne dépasse pas 8 caractères et soit accepté pour du FAT16
    }
    while (SD.exists(new_file)) {   // ON vérifie que le fichier n'existe déjà pas étant donné que le compteur est réinitialisé en cas de redémarrage
      i++;
      new_file = date + "_" + String(i) + ".LOG";
      if (i > 9) {
        String new_file = date + String(i) + ".LOG";
      }
    }
    myFile.rename((char*)new_file.c_str());   // Renommage du fichier actuel
    myFile.close();
    myFile = SD.open(current_file, FILE_WRITE);   // Création du nouveau fichier pour y écrire les données
  }
  if (myFile) {
    date = now.toString("hh:mm:ss");    // Écriture des données dans le fichier
    myFile.print(date);
    myFile.print("  T=");
    myFile.print(sensor_data[1]);
    myFile.print("C  H=");
    myFile.print(sensor_data[2]);
    myFile.print("%  P=");
    myFile.print(sensor_data[3]);
    myFile.print("hPa  lumin=");
    if (sensor_data[0] < LUMIN_LOW) {
      myFile.print("basse");
    }
    else if (sensor_data[0] > LUMIN_HIGH) {
      myFile.print("haute");
    }
    else {
      myFile.print("moyenne");
    }
    myFile.print("  Coordonnées=");
    myFile.print(gps_data);
    myFile.print("\n");
  }
  myFile.close();
}

void printData() {    // Fonction pour afficher les données sur le moniteur série
  Serial.print("T=");
  Serial.print(sensor_data[1]);
  Serial.print("C  H=");
  Serial.print(sensor_data[2]);
  Serial.print("%  P=");
  Serial.print(sensor_data[3]);
  Serial.print("hPa  Lumin=");
  if (sensor_data[0] < LUMIN_LOW) {
    Serial.print("basse");
  }
  else if (sensor_data[0] > LUMIN_HIGH) {
    Serial.print("haute");
  }
  else {
    Serial.print("moyenne");
  }
  Serial.print("  Coordonnées=");
  Serial.println(gps_data);
}

void setup() {
  Serial.begin(9600);   // Initialisation du port série
  pinMode(btnV, INPUT);   // Initialisation des broches en entrée
  pinMode(btnR, INPUT);
  setup_interrupt();
  leds.init();    // Initialisation de la LED
  if (EEPROM.read(255) == 0) {    // En cas de première utilisation, chargement des valeurs par défaut
    defaultValue();
  }
  if (digitalRead(btnR) == 0) {   // Si le bouton rouge est appuyé au démarrage, on entre en mode configuration
    station_mode = 4;
    Serial.println("Mode configuration\nVeuillez entrer une commande :");
    while ((millis() - config_timeout) < 1800000 && station_mode == 4) {   // Si le mode configuration est activé, on attend 30 minutes d'inactivité avant de revenir en mode standard
      commandEntry();
    }
  }
  station_mode = 1;
  Serial.println("Mode standard");
}

void loop() {
  if (station_mode == 2) {    // Si le mode économie d'énergie est activé, le temps entre deux mesures est doublé
    multiplicator = 2;
    gps = !gps;
    if (gps) {    // Permet de de récupérer les données GPS une fois sur deux en mode économique
      getGpsData();
    }
  }
  else {    // Sinon, le temps entre deux mesures est normal et on récupère les données gps
    multiplicator = 1;
    getGpsData();
  }
  getMeasures();
  getErrors();
  if (station_mode == 3) {    // Si le mode maintenance est activé, on affiche les données sur le moniteur série
    printData();
  }
  else {    // Sinon, on sauvegarde les données dans la carte SD
    saveData();
  }
  unsigned long wait_time = millis();
  while ((millis() - wait_time) < LOG_INTERVALL * 1000UL * 60 * multiplicator) {   // Temps entre deux mesures. La boucle while permet d'ajuster le temps d'attente en cas de changement de mode

  }
}