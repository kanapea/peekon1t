#include <OneWire.h>
#include <EEPROM.h>

dht11 DHT11;
#define DHT11PIN 2
#define relayPin_1 3 

OneWire  ds(10);  // 4.7K takisti on vajalik

const int hygroAO = 1; //hygromeetri sisend
float dsb_result = 0.0;

void setup(void) {
  Serial.begin(9600);
}

void loop(void) {
 
Serial.begin(9600);
  

  pinMode(hygroAO, INPUT);

    hygrometer();
    delay(500); 
    dht_sensor();
    delay(500);
    dsb_sensor();
    
  dsb_result = dsb_sensor();
  Serial.println("DSB sensori andmed: ");
  Serial.print("Temperatuur:  ");
  Serial.println(dsb_result);
  delay(500);
/*
    J2rgnevad if-klauslid k2ivitavad PIN 3 relee, kui dsb_sensor funktsioon
    tagastas suurema v22rtuse kui 25.
    Alla 25 lylitab PIN 3 relee uuesti v2lja.
*/

    if (dsb_result > 25.0) {
        digitalWrite(relayPin_1, HIGH);
    }
    

  if (dsb_result < 25.0) {
    digitalWrite(relayPin_1, LOW);
  }
}


void hygrometer(){

int A0 = 0;
int sensor_val = 0;
int soil = 0;

int tmp = analogRead(hygroA0); 
// loeme ainult analoogi
Serial.println("Niiskussensori andmed: ");

/*
Selles blokis piirame hygromeetri v22rtused 485 kuni 1023 vahemikku,
kus 1023 on t2iesti kuiv ja 485 on 100% niiske. Map funktsioon jaotab 
485-1023 vahemiku sajaks protsendiks. Siinkohal tasub m2rkida, et
hygromeetril v6ib kuluda pisut aega, enne, kui ta niiskust
registreerima hakkab.
*/

if ( tmp != A0){
    A0 = tmp;
    sensor_val = constrain(sensor_val, 485, 1023);
    soil = map(sensor_val, 485,1023,0,100);
    Serial.print("Niiskusprotsent: ");
    Serial.println(soil);
}}


void dht_sensor(){

/*
Oluline on m2rkida, et DHT11 teegi pidi t66lesaamiseks manuaalselt
m6nda standardteeki kopeerima, valisime <EEPROM.h
*/

DHT11.read(DHT11PIN);
  
  Serial.println("DHT sensori andmed: "):
  Serial.print("Niiskusprotsent: ");
  Serial.println((float)DHT11.humidity, 2);

  Serial.print("Temperature (°C): ");
  Serial.println((float)DHT11.temperature, 2);
}

float dsb_sensor(){

byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  float dsb_temp;
  
    if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // alustame sensori andmete digitaalselt konvertimist
  delay(1000);  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); //Loeme Scratchpadi ehk sensori sisem2lu

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) { //vajame 9 baidi jagu andmeid
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
/*
  /* Konverdime p2riselt v22rtused. Peame kasutama int16_t tyypi, et
     garanteerida, et igal 32-bitisel platvormil j22b see int 16-bitiseks.
     Vastasel juhul v6ib m6ni systeem kohelda seda kui 8- v6i 32-bitist. 
  */
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9-bitine resolutsioon
    if (data[7] == 0x10) {
      //V6i 12-bitine resolutsioon?
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // "alumised" bitid on defineerimata, nullime need
    if (cfg == 0x00) raw = raw & ~7;  // 9-bitine resolutsioon
    else if (cfg == 0x20) raw = raw & ~3; // 10-bitine resolutsioon
    else if (cfg == 0x40) raw = raw & ~1; // 11-bitine resolutsioon
    //// Vaikimisi on 12-bitine resolutsioon.
   // S6ltuvalt resolutsioonist on see umbes 93 - 750 millisekundit.
  }
  dsb_temp = (float)raw / 16.0;
  return dsb_temp;
}
