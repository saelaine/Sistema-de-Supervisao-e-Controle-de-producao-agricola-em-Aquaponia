#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"
#define ONE_WIRE_BUS 2                  // DI - 2  DS18B20
OneWire oneWire(ONE_WIRE_BUS);          // Configuracao da instancia oneWire para se comunicar com seus dispositivos
DallasTemperature sensors(&oneWire);    // Configura oneWire com a Biblioteca DallasTemperature
#define DHTPIN 10                       // DI - 10 DHT11
#define DHTTYPE DHT11                   // Configura o tipo DHT11
DHT dht (DHTPIN, DHTTYPE);              // Configuraçao DHT11
int vazaoPin = 3;                       // DI - 3  Sensor Vazao
int phPin = A2;                         // AI - 2  Sensor Ph

#define aquecedorPin 12                   // DO - 12 Acionamento do Aquecedor
#define bombaPin 4                        // DO - 4 Acionamento Bomba                  

//========================================================================

int deviceCount = 0;                    // DS18B20
float tempC;                            // DS18B20
int chk;                                // DHT11
float hum;                              // DHT11
float TempExt;                          // DHT11
String hum1;                            // DHT11
String TempExt1;                        // DHT11
float vazao;                            // Armazena valor em L/min
volatile int contpulso;                 // Quantidade de pulsos
int a = 0;                              // Sensor Ph
float media = 0;                        // Sensor Ph
float soma = 0;                         // Sensor Ph
float countPh = 0;                      // Sensor Ph

const int intervaloDS18B20 = 1000;       // num em miliseg entre as leituras temperatura DS18B20
const int intervaloDHT = 1000;          // num em miliseg entre as leituras DHT11
const int intervaloYF = 1000;          // num em miliseg entre as leituras Sensor Vazao
const int intervaloPH = 1000;

unsigned long currentMillis = 0;                // variavel armazena valor de miliseg em cada iteraçao loop
unsigned long previousMillisDS18B20 = 0;        // hora leitura DS18B20
unsigned long previousMillisDHT = 0;            // hora leitura DHT11
unsigned long previousMillisYF = 0;             // hora leitura Sensor Vazao
unsigned int copycontpulso;                     // Vazao Sensor
unsigned long previousMillisPH = 0;


bool releaquecedor = false;             // variavel de controle estado rele aquecedor
bool relebomba = false;                 // varial controle estado rele bomba
//=============================================================================================================

void setup (void)
{
  sensors.begin();                        // Inicializa sensor DS18B20
  Serial.begin(9600);
  deviceCount = sensors.getDeviceCount(); // DS18B20
  dht.begin();                            // Inicializa DHT11

  pinMode(aquecedorPin, OUTPUT);          // DO - 12
  pinMode(bombaPin, OUTPUT);
  pinMode(vazaoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(vazaoPin), incrpulso, RISING); // Configura o pino 3(Interrupçao 1)
}

%= == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == =

     void loop()
{
  leitura();                             // Vazao
  leitura_ph();
  currentMillis = millis();              // captura o valor mais recente de milis,
  // equivalente a anotar a hora do relogio
  if (Serial.available() > 0)
  {

    char inputNode = char (Serial.read());

    switch (inputNode)
    {
      case 'S':                        // Requisiçao leituras sensores
        tempDS18B20();
        tempDHT();
        sensor_ph();
        sensor_vazao();
        break;

      case 'A':                        // Acionamento aquecedor
        aquecedorL();
        break;

      case 'B':
        aquecedorD();
        break;

      case 'C':
        bombaL();
        break;

      case 'D':
        bombaD();
        break;
    }
  }
}
//======================== Functions=================================

void tempDS18B20()
{
  if (currentMillis - previousMillisDS18B20 >= intervaloDS18B20)
  {
    previousMillisDS18B20 = currentMillis;
    sensors.requestTemperatures();
    for (int i = 0;  i < deviceCount;  i++)
    {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(" : ");
      tempC = sensors.getTempCByIndex(i);
      Serial.println(tempC);
    }
  }
}

void tempDHT()
{
  if (currentMillis - previousMillisDHT >= intervaloDHT)
  {
    previousMillisDHT = currentMillis;
    hum = dht.readHumidity();
    TempExt = dht.readTemperature();
    Serial.print("TempExt: ");
    Serial.println(TempExt);
    Serial.print("Humidity: ");
    Serial.println(hum);
  }
}

void sensor_ph()
{
  Serial.print("PH: ");
  Serial.println(media, 2);
}

void sensor_vazao()
{
  vazao = (copycontpulso / 7.5);
  Serial.print("Vazao: ");
  Serial.println(vazao);
}

void leitura()
{
  if (currentMillis - previousMillisYF >= intervaloYF)
  {
    previousMillisYF = currentMillis;
    detachInterrupt(digitalPinToInterrupt(vazaoPin));
    copycontpulso = contpulso;
    contpulso = 0;
    attachInterrupt(digitalPinToInterrupt(vazaoPin), incrpulso, RISING);
  }
}

void leitura_ph()
{
  if ((currentMillis - previousMillisPH) >= intervaloPH)
  {
    previousMillisPH = currentMillis;
    int measure = analogRead(phPin);
    double voltage = (5 / 1024.0) * measure;
    float Ph = 7 + ((2.578 - voltage) / 0.174);
    soma = soma + Ph;
    a++;
    if (a == 60)
    {
      countPh = soma / 60;
      media = countPh;
      a = 0;
      soma = 0;
    }
  }
}

void incrpulso()                          // ISR sensor vazao
{
  contpulso++;                              //Incrmenta a variavel pulsos
}

void aquecedorL()
{
  digitalWrite(aquecedorPin, HIGH);
  releaquecedor = true;
  //        Serial.print("Aquecedor: ");
  //        Serial.println(releaquecedor);
}

void aquecedorD()
{
  digitalWrite(aquecedorPin, LOW);
  releaquecedor = false;
}

void bombaL()
{
  digitalWrite(bombaPin, HIGH);
  relebomba = true;
  //        Serial.print("Bomba: ");
  //        Serial.println(relebomba);
}

void bombaD()
{
  digitalWrite(bombaPin, LOW);
  relebomba = true;
}
