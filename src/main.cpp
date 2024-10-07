#include <Arduino.h>
#include <QTRSensors.h>
#include <headers.h>
#include <FastLED.h>
#include <WiFi.h>
#include <Wire.h>


// config: ////////////////////////////////////////////////////////////

#define UART_BAUD 9600
#define packTimeout 5 // ms (if nothing more on UART, then send packet)
#define bufferSize 8192

#define MODE_AP // phone connects directly to ESP
//#define MODE_STA // ESP connects to WiFi router

#define PROTOCOL_TCP
// #define PROTOCOL_UDP


#ifdef MODE_AP
// For AP mode:
const char *ssid = "Baratinha V5";  // You will connect your phone to this Access Point
const char *pw = "baratinha123"; // and this is the password
IPAddress ip(192, 168, 1, 47); // From RoboRemo app, connect to this IP
IPAddress netmask(255, 255, 255, 0);
const int port = 10; // and this port
// You must connect the phone to this AP, then:
// menu -> connect -> Internet(TCP) -> 192.168.0.1:9876
#endif


#ifdef MODE_STA
// For STATION mode:
const char *ssid = "Abondancia Oi Fibra 2.4GHz";  // Your ROUTER SSID
const char *pw = "aber2020"; // and WiFi PASSWORD
const int port = 10;
// You must connect the phone to the same router,
// Then somehow find the IP that the ESP got from router, then:
// menu -> connect -> Internet(TCP) -> [ESP_IP]:9876
#endif

//////////////////////////////////////////////////////////////////////////


int cmdIndex;
float recebido;


#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiServer server(port);
WiFiClient client;
#endif

#ifdef PROTOCOL_UDP
#include <WiFiUdp.h>
WiFiUDP udp;
IPAddress remoteIp;
#endif

uint8_t buf1[bufferSize];
uint16_t i1=0;

uint8_t buf2[bufferSize];
uint16_t i2=0;


//--------------------------- LEDs
#define NUM_LEDS 4
#define DATA_PIN GPIO_NUM_48
CRGB leds[NUM_LEDS];
//---------------------------

//--------------------------- Motores
//------Ponte h
#define in1 GPIO_NUM_39
#define in2 GPIO_NUM_45
#define in3 GPIO_NUM_40
#define in4 GPIO_NUM_41

#define pwmM2 GPIO_NUM_42
#define pwmM1 GPIO_NUM_46

//------Encoder
#define m1A GPIO_NUM_11
#define m1B GPIO_NUM_12
#define m2A GPIO_NUM_13
#define m2B GPIO_NUM_14
//---------------------------

//--------------------------- Sensores Linha
QTRSensors qtr;
const uint8_t SensorCount = 7;
uint16_t sensorValues[SensorCount];
#define tempoCalibracao 200

#define s1 GPIO_NUM_1
#define s2 GPIO_NUM_2
#define s3 GPIO_NUM_3
#define s4 GPIO_NUM_4
#define s5 GPIO_NUM_5
#define s6 GPIO_NUM_6
#define s7 GPIO_NUM_7
//---------------------------



//--------------------------- Botoes
#define botoes GPIO_NUM_17
int leit, lastLeit;
//keyboard biblioteca legal 
//---------------------------

//flag start/stop --> usada para pausar e despausar o robô
bool startStop = false;

//posicao de referencia para o PID
int posicao = 0; 

//erros (distancia da linha)
int error = 0;
int lastError = 0;
int second_lastError = 0;

//velocidade basal de cada motor
int M2 = 70;//Esquerda
int M1 = 70;//Direita

//velocidades maximas e minimas de cada motor  
int Mm2 = 255;
int Mm1 = 255;//Aumenta a velocidade quando erro tá pequeno(para a reta)
int MMAX = 140;//max Vel para curvas/correção
int MMAX2 = -170;//max inversão

#define setPoint 3000

float KP = 0.0704;       // proportional constant
float KI = 0.00071;    // integrative constant
float KD = 0.47;     // derivative constant
float k = 0;
int I = 0;
int modo;


#define KP_1 0.2
#define KI_1 0.0003
#define KD_1 1.75
#define M1_1 130
#define M2_1 130
#define Mm1_1 130
#define Mm2_1 130
#define MMAX_1 200
#define MMAX2_1 -200

#define KP_2 0.2
#define KI_2 0.0003
#define KD_2 1.75
#define M1_2 80
#define M2_2 80
#define Mm1_2 130
#define Mm2_2 130
#define MMAX_2 160
#define MMAX2_2 -160

#define KP_3 0.2
#define KI_3 0.0003
#define KD_3 1.75
#define M1_3 60
#define M2_3 60
#define Mm1_3 80
#define Mm2_3 80
#define MMAX_3 150
#define MMAX2_3 -150


int xaxys = 500, yaxys = 500, nxa, nya;
char sentido;

int velocidade_esquerda, velocidade_direita;



void setup() 
{
  Serial.begin(115200);


  #ifdef MODE_AP 
  //AP mode (phone connects directly to ESP) (no router)
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP 
  WiFi.softAP(ssid, pw); // configure ssid and password for softAP
  #endif

  
  #ifdef MODE_STA
  // STATION mode (ESP connects to router and gets an IP)
  // Assuming phone is also connected to that router
  // from RoboRemo you must connect to the IP of the ESP
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pw);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  #endif

  #ifdef PROTOCOL_TCP
  Serial.println("Starting TCP Server");
  server.begin(); // start TCP server 
  #endif

  #ifdef PROTOCOL_UDP
  Serial.println("Starting UDP Server");
  udp.begin(port); // start UDP server 
  #endif

  

  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  LEDS.setBrightness(100);

  //--------------------------- Setup Ponte h
  ledcSetup(0, 1000, 8);
  ledcAttachPin(pwmM1, 0);

  ledcSetup(1, 1000, 8);
  ledcAttachPin(pwmM2, 1);

  gpio_set_direction(in1, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction(in2, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction(in3, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction(in4, GPIO_MODE_INPUT_OUTPUT);
  //---------------------------

  //--------------------------- Setup Sensor de linha  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){s1, s2, s3, s4, s5, s6, s7}, SensorCount);
  //---------------------------
  //--------------------------- Setup Botoes
  gpio_set_direction(botoes, GPIO_MODE_INPUT);
  //---------------------------

  setColor('a', 120, 255, 100);
  FastLED.show();
  delay(300);
  setColor('a', 60, 255, 0);
  FastLED.show();
  delay(170);
  setColor('a', 60, 255, 100);
  FastLED.show();
  delay(300);
  setColor('a', 60, 255, 0);
  FastLED.show();
  //--------------------------- Calibração
  Serial.println("Aguardando inicio");
  // while(!(lastLeit == 0 and leit > 100))
  // {
  //   lastLeit = leit;
  //   leit = analogRead(botoes);
  // }
  // Serial.println("Iniciou");

  setColor('a', 60, 255, 100);
  delay(300);
  setColor('a', 60, 255, 0);
  delay(170);
  setColor('a', 60, 255, 100);
  delay(300);
  setColor('a', 60, 255, 0);
  



  Serial.print("Calibrando");
  setColor('a', 120, 255, 200);

  motor('e', 'f',60);
  motor('d', 't',60);
  
  for (int i = 0; i < tempoCalibracao; i++)  // calibracao
  {
    qtr.calibrate();
    if(i%5 == 0) Serial.print(".");      
  }
  motor('a', 'f',0);
  Serial.println("  Calibracao OK!");
  setColor('a', 120, 255, 0);

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  //---------------------------

  Serial.println("Fim setup.");
}

void loop() {

  #ifdef PROTOCOL_TCP
   if(!client.connected()) { // if client not connected
    client = server.available(); // wait for it to connect
    Serial.println(String(WiFi.localIP()));
    return;
  }
  if(client.available()) {
    while(client.available()) {
      buf1[i1] = (uint8_t)client.read(); // read char from client (RoboRemo app)
      if(i1<bufferSize-1) i1++;
    }
    // now send to UART:
    execute((char *)buf1);
    //Serial.write(buf1, i1);
    i1 = 0;
  }

  if(Serial.available()) {

    // read the data until pause:
    
    while(1) {
      if(Serial.available()) {
        buf2[i2] = (char)Serial.read(); // read char from UART
        if(i2<bufferSize-1) i2++;
      } else {
        //delayMicroseconds(packTimeoutMicros);
        delay(packTimeout);
        if(!Serial.available()) {
          break;
        }
      }
    }
    
    // now send to WiFi:
    client.write((char*)buf2, i2);
    client.printf((char*)buf2, i2);
    i2 = 0;
  }
  #endif

  #ifdef PROTOCOL_UDP
  // if there’s data available, read a packet
  int packetSize = udp.parsePacket();
  if(packetSize>0) {
    remoteIp = udp.remoteIP(); // store the ip of the remote device
    udp.read(buf1, bufferSize);
    // now send to UART:
    Serial.write(buf1, packetSize);
    Serial.println("opa");
  }
  IPAddress xip = udp.remoteIP();
  Serial.println(xip.toString());
  if(Serial.available())  {

    // read the data until pause:
    //Serial.println("sa");
    
    while(1) {
      Serial.println("entrei w1");
      if(Serial.available()) {
        buf2[i2] = (char)Serial.read(); // read char from UART
        if(i2<bufferSize-1) {
          i2++;
        }
      } else {
        //delayMicroseconds(packTimeoutMicros);
        //Serial.println("dl");
        delay(packTimeout);
        if(!Serial.available()) {
          //Serial.println("bk");
          break;
        }
      }
    }

    // now send to WiFi:  
    udp.beginPacket(remoteIp, port); // remote IP and port
    udp.write(buf2, i2);
    udp.endPacket();
    i2 = 0;
  }
    
  #endif

  lastLeit = leit;
  leit = analogRead(botoes);
  
  uint16_t posii;

  if(lastLeit == 0 and leit > 0)
  {
    if(leit > 4000) 
    {
      if(modo == 1) modo = 0;
      else modo = 1;
    }
    else if(leit > 1900 and leit < 2000)
    {
      if(modo == 2) modo = 5;
      else if(modo == 5) modo = 6;
      else if(modo == 6) modo = 7;
      else if(modo == 7) modo = 8;
      else modo = 2; 
    }
    else if(leit > 1250 and leit < 1400)
    {
      if(modo == 3) modo = 4;
      else modo = 3;
    } 
  }

  switch(modo)
  {
    case 0:
      setColor('a', 170, 100, 70);
      FastLED.show();
    nxa = xaxys; 
    nya = yaxys;

    // Aplica a zona morta de 460 a 540 em nya (eixo Y)
    if (nya >= 460 && nya <= 540) {
        nya = 500;  // Centraliza o valor na zona morta
    }

    // Aplica a zona morta de 460 a 540 em nxa (eixo X)
    if (nxa >= 460 && nxa <= 540) {
        nxa = 500;  // Centraliza o valor na zona morta
    }

    // Mapeia os valores de nxa e nya para o intervalo de -255 a 255
    nya = map(nya, 0, 1000, -120, 120);
    nxa = map(nxa, 0, 1000, -120, 120);

    // Verifica se nya está na zona morta para girar no próprio eixo
    if (nya == 0) {
        // Gira no próprio eixo: um motor para frente, o outro para trás
        if (nxa > 0) {
            // Gira para a direita
            motor('e', 't', nxa);  // Motor esquerdo gira para trás
            motor('d', 'f', nxa);  // Motor direito gira para frente
        } else if (nxa < 0) {
            // Gira para a esquerda
            motor('e', 'f', -nxa);  // Motor esquerdo gira para frente
            motor('d', 't', -nxa);  // Motor direito gira para trás
        } else {
            // Se nxa também está na zona morta, não faz nada (parado)
            motor('e', 'f', 0);
            motor('d', 'f', 0);
        }
    }else 
    {
        // Caso contrário, o robô se move para frente/trás conforme o eixo Y

        // Define o sentido com base no valor de nya
        if (nya >= 0) {
            sentido = 'f';  // Frente
        } else {
            sentido = 't';  // Trás
            nya = -nya;  // Usa o valor absoluto de nya
        }

        // Define as velocidades para cada motor
        velocidade_esquerda = nya - nxa;  // Ajuste para curvas
        velocidade_direita = nya + nxa;   // Ajuste para curvas

        // Limitar as velocidades para o intervalo de 0 a 255
        if (velocidade_esquerda < 0) velocidade_esquerda = 0;
        if (velocidade_esquerda > 120) velocidade_esquerda = 120;

        if (velocidade_direita < 0) velocidade_direita = 0;
        if (velocidade_direita > 120) velocidade_direita = 120;

        // Controle do motor esquerdo
        motor('e', sentido, velocidade_esquerda);

        // Controle do motor direito
        motor('d', sentido, velocidade_direita);
    }
    break;
    case 1:
      static uint8_t corzinha = 0;
      setColor('a', 0, 100, 100);
      fill_rainbow_circular(leds, 4, corzinha, 7);
      FastLED.show();
      corzinha++;
      if(corzinha == 255) corzinha = 0;

      delay(1);
      motor('a', 'f', 0);
      //Serial.println("  modo: " + String(modo));
    break;
    case 2:
      motor('a', 'f', 0);
      setColor('a', 240, 255, 100);
      Serial.print("S1: ");
      posii = qtr.readLineBlack(sensorValues);
      for(int i = 0; i < SensorCount; i++)
      {
        posii = qtr.readLineBlack(sensorValues);
        Serial.print(sensorValues[i]);
        Serial.print(" S" + String(i+2) + ": ");
      }
      posii = qtr.readLineBlack(sensorValues);
      Serial.print("  Posicao: " + String(posii) + String("   "));
      Serial.println("  modo: " + String(modo));
    break;
    case 5://esquerda frente
      setColor(0, 100, 255, 0);
      setColor(1, 100, 255, 0);
      setColor(2, 100, 255, 0);
      setColor(3, 100, 255, 100);

      motor('d', 'f', 0);
      motor('e', 'f', 100);
      Serial.println("  modo: " + String(modo));
    break;
    case 6://esquerda tras
      setColor(0, 100, 255, 100);
      setColor(1, 100, 255, 0);
      setColor(2, 100, 255, 0);
      setColor(3, 100, 255, 0);

      motor('e', 't', 100);
      motor('d', 'f', 0);
      Serial.println("  modo: " + String(modo));
    break;
    case 7://direita frente
      setColor(0, 100, 255, 0);
      setColor(1, 100, 255, 0);
      setColor(2, 100, 255, 100);
      setColor(3, 100, 255, 0);

      motor('e', 't', 0);
      motor('d', 'f', 100);
      Serial.println("  modo: " + String(modo));
    break;
    case 8://direita tras
      setColor(0, 100, 255, 0);
      setColor(1, 100, 255, 100);
      setColor(2, 100, 255, 0);
      setColor(3, 100, 255, 0);

      motor('e', 't', 0);
      motor('d', 't', 100);
      Serial.println("  modo: " + String(modo));
    break;

    case 9:
      motor('a', 'f', 100);
      break;
    case 10:
      motor('a', 't', 100);
      break;
    case 11:
      motor('e', 't', 100);
      motor('d', 'f', 100);
      break;
    case 12:
      motor('e', 'f', 100);
      motor('d', 't', 100);
    
      break;
    case 13:
      motor('a', 'f', 0);
      break;
    case 3:
    setColor('a', 120, 255, 200);
    motor('a', 'f', 0);

    break;
    case 4:
    pid_seguelinha();
    break;
  }
}


void execute(char* cmd) 
{
  Serial.print("Recebi: ");
  Serial.print(*cmd);
  Serial.print("  cmd[0] = ");
  Serial.print(cmd[0]);
  Serial.print("  cmd[0] == x? ");
  Serial.print(cmd[0] == 'x');
  Serial.print("  cmd[0] == y? ");
  Serial.print(cmd[0] == 'y');
  Serial.print("  cmd[0] == z? ");
  Serial.println(cmd[0] == 'z');
  //if(cmd[0] != ' ') return; // unknown command

  switch(cmd[0])
  {
    case 'p':
      {KP = recebido; Serial.println("KP FOI");}
      break;

    case 'i':
      {KI = recebido;Serial.println("KI FOI");}
      break;

    case 'd':
      {KD = recebido;Serial.println("KD FOI");}
      break;

    case 'x':
      Serial.print("x: ");
      xaxys = atoi(cmd+2);
      Serial.print(xaxys);
      Serial.print("  y: ");
      Serial.println(yaxys);
      break;

    case 'y':
      Serial.print("x: ");
      yaxys = atoi(cmd+2);
      Serial.print(xaxys);
      Serial.print("  y: ");
      Serial.println(yaxys);
      break;

    case 'b'://Velocidade base
      M1 = atoi(cmd+2);
      M2 = M1;
      break;

    case 'r'://Velocidade para reta
      Mm1 = atoi(cmd+2);
      Mm2 = Mm1;
      break;
      
    case 'c'://velocidade para curvas
      MMAX = atoi(cmd+2);
      break;
    case 'z':
      MMAX2 = atoi(cmd+2);
      break;
    
    default:
      if(strcmp(cmd, "S S") == 0)
      {
        //startStop = !startStop;
        //Serial.println("Entrei no Start Stop");

        modo = modo == 3 ? 4 : 3;
      }
      else
      {
        return;
      }
      break;
  }

  //atribui a recebido o float dps das duas primeiras casas de cmd
  if(strcmp(cmd, "S S") == 0)
  {
    startStop = !startStop;
    Serial.println("Entrei no Start Stop");
  }
  else
  {
    recebido = atof(cmd+2);
    if(cmd[0] == 'p'){KP = recebido; Serial.println("KP FOI");}
    else if(cmd[0] == 'i'){KI = recebido;Serial.println("KI FOI");}
    else if(cmd[0] == 'd'){KD = recebido;Serial.println("KD FOI");}
    else if(cmd[0] == 'n')
    {
      Serial.println("x foi");
      KP = KP_1; 
      KI = KI_1;
      KD = KD_1;
      M1 = M1_1;
      M2 = M2_1;
      Mm1 = Mm1_1;
      Mm2 = Mm2_1;
      MMAX = MMAX_1;
      MMAX2 = MMAX2_1;
      modo = 1;
    }
    else if(cmd[0] == 'm')
    {
      Serial.println("y foi");
      KP = KP_2;
      KI = KI_2;
      KD = KD_2;
      M1 = M1_2;
      M2 = M2_2;
      Mm1 = Mm1_2;
      Mm2 = Mm2_2;
      MMAX = MMAX_2;
      MMAX2 = MMAX2_2;
      modo = 2;
    }
    else if(cmd[0] == 'z')
    {
      Serial.println("z foi");
      KP = KP_3;
      KI = KI_3;
      KD = KD_3;
      M1 = M1_3;
      M2 = M2_3;
      Mm1 = Mm1_3;
      Mm2 = Mm2_3;
      MMAX = MMAX_3;
      MMAX2 = MMAX2_3;
      modo = 3;
    }
    else return; // comando desconhecido
  }

  // Serial.print("et: ");
  // Serial.print(startStop);
  // Serial.print("  m: ");
  // Serial.println(startStop);
  // Serial.print("Kp: ");
  // Serial.println(KP, 3);
  // Serial.print("Ki: ");
  // Serial.println(KI, 4);
  // Serial.print("Kd: ");
  // Serial.println(KD, 2);
}


void lerSens()
{
  posicao = qtr.readLineBlack(sensorValues);
  // for(int i = 0; i < SensorCount; i++)
  // {
  //   Serial.print("Sensor " + String(i+1) + ": " = String(sensorValues[i]) + String("  -> "));
  // }
  // Serial.println(posicao);
}

void pid_seguelinha()
{
    lerSens();
    int m1Speed;
    int m2Speed;
    
    second_lastError = lastError;
    lastError=error;
    error = posicao - setPoint;
    

    // Serial.println("Setpoint: " + String(setPoint) + " Erro: " + String(error) + " m1Speed: " + String(m1Speed) + " m2Speed: " + String(m2mSpeed) );
    
    if(abs(error) == 3000)
    {
        
        if(abs(lastError) < 1100 && abs(second_lastError) < 1100)
        {
            error = 0;  
            Serial.println("REEEEEEEEE");
        }  
        else Serial.println("talvez");
    } 

    I = I + error;
    
    int motorSpeed = KP * error + KD * (error - lastError) + KI*I;
    
    


    if(abs(error) == 3000)
    {
        Serial.print("  talvez Le: "+ String(abs(lastError)) + " LLE: "+String(abs(second_lastError)) + "   ");
        //delay(1300);
        if(abs(lastError) < 1000 && abs(second_lastError) < 1000)
        {
            error = 0;   
            Serial.println("REEEEEEEEE");
            for(int i = 0; i < 50; i ++)
            {
              Serial.println("REEEEEEEEE");
              motor('e','f',200);
              motor('d','f',200);
            }
            
            // delay(500);
            // motor('a','f',0);
        }  
    }  
 
    if(abs(error) <= 200)
    {
         m1Speed = Mm1 - motorSpeed;
    
         m2Speed = Mm2 + motorSpeed;  
    }
    else
    {
         m1Speed = M1 - motorSpeed;
    
         m2Speed = M2 + motorSpeed;  
    }
    if (m1Speed < MMAX2) m1Speed =  MMAX2;           // Determina o limite inferior

    //Serial.println("Setpoint: " + String(setPoint) + " Erro: " + String(error) + " m1Speed: " + String(m1Speed) + " m2Speed: " + String(m2Speed)  + " motorSpeed: " + String(motorSpeed));

    if (m2Speed < MMAX2) m2Speed = MMAX2;           // Determina o limite inferior
    
    
    if (m1Speed > MMAX) m1Speed = MMAX;     // Determina o limite superior
    
    if (m2Speed > MMAX)  m2Speed = MMAX;     // Determina o limite superior
    
    if(m1Speed < 0)  motor('d', 't', abs(m1Speed)); 
    else motor('d', 'f', abs(m1Speed));
    
   
    if(m2Speed < 0)
    {
        motor('e', 't', abs(m2Speed));
    }
    else
    {
        motor('e', 'f', abs(m2Speed));
    }

    //Serial.println("Setpoint: " + String(setPoint) + " Erro: " + String(error) + " m1Speed: " + String(m1Speed) + " m2Speed: " + String(m2Speed)  + " motorSpeed: " + String(motorSpeed));
    Serial.println("Setpoint: " + String(setPoint) + " Erro: " + String(error) + " m1Speed: " + String(m1Speed) + " m2Speed: " + String(m2Speed)  + " motorSpeed: " + String(motorSpeed));
    //Serial.println();
}

void motor(char lado, char dir , int pwm)
{
  if (lado == 'e' || lado == 'a')
  {
    if (dir == 't')
    {
      setColor(0, 200, 255, 100);
      setColor(3, 200, 255, 0);
      motorE_PWM(-pwm);
    }
    else
    {
      setColor(0, 100, 255, 0);
      setColor(3, 100, 255, 100);
      motorE_PWM(pwm);
    }
  }
  if (lado == 'd' || lado == 'a')
  {
    if (dir == 'f')
    {
      setColor(1, 100, 255, 0);
      setColor(2, 100, 255, 100);
      motorD_PWM(pwm);
    }
    else
    {
      setColor(1, 200, 255, 100);
      setColor(2, 200, 255, 0);
      motorD_PWM(-pwm);
    }
  }
}

void motorE_PWM(int vel)
{
  if(vel > 0)
  {
    gpio_set_pull_mode(in1, GPIO_PULLDOWN_ONLY);
    gpio_set_level(in2, 1);
  }
  else
  {
    gpio_set_pull_mode(in1, GPIO_PULLUP_ONLY);
    gpio_set_level(in2, 0);

    vel *= -1;
  }

  ledcWrite(0, vel);
}

void motorD_PWM(int vel)
{
  if(vel > 0)
  {
    gpio_set_pull_mode(in4, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(in3, GPIO_PULLDOWN_ONLY);

    ledcWrite(1, vel);
  }
  else
  {
    gpio_set_pull_mode(in3, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(in4, GPIO_PULLDOWN_ONLY);

    ledcWrite(1, -vel);
  }

  
}

void setColor(char sensor, int h, int s, int v)
{
  switch (sensor)
  {
  case '0':
  case 0:
    leds[0] = CHSV(h,s,v);
    break;

  case '1':
  case 1:
    leds[1] = CHSV(h,s,v);
    break;

  case '2':
  case 2:
    leds[2] = CHSV(h,s,v);
    break;

  case '3':
  case 3:
    leds[3] = CHSV(h,s,v);
    break;
  case 'a':
    leds[0] = CHSV(h,s,v);
    leds[1] = CHSV(h,s,v);
    leds[2] = CHSV(h,s,v);
    leds[3] = CHSV(h,s,v);
    break;
  default:
    leds[0] = CHSV(h,s,v);
    leds[1] = CHSV(h,s,v);
    leds[2] = CHSV(h,s,v);
    leds[3] = CHSV(h,s,v);
    break;
  }

  FastLED.show();
}