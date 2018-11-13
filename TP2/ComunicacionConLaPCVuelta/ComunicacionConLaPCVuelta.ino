#define FACTOR_AJUSTE_SERIAL_KP 100
#define FACTOR_AJUSTE_SERIAL_KI 100
#define FACTOR_AJUSTE_SERIAL_KD 100
#define FACTOR_AJUSTE_SERIAL_REFERENCIA 100
#define FACTOR_AJUSTE_SERIAL_PWM 1
#define FACTOR_AJUSTE_SERIAL_VELOCIDAD 100
#define FACTOR_AJUSTE_SERIAL_KP_RECEPCION 100
#define FACTOR_AJUSTE_SERIAL_KD_RECEPCION 100
#define FACTOR_AJUSTE_SERIAL_KI_RECEPCION 100
#define FACTOR_AJUSTE_SERIAL_N_RECEPCION 10
#define FACTOR_AJUSTE_SERIAL_REFERENCIA_RECEPCION 100
#define NRO_BYTES_TRAMA 10


float Kp=0.5;
float Ki=3;
float Kd=1.8;
float N=1000;
float RPM=120;
float RPM_Referencia=110;
int SalidaPID=53;
int estadoPin=0;
char Encabezado [4]={'a','b','c','d'};

void setup() {
  noInterrupts();           // disable all interrupts
  pinMode(LED_BUILTIN, OUTPUT); //Led de control de la placa arduino uno R3 
  ConfigurarComunicacionSerial();
  ConfigurarInterrupcionTimer1();
  interrupts();
  }



void loop() {


}


void ConfigurarComunicacionSerial(){
   Serial.begin(9600);
   while (!Serial) {
    ;              }
                              }

void ConfigurarInterrupcionTimer1(){
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624/4;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  }

ISR(TIMER1_COMPA_vect){
   noInterrupts();
   EnviarTramaArduinoPC();
   estadoPin=digitalRead(LED_BUILTIN);
    if (estadoPin==HIGH)
      estadoPin=LOW;
    else if(estadoPin==LOW)
      estadoPin=HIGH;
    digitalWrite(LED_BUILTIN, estadoPin);
   interrupts();

                  }

void EnviarTramaArduinoPC(){
uint8_t KpTrama=Kp*FACTOR_AJUSTE_SERIAL_KP;
uint8_t KiTrama=Ki*FACTOR_AJUSTE_SERIAL_KI;
uint8_t KdTrama=Kd*FACTOR_AJUSTE_SERIAL_KD;
uint8_t PWMTrama=SalidaPID*FACTOR_AJUSTE_SERIAL_PWM;
uint16_t ReferenciaTrama=RPM_Referencia*FACTOR_AJUSTE_SERIAL_REFERENCIA;
uint8_t ReferenciaTramaPrimerByte=highByte(ReferenciaTrama);
uint8_t ReferenciaTramaSegundoByte=lowByte(ReferenciaTrama);
uint16_t VelocidadTrama=RPM*FACTOR_AJUSTE_SERIAL_VELOCIDAD;
uint8_t VelocidadTramaPrimerByte=highByte(VelocidadTrama);
uint8_t VelocidadTramaSegundoByte=lowByte(VelocidadTrama);
uint8_t Velocidad=RPM;//La PC debe colocar la , en dos decimales
Serial.write(55);
Serial.write(56);
Serial.write(57);
Serial.write(58);
Serial.write(KpTrama);
Serial.write(KdTrama);
Serial.write(KiTrama);
Serial.write(ReferenciaTramaPrimerByte);
Serial.write(ReferenciaTramaSegundoByte);
Serial.write(PWMTrama);
Serial.write(VelocidadTramaPrimerByte);
Serial.write(VelocidadTramaSegundoByte);
}


void serialEvent(){
  noInterrupts();
  uint8_t Trama[NRO_BYTES_TRAMA];
  int i=0;
  if(Serial.available()==NRO_BYTES_TRAMA){
     for(i=0; i<NRO_BYTES_TRAMA; i=i+1){
      Trama[i] =Serial.read();}
//        Serial.write(Trama[0]);
//        Serial.write(Trama[1]);
        Serial.flush();   
  }
 if(i==(NRO_BYTES_TRAMA) && (Trama[0]=='e') && (Trama[1]=='f')&& (Trama[2]=='g') && (Trama[3]=='h')){
    Kp=float(float(Trama[4])/FACTOR_AJUSTE_SERIAL_KP_RECEPCION);
    Kd=float(float(Trama[5])/(FACTOR_AJUSTE_SERIAL_KD_RECEPCION));
    Ki= float(float(Trama[6])/(FACTOR_AJUSTE_SERIAL_KI_RECEPCION));
    RPM_Referencia=float (float(Trama[7])*256+Trama[8])/FACTOR_AJUSTE_SERIAL_REFERENCIA_RECEPCION;
    N=float (float(Trama[9])*FACTOR_AJUSTE_SERIAL_N_RECEPCION);
    i=0;
  }
  interrupts();

}
  


