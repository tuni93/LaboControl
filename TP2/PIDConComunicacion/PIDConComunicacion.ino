#define NumeroPI 3.141592653589793238462643
#define NroRanurasEncoder 20
#define PIN_PWM 11 //OC0A Timer2
#define ToleranciaParado 4000
#define h 0.000125//Ojo si la interrupcion por tiempo cambia, este lo tengo que cambiar
#define Salida_Maxima 255
#define Salida_Minima 0
#define FACTOR_AJUSTE_SERIAL_KP 10
#define FACTOR_AJUSTE_SERIAL_KI 10
#define FACTOR_AJUSTE_SERIAL_KD 100
#define FACTOR_AJUSTE_SERIAL_REFERENCIA 100
#define FACTOR_AJUSTE_SERIAL_PWM 1
#define FACTOR_AJUSTE_SERIAL_VELOCIDAD 100
#define FACTOR_AJUSTE_SERIAL_KP_RECEPCION 10
#define FACTOR_AJUSTE_SERIAL_KD_RECEPCION 100
#define FACTOR_AJUSTE_SERIAL_KI_RECEPCION 10
#define FACTOR_AJUSTE_SERIAL_N_RECEPCION 10
#define FACTOR_AJUSTE_SERIAL_REFERENCIA_RECEPCION 100
#define NRO_BYTES_TRAMA 11



int PinSensorVelocidad=A0;
int Ts=0;
int EstadoPrevio=0;
int EstadoActual=0;
int ContadorEncoder;
double RPM=0;
int CuentaCicloTrabajo=0;
int estadoPin=0;
float Salida_PID=0;
float Kp=0.5;
float Ki=3;
float Kd=0.1;
float N=1000;
float Salida=0;
float RPM_Referencia=110;
char Encabezado [4]={'a','b','c','d'};
double Dk=0;

void setup() {
  noInterrupts();           // disable all interrupts
  pinMode(LED_BUILTIN, OUTPUT); //Led de control de la placa arduino uno R3 
  pinMode(PinSensorVelocidad, OUTPUT);
  pinMode(PinSensorVelocidad, INPUT); 
  ConfigurarComunicacionSerial();
  ConfigurarInterrupcionTimer1();
  ConfigurarInterrupcionTimer0();
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


void ConfigurarInterrupcionTimer0(){
//set timer0 interrupt at 2kHz
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 64;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
}



double LeerRPMEncoder(){
    if(ContadorEncoder==0){
      CuentaCicloTrabajo=0;}
    CuentaCicloTrabajo++;
    EstadoActual=digitalRead(PinSensorVelocidad);
    if(EstadoActual!=EstadoPrevio){
      ContadorEncoder++;
      EstadoPrevio=EstadoActual;}    
    if(ContadorEncoder==2*NroRanurasEncoder){
      RPM=(double)60*(double)4000/CuentaCicloTrabajo;
      ContadorEncoder=0;                    }

if(CuentaCicloTrabajo>ToleranciaParado){           
        ContadorEncoder=0;
        CuentaCicloTrabajo=0;
        RPM=0;
     
    }

      return RPM;                                                         
                           }

ISR(TIMER0_COMPA_vect){
 noInterrupts();
 RPM=LeerRPMEncoder();
 Salida_PID=ControladorPID(RPM_Referencia,RPM,Kp,Kd,Ki,N);//Mas adelante Salida_PID=Salida
 analogWrite(PIN_PWM,Salida_PID);
 interrupts();

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

float ControladorPID(float Referencia,float Salida_Medida,float Kp,float Kd,float Ki,float N){
 // Creo que las variables que necesito almacenar la proxima iteracion
 static float Salida_previa=0;
 static float Referencia_previa=0;
 static float Ik_previo=0;
 static float Dk_previo=0;
 double gamma;
 double Pk=0;
 double Ik=0;
 //double Dk=0;
 double Salida;
 double terminod1;
 double terminod2;
    gamma=Kd/N; 
 //factor proporcional
    Pk= Kp*(Referencia-Salida_Medida);
  // Calculo Ik
   Ik = Ik_previo+Ki*Kp*h*(Referencia_previa-Salida_previa);  
 // Calculo factor derivativo
    terminod1=double(gamma*double(Dk_previo)/(gamma+h));
    terminod2=Kp*double(Kd*double(Salida_Medida-Salida_previa)/(gamma+h));
    Dk =terminod1-terminod2;
 // Emito la salida
    Salida  = Ik + Dk + Pk ;
    //Salida = Ik+Pk;
// Actualizo las variables la proxima iteracion
  Salida_previa=Salida_Medida;
  Referencia_previa=Referencia;
  Ik_previo=Ik;
  Dk_previo=Dk;     
 // Trunco la salida si se va de rango
    if (Salida>=Salida_Maxima)
        Salida=Salida_Maxima;
    else if(Salida<=Salida_Minima)
        Salida=Salida_Minima;
        
    return Salida;}

void EnviarTramaArduinoPC(){
uint8_t KpTrama=Kp*FACTOR_AJUSTE_SERIAL_KP;
uint8_t KiTrama=Ki*FACTOR_AJUSTE_SERIAL_KI;
uint8_t KdTrama=Kd*FACTOR_AJUSTE_SERIAL_KD;
uint8_t PWMTrama=Salida_PID*FACTOR_AJUSTE_SERIAL_PWM;
uint16_t ReferenciaTrama=RPM_Referencia*FACTOR_AJUSTE_SERIAL_REFERENCIA;
uint8_t ReferenciaTramaPrimerByte=highByte(ReferenciaTrama);
uint8_t ReferenciaTramaSegundoByte=lowByte(ReferenciaTrama);
uint16_t VelocidadTrama=RPM*FACTOR_AJUSTE_SERIAL_VELOCIDAD;
uint8_t VelocidadTramaPrimerByte=highByte(VelocidadTrama);
uint8_t VelocidadTramaSegundoByte=lowByte(VelocidadTrama);
uint8_t Velocidad=RPM;//La PC debe colocar la , en dos decimales
uint8_t DkSerie=Dk;
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
Serial.write(DkSerie);
//Serial.print(Kp);
//Serial.print('\t');
//Serial.print(Kd);
//Serial.print('\t');
//Serial.print(Ki);
//Serial.print('\t');
//Serial.print(RPM_Referencia);
//Serial.print('\t');
//Serial.print(N);
//Serial.print('\t');
//Serial.print(RPM);
//Serial.print('\t');
//Serial.print(Dk);
//Serial.print('\n');
}


void serialEvent(){
  noInterrupts();
  uint8_t Trama[NRO_BYTES_TRAMA];
  int i=0;
  if(Serial.available()==NRO_BYTES_TRAMA){
     for(i=0; i<NRO_BYTES_TRAMA; i=i+1){
      Trama[i] =Serial.read();}
  }
  if(i==(NRO_BYTES_TRAMA) && (Trama[0]=='e') && (Trama[1]=='f')&& (Trama[2]=='g') && (Trama[3]=='h')&& (Trama[10]==62)){
    Kp=Trama[4];
    Kp=Kp/FACTOR_AJUSTE_SERIAL_KP_RECEPCION;
    Kd=Trama[5];
    Kd=Kd/FACTOR_AJUSTE_SERIAL_KD_RECEPCION;
    Ki=Trama[6];
    Ki=Ki/(FACTOR_AJUSTE_SERIAL_KI_RECEPCION);
    RPM_Referencia=double(double(Trama[7])*256+Trama[8])/FACTOR_AJUSTE_SERIAL_REFERENCIA_RECEPCION;
    N=Trama[9]*FACTOR_AJUSTE_SERIAL_N_RECEPCION;
    i=0;
  }
  interrupts();

}
  

                  
