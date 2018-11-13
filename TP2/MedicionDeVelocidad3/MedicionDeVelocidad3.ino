#define NumeroPI 3.141592653589793238462643
#define NroRanurasEncoder 20
#define PIN_PWM 11 //OC0A Timer2
#define ToleranciaParado 4000
int PinSensorVelocidad=A0;
int Ts=0;
int EstadoPrevio=0;
int EstadoActual=0;
int ContadorEncoder;
double RPM=0;
int CuentaCicloTrabajo=0;
int estadoPin=0;
//Debug variables
int PinDebug=4;
double aux=0;
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
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
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
        Serial.print("HOLA");
        Serial.print('\n');
     
    }




      
      return RPM;                                       





                           
                           }

void EnviarRPMSerial(double RPM){
  Serial.print("\r\n");
  Serial.print(RPM);
  Serial.print("\r\n");
}

ISR(TIMER0_COMPA_vect){
 noInterrupts();
 RPM=LeerRPMEncoder();
 analogWrite(PIN_PWM,128);
 interrupts();

                      }
                      
ISR(TIMER1_COMPA_vect){
   noInterrupts();
   EnviarRPMSerial(RPM);
   estadoPin=digitalRead(LED_BUILTIN);
    if (estadoPin==HIGH)
      estadoPin=LOW;
    else if(estadoPin==LOW)
      estadoPin=HIGH;
    digitalWrite(LED_BUILTIN, estadoPin);
   interrupts();

                  }
