#define NumeroPI 3.141592653589793238462643
#define NroRanurasEncoder 20
#define PIN_PWM 11 //OC0A Timer2
#define ToleranciaParado 8000
#define h 0.000125//Ojo si la interrupcion por tiempo cambia, este lo tengo que cambiar
#define Salida_Maxima 255
#define Salida_Minima 0



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
float Ki=10;
float Kd=1.5;
float N=1000;
float Salida=0;
float RPM_Referencia=110;




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

void EnviarRPMSerial(double RPM){
  Serial.print(RPM);
  Serial.print("\t");
  Serial.println(RPM_Referencia);
}

ISR(TIMER0_COMPA_vect){
 noInterrupts();
 RPM=LeerRPMEncoder();
 Salida_PID=ControladorPID(RPM_Referencia,RPM,Kp,Kd,Ki,N);//Mas adelante Salida_PID=Salida
 Salida_PID=Salida_PID;
 analogWrite(PIN_PWM,Salida_PID);
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

float ControladorPID(float Referencia,float Salida_Medida,float Kp,float Kd,float Ki,float N){
 // Creo que las variables que necesito almacenar la proxima iteracion
 static float Salida_previa=0;
 static float Referencia_previa=0;
 static float Ik_previo=0;
 static float Dk_previo=0;
 float gamma;
 double Pk=0;
 double Ik=0;
 double Dk=0;
 double Salida;
 double terminod1;
 double terminod2;
    gamma=Kd/N; 
 //factor proporcional
    Pk= Kp*(Referencia-Salida_Medida);
  // Calculo Ik
   Ik = Ik_previo+Ki*Kp*h*(Referencia_previa-Salida_previa);  
 // Calculo factor derivativo
    terminod1=double(gamma/(gamma+h))*Dk_previo;
    terminod2=Kp*double(Kd/(gamma+h))*(Salida_Medida-Salida_previa);
    Dk =terminod1-terminod2;
 // Emito la salida
    Salida  = Ik + Dk + Pk ;

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


                  
