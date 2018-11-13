#define Salida_Maxima 1000
#define Salida_Minima -5
#define h 0.0005

int estadoPin=0;
float Salida_PID=0;
float Kp=0.2;
float Ki=10;
float Kd=100;
float N=100;
float Referencia=5;
float Salida=0;
 

void setup() {
  noInterrupts();           // disable all interrupts
  pinMode(LED_BUILTIN, OUTPUT); //Led de control de la placa arduino uno R3  

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
  OCR1A = 15624/10;// = (16*10^6) / (1*1024) - 1 (must be <65536)
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
  OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect){
    Salida_PID=ControladorPID(5,Salida_PID,Kp,Kd,Ki,N);//Mas adelante Salida_PID=Salida
   
                      }
                      
ISR(TIMER1_COMPA_vect){
   Serial.println(Salida_PID);
   estadoPin=digitalRead(LED_BUILTIN);
    if (estadoPin==HIGH)
      estadoPin=LOW;
    else if(estadoPin==LOW)
      estadoPin=HIGH;
    digitalWrite(LED_BUILTIN, estadoPin);
                  }



float ControladorPID(float Referencia,float Salida_Medida,float Kp,float Kd,float Ki,float N){
 // Creo que las variables que necesito almacenar la proxima iteracion
 static float Salida_previa=0;
 static float Referencia_previa=0;
 static float Ik_previo=0;
 static float Dk_previo=0;
 float gamma;
 float Pk=0;
 float Ik=0;
 float Dk=0;
 float Salida;
 
    gamma=Kd/N; 
 //factor proporcional
    Pk= Kp*(Referencia-Salida_Medida);
  // Calculo Ik
   Ik = Ik_previo+Ki*Kp*h*(Referencia_previa-Salida_previa);  
 // Calculo factor derivativo
    Dk = gamma/(gamma+h)*Dk_previo-Kp*Kd/(gamma+h)*(Salida_Medida-Salida_previa);
 // Emito la salida
    Salida  = Ik + Dk + Pk ;

// Actualizo las variables la proxima iteracion
  Salida_previa=Salida_Medida;
  Referencia_previa=Referencia;
  Ik_previo=Ik;
  Dk_previo=Dk;
  Referencia_previa=Referencia; 
     
 // Trunco la salida si se va de rango
    if (Salida>=Salida_Maxima)
        Salida=Salida_Maxima;
    else if(Salida<=Salida_Minima)
        Salida=Salida_Minima;
    return Salida;}























