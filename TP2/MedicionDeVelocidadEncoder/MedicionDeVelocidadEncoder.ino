#define NumeroPI 3.141592653589793238462643
#define NroRanurasEncoder 20

int PinSensorVelocidad=A0;
int Ts=0;
int EstadoPrevio=0;
int EstadoActual=0;
int ContadorEncoder;
int Frecuencia=0;
int TInicial=0;
int TFinal=0;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT); //Led de control de la placa arduino uno R3 
  pinMode(PinSensorVelocidad, INPUT); 
  ConfigurarComunicacionSerial();
  ConfigurarInterrupcionTimer1();
  
  
  }



void loop() {
}

void ConfigurarComunicacionSerial(){
   Serial.begin(9600);
   while (!Serial) {
    ;              }
                              }

void ConfigurarInterrupcionTimer1(){
  noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = Ts;   // preload timer
    TCCR1B |= (1 << CS12);    // 256 prescaler 
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();}

int LeerVelocidadEncoder(){
    if(ContadorEncoder==0)
      TInicial=micros();
    EstadoActual=digitalRead(PinSensorVelocidad);
    if(EstadoActual!=EstadoPrevio){
      ContadorEncoder=ContadorEncoder+1;
      EstadoPrevio=EstadoActual;}    
    if(ContadorEncoder==2*NroRanurasEncoder){
      TFinal=micros();
      Frecuencia=1/(TFinal-TInicial);
      Serial.print(TFinal);
      ContadorEncoder=0;
                                             }
      return Frecuencia;                                       
                           
                           }

void EnviarFrecuenciaSerial(int Frecuencia){
  Serial.write(Frecuencia);
}

ISR(TIMER1_OVF_vect){
 Frecuencia=LeerVelocidadEncoder();
 //Serial.print(Frecuencia);
 //EnviarFrecuenciaSerial(Frecuencia); 
}

  
  

