

#define NroRanurasEncoder 20
#define PinPWM 11 //OC0A Timer2
#define PinSensorVelocidad A0

#define SalidaMaxima 255
#define SalidaMinima 0
#define ToleranciaParado 100
#define NRO_BYTES_TRAMA_RX 11
#define NumeroPI 3.141592653589793238462643



float RPM=0;
uint8_t SalidaPID=0;
float Kp=0.2;
float Ki=0.01;
float Kd=5;


uint8_t N=1;
uint8_t Bias = 0;
float Salida=0;
float ReferenciaRPM=100;


uint8_t AUX=0;

// Manejo de interrupciones
bool T0CK = false;
bool T1CK = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); //Led de control de la placa arduino uno R3 
  pinMode(PinPWM, OUTPUT);
  pinMode(PinSensorVelocidad, INPUT); 
  ConfigurarComunicacionSerial();
  ConfigurarInterrupcionTimer0(4000);// LEE LA VELOCIDAD, CALCULA Y APLICA EL PID. a 4kHz
  ConfigurarInterrupcionTimer1(10);// ENVIO TRAMA ARDUINO A PC a 10Hz
  }

void loop() {
    if (T0CK){ // Cada 4kHz
        T0CK=false;
        RPM = LeerRPMEncoder();
        SalidaPID=ControladorPID(ReferenciaRPM,RPM);
        analogWrite( PinPWM,SalidaPID);}
        
    if (T1CK){ // cada 10Hz
        T1CK=false;
        EnviarTramaArduinoPC();}
    
}


float LeerRPMEncoder(){
  static int CuentaCiclos=0;  //dt
  static int EstadoPrevio=0;
  static int EstadoActual=0;
  static float Ton=0;
  static float Toff=0;
// Cuento flancos ascendentes y descendentes
    EstadoActual=digitalRead(PinSensorVelocidad);
      if(EstadoActual != EstadoPrevio){ // Surgió un flanco!
          EstadoPrevio = EstadoActual;
          if (EstadoActual){//Flanco Ascendente
              Toff = CuentaCiclos;
              CuentaCiclos  = 0;}
          else{ // Flanco Descendente
              Ton = CuentaCiclos;
              CuentaCiclos  = 0;
              
              if (Ton && Toff)
                return 60/NroRanurasEncoder*(float(4000/(Ton+Toff)));
              }    
       }
    CuentaCiclos ++ ;
    if(CuentaCiclos > ToleranciaParado){ //NT*20=0.5seg, T=1/4000 => N=100
        CuentaCiclos=0;
        return 0;}

    }
  
uint8_t ControladorPID(float Referencia,float SalidaMedida){
 // Creo que las variables que necesito almacenar la proxima iteracion
 static float SalidaPrevia=0;
 static float ReferenciaPrevia=0;
 static float Ik_previo=0;
 static float Dk_previo=0;
 float Pk=0;
 float Ik=0;
 float Dk=0;
 float Salida;
 float gamma;
 float  h;
  h= (float)1/4000;// Ts
 
 if (!N){N=1;}
 gamma=Kd/N; 
    
 //factor proporcional
    Pk= Kp*(Referencia-SalidaMedida);
    
  // Calculo Ik
    Ik = Ik_previo+Ki*Kp*h*(ReferenciaPrevia-SalidaPrevia);  

  // Calculo factor derivativo
    Dk =gamma/(gamma+h)*Dk_previo-Kp*Kd/(gamma+h)*(SalidaMedida-SalidaPrevia);
    
  // Emito la salida
    Salida  = Ik + Dk + Pk + Bias ;

  // Actualizo las variables la proxima iteracion
    SalidaPrevia=SalidaMedida;
    ReferenciaPrevia=Referencia;
    Dk_previo=Dk;     
    Ik_previo=Ik;
    
  // Trunco la salida si se va de rango
    if (Salida>SalidaMaxima)
        Salida=SalidaMaxima;
    else if(Salida <= SalidaMinima){
        Salida=SalidaMinima;Ik_previo=0;} // Arregla que cuando la velocidad es 0
                                          // La salida del PWM se tambien cero
                                          // Y no haga el ruidito de que amaga a arrancar
    else {Ik_previo=Ik;}
    
  // Fin
    return Salida;}

void EnviarTramaArduinoPC(){
  static bool toggle = 0;
    Serial.write("efgh"); // Envio trama
    // Flotante a Punto Fijo (8,5)
    // I.F*^2^5 =[I|F]
    Serial.write((uint8_t)(Kp*32));
    Serial.write((uint8_t)(Kd*32));
    Serial.write((uint8_t)(Ki*32));
    // Flotante a Punto Fijo (16,6)
    // I.F*^2^6 =[H|L|F]
    Serial.write(  highByte( (uint16_t)(ReferenciaRPM*64) )  );
    Serial.write(  lowByte ( (uint16_t)(ReferenciaRPM*64) )  );
    Serial.write( SalidaPID );
    Serial.write(  highByte( (uint16_t)(RPM*64) )  );
    Serial.write(  lowByte ( (uint16_t)(RPM*64) )  );
    toggle = ! toggle;
    Serial.write(  toggle  );
    }

// INTERRUPCIONES
// ISR maneja automaticamente noInterrupt()... interrupt()
// Utilizarlo genera comporatmientos indeseados
//
ISR(TIMER0_COMPA_vect){T0CK=true;}// Interrumpo cada 4KHz:
          
ISR(TIMER1_COMPA_vect){T1CK=true;} //cada 10Hz

void serialEvent(){
  // Recepcion de datos
    uint8_t Trama[NRO_BYTES_TRAMA_RX];
    int i=0;
    if(Serial.available()==NRO_BYTES_TRAMA_RX){
        for(i=0; i<NRO_BYTES_TRAMA_RX; i=i+1){Trama[i] = Serial.read();}
    if( (Trama[0]=='a') && (Trama[1]=='b')&& (Trama[2]=='c') && (Trama[3]=='d')){
      //Punto Fijo (8,5) A flotante
      // [I|F]*^2^-5 =I.F
        Kp=(float)Trama[4]*0.03125;
        Kd=(float)Trama[5]*0.03125;
        Ki=(float)Trama[6]*0.03125;
        
      //Punto Fijo (16,6) A flotante
      // [I|F]*^2^-6 =I.F  
        ReferenciaRPM=(float)( (Trama[7])*256+Trama[8] )*0.015625;
        N    = Trama[9];
        Bias = Trama[10];}}
        }

//
// CONFIGURACIONES:
//
void ConfigurarComunicacionSerial(){
   Serial.begin(9600);
   while (!Serial) {;};}

void ConfigurarInterrupcionTimer0(int frec){
  // LEE LA VELOCIDAD, CALCULA Y APLICA EL PID.
// Frec Int [hz] = Arduino Clk / (preescaler * (compareMatchRegister +1))
// compareMatchRegister = ( ArduinoClk / (preescaler * FrecInt))-1
//
// Si frec = 4kHz, preescaler = 64, ArduinoClk = 16MHz
// => OCR0A = 60.5
//
    TCCR0A = 0;// set entire TCCR0A register to 0
    TCCR0B = 0;// same for TCCR0B
    TCNT0  = 0;//initialize counter value to 0

//    OCR0A = 60;// = (16E6) / (64*4000) - 1 (must be <256)
    OCR0A = (16E6 / (64*frec))-1;
  
    TCCR0A |= (1 << WGM01);// turn on CTC mode
    TCCR0B |= (1 << CS01) | (1 << CS00);   // Set CS01 and CS00 bits for 64 prescaler
    TIMSK0 |= (1 << OCIE0A);}// enable timer compare interrupt

void ConfigurarInterrupcionTimer1(int frec){
  // ENVIO TRAMA ARDUINO A PC 
// Frec Int [hz] = Arduino Clk / (preescaler * (compareMatchRegister +1))
// compareMatchRegister = ( ArduinoClk / (preescaler * FrecInt))-1
//
// Si frec = 1Hz, preescaler = 256, ArduinoClk = 16MHz
// => OCR1A = 62499


    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0

//    OCR1A = 62499;// = (16E6) / (1*256) - 1 (must be <65536)
    OCR1A = (16E6 / (256*frec))-1;
  
    TCCR1B |= (1 << WGM12);// turn on CTC mode
    TCCR1B |= (1 << CS12) ;  // Set CS12 bits for 256 prescaler
    TIMSK1 |= (1 << OCIE1A);}// enable timer compare interrupt
                    
