int estado=0;
int Ts=40000;
void setup() {
  pinMode(LED_BUILTIN, OUTPUT); //Led de control de la placa arduino uno R3 
  ConfigurarInterrupcionTimer1();
}
  ISR(TIMER1_OVF_vect)        // interrupt service routine 
  {
    TCNT1 = Ts;   // preload timer
    estado=digitalRead(LED_BUILTIN);
    if (estado==HIGH)
      estado=LOW;
    else if(estado==LOW)
      estado=HIGH;
    digitalWrite(LED_BUILTIN, estado);   // turn the LED on (HIGH is the voltage level)
  
  }



void loop() {
  

}

void ConfigurarInterrupcionTimer1(){
  noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = Ts;   // preload timer
    TCCR1B |= (1 << CS12) | (1 << CS10);    // 1024 prescaler 
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();
                                   }




