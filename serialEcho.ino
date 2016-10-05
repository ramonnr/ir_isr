
#include <avr/interrupt.h>
#include <avr/io.h>

static unsigned int timerStarted=0;
static unsigned char timeoutCount = 0;
static unsigned int seconds = 0;


void outputTimePassed(unsigned int nTick, unsigned int nTimeout){
  const float ticksPerMs = 78.125;
  const float msTimeout = 838.8608;
  unsigned int lapTimeMs = nTimeout*msTimeout + nTick/ticksPerMs;
  Serial.print(lapTimeMs);
  Serial.println("ms");
  
}


void timeStamp(){
  Serial.write("0");
}

ISR(INT1_vect){
  PORTB ^= (1<<PB0);  //blink led to indicate ISR is running
  timeStamp();
  /*
  //enter critical section
  
  cli();
  PORTB ^= (1<<PB4);
  if(timerStarted == 0){
     timerStarted = 1;
     setupTimer0Interrupt();
    sei();
    return;
  }else{
    volatile int nTick = TCNT1; //get current ticks
    TCNT1 = 0;  //reset
    unsigned int nTimeout = timeoutCount;
    timeoutCount = 0;
    //exit critical section
    sei();
    outputTimePassed(nTick, nTimeout);
    
  }
  */
  /*
  PORTB |= (1<<PB0);
  delay(500);
  */
  
  
}



void outputSecMark(){
  timeoutCount=(timeoutCount+1)%15;
  //for(int i=0;i<seconds;i++){
    if(timeoutCount==14){
      PORTB ^= (1<<PB0);
      //Serial.println(seconds++);
    }
    //delay(100);
  //}
  
}

ISR(TIMER1_OVF_vect){
  timeoutCount++;
  //count++;
  //if(count>=8){
  //  count = 0;
    //outputSecMark();
    //PORTB^= (1<<PB0);
    //TIFR1 |= (1<<0);
  //}
}
  

/*
 * Timer1 is 16 bit timer 
 * To overide arduino settings for timer use = instead of |= for TCCR1B
 * Using prescaler 1024 gives 15.3 TO/s
 */
void setupTimer0Interrupt(){
  TIMSK1 |= (1 << 0);   // enable counter0 OVF ->counting RPM on future implementation
  //TCNT1 = 0x0;
  TCCR1B = 0x5; // prescale 1024 |--> CS02 = 1, CS01 = 0, CS00 = 1
}

void setupInfraRedInterrupt(){
  /*
   * EICRA  |= 0b00000011; // enable interruppt rising edge for INT0 
      EIMSK  |= 0b00000001; // enable interrupt on INT0 = 0 -->PD2 OBS NOT INT USE-->INT1 = 0 
   */
  EICRA |= 0b00001100;   //any edge on INT1
  EIMSK |= 0x2;
  EIFR  |= (1<<INTF1);
  sei();
}

void programStartBlink(unsigned int nBlinks, unsigned int delayTime){
   for(unsigned int i = 0; i < nBlinks*2; i++){
      
   }
}

void setup() {
  Serial.begin(115200);
  DDRB |= (1<<PB0);
  PORTB |= (1<<PB0);
  // put your setup code here, to run once:
  //Serial.begin(9600);
  //DDRC |= (1<<5);   //PC5 output ir led
  //PORTC |= (1<<PC5);
  DDRD &= ~(1<<PD1);  //PD1 input ir sensor
  //DDRD = 0;
  DDRB |= (1<<4);   //PB4 output led
  //PORTD |= (1<<1);  //Pull up
  //Serial.write("Starting program");
  setupInfraRedInterrupt();
  //setupTimer0Interrupt();
  PORTB |= (1<<PB4);
  delay(1000);
  PORTB &= ~(1<<PB4);
  //Serial.println("Starting program");
}

static unsigned int outputStartedFlag = 0;

void loop() {
  
   //PORTB &= ~(1<<PB0);
  /*
  if( ((PIND >> 1) &0x1 ) == 1){
    PORTB |= (1<<4);
    delay(500);
  }else{
    PORTB &= ~(1<<4);
  }
  */
}


