#include <avr/io.h>
#include <util/delay.h>
#include "ADC.h"
#include "LCD.h"



#define _BV(bit)  (1<<bit)
#define BV(bit)  (1<<bit)
#define bit_is_set(ts, bit) (_TS_BYTE(ts) & _BV(bit))
#define bit_is_clear(ts, bit) (!(_TS_BYTE(ts) & _BV(bit)))
#define loop_until_bit_is_set(ts, bit) do { } while (bit_is_clear(ts, bit))
#define loop_until_bit_is_clear(ts, bit) do { } while (bit_is_set(ts, bit))

#define setBit(ts, bit)     (_TS_BYTE(ts) |= (1 << bit))
#define clearBit(ts, bit)   (_TS_BYTE(ts) &= ~(1 << bit))
#define toggleBit(ts, bit)  (_TS_BYTE(ts) ^= (1 << bit))

#define Dio_SetPinDirection(ts, bit, direction) ((direction==1) ? (setBit(ts, bit)) : (clearBit(ts, bit)))
#define Dio_SetPinState(ts, bit, state) ((state==1) ? (setBit(ts, bit)) : (clearBit(ts, bit)))
#define Dio_GetPinState(ts, bit)  (bit_is_set(ts, bit))

#define LOWER_LED PB3
#define LOWER_LED_DDR DDRB
#define LOWER_LED_PORT PORTB

#define UPPER_LED PD3
#define UPPER_LED_DDR DDRD
#define UPPER_LED_PORT PORTD

#define DH_CH 1
#define DH_DDR DDRC

#define BUTTON_CH 0
#define BUTTON_DDR DDRC


#define HTLedPin   6 //PIN 6 IN PORTD
#define LTLedPin   11 //pin 3 in PORTB

void init();



int button = 0;  //to read initial status of multiple buttons on the LCD Shield
char _str[3];    
int main(void) { 

  init(); 

  while (1) {

    //check if the buttons are adjusted, adjust the limits accordingly
    button = ADC_Read(0);
    if ((button < 200)) {
      if ((lower + 1) > upper) continue;  //in case the lower limit would be higher than the upper limit
      lower += 1;
      _delay_ms(20);  //delays added for debouncing
    } else if (button < 505) {
      upper += 1;
      _delay_ms(20);
    } else if ((button < 680)) {
      if ((upper - 1) < lower) continue;
      upper -= 1;
      _delay_ms(20);
    } else if (button < 900) {
      lower -= 1;
      _delay_ms(20);
    }

    //print the upper and lower thresholds on the LCD
    itoa(upper, _str, 10);
    LCD_String_xy(1, 10, _str);
    itoa(lower, _str, 10);
    LCD_String_xy(1, 6, _str);

    //Timer1 (Counter): Checking the temperature only after certain amounts of time
    if (TCNT1 > 64000) {
      temperatue = ADC_Read(DH_CH)*0.005/900;
      /*converting from voltage to Celsius*/
      temperatue /= 4;
      temperatue += 20;
      /*resetting the clock*/
      TCNT1 = 0;
    }

    //the lcd display is updated when Temperatue changes
    if (temperatue != last_temperature) {
      itoa(temperatue, _str, 10);
      LCD_String_xy(0, 3, _str);

      //orange led on  when T>higher, no led on at normal, green led on T<lower
      if (temperatue > upper) {
        set_dutyCycle(11, 0);
                set_dutyCycle(3, (temperatue - upper) * 10);
        LCD_String_xy(0, 8, "S:High");
      } else if (temperatue < lower) {
        set_dutyCycle(3, 0);
                set_dutyCycle(11, (lower - temperatue) * 10);
        LCD_String_xy(0, 8, "S:Low");
      } else {
        set_dutyCycle(11, 0);
        set_dutyCycle(3, 0);
        LCD_String_xy(0, 8, "S:Normal");
      }

      last_temperature = temperatue;
      _delay_ms(5);
    }
  }
  return 0;
}

void init() {
  LCD_Init();
  LCD_String_xy(0, 0, "LT:9 to ");
  LCD_String_xy(0, 5, " HT:20");
  LCD_String_xy(1, 8, " - ");
  LCD_String_xy(1, 13, "C");

  ADC_Init();
    Dio_SetPinDirection(BUTTON_DDR, BUTTON_CH, 0);

  //choose upper and lower limits by leds (orange ,green) 
  // define pins of leds  as output
  Dio_SetPinDirection(LOWER_LED_DDR, LOWER_LED, 1);
  Dio_SetPinDirection(UPPER_LED_DDR, UPPER_LED, 1);
 

  //initializing timer 1 as a counter
  TCCR1B |= (1 << CS11) | (1 << CS10);
  TCNT1 = 0;
} 






int main()
{
IO_init(); //initilization of input_outputPins
ADC_init(); //initilization of ADC
Serial.begin(9600); // set data bit comunicate serial monitor


  while(1)
{
unsigned short sensorVal=ADC_readChannel() ; //read the value of the sensor through A0
Serial.println(sensorVal);
char outputValue = map(sensorVal, 0, 1023, 0, 255); // temparture voltage from 8 to 10 bit
  float sensorVolt = sensorVal * 4.99 / 1024; //arduino voltage 
  float tempSensor = sensorVolt * 25.0 / 5.0;
  int tempInt = tempSensor * 10;
  Serial.write(outputValue); //sending temperature to the other arduino to present on LCD

  if (Serial.available()) {
    char g = Serial.read(); // comunication between computer and arduino 
    }


    
  } else {
    

  }
  


}

return 0;
}

void IO_init()
{
  SET_BIT(DDRD,HTLedPin);  //HTLedPin as output
  SET_BIT(DDRB,LTLedPin);  //HTLedPin as output
  CLEAR_BIT(DDRC,sensorPin);  //SensorPin as input
}

void ADC_init()
{
  SET_BIT(ADMUX,REFS0); //AVCC WITH EXTERNAL CAPACITOR AT AFREF
	CLEAR_BIT(ADMUX,REFS1); //AVCC WITH EXTERNAL CAPACITOR AT AFREF
	CLEAR_BIT(ADMUX,ADLAR); //right adjust
	SET_BIT(ADCSRA,ADEN); //Enable the ADC
	SET_BIT(ADCSRA,ADPS2); //F/128
	SET_BIT(ADCSRA,ADPS1); //F/128
	SET_BIT(ADCSRA,ADPS0); //F/128
}

unsigned short ADC_readChannel() //This for ADC0 only
{
	SET_BIT(ADCSRA,ADSC); //start conversion
	while(!(GET_BIT(ADCSRA,ADIF))) //wait until the conversion is done
	{
	}
	CLEAR_BIT(ADCSRA,ADIF); //CLEAR THE FLAG
	return ADC;

}


}




