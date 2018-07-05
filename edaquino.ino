#define ONBOARD_LED  13  // on-board LED control pin
#define IR_LED        7  // infrared LED on/off control pin
#define PULLUP_A      3  // 10k pullup resistor switch drive port pin on channel 1 (0=ON, 1=OFF)
#define PULLUP_B      4  // 10k pullup resistor switch drive port pin on channel 2 (0=ON, 1=OFF)
#define PULLUP_C      5  // 10k pullup resistor switch drive port pin on channel 3 (0=ON, 1=OFF)
#define PULLDOWN_C    6  // 3k3 pulldown resistor switch drive port pin on channel 3 (0=OFF, 1=ON)

#define INPUT_A       3  // ADC multiplexer setting for channel 1
#define INPUT_B       4  // ADC multiplexer setting for channel 2
#define INPUT_C       5  // ADC multiplexer setting for channel 3
#define PHOTO_SENSOR  1  // ADC multiplexer setting for the internal photosensor
#define INAMP         2  // ADC multiplexer setting for the internal instrumentation amplifier

uint8_t channelArray[3] = { INPUT_A, INPUT_B, INPUT_C }; // channel sequencer array with default values
uint8_t currentChannel;    // variable used as index of channel sequencer
uint8_t averagingCode;     // code for averaging used by the host: 0,1,2,3 means 1,4,8,16 averages
uint8_t numberOfAverages;

// these variables are used both by the ADC interrupt routine and by the main code
volatile uint16_t adcData;           // used to send data from the ADC interrupt routine
volatile uint16_t adcTemporaryData;  // accumulates ADC samples
volatile uint8_t  averagingIndex;    // used to count averaged samples
volatile boolean  adcDataAvailable;  // set by the ISR if ADC data is available

uint16_t samplingFrequency;          // value of the desired sampling frequency
uint16_t timerTicksBetweenSamples;

void setup()
{
  pinMode(PULLUP_A, OUTPUT);
  pinMode(PULLUP_B, OUTPUT);
  pinMode(PULLUP_C, OUTPUT);
  pinMode(PULLDOWN_C, OUTPUT);
  pinMode(IR_LED, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(PULLUP_A, LOW);
  digitalWrite(PULLUP_B, LOW);
  digitalWrite(PULLUP_C, LOW);
  digitalWrite(PULLDOWN_C, LOW);
  digitalWrite(IR_LED, LOW);
  analogReference(EXTERNAL);
  Serial.begin(250000);
  samplingFrequency = 3 * 50; // default value, 3 channels, 50Hz per channel
  averagingCode = 0;
  timerTicksBetweenSamples = calculateTimerTicksBetweenSamples(samplingFrequency);
  for (uint8_t c = 0; c < 3; c++)  // flash the on-board LED three times to indicate booting
  {
    digitalWrite(ONBOARD_LED, LOW);
    delay(200);
    digitalWrite(ONBOARD_LED, HIGH);
    delay(200);
  }
  digitalWrite(ONBOARD_LED, LOW);
}

void loop()
{
  uint8_t c;

  while (serialReadByteWithEcho() != '@');  // all commands must be started with a '@' character
  c = serialReadByteWithEcho();
  if (c == 'I')       // send identification string command
  {
    SendID();
  }
  else if (c == 'S')     // start continuous sampling, ESC exits
  {
    startContinuousSampling();
  }
  else if (c == 'M')    // measure command, all three channels are measured and the data will be sent to the host
  {
    unsigned d;
    serialReadByteWithEcho();
    d = getAdcData(channelArray[0]) << 2;  // emulate 12-bit ADC
    serialSendByte(d >> 8);
    serialSendByte(d);
    d = getAdcData(channelArray[1]) << 2;
    serialSendByte(d >> 8);
    serialSendByte(d);
    d = getAdcData(channelArray[2]) << 2;
    serialSendByte(d >> 8);
    serialSendByte(d);
  }
  else if (c == 'f')     // set sampling frequency
  {
    averagingCode = serialReadByteWithEcho();  // code:0-3, corresponding averages: 1,4,8,16
    if (averagingCode > 3) averagingCode = 3;
    samplingFrequency = serialReadByteWithEcho();
    samplingFrequency = (samplingFrequency << 8) + serialReadByteWithEcho();
    if (samplingFrequency < 32) samplingFrequency = 32;
    if (samplingFrequency > 3000) samplingFrequency = 3000;
    timerTicksBetweenSamples = calculateTimerTicksBetweenSamples(samplingFrequency);
  }
  else if (c == 'Q')    // query parameters
  {
    uint16_t timerClockTicks;
    timerClockTicks = -timerTicksBetweenSamples;     // C8051F530 timer compatibility: 65536-value must be sent
    serialSendByte(timerClockTicks >> 8);
    serialSendByte(timerClockTicks);
    serialSendByte(averagingCode);  // code:0-3, corresponding averages: 1,4,8,16
  }
  else if (c == 'C')    // set channel sequencer for continuous sampling mode
  {
    c = serialReadByteWithEcho();     // read the selection for the first channel
    channelArray[0] = (c) ? ((c == 1) ? INAMP : PHOTO_SENSOR) : INPUT_A;
    c = serialReadByteWithEcho();     // read the selection for the second channel
    channelArray[1] = (c) ? ((c == 1) ? INAMP : PHOTO_SENSOR) : INPUT_B;
    c = serialReadByteWithEcho();     // read the selection for the third channel
    channelArray[2] = (c) ? ((c == 1) ? INAMP : PHOTO_SENSOR) : INPUT_C;
  }
  else if (c == 'P')    // switch pullup/pulldown resistors on/off
  {
    c = serialReadByteWithEcho();
    digitalWrite(PULLUP_A, (c & 1) ? LOW : HIGH);
    digitalWrite(PULLUP_B, (c & 2) ? LOW : HIGH);
    digitalWrite(PULLUP_C, (c & 4) ? LOW : HIGH);
    digitalWrite(PULLDOWN_C, (c & 8) ? HIGH : LOW);
  }
  else if (c == 'E')    // switch IR LED on/off
  {
    digitalWrite(IR_LED, serialReadByteWithEcho());
  }
}

uint16_t getAdcData(uint8_t channel)
{
  // two calls of analogRead is required to provide long enough settling time
  analogRead(channel);
  return analogRead(channel);
}

uint8_t serialGetByte(void)
{
  while (Serial.available() == 0);
  return Serial.read();
}

void serialSendByte(uint8_t a)
{
  Serial.write(a);
  Serial.flush();
}

uint8_t serialReadByteWithEcho(void)
{
  char c;
  c = serialGetByte();
  Serial.write(c);
  Serial.flush();
  return c;
}

uint16_t calculateTimerTicksBetweenSamples(uint16_t samplingFrequency)
{
  if (samplingFrequency > 1000)
  {
    averagingCode = 0;
  }
  else if (samplingFrequency > 500)
  {
    if (averagingCode > 1)
    {
      averagingCode = 1;
    }
  }
  else if (samplingFrequency > 250)
  {
    if (averagingCode > 2)
    {
      averagingCode = 2;
    }
  }
  numberOfAverages = 1;
  if (averagingCode > 0) numberOfAverages = 2 << averagingCode;
  // calculate timer1 output compare value at which timer1 resets to zero
  // 2MHz (prescaler:16MHz/8) will be used as timer1 clock
  return 2000000 / (samplingFrequency*numberOfAverages);
}

void SendID()
{
  const char *s = "EDAQ530C -> Edaquino (c) 2017-2018, http://www.inf.u-szeged.hu/miszak/en";
  do
  {
    if (serialGetByte() == 27) break; // the host can abort the process by sending an ESC character
    serialSendByte(*s);       // send the current character
    if (*s) s++;      // if there are more characters in the string, increment the pointer
  } while (*s);       // continue if the end of string has not been detected yet
}

void startContinuousSampling(void)
{
  bool stopRequested = false;
  uint8_t saveADCSRA, saveADCSRB;

  digitalWrite(ONBOARD_LED, HIGH);
  currentChannel = 0;
  adcDataAvailable = false;
  averagingIndex = numberOfAverages;
  adcTemporaryData = 0;

  noInterrupts();

  // timer1 mode: Clear Timer on Compare match A -> WGM12=1
  TCCR1A = 0;
  TCCR1B = bit(WGM12);
  // set timer1 output compare register A (OCR1A) value
  // after OCR1A-1 steps the timer restarts from zero
  // it is just the nubmer of timer clocks between A/D conversions,
  // i.e.: timer frequency / sample rate
  OCR1A = timerTicksBetweenSamples - 1;
  // timer1 compare B match register value is used to trigger the ADC
  OCR1B = 0;
  // clear timer
  TCNT1 = 0;

  // configure the ADC, save the content of the registers first
  saveADCSRA = ADCSRA;
  saveADCSRB = ADCSRB;
  // VREF and channel setup
  ADMUX = channelArray[currentChannel];
  // enable the ADC: ADEN=1
  // enable automatically triggered conversions: ADATE=1
  // enable interrupt generation: ADIE=1
  // select the ADC sucessive approximation clock:
  // ADC prescaler=128 (16MHz/128=125kHz): ADPS2=ADPS1=ADPS0=1
  ADCSRA = bit(ADEN) | bit(ADATE) | bit(ADIE) | bit(ADPS2) | bit(ADPS1) | bit(ADPS0);
  // select the A/D trigger source: timer1 compare B match
  // A/D conversion is started when match occurs
  ADCSRB = bit(ADTS2) | bit(ADTS0); // A/D trigger source: timer1 compare B match

  interrupts();       // enable interrupts

  TCCR1B |= 2;    // start timer1 with input clock 16MHz/8 = 2MHz

  while (!stopRequested)
  {
    if (adcDataAvailable)
    {
      Serial.write(adcData >> 8);
      Serial.write(adcData);
      adcDataAvailable = false;
    }
    if (Serial.available()) {
      if (Serial.read() == 27) stopRequested = true; // ESC character stops sampling
    }
  }
  TCCR1B &= ~0x07;      // stop timer1 (set input clock to none: clear the last 3 bits)
  ADCSRA = saveADCSRA;  // restore ADC register
  ADCSRB = saveADCSRB;  // restore ADC register
  digitalWrite(ONBOARD_LED, LOW);
}

// A/D conversion complete ISR
ISR(ADC_vect) {
  TIFR1 &= bit(OCF1B); // clear timer1 compare B match flag
  averagingIndex--;
  if (averagingIndex == 0)
  {
    currentChannel = (currentChannel + 1) % 3;
    ADMUX = (channelArray[currentChannel]); // set the next analog input channel
    adcTemporaryData += ADC;
    adcData = adcTemporaryData << 2;  // emulate 12-bit ADC 
    adcDataAvailable = true;
    adcTemporaryData = 0;
    averagingIndex = numberOfAverages;
  }
  else
  {
    adcTemporaryData += ADC;
  }
}

//Original sourcecode:
//http://www.noise.inf.u-szeged.hu/edudev/EDAQ530/edaq530.c


