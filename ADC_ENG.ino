#define RINGBUFFER_SIZE 10
volatile uint16_t adcBuffer[RINGBUFFER_SIZE];  // Array to store ADC values
volatile int sampleCount = 0;                  // Counter to track number of collected samples
volatile float average = 0;                    // To store the calculated average value

unsigned long previousMillis1500ms = 0;
unsigned long previousMillis10ms = 0;
const long interval1500ms = 1500;
const long interval10ms = 10;

// Flag to indicate whether new data is ready (used inside ISR)
volatile bool isDataReady = false;

void setup() {
  Serial.begin(9600);
  ADMUX = 0;
  ADCSRA = 0;
  ADCSRB = 0;

  // ADC Configuration

  ADCSRA |= (1 << ADEN);
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
  
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADATE);  // Auto Trigger Enable

  // Trigger Conditions

  ADCSRB &= ~((1 << ADTS0) | (1 << ADTS1) | (1 << ADTS2));  

  // Interrupts activated

  ADCSRA |= (1 << ADIE);

  sei();

}

void loop() {

  unsigned long currentMillis = millis();

  // Every 10ms, trigger ADC conversion

  if (currentMillis - previousMillis10ms >= interval10ms) {
    previousMillis10ms = currentMillis;

    // Begin Wandlung

    ADCSRA |= (1 << ADSC);
  }

if (currentMillis - previousMillis1500ms >= interval1500ms) {
    previousMillis1500ms = currentMillis;

  // Print the buffer contents

  Serial.println("Measured values : ");

  for (int i = 0; i < RINGBUFFER_SIZE; i++) {
    Serial.print(adcBuffer[i]);
    Serial.print(" ");
  }

  Serial.println(" ");
  Serial.println("Time : Average ");
  Serial.print(currentMillis);
  Serial.print(" : ");
  Serial.print(average);
  Serial.println(" ");

  // Reset the sample count for the next cycle
  sampleCount = 0;

  // Reset the flag for next cycle
  isDataReady = false;
}
}
// ADC Interrupt Service Routine

ISR(ADC_vect) {

  // Store the ADC value in the buffer
  adcBuffer[sampleCount] = ADC;

  // Increment sample count
  sampleCount++;

  // If we reach the buffer size, calculate the average and reset
  if (sampleCount >= RINGBUFFER_SIZE) {

    average = 0;

    for (int i = 0; i < RINGBUFFER_SIZE; i++) {
      average += adcBuffer[i];
    }
    average /= RINGBUFFER_SIZE; 

    // Set the flag to indicate data is ready for processing
    isDataReady = true;

    // Reset sampleCount for the next cycle
    sampleCount = 0;
  }
}
