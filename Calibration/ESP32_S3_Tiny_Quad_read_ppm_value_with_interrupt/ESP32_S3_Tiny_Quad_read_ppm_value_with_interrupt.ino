// Pin where the PPM signal is connected
#define PPM_PIN 13 // GPIO pin for PPM input

// Number of channels
#define NUM_CHANNELS 8

// Timing constants
#define PPM_SYNC_THRESHOLD 3000 // Sync pulse threshold (microseconds)
#define CHANNEL_MIN 1000        // Minimum valid pulse width (microseconds)
#define CHANNEL_MAX 2000        // Maximum valid pulse width (microseconds)

volatile int ReceiverValue[NUM_CHANNELS]; // Store channel values
volatile int channelIndex = 0;            // Current channel index
volatile unsigned long lastTime = 0;      // Last pulse time

void ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    // Detected sync pulse, reset channel index
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    // Store the pulse width as channel value
    ReceiverValue[channelIndex] = constrain(pulseWidth, CHANNEL_MIN, CHANNEL_MAX);
    channelIndex++;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("PPM Reader Initialized");

  // Set the PPM pin as input and attach interrupt
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
}

void read_receiver(int* channelValues) {
  noInterrupts(); // Temporarily disable interrupts
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts(); // Re-enable interrupts
}

void loop() {
  int channelValues[NUM_CHANNELS];

  // Read the receiver values
  read_receiver(channelValues);

  // Display the channel values
  Serial.print(" Roll [µs]: ");
  Serial.print(channelValues[0]);
  Serial.print(" Pitch [µs]: "); 
  Serial.print(channelValues[1]);
  Serial.print(" Throttle [µs]: "); 
  Serial.print(channelValues[2]);
  Serial.print(" Yaw [µs]: "); 
  Serial.print(channelValues[3]);

  Serial.print(" SWA [µs]: "); 
  Serial.print(channelValues[4]);

  Serial.print(" SWD [µs]: "); 
  Serial.print(channelValues[5]);

  Serial.print(" SWC [µs]: "); 
  Serial.print(channelValues[6]);

  Serial.print(" SWB [µs]: "); 
  Serial.println(channelValues[7]);
  delay(50);
}
