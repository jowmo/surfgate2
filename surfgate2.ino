/*
 * setup pins for actuator relays
 */
int relayPin1 = 4;                 // IN1 connected to digital pin 6
int relayPin2 = 5;                 // IN2 connected to digital pin 7
int relayPin3 = 9;                 // IN3 connected to digital pin 8
int relayPin4 = 10;                // IN4 connected to digital pin 9

int portSwitch = A0;               // The port gate switch analog pin 0
int starSwitch = A1;               // The starboard gate switch analog pin 1


/*
 * Airmar pulse input pin
 */
int pulsePin  = 3;

unsigned long duration;
int piezoPin = 12;

/*
 * Status LED's
 */
int ledGreen  = 6;
int ledRed    = 7;
int ledYellow = 8;

// speed stuff
int revolutions_per_mile = 20000;
int total_duration;
int rpm_counter;

// voilotile stuff for interupt code
volatile unsigned long pulseCount;
volatile unsigned long lastPulseTime;
volatile unsigned long lasttime;
volatile unsigned char gotdata=0;

float currentSpeed = 0.0;

/*
 * Session variables
 */
bool isPortOpen = false;
bool isStarOpen = false;
bool inMotion   = false;
bool isClosed   = false;
unsigned long gateStopTime = 0;
unsigned long gateDebounceTime = 0;
int gateDurationMs = 4000;  // the number of seconds the gates need to open/close

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  Serial.println("Surfgate system booting up...");

  pinMode(portSwitch, INPUT_PULLUP);
  pinMode(starSwitch, INPUT_PULLUP);
  
  // initialize digital pin LED_BUILTIN as an output.
  //pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledRed, HIGH);
  digitalWrite(ledYellow, HIGH);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  
  pinMode(pulsePin, INPUT);

  // setup relay pins - set HIGH (off) before enabling output
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);
  digitalWrite(relayPin3, HIGH);
  digitalWrite(relayPin4, HIGH);
  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  pinMode(relayPin3, OUTPUT);
  pinMode(relayPin4, OUTPUT);
  
  pinMode(piezoPin, OUTPUT);
  tone(piezoPin, 35, 10000000);

  rpm_counter = 0;
  total_duration = 0;

  attachInterrupt(digitalPinToInterrupt(3),falling_edge,FALLING); // catch falling edge and handle with falling_edge();

  lastPulseTime = micros();
  lasttime = micros();

  // led test
  test_led();

  // start relay test
  //test_relay();

  Serial.println("System bootup complete...");

  openStarGate();
  delay(2000);
  //openPortGate();
  //delay(2000);
  closeGates();
  //delay(2000);
}


/*
 * Interrupt call on falling Airmar speed pulse
 */
void falling_edge(void)
{
    pulseCount++;
    lastPulseTime = micros();
    Serial.println("EDGE!");
}


//
// TODO ADD A DEBOUNCER FOR THE GATE SELECTION SWITCH/LOGIC.  MAYBE TAKE NO ACTION FOR UP TO A SECOND OR SOMETHING???>
//
// the loop function runs over and over again forever
void loop() {
  // check for needed gate stop
  if (gateStopTime > 0 && gateStopTime < millis()) {
    gateStopTime = 0;
    stopGates();
  }
  
  bool isPortSet = false;
  bool isStarSet = false;
  
  int portSet = analogRead(portSwitch);
  int starSet = analogRead(starSwitch);

  if (portSet < 500) {
    isPortSet = true;
    Serial.println("PORT SET!");  Serial.println(portSet);
  }
  if (starSet < 500) {
    isStarSet = true;
    Serial.println("STAR SET!");  Serial.println(starSet);
  }

  float kph = getSpeed();
  kph = 7.1;
  Serial.print("KPH: "); Serial.println(kph);

  // if at speed, work to do to meet switch position, do it!
  if (kph > 7 && kph <= 14) {
    if (isPortSet && !isPortOpen) {
      // open port
      openPortGate();
    }
    if (isStarSet && !isStarOpen) {
      // open star
      openStarGate();
    }
    if (!isPortSet && !isStarSet && !isClosed) {
      // close
      closeGates();
    }
  }
  else {
    if (!isClosed) {
      closeGates();
    }
  }

  // led status junk...  delete?
  if (true) {
    if (kph > 30) {
      digitalWrite(ledGreen, LOW);
      digitalWrite(8, LOW);
      digitalWrite(7, LOW);
    }
    else if (kph > 20) {
      digitalWrite(ledGreen, LOW);
      digitalWrite(8, LOW);
      digitalWrite(7, HIGH);
    }
    else if (kph > 10) {
      digitalWrite(ledGreen, LOW);
      digitalWrite(8, HIGH);
      digitalWrite(7, HIGH);
    }
    else if (kph > 5) {
      digitalWrite(ledGreen, HIGH);
      digitalWrite(8, HIGH);
      digitalWrite(7, HIGH);
    }
    else {
      digitalWrite(ledGreen, HIGH);
      digitalWrite(8, HIGH);
      digitalWrite(7, HIGH);
    }

  }
  
}


void cueStop(){
  gateStopTime = millis() + gateDurationMs;
}

void openPortGate() {
  inMotion   = true;
  isPortOpen = true;
  isClosed   = false;
  isStarOpen = false;
  digitalWrite(relayPin1, LOW);
  digitalWrite(relayPin2, HIGH);
  digitalWrite(relayPin3, HIGH);
  digitalWrite(relayPin4, HIGH); 
  cueStop(); 
  Serial.println("Open Port Gate");
}
void openStarGate() {
  inMotion   = true;
  isPortOpen = false;
  isClosed   = false;
  isStarOpen = true;
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);
  digitalWrite(relayPin3, LOW);
  digitalWrite(relayPin4, HIGH);
  cueStop();
  Serial.println("Open Starboard Gate");
}
void closeGates() {
  inMotion   = true;
  isPortOpen = false;
  isClosed   = true;
  isStarOpen = false;
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, LOW);
  digitalWrite(relayPin3, HIGH);
  digitalWrite(relayPin4, LOW);
  cueStop();
  Serial.println("Close Gates");
}
void stopGates() {
  inMotion = false;
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);
  digitalWrite(relayPin3, HIGH);
  digitalWrite(relayPin4, HIGH);
  Serial.println("Stop Gates");
}

float getSpeed(){
  float microseconds = .5 * 1000000.0; // resolution
  unsigned long now = micros();
  if (now > (lasttime + microseconds)) {
    float seconds = ((now - lasttime)/1000000.0);
    float pulse_per_second = pulseCount / seconds;
    
    Serial.print("Seconds recorded: "); Serial.println(seconds);
    Serial.print("Cycles recorded: "); Serial.println(pulseCount);
    Serial.print("Pulses Per Second: "); Serial.println(pulse_per_second);

    // convert pulses per second to MPH (or KPH)
    float kmi_per_pulse = 1./20000.;
    float kph = pulse_per_second * 3600 / 20000; // 20000 pulse per mile according to Airmar
    currentSpeed = kph * 1.15078;
    
    pulseCount = 0;
    lasttime = micros();
  }

  currentSpeed += .5;

  return currentSpeed;
}


//
// TEST CODE
//
void test_led(void){
  Serial.println("Start LED Test (Green, Red, Yellow)");
  digitalWrite(ledRed, HIGH);
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledYellow, HIGH);
  digitalWrite(ledGreen, LOW);
  delay(1000);
  digitalWrite(ledRed, LOW);
  delay(1000);
  digitalWrite(ledYellow, LOW);
  delay(1000);
  digitalWrite(ledRed, HIGH);
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledYellow, HIGH);
  Serial.println("End LED Test");
}

void test_relay(void) {
  // LOW  = ON, HIGH = OFF
  digitalWrite(relayPin1, LOW);
  delay(500);
  digitalWrite(relayPin2, LOW);
  delay(500);
  digitalWrite(relayPin3, LOW);
  delay(500);
  digitalWrite(relayPin4, LOW);
  delay(500);
  digitalWrite(relayPin1, HIGH);
  delay(500);
  digitalWrite(relayPin2, HIGH);
  delay(500);
  digitalWrite(relayPin3, HIGH);
  delay(500);
  digitalWrite(relayPin4, HIGH);
  delay(500);
}

