const bool actuatorsEnabled = true;

/*
 * Setup pins for actuator relays
 */
int relayPin1PortOpen  = 4;        // IN1 connected to digital pin 6
int relayPin2PortClose = 5;        // IN2 connected to digital pin 7
int relayPin3StarOpen  = 9;        // IN3 connected to digital pin 8
int relayPin4StarClose = 10;       // IN4 connected to digital pin 9

/*
 * Single pole dual throw "Surf Left" "Surf Right" switch pins
 */
int portSwitch = A0;               // The port gate switch analog pin 0
int starSwitch = A1;               // The starboard gate switch analog pin 1

/*
 * Airmar pulse input pin
 */
int pulsePin  = 3;
int speedDisableSwitchPin = 12;

/*
 * Status LED's
 */
int ledGreen  = 6;
int ledRed    = 7;
int ledYellow = 8;

/*
 * voilotile stuff for interupt code
 */
volatile unsigned long pulseCount;
volatile unsigned long lastPulseTime;
volatile unsigned long lastSpeedPollTime;
volatile unsigned char gotdata=0;

/*
 * toggle switch debouncing info
 */
String mode, lastMode = "";
String debouncedMode = "none";
unsigned long lastDebounceTime = 0;      // the last time the output pin was toggled
unsigned long debounceDelay    = 100;    // the debounce time; increase if the output flickers

/* 
 * setup speed paramaters 
 */
float currentSpeed = 0.0;
float minSpeed     = 7.0;
float maxSpeed     = 14.0;

/*
 * Session variables
 */
bool isPortOpen = false;   // assume gates are open on boot to force reset
bool isStarOpen = false;   // assume gates are open on boot to force reset
bool inMotion   = false;
bool isClosed   = true;
unsigned long gateStopTime = 0;
unsigned long gateDebounceTime = 0;
int gateDurationMs = 3500;  // the number of seconds the gates need to open/close


/*
 * Setup()
 */
void setup() {
  Serial.begin(9600);
  Serial.println("Surfgate system booting up...");

  pinMode(portSwitch, INPUT_PULLUP);
  pinMode(starSwitch, INPUT_PULLUP);

  // setup LEDs
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledRed, HIGH);
  digitalWrite(ledYellow, HIGH);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);

  // setup pulse input pin
  pinMode(pulsePin, INPUT);

  // setup relay pins - set HIGH (off) before enabling output
  digitalWrite(relayPin1PortOpen, HIGH);
  digitalWrite(relayPin2PortClose, HIGH);
  digitalWrite(relayPin3StarOpen, HIGH);
  digitalWrite(relayPin4StarClose, HIGH);
  pinMode(relayPin1PortOpen, OUTPUT);
  pinMode(relayPin2PortClose, OUTPUT);
  pinMode(relayPin3StarOpen, OUTPUT);
  pinMode(relayPin4StarClose, OUTPUT);

  // setup on-pcb speed override switch
  pinMode(speedDisableSwitchPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pulsePin),falling_edge,FALLING); // catch falling edge and handle with falling_edge();

  lastSpeedPollTime = micros();

  /*
   * Optional test-on-boot routines 
   */
  //test_led();
  //test_relay();

  Serial.println("System bootup complete...");
}


void loop() {

  // check for needed gate stop
  if (gateStopTime > 0 && gateStopTime < millis()) {
    gateStopTime = 0;
    stopGates();
  }

  /*
   * Read speed, check for speed bypass, set enabled flag
   */
  float kph = getSpeed();
  bool speedBypass = digitalRead(speedDisableSwitchPin);
  bool enabled = speedBypass || (kph > minSpeed && kph <= maxSpeed);  // if at speed, work to do to meet switch position, do it!

  bool isPortSet   = false;
  bool isStarSet   = false;
  bool isCenterSet = false;
  
  int portSet = analogRead(portSwitch);
  int starSet = analogRead(starSwitch);

  if (portSet < 500) {
    mode = "port";
    isPortSet = true;
  }
  else if (starSet < 500) {
    mode = "starboard";
    isStarSet = true;
  }
  else {
    mode = "center";
    isCenterSet = true;
  }
  
  // debounce the toggle switch
  if (mode != lastMode) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    debouncedMode = mode;
  }
  lastMode = mode;

  if (debouncedMode == "port") {
      if(enabled && !isPortOpen) openPortGate();
  }
  if (debouncedMode == "starboard") {
    if (enabled && !isStarOpen) openStarGate();
  }
  if (debouncedMode == "center") {
    if (!isClosed) closeGates();
  }
 
  /* 
   * LED status lights 
   */
  if (kph > maxSpeed) {
    digitalWrite(ledGreen, HIGH);
    digitalWrite(ledRed, LOW);
    digitalWrite(ledYellow, HIGH);
  }
  else if (kph < minSpeed) {
    digitalWrite(ledGreen, HIGH);
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledYellow, LOW);
  }
  else {
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledYellow, HIGH);
  }
}


/*
 * Interrupt call on falling Airmar speed pulse
 */
void falling_edge(void)
{
  //Serial.println("PULSE");
    pulseCount++;
    lastPulseTime = micros();
}


/*
 * Setup timer for gate open/close stop action 
 */
void cueStop(){
  gateStopTime = millis() + gateDurationMs;
}


/*
 * Set appropriate relay pins and state to open the Port gate
 */
void openPortGate() {
  int starAction = isStarOpen ? LOW : HIGH;
  int portAction = isPortOpen ? LOW : HIGH;
  
  inMotion   = true;
  isPortOpen = true;
  isClosed   = false;
  isStarOpen = false;

  if (actuatorsEnabled) {
    digitalWrite(relayPin1PortOpen, LOW);          // open port gate
    digitalWrite(relayPin2PortClose, HIGH);
    digitalWrite(relayPin3StarOpen, HIGH);
    digitalWrite(relayPin4StarClose, LOW);   // close starboard gate 
  } 
  cueStop(); 
  Serial.println("Open Port Gate");
}


/*
 * Set appropriate relay pins and state to open the Starboard gate
 */
void openStarGate() {
  int starAction = isStarOpen ? LOW : HIGH;
  int portAction = isPortOpen ? LOW : HIGH;

  if (actuatorsEnabled) {
    digitalWrite(relayPin1PortOpen, HIGH);
    digitalWrite(relayPin2PortClose, LOW);   // close port gate
    digitalWrite(relayPin3StarOpen, LOW);          // open starboard gate
    digitalWrite(relayPin4StarClose, HIGH);
  }  
  inMotion   = true;
  isPortOpen = false;
  isClosed   = false;
  isStarOpen = true;
  
  cueStop();

  Serial.println("Open Starboard Gate");
}


/*
 * Set appropriate relay pins and state to close both gates
 */
void closeGates() {
  int starAction = isStarOpen ? LOW : HIGH;
  int portAction = isPortOpen ? LOW : HIGH;
  
  inMotion   = true;
  isPortOpen = false;
  isClosed   = true;
  isStarOpen = false;

  if (actuatorsEnabled) {
    digitalWrite(relayPin1PortOpen, HIGH);
    digitalWrite(relayPin2PortClose, LOW);
    digitalWrite(relayPin3StarOpen, HIGH);
    digitalWrite(relayPin4StarClose, LOW);
  }  
  cueStop();
  Serial.println("Close Gates");
}


/*
 * Set appropriate relay pins and state to stop all gate action
 */
void stopGates() {
  inMotion = false;
  if (actuatorsEnabled) {
    digitalWrite(relayPin1PortOpen, HIGH);
    digitalWrite(relayPin2PortClose, HIGH);
    digitalWrite(relayPin3StarOpen, HIGH);
    digitalWrite(relayPin4StarClose, HIGH);
  }
  Serial.println("Stop Gates");
}


/*
 * Calculate speed based on Airmar speed pulses coming from paddle wheel on boat
 */
float getSpeed(){
  float microseconds = .5 * 1000000.0; // resolution
  unsigned long now = micros();
  if (now > (lastSpeedPollTime + microseconds)) {
    float seconds = ((now - lastSpeedPollTime)/1000000.0);
    float pulse_per_second = pulseCount / seconds;
    
    // convert pulses per second to MPH (or KPH)
    float kmi_per_pulse = 1./20000.;
    float kph = pulse_per_second * 3600 / 20000; // 20000 pulse per mile according to Airmar
    currentSpeed = kph * 1.377; // 1.15078;
    
    pulseCount = 0;
    lastSpeedPollTime = micros();

    //Serial.print("Seconds recorded: "); Serial.println(seconds);
    //Serial.print("Cycles recorded: "); Serial.println(pulseCount);
    //Serial.print("Pulses Per Second: "); Serial.println(pulse_per_second);
    Serial.print("Current speed: "); Serial.println(currentSpeed);
  }

  return currentSpeed;
}


/*
 * TEST CODE
 */
void test_led(void){
  // LOW  = ON, HIGH = OFF
  Serial.println("Start LED test (Green, Red, Yellow)");
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
  Serial.println("End LED test");
}

void test_relay(void) {
  // LOW  = ON, HIGH = OFF
  Serial.println("Start relay test");
  digitalWrite(relayPin1PortOpen, LOW);
  delay(500);
  digitalWrite(relayPin2PortClose, LOW);
  delay(500);
  digitalWrite(relayPin3StarOpen, LOW);
  delay(500);
  digitalWrite(relayPin4StarClose, LOW);
  delay(500);
  digitalWrite(relayPin1PortOpen, HIGH);
  delay(500);
  digitalWrite(relayPin2PortClose, HIGH);
  delay(500);
  digitalWrite(relayPin3StarOpen, HIGH);
  delay(500);
  digitalWrite(relayPin4StarClose, HIGH);
  Serial.println("End relay test");
}

