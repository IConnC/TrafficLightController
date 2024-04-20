bool DEBUG_SENSOR = false;
bool DEBUG_TIMER = true;
bool TIMER_ACTIVE = true;

unsigned long previous_millis;
unsigned long sensor_previous_millis;

enum LightState {GREEN = 0, YELLOW = 1, RED = 2};

struct LightDirection {
  enum LightState state;
  byte pins[3];
  byte sensor_pins[4];
};

struct LightDirection NS_Direction;
struct LightDirection WE_Direction;

unsigned long time_normal;
unsigned long time_yellow;

bool sensor_interrupt;
bool allow_sensor_interrupt;
unsigned long time_sensor_cooldown;


void DirectionControl(LightDirection &direction, const LightState nextState) {
  // 2,3,4
  if (direction.state == nextState) {
    Serial.println("Invalid state change detected");
    return;
  }
  // Write old light pin low
  digitalWrite(direction.pins[direction.state], LOW);
  // Write next light pin high
  digitalWrite(direction.pins[nextState], HIGH);

  // Change State
  direction.state = nextState;
}


void setup() {
  Serial.begin(9600);

  sensor_interrupt = false;
  // Used for cooldown so sensor is not activated every loop
  allow_sensor_interrupt = true;
  time_sensor_cooldown = 10000;
  
  time_normal = 10000;
  time_yellow = 3000;


  // Setup Direction States
  NS_Direction.state = GREEN;
  NS_Direction.pins[0] = 2;
  NS_Direction.pins[1] = 3;
  NS_Direction.pins[2] = 4;
  NS_Direction.sensor_pins[0] = A0;
  NS_Direction.sensor_pins[1] = A1;
  NS_Direction.sensor_pins[2] = 12;
  NS_Direction.sensor_pins[3] = 13;
  digitalWrite(NS_Direction.pins[0], HIGH);

  WE_Direction.state = RED;
  WE_Direction.pins[0] = 5;
  WE_Direction.pins[1] = 6;
  WE_Direction.pins[2] = 7;
  WE_Direction.sensor_pins[0] = 8;
  WE_Direction.sensor_pins[1] = 9;
  WE_Direction.sensor_pins[2] = 10;
  WE_Direction.sensor_pins[3] = 11;
  digitalWrite(WE_Direction.pins[2], HIGH);
  
  // Output Pin Setup
  for (int i=0; i < 3; i++) {
    pinMode(NS_Direction.pins[i], OUTPUT);
    pinMode(WE_Direction.pins[i], OUTPUT);
  }
  // Input Pin Setup
  for (int i=0; i < 4; i++) {
    pinMode(NS_Direction.sensor_pins[i], INPUT);
    pinMode(WE_Direction.sensor_pins[i], INPUT);
  }

  // Setup Timer
  previous_millis = millis();
  sensor_previous_millis = millis();
}



void loop() {
  unsigned long current_millis = millis();

  // Sensor Testing
  if (DEBUG_SENSOR) {
    Serial.print("\nNS Sensor: ");
    for (int i=0; i < 4; i++) {
      Serial.print(digitalRead(NS_Direction.sensor_pins[i]));
      Serial.print(" ");
    }
    Serial.print("\nWE Sensor: ");
    for (int i=0; i < 4; i++) {
      Serial.print(digitalRead(WE_Direction.sensor_pins[i]));
      Serial.print(" ");
    }
    Serial.print("\n\n");
  }

  // If TIMER_ACTIVE is false, only run Sensor Testing code with 1s delay
  if (!TIMER_ACTIVE){
    delay(1000);
    return;
  }

  // Main Timer
  // Do normal timer unless switching states
  if ((current_millis - previous_millis >= time_normal | (allow_sensor_interrupt && sensor_interrupt)) && NS_Direction.state != YELLOW && WE_Direction.state != YELLOW) {
    // NS Green and WE Red start transition
    if (DEBUG_TIMER) {
      Serial.print("Normal Timer | Previous Millis: ");
      Serial.print(previous_millis);
      Serial.print(" | Current Millis: ");
      Serial.print(current_millis);
      Serial.print(" | Difference: ");
      Serial.print(current_millis - previous_millis);
      Serial.print("\n");
    }

    // Start sensor cooldown
    if (sensor_interrupt) {
      sensor_previous_millis = millis();
      allow_sensor_interrupt = false;
    }

    if (NS_Direction.state == GREEN && WE_Direction.state == RED) {
      DirectionControl(NS_Direction, LightState::YELLOW);
      Serial.println("NS: Green to Yellow");
    }
    // WE Green and NS Red start transition
    else if (WE_Direction.state == GREEN && NS_Direction.state == RED) {
      Serial.println("WE: Green to Yellow");
      DirectionControl(WE_Direction, LightState::YELLOW);
    }
    sensor_interrupt = false;
    previous_millis = millis();
  }
  // State Switch Timer
  else if (current_millis - previous_millis >= time_yellow && (NS_Direction.state == YELLOW || WE_Direction.state == YELLOW)) {
    if (DEBUG_TIMER) {
      Serial.print("Yellow Timer | Previous Millis: ");
      Serial.print(previous_millis);
      Serial.print(" | Current Millis: ");
      Serial.print(current_millis);
      Serial.print(" | Difference: ");
      Serial.print(current_millis - previous_millis);
      Serial.print("\n");
    }

    if (NS_Direction.state == YELLOW) {
      Serial.println("NS: Yellow to Red");
      DirectionControl(NS_Direction, LightState::RED);
      delay(2000);
      Serial.println("WE: Red to Green");
      DirectionControl(WE_Direction, LightState::GREEN);
    }

    else if (WE_Direction.state == YELLOW) {
      Serial.println("WE: Yellow to Red");
      DirectionControl(WE_Direction, LightState::RED);
      delay(2000);
      Serial.println("NS: Red to Green");
      DirectionControl(NS_Direction, LightState::GREEN);
    }
    previous_millis = millis();
  }

  // Sensor Input Algorithm
  
  // Read sensor input and toggle on if any sensors read true
  bool NS_sensor_active = false;
  bool WE_sensor_active = false;
  for (int i=0; i < 4; i++) {
    if (digitalRead(NS_Direction.sensor_pins[i]) == 0) {
      NS_sensor_active = true;
    }
    if (digitalRead(WE_Direction.sensor_pins[i]) == 0) {
      WE_sensor_active = true;
    }
  }

  
  if (current_millis - sensor_previous_millis >= time_sensor_cooldown) {
    allow_sensor_interrupt = true;
  }

  // Sets interupt if one direction has a red and their respective sensor is active, signal to start yellow sequence.
    sensor_interrupt = (NS_sensor_active && NS_Direction.state == RED) || (WE_sensor_active && WE_Direction.state == RED);


  if (DEBUG_SENSOR) {
    delay(500);
  }
}
