//////////////////////////////////////////////////////////////
///    ESP32 code to control milling machine              ////
//////////////////////////////////////////////////////////////

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>  // preferences library (non-volatile memory to store user variables)
//#include <Encoder.h>
#include <ESP32RotaryEncoder.h>

//// Version
const int software_version = 25;
// V1 = outline of code structure
// V2 = LCD 20x4 check
// V3 = LCD i2c scanner
// V4 = LCD 20x4 and 16x2 check
// V9 = add 2 LCD screens and the spindle encoder
// V13 = encoder pulses and rpm
// V14 = limit switches
// V15 = quill rapid, quill control, tool release, keypad
// V16 = better rpm calculation
// V18 = proper interrupts for spindle encoder (works much better now)
// V25 = pulses, limits, timing

/////////////////////////////////////////////////////////////
////                   initialise LCD screens            ////
/////////////////////////////////////////////////////////////
LiquidCrystal_I2C lcd20x4(0x27, 20, 4);  // Change 0x27 address if needed
LiquidCrystal_I2C lcd16x2(0x26, 16, 2);  // Change 0x26 address if needed

/////////////////////////////////////////////////////////////
////                   assign pins on the ESP32          ////
/////////////////////////////////////////////////////////////
// I2C
const int PIN_SDA = 21;  // LCD I2C SDA pin
const int PIN_CLK = 22;  // LCD I2C CLK pin
// Pins for the spindle rotary encoder (red wire = VCC, black wire = 0V)
const uint8_t PIN_SPINDLE_ENCODER_A = 25;  // = DT (data pin)   pulses out as encoder turns   [White wire]
const uint8_t PIN_SPINDLE_ENCODER_B = 26;  // = CLK (clock pin)   pulses showing direction of rotation  [Green wire]
// Pins for upper and lower head travel limit switches
const int PIN_UPPER_LIMIT = 4;   // microswitch head upper limit (pullup resistor OK)
const int PIN_LOWER_LIMIT = 15;  // microswitch head lower limit (pullup resistor OK)
// Menu keypad pin
const int PIN_KEY_PAD = 34;  // keypad analog input via resistor divider network
// Head step movement pins
const int PIN_HEAD_STEP_MOVE = 35;  // head movment in small steps analog input via resistor divider network
// Quill head controls
const int PIN_HEAD_CONTROL = 13;     // Quill head proportional control level (analog)
const int PIN_HEAD_RAPID_UP = 12;    // Quill head rapid up switch
const int PIN_HEAD_RAPID_DOWN = 14;  // Quill head rapid down switch
// Tool release solenoid control
const int PIN_TOOL_RELEASE_BUTTON = 5;  // button to release tool
const int PIN_TOOL_RELEASE_RELAY = 32;  // controls 3.3V relay which is connected to a 24V relay on the tool release solenoid
// Pins for servo step and direction outputs
const int PIN_SERVO_STEP = 18;  // step pin for the servo driver
const int PIN_SERVO_DIR = 19;   // dir pin for the servo driver
// E_STOP pin
const int ESP32_ESTOP = 23;  // e-stop pin



/////////////////////////////////////////////////////////////
////                   Programme variables               ////
/////////////////////////////////////////////////////////////
// Encoder SPINDLE ENCODER variables
volatile double spindle_encoder_position = 0;         // spindle encoder pulse count current value
volatile double spindle_encoder_last_position = 0;    // spindle encoder pulse count last value
volatile double spindle_encoder_pulses = 0;           // change in spindle pulse count since last trigger
volatile int spindle_encoder_direction = 0;           // 0 = not moving, 1 = clockwise, 2 = anticlockwise
volatile double spindle_encoder_rpm = 0;              // rpm calculation
volatile double spindle_encoder_time = 0;             // read current time in microseconds
volatile double spindle_encoder_last_time = 0;        // store last read time in microseconds
volatile double spindle_encoder_ppr = 500;            // number of pulses per revolution on the spindle encoder
volatile double spindle_encoder_sampletime = 300000;  // microseconds between calculating rpm
volatile int read_pin_spindle_last_encoder_B = LOW;   // Last state of Encoder B

// mechanical related variables
double head_Z_ballscrew_pitch = 5;                                           // ballscrew pitch in millimeters per revolution
double head_Z_steps_per_rev = 320;                                           // driver steps per revolution
double head_Z_servo_encoder_ppr = 10000;                                     // servo encoder pulses per revolution
double head_Z_servo_gearing = 5000;                                          // servo electronic step down gearing (configurable in servo driver parameters)
double head_Z_steps_per_mm = head_Z_steps_per_rev / head_Z_ballscrew_pitch;  // calculation for steps per mm taking into account ballscrew pitch, servo encoder ppr, servo e-gearing

// Head position and drill/tap variables
int drill_mode_selector = 0;  // 0 = normal drilling, quill handle controls head;   1 = rigid tapping, head follows spindle rotation in synch
// drilling variables
double head_Z_target_position = 0;        // main Z position in millimeters
double head_Z_target_quill_position = 0;  // additional quill position offset in millimeters (controlled by quill handle position)
double head_Z_target_total_position = 0;  // total position of head = sum of main Z position and Z quill position
// position variables
double head_Z_current_position = 0;  // actual position of head in millimeters
double head_Z_quill_range = -80;     // how far to move quill in millimeters for 100% handle movement
// velocity variables
double head_Z_velocity_mmps = 40;                                         // desired max velocity of head in mm/second
double head_Z_velocity_sps = head_Z_velocity_mmps * head_Z_steps_per_mm;  // calculated desired velocity of head in steps/second
// acceleration variables
double head_Z_acceleration_mmps = 5;  // desired max acceleration of head in mm/second^2 (rate of change of velocity every second)
double head_Z_acceleration_sps = 25;  // calculated desired acceleration of head in steps/second^2
double head_rapid = 2;                // rapid feed scalar to jump between rapid feed and quill position fine feed

// Head step timing variables
double head_Z_time = 0;                                        // current time
double head_Z_last_time = 0;                                   // store previous time to calculate how much time has passed
double head_Z_step_delay = ((1000000 / head_Z_velocity_sps));  // step generation timing delay for head servo in microseconds
double head_Z_step_counter = 0;                                // store how many steps have been taken
double prog_cycle_time = 0;                                    // timer to count cycle time

// programme timers
// double fast_code_time = 0;           // read current time
// double fast_code_last_time = 0;      // store last time
// double fast_code_sampletime = 2500000;  // interval between sampling for fast code in microseconds

double slow_code_time = 0;             // read current time
double slow_code_last_time = 0;        // store last time
double slow_code_sampletime = 500000;  // interval between sampling for slow code in microseconds

// upper and lower limit microswitch flags
int flag_upper_limit = 0;
int flag_lower_limit = 0;


// rigid tapping variables


// load meter variables
int load_meter_bars = 0;
byte fullSquare[8] = {
  B10001,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B10001
};
///////////////////////////////////////////////////////////
////                   users controls variables        ////
///////////////////////////////////////////////////////////
// tool control
int read_pin_tool_release_button = 0;  // read user tool release button state
int read_pin_head_control = 0;         // read analogue position of quill control handle
int read_pin_head_rapid_up = 0;        // read value of head rapid up button
int read_pin_head_rapid_down = 0;      // read value of head rapid down button
int mapped_pin_head_control = 0;       // mapped value to create Z offset required in millimeters

// user keypad values
int KEY_UP = 360;
int KEY_DOWN = 1140;
int KEY_LEFT = 1892;
int KEY_RIGHT = 0;
int KEY_SELECT = 3071;
int KEY_NO = 4095;        // 5V full range
int KEY_TOLERANCE = 100;  // +- tolerance for reading pin value


//////////////////////////////////////////////////////////
////                     menu structure               ////
//////////////////////////////////////////////////////////
struct menu_structure {
  String menu_label;
  int menu_value;
  int menu_inc_dec;
  int menu_value_max;
  int menu_value_min;
  int menu_factory_value;
  String menu_text;
};
// Create an array of Menu_Structure defined structures containing factory reset values
menu_structure menu[] = {
  // name, value, min, max, step, factory default

  { "Sample time (ms)", 250, 5, 2000, 25, 250, "" },  // spindle rpm calcs
  { "aaa", 1, 2, 10, 1, 5, "" },
  { "bbb", 1, 2, 10, 1, 5, "" },
  { "ccc", 1, 2, 10, 1, 5, "" },
  { "ddd", 1, 2, 10, 1, 5, "" },
  { "eee", 1, 2, 10, 1, 5, "" },
  { "fff", 1, 2, 10, 1, 5, "" },
  { "ggg", 1, 2, 10, 1, 5, "" },
  { "hhh", 1, 2, 10, 1, 5, "" }
};
const int number_of_menu_rows = sizeof(menu) / sizeof(menu[0]);
// function prototype declaration
void menu_subroutine(menu_structure menu[], int menu_size);

// Set up preferences library (where user data is stored)
Preferences preferences;


////////////////////////////////////////////////////////////////////////////////////
////           Main spindle encoder interrupt to read pulse count               ////
////////////////////////////////////////////////////////////////////////////////////
void IRAM_ATTR handleEncoder() {
  // Read the current state of Encoder B
  int currentStateB = digitalRead(read_pin_spindle_last_encoder_B);

  // Increase or decrease the encoder value based on the state of Encoder B
  if (currentStateB == HIGH) {
    spindle_encoder_pulses++;  // Moving clockwise
  } else {
    spindle_encoder_pulses--;  // Moving counter-clockwise
  }
}


////////////////////////////////////////////////////////////////////////////////////
////                            setup peripherals                               ////
////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // set up the serial monitor
  Serial.begin(115200);

  //// Initialize both LCDs (16x2 and 20x4)
  Wire.begin(PIN_SDA, PIN_CLK);  // Set the I2C SDA and SCL pins
  // 20x4
  lcd20x4.begin(20, 4);
  lcd20x4.backlight();  // Turn on the backlight for 20x4 LCD
  lcd20x4.clear();      // Clear the display
  //16x2
  lcd16x2.begin(16, 2);
  lcd16x2.backlight();  // Turn on the backlight for 16x2 LCD
  lcd16x2.clear();      // Clear the display

  // define pin types
  pinMode(PIN_SPINDLE_ENCODER_A, INPUT);  // Set pin A as input
  pinMode(PIN_SPINDLE_ENCODER_B, INPUT);  // Set pin B as input

  // Attach interrupts to encoder pin
  attachInterrupt(digitalPinToInterrupt(PIN_SPINDLE_ENCODER_A), handleEncoder, RISING);
  // Read initial state
  read_pin_spindle_last_encoder_B = digitalRead(PIN_SPINDLE_ENCODER_B);

  // limits
  pinMode(PIN_UPPER_LIMIT, INPUT_PULLUP);  // head travel limit switch
  pinMode(PIN_LOWER_LIMIT, INPUT_PULLUP);  // head travel limit switch
  // keypad
  pinMode(PIN_KEY_PAD, INPUT);  // keypad analog input (0-3.3V) (0-1023 units)
  // head incremental steps input
  pinMode(PIN_HEAD_STEP_MOVE, INPUT);  // Head step movments (0-3.3V) (0-1023 units)

  // head spindle control
  pinMode(PIN_HEAD_CONTROL, INPUT);            // Quill head proportional control level (analog)
  pinMode(PIN_HEAD_RAPID_UP, INPUT_PULLUP);    // Quill head rapid up control button
  pinMode(PIN_HEAD_RAPID_DOWN, INPUT_PULLUP);  // Quill head rapid up control button
  // tool release
  pinMode(PIN_TOOL_RELEASE_BUTTON, INPUT_PULLUP);  // Tool release button
  pinMode(PIN_TOOL_RELEASE_RELAY, OUTPUT);         //output(3.3V = release tool, 0V = no release)(controls 24V solenoid on air cylinder)

  // head servo pins
  pinMode(PIN_SERVO_STEP, OUTPUT);  // servo step pin
  pinMode(PIN_SERVO_DIR, OUTPUT);   // servo direction pin

  // estop pin
  pinMode(ESP32_ESTOP, INPUT_PULLUP);  // estop pin
}

///////////////////////////////////////////////////////////////////////////////
//                                main loop                                  //
///////////////////////////////////////////////////////////////////////////////

void loop() {

  //  check which drilling mode


  ///////////////////////////////////////////////////////////////////////////////
  //                         Normal drilling mode                              //
  ///////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////
  //  spindle encoder pulses        //
  ////////////////////////////////////
  // read internal clock counter
  spindle_encoder_time = micros();
  // check if enough time has elapsed to calculate rpm
  if (spindle_encoder_time > (spindle_encoder_last_time + spindle_encoder_sampletime)) {
    // calculate spindle encoder rpm
    spindle_encoder_rpm = (60 * 1000000 * (spindle_encoder_pulses / spindle_encoder_ppr)) / ((spindle_encoder_time - spindle_encoder_last_time));
    // reset clock counter
    spindle_encoder_last_time = spindle_encoder_time;
    // update rpm display
    lcd20x4.setCursor(0, 0);
    lcd20x4.print("eRPM ");
    lcd20x4.print(spindle_encoder_rpm, 0);
    lcd20x4.print("   ");

    // reset spindle pulse counter
    spindle_encoder_pulses = 0;

    ////////////////////////////////////////////
    // other non-time sensitive activities    //
    ////////////////////////////////////////////
    // show HEAD TARGET position
    lcd20x4.setCursor(0, 1);
    lcd20x4.print("HEAD ");
    lcd20x4.print(head_Z_target_position, 1);
    lcd20x4.print("  ");

    // display QUILL TARGET position in millimeters
    lcd20x4.setCursor(0, 2);
    lcd20x4.print("QUIL ");
    lcd20x4.print(mapped_pin_head_control);
    lcd20x4.print("  ");

    // display TOTAL TARGET position in millimeters
    lcd20x4.setCursor(0, 3);
    lcd20x4.print("TARG ");
    lcd20x4.print(head_Z_target_total_position);
    lcd20x4.print("  ");

    // display HEAD CURRENT position
    lcd20x4.setCursor(10, 0);
    lcd20x4.print("POS ");
    lcd20x4.print(head_Z_current_position, 1);
    lcd20x4.print(" ");

    // display SERVO STEP COUNTER
    lcd20x4.setCursor(10, 1);
    lcd20x4.print("ST ");
    lcd20x4.print(head_Z_step_counter, 0);
    lcd20x4.print(" ");

    // Quill head rapid
    lcd20x4.setCursor(12, 2);
    if (digitalRead(PIN_HEAD_RAPID_UP) == 0) {
      lcd20x4.print("RAUP");
    } else if (digitalRead(PIN_HEAD_RAPID_DOWN) == 0) {
      lcd20x4.print("RADO");
    } else if ((digitalRead(PIN_HEAD_RAPID_DOWN) == 1) && (digitalRead(PIN_HEAD_RAPID_UP == 1))) {
      lcd20x4.print("    ");
    }

    ////////////////////////////////////
    //       Quill head control       //
    ////////////////////////////////////
    // read quill handle position
    read_pin_head_control = analogRead(PIN_HEAD_CONTROL);
    // map it onto quill range (in millimeter)
    mapped_pin_head_control = map(read_pin_head_control, 0, 4095, 0, head_Z_quill_range);
    // set quill position
    head_Z_target_quill_position = mapped_pin_head_control;

    ////////////////////////////////////
    //       Main head control        //
    ////////////////////////////////////
    // read head movement buttons
    read_pin_head_rapid_up = digitalRead(PIN_HEAD_RAPID_UP);
    // check rapid feed joystick inputs
    // check if lower limit is trigger
    if (flag_upper_limit == 0) {
      if (read_pin_head_rapid_up == 0) {
        // head rapid up
        head_Z_target_position = head_Z_target_position + head_rapid;
      }
    }
    read_pin_head_rapid_down = digitalRead(PIN_HEAD_RAPID_DOWN);
    // check rapid feed joystick inputs
    // check if lower limit is trigger
    if (flag_lower_limit == 0) {
      if (read_pin_head_rapid_down == 0) {
        // head rapid down
        head_Z_target_position = head_Z_target_position - head_rapid;
      }
    }
  }
  /////////////////////////////////////////////////////////////////////////////////////
  //                             Time critical step generation code                  //
  /////////////////////////////////////////////////////////////////////////////////////
  // calculate total target position
  head_Z_target_total_position = head_Z_target_position + head_Z_target_quill_position;

  // calculate head_Z_step_delay (step timing)
  head_Z_step_delay = ((1000000 / head_Z_velocity_sps));

  // check if actual head position is less than target head position and move head DOWN
  // check if lower limit switch is tripped
  if (flag_lower_limit == 0) {
    if (head_Z_target_total_position < head_Z_current_position) {
      // calculate step timing to trigger pulse depending on velocity and acceleration
      head_Z_time = micros();
      if ((head_Z_time - head_Z_last_time) >= head_Z_step_delay) {
        // send direction (up)
        REG_WRITE(GPIO_OUT_W1TC_REG, BIT(PIN_SERVO_DIR));  // Set DIR_PIN LOW for DOWN direction
        // Pulse the step pin using direct port manipulation
        REG_WRITE(GPIO_OUT_W1TS_REG, BIT(PIN_SERVO_STEP));  // Set STEP_PIN high
        delayMicroseconds(5);                               // Short delay
        REG_WRITE(GPIO_OUT_W1TC_REG, BIT(PIN_SERVO_STEP));  // Set STEP_PIN low

        // reset step timer and increment step counter
        head_Z_last_time = head_Z_time;
        head_Z_step_counter--;
      }
    }
  }
  // check if actual head position is greater than target head position and move head UP
  // check if upper limit switch is tripped
  if (flag_upper_limit == 0) {
    if (head_Z_target_total_position > head_Z_current_position) {
      // calculate step timing to trigger pulse depending on velocity and acceleration
      head_Z_time = micros();
      if ((head_Z_time - head_Z_last_time) >= head_Z_step_delay) {
        // send direction (up)
        REG_WRITE(GPIO_OUT_W1TS_REG, BIT(PIN_SERVO_DIR));  // Set DIR_PIN HIGH for UP direction
        // send pulse
        // Pulse the step pin using direct port manipulation
        REG_WRITE(GPIO_OUT_W1TS_REG, BIT(PIN_SERVO_STEP));  // Set STEP_PIN high
        delayMicroseconds(5);                               // Short delay
        REG_WRITE(GPIO_OUT_W1TC_REG, BIT(PIN_SERVO_STEP));  // Set STEP_PIN low

        // reset step timer and increment step counter
        head_Z_last_time = head_Z_time;
        head_Z_step_counter++;
      }
    }
  }

  // calculate new head position (in millimeters) based on step counter
  head_Z_current_position = head_Z_step_counter / head_Z_steps_per_mm;



  //////////////////////////////////////////////////////////////////////
  //                    Tapping mode                                  //
  //////////////////////////////////////////////////////////////////////

  // insert code for rigid tapping
  // read spindle encoder position
  // work out how to move head servo to maintain synch depending on tap pitch






  //////////////////////////////////////////////////////////////////////
  //         Fast reading code applicable for all drilling modes      //
  //////////////////////////////////////////////////////////////////////

  //


  //////////////////////////////////////////////////////////////////////
  //         Slow reading code applicable for all drilling modes      //
  //////////////////////////////////////////////////////////////////////

  slow_code_time = micros();
  // check if enough time has elapsed to calculate rpm
  if (slow_code_time > (slow_code_last_time + slow_code_sampletime)) {
    ////////////////////////////////////
    //       Tool release             //
    ////////////////////////////////////
    read_pin_tool_release_button = digitalRead(PIN_TOOL_RELEASE_BUTTON);
    // if tool release button pressed
    if (read_pin_tool_release_button == 0) {
      // check if spindle is still turning
      if (spindle_encoder_rpm > 10) {
        // error, spindle still turning so cannot release tool
        lcd20x4.setCursor(18, 3);
        lcd20x4.print("ER");
        digitalWrite(PIN_TOOL_RELEASE_RELAY, LOW);
      } else {
        // release tool by triggering relay
        lcd20x4.setCursor(18, 3);
        lcd20x4.print("RE");
        digitalWrite(PIN_TOOL_RELEASE_RELAY, HIGH);
        // wait for button to be released
        while (digitalRead(PIN_TOOL_RELEASE_BUTTON) == 0) { delay(10); }
      }
    }
    // if tool release button not pressed
    if (read_pin_tool_release_button == 1) {
      // hold tool by deactivating relay
      lcd20x4.setCursor(18, 3);
      lcd20x4.print("OK");
      digitalWrite(PIN_TOOL_RELEASE_RELAY, LOW);
    }


    ////////////////////////////////////
    //     limit switches             //
    ////////////////////////////////////
    // reset upper and lower limit flags
    flag_upper_limit = 0;
    flag_lower_limit = 0;
    if (digitalRead(PIN_UPPER_LIMIT) == LOW) {
      // set upper limit flag
      flag_upper_limit = 1;
      lcd20x4.setCursor(12, 2);
      lcd20x4.print("UPLIM");
    } else if (digitalRead(PIN_LOWER_LIMIT) == LOW) {
      // set upper limit flag
      flag_lower_limit = 1;
      lcd20x4.setCursor(12, 2);
      lcd20x4.print("LOLIM");
    } else {
      lcd20x4.setCursor(12, 2);
      lcd20x4.print("     ");
    }

    ////////////////////////////////////
    //            E_STOP              //
    ////////////////////////////////////


    ////////////////////////////////////
    //        keypad buttons          //
    ////////////////////////////////////
    lcd20x4.setCursor(10, 3);
    lcd20x4.print(analogRead(PIN_KEY_PAD));
    lcd20x4.print("  ");

    // reset timer
    slow_code_last_time = slow_code_time;
  }




  // TEST CODE

  // #include <AccelStepper.h>

  // // Define stepper motor connections and interface type
  // #define STEP_PIN 32
  // #define DIR_PIN 33
  // #define ENABLE_PIN 25

  // AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

  // // S-curve parameters
  // const float maxSpeed = 1000; // Maximum speed in steps per second
  // const float acceleration = 500; // Acceleration in steps/second^2
  // const int steps = 2000; // Total steps to move

  // void setup() {
  //   Serial.begin(115200);
  //   pinMode(ENABLE_PIN, OUTPUT);
  //   digitalWrite(ENABLE_PIN, LOW); // Enable driver

  //   // Set the maximum speed and acceleration
  //   stepper.setMaxSpeed(maxSpeed);
  //   stepper.setAcceleration(acceleration);

  //   // Start the motion
  //   stepper.moveTo(steps); // Move forward
  // }

  // void loop() {
  //   // Execute the S-curve move
  //   if (stepper.distanceToGo() != 0) {
  //     stepper.run(); // Run the stepper motor
  //   } else {
  //     static int reverseSteps = -steps; // Reverse movement
  //     stepper.moveTo(reverseSteps);
  //     delay(1000); // Wait before reversing
  //   }
  // }
}
