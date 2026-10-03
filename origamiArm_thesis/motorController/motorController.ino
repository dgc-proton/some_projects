// motorMCU.ino by Dave Riley.
// This code is for the motor control MCU, part of the control system for an origami-based
// pantographic robot arm that I am developing for my MEng thesis design project.
// This code runs on an Arduino development board, moving the two stepper motor that control
// the arm position through different angles as requested by a computer that is connected to
// the Uno via serial/USB.
//
//
// Frame of reference
// Below both motors are at 0 steps / 0 degrees; this position has been chosen as the 0 point
// because the arm is not designed to get close to this configuration, meaning that the control
// algorithm does not need to account for steps overflowing beyond one rotation:
//
// |-------|          |-------|
// |       |          |       |
// |   |   |          |   |   |
// |   |   |          |   |   |
// |---+---|          |---+---|
//     |                  |
//
//
// Below both motors are at 90 degrees, number of steps = (steps per revolution / 4):
// |-------|          |-------|
// |       |          |       |
// |   ----+---       |   ----+---
// |       |          |       |
// |-------|          |-------|
//
//
// Connections for the motors from RS components that were used towards the end of summer 2025:
// red(closest to the driver chip end) - blue - black - green
// for Dave's new motors:
// blue red green black
// There is only one limit switch per motor; to the left hand side of the left motor and to the
// right hand side of the right motor.
//
// Units
// Steps are used to measure angle; this gives maximum control to the algorithm running on the
// computing device (it was requested by Zayne, the designer of the algorithm, on 11/02/2026
// to change from using tenths of a degree to using steps).

// The following macros should be commented out unless running basic debugging
// #define DEBUG_MSG_REPEAT 1
// #define DEBUG_SKIP_MOTORCAL 1

#include <Arduino.h>
#include <stdint.h>

// All constants that need to be changed to match hardware are here:
static constexpr unsigned long int BAUDRATE = 115200; // for serial comms
static constexpr uint_fast8_t MAX_MOVES = 128;        // max number of motor moves per message
static constexpr uint_fast8_t STEPS_PER_REV =
      200; // steps per revolution -> if changing this check if
           // data types for step count also need changing
static constexpr uint_fast8_t MICROS_PER_STEP = 16; // how many microsteps in a full step
static constexpr bool SWITCH_ACTIVE = LOW;
static constexpr unsigned long int PULSE_SETUP_MICROS =
      2; // minimum pulse setup time is 1us (set this a bit higher)
static constexpr unsigned long int PULSE_STEP_HIGH_MICROS =
      4; // minimum high pulse duration for a step is 2us (set this a bit higher)
static constexpr unsigned long int PULSE_STEP_LOW_MICROS =
      391; // minimum low pulse duration for a step (driver datasheet min 2), governs motor speed,
           // see speed chart, 20000 is about slowest for 0.8Nm motors, 5556 works well for full
           // step, 391 for 1/16 microstep
static constexpr uint_fast8_t RIGHT_STEP_PIN = 2;       // step pin right motor
static constexpr uint_fast8_t RIGHT_DIR_PIN = 3;        // direction pin right motor
static constexpr uint_fast8_t RIGHT_SWITCH_PIN = 12;    // clockwise limit switch for right motor
static constexpr uint_fast16_t RIGHT_SWITCH_STEPS = 82; // position of the right motor limit switch
static constexpr uint_fast8_t LEFT_STEP_PIN = 4;        // step pin left motor
static constexpr uint_fast8_t LEFT_DIR_PIN = 5;         // direction pin left motor
static constexpr uint_fast8_t LEFT_SWITCH_PIN = 11;     // anticlockwise limit switch left motor
static constexpr uint_fast16_t LEFT_SWITCH_STEPS = 112; // position of the left motor limit switch

// *Message encoding*
// A printable ASCII based system is used which tries to balance ease of debugging with
// sufficient speed of transmission.
//
// *Message format*
// Start character: always $
// Message type character: (see variables below)
// Message body: the only messages with a body are lists of angles or a single digit error code
// End character: \n (new line)
//
// Each angle is sent as 3 numbers to give the number of steps
// Lists of steps are sent comma separated, alternating left right motor
// An & character separates the left motor angle from the right motor angle
//
// e.g. a message for 120 then 121 steps for left motor, 106 then 107 steps for right, would be:
// |$|>|1|2|0|&|1|0|6|,|1|2|1|&|1|0|7|%|
// If a more robust format is desired, checksum could be introduced (at the cost of speed).
// If greater speed is desired, binary could be used in place of ASCII (at the cost of debugging
// ease).
static constexpr uint_fast8_t MSGCHAR_START = '$'; // start of message character
static constexpr uint_fast8_t MSGCHAR_END = '%';   // end of message character
static constexpr uint_fast8_t MSGCHAR_TYPE_ENQUIRE = '?';
static constexpr uint_fast8_t MSGCHAR_TYPE_ACK = '#';
static constexpr uint_fast8_t MSGCHAR_TYPE_SENDPOS = '@';   // send position in steps
static constexpr uint_fast8_t MSGCHAR_TYPE_ANGLES = '>';    // computer sending list of motor angles
static constexpr uint_fast8_t MSGCHAR_TYPE_ERRORCODE = '!'; // MCU flagging an issue
static constexpr uint_fast8_t MSGCHAR_TYPE_NOTAMSG = 255;   // reserved for use in code
static constexpr uint_fast8_t MSGCHAR_ANGLE_SEP = '&';
static constexpr uint_fast8_t MSGCHAR_DATA_SEP = ',';

struct Message
{
    uint_fast8_t msg_type;
    uint_fast8_t error_code;
    uint_fast8_t moves_per_motor;
    uint_fast8_t left_move_list[MAX_MOVES];
    uint_fast8_t right_move_list[MAX_MOVES];
};

enum DIRECTION
{
    DIR_ANTICLOCK = 0,
    DIR_CLOCK = 1
};

enum SIDE
{
    LEFT_SIDE = 0,
    RIGHT_SIDE = 1
};

struct Stepper
{
    SIDE motor_side;
    DIRECTION current_direction;
    uint_fast8_t step_pin;
    uint_fast8_t dir_pin;
    uint_fast8_t position_step_number; // step count of current position
    unsigned long int pulse_timer;
};

// function prototypes
void send_position(struct Stepper* lmotor, struct Stepper* rmotor);
void debug_message_repeater(struct Message*);
bool msgcheck(struct Message*);
bool receive_angle_pair(uint_fast8_t*, uint_fast8_t*);
uint_fast8_t move_to(struct Stepper*, uint_fast8_t);
bool step_motor(struct Stepper*, DIRECTION);
bool setup_motors(struct Stepper*, struct Stepper*);

// Arduino setup function. The only setup steps done here are ones that don't create
// additional global variables.
void setup()
{
    Serial.begin(BAUDRATE);

    // Setup pins
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RIGHT_STEP_PIN, OUTPUT);
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    pinMode(RIGHT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(LEFT_STEP_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN, OUTPUT);
    pinMode(LEFT_SWITCH_PIN, INPUT_PULLUP);
}

// Arduino main function.
void loop()
{
    // finish setup routine:
    static struct Stepper left_stepper;
    static struct Stepper right_stepper;
    static struct Message message;
    static bool success = false;
    static constexpr uint_fast8_t ack_msg[] = { MSGCHAR_START, MSGCHAR_TYPE_ACK, MSGCHAR_END };
    static constexpr uint_fast8_t error1_msg[] = {
        MSGCHAR_START, MSGCHAR_TYPE_ERRORCODE, '1', MSGCHAR_END
    }; // error 1 is message receive error
    static constexpr uint_fast8_t error2_msg[] = {
        MSGCHAR_START, MSGCHAR_TYPE_ERRORCODE, '2', MSGCHAR_END
    }; // error 2 is limit switch hit
    message.msg_type = 0;
    message.moves_per_motor = 0;
    left_stepper.position_step_number = 100;
    right_stepper.position_step_number = 100;

    setup_motors(&left_stepper, &right_stepper);

    // Conduct handshake to confirm that computer and MCU both ready to begin
    while (true) {
        success = msgcheck(&message);
        if (success && (message.msg_type == MSGCHAR_TYPE_ENQUIRE)) {
            Serial.write(ack_msg, sizeof(ack_msg) / sizeof(ack_msg[0]));
            break;
        }
    }

#ifdef DEBUG_MSG_REPEAT
    debug_message_repeater(&message);
#endif

    // loop that never returns, constantly communicating with main computer and updating
    // motor positions based on this
    while (true) {
        success = msgcheck(&message); // get the next message
        if (!success) {
            // error reading message, resend current motor position
            send_position(&left_stepper, &right_stepper);
            continue;
        }
        if (message.msg_type == MSGCHAR_TYPE_ENQUIRE) {
            // the systemcontroller is asking for the current position to be re-sent
            send_position(&left_stepper, &right_stepper);
            continue;
        }
        if (message.msg_type != MSGCHAR_TYPE_ANGLES) {
            send_position(&left_stepper, &right_stepper);
            continue;
        }

        // move the motors until completed the lists from the current message
        for (uint8_t i = 0; i < message.moves_per_motor; i++) {

            success = move_to(&left_stepper, message.left_move_list[i]);
            if (!success) {
                send_position(&left_stepper, &right_stepper);
                break;
            }
            success = move_to(&right_stepper, message.right_move_list[i]);
            if (!success) {
                send_position(&left_stepper, &right_stepper);
                break;
            }
        }

        send_position(&left_stepper, &right_stepper);
    }
}


// Send position message to the COM.
void send_position(struct Stepper* lmotor, struct Stepper* rmotor)
{
    static uint_fast8_t send_pos_begin[] = { MSGCHAR_START, MSGCHAR_TYPE_SENDPOS };

    // use conditionals to format to avoid adding e.g. sprintf to the code
    Serial.write(send_pos_begin, sizeof(send_pos_begin) / sizeof(send_pos_begin[0]));
    if (lmotor->position_step_number < 10) {
        Serial.print("00");
    } else if (lmotor->position_step_number < 100) {
        Serial.print("0");
    }
    Serial.print(lmotor->position_step_number);
    Serial.write(MSGCHAR_ANGLE_SEP);
    if (rmotor->position_step_number < 10) {
        Serial.print("00");
    } else if (rmotor->position_step_number < 100) {
        Serial.print("0");
    }
    Serial.print(rmotor->position_step_number);
    Serial.write(MSGCHAR_END);
    return;
}

// Repeats message contents back via serial for debugging. Never returns.
void debug_message_repeater(struct Message* message)
{
    static constexpr uint_fast8_t error1_msg[] = {
        MSGCHAR_START, MSGCHAR_TYPE_ERRORCODE, '1', MSGCHAR_END
    }; // error 1 is message receive error
    bool success;

    while (true) {
        success = msgcheck(message); // get the next message
        if (!success) { // if an error has occured then send error message and skip this loop
                        // itteration
            Serial.write(error1_msg, sizeof(error1_msg) / sizeof(error1_msg[0]));
            continue;
        }
        Serial.print(" msg_type: ");
        Serial.print((char)message->msg_type);
        Serial.print("  error_code: ");
        Serial.print(message->error_code);
        Serial.print("  moves_per_motor: ");
        Serial.print(message->moves_per_motor);
        Serial.print("  left_move_list: ");
        for (uint_fast8_t i = 0; i < message->moves_per_motor; i++) {
            Serial.print(message->left_move_list[i]);
            Serial.print(" ");
        }
        Serial.print("  right_move_list: ");
        for (uint_fast8_t i = 0; i < message->moves_per_motor; i++) {
            Serial.print(message->right_move_list[i]);
            Serial.print(" ");
        }
    }
}

// Checks the serial comms until a new message has been received. Once a new message has
// been received in its entirety, it will populate the msg_data variable and return true.
// If there is an error while receiving the message it will return false.
bool msgcheck(struct Message* msg_data)
{

    enum // states for the message processing state machine
    {
        NO_MSG = 0,
        MSG_STARTED,
        MSG_BODY_ANGLES,
        MSG_BODY_ERRORCODE,
        MSG_CHECK_END,
        MSG_ENDED,
        ERROR,
    } state = NO_MSG;

    uint_fast8_t angle_index = 0;
    bool success = false;

    // state machine to process the message
    while (state != MSG_ENDED) {
        switch (state) {
            case NO_MSG:                              // waiting for start character
                if (Serial.read() == MSGCHAR_START) { // Serial.read() returns -1 if no data present
                    state = MSG_STARTED;
                }
                break;
            case MSG_STARTED: // waiting for message type character
                if (Serial.available()) {
                    msg_data->msg_type = Serial.read();
                    switch (msg_data->msg_type) {
                        case MSGCHAR_TYPE_ENQUIRE:
                            state = MSG_CHECK_END;
                            break;
                        case MSGCHAR_TYPE_ACK:
                            state = MSG_CHECK_END;
                            break;
                        case MSGCHAR_TYPE_ANGLES:
                            state = MSG_BODY_ANGLES;
                            // receive the first pair of angles
                            success = receive_angle_pair(&(msg_data->left_move_list[0]),
                                                         &(msg_data->right_move_list[0]));
                            angle_index = 1;
                            if (!success) {
                                state = ERROR; // there was an error receiving the angle pair
                            }
                            break;
                        case MSGCHAR_TYPE_ERRORCODE:
                            state = MSG_BODY_ERRORCODE;
                            break;
                        default: // message type was invalid
                            state = ERROR;
                    }
                }
                break;
            case MSG_BODY_ANGLES:
                if (Serial.available()) {
                    switch (Serial.read()) {
                        case MSGCHAR_DATA_SEP: // there is another angle pair to receive
                            success = receive_angle_pair(&(msg_data->left_move_list[angle_index]),
                                                         &(msg_data->right_move_list[angle_index]));
                            angle_index++;
                            if (!success) {
                                state = ERROR;
                            }
                            break;
                        case MSGCHAR_END: // this is the end of the message
                            msg_data->moves_per_motor = angle_index;
                            state = MSG_ENDED;
                            break;
                        default: // character was invalid
                            state = ERROR;
                    }
                }
                break;
            case MSG_BODY_ERRORCODE:
                if (Serial.available()) {
                    msg_data->error_code = Serial.read();
                    state = MSG_CHECK_END;
                }
                break;
            case MSG_CHECK_END:
                if (Serial.available()) {
                    if (Serial.read() == MSGCHAR_END) {
                        state = MSG_ENDED;
                    } else {
                        state = ERROR;
                    }
                }
                break;
            case ERROR:
                return false;
        }
    }

    return true;
}


// Receives and validates a pair of angles in message body; returns true if sucessful,
// else false.
inline bool receive_angle_pair(uint_fast8_t* left_angle, uint_fast8_t* right_angle)
{
    uint_fast8_t angle_mul1;
    uint_fast8_t angle_mul10;
    uint_fast8_t angle_mul100;

    // read and process the first angle
    while (!Serial.available())
        ;                               // wait for next bit of data to be available
    angle_mul100 = Serial.read() - '0'; // read data and convert ASCII to int
    while (!Serial.available())
        ;
    angle_mul10 = Serial.read() - '0';
    while (!Serial.available())
        ;
    angle_mul1 = Serial.read() - '0';
    if (angle_mul1 > 9 || angle_mul10 > 9 || angle_mul100 > 9) {
        return false; // one of the bytes was not a valid number
    }
    *left_angle = (100 * angle_mul100) + (10 * angle_mul10) + angle_mul1;

    // check for the angle separator character
    while (!Serial.available())
        ;
    if (Serial.read() != MSGCHAR_ANGLE_SEP) {
        return false;
    }

    // read and process the second angle
    while (!Serial.available())
        ;
    angle_mul100 = Serial.read() - '0';
    while (!Serial.available())
        ;
    angle_mul10 = Serial.read() - '0';
    while (!Serial.available())
        ;
    angle_mul1 = Serial.read() - '0';
    if (angle_mul1 > 9 || angle_mul10 > 9 || angle_mul100 > 9) {
        return false; // one of the bytes was not a valid number
    }
    *right_angle = (100 * angle_mul100) + (10 * angle_mul10) + angle_mul1;

    return true;
}


// Move to a position. Returns true if successful, false if a limit switch hit.
uint_fast8_t move_to(struct Stepper* motor, uint_fast8_t desired_position)
{
    bool success;

    if (desired_position < motor->position_step_number) {
        // desired position is clockwise from current position
        while (desired_position < motor->position_step_number) {
            success = step_motor(motor, DIR_CLOCK);
            if (!success) {
                return false;
            }
        }
    } else if (desired_position > motor->position_step_number) {
        // desired position is anti-clockwise from current position
        while (desired_position > motor->position_step_number) {
            success = step_motor(motor, DIR_ANTICLOCK);
            if (!success) {
                return false;
            }
        }
    }

    return true;
}


// Step the motor. If a limit switch is hit will return false, if sucessful will return true.
// This will be executed very frequently, so has been written with some basic optimisations.
bool step_motor(struct Stepper* motor, DIRECTION dir)
{
    // Ensure motor direction is correct
    if (motor->current_direction != dir) {
        // set motor direction correctly
        unsigned long int temp_timer = micros();
        digitalWrite(motor->dir_pin, dir);
        motor->current_direction = dir;
        while ((micros() - temp_timer) < PULSE_SETUP_MICROS)
            ; // ensure adequate setup time has passed between changing motor direction and stepping
              // the motor
    }

    // Check that limit switch is not active
    if ((motor->motor_side == LEFT_SIDE) && (dir == DIR_ANTICLOCK) &&
        (digitalRead(LEFT_SWITCH_PIN) == SWITCH_ACTIVE)) {
        return false;
    } else if ((motor->motor_side == RIGHT_SIDE) && (dir == DIR_CLOCK) &&
               (digitalRead(RIGHT_SWITCH_PIN) == SWITCH_ACTIVE)) {
        return false;
    }

    // Step the motors the equivilent of one full step; motors have max torque in full step position
    // so stopping on this helps prevent losing steps
    for (int pulsecount = 0; pulsecount < MICROS_PER_STEP; pulsecount++) {
        while ((micros() - motor->pulse_timer) < PULSE_STEP_LOW_MICROS)
            ; // ensure adequate duration since previous low pulse
        // Do high pulse
        digitalWrite(motor->step_pin, 1); // high pulse
        motor->pulse_timer = micros();    // reset timer
        // Do low pulse
        while ((micros() - motor->pulse_timer) < PULSE_STEP_HIGH_MICROS)
            ;                             // ensure adequate duration since high pulse
        digitalWrite(motor->step_pin, 0); // low pulse
        motor->pulse_timer = micros();    // reset timer
    }

    // Update the motor position
    if (motor->current_direction == DIR_ANTICLOCK) {
        (motor->position_step_number)++;
    } else {
        (motor->position_step_number)--;
    }

    return true;
}

// Sets up the left and right stepper motors ready for use.
bool setup_motors(struct Stepper* left, struct Stepper* right)
{
    Serial.println("\nMotor setup begin");

    // Set constant values
    right->motor_side = RIGHT_SIDE;
    right->step_pin = RIGHT_STEP_PIN;
    right->dir_pin = RIGHT_DIR_PIN;
    left->motor_side = LEFT_SIDE;
    left->step_pin = LEFT_STEP_PIN;
    left->dir_pin = LEFT_DIR_PIN;

    // Set initial motor directions
    digitalWrite(right->dir_pin, DIR_CLOCK);
    right->current_direction = DIR_CLOCK;
    digitalWrite(left->dir_pin, DIR_ANTICLOCK);
    left->current_direction = DIR_ANTICLOCK;

    // initialise the pulse timers
    right->pulse_timer = micros();
    left->pulse_timer = micros();

#ifndef DEBUG_SKIP_MOTORCAL
    // Calibrate the left motor position
    Serial.println("Calibrating left motor position");
    while (true) {
        // take one step towards the left limit switch with both motors
        step_motor(left, DIR_ANTICLOCK);
        step_motor(right, DIR_ANTICLOCK);
        if (digitalRead(LEFT_SWITCH_PIN) == SWITCH_ACTIVE) {
            // switch active, position now known
            left->position_step_number = LEFT_SWITCH_STEPS;
            Serial.println("Left motor calibration complete\n");
            break;
        }
    }

    // Calibrate the right motor position
    Serial.println("Calibrating right motor position");
    while (true) {
        // take one step towards the right limit switch with both motors
        step_motor(right, DIR_CLOCK);
        step_motor(left, DIR_CLOCK);
        if (digitalRead(RIGHT_SWITCH_PIN) == SWITCH_ACTIVE) {
            // switch active, position now known
            right->position_step_number = RIGHT_SWITCH_STEPS;
            Serial.println("Right motor calibration complete\n");
            break;
        }
    }

    // move the arm away from the limit switch
    for (uint8_t i = 0; i < 20; i++) {
        step_motor(left, DIR_ANTICLOCK);
        step_motor(right, DIR_ANTICLOCK);
    }
#endif

#ifdef DEBUG_SKIP_MOTORCAL
    // initialise motor positions
    left->position_step_number = LEFT_SWITCH_STEPS;
    right->position_step_number = RIGHT_SWITCH_STEPS;
#endif

    return true;
}
