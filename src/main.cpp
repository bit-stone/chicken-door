#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// GENERAL CONFIG
#define TIMEOUT_COUNT 150
#define STATE_CHANGE_COUNT 25

#define ERROR_LED_COUNT 10
#define MOTOR_SWITCH_BOUNCE_TIME 800

// PIN CONFIG
#define LED_PIN PB3
#define LED_PORT PORTB
#define LED_DDR DDRB

#define MOTOR_ON_PIN PD6
#define MOTOR_ON_PORT PORTD
#define MOTOR_ON_DDR DDRD

#define MOTOR_UP_PIN PA0
#define MOTOR_UP_PORT PORTA
#define MOTOR_UP_DDR DDRA

#define MOTOR_DOWN_PIN PA1
#define MOTOR_DOWN_PORT PORTA
#define MOTOR_DOWN_DDR DDRA

#define END_STOP_UP_PIN PD2
#define END_STOP_UP_INPUT PIND
#define END_STOP_UP_PORT PORTD
#define END_STOP_UP_DDR DDRD

#define END_STOP_DOWN_PIN PD3
#define END_STOP_DOWN_INPUT PIND
#define END_STOP_DOWN_PORT PORTD
#define END_STOP_DOWN_DDR DDRD

#define SENSOR_INPUT_PIN PB0
#define SENSOR_INPUT_INPUT PINB
#define SENSOR_INPUT_PORT PORTB
#define SENSOR_INPUT_DDR DDRB

#define SWITCH_INPUT_PIN PB1
#define SWITCH_INPUT_INPUT PINB
#define SWITCH_INPUT_PORT PORTB
#define SWITCH_INPUT_DDR DDRB

// INTERNALS
#define STATE_MOVING_DOWN 101
#define STATE_MOVING_UP 102
#define STATE_IDLE_DOWN 103
#define STATE_IDLE_UP 104
#define STATE_IDLE_MIDDLE 105
#define STATE_ERROR 106

#define MOTOR_MOVE_UP 10
#define MOTOR_MOVE_DOWN 11
#define MOTOR_MOVE_NONE 12

#define LED_ON 22
#define LED_OFF 23
#define LED_BLINK 24

// timer presets for clock select
#define TIMER_VERY_FAST_CS 30 // only used to show error state!
#define TIMER_FAST_CS 31 // indicates motor movement
#define TIMER_SLOW_CS 32
#define TIMER_OFF 33

// global variables
uint8_t state = STATE_IDLE_MIDDLE;
uint8_t motorMovement = MOTOR_MOVE_NONE;
uint8_t timerSpeed = 0;
uint8_t stateChangeCounter = 0;
uint8_t timeoutCounter = 0;
uint8_t ledCounter = 0;

// set timer clock select
void setTimerSpeed(uint8_t timerSpeedId) {
  if(timerSpeed != timerSpeedId) {
    switch(timerSpeedId) {
      case TIMER_VERY_FAST_CS: // f = 8 000 000 / 65 535 = 122.07 Hz -> dont forget led counter -> 12.207 Hz
        TCCR1B |= (1 << CS10);
        TCCR1B &= ~(1 << CS12 | 1 << CS11);
      break;
      case TIMER_FAST_CS: // f = 8 000 000 / (64 * 65535) = 1.907 Hz -> about 0.524 seconds
        TCCR1B |= (1 << CS10 | 1 << CS11);
        TCCR1B &= ~(1 << CS12);
      break;
      case TIMER_SLOW_CS: // f = 8 000 000 / (1024 * 65535) = 0.1192 Hz -> about 8.389 seconds
        TCCR1B |= (1 << CS10 | 1 << CS12);
        TCCR1B &= ~(1 << CS11);
      break;
      case TIMER_OFF:
        TCCR1B &= ~(1 << CS02 | 1 << CS01 | 1 << CS00);
      break;
      default: break;
    }
    timerSpeed = timerSpeedId;
  }
}

// set motor move direction
void setMotorMovement(uint8_t motorMovementId) {
  if(motorMovement != motorMovementId) {
    // reset motor first
    MOTOR_DOWN_PORT &= ~(1 << MOTOR_DOWN_PIN);
    MOTOR_UP_PORT &= ~(1 << MOTOR_UP_PIN);
    MOTOR_ON_PORT &= ~(1 << MOTOR_ON_PIN);
    _delay_ms(10);
    // set new state
    switch(motorMovementId) {
      case MOTOR_MOVE_DOWN:
        MOTOR_ON_PORT |= (1 << MOTOR_ON_PIN);
        MOTOR_DOWN_PORT |= (1 << MOTOR_DOWN_PIN);
        motorMovement = MOTOR_MOVE_DOWN;
      break;
      case MOTOR_MOVE_UP:
        MOTOR_ON_PORT |= (1 << MOTOR_ON_PIN);
        MOTOR_UP_PORT |= (1 << MOTOR_UP_PIN);
        motorMovement = MOTOR_MOVE_UP;
      break;
      default:
        motorMovement = MOTOR_MOVE_NONE;
      break;
    }
  }
}

void setState(uint8_t stateId) {
  if(state != stateId) {
    switch(stateId) {
      case STATE_IDLE_UP:
        setTimerSpeed(TIMER_SLOW_CS);
        setMotorMovement(MOTOR_MOVE_NONE);
        state = STATE_IDLE_UP;
        LED_PORT &= ~(1 << LED_PIN);
      break;
      case STATE_IDLE_DOWN:
        setTimerSpeed(TIMER_SLOW_CS);
        setMotorMovement(MOTOR_MOVE_NONE);
        state = STATE_IDLE_DOWN;
        LED_PORT &= ~(1 << LED_PIN);
      break;
      case STATE_MOVING_UP:
        setTimerSpeed(TIMER_FAST_CS);
        setMotorMovement(MOTOR_MOVE_UP);
        state = STATE_MOVING_UP;
        timeoutCounter = 0;
      break;
      case STATE_MOVING_DOWN:
        setTimerSpeed(TIMER_FAST_CS);
        setMotorMovement(MOTOR_MOVE_DOWN);
        state = STATE_MOVING_DOWN;
        timeoutCounter = 0;
      break;
      case STATE_ERROR:
        setTimerSpeed(TIMER_VERY_FAST_CS);
        setMotorMovement(MOTOR_MOVE_NONE);
        state = STATE_ERROR;
        LED_PORT &= ~(1 << LED_PIN);
      break;
      default: break;
    }
  }
}

// ISR
// Interrupt 0: end stop up was pressed
ISR(INT0_vect) {
  if(state == STATE_MOVING_UP) {
    setState(STATE_MOVING_DOWN);
    setTimerSpeed(TIMER_OFF);
    _delay_ms(MOTOR_SWITCH_BOUNCE_TIME);
    setState(STATE_IDLE_UP);
  }
}

// Interrupt 1: end stop down was pressed
ISR(INT1_vect) {
  if(state == STATE_MOVING_DOWN) {
    setState(STATE_MOVING_UP);
    setTimerSpeed(TIMER_OFF);
    _delay_ms(MOTOR_SWITCH_BOUNCE_TIME);
    setState(STATE_IDLE_DOWN);
  }
}

ISR(TIMER1_OVF_vect) {
  switch(state) {
    case STATE_IDLE_UP:
      if(bit_is_set(SENSOR_INPUT_INPUT, SENSOR_INPUT_PIN)) {
        setTimerSpeed(TIMER_FAST_CS);
        LED_PORT |= (1 << LED_PIN);
        stateChangeCounter++;
        if(stateChangeCounter > STATE_CHANGE_COUNT) {
          stateChangeCounter = 0;
          setState(STATE_MOVING_DOWN);
        }
      } else {
        setTimerSpeed(TIMER_SLOW_CS);
        LED_PORT |= (1 << LED_PIN);
        _delay_ms(100);
        LED_PORT &= ~(1 << LED_PIN);
        _delay_ms(200);
        LED_PORT |= (1 << LED_PIN);
        _delay_ms(100);
        LED_PORT &= ~(1 << LED_PIN);
        stateChangeCounter = 0;
      }
    break;
    case STATE_IDLE_DOWN:
      if(bit_is_clear(SENSOR_INPUT_INPUT, SENSOR_INPUT_PIN)) {
        setTimerSpeed(TIMER_FAST_CS);
        LED_PORT |= (1 << LED_PIN);
        stateChangeCounter++;
        if(stateChangeCounter > STATE_CHANGE_COUNT) {
          stateChangeCounter = 0;
          setState(STATE_MOVING_UP);
        }
      } else {
        setTimerSpeed(TIMER_SLOW_CS);
        LED_PORT |= (1 << LED_PIN);
        _delay_ms(100);
        LED_PORT &= ~(1 << LED_PIN);
        stateChangeCounter = 0;
      }
    break;
    case STATE_MOVING_UP:
    case STATE_MOVING_DOWN:
      LED_PORT ^= (1 << LED_PIN);
      timeoutCounter++;
      if(timeoutCounter >= TIMEOUT_COUNT) {
        setState(STATE_ERROR);
      }
    break;
    case STATE_ERROR:
      ledCounter++;
      if(ledCounter > ERROR_LED_COUNT) {
        LED_PORT ^= (1 << LED_PIN);
        ledCounter = 0;
      }
    break;
  }
}

// MAIN
int main(void) {
  // init led
  LED_DDR |= 1 << LED_PIN;
  LED_PORT |= 1 << LED_PIN;

  // set inputs for end stops and activate pullups
  END_STOP_UP_DDR &= ~(1 << END_STOP_UP_PIN);
  END_STOP_UP_PORT |= (1 << END_STOP_UP_PIN);

  END_STOP_DOWN_DDR &= ~(1 << END_STOP_DOWN_PIN);
  END_STOP_DOWN_PORT |= (1 << END_STOP_DOWN_PIN);

  // sensor input
  SENSOR_INPUT_DDR &= ~(1 << SENSOR_INPUT_PIN);
  SENSOR_INPUT_PORT |= (1 << SENSOR_INPUT_PIN);

  // switch input
  SWITCH_INPUT_DDR &= ~(1 << SWITCH_INPUT_PIN);
  SWITCH_INPUT_PORT |= (1 << SWITCH_INPUT_PIN);

  // enable timer
  TIMSK |= (1 << TOIE1);
  setTimerSpeed(TIMER_VERY_FAST_CS);

  // enable interrupts on INT1 and INT0
  // reacting to falling edge
  GIMSK |= (1 << INT1 | 1 << INT0);
  MCUCR |= (1 << ISC11 | 1 << ISC01);

  // switch motors off
  MOTOR_ON_DDR |= 1 << MOTOR_ON_PIN;
  MOTOR_ON_PORT &= ~(1<<MOTOR_ON_PIN);

  // set both motor outputs to off
  MOTOR_DOWN_DDR |= 1 << MOTOR_DOWN_PIN;
  MOTOR_UP_DDR |= 1 << MOTOR_UP_PIN;

  MOTOR_DOWN_PORT &= ~(1 << MOTOR_DOWN_PIN);
  MOTOR_UP_PORT &= ~(1 << MOTOR_UP_PIN);

   _delay_ms(1000);

  LED_PORT &= ~(1 << LED_PIN);

  // check if one of the end switches is pushed
  // to prevent unneccesary movement
  if(bit_is_clear(END_STOP_DOWN_INPUT, END_STOP_DOWN_PIN)) {
    setState(STATE_IDLE_DOWN);
  } else if(bit_is_clear(END_STOP_UP_INPUT, END_STOP_UP_PIN)) {
    setState(STATE_IDLE_UP);
  } else {
    // none of the end stops is pressed. just move up
    setState(STATE_MOVING_UP);
  }

  // enable interrupts
  sei();

  while(1) {
    if(bit_is_clear(SWITCH_INPUT_INPUT, SWITCH_INPUT_PIN)) {
      switch(state) {
        case STATE_MOVING_UP:
          setState(STATE_IDLE_UP);
          _delay_ms(500);
        break;
        case STATE_IDLE_UP:
          setState(STATE_MOVING_DOWN);
          _delay_ms(500);
        break;
        case STATE_MOVING_DOWN:
          setState(STATE_IDLE_DOWN);
          _delay_ms(500);
        break;
        case STATE_IDLE_DOWN:
          setState(STATE_MOVING_UP);
          _delay_ms(500);
        break;
        default: break;
      }
    }
  }
}
