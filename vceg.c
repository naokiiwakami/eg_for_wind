#include <avr/io.h>
#include <avr/interrupt.h>

/************************************************************************/
/* Definitions and global variables                                     */
/************************************************************************/

// utility methods
uint32_t GetRatio(uint16_t key);

inline int32_t GetDiff(uint16_t key, int32_t maximum) {
  uint32_t denominator = key + 1;
  denominator *= 80;
  uint32_t fraction = 838860800; // (10 << 21) * 40;
  fraction /= denominator;
  return fraction;
}

/**
 * Calculates pseudo exponential MIDI level to volume conversion.
 * The input value range is 0 to 127 (inclusive). The output value range is 0 to 1023
 * X = x + offset
 * X / (X + (K * (127 + offset - X)))
 */
inline uint32_t GetLevelExp(uint8_t midi_level) {
  const uint8_t offset = 5;
  const uint8_t kvalue = 4;
  register uint32_t fraction = midi_level;
  fraction += offset;
  register uint32_t denominator = 127 + offset;
  denominator -= fraction;
  denominator *= kvalue;
  denominator += fraction;
  fraction *= 1023;
  fraction /= denominator;
  fraction <<= 21;
  return fraction;
}

/**
 * Convert a MIDI level to Q10.21 internal level linearly.
 * This is meant to be used for VCF control. In order to weaken effect of the velocity level,
 * the MIDI level affects only up to 50% of the output level (i.e. output would be 512 to 1023).
 *
 * TODO: The velocity depth affects the taste of articulation, it should be configurable in future.
 */
inline uint32_t GetLevelLin(uint8_t midi_level, uint8_t rshift) {
  uint32_t level = midi_level;
  level <<= 24 - rshift;
  return level;
}

inline uint32_t GrowOrDecayExponentially(uint32_t current_value, uint32_t target_value,
                                         uint32_t ratio) {
  register int64_t temp;
  if (current_value >= target_value) {
    temp = current_value;
    temp -= target_value;
    temp *= ratio;
    temp >>= 32;
    temp += target_value;
  } else {
    temp = target_value;
    temp -= current_value;
    temp *= ratio;
    temp >>= 32;
    temp *= -1;
    temp += target_value;
  }
  return temp;
}

inline uint32_t GrowOrDecayLinearly(uint32_t current_value, uint32_t target_value,
                                    uint32_t ratio) {
  register int32_t temp = current_value;
  if (current_value > target_value) {
    if (temp >= ratio) {
      temp -= ratio;
    } else {
      temp = 0;
    }
    if (temp < target_value) {
      temp = target_value;
    }
  } else {
    temp += ratio;
    if (temp > target_value) {
      temp = target_value;
    }
  }
  return temp;
}

/**
 * Main routine and ISR synchronization bits.
 *
 * ISRs and the main routine would synchronize using the flags
 * exchanged via variable g_sync_flags.  Bit assignments in the
 * g_sync_flags are defined here.
 */
volatile uint8_t g_sync_flags; // 8 bit flags used for main - ISR synchronization

#define SYNC_UPDATE_VALUE      _BV(0)
#define SYNC_UPDATE_VALUE_DONE (~SYNC_UPDATE_VALUE)

#define SYNC_ADC_INVOKE        _BV(1)
#define SYNC_ADC_STARTED       _BV(2)
#define SYNC_ADC_DONE          (~(SYNC_ADC_INVOKE | SYNC_ADC_STARTED))

#define SYNC_NOTE_ON           _BV(3)
#define SYNC_NOTE_OFF          _BV(4)

// MIDI //////////////////////////////////////////
#define F_CPU    20000000
#define UART_BAUD_RATE 31250
#define ESC            0x1b
#define UART_BUF_SIZE    16

#define MIDI_SYSEX_ENTER 0xF0
#define MIDI_SYSEX_EXIT  0xF7

#define MIDI_NOTE_OFF 0x80
#define MIDI_NOTE_ON  0x90
#define MIDI_CONTROL_CHANGE 0xb0
#define MIDI_PROGRAM_CHANGE 0xc0
#define MIDI_CHANNEL_PRESSURE 0xd0

#define MIDI_BREATH 0x02

volatile uint8_t uart_rxd_queue_length;
uint8_t *uart_rxd_in_ptr, *uart_rxd_out_ptr;
uint8_t uart_rxd_buffer[UART_BUF_SIZE];

volatile uint16_t rxByte;

volatile struct midi_message {
  uint8_t status;
  uint8_t channel;
  uint8_t data[2];
  uint8_t data_length_mask;
  uint8_t data_pointer;
}
g_midi_message;

volatile uint8_t g_notes_count;

volatile uint8_t g_velocity;
volatile uint8_t g_breath;
volatile uint32_t g_breath_target_1;
volatile uint32_t g_breath_target_2;
volatile uint32_t g_breath_value_1;
volatile uint32_t g_breath_value_2;
volatile uint32_t g_breath_ratio_1;
volatile uint32_t g_breath_ratio_2;

// Controller states /////////////////////////////

volatile int8_t g_switch;

volatile int8_t g_control_mode;
#define CONTROL_MODE_READ_1  0x1
#define CONTROL_MODE_READ_2  0x2
#define CONTROL_MODE_PEND_1  0x4
#define CONTROL_MODE_PEND_2  0x8

volatile uint16_t g_transition_counter;

#define SET_ADSR_PARAMS(input, param1, param2) {    \
    if (g_control_mode & CONTROL_MODE_READ_1) {     \
      param1 = input;                               \
    }                                               \
    if (g_control_mode & CONTROL_MODE_READ_2) {     \
      param2 = input;                               \
    }                                               \
  }

// Envelope generator states /////////////////////

// envelope generator states
#define STATE_RELEASE_1 0x01
#define STATE_ATTACK_1  0x02
#define STATE_DECAY_1   0x04
#define STATE_RELEASE_2 0x10
#define STATE_ATTACK_2  0x20
#define STATE_DECAY_2   0x40

#define OUTPUT_RANGE 1024  // 10 bit
#define CURVE_MAX_VALUE 65535  // maximum output value of the function GetCurve()
#define GET_CURVE_INPUT_RANGE 65536  // Input range of the function GetCurve()

volatile uint8_t g_eg_state; // current envelope generator state, 4 LSBs are for EG1, 4 MSBs: EG2

volatile uint32_t g_value_1;  // Current output-1 value. Q10.21 fixed point number
volatile uint32_t g_target_1;
volatile uint32_t g_ratio_1;  // Current output-1 transition ratio. Q0.32 fixed point, 0 <= x < 1.0
volatile uint32_t g_attack_threshold_1;

volatile uint32_t g_value_2;  // Current output-2 value. Q10.21 fixed point number
volatile uint32_t g_target_2;
volatile uint32_t g_ratio_2;  // Current output-2 transition ratio. Q0.32 fixed point, 0 <= x < 1.0
volatile uint32_t g_attack_threshold_2;

// ADSR parameters ///////////////////////////////
volatile uint16_t g_attack_1;
volatile uint16_t g_decay_1;
volatile uint16_t g_sustain_1;
volatile uint16_t g_release_1;

volatile uint16_t g_attack_2;
volatile uint16_t g_decay_2;
volatile uint16_t g_sustain_2;
volatile uint16_t g_release_2;

// ADC parameters ////////////////////////////////

// ADC channel mapping. these are 4-bit integers that are passed to ADMUX register
#define ADC_CH_ATTACK  0x00
#define ADC_CH_DECAY   0x01
#define ADC_CH_SUSTAIN 0x02
#define ADC_CH_RELEASE 0x03

#define ADC_NUM_CHANNELS 4

volatile uint8_t g_adc_next_channel;

// ADC is triggered in every ADC_INTERVAL Timer1 OVR interrupt handler calls.
// The time interval would be 10.24 ms as calculated as following:
//   (ADC_INTERVAL=100) * (TOP=1024) * 2 /  (clock=20E6) = 10.24 * 10E-3
// where multiplying by 2 is because the timer runs in phase-correct PWM mode.
#define ADC_INTERVAL 100
volatile uint8_t g_adc_timing_counter;

#define ADC_VALUE_RANGE 1024  // 10 bit
#define ADC_MAX_VALUE (ADC_VALUE_RANGE - 1)

// LED indicators /////////////////////////////////
#define LED1_ON(x) (PORTD |= _BV(PD7))
#define LED1_OFF(x) (PORTD &= ~_BV(PD7))
#define LED1_TOGGLE(x) (PORTD ^= _BV(PD7))
#define LED1_SET(x)                             \
  if (x) {                                      \
    LED1_ON();                                  \
  } else {                                      \
    LED1_OFF();                                 \
  }

#define LED2_ON(x) (PORTB |= _BV(PB0))
#define LED2_OFF(x) (PORTB &= ~_BV(PB0))
#define LED2_TOGGLE(x) (PORTB ^= _BV(PB0))
#define LED2_SET(x)                             \
  if (x) {                                      \
    LED2_ON();                                  \
  } else {                                      \
    LED2_OFF();                                 \
  }

/**
 * The TIMER1 (PWM) interrupt handler.
 *
 * The handler sets necessary flags in g_sync_flags. SYNC_UPDATE_VALUE bit is
 * always invoked. SYNC_ADC_INVOKE bit is invoked in every ADC_INTERVAL interruption.
 */
ISR(TIMER1_OVF_vect) {
  // Reflect the output values to PWM Output Control Registers
  uint32_t temp = g_value_1;
  temp += g_breath_value_1;
  temp >>= 21;
  if (temp > 1023) {
    temp = 1023;
  }
  OCR1A = temp;
  temp = g_value_2;
  temp += g_breath_value_2;
  temp >>= 21;
  if (temp > 1023) {
    temp = 1023;
  }
  OCR1B = temp;

  // Notify the main routine that we are ready to update the values
  g_sync_flags |= SYNC_UPDATE_VALUE;

  if (--g_adc_timing_counter == 0) {
    // Tell the main routine to pick the value from an ADC
    g_sync_flags |= SYNC_ADC_INVOKE;
    g_adc_timing_counter = ADC_INTERVAL;
  }
}

/**
 * Receive Complete Interrupt (RCI) handler
 */
ISR(USART_RX_vect) {
  *uart_rxd_in_ptr = UDR0;	 /* read byte from receive register */
  uart_rxd_queue_length++;
  if (++uart_rxd_in_ptr >= uart_rxd_buffer + UART_BUF_SIZE) {  /* Pointer wrapping */
    uart_rxd_in_ptr = uart_rxd_buffer;
  }
}

/**
 * uart_getchar()
 * reads UART read buffer and returns a single character.
 */
int16_t UartGetChar(void) {
  if (uart_rxd_queue_length > 0) {
    // TODO: Calling cli() here is a mutex lock essentially. Implement a lockless queue.
    cli();
    uart_rxd_queue_length--;
    uint8_t c = *uart_rxd_out_ptr;	 /* get a character from the buffer */
    if (++uart_rxd_out_ptr >= uart_rxd_buffer + UART_BUF_SIZE) {  /* pointer wrapping */
      uart_rxd_out_ptr = uart_rxd_buffer;
    }
    sei();
    return c;
  } else {
    return -1;							/* buffer is empty */
  }
}

/************************************************************************/
/* Setup routines                                                       */
/************************************************************************/
void SetUpIo()
{
  // I/O
  DDRB =
      _BV(DDB0)   // LED2
    | _BV(DDB1)   // Envelope out (VCF)
    | _BV(DDB2);  // Envelope out (VCA)

  DDRD =
      _BV(DDD7);  // LED1

  PORTD =
      _BV(PD2); // To pull up the control switch
}

void SetUpTimer()
{
  // Timer1, Phase and Frequency correct PWM, no prescale.
  TCCR1A =
      _BV(COM1A1)   // Clear OC1A on compare match
    | _BV(COM1B1);  // Clear OC1B on compare match
  TCCR1B =
      _BV(WGM13)  // Phase and frequency correct PWM
    | _BV(CS10);  // Timer enabled, no prescale

  // Set zero to PWM outputs
  OCR1A = 0;
  OCR1B = 0;
  TIMSK1 = _BV(TOIE1);  // Timer/Counter1, Overflow Interrupt Enabled
  ICR1 = 0x03FF;  // Set PWM resolution 10bit
}

void SetUpAdc()
{
  /** Setup and enable ADC **/
  ADMUX =
    (0 << REFS1)        // Reference Selection Bits
    | (1 << REFS0)      // AVcc - external cap at AREF
    // | (0 << ADLAR)   // ADC Left Adjust Result
    // | (0 << MUX2)    // Analog Channel Selection Bits
    // | (1 << MUX1)    // ADC2 (PC2 PIN25)
    // | (0 << MUX0)
    ;

  ADCSRA = (1 << ADEN) // ADC ENable
    // (0 << ADSC)|    // ADC Start Conversion
    // (0 << ADATE)|   // ADC Auto Trigger Enable
    // (0 << ADIF)|    // ADC Interrupt Flag
    // (0 << ADIE)|    // ADC Interrupt Enable
    | (1 << ADPS2)     // ADC prescaler Select Bits.  '111' is 128 that gives approx. 156kHz for 20MHz clock
    | (0 << ADPS1)
    | (1 << ADPS0);

  // ADC channel selector.
  // ADC is invoked every 10ms approximately in round robin manner over ADC channels.
  g_adc_next_channel = 0;
}

void SetUpUart(void)
{
  UBRR0 = F_CPU / (UART_BAUD_RATE * 16l) - 1;
  UCSR0B = (1 << RXCIE0) | (1 << RXEN0);

  uart_rxd_in_ptr  = uart_rxd_out_ptr = uart_rxd_buffer;
  uart_rxd_queue_length = 0;
}

void SetUpMisc()
{
  g_sync_flags = 0;

  g_eg_state = STATE_RELEASE_1 | STATE_RELEASE_2;

  g_value_1 = 0;
  g_target_1 = 0;
  g_ratio_1 = 0;
  g_attack_threshold_1 = 0;

  g_value_2 = 0;
  g_target_2 = 0;
  g_ratio_2 = 0;
  g_attack_threshold_2 = 0;

  g_attack_1 = 0;
  g_decay_1 = 0;
  g_sustain_1 = 0;
  g_release_1 = 0;

  g_attack_2 = 0;
  g_decay_2 = 0;
  g_sustain_2 = 0;
  g_release_2 = 0;

  g_adc_timing_counter = ADC_INTERVAL;

  g_notes_count = 0;
  g_velocity = 0;
  g_breath = 0;
  g_breath_target_1 = 0;
  g_breath_target_2 = 0;
  g_breath_value_1 = 0;
  g_breath_value_2 = 0;
  g_breath_ratio_1 = GetDiff(2, 1024);
  g_breath_ratio_2 = GetRatio(1);

  g_switch = 0;
  g_control_mode = CONTROL_MODE_READ_1 | CONTROL_MODE_READ_2;
  g_transition_counter = 0;
}

void Setup()
{
  cli();
  SetUpIo();
  SetUpTimer();
  SetUpAdc();
  SetUpUart();
  SetUpMisc();
  sei();
}

void ToggleControlMode()
{
  if (g_transition_counter == 0) {
    // set the initial pending mode
    uint8_t next = g_control_mode & 0x3;
    next = next == 0x3 ? 1 : next + 1;
    next <<= 2;
    g_control_mode |= next;
  } else {
    // increment the pending mode
    uint8_t next = (g_control_mode >> 2) & 0x3;
    next = next == 0x3 ? 1 : next + 1;
    next <<= 2;
    g_control_mode &= 0x3;
    g_control_mode |= next;
    g_transition_counter = 60000;
  }
}

volatile uint8_t sw_prev = 1;
void CheckInput()
{
  if (g_switch > 0) {
    --g_switch;
  }

  uint8_t sw_current = (PIND >> 2) & 0x01;
  if (g_switch == 0) {
    if (sw_prev ^ sw_current) {
      g_switch = 100;
      if (sw_current == 0) {
        ToggleControlMode();
        g_transition_counter = 50000;
      }
    }
  }
  sw_prev = sw_current;
}

void UpdateLeds() {
  if (g_transition_counter > 0) {
    if (--g_transition_counter == 0) {
      // commit the pending control modes
      g_control_mode >>= 2;
    } else {
      register uint8_t temp = g_transition_counter >> 9;
      if (temp & 0x08) {
        LED1_SET(g_control_mode & CONTROL_MODE_READ_1);
        LED2_SET(g_control_mode & CONTROL_MODE_READ_2);
      } else if (temp & 0x01) {
        LED1_SET(g_control_mode & CONTROL_MODE_PEND_1);
        LED2_SET(g_control_mode & CONTROL_MODE_PEND_2);
      } else {
        LED1_OFF();
        LED2_OFF();
      }
    }
  }
  if (g_transition_counter == 0) {
    LED1_SET(g_control_mode & CONTROL_MODE_READ_1);
    LED2_SET(g_control_mode & CONTROL_MODE_READ_2);
  }
}

#define UPDATE_VALUE_EXP(attack, decay, sustain, release, mask,         \
                         STATE_RELEASE, STATE_ATTACK, STATE_DECAY,      \
                         target, attack_threshold, ratio, value) {      \
    if (g_eg_state & STATE_ATTACK) {                                    \
      if (value > attack_threshold) {                                   \
        g_eg_state &= mask;                                             \
        g_eg_state |= STATE_DECAY;                                      \
      } else {                                                          \
        ratio = GetRatio(attack);                                       \
      }                                                                 \
    }                                                                   \
    if (g_eg_state & STATE_DECAY) {                                     \
      target = 0;                                                       \
      ratio = GetRatio(decay);                                          \
    }                                                                   \
    else if (g_eg_state & STATE_RELEASE) {                              \
      ratio = GetRatio(release);                                        \
    }                                                                   \
    value = GrowOrDecayExponentially(value, target, ratio);             \
  }

#define UPDATE_VALUE_LIN(attack, decay, sustain, release, mask,     \
                         STATE_RELEASE, STATE_ATTACK, STATE_DECAY,  \
                         target, attack_threshold, ratio, value) {  \
    if (g_eg_state & STATE_ATTACK) {                                \
      if (value > attack_threshold) {                               \
        g_eg_state &= mask;                                         \
        g_eg_state |= STATE_DECAY;                                  \
      } else {                                                      \
        ratio = GetDiff(attack, target);                            \
      }                                                             \
    }                                                               \
    if (g_eg_state & STATE_DECAY) {                                 \
      target = 0;                                                   \
      ratio = GetDiff(decay, attack_threshold - target);            \
    }                                                               \
    else if (g_eg_state & STATE_RELEASE) {                          \
      ratio = GetDiff(release, target);                             \
    }                                                               \
    value = GrowOrDecayLinearly(value, target, ratio);              \
  }

void UpdateValue()
{
  // setups for new states
  if ((g_sync_flags & SYNC_NOTE_ON) != 0) {

    uint64_t temp = GetLevelLin(g_velocity, 2);
    temp += GetLevelLin(96, 0);
    temp *= g_sustain_1;
    temp /= 1023;
    g_target_1 = temp;
    temp *= 0.9;
    g_attack_threshold_1 = temp;

    temp = GetLevelExp(g_velocity);
    temp *= g_sustain_2;
    temp /= 1023;
    g_target_2 = temp;
    temp *= 0.9;
    g_attack_threshold_2 = temp;

    g_ratio_1 = GetDiff(g_attack_1, g_target_1);
    // g_ratio_1 = GetRatio(g_attack_1);
    g_ratio_2 = GetRatio(g_attack_2);

    if (++g_notes_count == 1) {
      g_eg_state = STATE_ATTACK_1 | STATE_ATTACK_2;
    }
    g_sync_flags &= ~SYNC_NOTE_ON;
  }
  else if ((g_sync_flags & SYNC_NOTE_OFF)) {
    g_sync_flags &= ~SYNC_NOTE_OFF;
    if (--g_notes_count == 0) {
      g_eg_state = STATE_RELEASE_1 | STATE_RELEASE_2;
      g_ratio_1 = GetRatio(g_release_1);
      g_ratio_2 = GetRatio(g_release_2);
      g_target_1 = 0;
      g_target_2 = 0;
    }
  }

  // actions
  //
  UPDATE_VALUE_LIN(g_attack_1, g_decay_1, g_sustain_1, g_release_1,
                   0xf0, STATE_RELEASE_1, STATE_ATTACK_1, STATE_DECAY_1,
                   g_target_1, g_attack_threshold_1, g_ratio_1, g_value_1);
  UPDATE_VALUE_EXP(g_attack_2, g_decay_2, g_sustain_2, g_release_2,
                   0x0f, STATE_RELEASE_2, STATE_ATTACK_2, STATE_DECAY_2,
                   g_target_2, g_attack_threshold_2, g_ratio_2, g_value_2);

  g_breath_value_1 = GrowOrDecayLinearly(g_breath_value_1, g_breath_target_1, g_breath_ratio_1);
  g_breath_value_2 = GrowOrDecayExponentially(g_breath_value_2, g_breath_target_2, g_breath_ratio_2);
}

void InvokeAdc()
{
  ADMUX &= 0xF0; // clear ADC channels
  ADMUX |= g_adc_next_channel; // set the channel

  ADCSRA |= _BV(ADSC);
}

int16_t ReadAdc()
{
  if ( ADCSRA & _BV(ADSC) ) {
    // data is not ready yet
    return -1;
  }

  return ADC;
}

void ProcessMidiMessage() {
  switch (g_midi_message.status) {
    case MIDI_NOTE_ON:
      g_velocity = g_midi_message.data[1];
      if (g_velocity == 0) {
        g_sync_flags |= SYNC_NOTE_OFF;
      } else {
        g_sync_flags |= SYNC_NOTE_ON;
      }
      break;
    case MIDI_NOTE_OFF:
      g_sync_flags |= SYNC_NOTE_OFF;
      break;
    case MIDI_CONTROL_CHANGE:
      switch (g_midi_message.data[0]) {
        case MIDI_BREATH:
          g_breath = g_midi_message.data[1];
          g_breath_target_1 = GetLevelLin(g_breath, 2);
          g_breath_target_2 = GetLevelExp(g_breath);
          break;
      }
      break;
  }
}

void HandleMidiInput(uint8_t next_byte) {
  if (next_byte >= 0xf0) {
    // any system-common and system-realtime messages are ignored
    return;
  }
  if (next_byte > 0x7f) { // is status byte
    g_midi_message.status = next_byte & 0xf0;
    g_midi_message.channel = next_byte & 0x0f;
    switch (g_midi_message.status) {
      case MIDI_PROGRAM_CHANGE:
      case MIDI_CHANNEL_PRESSURE:
        g_midi_message.data_length_mask = 0;
      break;
      default:
        g_midi_message.data_length_mask = 1;
    }
    g_midi_message.data_pointer = 0;
  }
  else { // data byte
    g_midi_message.data[g_midi_message.data_pointer & 0x1] = next_byte;
    if ((++g_midi_message.data_pointer & g_midi_message.data_length_mask) == 0) {
      ProcessMidiMessage();
    }
  }
}

int main(void)
{
  Setup();

  while (1) {
    if ( ! g_sync_flags ) {
      continue;
    }

    if ( (g_sync_flags & SYNC_UPDATE_VALUE) ) {
      CheckInput();
      UpdateValue();
      UpdateLeds();
      g_sync_flags &= SYNC_UPDATE_VALUE_DONE;
    }

    if ( (g_sync_flags & SYNC_ADC_INVOKE) ) {
      if ( (g_sync_flags & SYNC_ADC_STARTED) ) {
        // try reading the value
        register int16_t temp = ReadAdc();
        if (temp >= 0) {
          switch (g_adc_next_channel) {
          case ADC_CH_ATTACK:
            SET_ADSR_PARAMS(temp, g_attack_1, g_attack_2);
            break;
          case ADC_CH_DECAY:
            SET_ADSR_PARAMS(temp, g_decay_1, g_decay_2);
            break;
          case ADC_CH_SUSTAIN:
            SET_ADSR_PARAMS(temp, g_sustain_1, g_sustain_2);
            break;
          case ADC_CH_RELEASE:
            SET_ADSR_PARAMS(temp, g_release_1, g_release_2);
            break;
          }
          if (++g_adc_next_channel == ADC_NUM_CHANNELS) {
            g_adc_next_channel = 0;
          }
          g_sync_flags &= SYNC_ADC_DONE;
        }
      }
      else {
        InvokeAdc();
        g_sync_flags |= SYNC_ADC_STARTED;
      }
    }
    int16_t rx_byte = UartGetChar();
    if (rx_byte >= 0) {
      HandleMidiInput(rx_byte);
    }
  }
}

/*
 * Exponential decay/grow ratio lookup table.
 *
 * The table stores data points to give decay/grow ratio for lookup keys ranging
 * from 0 to 1023 (inclusive). The key 0 receives the ratio for time constant
 * 0.001s (1ms) approximately. The key 1023 receives the ratio for time constant
 * about 5s. The values in th table are Q0.32 unsigned fixed point numbers.
 *
 * In order to save data memory space, the values have different key resolutions
 * depending on the position in the table.
 * Table values with indexes from 0 to 63 has 1-1 key mapping to index,
 * i.e.
 *   ratio = kTransientRatios[key]
 * But for the indexes from 64 to 127, the table keeps only values for every 15 keys,
 * i.e.
 *   ratio = kTransientRatios[(key - 64) / 15 + 64]
 * where (key - 64) % 15 == 0
 *
 * Ratios for keys that are not on these indexes need to be calculated
 * by interpolation or extrapolation. See the method GetRatio().
 */
uint32_t kTransientRatios[128] = {
  3886247118,  4222575580,  4255256816,  4267608186,
  4274098987,  4278100538,  4280814394,  4282775974,
  4284259997,  4285421933,  4286356375,  4287124175,
  4287766262,  4288311174,  4288779419,  4289186113,
  4289542645,  4289857756,  4290138268,  4290389583,
  4290616034,  4290821137,  4291007774,  4291178333,
  4291334804,  4291478865,  4291611934,  4291735225,
  4291849776,  4291956486,  4292056132,  4292149393,
  4292236865,  4292319070,  4292396469,  4292469473,
  4292538445,  4292603710,  4292665560,  4292724255,
  4292780031,  4292833101,  4292883656,  4292931872,
  4292977906,  4293021905,  4293063999,  4293104310,
  4293142949,  4293180018,  4293215611,  4293249813,
  4293282706,  4293314362,  4293344850,  4293374234,
  4293402573,  4293429921,  4293456330,  4293481846,
  4293506515,  4293530379,  4293553475,  4293575840,
  4293597508,  4293856889,  4294033677,  4294161903,
  4294259161,  4294335461,  4294396917,  4294447478,
  4294489805,  4294525758,  4294556676,  4294583547,
  4294607117,  4294627960,  4294646522,  4294663159,
  4294678155,  4294691742,  4294704109,  4294715414,
  4294725787,  4294735340,  4294744166,  4294752345,
  4294759946,  4294767027,  4294773641,  4294779831,
  4294785639,  4294791097,  4294796237,  4294801085,
  4294805667,  4294810002,  4294814111,  4294818011,
  4294821717,  4294825243,  4294828603,  4294831807,
  4294834867,  4294837792,  4294840590,  4294843270,
  4294845839,  4294848303,  4294850670,  4294852944,
  4294855131,  4294857236,  4294859264,  4294861218,
  4294863103,  4294864922,  4294866678,  4294868376,
  4294870017,  4294871604,  4294873141,  4294874628,
  4294876070,  4294877467,  4294878823,  4294880138,
};

uint32_t GetRatio(uint16_t key) {
  const int kTableBoundary = 64;
  if (key < kTableBoundary) {
    return kTransientRatios[key];
  }
  const int kResolution = 15;
  int index = (key - kTableBoundary) / kResolution + kTableBoundary;
  int ramainder = (key - kTableBoundary) % kResolution;
  register uint32_t temp = kTransientRatios[index];
  if (ramainder > 0) {
    register uint32_t temp2;
    if (index == 127) {
      temp2 = temp - kTransientRatios[index - 1];
    } else {
      temp2 = kTransientRatios[index + 1] - temp;
    }
    temp2 *= ramainder;
    temp2 /= kResolution;
    temp += temp2;
  }
  return temp;
}
