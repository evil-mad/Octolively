/*
  Title: octolively.c
  Author: Larry Ogrodnek <ogrodnek@gmail.com>
*/

#define F_CPU 8000000L

#include <avr/io.h> 
#include <avr/pgmspace.h> 
#include <avr/interrupt.h> 
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/eeprom.h> 
#include <stdlib.h>

#define __disable_interrupt()  cli()
#define __enable_interrupt()  sei()
#define __watchdog_reset()    wdt_reset()
#define set_input(portdir,pin) portdir &= ~(1<<pin)

#define DEFAULT_PROGRAM 0

uint8_t EEMEM ProgramConfig = DEFAULT_PROGRAM;
uint8_t EEMEM SensitivityConfig = 0;
uint8_t EEMEM NetworkConfig = 0;

void setup(void);
void loop(void);
//unsigned int readIR(uint8_t channel);
int16_t readIR(uint8_t channel);
int16_t _readIR(uint8_t channel);
int readButton();
void changeProgram();
void programOn(uint8_t prog);
void changeSensitivity();
void sensitivityOn(uint8_t sensitivity);
void netConfigOn(uint8_t net);
void saveConfig(void);
int handle_ir(int ir, int16_t bright);
void flashLeds();
void setFromDir(uint8_t dir, uint8_t val);
void setAll(uint8_t val);
void delay_ms(uint16_t ms);
unsigned long tclock();
uint8_t is_idle();

int dispatch(int program, int ir, int activated, int current_val);
typedef int (*ledHandler)(int, int, int);

// different fade routines
int basic_fade(int ir, int activated, int current_val);
int sine_fade(int ir, int activated, int current_val);
int sparkle_fade(int ir, int activated, int current_val);
int inv_fade(int ir, int activated, int current_val);
int quick_fade(int ir, int activated, int current_val);
int heat_fade(int ir, int activated, int current_val);
int quick_inv(int ir, int activated, int current_val);
int _slow_fade(int ir, int activated, int current_val, uint8_t factor);
int slow_fade(int ir, int activated, int current_val);
int inv_heat(int ir, int activated, int current_val);
int flash_fade(int ir, int activated, int current_val);

int activate(int ir, int bright);
int _activate(int16_t slope, int mode);

// networking
uint8_t in_network();
void broadcast();
void start_transfer(uint8_t val, long clock);
void continue_transfer(long clock);
void do_send(int act, long clock);
void handle_network(long clock);

typedef struct {
  uint8_t val;
  uint8_t dir;
} transmission;

transmission receive(long clock);
void do_receive(unsigned long clock);
int receive_from(uint8_t dir);

typedef int (*netHandler)(int);

int net_repeat(int val);
int net_decrement(int val);

int net_dispatch(int val);

int16_t filteredReading(int ir, int16_t reading);
int16_t slope(int ir, int16_t reading);

#define MAX_PROGRAMS 16
uint8_t program = DEFAULT_PROGRAM;
uint8_t netConfig = 0;

#define DEFAULT_SENSITIVITY 2
#define MAX_SENSITIVITY 4
uint8_t sensitivity = DEFAULT_SENSITIVITY;

ledHandler ledHandlers[] = {
  basic_fade,   // 0
  slow_fade,    // 1
  quick_fade,   // 2
  sine_fade,    // 3
  sparkle_fade, // 4
  heat_fade,    // 5
  inv_fade,     // 6
  flash_fade,   // 7
  // net + fade
  basic_fade,   //  8 -- all boards
  slow_fade,    //  9 -- all boards
  basic_fade,   // 10 -- no decay
  basic_fade,   // 11 -- decay
  basic_fade,   // 12 -- no decay, larger radius
  sparkle_fade, // 13 -- decay
  sine_fade, 
  flash_fade
};

netHandler netHandlers[] = {
  net_repeat,
  net_repeat,
  net_decrement,
  net_decrement,
  net_decrement,
  net_decrement,
  net_decrement,
  net_decrement
};

uint8_t netDistance[] = {
  4,
  4,
  4,
  4,
  8,
  4,
  4,
  4,
};


uint8_t dynamicLevel[] = {
  0,
  0,
  0,
  1,
  0,
  1,
  0,
  0
};

void init_sparkle();
int _random(int min, int max);
int _rstep();

// PWM buffers
unsigned char compare[8];
volatile unsigned char compbuff[8];

int sparkle_dir[8];
int sparkle_step[8];

int main (void)
{ 
  setup();
  loop();
  return 0;
}

uint8_t step = 1;

int last_act[] = {0,0,0,0,0,0,0,0};

#define configTime 40

void loop() {
  unsigned long clock = 0;

  // track fade steps
  unsigned long dimInterval = 75;
  unsigned long lastDim = 0;

  // track button readings
  unsigned long buttonInterval = 100;
  unsigned long lastButton = 0;

  int16_t bright = 0;

  int in_config = 0;
  int last_button = 0;
  int switched = 0;

  unsigned long buttonStart = 0;

  while (1) {
    int ir = 0;
  
    while (ir < 8) {
      if (! in_config) {
	bright = readIR(ir);

	if (bright >= 0) {
	  // got a reading (ADC finished), process signal and increment
	  int act = handle_ir(ir, bright);

	  do_send(act, clock);

	  ir++;
	}

	handle_network(clock);

	if (clock - lastDim > dimInterval) {
	  // step fade routines
	  lastDim = clock;
	  int j;
	  for (j = 0; j< 8; j++) {
	    compbuff[j] = dispatch(program, j, last_act[j], compbuff[j]);
	  }
	}
      }

      clock++;

      if (clock - lastButton > buttonInterval) {
	lastButton = clock;
	int button = readButton();

	// single button press, not in another config mode, enter program config
	if (button && (! last_button)) {
	  buttonStart = tclock();
	  switched = 0;

	  if (in_config == 0) {
	    in_config = 1;
	  }
	}

	// program config stable, switch out of config mode
	if ((in_config) && (! button) && (! last_button)) {

	  unsigned long diff = tclock() - buttonStart;
	  if ((in_config == 1 && (diff > configTime)) ||
	      (in_config && (diff > (configTime << 1)))) {
	  in_config = 0;
	  switched = 0;
	  saveConfig();
	  flashLeds();
	}
	}

	// program config
	if (in_config == 1) {
	  // button was released, advance program
	  if ((button == 0) && (last_button)) {
	    if (tclock() - buttonStart < configTime) {
	      changeProgram();
	      buttonStart = tclock();
	    }
	  }

	  programOn(program);
	}

	// sensitivity config
	if (in_config == 2) {
	  // button was released, change sensitivity
	  if ((button == 0) && (last_button)) {
	    if ((! switched) && tclock() - buttonStart < configTime) {
	      changeSensitivity();
	      buttonStart = tclock();
	    }
	  }

	  sensitivityOn(sensitivity);
	}

	if (in_config == 3) {
	  // button was released, change net config
	  if ((button == 0) && (last_button)) {
	    if ((! switched) && tclock() - buttonStart < configTime) {
	      netConfig++;
	      if (netConfig > 1) {
		netConfig = 0;
	      }
	      netConfigOn(netConfig);
	      buttonStart = tclock();
	    }
	  }	  
	}

	// button depressed for a bit, switch to sensitivity adjustment
	if (button && last_button) {

	  if ((in_config == 2) && tclock() - buttonStart > (configTime * 2)) {
	    in_config = 3;
	    netConfigOn(netConfig);
	    buttonStart = tclock();
	    switched++;
	  }

	  if ((! switched) && tclock() - buttonStart > configTime) {
	    switched++;

	    if (in_config >= 2) {
	      in_config = 0;
	    } else {
	      in_config = 2;
	      buttonStart = tclock();
	    }

	    if (! in_config) {
	      // switched out of config mode, save settings...
	      saveConfig();
	    }
	    // visual feedback for mode switch ...
	    flashLeds();
	  }
	}

	last_button = button;
      }
    }
  }
}

void saveConfig() {
  eeprom_write_byte(&ProgramConfig, program);
  eeprom_write_byte(&SensitivityConfig, sensitivity);
  eeprom_write_byte(&NetworkConfig, netConfig);
}

void flashLeds() {
  int i;
  for (i=0; i< 8; i++) {
    compbuff[i] = 255;
  }
  delay_ms(50);
  for (i=0; i< 8; i++) {
    compbuff[i] = 0;
  }
}

void delay_ms(uint16_t ms) {
  while (ms) {
    _delay_ms(1);
    ms--;
  }
}

int handle_ir(int ir, int16_t bright) {
  int act = activate(ir, bright);
  last_act[ir] = act;
  compbuff[ir] = dispatch(program, ir, act, compbuff[ir]);

  return act;
}

int dispatch(int program, int ir, int activated, int current_val) {
  last_act[ir] = activated;

  if (program < MAX_PROGRAMS) {
    return ledHandlers[program](ir, activated, current_val);
  }

  return 0;
}

#define nsamples 14
int16_t samples[8][nsamples];

enum actstate {neg=1, pos};
int activated[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned int actcount[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int activate(int ir, int16_t reading) {
  int16_t s = filteredReading(ir, reading);

  // tracking steady state
  if (actcount[ir] > 50) {
    activated[ir] = 0;
  }

  int a = _activate(s, activated[ir]);

  if (a) {
    activated[ir] = (s < 0 ? neg : pos);
    actcount[ir] = 0;
  } else {
    actcount[ir]++;
  }
  
  return a;
}

uint8_t cutoffs[] = {100, 50, 35, 20};

int _activate(int16_t slope, int mode) {
  if (mode == pos && slope < 0) {
    return 0;
  }

  if (mode == neg && slope > 0) {
    return 0;
  }

  if (slope < 0) {
    slope *= -1;
  }

  if (slope < cutoffs[sensitivity]) {
    return 0;
  }

  if (slope > 255) {
    return 255;
  }

  return slope;
}

// #define nasamples 4
int16_t asamples[8][8];
unsigned int acounter = 0;
int16_t filteredReading(int ir, int16_t reading) {
  uint8_t nasamples = (sensitivity <= 1 ? 8 : 4);
  int index = acounter++ % nasamples;

  asamples[ir][index] = slope(ir, reading);
  int i;
  int32_t sum = 0;
  for (i = 0; i< nasamples; i++) {
    sum += asamples[ir][i];
  }

  int16_t ret = (sum >> (nasamples == 8 ? 3 : 2));

  return ret;
}

int16_t slope(int ir, int16_t reading) {
  int i;
  // shift 
  for (i=0; i< nsamples - 1; i++) {
    samples[ir][i] = samples[ir][i+1];
  }
  samples[ir][nsamples - 1] = reading;

  int16_t diffs = 0;
  for (i=nsamples-1; i > 0; i--) {
    diffs += (samples[ir][i] - samples[ir][i-1]);
  }

  return diffs;
}

void s_init() {
  int i, j;

  for (i=0; i< 8; i++) {
    for (j=0; j< nsamples; j++) {
      samples[i][j] = 0;
    }
    for (j=0; j< 8; j++) {
      asamples[i][j] = 0;
    }
  }
}


// precomputed dampened sine for 1 -> 50
//  double c2 = cos(3*t);
//  return ((1/t) * (c2*c2) * 255) * 2;
#define sine_steps 50
const uint8_t damp_sine[] PROGMEM = {
  255, 234, 140, 90, 58, 36, 20, 10, 4, 0, 
  0, 0, 2, 4, 8, 12, 16, 18, 20, 22, 
  22, 22, 20, 18, 16, 14, 10, 8, 4, 2,
  0, 0, 0, 0, 0, 0, 2, 4, 6, 8, 
  8, 10, 10, 10, 10, 10, 8, 4, 2, 1, 1
};

int sine_fade(int ir, int activated, int current_val) {
  static int step[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  static int mstep[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  if (activated) {
    step[ir] = 1;
    mstep[ir] = 0;
    return 255;
  }

  if (step[ir] > 0 && step[ir] < sine_steps) {
    if (step[ir] > 10 && mstep[ir] < 2) {
      mstep[ir]++;
      return current_val;
    }

    mstep[ir] = 0;
    int target = pgm_read_byte(damp_sine + step[ir] + 1);

    if (current_val < target) {
      return current_val + 1;
    } else if (current_val > target) {
      return current_val - 1;
    } else {
      step[ir] += 1;
      return target;
    }
  }

  return 0;
}

int flash_fade(int ir, int activated, int current_val) {
  return _slow_fade(ir, activated, current_val, 10);
}

int sparkle_fade(int ir, int activated, int current_val) {
  static uint8_t offstate[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  if (activated) {
    if (! in_network()) {
      offstate[ir] = 200;
    } else {
      offstate[ir] = current_val;
    }
  }

  if (offstate[ir] > 0) {
    offstate[ir] -= 1;
  }

  if (offstate[ir] > 0 || current_val != 0) {
    int val = current_val + (sparkle_dir[ir] * sparkle_step[ir]);

    if (val > 255) {
      sparkle_dir[ir] = -1 * sparkle_dir[ir];
      sparkle_step[ir] = _rstep();
      return 255;
    }

    if (val < 0) {
      sparkle_dir[ir] = -1 * sparkle_dir[ir];
      sparkle_step[ir] = _rstep();
      return 0;
    }
    
    return val;
  }

  return 0;
}

int _random(int min, int max) {
  return (rand() % (max - min + 1) + min);
}

int _rstep() {
  return _random(3, 10);
}

int basic_fade(int ir, int activated, int current_val) {
  return _slow_fade(ir, activated, current_val, 1);
}

int slow_fade(int ir, int activated, int current_val) {
  return _slow_fade(ir, activated, current_val, 3);
}

int _slow_fade(int ir, int activated, int current_val, uint8_t factor) {
  static uint8_t msteps[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  if (activated) {
    msteps[ir] = 0;

    if (! in_network()) {
      return 255;
    }

    return current_val;
  }

  msteps[ir] += 1;

  if (msteps[ir] >= factor) {
    msteps[ir] = 0;

    if (current_val > step) {
      return current_val - step;
    } else {
      return 0;
    }
  }

  return current_val;
}

int quick_fade(int ir, int activated, int current_val) {
  if (activated) {
    // return 255;
    return activated;
  }

  if (current_val > 2) {
    return current_val - 3;
  }

  return 0;
}

int quick_inv(int ir, int activated, int current_val) {
  if (activated) {
    return 0;
  }

  if (current_val < 253) {
    return current_val + 3;
  }

  return 255;
}


#define hstep 16

int inv_heat(int ir, int activated, int current_val) {
  int ret;

  if (activated) {
    ret = current_val - hstep;
  } else {
    ret = current_val + step;
  }

  if (ret > 255) {
    return 255;
  }
  if (ret < 0) {
    return 0;
  }

  return ret;
}

int heat_fade(int ir, int activated, int current_val) {
  static unsigned long msteps[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int ret;

  if (activated) {
    if (tclock() - msteps[ir] > 0) {
      ret = current_val + hstep;
    } else {
      ret = current_val;
      if (ret == 0) {
	ret = hstep;
      }
    }
    // call slow_fade to init counters
    _slow_fade(ir, activated, current_val, 2);
    msteps[ir] = tclock();
  } else {
    int fade = current_val >> 4;
    if (fade > 6) {
      fade = 6;
    }
    
    return _slow_fade(ir, activated, current_val, fade);
  }

  if (ret > 255) {
    return 255;
  }

  return ret;
}

int inv_fade(int ir, int activated, int current_val) {
  if (activated) {
    return 0;
  }

  int ret = current_val + step;

  if (ret > 255) {
    return 255;
  }

  return ret;
}


#define	ComInputMask 62U
#define	ComOutputMask 1U

void setup() {
  CLKPR = (1 << CLKPCE);        // enable clock prescaler update
  CLKPR = 0;                    // set clock to maximum (= crystal)

  __watchdog_reset();           // reset watchdog timer
  MCUSR &= ~(1 << WDRF);        // clear the watchdog reset flag
  WDTCSR |= (1<<WDCE)|(1<<WDE); // start timed sequence
  WDTCSR = 0x00;                // disable watchdog timer

  // init PWM values
  int pwm = 0;
  int i;
  for (i=0; i< 8; i++) {
    compare[i] = pwm;
    compbuff[i] = pwm;
  }  

  DDRB  = ComOutputMask; // All B Inputs except for pin PB0, our output.
  PORTB = ComOutputMask | ComInputMask;		// Pull-ups on input lines, output line high to start.

  DDRD  = 255; // All D outputs to LEDs
  PORTD = 0;

  DDRC  = 255; // All C Outputs to IR LEDs
  PORTC = 0;

  DDRA  = 0; // All A Inputs
  PORTA = 0;  

  // Enable ADC, prescale at 128.
  ADCSRA = _BV(ADEN) |_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

  TCCR0A = 2;
  OCR0A = 128;

  TIFR0 = (1 << TOV0);           // clear interrupt flag
  TIMSK0 = (1 << OCIE0A);         // enable overflow interrupt
  TCCR0B = (1 << CS00);         // start timer, no prescale

  //TCCR0A |= _BV(WGM01);

  

  TIFR1 = (1 << TOV1);
  TIMSK1 = (1 << TOIE1);
  //TCCR1B |= (_BV(CS10) | _BV(CS11));
  TCCR1A = 0;
  TCCR1B = _BV(CS11);

  __enable_interrupt();         // enable interrupts

  program = eeprom_read_byte(&ProgramConfig);
  if (program >= MAX_PROGRAMS) {
    program = DEFAULT_PROGRAM;
  }

  sensitivity = eeprom_read_byte(&SensitivityConfig);
  if (sensitivity >= MAX_SENSITIVITY) {
    sensitivity = DEFAULT_SENSITIVITY;
  }

  netConfig = eeprom_read_byte(&NetworkConfig);
  if (netConfig > 1) {
    netConfig = 0;
  }

  //
  init_sparkle();
  // 
  s_init();
}


void init_sparkle() {
  int i;
  for (i=0; i < 8; i++) {
    sparkle_dir[i] = _random(0, 1) ? -1 : 1;
    sparkle_step[i] = _rstep();
  }
}

#define num_acd_readings 14

int16_t readIR(uint8_t channel) {
  static int started = 0;
  static int i = 0;
  static uint16_t sum = 0;

  if (! started) {
    PORTC = 1 << channel; // turn on IR LED

    sum = 0;
    i = 0;
    started++;
  }

  while (i < num_acd_readings) {
    int16_t v = _readIR(channel);

    if (v < 0) {
      return -1;
    }

    sum += v;
    i++;
  }

  PORTC = 0; // turn off IR LED
  started = 0; // reset

  int16_t ret = (sum >> 1);
  return ret;
}

int16_t _readIR(uint8_t channel) {
  static int started = 0;

  if (! started) {
    ADMUX = channel;
    ADCSRA |= _BV(ADSC); // Start initial ADC cycle

    started++;
  }

  if ((ADCSRA & _BV(ADSC)) != 0) {
    // conversion not finished
    return -1;
  }

  int16_t ADIn;
  ADIn = ADCW;

  // reset state
  started = 0;

  return ADIn;
}

int readButton() {
  return (! bit_is_set(PINB, PINB5));
}

void changeProgram() {
  program++;

  if (netConfig == 0 && program >= 8) {
    program = 0;
  } else if (netConfig == 1 && program >= MAX_PROGRAMS) {
    program = 0;
  }
}


void netConfigOn(uint8_t net) {
  if (net == 0) {
    compbuff[0] = 0;
    compbuff[1] = 255;
    compbuff[2] = 255;
    compbuff[3] = 0;
    compbuff[4] = 0;
    compbuff[5] = 255;
    compbuff[6] = 255;
    compbuff[7] = 0;
  } else {
    int i;
    for (i=0; i< 8; i++) {
      compbuff[i] = 0;
    }
    compbuff[0] = 255;
    delay_ms(20);
    compbuff[1] = 255;
    delay_ms(20);
    compbuff[2] = 255;
    delay_ms(20);
    compbuff[3] = 255;
    delay_ms(20);
    compbuff[7] = 255;
    delay_ms(20);
    compbuff[6] = 255;
    delay_ms(20);
    compbuff[5] = 255;
    delay_ms(20);
    compbuff[4] = 255;
    delay_ms(20);
  }
}


void programOn(uint8_t prog) {
  static uint8_t state = 0;
  static uint8_t counter = 0;

  int i;
  for (i=0; i< 8; i++) {
    compbuff[i] = 0;
  }

  if (prog < 8) {
    compbuff[prog] = 255;
    return;
  }

  if (counter++ > 50) {
    state = 1 - state;
    counter = 0;
  }

  uint8_t _prog = prog - 8;

  if (state) {
    compbuff[_prog] = 255;
  } else {
    compbuff[_prog] = 0;
  }
}


void changeSensitivity() {
  sensitivity++;
  if (sensitivity >= MAX_SENSITIVITY) {
    sensitivity = 0;
  }
}

void sensitivityOn(uint8_t sensitivity) {
  int j;
  for (j=0; j< 4; j++) {
    if (j <= sensitivity) {
      compbuff[j] = 255;
      compbuff[j + 4] = 255;
    } else {
      compbuff[j] = 0;
      compbuff[j + 4] = 0;
    }
  }
}

// networking

enum _transfer_state { t_idle, t_send, t_recv };
uint8_t transfer_state = t_idle;


uint16_t transfer_buffer = 0;
uint16_t transfer_duration = 0;
long transfer_last = 0;

void start_transfer(uint8_t val, long clock) {
  if (transfer_state == t_idle) {
    transfer_buffer = 10 * val;
    transfer_last = clock;
    transfer_state = t_send;
  }
}

void continue_transfer(long clock) {
  if (transfer_state == t_send) {
    transfer_last = clock;

    if (transfer_buffer > 0) {
      broadcast(1);
      transfer_buffer--;
    } else {
      transfer_state = t_idle;
      broadcast(0);
    }
  }
}


uint8_t receive_buffer = 0;
uint8_t receive_dir = 0;
long receive_last = 0;


// transmission receive_val = { -1, -1 };

transmission receive(long clock) {
  transmission ret = {0, 0};

  if (transfer_state == t_send) {
    return ret;
  }

  if (clock - transfer_last < 100) {
    return ret;
  }

  // in receive
  if (transfer_state == t_recv) {
    int val = receive_from(receive_dir);

    // transfer ended
    if (val == 0) {
      ret.val = receive_buffer / 9;
      ret.dir = receive_dir;

      receive_dir = 0;
      receive_buffer = 0;
      transfer_state = t_idle;
      return ret;
    }

    // ongoing
    receive_buffer++;
    return ret;
  }

  // idle, check for receive...
  receive_dir = receive_from(0);
  if (receive_dir > 0) {
    transfer_state = t_recv;
    receive_buffer++;
  }

  return ret;
}

uint8_t in_network() {
  return program > 7;
}

void do_send(int act, long clock) {
  if (! in_network()) {
    return;
  }

  uint8_t val = netDistance[program - 8];

  if (act > 0) {
    start_transfer(val, clock);
    setAll(255);
  }
}

void handle_network(long clock) {
  if (! in_network()) {
    return;
  }

  continue_transfer(clock);
  do_receive(clock);  
}

int receive_from(uint8_t dir) {
  if ((dir == 0 || dir == 1) && bit_is_clear(PINB, PB1)) {
    return 1;
  }

  if ((dir == 0 || dir == 2) && bit_is_clear(PINB, PB2)) {
    return 2;
  }

  if ((dir == 0 || dir == 3) && bit_is_clear(PINB, PB3)) {
    return 3;
  }

  if ((dir == 0 || dir == 4) && bit_is_clear(PINB, PB4)) {
    return 4;
  }

  return 0;
}

void do_receive(unsigned long clock) {
  transmission trans = receive(clock);
  int val = trans.val;
  if (val <= 0) {
    return;
  }

  if (! is_idle()) {
    return;
  }

  if (val > 8) {
    val = 8;
  }

  int next = net_dispatch(val);
  
  if (next > 0) {
    start_transfer(next, clock);
  }

  int level = 255;
  if (dynamicLevel[program - 8]) {
    level = 31 * val;
  }

  if (next < 0) {
    setFromDir(trans.dir, level);
  } else {
    setAll(level);
  }
}

uint8_t is_idle() {
  int i;
  for (i=0; i< 8; i++) {
    if (compbuff[i] > 0) {
      return 0;
    }
  }
  return 1;
}

int net_dispatch(int val) {
  int _program = program - 8;

  if (_program >= 0 && _program < 8) {
    return netHandlers[_program](val);
  }
  
  return 0;
}

int net_repeat(int val) {
  return val;
}

int net_decrement(int val) {
  return val - 2;
}

void broadcast(int val) {
  if (val) {
    PORTB &= ~(1);
  } else {
    PORTB |= 1;
  }
}


void setFromDir(uint8_t dir, uint8_t val) {
  if ((dir == 4) || (dir == 1)) {
    dispatch(program, 0, 1, val);
    dispatch(program, 1, 1, val);
    dispatch(program, 4, 1, val);
    dispatch(program, 5, 1, val);
  } else if ((dir == 2) || (dir == 3)) {
    dispatch(program, 2, 1, val);
    dispatch(program, 3, 1, val);
    dispatch(program, 6, 1, val);
    dispatch(program, 7, 1, val);
  }
}

void setAll(uint8_t val) {
  int i;
  for (i=0; i< 8; i++) {
    compbuff[i] = dispatch(program, i, 1, val);
  }
}

// interrupts

ISR (TIMER0_COMPA_vect) {
  static uint8_t pwmCnt = 0;

  pwmCnt++;

  uint8_t pd = 0;

  if (compbuff[0]  > pwmCnt) {
    pd = 1;
  }

  if (compbuff[1]  > pwmCnt) {
    pd |= 2;
  }

  if (compbuff[2]  > pwmCnt) {
    pd |= 4;
  }

  if (compbuff[3]  > pwmCnt) {
    pd |= 8;
  }

  if (compbuff[4]  > pwmCnt) {
    pd |= 16;
  }

  if (compbuff[5]  > pwmCnt) {
    pd |= 32;
  }

  if (compbuff[6]  > pwmCnt) {
    pd |= 64;
  }

  if (compbuff[7]  > pwmCnt) {
    pd |= 128;
  }

  PORTD = pd;
}

unsigned long timer_clock = 0;
ISR (TIMER1_OVF_vect) {
  timer_clock++;
}

unsigned long tclock() {
  unsigned long m;
  uint8_t oldSREG = SREG;
  
  cli();
  m = timer_clock;
  SREG = oldSREG;

  return m;
}
