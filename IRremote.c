/*
 * IRremote
 * Version 0.11 August, 2009
 * Copyright 2009 Ken Shirriff
 * For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
 *
 * Modified by Paul Stoffregen <paul@pjrc.com> to support other boards and timers
 * Modified  by Mitra Ardron <mitra@mitra.biz> 
 * Added Sanyo and Mitsubishi controllers
 * Modified Sony to spot the repeat codes that some Sony's send
 * Ported to PIC18F2550 by Marco Koehler, 2013
 *
 * Interrupt code based on NECIRrcv by Joe Knapp
 * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
 * Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
 *
 * JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and other people at the original blog post)
 */

#include "IRremoteInt.h"

volatile irparams_t irparams;


void ir_sendNECRepeatFrame(void)
{
  ir_enableIROut(38);
  // Disable the Timer Interrupt (which is used for receiving IR) to avoid back coupling while sending
  TIMER_DISABLE_INTR;
  ir_mark(NEC_HDR_MARK);
  ir_space(NEC_RPT_SPACE);
  ir_mark(NEC_BIT_MARK);
  ir_space(0);
  // Enable the Timer Interrupt again (which is used for receiving IR)
  TIMER_ENABLE_INTR;
}


void ir_sendNEC(unsigned long data, int nbits)
{
  int i = 0;
  ir_enableIROut(38);
  // Disable the Timer Interrupt (which is used for receiving IR) to avoid back coupling while sending
  TIMER_DISABLE_INTR;

  ir_mark(NEC_HDR_MARK);
  ir_space(NEC_HDR_SPACE);
  for (i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      ir_mark(NEC_BIT_MARK);
      ir_space(NEC_ONE_SPACE);
    } 
    else {
      ir_mark(NEC_BIT_MARK);
      ir_space(NEC_ZERO_SPACE);
    }
    data <<= 1;
  }
  ir_mark(NEC_BIT_MARK);
  ir_space(0);
  // Enable the Timer Interrupt again (which is used for receiving IR)
  TIMER_ENABLE_INTR;
}

void sendSony(unsigned long data, int nbits) {
  int i = 0;
  ir_enableIROut(40);
  // Disable the Timer Interrupt (which is used for receiving IR) to avoid back coupling while sending
  TIMER_DISABLE_INTR;
  ir_mark(SONY_HDR_MARK);
  ir_space(SONY_HDR_SPACE);
  data = data << (32 - nbits);
  for (i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      ir_mark(SONY_ONE_MARK);
      ir_space(SONY_HDR_SPACE);
    }
    else {
      ir_mark(SONY_ZERO_MARK);
      ir_space(SONY_HDR_SPACE);
    }
    data <<= 1;
  }
  // Enable the Timer Interrupt again (which is used for receiving IR)
  TIMER_ENABLE_INTR;
}

void ir_sendSigma(unsigned long data, int nbits) {
  int i = 0;
  if (nbits != SIGMA_BITS)
  {
      return;
  }
  data = data << (32 - nbits);
  ir_enableIROut(38);
  // Disable the Timer Interrupt (which is used for receiving IR) to avoid back coupling while sending
  TIMER_DISABLE_INTR;
  ir_mark(SIGMA_HDR_MARK);
  ir_space(SIGMA_HDR_SPACE);
  for (i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      ir_mark(SIGMA_BIT_MARK);
      ir_space(SIGMA_ONE_SPACE);
    } 
    else {
      ir_mark(SIGMA_BIT_MARK);
      ir_space(SIGMA_ZERO_SPACE);
    }
    data <<= 1;
    //after first byte we had to send an additional BIT_MARK and the NEXT_SPACE
    if (i == 7)
    {
      ir_mark(SIGMA_BIT_MARK);
      ir_space(SIGMA_NEXT_SPACE);
    }
  }
  // the final BIT_MARK
  ir_mark(SIGMA_BIT_MARK);
  ir_space(0);
  // Enable the Timer Interrupt again (which is used for receiving IR)
  TIMER_ENABLE_INTR;
}

void ir_sendRaw(unsigned int buf[], int len, int hz)
{
  int i = 0;
  ir_enableIROut(hz);
  // Disable the Timer Interrupt (which is used for receiving IR) to avoid back coupling while sending
  TIMER_DISABLE_INTR;
  for (i = 0; i < len; i++) {
    if (i & 1) {
      ir_space(buf[i]);
    } 
    else {
      ir_mark(buf[i]);
    }
  }
  ir_space(0); // Just to be sure
  // Enable the Timer Interrupt again (which is used for receiving IR)
  TIMER_ENABLE_INTR;
}

// Note: first bit must be a one (start bit)
void sendRC5(unsigned long data, int nbits)
{
  int i = 0;
  ir_enableIROut(36);
  // Disable the Timer Interrupt (which is used for receiving IR) to avoid back coupling while sending
  TIMER_DISABLE_INTR;
  data = data << (32 - nbits);
  ir_mark(RC5_T1); // First start bit
  ir_space(RC5_T1); // Second start bit
  ir_mark(RC5_T1); // Second start bit
  for (i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      ir_space(RC5_T1); // 1 is space, then mark
      ir_mark(RC5_T1);
    } 
    else {
      ir_mark(RC5_T1);
      ir_space(RC5_T1);
    }
    data <<= 1;
  }
  ir_space(0); // Turn off at end
  // Enable the Timer Interrupt again (which is used for receiving IR)
  TIMER_ENABLE_INTR;
}

// Caller needs to take care of flipping the toggle bit
void sendRC6(unsigned long data, int nbits)
{
  int t = 0;
  int i = 0;
  ir_enableIROut(36);
  // Disable the Timer Interrupt (which is used for receiving IR) to avoid back coupling while sending
  TIMER_DISABLE_INTR;
  data = data << (32 - nbits);
  ir_mark(RC6_HDR_MARK);
  ir_space(RC6_HDR_SPACE);
  ir_mark(RC6_T1); // start bit
  ir_space(RC6_T1);

  for (i = 0; i < nbits; i++) {
    if (i == 3) {
      // double-wide trailer bit
      t = 2 * RC6_T1;
    } 
    else {
      t = RC6_T1;
    }
    if (data & TOPBIT) {
      ir_mark(t);
      ir_space(t);
    } 
    else {
      ir_space(t);
      ir_mark(t);
    }

    data <<= 1;
  }
  ir_space(0); // Turn off at end
  // Enable the Timer Interrupt again (which is used for receiving IR)
  TIMER_ENABLE_INTR;
}
void sendPanasonic(unsigned int address, unsigned long data) {
    int i=0;
    ir_enableIROut(35);
    // Disable the Timer Interrupt (which is used for receiving IR) to avoid back coupling while sending
    TIMER_DISABLE_INTR;
    ir_mark(PANASONIC_HDR_MARK);
    ir_space(PANASONIC_HDR_SPACE);
    
    for(i=0;i<16;i++)
    {
        ir_mark(PANASONIC_BIT_MARK);
        if (address & 0x8000) {
            ir_space(PANASONIC_ONE_SPACE);
        } else {
            ir_space(PANASONIC_ZERO_SPACE);
        }
        address <<= 1;        
    }    
    for (i=0; i < 32; i++) {
        ir_mark(PANASONIC_BIT_MARK);
        if (data & TOPBIT) {
            ir_space(PANASONIC_ONE_SPACE);
        } else {
            ir_space(PANASONIC_ZERO_SPACE);
        }
        data <<= 1;
    }
    ir_mark(PANASONIC_BIT_MARK);
    ir_space(0);
    // Enable the Timer Interrupt again (which is used for receiving IR)
    TIMER_ENABLE_INTR;
}
void sendJVC(unsigned long data, int nbits, int repeat)
{
    int i = 0;
    ir_enableIROut(38);
    // Disable the Timer Interrupt (which is used for receiving IR) to avoid back coupling while sending
    TIMER_DISABLE_INTR;
    data = data << (32 - nbits);
    if (!repeat){
        ir_mark(JVC_HDR_MARK);
        ir_space(JVC_HDR_SPACE);
    }
    for (i = 0; i < nbits; i++) {
        if (data & TOPBIT) {
            ir_mark(JVC_BIT_MARK);
            ir_space(JVC_ONE_SPACE);
        } 
        else {
            ir_mark(JVC_BIT_MARK);
            ir_space(JVC_ZERO_SPACE);
        }
        data <<= 1;
    }
    ir_mark(JVC_BIT_MARK);
    ir_space(0);
    // Enable the Timer Interrupt again (which is used for receiving IR)
    TIMER_ENABLE_INTR;
}

static void ir_mark(int time) {
  // Sends an IR mark for the specified number of microseconds.
  // The mark output is modulated at the PWM frequency.
  TIMER_ENABLE_PWM; // Enable PWM output
  ir_delayMicroseconds(time);
}

/* Leave pin off for time (given in microseconds) */
static void ir_space(int time) {
  // Sends an IR space for the specified number of microseconds.
  // A space is no output, so the PWM output is disabled.
  TIMER_DISABLE_PWM; // Disable PWM output
  ir_delayMicroseconds(time);
}

static void ir_enableIROut(int khz) {
  // Enables IR output.  The khz value controls the modulation frequency in kilohertz.
  // This routine is designed for 36-40KHz; if you use it for other values, it's up to you
  // to make sure it gives reasonable results.  (Watch out for overflow / underflow / rounding.)
  // TIMER2 is used in phase-correct PWM mode.
  // To turn the output on and off, we leave the PWM running, but connect and disconnect the output pin.
  // A few hours staring at the Pic documentation and this will all make sense.

  ir_pinMode(TIMER_PWM_PIN, OUTPUT);
  ir_digitalWrite(TIMER_PWM_PIN, LOW); // When not sending PWM, we want it low
  ir_timerCfgKhz(khz);
}

// initialization
void ir_enableIRIn(void) {
  irparams.recvpin = IR_RECEIVE_PIN;
  irparams.blinkflag = 0;
  // initialize state machine variables
  irparams.rcvstate = STATE_IDLE;
  irparams.rawlen = 0;
  // set pin modes
  ir_pinMode(irparams.recvpin, INPUT);
  
  DISABLE_INTERRUPTS;
  // setup pulse clock timer interrupt for Timer
  ir_timerCfgNorm();
  ir_timerRst();

  //Timer2 Overflow Interrupt Enable
  TIMER_ENABLE_INTR;

  ENABLE_INTERRUPTS;  // enable interrupts

}

// enable/disable blinking of pin 13 on IR processing
void ir_blink13(int blinkflag)
{
  irparams.blinkflag = blinkflag;
  if (blinkflag)
    ir_pinMode(BLINKLED_PIN, OUTPUT);
}

// TIMER interrupt code to collect raw data.
// Widths of alternating SPACE, MARK are recorded in rawbuf.
// Recorded in ticks of 50 microseconds.
// rawlen counts the number of entries recorded so far.
// First entry is the SPACE between transmissions.
// As soon as a SPACE gets long, ready is set, state switches to IDLE, timing of SPACE continues.
// As soon as first MARK arrives, gap width is recorded, ready is cleared, and new logging starts

// call this function inside your InterruptServiceHigh()
void ir_interruptService(void)
{
  unsigned char irdata = 0;

  // timer is used for sampling IR signal
  if (TIMER_INT_FLAG == 1)
  {
    TIMER_INT_FLAG = 0;

    ir_timerRst();

    irdata = (unsigned char)ir_digitalRead(irparams.recvpin);

    irparams.timer++; // One more 50us tick
    if (irparams.rawlen >= RAWBUF) {
        // Buffer overflow
        irparams.rcvstate = STATE_STOP;
    }
    switch(irparams.rcvstate) {
      case STATE_IDLE: // In the middle of a gap
        if (irdata == MARK) {
        if (irparams.timer < GAP_TICKS) {
            // Not big enough to be a gap.
            irparams.timer = 0;
        } 
        else {
            // gap just ended, record duration and start recording transmission
            irparams.rawlen = 0;
            irparams.rawbuf[irparams.rawlen++] = irparams.timer;
            irparams.timer = 0;
            irparams.rcvstate = STATE_MARK;
        }
        }
        break;
      case STATE_MARK: // timing MARK
        if (irdata == SPACE) {   // MARK ended, record time
            irparams.rawbuf[irparams.rawlen++] = irparams.timer;
            irparams.timer = 0;
            irparams.rcvstate = STATE_SPACE;
        }
        break;
      case STATE_SPACE: // timing SPACE
        if (irdata == MARK) { // SPACE just ended, record it
            irparams.rawbuf[irparams.rawlen++] = irparams.timer;
            irparams.timer = 0;
            irparams.rcvstate = STATE_MARK;
        } 
        else { // SPACE
          if (irparams.timer > GAP_TICKS) {
            // big SPACE, indicates gap between codes
            // Mark current code as ready for processing
            // Switch to STOP
            // Don't reset timer; keep counting space width
            irparams.rcvstate = STATE_STOP;
          } 
        }
        break;
     case STATE_STOP: // waiting, measuring gap
        if (irdata == MARK) { // reset gap timer
           irparams.timer = 0;
        }
        break;
    }

    if (irparams.blinkflag) {
        if (irdata == MARK) {
            BLINKLED_ON(); 
        } 
        else {
            BLINKLED_OFF(); 
        }
    }
  }
}

void ir_resume(void) {
  irparams.rcvstate = STATE_IDLE;
  irparams.rawlen = 0;
}



// Decodes the received IR message
// Returns 0 if no data ready, 1 if data ready.
// Results of decoding are stored in results
int ir_decode(decode_results *results) {
  results->rawlen = irparams.rawlen;
  results->rawbuf = (volatile unsigned int *)&irparams.rawbuf[0];
  if (irparams.rcvstate != STATE_STOP) {
    return ERR;
  }
  if (ir_decodeSigma(results)) {
     return DECODED;
  }
  if (ir_decodeNEC(results)) {
    return DECODED;
  }
  if (ir_decodeSony(results)) {
    return DECODED;
  }
  if (ir_decodeSanyo(results)) {
    return DECODED;
  }
  if (ir_decodeMitsubishi(results)) {
    return DECODED;
  }
  if (ir_decodeRC5(results)) {
    return DECODED;
  }
  if (ir_decodeRC6(results)) {
    return DECODED;
  }
  if (ir_decodePanasonic(results)) {
     return DECODED;
  }
  if (ir_decodeJVC(results)) {
     return DECODED;
  }

  // decodeHash returns a hash on any input.
  // Thus, it needs to be last in the list.
  // If you add any decodes, add them before this.
  if (ir_decodeHash(results)) {
    return DECODED;
  }
  // Throw away and start over
  ir_resume();
  return ERR;
}

// NECs have a repeat only 4 items long
static long ir_decodeNEC(decode_results *results) {
  int i = 0;
  long data = 0;
  int offset = 1; // Skip first space
  // Initial mark
  if (!MATCH_MARK(results->rawbuf[offset], NEC_HDR_MARK)) {
    return ERR;
  }
  offset++;
  // Check for repeat
  if (irparams.rawlen == 4 &&
    MATCH_SPACE(results->rawbuf[offset], NEC_RPT_SPACE) &&
    MATCH_MARK(results->rawbuf[offset+1], NEC_BIT_MARK)) {
    results->bits = 0;
    results->value = REPEAT;
    results->decode_type = NEC;
    return DECODED;
  }
  if (irparams.rawlen < 2 * NEC_BITS + 4) {
    return ERR;
  }
  // Initial space  
  if (!MATCH_SPACE(results->rawbuf[offset], NEC_HDR_SPACE)) {
    return ERR;
  }
  offset++;
  for (i = 0; i < NEC_BITS; i++) {
    if (!MATCH_MARK(results->rawbuf[offset], NEC_BIT_MARK)) {
      return ERR;
    }
    offset++;
    if (MATCH_SPACE(results->rawbuf[offset], NEC_ONE_SPACE)) {
      data = (data << 1) | 1;
    } 
    else if (MATCH_SPACE(results->rawbuf[offset], NEC_ZERO_SPACE)) {
      data <<= 1;
    } 
    else {
      return ERR;
    }
    offset++;
  }
  // Success
  results->bits = NEC_BITS;
  results->value = data;
  results->decode_type = NEC;
  return DECODED;
}


// SIGMA ASC 333
static long ir_decodeSigma(decode_results *results) {
  int i = 0;
  long data = 0;
  int offset = 1; // Skip first space
  // Initial mark
  if (!MATCH_MARK(results->rawbuf[offset], SIGMA_HDR_MARK)) {
    return ERR;
  }
  offset++;

  
  if (irparams.rawlen < 2 * SIGMA_BITS + 6) {
    return ERR;
  }
  
  // Initial space
  if (!MATCH_SPACE(results->rawbuf[offset], SIGMA_HDR_SPACE)) {
    return ERR;
  }
  offset++;

  //first byte
  for (i = 0; i < SIGMA_BITS; i++) {
    if (!MATCH_MARK(results->rawbuf[offset], SIGMA_BIT_MARK)) {
      return ERR;
    }
    offset++;
    if (MATCH_SPACE(results->rawbuf[offset], SIGMA_ONE_SPACE)) {
      data = (data << 1) | 1;
    }
    else if (MATCH_SPACE(results->rawbuf[offset], SIGMA_ZERO_SPACE)) {
      data <<= 1;
    }
    else {
      return ERR;
    }
    offset++;

    // between the two bytes is an extra space
    if (i == 7)
    {
        // next space
        if (!MATCH_MARK(results->rawbuf[offset], SIGMA_BIT_MARK)) {
            return ERR;
        }
        offset++;
        if (!MATCH_SPACE(results->rawbuf[offset], SIGMA_NEXT_SPACE)) {
            return ERR;
        }
        offset++;
    }
  }

  //final mark bit
  if (!MATCH_MARK(results->rawbuf[offset], SIGMA_BIT_MARK)) {
    return ERR;
  }

  // Success
  results->bits = 2 * SIGMA_BITS;
  results->value = data;
  results->decode_type = SIGMA;
  return DECODED;
}


static long ir_decodeSony(decode_results *results) {
  long data = 0;
  int offset = 0; // Dont skip first space, check its size
  if (irparams.rawlen < 2 * SONY_BITS + 2) {
    return ERR;
  }
  

  // Some Sony's deliver repeats fast after first
  // unfortunately can't spot difference from of repeat from two fast clicks
  if (results->rawbuf[offset] < SONY_DOUBLE_SPACE_USECS) {
    results->bits = 0;
    results->value = REPEAT;
    results->decode_type = SANYO;
    return DECODED;
  }
  offset++;

  // Initial mark
  if (!MATCH_MARK(results->rawbuf[offset], SONY_HDR_MARK)) {
    return ERR;
  }
  offset++;

  while (offset + 1 < irparams.rawlen) {
    if (!MATCH_SPACE(results->rawbuf[offset], SONY_HDR_SPACE)) {
      break;
    }
    offset++;
    if (MATCH_MARK(results->rawbuf[offset], SONY_ONE_MARK)) {
      data = (data << 1) | 1;
    } 
    else if (MATCH_MARK(results->rawbuf[offset], SONY_ZERO_MARK)) {
      data <<= 1;
    } 
    else {
      return ERR;
    }
    offset++;
  }

  // Success
  results->bits = (offset - 1) / 2;
  if (results->bits < 12) {
    results->bits = 0;
    return ERR;
  }
  results->value = data;
  results->decode_type = SONY;
  return DECODED;
}

// I think this is a Sanyo decoder - serial = SA 8650B
// Looks like Sony except for timings, 48 chars of data and time/space different
static long ir_decodeSanyo(decode_results *results) {
  long data = 0;
  int offset = 0; // Skip first space
  if (irparams.rawlen < 2 * SANYO_BITS + 2) {
    return ERR;
  }
 
  // Initial space  
  if (results->rawbuf[offset] < SANYO_DOUBLE_SPACE_USECS) {
    results->bits = 0;
    results->value = REPEAT;
    results->decode_type = SANYO;
    return DECODED;
  }
  offset++;

  // Initial mark
  if (!MATCH_MARK(results->rawbuf[offset], SANYO_HDR_MARK)) {
    return ERR;
  }
  offset++;

  // Skip Second Mark
  if (!MATCH_MARK(results->rawbuf[offset], SANYO_HDR_MARK)) {
    return ERR;
  }
  offset++;

  while (offset + 1 < irparams.rawlen) {
    if (!MATCH_SPACE(results->rawbuf[offset], SANYO_HDR_SPACE)) {
      break;
    }
    offset++;
    if (MATCH_MARK(results->rawbuf[offset], SANYO_ONE_MARK)) {
      data = (data << 1) | 1;
    } 
    else if (MATCH_MARK(results->rawbuf[offset], SANYO_ZERO_MARK)) {
      data <<= 1;
    } 
    else {
      return ERR;
    }
    offset++;
  }

  // Success
  results->bits = (offset - 1) / 2;
  if (results->bits < 12) {
    results->bits = 0;
    return ERR;
  }
  results->value = data;
  results->decode_type = SANYO;
  return DECODED;
}

// Looks like Sony except for timings, 48 chars of data and time/space different
static long ir_decodeMitsubishi(decode_results *results) {
  long data = 0;
  int offset = 0; // Skip first space
  if (irparams.rawlen < 2 * MITSUBISHI_BITS + 2) {
    return ERR;
  }
  
  // Initial space  
  /* Not seeing double keys from Mitsubishi
  if (results->rawbuf[offset] < MITSUBISHI_DOUBLE_SPACE_USECS) {
    results->bits = 0;
    results->value = REPEAT;
    results->decode_type = MITSUBISHI;
    return DECODED;
  }
  */
  offset++;

  // Typical
  // 14200 7 41 7 42 7 42 7 17 7 17 7 18 7 41 7 18 7 17 7 17 7 18 7 41 8 17 7 17 7 18 7 17 7 

  // Initial Space
  if (!MATCH_MARK(results->rawbuf[offset], MITSUBISHI_HDR_SPACE)) {
    return ERR;
  }
  offset++;
  while (offset + 1 < irparams.rawlen) {
    if (MATCH_MARK(results->rawbuf[offset], MITSUBISHI_ONE_MARK)) {
      data = (data << 1) | 1;
    } 
    else if (MATCH_MARK(results->rawbuf[offset], MITSUBISHI_ZERO_MARK)) {
      data <<= 1;
    } 
    else {
      return ERR;
    }
    offset++;
    if (!MATCH_SPACE(results->rawbuf[offset], MITSUBISHI_HDR_SPACE)) {
      break;
    }
    offset++;
  }

  // Success
  results->bits = (offset - 1) / 2;
  if (results->bits < MITSUBISHI_BITS) {
    results->bits = 0;
    return ERR;
  }
  results->value = data;
  results->decode_type = MITSUBISHI;
  return DECODED;
}


// Gets one undecoded level at a time from the raw buffer.
// The RC5/6 decoding is easier if the data is broken into time intervals.
// E.g. if the buffer has MARK for 2 time intervals and SPACE for 1,
// successive calls to getRClevel will return MARK, MARK, SPACE.
// offset and used are updated to keep track of the current position.
// t1 is the time interval for a single bit in microseconds.
// Returns -1 for error (measured time interval is not a multiple of t1).
static int ir_getRClevel(decode_results *results, int *offset, int *used, int t1) {
  int width = 0;
  int val = 0;
  int correction = 0;
  int avail = 0;
  if (*offset >= results->rawlen) {
    // After end of recorded buffer, assume SPACE.
    return SPACE;
  }
  width = results->rawbuf[*offset];
  val = ((*offset) % 2) ? MARK : SPACE;
  correction = (val == MARK) ? MARK_EXCESS : - MARK_EXCESS;

  if (MATCH(width, t1 + correction)) {
    avail = 1;
  } 
  else if (MATCH(width, 2*t1 + correction)) {
    avail = 2;
  } 
  else if (MATCH(width, 3*t1 + correction)) {
    avail = 3;
  } 
  else {
    return -1;
  }

  (*used)++;
  if (*used >= avail) {
    *used = 0;
    (*offset)++;
  }
  return val;   
}

static long ir_decodeRC5(decode_results *results) {
  int offset = 1; // Skip gap space
  long data = 0;
  int used = 0;
  int nbits = 0;
  if (irparams.rawlen < MIN_RC5_SAMPLES + 2) {
    return ERR;
  }

  // Get start bits
  if (ir_getRClevel(results, &offset, &used, RC5_T1) != MARK) return ERR;
  if (ir_getRClevel(results, &offset, &used, RC5_T1) != SPACE) return ERR;
  if (ir_getRClevel(results, &offset, &used, RC5_T1) != MARK) return ERR;

  for (nbits = 0; offset < irparams.rawlen; nbits++) {
    int levelA = ir_getRClevel(results, &offset, &used, RC5_T1);
    int levelB = ir_getRClevel(results, &offset, &used, RC5_T1);
    if (levelA == SPACE && levelB == MARK) {
      // 1 bit
      data = (data << 1) | 1;
    } 
    else if (levelA == MARK && levelB == SPACE) {
      // zero bit
      data <<= 1;
    } 
    else {
      return ERR;
    } 
  }

  // Success
  results->bits = nbits;
  results->value = data;
  results->decode_type = RC5;
  return DECODED;
}

static long ir_decodeRC6(decode_results *results) {
  int offset = 1; // Skip first space
  long data = 0;
  int used = 0;
  int nbits = 0;
  
  if (results->rawlen < MIN_RC6_SAMPLES) {
    return ERR;
  }
  
  // Initial mark
  if (!MATCH_MARK(results->rawbuf[offset], RC6_HDR_MARK)) {
    return ERR;
  }
  offset++;
  if (!MATCH_SPACE(results->rawbuf[offset], RC6_HDR_SPACE)) {
    return ERR;
  }
  offset++;

  // Get start bit (1)
  if (ir_getRClevel(results, &offset, &used, RC6_T1) != MARK) return ERR;
  if (ir_getRClevel(results, &offset, &used, RC6_T1) != SPACE) return ERR;

  for (nbits = 0; offset < results->rawlen; nbits++) {
    int levelA, levelB; // Next two levels
    levelA = ir_getRClevel(results, &offset, &used, RC6_T1);
    if (nbits == 3) {
      // T bit is double wide; make sure second half matches
      if (levelA != ir_getRClevel(results, &offset, &used, RC6_T1)) return ERR;
    } 
    levelB = ir_getRClevel(results, &offset, &used, RC6_T1);
    if (nbits == 3) {
      // T bit is double wide; make sure second half matches
      if (levelB != ir_getRClevel(results, &offset, &used, RC6_T1)) return ERR;
    } 
    if (levelA == MARK && levelB == SPACE) { // reversed compared to RC5
      // 1 bit
      data = (data << 1) | 1;
    } 
    else if (levelA == SPACE && levelB == MARK) {
      // zero bit
      data <<= 1;
    } 
    else {
      return ERR; // Error
    } 
  }
  // Success
  results->bits = nbits;
  results->value = data;
  results->decode_type = RC6;
  return DECODED;
}

static long ir_decodePanasonic(decode_results *results) {
    unsigned long data = 0;
    int offset = 1;
    int i = 0;
    
    if (!MATCH_MARK(results->rawbuf[offset], PANASONIC_HDR_MARK)) {
        return ERR;
    }
    offset++;
    if (!MATCH_MARK(results->rawbuf[offset], PANASONIC_HDR_SPACE)) {
        return ERR;
    }
    offset++;
    
    // decode address
    for (i = 0; i < PANASONIC_BITS_ADR; i++) {
        if (!MATCH_MARK(results->rawbuf[offset++], PANASONIC_BIT_MARK)) {
            return ERR;
        }
        if (MATCH_SPACE(results->rawbuf[offset],PANASONIC_ONE_SPACE)) {
            data = (data << 1) | 1;
        } else if (MATCH_SPACE(results->rawbuf[offset],PANASONIC_ZERO_SPACE)) {
            data <<= 1;
        } else {
            return ERR;
        }
        offset++;
    }
    results->panasonicAddress = (unsigned int)(data);
    // decode value
    for (i = 0; i < PANASONIC_BITS_VAL; i++) {
        if (!MATCH_MARK(results->rawbuf[offset++], PANASONIC_BIT_MARK)) {
            return ERR;
        }
        if (MATCH_SPACE(results->rawbuf[offset],PANASONIC_ONE_SPACE)) {
            data = (data << 1) | 1;
        } else if (MATCH_SPACE(results->rawbuf[offset],PANASONIC_ZERO_SPACE)) {
            data <<= 1;
        } else {
            return ERR;
        }
        offset++;
    }
    results->value = (unsigned long)data;
    
    results->decode_type = PANASONIC;
    results->bits = PANASONIC_BITS;
    return DECODED;
}

static long ir_decodeJVC(decode_results *results) {
    int i = 0;
    long data = 0;
    int offset = 1; // Skip first space
    // Check for repeat
    if (irparams.rawlen - 1 == 33 &&
        MATCH_MARK(results->rawbuf[offset], JVC_BIT_MARK) &&
        MATCH_MARK(results->rawbuf[irparams.rawlen-1], JVC_BIT_MARK)) {
        results->bits = 0;
        results->value = REPEAT;
        results->decode_type = JVC;
        return DECODED;
    } 
    // Initial mark
    if (!MATCH_MARK(results->rawbuf[offset], JVC_HDR_MARK)) {
        return ERR;
    }
    offset++; 
    if (irparams.rawlen < 2 * JVC_BITS + 1 ) {
        return ERR;
    }
    // Initial space 
    if (!MATCH_SPACE(results->rawbuf[offset], JVC_HDR_SPACE)) {
        return ERR;
    }
    offset++;
    for (i = 0; i < JVC_BITS; i++) {
        if (!MATCH_MARK(results->rawbuf[offset], JVC_BIT_MARK)) {
            return ERR;
        }
        offset++;
        if (MATCH_SPACE(results->rawbuf[offset], JVC_ONE_SPACE)) {
            data = (data << 1) | 1;
        } 
        else if (MATCH_SPACE(results->rawbuf[offset], JVC_ZERO_SPACE)) {
            data <<= 1;
        } 
        else {
            return ERR;
        }
        offset++;
    }
    //Stop bit
    if (!MATCH_MARK(results->rawbuf[offset], JVC_BIT_MARK)){
        return ERR;
    }
    // Success
    results->bits = JVC_BITS;
    results->value = data;
    results->decode_type = JVC;
    return DECODED;
}

/* -----------------------------------------------------------------------
 * hashdecode - decode an arbitrary IR code.
 * Instead of decoding using a standard encoding scheme
 * (e.g. Sony, NEC, RC5), the code is hashed to a 32-bit value.
 *
 * The algorithm: look at the sequence of MARK signals, and see if each one
 * is shorter (0), the same length (1), or longer (2) than the previous.
 * Do the same with the SPACE signals.  Hszh the resulting sequence of 0's,
 * 1's, and 2's to a 32-bit value.  This will give a unique value for each
 * different code (probably), for most code systems.
 *
 * http://arcfn.com/2010/01/using-arbitrary-remotes-with-arduino.html
 */

// Compare two tick values, returning 0 if newval is shorter,
// 1 if newval is equal, and 2 if newval is longer
// Use a tolerance of 20%
static int ir_compare(unsigned int oldval, unsigned int newval) {
  if (newval < oldval * .8) {
    return 0;
  } 
  else if (oldval < newval * .8) {
    return 2;
  } 
  else {
    return 1;
  }
}

// Use FNV hash algorithm: http://isthe.com/chongo/tech/comp/fnv/#FNV-param
#define FNV_PRIME_32 16777619
#define FNV_BASIS_32 2166136261

/* Converts the raw code values into a 32-bit hash code.
 * Hopefully this code is unique for each button.
 * This isn't a "real" decoding, just an arbitrary value.
 */
static long ir_decodeHash(decode_results *results)
{
  long hash = FNV_BASIS_32;
  int i = 0;
  // Require at least 6 samples to prevent triggering on noise
  if (results->rawlen < 6) {
    return ERR;
  }

  for (i = 1; i+2 < results->rawlen; i++) {
    int value =  ir_compare(results->rawbuf[i], results->rawbuf[i+2]);
    // Add value into the hash
    hash = (hash * FNV_PRIME_32) ^ value;
  }
  results->value = hash;
  results->bits = 32;
  results->decode_type = UNKNOWN;
  return DECODED;
}

/* Sharp and DISH support by Todd Treece ( http://unionbridge.org/design/ircommand )

The Dish send function needs to be repeated 4 times, and the Sharp function
has the necessary repeat built in because of the need to invert the signal.

Sharp protocol documentation:
http://www.sbprojects.com/knowledge/ir/sharp.htm

Here are the LIRC files that I found that seem to match the remote codes
from the oscilloscope:

Sharp LCD TV:
http://lirc.sourceforge.net/remotes/sharp/GA538WJSA

DISH NETWORK (echostar 301):
http://lirc.sourceforge.net/remotes/echostar/301_501_3100_5100_58xx_59xx

For the DISH codes, only send the last for characters of the hex.
i.e. use 0x1C10 instead of 0x0000000000001C10 which is listed in the
linked LIRC file.
*/

void ir_sendSharp(unsigned long data, int nbits) {
  unsigned long invertdata = data ^ SHARP_TOGGLE_MASK;
  int i = 0;
  ir_enableIROut(38);
  for (i = 0; i < nbits; i++) {
    if (data & 0x4000) {
      ir_mark(SHARP_BIT_MARK);
      ir_space(SHARP_ONE_SPACE);
    }
    else {
      ir_mark(SHARP_BIT_MARK);
      ir_space(SHARP_ZERO_SPACE);
    }
    data <<= 1;
  }
  
  ir_mark(SHARP_BIT_MARK);
  ir_space(SHARP_ZERO_SPACE);
  ir_delay(46);
  for (i = 0; i < nbits; i++) {
    if (invertdata & 0x4000) {
      ir_mark(SHARP_BIT_MARK);
      ir_space(SHARP_ONE_SPACE);
    }
    else {
      ir_mark(SHARP_BIT_MARK);
      ir_space(SHARP_ZERO_SPACE);
    }
    invertdata <<= 1;
  }
  ir_mark(SHARP_BIT_MARK);
  ir_space(SHARP_ZERO_SPACE);
  ir_delay(46);
}

void ir_sendDISH(unsigned long data, int nbits)
{
  int i = 0;
  ir_enableIROut(56);
  ir_mark(DISH_HDR_MARK);
  ir_space(DISH_HDR_SPACE);
  for (i = 0; i < nbits; i++) {
    if (data & DISH_TOP_BIT) {
      ir_mark(DISH_BIT_MARK);
      ir_space(DISH_ONE_SPACE);
    }
    else {
      ir_mark(DISH_BIT_MARK);
      ir_space(DISH_ZERO_SPACE);
    }
    data <<= 1;
  }
}


static int MATCH(int measured, int desired)
{
    return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired);
}

static int MATCH_MARK(int measured_ticks, int desired_us)
{
    return MATCH(measured_ticks, (desired_us + MARK_EXCESS));
}

static int MATCH_SPACE(int measured_ticks, int desired_us)
{
    return MATCH(measured_ticks, (desired_us - MARK_EXCESS));
}
