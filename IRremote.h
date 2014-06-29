/*
 * IRremote
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.htm http://arcfn.com
 * Edited by Mitra to add new controller SANYO
 * Ported to PIC18F2550 by Marco Koehler, 2013
 *
 * Interrupt code based on NECIRrcv by Joe Knapp
 * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
 * Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
 *
 * JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and other people at the original blog post)
 */

#ifndef IRremote_h
#define IRremote_h

#define RAWBUF 100 // Length of raw duration buffer

// Results returned from the decoder
typedef struct {
  int decode_type; // NEC, SONY, RC5, UNKNOWN
  unsigned int panasonicAddress; // This is only used for decoding Panasonic data
  unsigned long value; // Decoded value
  int bits; // Number of bits in decoded value
  volatile unsigned int *rawbuf; // Raw intervals in .5 us ticks
  int rawlen; // Number of records in rawbuf.
} decode_results;

// Values for decode_type
#define NEC 1
#define SONY 2
#define RC5 3
#define RC6 4
#define DISH 5
#define SHARP 6
#define PANASONIC 7
#define JVC 8
#define SANYO 9
#define MITSUBISHI 10
#define SIGMA 11
#define UNKNOWN -1

//Bit length of the protocolls
#define NEC_BITS 32
#define SIGMA_BITS 16
#define SONY_BITS 12
#define SANYO_BITS 12
#define MITSUBISHI_BITS 16
#define MIN_RC5_SAMPLES 11
#define MIN_RC6_SAMPLES 1
#define PANASONIC_BITS 48
#define PANASONIC_BITS_ADR 16
#define PANASONIC_BITS_VAL 32
#define JVC_BITS 16

// Decoded value for NEC when a repeat code is received
#define REPEAT 0xffffffff

// return value of ir_decode()
#define ERR 0
#define DECODED 1

// call this function inside your InterruptServiceHigh()
extern void ir_interruptService(void);

// API calls
extern void ir_blink13(int blinkflag);
extern int ir_decode(decode_results *results);
extern void ir_enableIRIn(void);
extern void ir_resume(void);
extern void ir_sendNECRepeatFrame(void);
extern void ir_sendNEC(unsigned long data, int nbits);
extern void ir_sendSigma(unsigned long data, int nbits);
extern void ir_sendSony(unsigned long data, int nbits);
extern void ir_sendRaw(unsigned int buf[], int len, int hz);
extern void ir_sendRC5(unsigned long data, int nbits);
extern void ir_sendRC6(unsigned long data, int nbits);
extern void ir_sendDISH(unsigned long data, int nbits);
extern void ir_sendSharp(unsigned long data, int nbits);
extern void ir_sendPanasonic(unsigned int address, unsigned long data);
extern void ir_sendJVC(unsigned long data, int nbits, int repeat); // *Note instead of sending the REPEAT constant if you want the JVC repeat signal sent, send the original code value and change the repeat argument from 0 to 1. JVC protocol repeats by skipping the header NOT by sending a separate code value like NEC does.
extern void ir_delay(unsigned long time);

#endif
