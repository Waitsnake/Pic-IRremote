/* 
 * File:   main.c
 * Author: Marco Koehler
 *
 * Created on 8. Oktober 2013, 01:38
 */

#include <stdio.h>
#include <stdlib.h>

#include "p18f2550.h"

#define IRTOY
#include "configwords.h"	// JTR only included in main.c
#include "IRremote.h"

decode_results dec_results;

#define TERRATEC_HOME_KEY       0x28D7827D
#define TERRATEC_POWEROFF_KEY   0x28D7807F
#define TERRATEC_1_KEY          0x28D740BF
#define TERRATEC_2_KEY          0x28D7C03F
#define TERRATEC_3_KEY          0x28D720DF
#define TERRATEC_4_KEY          0x28D7A05F
#define TERRATEC_5_KEY          0x28D7609F
#define TERRATEC_6_KEY          0x28D7E01F
#define TERRATEC_7_KEY          0x28D710EF
#define TERRATEC_8_KEY          0x28D7906F
#define TERRATEC_9_KEY          0x28D750AF
#define TERRATEC_0_KEY          0x28D730CF
#define TERRATEC_OK_KEY         0x28D748B7
#define TERRATEC_UP_KEY         0x28D708F7
#define TERRATEC_DOWN_KEY       0x28D728D7
#define TERRATEC_LEFT_KEY       0x28D78877
#define TERRATEC_RIGHT_KEY      0x28D7C837
#define TERRATEC_INFO_KEY       0x28D76897

#define APPLE_PLAY_KEY          0x77E12044
#define APPLE_UP_KEY            0x77E1D044
#define APPLE_DOWN_KEY          0x77E1B044
#define APPLE_LEFT_KEY          0x77E11044
#define APPLE_RIGHT_KEY         0x77E1E044
#define APPLE_MENU_KEY          0x77E14044

/*
 * 
 */
void main(void) {
    unsigned short pwkey = 0;
    TRISAbits.RA0 = 0;

    ir_enableIRIn();
    ir_blink13(0);
    while(1)
    {
        if (ir_decode(&dec_results)) {
            if (dec_results.decode_type == NEC)
            {
                
                switch(dec_results.value)
                {
                    case TERRATEC_1_KEY:
                        pwkey = pwkey << 4;
                        pwkey |= 1;
                        break;
                    case TERRATEC_2_KEY:
                        pwkey = pwkey << 4;
                        pwkey |= 2;
                        break;
                    case TERRATEC_3_KEY:
                        pwkey = pwkey << 4;
                        pwkey |= 3;
                        break;
                    case TERRATEC_4_KEY:
                        pwkey = pwkey << 4;
                        pwkey |= 4;
                        break;
                    case TERRATEC_5_KEY:
                        pwkey = pwkey << 4;
                        pwkey |= 5;
                        break;
                    case TERRATEC_6_KEY:
                        pwkey = pwkey << 4;
                        pwkey |= 6;
                        break;
                    case TERRATEC_7_KEY:
                        pwkey = pwkey << 4;
                        pwkey |= 7;
                        break;
                    case TERRATEC_8_KEY:
                        pwkey = pwkey << 4;
                        pwkey |= 8;
                        break;
                    case TERRATEC_9_KEY:
                        pwkey = pwkey << 4;
                        pwkey |= 9;
                        break;
                    case TERRATEC_0_KEY:
                        pwkey = pwkey << 4;
                        pwkey |= 0;
                        break;
                    case REPEAT:
                        break;
                    case TERRATEC_OK_KEY:
                        ir_sendNEC(APPLE_PLAY_KEY,32);
                        break;
                    case TERRATEC_UP_KEY:
                        ir_sendNEC(APPLE_UP_KEY,32);
                        break;
                    case TERRATEC_DOWN_KEY:
                        ir_sendNEC(APPLE_DOWN_KEY,32);
                        break;
                    case TERRATEC_LEFT_KEY:
                        ir_sendNEC(APPLE_LEFT_KEY,32);
                        break;
                    case TERRATEC_RIGHT_KEY:
                        ir_sendNEC(APPLE_RIGHT_KEY,32);
                        break;
                    case TERRATEC_INFO_KEY:
                        ir_sendNEC(APPLE_MENU_KEY,32);
                        break;
                    default:
                        LATAbits.LATA0 = 0;
                        pwkey = pwkey << 4;
                        break;
                }
                if (pwkey == 0x7412)
                {

                        LATAbits.LATA0 = 1;
                }
                else
                {
                        LATAbits.LATA0 = 0;
                }
            }
            //do something here
            ir_resume(); // Receive the next value
        }
    }

}


// XC8 way of interrupt handler definition
void interrupt InterruptHandlerHigh(void)
{
    ir_interruptService();
}

/*
// C18 way of interrupt handler definition
#pragma code
#pragma interrupt InterruptHandlerHigh
void InterruptHandlerHigh ()
{
    ir_interruptService();
}

//----------------------------------------------------------------------------
// High priority interrupt vector

#pragma code InterruptVectorHigh = 0x08
void
InterruptVectorHigh (void)
{
  _asm
    goto InterruptHandlerHigh
  _endasm
}

//----------------------------------------------------------------------------
// High priority interrupt routine
*/
