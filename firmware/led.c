/**
* Woodchuck by CU Spaceflight
*
* This file is part of the Woodchuck project by Cambridge University Spaceflight.
*
* It is an adapted version of the JOEY-M version by Jon Sowman
*
*
* Jon Sowman 2012
*
* Gregory Brooks 2016
*
*/
#include <string.h>
#include "led.h"
//#include "clock.h"

#define PORT_LED GPIOA
#define PIN_GREEN (1 << GPIOA_LED_GREEN)
#define PIN_RED (1 << GPIOA_LED_RED)
#define PIN_BLUE (1 << GPIOA_LED_BLUE)
#define PIN_GB (PIN_GREEN | PIN_BLUE)
#define PIN_RG (PIN_RED | PIN_GREEN)
#define PIN_RB (PIN_RED | PIN_BLUE)
#define PIN_RGB (PIN_RED | PIN_GREEN | PIN_BLUE)


static const uint16_t pins[COLOURS_SIZE] = {PIN_GREEN, PIN_RED, PIN_BLUE, PIN_GB, PIN_RG, PIN_RB, PIN_RGB};
static binary_semaphore_t led_bsem;
static led_modes statuses[COLOURS_SIZE] = {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF};
static mutex_t led_status_mtx;
static thread_t * led_tp;


void led_out(led_colours colour){
    uint16_t bits;
    if(colour==COLOURS_SIZE){
        // Turn everything off
        bits = PIN_RGB;
    }
    else{
        bits = ~pins[colour];
    }
    chSysLock();
    palWritePort(PORT_LED, bits);
    chSysUnlock();
}


void led_set_stat(led_modes arg[COLOURS_SIZE]){
    bool ON_FLAG = false;
    chMtxLock(&led_status_mtx);
    for(int x = 0; x < COLOURS_SIZE; x++){
        if(arg[x] == LED_ON){
            if(!ON_FLAG){
                statuses[x] = arg[x];
                ON_FLAG = true;
            }
            else{
                statuses[x] = LED_OFF;
            }
        }
        else if (arg[x] < MODES_SIZE) statuses[x] = arg[x];
    }
    chMtxUnlock(&led_status_mtx);
    //led_reset();
    chBSemSignal(&led_bsem);
}


// bool checking_sleep(uint8_t sec, uint8_t freq_Hz){
//     for(int y = 0; y < sec*freq_Hz; y++){
//         if(chThdShouldTerminateX()){
//             return true;
//         }
//         chThdSleepMilliseconds(1000/freq_Hz);  // Argument is an integer
//     }
//     return false;
// }


static THD_WORKING_AREA(led_thread_wa,128);
static THD_FUNCTION(led_thread, arg){
    (void)arg;
    chRegSetThreadName("LED");

    while(true){
        chMtxLock(&led_status_mtx);

        // Copy shared status array into local array, to free mutex lock quickly
        led_modes status_copy[COLOURS_SIZE];
        memcpy(status_copy, statuses, sizeof(status_copy));

        chMtxUnlock(&led_status_mtx);


        /*
         * Variable to hold identity of 'ON' colour
         * Equals COLOURS_SIZE if nothing is 'ON'
         */
        led_colours on_led = COLOURS_SIZE;

        for (int x = 0; x < COLOURS_SIZE; x++){
            if (status_copy[x] == LED_ON){
                on_led = x;
            }
        }

        led_out(on_led);
        msg_t result;

        if(on_led == COLOURS_SIZE){
            // Time to wait between blinking runs
            uint8_t blink_wait_time = 5;
            result = chBSemWaitTimeout(&led_bsem,S2ST(blink_wait_time));
        }
        else{
            // led on overrides blinking routine
            result = chBSemWait(&led_bsem);
        }

        if(result == MSG_OK){
            // Semaphore has been set, refresh statuses
            chBSemReset(&led_bsem, TRUE);
            continue;
        }


        // Show each 'blinking' colour for 1 second
        uint8_t blink_time = 1;

        for(int x = 0; x < COLOURS_SIZE; x++){
            if (status_copy[x] == LED_BLINKING){
                led_out(x);
                //if(checking_sleep(blink_time, 10)) chThdExit((msg_t)0);
                msg_t result = chBSemWaitTimeout(&led_bsem,S2ST(blink_time));
                if(result == MSG_OK){
                    // Semaphore has been set, refresh statuses
                    chBSemReset(&led_bsem, TRUE);
                    break;
                }
            }
        }

    }
}


void led_init(void)
{
    chMtxObjectInit(&led_status_mtx);
    chBSemObjectInit(&led_bsem, true);
    led_tp = chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa), NORMALPRIO, led_thread, NULL);
}

void led_exit(bool blocking)
{
    chThdTerminate(led_tp);
    if(blocking){
        chThdWait(led_tp);
    }
}


void led_set(led_colours led, led_modes status)
/**
 * Set the given LED to the given state.
 */
{
    chMtxLock(&led_status_mtx);
    if(statuses[led] != status){
    	switch (status) {
    	case LED_OFF:
    		statuses[led] = LED_OFF;
    		break;
    	case LED_ON:
            /*
             * Only one colour can be fully 'ON' at a time
             * Use blinking to cycle through colours
             */
            for(int x = 0; x < COLOURS_SIZE; x++){
                if (statuses[x] == LED_ON){
                    statuses[x] = LED_OFF;
                }
            }
    		statuses[led] = LED_ON;
    		break;
    	case LED_BLINKING:
            // Blinking leds are queued and blinked sequentially
    		statuses[led] = LED_BLINKING;
    		break;
    	default:
    		// Do nothing
    		break;
    	}
    }
    chMtxUnlock(&led_status_mtx);

    //led_reset();
    chBSemSignal(&led_bsem);
}


void led_toggle(led_colours led){
    if (led_read(led) == LED_OFF){
        led_set(led, LED_ON);
    }
    else if (led_read(led) == LED_ON){
        led_set(led, LED_OFF);
    }
    else{
        // Do nothing
    }
}


void led_clear(void){
    led_modes off[COLOURS_SIZE] = {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF};
    led_set_stat(off);
}


void led_reset(void)
{
    // Terminate LED thread
    led_exit(true);

    // Restart LED thread
    led_tp = chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa), NORMALPRIO, led_thread, NULL);
}


led_modes led_read(led_colours led)
/**
 * Returns item from state array
 */
{
    chMtxLock(&led_status_mtx);
	led_modes rtrn = statuses[led];
    chMtxUnlock(&led_status_mtx);
    return rtrn;
}
