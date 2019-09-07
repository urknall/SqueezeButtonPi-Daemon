//
//  GPIO.h
//  SqueezeButtonPi
//
//  Created by Jörg Schwieder on 02.02.17.
//
//
//  Copyright (c) 2017, Joerg Schwieder, PenguinLovesMusic.com
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of ickStream nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
//  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#ifndef GPIO_h
#define GPIO_h

#include "sbpd.h"
#include "time.h"


//
//
//  Init GPIO functionality
//  Connect to the pigpiod interface.
//
//

int init_GPIO();

void shutdown_GPIO( int pi );
//
// Buttons and Rotary Encoders
// Rotary Encoder taken from https://github.com/astine/rotaryencoder
// http://theatticlight.net/posts/Reading-a-Rotary-Encoder-from-a-Raspberry-Pi/
//

//17 pins / 2 pins per encoder = 8 maximum encoders
#define max_encoders 8
//17 pins / 1 pins per button = 17 maximum buttons
#define max_buttons 17

struct button;

#define SHORTPRESS 0
#define LONGPRESS 1

//
//  A callback executed when a button gets triggered. Button struct and change returned.
//  Note: change might be "0" indicating no change, this happens when buttons chatter
//  Value in struct already updated.
//
typedef void (*button_callback_t)(const struct button * button, int change, bool presstype);

struct button {
    int pi;
    int pin;
    volatile bool value;
    button_callback_t callback;
    uint32_t timepressed;
    bool pressed;
    int long_press_time;
    int cb_id;
};

//
//
//  Configuration function to define a button
//  Should be run for every button you want to control
//  For each button a button struct will be created
//
//  Parameters:
//      pin: GPIO-Pin used in BCM numbering scheme
//      callback: callback function to be called when button state changed
//      edge: edge to be used for trigger events,
//            one of INT_EDGE_RISING, INT_EDGE_FALLING or INT_EDGE_BOTH (the default)
//  Returns: pointer to the new button structure
//           The pointer will be NULL is the function failed for any reason
//
//
struct button *setupbutton(int pi,
                           int pin,
                           button_callback_t b_callback,
                           int resist,
                           bool pressed,
                           int long_press_time);

struct encoder;

//
//  A callback executed when a rotary encoder changes it's value.
//  Encoder struct and change returned.
//  Value in struct already updated.
//
typedef void (*rotaryencoder_callback_t)(const struct encoder * encoder, long change);

struct encoder
{
    int pi;
    int pin_a;
    int pin_b;
    volatile long value;
    volatile int lastEncoded;
    rotaryencoder_callback_t callback;
    int mode;
    int cba_id;
    int cbb_id;
};

//
//
//  Configuration function to define a rotary encoder
//  Should be run for every rotary encoder you want to control
//  For each encoder a button struct will be created
//
//  Parameters:
//      pin_a, pin_b: GPIO-Pins used in BCM numbering scheme
//      callback: callback function to be called when encoder state changed
//      edge: edge to be used for trigger events,
//            one of INT_EDGE_RISING, INT_EDGE_FALLING or INT_EDGE_BOTH (the default)
//  Returns: pointer to the new encoder structure
//           The pointer will be NULL is the function failed for any reason
//
//
struct encoder *setupencoder(int pi,
                             int pin_a,
                             int pin_b,
                             rotaryencoder_callback_t callback,
                             int edge,
                             int mode);

#define ENCODER_MODE_DETENT 0
#define ENCODER_MODE_STEP   1

/*

RED starts a rotary encoder on Pi pi with GPIO gpioA,
GPIO gpioB, mode mode, and callback cb_func.  The mode
determines whether the four steps in each detent are
reported as changes or just the detents.

If cb_func in not null it will be called at each position
change with the new position.

The current position can be read with RED_get_position and
set with RED_set_position.

Mechanical encoders may suffer from switch bounce.
RED_set_glitch_filter may be used to filter out edges
shorter than glitch microseconds.  By default a glitch
filter of 1000 microseconds is used.

At program end the rotary encoder should be cancelled using
RED_cancel.  This releases system resources.


RED_t *RED                   (int pi,
                              int gpioA,
                              int gpioB,
                              int mode,
                              RED_CB_t cb_func);

void   RED_cancel            (RED_t *renc);

void   RED_set_glitch_filter (RED_t *renc, int glitch);

void   RED_set_position      (RED_t *renc, int position);

int    RED_get_position      (RED_t *renc);
*/



#endif /* GPIO_h */
