//
//  uinput.h
//  SqueezeButtonPi
//

#ifndef _uinput_h
#define _uinput_h

int init_uinput(void);
int disconnect_uinput(void);
int emitKey(int key, int value);

#endif
