//
//  ClickEncoderStream.h
//  
//
//  Created by Christophe Persoz on 17/07/2016.
//  Copyright © 2016 Christophe Persoz. All rights reserved.
//

#ifndef ClickEncoderStream_h
#define ClickEncoderStream_h

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "ClickEncoder.h"

/*  emulate a stream based on clickEncoder movement returning +/- for every 'sensivity' steps
 buffer not needer because we have an accumulator
 */
class ClickEncoderStream:public Stream {
public:
    ClickEncoder &enc; //associated hardware clickEncoder
    int8_t sensivity;
    int oldPos;
    int pos;
    ClickEncoder::Button btn;

    inline void update() {
      pos+=enc.getValue();
      if (btn==ClickEncoder::Open) btn=enc.getButton();
    }
    ClickEncoderStream(ClickEncoder &enc,int sensivity)
      :enc(enc),
      pos(0),
      oldPos(0),
      btn(ClickEncoder::Open),
      sensivity(sensivity) {}

    inline void setSensivity(int s) {sensivity = s;}
    int available(void) {return peek()!=-1;}

    int peek(void) {
        update();
        if (btn==ClickEncoder::Clicked) return menu::enterCode;
        int d = pos - oldPos;
        if (d <= -sensivity)
            return menu::downCode;
        if (d >= sensivity)
            return menu::upCode;
        return -1;
    }

    int read()
    {
        int ch=peek();
        btn=ClickEncoder::Open;
        if(ch==menu::upCode) oldPos += sensivity;
        else if(ch==menu::downCode) oldPos -= sensivity;
        return ch;
    }

    void flush() {
        update();
        oldPos = pos;//encValue;
    }
    size_t write(uint8_t v)
    {
        oldPos = v;
        return 1;
    }
};


#endif /* ClickEncoderStream_h */