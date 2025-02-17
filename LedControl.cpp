/*
 *    LedControl.cpp - A library for controling Leds with a MAX7219/MAX7221
 *    Copyright (c) 2007 Eberhard Fahle
 *
 *    Permission is hereby granted, free of charge, to any person
 *    obtaining a copy of this software and associated documentation
 *    files (the "Software"), to deal in the Software without
 *    restriction, including without limitation the rights to use,
 *    copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the
 *    Software is furnished to do so, subject to the following
 *    conditions:
 *
 *    This permission notice shall be included in all copies or
 *    substantial portions of the Software.
 *
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *    OTHER DEALINGS IN THE SOFTWARE.
 */


#include "LedControl.h"

//the opcodes for the MAX7221 and MAX7219
// 数字显示控制 (0-8)
// 空操作指令，不执行任何操作
#define OP_NOOP   0
// 控制数码管位置1~8的显示内容
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
// 设置解码模式，决定是使用BCD码解码还是直接显示段码
#define OP_DECODEMODE  9
// 控制显示亮度，可以设置不同级别的亮度
#define OP_INTENSITY   10
// 设置扫描限制，控制要显示的位数
#define OP_SCANLIMIT   11
// 关机模式控制，可以打开或关闭显示
#define OP_SHUTDOWN    12
// 显示测试模式，所有LED段都会被点亮
#define OP_DISPLAYTEST 15

LedControl::LedControl(int dataPin, int clkPin, int csPin, int numDevices) {
    mySPI_MOSI=dataPin;
    mySPI_CLK=clkPin;
    mySPI_CS=csPin;
    if(numDevices<=0 || numDevices>8 )
        numDevices=8;
    maxDevices=numDevices;
    pinMode(mySPI_MOSI,OUTPUT);
    pinMode(mySPI_CLK,OUTPUT);
    pinMode(mySPI_CS,OUTPUT);
    digitalWrite(mySPI_CS,HIGH);
    mySPI_MOSI=dataPin;
    for(int i=0;i<64;i++)
        status[i]=0x00;
    for(int i=0;i<maxDevices;i++) {
        spiTransfer(i,OP_DISPLAYTEST,0);
        //scanlimit is set to max on startup
        setScanLimit(i,7);
        //decode is done in source
        spiTransfer(i,OP_DECODEMODE,0);
        clearDisplay(i);
        //we go into shutdown-mode on startup
        shutdown(i,true);
    }
}

int LedControl::getDeviceCount() {
    return maxDevices;
}

void LedControl::shutdown(int addr, bool b) {
    if(addr<0 || addr>=maxDevices)
        return;
    if(b)
        spiTransfer(addr, OP_SHUTDOWN,0);
    else
        spiTransfer(addr, OP_SHUTDOWN,1);
}

void LedControl::setScanLimit(int addr, int limit) {
    if(addr<0 || addr>=maxDevices)
        return;
    if(limit>=0 && limit<8)
        spiTransfer(addr, OP_SCANLIMIT,limit);
}

void LedControl::setIntensity(int addr, int intensity) {
    if(addr<0 || addr>=maxDevices)
        return;
    if(intensity>=0 && intensity<16)
        spiTransfer(addr, OP_INTENSITY,intensity);
}

void LedControl::clearDisplay(int addr) {
    int offset;

    if(addr<0 || addr>=maxDevices)
        return;
    offset=addr*8;
    for(int i=0;i<8;i++) {
        status[offset+i]=0;
        spiTransfer(addr, i+1,status[offset+i]);
    }
}

void LedControl::setRotation(int rot) {
  rotation = rot;
}

coord LedControl::flipHorizontally(coord xy) {
  xy.x = 7- xy.x;
  return xy;
}

coord LedControl::flipVertically(coord xy) {
  xy.y = 7- xy.y;
  return xy;
}

coord LedControl::rotate90(coord xy) {
  int tmp = xy.y;
  xy.y = xy.x;
  xy.x = tmp;
  return flipHorizontally(xy);
}

coord LedControl::rotate180(coord xy) {
  return flipHorizontally(flipVertically(xy));
}

coord LedControl::rotate270(coord xy) {
  return rotate180(rotate90(xy));
}

coord LedControl::transform(coord xy) {
  if (rotation == 90) {
    xy = rotate90(xy);
  } else if (rotation == 180) {
    xy = rotate180(xy);
  } else if (rotation == 270) {
    xy = rotate270(xy);
  }
  return xy;
}

coord LedControl::transform(int x, int y) {
  coord xy;
  xy.x = x;
  xy.y =y;
  return transform(xy);
}

void LedControl::setXY(int addr, int x, int y, boolean state) {
  coord xy;
  xy.x = x;
  xy.y = y;
  xy = transform(xy);
  setLed(addr, xy.y, xy.x, state);
}

void LedControl::setRawXY(int addr, int x, int y, boolean state) {
  setLed(addr, y, x, state);
}

boolean LedControl::getXY(int addr, int x, int y) {
  coord xy;
  xy.x = x;
  xy.y = y;
  xy = transform(xy);
  return getLed(addr, xy.y, xy.x);
}

boolean LedControl::getRawXY(int addr, int x, int y) {
  return getLed(addr, y, x);
}

void LedControl::setXY(int addr, coord xy, boolean state) {
  setXY(addr, xy.x, xy.y, state);
}

void LedControl::setLed(int addr, int row, int column, boolean state) {
    int offset;
    byte val=0x00;

    if(addr<0 || addr>=maxDevices)
        return;
    if(row<0 || row>7 || column<0 || column>7)
        return;
    offset=addr*8;
    val=B10000000 >> column;
    if(state)
        status[offset+row]=status[offset+row]|val;
    else {
        val=~val;
        status[offset+row]=status[offset+row]&val;
    }
    spiTransfer(addr, row+1,status[offset+row]);
}

void LedControl::invertRawXY(int addr, int x, int y) {
  return setRawXY(addr, x, y, !getRawXY(addr, x, y));
}

void LedControl::invertXY(int addr, int x, int y) {
  return setXY(addr, x, y, !getXY(addr, x, y));
}

boolean LedControl::getXY(int addr, coord xy) {
  return getXY(addr, xy.x, xy.y);
}

boolean LedControl::getLed(int addr, int row, int column) {
    int offset;
    boolean state;

    if(addr<0 || addr>=maxDevices)
        return false;
    if(row<0 || row>7 || column<0 || column>7)
        return false;
    offset=addr*8;
    state = (1 == ( (status[offset+row] >> (7-column)) & 1));
    return state;
}

void LedControl::setRow(int addr, int row, byte value) {
    int offset;
    if(addr<0 || addr>=maxDevices)
        return;
    if(row<0 || row>7)
        return;
    offset=addr*8;
    status[offset+row]=value;
    spiTransfer(addr, row+1,status[offset+row]);
}

void LedControl::setColumn(int addr, int col, byte value) {
    byte val;

    if(addr<0 || addr>=maxDevices)
        return;
    if(col<0 || col>7)
        return;
    for(int row=0;row<8;row++) {
        val=value >> (7-row);
        val=val & 0x01;
        setLed(addr,row,col,val);
    }
}

void LedControl::setDigit(int addr, int digit, byte value, boolean dp) {
    int offset;
    byte v;

    if(addr<0 || addr>=maxDevices)
        return;
    if(digit<0 || digit>7 || value>15)
        return;
    offset=addr*8;
    v=pgm_read_byte_near(charTable + value);
    if(dp)
        v|=B10000000;
    status[offset+digit]=v;
    spiTransfer(addr, digit+1,v);
}

void LedControl::setChar(int addr, int digit, char value, boolean dp) {
    int offset;
    byte index,v;

    if(addr<0 || addr>=maxDevices)
        return;
    if(digit<0 || digit>7)
        return;
    offset=addr*8;
    index=(byte)value;
    if(index >127) {
        //no defined beyond index 127, so we use the space char
        index=32;
    }
    v=pgm_read_byte_near(charTable + index);
    if(dp)
        v|=B10000000;
    status[offset+digit]=v;
    spiTransfer(addr, digit+1,v);
}

void LedControl::spiTransfer(int addr, volatile byte opcode, volatile byte data) {
    //Create an array with the data to shift out
    int offset=addr*2;
    int maxbytes=maxDevices*2;

    for(int i=0;i<maxbytes;i++)
        spidata[i]=(byte)0;
    //put our device data into the array
    spidata[offset+1]=opcode;
    spidata[offset]=data;
    //enable the line
    digitalWrite(mySPI_CS,LOW);
    //Now shift out the data
    for(int i=maxbytes;i>0;i--)
        shiftOut(mySPI_MOSI,mySPI_CLK,MSBFIRST,spidata[i-1]);
    //latch the data onto the display
    digitalWrite(mySPI_CS,HIGH);
}

void LedControl::backup() {
  memcpy(backupStatus, status, 64);
}
void LedControl::restore() {
  memcpy(status, backupStatus, 64);
  int offset;
  for (int addr=0; addr<maxDevices; addr++) {
    offset=addr*8;
    for(int i=0;i<8;i++) {
      spiTransfer(addr, i+1,status[offset+i]);
    }
  }
}
