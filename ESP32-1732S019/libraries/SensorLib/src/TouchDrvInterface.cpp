/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
 *
 * Permission is hereby granted,free of charge,to any person obtaining a copy
 * of this software and associated documentation files (the "Software"),to deal
 * in the Software without restriction,including without limitation the rights
 * to use,copy,modify,merge,publish,distribute,sublicense,and/or sell
 * copies of the Software,and to permit persons to whom the Software is
 * furnished to do so,subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS",WITHOUT WARRANTY OF ANY KIND,EXPRESS OR
 * IMPLIED,INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,DAMAGES OR OTHER
 * LIABILITY,WHETHER IN AN ACTION OF CONTRACT,TORT OR OTHERWISE,ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      TouchDrvInterface.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-21
 *
 */
#include "TouchDrvInterface.hpp"

TouchDrvInterface::TouchDrvInterface() :
    __resX(0),
    __resY(0),
    __xMax(0),
    __yMax(0),
    __swapXY(false),
    __mirrorX(false),
    __mirrorY(false),
    __rst(-1),
    __irq(-1),
    __chipID(0x00),
    __homeButtonCb(NULL),
    __userData(NULL)
{

}

TouchDrvInterface::~TouchDrvInterface()
{

}

uint32_t TouchDrvInterface::getChipID()
{
    return __chipID;
}


void TouchDrvInterface::setPins(int rst, int irq)
{
    __irq = irq;
    __rst = rst;
}

void TouchDrvInterface::setSwapXY(bool swap)
{
    __swapXY = swap;
}

void TouchDrvInterface::setMirrorXY(bool mirrorX, bool mirrorY)
{
    __mirrorX = mirrorX;
    __mirrorY = mirrorY;
}

void TouchDrvInterface::setMaxCoordinates(uint16_t x, uint16_t y)
{
    __xMax = x;
    __yMax = y;
}

void TouchDrvInterface::updateXY(uint8_t pointNum, int16_t *xBuffer, int16_t *yBuffer)
{
    if (!pointNum)
        return;
    for (int i = 0; i < pointNum; ++i) {
        if (__swapXY) {
            uint16_t tmp = xBuffer[i];
            xBuffer[i] = yBuffer[i];
            yBuffer[i] = tmp;
        }
        if (__mirrorX && __xMax ) {
            xBuffer[i] = __xMax - xBuffer[i];
        }
        if (__mirrorY && __yMax) {
            yBuffer[i] = __yMax - yBuffer[i];
        }
    }
}
