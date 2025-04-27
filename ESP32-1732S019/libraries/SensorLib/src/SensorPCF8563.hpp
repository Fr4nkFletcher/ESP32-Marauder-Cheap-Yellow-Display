/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      SensorPCF8563.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-12-09
 *
 */
#pragma once

#include "REG/PCF8563Constants.h"
#include "SensorCommon.tpp"
#include "SensorRTC.h"

class SensorPCF8563 :
    public SensorCommon<SensorPCF8563>,
    public RTCCommon<SensorPCF8563>
{
    friend class SensorCommon<SensorPCF8563>;
    friend class RTCCommon<SensorPCF8563>;
public:

    enum {
        CLK_32_768KHZ,
        CLK_1024KHZ,
        CLK_32HZ,
        CLK_1HZ,
    };


#if defined(ARDUINO)
    SensorPCF8563(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = PCF8563_SLAVE_ADDRESS)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __addr = addr;
    }
#endif

    SensorPCF8563()
    {
#if defined(ARDUINO)
        __wire = &Wire;
        __sda = DEFAULT_SDA;
        __scl = DEFAULT_SCL;
#endif
        __addr = PCF8563_SLAVE_ADDRESS;
    }

    ~SensorPCF8563()
    {
        deinit();
    }

#if defined(ARDUINO)
    bool init(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = PCF8563_SLAVE_ADDRESS)
    {
        return SensorCommon::begin(w, addr, sda, scl);
    }
#endif


    void deinit()
    {
        // end();
    }

    void setDateTime(RTC_DateTime datetime)
    {
        setDateTime(datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);
    }


    void setDateTime(uint16_t year,
                     uint8_t month,
                     uint8_t day,
                     uint8_t hour,
                     uint8_t minute,
                     uint8_t second)
    {
        uint8_t buffer[7];
        buffer[0] = DEC2BCD(second) & 0x7F;
        buffer[1] = DEC2BCD(minute);
        buffer[2] = DEC2BCD(hour);
        buffer[3] = DEC2BCD(day);
        buffer[4] = getDayOfWeek(day, month, year);
        buffer[5] = DEC2BCD(month);
        buffer[6] = DEC2BCD(year % 100);

        if ((2000 % year) == 2000) {
            buffer[5] &= 0x7F;
        } else {
            buffer[5] |= 0x80;
        }
        writeRegister(PCF8563_SEC_REG, buffer, 7);
    }


    RTC_DateTime getDateTime()
    {
        RTC_DateTime datetime;
        uint8_t buffer[7];
        readRegister(PCF8563_SEC_REG, buffer, 7);
        datetime.available = ((buffer[0] & 0x80) == 0x80) ? false : true;
        datetime.second = BCD2DEC(buffer[0] & 0x7F);
        datetime.minute = BCD2DEC(buffer[1] & 0x7F);
        datetime.hour   = BCD2DEC(buffer[2] & 0x3F);
        datetime.day    = BCD2DEC(buffer[3] & 0x3F);
        datetime.week   = BCD2DEC(buffer[4] & 0x07);
        datetime.month  = BCD2DEC(buffer[5] & 0x1F);
        datetime.year   = BCD2DEC(buffer[6]);
        //cetury :  0 = 1900 , 1 = 2000
        datetime.year += (buffer[5] & PCF8563_CENTURY_MASK) ?  1900 : 2000;
        return datetime;
    }

    void getDateTime(struct tm *timeinfo)
    {
        if (!timeinfo)return;
        *timeinfo = conversionUnixTime(getDateTime());
    }

    RTC_Alarm getAlarm()
    {
        uint8_t buffer[4];
        readRegister(PCF8563_ALRM_MIN_REG, buffer, 4);
        buffer[0] = BCD2DEC(buffer[0] & 0x80); //minute
        buffer[1] = BCD2DEC(buffer[1] & 0x40); //hour
        buffer[2] = BCD2DEC(buffer[2] & 0x40); //day
        buffer[3] = BCD2DEC(buffer[3] & 0x08); //weekday
        // RTC_Alarm(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day, uint8_t week)
        return RTC_Alarm(buffer[1], buffer[0], 0, buffer[2], buffer[3]);
    }

    void enableAlarm()
    {
        setRegisterBit(PCF8563_STAT2_REG, 1);
    }

    void disableAlarm()
    {
        clrRegisterBit(PCF8563_STAT2_REG, 1);
    }

    void resetAlarm()
    {
        clrRegisterBit(PCF8563_STAT2_REG, 3);
    }

    bool isAlarmActive()
    {
        return getRegisterBit(PCF8563_STAT2_REG, 3);
    }

    void setAlarm(RTC_Alarm alarm)
    {
        setAlarm( alarm.hour, alarm.minute, alarm.day, alarm.week);
    }

    void setAlarm(uint8_t hour, uint8_t minute, uint8_t day, uint8_t week)
    {
        uint8_t buffer[4] = {0};

        RTC_DateTime datetime =  getDateTime();

        uint8_t daysInMonth =  getDaysInMonth(datetime.month, datetime.year);

        if (minute != PCF8563_NO_ALARM) {
            if (minute > 59) {
                minute = 59;
            }
            buffer[0] = DEC2BCD(minute);
            buffer[0] &= ~PCF8563_ALARM_ENABLE;
        } else {
            buffer[0] = PCF8563_ALARM_ENABLE;
        }

        if (hour != PCF8563_NO_ALARM) {
            if (hour > 23) {
                hour = 23;
            }
            buffer[1] = DEC2BCD(hour);
            buffer[1] &= ~PCF8563_ALARM_ENABLE;
        } else {
            buffer[1] = PCF8563_ALARM_ENABLE;
        }
        if (day != PCF8563_NO_ALARM) {
            //TODO:Check  day in Month
            buffer[2] = DEC2BCD(((day) < (1) ? (1) : ((day) > (daysInMonth) ? (daysInMonth) : (day))));
            buffer[2] &= ~PCF8563_ALARM_ENABLE;
        } else {
            buffer[2] = PCF8563_ALARM_ENABLE;
        }
        if (week != PCF8563_NO_ALARM) {
            if (week > 6) {
                week = 6;
            }
            buffer[3] = DEC2BCD(week);
            buffer[3] &= ~PCF8563_ALARM_ENABLE;
        } else {
            buffer[3] = PCF8563_ALARM_ENABLE;
        }
        writeRegister(PCF8563_ALRM_MIN_REG, buffer, 4);
    }

    void setAlarmByMinutes(uint8_t minute)
    {
        setAlarm(PCF8563_NO_ALARM, minute, PCF8563_NO_ALARM, PCF8563_NO_ALARM);
    }
    void setAlarmByDays(uint8_t day)
    {
        setAlarm(PCF8563_NO_ALARM, PCF8563_NO_ALARM, day, PCF8563_NO_ALARM);
    }
    void setAlarmByHours(uint8_t hour)
    {
        setAlarm(hour, PCF8563_NO_ALARM, PCF8563_NO_ALARM, PCF8563_NO_ALARM);
    }
    void setAlarmByWeekDay(uint8_t week)
    {
        setAlarm(PCF8563_NO_ALARM, PCF8563_NO_ALARM, PCF8563_NO_ALARM, week);
    }

    bool isCountdownTimerEnable()
    {
        uint8_t buffer[2];
        buffer[0] = readRegister(PCF8563_STAT2_REG);
        buffer[1] = readRegister(PCF8563_TIMER1_REG);
        if (buffer[0] & PCF8563_TIMER_TIE) {
            return buffer[1] & PCF8563_TIMER_TE ? true : false;
        }
        return false;
    }

    bool isCountdownTimerActive()
    {
        return getRegisterBit(PCF8563_STAT2_REG, 2);
    }

    void enableCountdownTimer()
    {
        setRegisterBit(PCF8563_STAT2_REG, 0);
    }

    void disableCountdownTimer()
    {
        clrRegisterBit(PCF8563_STAT2_REG, 0);
    }

    void setCountdownTimer(uint8_t val, uint8_t freq)
    {
        uint8_t buffer[3];
        buffer[1] = readRegister(PCF8563_TIMER1_REG);
        buffer[1] |= (freq &  PCF8563_TIMER_TD10);
        buffer[2] = val;
        writeRegister(PCF8563_TIMER1_REG, buffer[1]);
        writeRegister(PCF8563_TIMER2_REG, buffer[2]);
    }

    void clearCountdownTimer()
    {
        uint8_t val;
        val = readRegister(PCF8563_STAT2_REG);
        val &= ~(PCF8563_TIMER_TF | PCF8563_TIMER_TIE);
        val |= PCF8563_ALARM_AF;
        writeRegister(PCF8563_STAT2_REG, val);
        writeRegister(PCF8563_TIMER1_REG, 0x00);
    }

    void enableCLK(uint8_t freq)
    {
        if (freq > CLK_1HZ) return;
        writeRegister(PCF8563_SQW_REG,  freq | PCF8563_CLK_ENABLE);
    }

    void disableCLK()
    {
        clrRegisterBit(PCF8563_SQW_REG, 7);
    }



private:

    bool initImpl()
    {
        // Check whether RTC time is valid? If it is invalid, it can be judged
        // that there is a problem with the hardware, or the RTC power supply voltage is too low
        /*
        int count = 0;
        for (int i = 0; i < 3; ++i) {
            if (!getRegisterBit(PCF8563_SEC_REG, 7)) {
                count++;
            }
        }
        if (count != 3 ) {
            return false;
        }
        */

        // 230704:Does not use power-off judgment, if the RTC backup battery is not installed,
        //    it will return failure. Here only to judge whether the device communication is normal

        //Check device is online
        int ret = readRegister(PCF8563_SEC_REG);
        if (ret == DEV_WIRE_ERR) {
            return false;
        }
        if (BCD2DEC(ret & 0x7F) > 59) {
            return false;
        }

        // Determine whether the hardware clock year, month, and day match the internal time of the RTC.
        // If they do not match, it will be updated to the compilation date
        // RTC_DateTime compileDatetime =  RTC_DateTime(__DATE__, __TIME__);
        // RTC_DateTime hwDatetime = getDateTime();
        // if (compileDatetime.year != hwDatetime.year ||
        //         compileDatetime.month != hwDatetime.month ||
        //         compileDatetime.day != hwDatetime.month
        //    ) {
        //     LOG("No match yy:mm:dd . set datetime to compilation date time");
        //     setDateTime(compileDatetime);
        // }
        return true;
    }

    int getReadMaskImpl()
    {
        return -1;
    }

protected:


};



