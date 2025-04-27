/**
 *
 * @license MIT License
 *
 * Copyright (c) 2023 lewis he
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
 * @file      SensorRTC.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-07
 *
 */
#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdint.h>
#endif /*defined(ARDUINO)*/

#include <sys/time.h>

#define RTC_DAYS_IN_JANUARY     (31u)
#define RTC_DAYS_IN_FEBRUARY    (28u)
#define RTC_DAYS_IN_MARCH       (31u)
#define RTC_DAYS_IN_APRIL       (30u)
#define RTC_DAYS_IN_MAY         (31u)
#define RTC_DAYS_IN_JUNE        (30u)
#define RTC_DAYS_IN_JULY        (31u)
#define RTC_DAYS_IN_AUGUST      (31u)
#define RTC_DAYS_IN_SEPTEMBER   (30u)
#define RTC_DAYS_IN_OCTOBER     (31u)
#define RTC_DAYS_IN_NOVEMBER    (30u)
#define RTC_DAYS_IN_DECEMBER    (31u)

enum {
    DATETIME_FORMAT_HM,
    DATETIME_FORMAT_HMS,
    DATETIME_FORMAT_YYYY_MM_DD,
    DATETIME_FORMAT_MM_DD_YYYY,
    DATETIME_FORMAT_DD_MM_YYYY,
    DATETIME_FORMAT_YYYY_MM_DD_H_M_S,
    DATETIME_FORMAT_YYYY_MM_DD_H_M_S_WEEK,
};



class RTC_DateTime
{
public:
    RTC_DateTime() : year(0), month(0), day(0), hour(0), minute(0), second(0), week(0), available(false) {}
    RTC_DateTime(const char *date, const char *time)
    {
        // sample input: date = "Dec 26 2009", time = "12:34:56"
        year = 2000 + StringToUint8(date + 9);
        // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
        switch (date[0]) {
        case 'J':
            if ( date[1] == 'a' )
                month = 1;
            else if ( date[2] == 'n' )
                month = 6;
            else
                month = 7;
            break;
        case 'F':
            month = 2;
            break;
        case 'A':
            month = date[1] == 'p' ? 4 : 8;
            break;
        case 'M':
            month = date[2] == 'r' ? 3 : 5;
            break;
        case 'S':
            month = 9;
            break;
        case 'O':
            month = 10;
            break;
        case 'N':
            month = 11;
            break;
        case 'D':
            month = 12;
            break;
        }
        day = StringToUint8(date + 4);
        hour = StringToUint8(time);
        minute = StringToUint8(time + 3);
        second = StringToUint8(time + 6);
    }
    RTC_DateTime(uint16_t year,
                 uint8_t month,
                 uint8_t day,
                 uint8_t hour,
                 uint8_t minute,
                 uint8_t second
                )
    {
        this->year = year;
        this->month = month;
        this->day = day;
        this->hour = hour;
        this->minute = minute;
        this->second = second;
        this->week = week;
    };
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t week;
    char AMPM;
    bool available;
    bool operator==(RTC_DateTime d)
    {
        return ((d.year == year) &&
                (d.month == month) &&
                (d.day == day) &&
                (d.hour == hour) &&
                (d.minute == minute));
    }
private:
    uint8_t StringToUint8(const char *pString)
    {
        uint8_t value = 0;

        // skip leading 0 and spaces
        while ('0' == *pString || *pString == ' ') {
            pString++;
        }

        // calculate number until we hit non-numeral char
        while ('0' <= *pString && *pString <= '9') {
            value *= 10;
            value += *pString - '0';
            pString++;
        }
        return value;
    }
};

class RTC_Alarm
{
public:
    RTC_Alarm(void): second(0), minute(0), hour(0), day(0), week(0) {}
    RTC_Alarm(
        uint8_t hour,
        uint8_t minute,
        uint8_t second,
        uint8_t day,
        uint8_t week
    ): second(second), minute(minute), hour(hour), day(day), week(week)
    {
    }
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t week;
};

template <class chipType>
class RTCCommon
{
public:
    const char *strftime(uint8_t sytle = DATETIME_FORMAT_YYYY_MM_DD_H_M_S_WEEK)
    {
        const char *weekString[] = {"Sun", "Mon", "Tue", "Wed", "Thur", "Fri", "Sat"};
        static char format[64];
        RTC_DateTime t = thisChip().getDateTime();
        switch (sytle) {
        case DATETIME_FORMAT_HM:
            snprintf(format, sizeof(format), "%02d:%02d", t.hour, t.minute);
            break;
        case DATETIME_FORMAT_HMS:
            snprintf(format, sizeof(format), "%02d:%02d:%02d", t.hour, t.minute, t.second);
            break;
        case DATETIME_FORMAT_YYYY_MM_DD:
            snprintf(format, sizeof(format), "%d-%02d-%02d", t.year, t.month, t.day);
            break;
        case DATETIME_FORMAT_MM_DD_YYYY:
            snprintf(format, sizeof(format), "%02d-%02d-%d", t.month, t.day, t.year);
            break;
        case DATETIME_FORMAT_DD_MM_YYYY:
            snprintf(format, sizeof(format), "%02d-%02d-%d", t.day, t.month, t.year);
            break;
        case DATETIME_FORMAT_YYYY_MM_DD_H_M_S:
            snprintf(format, sizeof(format), "%d-%02d-%02d/%02d:%02d:%02d", t.year, t.month, t.day, t.hour, t.minute, t.second);
            break;
        case DATETIME_FORMAT_YYYY_MM_DD_H_M_S_WEEK:
            snprintf(format, sizeof(format), "%d-%02d-%02d/%02d:%02d:%02d - %s", t.year, t.month, t.day, t.hour, t.minute, t.second, weekString[t.week > 6 ? 0 : t.week]);
            break;
        default:
            snprintf(format, sizeof(format), "%02d:%02d", t.hour, t.minute);
            break;
        }
        return format;
    }

    struct tm conversionUnixTime(RTC_DateTime datetime)
    {
        struct tm t_tm;
        t_tm.tm_hour = datetime.hour;
        t_tm.tm_min = datetime.minute;
        t_tm.tm_sec = datetime.second;
        t_tm.tm_year = datetime.year - 1900;    //Year, whose value starts from 1900
        t_tm.tm_mon = datetime.month - 1;       //Month (starting from January, 0 for January) - Value range is [0,11]
        t_tm.tm_mday = datetime.day;
        t_tm.tm_wday = datetime.week;
        return t_tm;
    }

    time_t hwClockRead()
    {
        struct tm t_tm = conversionUnixTime(thisChip().getDateTime());
        struct timeval val;
        val.tv_sec = mktime(&t_tm);
        val.tv_usec = 0;
#if __BSD_VISIBLE
        settimeofday(&val, NULL);
#endif /*__BSD_VISIBLE*/
        return val.tv_sec;
    }

    void hwClockWrite()
    {
        time_t now;
        struct tm  info;
        time(&now);
        localtime_r(&now, &info);
        thisChip().setDateTime(info.tm_year + 1900,
                               info.tm_mon + 1,
                               info.tm_mday,
                               info.tm_hour,
                               info.tm_min,
                               info.tm_sec);
    }

    uint32_t getDayOfWeek(uint32_t day, uint32_t month, uint32_t year)
    {
        uint32_t val;
        if (month < 3) {
            month = 12u + month;
            year--;
        }
        val = (day + (((month + 1u) * 26u) / 10u) + year + (year / 4u) + (6u * (year / 100u)) + (year / 400u)) % 7u;
        if (0u == val) {
            val = 7;
        }
        return (val - 1);
    }

    uint8_t getNextMonth(uint8_t curMonth)
    {
        return ((curMonth < 12u) ? (curMonth + 1u) : 1u);
    }

    uint16_t getNextYear(uint16_t curYear)
    {
        return (curYear + 1u);
    }

    uint32_t getLeapYear(uint32_t year)
    {
        uint32_t val;
        if (((0u == (year % 4Lu)) && (0u != (year % 100Lu))) || (0u == (year % 400Lu))) {
            val = 1uL;
        } else {
            val = 0uL;
        }
        return val;
    }

    uint8_t getDaysInMonth(uint8_t month, uint16_t year)
    {
        const uint8_t daysInMonthTable[12] = {RTC_DAYS_IN_JANUARY,
                                              RTC_DAYS_IN_FEBRUARY,
                                              RTC_DAYS_IN_MARCH,
                                              RTC_DAYS_IN_APRIL,
                                              RTC_DAYS_IN_MAY,
                                              RTC_DAYS_IN_JUNE,
                                              RTC_DAYS_IN_JULY,
                                              RTC_DAYS_IN_AUGUST,
                                              RTC_DAYS_IN_SEPTEMBER,
                                              RTC_DAYS_IN_OCTOBER,
                                              RTC_DAYS_IN_NOVEMBER,
                                              RTC_DAYS_IN_DECEMBER
                                             };

        uint8_t val;
        val = daysInMonthTable[month - 1u];
        if (2 == month) {
            if (0u != getLeapYear(year)) {
                val++;
            }
        }
        return val;
    }

    uint8_t BCD2DEC(uint8_t val)
    {
        return ((val / 16 * 10) + (val % 16));
    }
    uint8_t DEC2BCD(uint8_t val)
    {
        return ((val / 10 * 16) + (val % 10));
    }

    /*
     * CRTP Helper
     */
protected:
    inline const chipType &thisChip() const
    {
        return static_cast<const chipType &>(*this);
    }

    inline chipType &thisChip()
    {
        return static_cast<chipType &>(*this);
    }

};