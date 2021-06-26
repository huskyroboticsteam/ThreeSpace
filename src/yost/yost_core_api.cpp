/********************************************//**
 * Copyright 1998-2014, Yost Labs Corporation.
 * This Source Code Form is subject to the terms of the YOST 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#include "yost_core_api.hpp"
#include <thread>
#include <algorithm>
#include <time.h>


namespace yost
{
    void swap16(U8* data)
    {
        U8 temp;
    
        _swap(data[0], data[1], temp);
    }
    
    void swap32(U8* data)
    {
        U8 temp;
    
        _swap(data[0], data[3], temp);
        _swap(data[1], data[2], temp);
    }
    
    void swap64(U8* data)
    {
        U8 temp;
    
        _swap(data[0], data[7], temp);
        _swap(data[1], data[6], temp);
        _swap(data[2], data[5], temp);
        _swap(data[3], data[4], temp);
    }

    float get_time()
    {
    	return float(clock()) / CLOCKS_PER_SEC;
    }
    
    void sleep_ms(U32 time_ms)
    {
    	std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
    }

    bool startsWith(std::string starts, std::string with)
    {
        return starts.substr(0, with.size()) == with;
    }

    uint8_t _compareFirmwareVersion(std::string version_string1, std::string version_string2)
    {
        int32_t day_int1;
        int32_t month_int1;
        int32_t year_int1;
        int32_t hour_int1;
        int32_t minute_int1;
        int32_t day_int2;
        int32_t month_int2;
        int32_t year_int2;
        int32_t hour_int2;
        int32_t minute_int2;
        int8_t i;
        const char * month_list[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
        
        // check year
        year_int1 = std::stoi(version_string1.substr(5, 4));
        year_int2 = std::stoi(version_string2.substr(5, 4));
        if (year_int1 < year_int2)
        {
            return 0;
        }
        else if (year_int1 > year_int2)
        {
            return 1;
        }
        // check month
        month_int1 = 0;
        month_int2 = 0;
        std::string month1 = version_string1.substr(2, 3);
        for (i=0; i<12; i++)
        {
            if (month1.compare(month_list[i]) == 0)
            {
                month_int1 = i + 1;
                break;
            }
        }
        std::string month2 = version_string2.substr(2, 3);
        for (i=0; i<12; i++)
        {
            if (month2.compare(month_list[i]) == 0)
            {
                month_int2 = i + 1;
                break;
            }
        }

        if (month_int1 < month_int2)
        {
            return 0;
        }
        else if (month_int1 > month_int2)
        {
            return 1;
        }
        // check day
        day_int1 = std::stoi(version_string1.substr(0, 2));
        day_int2 = std::stoi(version_string2.substr(0, 2));
        if (day_int1 < day_int2)
        {
            return 0;
        }
        else if (day_int1 > day_int2)
        {
            return 1;
        }
        // check hour
        hour_int1 = version_string1[9];
        hour_int2 = version_string2[9];
        if (hour_int1 < hour_int2){
            return 0;
        }
        else if (hour_int1 > hour_int2){
            return 1;
        }
        // check minute
        minute_int1 = std::stoi(version_string1.substr(10, 2));
        minute_int2 = std::stoi(version_string2.substr(10, 2));
        if (minute_int1 < minute_int2){
            return 0;
        }
        else if (minute_int1 > minute_int2){
            return 1;
        }
        return 0;
    }

    uint8_t _createChecksum(const uint8_t* command_bytes, const uint8_t num_bytes)
    {
        uint8_t chkSum = 0;
        uint8_t i;
        for (i=0; i<num_bytes; i++){
            chkSum += command_bytes[i];
        }
        return (chkSum % 256);
    }

    uint8_t _parseData(uint8_t* data, uint32_t len, std::string parser_str)
    {
        uint16_t parser_index = 0;
        uint16_t data_index = 0;
        while (parser_index < parser_str.size())
        {
            switch (parser_str[parser_index])
            {
                case 'q': //long long
                case 'Q': //unsigned long long
                case 'd': //double
                    yost::swap64(&data[data_index]);
                    data_index += 8;
                    break;
                case 'i': //int
                case 'I': //unsigned int
                case 'f': //float
                    yost::swap32(&data[data_index]);
                    data_index += 4;
                    break;
                case 'h': //short
                case 'H': //unsigned short
                    yost::swap16(&data[data_index]);
                    data_index += 2;
                    break;
                case 'b': //signed char
                case 'B': //unsigned char
                case '?': //bool
                case 's': //char[]
                case 'x': //pad byte
                    //No swaps needed on single bytes
                    data_index += 1;
                    break;
                default:
                    //should not get here
                    printf("should not get here: %c\n", parser_str[parser_index]);
                    break;
            }
            parser_index += 1;
        }

        if (data_index + 1 == len){
            return 1;
        }
        return 0;
    }

    uint16_t _calculateParseDataSize(std::string parser_str)
    {
        uint16_t parser_index = 0;
        uint16_t data_index = 0;
        while (parser_index < parser_str.size())
        {
            switch (parser_str[parser_index])
            {
                case 'i': //int
                case 'I': //unsigned int
                case 'f': //float
                    data_index += 4;
                    break;
                case 'h': //short
                case 'H': //unsigned short
                    data_index += 2;
                    break;
                case 'b': //signed char
                case 'B': //unsigned char
                case '?': //bool
                case 's': //char[]
                case 'x': //pad byte
                    //No swaps needed on single bytes
                    data_index += 1;
                    break;
                default:
                    //should not get here
                    printf("should not get here: %c\n", parser_str[parser_index]);
                    break;
            }
            parser_index += 1;
        }
        return data_index;
    }
};
