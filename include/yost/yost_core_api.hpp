/********************************************//**
 * Copyright 1998-2014, Yost Labs Corporation
 * This Source Code Form is subject to the terms of the YOST 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#ifndef _YOST_CORE_API_H_
#define _YOST_CORE_API_H_
#include <stdint.h>
#include <string>

//useful types
typedef uint8_t U8;
typedef uint16_t U16;
typedef uint32_t U32;
typedef uint64_t U64;

#define _swap(a,b,c) c=a;a=b;b=c

namespace yost
{
    typedef struct Command
    {
        uint8_t command;
        std::string description_str;
        uint16_t rtn_data_len;
        std::string rtn_data_detail;
        uint16_t in_data_len;
        std::string in_data_detail;
        uint32_t compatibility_mask;
        uint8_t fw_compatibility;
    }Command;

    void swap16(U8* bytes);

    void swap32(U8* bytes);

    void swap64(U8* bytes);

    float get_time();

    void sleep_ms(uint32_t time_ms);

    bool startsWith(std::string starts, std::string with);

    uint8_t _compareFirmwareVersion(std::string version_string1, std::string version_string2);

    uint8_t _createChecksum(const uint8_t* command_bytes, const uint8_t num_bytes);

    uint8_t _parseData(uint8_t* data, uint32_t len, std::string parser_str);

    uint16_t _calculateParseDataSize(std::string parser_str);
};
#endif // _YOST_CORE_API_H_
