/********************************************//**
 * Copyright 1998-2014, yost Corporation.
 * This Source Code Form is subject to the terms of the yost 3-Space Open
 * Source License available online at:
 * http://www.yosttechnology.com/yost-3-space-open-source-license
 ***********************************************/

#include "serial_enumerator.hpp"

#include <algorithm>
#include <stdlib.h>


SerialEnumeratorPrivate::SerialEnumeratorPrivate()
{
}


SerialEnumeratorPrivate::~SerialEnumeratorPrivate()
{
}


SerialEnumerator::SerialEnumerator() : sep_ptr(new SerialEnumeratorPrivate())
{
}


SerialEnumerator::~SerialEnumerator()
{
    delete sep_ptr;
}


std::vector<BasicPortInfo> SerialEnumerator::getPorts()
{
    return sep_ptr->getPorts_sys();
}


int8_t startsWith(std::string arg, std::string prefix, int8_t case_insensitive)
{
    if (arg.size() < prefix.size())
    {
        return 0;
    }
    if (case_insensitive)
    {
        std::transform(arg.begin(), arg.begin() + prefix.size(), arg.begin(), ::toupper);
        std::transform(prefix.begin(), prefix.end(), prefix.begin(), ::toupper);
        return (arg.substr(0, prefix.size()) == prefix);
    }
    return (arg.substr(0, prefix.size()) == prefix);
}


int8_t lessThan(const BasicPortInfo &port_1, const BasicPortInfo &port_2)
{
#if defined(_WIN32)
    if (startsWith(port_1.port_name, "COM") && startsWith(port_2.port_name, "COM"))
    {
        return atoi(port_1.port_name.substr(3).c_str()) < atoi(port_2.port_name.substr(3).c_str());
    }
#elif defined(__linux__)
    std::string p_name_1 = port_1.port_name.substr(5);
    std::string p_name_2 = port_2.port_name.substr(5);
    if (startsWith(p_name_1, "tty") && startsWith(p_name_2, "rfcomm"))
    {
        return true;
    }
    else if (startsWith(p_name_1, "rfcomm") && startsWith(p_name_2, "tty"))
    {
        return false;
    }
    else if (startsWith(p_name_1, "rfcomm") && startsWith(p_name_2, "rfcomm"))
    {
        return atoi(p_name_1.substr(6).c_str()) < atoi(p_name_2.substr(6).c_str());
    }
    else if (startsWith(p_name_1, "ttyS") && startsWith(p_name_2, "ttyACM"))
    {
        return false;
    }
    else if (startsWith(p_name_1, "ttyS") && startsWith(p_name_2, "ttyUSB"))
    {
        return true;
    }
    else if (startsWith(p_name_1, "ttyS") && startsWith(p_name_2, "ttyS"))
    {
        return atoi(p_name_1.substr(4).c_str()) < atoi(p_name_2.substr(4).c_str());
    }
    else if (startsWith(p_name_1, "ttyACM") && (startsWith(p_name_2, "ttyS") || startsWith(p_name_2, "ttyUSB")))
    {
        return true;
    }
    else if (startsWith(p_name_1, "ttyACM") && startsWith(p_name_2, "ttyACM"))
    {
        return atoi(p_name_1.substr(6).c_str()) < atoi(p_name_2.substr(6).c_str());
    }
    else if (startsWith(p_name_1, "ttyUSB") && (startsWith(p_name_2, "ttyS") || startsWith(p_name_2, "ttyACM"))) {
        return false;
    }
    else if (startsWith(p_name_1, "ttyUSB") && startsWith(p_name_2, "ttyUSB")) {
        return atoi(p_name_1.substr(6).c_str()) < atoi(p_name_2.substr(6).c_str());
    }
#elif defined(__APPLE__)    
    std::string p_name_1 = port_1.port_name.substr(9);
    std::string p_name_2 = port_2.port_name.substr(9);
    if (startsWith(p_name_1, "usbmodem") && startsWith(p_name_2, "usbserial")) {
        return true;
    }
    else if (startsWith(p_name_1, "usbmodem") && startsWith(p_name_2, "yost")) {
        return true;
    }
    else if (startsWith(p_name_1, "usbmodem") && startsWith(p_name_2, "YEI")) {
        return true;
    }
    else if (startsWith(p_name_1, "usbmodem") && startsWith(p_name_2, "usbmodem")) {
        return atoi(p_name_1.substr(10).c_str()) < atoi(p_name_2.substr(10).c_str());
    }
    else if (startsWith(p_name_1, "usbserial") && startsWith(p_name_2, "usbmodem")) {
        return false;
    }
    else if (startsWith(p_name_1, "usbserial") && startsWith(p_name_2, "yost")) {
        return true;
    }
    else if (startsWith(p_name_1, "usbserial") && startsWith(p_name_2, "YEI")) {
        return true;
    }
    else if (startsWith(p_name_1, "usbserial") && startsWith(p_name_2, "usbserial")) {
        return atoi(p_name_1.substr(10, 1).c_str()) < atoi(p_name_2.substr(10, 1).c_str());
    }
    else if (startsWith(p_name_1, "yost") && startsWith(p_name_2, "usbmodem")) {
        return false;
    }
    else if (startsWith(p_name_1, "yost") && startsWith(p_name_2, "usbserial")) {
        return false;
    }
    else if (startsWith(p_name_1, "yost") && startsWith(p_name_2, "yost")) {
        return strtoul(p_name_1.substr(13, 4).c_str(), NULL, 16) < strtoul(p_name_2.substr(13, 4).c_str(), NULL, 16);
    }
    else if (startsWith(p_name_1, "YEI") && startsWith(p_name_2, "usbmodem")) {
        return false;
    }
    else if (startsWith(p_name_1, "YEI") && startsWith(p_name_2, "usbserial")) {
        return false;
    }
    else if (startsWith(p_name_1, "YEI") && startsWith(p_name_2, "YEI")) {
        return strtoul(p_name_1.substr(13, 4).c_str(), NULL, 16) < strtoul(p_name_2.substr(13, 4).c_str(), NULL, 16);
    }
#endif

    return port_1.port_name < port_2.port_name;
}
