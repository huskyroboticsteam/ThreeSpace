/********************************************//**
 * Copyright 1998-2014, yost Corporation
 * This Source Code Form is subject to the terms of the yost 3-Space Open
 * Source License available online at:
 * http://www.yosttechnology.com/yost-3-space-open-source-license
 ***********************************************/
#ifndef _SERIAL_ENUMERATOR_H_
#define _SERIAL_ENUMERATOR_H_

#include <string>
#include <vector>
#include <stdint.h>


/********************************************//**
 * An enum expressing the different types of product ids.
 ***********************************************/
enum yost_PID
{
    BTL_PID = 0x1000,   /**< Bootloader */
    USB_PID = 0x1010,   /**< 3-Space USB, Watertight USB, Mico USB */
    DNG_PID = 0x1020,   /**< 3-Space Dongle */
    WL_PID = 0x1030,    /**< 3-Space Wireless */
    EM_PID = 0x1040,    /**< 3-Space Embedded */
    DL_PID = 0x1050,    /**< 3-Space Data-logging */
    BT_PID = 0x1060,    /**< 3-Space Bluetooth */
    LE_PID = 0x1080,    /**< 3-Space LE */
    LX_PID = 0x1090,    /**< 3-Space LX */
	MBT_PID = 0x1100,   /**< 3-Space MiniBT */
    BS_PID = 0x3000,    /**< Prio Base Station */
    HUB_PID = 0x3010,   /**< Prio Hub */
};


/********************************************//**
 * Structure containing basic port information.
 ***********************************************/
struct BasicPortInfo
{
    std::string port_name;          /**< Port name */
	uint8_t port_type;              /**< Port Type */
	uint16_t vendor_id;             /**< Vendor ID */
    uint16_t product_id;            /**< Product ID */	
};


class SerialEnumeratorPrivate
{
    public:
        SerialEnumeratorPrivate();
        ~SerialEnumeratorPrivate();

        /********************************************//**
         * Gets a list of ports that are or could be yost devices based on type of operating system.
         ***********************************************/
        std::vector<BasicPortInfo> getPorts_sys();
};


/********************************************//**
 * Provides list of ports that are or couble be yost devices available on the system.
 ***********************************************/
class SerialEnumerator
{
    public:
        SerialEnumerator();
        ~SerialEnumerator();

        /********************************************//**
         * Gets a list of ports that are or could be yost devices.
         ***********************************************/
        std::vector<BasicPortInfo> getPorts();

    private:
        SerialEnumeratorPrivate *sep_ptr;
};


/********************************************//**
 * Checks if a string starts with a given prefix.
 ***********************************************/
int8_t startsWith(std::string arg, std::string prefix, int8_t case_insensitive=false);


/********************************************//**
 * Checks if the port name is less than the other.
 ***********************************************/
int8_t lessThan(const BasicPortInfo &port_1, const BasicPortInfo &port_2);


#endif //_SERIAL_ENUMERATOR_H_
