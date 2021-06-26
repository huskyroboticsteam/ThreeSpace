/********************************************//**
 * Copyright 1998-2014, yost Corporation.
 * This Source Code Form is subject to the terms of the yost 3-Space Open
 * Source License available online at:
 * http://www.yosttechnology.com/yost-3-space-open-source-license
 ***********************************************/

#ifdef __APPLE__
#include "serial_enumerator.hpp"

#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <CoreFoundation/CFNumber.h>
#include <sys/param.h>

#include <algorithm>
#include <stdlib.h>

/********************************************//**
 * A list of port name prefixes to check.
 ***********************************************/
const char *g_device_port_name_prefix[] =
{
    // Ports (COM & LPT ports), Class = Ports
    "/dev/ttys",
    "/dev/tty.usbmodem",
    "/dev/tty.usbserial-",
    // Bluetooth Devices, Class = Bluetooth
    "/dev/tty.YEI_3SpaceBT",
    "/dev/tty.yost_3SpaceBT",
	"/dev/tty.YostLabs-MBT-HM-13"
};

/********************************************//**
 * Gets detail information about the device.
 ***********************************************/
static void getDeviceDetailsOSX(io_object_t service, BasicPortInfo *port_info)
{
    CFTypeRef serial_name_as_cfstring = NULL;
    CFTypeRef vendor_id_as_cfnumber = NULL;
    CFTypeRef product_id_as_cfnumber = NULL;

    // Check the name of the modem's tty device
    serial_name_as_cfstring = IORegistryEntryCreateCFProperty(service, CFSTR(kIOTTYDeviceKey), kCFAllocatorDefault, 0);    

    // Wander up the hierarchy until we find the level that can give us the vendor/product IDs
    io_registry_entry_t parent;
    kern_return_t kern_result = IORegistryEntryGetParentEntry(service, kIOServicePlane, &parent);
    while (kern_result == KERN_SUCCESS && !vendor_id_as_cfnumber && !product_id_as_cfnumber)
    {
        vendor_id_as_cfnumber = IORegistryEntrySearchCFProperty(parent, kIOServicePlane, CFSTR(kUSBVendorID), kCFAllocatorDefault, 0);

        product_id_as_cfnumber = IORegistryEntrySearchCFProperty(parent, kIOServicePlane, CFSTR(kUSBProductID), kCFAllocatorDefault, 0);

        io_registry_entry_t old_parent = parent;
        kern_result = IORegistryEntryGetParentEntry(parent, kIOServicePlane, &parent);
        IOObjectRelease(old_parent);
    }

    if (serial_name_as_cfstring)
    {
        char path[MAXPATHLEN] = {"/dev/tty."};
        if (CFStringGetCString((CFStringRef)serial_name_as_cfstring, &path[9], PATH_MAX, kCFStringEncodingUTF8))
        {            
            port_info->port_name = path;
        }
        CFRelease(serial_name_as_cfstring);
    }

    if (vendor_id_as_cfnumber)
    {
        SInt16 v_id;
        if (CFNumberGetValue((CFNumberRef)vendor_id_as_cfnumber, kCFNumberSInt16Type, &v_id))
        {
            port_info->vendor_id = v_id;
        }
        CFRelease(vendor_id_as_cfnumber);
    }

    if (product_id_as_cfnumber)
    {
        SInt16 p_id;
        if (CFNumberGetValue((CFNumberRef)product_id_as_cfnumber, kCFNumberSInt16Type, &p_id))
        {
            port_info->product_id = p_id;
        }
        CFRelease(product_id_as_cfnumber);
    }

    IOObjectRelease(service);
}


/********************************************//**
 * Gets detail information about the device.
 ***********************************************/
static void iterateDevicesOSX(io_object_t service, std::vector<BasicPortInfo> &info_list)
{
    const int8_t count = sizeof(g_device_port_name_prefix) / sizeof(g_device_port_name_prefix[0]);

    // Iterate through all modems found.
    io_object_t usb_service;
    while ((usb_service = IOIteratorNext(service)))
    {
        BasicPortInfo info;
        info.vendor_id = 0;
        info.product_id = 0;
        getDeviceDetailsOSX(usb_service, &info);

        for (int8_t i = 0; i < count; ++i)
        {
            if (startsWith(info.port_name, g_device_port_name_prefix[i]))
            {
                switch (i)
                {
                    case 0:
                    case 1:
                    case 2:
                        break;
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                        info.vendor_id = 0x2476;
                        info.product_id = 0x1060;
                        break;
                }
                info_list.push_back(info);
                break;
            }
        }
    }
}


std::vector<BasicPortInfo> SerialEnumeratorPrivate::getPorts_sys()
{
    std::vector<BasicPortInfo> info_list;
    io_iterator_t serial_port_iterator = 0;
    CFMutableDictionaryRef matching_dictionary;

    // Try to get any serialbsd devices
    if (!(matching_dictionary = IOServiceMatching(kIOSerialBSDServiceValue)))
    {
        return info_list;
    }
    CFDictionaryAddValue(matching_dictionary, CFSTR(kIOSerialBSDTypeKey), CFSTR(kIOSerialBSDAllTypes));

    // Create the iterator with all the matching devices
    if (IOServiceGetMatchingServices(kIOMasterPortDefault, matching_dictionary, &serial_port_iterator) != KERN_SUCCESS)
    {
        return info_list;
    }

    // Iterate through the devices matched
    iterateDevicesOSX(serial_port_iterator, info_list);

    IOObjectRelease(serial_port_iterator);

    std::sort(info_list.begin(), info_list.end(), lessThan);

    return info_list;
}


#endif // __APPLE__
