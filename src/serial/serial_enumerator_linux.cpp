/********************************************//**
 * Copyright 1998-2014, yost Corporation.
 * This Source Code Form is subject to the terms of the yost 3-Space Open
 * Source License available online at:
 * http://www.yosttechnology.com/yost-3-space-open-source-license
 ***********************************************/

#ifdef __linux__
#include "serial_enumerator.hpp"

extern "C" {
    #include <libudev.h>
}

#include <algorithm>
#include <stdlib.h>


/********************************************//**
 * A list of port name prefixes to check.
 ***********************************************/
const char *g_device_port_name_prefix[] =
{
    // Ports (COM & LPT ports), Class = Ports
    "/dev/ttyS",
    "/dev/ttyACM",
    "/dev/ttyUSB",
    // Bluetooth Devices, Class = Bluetooth
    "/dev/rfcomm"
};


/********************************************//**
 * Gets detail information about the device.
 ***********************************************/
static void portInfoFromDevice(BasicPortInfo *port_info, struct udev_device *dev)
{
    const char *v_id = udev_device_get_property_value(dev, "ID_VENDOR_ID");
    const char *p_id = udev_device_get_property_value(dev, "ID_MODEL_ID");

    if (v_id != NULL)
    {
        port_info->vendor_id = strtoul(v_id, NULL, 16);
    }
    if (p_id != NULL)
    {
        port_info->product_id = strtoul(p_id, NULL, 16);
    }
    port_info->port_name = std::string(udev_device_get_devnode(dev));
}


std::vector<BasicPortInfo> SerialEnumeratorPrivate::getPorts_sys()
{
    std::vector<BasicPortInfo> info_list;
    struct udev *ud = udev_new();
    if (!ud)
    {
        return info_list;
    }

    const int8_t count = sizeof(g_device_port_name_prefix) / sizeof(g_device_port_name_prefix[0]);

    // Search for appropriate udev devices
    struct udev_enumerate *enumerate = udev_enumerate_new(ud);
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);
    struct udev_list_entry *list = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry *entry;
    udev_list_entry_foreach(entry, list)
    {
        const char *path;
        struct udev_device *dev;

        // Grab the actual udev device here
        path = udev_list_entry_get_name(entry);
        dev = udev_device_new_from_syspath(ud, path);

        BasicPortInfo info;
        info.vendor_id = info.product_id = 0;
        portInfoFromDevice(&info, dev);
        for (int8_t i=0; i<count; ++i)
        {
            if (startsWith(info.port_name, g_device_port_name_prefix[i]))
            {
                info_list.push_back(info);
                break;
            }
        }

        // Done with this device
        udev_device_unref(dev);
    }

    // Done with the list and this udev
    udev_enumerate_unref(enumerate);
    udev_unref(ud);

    std::sort(info_list.begin(), info_list.end(), lessThan);

    return info_list;
}


#endif // __linux__
