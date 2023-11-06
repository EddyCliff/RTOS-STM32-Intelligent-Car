// (c)2010 by Texas Instruments Incorporated, All Rights Reserved.
/*-----------------------------------------------------------------------------+
|                                                                              |
|                              Texas Instruments                               |
|                                                                              |
|    This is an automatically generated script by MSP430 USB Descriptor Tool   |
|                                                                              |
|    Descriptor Tool Version: 3.0.10                                            |
|    Date: 2012/04/02 18:06:37                                                 |
|                                                                              |
|                                Descriptor.c                                  |
|-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------+
| Include files                                                                |
|-----------------------------------------------------------------------------*/
#include <USB_API/USB_Common/device.h>
#include <USB_API/USB_Common/types.h>                // Basic Type declarations
#include <USB_API/USB_Common/defMSP430USB.h>
#include <USB_API/USB_Common/usb.h>              // USB-specific Data Structures
#include "descriptors.h"
#include <USB_API/USB_CDC_API/UsbCdc.h>
#include <USB_API/USB_HID_API/UsbHidReq.h>

/*-----------------------------------------------------------------------------+
| Device Descriptor                                                            |
|-----------------------------------------------------------------------------*/
BYTE const abromDeviceDescriptor[SIZEOF_DEVICE_DESCRIPTOR] = {
    SIZEOF_DEVICE_DESCRIPTOR,               // Length of this descriptor
    DESC_TYPE_DEVICE,                       // Type code of this descriptor
    0x00, 0x02,                             // Release of USB spec
    0x02,                                   // Device's base class code
    0x00,                                   // Device's sub class code
    0x00,                                   // Device's protocol type code
    EP0_PACKET_SIZE,                        // End point 0's packet size
    USB_VID&0xFF, USB_VID>>8,               // Vendor ID for device, TI=0x0451
                                            // You can order your own VID at www.usb.org
    USB_PID&0xFF, USB_PID>>8,               // Product ID for device,
                                            // this ID is to only with this example
    VER_FW_L, VER_FW_H,                     // Revision level of device
    1,                                      // Index of manufacturer name string desc
    2,                                      // Index of product name string desc
    USB_STR_INDEX_SERNUM,                   // Index of serial number string desc
    1                                       //  Number of configurations supported
};

/*-----------------------------------------------------------------------------+
| Configuration Descriptor                                                     |
|-----------------------------------------------------------------------------*/
const struct abromConfigurationDescriptorGroup abromConfigurationDescriptorGroup=
{
    /* Generic part */
    {
        // CONFIGURATION DESCRIPTOR (9 bytes)
        SIZEOF_CONFIG_DESCRIPTOR,                          // bLength
        DESC_TYPE_CONFIG,                                  // bDescriptorType
        DESCRIPTOR_TOTAL_LENGTH, 0x00,                     // wTotalLength
        USB_NUM_INTERFACES,                                // bNumInterfaces
        USB_CONFIG_VALUE,                                  // bConfigurationvalue
        CONFIG_STRING_INDEX,                               // iConfiguration Description offset
        USB_SUPPORT_SELF_POWERED | USB_SUPPORT_REM_WAKE,   // bmAttributes, bus power, remote wakeup
        USB_MAX_POWER                                      // Max. Power Consumption
    },

    /******************************************************* start of CDC*************************************/

    {
        /* start CDC[0] */
        {

            //INTERFACE DESCRIPTOR (9 bytes)
            0x09,                              // bLength: Interface Descriptor size
            DESC_TYPE_INTERFACE,               // bDescriptorType: Interface
            CDC0_COMM_INTERFACE,               // bInterfaceNumber
            0x00,                              // bAlternateSetting: Alternate setting
            0x01,                              // bNumEndpoints: Three endpoints used
            0x02,                              // bInterfaceClass: Communication Interface Class
            0x02,                              // bInterfaceSubClass: Abstract Control Model
            0x01,                              // bInterfaceProtocol: Common AT commands
            INTF_STRING_INDEX + 0,             // iInterface:

            //Header Functional Descriptor
            0x05,                               // bLength: Endpoint Descriptor size
            0x24,                               // bDescriptorType: CS_INTERFACE
            0x00,                               // bDescriptorSubtype: Header Func Desc
            0x10,                               // bcdCDC: spec release number
            0x01,

            //Call Managment Functional Descriptor
            0x05,                               // bFunctionLength
            0x24,                               // bDescriptorType: CS_INTERFACE
            0x01,                               // bDescriptorSubtype: Call Management Func Desc
            0x00,                               // bmCapabilities: D0+D1
            CDC0_DATA_INTERFACE,                // bDataInterface: 0

            //ACM Functional Descriptor
            0x04,                               // bFunctionLength
            0x24,                               // bDescriptorType: CS_INTERFACE
            0x02,                               // bDescriptorSubtype: Abstract Control Management desc
            0x02,                               // bmCapabilities

            // Union Functional Descriptor
            0x05,                               // Size, in bytes
            0x24,                               // bDescriptorType: CS_INTERFACE
            0x06,                               // bDescriptorSubtype: Union Functional Desc
            CDC0_COMM_INTERFACE,                // bMasterInterface -- the controlling intf for the union
            CDC0_DATA_INTERFACE,                // bSlaveInterface -- the controlled intf for the union

            //EndPoint Descriptor for Interrupt endpoint
            SIZEOF_ENDPOINT_DESCRIPTOR,         // bLength: Endpoint Descriptor size
            DESC_TYPE_ENDPOINT,                 // bDescriptorType: Endpoint
            CDC0_INTEP_ADDR,                    // bEndpointAddress: (IN2)
            EP_DESC_ATTR_TYPE_INT,              // bmAttributes: Interrupt
            0x40, 0x00,                         // wMaxPacketSize, 64 bytes
            0xFF,                               // bInterval

            //DATA INTERFACE DESCRIPTOR (9 bytes)
            0x09,                               // bLength: Interface Descriptor size
            DESC_TYPE_INTERFACE,                // bDescriptorType: Interface
            CDC0_DATA_INTERFACE,                // bInterfaceNumber
            0x00,                               // bAlternateSetting: Alternate setting
            0x02,                               // bNumEndpoints: Three endpoints used
            0x0A,                               // bInterfaceClass: Data Interface Class
            0x00,                               // bInterfaceSubClass:
            0x00,                               // bInterfaceProtocol: No class specific protocol required
            0x00,                               // iInterface:

            //EndPoint Descriptor for Output endpoint
            SIZEOF_ENDPOINT_DESCRIPTOR,         // bLength: Endpoint Descriptor size
            DESC_TYPE_ENDPOINT,                 // bDescriptorType: Endpoint
            CDC0_OUTEP_ADDR,                    // bEndpointAddress: (OUT3)
            EP_DESC_ATTR_TYPE_BULK,             // bmAttributes: Bulk
            0x40, 0x00,                         // wMaxPacketSize, 64 bytes
            0xFF,                               // bInterval: ignored for Bulk transfer

            //EndPoint Descriptor for Input endpoint
            SIZEOF_ENDPOINT_DESCRIPTOR,         // bLength: Endpoint Descriptor size
            DESC_TYPE_ENDPOINT,                 // bDescriptorType: Endpoint
            CDC0_INEP_ADDR,                     // bEndpointAddress: (IN3)
            EP_DESC_ATTR_TYPE_BULK,             // bmAttributes: Bulk
            0x40, 0x00,                         // wMaxPacketSize, 64 bytes
            0xFF                                // bInterval: ignored for bulk transfer
        }

        /* end CDC[0]*/

    }
    /******************************************************* end of CDC**************************************/

};
/*-----------------------------------------------------------------------------+
| String Descriptor                                                            |
|-----------------------------------------------------------------------------*/
BYTE const abromStringDescriptor[] = {

    // String index0, language support
    4,      // Length of language descriptor ID
    3,      // LANGID tag
    0x09, 0x04, // 0x0409 for English

    // String index1, Manufacturer
    22,     // Length of this string descriptor
    3,      // bDescriptorType
    'I',0x00,'n',0x00,'v',0x00,'e',0x00,'n',0x00,'s',0x00,
    'e',0x00,'n',0x00,'s',0x00,'e',0x00,

    // String index2, Product
    40,     // Length of this string descriptor
    3,      // bDescriptorType
    'E',0x00,'m',0x00,'b',0x00,'e',0x00,'d',0x00,'d',0x00,
    'e',0x00,'d',0x00,' ',0x00,'M',0x00,'o',0x00,'t',0x00,
    'i',0x00,'o',0x00,'n',0x00,'A',0x00,'p',0x00,'p',0x00,
    's',0x00,

    // String index3, Serial Number
    4,      // Length of this string descriptor
    3,      // bDescriptorType
    '0',0x00,

    // String index4, Configuration String
    16,     // Length of this string descriptor
    3,      // bDescriptorType
    'M',0x00,'P',0x00,'U',0x00,'6',0x00,'0',0x00,'5',0x00,
    '0',0x00,

    // String index5, Interface String
    52,     // Length of this string descriptor
    3,      // bDescriptorType
    'E',0x00,'m',0x00,'b',0x00,'e',0x00,'d',0x00,'d',0x00,
    'e',0x00,'d',0x00,' ',0x00,'M',0x00,'o',0x00,'t',0x00,
    'i',0x00,'o',0x00,'n',0x00,'A',0x00,'p',0x00,'p',0x00,
    's',0x00,' ',0x00,'-',0x00,' ',0x00,'C',0x00,'D',0x00,
    'C',0x00
};

/**** Populating the endpoint information handle here ****/

const struct tUsbHandle stUsbHandle[]=
{
    {
        CDC0_INEP_ADDR, 
        CDC0_OUTEP_ADDR,
        1,
        CDC_CLASS,
        IEP1_X_BUFFER_ADDRESS,
        IEP1_Y_BUFFER_ADDRESS,
        OEP2_X_BUFFER_ADDRESS,
        OEP2_Y_BUFFER_ADDRESS,
        IEP2_X_BUFFER_ADDRESS,
        IEP2_Y_BUFFER_ADDRESS
    }
};
//-------------DEVICE REQUEST LIST---------------------------------------------

const tDEVICE_REQUEST_COMPARE tUsbRequestList[] = 
{

    //---- CDC 0 Class Requests -----//
    // GET LINE CODING
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
    USB_CDC_GET_LINE_CODING,
    0x00,0x00,                                 // always zero
    CDC0_COMM_INTERFACE,0x00,                 // CDC interface is 0
    0x07,0x00,                                 // Size of Structure (data length)
    0xff,&usbGetLineCoding,

    // SET LINE CODING
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
    USB_CDC_SET_LINE_CODING,
    0x00,0x00,                                 // always zero
    CDC0_COMM_INTERFACE,0x00,                  // CDC interface is 0
    0x07,0x00,                                 // Size of Structure (data length)
    0xff,&usbSetLineCoding,

    // SET CONTROL LINE STATE
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
    USB_CDC_SET_CONTROL_LINE_STATE,
    0xff,0xff,                                 // Contains data
    CDC0_COMM_INTERFACE,0x00,                 // CDC interface is 0
    0x00,0x00,                                 // No further data
    0xcf,&usbSetControlLineState,

    //---- USB Standard Requests -----//
    // clear device feature
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_CLEAR_FEATURE,
    FEATURE_REMOTE_WAKEUP,0x00,         // feature selector
    0x00,0x00,
    0x00,0x00,
    0xff,&usbClearDeviceFeature,

    // clear endpoint feature
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_ENDPOINT,
    USB_REQ_CLEAR_FEATURE,
    FEATURE_ENDPOINT_STALL,0x00,
    0xff,0x00,
    0x00,0x00,
    0xf7,&usbClearEndpointFeature,

    // get configuration
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_GET_CONFIGURATION,
    0x00,0x00, 
    0x00,0x00, 
    0x01,0x00,
    0xff,&usbGetConfiguration,

    // get device descriptor
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_GET_DESCRIPTOR,
    0xff,DESC_TYPE_DEVICE,              // bValueL is index and bValueH is type
    0xff,0xff,
    0xff,0xff,
    0xd0,&usbGetDeviceDescriptor,

    // get configuration descriptor
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_GET_DESCRIPTOR,
    0xff,DESC_TYPE_CONFIG,              // bValueL is index and bValueH is type
    0xff,0xff,
    0xff,0xff,
    0xd0,&usbGetConfigurationDescriptor,

    // get string descriptor
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_GET_DESCRIPTOR,
    0xff,DESC_TYPE_STRING,              // bValueL is index and bValueH is type
    0xff,0xff,
    0xff,0xff,
    0xd0,&usbGetStringDescriptor,

    // get interface
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
    USB_REQ_GET_INTERFACE,
    0x00,0x00,
    0xff,0xff,
    0x01,0x00,
    0xf3,&usbGetInterface,

    // get device status
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_GET_STATUS,
    0x00,0x00,
    0x00,0x00,
    0x02,0x00,
    0xff,&usbGetDeviceStatus, 
    // get interface status
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
    USB_REQ_GET_STATUS,
    0x00,0x00,
    0xff,0x00,
    0x02,0x00,
    0xf7,&usbGetInterfaceStatus,
    //  get endpoint status
    USB_REQ_TYPE_INPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_ENDPOINT,
    USB_REQ_GET_STATUS,
    0x00,0x00,
    0xff,0x00,
    0x02,0x00,
    0xf7,&usbGetEndpointStatus,

    // set address
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_SET_ADDRESS,
    0xff,0x00,
    0x00,0x00,
    0x00,0x00,
    0xdf,&usbSetAddress,

    // set configuration
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_SET_CONFIGURATION,
    0xff,0x00,
    0x00,0x00,
    0x00,0x00,
    0xdf,&usbSetConfiguration,

    // set device feature
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_DEVICE,
    USB_REQ_SET_FEATURE,
    0xff,0x00,                      // feature selector
    0x00,0x00,
    0x00,0x00,
    0xdf,&usbSetDeviceFeature,

    // set endpoint feature
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_ENDPOINT,
    USB_REQ_SET_FEATURE,
    0xff,0x00,                      // feature selector
    0xff,0x00,                      // endpoint number <= 127
    0x00,0x00,
    0xd7,&usbSetEndpointFeature,

    // set interface
    USB_REQ_TYPE_OUTPUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
    USB_REQ_SET_INTERFACE,
    0xff,0x00,                      // feature selector
    0xff,0x00,                      // interface number
    0x00,0x00,
    0xd7,&usbSetInterface,

    // end of usb descriptor -- this one will be matched to any USB request
    // since bCompareMask is 0x00.
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff, 
    0x00,&usbInvalidRequest     // end of list
};

/*-----------------------------------------------------------------------------+
| END OF Descriptor.c FILE                                                     |
|-----------------------------------------------------------------------------*/
