//(c)2009 by Texas Instruments Incorporated, All Rights Reserved.
/*----------------------------------------------------------------------------+
 |                                                                             |
 |                              Texas Instruments                              |
 |                                                                             |
 |                          MSP430 USB-Example (HID Driver)                    |
 |                                                                             |
 +-----------------------------------------------------------------------------+
 |  Source: UsbHidReq.c, File Version 1.00 2009/12/03                          |
 |  Author: RSTO                                                               |
 |                                                                             |
 |  WHO          WHEN         WHAT                                             |
 |  ---          ----------   ------------------------------------------------ |
 |  RSTO         2009/03/03   born                                             |
 |  MSP/Biju     2009/10/21   Changes for composite support                    |
 +----------------------------------------------------------------------------*/

#include "../USB_Common/device.h"
#include "../USB_Common/types.h"                            //Basic Type declarations
#include "../USB_Common/defMSP430USB.h"
#include "../USB_Common/usb.h"                              //USB-specific Data Structures
#include "UsbHid.h"
#include "UsbHidReq.h"
#include <descriptors.h>

#ifdef _HID_
#include "usb_hid.h"

VOID usbClearOEP0ByteCount (VOID);
VOID usbSendDataPacketOnEP0 (PBYTE pbBuffer);
VOID usbReceiveDataPacketOnEP0 (PBYTE pbBuffer);

extern const WORD report_desc_size[HID_NUM_INTERFACES];
extern const PBYTE report_desc[HID_NUM_INTERFACES];         //KLQ
extern BYTE hidProtocol[];
extern BYTE hidIdleRate[];
extern WORD wUsbHidEventMask;

//Local Macros
#define INTERFACE_OFFSET(X)   (X - HID0_REPORT_INTERFACE)   //Get the HID offset

BYTE usbGetHidDescriptor (VOID)
{
    usbClearOEP0ByteCount();
    wBytesRemainingOnIEP0 = 9;
    usbSendDataPacketOnEP0((PBYTE)&abromConfigurationDescriptorGroup.stHid[
            INTERFACE_OFFSET(tSetupPacket.wIndex)].blength_hid_descriptor);

    return (FALSE);
}

BYTE usbGetReportDescriptor (VOID)
{
    wBytesRemainingOnIEP0 =
        report_desc_size[INTERFACE_OFFSET(tSetupPacket.wIndex)];
    usbSendDataPacketOnEP0(report_desc[INTERFACE_OFFSET(tSetupPacket.wIndex)]);

    return (FALSE);
}

//----------------------------------------------------------------------------

BYTE usbSetReport (VOID)
{
    BYTE *buffer;

    //tSetupPacket.wValue = USB_REQ_HID_FEATURE or USB_REQ_HID_INPUT
    buffer = Handler_SetReport(tSetupPacket.wValue,
        tSetupPacket.wLength,
        tSetupPacket.wIndex);

    //What if buffer is NULL?
    if (buffer == 0){
        usbReceiveDataPacketOnEP0((PBYTE)&abUsbRequestIncomingData);
    } else {
        usbReceiveDataPacketOnEP0((PBYTE)buffer);   //receive data over EP0 from Host
    }

    return (FALSE);
}

//----------------------------------------------------------------------------

BYTE usbGetReport (VOID)
{
    BYTE *buffer;

    //tSetupPacket.wValue = USB_REQ_HID_FEATURE or USB_REQ_HID_INPUT
    buffer = Handler_GetReport((BYTE)tSetupPacket.wValue,
        tSetupPacket.wLength,
        tSetupPacket.wIndex);
    if (buffer != 0){
        usbSendDataPacketOnEP0((PBYTE)buffer);
    }

    return (FALSE);
}

BYTE usbSetProtocol (VOID)
{
    BYTE bWakeUp = FALSE;

    hidProtocol[INTERFACE_OFFSET(tSetupPacket.wIndex)] =
        (BYTE)tSetupPacket.wValue;
    //tSetupPacket.wValue = USB_REQ_HID_BOOT_PROTOCOL or USB_REQ_HID_REPORT_PROTOCOL
    bWakeUp = USBHID_handleBootProtocol((BYTE)tSetupPacket.wValue,
        tSetupPacket.wIndex);
    usbSendZeroLengthPacketOnIEP0();

    return (bWakeUp);
}

//----------------------------------------------------------------------------

BYTE usbGetProtocol (VOID)
{
    usbSendDataPacketOnEP0(&hidProtocol[INTERFACE_OFFSET(tSetupPacket.wIndex)]);

    return (FALSE);
}

//----------------------------------------------------------------------------

BYTE usbSetIdle (VOID)
{
    if (hidProtocol[INTERFACE_OFFSET(tSetupPacket.wIndex)] ==
        USB_REQ_HID_BOOT_PROTOCOL){
        hidIdleRate[INTERFACE_OFFSET(tSetupPacket.wIndex)] =
            tSetupPacket.wValue >> 8;
        usbSendZeroLengthPacketOnIEP0();
    } else {
        usbInvalidRequest();
    }

    return (FALSE);
}

//----------------------------------------------------------------------------

BYTE usbGetIdle (VOID)
{
    if (hidProtocol[INTERFACE_OFFSET(tSetupPacket.wIndex)] ==
        USB_REQ_HID_BOOT_PROTOCOL){
        usbSendDataPacketOnEP0(&hidIdleRate[INTERFACE_OFFSET(tSetupPacket.
                                                wIndex)]);
    } else {
        usbInvalidRequest();
    }

    return (FALSE);
}

BYTE setReportBuffer[64];

/*
 * This event indicates that a Set_Report request was received from the host
 */
BYTE *Handler_SetReport (WORD reportType_reportId,
    WORD dataLength,
    BYTE intfnum)
{
    return (&setReportBuffer[0]);
}

/*
 * This event indicates that a Set_Report request data has been received
 */
BYTE Handler_SetReportDataAvailable (BYTE intfnum)
{
    return usbhid_set_report_completed();
}

/*
 * This event indicates that a Get_Report request was received from the host
 */
BYTE *Handler_GetReport (WORD reportType_reportId,
    WORD requestedLength,
    BYTE intfnum)
{
    return (&setReportBuffer[0]);
}

//----------------------------------------------------------------------------

#endif  //_HID_
/*----------------------------------------------------------------------------+
 | End of source file                                                          |
 +----------------------------------------------------------------------------*/
/*------------------------ Nothing Below This Line --------------------------*/
