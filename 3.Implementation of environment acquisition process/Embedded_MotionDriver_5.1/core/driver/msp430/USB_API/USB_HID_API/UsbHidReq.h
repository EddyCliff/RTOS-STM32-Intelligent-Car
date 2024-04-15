//(c)2009 by Texas Instruments Incorporated, All Rights Reserved.
/*----------------------------------------------------------------------------+
 |                                                                             |
 |                              Texas Instruments                              |
 |                                                                             |
 |                          MSP430 USB-Example (HID Driver)                    |
 |                                                                             |
 +-----------------------------------------------------------------------------+
 |  Source: UsbHidReq.h, File Version 1.00 2009/12/03                          |
 |  Author: RSTO                                                               |
 |                                                                             |
 |  WHO          WHEN         WHAT                                             |
 |  ---          ----------   ------------------------------------------------ |
 |  RSTO         2009/03/03   born                                             |
 |  MSP,Biju     2009/12/03   file versioning started                          |
 +----------------------------------------------------------------------------*/

#ifndef _UsbHidReq_H_
#define _UsbHidReq_H_

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * Return Hid descriptor to host over control endpoint
 */
BYTE usbGetHidDescriptor(VOID);
/**
 * Return HID report descriptor to host over control endpoint
 */
BYTE usbGetReportDescriptor(VOID);
/**
 * Receive Set_Report from host over control endpoint
 */
BYTE usbSetReport(VOID);
/**
 * Process Get_Report request from host over control endpoint
 */
BYTE usbGetReport(VOID);
/**
 * Receive Set_Idle from host over control endpoint
 */
BYTE usbSetIdle(VOID);
/**
 * Process Get_Idle request from host over control endpoint
 */
BYTE usbGetIdle(VOID);
/**
 * Receive Set_Protocol from host over control endpoint
 */
BYTE usbSetProtocol(VOID);
/**
 * Process Get_Protocol request from host over control endpoint
 */
BYTE usbGetProtocol(VOID);

/*
 * This event indicates that a Set_Report request was received from the host
 * The application needs to supply a buffer to retrieve the report data that will be sent
 * as part of this request. This handler is passed the wValue from the setup packet
 * which contains the reportType (upper byte) and reportId(lower byte), the length of data
 * phase as well as the interface number.
 */
BYTE *Handler_SetReport (WORD reportType_reportId,
    WORD requestedLength,
    BYTE intfnum);
/*
 * This event indicates that data as part of Set_Report request was received from the host
 * Tha application can return TRUE to wake up the CPU. If the application supplied a buffer
 * as part of USBHID_handleSetReport, then this buffer will contain the Set Report data.
 */
BYTE Handler_SetReportDataAvailable (BYTE intfnum);
/*
 * This event indicates that a Get_Report request was received from the host
 * The application can supply a buffer of data that will be sent to the host.
 * This handler is passed the wValue from the setup packet which contains the
 * reportType(high byte) and reportId(low byte), the requested length as well
 * as the interface number.
 */
BYTE *Handler_GetReport (WORD reportType_reportId,
    WORD requestedLength,
    BYTE intfnum);

#ifdef __cplusplus
}
#endif
#endif  //_UsbHidReq_H_
