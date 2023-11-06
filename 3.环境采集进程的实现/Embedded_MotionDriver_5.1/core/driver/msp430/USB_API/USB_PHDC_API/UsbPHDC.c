//(c)2009 by Texas Instruments Incorporated, All Rights Reserved.
/*----------------------------------------------------------------------------+
 |                                                                             |
 |                              Texas Instruments                              |
 |                                                                             |
 |                          MSP430 USB-Example (PHDC Driver)                    |
 |                                                                             |
 +-----------------------------------------------------------------------------+
 |  Source: UsbPHDC.c, File Version 1.01 2009/12/03                             |
 |  Author: RSTO                                                               |
 |                                                                             |
 |  WHO          WHEN         WHAT                                             |
 |  ---          ----------   ------------------------------------------------ |
 |  RSTO         2008/09/03   born                                             |
 |  RSTO         2008/09/19   Changed USBPHDC_sendData to send more then 64bytes|
 |  RSTO         2008/12/23   enhancements of PHDC API                          |
 |  RSTO         2008/05/19   updated USBPHDC_intfStatus()                      |
 |  RSTO         2009/05/26   added USBPHDC_bytesInUSBBuffer()                  |
 |  RSTO         2009/05/28   changed USBPHDC_sendData()                        |
 |  RSTO         2009/07/17   updated USBPHDC_bytesInUSBBuffer()                |
 |  RSTO         2009/10/21   move __disable_interrupt() before                |
 |                            checking for suspend                             |
 +----------------------------------------------------------------------------*/
#include <descriptors.h>

#ifdef _PHDC_


#include "../USB_Common/device.h"
#include "../USB_Common/types.h"                //Basic Type declarations
#include "../USB_Common/defMSP430USB.h"
#include "../USB_Common/usb.h"                  //USB-specific Data Structures
#include "..\USB_PHDC_API\UsbPHDC.h"
#include <string.h>

//Local Macros
#define INTFNUM_OFFSET(X)   (X - PHDC0_INTFNUM) //Get the PHDC offset

static struct _PHDCWrite {
    WORD nPHDCBytesToSend;                      //holds counter of bytes to be sent
    WORD nPHDCBytesToSendLeft;                  //holds counter how many bytes is still to be sent
    const BYTE* pUsbBufferToSend;               //holds the buffer with data to be sent
    BYTE bCurrentBufferXY;                      //is 0 if current buffer to write data is X, or 1 if current buffer is Y
    BYTE bZeroPacketSent;                       //= FALSE;
    BYTE last_ByteSend;
} PHDCWriteCtrl[PHDC_NUM_INTERFACES];

static struct _PHDCRead {
    BYTE *pUserBuffer;                          //holds the current position of user's receiving buffer. If NULL- no receiving
                                                //operation started
    BYTE *pCurrentEpPos;                        //current positon to read of received data from curent EP
    WORD nBytesToReceive;                       //holds how many bytes was requested by receiveData() to receive
    WORD nBytesToReceiveLeft;                   //holds how many bytes is still requested by receiveData() to receive
    BYTE * pCT1;                                //holds current EPBCTxx register
    BYTE * pCT2;                                //holds next EPBCTxx register
    BYTE * pEP2;                                //holds addr of the next EP buffer
    BYTE nBytesInEp;                            //how many received bytes still available in current EP
    BYTE bCurrentBufferXY;                      //indicates which buffer is used by host to transmit data via OUT endpoint3
} PHDCReadCtrl[PHDC_NUM_INTERFACES];

extern WORD wUsbEventMask;

//function pointers
extern VOID *(*USB_TX_memcpy)(VOID * dest, const VOID * source, size_t count);
extern VOID *(*USB_RX_memcpy)(VOID * dest, const VOID * source, size_t count);


/*----------------------------------------------------------------------------+
 | Global Variables                                                            |
 +----------------------------------------------------------------------------*/

extern __no_init tEDB __data16 tInputEndPointDescriptorBlock[];
extern __no_init tEDB __data16 tOutputEndPointDescriptorBlock[];


VOID PHDCResetData ()
{
    //indicates which buffer is used by host to transmit data via OUT endpoint3 - X buffer is first
    //PHDCReadCtrl[intfIndex].bCurrentBufferXY = X_BUFFER;

    memset(&PHDCWriteCtrl, 0, sizeof(PHDCWriteCtrl));
    memset(&PHDCReadCtrl, 0, sizeof(PHDCReadCtrl));
}

/*
 * Sends data over interface intfNum, of size size and starting at address data.
 * Returns: kUSBPHDC_sendStarted
 *       kUSBPHDC_sendComplete
 *       kUSBPHDC_intfBusyError
 */
BYTE USBPHDC_sendData (const BYTE* data, WORD size, BYTE intfNum)
{
    BYTE edbIndex;
    unsigned short bGIE;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (size == 0){
        return (kUSBPHDC_generalError);
    }

    bGIE  = (__get_SR_register() & GIE);                                    //save interrupt status
    //atomic operation - disable interrupts
    __disable_interrupt();                                                  //Disable global interrupts

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //data can not be read because of USB suspended
        __bis_SR_register(bGIE);                                            //restore interrupt status
        return (kUSBPHDC_busNotAvailable);
    }

    if (PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSendLeft != 0){
        //the USB still sends previous data, we have to wait
        __bis_SR_register(bGIE);                                            //restore interrupt status
        return (kUSBPHDC_intfBusyError);
    }

    //This function generate the USB interrupt. The data will be sent out from interrupt

    PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSend = size;
    PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSendLeft = size;
    PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].pUsbBufferToSend = data;

    //trigger Endpoint Interrupt - to start send operation
    USBIEPIFG |= 1 << (edbIndex + 1);                                       //IEPIFGx;

    __bis_SR_register(bGIE);                                                //restore interrupt status

    return (kUSBPHDC_sendStarted);
}

//workaround for PHDC windows driver: it doesn't give data to Application if was sent 64 byte
#define EP_MAX_PACKET_SIZE_PHDC      0x40

//this function is used only by USB interrupt
BOOL PHDCToHostFromBuffer (BYTE intfNum)
{
    BYTE byte_count, nTmp2;
    BYTE * pEP1;
    BYTE * pEP2;
    BYTE * pCT1;
    BYTE * pCT2;
    BYTE bWakeUp = FALSE;                                                   //TRUE for wake up after interrupt
    BYTE edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSendLeft == 0){  //do we have somtething to send?
        if (!PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].bZeroPacketSent){       //zero packet was not yet sent
            PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].bZeroPacketSent = TRUE;

            if (PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].last_ByteSend ==
                EP_MAX_PACKET_SIZE_PHDC){
                if (PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY ==
                    X_BUFFER){
                    if (tInputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                        EPBCNT_NAK){
                    tInputEndPointDescriptorBlock[edbIndex].bEPBCTX = 0;
                        PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY
                            = Y_BUFFER;                                     //switch buffer
                    }
                } else   {
                    if (tInputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                        EPBCNT_NAK){
                    tInputEndPointDescriptorBlock[edbIndex].bEPBCTY = 0;
                        PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY
                            = X_BUFFER;                                     //switch buffer
                }
                }
            }

            PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSend = 0;    //nothing to send

            //call event callback function
            if (wUsbEventMask & kUSB_sendCompletedEvent){
                bWakeUp = USBPHDC_handleSendCompleted(intfNum);
            }
        } //if (!bSentZeroPacket)

        return (bWakeUp);
    }

    PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].bZeroPacketSent = FALSE;         //zero packet will be not sent: we have data

    if (PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){
        //this is the active EP buffer
        pEP1 = (BYTE*)stUsbHandle[intfNum].iep_X_Buffer;
        pCT1 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTX;

        //second EP buffer
        pEP2 = (BYTE*)stUsbHandle[intfNum].iep_Y_Buffer;
        pCT2 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTY;
    } else   {
        //this is the active EP buffer
        pEP1 = (BYTE*)stUsbHandle[intfNum].iep_Y_Buffer;
        pCT1 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTY;

        //second EP buffer
        pEP2 = (BYTE*)stUsbHandle[intfNum].iep_X_Buffer;
        pCT2 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTX;
    }

    //how many byte we can send over one endpoint buffer
    byte_count =
        (PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSendLeft >
         EP_MAX_PACKET_SIZE_PHDC) ? EP_MAX_PACKET_SIZE_PHDC : PHDCWriteCtrl[
            INTFNUM_OFFSET(intfNum)].nPHDCBytesToSendLeft;
    nTmp2 = *pCT1;

    if (nTmp2 & EPBCNT_NAK){
        USB_TX_memcpy(pEP1, PHDCWriteCtrl[INTFNUM_OFFSET(
                                              intfNum)].pUsbBufferToSend,
            byte_count);                                                        //copy data into IEP3 X or Y buffer
        *pCT1 = byte_count;                                                     //Set counter for usb In-Transaction
        PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
            (PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY +
             1) & 0x01;                                                         //switch buffer
        PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSendLeft -=
            byte_count;
        PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].pUsbBufferToSend += byte_count;  //move buffer pointer
        PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].last_ByteSend = byte_count;

        //try to send data over second buffer
        nTmp2 = *pCT2;
        if ((PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSendLeft >
             0) &&                                                              //do we have more data to send?
            (nTmp2 & EPBCNT_NAK)){ //if the second buffer is free?
            //how many byte we can send over one endpoint buffer
            byte_count =
                (PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSendLeft >
                 EP_MAX_PACKET_SIZE_PHDC) ? EP_MAX_PACKET_SIZE_PHDC :
                PHDCWriteCtrl[
                    INTFNUM_OFFSET(intfNum)].nPHDCBytesToSendLeft;

            USB_TX_memcpy(pEP2, PHDCWriteCtrl[INTFNUM_OFFSET(
                                                  intfNum)].pUsbBufferToSend,
                byte_count);                                                    //copy data into IEP3 X or Y buffer
            *pCT2 = byte_count;                                                 //Set counter for usb In-Transaction
            PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
                (PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY +
                 1) & 0x01;                                                     //switch buffer
            PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSendLeft -=
                byte_count;
            PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].pUsbBufferToSend +=
                byte_count;                                                     //move buffer pointer
            PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].last_ByteSend = byte_count;
        }
    }
    return (bWakeUp);
}

/*
 * Aborts an active send operation on interface intfNum.
 * Returns the number of bytes that were sent prior to the abort, in size.
 */
BYTE USBPHDC_abortSend (WORD* size, BYTE intfNum)
{
    unsigned short bGIE;

    bGIE  = (__get_SR_register() & GIE);                                        //save interrupt status
    __disable_interrupt();                                                      //disable interrupts - atomic operation

    *size =
        (PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSend -
         PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSendLeft);
    PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSend = 0;
    PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSendLeft = 0;

    __bis_SR_register(bGIE);                                                    //restore interrupt status
    return (kUSB_succeed);
}

//This function copies data from OUT endpoint into user's buffer
//Arguments:
//pEP - pointer to EP to copy from
//pCT - pointer to pCT control reg
//
VOID PHDCCopyUsbToBuff (BYTE* pEP, BYTE* pCT, BYTE intfNum)
{
    BYTE nCount;

    //how many byte we can get from one endpoint buffer
    nCount =
        (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft >
         PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp) ? PHDCReadCtrl[
            INTFNUM_OFFSET(intfNum)].nBytesInEp : PHDCReadCtrl[INTFNUM_OFFSET(
                                                                   intfNum)].
        nBytesToReceiveLeft;

    USB_RX_memcpy(PHDCReadCtrl[INTFNUM_OFFSET(
                                   intfNum)].pUserBuffer, pEP, nCount); //copy data from OEP3 X or Y buffer
    PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft -= nCount;
    PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer += nCount;        //move buffer pointer
    //to read rest of data next time from this place

    if (nCount == PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp){    //all bytes are copied from receive buffer?
        //switch current buffer
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
            (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY + 1) & 0x01;

        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;

        //clear NAK, EP ready to receive data
        *pCT = 0x00;
    } else   {
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp -= nCount;
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos = pEP + nCount;
    }
}

/*
 * Receives data over interface intfNum, of size size, into memory starting at address data.
 * Returns:
 *  kUSBPHDC_receiveStarted  if the receiving process started.
 *  kUSBPHDC_receiveCompleted  all requested date are received.
 *  kUSBPHDC_receiveInProgress  previous receive opereation is in progress. The requested receive operation can be not started.
 *  kUSBPHDC_generalError  error occurred.
 */
BYTE USBPHDC_receiveData (BYTE* data, WORD size, BYTE intfNum)
{
    BYTE nTmp1;
    BYTE edbIndex;
    unsigned short bGIE;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if ((size == 0) ||                                                          //read size is 0
        (data == NULL)){
        return (kUSBPHDC_generalError);
    }

    bGIE  = (__get_SR_register() & GIE);                                        //save interrupt status
    //atomic operation - disable interrupts
    __disable_interrupt();                                                      //Disable global interrupts

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //data can not be read because of USB suspended
        __bis_SR_register(bGIE);                                                //restore interrupt status
        return (kUSBPHDC_busNotAvailable);
    }

    if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL){             //receive process already started
        __bis_SR_register(bGIE);                                                //restore interrupt status
        return (kUSBPHDC_intfBusyError);
    }

    PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive = size;               //bytes to receive
    PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft = size;           //left bytes to receive
    PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = data;                   //set user receive buffer

    //read rest of data from buffer, if any4
    if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > 0){
        //copy data from pEP-endpoint into User's buffer
        PHDCCopyUsbToBuff(PHDCReadCtrl[INTFNUM_OFFSET(
                                           intfNum)].pCurrentEpPos,
            PHDCReadCtrl[INTFNUM_OFFSET(
                             intfNum)
            ].pCT1, intfNum);

        if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){    //the Receive opereation is completed
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;           //no more receiving pending
            if (wUsbEventMask & kUSB_receiveCompletedEvent){
                USBPHDC_handleReceiveCompleted(intfNum);                        //call event handler in interrupt context
            }
            __bis_SR_register(bGIE);                                            //restore interrupt status
            return (kUSBPHDC_receiveCompleted);                                 //receive completed
        }

        //check other EP buffer for data - exchange pCT1 with pCT2
        if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 ==
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX){
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (BYTE*)stUsbHandle[intfNum].oep_Y_Buffer;
        } else   {
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (BYTE*)stUsbHandle[intfNum].oep_X_Buffer;
        }

        nTmp1 = *PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        //try read data from second buffer
        if (nTmp1 & EPBCNT_NAK){                                                //if the second buffer has received data?
            nTmp1 = nTmp1 & 0x7f;                                               //clear NAK bit
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;           //holds how many valid bytes in the EP buffer
            PHDCCopyUsbToBuff(PHDCReadCtrl[INTFNUM_OFFSET(
                                               intfNum)].pCurrentEpPos,
                PHDCReadCtrl[INTFNUM_OFFSET(
                                 intfNum)
                ].pCT1, intfNum);
        }

        if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){    //the Receive opereation is completed
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;           //no more receiving pending
            if (wUsbEventMask & kUSB_receiveCompletedEvent){
                USBPHDC_handleReceiveCompleted(intfNum);                        //call event handler in interrupt context
            }
            __bis_SR_register(bGIE);                                            //restore interrupt status
            return (kUSBPHDC_receiveCompleted);                                 //receive completed
        }
    } //read rest of data from buffer, if any

    //read 'fresh' data, if available
    nTmp1 = 0;
    if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){    //this is current buffer
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & EPBCNT_NAK){ //this buffer has a valid data packet
            //this is the active EP buffer
            //pEP1
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (BYTE*)stUsbHandle[intfNum].oep_X_Buffer;
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;

            //second EP buffer
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (BYTE*)stUsbHandle[intfNum].oep_Y_Buffer;
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
            nTmp1 = 1;                                                          //indicate that data is available
        }
    } else   { //Y_BUFFER
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & EPBCNT_NAK){
            //this is the active EP buffer
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (BYTE*)stUsbHandle[intfNum].oep_Y_Buffer;
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;

            //second EP buffer
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (BYTE*)stUsbHandle[intfNum].oep_X_Buffer;
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
            nTmp1 = 1;                                                          //indicate that data is available
        }
    }
    if (nTmp1){
        //how many byte we can get from one endpoint buffer
        nTmp1 = *PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        while (nTmp1 == 0)
        {
            nTmp1 = *PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        }

        if (nTmp1 & EPBCNT_NAK){
            nTmp1 = nTmp1 & 0x7f;                                               //clear NAK bit
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;           //holds how many valid bytes in the EP buffer

            PHDCCopyUsbToBuff(PHDCReadCtrl[INTFNUM_OFFSET(
                                               intfNum)].pCurrentEpPos,
                PHDCReadCtrl[INTFNUM_OFFSET(
                                 intfNum)
                ].pCT1, intfNum);

            nTmp1 = *PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
            //try read data from second buffer
            if ((PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft >
                 0) &&                                                          //do we have more data to send?
                (nTmp1 & EPBCNT_NAK)){                                          //if the second buffer has received data?
                nTmp1 = nTmp1 & 0x7f;                                           //clear NAK bit
                PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;       //holds how many valid bytes in the EP buffer
                PHDCCopyUsbToBuff(PHDCReadCtrl[INTFNUM_OFFSET(
                                                   intfNum)].pEP2,
                    PHDCReadCtrl[INTFNUM_OFFSET(
                                     intfNum)
                    ].pCT2, intfNum);
                PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                    PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
            }
        }
    }

    if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){        //the Receive opereation is completed
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;               //no more receiving pending
        if (wUsbEventMask & kUSB_receiveCompletedEvent){
            USBPHDC_handleReceiveCompleted(intfNum);                            //call event handler in interrupt context
        }
        __bis_SR_register(bGIE);                                                //restore interrupt status
        return (kUSBPHDC_receiveCompleted);
    }

    //interrupts enable
    __bis_SR_register(bGIE);                                                    //restore interrupt status
    return (kUSBPHDC_receiveStarted);
}

//this function is used only by USB interrupt.
//It fills user receiving buffer with received data
BOOL PHDCToBufferFromHost (BYTE intfNum)
{
    BYTE * pEP1;
    BYTE nTmp1;
    BYTE bWakeUp = FALSE;                                                       //per default we do not wake up after interrupt

    BYTE edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){        //do we have somtething to receive?
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;               //no more receiving pending
        return (bWakeUp);
    }

    //No data to receive...
    if (!((tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX |
           tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY)
          & 0x80)){
        return (bWakeUp);
    }

    if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){    //X is current buffer
        //this is the active EP buffer
        pEP1 = (BYTE*)stUsbHandle[intfNum].oep_X_Buffer;
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;

        //second EP buffer
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
            (BYTE*)stUsbHandle[intfNum].oep_Y_Buffer;
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
    } else   {
        //this is the active EP buffer
        pEP1 = (BYTE*)stUsbHandle[intfNum].oep_Y_Buffer;
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;

        //second EP buffer
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
            (BYTE*)stUsbHandle[intfNum].oep_X_Buffer;
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
    }

    //how many byte we can get from one endpoint buffer
    nTmp1 = *PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;

    if (nTmp1 & EPBCNT_NAK){
        nTmp1 = nTmp1 & 0x7f;                                                   //clear NAK bit
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;               //holds how many valid bytes in the EP buffer

        PHDCCopyUsbToBuff(pEP1, PHDCReadCtrl[INTFNUM_OFFSET(
                                                 intfNum)].pCT1, intfNum);

        nTmp1 = *PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
        //try read data from second buffer
        if ((PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft > 0) &&  //do we have more data to send?
            (nTmp1 & EPBCNT_NAK)){                                              //if the second buffer has received data?
            nTmp1 = nTmp1 & 0x7f;                                               //clear NAK bit
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;           //holds how many valid bytes in the EP buffer
            PHDCCopyUsbToBuff(PHDCReadCtrl[INTFNUM_OFFSET(
                                               intfNum)].pEP2,
                PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2, intfNum);
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
        }
    }

    if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){        //the Receive opereation is completed
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;               //no more receiving pending
        if (wUsbEventMask & kUSB_receiveCompletedEvent){
            bWakeUp = USBPHDC_handleReceiveCompleted(intfNum);
        }

        if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp){                  //Is not read data still available in the EP?
            if (wUsbEventMask & kUSB_dataReceivedEvent){
                bWakeUp = USBPHDC_handleDataReceived(intfNum);
            }
        }
    }
    return (bWakeUp);
}

//helper for USB interrupt handler
BOOL PHDCIsReceiveInProgress (BYTE intfNum)
{
    return (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL);
}

/*
 * Aborts an active receive operation on interface intfNum.
 * Returns the number of bytes that were received and transferred
 * to the data location established for this receive operation.
 */
BYTE USBPHDC_abortReceive (WORD* size, BYTE intfNum)
{
    //interrupts disable
    unsigned short bGIE;

    bGIE  = (__get_SR_register() & GIE);                        //save interrupt status
    //atomic operation - disable interrupts
    __disable_interrupt();                                      //Disable global interrupts

    *size = 0;                                                  //set received bytes count to 0

    //is receive operation underway?
    if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer){
        //how many bytes are already received?
        *size = PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive -
                PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft;

        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft = 0;
    }

    //restore interrupt status
    __bis_SR_register(bGIE);                                    //restore interrupt status
    return (kUSB_succeed);
}

/*
 * This function rejects payload data that has been received from the host.
 */
BYTE USBPHDC_rejectData (BYTE intfNum)
{
    BYTE edbIndex;
    unsigned short bGIE;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    bGIE  = (__get_SR_register() & GIE);                        //save interrupt status

    //atomic operation - disable interrupts
    __disable_interrupt();                                      //Disable global interrupts

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if (bFunctionSuspended){
        __bis_SR_register(bGIE);                                //restore interrupt status
        return (kUSBPHDC_busNotAvailable);
    }

    //Is receive operation underway?
    //- do not flush buffers if any operation still active.
    if (!PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer){
        BYTE tmp1 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                    EPBCNT_NAK;
        BYTE tmp2 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                    EPBCNT_NAK;

        if (tmp1 ^ tmp2){ //switch current buffer if any and only ONE of buffers is full
            //switch current buffer
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
                (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY +
                 1) & 0x01;
        }

        tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX = 0;   //flush buffer X
        tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY = 0;   //flush buffer Y
        PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;   //indicates that no more data available in the EP
    }

    __bis_SR_register(bGIE);                                    //restore interrupt status
    return (kUSB_succeed);
}

/*
 * This function indicates the status of the itnerface intfNum.
 * If a send operation is active for this interface,
 * the function also returns the number of bytes that have been transmitted to the host.
 * If a receiver operation is active for this interface, the function also returns
 * the number of bytes that have been received from the host and are waiting at the assigned address.
 *
 * returns kUSBPHDC_waitingForSend (indicates that a call to USBPHDC_SendData()
 * has been made, for which data transfer has not been completed)
 *
 * returns kUSBPHDC_waitingForReceive (indicates that a receive operation
 * has been initiated, but not all data has yet been received)
 *
 * returns kUSBPHDC_dataWaiting (indicates that data has been received
 * from the host, waiting in the USB receive buffers)
 */
BYTE USBPHDC_intfStatus (BYTE intfNum, WORD* bytesSent, WORD* bytesReceived)
{
    BYTE ret = 0;
    unsigned short bGIE;

    *bytesSent = 0;
    *bytesReceived = 0;

    BYTE edbIndex;
    edbIndex = stUsbHandle[intfNum].edb_Index;

    bGIE  = (__get_SR_register() & GIE);                        //save interrupt status
    __disable_interrupt();                                      //disable interrupts - atomic operation

    //Is send operation underway?
    if (PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSendLeft != 0){
        ret |= kUSBPHDC_waitingForSend;
        *bytesSent = PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].nPHDCBytesToSend -
                     PHDCWriteCtrl[INTFNUM_OFFSET(intfNum)].
                     nPHDCBytesToSendLeft;
    }

    //Is receive operation underway?
    if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL){
        ret |= kUSBPHDC_waitingForReceive;
        *bytesReceived =
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive -
            PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft;
    } else   { //receive operation not started
                                                                //do not access USB memory if suspended (PLL off). It may produce
                                                                //BUS_ERROR
        if (!bFunctionSuspended){
            if ((tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                 EPBCNT_NAK)  |                                 //any of buffers has a valid data packet
                (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                 EPBCNT_NAK)){
                ret |= kUSBPHDC_dataWaiting;
            }
        }
    }

    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //if suspended or not enumerated - report no other tasks pending
        ret = kUSBPHDC_busNotAvailable;
    }

    //restore interrupt status
    __bis_SR_register(bGIE);                                    //restore interrupt status

    __no_operation();
    return (ret);
}

/*
 * Returns how many bytes are in the buffer are received and ready to be read.
 */
BYTE USBPHDC_bytesInUSBBuffer (BYTE intfNum)
{
    BYTE bTmp1 = 0;
    unsigned short bGIE;
    BYTE edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    bGIE  = (__get_SR_register() & GIE);                        //save interrupt status
    //atomic operation - disable interrupts
    __disable_interrupt();                                      //Disable global interrupts

    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        __bis_SR_register(bGIE);                                //restore interrupt status
        //if suspended or not enumerated - report 0 bytes available
        return (0);
    }

    if (PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > 0){  //If a RX operation is underway, part of data may was read of the
                                                                //OEP buffer
        bTmp1 = PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp;
        if (*PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 & EPBCNT_NAK){ //the next buffer has a valid data packet
            bTmp1 += *PHDCReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 & 0x7F;
        }
    } else   {
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & EPBCNT_NAK){ //this buffer has a valid data packet
            bTmp1 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & 0x7F;
        }
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & EPBCNT_NAK){ //this buffer has a valid data packet
            bTmp1 += tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & 0x7F;
        }
    }

    __bis_SR_register(bGIE);                                    //restore interrupt status
    return (bTmp1);
}

//----------------------------------------------------------------------------
//Line Coding Structure
//dwDTERate     | 4 | Data terminal rate, in bits per second
//bCharFormat   | 1 | Stop bits, 0 = 1 Stop bit, 1 = 1,5 Stop bits, 2 = 2 Stop bits
//bParityType   | 1 | Parity, 0 = None, 1 = Odd, 2 = Even, 3= Mark, 4 = Space
//bDataBits     | 1 | Data bits (5,6,7,8,16)
//----------------------------------------------------------------------------
BYTE USBPHDC_GetDataStatusReq (VOID)
{
    BYTE i;

    //Initialize response
    abUsbRequestReturnData[0] = 0;
    abUsbRequestReturnData[1] = 0;

    for (i = 0; i < PHDC_NUM_INTERFACES; i++)
    {
        if (abromConfigurationDescriptorGroup.stPhdc[i].bInterfaceNumber ==
            tSetupPacket.wIndex){
            if (PHDCWriteCtrl[i].nPHDCBytesToSendLeft){
                abUsbRequestReturnData[0] |= 1 <<
                                             (stUsbHandle[PHDC0_INTFNUM +
                                                          i].ep_Out_Addr);
            }

            if (PHDCReadCtrl[i].nBytesInEp){
                abUsbRequestReturnData[0] |= 1 <<
                                             (stUsbHandle[PHDC0_INTFNUM +
                                                          i].ep_In_Addr & 0x7F);
            }
            break;
        }
    }

    /*
     * edbIndex = stUsbHandle[intfNum].edb_Index;
     * tInputEndPointDescriptorBlock[edbIndex].bEPCNF = 0;
     *
     * abromConfigurationDescriptorGroup.stPhdc[0].bEndpointAddress_intp
     #ifdef PHDC_USE_INT_ENDPOINT
     *       // ENDPOINT #1 INPUT DESCRIPTOR, (7 bytes)
     *       SIZEOF_ENDPOINT_DESCRIPTOR,     // bLength: Endpoint Descriptor size
     *       DESC_TYPE_ENDPOINT,	            // bDescriptorType: Endpoint
     *       PHDC0_INTEP_ADDR,                // bEndpointAddress:
     *
     * for (i=0; i < PHDC_NUM_INTERFACES; i++)
     * {
     *   for (j=0; j< abromConfigurationDescriptorGroup.stPhdc[i].bNumEndpoints; j++)
     *   {
     *
     *   }
     * }
     * bNumEndpoints
     * abromConfigurationDescriptorGroup.stPhdc[PHDC_NUM_INTERFACES]
     *   if(tSetupPacket.wIndex & EP_DESC_ADDR_DIR_IN)
     *       {
     *           // input endpoint
     *           abUsbRequestReturnData[0] = (BYTE)(tInputEndPointDescriptorBlock[bEndpointNumber].bEPCNF & EPCNF_STALL);
     *       }else
     *       {
     *           // output endpoint
     *           abUsbRequestReturnData[0] = (BYTE)(tOutputEndPointDescriptorBlock[bEndpointNumber].bEPCNF & EPCNF_STALL);
     *       }
     *   }   // no response if endpoint is not supported.
     *   abUsbRequestReturnData[0] = abUsbRequestReturnData[0] >> 3; // STALL is on bit 3
     */
    wBytesRemainingOnIEP0 = 0x02;
    usbSendDataPacketOnEP0((PBYTE)&abUsbRequestReturnData[0]);
    return (FALSE);
}

#endif  //ifdef _PHDC_

/*----------------------------------------------------------------------------+
 | End of source file                                                          |
 +----------------------------------------------------------------------------*/
/*------------------------ Nothing Below This Line --------------------------*/
