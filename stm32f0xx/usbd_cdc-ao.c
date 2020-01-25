/* patched and reworked by Aegean Associates, Inc. (AO)
   Copyright (c) 2019 Aegean Associates, Inc.
*/
/**
***************************************************************************
* @file    usbd_cdc.c
* @author  MCD Application Team
* @version V2.4.2
* @date    11-December-2015
* @brief   This file provides the high layer firmware functions to manage
*          the following functionalities of the USB CDC Class:
*          - Initialization and Configuration of high and low layer
*          - Enumeration as CDC Device (and enumeration for each
*            implemented memory interface)
*          - OUT/IN data transfer
*          - Command IN transfer (class requests management)
*          - Error management
*  @verbatim
*          ================================================================
*                             CDC Class Driver Description
*          ================================================================
*          This driver manages the "Universal Serial Bus Class Definitions
*          for Communications Devices Revision 1.2 November 16, 2007" and the
*          sub-protocol specification of "Universal Serial Bus Communications
*          Class Subclass Specification for PSTN Devices Revision 1.2
*          February 9, 2007"
*          This driver implements the following aspects of the specification:
*          - Device descriptor management
*          - Configuration descriptor management
*          - Enumeration as CDC device with 2 data endpoints (IN and OUT)
*            and 1 command endpoint (IN)
*          - Requests management (as described in section 6.2 in specification)
*          - Abstract Control Model compliant
*          - Union Functional collection (using 1 IN endpoint for control)
*          - Data interface class
*          These aspects may be enriched or modified for a specific user
*          application. This driver doesn't implement the following aspects
*          of the specification (but it is possible to manage these features
*          with some modifications on this driver):
*          - Any class-specific aspect relative to communication classes
*            should be managed by user application.
*          - All communication classes other than PSTN are not managed
*  @endverbatim
***************************************************************************
* @attention
* <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*        http://www.st.com/software_license_agreement_liberty_v2
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
***************************************************************************
*/

#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

#define FS_PKTSZ  CDC_DATA_FS_IN_PACKET_SIZE
#define HS_PKTSZ  CDC_DATA_HS_IN_PACKET_SIZE
#define CC_PKTSZ  CDC_CMD_PACKET_SIZE

#define PCD_t  PCD_HandleTypeDef
#define USB_t  USBD_HandleTypeDef
#define CDC_t  USBD_CDC_HandleTypeDef
#define ITF_t  USBD_CDC_ItfTypeDef

static uint8_t USBD_CDC_Init (USB_t * pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_DeInit (USB_t * pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_Setup (USB_t * pdev, USBD_SetupReqTypedef *r);
static uint8_t USBD_CDC_DataIn (USB_t * pdev, uint8_t epnum);
static uint8_t USBD_CDC_DataOut (USB_t * pdev, uint8_t epnum);
static uint8_t USBD_CDC_EP0_RxReady (USB_t * pdev);
static uint8_t * USBD_CDC_GetFSCfgDesc (uint16_t * length);
static uint8_t * USBD_CDC_GetHSCfgDesc (uint16_t * length);
static uint8_t * USBD_CDC_GetOtherSpeedCfgDesc (uint16_t * length);
uint8_t * USBD_CDC_GetDeviceQualifierDescriptor (uint16_t * length);


/* USB Standard Device Descriptor */
__ALIGN_BEGIN
static uint8_t USBD_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC]
__ALIGN_END =
    {
     USB_LEN_DEV_QUALIFIER_DESC,
     USB_DESC_TYPE_DEVICE_QUALIFIER,
     0x00,
     0x02,
     0x00,
     0x00,
     0x00,
     0x40,
     0x01,
     0x00
    };

/* CDC interface class callbacks structure */
USBD_ClassTypeDef USBD_CDC =
    {
     USBD_CDC_Init,
     USBD_CDC_DeInit,
     USBD_CDC_Setup,
     NULL, /* EP0_TxSent, */
     USBD_CDC_EP0_RxReady,
     USBD_CDC_DataIn,
     USBD_CDC_DataOut,
     NULL, /* SOF */
     NULL,
     NULL,
#if 0  // UNNECESSARY CODE FOR OUR APPLICATION
     USBD_CDC_GetHSCfgDesc,
#else
     NULL,
#endif
     USBD_CDC_GetFSCfgDesc,
     USBD_CDC_GetOtherSpeedCfgDesc,
     USBD_CDC_GetDeviceQualifierDescriptor
    };


#if 0  // UNNECESSARY CODE FOR OUR APPLICATION
__ALIGN_BEGIN
uint8_t USBD_CDC_CfgHSDesc[USB_CDC_CONFIG_DESC_SIZ]
__ALIGN_END =
    {
     /* Configuration Descriptor */
     0x09,                                // bLength: Cfg Descriptor size
     USB_DESC_TYPE_CONFIGURATION,         // bDescriptorType: Configuration
     USB_CDC_CONFIG_DESC_SIZ,             // wTotalLength:no of returned bytes
     0x00,
     0x02,                                // bNumInterfaces: 2 interfaces
     0x01,                                // bConfigurationValue: Cfg value
     0x00,                                // iConfiguration: index to str desc
     0xC0,                                // bmAttributes: self powered
     0x32,                                // MaxPower 0 mA

     /* ------------------------------------------------------------------ */

     /* Interface Descriptor */
     0x09,                                // bLength: Interface Descriptor size
     USB_DESC_TYPE_INTERFACE,             // bDescriptorType: Interface
     0x00,                                // bInterfaceNumber:
     0x00,                                // bAlternateSetting:
     0x01,                                // bNumEndpoints: One endpoints used
     0x02,                                // bInterfaceClass: Communication
     0x02,  // bInterfaceSubClass:        // Abstract Control Model
     0x01,  // bInterfaceProtocol:        // Common AT commands
     0x00,                                // iInterface:

     /* Header Functional Descriptor */
     0x05,                                // bLength: Endpoint Descriptor size
     0x24,                                // bDescriptorType: CS_INTERFACE
     0x00,                                // bDescriptorSubtype: Header
     0x10,                                // bcdCDC: spec release number
     0x01,

     /* Call Management Functional Descriptor */
     0x05,                                // bFunctionLength
     0x24,                                // bDescriptorType: CS_INTERFACE
     0x01,                                // bDescriptorSubtype: Call Mgmt
     0x00,                                // bmCapabilities: D0+D1
     0x01,                                // bDataInterface: 1

     /* ACM Functional Descriptor */
     0x04,                                // bFunctionLength
     0x24,                                // bDescriptorType: CS_INTERFACE
     0x02, // bDescriptorSubtype:         // Abstract Control Mgmt
     0x02,                                // bmCapabilities

     /* Union Functional Descriptor*/
     0x05,                                // bFunctionLength
     0x24,                                // bDescriptorType: CS_INTERFACE
     0x06,                                // bDescriptorSubtype: Union
     0x00,                                // bMasterInterface: Communication
     0x01,                                // bSlaveInterface0: Data Class

     /* Endpoint 2 Descriptor */
     0x07,                                // bLength: Endpoint Descriptor size
     USB_DESC_TYPE_ENDPOINT,              // bDescriptorType: Endpoint
     CDC_CMD_EP,                          // bEndpointAddress
     0x03,                                // bmAttributes: Interrupt
     LOBYTE(CDC_CMD_PACKET_SIZE),         // wMaxPacketSize:
     HIBYTE(CDC_CMD_PACKET_SIZE),
     0x10,                                // bInterval:

     /* ------------------------------------------------------------------ */

     /* Data class interface descriptor */
     0x09,                                // bLength: Endpoint Descriptor size
     USB_DESC_TYPE_INTERFACE,             // bDescriptorType:
     0x01,                                // bInterfaceNumber:
     0x00,                                // bAlternateSetting:
     0x02,                                // bNumEndpoints: two endpoints used
     0x0A,                                // bInterfaceClass: CDC
     0x00,                                // bInterfaceSubClass:
     0x00,                                // bInterfaceProtocol:
     0x00,                                // iInterface:

     /* Endpoint OUT Descriptor*/
     0x07,                                // bLength: Endpoint Descriptor size
     USB_DESC_TYPE_ENDPOINT,              // bDescriptorType: Endpoint
     CDC_OUT_EP,                          // bEndpointAddress
     0x02,                                // bmAttributes: Bulk
     LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), // wMaxPacketSize:
     HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
     0x00,                                // bInterval: not used for Bulk

     /* Endpoint IN Descriptor */
     0x07,                                // bLength: Endpoint Descriptor size
     USB_DESC_TYPE_ENDPOINT,              // bDescriptorType: Endpoint
     CDC_IN_EP,                           // bEndpointAddress
     0x02,                                // bmAttributes: Bulk
     LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), // wMaxPacketSize:
     HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
     0x00                                 // not used for Bulk
    };
#endif // UNNECESSARY CODE FOR OUR APPLICATION


__ALIGN_BEGIN
uint8_t USBD_CDC_CfgFSDesc[USB_CDC_CONFIG_DESC_SIZ]
__ALIGN_END =
    {
     /* Configuration Descriptor */
     0x09,                                // bLength: Cfg Descriptor size
     USB_DESC_TYPE_CONFIGURATION,         // bDescriptorType: Configuration
     USB_CDC_CONFIG_DESC_SIZ,             // wTotalLength:no of returned bytes
     0x00,
     0x02,                                // bNumInterfaces: 2 interface
     0x01,                                // bConfigurationValue:
     0x00,                                // iConfiguration: Index to str desc
     0xC0,                                // bmAttributes: self powered
     0x32,                                // MaxPower 0 mA

     /* ------------------------------------------------------------------ */

     /* Interface Descriptor */
     0x09,                                // bLength: Interface Descriptor size
     USB_DESC_TYPE_INTERFACE,             // bDescriptorType: Interface
     0x00,                                // bInterfaceNumber:
     0x00,                                // bAlternateSetting:
     0x01,                                // bNumEndpoints: One endpoints used
     0x02, // bInterfaceClass:            // Communication Interface Class
     0x02, // bInterfaceSubClass:         // Abstract Control Model
     0x01, // bInterfaceProtocol:         // Common AT commands
     0x00,                                // iInterface:

     /* Header Functional Descriptor */
     0x05,                                // bLength: Endpoint Descriptor size
     0x24,                                // bDescriptorType: CS_INTERFACE
     0x00,                                // bDescriptorSubtype: Header
     0x10,                                // bcdCDC: spec release number
     0x01,

     /* Call Management Functional Descriptor */
     0x05,                                // bFunctionLength
     0x24,                                // bDescriptorType: CS_INTERFACE
     0x01, // bDescriptorSubtype:         // Call Management
     0x00,                                // bmCapabilities: D0+D1
     0x01,                                // bDataInterface: 1

     /* ACM Functional Descriptor */
     0x04,                                // bFunctionLength
     0x24,                                // bDescriptorType: CS_INTERFACE
     0x02, // bDescriptorSubtype:         // Abstract Control Mgmt
     0x02,                                // bmCapabilities

     /* Union Functional Descriptor */
     0x05,                                // bFunctionLength
     0x24,                                // bDescriptorType: CS_INTERFACE
     0x06,                                // bDescriptorSubtype: Union
     0x00,  // bMasterInterface:          // Communication Class Interface
     0x01,  // bSlaveInterface0:          // Data Class Interface

     /* Endpoint 2 Descriptor */
     0x07,                                // bLength: Endpoint Descriptor size
     USB_DESC_TYPE_ENDPOINT,              // bDescriptorType: Endpoint
     CDC_CMD_EP,                          // bEndpointAddress
     0x03,                                // bmAttributes: Interrupt
     LOBYTE(CDC_CMD_PACKET_SIZE),         // wMaxPacketSize:
     HIBYTE(CDC_CMD_PACKET_SIZE),
     0x10,                                // bInterval:

     /* ------------------------------------------------------------------ */

     /* Data class interface descriptor */
     0x09,                                // bLength: Endpoint Descriptor size
     USB_DESC_TYPE_INTERFACE,             // bDescriptorType:
     0x01,                                // bInterfaceNumber:
     0x00,                                // bAlternateSetting:
     0x02,                                // bNumEndpoints: Two endpoints used
     0x0A,                                // bInterfaceClass: CDC
     0x00,                                // bInterfaceSubClass:
     0x00,                                // bInterfaceProtocol:
     0x00,                                // iInterface:

     /* Endpoint OUT Descriptor */
     0x07,                                // bLength: Endpoint Descriptor size
     USB_DESC_TYPE_ENDPOINT,              // bDescriptorType: Endpoint
     CDC_OUT_EP,                          // bEndpointAddress
     0x02,                                // bmAttributes: Bulk transfer
     LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), // wMaxPacketSize:
     HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
     0x00,                                // bInterval: not used with Bulk

     /* Endpoint IN Descriptor */
     0x07,                                // bLength: Endpoint Descriptor size
     USB_DESC_TYPE_ENDPOINT,              // bDescriptorType: Endpoint
     CDC_IN_EP,                           // bEndpointAddress
     0x02,                                // bmAttributes: Bulk transfer
     LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), // wMaxPacketSize:
     HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
     0x00                                 // bInterval: not used for Bulk
    };


__ALIGN_BEGIN
uint8_t USBD_CDC_OtherSpeedCfgDesc[USB_CDC_CONFIG_DESC_SIZ]
__ALIGN_END =
    {
     0x09,                                // bLength: Cfg Descriptor size
     USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION,
     USB_CDC_CONFIG_DESC_SIZ,
     0x00,
     0x02,                                // bNumInterfaces: 2 interfaces
     0x01,                                // bConfigurationValue:
     0x04,                                // iConfiguration:
     0xC0,                                // bmAttributes:
     0x32,                                // MaxPower 100 mA

     /* Interface Descriptor */
     0x09,                                // bLength: Interface Descriptor size
     USB_DESC_TYPE_INTERFACE,             // bDescriptorType: Interface
     0x00,                                // bInterfaceNumber:
     0x00,                                // bAlternateSetting:
     0x01,                                // bNumEndpoints: One endpoints used
     0x02,  // bInterfaceClass:           // Communication Interface Class
     0x02,  // bInterfaceSubClass:        // Abstract Control Model
     0x01,  // bInterfaceProtocol:        // Common AT commands
     0x00,  // iInterface:

     /* Header Functional Descriptor */
     0x05,                                // bLength: Endpoint Descriptor size
     0x24,                                // bDescriptorType: CS_INTERFACE
     0x00,                                // bDescriptorSubtype: Header
     0x10,                                // bcdCDC: spec release number
     0x01,

     /* Call Management Functional Descriptor */
     0x05,                                // bFunctionLength
     0x24,                                // bDescriptorType: CS_INTERFACE
     0x01,                                // bDescriptorSubtype: Call Mgmt
     0x00,                                // bmCapabilities: D0+D1
     0x01,                                // bDataInterface: 1

     /* ACM Functional Descriptor */
     0x04,                                // bFunctionLength
     0x24,                                // bDescriptorType: CS_INTERFACE
     0x02,  // bDescriptorSubtype:        // Abstract Control Mgmt
     0x02,  // bmCapabilities

     /* Union Functional Descriptor */
     0x05,                                // bFunctionLength
     0x24,                                // bDescriptorType: CS_INTERFACE
     0x06,                                // bDescriptorSubtype: Union
     0x00,                                // bMasterInterface: Communication
     0x01,                                // bSlaveInterface0: Data Class

     /* Endpoint 2 Descriptor */
     0x07,                                // bLength: Endpoint Descriptor size
     USB_DESC_TYPE_ENDPOINT,              // bDescriptorType: Endpoint
     CDC_CMD_EP,                          // bEndpointAddress
     0x03,                                // bmAttributes: Interrupt
     LOBYTE(CDC_CMD_PACKET_SIZE),         // wMaxPacketSize:
     HIBYTE(CDC_CMD_PACKET_SIZE),
     0xFF,                                // bInterval:

     /* ------------------------------------------------------------------ */

     /* Data class interface descriptor */
     0x09,                                // bLength: Endpoint Descriptor size
     USB_DESC_TYPE_INTERFACE,             // bDescriptorType:
     0x01,                                // bInterfaceNumber:
     0x00,                                // bAlternateSetting:
     0x02,                                // bNumEndpoints: Two endpoints used
     0x0A,                                // bInterfaceClass: CDC
     0x00,                                // bInterfaceSubClass:
     0x00,                                // bInterfaceProtocol:
     0x00,                                // iInterface:

     /* Endpoint OUT Descriptor */
     0x07,                                // bLength: Endpoint Descriptor size
     USB_DESC_TYPE_ENDPOINT,              // bDescriptorType: Endpoint
     CDC_OUT_EP,                          // bEndpointAddress
     0x02,                                // bmAttributes: Bulk transfer
     0x40,                                // wMaxPacketSize:
     0x00,
     0x00,                                // bInterval: not used for Bulk

     /* Endpoint IN Descriptor */
     0x07,                                // bLength: Endpoint Descriptor size
     USB_DESC_TYPE_ENDPOINT,              // bDescriptorType: Endpoint
     CDC_IN_EP,                           // bEndpointAddress
     0x02,                                // bmAttributes: Bulk
     0x40,                                // wMaxPacketSize:
     0x00,
     0x00                                 // bInterval
    };

static uint8_t USBD_CDC_Init (USB_t * pdev, uint8_t cfgidx)
{
    CDC_t * cdc;

#if 0  // UNNECESSARY CODE FOR OUR APPLICATION
    if (pdev->dev_speed == USBD_SPEED_HIGH) {
	USBD_LL_OpenEP(pdev, CDC_IN_EP, USBD_EP_TYPE_BULK, HS_PKTSZ);
	USBD_LL_OpenEP(pdev, CDC_OUT_EP, USBD_EP_TYPE_BULK, HS_PKTSZ);
    }
    else
#endif
    {
	USBD_LL_OpenEP(pdev, CDC_IN_EP, USBD_EP_TYPE_BULK, FS_PKTSZ);
	USBD_LL_OpenEP(pdev, CDC_OUT_EP, USBD_EP_TYPE_BULK, FS_PKTSZ);
    }
    USBD_LL_OpenEP(pdev, CDC_CMD_EP, USBD_EP_TYPE_INTR, CC_PKTSZ);

    pdev->pClassData = USBD_malloc(sizeof(CDC_t));
    if (! pdev->pClassData)
	return USBD_FAIL;

    cdc = (CDC_t *) pdev->pClassData;
    ((ITF_t *) pdev->pUserData)->Init();
    cdc->TxState = 0;
    cdc->RxState = 0;

#if 0  // UNNECESSARY CODE FOR OUR APPLICATION
    if (pdev->dev_speed == USBD_SPEED_HIGH)
	USBD_LL_PrepareReceive(pdev, CDC_OUT_EP, cdc->RxBuffer, HS_PKTSZ);
    else
#endif
	USBD_LL_PrepareReceive(pdev, CDC_OUT_EP, cdc->RxBuffer, FS_PKTSZ);

    return USBD_OK;
}

static uint8_t USBD_CDC_DeInit (USB_t * pdev, uint8_t cfgidx)
{
    USBD_LL_CloseEP(pdev, CDC_IN_EP);
    USBD_LL_CloseEP(pdev, CDC_OUT_EP);
    USBD_LL_CloseEP(pdev, CDC_CMD_EP);
    if (pdev->pClassData) {
	((ITF_t *) pdev->pUserData)->DeInit();
	USBD_free(pdev->pClassData);
	pdev->pClassData = NULL;
    }
    return USBD_OK;
}

static uint8_t USBD_CDC_Setup(USB_t * pdev, USBD_SetupReqTypedef * req)
{
    static uint8_t ifalt = 0;

    CDC_t * cdc = (CDC_t *) pdev->pClassData;
    ITF_t * itf = (ITF_t *) pdev->pUserData;

    switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    case USB_REQ_TYPE_CLASS:
	if (req->wLength) {
	    if (req->bmRequest & 0x80) {
		itf->Control(req->bRequest, (uint8_t*)cdc->data, req->wLength);
		USBD_CtlSendData(pdev, (uint8_t *) cdc->data, req->wLength);
	    }
	    else {
		cdc->CmdOpCode = req->bRequest;
		cdc->CmdLength = req->wLength;
		USBD_CtlPrepareRx(pdev, (uint8_t *) cdc->data, req->wLength);
	    }
	}
	else {
	    itf->Control(req->bRequest, (uint8_t *) req, 0);
	}
	break;
    case USB_REQ_TYPE_STANDARD:
	switch (req->bRequest) {
	case USB_REQ_GET_INTERFACE:
	    USBD_CtlSendData (pdev, &ifalt, 1);
	    break;
	case USB_REQ_SET_INTERFACE:
	    break;
	}
    default:
	break;
    }
    return USBD_OK;
}

static uint8_t USBD_CDC_DataIn(USB_t * pdev, uint8_t epnum)
{
    CDC_t * cdc = (CDC_t *) pdev->pClassData;

    if (cdc) {
#if 0
	PCD_t * hpcd = (PCD_t *) pdev->pData;
	uint32_t m = hpcd->IN_ep[epnum].maxpacket;
	uint32_t t = pdev->ep_in[epnum].total_length;
	if ((t > m) && ((t % m) == 0)) {
	    pdev->ep_in[epnum].total_length = 0;
	    USBD_LL_Transmit(pdev, epnum, NULL, 0);
	}
	else
#endif
	    cdc->TxState = 0;
	return USBD_OK;
    }
    return USBD_FAIL;
}

static uint8_t USBD_CDC_DataOut(USB_t * pdev, uint8_t epnum)
{
    CDC_t * cdc = (CDC_t *) pdev->pClassData;
    ITF_t * itf = (ITF_t *) pdev->pUserData;

    if (cdc) {
	// USB data will be immediately processed, this causes the
	// next USB traffic to be NAKed till the application "readies"
	// the receiver (prepare receive).
	cdc->RxLength = USBD_LL_GetRxDataSize(pdev, epnum);
	itf->Receive(cdc->RxBuffer, &cdc->RxLength);
	return USBD_OK;
    }
    return USBD_FAIL;
}

static uint8_t USBD_CDC_EP0_RxReady(USB_t * pdev)
{
    CDC_t * cdc = (CDC_t *) pdev->pClassData;
    ITF_t * itf = (ITF_t *) pdev->pUserData;

    if (itf && (cdc->CmdOpCode != 0xff)) {
	itf->Control(cdc->CmdOpCode, (uint8_t *) cdc->data, cdc->CmdLength);
	cdc->CmdOpCode = 0xff;
    }
    return USBD_OK;
}

#if 0  // UNNECESSARY CODE FOR OUR APPLICATION
static uint8_t * USBD_CDC_GetHSCfgDesc(uint16_t * length)
{
    *length = sizeof(USBD_CDC_CfgHSDesc);
    return USBD_CDC_CfgHSDesc;
}
#endif

static uint8_t * USBD_CDC_GetFSCfgDesc(uint16_t * length)
{
    *length = sizeof(USBD_CDC_CfgFSDesc);
    return USBD_CDC_CfgFSDesc;
}

static uint8_t * USBD_CDC_GetOtherSpeedCfgDesc(uint16_t * length)
{
    *length = sizeof(USBD_CDC_OtherSpeedCfgDesc);
    return USBD_CDC_OtherSpeedCfgDesc;
}

uint8_t * USBD_CDC_GetDeviceQualifierDescriptor(uint16_t * length)
{
    *length = sizeof(USBD_CDC_DeviceQualifierDesc);
    return USBD_CDC_DeviceQualifierDesc;
}

uint8_t USBD_CDC_RegisterInterface(USB_t * pdev, ITF_t * fops)
{
    if (fops) {
	pdev->pUserData = fops;
	return USBD_OK;
    }
    return USBD_FAIL;
}

uint8_t USBD_CDC_SetTxBuffer(USB_t * pdev, uint8_t * pbuff, uint16_t length)
{
    CDC_t * cdc = (CDC_t *) pdev->pClassData;
    cdc->TxBuffer = pbuff;
    cdc->TxLength = length;
    return USBD_OK;
}

uint8_t USBD_CDC_SetRxBuffer(USB_t * pdev, uint8_t * pbuff)
{
    CDC_t * cdc = (CDC_t *) pdev->pClassData;
    cdc->RxBuffer = pbuff;
    return USBD_OK;
}

uint8_t USBD_CDC_TransmitPacket(USB_t * pdev)
{
    CDC_t * cdc = (CDC_t *) pdev->pClassData;
    if (cdc) {
	if (! cdc->TxState) {
	    cdc->TxState = 1;  // tx transfer in progress
	    USBD_LL_Transmit(pdev, CDC_IN_EP, cdc->TxBuffer, cdc->TxLength);
	    return USBD_OK;
	}
	return USBD_BUSY;
    }
    return USBD_FAIL;
}

uint8_t USBD_CDC_ReceivePacket(USB_t * pdev)
{
    CDC_t * cdc = (CDC_t *) pdev->pClassData;
    if (cdc) {
#if 0  // UNNECESSARY CODE FOR OUR APPLICATION
	if (pdev->dev_speed == USBD_SPEED_HIGH)
	    USBD_LL_PrepareReceive(pdev, CDC_OUT_EP, cdc->RxBuffer, HS_PKTSZ);
	else
#endif
	    USBD_LL_PrepareReceive(pdev, CDC_OUT_EP, cdc->RxBuffer, FS_PKTSZ);
	return USBD_OK;
    }
    return USBD_FAIL;
}

/*** (c) COPYRIGHT STMicroelectronics ***/
/*** END OF FILE ***/
