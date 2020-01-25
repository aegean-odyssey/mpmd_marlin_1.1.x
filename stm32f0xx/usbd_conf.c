/**
 * PCD (low level USB support for USB CDC)
 * see usbd_conf.h, usbd_conf.c
 *
 * FIXME!? should use PCD or USBD, not both. Yuck
 */

#include "stm32f0xx_hal.h"
#include "usbd_cdc.h"


static PCD_HandleTypeDef _pcd;

void USB_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&_pcd);
}

uint8_t HAL_usb_IsConfigured(void)
{
    return _pcd.pData
	&& (((USBD_HandleTypeDef *) _pcd.pData)->dev_state
	    == USBD_STATE_CONFIGURED);
}

/* NOTE: to avoid the need for heap space and dynamic memory allocation,
   the single call to malloc (in the CDC class driver) is replaced with
   a static memory allocation. (see usbd_conf.h) */

void * HAL_pcd_static_malloc(uint32_t size)
{
    static USBD_CDC_HandleTypeDef _cdc;
    if (size <= sizeof(USBD_CDC_HandleTypeDef))
	return &_cdc;
    return NULL;
}
    
void HAL_pcd_static_free(void * p)
{

}

/* these functions are defined in HAL_stm32.c */
// void HAL_PCD_MspInit(PCD_HandleTypeDef * pcd);
// void HAL_PCD_MspDeInit(PCD_HandleTypeDef * pcd);

static int HAL_usb_init(USBD_HandleTypeDef * usbd)
{
    _pcd.Instance = USB;
    _pcd.Init.dev_endpoints = 8;
    _pcd.Init.ep0_mps = DEP0CTL_MPS_8;
    _pcd.Init.phy_itface = PCD_PHY_EMBEDDED;
    _pcd.Init.speed = PCD_SPEED_FULL;
    _pcd.Init.low_power_enable = 0;
    // link pcd and usbd
    _pcd.pData = usbd;
    usbd->pData = &_pcd;
    HAL_PCD_Init(&_pcd);

    HAL_PCDEx_PMAConfig(&_pcd, 0x00, PCD_SNG_BUF, 0x18);
    HAL_PCDEx_PMAConfig(&_pcd, 0x80, PCD_SNG_BUF, 0x58);
    HAL_PCDEx_PMAConfig(&_pcd, CDC_IN_EP, PCD_SNG_BUF, 0xC0);
    HAL_PCDEx_PMAConfig(&_pcd, CDC_OUT_EP, PCD_SNG_BUF, 0x110);
    HAL_PCDEx_PMAConfig(&_pcd, CDC_CMD_EP, PCD_SNG_BUF, 0x100);
    return 0;
}

static int HAL_usb_deinit(USBD_HandleTypeDef * usbd)
{
    HAL_PCD_DeInit((PCD_HandleTypeDef *) usbd->pData);
    return 0;
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef * pcd)
{
    USBD_LL_SetupStage((USBD_HandleTypeDef *) pcd->pData,
		       (uint8_t *) pcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef * pcd, uint8_t epn)
{
    USBD_HandleTypeDef * p = (USBD_HandleTypeDef *) pcd->pData;
    if ((epn == 0) && (p->ep0_state == USBD_EP0_STATUS_OUT)) {
	// STATUS PHASE completed, update ep0_state to idle
	p->ep0_state = USBD_EP0_IDLE;
	USBD_LL_StallEP(p, 0);
	return;
    }
    USBD_LL_DataOutStage(p, epn, pcd->OUT_ep[epn].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef * pcd, uint8_t epn)
{
    USBD_HandleTypeDef * p = (USBD_HandleTypeDef *) pcd->pData;
    USBD_LL_DataInStage(p, epn, pcd->IN_ep[epn].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef * pcd)
{
    USBD_LL_SOF((USBD_HandleTypeDef *) pcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef * pcd)
{
    USBD_HandleTypeDef * p = (USBD_HandleTypeDef *) pcd->pData;
    USBD_LL_SetSpeed(p, USBD_SPEED_FULL);
    USBD_LL_Reset(p);
    p->ep0_state = USBD_EP0_IDLE;
    p->dev_config = 0;
    p->dev_remote_wakeup = 0;
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef * pcd)
{
    USBD_HandleTypeDef * p = (USBD_HandleTypeDef *) pcd->pData;
    if (p->dev_state != USBD_STATE_SUSPENDED) USBD_LL_Suspend(p);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef * pcd)
{
    USBD_HandleTypeDef * p = (USBD_HandleTypeDef *) pcd->pData;
    if (p->dev_state == USBD_STATE_SUSPENDED) USBD_LL_Resume(p);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef * pcd, uint8_t epn)
{
    USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *) pcd->pData, epn);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef * pcd, uint8_t epn)
{
    USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *) pcd->pData, epn);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef * pcd)
{
    USBD_LL_DevConnected((USBD_HandleTypeDef *) pcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef * pcd)
{
    USBD_LL_DevDisconnected((USBD_HandleTypeDef *) pcd->pData);
}

/**
 * *************************************************************************
 * @file    USB_Device/CDC_Standalone/Src/usbd_conf.c
 * @author  MCD Application Team
 * @brief   This file implements the USB Device library callbacks and MSP
 ***************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted, provided that the following
 * conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING,
 * BUT NOT  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL
 * PROPERTY RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW.
 * IN NO EVENT SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 ****************************************************************************
 */

/* Includes ---------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "usbd_core.h"
/* Private typedef --------------------------------------------------------*/
/* Private define ---------------------------------------------------------*/
/* Private macro ----------------------------------------------------------*/
/* Private variables ------------------------------------------------------*/
/* Private function prototypes --------------------------------------------*/
/* Private functions ------------------------------------------------------*/

/****************************************************************************
 * LL Driver Interface (USB Device Library --> PCD)
 */

/**
 * @brief  Initializes the Low Level portion of the Device driver.
 * @param  pdev: Device handle
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef * pdev)
{
    HAL_usb_init(pdev);
    return USBD_OK;
}

/**
 * @brief  De-Initializes the Low Level portion of the Device driver.
 * @param  pdev: Device handle
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef * pdev)
{
    HAL_usb_deinit(pdev);
    return USBD_OK;
}

/**
 * @brief  Starts the Low Level portion of the Device driver. 
 * @param  pdev: Device handle
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef * pdev)
{
    HAL_PCD_Start((PCD_HandleTypeDef *) pdev->pData);
    return USBD_OK;
}

/**
 * @brief  Stops the Low Level portion of the Device driver.
 * @param  pdev: Device handle
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef * pdev)
{
    HAL_PCD_Stop((PCD_HandleTypeDef *) pdev->pData);
    return USBD_OK;
}

/**
 * @brief  Opens an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @param  ep_type: Endpoint Type
 * @param  ep_mps: Endpoint Max Packet Size
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev,
                                  uint8_t ep_addr,
                                  uint8_t ep_type,
                                  uint16_t ep_mps)
{
    HAL_PCD_EP_Open((PCD_HandleTypeDef *) pdev->pData,
		    ep_addr, ep_mps, ep_type);
    return USBD_OK;
}

/**
 * @brief  Closes an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev,
				   uint8_t ep_addr)
{
    HAL_PCD_EP_Close((PCD_HandleTypeDef *) pdev->pData, ep_addr);
    return USBD_OK;
}

/**
 * @brief  Flushes an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef * pdev,
				   uint8_t ep_addr)
{
    HAL_PCD_EP_Flush((PCD_HandleTypeDef *) pdev->pData, ep_addr);
    return USBD_OK;
}

/**
 * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef * pdev,
				   uint8_t ep_addr)
{
    HAL_PCD_EP_SetStall((PCD_HandleTypeDef *) pdev->pData, ep_addr);
    return USBD_OK;
}

/**
 * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef * pdev,
					uint8_t ep_addr)
{
    HAL_PCD_EP_ClrStall((PCD_HandleTypeDef *) pdev->pData, ep_addr);
    return USBD_OK; 
}

/**
 * @brief  Returns Stall condition.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval Stall (1: Yes, 0: No)
 */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef * pdev,
			  uint8_t ep_addr)
{
    PCD_HandleTypeDef * pcd = (PCD_HandleTypeDef *) pdev->pData;

    if (ep_addr & 0x80)
	return pcd->IN_ep[ep_addr & 0x7F].is_stall;

    return pcd->OUT_ep[ep_addr & 0x7F].is_stall;
}

/**
 * @brief  Assigns a USB address to the device.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef * pdev,
					 uint8_t dev_addr)
{
    HAL_PCD_SetAddress((PCD_HandleTypeDef *) pdev->pData, dev_addr);
    return USBD_OK; 
}

/**
 * @brief  Transmits data over an endpoint.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @param  pbuf: Pointer to data to be sent
 * @param  size: Data size    
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef * pdev, 
                                    uint8_t ep_addr,
                                    uint8_t * pbuf,
                                    uint16_t size)
{
    HAL_PCD_EP_Transmit((PCD_HandleTypeDef *) pdev->pData,
			ep_addr, pbuf, size);
    return USBD_OK;
}

/**
 * @brief  Prepares an endpoint for reception.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @param  pbuf: Pointer to data to be received
 * @param  size: Data size
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef * pdev, 
                                          uint8_t ep_addr,
                                          uint8_t * pbuf,
                                          uint16_t size)
{
    HAL_PCD_EP_Receive((PCD_HandleTypeDef *) pdev->pData,
		       ep_addr, pbuf, size);
    return USBD_OK;
}

/**
 * @brief  Returns the last transfered packet size.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval Received Data Size
 */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef * pdev,
			       uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef *) pdev->pData,
				 ep_addr);
}

/**
  * @brief  Delays routine for the USB Device Library.
  * @param  Delay: Delay in ms
  * @retval None
  */
void USBD_LL_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/*** (C) COPYRIGHT STMicroelectronics ***/
/*** END OF FILE ***/
