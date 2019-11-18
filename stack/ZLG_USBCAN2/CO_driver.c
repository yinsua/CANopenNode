/*
 * CAN module object for Linux SocketCAN.
 *
 * @file        CO_driver.c
 * @author      yinsua
 * @copyright   2019 ACell
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * CANopenNode is free and open source software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */


#include "CO_driver.h"
#include "CO_Emergency.h"
#include <string.h> /* for memcpy */
#include <stdlib.h> /* for malloc, free */
#include <errno.h>
#include <sys/socket.h>
#include <unistd.h> /* for sleep */


/******************************************************************************/
#ifndef CO_SINGLE_THREAD
    pthread_mutex_t CO_EMCY_mtx = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t CO_OD_mtx = PTHREAD_MUTEX_INITIALIZER;
#endif


/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANdriverState){
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
    /* set CAN filters */
    if(CANmodule == NULL || setFilters(CANmodule) != CO_ERROR_NO){
        CO_errExit("CO_CANsetNormalMode failed");
    }
    CANmodule->CANnormal = true;
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        void                   *CANdriverState,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    CO_ReturnError_t ret = CO_ERROR_NO;
    uint16_t i;

    /* verify arguments */
    if(CANmodule==NULL || CANdriverState==NULL || rxArray==NULL || txArray==NULL){
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure object variables */
    if(ret == CO_ERROR_NO){
        CANmodule->CANdriverState = CANdriverState;
        CANmodule->rxArray = rxArray;
        CANmodule->rxSize = rxSize;
        CANmodule->txArray = txArray;
        CANmodule->txSize = txSize;
        CANmodule->CANnormal = false;
        CANmodule->bufferInhibitFlag = false;
        CANmodule->firstCANtxMessage = true;
        CANmodule->error = 0;
        CANmodule->CANtxCount = 0U;
        CANmodule->errOld = 0U;
        CANmodule->em = NULL;

#ifdef CO_LOG_CAN_MESSAGES
        CANmodule->useCANrxFilters = false;
#endif

        for(i=0U; i<rxSize; i++){
            rxArray[i].ident = 0U;
            rxArray[i].mask = 0xFFFFFFFF;
            rxArray[i].object = NULL;
            rxArray[i].pFunct = NULL;
        }
        for(i=0U; i<txSize; i++){
            txArray[i].bufferFull = false;
        }
    }

    /* First time only configuration */
    if(ret == CO_ERROR_NO && CANmodule->wasConfigured == 0){
        CANmodule->wasConfigured = 1;

        do{
            /* init CAN device */
            const CANDeviceType * const device = CANdriverState;
            if( VCI_OpenDevice(device->device_type, device->device_id, 0) == STATUS_ERR ){
                ret = CO_ERROR_ILLEGAL_ARGUMENT;
                break;
            }
            VCI_INIT_CONFIG config;
            config.AccCode = 0;
            config.AccMask = 0xffffffff;
            config.Filter = 1;
            config.Mode = 0;
            config.Timing0 = device->baud & 0xff;
            config.Timing1 = device->baud >> 8;
            if( VCI_InitCAN(device->device_type, device->device_id, device->channel, &config) == STATUS_ERR ){
                ret = CO_ERROR_ILLEGAL_ARGUMENT;
                break;
            }
            sleep(1);
            if( VCI_StartCAN(device->device_type, device->device_id, device->channel) == STATUS_ERR){
                ret = CO_ERROR_ILLEGAL_ARGUMENT;
                break;
            }
            sleep(1);
            if( VCI_ClearBuffer(device->device_type, device->device_id, device->channel) == STATUS_ERR){
                ret = CO_ERROR_ILLEGAL_ARGUMENT;
                break;
            }
            sleep(1);
        }while(0);
    }
    return ret;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule){
    const CANDeviceType * const device = CANmodule->CANdriverState;
    VCI_CloseDevice(device->device_type, device->device_id);
}


/******************************************************************************/
uint16_t CO_CANrxMsg_readIdent(const CO_CANrxMsg_t *rxMsg){
    return (uint16_t) rxMsg->ident;
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (object!=NULL) && (pFunct!=NULL) &&
       (index < CANmodule->rxSize)){
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->pFunct = pFunct;

        /* Configure CAN identifier and CAN mask, bit aligned with CAN module. */
        buffer->ident = ident & CAN_SFF_MASK;
        if(rtr){
            buffer->ident |= CAN_RTR_FLAG;
        }
        buffer->mask = (mask & CAN_SFF_MASK) | CAN_EFF_FLAG | CAN_RTR_FLAG;
    }
    else{
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if((CANmodule != NULL) && (index < CANmodule->txSize)){
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, bit aligned with CAN module registers */
        buffer->ident = ident & CAN_SFF_MASK;
        if(rtr){
            buffer->ident |= CAN_RTR_FLAG;
        }

        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}


/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer){
    CO_ReturnError_t err = CO_ERROR_NO;
    const CANDeviceType * const device = CANmodule->CANdriverState;
    VCI_CAN_OBJ can;
    can.ID = buffer->ident;
    can.SendType = device->tx_type;
    can.DataLen = buffer->DLC;
    for(int i=0; i<can.DataLen; i++){
        can.Data[i] = buffer->data[i];
    }
    can.ExternFlag = 0;
    can.RemoteFlag = 0;

    if( VCI_Transmit(device->device_type, device->device_id, device->channel, &can, 1) == STATUS_ERR){
        // CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, n);
        err = CO_ERROR_TX_OVERFLOW;
    }

#ifdef CO_LOG_CAN_MESSAGES
    void CO_logMessage(const CanMsg *msg);
    CO_logMessage((const CanMsg*) buffer);
#endif

    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
    /* Messages can not be cleared, because they are allready in kernel */
}


/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule){
#if 0
    unsigned rxErrors, txErrors;
    CO_EM_t* em = (CO_EM_t*)CANmodule->em;
    uint32_t err;

    canGetErrorCounters(CANmodule->CANdriverState, &rxErrors, &txErrors);
    if(txErrors > 0xFFFF) txErrors = 0xFFFF;
    if(rxErrors > 0xFF) rxErrors = 0xFF;

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | CANmodule->error;

    if(CANmodule->errOld != err){
        CANmodule->errOld = err;

        if(txErrors >= 256U){                               /* bus off */
            CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, err);
        }
        else{                                               /* not bus off */
            CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, err);

            if((rxErrors >= 96U) || (txErrors >= 96U)){     /* bus warning */
                CO_errorReport(em, CO_EM_CAN_BUS_WARNING, CO_EMC_NO_ERROR, err);
            }

            if(rxErrors >= 128U){                           /* RX bus passive */
                CO_errorReport(em, CO_EM_CAN_RX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
            }
            else{
                CO_errorReset(em, CO_EM_CAN_RX_BUS_PASSIVE, err);
            }

            if(txErrors >= 128U){                           /* TX bus passive */
                if(!CANmodule->firstCANtxMessage){
                    CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
                }
            }
            else{
                bool_t isError = CO_isError(em, CO_EM_CAN_TX_BUS_PASSIVE);
                if(isError){
                    CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, err);
                    CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
                }
            }

            if((rxErrors < 96U) && (txErrors < 96U)){       /* no error */
                bool_t isError = CO_isError(em, CO_EM_CAN_BUS_WARNING);
                if(isError){
                    CO_errorReset(em, CO_EM_CAN_BUS_WARNING, err);
                    CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
                }
            }
        }

        if(CANmodule->error & 0x02){                       /* CAN RX bus overflow */
            CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, err);
        }
    }
#endif
}


/******************************************************************************/
void CO_CANrxWait(CO_CANmodule_t *CANmodule){
    const CANDeviceType * const device = CANmodule->CANdriverState;
    size_t obj_count = 32;
    VCI_CAN_OBJ can[obj_count];
    int rx_obj_count = 0;

    if(CANmodule == NULL){
        errno = EFAULT;
        CO_errExit("CO_CANreceive - CANmodule not configured.");
    }

    /* Read socket and pre-process message */
    rx_obj_count = VCI_Receive(
            device->device_type,
            device->device_id,
            device->channel,
            can,
            obj_count,
            device->rx_timeout);

    if(CANmodule->CANnormal){
        if(rx_obj_count < 1){
            /* This happens only once after error occurred (network down or something). */
            // CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_COMMUNICATION, n);
            return;
        }
        else{
            for(int i=0; i<rx_obj_count; i++){
                CO_CANrxMsg_t *rcvMsg;      /* pointer to received message in CAN module */
                uint32_t rcvMsgIdent;       /* identifier of the received message */
                CO_CANrx_t *buffer;         /* receive message buffer from CO_CANmodule_t object. */
                int i;
                bool_t msgMatched = false;

                rcvMsg->ident = can[i].ID;
                rcvMsg->DLC = can[i].DataLen;
                for(int j=0; j<8; j++)
                    rcvMsg->data[j] = can[i].Data[j];
                rcvMsgIdent = rcvMsg->ident;

                /* Search rxArray form CANmodule for the matching CAN-ID. */
                buffer = &CANmodule->rxArray[0];
                for(i = CANmodule->rxSize; i > 0U; i--){
                    if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
                        msgMatched = true;
                        break;
                    }
                    buffer++;
                }

                /* Call specific function, which will process the message */
                if(msgMatched && (buffer->pFunct != NULL)){
                    buffer->pFunct(buffer->object, rcvMsg);
                }

#ifdef CO_LOG_CAN_MESSAGES
                void CO_logMessage(const CanMsg *msg);
                CO_logMessage((CanMsg*)&rcvMsg);
#endif
            }
        }
    }
}
