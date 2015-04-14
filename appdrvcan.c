/******************************************************************************
 * Project          Harman Car Multimedia System
 * (c) copyright    2008
 * Company          Harman/Becker Automotive Systems GmbH
 *                  All rights reserved
 *
 * Secrecy Level    STRICTLY CONFIDENTIAL
 ******************************************************************************/
/**
 * @file           ApplDrvCan.c
 * @ingroup        CAN_APPL
 * @author         Markus Holzapfel
 *
 * @brief application´s CAN driver callback functions
 *
 ******************************************************************************/

/*---------------------------------------------------------------------
 *    INCLUDES
 *--------------------------------------------------------------------*/
#if 0
#include <cfg/can/canpool_inc.h>

#include <cfg/can/canap.cfg>
#include <Can/CAN_Appl/appcantask.h>
#include <Can/CAN_Pool/CanRxFilter.h>
#include <Can/CAN_Appl/appIrqFlags.h>

#include <Can/CAN_Appl/appCanErr.h>

#include <can_inc.h>
#include <can_def.h>

#include <Can/CAN_Appl/gw_rc.h>
#include <CAN/CAN_Appl/can_filter.h>

#define CAN_MSG_ID_BIOS_RESET 0x7B5

// Global variables:
UINT8 flagFirstMessageTransmittedAfterStartup = 0;


/**
 * @brief CAN overrun callback function of the CAN driver.
 *
 * The function is called whenever an CAN overrun occurs, i.e. when a received message is overwritten
 * by another CAN message, before the first message has been processed by the CAN driver.
 *
 *  @date   23.03.2012
 *  @author Markus Holzapfel
 *
 * @param CanNum  CAN channel number
 */
#if defined(C_ENABLE_OVERRUN)
void ApplCanOverrun (CanChannelHandle CanNum)
{
   //SYS_ERROR( ERROR_BASICCAN_OVERRUN_DETECTED );
   setIrqTraceFlag( IRQFLAG_E_BASIC_CAN_OVERRUN );
}
#else
 #error "Basic CAN Overrun detection must be enabled to detect any loss of CAN messages!!!"
#endif

/**
 * @brief Project-specific handling of CAN wake-ups.
 * This callback function is called if a wake-up condition on the CAN bus is detected during sleep mode
 * of the CAN Controller. The function is called in the wakeup interrupt, in the CanTask() or in the
 * CanWakeupTask().
 *
 *  @date   26.03.2012
 *  @author Markus Holzapfel
 *
 */
void ApplCanWakeUp(CanChannelHandle channel)
{

}

/**
 * @brief CAN receive callback function of the CAN driver.
 *
 * This callback function is called on every reception of a CAN message when the hardware acceptance
 * filter is passed.
 *
 * @attention The function is called in the receive interrupt
 *
 * @date   23.03.2012
 * @author Markus Holzapfel
 *
 * @param CanRxInfoStructPtr  Pointer to receive information structure
 *
 * @return  kCanCopyData    Receive processing will be continued
 * @return  kCanNoCopyData  Receive processing will be terminated
 */
#if defined(C_ENABLE_RECEIVE_FCT)
UINT8 ApplCanMsgReceived(CanRxInfoStructPtr rxStruct)
{
   UInt8 retValue = kCanNoCopyData;

   /** CAN fast filter  */
   retValue = (UINT8)Interrupt_CanFilterMessageRecv(rxStruct);

   return (retValue);

}
#else
 #error "Rx Notification (=ApplCanMsgReceived) must be enabled to pass Rx messages to the gateway!!!"
#endif


void  ApplCanFatalError(CAN_CHANNEL_CANTYPE_FIRST vuint8 errorNumber)
{

}
#endif
 defined(C_ENABLE_FULLCAN_OVERRUN)
C_CALLBACK_1 void C_CALLBACK_2 ApplCanFullCanOverrun ( CAN_CHANNEL_CANTYPE_ONLY )
{

}
#else
 #error "Full CAN Overrun detection must be enabled to detect any loss of CAN messages!!!"
#endif


#if defined(C_ENABLE_CAN_TX_CONF_FCT)
void C_CALLBACK_2 ApplCanTxConfirmation(CanTxInfoStructPtr txStruct)
{
   UINT8 canChannel = (UINT8)(txStruct->Channel);

   flagFirstMessageTransmittedAfterStartup = 1;

   setBusOffInactive(canChannel);
}
#else
 #error "Common Confirmation Function must be enabled to detect first CAN transmission time!!!"
#endif

vuint8 PretransmitTMU11(CanTxInfoStruct ctis)
{
   static UINT8 cyclCounter = 0;
   b_CF_Tmu_AliveCnt1_b = cyclCounter;

   /* Increase counter for next transmission */
   cyclCounter++;
   /* 0xF is not a valid counter -> overrun to 0 */
   if (cyclCounter > 0xE)
   {
      cyclCounter = 0;
   }

   /* The driver shall copy the TX data itself into the target buffer */
   return(kCanCopyData);
}




