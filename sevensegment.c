/*************************************************************************
 * Project         Platform CAN                                          *
 * (c) copyright   2012                                                  *
 * Company         Harman/Becker Automotive Systems GmbH                 *
 *                 All rights reserved                                   *
 * Secrecy Level   STRICTLY CONFIDENTIAL                                 *
 *************************************************************************/
/**
 * @file
 *            appCanTimer.c
 *
 * @ingroup comp_IOController_CANPool
 *
 *  Function Periodic call of NM, TP, Canap, Diag, Gateway
 *           Generation of periodic timers for the CAN Application
 *
 * @date       23.03.2012
 * @author     Markus Holzapfel (First Version)
 */

#include <Can/CAN_Appl/appCanTimer.h>
#include <Can/CAN_Pool/private/CanTask_pri.h> // Events
#include <Can/CAN_Pool/private/Canap_pri.h>   // Statemachine Timer
#include <Can/CAN_Pool/Private/CanCycTx_pri.h>
#include <Can/CAN_Appl/appIrqFlags.h>
#include <Can/CAN_Appl/CanSgw.h>              // check Tx pending timeout
#include <Can/CAN_Appl/can_ud.h>
#include <Can/CAN_Appl/appDsiRetry.h>
#include <Can/CAN_Appl/appCanErr.h>
#include <Can/CAN_Appl/appcanap.h>
#include <cfg/CAN/canpool_inc.h>
#include <Can/CAN_Appl/gw_rc.h>
#include <Can/CAN_Appl/can_filter.h>
#include <Can/CAN_Appl/appCanPreboot.h>

//MHO #include <CanNm.h>
//MHO #include <GenericNm.h>
#include <nm_osek.h>
#include <nm_basic.h>
#define CLEAR_FLAG_TIMER 70
#define PRE_BOOT_WAKEUP_ACC_IGN  0x02
#define PRE_BOOT_STOP_TIME_STP   500
#define IGN_ON                   0x03

// TEST VARIABLES
UINT8 modifCan0 = 0;
UINT8 reqCan0 = 0;
UINT8 modifCan1 = 0;
UINT8 reqCan1 = 0;
UINT8 modifCan2 = 0;
UINT8 reqCan2 = 0;
extern bool bIPC_O5_Ready;
UINT8 u84msTimer = 0;

UINT32 tmpTimer = 0;
bool flagToStartClrTimer = FALSE; 

bool b_nadFlag = FALSE;
bool b_CANWakeupFlag = FALSE;
UINT8 CANWakeupTimer = APP_REQ_TIMER; /* Start Application message after 80ms from NAD Wakeup */

TTimerCan TimC;

// measurement of CAN Tx startup time:
extern UINT8 flagFirstMessageTransmittedAfterStartup;
UINT32 msAfterFirstTx = 0;
UINT8  flagTraceMsgOnce = 0;
uint8 Flag = 5;
static bool canInitialized = FALSE;

static void Can1msTimer_Int(void);
static void Can10msTimer_Int(void);
static void appTaskTimer10(void);
static void appTaskTimer50(void);
static void appTaskTimer100(void);
static void appTaskTimer1000(void);
static void appPreboot_Wakeup_check(void);
static void appNM_DTC_check(void);
static void clearValidMsgRecvdFlag(void);
extern UINT8 CAN_TXEnable ;
extern UINT8 CAN_TXDisable ;

/**
 * @brief Initializes all CAN relevant timers.
 *
 *  @date   23.03.2012
 *  @author Markus Holzapfel
 */
void InitCanTaskTimers(void)
{
   // Interrupt driven timers
   TimC.Timer10ms        = TIMER_10_MS;
   TimC.Timer20ms        = TIMER_20_MS_10MS;

   // Task driven timers
   TimC.appTimer50ms     = TIMER_50_MS_10MS;
   TimC.appTimer100ms    = TIMER_100_MS_10MS;
   TimC.appTimer1000ms   = TIMER_1000_MS_10MS;

   canInitialized = TRUE;
}

/***********************************************************************************
 *
 * TIMER FUNCTIONS CALLED WITHIN INTERRUPT CONTEXT - KEEP THEM AS SHORT AS POSSIBLE!
 *
 **********************************************************************************/

/**
 * @brief calls CAN interrupt container for 1ms- and 10ms- processings
 *
 * Called by system's 1ms interrupt
 *
 *  @date   02.04.2012
 *  @author Markus Holzapfel
 */
void CANTIMER_Can1msTimer_Int(void)
{
   Can1msTimer_Int();
   
   u84msTimer++;

   /* generation of 10 msec timer */
   if ((TimC.Timer10ms >= 1))
   {
      TimC.Timer10ms--;
      if ((TimC.Timer10ms == 0))
      {
         Can10msTimer_Int();
         TimC.Timer10ms = TIMER_10_MS;
      }
   }
   
   if((4 == u84msTimer) &&  bIPC_O5_Ready)
   {
       /** send IPC Router event */
       CANTASK_IpcRouter_Event();
       u84msTimer = 0;
   } 
}

/**
 * @brief cyclic processings of CAN within 1ms-interrupt
 *
 *  @date   02.04.2012
 *  @author Markus Holzapfel
 */
static void Can1msTimer_Int(void)
{
   if ( ( flagFirstMessageTransmittedAfterStartup == 0 ) && ( flagTraceMsgOnce == 0 ) )
   {
      // Count elapsed ms after first Tx (until the first time has been sent by the IOC-Trace)
      msAfterFirstTx++;

   }
   Flag--;
   if (Flag == 0)
   {
      TpTxTask();                            // cyclic call of TP task
      TpRxTask();                            // cyclic call of TP task
      Flag = 5;
   }
}

/**
 * @brief cyclic processings of CAN within 10ms-interrupt
 *
 *  @date   02.04.2012
 *  @author Markus Holzapfel
 */
static void Can10msTimer_Int(void)
{
   if (TimC.Timer20ms)
   {
      if (/* 20ms have elapsed */
            0 == --TimC.Timer20ms)
      {
         /* send a EVENT_CAN_RUN to process the CAN State machine cyclically*/
         CANTASK_RunEvent();
         TimC.Timer20ms = TIMER_20_MS_10MS;
      }
   }
   CANTASK_TimerEvent();
}

/********************************************************************************
 *
 * TIMER FUNCTIONS CALLED WITHIN TASK CONTEXT
 *
 *******************************************************************************/


/**
 * @brief cyclic processing of application specific functions function is called every 10 msec within task context
 *
 *  @date   02.04.2012
 *  @author Markus Holzapfel
 */
void CanTaskCyclic10ms(void)
{
   appTaskTimer10();       // Application specific processings (every 10 msec)

   /* Check timeouts */
   if (TimC.appTimer50ms)
   {
      TimC.appTimer50ms--;
   }
   else
   {
      appTaskTimer50();    // Application specific processings (every 50 msec)
      TimC.appTimer50ms = TIMER_50_MS_10MS-1;
   }
   if (TimC.appTimer100ms)
   {
      TimC.appTimer100ms--;
   }
   else
   {
      appTaskTimer100();   // Application specific processings (every 100 msec)
      TimC.appTimer100ms = TIMER_100_MS_10MS-1;
   }
   if (TimC.appTimer1000ms)
   {
      TimC.appTimer1000ms--;
   }
   else
   {
      appTaskTimer1000();  // Application specific processings (every 1000 msec)
      TimC.appTimer1000ms = TIMER_1000_MS_10MS-1;
   }
}

/**
 * @brief Project-specific function for cyclic (10ms) operations
 *
 *  @date   02.04.2012
 *  @author Markus Holzapfel
 */
static void appTaskTimer10(void)
{
   // Cyclic calls to 3rd party tasks (NM, TP, ...)
   #if TP_ACTIVE == CAN_ON
   #endif

   #if CAN_NM_ACTIVE == CAN_ON
   NmBasicTask();                         // I-CAN   (CAN_0)   Netzwork management cycle

   #if (CAN_NM_TYPE == 1)   // OSEK
   NmTask(CAN_1);                         // MM-CAN  (CAN_1)   Netzwork management cycle
   NmTask(CAN_2);                         // P2P-CAN (CAN_2)   Netzwork management cycle
   #elif (CAN_NM_TYPE == 2) //  AutoSar
   Nm_MainFunction(CAN_0);
   CanNm_MainFunction(CAN_0);
   #endif // CAN_NM_TYPE
   #endif // CAN_NM_ACTIVE == CAN_ON
   
  
//MHO    checkTxObjPendingTimout();

   // Busoff cyclic handler
// TODO:   CheckCyclicBusOffTimer();

   if ( ( flagFirstMessageTransmittedAfterStartup == 1 ) && ( flagTraceMsgOnce == 0 ) )
   {
      TRACE_VALUE(CAN_STATUS, MOD_CAN, "First CAN message has been sent %d ms ago", msAfterFirstTx);
      flagTraceMsgOnce = 1; // stop tracing & stop counting
   }

   /* Application message transmission implementation for NAD_WAKEUP */
   if ( ( b_nadFlag       == TRUE  ) && \
        ( b_CANWakeupFlag == FALSE ) )
   {
      /* After NAD Wakeup 80ms wait time */
      CANWakeupTimer--;
   }
   if( ( CANWakeupTimer  == CAN_WAKEUP_THRESHOLD ) && \
       ( b_CANWakeupFlag == FALSE                ) && \
       ( CAN_TXEnable    == FALSE                ) )
   {
      CAN_TXEnable = TRUE;
      b_CANWakeupFlag = TRUE;
      b_nadFlag = FALSE;
      CANCYC_StartCycTx( CAN_0 );
      CANCYC_StartCycTx( CAN_1 );
      CANCYC_StartCycTx( CAN_2 );
   }
}

/**
 * @brief Project-specific function for cyclic (50ms) operations
 *
 *  @date   02.04.2012
 *  @author Markus Holzapfel
 */
static void appTaskTimer50(void)
{
    /* Retry pending DSI TP Rx indications */
    cyclicRxIndRetryProcessing();
}

/**
 * @brief Project-specific function for cyclic (100ms) operations
 *
 *  @date   02.04.2012
 *  @author Markus Holzapfel
 */
static void appTaskTimer100(void)
{
   UINT8 CanNum;

   // cylic call of statemachine relevant timers
   cyclic_statemachine_timer_call_100ms();

   appNM_DTC_check();
   appPreboot_Wakeup_check();
   // cyclic check of NM Limphome state for CAN_1 and CAN_2
   for(CanNum = CAN_1; CanNum < ANZ_CAN; CanNum++)
   {
      if (/* NM is in limphome state */
          NmStateLimphome(NmGetStatus(CanNum)) )
      {
         setLimphomeActive(CanNum);
      }
      else
      {
         setLimphomeInactive(CanNum);
      }
   }

   // cyclic check of Error Pin of MM-CAN
   if ( /* Error Pin of MM-CAN is active, i.e.low */
        CAN2_ERRQ_ISACTIVE && appNmIsInActiveState(CAN_1) )
   {
      setSingleLineErrorActive();
   }
   else
   {
      setSingleLineErrorInactive();
   }

   printIrqTraceFlagsMessages();
   if(flagToStartClrTimer == TRUE)
   {
       clearValidMsgRecvdFlag();
   }
}
void clearValidMsgRecvdFlag()
{
    if(tmpTimer < CLEAR_FLAG_TIMER)
	{
		tmpTimer++;
	}
	else
	{
		setValidMsgReceivedFlag(1,0);
		tmpTimer = 0;
	}
}

/**
 * @brief Project-specific function for cyclic (1000ms) operations
 *
 *  @date   02.04.2012
 *  @author Markus Holzapfel
 */
static void appTaskTimer1000(void)
{
   if (modifCan0 != 0)
   {
      if (reqCan0 == 0)
      {
         (void)releaseCanBus(CTRL_HANDLE_1_ONOFF, CAN_0);
      }
      else
      {
         (void)requestCanBus(CTRL_HANDLE_1_ONOFF, CAN_0);
      }
   }
   if (modifCan1 != 0)
   {
      if (reqCan1 == 0)
      {
         (void)releaseCanBus(CTRL_HANDLE_1_ONOFF, CAN_1);
      }
      else
      {
         (void)requestCanBus(CTRL_HANDLE_1_ONOFF, CAN_1);
      }
   }
   if (modifCan2 != 0)
   {
      if (reqCan2 == 0)
      {
         (void)releaseCanBus(CTRL_HANDLE_1_ONOFF, CAN_2);
      }
      else
      {
         (void)requestCanBus(CTRL_HANDLE_1_ONOFF, CAN_2);
      }
   }
}

static void appNM_DTC_check(void)
{

   UINT8 nm_node;
   UInt8 nm_status;
   static UINT8 CanNM_timer[NM_MAX_NODE] = {TIMER_5000_MS_100MS,TIMER_5000_MS_100MS,TIMER_5000_MS_100MS,TIMER_5000_MS_100MS,TIMER_5000_MS_100MS,TIMER_5000_MS_100MS,TIMER_5000_MS_100MS};
   static bool dtc_set[NM_MAX_NODE];
   static UINT8 CAN_NM_dtc_enable_timer = TIMER_15000_MS_100MS;/* This variable will be made as 0 after 15s from first message is transmitted*/

   if((NM_DTC_ENABLE_TIMER_EXPIRED != CAN_NM_dtc_enable_timer) && (TRUE == canInitialized))
   {
      CAN_NM_dtc_enable_timer--;
   }

   nm_status = CANNM_Node_Get();
   CANNM_Node_Reset();

   /* Cyclic check for NM  related DTC*/
   /* Check DTC for all the nodes*/
   for (nm_node = NM_MIN_NODE; nm_node < MAX_NMNODE_CAN_1 ; nm_node++)
   {
	 if(nm_status & (NM_NODE_SET<<nm_node))
	 {
		CanNM_timer[nm_node] = TIMER_5000_MS_100MS;
		if (dtc_set[nm_node])
		{
		   /*Clear DTC*/
		   dtc_set[nm_node] = FALSE;
		   setNmDTCInactive(nm_node);
		}
	 }
	 else
	 {
         if( (NM_DTC_ENABLE_TIMER_EXPIRED == CAN_NM_dtc_enable_timer) &&
             (NM_TIMER_EXPIRED == CanNM_timer[nm_node]) &&
             (FALSE == isCanRxNM_Timeout_Enabled()) &&
             (PRE_BOOT_WAKEUP_ACC_IGN == preboot_Wakeup_Signal_api()) &&
             (!dtc_set[nm_node])
           )
		{
		   /*Set DTC*/
		   dtc_set[nm_node] = TRUE;
		   setNmDTCActive(nm_node);
		}

		if(NM_TIMER_EXPIRED != CanNM_timer[nm_node])
		{
		   CanNM_timer[nm_node] --;
		}
	 }
   }

   /* Cyclic check for NM  related DTC*/
   /* Check DTC for all the nodes*/
   for (nm_node = MAX_NMNODE_CAN_1; nm_node < NM_MAX_NODE ; nm_node++)
   {
	 if(nm_status & (NM_NODE_SET<<nm_node))
	 {
		CanNM_timer[nm_node] = TIMER_5000_MS_100MS;
		if (dtc_set[nm_node])
		{
		   /*Clear DTC*/
		   dtc_set[nm_node] = FALSE;
		   setNmDTCInactive(nm_node);
		}
	 }
	 else
	 {
         if( (NM_DTC_ENABLE_TIMER_EXPIRED == CAN_NM_dtc_enable_timer) &&
             (NM_TIMER_EXPIRED == CanNM_timer[nm_node]) &&
             (FALSE == isCanRxApplication_Timeout_Enabled())&&
             (PRE_BOOT_WAKEUP_ACC_IGN == preboot_Wakeup_Signal_api()) &&
             (!dtc_set[nm_node]) )
		{
		   /*Set DTC*/
		   dtc_set[nm_node] = TRUE;
		   setNmDTCActive(nm_node);
		}
		if(NM_TIMER_EXPIRED != CanNM_timer[nm_node])
		{
		   CanNM_timer[nm_node] --;
		}
	 }
   }

}

static void appPreboot_Wakeup_check()
{
   static UINT8 WakeUpSignal = 0xFF;
   static bool preboot_WakeUp_NMReq = FALSE;
   static bool b_Ican_TxStart = FALSE;
   static bool b_Ican_TxStop = TRUE;

   if(WakeUpSignal != preboot_Wakeup_Signal_api())
   {
      TRACE_VALUE(CAN_STATUS, MOD_CAN, "preboot wakeup signal value from can %d ", preboot_Wakeup_Signal_api());
      WakeUpSignal = preboot_Wakeup_Signal_api();
   }

   if ((PRE_BOOT_WAKEUP_ACC_IGN == preboot_Wakeup_Signal_api()) && (CAN_TXEnable== FALSE))
   {
      CAN_TXEnable = TRUE;
      TRACE_VALUE(CAN_STATUS, MOD_CAN, "CAN  Send application Message for ACC/IGN TXEnable value  %d ", CAN_TXEnable);
      (void)requestCanBus(CTRL_HANDLE_1_ONOFF, CAN_0);
      (void)requestCanBus(CTRL_HANDLE_1_ONOFF, CAN_1);
      (void)requestCanBus(CTRL_HANDLE_1_ONOFF, CAN_2);
      CANCYC_StartCycTx(CAN_1);
      CANCYC_StartCycTx(CAN_2);
      CAN_TXDisable = FALSE;      
   }
   else
   {
      if (TRUE == preboot_NM_Request_api())
      {
         if(PRE_BOOT_WAKEUP_ACC_IGN == preboot_Wakeup_Signal_api())
         {
            sysRemoveSingleTimer(TIMERID_CAN_STP_Release);
         }
         else
         {
            if ((CAN_TXEnable == FALSE) &&(!preboot_WakeUp_NMReq))
            {
               CAN_TXEnable = TRUE;
               TRACE_VALUE(CAN_STATUS, MOD_CAN, "CAN Send application Message for B+ Wakeup signals TXEnable value  %d ", CAN_TXEnable);
               (void)requestCanBus(CTRL_HANDLE_1_ONOFF, CAN_0);
               (void)requestCanBus(CTRL_HANDLE_1_ONOFF, CAN_1);
               (void)requestCanBus(CTRL_HANDLE_1_ONOFF, CAN_2);
               CANCYC_StartCycTx(CAN_1);
               CANCYC_StartCycTx(CAN_2);
               CAN_TXDisable = FALSE;
               sysSetSingleEventTimer(TIMERID_CAN_STP_Release, PRE_BOOT_STOP_TIME_STP, TaskID_CAN, EVENT_CAN_DISABLE_CANBUS_STP);
               preboot_WakeUp_NMReq = TRUE;
            }
         }
      }
   }

   /*Check IGN on to start the I_CAN Tx Messages*/
   if (IGN_ON == b_C_IGNSW_b)
   {
      if ((!b_Ican_TxStart)&&(TRUE == appNmIsInActiveState(CAN_0) ))
      {
         TRACE_TEXT(CAN_STATUS, MOD_CAN, "CAN Send application Message for I-CAN Since IGN is On");
         CANCYC_StartCycTx(CAN_0);
         b_Ican_TxStop  = TRUE;
         b_Ican_TxStart = TRUE;
      }
    }
   else
   {
      if(b_Ican_TxStop)
      {
         TRACE_TEXT(CAN_STATUS, MOD_CAN, "CAN Stop application Message for I-CAN Since IGN is Off");
         CANCYC_StopCycTx(CAN_0);
         b_Ican_TxStop = FALSE;
         b_Ican_TxStart = FALSE;
      }
   }
}

void CAN_ApplicationMsgStart (void)
{
   requestCanBus(CTRL_HANDLE_1_ONOFF, CAN_0);
   requestCanBus(CTRL_HANDLE_1_ONOFF, CAN_1);
   requestCanBus(CTRL_HANDLE_1_ONOFF, CAN_2);
   b_nadFlag = TRUE;
   CAN_TXDisable = FALSE;/*BugFix:1608340*/
}

void stopCan_Wakeup_STP(void)
{
   if(PRE_BOOT_WAKEUP_ACC_IGN == preboot_Wakeup_Signal_api())
   {
      sysRemoveSingleTimer(TIMERID_CAN_STP_Release);
   }
   else
   {
      if(CAN_TXDisable == FALSE)
      {
         TRACE_TEXT(TRACE_SWITCH_ALWAYS, MOD_ALWAYS, "[CAN] Stop the CAN trasmission");
         CANCYC_StopCycTx(CAN_0);
         CANCYC_StopCycTx(CAN_1);
         CANCYC_StopCycTx(CAN_2);

         //Release all three CAN busses
         (void)releaseCanBus(CTRL_HANDLE_1_ONOFF, CAN_0);
         (void)releaseCanBus(CTRL_HANDLE_1_ONOFF, CAN_1);
         (void)releaseCanBus(CTRL_HANDLE_1_ONOFF, CAN_2);
         CAN_TXEnable = FALSE;
         CAN_TXDisable = TRUE;
      }
   }
}

