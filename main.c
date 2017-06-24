/******************************************************************************
 * (c) Copyright 2017 Schindler
 *-----------------------------------------------------------------------------
 * File Name       : main.c
 * Project         : SWIM-SW
 * File Description:  
 * Update Software for STM8 chip thru SWIM protocol
 *-----------------------------------------------------------------------------
 * All rights reserved.
 *-----------------------------------------------------------------------------
 +-------------+----------+------------+--------------------------------------+
 | Version     | Date     | Author     | Description                          |
 +-------------+----------+------------+--------------------------------------+
 |V0001        |04-14-2016|Richard Sha | First creation                       |
 |             |          |            |                                      |
 |             |          |            |                                      |
 |             |          |            |                                      |
 |             |          |            |                                      |
 |             |          |            |                                      |
 |             |          |            |                                      |
 +-------------+----------+------------+--------------------------------------+
*****************************************************************************/
/* ---------------------------------------------------------------------------
INCLUDE FILES
--------------------------------------------------------------------------- */
#include "srec_os_PEM6xHW.h"
#include "typedefs.h"
#include "MPC5744P.h"
#include "psim_sys.h"
#include "srec_os_pwsbc.h"
#include "stmcu.h"
#include "psim_io.h"
#include "psim_sys.h"
#include "psim_control.h"
#include "psim_timer.h"
#include "psim_menu.h"

/* ---------------------------------------------------------------------------
TYPE DEFINITION
--------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
CONSTANT DEFINITION
--------------------------------------------------------------------------- */
__attribute__((section(".Software_Nr")))
const UINT8 K_SWNR[32] =  
    {'X','1','2','3','4','5','6','_','9','9',' ',' ',' ',' ',' ',
     'G','6','0','0','2',                        //Software number
     0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,    //Free memory
     0xFF,0xFF,                                  //Free memory
     0xFF,0xFF};                                 //Free memror


/* ---------------------------------------------------------------------------
MACRO DEFINITION
--------------------------------------------------------------------------- */
#define ST_SWIM_INIT          0
#define ST_SWIM_WAIT          1
#define ST_SWIM_PROGRAM       2
#define ST_SWIM_SUCCESS       3
#define ST_SWIM_FAULT         4
#define ST_SWIM_TEST          100
#define ST_SWIM_START         0xFF


#define TIME_1S               40000000  // 10MHz, 0.025us*400000 = 1s

#define TIME_100MS            4000000   // 10MHz, 0.025us*40000 = 0.1s

#define SIUL2_SPI_SSPL     (*(volatile struct SIUL2_SSPL_tag *) 0xFFFC132CUL)
#define HW_OUT_SSINPL      SIUL2_SPI_SSPL.GPDO_SSPL.R


/* ---------------------------------------------------------------------------
VARIABLE DEFINITION
--------------------------------------------------------------------------- */
UINT8 h_ucdata1_;
UINT8 h_ucdata2_;
UINT8 h_ucdata3_;
UINT8 h_ucdisplay_idx = 0;



/******************************************************************************
 * Function name: fn_Swim_memcpy
 * ---------------------------------------------------------------------------
 * Description: 
 * memory copy
 *
 * ---------------------------------------------------------------------------
 * Invoking function:
 * None
 *
 * ---------------------------------------------------------------------------
 * Global/Private Variable:
 * RW          Name        Description
 * ---------------------------------------------------------------------------
 * None
 *
 * ---------------------------------------------------------------------------
 * Interface Variable:
 * Direction   Name        Description
 * ---------------------------------------------------------------------------
 * OUT         i_puldestaddr   dest address
 * IN          i_pulsrcaddr    source address
 * IN          i_ulsize        size of the data
 * 
 * ---------------------------------------------------------------------------
 * Return value:
 * 1             - finished
 *
 *****************************************************************************/

UINT32 fn_Swim_memcpy(void*i_puldestaddr,void const* i_pulsrcaddr,UINT32 i_ulsize)
{
  UINT32* dest=i_puldestaddr;
  UINT32 const* src=i_pulsrcaddr;
  while(i_ulsize-->0)
  {
    *dest++ = *src++;
  }
  return 1;
}
/******************************************************************************
 * Function name: fn_Swim_SPI_init
 * ---------------------------------------------------------------------------
 * Description: 
 * Initializes the SPI
 *
 * ---------------------------------------------------------------------------
 * Invoking function:
 * None
 *
 * ---------------------------------------------------------------------------
 * Global/Private Variable:
 * RW          Name        Description
 * ---------------------------------------------------------------------------
 * None
 *
 * ---------------------------------------------------------------------------
 * Interface Variable:
 * Direction   Name        Description
 * ---------------------------------------------------------------------------
 * None
 * 
 * ---------------------------------------------------------------------------
 * Return value:
 * None
 *
 *****************************************************************************/
__attribute__ ((section (".srec_rom")))
void fn_Swim_SPI_init(void)
{
  HW_OUT_SINKOUTON = HW_PIN_ON;           
   
  SIUL2.MSCR[SIUL_PC14].R = SIUL_OUTPUT | SIUL_GPIO;    // SPI1_SSPL_DO  ->OUT
  SIUL2.MSCR[SIUL_PC13].R = SIUL_OUTPUT | SIUL_GPIO;    // SPI3_SSPL_DO ->OUT
  //SIUL2.GPDO[SIUL_PC14].R = 0;   // Input PL->L, load inputs to shift registers
  //SIUL2.GPDO[SIUL_PC13].R = 0;    // Input PL->L, load inputs to shift registers
  HW_OUT_SSINPL &= ~0x00010100;
    
  //fn_Srec_dspi_ssio_init();
}


void fn_Test(void)
{

    if (fn_Psim_TimerIsOut(TIME_100us_01))
    {
      //fn_Psim_TimerSet(TIME_100us_01, 20000); // 2s
      fn_Psim_TimerSet(TIME_100us_01, 2500); // 250ms
      h_ucdisplay_idx++;
      /*if (h_ucdisplay_idx >= 3)
      {
        h_ucdisplay_idx = 0;
      }*/
      if (h_ucdisplay_idx&0x01)
      {
        SIUL2.GPDO[152].R = 1;  // LED

      }
      else
      {
        SIUL2.GPDO[152].R = 0;  // LED
      }
    }
      /*switch(h_ucdisplay_idx)
      {
        case 0:
          h_ucdata1_ = (sSpeedValue.ulSpeed%1000)/100;
          h_ucdata2_ = (sSpeedValue.ulSpeed%100)/10;
          h_ucdata3_ = sSpeedValue.ulSpeed%10;
          fn_Psim_setdisplay(0,appl_ascii_value('r'));   // motor, 

          break;

        case 1:
          h_ucdata1_ = (sSpeedValue.ulStep%1000)/100;
          h_ucdata2_ = (sSpeedValue.ulStep%100)/10;
          h_ucdata3_ = sSpeedValue.ulStep%10;
          fn_Psim_setdisplay(0,appl_ascii_value('t'));   // step, 

          break;
        case 2:
          h_ucdata1_ = (sSpeedValue.ulHanrail%1000)/100;
          h_ucdata2_ = (sSpeedValue.ulHanrail%100)/10;
          h_ucdata3_ = sSpeedValue.ulHanrail%10;
          fn_Psim_setdisplay(0,appl_ascii_value('h'));   // handrail, 
          break;

      }


    fn_Psim_setdisplay(1,appl_digvalue_anz[h_ucdata1_]);
    fn_Psim_setdisplay(2,appl_digvalue_anz[h_ucdata2_]);
    fn_Psim_setdisplay(3,appl_digvalue_anz[h_ucdata3_]);*/

}
/******************************************************************************
 * Function name: main()
 * ---------------------------------------------------------------------------
 * Description: 
 * This is the main rountine of the SWIM. It will be directly executed after the 
 * initialization.
 * ---------------------------------------------------------------------------
 * Invoking function:
 * fn_Swim_init
 * SIUL_DigitalInputSimple
 * fn_Swim_SPI_init
 * fn_Srec_pwsbc_init
 * fn_Srec_pwsbc_changewdwindow
 * fn_Swim_memcpy
 * SIUL_GetPadState
 * fn_Swim_Programming
 * fn_Swim_delay
 *
 *
 * ---------------------------------------------------------------------------
 * Global/Private Variable:
 * RW          Name        Description
 * ---------------------------------------------------------------------------
 * RW         h_ulSwim_state SWIM state machine
 *
 * ---------------------------------------------------------------------------
 * Interface Variable:
 * Direction   Name        Description
 * ---------------------------------------------------------------------------
 * None
 * ---------------------------------------------------------------------------
 * Return value:
 * None
 *****************************************************************************/
__attribute__((section(".srec_rom"))) 
void main(void) 
{
  fn_Swim_init();


  SIUL2.MSCR[152].B.OBE = 1;   // PE13: output driver enabled 
  SIUL2.GPDO[152].R = 1;  // LED
  
  fn_Swim_SPI_init();
  
  // init WD of MC33907
  fn_Srec_pwsbc_init();
  fn_Srec_pwsbc_changewdwindow(WD_DISABLE);

  fn_stuart_config(ST_BAUD_57600, 1, ST_PARITY_EVEN);
  fn_Psim_io_init();
  fn_Psim_timer_init();
  fn_Psim_SpeedCtl_init();
  
  while (1)
  {
    fn_SIM_stcom_proc();
    fn_Psim_Button_read();

    //fn_Psim_SpeedAccDec();
    fn_Psim_Speed_Mng();
    fn_Psim_SelectMenu();
    fn_Test();
  }

}
/* -------------------------------------------------------------------------*/
/* File End                                                                 */
/* ------------------------------------------------------------------------ */
