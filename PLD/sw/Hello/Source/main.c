/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 2014 - 2019 SEGGER Microcontroller GmbH             *
*                                                                    *
*           www.segger.com     Support: support@segger.com           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* conditions are met:                                                *
*                                                                    *
* - Redistributions of source code must retain the above copyright   *
*   notice, this list of conditions and the following disclaimer.    *
*                                                                    *
* - Neither the name of SEGGER Microcontroller GmbH                  *
*   nor the names of its contributors may be used to endorse or      *
*   promote products derived from this software without specific     *
*   prior written permission.                                        *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED.                                                        *
* IN NO EVENT SHALL SEGGER Microcontroller GmbH BE LIABLE FOR        *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "ibex_core.h"

/*********************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
*/

#define tmInt   0x1
#define uartInt 0x2

uint32_t x,i, pullInt;
uint8_t messege[] = "Super Puper Word!!! ";
uint8_t sendEV0[] = " ev0 = ";
uint8_t sendEV1[] = " ev1 = ";
uint8_t ISR_uart;
uint32_t ISR_tm;
uint8_t str[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t rpm_min = 100;
uint8_t rpm_max = 245;



uint8_t minU = 0x92;
uint8_t maxU = 0xFE;
uint8_t goalRPM_left  = 100;
uint8_t goalRPM_Right = 100;
uint32_t cnt_Left  = 0;
uint32_t cnt_Right = 0;
uint32_t drive = 0;

float Kp = 1; // Коэффициенты П, И и Д - звеньев
float Ki = 0;
float Kd = 0;

float integral = 0;
float pred_err = 0;
float min_integral = -100.0;
float max_integral = 100.0;
float error;
float Y,U;

float Krpm = 6.0006;
float tic = 0.03333;
uint32_t min = 60000000;
uint32_t rpm,rpm1;


void ComSendInt(uint8_t * text, uint32_t value){
  uint8_t i;
  uint8_t buf[10];
  sprintf(buf, "%d", value);
  for(i = 0; i < strlen(text); i++) UART0_REG(dFIFOtx_UART) = text[i];
  for(i = 0; i < strlen(buf);  i++) UART0_REG(dFIFOtx_UART) = buf[i];
  UART0_REG(dFIFOtx_UART) = 0x0A;
}

void ComSendFl(uint8_t * text, float value){
  uint8_t i;
  uint8_t buf[10];

  int tmpInt1 = value;
  float tmpFrac = value - tmpInt1;
  if(tmpFrac < 0) tmpFrac = tmpFrac*(-1);
  int   tmpInt2 = tmpFrac * 100;

  sprintf(buf, "%d.%d", tmpInt1, tmpInt2);
  for(i = 0; i < strlen(text); i++) UART0_REG(dFIFOtx_UART) = text[i];
  for(i = 0; i < strlen(buf); i++)  UART0_REG(dFIFOtx_UART)  = buf[i];
  UART0_REG(dFIFOtx_UART) = 0x0A;
}

void print_to_LED8x8(int value){
        uint8_t len;
        sprintf(str, "%d", value);
        len = strlen(str);
        for(i = 0; i < 8; i++)  {
          if(i > len) IO_REG8(0x4 + i) = 0;
          else IO_REG8(0x4 + i) = str[len - i - 1];
        }
}

float Eval(float err)
{ 
  float y;

  //ComSendFl("err = ",err);

  integral = integral + err; // добавить ошибку в сумму ошибок
  if(integral>max_integral) integral=max_integral;
  if(integral<min_integral) integral=min_integral; 
  float rdiff = Kd*(err - pred_err);
  // вычисление управляющего воздействия
  y = (Kp*err + Ki*integral + rdiff);
  pred_err = err; // текущая ошибка стала "прошлой ошибкой"
  // для след. вычисления
  return y;
}

float y2u(float y)
{
  float u;
  float maxY;
  float minY;

  maxY = Kp*rpm_max + Ki*max_integral + Kd*rpm_max;
  minY = 0;
  u = (y-minY)/(maxY-minY)*(maxU-minU)+minU;

  if(u>maxU) u = maxU;
  if(u<minU) u = minU;
  return u;
}


/*
void handle_UART_interrupt()
{
  uint8_t ISR, RxCntL, n, data; 

  if(ISR & 0x1){  //Rx data availble
    RxCntL = UART0_REG(dRxCntL_UART);
    for (n = 0; n < RxCntL; n++) data = UART0_REG(dFIFOrx_UART);
  }
  else if(ISR & 0x2){
    int32_t f = strlen(messege);
    for(i = 0; i < f; i++) UART0_REG(dFIFOtx_UART) = messege[i]; 
  }

}
*/

/*
void handle_TIMER_interrupt(unsigned ticks)
{
   //Write period val
 Timer_REG(dTmPrSh) = ticks;

 //Set Timer Run Bit
 Timer_REG(dTRS) = 0x1;

  //Send Hello word
 int32_t f = strlen(messege);
 for(i = 0; i < f; i++) UART0_REG(dFIFOtx_UART) = messege[i]; //strlen(messege)

}
*/

uintptr_t 
handle_trap(uintptr_t mcause, uintptr_t epc)
{
    if (mcause == 0x80000003){ //UART interrupt
      ISR_uart = UART0_REG(dISR_UART);
      pullInt  = pullInt | uartInt;
      //print_to_LED8x8(55);
      //IO_REG32(0x8) = pullInt; 
      //IO_REG32(0x8) = ISR_uart;
    }
    else if (mcause == 0x80000007){ //Timer interrupt
      ISR_tm = ISR_tm | Timer_REG(dISR);
      Timer_REG(dISC) = ISR_tm;  
      pullInt  = pullInt | tmInt;
      //IO_REG32(0x8) = pullInt;
      //IO_REG32(0x8) = ISR_tm;
    }
  return epc;
}

void init_timer(void) {
  uint32_t TMS;
  uint32_t CMC;
  uint32_t ECR;
  uint32_t IEC;

  Timer_REG(dTmPrSh) = 0xE4E1C0;//0x16E360;// 0x5F5E10;

  //Set Timer mode
  TMS = 0; //Set TCM Center aligned mode
  TMS = TMS | (0x1 << 2); //Capture Compare Mode
//  TMS = TMS | (0x1 << 9); //Capture Compare Mode
  Timer_REG(dTMS) = TMS; 

 //Connection Matrix Control
  CMC = 0;
  CMC = CMC | (0x1 << 18); //External Count to Capture0 reg triggered by Event 0
  CMC = CMC | (0x2 << 20); //External Count to Capture2 reg triggered by Event 1
  Timer_REG(dCMC) = CMC;

 //Event control
  ECR = 1;                 // Event 0 Edge Selection on rising edge
  ECR = ECR | (0x1 << 2);  // Event 1 Edge Selection on rising edge
  ECR = ECR | (0x3 << 9);  // Event 0 Low Pass Filter Configuration
  ECR = ECR | (0x3 << 11); // Event 1 Low Pass Filter Configuration

  Timer_REG(dECR) = ECR;

  //Set Interrupt 
  IEC = 0;
  IEC = IEC | tm_PMup; // set periode math interrupt
//  IEC = IEC | tm_Ev0DS | tm_Ev1DS | tm_PMup;
  Timer_REG(dIEC) =  IEC; 

}

void init_timer1(void) {
  uint32_t TMS,PLC;

  //set PWM out
  Timer1_REG(dTmPrSh)   = 0x1CE3;
  Timer1_REG(dTmCmpSh)  = 0xfe* 0x1d;
  Timer1_REG(dTmCmpSh1) = 0xfe* 0x1d;

  //Set Timer mode
  TMS = 0; //Set TCM Center aligned mode
  Timer1_REG(dTMS) = TMS;

  PLC = 0x3;
  Timer1_REG(dPLC) = PLC;
}

void init_intrupt(void) {

  // Clear interrupts
  clear_csr(mie, MIP_MSIP);
  clear_csr(mie, MIP_MTIP);

  // Enable the Machine-Timer bit in MIE
  set_csr(mie, MIP_MSIP);
  set_csr(mie, MIP_MTIP);

  // Enable interrupts in general.
  set_csr(mstatus, MSTATUS_MIE);
}

void init_UART(void) {
  //UART initialization
  uint8_t CR;

  CR = 0;
 // CR = CR | 0x01;  //Set Odd parity
 // CR = CR | (0x1 << 3); //set Rx Data Available Interrupt
 //CR = CR | (0x1 << 4); //set Tx Empty Interrupt
 //CR = CR | (0x1 << 5); //set Rx Error Status Interrupt
 //CR = CR | (1'b1 << 6); //Internal Loop TX to RX

  UART0_REG(dCR_UART)  = CR; //Set Odd parity
  UART0_REG(dDLL_UART) = 0;// BR9600 & 0xff;// BR9600;//  BR921600; //Set 
  UART0_REG(dDLH_UART) = 3;// BR9600;//  BR921600; //Set 
}

uint32_t PID(uint32_t cntR,uint8_t goalRPM){
  
  rpm =  Krpm*cntR;
  error = ((float) goalRPM) - ((float) rpm);
  Y = Eval(error);
  U = 1.33 * (Y + goalRPM );
  
  if(U > 254) U = 0xfe;
  else if(U < 0x60) U = 0x60;

  return ((int) U) * 0x1d;
}

void main(void) {
 
 uint32_t i, TmCap1mCap0;
  
 uint32_t len, cmd_val = 0;
 uint8_t RxCntL;
 uint8_t j,state = 0, cmd = 0, cmdRX = 0;
 
 //Timer initialization
  init_timer();
  init_timer1();
  init_intrupt();
  init_UART();
	
  //Run Timer's 
  Timer_REG(dTRS)  = 0x1;
  Timer1_REG(dTRS) = 0x1;

 for(i = 0; i < strlen(messege); i++) UART0_REG(dFIFOtx_UART) = messege[i]; //strlen(messege)

 while(1){  

  if(pullInt & tmInt){

    if(ISR_tm & tm_PMup){
      TmCap1mCap0 = Timer_REG(dTmCap1);
      Timer1_REG(dTmCmpSh) = PID(TmCap1mCap0,goalRPM_left);

      if(cnt_Left < TmCap1mCap0){
        cnt_Left = 0;
        drive    = drive & ~(0x4);
        IO_REG32(0x0) = drive;
      }
      else cnt_Left = cnt_Left - TmCap1mCap0;

      TmCap1mCap0 = Timer_REG(dTmCap3);
      Timer1_REG(dTmCmpSh1) = PID(TmCap1mCap0,goalRPM_Right);

      if(cnt_Right < TmCap1mCap0){
        cnt_Right = 0;
        drive    = drive & ~(0x8);
        IO_REG32(0x0) = drive;
      }
      else cnt_Right = cnt_Right - TmCap1mCap0;


      //if(UART0_REG(dFCR_UART) & 0x2){
       //ComSendInt(sendEV0,rpm);
       //ComSendInt(sendEV1,rpm1);
     // }
      

      ISR_tm =ISR_tm & ~tm_PMup; // clear PM interrupt
      pullInt = pullInt & (~tmInt); // clear timer interrupt pull flag
    }



  }
  else if (!(UART0_REG(dFCR_UART) & 0x8)){

    RxCntL = UART0_REG(dRxCntL_UART);
    print_to_LED8x8(RxCntL);

//    while(!(UART0_REG(dFCR_UART) & 0x8)){
//      str[0] = UART0_REG(dFIFOrx_UART);
//      UART0_REG(dFIFOtx_UART) = str[0];
//    }

    //print_to_LED8x8(RxCntL);
    //IO_REG32(0x8) = RxCntL;

    //for (j =0; j < RxCntL; j++) str[j] = UART0_REG(dFIFOrx_UART);
    //for(j = 0; j < RxCntL; j++) UART0_REG(dFIFOtx_UART) = str[j];
    //pullInt = pullInt & (~uartInt);
    //RxCntL = 0;

    //RX cmd
    for (j =0; j < RxCntL; j++){
      str[0] = UART0_REG(dFIFOrx_UART);
      UART0_REG(dFIFOtx_UART) = str[0];
      //IO_REG32(0x8) = str[0];
//      print_to_LED8x8(str[0]);

      switch(state)
      {
        case 0:
          if(str[0] ==  0x63) state = 1; // c
          else state = 0;
          //IO_REG32(0x4) = state;
	  break;
        case 1:
          if(str[0] == 0x6D) state = 2; // m
          else state = 0;
          //IO_REG32(0x4) = state;
	  break;
        case 2:
          if(str[0] == 0x64) state = 3; // d
          else state = 0;
          //IO_REG32(0x4) = state;
	  break;
        case 3:
          cmd =  str[0];
          state = 4;
          //IO_REG32(0x4) = state;
	  break;
        case 4:
          cmd_val =  str[0];
          cmdRX = 1;
          state = 0;
          //IO_REG32(0x4) = state;
	  break;
        default:
          state = 0;
          break;
      }
      //IO_REG32(0x8) = state;
    }
    if(UART0_REG(dRxCntL_UART) == 0) pullInt = pullInt & (~uartInt);
  }
  else if (cmdRX != 0){
    if(cmd == 0x1){ //enable Drive
      drive = cmd_val;
      IO_REG32(0x0) = drive;
      print_to_LED8x8(cmd_val);
      cmdRX = 0;
    }
    else if(cmd == 0x2){ //set rpm Left
      goalRPM_left  = cmd_val;
      print_to_LED8x8(cmd_val);
      cmdRX = 0;
    }
    else if(cmd == 0x3){ //set rpm Right
      goalRPM_Right = cmd_val;
      cmdRX = 0;
    }
    else if(cmd == 0x4){ //set count of left whell decoder
      cnt_Left = cmd_val; 
      cmdRX = 0;
    }
    else if(cmd == 0x5){ //set count of right whell decoder
      cnt_Right = cmd_val; 
      cmdRX = 0;
    }
    else{ //cmd unnow
      cmdRX = 0;
    }

  }

 /*
 for (i = 0; i < 4; i++) {
    IO_REG32(0) = i;
  }
*/
 }
}

/*************************** End of file ****************************/
