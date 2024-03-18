/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
#include "adc.h"
#include "sci.h"
#include "het.h"
#include "rti.h"
#include "stdio.h"
#include "stdint.h"
#include <math.h>
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
void rtiNotification(uint32 notification);
void wait(uint32);
void UART_Display_Text(sciBASE_t *, uint32, uint8 *);
void My_pwmSetSignal(hetRAMBASE_t * , uint32 , hetSIGNAL_t );
int Average (int );

#define UART scilinREG
adcData_t adc_data[1];
uint32 ch_count;
int value;
hetSIGNAL_t pwm0_het20_Servo;
static const uint32 s_het1pwmPolarity[8U] = { 3U, 3U, 3U, 3U, 3U, 3U, 3U, 3U, };
uint8_t Buffer[100];
float distance =0.0;
const int Init_duty = 80;
int Length_buffer=0;
float Kp =0.95, Ki=0.0, Kd=0.01;
float integral=0.0, derivative=0.0;
float error_k=0.0, error_k_1=0.0;
float delta_t= 0.05, ref=10.0;
float PID_signal=0.0;
int PID_duty=0;
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    // L= 15cm ---> adc : 1058
    // L= 20 cm --->adc : 820
    // L= 7 cm  --> adc: 2058

    //Lower limit duty --> 30
    //Upper limit duty --> 130

    adcInit();
    sciInit();
    hetInit();
    rtiInit();

    rtiEnableNotification(rtiNOTIFICATION_COMPARE0);
    _enable_IRQ();
    rtiStartCounter(rtiCOUNTER_BLOCK0);


    pwm0_het20_Servo.period = 20E3;
    pwm0_het20_Servo.duty = 0;
    int raw_value;
    float a= 42203.11278, b= -1.140880872;

    while(1){
        raw_value = Average(30);
        distance = a * powf(raw_value, b);
        PID_duty = (int)PID_signal + Init_duty;
        if (PID_duty > 130){
            PID_duty=130;
        }
        else if(PID_duty<30){
            PID_duty=30;
        }
        pwm0_het20_Servo.duty = PID_duty;
        My_pwmSetSignal(hetRAM1, 0, pwm0_het20_Servo);

        Length_buffer=sprintf(Buffer, "Distance: %f PID signal + comp : %d\n", distance, (int)PID_signal+ Init_duty);
        UART_Display_Text(UART, Length_buffer, Buffer);
        wait(1000);
    }
/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
void rtiNotification(uint32 notification)
{
    error_k= ref - distance;
    integral = integral + error_k *delta_t;
    derivative= (error_k - error_k_1)/delta_t;
    PID_signal = Kp* error_k + Ki * integral + Kd * derivative;
    error_k_1= error_k;
}

int Average (int num){
    int sum= 0;
    int i;
    for (i=0; i< num ; i++){
        adcStartConversion(adcREG1,adcGROUP1);
        while((adcIsConversionComplete(adcREG1,adcGROUP1)==0));
        ch_count=adcGetData(adcREG1,adcGROUP1,&adc_data[0]);
        sum = sum + (int)adc_data[0].value;
    }

    return (sum/num);
}
void wait(uint32 time)
{
    while(time){time--;};
}

void UART_Display_Text(sciBASE_t * UART_, uint32 length, uint8 * Data){
      sciSend( UART_, length, (uint8 *) Data);
      sciSend( UART_, (uint32) 2, (uint8 *)"\r\n");
}

void My_pwmSetSignal(hetRAMBASE_t * hetRAM, uint32 pwm, hetSIGNAL_t signal)
{
    uint32 action;
    uint32 pwmPolarity = 0U;
    float64 pwmPeriod = 0.0F;

    if(hetRAM == hetRAM1)
    {
        pwmPeriod = (signal.period * 1000.0F) / 800.000F;
        pwmPolarity = s_het1pwmPolarity[pwm];
    }
    else
    {
    }
    if (signal.duty == 0U)
    {
        action = (pwmPolarity == 3U) ? 0U : 2U;
    }
    else if (signal.duty >= 1000U)
    {
        action = (pwmPolarity == 3U) ? 2U : 0U;
    }
    else
    {
        action = pwmPolarity;
    }

    hetRAM->Instruction[(pwm << 1U) + 41U].Control = ((hetRAM->Instruction[(pwm << 1U) + 41U].Control) & (~(uint32)(0x00000018U))) | (action << 3U);
    hetRAM->Instruction[(pwm << 1U) + 41U].Data = ((((uint32)pwmPeriod * signal.duty) / 1000U) << 7U ) + 128U;
    hetRAM->Instruction[(pwm << 1U) + 42U].Data = ((uint32)pwmPeriod << 7U) - 128U;

}

/* USER CODE END */
