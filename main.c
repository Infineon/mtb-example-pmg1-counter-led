/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the PMG1 Counter LED Example
*                    for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
* Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include <stdio.h>
#include <inttypes.h>

/*******************************************************************************
* Macros
*******************************************************************************/
#define CY_ASSERT_FAILED          (0u)

/* Debug print macro to enable UART print */
#define DEBUG_PRINT               (0u)

/* Initialize the interrupt vector table with the timer interrupt handler address and assign priority. */
cy_stc_sysint_t intrCfg =
{
    /*.intrSrc =*/ CYBSP_COUNTER_IRQ,    /* Interrupt source is Timer interrupt */
    /*.intrPriority =*/ 3UL            /* Interrupt priority is 3 */
};

/*******************************************************************************
* Function prototype
*******************************************************************************/
void timer_interrupt_handler(void);

#if DEBUG_PRINT

/* Structure for UART Context */
cy_stc_scb_uart_context_t CYBSP_UART_context;

/* Variable used for tracking the print status */
volatile bool ENTER_LOOP = true;

/*******************************************************************************
* Function Name: check_status
********************************************************************************
* Summary:
*  Prints the error message.
*
* Parameters:
*  error_msg - message to print if any error encountered.
*  status - status obtained after evaluation.
*
* Return:
*  void
*
*******************************************************************************/
void check_status(char *message, cy_rslt_t status)
{
    char error_msg[50];

    sprintf(error_msg, "Error Code: 0x%08" PRIX32 "\n", status);

    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n=====================================================\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\nFAIL: ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, message);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, error_msg);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n=====================================================\r\n");
}
#endif

/*******************************************************************************
* Function Name: main
*******************************************************************************
* Summary:
* The main function performs the following actions:
*    1. Initializes the BSP
*    2. Calls the function to set up the TCPWM Counter
*    3. Calls the function to set up the interrupt on terminal count
*
* Parameters:
*  void
*
* Return:
*  int
*
******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_en_sysint_status_t intr_result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

#if DEBUG_PRINT

    /* Configure and enable the UART peripheral */
    result = Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* Sequence to clear screen */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\x1b[2J\x1b[;H");

    /* Print "PMG1 MCU: Counter LED" */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "****************** ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "PMG1 MCU: Counter LED");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "****************** \r\n\n");
#endif

    intr_result = Cy_SysInt_Init(&intrCfg, timer_interrupt_handler);
    if(intr_result != CY_SYSINT_SUCCESS)
    {
#if DEBUG_PRINT
    check_status("API Cy_SysInt_Init failed with error code", intr_result);
#endif
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Initialize the TCPWM component in timer/counter mode. The return value of the
     * function indicates whether the arguments are valid or not.
     */
    result = Cy_TCPWM_Counter_Init(CYBSP_COUNTER_HW, CYBSP_COUNTER_NUM, &CYBSP_COUNTER_config);
    if(result != CY_TCPWM_SUCCESS)
    {
#if DEBUG_PRINT
    check_status("API Cy_TCPWM_Counter_Init failed with error code", result);
#endif
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /*Enables the counter in the TCPWM block for the Counter operation*/
    Cy_TCPWM_Counter_Enable(CYBSP_COUNTER_HW, CYBSP_COUNTER_NUM);

    /* Check if the desired interrupt is enabled prior to triggering */
    if (0UL != (CY_TCPWM_INT_ON_TC & Cy_TCPWM_GetInterruptMask(CYBSP_COUNTER_HW, CYBSP_COUNTER_NUM)))
    {
       Cy_TCPWM_SetInterrupt(CYBSP_COUNTER_HW, CYBSP_COUNTER_NUM, CY_TCPWM_INT_ON_TC);
    }

    /* Enable Interrupt */
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Enable global interrupts */
    __enable_irq();

    /* Triggers a software start on the selected TCPWMs.
     * This is required when no other hardware input signal is connected to the component to act as
     * a trigger source. */
    Cy_TCPWM_TriggerStart(CYBSP_COUNTER_HW, CYBSP_COUNTER_MASK);

    for (;;)
    {

#if DEBUG_PRINT
        if (ENTER_LOOP)
        {
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Entered for loop\r\n");
            ENTER_LOOP = false;
        }
#endif

    }
}

/*******************************************************************************
* Function Name: Timer_Interrupt_Handler
********************************************************************************
* Summary:
* Handler function for the timer interrupt that toggles the LED.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void timer_interrupt_handler(void)
{
    /* Clear the terminal count interrupt */
    Cy_TCPWM_ClearInterrupt(CYBSP_COUNTER_HW, CYBSP_COUNTER_NUM, CY_TCPWM_INT_ON_TC );

    /* Toggle the user LED state */
    Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
}

/* [] END OF FILE */
