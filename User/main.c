/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/10/01 10:36a $
 * @brief    Template project for NUC029 series MCU
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC029xAN.h"

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    // ------------------------
    // CLOCK CONFIGURE
    /********************
    /* If the macros do not exist in your project, please refer to the related clk.h in Header folder of the tool package */
    /* Enable clock source */
    CLK_EnableXtalRC(CLK_PWRCON_OSC10K_EN_Msk|CLK_PWRCON_OSC22M_EN_Msk|CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for clock source ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC10K_STB_Msk|CLK_CLKSTATUS_OSC22M_STB_Msk|CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Disable PLL first to avoid unstable when setting PLL */
    CLK_DisablePLL();

    /* Set PLL frequency */
    CLK->PLLCON = (CLK->PLLCON & ~(0x000FFFFFUL)) | 0x0000C22EUL;

    /* Waiting for PLL ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk);

    /* Set HCLK clock */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT, CLK_CLKDIV_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(EBI_MODULE);
    CLK_EnableModuleClock(ISP_MODULE);
    CLK_EnableModuleClock(PWM01_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(WWDT_MODULE);

    /* Set IP clock */
    CLK_SetModuleClock(PWM01_MODULE, CLK_CLKSEL1_PWM01_S_HIRC, MODULE_NoMsk);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDT_S_LIRC, MODULE_NoMsk);
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL2_WWDT_S_HCLK_DIV2048, MODULE_NoMsk);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    // ------------------------
    // PIN CONFIGURE
    SYS->P0_MFP = 0x00000000;
    SYS->P1_MFP = 0x00000000;
    SYS->P2_MFP |= (SYS_MFP_P20_PWM0);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD | SYS_MFP_P31_TXD);
    SYS->P4_MFP |= (SYS_MFP_P47_ICE_DAT | SYS_MFP_P46_ICE_CLK);

    /* Lock protected registers */
    SYS_LockReg();
}

void PWMA_IRQHandler(void)
{
    static int toggle = 0;

    // Update PWMA channel 0 period and duty
    if(toggle == 0)
    {
        PWM_SET_CNR(PWMA, PWM_CH0, 200);
        PWM_SET_CMR(PWMA, PWM_CH0, 150);
    }
    else
    {
        PWM_SET_CNR(PWMA, PWM_CH0, 200);
        PWM_SET_CMR(PWMA, PWM_CH0, 100);
    }
    toggle ^= 1;
    // Clear channel 0 period interrupt flag
    PWM_ClearPeriodIntFlag(PWMA, 0);
}

void UART0_Init()
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}


int main()
{

	/* Unlock protected registers */
	    SYS_UnlockReg();

	    /* Init System, IP clock and multi-function I/O */
	    SYS_Init();

	    /* Lock protected registers */
	    SYS_LockReg();

	    /* Init UART to 115200-8n1 for print message */
	    UART_Open(UART0, 115200);
	    printf("+------------------------------------------------------------------------+\n");
	    printf("|                          PWM Driver Sample Code                        |\n");
	    printf("|                                                                        |\n");
	    printf("+------------------------------------------------------------------------+\n");
	    printf("  This sample code will use PWMA channel 0 to output waveform\n");
	    printf("  I/O configuration:\n");
	    printf("    waveform output pin: PWM0(P2.0)\n");
	    printf("\nUse double buffer feature.\n");

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    PWM_ConfigOutputChannel(PWMA, PWM_CH0, 1000, 50);

    PWM_EnableOutput(PWMA, 0x1);

    PWMA->PIER = PWM_PIER_PWMIE0_Msk;
    NVIC_EnableIRQ(PWMA_IRQn);

    // Start
    PWM_Start(PWMA, 0x1);


    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
