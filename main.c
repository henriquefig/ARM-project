/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/





/*
 * Trabalho 2
 * Analog-to-digital Converter com FreeRTOS
 * 2015-2016
 *
 * Henrique Figueiredo - 1120401
 * João Brandão - 1120484
 */
 

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/* Task priorities. */
#define mainFLASH_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainLCD_TASK_PRIORITY	( tskIDLE_PRIORITY + 2)
/* The rate at which the flash task toggles the LED. */
#define mainFLASH_DELAY1000			( ( TickType_t ) 1000 / portTICK_RATE_MS )

 /* Configure the RCC clocks */
static void prvSetupRCC( void );

 /* Configure the GPIO. */
static void prvSetupGPIO( void );

/* Configure the ADC */
static void prvSetupADC( void );

/* Simple LED toggle task + USART. */
static void prvFlashTask( void *pvParameters );

/* LCD activity task. */
static void prvLcdTask( void *pvParameters );

/* ADC temperature read task. */
static void prvTempTask( void *pvParameters );
/* ADC temperature read task. */
static void prvadcTask( void *pvParameters );

/********** Usefull functions **********/
/* USART2 configuration. */
static void prvSetupUSART2( void );

/* USART2 send message. */
static void prvSendMessageUSART2(char *message);

/* LCD display messages. */
static void prvDisplayMessageLCD(int line_number, char *message );
/***************************************/

/* Task 1 handle variable. */
TaskHandle_t HandleTask1;
/* Task 2 handle variable. */
TaskHandle_t HandleTask2;
/* Task 3 handle variable. */
TaskHandle_t HandleTask3;

/* Queue USART handle variable */
QueueHandle_t xQueueUSART;
/* Queue ADC handle variable */
QueueHandle_t xQueueADC;

    int i=0; 

u8 pontos(unsigned int x)
{
    return (x*0.021)+15;
}

/* Função para desenhar o titulo, os eixos e a escala do gráfico a apresentar no LCD do Primer */
void LCD_GRAPH()
{
    LCD_FillRect(0, 0, 127, 127, 0xFFFF);
    //titulo
    DRAW_DisplayString(20, 110,"Grafico", 10);
    //eixos
    LCD_DrawRect(15,53,100,1,0x0000);
    LCD_DrawRect(15,15,1,100,0x0000);
    DRAW_DisplayString(110, 45,"s", 7);
    //escala
    DRAW_DisplayString(1, 51,"0",2);
    DRAW_DisplayString(1, 95,"30",2);
    DRAW_DisplayString(1, 10,"-30",2);
}
/* Função para atualizar a legenda do gráfico
   dependendo de qual eixo queremos nele apresentar
*/
void LCD_UPDATE(char axis)
{
    LCD_GRAPH();

    if(axis=='x')
        DRAW_DisplayString(1, 110,"x", 1);
    if(axis=='y')
        DRAW_DisplayString(1, 110,"y", 1);
    if(axis=='z')
        DRAW_DisplayString(1, 110,"z", 1);
    if(axis=='a')
    {
        LCD_DrawRect(83,10,4,4,0xD2F0);
        DRAW_DisplayString(87,10,"x", 1);
        LCD_DrawRect(98,10,4,4,0xFC00);
        DRAW_DisplayString(102,10,"y", 1);
        LCD_DrawRect(112,10,4,4,0x0000);
        DRAW_DisplayString(117,10,"z", 1);
     }
}

/* Configurações necessárias do ADC */
void ADC_CONFIG()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
        
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
    /* Enable ADC1 clock so that we can talk to it */ 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
    /* Put everything back to power-on defaults */ 
    ADC_DeInit(ADC1); 
        NVIC_InitTypeDef NVIC_InitStructure;
    /* Configura o Priority Group com 1 bit */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    /* Interrupção global do USART2 com prioridade 0 sub-prioridade 2 */
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQChannel; //ADC_IRQChannel
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // no dual mode
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 3;

    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    /* Check the end of ADC1 reset calibration register */ 
    while(ADC_GetResetCalibrationStatus(ADC1));
    /* Start ADC1 calibaration */ 
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */ 
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
}

/* Configurações necessárias da USART */
void Usart_config()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configura o Priority Group com 1 bit */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    /* Interrupção global do USART2 com prioridade 0 sub-prioridade 2 */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //Baudrate=9600 palavras de 8bits sem paridade com 1 stopbit e sem hardware flow control
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
    //Enable da interrupção da USART2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}


/* Interrupção ADC 
Ocorre quando despoleta o Evento "End Of Conversion".
*/
void ADC_IRQHandler(void)
{
    static BaseType_t xHigherPriorityTaskWoken;

    if (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == SET)
    {      
        unsigned int ad_vbat = ADC_GetConversionValue(ADC1);
        xQueueSendToBackFromISR(xQueueADC, &ad_vbat, &xHigherPriorityTaskWoken);
    
        if(xHigherPriorityTaskWoken == pdTRUE)
        {
            taskYIELD();
        }
        /* clear ADC1 irq pending bit */
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);   
    }
}
/* Interrupção da USART 2 
Ocorre sempre que é recebido um caracter introduzido
pelo utilizador no teclado do computador.
*/
void USART2_IRQHandler(void)
{
    static BaseType_t pxHigherPriorityTaskWoken;
    char carc=USART_ReceiveData(USART2);

    if(carc=='x' || carc=='y' || carc=='z' || carc=='a')
    {
        xQueueSendToBackFromISR(xQueueUSART,&carc,&pxHigherPriorityTaskWoken);
        if(pxHigherPriorityTaskWoken == pdTRUE )
        {
            taskYIELD();
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
/*-----------------------------------------------------------*/
/*                            MAIN                           */
/*-----------------------------------------------------------*/
int main( void )
{

	/*Setup the hardware, RCC, GPIO, etc...*/
    prvSetupRCC();
    prvSetupGPIO();
    prvSetupUSART2();
    Usart_config();
    ADC_CONFIG();
    DRAW_Init();

    /* Criação das filas de mensagens para a USART e para o ADC */
    xQueueUSART=xQueueCreate(10,sizeof(char));
    xQueueADC=xQueueCreate(10,sizeof(unsigned int));

    if(xQueueUSART != 0 && xQueueADC != 0)
    {
        /* Start the tasks */	 
        xTaskCreate(prvFlashTask, "Flash", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, &HandleTask2 );
        xTaskCreate(prvadcTask, "adc", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, &HandleTask3 );
    
        /* Start the scheduler. */
        vTaskStartScheduler();
	}
    
	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}
/*-----------------------------------------------------------*/


/* Tarefa de Debug
Coloca um LED a piscar de modo a perceber se o programa
se encontra a correr.
*/
static void prvFlashTask( void *pvParameters )
{
    TickType_t xLastExecutionTime;

    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY1000 );
        GPIO_WriteBit(GPIOB, GPIO_Pin_9, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_9)));
    }
}
/* Tarefa do ADC
- Recebe caracter introduzido pelo utilizador através da fila de mensagens xQueueUSART
- Prepara canais para leitura do sensor consoante a escolha do eixo escolhido
- Efetua a leitura desse canal do ADC1
- Representa no LCD os valores obtidos do acelerómetro 
*/
static void prvadcTask( void *pvParameters )
{
    int abcissa=14;

    for( ;; )
	{
        char carc;
    
        if(xQueueUSART != 0 )
        {
            if(xQueueReceive( xQueueUSART, &carc, ( TickType_t ) 10 ) == pdTRUE )
            {
                /*Verificação de qual caracter foi introduzido pelo utilizador, 
                recebido pela USART e transmitido na fila de mensagens xQueueUSART.*/
                if(carc == 'x')
                {
                    i=4;
                }
                if(carc == 'y')
                {
                    i=5;
                }
                if(carc == 'z')
                {
                    i=6;
                }
                if(carc == 'a')
                {
                    i=7;
                }
            }
        }
    
        if(abcissa>=101 || abcissa==14)
        {          
            abcissa=15;
            LCD_UPDATE(carc);
        }

        switch(i)
        {
            case 4: //Seleciona o canal 4 do ADC1 para ler
            {
                ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_41Cycles5);
                abcissa++;
                break;
            }
            case 5: //Seleciona o canal 5 do ADC1 para ler
            {
                ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_41Cycles5);
                abcissa++;
                break;
            }
            case 6: //Seleciona o canal 6 do ADC1 para ler
            {
                ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_41Cycles5);
                abcissa++;
                break;
            }
        }
        /* Efetua a leitura do ADC1 */
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
        /*Recebe-se da fila de mensagens xQueueADC o valor
        lido pelo ADC através da variável ad_vbat*/
        unsigned int ad_vbat = 0;
        if(xQueueReceive(xQueueADC, &ad_vbat, portMAX_DELAY)==pdTRUE)
        {
            starting_delay(500);      
            switch(i)
            {
                case 4: //Desenha no LCD um pixel relativo ao valor lido pelo ADC1 do eixo X
                {
                    LCD_DrawPixel(abcissa,pontos(ad_vbat),0x0000);
                    break;
                }
                case 5: //Desenha no LCD um pixel relativo ao valor lido pelo ADC1 do eixo Y
                {
                    LCD_DrawPixel(abcissa,pontos(ad_vbat),0x0000);
                    break;
                }
                case 6: //Desenha no LCD um pixel relativo ao valor lido pelo ADC1 do eixo Z
                {
                      LCD_DrawPixel(abcissa,pontos(ad_vbat),0x0000);
                    break;
                }
                case 7: //Desenha no LCD os 3 gráficos dos 3 eixos em "simultaneo"
                {
                    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_41Cycles5);
                    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
                    if(xQueueReceive(xQueueADC, &ad_vbat, portMAX_DELAY)==pdTRUE)
                        LCD_DrawPixel(abcissa,pontos(ad_vbat),0xD2F0);
                    
                    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_41Cycles5);
                    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
                    if(xQueueReceive(xQueueADC, &ad_vbat, portMAX_DELAY)==pdTRUE)
                        LCD_DrawPixel(abcissa,pontos(ad_vbat),0xFC00);
                    
                    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_41Cycles5);
                    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
                    if(xQueueReceive(xQueueADC, &ad_vbat, portMAX_DELAY)==pdTRUE)
                        LCD_DrawPixel(abcissa,pontos(ad_vbat),0x0000);
                    
                    abcissa++;
                    break;
                }
            }
        }
	}
}





/*-----------------------------------------------------------*/
static void prvSetupRCC( void )
{
    /* RCC configuration - 72 MHz */
 
    ErrorStatus HSEStartUpStatus;

    RCC_DeInit();
    /*Enable the HSE*/
    RCC_HSEConfig(RCC_HSE_ON); 
    /* Wait untill HSE is ready or time out */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS)
    {
        /* Enable The Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        /* 72 MHZ - 2 wait states */
        FLASH_SetLatency(FLASH_Latency_2);
    
        /* No division HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /* PCLK1 = HCLK/2 (36MHz) */
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /* PCLK2 = HCLK (72MHz)*/
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* Use PLL with HSE=12MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);
        /* Enable the PLL */
        RCC_PLLCmd(ENABLE);
        /* Wait for PLL ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );
        
        /* Select the PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        /* Wait until PLL is used as system clock */
        while( RCC_GetSYSCLKSource() != 0x08 );
    }
    else 
    {
        while(1);
    }
}
/*-----------------------------------------------------------*/
static void prvSetupGPIO( void )
{
    /* GPIO configuration - GREEN LED*/

    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);	
}
/*-----------------------------------------------------------*/
void prvSetupUSART2( void )
{
USART_InitTypeDef USART_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

    /* USART2 is configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - 1 Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled */

    /* Enable GPIOA clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

    /* USART Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    
    /* Configure the USART2 */ 
    USART_Init(USART2, &USART_InitStructure);
    /* Enable the USART2 */
    USART_Cmd(USART2, ENABLE);
 }
/*-----------------------------------------------------------*/
