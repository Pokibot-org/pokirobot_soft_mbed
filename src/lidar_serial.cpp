/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include "lidar_serial.h"

EventFlags lidarThreadFlag;
uint8_t lidar_new_value, lidar_processing;
uint16_t lidar_overflow = 0, start_sequence_incr = 0;

//extern DMA_HandleTypeDef hdma_usart1_tx;
UART_HandleTypeDef huart1;

// General handler, need to be linked to NVIC
void custom_usart1_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart1);
}

// Called by HAL_UART_IRQHandler
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

    if ((start_sequence_incr == 0) && (lidar_new_value == 0xFA)){
        start_sequence_incr = 1;
    } else if((start_sequence_incr == 1) && (lidar_new_value == 0xA0)){
        start_sequence_incr = 0;
        lidarThreadFlag.set(LIDAR_THREAD_FLAG);
        // setup DMA here

        if (lidar_processing) {
            lidar_overflow++;
        }

    } else {
        start_sequence_incr = 0;
    }

    // Setup UART for next interrupt
    HAL_UART_Receive_IT(&huart1, &lidar_new_value, 1);
}

// Lidar serial on PA9 / PA10 at 230400bps (Robotis LDS-01)
void init_lidar_serial() {

    /* USART gpios init */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART interrupt init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    NVIC_SetVector(USART1_IRQn, (uint32_t)&custom_usart1_IRQHandler);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* USART module init */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 230400;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    /* Call interrupt function one time to begin the uart reception from interrupt */
    HAL_UART_Receive_IT(&huart1, &lidar_new_value, 1);
}
