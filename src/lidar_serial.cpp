/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include "lidar_serial.h"

EventFlags lidarThreadFlag;
uint8_t lidar_new_value, lidar_processing;
uint16_t lidar_overflow = 0, start_sequence_incr = 0;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
uint8_t lidar_frame[LDS_01_TRAM_LENGTH];
uint16_t lidar_distances_mean[LDS_01_MEDIAN_RES];
uint8_t dma_mode = 0;
uint8_t lidar_back_trig = 0, lidar_front_trig = 0;

// General handler for UART Interrupt, need to be linked to NVIC using vector
// See https://os.mbed.com/forum/mbed/topic/33580/
void custom_usart1_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart1);
}

// General handler for DMA Interrupt, need to be linked to NVIC using vector
void custom_DMA2_Stream2_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

// Called by HAL_UART_IRQHandler and HAL_DMA_IRQHandler
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if (dma_mode) {
        dma_mode = 0;
        lidarThreadFlag.set(LIDAR_THREAD_FLAG);
        HAL_UART_Receive_IT(&huart1, &lidar_new_value, 1);
    } else {

        if ((start_sequence_incr == 0) && (lidar_new_value == LDS_01_START_FIRST)) {
            start_sequence_incr = 1;
            lidar_frame[0] = lidar_new_value;
            HAL_UART_Receive_IT(&huart1, &lidar_new_value, 1);
        } else if ((start_sequence_incr == 1) && (lidar_new_value == LDS_01_START_SECOND)) {
            start_sequence_incr = 0;
            lidar_frame[1] = lidar_new_value;
            //            lidarThreadFlag.set(LIDAR_THREAD_FLAG);
            // setup DMA to get the rest of the message
            HAL_UART_Receive_DMA(&huart1, lidar_frame + 2, LDS_01_TRAM_LENGTH - 2);
            dma_mode = 1;

            if (lidar_processing) {
                lidar_overflow++;
            }

        } else {
            start_sequence_incr = 0;
            HAL_UART_Receive_IT(&huart1, &lidar_new_value, 1);
        }
    }
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

    /* DMA Interrupt enable */
    __HAL_RCC_DMA2_CLK_ENABLE();
    //    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
    NVIC_SetVector(DMA2_Stream2_IRQn, (uint32_t)&custom_DMA2_Stream2_IRQHandler);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    /* DMA module Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_usart1_rx);
    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

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

void lidarMain() {

    init_lidar_serial();

    uint32_t motor_speed = 0;
    uint16_t rpms;
    int lidar_index = 0;
    //    int print_wait = 0;
    while (true) {
        // Wait for asserv tick
        lidarThreadFlag.wait_any(LIDAR_THREAD_FLAG);
        lidar_processing = 1;

        // part of source code from Robotis:
        // https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/blob/master/applications/lds_driver/lds_driver.cpp
        for (uint16_t i = 0; i < LDS_01_TRAM_LENGTH; i = i + 42) {
            if (lidar_frame[i] == 0xFA && lidar_frame[i + 1] == (0xA0 + i / 42)) {
                motor_speed += (lidar_frame[i + 3] << 8) + lidar_frame[i + 2];
                rpms = (lidar_frame[i + 3] << 8 | lidar_frame[i + 2]) / 10;

                uint32_t sum = 0;
                uint8_t sum_num = 0;
                for (uint16_t j = i + 4; j < i + 40; j = j + 6) { // 6 paquets
                    lidar_index = 6 * (i / 42) + (j - 4 - i) / 6;

                    uint8_t byte0 = lidar_frame[j];
                    uint8_t byte1 = lidar_frame[j + 1];
                    uint8_t byte2 = lidar_frame[j + 2];
                    uint8_t byte3 = lidar_frame[j + 3];

                    uint16_t intensity = (byte1 << 8) + byte0;
                    uint16_t range = (byte3 << 8) + byte2;

                    // add to median values
                    if (range != 0) { // if 0, then probably invalid data, or too far
                        sum += range;
                        sum_num++;
                    }
                }

                // update median
                lidar_distances_mean[(lidar_index / 6)]
                        = sum_num == 0 ? 0xFFFF : uint16_t(sum / sum_num);
            }
        }

        updateLidarDetect();
        lidar_processing = 0;

        //        print_wait++;
        //        if (print_wait > 5) {
        //            print_wait = 0;
        //            for (int i = 0; i < LDS_01_MEDIAN_RES; i++) {
        //                terminal_printf("r[%3d]=%3d\n", i, lidar_distances_mean[i]);
        //            }
        //        }
    }
}

void updateLidarDetect() {

    // back
    for (int index = LIDAR_BACK_MIN; index < LIDAR_BACK_MAX; index++) {
        lidar_back_trig = 0;
        if (lidar_distances_mean[index] < LIDAR_TRIG_DETECT) {
            lidar_back_trig = 1;
            break;
        }
    }

    // front
    for (int index = LIDAR_FRONT_MIN; index < LIDAR_FRONT_MAX; index++) {
        lidar_front_trig = 0;
        if (lidar_distances_mean[index] < LIDAR_TRIG_DETECT) {
            lidar_front_trig = 1;
            break;
        }
    }
}
