//
// Created by bismarckkk on 2025/3/22.
// Dynamic Edition
//

#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ui_types.h"
#include "usart.h"

#define UI_TX_QUEUE_DEPTH 8
#define UI_TX_MAX_PACKET_LEN 128

extern int ui_self_id;
extern volatile uint8_t ui_tx_dma_busy;

void print_message(const uint8_t* message, int length);

// User Code Begin
#define SEND_MESSAGE(message, length) HAL_UART_Transmit_DMA(&huart10, (uint8_t *)(message), (uint16_t)(length))
// User Code End

#define ui_set(obj, field, value) { \
    obj->field = value; \
    obj##_dirty = obj##_max_send_count; \
}

void ui_proc_1_frame(ui_1_frame_t *msg);
void ui_proc_2_frame(ui_2_frame_t *msg);
void ui_proc_5_frame(ui_5_frame_t *msg);
void ui_proc_7_frame(ui_7_frame_t *msg);
void ui_proc_string_frame(ui_string_frame_t *msg);
void ui_proc_delete_frame(ui_delete_frame_t *msg);

void ui_delete_layer(const uint8_t delete_type, const uint8_t layer);

void ui_tx_enqueue_message(const uint8_t *message, int length);
void ui_tx_service(void);
int ui_tx_queue_count(void);
void ui_tx_on_dma_complete(void);

void ui_scan_and_send(const ui_interface_figure_t* ui_now_figures, uint8_t* ui_dirty_figure, const ui_interface_string_t* ui_now_strings, uint8_t* ui_dirty_string, int total_figures, int total_strings);

#ifdef __cplusplus
}
#endif

#endif //UI_INTERFACE_H
