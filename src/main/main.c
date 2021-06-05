#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// UART 2 pins of ESP32
#define RXD2 16
#define TXD2 17

// Sending time interval (microseconds)
#define HB_SEND_INTERVAL 3000000  // 3000 milliseconds

// Buffer size for UART
#define HB_BUFFER_SIZE 1024

// Start frame definition for reliable serial communication
#define HB_FIRST_FRAME 0xABCD

// Pointer declaration for the new received data
static uint8_t *hb_in_pointer;

// Index for new data pointer
static uint8_t hb_in_id = 0;

// Buffer Start Frame
static uint16_t hb_first_frame;

// Previous incoming byte
static uint8_t hb_incoming_byte_prev;

typedef struct {
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} HoverboardCommand;
HoverboardCommand hb_command;

typedef struct {
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} HoverboardFeedback;
HoverboardFeedback hb_feedback;
HoverboardFeedback hb_new_feedback;

static void hb_send(int16_t steer, int16_t speed) {
  hb_command.start = (uint16_t)HB_FIRST_FRAME;
  hb_command.steer = (int16_t)steer;
  hb_command.speed = (int16_t)speed;
  hb_command.checksum =
      (uint16_t)(hb_command.start ^ hb_command.steer ^ hb_command.speed);
  uart_write_bytes(UART_NUM_2, (const char *)&hb_command, sizeof(hb_command));
}

static void hb_on_receive(uint8_t hb_incoming_byte) {
  // Construct the start frame
  hb_first_frame = ((uint16_t)(hb_incoming_byte) << 8) | hb_incoming_byte_prev;

  // Copy received data
  if (hb_first_frame == HB_FIRST_FRAME) {
    // Initialize if new data is detected
    hb_in_pointer = (uint8_t *)&hb_new_feedback;
    *hb_in_pointer++ = hb_incoming_byte_prev;
    *hb_in_pointer++ = hb_incoming_byte;
    hb_in_id = 2;
  } else if (hb_in_id >= 2 && hb_in_id < sizeof(HoverboardFeedback)) {
    // Save the new received data
    *hb_in_pointer++ = hb_incoming_byte;
    hb_in_id++;
  }

  // Check if we reached the end of the package
  if (hb_in_id == sizeof(HoverboardFeedback)) {
    uint16_t checksum;
    checksum =
        (uint16_t)(hb_new_feedback.start ^ hb_new_feedback.cmd1 ^
                   hb_new_feedback.cmd2 ^ hb_new_feedback.speedR_meas ^
                   hb_new_feedback.speedL_meas ^ hb_new_feedback.batVoltage ^
                   hb_new_feedback.boardTemp ^ hb_new_feedback.cmdLed);

    // Check validity of the new data
    if (hb_new_feedback.start == HB_FIRST_FRAME &&
        checksum == hb_new_feedback.checksum) {
      // Copy the new data
      memcpy(&hb_feedback, &hb_new_feedback, sizeof(HoverboardFeedback));
      printf(
          "steer: %d speed: %d speed_R: %d speed_L: %d bat: %d temp: %d led: "
          "%d\n",
          hb_feedback.cmd1, hb_feedback.cmd2, hb_feedback.speedR_meas,
          hb_feedback.speedL_meas, hb_feedback.batVoltage,
          hb_feedback.boardTemp, hb_feedback.cmdLed);
    }
    // Reset the index for prevents to enter in this in the next cycle
    hb_in_id = 0;
  }

  hb_incoming_byte_prev = hb_incoming_byte;
}

static void hb_send_task() {
  int64_t time_now;
  int64_t last_send_time = 0;

  int i = 0;

  // forward, back, left, right, stop
  int steer[5] = {0, 0, -100, 100, 0};
  int speed[5] = {100, -100, 0, 0, 0};

  while (1) {
    time_now = esp_timer_get_time();

    if (last_send_time <= time_now) {
      last_send_time = time_now + HB_SEND_INTERVAL;
      hb_send(steer[i], speed[i]);
      i++;
      if (i == 5) {
        i = 0;
      }
    }

    vTaskDelay(20 / portTICK_RATE_MS);
  }
}

static void hb_receive_task() {
  uint8_t *data = (uint8_t *)malloc(HB_BUFFER_SIZE);
  while (1) {
    int len = uart_read_bytes(UART_NUM_2, data, HB_BUFFER_SIZE,
                              20 / portTICK_RATE_MS);
    if (len > 0) {
      for (int i = 0; i < len; i++) {
        hb_on_receive(data[i]);
      }
    }
  }
}

static void hb_uart_setup() {
  uart_config_t uart_config = {.baud_rate = 115200,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  uart_param_config(UART_NUM_2, &uart_config);
  uart_set_pin(UART_NUM_2, TXD2, RXD2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_2, HB_BUFFER_SIZE * 2, 0, 0, NULL, 0);
}

void app_main() {
  printf("Hoverboard test started\n");
  hb_uart_setup();
  xTaskCreate(hb_send_task, "hb_send_task", HB_BUFFER_SIZE * 2, NULL, 10, NULL);
  xTaskCreate(hb_receive_task, "hb_receive_task", HB_BUFFER_SIZE * 2, NULL, 10,
              NULL);
}