/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint64_t timestamp_us;
  uint32_t delta_us;
} Wheel_LogEvent_t;

typedef struct
{
  uint32_t timestamp_ms;
  uint16_t raw[6];
} ADC_LogSample_t;

typedef struct
{
  uint64_t timestamp_us;
  uint32_t timestamp_ms;
  uint32_t std_id;
  uint8_t dlc;
  uint8_t data[8];
} EMU_CAN_Frame_t;

typedef struct
{
  uint64_t timestamp_us;
  uint8_t channel;
  uint32_t delta_us;
  uint32_t speed_centi_kmh;
} Wheel4_LogEvent_t;

typedef struct
{
  uint64_t timestamp_us;
  uint16_t raw[6];
} ADC6_LogSample_t;

typedef struct
{
  uint8_t valid;
  uint64_t timestamp_us;
  uint32_t std_id;
  uint8_t dlc;
  uint8_t data[8];
} Telemetry_CANLatest_t;

typedef struct
{
  uint8_t valid;
  uint64_t timestamp_us;
  uint32_t delta_us;
  uint32_t speed_centi_kmh;
} Telemetry_WheelLatest_t;

typedef enum
{
  RACE_LOGGER_WAIT_CAN = 0,
  RACE_LOGGER_START_DELAY = 1,
  RACE_LOGGER_LOGGING = 2
} RaceLoggerState_t;

typedef struct
{
  uint8_t rpm_valid;
  uint8_t water_valid;
  uint8_t oil_valid;
  uint8_t speed_valid;
  uint8_t gear_valid;
  uint8_t battery_valid;
  uint16_t rpm;
  int16_t water_c;
  int16_t oil_c;
  uint16_t speed_kmh;
  uint8_t gear;
  uint32_t battery_mV;
} RaceCANLatest_t;

typedef enum
{
  SD_FAULT_NONE = 0,
  SD_FAULT_NO_CARD = 1,
  SD_FAULT_MOUNT = 2,
  SD_FAULT_OPEN_TELEMETRY = 3,
  SD_FAULT_HEADER_WRITE = 4,
  SD_FAULT_HEADER_SHORT_WRITE = 5,
  SD_FAULT_HEADER_SYNC = 6,
  SD_FAULT_TELEMETRY_DRAIN = 7,
  SD_FAULT_SNAPSHOT_FORMAT = 8,
  SD_FAULT_SNAPSHOT_FLUSH = 9,
  SD_FAULT_SNAPSHOT_BUFFER_FULL = 10,
  SD_FAULT_IDLE_FLUSH = 11,
  SD_FAULT_INTERVAL_SYNC = 12,
  SD_FAULT_FLUSH_NO_ACTIVE_FILE = 13,
  SD_FAULT_FLUSH_WRITE = 14,
  SD_FAULT_FLUSH_SHORT_WRITE = 15,
  SD_FAULT_FLUSH_SYNC = 16
} SD_FaultStage_t;

typedef enum
{
  SD_LOG_FLUSH_REASON_NONE = 0,
  SD_LOG_FLUSH_REASON_BUFFER_FULL = 1,
  SD_LOG_FLUSH_REASON_IDLE = 2,
  SD_LOG_FLUSH_REASON_SYNC = 3,
  SD_LOG_FLUSH_REASON_LEGACY = 4
} SD_LogFlushReason_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SD_BRINGUP_TEST_FILE     "0:/DATALOG_TEST.TXT"
#define SD_BRINGUP_TEST_PAYLOAD  "STM32F405 1-bit SDIO FATFS write/read test OK\r\nRemove card and check this file on your PC.\r\n"
#define SD_LED_ERR_HAL_INIT      6U
#define SD_LED_ERR_WIDE_BUS      7U
#define SD_LED_ERR_FATFS         8U
#define SD_LED_ERR_TIM           10U
#define SD_LED_ERR_MOUNT         11U
#define SD_LED_ERR_OPEN_WHEEL    13U
#define SD_LED_ERR_OPEN_ADC      13U
#define SD_LED_ERR_WRITE_HEADER  14U
#define SD_LED_ERR_SYNC          15U
#define SD_LED_ERR_WRITE_LOG     16U
#define SD_LED_ERR_ADC_CONFIG    17U
#define SD_LED_ERR_CAN_CONFIG    18U
#define SD_LED_ERR_OPEN_EMU      19U
#define SD_LED_ERR_GNSS_UART     20U
#define SD_FAULT_LOG_FILE        "0:/FAULT.TXT"

/* USB Mass Storage is disabled for all logger modes.
 * The old USB MSC bring-up path exposes the same SD card to the host PC while
 * the firmware is also using FatFs. Even read-only host access can reinitialize
 * or read the card through BSP_SD at the same time as f_write()/f_sync(), which
 * can corrupt the active logger session or produce intermittent write errors.
 */
#define USB_MSC_ENABLE           0U

/* Firmware mode switch.
 * 1: Boot directly into the raw GNSS logger. USART1 receives NMEA/UBX bytes and
 *    stores them in a standalone GNSS_xxx.nmea file on the SD card.
 * 0: Run the preserved telemetry logger path for CAN, ADC, wheel speed, and
 *    optional Nextion RPM output.
 *
 * Keep both paths in the same source file for now. This makes it easy to return
 * to the full telemetry logger after validating the GNSS hardware and serial
 * data path.
 */
#define GNSS_LOGGER_MODE         0U

/* Current in-car firmware path.
 * The active runtime is intentionally narrow:
 *   boot -> receive EMU Black CAN stream -> update the Nextion HMI and shift
 *   light immediately -> rpm >= 1000 -> wait 2 s -> log accepted CAN frames.
 */
#define RACE_CAN_LOGGER_MODE     1U

/* GNSS serial logging tuning.
 * USART1 is configured at 115200 8N1 by MX_USART1_UART_Init().
 * At 115200 baud the maximum payload rate is roughly 11.5 kB/s, so this 16 kB
 * queue gives the SD writer some time to recover from short FATFS write stalls.
 * The queue is placed in CCMRAM to avoid consuming the normal SRAM region.
 */
#define GNSS_RX_QUEUE_LEN        16384U
#define GNSS_RX_DRAIN_CHUNK_SIZE 128U
#define GNSS_LOG_FLUSH_IDLE_MS   5000U
#define GNSS_LOG_SYNC_INTERVAL_MS 60000U
#define SD_LOG_SYNC_FAIL_FATAL   0U
#define SD_BRINGUP_USE_4BIT_BUS  0U
#define SD_LOG_PATH_LEN          32U
#define SD_LOG_FILE_MAX_INDEX    999U
#define EMU_CAN_BASE_ID          0x600U
#define EMU_CAN_ID_MASK          0x7F8U
#define EMU_CAN_FRAME_COUNT      8U
#define EMU_CAN_RX_QUEUE_LEN     2048U
#define WHEEL_CHANNEL_COUNT      4U
#define WHEEL4_RX_QUEUE_LEN      1024U
#define ADC6_LOG_QUEUE_LEN       512U
#define ADC_CHANNEL_COUNT        6U
#define ADC_LOG_QUEUE_LEN        256U
#define WHEEL_RX_QUEUE_LEN       512U
#define TIM1_TICKS_PER_OVERFLOW  65536ULL
#define WHEEL_CIRCUMFERENCE_MM   1436ULL
#define WHEEL_TEETH_COUNT        3ULL
#define SPEED_CENTI_KMH_SCALE    ((uint32_t)((WHEEL_CIRCUMFERENCE_MM * 360000ULL) / WHEEL_TEETH_COUNT))
#define SPEED_MIN_VALID_DELTA_US 800U
#define SPEED_TIMEOUT_US         ((uint32_t)SPEED_TIMEOUT_MS * 1000U)
#define SPEED_FILTER_ALPHA_NUM   1U
#define SPEED_FILTER_ALPHA_DEN   4U
#define SPEED_TIMEOUT_MS         1000U
#define SPEED_LOG_INTERVAL_MS    100U
#define SD_LOG_BUFFER_SIZE       16384U
#define SD_LOG_FLUSH_MARGIN      512U
#define SD_LOG_FLUSH_IDLE_MS     5000U
#define SD_LOG_SYNC_EVERY        256U
#define SD_LOG_SYNC_INTERVAL_MS  10000U
#define SD_LOG_IO_RETRY_COUNT    5U
#define SD_LOG_IO_RETRY_DELAY_MS 50U
#define SD_LOWLEVEL_IO_RETRY_COUNT     3U
#define SD_LOWLEVEL_IO_RETRY_DELAY_MS  20U
#define SD_LOWLEVEL_READY_TIMEOUT_MS   1000U
#define SD_LOWLEVEL_OP_NONE            0U
#define SD_LOWLEVEL_OP_READ            1U
#define SD_LOWLEVEL_OP_WRITE           2U
#define SD_LOG_DIAG_FORCE_SYNC   0U
#define TELEMETRY_SNAPSHOT_INTERVAL_US 10000ULL
#define TELEMETRY_CAN_TIMEOUT_US       500000ULL
#define TELEMETRY_WHEEL_TIMEOUT_US     1000000ULL
#define TELEMETRY_SD_MISSING_BLINK_MS  350U
#define RACE_LOG_RPM_START_THRESHOLD   1000U
#define RACE_LOG_START_DELAY_MS        2000U
#define RACE_NEXTION_RPM_UPDATE_MS     50U
#define RACE_NEXTION_GEAR_UPDATE_MS    250U
#define RACE_NEXTION_DATA_UPDATE_MS    500U
#define RACE_NEXTION_UART_TIMEOUT_MS   50U
#define RACE_LOG_FLUSH_IDLE_MS         1000U
#define RACE_LOG_SYNC_INTERVAL_MS      5000U
#define NEXTION_HMI_TEST_MODE          0U
#define NEXTION_HMI_RPM_INTERVAL_MS    50U
#define NEXTION_HMI_GEAR_INTERVAL_MS   250U
#define NEXTION_HMI_DATA_INTERVAL_MS   500U
#define TEST_RPM_SWEEP_HALF_MS         500U
#define NEXTION_HMI_UART_TIMEOUT_MS    10U
#define NEXTION_HMI_RPM_MAX            10000U
#define NEXTION_HMI_RPM_STEP           250U
#define NEXTION_HMI_SMALL_MAX          100U
#define NEXTION_HMI_SMALL_STEP         5U
#define NEXTION_RPM_TX_INTERVAL_MS     50U
#define NEXTION_RPM_CAN_ID             (EMU_CAN_BASE_ID + 0U)
#define NEXTION_RPM_PROP               "rpm.val"
#define WS2812_LED_COUNT        12U
#define WS2812_BITS_PER_LED     24U
#define WS2812_RESET_SLOTS      64U
#define WS2812_BUFFER_LEN       ((WS2812_LED_COUNT * WS2812_BITS_PER_LED) + WS2812_RESET_SLOTS)
#define WS2812_T0H_NS           350U
#define WS2812_T1H_NS           700U
#define WS2812_PERIOD_NS        1250U
#define WS2812_DMA_TIMEOUT_MS   20U
#define WS2812_UPDATE_MS        40U
#define RPM_MIN                 0U
#define RPM_MAX                 10000U
#define RPM_PER_LED             1000U
#define RPM_USED_LED_COUNT      10U
#define RPM_REDZONE_START       10000U
#define RPM_LED_BRIGHTNESS      90U
#define RPM_REDZONE_BLINK_MS    80U
#define RPM_LED_MODE_BAR        1U
#define RPM_LED_MODE_DEFAULT    RPM_LED_MODE_BAR
#define LED_BLINK_ON_MS          350U
#define LED_BLINK_OFF_MS         350U
#define LED_BLINK_PAUSE_MS       1600U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAYBE_UNUSED __attribute__((unused))
#define CCMRAM_BUFFER __attribute__((section(".ccmram"), aligned(8)))
#define SDIO_ALIGNED_BUFFER __attribute__((aligned(32)))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim8_ch1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile FRESULT g_sd_last_fresult = FR_OK;
static volatile SD_FaultStage_t g_sd_fault_stage = SD_FAULT_NONE;
static volatile FRESULT g_sd_fault_fresult = FR_OK;
static volatile FRESULT g_sd_fault_log_fresult = FR_OK;
static volatile uint32_t g_sd_fault_detail = 0U;
static volatile uint32_t g_sd_fault_tick = 0U;
static volatile UINT g_sd_fault_buffer_len = 0U;
static volatile uint32_t g_sd_fault_unsynced_count = 0U;
static volatile uint32_t g_sd_fault_telemetry_count = 0U;
static volatile uint32_t g_sd_write_fail_count = 0U;
static volatile uint32_t g_sd_sync_fail_count = 0U;
static volatile FRESULT g_sd_last_sync_fresult = FR_OK;
static volatile uint32_t g_sd_write_call_count = 0U;
static volatile uint32_t g_sd_sync_ok_count = 0U;
static volatile uint32_t g_sd_last_write_size = 0U;
static volatile uint32_t g_sd_max_buffer_len = 0U;
static volatile uint64_t g_sd_total_bytes_written = 0ULL;
static volatile SD_LogFlushReason_t g_sd_last_flush_reason = SD_LOG_FLUSH_REASON_NONE;
static volatile uint8_t g_sd_bringup_status = 0U;
static volatile GPIO_PinState g_sd_detect_state = GPIO_PIN_RESET;
static volatile uint8_t g_sd_hal_stage = 0U;
static volatile uint32_t g_sd_hal_error_code = HAL_SD_ERROR_NONE;
static volatile uint32_t g_sd_ll_last_op = SD_LOWLEVEL_OP_NONE;
static volatile uint32_t g_sd_ll_read_call_count = 0U;
static volatile uint32_t g_sd_ll_write_call_count = 0U;
static volatile uint32_t g_sd_ll_retry_count = 0U;
static volatile uint32_t g_sd_ll_read_fail_count = 0U;
static volatile uint32_t g_sd_ll_write_fail_count = 0U;
static volatile uint32_t g_sd_ll_ready_timeout_count = 0U;
static volatile uint32_t g_sd_ll_last_sector = 0U;
static volatile uint32_t g_sd_ll_last_blocks = 0U;
static volatile uint32_t g_sd_ll_last_buffer_addr = 0U;
static volatile uint32_t g_sd_ll_last_hal_status = HAL_OK;
static volatile uint32_t g_sd_ll_last_hal_error = HAL_SD_ERROR_NONE;
static volatile uint32_t g_sd_ll_last_hal_state = HAL_SD_STATE_RESET;
static volatile uint32_t g_sd_ll_last_card_state = 0U;
static volatile uint32_t g_sd_ll_last_wait_ms = 0U;
static volatile uint32_t g_sd_ll_last_tick = 0U;
static FIL g_wheel_file;
static FIL g_adc_file;
static FIL g_emu_can_file;
static FIL g_gnss_file;
static FIL *g_sd_active_file = NULL;
static volatile uint32_t g_wheel_log_count = 0U;
static volatile uint32_t g_adc_log_count = 0U;
static volatile uint32_t g_emu_can_log_count = 0U;
static volatile uint32_t g_emu_can_rx_count = 0U;
static volatile uint32_t g_emu_can_rx_overflow_count = 0U;
static volatile uint32_t g_emu_can_hal_error_count = 0U;
static volatile uint8_t g_emu_can_enabled = 0U;
static volatile uint32_t g_telemetry_log_count = 0U;
static volatile uint32_t g_wheel4_log_count = 0U;
static volatile uint32_t g_adc6_log_count = 0U;
static volatile uint32_t g_wheel4_rx_overflow_count = 0U;
static volatile uint32_t g_adc6_rx_overflow_count = 0U;
static volatile uint8_t g_telemetry_queue_enabled = 0U;
static volatile uint8_t g_wheel_legacy_queue_enabled = 0U;
static volatile uint8_t g_adc_legacy_queue_enabled = 0U;
static volatile uint32_t g_adc_rx_overflow_count = 0U;
static volatile uint16_t g_adc_dma_buffer[ADC_CHANNEL_COUNT];
static volatile uint16_t g_adc_rx_head = 0U;
static volatile uint16_t g_adc_rx_tail = 0U;
static ADC_LogSample_t g_adc_rx_queue[ADC_LOG_QUEUE_LEN];
static volatile uint16_t g_adc6_rx_head = 0U;
static volatile uint16_t g_adc6_rx_tail = 0U;
static ADC6_LogSample_t g_adc6_rx_queue[ADC6_LOG_QUEUE_LEN] CCMRAM_BUFFER;
static volatile uint16_t g_emu_can_rx_head = 0U;
static volatile uint16_t g_emu_can_rx_tail = 0U;
static EMU_CAN_Frame_t g_emu_can_rx_queue[EMU_CAN_RX_QUEUE_LEN];
static volatile uint16_t g_wheel4_rx_head = 0U;
static volatile uint16_t g_wheel4_rx_tail = 0U;
static Wheel4_LogEvent_t g_wheel4_rx_queue[WHEEL4_RX_QUEUE_LEN] CCMRAM_BUFFER;
static volatile uint64_t g_wheel4_last_timestamp_us[WHEEL_CHANNEL_COUNT];
static volatile uint32_t g_wheel4_last_delta_us[WHEEL_CHANNEL_COUNT];
static volatile uint32_t g_wheel4_filtered_centi_kmh[WHEEL_CHANNEL_COUNT];
static volatile uint8_t g_wheel4_filter_valid[WHEEL_CHANNEL_COUNT];
static volatile uint32_t g_wheel_rx_overflow_count = 0U;
static volatile uint32_t g_tim1_overflow_count = 0U;
static volatile uint8_t g_tim1_timebase_ready = 0U;
static volatile uint64_t g_wheel_last_timestamp_us = 0ULL;
static volatile uint16_t g_wheel_rx_head = 0U;
static volatile uint16_t g_wheel_rx_tail = 0U;
static Wheel_LogEvent_t g_wheel_rx_queue[WHEEL_RX_QUEUE_LEN];

/* Raw GNSS UART receive path.
 * The UART ISR only moves one received byte into this ring buffer and immediately
 * arms the next HAL_UART_Receive_IT() call. The main loop drains this queue into
 * the shared SD write buffer. Keeping FATFS out of the interrupt path is
 * important: SD writes can block for milliseconds and would otherwise lose UART
 * bytes or break other interrupt timing.
 */
static volatile uint8_t g_gnss_rx_byte = 0U;
static volatile uint16_t g_gnss_rx_head = 0U;
static volatile uint16_t g_gnss_rx_tail = 0U;
static volatile uint32_t g_gnss_rx_count = 0U;
static volatile uint32_t g_gnss_rx_overflow_count = 0U;
static volatile uint32_t g_gnss_uart_error_count = 0U;
static volatile uint32_t g_gnss_drain_call_count = 0U;
static volatile uint64_t g_gnss_drain_byte_count = 0ULL;
static volatile uint8_t g_gnss_rx_queue[GNSS_RX_QUEUE_LEN] CCMRAM_BUFFER;

/* Shared SD staging buffer.
 * Telemetry CSV and raw GNSS logging both reuse this buffer, but only one logger
 * mode is active at a time. g_sd_active_file selects the currently open FATFS
 * file, and SD_PulseLoggerFlush() writes this buffer to that file.
 */
static char g_sd_log_buffer[SD_LOG_BUFFER_SIZE] SDIO_ALIGNED_BUFFER;
static char g_sd_log_path[SD_LOG_PATH_LEN];
static UINT g_sd_log_buffer_len = 0U;
static uint32_t g_sd_log_last_write_tick = 0U;
static uint32_t g_sd_log_last_sync_tick = 0U;
static uint32_t g_sd_log_unsynced_count = 0U;
static Telemetry_CANLatest_t g_telemetry_latest_can;
static Telemetry_WheelLatest_t g_telemetry_latest_wheel[WHEEL_CHANNEL_COUNT];
static uint16_t g_telemetry_latest_adc_raw[ADC_CHANNEL_COUNT];
static uint8_t g_telemetry_adc_seen = 0U;
static uint64_t g_telemetry_next_snapshot_us = 0ULL;
static uint32_t g_telemetry_snapshot_seq = 0U;
static uint32_t g_speed_filtered_centi_kmh = 0U;
static uint32_t g_speed_last_pulse_ms = 0U;
static uint32_t g_speed_last_log_ms = 0U;
static uint8_t g_speed_filter_valid = 0U;
static uint8_t g_speed_seen_pulse = 0U;
static RaceLoggerState_t g_race_logger_state = RACE_LOGGER_WAIT_CAN;
static RaceCANLatest_t g_race_can_latest;
static uint32_t g_race_start_due_tick = 0U;
static uint32_t g_race_nextion_last_rpm_tick = 0U;
static uint32_t g_race_nextion_last_gear_tick = 0U;
static uint32_t g_race_nextion_last_data_tick = 0U;
static uint32_t g_race_can_seen_count = 0U;
static uint32_t g_race_log_seq = 0U;
static volatile uint32_t g_race_log_count = 0U;
static volatile uint32_t g_race_start_trigger_count = 0U;
static volatile uint32_t g_race_nextion_tx_count = 0U;
static volatile uint32_t g_race_nextion_tx_fail_count = 0U;
static uint16_t g_nextion_latest_rpm = 0U;
static uint8_t g_nextion_rpm_valid = 0U;
static uint32_t g_nextion_last_tx_tick = 0U;
static uint32_t g_nextion_hmi_last_rpm_tick = 0U;
static uint32_t g_nextion_hmi_last_gear_tick = 0U;
static uint32_t g_nextion_hmi_last_data_tick = 0U;
static uint16_t g_nextion_hmi_rpm = 0U;
static uint16_t g_nextion_hmi_water = 0U;
static uint16_t g_nextion_hmi_oil = 0U;
static uint16_t g_nextion_hmi_speed = 0U;
static uint32_t g_nextion_hmi_random_state = 0x12345678UL;
static uint8_t g_nextion_hmi_gear_index = 0U;
static volatile uint32_t g_nextion_hmi_tx_count = 0U;
static volatile uint32_t g_nextion_hmi_tx_fail_count = 0U;
static volatile uint16_t g_current_rpm = 0U;
static uint32_t g_test_rpm_sweep_start_tick = 0U;
static uint8_t g_rpm_led_mode = RPM_LED_MODE_DEFAULT;
static uint32_t g_ws2812_last_update_tick = 0U;
static uint16_t g_ws2812_pwm_buffer[WS2812_BUFFER_LEN];
static volatile uint8_t g_ws2812_dma_ready = 1U;
static volatile uint32_t g_ws2812_dma_done_count = 0U;
static volatile uint32_t g_ws2812_dma_start_fail_count = 0U;
static uint16_t g_ws2812_pwm_0 = 0U;
static uint16_t g_ws2812_pwm_1 = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
static GPIO_PinState SD_IsCardInserted(void);
static FRESULT SD_PulseLoggerInit(void) MAYBE_UNUSED;
static FRESULT SD_OpenNextLogFile(void);
static FRESULT SD_PulseLoggerProcess(void) MAYBE_UNUSED;
static FRESULT SD_PulseLoggerAppendSample(uint32_t timestamp_ms, uint32_t speed_centi_kmh);
static FRESULT SD_PulseLoggerFlush(uint8_t force_sync);
static void SD_LogWriterReset(FIL *file);
static FRESULT SD_LogWriterAppend(const uint8_t *data, UINT len);
static FRESULT SD_LogWriterService(uint32_t flush_idle_ms, uint32_t sync_interval_ms);
static FRESULT SD_LogWriterFlush(uint8_t force_sync, SD_LogFlushReason_t reason);
static void Speed_ProcessPulse(const Wheel_LogEvent_t *event);
static void Speed_UpdateTimeout(uint32_t now_ms);
static void TIM1_WheelCaptureStart(void) MAYBE_UNUSED;
static uint8_t Wheel_QueuePop(Wheel_LogEvent_t *event);
static void Wheel_QueuePushFromIsr(const Wheel_LogEvent_t *event);
static uint8_t ADC_LoggerConfigureHardware(void) MAYBE_UNUSED;
static FRESULT SD_ADCLoggerInit(void) MAYBE_UNUSED;
static FRESULT SD_OpenNextADCLogFile(void);
static FRESULT SD_ADCLoggerProcess(void) MAYBE_UNUSED;
static FRESULT SD_ADCLoggerAppendSample(const ADC_LogSample_t *sample);
static void ADC_LoggerStart(void) MAYBE_UNUSED;
static uint8_t ADC_QueuePop(ADC_LogSample_t *sample);
static void ADC_QueuePushFromIsr(const uint16_t *raw);
static uint8_t EMU_CANLoggerConfigureHardware(void) MAYBE_UNUSED;
static void EMU_CANLoggerStart(void) MAYBE_UNUSED;
static void EMU_CANDrainRxFifo(void);
static uint8_t EMU_CANQueuePop(EMU_CAN_Frame_t *frame);
static void EMU_CANQueuePushFromIsr(const CAN_RxHeaderTypeDef *rx_header,
                                    const uint8_t *data);
static FRESULT SD_EMUCANLoggerInit(void) MAYBE_UNUSED;
static FRESULT SD_OpenNextEMUCANLogFile(void);
static FRESULT SD_EMUCANLoggerProcess(void) MAYBE_UNUSED;
static FRESULT SD_EMUCANLoggerAppendFrame(const EMU_CAN_Frame_t *frame) MAYBE_UNUSED;
static void TelemetryLoggerStart(void) MAYBE_UNUSED;
static uint64_t Telemetry_GetTimestampUs(void);
static uint8_t Wheel4_ChannelIndex(const TIM_HandleTypeDef *htim);
static uint32_t Wheel4_HALChannel(uint8_t channel);
static TIM_HandleTypeDef *Wheel4_HALTimer(uint8_t channel);
static void Wheel4_QueuePushFromIsr(const Wheel4_LogEvent_t *event);
static uint8_t Wheel4_QueuePop(Wheel4_LogEvent_t *event);
static void ADC6_QueuePushFromIsr(const uint16_t *raw);
static uint8_t ADC6_QueuePop(ADC6_LogSample_t *sample);
static void SD_WaitForCardInserted(void) MAYBE_UNUSED;
static FRESULT SD_TelemetryLoggerInit(void) MAYBE_UNUSED;
static FRESULT SD_OpenNextTelemetryLogFile(void);
static FRESULT SD_TelemetryLoggerProcess(void) MAYBE_UNUSED;
static FRESULT SD_TelemetryLoggerDrainInputs(void);
static FRESULT SD_TelemetryLoggerAppendSnapshot(uint64_t timestamp_us);
static void Nextion_HMITestInit(void) MAYBE_UNUSED;
static void Nextion_HMITestProcess(void) MAYBE_UNUSED;
static void Nextion_HMITestTransmitValues(void);
static void Nextion_SendTextCommand(const char *component, const char *text);
static uint16_t Nextion_TestRandomRange(uint16_t min_value, uint16_t max_value);
static void TestRPM_UpdateSlowSweep(void) MAYBE_UNUSED;
static void RPMOutputs_Init(void);
static void RPMOutputs_Process(void);
static void RPMOutputs_SetRPM(uint16_t rpm);
static void WS2812_InitTimings(void);
static uint8_t PercentToByte(uint8_t percent);
static void WS2812_SetLedBits(uint32_t *index, uint8_t red, uint8_t green, uint8_t blue);
static uint8_t WS2812_WaitUntilReady(uint32_t timeout_ms);
static void WS2812_ShowRPM(uint16_t rpm, uint8_t mode);
static void WS2812_ShowRPMBar(uint16_t rpm);
static void WS2812_SendBuffer(uint32_t used_len);
static void Nextion_ProcessRPMTx(void);
static void Nextion_SendRPM(uint16_t rpm);
static void RaceLoggerReset(void);
static uint8_t RaceLoggerConfigureAndStartCAN(void);
static FRESULT RaceLoggerProcess(void);
static void RaceLoggerUpdateLatest(const EMU_CAN_Frame_t *frame);
static FRESULT SD_RaceLoggerInit(void);
static FRESULT SD_OpenNextRaceLogFile(void);
static FRESULT SD_RaceLoggerAppendFrame(const EMU_CAN_Frame_t *frame);
static void RaceNextionProcess(void);
static void RaceNextionSendNumber(const char *component, int32_t value);
static FRESULT SD_TelemetryLoggerAppendCANFrame(const EMU_CAN_Frame_t *frame) MAYBE_UNUSED;
static FRESULT SD_TelemetryLoggerAppendWheelEvent(const Wheel4_LogEvent_t *event) MAYBE_UNUSED;
static FRESULT SD_TelemetryLoggerAppendADC6Sample(const ADC6_LogSample_t *sample) MAYBE_UNUSED;
static FRESULT SD_GNSSLoggerInit(void) MAYBE_UNUSED;
static FRESULT SD_OpenNextGNSSLogFile(void);
static void GNSSLoggerStart(void) MAYBE_UNUSED;
static FRESULT SD_GNSSLoggerProcess(void) MAYBE_UNUSED;
static void GNSS_QueuePushFromIsr(uint8_t byte);
static uint8_t GNSS_QueuePop(uint8_t *byte) MAYBE_UNUSED;
static UINT GNSS_QueuePopChunk(uint8_t *buffer, UINT max_len);
static void SD_RecordFault(SD_FaultStage_t stage, FRESULT result, uint32_t detail);
static void SD_WriteFaultLog(void);
static const char *SD_FaultStageName(SD_FaultStage_t stage);
static void SD_RecoverAfterIoError(void);
static uint8_t SD_WaitForTransferReady(uint32_t timeout_ms);
static void LED_Set(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
static void LED_BlinkCode(uint8_t code);
static void LED_BlinkCodeDetail(uint8_t code, uint8_t detail);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);

  for (uint8_t i = 0U; i < 3U; i++)
  {
    LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_SET);
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_SET);
    HAL_Delay(120U);
    LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);
    HAL_Delay(120U);
  }

#if (RACE_CAN_LOGGER_MODE != 0U)
  SD_WaitForCardInserted();

  RaceLoggerReset();
  RPMOutputs_Init();
  if (RaceLoggerConfigureAndStartCAN() == 0U)
  {
    LED_BlinkCode(SD_LED_ERR_CAN_CONFIG);
  }
#else
#if (NEXTION_HMI_TEST_MODE != 0U)
  Nextion_HMITestInit();
  RPMOutputs_Init();
#else
  SD_WaitForCardInserted();

#if (GNSS_LOGGER_MODE != 0U)
  g_sd_last_fresult = SD_GNSSLoggerInit();
  if (g_sd_last_fresult != FR_OK)
  {
    LED_BlinkCodeDetail(g_sd_bringup_status, (uint8_t)g_sd_last_fresult);
  }

  GNSSLoggerStart();
#else
  /* Preserved full telemetry mode.
   * CAN is optional so logging can still start with ADC and wheel speed even if
   * the ECU/CAN bus is disconnected during bench testing.
   */
  g_emu_can_enabled = EMU_CANLoggerConfigureHardware();

  if (ADC_LoggerConfigureHardware() == 0U)
  {
    LED_BlinkCode(SD_LED_ERR_ADC_CONFIG);
  }

  g_sd_last_fresult = SD_TelemetryLoggerInit();
  if (g_sd_last_fresult != FR_OK)
  {
    LED_BlinkCodeDetail(g_sd_bringup_status, (uint8_t)g_sd_last_fresult);
  }

  TelemetryLoggerStart();
  if (g_emu_can_enabled != 0U)
  {
    EMU_CANLoggerStart();
  }
#endif
#endif
#endif

  LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if (RACE_CAN_LOGGER_MODE != 0U)
    g_sd_last_fresult = RaceLoggerProcess();
    RPMOutputs_Process();
    if (g_sd_last_fresult != FR_OK)
    {
      LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);
      LED_BlinkCodeDetail(SD_LED_ERR_WRITE_LOG, (uint8_t)g_sd_last_fresult);
    }
#else
#if (NEXTION_HMI_TEST_MODE != 0U)
    TestRPM_UpdateSlowSweep();
    Nextion_HMITestProcess();
    RPMOutputs_Process();
#else
#if (GNSS_LOGGER_MODE != 0U)
    /* Drain USART1 bytes into GNSS_xxx.nmea and periodically sync FATFS. */
    g_sd_last_fresult = SD_GNSSLoggerProcess();
#else
    /* Build periodic telemetry snapshots from the latest CAN/ADC/TIM inputs. */
    g_sd_last_fresult = SD_TelemetryLoggerProcess();
#endif
    if (g_sd_last_fresult != FR_OK)
    {
      LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);
      LED_BlinkCodeDetail(SD_LED_ERR_WRITE_LOG, (uint8_t)g_sd_last_fresult);
    }
#endif
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDIO_Init 2 */
  /* The logger writes slowly, so prefer signal margin over bus speed. Hardware
   * flow control also lets SDIO pause the clock while the FIFO is tight, which
   * is useful with long wires or marginal cards.
   */
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd.Init.ClockDiv = 12;

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 89;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USER_LED_0_Pin|USER_LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_Detect_Pin */
  GPIO_InitStruct.Pin = SD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_LED_0_Pin USER_LED_1_Pin */
  GPIO_InitStruct.Pin = USER_LED_0_Pin|USER_LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_Detection_Pin */
  GPIO_InitStruct.Pin = VBUS_Detection_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_Detection_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void SD_RecordFault(SD_FaultStage_t stage, FRESULT result, uint32_t detail)
{
  g_sd_fault_stage = stage;
  g_sd_fault_fresult = result;
  g_sd_fault_detail = detail;
  g_sd_fault_tick = HAL_GetTick();
  g_sd_fault_buffer_len = g_sd_log_buffer_len;
  g_sd_fault_unsynced_count = g_sd_log_unsynced_count;
  g_sd_fault_telemetry_count = g_telemetry_log_count;

  SD_WriteFaultLog();
}

static void SD_WriteFaultLog(void)
{
  static uint8_t fault_log_busy = 0U;
  FIL fault_file;
  UINT bytes_done = 0U;
  char line[768];
  int line_len;
  FRESULT res;

  if (fault_log_busy != 0U)
  {
    return;
  }

  fault_log_busy = 1U;
  line_len = snprintf(line, sizeof(line),
                      "tick=%lu,stage=%lu,%s,fresult=%u,detail=%lu,"
                      "buf_len=%u,unsynced=%lu,telemetry=%lu,"
                      "last_write_tick=%lu,last_sync_tick=%lu,"
                      "hal_error=0x%08lX,hal_stage=%lu,"
                      "writer_bytes=%lu,write_calls=%lu,last_write=%lu,write_fail=%lu,"
                      "sync_ok=%lu,sync_fail=%lu,last_sync_res=%u,flush_reason=%lu,"
                      "ll_op=%lu,ll_rd=%lu,ll_wr=%lu,ll_retry=%lu,ll_rd_fail=%lu,"
                      "ll_wr_fail=%lu,ll_ready_to=%lu,ll_sector=%lu,ll_blocks=%lu,"
                      "ll_buf=0x%08lX,ll_hal=%lu,ll_err=0x%08lX,ll_state=%lu,"
                      "ll_card=%lu,ll_wait=%lu,ll_tick=%lu,"
                      "gnss_rx=%lu,gnss_drain=%lu,gnss_ovf=%lu,gnss_uart=%lu,"
                      "can_rx=%lu,can_ovf=%lu,can_hal=%lu,wheel_ovf=%lu,adc_ovf=%lu,"
                      "active_path=%s\r\n",
                      (unsigned long)g_sd_fault_tick,
                      (unsigned long)g_sd_fault_stage,
                      SD_FaultStageName(g_sd_fault_stage),
                      (unsigned int)g_sd_fault_fresult,
                      (unsigned long)g_sd_fault_detail,
                      (unsigned int)g_sd_fault_buffer_len,
                      (unsigned long)g_sd_fault_unsynced_count,
                      (unsigned long)g_sd_fault_telemetry_count,
                      (unsigned long)g_sd_log_last_write_tick,
                      (unsigned long)g_sd_log_last_sync_tick,
                      (unsigned long)hsd.ErrorCode,
                      (unsigned long)g_sd_hal_stage,
                      (unsigned long)g_sd_total_bytes_written,
                      (unsigned long)g_sd_write_call_count,
                      (unsigned long)g_sd_last_write_size,
                      (unsigned long)g_sd_write_fail_count,
                      (unsigned long)g_sd_sync_ok_count,
                      (unsigned long)g_sd_sync_fail_count,
                      (unsigned int)g_sd_last_sync_fresult,
                      (unsigned long)g_sd_last_flush_reason,
                      (unsigned long)g_sd_ll_last_op,
                      (unsigned long)g_sd_ll_read_call_count,
                      (unsigned long)g_sd_ll_write_call_count,
                      (unsigned long)g_sd_ll_retry_count,
                      (unsigned long)g_sd_ll_read_fail_count,
                      (unsigned long)g_sd_ll_write_fail_count,
                      (unsigned long)g_sd_ll_ready_timeout_count,
                      (unsigned long)g_sd_ll_last_sector,
                      (unsigned long)g_sd_ll_last_blocks,
                      (unsigned long)g_sd_ll_last_buffer_addr,
                      (unsigned long)g_sd_ll_last_hal_status,
                      (unsigned long)g_sd_ll_last_hal_error,
                      (unsigned long)g_sd_ll_last_hal_state,
                      (unsigned long)g_sd_ll_last_card_state,
                      (unsigned long)g_sd_ll_last_wait_ms,
                      (unsigned long)g_sd_ll_last_tick,
                      (unsigned long)g_gnss_rx_count,
                      (unsigned long)g_gnss_drain_byte_count,
                      (unsigned long)g_gnss_rx_overflow_count,
                      (unsigned long)g_gnss_uart_error_count,
                      (unsigned long)g_emu_can_rx_count,
                      (unsigned long)g_emu_can_rx_overflow_count,
                      (unsigned long)g_emu_can_hal_error_count,
                      (unsigned long)g_wheel4_rx_overflow_count,
                      (unsigned long)g_adc6_rx_overflow_count,
                      g_sd_log_path);

  if ((line_len <= 0) || ((size_t)line_len >= sizeof(line)))
  {
    g_sd_fault_log_fresult = FR_INT_ERR;
    fault_log_busy = 0U;
    return;
  }

  res = f_open(&fault_file, SD_FAULT_LOG_FILE, FA_OPEN_APPEND | FA_WRITE);
  if (res == FR_OK)
  {
    res = f_write(&fault_file, line, (UINT)line_len, &bytes_done);
    if ((res == FR_OK) && (bytes_done != (UINT)line_len))
    {
      res = FR_DISK_ERR;
    }

    if (res == FR_OK)
    {
      res = f_close(&fault_file);
    }
    else
    {
      (void)f_close(&fault_file);
    }
  }

  g_sd_fault_log_fresult = res;
  fault_log_busy = 0U;
}

static const char *SD_FaultStageName(SD_FaultStage_t stage)
{
  switch (stage)
  {
    case SD_FAULT_NO_CARD:
      return "NO_CARD";
    case SD_FAULT_MOUNT:
      return "MOUNT";
    case SD_FAULT_OPEN_TELEMETRY:
      return "OPEN_TELEMETRY";
    case SD_FAULT_HEADER_WRITE:
      return "HEADER_WRITE";
    case SD_FAULT_HEADER_SHORT_WRITE:
      return "HEADER_SHORT_WRITE";
    case SD_FAULT_HEADER_SYNC:
      return "HEADER_SYNC";
    case SD_FAULT_TELEMETRY_DRAIN:
      return "TELEMETRY_DRAIN";
    case SD_FAULT_SNAPSHOT_FORMAT:
      return "SNAPSHOT_FORMAT";
    case SD_FAULT_SNAPSHOT_FLUSH:
      return "SNAPSHOT_FLUSH";
    case SD_FAULT_SNAPSHOT_BUFFER_FULL:
      return "SNAPSHOT_BUFFER_FULL";
    case SD_FAULT_IDLE_FLUSH:
      return "IDLE_FLUSH";
    case SD_FAULT_INTERVAL_SYNC:
      return "INTERVAL_SYNC";
    case SD_FAULT_FLUSH_NO_ACTIVE_FILE:
      return "FLUSH_NO_ACTIVE_FILE";
    case SD_FAULT_FLUSH_WRITE:
      return "FLUSH_WRITE";
    case SD_FAULT_FLUSH_SHORT_WRITE:
      return "FLUSH_SHORT_WRITE";
    case SD_FAULT_FLUSH_SYNC:
      return "FLUSH_SYNC";
    case SD_FAULT_NONE:
    default:
      return "NONE";
  }
}

static void SD_RecoverAfterIoError(void)
{
  HAL_SD_Abort(&hsd);
  (void)HAL_SD_DeInit(&hsd);
  HAL_Delay(SD_LOG_IO_RETRY_DELAY_MS);
  (void)HAL_SD_Init(&hsd);

  if (SD_BRINGUP_USE_4BIT_BUS != 0U)
  {
    (void)HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);
  }

  HAL_Delay(SD_LOG_IO_RETRY_DELAY_MS);
}

static uint8_t SD_WaitForTransferReady(uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();
  HAL_SD_CardStateTypeDef card_state;

  do
  {
    card_state = HAL_SD_GetCardState(&hsd);
    g_sd_ll_last_card_state = card_state;
    g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);
    g_sd_ll_last_hal_error = hsd.ErrorCode;

    if (card_state == HAL_SD_CARD_TRANSFER)
    {
      g_sd_ll_last_wait_ms = HAL_GetTick() - start_tick;
      return MSD_OK;
    }
  } while ((HAL_GetTick() - start_tick) < timeout_ms);

  g_sd_ll_ready_timeout_count++;
  g_sd_ll_last_wait_ms = HAL_GetTick() - start_tick;
  g_sd_hal_error_code = hsd.ErrorCode;
  return MSD_ERROR;
}

static void RaceLoggerReset(void)
{
  uint32_t now = HAL_GetTick();

  memset(&g_race_can_latest, 0, sizeof(g_race_can_latest));
  g_race_logger_state = RACE_LOGGER_WAIT_CAN;
  g_race_start_due_tick = 0U;
  g_race_nextion_last_rpm_tick = now - RACE_NEXTION_RPM_UPDATE_MS;
  g_race_nextion_last_gear_tick = now - RACE_NEXTION_GEAR_UPDATE_MS;
  g_race_nextion_last_data_tick = now - RACE_NEXTION_DATA_UPDATE_MS;
  g_race_can_seen_count = 0U;
  g_race_log_seq = 0U;
  g_race_log_count = 0U;
  g_race_start_trigger_count = 0U;
  g_race_nextion_tx_count = 0U;
  g_race_nextion_tx_fail_count = 0U;
  g_sd_active_file = NULL;
  g_sd_log_buffer_len = 0U;

  g_tim1_overflow_count = 0U;
  g_tim1_timebase_ready = 0U;
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  if (HAL_TIM_Base_Start_IT(&htim1) == HAL_OK)
  {
    g_tim1_timebase_ready = 1U;
  }
}

static uint8_t RaceLoggerConfigureAndStartCAN(void)
{
  g_emu_can_enabled = EMU_CANLoggerConfigureHardware();
  if (g_emu_can_enabled == 0U)
  {
    return 0U;
  }

  EMU_CANLoggerStart();
  return g_emu_can_enabled;
}

static FRESULT RaceLoggerProcess(void)
{
  FRESULT res = FR_OK;
  EMU_CAN_Frame_t frame;
  uint32_t now = HAL_GetTick();

  if ((g_race_logger_state == RACE_LOGGER_START_DELAY) &&
      ((int32_t)(now - g_race_start_due_tick) >= 0))
  {
    res = SD_RaceLoggerInit();
    if (res != FR_OK)
    {
      return res;
    }

    g_race_logger_state = RACE_LOGGER_LOGGING;
  }

  while (EMU_CANQueuePop(&frame) != 0U)
  {
    g_race_can_seen_count++;
    RaceLoggerUpdateLatest(&frame);
    HAL_GPIO_TogglePin(USER_LED_0_GPIO_Port, USER_LED_0_Pin);

    if ((g_race_logger_state == RACE_LOGGER_WAIT_CAN) &&
        (g_race_can_latest.rpm_valid != 0U) &&
        (g_race_can_latest.rpm >= RACE_LOG_RPM_START_THRESHOLD))
    {
      g_race_logger_state = RACE_LOGGER_START_DELAY;
      g_race_start_due_tick = HAL_GetTick() + RACE_LOG_START_DELAY_MS;
      g_race_start_trigger_count++;
    }

    if (g_race_logger_state == RACE_LOGGER_LOGGING)
    {
      res = SD_RaceLoggerAppendFrame(&frame);
      if (res != FR_OK)
      {
        return res;
      }
    }
  }

  now = HAL_GetTick();
  if ((g_race_logger_state == RACE_LOGGER_START_DELAY) &&
      ((int32_t)(now - g_race_start_due_tick) >= 0))
  {
    res = SD_RaceLoggerInit();
    if (res != FR_OK)
    {
      return res;
    }

    g_race_logger_state = RACE_LOGGER_LOGGING;
  }

  if (g_race_logger_state == RACE_LOGGER_LOGGING)
  {
    res = SD_LogWriterService(RACE_LOG_FLUSH_IDLE_MS, RACE_LOG_SYNC_INTERVAL_MS);
    if (res != FR_OK)
    {
      return res;
    }
  }

  RaceNextionProcess();

  if (g_race_logger_state == RACE_LOGGER_LOGGING)
  {
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_SET);
  }
  else if (g_race_can_seen_count != 0U)
  {
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin,
            (((HAL_GetTick() / 250U) & 1U) == 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else
  {
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);
  }

  return FR_OK;
}

static void RaceLoggerUpdateLatest(const EMU_CAN_Frame_t *frame)
{
  const uint8_t *data = frame->data;

  if ((frame->std_id == (EMU_CAN_BASE_ID + 0U)) && (frame->dlc >= 8U))
  {
    g_race_can_latest.rpm = (uint16_t)((uint16_t)data[0] |
                                       ((uint16_t)data[1] << 8));
    g_race_can_latest.rpm_valid = 1U;
    RPMOutputs_SetRPM(g_race_can_latest.rpm);
  }
  else if ((frame->std_id == (EMU_CAN_BASE_ID + 2U)) && (frame->dlc >= 8U))
  {
    g_race_can_latest.speed_kmh = (uint16_t)((uint16_t)data[0] |
                                             ((uint16_t)data[1] << 8));
    g_race_can_latest.oil_c = (int16_t)data[3];
    g_race_can_latest.water_c = (int16_t)((uint16_t)data[6] |
                                          ((uint16_t)data[7] << 8));
    g_race_can_latest.speed_valid = 1U;
    g_race_can_latest.oil_valid = 1U;
    g_race_can_latest.water_valid = 1U;
  }
  else if ((frame->std_id == (EMU_CAN_BASE_ID + 4U)) && (frame->dlc >= 8U))
  {
    g_race_can_latest.gear = data[0];
    g_race_can_latest.battery_mV =
        (uint32_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8)) * 27U;
    g_race_can_latest.gear_valid = 1U;
    g_race_can_latest.battery_valid = 1U;
  }
}

static FRESULT SD_RaceLoggerInit(void)
{
  FRESULT res;
  UINT bytes_done = 0U;
  static const char header[] =
      "timestamp_us,seq,std_id,dlc,d0,d1,d2,d3,d4,d5,d6,d7,"
      "rpm,water_c,oil_c,speed_kmh,gear,battery_mV\r\n";

  g_sd_detect_state = SD_IsCardInserted();
  if (g_sd_detect_state == GPIO_PIN_RESET)
  {
    g_sd_bringup_status = SD_LED_ERR_FATFS;
    SD_RecordFault(SD_FAULT_NO_CARD, FR_NOT_READY, 0U);
    return FR_NOT_READY;
  }

  res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_MOUNT;
    SD_RecordFault(SD_FAULT_MOUNT, res, 0U);
    return res;
  }

  res = SD_OpenNextRaceLogFile();
  if (res != FR_OK)
  {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 0U);
    g_sd_bringup_status = SD_LED_ERR_OPEN_EMU;
    SD_RecordFault(SD_FAULT_OPEN_TELEMETRY, res, 0U);
    return res;
  }

  res = f_write(&g_emu_can_file, header, (UINT)(sizeof(header) - 1U), &bytes_done);
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_WRITE_HEADER;
    SD_RecordFault(SD_FAULT_HEADER_WRITE, res, bytes_done);
    return res;
  }

  if (bytes_done != (sizeof(header) - 1U))
  {
    g_sd_bringup_status = SD_LED_ERR_WRITE_HEADER;
    SD_RecordFault(SD_FAULT_HEADER_SHORT_WRITE, FR_DISK_ERR, bytes_done);
    return FR_DISK_ERR;
  }

  res = f_sync(&g_emu_can_file);
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_SYNC;
    SD_RecordFault(SD_FAULT_HEADER_SYNC, res, 0U);
    return res;
  }

  SD_LogWriterReset(&g_emu_can_file);
  g_race_log_seq = 0U;
  g_race_log_count = 0U;
  g_sd_bringup_status = 0U;
  return FR_OK;
}

static FRESULT SD_OpenNextRaceLogFile(void)
{
  FRESULT res;

  for (uint16_t i = 1U; i <= SD_LOG_FILE_MAX_INDEX; i++)
  {
    (void)snprintf(g_sd_log_path, sizeof(g_sd_log_path), "0:/loggedfile%03u.csv", i);
    res = f_open(&g_emu_can_file, g_sd_log_path, FA_CREATE_NEW | FA_WRITE);
    if (res == FR_OK)
    {
      return FR_OK;
    }

    if (res != FR_EXIST)
    {
      return res;
    }
  }

  return FR_EXIST;
}

static FRESULT SD_RaceLoggerAppendFrame(const EMU_CAN_Frame_t *frame)
{
  char line[192];
  int line_len;

  line_len = snprintf(line, sizeof(line),
                      "%lu%06lu,%lu,0x%03lX,%u,"
                      "%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,"
                      "%u,%d,%d,%u,%u,%lu\r\n",
                      (unsigned long)(frame->timestamp_us / 1000000ULL),
                      (unsigned long)(frame->timestamp_us % 1000000ULL),
                      (unsigned long)g_race_log_seq,
                      (unsigned long)frame->std_id,
                      (unsigned int)frame->dlc,
                      (unsigned int)frame->data[0],
                      (unsigned int)frame->data[1],
                      (unsigned int)frame->data[2],
                      (unsigned int)frame->data[3],
                      (unsigned int)frame->data[4],
                      (unsigned int)frame->data[5],
                      (unsigned int)frame->data[6],
                      (unsigned int)frame->data[7],
                      (unsigned int)((g_race_can_latest.rpm_valid != 0U) ? g_race_can_latest.rpm : 0U),
                      (int)((g_race_can_latest.water_valid != 0U) ? g_race_can_latest.water_c : 0),
                      (int)((g_race_can_latest.oil_valid != 0U) ? g_race_can_latest.oil_c : 0),
                      (unsigned int)((g_race_can_latest.speed_valid != 0U) ? g_race_can_latest.speed_kmh : 0U),
                      (unsigned int)((g_race_can_latest.gear_valid != 0U) ? g_race_can_latest.gear : 0U),
                      (unsigned long)((g_race_can_latest.battery_valid != 0U) ? g_race_can_latest.battery_mV : 0U));

  if ((line_len <= 0) || ((size_t)line_len >= sizeof(line)))
  {
    SD_RecordFault(SD_FAULT_SNAPSHOT_FORMAT, FR_INT_ERR, (uint32_t)sizeof(line));
    return FR_INT_ERR;
  }

  g_race_log_seq++;
  g_race_log_count++;
  return SD_LogWriterAppend((const uint8_t *)line, (UINT)line_len);
}

static void RaceNextionProcess(void)
{
  uint32_t now = HAL_GetTick();

  if (((now - g_race_nextion_last_rpm_tick) >= RACE_NEXTION_RPM_UPDATE_MS) &&
      (g_race_can_latest.rpm_valid != 0U))
  {
    g_race_nextion_last_rpm_tick = now;
    RaceNextionSendNumber("t_rpm", (int32_t)g_race_can_latest.rpm);
  }

  if (((now - g_race_nextion_last_gear_tick) >= RACE_NEXTION_GEAR_UPDATE_MS) &&
      (g_race_can_latest.gear_valid != 0U))
  {
    g_race_nextion_last_gear_tick = now;
    RaceNextionSendNumber("t_gear", (int32_t)g_race_can_latest.gear);
  }

  if (((now - g_race_nextion_last_data_tick) >= RACE_NEXTION_DATA_UPDATE_MS) &&
      ((g_race_can_latest.water_valid != 0U) ||
       (g_race_can_latest.oil_valid != 0U) ||
       (g_race_can_latest.speed_valid != 0U)))
  {
    g_race_nextion_last_data_tick = now;
    if (g_race_can_latest.water_valid != 0U)
    {
      RaceNextionSendNumber("t_water", (int32_t)g_race_can_latest.water_c);
    }
    if (g_race_can_latest.oil_valid != 0U)
    {
      RaceNextionSendNumber("t_oil", (int32_t)g_race_can_latest.oil_c);
    }
    if (g_race_can_latest.speed_valid != 0U)
    {
      RaceNextionSendNumber("t_speed", (int32_t)g_race_can_latest.speed_kmh);
    }
  }
}

static void RaceNextionSendNumber(const char *component, int32_t value)
{
  char text[16];

  (void)snprintf(text, sizeof(text), "%ld", (long)value);
  Nextion_SendTextCommand(component, text);
}

static uint8_t EMU_CANLoggerConfigureHardware(void)
{
  CAN_FilterTypeDef can_filter = {0};

  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = (uint16_t)(EMU_CAN_BASE_ID << 5);
  can_filter.FilterIdLow = 0x0000U;
  can_filter.FilterMaskIdHigh = (uint16_t)(EMU_CAN_ID_MASK << 5);
  can_filter.FilterMaskIdLow = 0x0000U;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank = 14;

  return (HAL_CAN_ConfigFilter(&hcan1, &can_filter) == HAL_OK) ? 1U : 0U;
}

static void EMU_CANLoggerStart(void)
{
  g_emu_can_rx_head = 0U;
  g_emu_can_rx_tail = 0U;
  g_emu_can_rx_count = 0U;
  g_emu_can_log_count = 0U;
  g_emu_can_rx_overflow_count = 0U;
  g_emu_can_hal_error_count = 0U;

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    g_emu_can_enabled = 0U;
    return;
  }

  if (HAL_CAN_ActivateNotification(&hcan1,
                                    CAN_IT_RX_FIFO0_MSG_PENDING |
                                    CAN_IT_RX_FIFO0_FULL |
                                    CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK)
  {
    g_emu_can_enabled = 0U;
    return;
  }

  EMU_CANDrainRxFifo();
}

static void EMU_CANDrainRxFifo(void)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t data[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, data) != HAL_OK)
    {
      g_emu_can_hal_error_count++;
      return;
    }

    if ((rx_header.IDE == CAN_ID_STD) &&
        (rx_header.RTR == CAN_RTR_DATA) &&
        ((rx_header.StdId & EMU_CAN_ID_MASK) == EMU_CAN_BASE_ID))
    {
      EMU_CANQueuePushFromIsr(&rx_header, data);
    }
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance != CAN1)
  {
    return;
  }

  EMU_CANDrainRxFifo();
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance != CAN1)
  {
    return;
  }

  EMU_CANDrainRxFifo();
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    g_emu_can_hal_error_count++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if (GNSS_LOGGER_MODE != 0U)
  if (huart->Instance == USART1)
  {
    /* USART1 is connected to the GNSS module TX pin. Store exactly the byte
     * received by the module, then re-arm reception for the next byte.
     */
    GNSS_QueuePushFromIsr(g_gnss_rx_byte);
    if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_gnss_rx_byte, 1U) != HAL_OK)
    {
      g_gnss_uart_error_count++;
    }
  }
#else
  (void)huart;
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
#if (GNSS_LOGGER_MODE != 0U)
  if (huart->Instance == USART1)
  {
    /* Framing, parity, noise, or overrun errors can stop byte reception in the
     * HAL state machine. Count the event and immediately try to resume logging
     * so a noisy GNSS line does not permanently stop the file.
     */
    g_gnss_uart_error_count++;
    if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_gnss_rx_byte, 1U) != HAL_OK)
    {
      g_gnss_uart_error_count++;
    }
  }
#else
  (void)huart;
#endif
}

static void EMU_CANQueuePushFromIsr(const CAN_RxHeaderTypeDef *rx_header,
                                    const uint8_t *data)
{
  EMU_CAN_Frame_t frame;
  uint16_t next_head = (uint16_t)((g_emu_can_rx_head + 1U) % EMU_CAN_RX_QUEUE_LEN);
  uint8_t dlc = (rx_header->DLC > 8U) ? 8U : rx_header->DLC;

  if (next_head == g_emu_can_rx_tail)
  {
    g_emu_can_rx_overflow_count++;
    return;
  }

  frame.timestamp_us = Telemetry_GetTimestampUs();
  frame.timestamp_ms = (uint32_t)(frame.timestamp_us / 1000ULL);
  frame.std_id = rx_header->StdId;
  frame.dlc = dlc;
  memset(frame.data, 0, sizeof(frame.data));
  memcpy(frame.data, data, dlc);

  g_emu_can_rx_queue[g_emu_can_rx_head] = frame;
  g_emu_can_rx_head = next_head;
  g_emu_can_rx_count++;
}

static uint8_t EMU_CANQueuePop(EMU_CAN_Frame_t *frame)
{
  uint8_t has_frame = 0U;

  __disable_irq();
  if (g_emu_can_rx_tail != g_emu_can_rx_head)
  {
    *frame = g_emu_can_rx_queue[g_emu_can_rx_tail];
    g_emu_can_rx_tail = (uint16_t)((g_emu_can_rx_tail + 1U) % EMU_CAN_RX_QUEUE_LEN);
    has_frame = 1U;
  }
  __enable_irq();

  return has_frame;
}

static FRESULT SD_EMUCANLoggerInit(void)
{
  FRESULT res;
  UINT bytes_done = 0U;
  static const char header[] =
      "timestamp_ms,seq,std_id,dlc,d0,d1,d2,d3,d4,d5,d6,d7\r\n";

  g_sd_detect_state = SD_IsCardInserted();
  if (g_sd_detect_state == GPIO_PIN_RESET)
  {
    LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);
    g_sd_bringup_status = SD_LED_ERR_FATFS;
    return FR_NOT_READY;
  }

  res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_MOUNT;
    return res;
  }

  res = SD_OpenNextEMUCANLogFile();
  if (res != FR_OK)
  {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 0U);
    g_sd_bringup_status = SD_LED_ERR_OPEN_EMU;
    return res;
  }

  res = f_write(&g_emu_can_file, header, (UINT)(sizeof(header) - 1U), &bytes_done);
  if ((res == FR_OK) && (bytes_done != (sizeof(header) - 1U)))
  {
    res = FR_DISK_ERR;
  }
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_WRITE_HEADER;
    return res;
  }

  res = f_sync(&g_emu_can_file);
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_SYNC;
    return res;
  }

  g_emu_can_log_count = 0U;
  g_sd_active_file = &g_emu_can_file;
  g_sd_log_buffer_len = 0U;
  g_sd_log_last_write_tick = HAL_GetTick();
  g_sd_log_last_sync_tick = g_sd_log_last_write_tick;
  g_sd_log_unsynced_count = 0U;
  g_sd_bringup_status = 0U;
  return FR_OK;
}

static FRESULT SD_OpenNextEMUCANLogFile(void)
{
  FRESULT res;

  res = f_open(&g_emu_can_file, "0:/EMU.CSV", FA_CREATE_NEW | FA_WRITE);
  if (res == FR_OK)
  {
    (void)snprintf(g_sd_log_path, sizeof(g_sd_log_path), "0:/EMU.CSV");
    return FR_OK;
  }
  if (res != FR_EXIST)
  {
    return res;
  }

  for (uint16_t i = 1U; i <= SD_LOG_FILE_MAX_INDEX; i++)
  {
    (void)snprintf(g_sd_log_path, sizeof(g_sd_log_path), "0:/EMU%03u.CSV", i);
    res = f_open(&g_emu_can_file, g_sd_log_path, FA_CREATE_NEW | FA_WRITE);
    if (res == FR_OK)
    {
      return FR_OK;
    }

    if (res != FR_EXIST)
    {
      return res;
    }
  }

  return FR_EXIST;
}

static FRESULT SD_EMUCANLoggerProcess(void)
{
  FRESULT res = FR_OK;
  EMU_CAN_Frame_t frame;
  uint32_t now;

  while (EMU_CANQueuePop(&frame) != 0U)
  {
    res = SD_EMUCANLoggerAppendFrame(&frame);
    if (res != FR_OK)
    {
      return res;
    }
  }

  now = HAL_GetTick();
  if ((g_sd_log_buffer_len > 0U) &&
      ((now - g_sd_log_last_write_tick) >= SD_LOG_FLUSH_IDLE_MS))
  {
    return SD_PulseLoggerFlush(0U);
  }

  if ((g_sd_log_unsynced_count > 0U) &&
      ((now - g_sd_log_last_sync_tick) >= SD_LOG_SYNC_INTERVAL_MS))
  {
    return SD_PulseLoggerFlush(1U);
  }

  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin,
          ((g_emu_can_rx_overflow_count == 0U) &&
           (g_emu_can_hal_error_count == 0U)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  return FR_OK;
}

static FRESULT SD_EMUCANLoggerAppendFrame(const EMU_CAN_Frame_t *frame)
{
  FRESULT res;
  char line[128];
  int line_len;

  line_len = snprintf(line, sizeof(line),
                      "%lu,%lu,0x%03lX,%u,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\r\n",
                      (unsigned long)frame->timestamp_ms,
                      (unsigned long)g_emu_can_log_count,
                      (unsigned long)frame->std_id,
                      (unsigned int)frame->dlc,
                      (unsigned int)frame->data[0],
                      (unsigned int)frame->data[1],
                      (unsigned int)frame->data[2],
                      (unsigned int)frame->data[3],
                      (unsigned int)frame->data[4],
                      (unsigned int)frame->data[5],
                      (unsigned int)frame->data[6],
                      (unsigned int)frame->data[7]);

  if ((line_len <= 0) || ((size_t)line_len >= sizeof(line)))
  {
    SD_RecordFault(SD_FAULT_SNAPSHOT_FORMAT, FR_INT_ERR, (uint32_t)sizeof(line));
    return FR_INT_ERR;
  }

  if (((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len)) &&
      (g_sd_log_buffer_len > 0U))
  {
    res = SD_PulseLoggerFlush(0U);
    if (res != FR_OK)
    {
      SD_RecordFault(SD_FAULT_SNAPSHOT_FLUSH, res, g_sd_log_buffer_len);
      return res;
    }
  }

  if ((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len))
  {
    SD_RecordFault(SD_FAULT_SNAPSHOT_BUFFER_FULL, FR_INT_ERR, (uint32_t)line_len);
    return FR_INT_ERR;
  }

  memcpy(&g_sd_log_buffer[g_sd_log_buffer_len], line, (size_t)line_len);
  g_sd_log_buffer_len += (UINT)line_len;
  g_emu_can_log_count++;
  g_sd_log_unsynced_count++;
  HAL_GPIO_TogglePin(USER_LED_0_GPIO_Port, USER_LED_0_Pin);

  if ((g_sd_log_buffer_len >= (SD_LOG_BUFFER_SIZE - 128U)) ||
      ((g_sd_log_unsynced_count % SD_LOG_SYNC_EVERY) == 0U))
  {
    return SD_PulseLoggerFlush((g_sd_log_unsynced_count % SD_LOG_SYNC_EVERY) == 0U);
  }

  return FR_OK;
}

static void TelemetryLoggerStart(void)
{
  memset((void *)g_wheel4_last_timestamp_us, 0, sizeof(g_wheel4_last_timestamp_us));
  memset((void *)g_wheel4_last_delta_us, 0, sizeof(g_wheel4_last_delta_us));
  memset((void *)g_wheel4_filtered_centi_kmh, 0, sizeof(g_wheel4_filtered_centi_kmh));
  memset((void *)g_wheel4_filter_valid, 0, sizeof(g_wheel4_filter_valid));
  memset((void *)g_adc_dma_buffer, 0, sizeof(g_adc_dma_buffer));

  g_tim1_overflow_count = 0U;
  g_wheel4_rx_head = 0U;
  g_wheel4_rx_tail = 0U;
  g_adc6_rx_head = 0U;
  g_adc6_rx_tail = 0U;
  g_wheel4_rx_overflow_count = 0U;
  g_adc6_rx_overflow_count = 0U;
  g_wheel4_log_count = 0U;
  g_adc6_log_count = 0U;
  g_telemetry_log_count = 0U;
  g_wheel_legacy_queue_enabled = 0U;
  g_adc_legacy_queue_enabled = 0U;
  g_telemetry_queue_enabled = 1U;
  g_nextion_latest_rpm = 0U;
  g_nextion_rpm_valid = 0U;
  g_nextion_last_tx_tick = HAL_GetTick();

  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

  g_tim1_timebase_ready = 0U;
  if (HAL_TIM_Base_Start_IT(&htim1) == HAL_OK)
  {
    g_tim1_timebase_ready = 1U;
  }
  else
  {
    g_wheel4_rx_overflow_count++;
  }

  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    g_wheel4_rx_overflow_count++;
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_dma_buffer, ADC_CHANNEL_COUNT) != HAL_OK)
  {
    g_adc6_rx_overflow_count++;
  }

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    g_wheel4_rx_overflow_count++;
  }

  for (uint8_t channel = 1U; channel < WHEEL_CHANNEL_COUNT; channel++)
  {
    if (HAL_TIM_IC_Start_IT(Wheel4_HALTimer(channel), Wheel4_HALChannel(channel)) != HAL_OK)
    {
      g_wheel4_rx_overflow_count++;
    }
  }

  /* Align snapshot scheduling with the active runtime timestamp source. */
  g_telemetry_next_snapshot_us = Telemetry_GetTimestampUs() + TELEMETRY_SNAPSHOT_INTERVAL_US;
}

static uint64_t Telemetry_GetTimestampUs(void)
{
  uint32_t overflow_count;
  uint32_t counter;
  uint32_t primask;

  if (g_tim1_timebase_ready == 0U)
  {
    return ((uint64_t)HAL_GetTick()) * 1000ULL;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  overflow_count = g_tim1_overflow_count;
  counter = __HAL_TIM_GET_COUNTER(&htim1);
  if ((__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET) && (counter < 32768U))
  {
    overflow_count++;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  return ((uint64_t)overflow_count * TIM1_TICKS_PER_OVERFLOW) + counter;
}

static uint8_t Wheel4_ChannelIndex(const TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    return (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) ? 0U : WHEEL_CHANNEL_COUNT;
  }

  if (htim->Instance == TIM2)
  {
    switch (htim->Channel)
    {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        return 1U;
      case HAL_TIM_ACTIVE_CHANNEL_2:
        return 2U;
      case HAL_TIM_ACTIVE_CHANNEL_3:
        return 3U;
      default:
        return WHEEL_CHANNEL_COUNT;
    }
  }

  return WHEEL_CHANNEL_COUNT;
}

static uint32_t Wheel4_HALChannel(uint8_t channel)
{
  static const uint32_t hal_channels[WHEEL_CHANNEL_COUNT] =
  {
    TIM_CHANNEL_1,
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3
  };

  return hal_channels[channel];
}

static TIM_HandleTypeDef *Wheel4_HALTimer(uint8_t channel)
{
  return (channel == 0U) ? &htim1 : &htim2;
}

static void Wheel4_QueuePushFromIsr(const Wheel4_LogEvent_t *event)
{
  uint16_t next_head = (uint16_t)((g_wheel4_rx_head + 1U) % WHEEL4_RX_QUEUE_LEN);

  if (next_head == g_wheel4_rx_tail)
  {
    g_wheel4_rx_overflow_count++;
    return;
  }

  g_wheel4_rx_queue[g_wheel4_rx_head] = *event;
  g_wheel4_rx_head = next_head;
}

static uint8_t Wheel4_QueuePop(Wheel4_LogEvent_t *event)
{
  uint8_t has_event = 0U;

  __disable_irq();
  if (g_wheel4_rx_tail != g_wheel4_rx_head)
  {
    *event = g_wheel4_rx_queue[g_wheel4_rx_tail];
    g_wheel4_rx_tail = (uint16_t)((g_wheel4_rx_tail + 1U) % WHEEL4_RX_QUEUE_LEN);
    has_event = 1U;
  }
  __enable_irq();

  return has_event;
}

static void ADC6_QueuePushFromIsr(const uint16_t *raw)
{
  ADC6_LogSample_t sample;
  uint16_t next_head = (uint16_t)((g_adc6_rx_head + 1U) % ADC6_LOG_QUEUE_LEN);

  if (next_head == g_adc6_rx_tail)
  {
    g_adc6_rx_overflow_count++;
    return;
  }

  sample.timestamp_us = Telemetry_GetTimestampUs();
  for (uint32_t i = 0U; i < ADC_CHANNEL_COUNT; i++)
  {
    sample.raw[i] = raw[i];
  }

  g_adc6_rx_queue[g_adc6_rx_head] = sample;
  g_adc6_rx_head = next_head;
}

static uint8_t ADC6_QueuePop(ADC6_LogSample_t *sample)
{
  uint8_t has_sample = 0U;

  __disable_irq();
  if (g_adc6_rx_tail != g_adc6_rx_head)
  {
    *sample = g_adc6_rx_queue[g_adc6_rx_tail];
    g_adc6_rx_tail = (uint16_t)((g_adc6_rx_tail + 1U) % ADC6_LOG_QUEUE_LEN);
    has_sample = 1U;
  }
  __enable_irq();

  return has_sample;
}

static void SD_WaitForCardInserted(void)
{
  while (SD_IsCardInserted() == GPIO_PIN_RESET)
  {
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_TogglePin(USER_LED_0_GPIO_Port, USER_LED_0_Pin);
    HAL_Delay(TELEMETRY_SD_MISSING_BLINK_MS);
  }

  LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
}

static FRESULT SD_TelemetryLoggerInit(void)
{
  FRESULT res;
  UINT bytes_done = 0U;
  static const char header[] =
      "timestamp_us,seq,"
      "adc1_raw,adc2_raw,adc3_raw,adc4_raw,adc5_raw,adc6_raw,"
      "can_valid,can_age_ms,can_id,can_dlc,can_d0,can_d1,can_d2,can_d3,can_d4,can_d5,can_d6,can_d7,"
      "wheel1_valid,wheel1_age_ms,wheel1_delta_us,wheel1_speed_centi_kmh,"
      "wheel2_valid,wheel2_age_ms,wheel2_delta_us,wheel2_speed_centi_kmh,"
      "wheel3_valid,wheel3_age_ms,wheel3_delta_us,wheel3_speed_centi_kmh,"
      "wheel4_valid,wheel4_age_ms,wheel4_delta_us,wheel4_speed_centi_kmh,"
      "can_rx_overflow,can_hal_error,wheel_overflow,adc_overflow\r\n";

  g_sd_detect_state = SD_IsCardInserted();
  if (g_sd_detect_state == GPIO_PIN_RESET)
  {
    LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_SET);
    g_sd_bringup_status = SD_LED_ERR_FATFS;
    SD_RecordFault(SD_FAULT_NO_CARD, FR_NOT_READY, 0U);
    return FR_NOT_READY;
  }

  res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_MOUNT;
    SD_RecordFault(SD_FAULT_MOUNT, res, 0U);
    return res;
  }

  res = SD_OpenNextTelemetryLogFile();
  if (res != FR_OK)
  {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 0U);
    g_sd_bringup_status = SD_LED_ERR_OPEN_EMU;
    SD_RecordFault(SD_FAULT_OPEN_TELEMETRY, res, 0U);
    return res;
  }

  res = f_write(&g_emu_can_file, header, (UINT)(sizeof(header) - 1U), &bytes_done);
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_WRITE_HEADER;
    SD_RecordFault(SD_FAULT_HEADER_WRITE, res, bytes_done);
    return res;
  }

  if (bytes_done != (sizeof(header) - 1U))
  {
    g_sd_bringup_status = SD_LED_ERR_WRITE_HEADER;
    SD_RecordFault(SD_FAULT_HEADER_SHORT_WRITE, FR_DISK_ERR, bytes_done);
    return FR_DISK_ERR;
  }

  res = f_sync(&g_emu_can_file);
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_SYNC;
    SD_RecordFault(SD_FAULT_HEADER_SYNC, res, 0U);
    return res;
  }

  g_sd_active_file = &g_emu_can_file;
  g_sd_log_buffer_len = 0U;
  g_sd_log_last_write_tick = HAL_GetTick();
  g_sd_log_last_sync_tick = g_sd_log_last_write_tick;
  g_sd_log_unsynced_count = 0U;
  g_telemetry_snapshot_seq = 0U;
  g_telemetry_next_snapshot_us = Telemetry_GetTimestampUs() + TELEMETRY_SNAPSHOT_INTERVAL_US;
  memset(&g_telemetry_latest_can, 0, sizeof(g_telemetry_latest_can));
  memset(g_telemetry_latest_wheel, 0, sizeof(g_telemetry_latest_wheel));
  memset(g_telemetry_latest_adc_raw, 0, sizeof(g_telemetry_latest_adc_raw));
  g_telemetry_adc_seen = 0U;
  g_sd_bringup_status = 0U;
  return FR_OK;
}

static FRESULT SD_OpenNextTelemetryLogFile(void)
{
  FRESULT res;

  for (uint16_t i = 0U; i <= SD_LOG_FILE_MAX_INDEX; i++)
  {
    (void)snprintf(g_sd_log_path, sizeof(g_sd_log_path), "0:/Telemetry_%03u.csv", i);
    res = f_open(&g_emu_can_file, g_sd_log_path, FA_CREATE_NEW | FA_WRITE);
    if (res == FR_OK)
    {
      return FR_OK;
    }

    if (res != FR_EXIST)
    {
      return res;
    }
  }

  return FR_EXIST;
}

static FRESULT SD_GNSSLoggerInit(void)
{
  FRESULT res;

  /* Mount the card and open the next raw GNSS file. No CSV header is written:
   * this file should contain only the original serial byte stream from the GNSS
   * receiver, which is useful for later NMEA/UBX decoding.
   */
  g_sd_detect_state = SD_IsCardInserted();
  if (g_sd_detect_state == GPIO_PIN_RESET)
  {
    LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_SET);
    g_sd_bringup_status = SD_LED_ERR_FATFS;
    SD_RecordFault(SD_FAULT_NO_CARD, FR_NOT_READY, 0U);
    return FR_NOT_READY;
  }

  res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_MOUNT;
    SD_RecordFault(SD_FAULT_MOUNT, res, 0U);
    return res;
  }

  res = SD_OpenNextGNSSLogFile();
  if (res != FR_OK)
  {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 0U);
    g_sd_bringup_status = SD_LED_ERR_OPEN_EMU;
    SD_RecordFault(SD_FAULT_OPEN_TELEMETRY, res, 0U);
    return res;
  }

  SD_LogWriterReset(&g_gnss_file);
  g_gnss_rx_head = 0U;
  g_gnss_rx_tail = 0U;
  g_gnss_rx_count = 0U;
  g_gnss_rx_overflow_count = 0U;
  g_gnss_uart_error_count = 0U;
  g_gnss_drain_call_count = 0U;
  g_gnss_drain_byte_count = 0ULL;
  g_sd_bringup_status = 0U;
  return FR_OK;
}

static FRESULT SD_OpenNextGNSSLogFile(void)
{
  FRESULT res;

  /* Use CREATE_NEW so each power cycle/test run gets a separate file and never
   * overwrites a previous capture.
   */
  for (uint16_t i = 0U; i <= SD_LOG_FILE_MAX_INDEX; i++)
  {
    (void)snprintf(g_sd_log_path, sizeof(g_sd_log_path), "0:/GNSS_%03u.nmea", i);
    res = f_open(&g_gnss_file, g_sd_log_path, FA_CREATE_NEW | FA_WRITE);
    if (res == FR_OK)
    {
      return FR_OK;
    }

    if (res != FR_EXIST)
    {
      return res;
    }
  }

  return FR_EXIST;
}

static void GNSSLoggerStart(void)
{
  /* Reset runtime counters after the file is open, then start interrupt-driven
   * byte reception. The actual SD writing happens in SD_GNSSLoggerProcess().
   */
  g_gnss_rx_byte = 0U;
  g_gnss_rx_head = 0U;
  g_gnss_rx_tail = 0U;
  g_gnss_rx_count = 0U;
  g_gnss_rx_overflow_count = 0U;
  g_gnss_uart_error_count = 0U;

  if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_gnss_rx_byte, 1U) != HAL_OK)
  {
    g_gnss_uart_error_count++;
    LED_BlinkCode(SD_LED_ERR_GNSS_UART);
  }
}

static FRESULT SD_GNSSLoggerProcess(void)
{
  FRESULT res;
  uint8_t chunk[GNSS_RX_DRAIN_CHUNK_SIZE];
  UINT chunk_len;
  uint32_t drained = 0U;

  /* Producer/consumer split:
   * - USART1 ISR only pushes raw GNSS bytes into g_gnss_rx_queue.
   * - This process function drains that queue in small chunks.
   * - SD_LogWriterAppend()/Service() are the only places that touch FatFs.
   *
   * The chunk size is intentionally modest so interrupts stay disabled only for
   * a short memory copy in GNSS_QueuePopChunk().
   */
  while ((chunk_len = GNSS_QueuePopChunk(chunk, (UINT)sizeof(chunk))) != 0U)
  {
    res = SD_LogWriterAppend(chunk, chunk_len);
    if (res != FR_OK)
    {
      return res;
    }
    drained += chunk_len;
  }

  if (drained != 0U)
  {
    HAL_GPIO_TogglePin(USER_LED_0_GPIO_Port, USER_LED_0_Pin);
    g_gnss_drain_call_count++;
    g_gnss_drain_byte_count += drained;
  }

  res = SD_LogWriterService(GNSS_LOG_FLUSH_IDLE_MS, GNSS_LOG_SYNC_INTERVAL_MS);
  if (res != FR_OK)
  {
    return res;
  }

  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin,
          ((g_gnss_rx_overflow_count == 0U) &&
           (g_gnss_uart_error_count == 0U) &&
           (g_sd_sync_fail_count == 0U)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  return FR_OK;
}

static void GNSS_QueuePushFromIsr(uint8_t byte)
{
  uint16_t next_head = (uint16_t)((g_gnss_rx_head + 1U) % GNSS_RX_QUEUE_LEN);

  /* Drop the newest byte if the main loop cannot keep up. The overflow counter
   * makes this visible through LED1 and during debugger inspection.
   */
  if (next_head == g_gnss_rx_tail)
  {
    g_gnss_rx_overflow_count++;
    return;
  }

  g_gnss_rx_queue[g_gnss_rx_head] = byte;
  g_gnss_rx_head = next_head;
  g_gnss_rx_count++;
}

static uint8_t GNSS_QueuePop(uint8_t *byte)
{
  uint8_t has_byte = 0U;

  /* Head/tail are shared with the UART ISR. Disable interrupts only for the
   * pointer update, not for SD writes.
   */
  __disable_irq();
  if (g_gnss_rx_tail != g_gnss_rx_head)
  {
    *byte = g_gnss_rx_queue[g_gnss_rx_tail];
    g_gnss_rx_tail = (uint16_t)((g_gnss_rx_tail + 1U) % GNSS_RX_QUEUE_LEN);
    has_byte = 1U;
  }
  __enable_irq();

  return has_byte;
}

static UINT GNSS_QueuePopChunk(uint8_t *buffer, UINT max_len)
{
  UINT count = 0U;

  if ((buffer == NULL) || (max_len == 0U))
  {
    return 0U;
  }

  __disable_irq();
  while ((count < max_len) && (g_gnss_rx_tail != g_gnss_rx_head))
  {
    buffer[count] = g_gnss_rx_queue[g_gnss_rx_tail];
    g_gnss_rx_tail = (uint16_t)((g_gnss_rx_tail + 1U) % GNSS_RX_QUEUE_LEN);
    count++;
  }
  __enable_irq();

  return count;
}

static FRESULT SD_TelemetryLoggerProcess(void)
{
  FRESULT res = FR_OK;
  uint32_t now;
  uint64_t timestamp_us;

  res = SD_TelemetryLoggerDrainInputs();
  if (res != FR_OK)
  {
    SD_RecordFault(SD_FAULT_TELEMETRY_DRAIN, res, 0U);
    return res;
  }

  timestamp_us = Telemetry_GetTimestampUs();
  while ((int64_t)(timestamp_us - g_telemetry_next_snapshot_us) >= 0)
  {
    res = SD_TelemetryLoggerAppendSnapshot(g_telemetry_next_snapshot_us);
    if (res != FR_OK)
    {
      return res;
    }

    g_telemetry_next_snapshot_us += TELEMETRY_SNAPSHOT_INTERVAL_US;
    timestamp_us = Telemetry_GetTimestampUs();
  }

  now = HAL_GetTick();
  if ((g_sd_log_buffer_len > 0U) &&
      ((now - g_sd_log_last_write_tick) >= SD_LOG_FLUSH_IDLE_MS))
  {
    return SD_PulseLoggerFlush(0U);
  }

  if ((g_sd_log_unsynced_count > 0U) &&
      ((now - g_sd_log_last_sync_tick) >= SD_LOG_SYNC_INTERVAL_MS))
  {
    return SD_PulseLoggerFlush(1U);
  }

  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin,
          ((g_emu_can_rx_overflow_count == 0U) &&
           (g_emu_can_hal_error_count == 0U) &&
           (g_wheel4_rx_overflow_count == 0U) &&
           (g_adc6_rx_overflow_count == 0U)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  return FR_OK;
}

static FRESULT SD_TelemetryLoggerDrainInputs(void)
{
  EMU_CAN_Frame_t can_frame;
  Wheel4_LogEvent_t wheel_event;
  ADC6_LogSample_t adc_sample;

  while (EMU_CANQueuePop(&can_frame) != 0U)
  {
    g_telemetry_latest_can.valid = 1U;
    g_telemetry_latest_can.timestamp_us = can_frame.timestamp_us;
    g_telemetry_latest_can.std_id = can_frame.std_id;
    g_telemetry_latest_can.dlc = can_frame.dlc;
    memcpy(g_telemetry_latest_can.data, can_frame.data, sizeof(g_telemetry_latest_can.data));

    if ((can_frame.std_id == NEXTION_RPM_CAN_ID) && (can_frame.dlc >= 2U))
    {
      g_nextion_latest_rpm = (uint16_t)((uint16_t)can_frame.data[0] |
                                         ((uint16_t)can_frame.data[1] << 8));
      g_nextion_rpm_valid = 1U;
    }
  }

  while (Wheel4_QueuePop(&wheel_event) != 0U)
  {
    if (wheel_event.channel < WHEEL_CHANNEL_COUNT)
    {
      g_telemetry_latest_wheel[wheel_event.channel].valid = 1U;
      g_telemetry_latest_wheel[wheel_event.channel].timestamp_us = wheel_event.timestamp_us;
      g_telemetry_latest_wheel[wheel_event.channel].delta_us = wheel_event.delta_us;
      g_telemetry_latest_wheel[wheel_event.channel].speed_centi_kmh = wheel_event.speed_centi_kmh;
    }
  }

  while (ADC6_QueuePop(&adc_sample) != 0U)
  {
    for (uint8_t i = 0U; i < ADC_CHANNEL_COUNT; i++)
    {
      g_telemetry_latest_adc_raw[i] = adc_sample.raw[i];
    }
    g_telemetry_adc_seen = 1U;
  }

  Nextion_ProcessRPMTx();

  return FR_OK;
}

static void Nextion_HMITestInit(void)
{
  g_nextion_hmi_last_rpm_tick = 0U;
  g_nextion_hmi_last_gear_tick = 0U;
  g_nextion_hmi_last_data_tick = 0U;
  g_current_rpm = 0U;
  g_nextion_hmi_rpm = g_current_rpm;
  g_nextion_hmi_water = 0U;
  g_nextion_hmi_oil = 0U;
  g_nextion_hmi_speed = 0U;
  g_nextion_hmi_random_state = 0x12345678UL ^ HAL_GetTick();
  g_nextion_hmi_gear_index = 0U;
  g_nextion_hmi_tx_count = 0U;
  g_nextion_hmi_tx_fail_count = 0U;
  g_test_rpm_sweep_start_tick = HAL_GetTick();

  Nextion_HMITestTransmitValues();
}

static void Nextion_HMITestProcess(void)
{
  uint32_t now = HAL_GetTick();
  char value[16];
  uint8_t transmitted = 0U;
  static const char *const gear_sequence[7] =
  {
    "1", "N", "2", "3", "4", "5", "6"
  };

  if ((now - g_nextion_hmi_last_rpm_tick) >= NEXTION_HMI_RPM_INTERVAL_MS)
  {
    g_nextion_hmi_last_rpm_tick = now;
    g_nextion_hmi_rpm = g_current_rpm;
    (void)snprintf(value, sizeof(value), "%u", (unsigned int)g_nextion_hmi_rpm);
    Nextion_SendTextCommand("t_rpm", value);
    transmitted = 1U;
  }

  if ((now - g_nextion_hmi_last_gear_tick) >= NEXTION_HMI_GEAR_INTERVAL_MS)
  {
    g_nextion_hmi_last_gear_tick = now;
    g_nextion_hmi_gear_index = (uint8_t)Nextion_TestRandomRange(0U, 6U);
    Nextion_SendTextCommand("t_gear", gear_sequence[g_nextion_hmi_gear_index]);
    transmitted = 1U;
  }

  if ((now - g_nextion_hmi_last_data_tick) >= NEXTION_HMI_DATA_INTERVAL_MS)
  {
    g_nextion_hmi_last_data_tick = now;

    g_nextion_hmi_oil = Nextion_TestRandomRange(50U, 140U);
    (void)snprintf(value, sizeof(value), "%u", (unsigned int)g_nextion_hmi_oil);
    Nextion_SendTextCommand("t_oil", value);

    g_nextion_hmi_water = Nextion_TestRandomRange(60U, 120U);
    (void)snprintf(value, sizeof(value), "%u", (unsigned int)g_nextion_hmi_water);
    Nextion_SendTextCommand("t_water", value);

    g_nextion_hmi_speed = Nextion_TestRandomRange(0U, 180U);
    (void)snprintf(value, sizeof(value), "%u", (unsigned int)g_nextion_hmi_speed);
    Nextion_SendTextCommand("t_speed", value);
    transmitted = 1U;
  }

  if (transmitted != 0U)
  {
    HAL_GPIO_TogglePin(USER_LED_0_GPIO_Port, USER_LED_0_Pin);
  }
}

static void Nextion_HMITestTransmitValues(void)
{
  char value[16];
  static const char *const gear_sequence[7] =
  {
    "1", "N", "2", "3", "4", "5", "6"
  };

  (void)snprintf(value, sizeof(value), "%u", (unsigned int)g_nextion_hmi_rpm);
  Nextion_SendTextCommand("t_rpm", value);

  Nextion_SendTextCommand("t_gear", gear_sequence[g_nextion_hmi_gear_index]);

  (void)snprintf(value, sizeof(value), "%u", (unsigned int)g_nextion_hmi_oil);
  Nextion_SendTextCommand("t_oil", value);

  (void)snprintf(value, sizeof(value), "%u", (unsigned int)g_nextion_hmi_water);
  Nextion_SendTextCommand("t_water", value);

  (void)snprintf(value, sizeof(value), "%u", (unsigned int)g_nextion_hmi_speed);
  Nextion_SendTextCommand("t_speed", value);
}

static void Nextion_SendTextCommand(const char *component, const char *text)
{
  char command[48];
  int command_len;
  static const uint8_t nextion_end[3] = {0xFFU, 0xFFU, 0xFFU};

  command_len = snprintf(command, sizeof(command), "%s.txt=\"%s\"", component, text);
  if ((command_len <= 0) || ((size_t)command_len >= sizeof(command)))
  {
    g_nextion_hmi_tx_fail_count++;
    return;
  }

  if (HAL_UART_Transmit(&huart4, (uint8_t *)command, (uint16_t)command_len,
                        RACE_NEXTION_UART_TIMEOUT_MS) != HAL_OK)
  {
    g_nextion_hmi_tx_fail_count++;
    g_race_nextion_tx_fail_count++;
    return;
  }

  if (HAL_UART_Transmit(&huart4, (uint8_t *)nextion_end, sizeof(nextion_end),
                        RACE_NEXTION_UART_TIMEOUT_MS) != HAL_OK)
  {
    g_nextion_hmi_tx_fail_count++;
    g_race_nextion_tx_fail_count++;
    return;
  }

  g_nextion_hmi_tx_count++;
  g_race_nextion_tx_count++;
}

static uint16_t Nextion_TestRandomRange(uint16_t min_value, uint16_t max_value)
{
  uint32_t range;

  if (max_value <= min_value)
  {
    return min_value;
  }

  g_nextion_hmi_random_state =
      (g_nextion_hmi_random_state * 1664525UL) + 1013904223UL;
  range = (uint32_t)max_value - (uint32_t)min_value + 1UL;
  return (uint16_t)(min_value + (uint16_t)(g_nextion_hmi_random_state % range));
}

static void TestRPM_UpdateSlowSweep(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t cycle_ms = TEST_RPM_SWEEP_HALF_MS * 2U;
  uint32_t elapsed_ms;
  uint32_t phase_ms;
  uint32_t rpm;

  if ((cycle_ms == 0U) || (TEST_RPM_SWEEP_HALF_MS == 0U))
  {
    RPMOutputs_SetRPM(RPM_MIN);
    return;
  }

  elapsed_ms = now - g_test_rpm_sweep_start_tick;
  phase_ms = elapsed_ms % cycle_ms;

  if (phase_ms <= TEST_RPM_SWEEP_HALF_MS)
  {
    rpm = ((uint32_t)RPM_MAX * phase_ms) / TEST_RPM_SWEEP_HALF_MS;
  }
  else
  {
    rpm = ((uint32_t)RPM_MAX * (cycle_ms - phase_ms)) / TEST_RPM_SWEEP_HALF_MS;
  }

  RPMOutputs_SetRPM((uint16_t)rpm);
}

static void RPMOutputs_Init(void)
{
  WS2812_InitTimings();
  g_ws2812_dma_ready = 1U;
  g_ws2812_last_update_tick = 0U;
  RPMOutputs_SetRPM(g_current_rpm);
  WS2812_ShowRPM(g_current_rpm, g_rpm_led_mode);
}

static void RPMOutputs_Process(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - g_ws2812_last_update_tick) < WS2812_UPDATE_MS)
  {
    return;
  }

  g_ws2812_last_update_tick = now;
  WS2812_ShowRPM(g_current_rpm, g_rpm_led_mode);
}

static void RPMOutputs_SetRPM(uint16_t rpm)
{
  if (rpm > RPM_MAX)
  {
    rpm = RPM_MAX;
  }

  g_current_rpm = rpm;
  g_nextion_hmi_rpm = rpm;
  g_nextion_latest_rpm = rpm;
  g_nextion_rpm_valid = 1U;
}

static void WS2812_InitTimings(void)
{
  uint32_t timer_period = htim8.Init.Period + 1U;

  g_ws2812_pwm_0 =
      (uint16_t)(((timer_period * WS2812_T0H_NS) + (WS2812_PERIOD_NS / 2U)) / WS2812_PERIOD_NS);
  g_ws2812_pwm_1 =
      (uint16_t)(((timer_period * WS2812_T1H_NS) + (WS2812_PERIOD_NS / 2U)) / WS2812_PERIOD_NS);
}

static uint8_t PercentToByte(uint8_t percent)
{
  if (percent > 100U)
  {
    percent = 100U;
  }

  return (uint8_t)(((uint32_t)255U * percent) / 100U);
}

static void WS2812_SetLedBits(uint32_t *index, uint8_t red, uint8_t green, uint8_t blue)
{
  uint8_t color[3] = {green, red, blue};

  for (uint32_t color_index = 0U; color_index < 3U; color_index++)
  {
    for (int8_t bit = 7; bit >= 0; bit--)
    {
      g_ws2812_pwm_buffer[*index] =
          ((color[color_index] >> bit) & 0x01U) ? g_ws2812_pwm_1 : g_ws2812_pwm_0;
      (*index)++;
    }
  }
}

static uint8_t WS2812_WaitUntilReady(uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();

  while (g_ws2812_dma_ready == 0U)
  {
    if ((HAL_GetTick() - start_tick) >= timeout_ms)
    {
      return 0U;
    }
  }

  return 1U;
}

static void WS2812_ShowRPM(uint16_t rpm, uint8_t mode)
{
  (void)mode;
  WS2812_ShowRPMBar(rpm);
}

static void WS2812_ShowRPMBar(uint16_t rpm)
{
  uint32_t index = 0U;
  uint8_t led_colors[WS2812_LED_COUNT][3] = {{0U}};
  uint8_t lit_count;
  uint8_t green = PercentToByte(RPM_LED_BRIGHTNESS);
  uint8_t yellow_red = PercentToByte(RPM_LED_BRIGHTNESS);
  uint8_t yellow_green = PercentToByte(RPM_LED_BRIGHTNESS);
  uint8_t red = PercentToByte(RPM_LED_BRIGHTNESS);
  uint8_t blink_on = 1U;

  if (rpm >= RPM_REDZONE_START)
  {
    blink_on = (uint8_t)(((HAL_GetTick() / RPM_REDZONE_BLINK_MS) % 2U) == 0U);

    for (uint8_t led_number = 1U; led_number <= WS2812_LED_COUNT; led_number++)
    {
      if (blink_on != 0U)
      {
        led_colors[led_number - 1U][0] = red;
      }
    }
  }
  else
  {
    /* Driver view is left to right: LED 12, 11, ..., 2, 1. */
    led_colors[0U][1] = green;
    led_colors[WS2812_LED_COUNT - 1U][1] = green;

    lit_count = (uint8_t)(rpm / RPM_PER_LED);
    if (lit_count > RPM_USED_LED_COUNT)
    {
      lit_count = RPM_USED_LED_COUNT;
    }

    for (uint8_t step = 0U; step < lit_count; step++)
    {
      uint8_t led_number = (uint8_t)(11U - step);
      uint8_t led_index = (uint8_t)(led_number - 1U);

      if (led_number >= 8U)
      {
        led_colors[led_index][1] = green;
      }
      else if (led_number >= 5U)
      {
        led_colors[led_index][0] = yellow_red;
        led_colors[led_index][1] = yellow_green;
      }
      else
      {
        led_colors[led_index][0] = red;
      }
    }
  }

  for (uint8_t led_index = 0U; led_index < WS2812_LED_COUNT; led_index++)
  {
    WS2812_SetLedBits(&index,
                      led_colors[led_index][0],
                      led_colors[led_index][1],
                      led_colors[led_index][2]);
  }

  WS2812_SendBuffer(index);
}

static void WS2812_SendBuffer(uint32_t used_len)
{
  uint32_t index = used_len;

  while (index < WS2812_BUFFER_LEN)
  {
    g_ws2812_pwm_buffer[index++] = 0U;
  }

  if (WS2812_WaitUntilReady(WS2812_DMA_TIMEOUT_MS) == 0U)
  {
    g_ws2812_dma_start_fail_count++;
    return;
  }

  g_ws2812_dma_ready = 0U;
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COUNTER(&htim8, 0U);
  if (HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1,
                            (uint32_t *)g_ws2812_pwm_buffer,
                            WS2812_BUFFER_LEN) != HAL_OK)
  {
    g_ws2812_dma_ready = 1U;
    g_ws2812_dma_start_fail_count++;
  }
}

static void Nextion_ProcessRPMTx(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - g_nextion_last_tx_tick) < NEXTION_RPM_TX_INTERVAL_MS)
  {
    return;
  }

  g_nextion_last_tx_tick += NEXTION_RPM_TX_INTERVAL_MS;
  if ((now - g_nextion_last_tx_tick) >= NEXTION_RPM_TX_INTERVAL_MS)
  {
    g_nextion_last_tx_tick = now;
  }

  if (g_nextion_rpm_valid == 0U)
  {
    return;
  }

  Nextion_SendRPM(g_nextion_latest_rpm);
}

static void Nextion_SendRPM(uint16_t rpm)
{
  char command[32];
  int command_len;
  static const uint8_t nextion_end[3] = {0xFFU, 0xFFU, 0xFFU};

  command_len = snprintf(command, sizeof(command), NEXTION_RPM_PROP "=%u", (unsigned int)rpm);
  if ((command_len <= 0) || ((size_t)command_len >= sizeof(command)))
  {
    return;
  }

  if (HAL_UART_Transmit(&huart4, (uint8_t *)command, (uint16_t)command_len, 2U) != HAL_OK)
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart4, (uint8_t *)nextion_end, sizeof(nextion_end), 2U);
}

static FRESULT SD_TelemetryLoggerAppendSnapshot(uint64_t timestamp_us)
{
  FRESULT res;
  char line[768];
  int line_len;
  uint8_t can_valid = 0U;
  uint32_t can_age_ms = 0U;
  uint32_t can_id = 0U;
  uint8_t can_dlc = 0U;
  uint8_t can_data[8] = {0};
  uint16_t adc_raw[ADC_CHANNEL_COUNT];
  uint8_t wheel_valid[WHEEL_CHANNEL_COUNT];
  uint32_t wheel_age_ms[WHEEL_CHANNEL_COUNT];
  uint32_t wheel_delta_us[WHEEL_CHANNEL_COUNT];
  uint32_t wheel_speed_centi_kmh[WHEEL_CHANNEL_COUNT];

  for (uint8_t i = 0U; i < ADC_CHANNEL_COUNT; i++)
  {
    adc_raw[i] = g_telemetry_latest_adc_raw[i];
  }

  if ((g_telemetry_latest_can.valid != 0U) &&
      (timestamp_us >= g_telemetry_latest_can.timestamp_us))
  {
    uint64_t age_us = timestamp_us - g_telemetry_latest_can.timestamp_us;
    can_age_ms = (uint32_t)(age_us / 1000ULL);
    if (age_us <= TELEMETRY_CAN_TIMEOUT_US)
    {
      can_valid = 1U;
      can_id = g_telemetry_latest_can.std_id;
      can_dlc = g_telemetry_latest_can.dlc;
      memcpy(can_data, g_telemetry_latest_can.data, sizeof(can_data));
    }
  }

  for (uint8_t i = 0U; i < WHEEL_CHANNEL_COUNT; i++)
  {
    wheel_valid[i] = 0U;
    wheel_age_ms[i] = 0U;
    wheel_delta_us[i] = 0U;
    wheel_speed_centi_kmh[i] = 0U;

    if ((g_telemetry_latest_wheel[i].valid != 0U) &&
        (timestamp_us >= g_telemetry_latest_wheel[i].timestamp_us))
    {
      uint64_t age_us = timestamp_us - g_telemetry_latest_wheel[i].timestamp_us;
      wheel_age_ms[i] = (uint32_t)(age_us / 1000ULL);
      wheel_delta_us[i] = g_telemetry_latest_wheel[i].delta_us;
      if (age_us <= TELEMETRY_WHEEL_TIMEOUT_US)
      {
        wheel_valid[i] = 1U;
        wheel_speed_centi_kmh[i] = g_telemetry_latest_wheel[i].speed_centi_kmh;
      }
    }
  }

  line_len = snprintf(line, sizeof(line),
                      "%lu%06lu,%lu,"
                      "%u,%u,%u,%u,%u,%u,"
                      "%u,%lu,0x%03lX,%u,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,"
                      "%u,%lu,%lu,%lu,"
                      "%u,%lu,%lu,%lu,"
                      "%u,%lu,%lu,%lu,"
                      "%u,%lu,%lu,%lu,"
                      "%lu,%lu,%lu,%lu\r\n",
                      (unsigned long)(timestamp_us / 1000000ULL),
                      (unsigned long)(timestamp_us % 1000000ULL),
                      (unsigned long)g_telemetry_snapshot_seq,
                      (unsigned int)adc_raw[0],
                      (unsigned int)adc_raw[1],
                      (unsigned int)adc_raw[2],
                      (unsigned int)adc_raw[3],
                      (unsigned int)adc_raw[4],
                      (unsigned int)adc_raw[5],
                      (unsigned int)can_valid,
                      (unsigned long)can_age_ms,
                      (unsigned long)can_id,
                      (unsigned int)can_dlc,
                      (unsigned int)can_data[0],
                      (unsigned int)can_data[1],
                      (unsigned int)can_data[2],
                      (unsigned int)can_data[3],
                      (unsigned int)can_data[4],
                      (unsigned int)can_data[5],
                      (unsigned int)can_data[6],
                      (unsigned int)can_data[7],
                      (unsigned int)wheel_valid[0],
                      (unsigned long)wheel_age_ms[0],
                      (unsigned long)wheel_delta_us[0],
                      (unsigned long)wheel_speed_centi_kmh[0],
                      (unsigned int)wheel_valid[1],
                      (unsigned long)wheel_age_ms[1],
                      (unsigned long)wheel_delta_us[1],
                      (unsigned long)wheel_speed_centi_kmh[1],
                      (unsigned int)wheel_valid[2],
                      (unsigned long)wheel_age_ms[2],
                      (unsigned long)wheel_delta_us[2],
                      (unsigned long)wheel_speed_centi_kmh[2],
                      (unsigned int)wheel_valid[3],
                      (unsigned long)wheel_age_ms[3],
                      (unsigned long)wheel_delta_us[3],
                      (unsigned long)wheel_speed_centi_kmh[3],
                      (unsigned long)g_emu_can_rx_overflow_count,
                      (unsigned long)g_emu_can_hal_error_count,
                      (unsigned long)g_wheel4_rx_overflow_count,
                      (unsigned long)g_adc6_rx_overflow_count);

  if ((line_len <= 0) || ((size_t)line_len >= sizeof(line)))
  {
    SD_RecordFault(SD_FAULT_SNAPSHOT_FORMAT, FR_INT_ERR, (uint32_t)sizeof(line));
    return FR_INT_ERR;
  }

  if (((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len)) &&
      (g_sd_log_buffer_len > 0U))
  {
    res = SD_PulseLoggerFlush(0U);
    if (res != FR_OK)
    {
      return res;
    }
  }

  if ((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len))
  {
    SD_RecordFault(SD_FAULT_SNAPSHOT_BUFFER_FULL, FR_INT_ERR, (uint32_t)line_len);
    return FR_INT_ERR;
  }

  memcpy(&g_sd_log_buffer[g_sd_log_buffer_len], line, (size_t)line_len);
  g_sd_log_buffer_len += (UINT)line_len;
  g_telemetry_snapshot_seq++;
  g_telemetry_log_count++;
  g_sd_log_unsynced_count++;
  HAL_GPIO_TogglePin(USER_LED_0_GPIO_Port, USER_LED_0_Pin);

  if (g_telemetry_snapshot_seq == 1U)
  {
    return SD_PulseLoggerFlush(1U);
  }

#if (SD_LOG_DIAG_FORCE_SYNC != 0U)
  return SD_PulseLoggerFlush(1U);
#endif

  if (g_sd_log_buffer_len >= (SD_LOG_BUFFER_SIZE - (UINT)sizeof(line)))
  {
    res = SD_PulseLoggerFlush(0U);
    if (res != FR_OK)
    {
      return res;
    }
    return res;
  }

  return FR_OK;
}

static FRESULT SD_TelemetryLoggerAppendCANFrame(const EMU_CAN_Frame_t *frame)
{
  FRESULT res;
  char line[160];
  int line_len;

  line_len = snprintf(line, sizeof(line),
                      "%lu%06lu,%lu,CAN,,0x%03lX,%u,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,,,,,,,,,,,,,,,,\r\n",
                      (unsigned long)(frame->timestamp_us / 1000000ULL),
                      (unsigned long)(frame->timestamp_us % 1000000ULL),
                      (unsigned long)g_telemetry_log_count,
                      (unsigned long)frame->std_id,
                      (unsigned int)frame->dlc,
                      (unsigned int)frame->data[0],
                      (unsigned int)frame->data[1],
                      (unsigned int)frame->data[2],
                      (unsigned int)frame->data[3],
                      (unsigned int)frame->data[4],
                      (unsigned int)frame->data[5],
                      (unsigned int)frame->data[6],
                      (unsigned int)frame->data[7]);

  if ((line_len <= 0) || ((size_t)line_len >= sizeof(line)))
  {
    return FR_INT_ERR;
  }

  if (((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len)) &&
      (g_sd_log_buffer_len > 0U))
  {
    res = SD_PulseLoggerFlush(0U);
    if (res != FR_OK)
    {
      return res;
    }
  }

  if ((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len))
  {
    return FR_INT_ERR;
  }

  memcpy(&g_sd_log_buffer[g_sd_log_buffer_len], line, (size_t)line_len);
  g_sd_log_buffer_len += (UINT)line_len;
  g_telemetry_log_count++;
  g_emu_can_log_count++;
  g_sd_log_unsynced_count++;
  HAL_GPIO_TogglePin(USER_LED_0_GPIO_Port, USER_LED_0_Pin);

  if ((g_sd_log_buffer_len >= (SD_LOG_BUFFER_SIZE - 160U)) ||
      ((g_sd_log_unsynced_count % SD_LOG_SYNC_EVERY) == 0U))
  {
    return SD_PulseLoggerFlush((g_sd_log_unsynced_count % SD_LOG_SYNC_EVERY) == 0U);
  }

  return FR_OK;
}

static FRESULT SD_TelemetryLoggerAppendWheelEvent(const Wheel4_LogEvent_t *event)
{
  FRESULT res;
  char line[160];
  int line_len;

  line_len = snprintf(line, sizeof(line),
                      "%lu%06lu,%lu,WHEEL,%u,,,,,,,,,,,%lu,%lu.%02lu,,,,,,,,,,,,,,\r\n",
                      (unsigned long)(event->timestamp_us / 1000000ULL),
                      (unsigned long)(event->timestamp_us % 1000000ULL),
                      (unsigned long)g_telemetry_log_count,
                      (unsigned int)(event->channel + 1U),
                      (unsigned long)event->delta_us,
                      (unsigned long)(event->speed_centi_kmh / 100U),
                      (unsigned long)(event->speed_centi_kmh % 100U));

  if ((line_len <= 0) || ((size_t)line_len >= sizeof(line)))
  {
    return FR_INT_ERR;
  }

  if (((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len)) &&
      (g_sd_log_buffer_len > 0U))
  {
    res = SD_PulseLoggerFlush(0U);
    if (res != FR_OK)
    {
      return res;
    }
  }

  if ((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len))
  {
    return FR_INT_ERR;
  }

  memcpy(&g_sd_log_buffer[g_sd_log_buffer_len], line, (size_t)line_len);
  g_sd_log_buffer_len += (UINT)line_len;
  g_telemetry_log_count++;
  g_wheel4_log_count++;
  g_sd_log_unsynced_count++;
  HAL_GPIO_TogglePin(USER_LED_0_GPIO_Port, USER_LED_0_Pin);

  if ((g_sd_log_buffer_len >= (SD_LOG_BUFFER_SIZE - 160U)) ||
      ((g_sd_log_unsynced_count % SD_LOG_SYNC_EVERY) == 0U))
  {
    return SD_PulseLoggerFlush((g_sd_log_unsynced_count % SD_LOG_SYNC_EVERY) == 0U);
  }

  return FR_OK;
}

static FRESULT SD_TelemetryLoggerAppendADC6Sample(const ADC6_LogSample_t *sample)
{
  FRESULT res;
  char line[256];
  int line_len;
  uint32_t wheel_delta_us[WHEEL_CHANNEL_COUNT];
  uint32_t wheel_speed_centi_kmh[WHEEL_CHANNEL_COUNT];

  __disable_irq();
  for (uint8_t i = 0U; i < WHEEL_CHANNEL_COUNT; i++)
  {
    wheel_delta_us[i] = g_wheel4_last_delta_us[i];
    wheel_speed_centi_kmh[i] = g_wheel4_filtered_centi_kmh[i];

    if ((g_wheel4_last_timestamp_us[i] == 0ULL) ||
        ((sample->timestamp_us - g_wheel4_last_timestamp_us[i]) >= SPEED_TIMEOUT_US))
    {
      wheel_speed_centi_kmh[i] = 0U;
      g_wheel4_filtered_centi_kmh[i] = 0U;
      g_wheel4_filter_valid[i] = 0U;
    }
  }
  __enable_irq();

  line_len = snprintf(line, sizeof(line),
                      "%lu%06lu,%lu,SAMPLE,,,,,,,,,,,,,,%u,%u,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%lu.%02lu,%lu.%02lu,%lu.%02lu,%lu.%02lu\r\n",
                      (unsigned long)(sample->timestamp_us / 1000000ULL),
                      (unsigned long)(sample->timestamp_us % 1000000ULL),
                      (unsigned long)g_telemetry_log_count,
                      (unsigned int)sample->raw[0],
                      (unsigned int)sample->raw[1],
                      (unsigned int)sample->raw[2],
                      (unsigned int)sample->raw[3],
                      (unsigned int)sample->raw[4],
                      (unsigned int)sample->raw[5],
                      (unsigned long)wheel_delta_us[0],
                      (unsigned long)wheel_delta_us[1],
                      (unsigned long)wheel_delta_us[2],
                      (unsigned long)wheel_delta_us[3],
                      (unsigned long)(wheel_speed_centi_kmh[0] / 100U),
                      (unsigned long)(wheel_speed_centi_kmh[0] % 100U),
                      (unsigned long)(wheel_speed_centi_kmh[1] / 100U),
                      (unsigned long)(wheel_speed_centi_kmh[1] % 100U),
                      (unsigned long)(wheel_speed_centi_kmh[2] / 100U),
                      (unsigned long)(wheel_speed_centi_kmh[2] % 100U),
                      (unsigned long)(wheel_speed_centi_kmh[3] / 100U),
                      (unsigned long)(wheel_speed_centi_kmh[3] % 100U));

  if ((line_len <= 0) || ((size_t)line_len >= sizeof(line)))
  {
    return FR_INT_ERR;
  }

  if (((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len)) &&
      (g_sd_log_buffer_len > 0U))
  {
    res = SD_PulseLoggerFlush(0U);
    if (res != FR_OK)
    {
      return res;
    }
  }

  if ((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len))
  {
    return FR_INT_ERR;
  }

  memcpy(&g_sd_log_buffer[g_sd_log_buffer_len], line, (size_t)line_len);
  g_sd_log_buffer_len += (UINT)line_len;
  g_telemetry_log_count++;
  g_adc6_log_count++;
  g_sd_log_unsynced_count++;
  HAL_GPIO_TogglePin(USER_LED_0_GPIO_Port, USER_LED_0_Pin);

  if ((g_sd_log_buffer_len >= (SD_LOG_BUFFER_SIZE - 256U)) ||
      ((g_sd_log_unsynced_count % SD_LOG_SYNC_EVERY) == 0U))
  {
    return SD_PulseLoggerFlush((g_sd_log_unsynced_count % SD_LOG_SYNC_EVERY) == 0U);
  }

  return FR_OK;
}

static uint8_t ADC_LoggerConfigureHardware(void)
{
  static const uint32_t adc_channels[ADC_CHANNEL_COUNT] =
  {
    ADC_CHANNEL_10,
    ADC_CHANNEL_11,
    ADC_CHANNEL_12,
    ADC_CHANNEL_13,
    ADC_CHANNEL_14,
    ADC_CHANNEL_15
  };
  ADC_ChannelConfTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  if (HAL_ADC_DeInit(&hadc1) != HAL_OK)
  {
    return 0U;
  }

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = ADC_CHANNEL_COUNT;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    return 0U;
  }

  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  for (uint32_t i = 0U; i < ADC_CHANNEL_COUNT; i++)
  {
    sConfig.Channel = adc_channels[i];
    sConfig.Rank = (uint32_t)(i + 1U);
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      return 0U;
    }
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    return 0U;
  }

  return 1U;
}

static void ADC_LoggerStart(void)
{
  g_adc_rx_head = 0U;
  g_adc_rx_tail = 0U;
  g_adc_rx_overflow_count = 0U;
  g_adc_log_count = 0U;
  g_adc_legacy_queue_enabled = 1U;
  memset((void *)g_adc_dma_buffer, 0, sizeof(g_adc_dma_buffer));

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_dma_buffer, ADC_CHANNEL_COUNT) != HAL_OK)
  {
    LED_BlinkCode(SD_LED_ERR_ADC_CONFIG);
  }

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    LED_BlinkCode(SD_LED_ERR_TIM);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance != ADC1)
  {
    return;
  }

  if (g_telemetry_queue_enabled != 0U)
  {
    ADC6_QueuePushFromIsr((const uint16_t *)g_adc_dma_buffer);
  }
  if (g_adc_legacy_queue_enabled != 0U)
  {
    ADC_QueuePushFromIsr((const uint16_t *)g_adc_dma_buffer);
  }
}

static void ADC_QueuePushFromIsr(const uint16_t *raw)
{
  ADC_LogSample_t sample;
  uint16_t next_head = (uint16_t)((g_adc_rx_head + 1U) % ADC_LOG_QUEUE_LEN);

  if (next_head == g_adc_rx_tail)
  {
    g_adc_rx_overflow_count++;
    return;
  }

  sample.timestamp_ms = HAL_GetTick();
  for (uint32_t i = 0U; i < ADC_CHANNEL_COUNT; i++)
  {
    sample.raw[i] = raw[i];
  }

  g_adc_rx_queue[g_adc_rx_head] = sample;
  g_adc_rx_head = next_head;
}

static uint8_t ADC_QueuePop(ADC_LogSample_t *sample)
{
  uint8_t has_sample = 0U;

  __disable_irq();
  if (g_adc_rx_tail != g_adc_rx_head)
  {
    *sample = g_adc_rx_queue[g_adc_rx_tail];
    g_adc_rx_tail = (uint16_t)((g_adc_rx_tail + 1U) % ADC_LOG_QUEUE_LEN);
    has_sample = 1U;
  }
  __enable_irq();

  return has_sample;
}

static FRESULT SD_ADCLoggerInit(void)
{
  FRESULT res;
  UINT bytes_done = 0U;
  static const char header[] =
      "timestamp_ms,ADC1,ADC2,ADC3,ADC4,ADC5,ADC6\r\n";

  g_sd_detect_state = SD_IsCardInserted();
  if (g_sd_detect_state == GPIO_PIN_RESET)
  {
    LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_SET);
    g_sd_bringup_status = SD_LED_ERR_FATFS;
    return FR_NOT_READY;
  }

  res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_MOUNT;
    return res;
  }

  res = SD_OpenNextADCLogFile();
  if (res != FR_OK)
  {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 0U);
    g_sd_bringup_status = SD_LED_ERR_OPEN_ADC;
    return res;
  }

  res = f_write(&g_adc_file, header, (UINT)(sizeof(header) - 1U), &bytes_done);
  if ((res == FR_OK) && (bytes_done != (sizeof(header) - 1U)))
  {
    res = FR_DISK_ERR;
  }
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_WRITE_HEADER;
    return res;
  }

  res = f_sync(&g_adc_file);
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_SYNC;
    return res;
  }

  g_adc_log_count = 0U;
  g_sd_active_file = &g_adc_file;
  g_sd_log_buffer_len = 0U;
  g_sd_log_last_write_tick = HAL_GetTick();
  g_sd_log_last_sync_tick = g_sd_log_last_write_tick;
  g_sd_log_unsynced_count = 0U;
  g_sd_bringup_status = 0U;
  return FR_OK;
}

static FRESULT SD_OpenNextADCLogFile(void)
{
  FRESULT res;

  res = f_open(&g_adc_file, "0:/ADC.CSV", FA_CREATE_NEW | FA_WRITE);
  if (res == FR_OK)
  {
    (void)snprintf(g_sd_log_path, sizeof(g_sd_log_path), "0:/ADC.CSV");
    return FR_OK;
  }
  if (res != FR_EXIST)
  {
    return res;
  }

  for (uint16_t i = 1U; i <= SD_LOG_FILE_MAX_INDEX; i++)
  {
    (void)snprintf(g_sd_log_path, sizeof(g_sd_log_path), "0:/ADC%03u.CSV", i);
    res = f_open(&g_adc_file, g_sd_log_path, FA_CREATE_NEW | FA_WRITE);
    if (res == FR_OK)
    {
      return FR_OK;
    }

    if (res != FR_EXIST)
    {
      return res;
    }
  }

  return FR_EXIST;
}

static FRESULT SD_ADCLoggerProcess(void)
{
  FRESULT res = FR_OK;
  ADC_LogSample_t sample;
  uint32_t now;

  while (ADC_QueuePop(&sample) != 0U)
  {
    res = SD_ADCLoggerAppendSample(&sample);
    if (res != FR_OK)
    {
      return res;
    }
  }

  now = HAL_GetTick();
  if ((g_sd_log_buffer_len > 0U) &&
      ((now - g_sd_log_last_write_tick) >= SD_LOG_FLUSH_IDLE_MS))
  {
    return SD_PulseLoggerFlush(0U);
  }

  if ((g_sd_log_unsynced_count > 0U) &&
      ((now - g_sd_log_last_sync_tick) >= SD_LOG_SYNC_INTERVAL_MS))
  {
    return SD_PulseLoggerFlush(1U);
  }

  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin,
          (g_adc_rx_overflow_count == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  return FR_OK;
}

static FRESULT SD_ADCLoggerAppendSample(const ADC_LogSample_t *sample)
{
  FRESULT res;
  char line[128];
  int line_len;

  line_len = snprintf(line, sizeof(line),
                      "%lu,%u,%u,%u,%u,%u,%u\r\n",
                      (unsigned long)sample->timestamp_ms,
                      (unsigned int)sample->raw[0],
                      (unsigned int)sample->raw[1],
                      (unsigned int)sample->raw[2],
                      (unsigned int)sample->raw[3],
                      (unsigned int)sample->raw[4],
                      (unsigned int)sample->raw[5]);

  if ((line_len <= 0) || ((size_t)line_len >= sizeof(line)))
  {
    return FR_INT_ERR;
  }

  if (((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len)) &&
      (g_sd_log_buffer_len > 0U))
  {
    res = SD_PulseLoggerFlush(0U);
    if (res != FR_OK)
    {
      return res;
    }
  }

  if ((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len))
  {
    return FR_INT_ERR;
  }

  memcpy(&g_sd_log_buffer[g_sd_log_buffer_len], line, (size_t)line_len);
  g_sd_log_buffer_len += (UINT)line_len;
  g_adc_log_count++;
  g_sd_log_unsynced_count++;
  HAL_GPIO_TogglePin(USER_LED_0_GPIO_Port, USER_LED_0_Pin);

  if ((g_sd_log_buffer_len >= (SD_LOG_BUFFER_SIZE - 128U)) ||
      ((g_sd_log_unsynced_count % SD_LOG_SYNC_EVERY) == 0U))
  {
    return SD_PulseLoggerFlush((g_sd_log_unsynced_count % SD_LOG_SYNC_EVERY) == 0U);
  }

  return FR_OK;
}

static void TIM1_WheelCaptureStart(void)
{
  g_tim1_overflow_count = 0U;
  g_wheel_last_timestamp_us = 0ULL;
  g_wheel_legacy_queue_enabled = 1U;
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    LED_BlinkCode(SD_LED_ERR_TIM);
  }
}

static void Wheel_QueuePushFromIsr(const Wheel_LogEvent_t *event)
{
  uint16_t next_head = (uint16_t)((g_wheel_rx_head + 1U) % WHEEL_RX_QUEUE_LEN);

  if (next_head == g_wheel_rx_tail)
  {
    g_wheel_rx_overflow_count++;
    return;
  }

  g_wheel_rx_queue[g_wheel_rx_head] = *event;
  g_wheel_rx_head = next_head;
}

static uint8_t Wheel_QueuePop(Wheel_LogEvent_t *event)
{
  uint8_t has_event = 0U;

  __disable_irq();
  if (g_wheel_rx_tail != g_wheel_rx_head)
  {
    *event = g_wheel_rx_queue[g_wheel_rx_tail];
    g_wheel_rx_tail = (uint16_t)((g_wheel_rx_tail + 1U) % WHEEL_RX_QUEUE_LEN);
    has_event = 1U;
  }
  __enable_irq();

  return has_event;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    g_tim1_overflow_count++;
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM8)
  {
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0U);
    g_ws2812_dma_done_count++;
    g_ws2812_dma_ready = 1U;
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  Wheel_LogEvent_t event;
  Wheel4_LogEvent_t wheel4_event;
  uint8_t channel;
  uint16_t capture;
  uint64_t timestamp_us;
  uint32_t delta_us;
  uint32_t raw_centi_kmh;

  channel = Wheel4_ChannelIndex(htim);
  if (channel >= WHEEL_CHANNEL_COUNT)
  {
    return;
  }

  capture = (uint16_t)HAL_TIM_ReadCapturedValue(htim, Wheel4_HALChannel(channel));
  if (htim->Instance == TIM1)
  {
    uint32_t overflow_count = g_tim1_overflow_count;

    if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) && (capture < 32768U))
    {
      overflow_count++;
    }

    timestamp_us = ((uint64_t)overflow_count * TIM1_TICKS_PER_OVERFLOW) + capture;
  }
  else
  {
    timestamp_us = Telemetry_GetTimestampUs();
  }

  delta_us = (g_wheel4_last_timestamp_us[channel] == 0ULL)
           ? 0U
           : (uint32_t)(timestamp_us - g_wheel4_last_timestamp_us[channel]);
  g_wheel4_last_timestamp_us[channel] = timestamp_us;
  g_wheel4_last_delta_us[channel] = delta_us;

  wheel4_event.timestamp_us = timestamp_us;
  wheel4_event.channel = channel;
  wheel4_event.delta_us = delta_us;
  wheel4_event.speed_centi_kmh = g_wheel4_filtered_centi_kmh[channel];

  if (delta_us >= SPEED_MIN_VALID_DELTA_US)
  {
    raw_centi_kmh = SPEED_CENTI_KMH_SCALE / delta_us;
    if (g_wheel4_filter_valid[channel] == 0U)
    {
      g_wheel4_filtered_centi_kmh[channel] = raw_centi_kmh;
      g_wheel4_filter_valid[channel] = 1U;
    }
    else
    {
      g_wheel4_filtered_centi_kmh[channel] =
          ((g_wheel4_filtered_centi_kmh[channel] * (SPEED_FILTER_ALPHA_DEN - SPEED_FILTER_ALPHA_NUM)) +
           (raw_centi_kmh * SPEED_FILTER_ALPHA_NUM)) / SPEED_FILTER_ALPHA_DEN;
    }

    wheel4_event.speed_centi_kmh = g_wheel4_filtered_centi_kmh[channel];
  }

  if (g_telemetry_queue_enabled != 0U)
  {
    Wheel4_QueuePushFromIsr(&wheel4_event);
  }

  if ((channel != 0U) || (g_wheel_legacy_queue_enabled == 0U))
  {
    return;
  }

  event.timestamp_us = timestamp_us;
  event.delta_us = (g_wheel_last_timestamp_us == 0ULL)
                 ? 0U
                 : (uint32_t)(timestamp_us - g_wheel_last_timestamp_us);
  g_wheel_last_timestamp_us = timestamp_us;

  Wheel_QueuePushFromIsr(&event);
}

uint8_t BSP_SD_Init(void)
{
  g_sd_hal_stage = 0U;
  g_sd_hal_error_code = HAL_SD_ERROR_NONE;

  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    g_sd_hal_stage = SD_LED_ERR_HAL_INIT;
    g_sd_hal_error_code = hsd.ErrorCode;
    return MSD_ERROR;
  }

  if ((SD_BRINGUP_USE_4BIT_BUS != 0U) &&
      (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK))
  {
    g_sd_hal_stage = SD_LED_ERR_WIDE_BUS;
    g_sd_hal_error_code = hsd.ErrorCode;
    return MSD_ERROR;
  }

  return MSD_OK;
}

uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  HAL_StatusTypeDef hal_status = HAL_ERROR;

  g_sd_ll_last_op = SD_LOWLEVEL_OP_READ;
  g_sd_ll_read_call_count++;
  g_sd_ll_last_sector = ReadAddr;
  g_sd_ll_last_blocks = NumOfBlocks;
  g_sd_ll_last_buffer_addr = (uint32_t)pData;
  g_sd_ll_last_tick = HAL_GetTick();

  if ((pData == NULL) || (NumOfBlocks == 0U) ||
      (SD_IsCardInserted() == GPIO_PIN_RESET))
  {
    g_sd_ll_read_fail_count++;
    g_sd_ll_last_hal_status = HAL_ERROR;
    g_sd_ll_last_hal_error = hsd.ErrorCode;
    g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);
    g_sd_hal_error_code = hsd.ErrorCode;
    return MSD_ERROR;
  }

  for (uint8_t attempt = 0U; attempt < SD_LOWLEVEL_IO_RETRY_COUNT; attempt++)
  {
    if (attempt != 0U)
    {
      g_sd_ll_retry_count++;
      HAL_Delay(SD_LOWLEVEL_IO_RETRY_DELAY_MS);
    }

    if (SD_WaitForTransferReady(SD_LOWLEVEL_READY_TIMEOUT_MS) != MSD_OK)
    {
      hal_status = HAL_TIMEOUT;
      (void)HAL_SD_Abort(&hsd);
      continue;
    }

    hal_status = HAL_SD_ReadBlocks(&hsd, (uint8_t *)pData, ReadAddr,
                                   NumOfBlocks, Timeout);
    g_sd_ll_last_hal_status = hal_status;
    g_sd_ll_last_hal_error = hsd.ErrorCode;
    g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);

    if (hal_status == HAL_OK)
    {
      if (SD_WaitForTransferReady(SD_LOWLEVEL_READY_TIMEOUT_MS) == MSD_OK)
      {
        g_sd_ll_last_hal_status = HAL_OK;
        g_sd_ll_last_hal_error = hsd.ErrorCode;
        g_sd_hal_error_code = hsd.ErrorCode;
        return MSD_OK;
      }

      hal_status = HAL_TIMEOUT;
      g_sd_ll_last_hal_status = hal_status;
    }

    g_sd_hal_error_code = hsd.ErrorCode;
    (void)HAL_SD_Abort(&hsd);
  }

  g_sd_ll_read_fail_count++;
  g_sd_ll_last_hal_status = hal_status;
  g_sd_ll_last_hal_error = hsd.ErrorCode;
  g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);
  g_sd_hal_error_code = hsd.ErrorCode;
  return MSD_ERROR;
}

uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  HAL_StatusTypeDef hal_status = HAL_ERROR;

  g_sd_ll_last_op = SD_LOWLEVEL_OP_WRITE;
  g_sd_ll_write_call_count++;
  g_sd_ll_last_sector = WriteAddr;
  g_sd_ll_last_blocks = NumOfBlocks;
  g_sd_ll_last_buffer_addr = (uint32_t)pData;
  g_sd_ll_last_tick = HAL_GetTick();

  if ((pData == NULL) || (NumOfBlocks == 0U) ||
      (SD_IsCardInserted() == GPIO_PIN_RESET))
  {
    g_sd_ll_write_fail_count++;
    g_sd_ll_last_hal_status = HAL_ERROR;
    g_sd_ll_last_hal_error = hsd.ErrorCode;
    g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);
    g_sd_hal_stage = SD_LED_ERR_WRITE_LOG;
    g_sd_hal_error_code = hsd.ErrorCode;
    return MSD_ERROR;
  }

  for (uint8_t attempt = 0U; attempt < SD_LOWLEVEL_IO_RETRY_COUNT; attempt++)
  {
    if (attempt != 0U)
    {
      g_sd_ll_retry_count++;
      HAL_Delay(SD_LOWLEVEL_IO_RETRY_DELAY_MS);
    }

    if (SD_WaitForTransferReady(SD_LOWLEVEL_READY_TIMEOUT_MS) != MSD_OK)
    {
      hal_status = HAL_TIMEOUT;
      (void)HAL_SD_Abort(&hsd);
      continue;
    }

    hal_status = HAL_SD_WriteBlocks(&hsd, (uint8_t *)pData, WriteAddr,
                                    NumOfBlocks, Timeout);
    g_sd_ll_last_hal_status = hal_status;
    g_sd_ll_last_hal_error = hsd.ErrorCode;
    g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);

    if (hal_status == HAL_OK)
    {
      if (SD_WaitForTransferReady(SD_LOWLEVEL_READY_TIMEOUT_MS) == MSD_OK)
      {
        g_sd_ll_last_hal_status = HAL_OK;
        g_sd_ll_last_hal_error = hsd.ErrorCode;
        g_sd_hal_error_code = hsd.ErrorCode;
        return MSD_OK;
      }

      hal_status = HAL_TIMEOUT;
      g_sd_ll_last_hal_status = hal_status;
    }

    g_sd_hal_stage = SD_LED_ERR_WRITE_LOG;
    g_sd_hal_error_code = hsd.ErrorCode;
    (void)HAL_SD_Abort(&hsd);
  }

  g_sd_ll_write_fail_count++;
  g_sd_ll_last_hal_status = hal_status;
  g_sd_ll_last_hal_error = hsd.ErrorCode;
  g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);
  g_sd_hal_stage = SD_LED_ERR_WRITE_LOG;
  g_sd_hal_error_code = hsd.ErrorCode;
  return MSD_ERROR;
}

static GPIO_PinState SD_IsCardInserted(void)
{
  return (HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin) == GPIO_PIN_RESET)
      ? GPIO_PIN_SET
      : GPIO_PIN_RESET;
}

static FRESULT SD_PulseLoggerInit(void)
{
  FRESULT res;
  UINT bytes_done = 0U;
  static const char header[] =
      "timestamp_ms,speed_kmh\r\n";

  g_sd_detect_state = SD_IsCardInserted();
  if (g_sd_detect_state == GPIO_PIN_RESET)
  {
    LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_SET);
    g_sd_bringup_status = SD_LED_ERR_FATFS;
    return FR_NOT_READY;
  }

  res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (res != FR_OK)
  {
    g_sd_bringup_status = SD_LED_ERR_MOUNT;
    return res;
  }

  res = SD_OpenNextLogFile();
  if (res != FR_OK)
  {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 0U);
    g_sd_bringup_status = SD_LED_ERR_OPEN_WHEEL;
    return res;
  }

  if (res == FR_OK)
  {
    res = f_write(&g_wheel_file, header, (UINT)(sizeof(header) - 1U), &bytes_done);
    if ((res == FR_OK) && (bytes_done != (sizeof(header) - 1U)))
    {
      res = FR_DISK_ERR;
    }
    if (res != FR_OK)
    {
      g_sd_bringup_status = SD_LED_ERR_WRITE_HEADER;
    }
  }

  if (res == FR_OK)
  {
    res = f_sync(&g_wheel_file);
    if (res != FR_OK)
    {
      g_sd_bringup_status = SD_LED_ERR_SYNC;
    }
  }

  g_wheel_log_count = 0U;
  g_sd_log_buffer_len = 0U;
  g_sd_log_last_write_tick = HAL_GetTick();
  g_sd_log_last_sync_tick = g_sd_log_last_write_tick;
  g_sd_log_unsynced_count = 0U;
  g_speed_filtered_centi_kmh = 0U;
  g_speed_last_pulse_ms = HAL_GetTick();
  g_speed_last_log_ms = g_speed_last_pulse_ms;
  g_speed_filter_valid = 0U;
  g_speed_seen_pulse = 0U;
  if (res == FR_OK)
  {
    g_sd_bringup_status = 0U;
  }
  g_sd_active_file = &g_wheel_file;
  return res;
}

static FRESULT SD_OpenNextLogFile(void)
{
  FRESULT res = FR_OK;

  for (uint16_t i = 0U; i <= SD_LOG_FILE_MAX_INDEX; i++)
  {
    (void)snprintf(g_sd_log_path, sizeof(g_sd_log_path), "0:/SPEED%03u.CSV", i);
    res = f_open(&g_wheel_file, g_sd_log_path, FA_CREATE_NEW | FA_WRITE);
    if (res == FR_OK)
    {
      return FR_OK;
    }

    if (res != FR_EXIST)
    {
      return res;
    }
  }

  return FR_EXIST;
}

static FRESULT SD_PulseLoggerProcess(void)
{
  FRESULT res = FR_OK;
  Wheel_LogEvent_t event;
  uint32_t now;

  while (Wheel_QueuePop(&event) != 0U)
  {
    Speed_ProcessPulse(&event);
  }

  now = HAL_GetTick();
  if (g_speed_seen_pulse == 0U)
  {
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin,
            (g_wheel_rx_overflow_count == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    return FR_OK;
  }

  Speed_UpdateTimeout(now);

  while ((now - g_speed_last_log_ms) >= SPEED_LOG_INTERVAL_MS)
  {
    g_speed_last_log_ms += SPEED_LOG_INTERVAL_MS;
    res = SD_PulseLoggerAppendSample(g_speed_last_log_ms, g_speed_filtered_centi_kmh);
    if (res != FR_OK)
    {
      return res;
    }
  }

  if ((g_sd_log_buffer_len > 0U) &&
      ((now - g_sd_log_last_write_tick) >= SD_LOG_FLUSH_IDLE_MS))
  {
    return SD_PulseLoggerFlush(0U);
  }

  if ((g_sd_log_unsynced_count > 0U) &&
      ((now - g_sd_log_last_sync_tick) >= SD_LOG_SYNC_INTERVAL_MS))
  {
    return SD_PulseLoggerFlush(1U);
  }

  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin,
          (g_wheel_rx_overflow_count == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  return FR_OK;
}

static void Speed_ProcessPulse(const Wheel_LogEvent_t *event)
{
  uint32_t raw_centi_kmh;

  if (event->delta_us == 0U)
  {
    g_speed_last_pulse_ms = (uint32_t)(event->timestamp_us / 1000ULL);
    g_speed_last_log_ms = g_speed_last_pulse_ms;
    g_speed_seen_pulse = 1U;
    return;
  }

  if (event->delta_us < SPEED_MIN_VALID_DELTA_US)
  {
    return;
  }

  if ((event->delta_us > SPEED_TIMEOUT_US) && (g_speed_filter_valid == 0U))
  {
    g_speed_last_pulse_ms = (uint32_t)(event->timestamp_us / 1000ULL);
    g_speed_last_log_ms = g_speed_last_pulse_ms;
    g_speed_seen_pulse = 1U;
    return;
  }

  raw_centi_kmh = SPEED_CENTI_KMH_SCALE / event->delta_us;
  if (g_speed_filter_valid == 0U)
  {
    g_speed_filtered_centi_kmh = raw_centi_kmh;
    g_speed_filter_valid = 1U;
  }
  else
  {
    g_speed_filtered_centi_kmh =
        ((g_speed_filtered_centi_kmh * (SPEED_FILTER_ALPHA_DEN - SPEED_FILTER_ALPHA_NUM)) +
         (raw_centi_kmh * SPEED_FILTER_ALPHA_NUM)) / SPEED_FILTER_ALPHA_DEN;
  }

  g_speed_last_pulse_ms = (uint32_t)(event->timestamp_us / 1000ULL);
  g_speed_seen_pulse = 1U;
  HAL_GPIO_TogglePin(USER_LED_0_GPIO_Port, USER_LED_0_Pin);
}

static void Speed_UpdateTimeout(uint32_t now_ms)
{
  if ((g_speed_filter_valid != 0U) &&
      ((now_ms - g_speed_last_pulse_ms) >= SPEED_TIMEOUT_MS))
  {
    g_speed_filtered_centi_kmh = 0U;
    g_speed_filter_valid = 0U;
  }
}

static FRESULT SD_PulseLoggerAppendSample(uint32_t timestamp_ms, uint32_t speed_centi_kmh)
{
  FRESULT res;
  char line[128];
  int line_len;

  line_len = snprintf(line, sizeof(line),
                      "%lu,%lu.%02lu\r\n",
                      (unsigned long)timestamp_ms,
                      (unsigned long)(speed_centi_kmh / 100U),
                      (unsigned long)(speed_centi_kmh % 100U));

  if ((line_len <= 0) || ((size_t)line_len >= sizeof(line)))
  {
    return FR_INT_ERR;
  }

  if (((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len)) &&
      (g_sd_log_buffer_len > 0U))
  {
    res = SD_PulseLoggerFlush(0U);
    if (res != FR_OK)
    {
      return res;
    }
  }

  if ((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len))
  {
    return FR_INT_ERR;
  }

  memcpy(&g_sd_log_buffer[g_sd_log_buffer_len], line, (size_t)line_len);
  g_sd_log_buffer_len += (UINT)line_len;
  g_wheel_log_count++;
  g_sd_log_unsynced_count++;

  if ((g_sd_log_buffer_len >= (SD_LOG_BUFFER_SIZE - 128U)) ||
      ((g_sd_log_unsynced_count % SD_LOG_SYNC_EVERY) == 0U))
  {
    return SD_PulseLoggerFlush((g_sd_log_unsynced_count % SD_LOG_SYNC_EVERY) == 0U);
  }

  return FR_OK;
}

static void SD_LogWriterReset(FIL *file)
{
  g_sd_active_file = file;
  g_sd_log_buffer_len = 0U;
  g_sd_log_last_write_tick = HAL_GetTick();
  g_sd_log_last_sync_tick = g_sd_log_last_write_tick;
  g_sd_log_unsynced_count = 0U;
  g_sd_write_fail_count = 0U;
  g_sd_sync_fail_count = 0U;
  g_sd_last_sync_fresult = FR_OK;
  g_sd_write_call_count = 0U;
  g_sd_sync_ok_count = 0U;
  g_sd_last_write_size = 0U;
  g_sd_max_buffer_len = 0U;
  g_sd_total_bytes_written = 0ULL;
  g_sd_last_flush_reason = SD_LOG_FLUSH_REASON_NONE;
  g_sd_ll_last_op = SD_LOWLEVEL_OP_NONE;
  g_sd_ll_read_call_count = 0U;
  g_sd_ll_write_call_count = 0U;
  g_sd_ll_retry_count = 0U;
  g_sd_ll_read_fail_count = 0U;
  g_sd_ll_write_fail_count = 0U;
  g_sd_ll_ready_timeout_count = 0U;
  g_sd_ll_last_sector = 0U;
  g_sd_ll_last_blocks = 0U;
  g_sd_ll_last_buffer_addr = 0U;
  g_sd_ll_last_hal_status = HAL_OK;
  g_sd_ll_last_hal_error = HAL_SD_ERROR_NONE;
  g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);
  g_sd_ll_last_card_state = 0U;
  g_sd_ll_last_wait_ms = 0U;
  g_sd_ll_last_tick = HAL_GetTick();
}

static FRESULT SD_LogWriterAppend(const uint8_t *data, UINT len)
{
  FRESULT res;
  UINT copy_len;

  if ((data == NULL) || (len == 0U))
  {
    return FR_OK;
  }

  while (len > 0U)
  {
    if (g_sd_log_buffer_len >= SD_LOG_BUFFER_SIZE)
    {
      res = SD_LogWriterFlush(0U, SD_LOG_FLUSH_REASON_BUFFER_FULL);
      if (res != FR_OK)
      {
        return res;
      }
    }

    copy_len = (UINT)(SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len);
    if (copy_len > len)
    {
      copy_len = len;
    }

    memcpy(&g_sd_log_buffer[g_sd_log_buffer_len], data, (size_t)copy_len);
    g_sd_log_buffer_len += copy_len;
    g_sd_log_unsynced_count += copy_len;

    if (g_sd_log_buffer_len > g_sd_max_buffer_len)
    {
      g_sd_max_buffer_len = g_sd_log_buffer_len;
    }

    data += copy_len;
    len -= copy_len;

    if (g_sd_log_buffer_len >= (SD_LOG_BUFFER_SIZE - SD_LOG_FLUSH_MARGIN))
    {
      res = SD_LogWriterFlush(0U, SD_LOG_FLUSH_REASON_BUFFER_FULL);
      if (res != FR_OK)
      {
        return res;
      }
    }
  }

  return FR_OK;
}

static FRESULT SD_LogWriterService(uint32_t flush_idle_ms, uint32_t sync_interval_ms)
{
  uint32_t now = HAL_GetTick();

  if ((g_sd_log_buffer_len > 0U) &&
      ((now - g_sd_log_last_write_tick) >= flush_idle_ms))
  {
    return SD_LogWriterFlush(0U, SD_LOG_FLUSH_REASON_IDLE);
  }

  if ((g_sd_log_unsynced_count > 0U) &&
      ((now - g_sd_log_last_sync_tick) >= sync_interval_ms))
  {
    return SD_LogWriterFlush(1U, SD_LOG_FLUSH_REASON_SYNC);
  }

  return FR_OK;
}

static FRESULT SD_LogWriterFlush(uint8_t force_sync, SD_LogFlushReason_t reason)
{
  FRESULT res = FR_OK;
  UINT bytes_done = 0U;
  UINT bytes_to_write;

  g_sd_last_flush_reason = reason;

  if (g_sd_log_buffer_len > 0U)
  {
    if (g_sd_active_file == NULL)
    {
      SD_RecordFault(SD_FAULT_FLUSH_NO_ACTIVE_FILE, FR_INVALID_OBJECT, 0U);
      return FR_INVALID_OBJECT;
    }

    bytes_to_write = g_sd_log_buffer_len;
    for (uint8_t attempt = 0U; attempt < SD_LOG_IO_RETRY_COUNT; attempt++)
    {
      bytes_done = 0U;
      res = f_write(g_sd_active_file, g_sd_log_buffer, bytes_to_write, &bytes_done);
      if ((res == FR_OK) && (bytes_done == bytes_to_write))
      {
        break;
      }

      if (bytes_done != 0U)
      {
        break;
      }

      SD_RecoverAfterIoError();
      HAL_Delay(SD_LOG_IO_RETRY_DELAY_MS);
    }

    if (res != FR_OK)
    {
      g_sd_write_fail_count++;
      SD_RecordFault(SD_FAULT_FLUSH_WRITE, res, bytes_done);
      return res;
    }

    if (bytes_done != bytes_to_write)
    {
      g_sd_write_fail_count++;
      SD_RecordFault(SD_FAULT_FLUSH_SHORT_WRITE, FR_DISK_ERR, bytes_done);
      return FR_DISK_ERR;
    }

    g_sd_write_call_count++;
    g_sd_last_write_size = bytes_done;
    g_sd_total_bytes_written += bytes_done;
    g_sd_log_buffer_len = 0U;
    g_sd_log_last_write_tick = HAL_GetTick();
  }

  if ((force_sync != 0U) && (g_sd_log_unsynced_count > 0U))
  {
    if (g_sd_active_file == NULL)
    {
      SD_RecordFault(SD_FAULT_FLUSH_NO_ACTIVE_FILE, FR_INVALID_OBJECT, 1U);
      return FR_INVALID_OBJECT;
    }

    for (uint8_t attempt = 0U; attempt < SD_LOG_IO_RETRY_COUNT; attempt++)
    {
      res = f_sync(g_sd_active_file);
      if (res == FR_OK)
      {
        break;
      }

      HAL_Delay(SD_LOG_IO_RETRY_DELAY_MS);
    }

    if (res == FR_OK)
    {
      g_sd_log_unsynced_count = 0U;
      g_sd_log_last_sync_tick = HAL_GetTick();
      g_sd_last_sync_fresult = FR_OK;
      g_sd_sync_ok_count++;
    }
    else
    {
      g_sd_sync_fail_count++;
      g_sd_last_sync_fresult = res;
#if (SD_LOG_SYNC_FAIL_FATAL != 0U)
      SD_RecordFault(SD_FAULT_FLUSH_SYNC, res, g_sd_log_unsynced_count);
#else
      g_sd_fault_stage = SD_FAULT_FLUSH_SYNC;
      g_sd_fault_fresult = res;
      g_sd_fault_detail = g_sd_log_unsynced_count;
      g_sd_fault_tick = HAL_GetTick();
      g_sd_fault_buffer_len = g_sd_log_buffer_len;
      g_sd_fault_unsynced_count = g_sd_log_unsynced_count;
      g_sd_fault_telemetry_count = g_telemetry_log_count;
      g_sd_log_last_sync_tick = HAL_GetTick();
      res = FR_OK;
#endif
    }
  }

  return res;
}

static FRESULT SD_PulseLoggerFlush(uint8_t force_sync)
{
  return SD_LogWriterFlush(force_sync,
                           (force_sync != 0U) ? SD_LOG_FLUSH_REASON_SYNC
                                               : SD_LOG_FLUSH_REASON_LEGACY);
}

static void LED_Set(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  HAL_GPIO_WritePin(port, pin, state);
}

static void LED_BlinkCode(uint8_t code)
{
  LED_BlinkCodeDetail(code, 0U);
}

static void LED_BlinkCodeDetail(uint8_t code, uint8_t detail)
{
  if (code == 0U)
  {
    code = 1U;
  }

  LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);

  while (1)
  {
    for (uint8_t i = 0U; i < code; i++)
    {
      HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
      HAL_Delay(LED_BLINK_ON_MS);
      HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
      HAL_Delay(LED_BLINK_OFF_MS);
    }

    if (detail != 0U)
    {
      HAL_Delay(LED_BLINK_PAUSE_MS);
      for (uint8_t i = 0U; i < detail; i++)
      {
        HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
        HAL_Delay(LED_BLINK_ON_MS);
        HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
        HAL_Delay(LED_BLINK_OFF_MS);
      }
    }

    HAL_Delay(LED_BLINK_PAUSE_MS);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
  while (1)
  {
    HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
    HAL_Delay(LED_BLINK_ON_MS);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
