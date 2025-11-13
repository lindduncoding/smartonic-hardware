/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : FIXED 3-SERVO SPEED BUMP - ACCURATE SPEED DETECTION
  *                   Optimized for 3m track with better speed calculation
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "ESP8266_STM32.h"
#include "cJSON.h"
#include "ssd1306.h"
#include "fonts.h"
#include "q_table.h"
#include "state_classifier.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Pins
#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA
#define IR_SENSOR_PIN GPIO_PIN_1
#define IR_SENSOR_PORT GPIOA

// Timing - OPTIMIZED for better speed detection
#define SAMPLE_INTERVAL_MS 30        // Faster sampling (was 50ms)
#define SPEED_SAMPLES 10             // More samples for better average (was 3)
#define SPEED_CALCULATION_SAMPLES 3  // Use last 3 readings for speed calc

// Thresholds - ADJUSTED for realistic speeds
#define SPEED_THRESHOLD_LOW 3        // Lower threshold (was 5)
#define SPEED_THRESHOLD_HIGH 10      // Lower threshold (was 15)
#define MIN_SPEED_THRESHOLD 0.5f     // Lower minimum (was 1.0f)

// Servo Angles - REDUCED POWER CONSUMPTION
#define SERVO1_DOWN_ANGLE 0
#define SERVO1_UP_ANGLE 30           // Reduced from 45
#define SERVO2_DOWN_ANGLE 0
#define SERVO2_UP_ANGLE 30           // Reduced from 60
#define SERVO3_DOWN_ANGLE 0
#define SERVO3_UP_ANGLE 30           // Reduced from 45
#define SERVO4_DOWN_ANGLE 0
#define SERVO4_UP_ANGLE 30

// Detection - OPTIMIZED for 3m track
#define VALID_DISTANCE_MIN 3         // Closer minimum (was 5)
#define VALID_DISTANCE_MAX 100       // Wider range (was 50)
#define IR_DEBOUNCE_MS 300           // Longer debounce (was 50)
#define BUMP_COOLDOWN_MS 1500        // Shorter cooldown (was 2000)
#define NO_VEHICLE_TIMEOUT_MS 2000   // Longer timeout (was 1000)

// Speed calculation constants for 3m track
#define MIN_DISTANCE_CHANGE 2        // Minimum 2cm change to calculate speed
#define MAX_REASONABLE_SPEED 10.0f   // Maximum 30 km/h for safety
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
// Ultrasonic - IMPROVED STRUCTURE
volatile uint32_t IC_Val1 = 0;
volatile uint32_t IC_Val2 = 0;
volatile uint32_t Difference = 0;
volatile uint8_t Is_First_Captured = 0;
volatile uint8_t capture_done = 0;
volatile uint32_t distance_cm = 0;

typedef struct {
    uint32_t distance;
    uint32_t timestamp;
    uint8_t valid;
} DistanceReading;

DistanceReading distance_buffer[SPEED_SAMPLES];
uint8_t buffer_index = 0;
uint8_t buffer_filled = 0;

// Speed calculation variables
float current_speed_kmh = 0.0f;
float smoothed_speed_kmh = 0.0f;
float max_speed_detected = 0.0f;
uint32_t last_valid_detection = 0;
uint32_t valid_speed_readings = 0;

// Ultrasonic statistics
uint32_t ultrasonic_read_count = 0;
uint32_t ultrasonic_success_count = 0;

// IR Sensor
uint32_t ir_vehicle_count = 0;
uint32_t ir_last_detection_time = 0;
uint8_t ir_last_stable_state = 1;
uint8_t ir_current_state = 1;
uint8_t ir_sensor_changed = 0;

// Density
float current_density = 0.0f;
uint32_t density_window_start = 0;
uint32_t density_window_count = 0;

// Control
uint8_t bump_is_up = 0;
uint32_t last_bump_change = 0;
uint32_t last_oled_update = 0;
uint32_t last_mqtt_publish = 0;
uint32_t last_diagnostic = 0;
volatile uint8_t mqtt_publish_pending = 0;
uint32_t total_vehicles = 0;

// Bump
struct angles {
    uint8_t target_angle;
    uint8_t center_angle;
    uint8_t bump_status;
};

// Message buffers
char MSG[250] = {0};
char *json_str = NULL;
char ip_buf[16];
char mqtt_buffer[256] = {0};
// DIAGNOSTIC MODE
uint8_t diagnostic_mode = 1;

// Previous distance for speed calculation
uint32_t prev_valid_distance = 0;
uint32_t prev_valid_time = 0;

// RL
uint8_t current_state = 0;
uint8_t action = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void delay_us(uint16_t us);
uint8_t HCSR04_Read(void);
void add_distance_reading(uint32_t distance, uint32_t timestamp);
float calculate_speed_improved(void);
void check_ir_sensor_state(void);
void update_density_calculation(uint32_t current_time);
void servo_set_angle(uint8_t servo_num, uint8_t angle);
void control_speed_bump(float speed_kmh, float density);
void update_oled_display(float speed, float density, uint8_t bump_status);
char *create_data_packet(int speed, int density, int bump, int count);
void print_diagnostics(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 100);
    return len;
}

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

struct angles Set_Servo_RL_Action(uint8_t action)
{
    struct angles A;
    if (action == 0) {
        A.target_angle = SERVO1_DOWN_ANGLE;
        A.center_angle = SERVO2_DOWN_ANGLE;
        A.bump_status = 0;
    } else {
        A.target_angle = SERVO1_UP_ANGLE;
        A.center_angle = SERVO2_UP_ANGLE;
        A.bump_status = 1;
    }
    return A;
}

uint8_t select_best_action(int8_t state)
{
    if (state < 0 || state >= NUM_STATES) {
        return 0;
    }
    return OPTIMAL_POLICY[state];
}

void add_distance_reading(uint32_t distance, uint32_t timestamp)
{
    distance_buffer[buffer_index].distance = distance;
    distance_buffer[buffer_index].timestamp = timestamp;
    distance_buffer[buffer_index].valid = 1;

    buffer_index++;
    if (buffer_index >= SPEED_SAMPLES) {
        buffer_index = 0;
        buffer_filled = 1;
    }
}

// IMPROVED SPEED CALCULATION - Better for 3m track
float calculate_speed_improved(void)
{
    // Need at least 2 valid readings
    if (!buffer_filled && buffer_index < 2) {
        return 0.0f;
    }

    // Get the most recent valid readings
    int count = 0;
    int indices[SPEED_CALCULATION_SAMPLES];

    // Find last N valid readings
    for (int i = 0; i < SPEED_SAMPLES && count < SPEED_CALCULATION_SAMPLES; i++) {
        int idx = (buffer_index - 1 - i + SPEED_SAMPLES) % SPEED_SAMPLES;
        if (distance_buffer[idx].valid) {
            indices[count++] = idx;
        }
    }

    if (count < 2) {
        return 0.0f;
    }

    // Calculate speed using first and last reading
    int oldest_idx = indices[count - 1];
    int newest_idx = indices[0];

    int32_t distance_diff = (int32_t)distance_buffer[oldest_idx].distance -
                            (int32_t)distance_buffer[newest_idx].distance;

    float time_diff_s = (distance_buffer[newest_idx].timestamp -
                         distance_buffer[oldest_idx].timestamp) / 1000.0f;

    // Need minimum time difference
    if (time_diff_s < 0.05f) {  // At least 50ms
        return 0.0f;
    }

    // Need minimum distance change
    if (abs(distance_diff) < MIN_DISTANCE_CHANGE) {
        return 0.0f;
    }

    // Calculate speed in cm/s
    float speed_cms = distance_diff / time_diff_s;

    // Convert to km/h
    float speed_kmh = speed_cms * 0.036f;

    // Filter out negative speeds (moving away) and unreasonable speeds
    if (speed_kmh < 0 || speed_kmh > MAX_REASONABLE_SPEED) {
        return 0.0f;
    }

    // Only count approaching vehicles (positive speed)
    if (speed_kmh > MIN_SPEED_THRESHOLD) {
        valid_speed_readings++;

        // Track maximum speed
        if (speed_kmh > max_speed_detected) {
            max_speed_detected = speed_kmh;
        }

        if (diagnostic_mode) {
            sprintf(MSG, "[SPEED] Δd=%ld cm, Δt=%.2fs, v=%.1f km/h ✓\r\n",
                    distance_diff, time_diff_s, speed_kmh);
            HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
        }
    }

    return speed_kmh;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (Is_First_Captured == 0)
        {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            Is_First_Captured = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            if (IC_Val2 >= IC_Val1)
                Difference = IC_Val2 - IC_Val1;
            else
                Difference = (0xFFFFFFFF - IC_Val1) + IC_Val2 + 1;

            distance_cm = Difference / 58;
            Is_First_Captured = 0;
            capture_done = 1;

            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}

uint8_t HCSR04_Read(void)
{
    uint32_t timeout;

    ultrasonic_read_count++;
    capture_done = 0;
    Is_First_Captured = 0;
    distance_cm = 0;
    Difference = 0;

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    timeout = HAL_GetTick() + 60;
    while (!capture_done && HAL_GetTick() < timeout);

    if (!capture_done) {
        return 0;
    }

    if (distance_cm < VALID_DISTANCE_MIN || distance_cm > VALID_DISTANCE_MAX) {
        return 0;
    }

    ultrasonic_success_count++;
    return 1;
}

void check_ir_sensor_state(void)
{
    uint8_t new_state = HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN);
    uint32_t current_time = HAL_GetTick();

    if (new_state != ir_current_state) {
        ir_current_state = new_state;
        ir_last_detection_time = current_time;
        ir_sensor_changed = 1;
    }

    if (ir_sensor_changed &&
        (current_time - ir_last_detection_time >= IR_DEBOUNCE_MS))
    {
        if (ir_current_state == 0 && ir_last_stable_state == 1)
        {
            ir_vehicle_count++;
            density_window_count++;
            total_vehicles++;

            sprintf(MSG, "[IR] ✓ VEHICLE #%lu detected!\r\n", total_vehicles);
            HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
        }

        ir_last_stable_state = ir_current_state;
        ir_sensor_changed = 0;
    }
}

void update_density_calculation(uint32_t current_time)
{
    if (current_time - density_window_start >= 60000)
    {
        current_density = (float)density_window_count;

        sprintf(MSG, "[DENSITY] %.0f vehicles/minute\r\n", current_density);
        HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

        density_window_start = current_time;
        density_window_count = 0;
    }
}

// Servo control with REDUCED ANGLES for lower power
// Servo control dengan TIM3 Channel 1, 2, 3
void servo_set_angle(uint8_t servo_num, uint8_t angle)
{
    if (angle > 180) angle = 180;
    uint32_t pulse = 500 + ((angle * 2000) / 180);

    switch(servo_num) {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);  // Servo 1 - TIM3 CH1
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);  // Servo 2 - TIM3 CH2
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse);  // Servo 3 - TIM3 CH3
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse);  // Servo 4 - TIM4 CH3 (NEW!)
            break;
    }
}

// MODIFIED: Control all 3 servos simultaneously
void control_speed_bump(float speed_kmh, float density)
{
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_bump_change < BUMP_COOLDOWN_MS) return;

    // Decide using RL
    current_state = classify_state(density, speed_kmh);

    if (current_state >= 0 && current_state < NUM_STATES) {
        // Pilih action berdasarkan Q-table
        action = select_best_action(current_state);

        snprintf(MSG, sizeof(MSG), "Q-Table Decision: Action=%u\r\n", action);
        HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

        // Eksekusi action dengan animasi - 4 servos
        struct angles result = Set_Servo_RL_Action(action);

        servo_set_angle(1, result.target_angle);
        HAL_Delay(50);
        servo_set_angle(2, result.center_angle);
        HAL_Delay(50);
        servo_set_angle(3, result.target_angle);
        HAL_Delay(50);
        servo_set_angle(4, result.target_angle);  // NEW: 4th servo

        bump_is_up = result.bump_status;
        last_bump_change = current_time;

        snprintf(MSG, sizeof(MSG),
                 "[BUMP] ALL 4 SERVOS ACTION %u - Speed: %.1f km/h\r\n",
                 bump_is_up, speed_kmh);
        HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

    } else {
        if (speed_kmh < MIN_SPEED_THRESHOLD) return;

        if (speed_kmh > SPEED_THRESHOLD_HIGH && !bump_is_up)
        {
            // Raise all 4 servos with staggered timing
            servo_set_angle(1, SERVO1_UP_ANGLE);
            HAL_Delay(50);
            servo_set_angle(2, SERVO2_UP_ANGLE);
            HAL_Delay(50);
            servo_set_angle(3, SERVO3_UP_ANGLE);
            HAL_Delay(50);
            servo_set_angle(4, SERVO4_UP_ANGLE);  // NEW: 4th servo

            bump_is_up = 1;
            last_bump_change = current_time;

            sprintf(MSG, "[BUMP] ⚠️  ALL 4 SERVOS RAISED - Speed: %.1f km/h\r\n", speed_kmh);
            HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
        }
        else if (speed_kmh < SPEED_THRESHOLD_LOW && bump_is_up)
        {
            // Lower all 4 servos
            servo_set_angle(1, SERVO1_DOWN_ANGLE);
            HAL_Delay(50);
            servo_set_angle(2, SERVO2_DOWN_ANGLE);
            HAL_Delay(50);
            servo_set_angle(3, SERVO3_DOWN_ANGLE);
            HAL_Delay(50);
            servo_set_angle(4, SERVO4_DOWN_ANGLE);  // NEW: 4th servo

            bump_is_up = 0;
            last_bump_change = current_time;

            sprintf(MSG, "[BUMP] ✓ ALL 4 SERVOS LOWERED - Speed: %.1f km/h\r\n", speed_kmh);
            HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
        }
    }
}


void update_oled_display(float speed, float density, uint8_t bump_status)
{
    char line[32];

    SSD1306_Fill(SSD1306_COLOR_BLACK);

    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("3-SERVO BUMP", &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_GotoXY(0, 12);
    snprintf(line, sizeof(line), "Speed:%.1f km/h", speed);
    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_GotoXY(0, 24);
    snprintf(line, sizeof(line), "Max:%.1f km/h", max_speed_detected);
    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_GotoXY(0, 36);
    snprintf(line, sizeof(line), "IR:%lu Den:%.0f", ir_vehicle_count, density);
    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_GotoXY(0, 48);
    snprintf(line, sizeof(line), "Bump:%s D:%lucm",
             bump_status ? "UP" : "DN", distance_cm);
    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_UpdateScreen();
}

char *create_data_packet(int speed, int density, int bump, int count)
{
    char *string = NULL;
    cJSON *packet = cJSON_CreateObject();
    if (packet == NULL) return NULL;

    cJSON_AddNumberToObject(packet, "speed", speed);
    cJSON_AddNumberToObject(packet, "density", density);
    cJSON_AddNumberToObject(packet, "bump", bump);
    cJSON_AddNumberToObject(packet, "total_vehicles", count);

    string = cJSON_Print(packet);
    cJSON_Delete(packet);
    return string;
}

void print_diagnostics(void)
{
    uint32_t success_rate = (ultrasonic_read_count > 0) ?
        (ultrasonic_success_count * 100 / ultrasonic_read_count) : 0;

    sprintf(MSG, "\r\n╔═══ DIAGNOSTIC (4-SERVO) ═══╗\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

    sprintf(MSG, "║ US Success: %lu%% | Reads: %lu/%lu\r\n",
            success_rate, ultrasonic_success_count, ultrasonic_read_count);
    HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

    sprintf(MSG, "║ Distance: %lu cm | Valid speeds: %lu\r\n",
            distance_cm, valid_speed_readings);
    HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

    sprintf(MSG, "║ Current: %.1f km/h | Max: %.1f km/h\r\n",
            smoothed_speed_kmh, max_speed_detected);
    HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

    sprintf(MSG, "║ IR: %s | Count: %lu\r\n",
            HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN) ? "Clear" : "Blocked",
            ir_vehicle_count);
    HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

    sprintf(MSG, "║ 4 Servos: %s | Density: %.0f v/m\r\n",
            bump_is_up ? "UP" : "DOWN", current_density);
    HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

    sprintf(MSG, "╚═════════════════════════════╝\r\n\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
}

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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */
  // Initialize OLED
  SSD1306_Init();
  HAL_Delay(100);

  SSD1306_Fill(SSD1306_COLOR_BLACK);
  SSD1306_GotoXY(0, 10);
  SSD1306_Puts("3-SERVO BUMP", &Font_11x18, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(0, 35);
  SSD1306_Puts("TIM3 CONTROL", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(0, 50);
  SSD1306_Puts("OPTIMIZED v2", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_UpdateScreen();
  HAL_Delay(2000);

  // Initialize servos dengan TIM3 - REDUCED power draw
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // Servo 1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // Servo 2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  // Servo 3
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);  // Servo 4
  // Initialize satu per satu
  servo_set_angle(1, SERVO1_DOWN_ANGLE);
    HAL_Delay(200);
    servo_set_angle(2, SERVO2_DOWN_ANGLE);
    HAL_Delay(200);
    servo_set_angle(3, SERVO3_DOWN_ANGLE);
    HAL_Delay(200);
    servo_set_angle(4, SERVO4_DOWN_ANGLE);
    HAL_Delay(200);
    bump_is_up = 0;


  sprintf(MSG, "\r\n╔═══════════════════════════╗\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  sprintf(MSG, "║ 4-SERVO BUMP SYSTEM      ║\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  sprintf(MSG, "║ TIM3: CH1,CH2,CH3        ║\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  sprintf(MSG, "║ TIM4: CH3 (PB8)          ║\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  sprintf(MSG, "╚═══════════════════════════╝\r\n\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

  // ESP Init
  if (ESP_Init() != ESP8266_OK){
      sprintf(MSG, "[ERROR] ESP8266 initialization failed\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

      SSD1306_Fill(SSD1306_COLOR_BLACK);
      SSD1306_GotoXY(0, 20);
      SSD1306_Puts("ESP8266 ERROR!", &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen();
      Error_Handler();
  }

  if (ESP_ConnectWiFi("Pixel_6094", "kurangpanjang", ip_buf, sizeof(ip_buf)) != ESP8266_OK){
      sprintf(MSG, "[ERROR] WiFi connection failed\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

      SSD1306_Fill(SSD1306_COLOR_BLACK);
      SSD1306_GotoXY(0, 20);
      SSD1306_Puts("WiFi FAILED!", &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen();
      HAL_Delay(2000);

  } else {
      sprintf(MSG, "[OK] WiFi connected: %s\r\n", ip_buf);
      HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

      SSD1306_Fill(SSD1306_COLOR_BLACK);
      SSD1306_GotoXY(0, 15);
      SSD1306_Puts("WiFi Connected!", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY(0, 30);
      SSD1306_Puts("IP:", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY(0, 45);
      SSD1306_Puts(ip_buf, &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen();
      HAL_Delay(1500);
  }

  if (ESP_MQTT_Connect("10.73.13.82", 1883, "STM32SpeedBump", NULL, NULL, 60) != ESP8266_OK){
      sprintf(MSG, "[WARN] MQTT connection failed\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

      SSD1306_Fill(SSD1306_COLOR_BLACK);
      SSD1306_GotoXY(0, 20);
      SSD1306_Puts("MQTT Failed", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen();
      HAL_Delay(1000);

  } else {
      sprintf(MSG, "[OK] MQTT connected\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

      SSD1306_Fill(SSD1306_COLOR_BLACK);
      SSD1306_GotoXY(0, 25);
      SSD1306_Puts("MQTT Connected!", &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen();
      HAL_Delay(1000);
  }

  // POWER WARNING
  sprintf(MSG, "⚠  POWER INFO:\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  sprintf(MSG, "   4 servos = ~800mA @ 5V (200mA each)\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  sprintf(MSG, "   USB 2.0 = 500mA limit\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  sprintf(MSG, "   USB 3.0 = 900mA limit\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  sprintf(MSG, "   RECOMMENDATION: Use external 5V power!\r\n\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

  // Start timers
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

  // Initialize buffers
  for (uint8_t i = 0; i < SPEED_SAMPLES; i++) {
      distance_buffer[i].valid = 0;
  }

  // Initialize IR
  ir_last_stable_state = HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN);
  ir_current_state = ir_last_stable_state;
  density_window_start = HAL_GetTick();

  sprintf(MSG, "[CONFIG] Servo angles reduced for lower power\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  sprintf(MSG, "[CONFIG] TIM3 Control: CH1, CH2, CH3\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  sprintf(MSG, "[CONFIG] Speed sampling: %dms interval\r\n", SAMPLE_INTERVAL_MS);
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  sprintf(MSG, "[CONFIG] Buffer size: %d samples\r\n", SPEED_SAMPLES);
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  sprintf(MSG, "[CONFIG] Speed thresholds: LOW=%d, HIGH=%d km/h\r\n\r\n",
          SPEED_THRESHOLD_LOW, SPEED_THRESHOLD_HIGH);
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

  sprintf(MSG, "[OK] System ready! Waiting for vehicles...\r\n\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);

  uint32_t last_measurement_time = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  	    {
  	      uint32_t current_time = HAL_GetTick();

  	      // FASTER SENSOR READING (every 30ms)
  	      if (current_time - last_measurement_time >= SAMPLE_INTERVAL_MS)
  	      {
  	          last_measurement_time = current_time;

  	          if (HCSR04_Read())
  	          {
  	              add_distance_reading(distance_cm, current_time);
  	              last_valid_detection = current_time;

  	              // Calculate speed with improved algorithm
  	              current_speed_kmh = calculate_speed_improved();

  	              // Apply smoothing only if we have a valid speed
  	              if (current_speed_kmh > MIN_SPEED_THRESHOLD) {
  	                  if (smoothed_speed_kmh == 0.0f) {
  	                      smoothed_speed_kmh = current_speed_kmh;
  	                  } else {
  	                      // Less aggressive smoothing for better responsiveness
  	                      smoothed_speed_kmh = 0.6f * smoothed_speed_kmh + 0.4f * current_speed_kmh;
  	                  }
  	              }
  	          }
      	      // IR SENSOR CHECK runs regardless of US state
      	      check_ir_sensor_state();

      	      // DENSITY CALCULATION
      	      update_density_calculation(current_time);

      	      // CONTROL BUMP BASED ON SPEED AND DENSITY
      	      control_speed_bump(smoothed_speed_kmh, current_density);

  	          // Check timeout
  	          if (current_time - last_valid_detection > NO_VEHICLE_TIMEOUT_MS)
  	          {
  	              smoothed_speed_kmh = 0.0f;
  	              current_speed_kmh = 0.0f;
  	          }
  	      }

  	      // OLED UPDATE (every 300ms for responsiveness)
  	      if (current_time - last_oled_update >= 300)
  	      {
  	          last_oled_update = current_time;
  	          update_oled_display(smoothed_speed_kmh, current_density, bump_is_up);
  	      }

  	      // DIAGNOSTIC REPORT (every 3 seconds)
  	      if (current_time - last_diagnostic >= 3000)
  	      {
  	          last_diagnostic = current_time;

  	          if (smoothed_speed_kmh > 0 && current_density > 0) {
  	        	  json_str = create_data_packet(smoothed_speed_kmh, current_density, bump_is_up, total_vehicles);
  	          } else {
  		          sprintf(MSG, "[MQTT] Skipping JSON because speed and density is null\r\n");
  		          HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  	          }

  	          if (json_str != NULL) {
  	              strncpy(mqtt_buffer, json_str, sizeof(mqtt_buffer) - 1);
  	              mqtt_publish_pending = 1;
  		          sprintf(MSG, "[MQTT] Created JSON and MQTT status is: %u\r\n", mqtt_publish_pending);
  		          HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  	          }
  	          print_diagnostics();
  	      }

  	      // SEND MQTT ONLY when no vehicle is nearby (non-critical time)
  	      if (mqtt_publish_pending &&
  	          (current_time - last_valid_detection > 1000)) {  // 1 second after last detection

  	          ESP_MQTT_Publish("smartronic", mqtt_buffer, 0);
  	          mqtt_publish_pending = 0;

  	          sprintf(MSG, "[MQTT] Published during idle time\r\n");
  	          HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  	      }
  	    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
