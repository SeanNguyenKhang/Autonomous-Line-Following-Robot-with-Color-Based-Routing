/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ENCODER_CPR        1320.0f     // Số xung mỗi vòng (sửa theo encoder của bạn)
#define SAMPLE_TIME_MS     20          // Thời gian lấy mẫu = 20ms
#define MOTOR_GEAR_RATIO   1.0f        // Nếu có hộp số, nhân thêm

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;  
extern ADC_HandleTypeDef hadc1;  // Khai báo ADC
/* --- CẤU HÌNH TCS3200 --- */
// S0, S1: Chỉnh tần số (Output Scaling)
#define TCS_S0_PIN   GPIO_PIN_11
#define TCS_S0_PORT  GPIOA
#define TCS_S1_PIN   GPIO_PIN_12
#define TCS_S1_PORT  GPIOA

// S2, S3: Chọn bộ lọc màu (Filter)
#define TCS_S2_PIN   GPIO_PIN_3
#define TCS_S2_PORT  GPIOB
#define TCS_S3_PIN   GPIO_PIN_4
#define TCS_S3_PORT  GPIOB

// OUT: Chân đọc tín hiệu từ TCS3200
#define TCS_OUT_PIN  GPIO_PIN_15
#define TCS_OUT_PORT GPIOA
char uart_buf[100];
uint32_t prev_encoder = 0;
uint32_t curr_encoder = 0;
float rpm = 0;
float pwm_percent = 0;
uint32_t t_ms = 0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* PID config (dùng hệ số bạn cho) */
//static const float PID_KP = 0.73716f;
//static const float PID_KI = 9.9371f;
//static const float PID_KD = 0.011797f;

///* PID runtime variables */
//static float pid_error_prev = 0.0f;
//static float pid_error_sum  = 0.0f;

///* Setpoint: 0.3 m/s -> RPM */
#define WHEEL_DIAMETER_M 0.064f   // 64 mm = 0.064 m
static const float RPM_SETPOINT = (0.3f * 60.0f) / (3.14159265f * WHEEL_DIAMETER_M); // ~89.55 RPM

/* Hằng số Calib */
#define NUM_SENSORS 5 
#define CALIB_DURATION_MS 7000

///* Optional: anti-windup and PWM limits */
//#define PWM_MAX 100.0f
//#define PWM_MIN 0.0f
//#define I_SUM_LIMIT 500.0f   // giới hạn cho tích phân (tùy chỉnh nếu cần)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t go_to_finish_start_ms = 0;
uint8_t  branch_detected = 0;    // đã gặp ngã rẽ chưa?
typedef enum {
    STATE_GO_TO_STATION = 0,   // Đang chạy từ Start tới trạm tải hàng
    STATE_READ_COLOR,          // Đã tới trạm, dừng & đọc màu
    STATE_ESCAPE_STATION,      // Chạy thẳng ra khỏi trạm
    STATE_GO_TO_FINISH,        // Bám line, rẽ nhánh theo màu, tới Kết thúc
    STATE_FINISHED             // Đã tới Kết thúc, dừng hẳn
} RobotState;

RobotState robot_state = STATE_GO_TO_STATION;
int target_color = 0;   // 0 = chưa biết, 1 = ĐỎ, 2 = XANH
uint32_t escape_start_ms = 0;
uint16_t adc_values[NUM_SENSORS]; // Mảng DMA lưu 5 giá trị ADC
uint16_t sensor_min[NUM_SENSORS]; // Mảng lưu MIN
uint16_t sensor_max[NUM_SENSORS]; // Mảng lưu MAX

// Khai báo hàm
void motor1_set_pwm(float duty);
void motor2_set_pwm(float duty);
void calib_auto_run(float calib_pwm);

void motor1_set_pwm(float duty)
{
    if (duty > 100) duty = 100;
    if (duty < 0)   duty = 0;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
        duty / 100.0f * __HAL_TIM_GET_AUTORELOAD(&htim1));
}

void motor2_set_pwm(float duty)
{
    if (duty > 100) duty = 100;
    if (duty < 0)   duty = 0;

    // Sử dụng TIM_CHANNEL_2 cho Động cơ 2 (PA9)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
        duty / 100.0f * __HAL_TIM_GET_AUTORELOAD(&htim1));
}

// --- BIẾN PID ---
float Kp = 0.0f;  // Hệ số P (Sẽ chỉnh sau)
float Ki = 0.0f;  // Hệ số I
float Kd = 0.0f;  // Hệ số D

float error = 0;
float last_error = 0;
float total_error = 0; // Dùng cho Ki (ít dùng cho dò line đơn giản)

float base_speed = 25.0f; // Tốc độ chạy cơ bản (25%)
float max_speed = 60.0f;  // Tốc độ tối đa cho phép

// Khai báo hàm tính lỗi
float get_line_error(void);

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM3)  // Timer lấy mẫu 20 ms
//    {
//        // ... (phần tính toán RPM giữ nguyên) ...
//        curr_encoder = __HAL_TIM_GET_COUNTER(&htim2);
//        int32_t diff = (int32_t)(curr_encoder - prev_encoder);

//        if (diff > 32767)  diff -= 65536;
//        if (diff < -32768) diff += 65536;

//        rpm = (diff / ENCODER_CPR) * (60000.0f / SAMPLE_TIME_MS);

//        t_ms += SAMPLE_TIME_MS;

//        // ========== PID control ==========
//        float dt = (float)SAMPLE_TIME_MS / 1000.0f;    // seconds
//        float error = RPM_SETPOINT - rpm;
//        
//        // Đạo hàm
//        float derivative = (error - pid_error_prev) / dt;

//        // PID output TẠM THỜI (chưa bao gồm tích phân)
//        float pwm_out = PID_KP * error + PID_KD * derivative + PID_KI * pid_error_sum; // Dùng pid_error_sum cũ

//        // ======================================
//        // >> CHÈN ANTI-WINDUP (CONDITIONAL INTEGRATION) TẠI ĐÂY <<
//        // ======================================
//        
//        // 1. Giới hạn đầu ra TẠM THỜI trước
//        float pwm_clamped = pwm_out;
//        if (pwm_clamped > PWM_MAX) pwm_clamped = PWM_MAX;
//        if (pwm_clamped < PWM_MIN) pwm_clamped = PWM_MIN;

//        // 2. Chỉ tích lũy sai số (tính toán pid_error_sum mới)
//        // Nếu đầu ra PWM KHÔNG bị giới hạn (pwm_out == pwm_clamped), thì Tích phân (pid_error_sum) được tính.
//        // Nếu bị giới hạn, pid_error_sum sẽ giữ nguyên (hoặc thay đổi rất ít)
//        if (pwm_out == pwm_clamped) {
//            pid_error_sum += error * dt;
//            // *Optional*: Vẫn có thể giới hạn tối đa tích phân để đảm bảo an toàn
//            if (pid_error_sum > I_SUM_LIMIT) pid_error_sum = I_SUM_LIMIT;
//            if (pid_error_sum < -I_SUM_LIMIT) pid_error_sum = -I_SUM_LIMIT;
//        }

//        // 3. Đầu ra cuối cùng là giá trị đã được giới hạn
//        pwm_percent = pwm_clamped;
//        
//        // Áp dụng PWM
//        motor_set_pwm1(pwm_percent);

//        // Lưu lại error cũ
//        pid_error_prev = error;
//        // ===================================

//        // ... (phần log giữ nguyên) ...
//        int len = sprintf(uart_buf, "%lu, %.2f, %.2f\r\n",
//                          (unsigned long)t_ms, pwm_percent, rpm);
//        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, 100);

//        prev_encoder = curr_encoder;
//    }
//}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t Is_At_Branch(void);
float get_line_error(void);
uint8_t Is_Middle_3_Black(void);
uint8_t Is_Finish_Line(void); 
// TCS3200
void TCS_Select_Filter(uint8_t color);
uint32_t TCS_Measure_Frequency(void);
int TCS_Identify_Color(void);

// Xử lý màu & quay robot
int Read_Package_Color(void);
void Robot_Spin_Circles(uint8_t circles);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE BEGIN 0 */
void calib_auto_run(float calib_pwm)
{
    __HAL_TIM_MOE_ENABLE(&htim1);

    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_min[i] = 4095;
        sensor_max[i] = 0;
    }

    // ----------------------------
    // QUAY 1 VÒNG ĐÚNG CHIỀU
    // ----------------------------

    // MOTOR TRÁI: TIẾN (theo chiều kim đồng hồ)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    motor1_set_pwm(calib_pwm);

    // MOTOR PHẢI: LÙI (ngược chiều kim đồng hồ)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    motor2_set_pwm(calib_pwm);

    // Thu thập 5 giây
    for (int k = 0; k < 1000; k++) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (adc_values[i] < sensor_min[i]) sensor_min[i] = adc_values[i];
            if (adc_values[i] > sensor_max[i]) sensor_max[i] = adc_values[i];
        }
        HAL_Delay(5);
    }

    // Dừng motor
    motor1_set_pwm(0);
    motor2_set_pwm(0);
    HAL_Delay(300);
}
/* USER CODE END 0 */
   
    // Để debug (tùy chọn): Nếu bạn muốn in kết quả cuối cùng ra UART
    /*
    char calib_msg[100];
    for(int i = 0; i < NUM_SENSORS; i++) {
        int len = sprintf(calib_msg, "Sensor %d: MIN=%u, MAX=%u\r\n", 
                      i + 1, sensor_min[i], sensor_max[i]);
        HAL_UART_Transmit(&huart2, (uint8_t*)calib_msg, len, 100);
    }
    */


/* USER CODE BEGIN 0 */
float get_line_error(void)
{
    float weighted_sum = 0;
    float sum = 0;
    
    // Trọng số vị trí: -2000 (Trái cùng) đến +2000 (Phải cùng)
    int weights[5] = {-2000, -1000, 0, 1000, 2000}; 
    
    // Đếm xem có bao nhiêu cảm biến nhìn thấy vạch đen
    int active_sensors = 0; 
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        // 1. Chuẩn hóa: (Giá trị - Min) / (Max - Min) * 1000
        // Kết quả: 0 (Trắng) -> 1000 (Đen)
        long numerator = (long)(adc_values[i] - sensor_min[i]) * 1000;
        long denominator = (long)(sensor_max[i] - sensor_min[i]);
        
        if (denominator == 0) denominator = 1; // Tránh chia cho 0
        
        long normalized = numerator / denominator;
        
        // Kẹp giá trị trong khoảng 0-1000
        if (normalized < 0) normalized = 0;
        if (normalized > 1000) normalized = 1000;
        
        // Cộng dồn trọng số
        // Chỉ tính nếu giá trị > 200 (tức là có thấy hơi đen đen)
        if (normalized > 200) {
            weighted_sum += (float)(normalized * weights[i]);
            sum += normalized;
            active_sensors++;
        }
    }
    
    // 2. Xử lý trường hợp mất line (Ra ngoài trắng hoàn toàn)
    if (sum == 0) {
        // Nếu lần cuối cùng robot đang lệch phải -> Giữ nguyên lỗi max phải
        if (last_error > 0) return 1500; 
        // Nếu lần cuối cùng robot đang lệch trái -> Giữ nguyên lỗi max trái
        else return -1500;
    }
    
    // 3. Trả về sai số trung bình trọng số
    return weighted_sum / sum;
}
// Hàm kiểm tra 3 sensor giữa (1,2,3) đều đang thấy đen
// Trả về 1 nếu là "vạch trạm", 0 nếu không
uint8_t Is_Middle_3_Black(void)
{
    int black_count = 0;

    // Dùng ngưỡng tương đối: > 80% giá trị max đã calib
    for (int i = 1; i <= 3; i++)   // sensor 1,2,3 (giữa)
    {
        if (adc_values[i] > (uint16_t)(sensor_max[i] * 0.8f)) {
            black_count++;
        }
    }

    // Nếu cả 3 mắt giữa đều "rất đen" => vạch ngang
    if (black_count >= 3) {
        return 1;
    }
    return 0;
}
// Phát hiện NGÃ RẼ: ít nhất 3 mắt đen và có 1 mắt ngoài (0 hoặc 4) đen
uint8_t Is_At_Branch(void)
{
    int black_count = 0;
    int outer_black = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        long numerator   = (long)(adc_values[i] - sensor_min[i]) * 1000;
        long denominator = (long)(sensor_max[i] - sensor_min[i]);
        if (denominator == 0) denominator = 1;

        long normalized = numerator / denominator;  // 0 = trắng, 1000 = đen

        if (normalized > 300) {   // ngưỡng “đen”; chỉnh sau khi test
            black_count++;
            if (i == 0 || i == 4) outer_black = 1;
        }
    }

    if (black_count >= 3 && outer_black) {
        return 1;   // đang ở vùng giao/nhánh
    }
    return 0;
}
// Hàm kiểm tra vạch KẾT THÚC: 4–5 sensor đều “rất đen”
// KẾT THÚC KHI 5 SENSOR THẤY TRẮNG
// Dựa trên chuẩn hoá như get_line_error():
//   0   ~ trắng, 
//   1000 ~ đen
uint8_t Is_Finish_Line(void)
{
    int white_count = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        // Chuẩn hoá giống trong get_line_error()
        long numerator   = (long)(adc_values[i] - sensor_min[i]) * 1000;
        long denominator = (long)(sensor_max[i] - sensor_min[i]);
        if (denominator == 0) denominator = 1;

        long normalized = numerator / denominator;  // 0 = trắng, 1000 = đen

        // Nếu rất TRẮNG -> normalized nhỏ
        // Ngưỡng 100 bạn có thể chỉnh lên/xuống tuỳ thực tế
        if (normalized < 100) {
            white_count++;
        }
    }

    // Nếu CẢ 5 sensor đều trắng -> coi là KẾT THÚC
    if (white_count >= 5) {
        return 1;
    }
    return 0;
}

// --- CHỌN FILTER MÀU CHO TCS3200 ---
// color = 0: RED, color = 1: BLUE (đủ dùng cho đề)
void TCS_Select_Filter(uint8_t color)
{
    switch (color)
    {
        case 0: // RED
            HAL_GPIO_WritePin(TCS_S2_PORT, TCS_S2_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(TCS_S3_PORT, TCS_S3_PIN, GPIO_PIN_RESET);
            break;

        case 1: // BLUE
            HAL_GPIO_WritePin(TCS_S2_PORT, TCS_S2_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(TCS_S3_PORT, TCS_S3_PIN, GPIO_PIN_SET);
            break;
    }
}

// --- ĐO TẦN SỐ XUNG TỪ CHÂN OUT TRONG 20ms ---
uint32_t TCS_Measure_Frequency(void)
{
    uint32_t pulse_count   = 0;
    uint32_t last_state    = 0;
    uint32_t current_state = 0;
    uint32_t start_time    = HAL_GetTick();

    while ((HAL_GetTick() - start_time) < 20)  // đo trong 20 ms
    {
        current_state = HAL_GPIO_ReadPin(TCS_OUT_PORT, TCS_OUT_PIN);

        // đếm cạnh lên 0 -> 1
        if (last_state == 0 && current_state == 1) {
            pulse_count++;
        }

        last_state = current_state;
    }

    return pulse_count;
}

// --- ĐỌC RED & BLUE, QUYẾT ĐỊNH MÀU ---
// return: 0 = UNKNOWN, 1 = RED, 2 = BLUE
int TCS_Identify_Color(void)
{
    // Đo RED
    TCS_Select_Filter(0);    // RED
    HAL_Delay(20);
    uint32_t red_value = TCS_Measure_Frequency();

    // Đo BLUE
    TCS_Select_Filter(1);    // BLUE
    HAL_Delay(20);
    uint32_t blue_value = TCS_Measure_Frequency();

    // So sánh – THAM SỐ 20 LÀ NGƯỠNG CHÊNH LỆCH, SAU NÀY BẠN CHỈNH THEO SỐ LIỆU THẬT
    if (blue_value > (red_value + 20)) return 2; // XANH
    if (red_value > (blue_value + 20)) return 1; // ĐỎ
    return 0;
}

// --- ĐỌC MÀU NHIỀU LẦN CHO CHẮC ĂN ---
int Read_Package_Color(void)
{
    int red_count  = 0;
    int blue_count = 0;

    for (int i = 0; i < 10; i++)
    {
        int c = TCS_Identify_Color();   // 0 / 1 / 2

        if (c == 1) red_count++;
        if (c == 2) blue_count++;

        HAL_Delay(30);   // nghỉ tí giữa các lần đo
    }

    if (red_count > blue_count && red_count >= 3) {
        return 1;   // ĐỎ
    } else if (blue_count > red_count && blue_count >= 3) {
        return 2;   // XANH
    } else {
        return 0;   // không chắc màu
    }
}

// --- CHO ROBOT XOAY CIRCLE VÒNG TẠI CHỖ ---
void Robot_Spin_Circles(uint8_t circles)
{
    float spin_pwm = 40.0f;          // tốc độ quay (bạn chỉnh sau)
    uint32_t time_per_circle = 2500; // ms / 1 vòng – TEST rồi chỉnh lại

    // Dùng đúng logic chiều QUAY TRONG calib_auto_run (quay tại chỗ)
    // MOTOR TRÁI: TIẾN
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    motor1_set_pwm(spin_pwm);

    // MOTOR PHẢI: LÙI
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    motor2_set_pwm(spin_pwm);

    uint32_t start    = HAL_GetTick();
    uint32_t duration = (uint32_t)circles * time_per_circle;

    while (HAL_GetTick() - start < duration)
    {
        HAL_Delay(1);  // cho CPU thở
    }

    // DỪNG LẠI
    motor1_set_pwm(0);
    motor2_set_pwm(0);
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
 // 1. Khởi động DMA/PWM/Encoder
		HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, NUM_SENSORS); 
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); 
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    calib_auto_run(25.0f); // Bắt đầu quay calib
// Cấu hình TCS3200: Scaling 20%
HAL_GPIO_WritePin(TCS_S0_PORT, TCS_S0_PIN, GPIO_PIN_SET);
HAL_GPIO_WritePin(TCS_S1_PORT, TCS_S1_PIN, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	/* USER CODE BEGIN WHILE */
  
  // --- CẤU HÌNH PID ---
  Kp = 0.05f;   // Tăng Kp lên chút (0.03 -> 0.05) để phản ứng mạnh hơn
  Ki = 0.00f;   
  Kd = 0.01f;   // Tăng Kd để giảm rung lắc
  
  base_speed = 35.0f; // Tăng tốc độ nền lên 35% (Chậm quá xe sẽ yếu)
  max_speed = 70.0f;  // Cho phép max speed cao hơn
  
   while (1)
{
    // 1. Tính sai số "thô" từ line
    float raw_error = get_line_error();

    // ================== MÁY TRẠNG THÁI ==================
    switch (robot_state)
    {
        // --- GIAI ĐOẠN 1: CHẠY TỚI TRẠM TẢI HÀNG ---
        case STATE_GO_TO_STATION:
        {
            // Bám line bình thường
            error = raw_error;

            // Nếu 3 sensor giữa đều đen -> tới trạm
            if (Is_Middle_3_Black())
            {
                // Dừng xe
                motor1_set_pwm(0);
                motor2_set_pwm(0);

                // Cho 1–2 giây để đặt hàng lên
                HAL_Delay(10000);

                // Chuyển sang trạng thái đọc màu
                robot_state = STATE_READ_COLOR;
            }
            break;
        }

        // --- GIAI ĐOẠN 2: ĐỌC MÀU ---
        case STATE_READ_COLOR:
{
    // ĐẢM BẢO DỪNG XE
    motor1_set_pwm(0);
    motor2_set_pwm(0);

    // Đọc màu gói hàng nhiều lần cho chắc
    target_color = Read_Package_Color();
    // target_color: 1 = ĐỎ, 2 = XANH, 0 = không chắc

    // Sau khi đọc xong -> chuẩn bị ESCAPE trạm
    escape_start_ms = HAL_GetTick();
    robot_state     = STATE_ESCAPE_STATION;

    // QUAN TRỌNG: KHÔNG CHO PID CHẠY Ở VÒNG LẶP NÀY
    HAL_Delay(10);       // nghỉ nhẹ
    continue;            // nhảy lên đầu while(1), bỏ qua đoạn PID bên dưới
}


        // --- GIAI ĐOẠN 3: CHẠY THẲNG RA KHỎI TRẠM ---
        case STATE_ESCAPE_STATION:
        {
            // Chạy thẳng "mù": không dùng PID, chỉ đặt PWM cố định

            float escape_pwm = 30.0f;  // tốc độ bạn chỉnh tuỳ

            // CHIỀU TIẾN: dùng đúng chiều bạn đang dùng khi chạy thẳng
            // MOTOR TRÁI TIẾN
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); 
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
            motor1_set_pwm(escape_pwm);

            // MOTOR PHẢI TIẾN
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); 
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            motor2_set_pwm(escape_pwm);

            // Sau ~800 ms thì coi như đã ra khỏi vùng trạm
           if (HAL_GetTick() - escape_start_ms > 800)
{
    motor1_set_pwm(0);
    motor2_set_pwm(0);

    go_to_finish_start_ms = HAL_GetTick();  // bắt đầu phase về đích
    branch_detected = 0;                    // chưa gặp ngã rẽ
    robot_state = STATE_GO_TO_FINISH;
}



            // Ở state này KHÔNG chạy PID bên dưới
            HAL_Delay(2);
            continue;
        }

        case STATE_GO_TO_FINISH:
{
    uint32_t elapsed = HAL_GetTick() - go_to_finish_start_ms;
    float bias = 800.0f;
    // ❶ Giai đoạn đầu: vừa rời trạm -> bám line bình thường 1 thời gian
    if (elapsed < 800)   // 0.8s đầu, chỉnh 800–1200 tùy đường
    {
        error = raw_error;
    }
    else
    {
        // ❷ Sau 0.8s, bắt đầu tìm ngã rẽ
        if (!branch_detected) {
    error = raw_error;
    if (Is_At_Branch()) branch_detected = 1;
}
else {
    // ĐANG Ở NGÃ RẼ -> áp bias theo màu
    if (target_color == 1) {
        // ĐỎ -> TRÁI
        error = raw_error - bias;
    } else if (target_color == 2) {
        // XANH -> PHẢI
        error = raw_error + bias;
    } else {
        error = raw_error;
    }
}

    }

    // Kết thúc khi 5 sensor trắng
    if (Is_Finish_Line())
    {
        motor1_set_pwm(0);
        motor2_set_pwm(0);
        robot_state = STATE_FINISHED;
    }
    break;
}

        // --- GIAI ĐOẠN 5: DỪNG HẲN ---
        case STATE_FINISHED:
        default:
        {
            motor1_set_pwm(0);
            motor2_set_pwm(0);

            // Nếu muốn: nháy LED báo kết thúc
            // while(1) { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(200); }

            while (1) { }   // đứng im luôn
        }
    }
    // ================== HẾT STATE MACHINE ==================

    // 2. TÍNH PID & ĐIỀU KHIỂN MOTOR
    // (chỉ có ý nghĩa khi đang ở GO_TO_STATION hoặc GO_TO_FINISH,
    // nhưng mình cứ để chung, vì các state khác đã dùng continue ở trên)

    float P = error;
    float I = total_error + error; 
    float D = error - last_error;
    last_error = error; 
    
    float pid_value = (Kp * P) + (Ki * I) + (Kd * D);

    float left_motor_speed  = base_speed + pid_value;   
    float right_motor_speed = base_speed - pid_value;
    
    // 3. Giới hạn tốc độ (Clamp)
    if (left_motor_speed > max_speed)  left_motor_speed  = max_speed;
    if (left_motor_speed < -30)        left_motor_speed  = -30; 
    if (right_motor_speed > max_speed) right_motor_speed = max_speed;
    if (right_motor_speed < -30)       right_motor_speed = -30;
    
    // 4. Xuất PWM & chiều y như code gốc của bạn
    // --- MOTOR 1 (TRÁI) ---
    if (left_motor_speed >= 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        motor1_set_pwm(left_motor_speed);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        motor1_set_pwm(-left_motor_speed);
    }

    // --- MOTOR 2 (PHẢI) ---
    if (right_motor_speed >= 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        motor2_set_pwm(right_motor_speed);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        motor2_set_pwm(-right_motor_speed);
    }

    HAL_Delay(2); 
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
