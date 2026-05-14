#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "Fusion.h"

// =====================================================
// CONFIG
// =====================================================

#define MPU_ADDRESS 0x68

#define I2C_PORT i2c1
#define SDA_PIN 2
#define SCL_PIN 3

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define SAMPLE_PERIOD 0.02f

// RGB
#define LED_R_PIN 13
#define LED_G_PIN 14
#define LED_B_PIN 15

// =====================================================
// PINOS DE INSTRUMENTALIZAÇÃO
// =====================================================

#define PIN_MPU_TASK    16
#define PIN_FUSION_TASK 17
#define PIN_UART_TASK   18
#define PIN_PWM_TASK    19

// =====================================================
// SMP - AFINIDADE DE CORE
// =====================================================

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

// CLICK
#define JAB_PEAK_THRESHOLD   12000
#define JAB_RETURN_THRESHOLD  4000
#define JAB_LATERAL_MAX      10000
#define JAB_TIMEOUT_MS         350

// =====================================================
// FILAS / SEMÁFOROS
// =====================================================

QueueHandle_t xQueueMPU;
QueueHandle_t xQueuePos;
QueueHandle_t xQueueColor;
SemaphoreHandle_t xSemaphoreBtn;

// =====================================================
// STRUCTS
// =====================================================

typedef struct {
    int16_t acc[3];
    int16_t gyro[3];
} MPUData;

typedef struct {
    int16_t x;
    int16_t y;
} MousePos;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGBColor;

typedef enum { JAB_IDLE, JAB_PEAKED } JabState;

// =====================================================
// INSTRUMENTALIZAÇÃO - helper
// =====================================================

static inline void probe_init(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

// =====================================================
// MPU6050
// =====================================================

void mpu_init() {
    i2c_init(I2C_PORT, 400000);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    uint8_t buf[2] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, buf, 2, false);
}

void mpu_read(int16_t acc[3], int16_t gyro[3]) {
    uint8_t reg = 0x3B;
    uint8_t buffer[14];

    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 14, false);

    for (int i = 0; i < 3; i++) {
        acc[i]  = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
        gyro[i] = (buffer[8 + i * 2] << 8) | buffer[9 + i * 2];
    }
}

// =====================================================
// PWM RGB
// =====================================================

void pwm_rgb_init() {
    gpio_set_function(LED_R_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LED_G_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LED_B_PIN, GPIO_FUNC_PWM);

    uint slice_r = pwm_gpio_to_slice_num(LED_R_PIN);
    uint slice_g = pwm_gpio_to_slice_num(LED_G_PIN);
    uint slice_b = pwm_gpio_to_slice_num(LED_B_PIN);

    pwm_set_wrap(slice_r, 255);
    pwm_set_wrap(slice_g, 255);
    pwm_set_wrap(slice_b, 255);

    pwm_set_enabled(slice_r, true);
    pwm_set_enabled(slice_g, true);
    pwm_set_enabled(slice_b, true);
}

void set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    pwm_set_gpio_level(LED_R_PIN, r);
    pwm_set_gpio_level(LED_G_PIN, g);
    pwm_set_gpio_level(LED_B_PIN, b);
}

// =====================================================
// TASK STACK MONITOR
// ATENÇÃO: desative os probes GPIO antes de usar esta
// task, pois o printf afeta o tempo de execução.
// =====================================================

void stack_monitor_task(void* p) {
    static TaskStatus_t tasks[16];
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        UBaseType_t n = uxTaskGetSystemState(tasks, 16, NULL);
        printf("+------------------+-------+\n");
        printf("| %-16s | %5s |\n", "task", "free");
        printf("+------------------+-------+\n");
        for (UBaseType_t i = 0; i < n; i++) {
            printf("| %-16s | %5u |\n",
                   tasks[i].pcTaskName,
                   (unsigned)tasks[i].usStackHighWaterMark);
        }
        printf("+------------------+-------+\n");
        printf("| heap livre min   | %5u |\n",
               (unsigned)xPortGetMinimumEverFreeHeapSize());
        printf("+------------------+-------+\n\n");
    }
}

// =====================================================
// TASK MPU  →  GP16  |  Core 0
// =====================================================

void mpu_task(void *p) {
    MPUData data;

    probe_init(PIN_MPU_TASK);
    mpu_init();
    printf("MPU OK\n");

    while (1) {
        gpio_put(PIN_MPU_TASK, 1);

        mpu_read(data.acc, data.gyro);
        xQueueSend(xQueueMPU, &data, 0);

        gpio_put(PIN_MPU_TASK, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// =====================================================
// TASK FUSION  →  GP17  |  Core 0
// =====================================================

void fusion_task(void *p) {

    MPUData data;
    MousePos pos;
    RGBColor cor;

    probe_init(PIN_FUSION_TASK);

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    float roll_filtered  = 0;
    float pitch_filtered = 0;
    float roll_offset    = 0;
    float pitch_offset   = 0;
    float r_f = 0, g_f = 0, b_f = 0;

    const float alpha = 0.2f;

    int16_t prev_acc_y = 0;
    bool first_reading = true;

    JabState jab_state       = JAB_IDLE;
    TickType_t jab_peak_time = 0;

    // =========================
    // CALIBRAÇÃO
    // =========================

    printf("Calibrando... mantenha parado\n");

    float roll_sum  = 0;
    float pitch_sum = 0;
    int   samples   = 0;

    for (int i = 0; i < 200; i++) {
        if (xQueueReceive(xQueueMPU, &data, pdMS_TO_TICKS(100))) {

            FusionVector gyro_v = {
                .axis.x = data.gyro[0] / 131.0f,
                .axis.y = data.gyro[1] / 131.0f,
                .axis.z = data.gyro[2] / 131.0f
            };
            FusionVector acc_v = {
                .axis.x = data.acc[0] / 16384.0f,
                .axis.y = data.acc[1] / 16384.0f,
                .axis.z = data.acc[2] / 16384.0f
            };

            FusionAhrsUpdateNoMagnetometer(&ahrs, gyro_v, acc_v, SAMPLE_PERIOD);

            if (i >= 150) {
                FusionEuler euler =
                    FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
                roll_sum  += euler.angle.roll;
                pitch_sum += euler.angle.pitch;
                samples++;
            }
        }
    }

    if (samples > 0) {
        roll_offset  = roll_sum  / samples;
        pitch_offset = pitch_sum / samples;
    }

    printf("Roll offset: %.2f\n",  roll_offset);
    printf("Pitch offset: %.2f\n", pitch_offset);

    // =========================
    // LOOP PRINCIPAL
    // =========================

    while (1) {

        if (xQueueReceive(xQueueMPU, &data, portMAX_DELAY)) {

            gpio_put(PIN_FUSION_TASK, 1);

            FusionVector gyro_v = {
                .axis.x = data.gyro[0] / 131.0f,
                .axis.y = data.gyro[1] / 131.0f,
                .axis.z = data.gyro[2] / 131.0f
            };
            FusionVector acc_v = {
                .axis.x = data.acc[0] / 16384.0f,
                .axis.y = data.acc[1] / 16384.0f,
                .axis.z = data.acc[2] / 16384.0f
            };

            FusionAhrsUpdateNoMagnetometer(&ahrs, gyro_v, acc_v, SAMPLE_PERIOD);

            FusionEuler euler =
                FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

            float roll  = euler.angle.roll  - roll_offset;
            float pitch = euler.angle.pitch - pitch_offset;

            roll_filtered  = alpha * roll  + (1.0f - alpha) * roll_filtered;
            pitch_filtered = alpha * pitch + (1.0f - alpha) * pitch_filtered;

            if (fabs(roll_filtered)  < 3) roll_filtered  = 0;
            if (fabs(pitch_filtered) < 3) pitch_filtered = 0;

            // MOUSE
            pos.x = (int16_t)(roll_filtered  *  5);
            pos.y = (int16_t)(-pitch_filtered *  5);

            if (pos.x >  200) pos.x =  200;
            if (pos.x < -200) pos.x = -200;
            if (pos.y >  200) pos.y =  200;
            if (pos.y < -200) pos.y = -200;

            xQueueSend(xQueuePos, &pos, 0);

            // CLICK
            if (!first_reading) {

                int16_t fwd     = data.acc[0];
                int16_t lateral = (int16_t)abs(data.acc[1] - prev_acc_y);

                if (jab_state == JAB_IDLE) {
                    if (fwd > JAB_PEAK_THRESHOLD && lateral < JAB_LATERAL_MAX) {
                        jab_state     = JAB_PEAKED;
                        jab_peak_time = xTaskGetTickCount();
                    }
                }
                else if (jab_state == JAB_PEAKED) {
                    TickType_t elapsed = (xTaskGetTickCount() - jab_peak_time)
                                        * portTICK_PERIOD_MS;

                    if (elapsed > JAB_TIMEOUT_MS || lateral >= JAB_LATERAL_MAX) {
                        jab_state = JAB_IDLE;
                    }
                    else if (abs(fwd) < JAB_RETURN_THRESHOLD) {
                        jab_state = JAB_IDLE;
                        xSemaphoreGive(xSemaphoreBtn);
                    }
                }

            } else {
                first_reading = false;
            }

            prev_acc_y = data.acc[1];

            // RGB
            float r_target = 0, g_target = 0, b_target = 0;

            if (roll_filtered > 0)
                r_target = roll_filtered * 8;
            else if (roll_filtered < 0)
                b_target = -roll_filtered * 8;

            if (pitch_filtered > 0)
                g_target = pitch_filtered * 8;
            else if (pitch_filtered < 0)
                g_target = -pitch_filtered * 5;

            if (r_target > 255) r_target = 255;
            if (g_target > 255) g_target = 255;
            if (b_target > 255) b_target = 255;

            const float smooth = 0.25f;
            r_f = smooth * r_target + (1.0f - smooth) * r_f;
            g_f = smooth * g_target + (1.0f - smooth) * g_f;
            b_f = smooth * b_target + (1.0f - smooth) * b_f;

            cor.r = (uint8_t)r_f;
            cor.g = (uint8_t)g_f;
            cor.b = (uint8_t)b_f;

            xQueueOverwrite(xQueueColor, &cor);

            gpio_put(PIN_FUSION_TASK, 0);
        }
    }
}

// =====================================================
// TASK UART  →  GP18  |  Core 1
// =====================================================

void uart_task(void *p) {

    MousePos pos;

    probe_init(PIN_UART_TASK);

    while (1) {

        gpio_put(PIN_UART_TASK, 1);

        if (xQueueReceive(xQueuePos, &pos, 0)) {

            uint8_t pacote[4];

            pacote[0] = 0x00;
            pacote[1] = pos.x & 0xFF;
            pacote[2] = (pos.x >> 8) & 0xFF;
            pacote[3] = 0xFF;
            uart_write_blocking(UART_ID, pacote, 4);

            vTaskDelay(pdMS_TO_TICKS(1));

            pacote[0] = 0x01;
            pacote[1] = pos.y & 0xFF;
            pacote[2] = (pos.y >> 8) & 0xFF;
            pacote[3] = 0xFF;
            uart_write_blocking(UART_ID, pacote, 4);
        }

        if (xSemaphoreTake(xSemaphoreBtn, 0)) {
            uint8_t click[2] = {0xFE, 0x01};
            uart_write_blocking(UART_ID, click, 2);
        }

        gpio_put(PIN_UART_TASK, 0);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// =====================================================
// TASK PWM RGB  →  GP19  |  Core 1
// =====================================================

void pwm_task(void *p) {

    RGBColor cor;

    probe_init(PIN_PWM_TASK);
    pwm_rgb_init();

    while (1) {
        if (xQueueReceive(xQueueColor, &cor, portMAX_DELAY)) {

            gpio_put(PIN_PWM_TASK, 1);

            set_rgb(cor.r, cor.g, cor.b);

            gpio_put(PIN_PWM_TASK, 0);
        }
    }
}

// =====================================================
// MAIN
// =====================================================

int main() {

    stdio_init_all();
    sleep_ms(3000);

    uart_init(UART_ID, BAUD_RATE);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    xQueueMPU   = xQueueCreate(10, sizeof(MPUData));
    xQueuePos   = xQueueCreate(10, sizeof(MousePos));
    xQueueColor = xQueueCreate(1,  sizeof(RGBColor));

    xSemaphoreBtn = xSemaphoreCreateBinary();

    TaskHandle_t h_mpu     = NULL;
    TaskHandle_t h_fusion  = NULL;
    TaskHandle_t h_uart    = NULL;
    TaskHandle_t h_pwm     = NULL;
    TaskHandle_t h_monitor = NULL;

    xTaskCreate(mpu_task,           "MPU",     2048, NULL, 2, &h_mpu);
    xTaskCreate(fusion_task,        "Fusion",  4096, NULL, 2, &h_fusion);
    xTaskCreate(uart_task,          "UART",    2048, NULL, 1, &h_uart);
    xTaskCreate(pwm_task,           "PWM",     1024, NULL, 1, &h_pwm);
    xTaskCreate(stack_monitor_task, "Monitor", 2048, NULL, 1, &h_monitor);

    // Monitor fica no Core 1 junto com UART e PWM
    // para não interferir no pipeline do Core 0
    vTaskCoreAffinitySet(h_mpu,     CORE_0);
    vTaskCoreAffinitySet(h_fusion,  CORE_0);
    vTaskCoreAffinitySet(h_uart,    CORE_1);
    vTaskCoreAffinitySet(h_pwm,     CORE_1);
    vTaskCoreAffinitySet(h_monitor, CORE_1);

    vTaskStartScheduler();

    while (1);
}