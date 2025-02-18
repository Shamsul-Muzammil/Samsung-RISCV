#include "debug.h"
#include <ch32v00x.h>

// Ultrasonic sensor pins
#define TRIGGER_PIN GPIO_Pin_4
#define ECHO_PIN GPIO_Pin_3

// Servo motor PWM pin
#define SERVO_PWM_PIN GPIO_Pin_2

// Threshold distance for object detection (in cm)
#define OBSTACLE_THRESHOLD 20

// Servo motor PWM parameters
#define SERVO_PWM_PERIOD 20000 // 20 ms period (50 Hz frequency)
#define SERVO_PWM_PULSE_MIN 1000 // 1 ms pulse (0 degrees) - door closed
#define SERVO_PWM_PULSE_MAX 2000 // 2 ms pulse (180 degrees) - door open

// Function to configure GPIO pins for ultrasonic sensor
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    // Pin PD3: Input for Ultrasonic sensor echo
    GPIO_InitStructure.GPIO_Pin = ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Input with Pull-Up
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Pin PD4: Output for Ultrasonic sensor trigger
    GPIO_InitStructure.GPIO_Pin = TRIGGER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // Output Push-Pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

// Function to initialize PWM for the servo motor
void TIM1_PWMOut_Init(uint16_t pulseWidth)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

    // Enable GPIO clock for PD2 (servo PWM pin)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = SERVO_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push-Pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Enable Timer 1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // Configure Timer 1 for PWM
    TIM_TimeBaseInitStructure.TIM_Period = SERVO_PWM_PERIOD - 1; // 20 ms period
    TIM_TimeBaseInitStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1; // 1 MHz clock
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    // Configure PWM mode
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pulseWidth; // Set pulse width
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    // Enable PWM outputs
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

// Function to set servo motor angle
void Set_Servo_Angle(uint16_t angle)
{
    // Map angle (0-180 degrees) to pulse width (1000-2000 us)
    uint16_t pulseWidth = SERVO_PWM_PULSE_MIN + (angle * (SERVO_PWM_PULSE_MAX - SERVO_PWM_PULSE_MIN) / 180);
    TIM1_PWMOut_Init(pulseWidth);
    Delay_Ms(500); // Wait for servo to reach the desired position
}

// Function to trigger the ultrasonic sensor and read the echo duration
uint32_t Ultrasonic_Read(void)
{
    uint32_t echoTime = 0;

    GPIO_WriteBit(GPIOD, TRIGGER_PIN, SET); // Setting Trigger Pin to send pulses
    Delay_Us(10); // Pulse Width
    GPIO_WriteBit(GPIOD, TRIGGER_PIN, RESET); // Resetting Trigger Pin

    while (GPIO_ReadInputDataBit(GPIOD, ECHO_PIN) == Bit_RESET); // Wait for Echo to go high
    while (GPIO_ReadInputDataBit(GPIOD, ECHO_PIN) == Bit_SET) echoTime++; // Measure the time Echo is high

    return echoTime;
}

// Function to calculate distance from echo time
float Calculate_Distance(uint32_t echoTime)
{
    // Speed of sound in air is 340 m/s or 0.034 cm/us
    // Distance is (time / 2) * speed_of_sound
    return (echoTime / 2.0) * 0.034;
}

// Main function for smart door system
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    GPIO_Config(); // Initialize ultrasonic sensor pins
    USART_Printf_Init(115200); // Initialize debug USART

    // Initially close the door (servo at 0 degrees)
    Set_Servo_Angle(0);
    Delay_Ms(1000); // Wait for servo to stabilize

    while (1)
    {
        // Check forward path distance
        uint32_t echoTime = Ultrasonic_Read();
        float distance = Calculate_Distance(echoTime);
        printf("Distance: %.2f cm\n", distance); // Print the distance

        if (distance <= OBSTACLE_THRESHOLD)
        {
            // Object detected, open the door (servo to 90 degrees)
            Set_Servo_Angle(0);
            printf("Door Opened\n");

            // Wait for a short delay before closing the door (3 seconds)
            Delay_Ms(7000);

            // Close the door after the delay (servo back to 0 degrees)
            Set_Servo_Angle(120);
            printf("Door Closed\n");
        }

        Delay_Ms(100); // Wait for a short period before checking again
    }
}