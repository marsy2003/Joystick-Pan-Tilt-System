#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

volatile bool gCheckADC = false;
volatile uint16_t sX = 0;
volatile uint16_t sY = 0;

// Servo limits
#define PAN_POS_MIN     1024
#define PAN_POS_MAX     3072
#define PAN_POS_CENTER  2048

#define TILT_POS_MIN    2100
#define TILT_POS_MAX    3000
#define TILT_POS_CENTER 2550

#define PAN_SERVO_ID    0
#define TILT_SERVO_ID   2

#define STS_INST_WRITE          0x03
#define STS_ADDR_GOAL_POSITION  0x2A

// Joystick control
static int32_t xCenter = 2048;
static int32_t yCenter = 2048;

#define DEADZONE           150   // Raw ADC counts around center to ignore
#define TICK_MS            20
#define MAX_STEP_PER_TICK  15

#define INVERT_X           0
#define INVERT_Y           1

static int32_t panPos  = PAN_POS_CENTER;
static int32_t tiltPos = TILT_POS_CENTER;

// UART packets
uint8_t stsChecksum(uint8_t *packet, uint8_t length)
{
    uint16_t sum = 0;

    for (uint8_t i = 2; i < length - 1; i++) {
        sum += packet[i];
    }

    return (uint8_t)(~sum);
}

void uartSendByte(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART1)) {
        // Wait until UART TX FIFO has space
    }

    DL_UART_Main_transmitData(UART1, data);
}

void uartSendPacket(uint8_t *packet, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++) {
        uartSendByte(packet[i]);
    }
}

void stsWritePosition(uint8_t id, uint16_t position, uint16_t time, uint16_t speed)
{
    uint8_t packet[13];

    packet[0]  = 0xFF;
    packet[1]  = 0xFF;
    packet[2]  = id;
    packet[3]  = 0x09;
    packet[4]  = STS_INST_WRITE;
    packet[5]  = STS_ADDR_GOAL_POSITION;
    packet[6]  = position & 0xFF;
    packet[7]  = (position >> 8) & 0xFF;
    packet[8]  = time & 0xFF;
    packet[9]  = (time >> 8) & 0xFF;
    packet[10] = speed & 0xFF;
    packet[11] = (speed >> 8) & 0xFF;
    packet[12] = stsChecksum(packet, 13);

    uartSendPacket(packet, 13);
}

static int32_t clampPan(int32_t pos)
{
    if (pos < PAN_POS_MIN) {
        return PAN_POS_MIN;
    }

    if (pos > PAN_POS_MAX) {
        return PAN_POS_MAX;
    }

    return pos;
}

static int32_t clampTilt(int32_t pos)
{
    if (pos < TILT_POS_MIN) {
        return TILT_POS_MIN;
    }

    if (pos > TILT_POS_MAX) {
        return TILT_POS_MAX;
    }

    return pos;
}

void servoMovePan(int32_t pos)
{
    pos = clampPan(pos);
    panPos = pos;

    stsWritePosition(PAN_SERVO_ID, (uint16_t)pos, TICK_MS, 0);
}

void servoMoveTilt(int32_t pos)
{
    pos = clampTilt(pos);
    tiltPos = pos;

    stsWritePosition(TILT_SERVO_ID, (uint16_t)pos, TICK_MS, 0);
}

void servoMovePanSlow(int32_t pos)
{
    pos = clampPan(pos);
    panPos = pos;

    stsWritePosition(PAN_SERVO_ID, (uint16_t)pos, 2000, 0);
}

void servoMoveTiltSlow(int32_t pos)
{
    pos = clampTilt(pos);
    tiltPos = pos;

    stsWritePosition(TILT_SERVO_ID, (uint16_t)pos, 2000, 0);
}

// ADC
void readJoystick(void)
{
    gCheckADC = false;

    DL_ADC12_startConversion(ADC12_0_INST);

    while (gCheckADC == false) {
        __WFE();
    }

    sX = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
    sY = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);

    DL_ADC12_disableConversions(ADC12_0_INST);
    DL_ADC12_enableConversions(ADC12_0_INST);
}

static int32_t deflectionToStep(int32_t reading, int32_t center)
{
    int32_t deflection = reading - center;

    if (deflection > DEADZONE) {
        deflection -= DEADZONE;
    } else if (deflection < -DEADZONE) {
        deflection += DEADZONE;
    } else {
        return 0;
    }

    int32_t step = deflection / (1850 / MAX_STEP_PER_TICK);

    if (step > MAX_STEP_PER_TICK) {
        step = MAX_STEP_PER_TICK;
    }

    if (step < -MAX_STEP_PER_TICK) {
        step = -MAX_STEP_PER_TICK;
    }

    return step;
}

int main(void)
{
    SYSCFG_DL_init();
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    delay_cycles(3200000);  // 100 ms boot delay

    servoMovePanSlow(PAN_POS_CENTER);
    servoMoveTiltSlow(TILT_POS_CENTER);

    delay_cycles(64000000);  // 2 seconds to actually reach center

    // Calibrate joystick
    {
        int32_t sumX = 0;
        int32_t sumY = 0;
        const int N = 16;

        for (int i = 0; i < N; i++) {
            readJoystick();

            sumX += sX;
            sumY += sY;

            delay_cycles(320000);  // 10 ms between samples
        }

        xCenter = sumX / N;
        yCenter = sumY / N;
    }

    printf("\r\n=== Joystick Control ===\r\n");
    printf("Calibrated: xCenter=%ld, yCenter=%ld\r\n",
           (long)xCenter,
           (long)yCenter);
    printf("Push stick to move arm. Press button to recenter.\r\n");

    uint32_t tick = 0;
    bool buttonLatch = false;  // Edge detection for button press

    while (1) {
        readJoystick();

        int32_t xStep = deflectionToStep((int32_t)sX, xCenter);
        int32_t yStep = deflectionToStep((int32_t)sY, yCenter);

        if (INVERT_X) {
            xStep = -xStep;
        }

        if (INVERT_Y) {
            yStep = -yStep;
        }

        int32_t newPan  = panPos + xStep;
        int32_t newTilt = tiltPos + yStep;

        if (newPan != panPos) {
            servoMovePan(newPan);
        }

        if (newTilt != tiltPos) {
            servoMoveTilt(newTilt);
        }

        if ((tick % 25) == 0) {
            printf("X=%4u Y=%4u | dx=%+4ld dy=%+4ld | pan=%4ld tilt=%4ld\r\n",
                   sX,
                   sY,
                   (long)xStep,
                   (long)yStep,
                   (long)panPos,
                   (long)tiltPos);
        }

        tick++;

        delay_cycles(32000000UL * TICK_MS / 1000UL);
    }
}

void ADC12_0_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:  // Last MEM in sequence
            gCheckADC = true;
            break;

        default:
            break;
    }
}
