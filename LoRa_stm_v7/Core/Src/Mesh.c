/*
 * Mesh.c
 *
 *  Created on: Jan 24, 2026
 *      Author: 22142
 */
#include "Mesh.h"
#include "usart.h"
#include "gpio.h"
#include "LoRa.h"
#include "stdio.h"

uint32_t lastActivityTime = 0;
uint32_t watchdogTimeout = 30000;
uint8_t radioErrorCount = 0;
uint32_t lastResetTime = 0;
uint32_t lcg_seed = 0;

float dp = 1;
float t_in = 2;
float t_out = 3;
float p_header = 4;
float pm = 5;
uint8_t cleaning = 0;

#define DUP_CACHE_SIZE        16
typedef struct {
  uint8_t src;
  uint16_t msg_id;
} SeenPacket;
SeenPacket seenPackets[DUP_CACHE_SIZE];
uint8_t seenIndex = 0;

void Mesh_reset_watchdog(void) {
    lastActivityTime = HAL_GetTick();
}

void Mesh_check_watchdog(void) {
    if (HAL_GetTick() - lastActivityTime > watchdogTimeout) {
        for (int i = 0; i < 5; i++) {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
            HAL_Delay(100);
        }
        NVIC_SystemReset();
    }
}

void Mesh_hard_reset_radio(void) {
    // Hard reset the radio
    HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    // Ensure NSS is high
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    radioErrorCount = 0;
    lastResetTime = HAL_GetTick();
}

uint8_t Mesh_isDuplicate(uint8_t src, uint16_t msg_id)
{
  for (uint8_t i = 0; i < DUP_CACHE_SIZE; i++)
  {
    if (seenPackets[i].src == src &&
        seenPackets[i].msg_id == msg_id)
    {
      return 1;
    }
  }
  return 0;
}

void Mesh_rememberPacket(uint8_t src, uint16_t msg_id)
{
  seenPackets[seenIndex].src = src;
  seenPackets[seenIndex].msg_id = msg_id;

  seenIndex++;
  if (seenIndex >= DUP_CACHE_SIZE)
    seenIndex = 0;
}

uint32_t Mesh_lcg_rand(void)
{
    lcg_seed = (1103515245 * lcg_seed + 12345);
    return (lcg_seed >> 16) & 0x7FFF;
}

float Mesh_rand_range(float min, float max)
{
    return min + ((float)Mesh_lcg_rand() / 32767.0f) * (max - min);
}

uint8_t Mesh_check_radio_status(LoRa *radio)
{
    uint8_t version = LoRa_read(radio, RegVersion);
    if (version != 0x12) {
        return 0;
    }

    uint8_t op_mode = LoRa_read(radio, RegOpMode);
    uint8_t mode = op_mode & 0x07;

    if (mode != 0x01 && mode != 0x03 && mode != 0x05) {
        return 0;
    }
    return 1;
}

void Mesh_check_radio_health(LoRa *radio) {
    static uint32_t lastCheck = 0;

    if (HAL_GetTick() - lastCheck > 5000) {
        uint8_t version = LoRa_read(radio, RegVersion);
        if (version != 0x12) {
            // Only reset if we've had multiple errors
            if (radioErrorCount++ > 3) {
                Mesh_hard_reset_radio();
                HAL_Delay(100);
                LoRa_init(radio);
                LoRa_setSyncWord(radio, 0x34);
                LoRa_setSpreadingFactor(radio, 7);
                LoRa_enableCRC(radio, 1);
                LoRa_startReceiving(radio);
            }
        } else {
            radioErrorCount = 0; // Reset error count on success
        }
        lastCheck = HAL_GetTick();
    }
}

uint8_t Mesh_wait_for_tx_complete(LoRa *radio) {
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 1000) { // 1 second timeout
        uint8_t irq_flags = LoRa_read(radio, RegIrqFlags);
        if (irq_flags & 0x08) { // TxDone flag
            LoRa_write(radio, RegIrqFlags, 0xFF); // Clear all IRQ flags
            return 1;
        }
        HAL_Delay(1);
    }
    return 0;
}

/**************************************************************************************************************/

										//  LoRa ***IMP*** Functions //

/**************************************************************************************************************/

void Mesh_handle_req_data(
    LoRa *radio,
    uint8_t src_id,
    uint16_t msg_id,
    uint8_t self_id,
    char *payloadBuf,
    size_t payloadBufLen,
    uint8_t *txBuf,
    size_t txBufLen
)
{
    float dp       = Mesh_rand_range(1200.0f, 1300.0f);
    float t_in     = Mesh_rand_range(10.0f, 20.0f);
    float t_out    = Mesh_rand_range(170.0f, 190.0f);
    float p_header = Mesh_rand_range(55.0f, 65.0f);
    float pm       = Mesh_rand_range(10.0f, 15.0f);
    uint8_t cleaning = (Mesh_lcg_rand() % 2) ? 1 : 0;

    snprintf(payloadBuf, payloadBufLen,
             "%.1f,%.1f,%.1f,%.1f,%.1f,%d",
             dp, t_in, t_out, p_header, pm, cleaning);

    Mesh_transmit(
        radio,
        PKT_SENSOR_DATA,
        src_id,
        self_id,
        payloadBuf,
        msg_id,
        txBuf,
        txBufLen
    );

}

uint8_t Mesh_request_registration(
    LoRa *radio,
    uint32_t uid[3],
    int *nodeId,
    char *payloadBuf,
    size_t payloadBufLen,
    uint8_t *txBuf,
    size_t txBufLen,
    uint8_t *rxBuf,
    size_t rxBufLen
)
{
    snprintf(payloadBuf, payloadBufLen,
             "%08lX-%08lX-%08lX",
             uid[0], uid[1], uid[2]);

    LoRa_gotoMode(radio, STNBY_MODE);
    HAL_Delay(2);

    if (LoRa_performCAD(radio, 100) != 0) {
        LoRa_startReceiving(radio);
        return 0xFF;
    }

    uint16_t msg_id = Mesh_lcg_rand() & 0xFFFF;

    Mesh_transmit_ack(
        radio,
        PKT_REQ_ADDRESS,
        0xFF,
        0,
        payloadBuf,
        msg_id,
        txBuf,
        txBufLen
    );

    LoRa_startReceiving(radio);

    return Mesh_wait_for_ack(
        radio,
        uid,
        nodeId,
        rxBuf,
        rxBufLen,
        payloadBuf,
        payloadBufLen,
        5000
    );
}

uint8_t Mesh_wait_for_ack(
    LoRa *radio,
    uint32_t uid[3],
    int *nodeId,
    uint8_t *rxBuf,
    size_t rxBufLen,
    char *payloadBuf,
    size_t payloadBufLen,
    uint16_t timeout_ms
)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        uint8_t len = LoRa_receive(radio, rxBuf, rxBufLen - 1);
        if (len == 0) {
            HAL_Delay(2);
            continue;
        }

        if (len < sizeof(MeshHeader))
            continue;

        MeshHeader *hdr = (MeshHeader *)rxBuf;

        if (hdr->version != MESH_VERSION)
            continue;

        if (hdr->type != PKT_ACK_ADDRESS)
            continue;

        if (Mesh_isDuplicate(hdr->src, hdr->msg_id))
            continue;

        Mesh_rememberPacket(hdr->src, hdr->msg_id);

        uint8_t payloadStart = sizeof(MeshHeader);
        uint8_t payloadLen = hdr->payload_len;

        if (payloadLen > (len - payloadStart))
            payloadLen = len - payloadStart;

        if (payloadLen >= payloadBufLen)
            payloadLen = payloadBufLen - 1;

        memcpy(payloadBuf, &rxBuf[payloadStart], payloadLen);
        payloadBuf[payloadLen] = '\0';

        uint32_t r_uid0, r_uid1, r_uid2;
        unsigned int assignedId;

        int parsed = sscanf(
            payloadBuf,
            "%08lX-%08lX-%08lX|%u",
            &r_uid0, &r_uid1, &r_uid2, &assignedId
        );

        if (parsed != 4)
            continue;

        if (r_uid0 != uid[0] || r_uid1 != uid[1] || r_uid2 != uid[2])
            continue;

        *nodeId = (int)assignedId;

        return (uint8_t)(*nodeId);
    }

    return 0xFF;  // timeout
}

uint8_t Mesh_transmit_ack(
    LoRa *radio,
    uint8_t type,
    uint8_t dst,
    uint8_t src,
    const char *payload,
    uint16_t msg_id,
    uint8_t *txBuf,
    size_t txBufLen
)
{
    MeshHeader hdr;
    uint8_t payload_len = payload ? strlen(payload) : 0;
    uint16_t total_len = sizeof(MeshHeader) + payload_len;

    if (total_len > txBufLen)
        return 0;

    hdr.version = MESH_VERSION;
    hdr.type = type;
    hdr.src = src;
    hdr.dest = dst;
    hdr.ttl = 5;
    hdr.flags = 0;
    hdr.msg_id = msg_id;
    hdr.payload_len = payload_len;

    memcpy(txBuf, &hdr, sizeof(MeshHeader));
    if (payload_len)
        memcpy(txBuf + sizeof(MeshHeader), payload, payload_len);

    LoRa_gotoMode(radio, STNBY_MODE);
    HAL_Delay(2);

    uint8_t ok = LoRa_transmit(radio, txBuf, total_len, 2000);

    LoRa_startReceiving(radio);

    return (ok == 1);
}

uint8_t Mesh_transmit(
    LoRa *radio,
    uint8_t type,
    uint8_t dst,
    uint8_t src,
    const char *payload,
    uint16_t msg_id,
    uint8_t *txBuf,
    size_t txBufLen
)
{
    MeshHeader hdr;
    uint8_t payload_len = payload ? strlen(payload) : 0;
    uint16_t total_len = sizeof(MeshHeader) + payload_len;

    if (total_len > txBufLen)
        return 0;

    hdr.version = MESH_VERSION;
    hdr.type = type;
    hdr.src = src;
    hdr.dest = dst;
    hdr.ttl = 5;
    hdr.flags = 0;
    hdr.msg_id = msg_id;
    hdr.payload_len = payload_len;

    memcpy(txBuf, &hdr, sizeof(MeshHeader));
    if (payload_len)
        memcpy(txBuf + sizeof(MeshHeader), payload, payload_len);

    if (!Mesh_check_radio_status(radio))
    {
        Mesh_hard_reset_radio();
        HAL_Delay(100);
        LoRa_init(radio);
        LoRa_setSyncWord(radio, 0x34);
        LoRa_setSpreadingFactor(radio, 7);
        LoRa_enableCRC(radio, 1);
        LoRa_startReceiving(radio);
        HAL_Delay(100);

        if (!Mesh_check_radio_status(radio))
            return 0;
    }

    LoRa_gotoMode(radio, STNBY_MODE);
    HAL_Delay(20);

    LoRa_write(radio, RegIrqFlags, 0xFF);

    uint8_t ok = LoRa_transmit(radio, txBuf, total_len, 2000);
    if (!ok)
        return 0;

    if (!Mesh_wait_for_tx_complete(radio))
        return 0;

    LoRa_startReceiving(radio);
    return 1;
}

void Mesh_poll(
    LoRa *radio,
    uint8_t self_id,
    uint8_t *rxBuf,
    size_t rxBufLen,
    char *payloadBuf,
    size_t payloadBufLen
)
{
    static uint32_t lastModeCheck = 0;

    if (self_id == 0)
        return;

    if (HAL_GetTick() - lastModeCheck > 10000) {
        uint8_t op_mode = LoRa_read(radio, RegOpMode);
        if ((op_mode & 0x07) != 0x05) {
            LoRa_startReceiving(radio);
        }
        lastModeCheck = HAL_GetTick();
    }

    uint8_t len = LoRa_receive(radio, rxBuf, rxBufLen - 1);
    if (!len)
        return;

    Mesh_reset_watchdog();

    if (len < sizeof(MeshHeader))
        return;

    MeshHeader *hdr = (MeshHeader *)rxBuf;

    if (hdr->version != MESH_VERSION)
        return;

    if (hdr->dest != self_id && hdr->dest != 0xFF)
        return;

    if (Mesh_isDuplicate(hdr->src, hdr->msg_id))
        return;

    Mesh_rememberPacket(hdr->src, hdr->msg_id);

    uint8_t payloadStart = sizeof(MeshHeader);
    uint8_t payloadLen = hdr->payload_len;

    if (payloadLen > (len - payloadStart))
        payloadLen = len - payloadStart;

    if (payloadLen && payloadLen < payloadBufLen) {
        memcpy(payloadBuf, rxBuf + payloadStart, payloadLen);
        payloadBuf[payloadLen] = '\0';
    } else {
        payloadBuf[0] = '\0';
    }

    switch (hdr->type) {
        case PKT_REQ_DATA:
            Mesh_transmit(
                radio,
                PKT_ACK,
                hdr->src,
                self_id,
                "",
                hdr->msg_id,
                rxBuf,
                rxBufLen
            );

            HAL_Delay(50);

            Mesh_handle_req_data(
                radio,          // LoRa*
                hdr->src,       // src_id
                hdr->msg_id,    // msg_id
                self_id,        // self_id
                payloadBuf,     // payload buffer
                payloadBufLen,  // payload buffer length
                rxBuf,          // tx buffer (reuse rxBuf is OK)
                rxBufLen        // tx buffer length
            );
            break;

        case PKT_ACK:
            break;

        default:
            break;
    }
}


