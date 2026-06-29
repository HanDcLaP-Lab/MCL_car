#include "board_float8_frame.h"
#include <string.h>

#define BOARD_FLOAT8_HEADER1      0xAAu
#define BOARD_FLOAT8_HEADER2      0x55u
#define BOARD_FLOAT8_TAIL         0x7Fu
#define BOARD_FLOAT8_CMD_INDEX    2u
#define BOARD_FLOAT8_SEQ_INDEX    3u
#define BOARD_FLOAT8_DATA_INDEX   4u
#define BOARD_FLOAT8_SUM_INDEX    (BOARD_FLOAT8_FRAME_SIZE - 2u)
#define BOARD_FLOAT8_TAIL_INDEX   (BOARD_FLOAT8_FRAME_SIZE - 1u)

static uint8_t Board_Float8_Checksum(const uint8_t *data, uint32_t length)
{
    uint8_t sum = 0;

    for (uint32_t i = 0; i < length; i++) {
        sum = (uint8_t)(sum + data[i]);
    }

    return sum;
}

void Board_Float8_Frame_Encode(uint8_t cmd, uint8_t seq, const float data[BOARD_FLOAT8_COUNT], uint8_t frame[BOARD_FLOAT8_FRAME_SIZE])
{
    frame[0] = BOARD_FLOAT8_HEADER1;
    frame[1] = BOARD_FLOAT8_HEADER2;
    frame[BOARD_FLOAT8_CMD_INDEX] = cmd;
    frame[BOARD_FLOAT8_SEQ_INDEX] = seq;
    memcpy(&frame[BOARD_FLOAT8_DATA_INDEX], data, BOARD_FLOAT8_DATA_BYTES);
    frame[BOARD_FLOAT8_SUM_INDEX] = Board_Float8_Checksum(&frame[BOARD_FLOAT8_CMD_INDEX], 2u + BOARD_FLOAT8_DATA_BYTES);
    frame[BOARD_FLOAT8_TAIL_INDEX] = BOARD_FLOAT8_TAIL;
}

uint8_t Board_Float8_Frame_Decode(const uint8_t frame[BOARD_FLOAT8_FRAME_SIZE], board_float8_frame_t *out)
{
    uint8_t checksum;

    if (frame[0] != BOARD_FLOAT8_HEADER1 || frame[1] != BOARD_FLOAT8_HEADER2) {
        return 0;
    }
    if (frame[BOARD_FLOAT8_TAIL_INDEX] != BOARD_FLOAT8_TAIL) {
        return 0;
    }

    checksum = Board_Float8_Checksum(&frame[BOARD_FLOAT8_CMD_INDEX], 2u + BOARD_FLOAT8_DATA_BYTES);
    if (checksum != frame[BOARD_FLOAT8_SUM_INDEX]) {
        return 0;
    }

    out->cmd = frame[BOARD_FLOAT8_CMD_INDEX];
    out->seq = frame[BOARD_FLOAT8_SEQ_INDEX];
    memcpy(out->data, &frame[BOARD_FLOAT8_DATA_INDEX], BOARD_FLOAT8_DATA_BYTES);

    return 1;
}
