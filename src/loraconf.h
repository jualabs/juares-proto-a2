#if(HAS_LORA)

/************************************************************
 * LMIC LoRaWAN configuration
 *
 * Read the values from TTN console (or whatever applies), insert them here,
 * and rename this file to src/loraconf.h
 *
 * Note that DEVEUI, APPEUI and APPKEY should all be specified in MSB format.
 * (This is different from standard LMIC-Arduino which expects DEVEUI and APPEUI
 * in LSB format.)

 * Set your DEVEUI here, if you have one. You can leave this untouched,
 * then the DEVEUI will be generated during runtime from device's MAC adress
 * and will be displayed on device's screen as well as on serial console.
 *
 * NOTE: Use MSB format (as displayed in TTN console, so you can cut & paste
 * from there)
 * For TTN, APPEUI in MSB format always starts with 0x70, 0xB3, 0xD5
 *
 * Note: If using a board with Microchip 24AA02E64 Uinique ID for deveui,
 * the DEVEUI will be overwriten by the one contained in the Microchip module
 *
 ************************************************************/

#define DK002

#ifdef DK002
static const u1_t DEVEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
#endif

#ifdef DK003
static const u1_t DEVEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
#endif

#ifdef DK004
static const u1_t DEVEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};
#endif

#ifdef DK005
static const u1_t DEVEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05};
#endif

// 70 B3 D5 7E D0 01 F1 56
// 00 00 00 00 00 00 00 05
static const u1_t APPEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x01, 0xF1, 0x56};

// 20 60 44 35 51 D7 2E D6 
// FD FB 89 32 C5 FE 4F 2D

static const u1_t APPKEY[16] = {0x20, 0x60, 0x44, 0x35, 0x51, 0xD7, 0x2E, 0xD6,
                                0xFD, 0xFB, 0x89, 0x32, 0xC5, 0xFE, 0x4F, 0x2D};

#endif // HAS_LORA