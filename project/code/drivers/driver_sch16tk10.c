/*********************************************************************************************************************
* ÎÄ¼þÃû³Æ          driver_sch16tk10.c
* ¹¦ÄÜËµÃ÷          SCH16TK10¸ß¾«¶ÈIMU´«¸ÐÆ÷Çý¶¯³ÌÐòÊµÏÖÎÄ¼þ
* ×÷Õß              LittleMaster
* °æ±¾ÐÅÏ¢          v1.0
* ÐÞ¸Ä¼ÇÂ¼
* ÈÕÆÚ              ×÷Õß                °æ±¾
* 2025-09-17        LittleMaster       1.0v
* 
* ÎÄ¼þ×÷ÓÃËµÃ÷£º
* ±¾ÎÄ¼þÊÇMurata SCH1600ÏµÁÐ¸ß¾«¶ÈIMU´«¸ÐÆ÷µÄÇý¶¯³ÌÐòÊµÏÖ
* Ìá¹©ÍêÕûµÄ´«¸ÐÆ÷¿ØÖÆ¡¢Êý¾Ý¶ÁÈ¡¡¢ÅäÖÃ¹ÜÀíºÍÕï¶Ï¹¦ÄÜ
* 
* Ê¹ÓÃ×¢ÒâÊÂÏî£º
* 1. È·±£SPIÊ±ÐòÂú×ã´«¸ÐÆ÷ÒªÇó
* 2. ³õÊ¼»¯Ç°¼ì²éÓ²¼þÁ¬½Ó
* 3. ¸ù¾ÝÓ¦ÓÃÐèÇóÅäÖÃÂË²¨Æ÷ºÍÁéÃô¶È
* 4. ¶¨ÆÚ¼ì²é´«¸ÐÆ÷×´Ì¬
* 5. ÔÚ¹Ø¼üÓ¦ÓÃÖÐÆôÓÃÍ¨ÐÅ¼à¿Ø
********************************************************************************************************************/

#include "driver_sch16tk10.h"
#include "zf_common_headfile.h"

// Ó²¼þÒý½Å¶¨Òå / Hardware pin definitions (¸ù¾ÝÔ­ÀíÍ¼ U42)
// ×¢Òâ£ºÔ­ÀíÍ¼P07.xÒý½ÅÎÞ·¨ÓÃÓ²¼þSPI£¬Ê¹ÓÃÈí¼þSPI
#define EXTRESN_PORT    P07_4               // ¸´Î»Òý½Å / Reset pin EXTRESN (Òý½Å44)
#define SPI_CS_PIN      P07_3               // GPIO·½Ê½µÄÆ¬Ñ¡Òý½Å / CS pin as GPIO (Òý½Å43)
#define SOFT_SPI_SCK    P07_2               // Èí¼þSPIÊ±ÖÓÒý½Å / Software SPI SCK pin (Òý½Å42)
#define SOFT_SPI_MOSI   P07_1               // Èí¼þSPI MOSIÒý½Å / Software SPI MOSI pin (Òý½Å41)
#define SOFT_SPI_MISO   P07_0               // Èí¼þSPI MISOÒý½Å / Software SPI MISO pin (Òý½Å40)
#define GPIO_RESET      (P07_4)             // ±¸ÓÃ¸´Î»Òý½Å / Alternative reset pin

// Èí¼þSPI½á¹¹Ìå
static soft_spi_info_struct sch16_spi;

// Íâ²¿º¯ÊýÉùÃ÷£¨¿âµÄÍ·ÎÄ¼þÉùÃ÷ÓëÊµÏÖ²»Ò»ÖÂ£¬ÊÖ¶¯ÉùÃ÷Êµ¼Êº¯Êý£©
// ×¢Òâ£ºÊµ¼ÊÊµÏÖµÄº¯ÊýÃûÊÇ soft_spi_16bit_transfer£¬¶øÍ·ÎÄ¼þÖÐÉùÃ÷µÄÊÇ soft_spi_transfer_16bit
extern void soft_spi_16bit_transfer(soft_spi_info_struct *soft_spi_obj, const uint16 *write_buffer, uint16 *read_buffer, uint32 len);

// ¾²Ì¬±äÁ¿¶¨Òå / Static variable definitions
static pit_index_enum sampling_timer = PIT_CH0;  // ²ÉÑù¶¨Ê±Æ÷Í¨µÀ / Sampling timer channel

// ¾²Ì¬º¯ÊýÉùÃ÷ / Static function declarations
static void GPIO_init(void);               // GPIO³õÊ¼»¯º¯Êý / GPIO initialization function
static void SPI_Init(void);                // SPI³õÊ¼»¯º¯Êý / SPI initialization function
static uint8_t CRC8(uint64_t SPIframe);    // ¼ÆËã8Î»CRCÐ£Ñé / Calculate 8-bit CRC
static uint8_t CRC3(uint32_t SPIframe);    // ¼ÆËã3Î»CRCÐ£Ñé / Calculate 3-bit CRC

// ================================
// Ó²¼þ³éÏó²ãº¯ÊýÊµÏÖ / Hardware abstraction layer functions
// ================================

/**
 * @brief  GPIOÒý½Å³õÊ¼»¯ / GPIO pins initialization
 */
static void GPIO_init(void) {
    // ³õÊ¼»¯Íâ²¿¸´Î»Òý½Å£¬ÍÆÍìÊä³ö£¬Ä¬ÈÏ¸ßµçÆ½
    // Initialize external reset pin, push-pull output, default high level
    gpio_init(EXTRESN_PORT, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    
    // ³õÊ¼»¯SPIÆ¬Ñ¡Òý½Å£¬ÍÆÍìÊä³ö£¬Ä¬ÈÏ¸ßµçÆ½£¨Î´Ñ¡ÖÐ×´Ì¬£©
    // Initialize SPI chip select pin, push-pull output, default high level (deselected state)
    gpio_init(SPI_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
}

/**
 * @brief  SPIÍâÉè³õÊ¼»¯ / SPI peripheral initialization (Ê¹ÓÃÈí¼þSPI)
 */
static void SPI_Init(void) {
    // Ê¹ÓÃÈí¼þSPI£¬ÒòÎªP07.x²»Ö§³ÖÓ²¼þSPI :=(
    soft_spi_init(&sch16_spi, 0, 1, SOFT_SPI_SCK, SOFT_SPI_MOSI, SOFT_SPI_MISO, SOFT_SPI_PIN_NULL);
}

/**
 * @brief  Ó²¼þ³õÊ¼»¯ / Hardware initialization
 */
void hw_init(void) {
    GPIO_init();            // IOÒý½Å³õÊ¼»¯ / IO-pin initializations
    SPI_Init();             // SCH1600µÄSPIÍ¨µÀ³õÊ¼»¯ / SPI channel for SCH1600
    system_delay_ms(100);   // µÈ´ý100msÈÃÓ²¼þÎÈ¶¨ / Wait 100ms for hardware stabilization
}

/**
 * @brief  ½«Íâ²¿¸´Î»Òý½ÅÀ­¸ß / Set external reset pin to high level
 */
void hw_EXTRESN_High(void) {
    gpio_set_level(EXTRESN_PORT, GPIO_HIGH);
}

/**
 * @brief  ½«Íâ²¿¸´Î»Òý½ÅÀ­µÍ / Set external reset pin to low level
 */
void hw_EXTRESN_Low(void) {
    gpio_set_level(EXTRESN_PORT, GPIO_LOW);
}

/**
 * @brief  ½«SPIÆ¬Ñ¡Òý½ÅÀ­¸ß£¨È¡ÏûÑ¡ÖÐ£© / Set SPI chip select pin to high level (deselect)
 */
void hw_CS_High(void) {
    gpio_set_level(SPI_CS_PIN, GPIO_HIGH);
}

/**
 * @brief  ½«SPIÆ¬Ñ¡Òý½ÅÀ­µÍ£¨Ñ¡ÖÐ£© / Set SPI chip select pin to low level (select)
 */
void hw_CS_Low(void) {
    gpio_set_level(SPI_CS_PIN, GPIO_LOW);
}

/**
 * @brief  ºÁÃëÑÓÊ±º¯Êý / Millisecond delay function
 */
void hw_delay(uint32_t ms) {
    system_delay_ms(ms);
}

/**
 * @brief  ÉèÖÃ¶¨Ê±Æ÷ÆµÂÊ / Set timer frequency
 */
void hw_timer_setFreq(uint32_t freq) {
    uint32_t period_us = 1000000 / freq;  // ¼ÆËãÖÜÆÚ£¨Î¢Ãë£© / Calculate period in microseconds
    pit_us_init(sampling_timer, period_us);  // ³õÊ¼»¯PIT¶¨Ê±Æ÷ / Initialize PIT timer
}

/**
 * @brief  Í£Ö¹¶¨Ê±Æ÷ÖÐ¶Ï / Stop timer interrupt
 */
void hw_timer_stopIT(void) {
    pit_disable(sampling_timer);  // ½ûÓÃPIT¶¨Ê±Æ÷ / Disable PIT timer
}

/**
 * @brief  Æô¶¯¶¨Ê±Æ÷ÖÐ¶Ï / Start timer interrupt
 */
void hw_timer_startIT(void) {
    pit_enable(sampling_timer);   // Ê¹ÄÜPIT¶¨Ê±Æ÷ / Enable PIT timer
}

/**
 * @brief  ·¢ËÍ48Î»SPIÇëÇó²¢½ÓÊÕÏìÓ¦ / Send 48-bit SPI request and receive response
 */


// ================================
// ´«¸ÐÆ÷Çý¶¯º¯ÊýÊµÏÖ / Sensor driver functions
// ================================

/**
 * @brief  ¸´Î»SCH1´«¸ÐÆ÷£¨Ó²¼þ¸´Î»£© / Reset SCH1 sensor (hardware reset)
 */
void SCH1_reset(void)
{              
    hw_EXTRESN_Low();           // À­µÍ¸´Î»ÐÅºÅ / Pull reset signal low
    hw_delay(2);                // ÑÓÊ±2ms / Delay 2ms
    hw_EXTRESN_High();          // À­¸ß¸´Î»ÐÅºÅ / Pull reset signal high
}

void SCH1_sendSPIreset(void)
{
    SCH1_sendRequest(REQ_SOFTRESET);
}



bool SCH1_isValidFilterFreq(uint32_t Freq)
{   
    if (Freq == 13 || Freq == 30 || Freq == 68 || Freq == 235 || Freq == 280 || Freq == 370 || Freq == SCH1_FILTER_BYPASS) 
        return true;
    else    
        return false;
}



bool SCH1_isValidRateSens(uint32_t Sens)
{   
    if (Sens == 1600 || Sens == 3200 || Sens == 6400) 
        return true;
    else    
        return false;
}



bool SCH1_isValidAccSens(uint32_t Sens)
{   
    if (Sens == 3200 || Sens == 6400 || Sens == 12800 || Sens == 25600) 
        return true;
    else    
        return false;
}


bool SCH1_isValidDecimation(uint32_t Decimation)
{   
    if (Decimation == 2 || Decimation == 4 || Decimation == 8 || Decimation == 16 || Decimation == 32) 
        return true;
    else    
        return false;
}


/**
 * @brief ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç·ï¿½ï¿½ï¿½Ð§
 *
 * @param Freq - ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ [Hz]
 * 
 * @return true = ï¿½ï¿½Ð§
 *         false = ï¿½ï¿½Ð§                          
 */
bool SCH1_isValidSampleRate(uint32_t Freq)
{   
    if ((Freq >= 1) && (Freq <= 10000))
        return true;

    return false;
}


int SCH1_setFilters(uint32_t Freq_Rate12, uint32_t Freq_Acc12, uint32_t Freq_Acc3)
{
    uint32_t dataField;
    uint64_t requestFrame_Rate12;
    uint64_t responseFrame_Rate12;
    uint64_t requestFrame_Acc12;
    uint64_t responseFrame_Acc12;
    uint64_t requestFrame_Acc3;
    uint64_t responseFrame_Acc3;
    uint8_t  CRCvalue;

    if (SCH1_isValidFilterFreq(Freq_Rate12) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH1_isValidFilterFreq(Freq_Acc12) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH1_isValidFilterFreq(Freq_Acc3) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    
    // Set filters for Rate_XYZ1 (interpolated) and Rate_XYZ2 (decimated) outputs.
    requestFrame_Rate12 = REQ_SET_FILT_RATE;
    dataField = SCH1_convertFilterToBitfield(Freq_Rate12);
    requestFrame_Rate12 |= dataField;
    requestFrame_Rate12 <<= 8;
    CRCvalue = CRC8(requestFrame_Rate12);
    requestFrame_Rate12 |= CRCvalue;
    SCH1_sendRequest(requestFrame_Rate12);

    // Set filters for Acc_XYZ1 (interpolated) and Acc_XYZ2 (decimated) outputs.
    requestFrame_Acc12 = REQ_SET_FILT_ACC12;
    dataField = SCH1_convertFilterToBitfield(Freq_Acc12);
    requestFrame_Acc12 |= dataField;
    requestFrame_Acc12 <<= 8;
    CRCvalue = CRC8(requestFrame_Acc12);
    requestFrame_Acc12 |= CRCvalue;
    SCH1_sendRequest(requestFrame_Acc12);

    // Set filters for Acc_XYZ3 (interpolated) output.
    requestFrame_Acc3 = REQ_SET_FILT_ACC3;
    dataField = SCH1_convertFilterToBitfield(Freq_Acc3);
    requestFrame_Acc3 |= dataField;
    requestFrame_Acc3 <<= 8;
    CRCvalue = CRC8(requestFrame_Acc3);
    requestFrame_Acc3 |= CRCvalue;
    SCH1_sendRequest(requestFrame_Acc3);
    
    // Read back filter register contents.
    SCH1_sendRequest(REQ_READ_FILT_RATE);        
    responseFrame_Rate12 = SCH1_sendRequest(REQ_READ_FILT_ACC12);
    responseFrame_Acc12 = SCH1_sendRequest(REQ_READ_FILT_ACC3);
    responseFrame_Acc3 = SCH1_sendRequest(REQ_READ_FILT_ACC3);
    
    // Check that return frame is not blank.
    if ((responseFrame_Rate12 == 0xFFFFFFFFFFFF) || (responseFrame_Rate12 == 0x00))
        return SCH1_ERR_OTHER;
    if ((responseFrame_Acc12 == 0xFFFFFFFFFFFF) || (responseFrame_Acc12 == 0x00))
        return SCH1_ERR_OTHER;
    if ((responseFrame_Acc3 == 0xFFFFFFFFFFFF) || (responseFrame_Acc3 == 0x00))
        return SCH1_ERR_OTHER;
    
    // Check that Source Address matches Target Address.
    if (((requestFrame_Rate12 & TA_FIELD_MASK) >> 38) != ((responseFrame_Rate12 & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    if (((requestFrame_Acc12 & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc12 & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    if (((requestFrame_Acc3 & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc3 & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    
    // Check that read and written data match.
    if ((requestFrame_Rate12 & DATA_FIELD_MASK) != (responseFrame_Rate12 & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    if ((requestFrame_Acc12 & DATA_FIELD_MASK) != (responseFrame_Acc12 & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    if ((requestFrame_Acc3 & DATA_FIELD_MASK) != (responseFrame_Acc3 & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    
    return SCH1_OK;
}



int SCH1_setRateSensDec(uint16_t Sens_Rate1, uint16_t Sens_Rate2, uint16_t Dec_Rate2)
{
    uint32_t dataField;
    uint32_t bitField;
    uint64_t requestFrame_Rate_Ctrl;
    uint64_t responseFrame_Rate_Ctrl;
    uint8_t  CRCvalue;
 
    if (SCH1_isValidRateSens(Sens_Rate1) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH1_isValidRateSens(Sens_Rate2) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH1_isValidDecimation(Dec_Rate2) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }

    // Set sensitivities for Rate_XYZ1 (interpolated) and Rate_XYZ2 (decimated) outputs.
    // Also set decimation for Rate_XYZ2.
    requestFrame_Rate_Ctrl = REQ_SET_RATE_CTRL;
    dataField = SCH1_convertRateSensToBitfield(Sens_Rate1);
    dataField <<= 3;
    bitField = SCH1_convertRateSensToBitfield(Sens_Rate2);
    dataField |= bitField;
    dataField <<= 3;
    bitField = SCH1_convertDecimationToBitfield(Dec_Rate2);
    dataField |= bitField;
    dataField <<= 3;
    dataField |= bitField;
    dataField <<= 3;
    dataField |= bitField;
    
    requestFrame_Rate_Ctrl |= dataField;
    requestFrame_Rate_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_Rate_Ctrl);
    requestFrame_Rate_Ctrl |= CRCvalue;
    SCH1_sendRequest(requestFrame_Rate_Ctrl);

    // Read back rate control register contents.
    SCH1_sendRequest(REQ_READ_RATE_CTRL);
    responseFrame_Rate_Ctrl = SCH1_sendRequest(REQ_READ_RATE_CTRL);

    // Check that return frame is not blank.
    if ((responseFrame_Rate_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Rate_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((requestFrame_Rate_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_Rate_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    
    // Check that read and written data match.
    if ((requestFrame_Rate_Ctrl & DATA_FIELD_MASK) != (responseFrame_Rate_Ctrl & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;

    return SCH1_OK;
}



int SCH1_getRateSensDec(uint16_t *Sens_Rate1, uint16_t *Sens_Rate2, uint16_t *Dec_Rate2)
{
    uint32_t dataField;
    uint64_t responseFrame_Rate_Ctrl;

    SCH1_sendRequest(REQ_READ_RATE_CTRL);
    responseFrame_Rate_Ctrl = SCH1_sendRequest(REQ_READ_RATE_CTRL);

    if ((responseFrame_Rate_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Rate_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    if (((REQ_READ_RATE_CTRL & TA_FIELD_MASK) >> 38) != ((responseFrame_Rate_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;

    dataField = (uint16_t)(responseFrame_Rate_Ctrl >> 8) & 0x07;
    *Dec_Rate2 = (uint16_t)SCH1_convertBitfieldToDecimation(dataField);

    dataField = (uint16_t)(responseFrame_Rate_Ctrl >> 17) & 0x07;
    *Sens_Rate2 = (uint16_t)SCH1_convertBitfieldToRateSens(dataField);

    dataField = (uint16_t)(responseFrame_Rate_Ctrl >> 20) & 0x07;
    *Sens_Rate1 = (uint16_t)SCH1_convertBitfieldToRateSens(dataField);

    return SCH1_OK;
}



int SCH1_setAccSensDec(uint16_t Sens_Acc1, uint16_t Sens_Acc2, uint16_t Sens_Acc3, uint16_t Dec_Acc2)
{
    uint32_t dataField;
    uint32_t bitField;
    uint64_t requestFrame_Acc12_Ctrl;
    uint64_t responseFrame_Acc12_Ctrl;
    uint64_t requestFrame_Acc3_Ctrl;
    uint64_t responseFrame_Acc3_Ctrl;
    uint8_t  CRCvalue;
 
    if (SCH1_isValidAccSens(Sens_Acc1) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH1_isValidAccSens(Sens_Acc2) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH1_isValidAccSens(Sens_Acc3) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH1_isValidDecimation(Dec_Acc2) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }

    // Set sensitivities for Acc_XYZ1 (interpolated) and Acc_XYZ2 (decimated) outputs.
    // Also set decimation for Acc_XYZ2.
    requestFrame_Acc12_Ctrl = REQ_SET_ACC12_CTRL;
    dataField = SCH1_convertAccSensToBitfield(Sens_Acc1);
    dataField <<= 3;
    bitField = SCH1_convertAccSensToBitfield(Sens_Acc2);
    dataField |= bitField;
    dataField <<= 3;
    bitField = SCH1_convertDecimationToBitfield(Dec_Acc2);
    dataField |= bitField;
    dataField <<= 3;
    dataField |= bitField;
    dataField <<= 3;
    dataField |= bitField;
    
    requestFrame_Acc12_Ctrl |= dataField;
    requestFrame_Acc12_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_Acc12_Ctrl);
    requestFrame_Acc12_Ctrl |= CRCvalue;
    SCH1_sendRequest(requestFrame_Acc12_Ctrl);

    // Set sensitivity for Acc_XYZ3 (interpolated) output.
    requestFrame_Acc3_Ctrl = REQ_SET_ACC3_CTRL;
    dataField = SCH1_convertAccSensToBitfield(Sens_Acc3);
    requestFrame_Acc3_Ctrl |= dataField;
    requestFrame_Acc3_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_Acc3_Ctrl);
    requestFrame_Acc3_Ctrl |= CRCvalue;
    SCH1_sendRequest(requestFrame_Acc3_Ctrl);

    // Read back sensitivity control register contents.
    SCH1_sendRequest(REQ_READ_ACC12_CTRL);
    responseFrame_Acc12_Ctrl = SCH1_sendRequest(REQ_READ_ACC3_CTRL);
    responseFrame_Acc3_Ctrl = SCH1_sendRequest(REQ_READ_ACC3_CTRL);

    // Check that return frame is not blank.
    if ((responseFrame_Acc12_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Acc12_Ctrl == 0x00))
        return SCH1_ERR_OTHER;
    if ((responseFrame_Acc3_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Acc3_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((requestFrame_Acc12_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc12_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    if (((requestFrame_Acc3_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc3_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    
    // Check that read and written data match.
    if ((requestFrame_Acc12_Ctrl & DATA_FIELD_MASK) != (responseFrame_Acc12_Ctrl & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    if ((requestFrame_Acc3_Ctrl & DATA_FIELD_MASK) != (responseFrame_Acc3_Ctrl & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;

    return SCH1_OK;
}



int SCH1_getAccSensDec(uint16_t *Sens_Acc1, uint16_t *Sens_Acc2, uint16_t *Sens_Acc3, uint16_t *Dec_Acc2)
{
    uint32_t dataField;
    uint64_t responseFrame_Acc12_Ctrl;
    uint64_t responseFrame_Acc3_Ctrl;

    // Read Acc12 and Acc3 control register contents.
    SCH1_sendRequest(REQ_READ_ACC12_CTRL);
    responseFrame_Acc12_Ctrl = SCH1_sendRequest(REQ_READ_ACC3_CTRL);
    responseFrame_Acc3_Ctrl = SCH1_sendRequest(REQ_READ_ACC3_CTRL);


    // Check that return frame is not blank.
    if ((responseFrame_Acc12_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Acc12_Ctrl == 0x00))
        return SCH1_ERR_OTHER;
    if ((responseFrame_Acc3_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Acc3_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((REQ_READ_ACC12_CTRL & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc12_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    if (((REQ_READ_ACC3_CTRL & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc3_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;

    

    dataField = (uint16_t)(responseFrame_Acc12_Ctrl >> 8) & 0x07;
    *Dec_Acc2 = (uint16_t)SCH1_convertBitfieldToDecimation(dataField);

    dataField = (uint16_t)(responseFrame_Acc12_Ctrl >> 17) & 0x07;
    *Sens_Acc2 = (uint16_t)SCH1_convertBitfieldToAccSens(dataField);

   
    dataField = (uint16_t)(responseFrame_Acc12_Ctrl >> 20) & 0x07;
    *Sens_Acc1 = (uint16_t)SCH1_convertBitfieldToAccSens(dataField);

    dataField = (uint16_t)(responseFrame_Acc3_Ctrl >> 8) & 0x07;
    *Sens_Acc3 = (uint16_t)SCH1_convertBitfieldToAccSens(dataField);

    return SCH1_OK;
}



int SCH1_enableMeas(bool enableSensor, bool setEOI)
{
    uint64_t requestFrame_Mode_Ctrl;
    uint64_t responseFrame_Mode_Ctrl;
    uint8_t  CRCvalue;

    requestFrame_Mode_Ctrl = REQ_SET_MODE_CTRL;

    // Handle EN_SENSOR -bit
    if (enableSensor)
        requestFrame_Mode_Ctrl |= 0x01;

    // Handle EOI_CTRL -bit
    if (setEOI)
        requestFrame_Mode_Ctrl |= 0x02;

    requestFrame_Mode_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_Mode_Ctrl);
    requestFrame_Mode_Ctrl |= CRCvalue;
    SCH1_sendRequest(requestFrame_Mode_Ctrl);

    // Read back sensitivity control register contents.
    SCH1_sendRequest(REQ_READ_MODE_CTRL);
    responseFrame_Mode_Ctrl = SCH1_sendRequest(REQ_READ_MODE_CTRL);

    // Check that return frame is not blank.
    if ((responseFrame_Mode_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Mode_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((requestFrame_Mode_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_Mode_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    
    return SCH1_OK;
}


int SCH1_setDRY(int8_t polarity, bool enable)
{
    uint64_t requestFrame_User_If_Ctrl;
    uint64_t responseFrame_User_If_Ctrl;
    uint64_t dataContent;
    uint8_t  CRCvalue;

    if ((polarity < -1) || (polarity > 1))
        return SCH1_ERR_INVALID_PARAM;
    
    // Read USER_IF_CTRL -register content
    SCH1_sendRequest(REQ_READ_USER_IF_CTRL);
    responseFrame_User_If_Ctrl = SCH1_sendRequest(REQ_READ_USER_IF_CTRL);
    dataContent = (responseFrame_User_If_Ctrl & DATA_FIELD_MASK) >> 8;
    
    if (polarity == 0)
        dataContent &= (uint16_t)~0x40;   // Set DRY active high (0b01000000)
    else if (polarity == 1)
        dataContent |= 0x40;              // Set DRY active low
    
    if (enable)
        dataContent |= 0x20;              // Set DRY enabled (0b00100000)
    else
        dataContent &= (uint16_t)~0x20;   // Set DRY disabled
        
    requestFrame_User_If_Ctrl = REQ_SET_USER_IF_CTRL;
    requestFrame_User_If_Ctrl |= dataContent;
    requestFrame_User_If_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_User_If_Ctrl);
    requestFrame_User_If_Ctrl |= CRCvalue;
    SCH1_sendRequest(requestFrame_User_If_Ctrl);

    // Read back sensitivity control register contents.
    SCH1_sendRequest(REQ_READ_USER_IF_CTRL);
    responseFrame_User_If_Ctrl = SCH1_sendRequest(REQ_READ_USER_IF_CTRL);

    // Check that return frame is not blank.
    if ((responseFrame_User_If_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_User_If_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((requestFrame_User_If_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_User_If_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;

    // Check that read and written data match.
    if ((requestFrame_User_If_Ctrl & DATA_FIELD_MASK) != (responseFrame_User_If_Ctrl & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    
    return SCH1_OK;
}



uint32_t SCH1_convertFilterToBitfield(uint32_t Freq)
{
    switch (Freq)
    {
        case 13:
            return 0x092;   // 010 010 010
        case 30:
            return 0x049;   // 001 001 001
        case 68:
            return 0x000;   // 000 000 000        
        case 235:
            return 0x16D;   // 101 101 101
        case 280:
            return 0x0DB;   // 011 011 011
        case 370:
            return 0x124;   // 100 100 100
        case SCH1_FILTER_BYPASS:
            return 0x1FF;   // 111 111 111, filter bypass mode
        default:
            return 0x000;       
    }
}



uint32_t SCH1_convertRateSensToBitfield(uint32_t Sens)
{
    switch (Sens)
    {
        case 1600:
            return 0x02;   // 010
        case 3200:
            return 0x03;   // 011      
        case 6400:
            return 0x04;   // 100
        default:
            return 0x01;       
    }
}


uint32_t SCH1_convertBitfieldToRateSens(uint32_t bitfield)
{
    switch (bitfield)
    {
        case 0x02:          // 010
            return 1600;
        case 0x03:          // 011
            return 3200;      
        case 0x04:          // 100
            return 6400;
        default:
            return 0x00;       
    }
}



uint32_t SCH1_convertAccSensToBitfield(uint32_t Sens)
{
    switch (Sens)
    {
        case 3200:
            return 0x01;   // 001
        case 6400:
            return 0x02;   // 010
        case 12800:
            return 0x03;   // 011      
        case 25600:
            return 0x04;   // 100
        default:
            return 0x00;       
    }
}



uint32_t SCH1_convertBitfieldToAccSens(uint32_t bitfield)
{
    switch (bitfield)
    {
        case 0x01:          // 001
            return 3200;
        case 0x02:          // 010
            return 6400;
        case 0x03:          // 011
            return 12800;      
        case 0x04:          // 100
            return 25600;
        default:
            return 0x00;       
    }
}


uint32_t SCH1_convertDecimationToBitfield(uint32_t Decimation)
{
    switch (Decimation)
    {
        case 2:
            return 0x00;   // 001
        case 4:
            return 0x01;   // 010
        case 8:
            return 0x02;   // 011      
        case 16:
            return 0x03;   // 100
        case 32:
            return 0x04;   // 100
        default:
            return 0x00;       
    }
}


uint32_t SCH1_convertBitfieldToDecimation(uint32_t bitfield)
{
    switch (bitfield)
    {
        case 0x00:      // 001
            return 2;
        case 0x01:      // 010
            return 4;
        case 0x02:      // 011
            return 8;      
        case 0x03:      // 100
            return 16;
        case 0x04:      // 100
            return 32;
        default:
            return 0x00;       
    }
}


int SCH1_getStatus(SCH1_status *Status)
{
    if (Status == NULL) {
        return SCH1_ERR_NULL_POINTER;
    }
    
    SCH1_sendRequest(REQ_READ_STAT_SUM);
    Status->Summary     = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_SUM_SAT));
    Status->Summary_Sat = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_COM));
    Status->Common      = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_RATE_COM));
    Status->Rate_Common = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_RATE_X));
    Status->Rate_X      = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_RATE_Y));
    Status->Rate_Y      = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_RATE_Z));
    Status->Rate_Z      = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_ACC_X));    
    Status->Acc_X       = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_ACC_Y));
    Status->Acc_Y       = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_ACC_Z));
    Status->Acc_Z       = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_ACC_Z));

    return SCH1_OK;
}



bool SCH1_verifyStatus(SCH1_status *Status)
{
    if (Status == NULL) {
        return false;
    }

    // æ£€æŸ¥å…³é”?çŠ¶æ€ä½ï¼Œè€Œä¸æ˜?è¦æ±‚æ‰€æœ‰å¯„å­˜å™¨éƒ½æ˜¯0xffff
    // æ£€æŸ?Summaryå¯„å­˜å™¨ä¸­çš„å…³é”?ä½?
    if (!(Status->Summary & S_SUM_INIT_RDY))  // æ£€æŸ¥åˆå§‹åŒ–å°±ç»ªä½?
        return false;
    
    // æ£€æŸ?CommonçŠ¶æ€å¯„å­˜å™¨ä¸?çš„å…³é”?ä½?
    if (!(Status->Common & S_COM_CMN_STS_RDY))  // æ£€æŸ¥é€šç”¨çŠ¶æ€å°±ç»?ä½?
        return false;
    
    // æ£€æŸ?Rate CommonçŠ¶æ€å¯„å­˜å™¨ä¸?çš„å…³é”?ä½?
    if (!(Status->Rate_Common & S_RATE_COM_STS_RDY))  // æ£€æŸ¥è?’é€Ÿåº¦çŠ¶æ€å°±ç»?ä½?
        return false;
    
    // æ£€æŸ¥å„ä¸?è½´çš„çŠ¶æ€å°±ç»?ä½?
    if (!(Status->Rate_X & S_RATE_X_QC) || !(Status->Rate_Y & S_RATE_Y_QC) || !(Status->Rate_Z & S_RATE_Z_QC))
        return false;
    
    if (!(Status->Acc_X & S_ACC_X_STS_RDY) || !(Status->Acc_Y & S_ACC_Y_STS_RDY) || !(Status->Acc_Z & S_ACC_Z_STS_RDY))
        return false;
    
    return true;
}



char* SCH1_getSnbr(void)
{
    uint16_t sn_id1;
    uint16_t sn_id2;
    uint16_t sn_id3;
    static char strBuffer[15];

    SCH1_sendRequest(REQ_READ_SN_ID1);
    sn_id1 = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_SN_ID1)); // ä¿?æ­£ï¼šä½¿ç”¨æ­£ç¡®çš„ID1è¯·æ±‚
    sn_id2 = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_SN_ID2));
    sn_id3 = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_SN_ID3));

    // Build serial number string 
    snprintf(strBuffer, 14, "%05d%01X%04X", sn_id2, sn_id1 & 0x000F, sn_id3);
    
    return strBuffer;
}

uint64_t hw_SPI48_Send_Request(uint64_t Request)
{
  
    uint64_t ReceivedData = 0;
    uint16_t txBuffer[3];
    uint16_t rxBuffer[3];
    uint8_t index;
    uint8_t size = 3;   // 48-bit SPI-transfer consists of three 16-bit transfers.
    
    // Split Request qword (MOSI data) to tx buffer
    for (index = 0; index < size; index++) {
        txBuffer[size - index - 1] = (Request >> (index << 4)) & 0xFFFF;
    }

    // Send tx buffer and receive rx buffer simultaneously (Ê¹ÓÃÈí¼þSPI)
    hw_CS_Low();
    system_delay_us(1);  // Small delay for CS to settle (¸ù¾ÝSCH16TK10Êý¾ÝÊÖ²áÒªÇó)
    // ×¢Òâ£ºÊ¹ÓÃÊµÏÖÎÄ¼þÖÐµÄÊµ¼Êº¯ÊýÃû£¨¿âµÄÍ·ÎÄ¼þÉùÃ÷ÓÐÎó£©
    soft_spi_16bit_transfer(&sch16_spi, txBuffer, rxBuffer, size);
    system_delay_us(1);  // Small delay before releasing CS
    hw_CS_High();

    // Create ReceivedData qword from received rx buffer (MISO data)
    for (index = 0; index < size; index++) {
        ReceivedData |= (uint64_t)rxBuffer[index] << ((size - index - 1) << 4);
    }

    return ReceivedData;

}


uint64_t SCH1_sendRequest(uint64_t Request)
{
    uint64_t response = hw_SPI48_Send_Request(Request);
    
    // è‡?åŠ¨æ›´æ–°SPIç»Ÿè?¡ä¿¡æ?
    if (response & ERROR_FIELD_MASK) {
        // æ£€æŸ¥æ˜¯å¦æ˜¯CRCé”™è??
        if (!SCH1_checkCRC8(response)) {
            SCH1_updateSPIStats(false, 1);  // CRCé”™è??
        } else {
            SCH1_updateSPIStats(false, 2);  // å¸§é”™è¯?
        }
    } else {
        SCH1_updateSPIStats(true, 0);       // æˆåŠŸ
    }
    
    return response;
}



uint8_t CRC8(uint64_t SPIframe)
{
    uint64_t data = SPIframe & 0xFFFFFFFFFF00LL;
    uint8_t crc = 0xFF;

    for (int i = 47; i >= 0; i--)
    {
        uint8_t data_bit = (data >> i) & 0x01;
        crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0x2F) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
    }
    
    return crc;
}



bool SCH1_checkCRC8(uint64_t SPIframe)
{
    if((uint8_t)(SPIframe & 0xff) == CRC8(SPIframe))
        return true;
    else
        return false;
}



uint8_t CRC3(uint32_t SPIframe)
{
    uint32_t data = SPIframe & 0xFFFFFFF8;
    uint8_t crc = 0x05;
 
    for (int i = 31; i >= 0; i--)
    {
        uint8_t data_bit = (data >> i) & 0x01;
        crc = crc & 0x4 ? (uint8_t)((crc << 1) ^ 0x3) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
        crc &= 0x07;
    }
 
    return crc;
}



bool SCH1_checkCRC3(uint32_t SPIframe)
{
    if((uint8_t)(SPIframe & 0x07) == CRC3(SPIframe))
        return true;
    else
        return false;
}



int SCH1_init(SCH1_filter sFilter, SCH1_sensitivity sSensitivity, SCH1_decimation sDecimation, bool enableDRY) 
{
    // ä½¿ç”¨ç¡?ä»¶æŠ½è±¡å±‚åˆå?‹åŒ–
    hw_init();
  
    int ret = SCH1_OK;
    uint8_t startup_attempt = 0;
    bool SCH1status = false;
    SCH1_status SCH1statusAll;
        
    SCH1_reset();
    
    for (startup_attempt = 0; startup_attempt < 2; startup_attempt++) {
                                    
        hw_delay(32);       
      
        SCH1_setFilters(sFilter.Rate12, sFilter.Acc12, sFilter.Acc3);
        SCH1_setRateSensDec(sSensitivity.Rate1, sSensitivity.Rate2, sDecimation.Rate2);
        SCH1_setAccSensDec(sSensitivity.Acc1, sSensitivity.Acc2, sSensitivity.Acc3, sDecimation.Acc2);
        if (enableDRY)
            SCH1_setDRY(0, true);   // 0 = DRY active high
        else
            SCH1_setDRY(0, false);
   
        SCH1_enableMeas(true, false);

        hw_delay(215);
   
        SCH1_getStatus(&SCH1statusAll);

        SCH1_enableMeas(true, true);
  
        hw_delay(3);
     
        SCH1_getStatus(&SCH1statusAll);
        SCH1_getStatus(&SCH1statusAll);

        if (!SCH1_verifyStatus(&SCH1statusAll)) {
            SCH1status = false;            
            SCH1_reset();
        }
        else {
            SCH1status = true;           
            break;
        }
        
    } 

    if (SCH1status != true)
        ret = SCH1_ERR_SENSOR_INIT;
             
    return ret;
}


void SCH1_getData(SCH1_raw_data *data)
{
    // Read all channels sequentially
    // Rate1 channels
    SCH1_sendRequest(REQ_READ_RATE_X1);
    uint64_t rate1_x_raw = SCH1_sendRequest(REQ_READ_RATE_Y1);
    uint64_t rate1_y_raw = SCH1_sendRequest(REQ_READ_RATE_Z1);
    uint64_t rate1_z_raw = SCH1_sendRequest(REQ_READ_ACC_X1);
    
    // Acc1 channels
    uint64_t acc1_x_raw  = SCH1_sendRequest(REQ_READ_ACC_Y1);
    uint64_t acc1_y_raw  = SCH1_sendRequest(REQ_READ_ACC_Z1);
    uint64_t acc1_z_raw  = SCH1_sendRequest(REQ_READ_RATE_X2);
    
    // Rate2 channels
    uint64_t rate2_x_raw = SCH1_sendRequest(REQ_READ_RATE_Y2);
    uint64_t rate2_y_raw = SCH1_sendRequest(REQ_READ_RATE_Z2);
    uint64_t rate2_z_raw = SCH1_sendRequest(REQ_READ_ACC_X2);
    
    // Acc2 channels
    uint64_t acc2_x_raw  = SCH1_sendRequest(REQ_READ_ACC_Y2);
    uint64_t acc2_y_raw  = SCH1_sendRequest(REQ_READ_ACC_Z2);
    uint64_t acc2_z_raw  = SCH1_sendRequest(REQ_READ_ACC_X3);
    
    // Acc3 channels
    uint64_t acc3_x_raw  = SCH1_sendRequest(REQ_READ_ACC_Y3);
    uint64_t acc3_y_raw  = SCH1_sendRequest(REQ_READ_ACC_Z3);
    uint64_t acc3_z_raw  = SCH1_sendRequest(REQ_READ_TEMP);
    
    // Temperature
    uint64_t temp_raw   = SCH1_sendRequest(REQ_READ_TEMP);

    // Get possible frame errors
    uint64_t miso_words[] = {rate1_x_raw, rate1_y_raw, rate1_z_raw, 
                            acc1_x_raw, acc1_y_raw, acc1_z_raw,
                            rate2_x_raw, rate2_y_raw, rate2_z_raw,
                            acc2_x_raw, acc2_y_raw, acc2_z_raw,
                            acc3_x_raw, acc3_y_raw, acc3_z_raw,
                            temp_raw};       
    data->frame_error = SCH1_check_48bit_frame_error(miso_words, (sizeof(miso_words) / sizeof(uint64_t)));
    
    // Parse MISO data to structure
    // Rate1
    data->Rate1_raw[AXIS_X] = SPI48_DATA_INT32(rate1_x_raw);
    data->Rate1_raw[AXIS_Y] = SPI48_DATA_INT32(rate1_y_raw);
    data->Rate1_raw[AXIS_Z] = SPI48_DATA_INT32(rate1_z_raw);
    
    // Acc1
    data->Acc1_raw[AXIS_X]  = SPI48_DATA_INT32(acc1_x_raw);
    data->Acc1_raw[AXIS_Y]  = SPI48_DATA_INT32(acc1_y_raw);
    data->Acc1_raw[AXIS_Z]  = SPI48_DATA_INT32(acc1_z_raw);
    
    // Rate2
    data->Rate2_raw[AXIS_X] = SPI48_DATA_INT32(rate2_x_raw);
    data->Rate2_raw[AXIS_Y] = SPI48_DATA_INT32(rate2_y_raw);
    data->Rate2_raw[AXIS_Z] = SPI48_DATA_INT32(rate2_z_raw);
    
    // Acc2
    data->Acc2_raw[AXIS_X]  = SPI48_DATA_INT32(acc2_x_raw);
    data->Acc2_raw[AXIS_Y]  = SPI48_DATA_INT32(acc2_y_raw);
    data->Acc2_raw[AXIS_Z]  = SPI48_DATA_INT32(acc2_z_raw);
    
    // Acc3
    data->Acc3_raw[AXIS_X]  = SPI48_DATA_INT32(acc3_x_raw);
    data->Acc3_raw[AXIS_Y]  = SPI48_DATA_INT32(acc3_y_raw);
    data->Acc3_raw[AXIS_Z]  = SPI48_DATA_INT32(acc3_z_raw);

    // Temperature data is always 16 bits wide. Drop 4 LSBs as they are not used.
    data->Temp_raw = SPI48_DATA_INT32(temp_raw) >> 4;
    
}


void SCH1_convert_data(SCH1_raw_data *data_in, SCH1_result *data_out)
{
    // Convert from raw counts to sensitivity - data_in contains summed data, need to average first
    
    // Rate channels
    data_out->Rate1[AXIS_X] = (float)data_in->Rate1_raw[AXIS_X] / (SENSITIVITY_RATE1 * (float)AVG_FACTOR);
    data_out->Rate1[AXIS_Y] = (float)data_in->Rate1_raw[AXIS_Y] / (SENSITIVITY_RATE1 * (float)AVG_FACTOR);
    data_out->Rate1[AXIS_Z] = (float)data_in->Rate1_raw[AXIS_Z] / (SENSITIVITY_RATE1 * (float)AVG_FACTOR);
    
    data_out->Rate2[AXIS_X] = (float)data_in->Rate2_raw[AXIS_X] / (SENSITIVITY_RATE2 * (float)AVG_FACTOR);
    data_out->Rate2[AXIS_Y] = (float)data_in->Rate2_raw[AXIS_Y] / (SENSITIVITY_RATE2 * (float)AVG_FACTOR);
    data_out->Rate2[AXIS_Z] = (float)data_in->Rate2_raw[AXIS_Z] / (SENSITIVITY_RATE2 * (float)AVG_FACTOR);
    
    // Acceleration channels
    data_out->Acc1[AXIS_X]  = (float)data_in->Acc1_raw[AXIS_X] / (SENSITIVITY_ACC1 * (float)AVG_FACTOR);
    data_out->Acc1[AXIS_Y]  = (float)data_in->Acc1_raw[AXIS_Y] / (SENSITIVITY_ACC1 * (float)AVG_FACTOR);
    data_out->Acc1[AXIS_Z]  = (float)data_in->Acc1_raw[AXIS_Z] / (SENSITIVITY_ACC1 * (float)AVG_FACTOR);
    
    data_out->Acc2[AXIS_X]  = (float)data_in->Acc2_raw[AXIS_X] / (SENSITIVITY_ACC2 * (float)AVG_FACTOR);
    data_out->Acc2[AXIS_Y]  = (float)data_in->Acc2_raw[AXIS_Y] / (SENSITIVITY_ACC2 * (float)AVG_FACTOR);
    data_out->Acc2[AXIS_Z]  = (float)data_in->Acc2_raw[AXIS_Z] / (SENSITIVITY_ACC2 * (float)AVG_FACTOR);
    
    data_out->Acc3[AXIS_X]  = (float)data_in->Acc3_raw[AXIS_X] / (SENSITIVITY_ACC3 * (float)AVG_FACTOR);
    data_out->Acc3[AXIS_Y]  = (float)data_in->Acc3_raw[AXIS_Y] / (SENSITIVITY_ACC3 * (float)AVG_FACTOR);
    data_out->Acc3[AXIS_Z]  = (float)data_in->Acc3_raw[AXIS_Z] / (SENSITIVITY_ACC3 * (float)AVG_FACTOR);

    // Convert temperature and calculate average
    data_out->Temp = GET_TEMPERATURE((float)data_in->Temp_raw / (float)AVG_FACTOR);
        
}



bool SCH1_check_48bit_frame_error(uint64_t *data, int size)
{
    for (int i = 0; i < size; i++) 
    {
        uint64_t value = data[i];
        if (value & ERROR_FIELD_MASK)
            return true;
    }
    
    return false;
}


bool SCH1_testSPIStability(void)
{
    const int test_count = 10;
    uint16_t test_values[10];
    bool is_stable = true;
    
    printf("Testing SPI communication stability...\r\n");
    
   
    for (int i = 0; i < test_count; i++) {
        uint64_t response = SCH1_sendRequest(REQ_READ_STAT_SUM);
        test_values[i] = SPI48_DATA_UINT16(response);
        
       
        if (response & ERROR_FIELD_MASK) {
            printf("SPI test %d: CRC error detected\r\n", i);
            is_stable = false;
        }
        
        hw_delay(1);
    }
  
    uint16_t first_value = test_values[0];
    for (int i = 1; i < test_count; i++) {
        if (test_values[i] != first_value) {
            printf("SPI test: Value inconsistency detected\r\n");
            printf("  Expected: 0x%04X, Got: 0x%04X at test %d\r\n", first_value, test_values[i], i);
            is_stable = false;
        }
    }
    
    if (is_stable) {
        printf("SPI communication is stable. Consistent value: 0x%04X\r\n", first_value);
    } else {
        printf("SPI communication is unstable!\r\n");
    }
    
    return is_stable;
}

// å…¨å±€SPIç»Ÿè?¡å˜é‡?
static SCH1_spi_stats g_spi_stats = {0};

// é‡ç½®SPIç»Ÿè??
void SCH1_resetSPIStats(void)
{
    memset(&g_spi_stats, 0, sizeof(SCH1_spi_stats));
    printf("SPI statistics reset\r\n");
}

// æ›´æ–°SPIç»Ÿè??
void SCH1_updateSPIStats(bool success, uint32_t error_type)
{
    g_spi_stats.total_requests++;
    
    if (success) {
        g_spi_stats.successful_requests++;
        g_spi_stats.consecutive_errors = 0;
        g_spi_stats.last_error_type = 0;
    } else {
        g_spi_stats.consecutive_errors++;
        g_spi_stats.last_error_type = error_type;
        
        // æ›´æ–°æœ€å¤§è¿žç»?é”™è??æ¬¡æ•°
        if (g_spi_stats.consecutive_errors > g_spi_stats.max_consecutive_errors) {
            g_spi_stats.max_consecutive_errors = g_spi_stats.consecutive_errors;
        }
        
        // æ ¹æïúé”™è??ç±»åž‹æ›´æ–°å¯¹åº”è®¡æ•°å™?
        switch (error_type) {
            case 1: g_spi_stats.crc_errors++; break;
            case 2: g_spi_stats.frame_errors++; break;
            case 3: g_spi_stats.timeout_errors++; break;
            case 4: g_spi_stats.value_inconsistencies++; break;
        }
    }
    
    // è®¡ç®—é”™è??çŽ?
    if (g_spi_stats.total_requests > 0) {
        g_spi_stats.error_rate = (float)(g_spi_stats.total_requests - g_spi_stats.successful_requests) / 
                                (float)g_spi_stats.total_requests * 100.0f;
    }
}

// æ‰“å°SPIç»Ÿè?¡ä¿¡æ?
void SCH1_printSPIStats(void)
{
    printf("\r\n=== SPIé€šä¿¡ç»Ÿè?¡æŠ¥å‘? ===\r\n");
    printf("æ€»è?·æ±‚æ¬¡æ•°: %lu\r\n", g_spi_stats.total_requests);
    printf("æˆåŠŸè¯·æ±‚æ¬¡æ•°: %lu\r\n", g_spi_stats.successful_requests);
    printf("å¤±è´¥è¯·æ±‚æ¬¡æ•°: %lu\r\n", g_spi_stats.total_requests - g_spi_stats.successful_requests);
    printf("é”™è??çŽ?: %.2f%%\r\n", g_spi_stats.error_rate);
    printf("\r\n--- é”™è??ç±»åž‹ç»Ÿè?? ---\r\n");
    printf("CRCé”™è??: %lu\r\n", g_spi_stats.crc_errors);
    printf("å¸§é”™è¯?: %lu\r\n", g_spi_stats.frame_errors);
    printf("è¶…æ—¶é”™è??: %lu\r\n", g_spi_stats.timeout_errors);
    printf("æ•°å€¼ä¸ä¸€è‡?: %lu\r\n", g_spi_stats.value_inconsistencies);
    printf("\r\n--- è¿žç»­é”™è??ç»Ÿè?? ---\r\n");
    printf("å½“å‰è¿žç»­é”™è??: %lu\r\n", g_spi_stats.consecutive_errors);
    printf("æœ€å¤§è¿žç»?é”™è??: %lu\r\n", g_spi_stats.max_consecutive_errors);
    printf("æœ€åŽä¸€æ¬¡é”™è¯?ç±»åž‹: %lu\r\n", g_spi_stats.last_error_type);
    printf("========================================\r\n");
}

// èŽ·å–SPIç»Ÿè?¡ä¿¡æ?
void SCH1_getSPIStats(SCH1_spi_stats *stats)
{
    if (stats != NULL) {
        memcpy(stats, &g_spi_stats, sizeof(SCH1_spi_stats));
    }
}

// è¯¦ç»†çš„SPIç¨³å®šæ€§æµ‹è¯?
bool SCH1_testSPIStabilityDetailed(SCH1_spi_stats *stats)
{
    const int test_count = 20;  // å¢žåŠ æµ‹è¯•æ¬¡æ•°ä»¥èŽ·å¾—æ›´å‡†ç¡®çš„ç»Ÿè®?
    uint16_t test_values[20];
    bool is_stable = true;
    uint32_t consistent_count = 0;
    
    printf("=== è¯¦ç»†SPIç¨³å®šæ€§æµ‹è¯? ===\r\n");
    printf("æµ‹è¯•æ¬¡æ•°: %d\r\n", test_count);
    
    // é‡ç½®ç»Ÿè??
    SCH1_resetSPIStats();
    
    for (int i = 0; i < test_count; i++) {
        uint64_t response = SCH1_sendRequest(REQ_READ_STAT_SUM);
        test_values[i] = SPI48_DATA_UINT16(response);
        
        // æ£€æŸ?CRCé”™è??
        if (response & ERROR_FIELD_MASK) {
            printf("æµ‹è¯• %2d: CRCé”™è?? - å“åº”: 0x%012llX\r\n", i+1, response);
            SCH1_updateSPIStats(false, 1);  // CRCé”™è??
            is_stable = false;
        } else {
            SCH1_updateSPIStats(true, 0);   // æˆåŠŸ
        }
        
        hw_delay(2);  // å¢žåŠ å»¶æ—¶ä»¥ç¡®ä¿ç¨³å®šæ€?
    }
    
    // æ£€æŸ¥æ•°å€¼ä¸€è‡´æ€?
    uint16_t first_value = test_values[0];
    for (int i = 1; i < test_count; i++) {
        if (test_values[i] == first_value) {
            consistent_count++;
        } else {
            printf("æ•°å€¼ä¸ä¸€è‡?: æœŸæœ› 0x%04X, å®žé™… 0x%04X (æµ‹è¯• %d)\r\n", 
                   first_value, test_values[i], i+1);
            SCH1_updateSPIStats(false, 4);  // æ•°å€¼ä¸ä¸€è‡?
            is_stable = false;
        }
    }
    
    // è®¡ç®—ä¸€è‡´æ€§ç™¾åˆ†æ¯”
    float consistency_rate = (float)consistent_count / (float)(test_count - 1) * 100.0f;
    
    printf("\r\n--- æµ‹è¯•ç»“æžœ ---\r\n");
    printf("ä¸€è‡´æ€§çŽ‡: %.1f%% (%d/%d)\r\n", consistency_rate, consistent_count, test_count-1);
    printf("æœŸæœ›å€?: 0x%04X\r\n", first_value);
    
    if (is_stable) {
        printf("âœ? SPIé€šä¿¡ç¨³å®š\r\n");
    } else {
        printf("âœ? SPIé€šä¿¡ä¸ç¨³å®š\r\n");
    }
    
    // æ‰“å°è¯¦ç»†ç»Ÿè??
    SCH1_printSPIStats();
    
    // å¦‚æžœæä¾›äº†statså‚æ•°ï¼Œå?åˆ¶ç»Ÿè?¡ç»“æž?
    if (stats != NULL) {
        memcpy(stats, &g_spi_stats, sizeof(SCH1_spi_stats));
    }
    
    return is_stable;
}

// æ¼”ç¤ºSPIç»Ÿè?¡åŠŸèƒ½çš„ä½¿ç”¨
void SCH1_demoSPIStats(void)
{
    printf("\r\n=== SPIç»Ÿè?¡åŠŸèƒ½æ¼”ç¤? ===\r\n");
    
    // é‡ç½®ç»Ÿè??
    SCH1_resetSPIStats();
    
    // è¿›è?Œä¸€äº›æµ‹è¯•è?·æ±‚
    printf("è¿›è?Œæµ‹è¯•è?·æ±‚...\r\n");
    for (int i = 0; i < 20; i++) {
        SCH1_sendRequest(REQ_READ_STAT_SUM);
        hw_delay(1);
    }
    
    // æ‰“å°å½“å‰ç»Ÿè??
    printf("å½“å‰ç»Ÿè?¡ä¿¡æ?:\r\n");
    SCH1_printSPIStats();
    
    // è¿›è?Œè?¦ç»†ç¨³å®šæ€§æµ‹è¯?
    printf("\r\nå¼€å§‹è?¦ç»†ç¨³å®šæ€§æµ‹è¯?...\r\n");
    SCH1_spi_stats test_stats;
    bool is_stable = SCH1_testSPIStabilityDetailed(&test_stats);
    
    printf("\r\næµ‹è¯•å®Œæˆ! ç¨³å®šæ€?: %s\r\n", is_stable ? "ç¨³å®š" : "ä¸ç¨³å®?");
}
