/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          zf_driver_spi
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-9       pudding            first version
********************************************************************************************************************/
#ifndef _zf_driver_spi_h_
#define _zf_driver_spi_h_

#include "zf_common_typedef.h"

typedef enum        				// SPIģ���
{
    SPI_0,
    SPI_1,
    SPI_2,
    SPI_3,
    SPI_4,
}spi_index_enum;

typedef enum        				// ö�� SPI ģʽ ��ö�ٶ��岻�����û��޸�
{
    SPI_MODE0,
    SPI_MODE1,
    SPI_MODE2,
    SPI_MODE3,
}spi_mode_enum;

typedef enum                           	// ö��SPI CLK���� ��ö�ٶ��岻�����û��޸�
{
    SPI0_CLK_P02_2,                		// SPI0 CLK ���ſ�ѡ��Χ

    SPI1_CLK_P12_2,             		// SPI1 CLK ���ſ�ѡ��Χ

    SPI2_CLK_P15_2,				// SPI2 CLK ���ſ�ѡ��Χ

    SPI3_CLK_P03_2,				// SPI3 CLK ���ſ�ѡ��Χ

    SPI4_CLK_P07_2,				// SPI4 CLK ���ſ�ѡ��Χ (SCB5)
}spi_clk_pin_enum;

typedef enum                       		// ö��SPI MOSI���� ��ö�ٶ��岻�����û��޸�
{
    SPI0_MOSI_P02_1,                    	// SPI0 MOSI���ſ�ѡ��Χ

    SPI1_MOSI_P12_1,                  	        // SPI1 MOSI���ſ�ѡ��Χ

    SPI2_MOSI_P15_1,                  	        // SPI2 MOSI���ſ�ѡ��Χ

    SPI3_MOSI_P03_1,                  	        // SPI3 MOSI���ſ�ѡ��Χ

    SPI4_MOSI_P07_1,                  	        // SPI4 MOSI���ſ�ѡ��Χ (SCB5)
}spi_mosi_pin_enum;

typedef enum             			// ö��SPI MISO���� ��ö�ٶ��岻�����û��޸�
{
    SPI0_MISO_P02_0,	                 	// SPI0 MISO���ſ�ѡ��Χ

    SPI1_MISO_P12_0,              		// SPI1 MISO���ſ�ѡ��Χ

    SPI2_MISO_P15_0, 				// SPI2 MISO���ſ�ѡ��Χ

    SPI3_MISO_P03_0, 				// SPI3 MISO���ſ�ѡ��Χ

    SPI4_MISO_P07_0, 				// SPI4 MISO���ſ�ѡ��Χ (SCB5)

    SPI_MISO_NULL,
}spi_miso_pin_enum;

typedef enum                       		// ö��SPI CS���� ��ö�ٶ��岻�����û��޸�
{
    SPI0_CS0_P02_3,	        		// SPI0 CS0 ���ſ�ѡ��Χ
    SPI0_CS1_P02_4,

    SPI1_CS0_P12_3,	               		// SPI1 CS0 ���ſ�ѡ��Χ
    SPI1_CS1_P12_4,

    SPI2_CS0_P15_3,                 	        // SPI2 CS0 ���ſ�ѡ��Χ
    SPI2_CS3_P05_1,

    SPI3_CS0_P03_3,                             // SPI3 CS0 ���ſ�ѡ��Χ
    SPI3_CS1_P03_4,

    SPI4_CS0_P07_3,                             // SPI4 CS0 ���ſ�ѡ��Χ (SCB5)
    SPI4_CS1_P07_4,
    SPI4_CS2_P07_5,

    SPI_CS_NULL,
}spi_cs_pin_enum;

//====================================================SPI ��������====================================================
void        spi_write_8bit                  (spi_index_enum spi_n, const uint8 data);
void        spi_write_8bit_array            (spi_index_enum spi_n, const uint8 *data, uint32 len);

void        spi_write_16bit                 (spi_index_enum spi_n, const uint16 data);
void        spi_write_16bit_array           (spi_index_enum spi_n, const uint16 *data, uint32 len);

void        spi_write_8bit_register         (spi_index_enum spi_n, const uint8 register_name, const uint8 data);
void        spi_write_8bit_registers        (spi_index_enum spi_n, const uint8 register_name, const uint8 *data, uint32 len);

void        spi_write_16bit_register        (spi_index_enum spi_n, const uint16 register_name, const uint16 data);
void        spi_write_16bit_registers       (spi_index_enum spi_n, const uint16 register_name, const uint16 *data, uint32 len);

uint8       spi_read_8bit                   (spi_index_enum spi_n);
void        spi_read_8bit_array             (spi_index_enum spi_n, uint8 *data, uint32 len);

uint16      spi_read_16bit                  (spi_index_enum spi_n);
void        spi_read_16bit_array            (spi_index_enum spi_n, uint16 *data, uint32 len);

uint8       spi_read_8bit_register          (spi_index_enum spi_n, const uint8 register_name);
void        spi_read_8bit_registers         (spi_index_enum spi_n, const uint8 register_name, uint8 *data, uint32 len);

uint16      spi_read_16bit_register         (spi_index_enum spi_n, const uint16 register_name);
void        spi_read_16bit_registers        (spi_index_enum spi_n, const uint16 register_name, uint16 *data, uint32 len);

void        spi_transfer_8bit               (spi_index_enum spi_n, const uint8 *write_buffer, uint8 *read_buffer, uint32 len);
void        spi_transfer_16bit              (spi_index_enum spi_n, const uint16 *write_buffer, uint16 *read_buffer, uint32 len);

void        spi_init                        (spi_index_enum spi_n, spi_mode_enum mode, uint32 baud, spi_clk_pin_enum sck_pin, spi_mosi_pin_enum mosi_pin, spi_miso_pin_enum miso_pin, spi_cs_pin_enum cs_pin);
//====================================================SPI ��������====================================================

#endif
