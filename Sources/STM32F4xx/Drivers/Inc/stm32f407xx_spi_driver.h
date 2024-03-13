/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jan 21, 2024
 *      Author: muhammed
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_


#include "stm32f407xx.h"


/*
 * SPI Konfigürasyonu İçin Gerekli Veri Yapısı
 */
typedef struct
{
	uint8_t SPI_DeviceMode; 		// Cihaz Modu @SPI_Modes
	uint8_t SPI_BusConfig; 			// Haberleşme Türü @SPI_BusConfig
	uint8_t SPI_DFF;				// Veri Boyutu @SPI_DataFrame
	uint8_t SPI_CPOL;				// Saat Sinyali Başlangıç Konumu ( Clock Polarity ) @SPI_ClockPol
	uint8_t SPI_CPHA;				// Saat Sinyal Fazı ( Clock Phase ) @SPI_ClockPha
	uint8_t SPI_SSM;				// Yazılımsal Slave Yönetimi ( Software Slave Management ) @SPI_SoftwareSlaveMng
	uint8_t SPI_Speed;				// Haberleşme Hızı ( SCLK ) @SPI_ClockSpeed
}SPI_Config_t;


/*
 * SPI Handle Yapısı
 */
typedef struct
{
	// SPI peripheral başlangıç adresini tutan pointer
	SPI_RegDef_t *pSPIx;

	// SPI peripheral kullanıcı konfigürasyonuu sağlayan değişken
	SPI_Config_t SPI_Config;

}SPI_Handle_t;


/*
 * @SPI_Modes
 * SPI cihaz modu makroları
 */
#define SPI_DEVICE_MODE_MASTER					1
#define SPI_DEVICE_MODE_SLAVE					0


/*
 * @SPI_BusConfig
 * SPI haberleşme türü makroları
 */
#define SPI_BUS_CONFIG_FULL_D					0
#define SPI_BUS_CONFIG_HALF_D_RX				1
#define SPI_BUS_CONFIG_HALF_D_TX				2
#define SPI_BUS_CONFIG_SIMP_RXONLY				3
#define SPI_BUS_CONFIG_SIMP_TXONLY				4
// Simplex-TxOnly için full-duplex moddan Rx hattını kaldırmak yeterli.

/*
 * @SPI_DataFrame
 * SPI veri biçimi makroları
 */
#define SPI_DFF_8BITS							0
#define SPI_DFF_16BITS							1

/*
 * @SPI_ClockPol
 * SPI saat sinyali başlangıç durumu makroları
 */
#define SPI_CPOL_LOW							0
#define SPI_CPOL_HIGH							1


/*
 * @SPI_ClockPha
 * SPI saat sinyali fazı makroları
 */
#define SPI_CPHA_FIRST							0
#define SPI_CPHA_SCND							1


/*
 * @SPI_SoftwareSlaveMng
 * SPI yazılımsal slave yönetimi makroları
 */
#define SPI_SSM_EN								1
#define SPI_SSM_DI								0


/*
 * @SPI_ClockSpeed
 * SPI haberleşme hızı makroları
 */
#define SPI_SCLK_SPEED_DIV2						0
#define SPI_SCLK_SPEED_DIV4						1
#define SPI_SCLK_SPEED_DIV8						2
#define SPI_SCLK_SPEED_DIV16					3
#define SPI_SCLK_SPEED_DIV32					4
#define SPI_SCLK_SPEED_DIV64					5
#define SPI_SCLK_SPEED_DIV128					6
#define SPI_SCLK_SPEED_DIV256					7

/*
 * SPI birimi status flag tanımlamaları
 */
#define SPI_FLAG_RXNE							( 1 << SPI_SR_RXNE )
#define SPI_FLAG_TXE							( 1 << SPI_SR_TXE )
#define SPI_FLAG_CRCERR							( 1 << SPI_SR_CRCERR )
#define SPI_FLAG_MODF							( 1 << SPI_SR_MODF )
#define SPI_FLAG_BSY							( 1 << SPI_SR_BSY )
#define SPI_FLAG_FRE							( 1 << SPI_SR_FRE ) // Bu bayrak SPI çevresel birimi TI Slave Modda çalıştığında kullanılır.


/***********************************************************************
 * 			Bu driver dosyası tarafından desteklenen API'lar
 * 	Daha fazla bilgi için fonksiyon açıklamalarına bak.
 ***********************************************************************/

/*
 * Peripheral Clock Sinyali Kurulumu
 */
void SPI_PeriClkControl( SPI_RegDef_t *pSPIx, uint8_t EnOrDi );


/*
 * Başlatma  ve Sonlandırma
 */
void SPI_Init( SPI_Handle_t *pSPIHandle );
void SPI_DeInit( SPI_RegDef_t *pSPIx );


/*
 * Veri Alma ve Veri Yollama
 */
void SPI_SendData( SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint32_t dataLen );
uint8_t* SPI_ReceiveData( SPI_RegDef_t *pSPIx, uint8_t *pRxBuff, uint32_t dataLen );


/*
 * SPI Interrupt Kurulumu ve Interrupt İşleyici
 */
void SPI_IRQ_Interrupt_Config( uint8_t IRQNumber, uint8_t EnOrDi );
void SPI_IRQ_PriorityConfig( uint8_t IRQNumber, uint8_t IRQPriority );
void SPI_IRQHandling( SPI_Handle_t *pSPIHandle );


/*
 * Diğer Peripheral Kontrol Arayüz Fonksiyonları [ Sonradan eklenebilir! ]
 */
void SPI_PeriControl( SPI_RegDef_t *pSPIx, uint8_t enOrDi );
uint8_t SPI_GetFlagStatus( SPI_RegDef_t *pSPIx, uint16_t flagName );
//void SPI_SSI_Config( SPI_RegDef_t *pSPIx, uint8_t enOrDi );
//void SPI_SSOE_Config( SPI_RegDef_t *pSPIx, uint8_t enOrDi );


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
