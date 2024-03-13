/*
 * SPI_sendData_ArduinoExample.c
 *
 *  Created on: Feb 27, 2024
 *      Author: muhammed
 */
#include "stm32f407xx.h"

void delay( void );
void GPIO_Btn_Init( void );
void SPI2_GPIO_Init( void );
void SPI2_Init( void );

/*
 * Master cihaz üzerindeki butona her basıldığında oluşturulan veri slave cihaza iletilir.
 * Master - STM32
 * Slave - Arduino UNO
 * Alınan veri Arduino IDE kullanılarak Serial Port üzerinden görüntülenir.
 */

/*
 * SPI modu Full-Duplex
 * STM32-> MasterMode, Arduino-> SlaveMode
 * DFF = 0 ( 8-bit )
 * Veri iletimi sırasında donanımsal slave cihaz yönetimi ( hardware slave select management ) kullanıldı.
 * Clock sinyali olarak HSI kullanıldı.( ~16Mhz )
 * SCLK hızı 2MHz olarak belirlendi.
 *
 * Bu uygulamada master cihaz herhangi bir veri almayacağı için MISO pini kullanılmayabilir.
 *
 * !Slave cihaza ilk olarak gönderilecek verinin boyutu bildirilmiştir.
 */

/*
 * SPI2/MOSI - PB15
 * SPI2/SCLK - PB13
 * SPI2/MISO - PB14
 * SPI2/NSS  - PB9
 *
 */

GPIO_Handle_t gpio_spi2, gpio_button;
SPI_Handle_t spi2;

char data[] = "Merhaba Dunya!";
uint8_t len = sizeof( data );
uint8_t btnState = 0;

int main( void )
{
	// GPIO buton ayarları
	GPIO_Btn_Init();

	// SPI2 GPIO pin ayarları konfigürasyonu
	SPI2_GPIO_Init();

	// SPI2 konfigürasyonu
	SPI2_Init();

	while( 1 )
	{
		btnState = GPIO_ReadFromInputPin( gpio_button.pGPIOx, gpio_button.GPIO_PinConfig.GPIO_PinNumber );

		if( btnState )
		{
			delay();

			// SPI2 peripheralını başlat. NSS pini otomatik olarak lojik low seviyesine çekilir.
			SPI_PeriControl( spi2.pSPIx , ENABLE );

			// Tx buffer boşalana kadar bekle. Sonradan eklendi.
			while( FLAG_RESET == SPI_GetFlagStatus( spi2.pSPIx , SPI_FLAG_TXE  ));

			// İlk olarak veri boyutunu yolla
			SPI_SendData( spi2.pSPIx , &len, 1 );

			// Tx buffer boşalana kadar bekle. Sonradan eklendi.
			while( FLAG_RESET == SPI_GetFlagStatus( spi2.pSPIx , SPI_FLAG_TXE  ));

			// Veriyi yolla
			SPI_SendData( spi2.pSPIx , (uint8_t*)data, len );

			// Tx buffer boşalana kadar ( TXE = 1 olana kadar ) ve BSY = 0 olana kadar bekle
			while( ( SPI_GetFlagStatus( spi2.pSPIx , SPI_FLAG_TXE) == FLAG_RESET ) || ( SPI_GetFlagStatus( spi2.pSPIx , SPI_FLAG_BSY ) == FLAG_SET ));

			// SPI2 peripheral'ını durdur. NSS pini otomatik olarak lojik high seviysine çekilir.
			SPI_PeriControl( spi2.pSPIx , DISABLE );

		}
	}

	return 0;
}


void GPIO_Btn_Init( void )
{
	gpio_button.pGPIOx = GPIOA;
	gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	gpio_button.GPIO_PinConfig.GPIO_Pin_PUPD = GPIO_PIN_NO_PUPD;

	GPIO_Init( &gpio_button );
}


void SPI2_Init( void )
{
	spi2.pSPIx = SPI2;
	spi2.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FULL_D;
	spi2.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST;
	spi2.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	spi2.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	spi2.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi2.SPI_Config.SPI_SSM = SPI_SSM_DI;
	spi2.SPI_Config.SPI_Speed = SPI_SCLK_SPEED_DIV8;

	// Belirlenen ayarları SPI çevresel birimine uygula
	SPI_Init( &spi2 );
}


void SPI2_GPIO_Init( void )
{
	gpio_spi2.pGPIOx = GPIOB;
	gpio_spi2.GPIO_PinConfig.GPIO_Pin_O_Type = GPIO_OTYPE_PP;
	gpio_spi2.GPIO_PinConfig.GPIO_Pin_PUPD = GPIO_PIN_NO_PUPD;
	gpio_spi2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	gpio_spi2.GPIO_PinConfig.GPIO_PinAltFun = GPIO_AF_5;

	// SPI2/MOSI
	gpio_spi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init( &gpio_spi2 );

	// SPI2/SCLK
	gpio_spi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init( &gpio_spi2 );

	// SPI2/MISO Kullanılmadı!!!
	gpio_spi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init( &gpio_spi2 );

	// SPI2/NSS
	gpio_spi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init( &gpio_spi2 );
}


void delay( void )
{
	for( int i = 0; i < 500000; i++ );
}
