/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Jan 21, 2024
 *      Author: muhammed
 */

#include "stm32f407xx_spi_driver.h"

/*********************************************************************************************
 *
 *@fn			 	-	SPI_PeriClkControl
 *
 *@özet			 	-	SPI peripheral'ına ait clock sinyali kontrolünü sağlar.
 *
 *@param[0]		 	-	SPI peripheral başlangıç adresi pointer'ı
 *@param[1]			-	Clock sinyal kaynağı etkinleştir ya da devre dışı bırak
 *
 *@return			-	none
 *
 *@NOT				-	EnOrDi değişkeni için ENABLE ya da DISABLE makroları kullanılmalı
 *
 **********************************************************************************************/
void SPI_PeriClkControl( SPI_RegDef_t *pSPIx, uint8_t EnOrDi )
{
	if( EnOrDi == ENABLE )
	{
		if( pSPIx == SPI1 )
		{
			SPI1_CLK_EN();
		}else if( pSPIx == SPI2 )
		{
			SPI2_CLK_EN();
		}else if( pSPIx == SPI3 )
		{
			SPI3_CLK_EN();
		}
	}else
	{
		if( pSPIx == SPI1 )
		{
			SPI1_CLK_DI();
		}else if( pSPIx == SPI2 )
		{
			SPI2_CLK_DI();
		}else if( pSPIx == SPI3 )
		{
			SPI3_CLK_DI();
		}
	}
}


/*********************************************************************************************
 *
 *@fn			 	-	SPI_Init
 *
 *@özet			 	-	SPI peripheral'ına ait belirlenen ayarlamaları yaparak haberleşmeyi başlatır.
 *
 *@param[0]		 	-	SPI İşleyici( Handle ) veri yapısı kullanılarak oluşturulmuş pointer
 *
 *@return			-	none
 *
 *@NOT				-	none
 *
 **********************************************************************************************/
void SPI_Init( SPI_Handle_t *pSPIHandle )
{
	// Peripheral saat sinyalini aktif et.
	SPI_PeriClkControl(pSPIHandle->pSPIx, ENABLE);

	/*
	 * Desteklenen modlar : Full-Duplex, Half-Duplex, Simplex-TxOnly, Simplex-RxOnly
	 *
	 * Multi-master modu eklenmedi. Sonradan eklenebilir.
	 */

	uint32_t tempReg = 0;


	// 1. Cihaz modunu ayarla. Master/Slave
	tempReg |= ( pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR );


	// 2. Cihaz master ise haberleşme hızını belirle.
	if( pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER )
	{
		tempReg |= ( pSPIHandle->SPI_Config.SPI_Speed << SPI_CR1_BR_START );
	}


	// 3. CPOL ve CPHA ayarla.
	tempReg |= ( pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA );
	tempReg |= ( pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL );


	// 4. Veri boyutunu ayarla. [DFF]
	tempReg |= ( pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF );


	// 5. Veri biçimini ayarla. [LSBFIRST/MSBFIRST]. İleride eklenecek olursa SPI_Config_t güncelle.

	// 6. Haberleşme Modunu ayarla. [Full-Duplex, Half-Duplex, Simplex_Tx, Simplex_Rx]
	if( (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FULL_D) || (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMP_TXONLY) )
	{
		tempReg &= ~( 1 << SPI_CR1_BIDIMODE );
		tempReg &= ~( 1 << SPI_CR1_RXONLY );
		// Simplex_Tx konfigürasyonu seçiliyse Full-Duplex konfigürasyonundan cihaz master ise MISO pinini, slave ise MOSI pinini devre dışı bırakmak yeterli.

	}else if( pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HALF_D_TX )
	{
		// BIDIMODE = 1 BIDIOE = 1(Tx)
		tempReg |= ( 1 << SPI_CR1_BIDIMODE );
		tempReg |= ( 1 << SPI_CR1_BIDIOE );

	}else if ( pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HALF_D_RX )
	{
		// BIDIMODE = 1 BIDIOE = 0(Rx)
		tempReg |= ( 1 << SPI_CR1_BIDIMODE );
		tempReg &= ~( 1 << SPI_CR1_BIDIOE );
	}else if( pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMP_RXONLY )
	{
		tempReg &= ~( 1 << SPI_CR1_BIDIMODE );
		tempReg |= ( 1 << SPI_CR1_RXONLY );

		//Cihaz master ise MOSI pinini, slave ise MISO pinini devre dışı bırak.
	}


	// 7. NSS ayarla. Multi-Master Modu dahil edilmedi!!!!!!!!!!!!!!!!!!!
	if( pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER )
	{
		if( pSPIHandle->SPI_Config.SPI_SSM == SPI_SSM_EN )
		{
			// SSM = 1, SSI = 1 (giriş için)
			tempReg |= ( 1 << SPI_CR1_SSM );
			tempReg |= ( 1 << SPI_CR1_SSI );

		}else
		{
			/*
			 * Peripheral başlatıldığında NSS pini master cihaz tarafından otomatik olarak lojik low seviyesine çekilir.
			 * Peripheral sonladırıldığında ise yine otomatik olarak lojik high seviyesine çekilir.
			 * Bu özellik için SSOE = 1 olmalıdır.
			 */
			pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE );


		}

	}else if( pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_SLAVE )
	{
		if( pSPIHandle->SPI_Config.SPI_SSM == SPI_SSM_EN )
		{
			// SSM = 1, SSI = 0
			tempReg |= ( 1 << SPI_CR1_SSM );
			tempReg &= ~( 1 << SPI_CR1_SSI );
		}else{
			// NSS pini master cihaz tarafından yönetilecek.
		}
	}


	// 8. SPI aktif et. SPE = 1 Ayrı bir fonksiyon olarak taşındı!
	//tempReg |= ( 1 << SPI_CR1_SPE );


	// Yapılan değişiklikleri register'a uygula.
	pSPIHandle->pSPIx->CR1 = tempReg;
	tempReg = 0;

}


/*********************************************************************************************
 *
 *@fn			 	-	SPI_PeriControl
 *
 *@özet			 	-	Başlangıç adresi verilen SPI peripheralını aktif eder ya da devre dışı bırakır.
 *
 *@param[0]		 	-	SPI peripheral başlangıç adresi pointerı
 *@param[0]		 	-	ENABLE ya da DISABLE makrosu
 *
 *@return			-	none
 *
 *@NOT				-	enOrDi değişkeni için mutlaka ENABLE ya da DISABLE makroları kullanılmalı
 *@NOT				-	SPI peripheralı aktif edilecekse bu fonksiyon KESİNLİKLE SPI_Init fonksiyonundan sonra kullanılmalıdır.
 *
 **********************************************************************************************/
void SPI_PeriControl( SPI_RegDef_t *pSPIx, uint8_t enOrDi )
{
	if( enOrDi == ENABLE )
	{
		// SPIx aktif et.
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE );

	}else
	{
		// SPIx devre dışı bırak.
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE );
	}
}


/*********************************************************************************************
 *
 *@fn			 	-	SPI_SSI_Config
 *
 *@özet			 	-	SSI bitini ayarlar.
 *
 *@param[0]		 	-	SPI peripheral başlangıç adresi pointerı
 *@param[0]		 	-	ENABLE ya da DISABLE makrosu
 *
 *@return			-	none
 *
 *@NOT				-	enOrDi değişkeni için mutlaka ENABLE ya da DISABLE makroları kullanılmalı
 *
 **********************************************************************************************/
/*void SPI_SSI_Config( SPI_RegDef_t *pSPIx, uint8_t enOrDi )
{
	if( enOrDi == ENABLE )
	{
		// SSI pini lojik high
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI );

	}else
	{
		// SSI pini lojik low ( multimaster konfigürasyonu için )
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI );
	}
}
*/

/*********************************************************************************************
 *
 *@fn			 	-	SPI_SSOE_Config
 *
 *@özet			 	-	SSOE bitini ayarlar.
 *
 *@param[0]		 	-	SPI peripheral başlangıç adresi pointerı
 *@param[0]		 	-	ENABLE ya da DISABLE makrosu
 *
 *@return			-	none
 *
 *@NOT				-	enOrDi değişkeni için mutlaka ENABLE ya da DISABLE makroları kullanılmalı
 *
 **********************************************************************************************/
/*void SPI_SSOE_Config( SPI_RegDef_t *pSPIx, uint8_t enOrDi )
{
	if( enOrDi == ENABLE )
	{
		// SSOE pini lojik high
		pSPIx->CR1 |= ( 1 << SPI_CR2_SSOE );

	}else
	{
		// SSOE pini lojik low
		pSPIx->CR1 &= ~( 1 << SPI_CR2_SSOE );
	}
}
*/

/*********************************************************************************************
 *
 *@fn			 	-	SPI_DeInit
 *
 *@özet			 	-	Başlangıç adresi verilen SPI peripheral'ını resetler.
 *
 *@param[0]		 	-	SPI peripheral başlangıç adresi pointerı
 *
 *@return			-	none
 *
 *@NOT				-	none
 *
 **********************************************************************************************/
void SPI_DeInit( SPI_RegDef_t *pSPIx )
{
	if( pSPIx == SPI1 )
	{
		SPI1_RESET();
	}else if( pSPIx == SPI2 )
	{
		SPI2_RESET();
	}else if( pSPIx == SPI3 )
	{
		SPI3_RESET();
	}
}


/*********************************************************************************************
 *
 *@fn			 	-	SPI_GetFlagStatus
 *
 *@özet			 	-	SPI peripheralına ait istenen bayrak durumlarını döndürür.
 *
 *@param[0]		 	-	SPI peripheral başlangıç adresi pointer'ı
 *@param[1]			-	Bayrak makrosu
 *
 *@return			-	none
 *
 *@NOT				-	Bu fonksiyon kaynak kod dosyasında kullanılamaz. flagName değişkeni için stm32f407xx_spi_driver.h dosyasındaki tanımlı makroları kullan!
 *
 **********************************************************************************************/
uint8_t SPI_GetFlagStatus( SPI_RegDef_t *pSPIx, uint16_t flagName )
{
	// (( pSPIx->SR ) & ( 1 << SPI_SR_TXE ))
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	if( pSPIx->SR & flagName )
	{
		return FLAG_SET;
	}else
	{
		return FLAG_RESET;
	}

}


/*********************************************************************************************
 *
 *@fn			 	-	SPI_SendData
 *
 *@özet			 	-	SPI peripheral'ı üzerinden haberleşme protokolünü kullanarak uzunluğu belirtilmiş veriyi gönderir.
 *
 *@param[0]		 	-	SPI peripheral başlangıç adresi pointer'ı
 *@param[1]			-	Gönderilecek veri
 *@param[2]			-	Veri uzunluğu/boyutu
 *
 *@return			-	none
 *
 *@NOT				-	Bu fonksiyon blocking call özelliğindedir. Yani işlem bitene kadar fonksiyon sonlanmaz.
 *
 **********************************************************************************************/
void SPI_SendData( SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint32_t dataLen )
{
	//MSBFIRST , LSBFIRST eklenecek olursa düzenle.

	while( dataLen > 0 )
	{
		// TX buffer'ı boşaltılana kadar bekle.

		while( SPI_GetFlagStatus( pSPIx, SPI_FLAG_TXE ) == FLAG_RESET );

		// DFF kontrol et.
		if( pSPIx->CR1 & ( 1 << SPI_CR1_DFF ) )
		{
			// 16 bit
			// DR register'ına 2 bytelık veri at, buffer addresini 2 arttır, dataLen' i 2 azalt.
			if( dataLen == 1 )
			{
				// dataLen = 1 olduysa 1 byte veri at, buffer adresini 1 arttır, dataLen'i 1 azalt.
				pSPIx->DR = *pTxBuff;
				pTxBuff++;
				dataLen--;

			}else
			{
				//pSPIx->DR = ( *pTxBuff << 8 ) + ( *(pTxBuff + 1) )
				pSPIx->DR = *(( uint16_t* )pTxBuff );
				//pTxBuff += 2;
				( uint16_t* )pTxBuff++;
				dataLen -= 2;
			}

		}else
		{
			// 8 bit
			// DR register'ına 1 byte veri at, buffer adresini 1 arttır, dataLen'i 1 azalt.
			pSPIx->DR = *pTxBuff;
			pTxBuff++;
			dataLen--;
		}
	}
}


/*********************************************************************************************
 *
 *@fn			 	-	SPI_ReceiveData
 *
 *@özet			 	-	SPI peripheral'ı üzerinden haberleşme protokolünü kullanarak uzunluğu belirlenen veriyi alır.
 *
 *@param[0]		 	-	SPI peripheral başlangıç adresi pointer'ı
 *@param[1]			-	Alınacak veri
 *@param[2]			-	Veri uzunluğu/boyutu
 *
 *@return			-	uint8_t*
 *
 *@NOT				-	none
 *
 **********************************************************************************************/
uint8_t* SPI_ReceiveData( SPI_RegDef_t *pSPIx, uint8_t *pRxBuff, uint32_t dataLen );


/*********************************************************************************************
 *
 *@fn			 	-	SPI_IRQ_Interrupt_Config
 *
 *@özet			 	-	Peripheral tarafında ayarlanan interrupt'ın işlemci tarafında aktif edilmesini sağlar.
 *
 *@param[0]		 	-	Interrupt numarası
 *@param[1]			-	Interrupt'ı etkinleştir ya da devre dışı bırak
 *
 *@return			-	none
 *
 *@NOT				-	EnOrDi değişkeni için ENABLE ya da DISABLE makroları kullanılmalı
 *
 **********************************************************************************************/
void SPI_IRQ_Interrupt_Config( uint8_t IRQNumber, uint8_t EnOrDi );


/*********************************************************************************************
 *
 *@fn			 	-	SPI_IRQ_PriorityConfig
 *
 *@özet			 	-	Belirtilen IRQ numarasına belirlenen öncelik değerini[0-15] atar. Öncelik değeri düşük olan interrupt daha yüksek önceliklidir. Örneğin 7 > 10
 *
 *@param[0]		 	-	Interrupt numarası
 *@param[1]			-	Interrupt öncelik değeri[0-15]
 *
 *@return			-	none
 *
 *@NOT				-	IRQPriority değeri 0-15 arasında olmalıdır!
 *
 **********************************************************************************************/
void SPI_IRQ_PriorityConfig( uint8_t IRQNumber, uint8_t IRQPriority );


/*********************************************************************************************
 *
 *@fn			 	-	SPI_IRQHandling
 *
 *@özet			 	-	Belirlenen interrupt'ı işler.
 *
 *@param[0]		 	-	SPI İşleyici( Handle ) veri yapısı kullanılarak oluşturulmuş pointer
 *
 *@return			-	none
 *
 *@NOT				-	EnOrDi değişkeni için ENABLE ya da DISABLE makroları kullanılmalı
 *
 **********************************************************************************************/
void SPI_IRQHandling( SPI_Handle_t *pSPIHandle );



