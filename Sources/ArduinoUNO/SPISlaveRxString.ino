/* SPI Slave Demo
   SPI pin numaraları:
   SCK   13  // Serial Clock.
   MISO  12  // Master In Slave Out.
   MOSI  11  // Master Out Slave In.
   SS    10  // Slave Select . Arduino SPI pinleri sadece SS pini master tarafından lojik sıfır seviyesine çekildiğinde yanıt verir.
*/

#include <SPI.h>
#include<stdint.h>

#define SPI_SCK 13
#define SPI_MISO 12
#define SPI_MOSI 11
#define SPI_SS 10


//Initialize SPI slave.
void SPI_SlaveInit(void)
{
  // SPI pinlerini ata.
  pinMode(SPI_SCK, INPUT);
  pinMode(SPI_MOSI, INPUT);
  pinMode(SPI_MISO, OUTPUT);
  pinMode(SPI_SS, INPUT);

  // SPI protokolünü slave olarak aktif et.
  SPCR = (1 << SPE);
}


//Bu fonksiyon SPDR değerini döndrürür.
uint8_t SPI_SlaveReceive(void)
{
  /* Alımın tamamlanmasını bekle */
  while (!(SPSR & (1 << SPIF)));

  /* Data Register'ını döndür */
  return SPDR;
}


void setup()
{
  // Seri haberleşmeyi başlat.
  Serial.begin(9600);

  // SPI slave cihazını başlat.
  SPI_SlaveInit();
  Serial.println("Slave Baslatildi!");
}


void loop()
{
  uint32_t i = 0;
  uint32_t dataLen = 0;

  Serial.println("Slave cihaz, SS pininin lojik low seviyesine cekilmesini bekliyor...");
  while (digitalRead(SS));

  dataLen = ( uint32_t )SPI_SlaveReceive();
  char dataBuff[dataLen];

  for (i = 0 ; i < dataLen ; i++ )
  {
    dataBuff[i] =  (char)SPI_SlaveReceive();
  }

  Serial.println("Alındı:");
  Serial.println(dataBuff);
  Serial.print("Boyut:");
  Serial.println(dataLen);
}
