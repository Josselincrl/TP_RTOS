#include "main.h"
#include "spi.h"

int ACC_READ(uint8_t address, uint8_t* p_data){
	address |= 0x80;
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET); //ChipSelect a 0
	HAL_SPI_Transmit(&hspi2, &address, 1, HAL_MAX_DELAY); //On transmet a l'adresse 0x00
	HAL_SPI_Receive(&hspi2, p_data, 1, HAL_MAX_DELAY); // On recoit en SPI de la data
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET); //On remet le chipSelect a 1
	return 0;
	}

int ACC_TRANSMIT(uint8_t address, uint8_t p_data  ){
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET); //ChipSelect a 0
	HAL_SPI_Transmit(&hspi2, &address, 1, HAL_MAX_DELAY); //On transmet l'adresse
	HAL_SPI_Transmit(&hspi2, &p_data, 1, HAL_MAX_DELAY); //On transmet un msg
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET); //On remet le chipSelect a 1
	return 0;
	}

void ACC_READ_Mult(uint8_t* p_data, uint8_t address, uint16_t size){
	address |= 0x80; //Active le bit de lecture
	address |= 0b01000000; //Active le 'multiple bit' afin de pouvoir lire toute la data de chaque axe
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET); //ChipSelect a 0
	HAL_SPI_Transmit(&hspi2, &address, 1, HAL_MAX_DELAY); //On transmet a l'adresse
	HAL_SPI_Receive(&hspi2, p_data, size, HAL_MAX_DELAY); // On recoit en SPI de la data sur size octets (ici 6)
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET); //On remet le chipSelect a 1
}
