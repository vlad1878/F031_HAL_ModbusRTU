#include "ModbusRTU_lib.h"

uint8_t ModbusRTU_tx_buffer[ModbusRTU_TX_BUFFER_SIZE] = {0, };
uint8_t ModbusRTU_rx_buffer[ModbusRTU_RX_BUFFER_SIZE] = {0, };

extern UART_HandleTypeDef huart1;

uint16_t ModbusRTU_CRC16_Calculate(uint8_t *data, uint8_t lenght, uint8_t byte_order) {
	uint16_t crc = 0xFFFF;
	while (lenght--) {
		crc ^= *data++;
		for (int i = 0; i < 8; i++) {
			if (crc & 0x01) {
				crc = (crc >> 1u) ^ 0xA001;
			} else {
				crc = crc >> 1u;
			}
		}
	}
	switch (byte_order) {
	case (0): //1234(младшим регистром вперед, младшим байтом вперед),
		break;
	case (1): //3412(старшим регистром вперед, младшим байтом вперед),
		crc = (crc << 8u) | (crc >> 8u);
		break;
	case (2): //2143(младшим регистром вперед, старшим байтом вперед),
		crc = (((crc >> 8u) & 0x0F) << 12u) | ((crc >> 12u) << 8u) | ((crc << 12u) << 4u) | ((crc >> 4u) & 0x00F);
		break;
	case (3): //4321(старшим регистром вперед, старшим байтом вперед).
		crc = (((crc >> 8u) & 0x0F) << 4u) | (crc >> 12u) | ((crc << 12u) << 12u) | (((crc >> 4u) & 0x00F) << 8u);
		break;
	}
	return crc;
}

bool ModbusRTU_CRC16_Check(UART_HandleTypeDef *huart) {
///Функция проверки CRC16 входящего пакета данных.
///Если CRC16 входящего пакета равна CRC16, содержащейся в пакете - Данные целые. Функция вернет true. В противном случае false.
/// \param *huart - UART, по которому принимаем данные.
	uint16_t len = ModbusRTU_RX_BUFFER_SIZE - huart->RxXferCount; //Узнаем длину пакета входящих данных
	uint16_t CRC_check = ModbusRTU_CRC16_Calculate(ModbusRTU_rx_buffer, len - 2, 1); //Считаем CRC входящих данных
	uint16_t CRC_rx_buffer = ModbusRTU_rx_buffer[len - 2] << 8u | ModbusRTU_rx_buffer[len - 1]; //Смотрим CRC, которая была в пакете

	if (CRC_check == CRC_rx_buffer) {
		return true;
	} else {
		return false;
	}
}

void ModbusRTU_Read_Coils_0x01(uint8_t Slave_ID, uint16_t Read_adress, uint16_t Quantity, uint8_t Slave_byte_order){
	ModbusRTU_tx_buffer[0] = Slave_ID;
	ModbusRTU_tx_buffer[1] = 0x01;
	ModbusRTU_tx_buffer[2] = (uint16_t)Read_adress >> 8u;
	ModbusRTU_tx_buffer[3] = (uint16_t)Read_adress & 0x00ff;
	ModbusRTU_tx_buffer[4] = (uint16_t)Quantity >> 8u;
	ModbusRTU_tx_buffer[5] = (uint16_t)Quantity & 0x00ff;
	uint16_t CRC16 = ModbusRTU_CRC16_Calculate(ModbusRTU_tx_buffer, 6, Slave_byte_order);
	ModbusRTU_tx_buffer[6] = (uint16_t)CRC16 >> 8u;
	ModbusRTU_tx_buffer[7] = (uint16_t)CRC16 & 0x00ff;
}

void ModbusRTU_Read_Discrete_Inputs_0x02(uint8_t Slave_ID, uint16_t Read_adress, uint16_t Quantity, uint8_t Slave_byte_order){
	ModbusRTU_tx_buffer[0] = Slave_ID;
	ModbusRTU_tx_buffer[1] = 0x02;
	ModbusRTU_tx_buffer[2] = (uint16_t)Read_adress >> 8u;
	ModbusRTU_tx_buffer[3] = (uint16_t)Read_adress & 0x00ff;
	ModbusRTU_tx_buffer[4] = (uint16_t)Quantity >> 8u;
	ModbusRTU_tx_buffer[5] = (uint16_t)Quantity & 0x00ff;
	uint16_t CRC16 = ModbusRTU_CRC16_Calculate(ModbusRTU_tx_buffer, 6, Slave_byte_order);
	ModbusRTU_tx_buffer[6] = (uint16_t)CRC16 >> 8u;
	ModbusRTU_tx_buffer[7] = (uint16_t)CRC16 & 0x00ff;
}

void ModbusRTU_Read_Holding_Registers_0x03(uint8_t Slave_ID, uint16_t Read_adress, uint8_t Quantity, uint8_t Slave_byte_order){
	ModbusRTU_tx_buffer[0] = Slave_ID;
	ModbusRTU_tx_buffer[1] = 0x03;
	ModbusRTU_tx_buffer[2] = (uint16_t)Read_adress >> 8u;
	ModbusRTU_tx_buffer[3] = (uint16_t)Read_adress & 0x00ff;
	ModbusRTU_tx_buffer[4] = 0x00;
	ModbusRTU_tx_buffer[5] = Quantity;
	uint16_t CRC16 = ModbusRTU_CRC16_Calculate(ModbusRTU_tx_buffer, 6, Slave_byte_order);
	ModbusRTU_tx_buffer[6] = (uint16_t)CRC16 >> 8u;
	ModbusRTU_tx_buffer[7] = (uint16_t)CRC16 & 0x00ff;
}

void ModbusRTU_Read_Input_Registers_0x04(uint8_t Slave_ID, uint16_t Read_adress, uint8_t Quantity, uint8_t Slave_byte_order){
	ModbusRTU_tx_buffer[0] = Slave_ID;
	ModbusRTU_tx_buffer[1] = 0x04;
	ModbusRTU_tx_buffer[2] = (uint16_t)Read_adress >> 8u;
	ModbusRTU_tx_buffer[3] = (uint16_t)Read_adress & 0x00ff;
	ModbusRTU_tx_buffer[4] = 0x00;
	ModbusRTU_tx_buffer[5] = Quantity;
	uint16_t CRC16 = ModbusRTU_CRC16_Calculate(ModbusRTU_tx_buffer, 6, Slave_byte_order);
	ModbusRTU_tx_buffer[6] = (uint16_t)CRC16 >> 8u;
	ModbusRTU_tx_buffer[7] = (uint16_t)CRC16 & 0x00ff;
}

void ModbusRTU_Write_Single_Coil_0x05(uint8_t Slave_ID, uint16_t Write_adress, bool Coil_set_or_reset, uint8_t Slave_byte_order){
	ModbusRTU_tx_buffer[0] = Slave_ID;
	ModbusRTU_tx_buffer[1] = 0x05;
	ModbusRTU_tx_buffer[2] = (uint16_t)Write_adress >> 8u;
	ModbusRTU_tx_buffer[3] = (uint16_t)Write_adress & 0x00ff;
	if(Coil_set_or_reset){
		ModbusRTU_tx_buffer[4] = 0xff;
		ModbusRTU_tx_buffer[5] = 0x00;
	}
	else{
		ModbusRTU_tx_buffer[4] = 0x00;
		ModbusRTU_tx_buffer[5] = 0x00;
	}
	uint16_t CRC16 = ModbusRTU_CRC16_Calculate(ModbusRTU_tx_buffer, 6, Slave_byte_order);
	ModbusRTU_tx_buffer[6] = (uint16_t)CRC16 >> 8u;
	ModbusRTU_tx_buffer[7] = (uint16_t)CRC16 & 0x00ff;
}

void ModbusRTU_Write_Single_Register_0x06(uint8_t Slave_ID, uint16_t Write_adress, uint16_t Data, uint8_t Slave_byte_order){
	ModbusRTU_tx_buffer[0] = Slave_ID;
	ModbusRTU_tx_buffer[1] = 0x06;
	ModbusRTU_tx_buffer[2] = (uint16_t)Write_adress >> 8u;
	ModbusRTU_tx_buffer[3] = (uint16_t)Write_adress & 0x00ff;
	ModbusRTU_tx_buffer[4] = (uint16_t)Data >> 8u;
	ModbusRTU_tx_buffer[5] = (uint16_t)Data & 0x00ff;
	uint16_t CRC16 = ModbusRTU_CRC16_Calculate(ModbusRTU_tx_buffer, 6, Slave_byte_order);
	ModbusRTU_tx_buffer[6] = (uint16_t)CRC16 >> 8u;
	ModbusRTU_tx_buffer[7] = (uint16_t)CRC16 & 0x00ff;
}
