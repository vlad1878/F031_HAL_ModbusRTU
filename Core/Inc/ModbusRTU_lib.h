#include "main.h"
#include <stdbool.h>

#define ModbusRTU_TX_BUFFER_SIZE 64
#define ModbusRTU_RX_BUFFER_SIZE 255

uint16_t ModbusRTU_CRC16_Calculate(uint8_t *data, uint8_t lenght, uint8_t byte_order);
bool ModbusRTU_CRC16_Check(UART_HandleTypeDef *huart);
void ModbusRTU_Read_Coils_0x01(uint8_t Slave_ID, uint16_t Read_adress, uint16_t Quantity, uint8_t Slave_byte_order);
void ModbusRTU_Read_Discrete_Inputs_0x02(uint8_t Slave_ID, uint16_t Read_adress, uint16_t Quantity, uint8_t Slave_byte_order);
void ModbusRTU_Read_Holding_Registers_0x03(uint8_t Slave_ID, uint16_t Read_adress, uint8_t Quantity, uint8_t Slave_byte_order);
void ModbusRTU_Read_Input_Registers_0x04(uint8_t Slave_ID, uint16_t Read_adress, uint8_t Quantity, uint8_t Slave_byte_order);
void ModbusRTU_Write_Single_Coil_0x05(uint8_t Slave_ID, uint16_t Write_adress, bool Coil_set_or_reset, uint8_t Slave_byte_order);
void ModbusRTU_Write_Single_Register_0x06(uint8_t Slave_ID, uint16_t Write_adress, uint16_t Data, uint8_t Slave_byte_order);
