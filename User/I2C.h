#ifndef __I2C_H
#define __I2C_H

extern void I2C1_Configuration(void);

extern unsigned char I2C1_Read_Byte(unsigned char Device_Addr,unsigned char Reg_Addr);
extern void I2C_Write_Byte(unsigned char Device_Addr,unsigned char Reg_Addr,unsigned char Reg_Value);

extern void I2C1_ReadS(unsigned short addr ,unsigned char * pBuffer,unsigned short Length);
extern unsigned char I2C_WriteS(unsigned short addr,unsigned char* pBuffer, unsigned char Length);

#endif
