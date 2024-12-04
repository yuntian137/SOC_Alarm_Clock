#include "IAP_Program.h"

#define FLASH_USER_ADDR  0x801FE00//定义IAP操作目标地址

static volatile uint8_t IAP_Flag = 0;//自定义标志位
void IAP_Program(void)
{
  uint32_t ReadWord = 0;//定义变量存放单字读数据
  uint16_t ReadHalfWord = 0;//定义变量存放单半字读数据
  uint8_t ReadByte = 0;//定义变量存放单字节读数据
  uint32_t Array_WriteWord[3] = {0x11223344, 0x22334455, 0x33445566}, Array_ReadWord[3];//定义读、写字数组
  uint16_t Array_WriteHalfWord[5] = {0x1111, 0x2222, 0x3333, 0x4444, 0x5555}, Array_ReadHalfWord[5];//定义读、写半字数组
  uint8_t Array_WriteByte[5] = {0x99, 0x88, 0x77, 0x66, 0x55}, Array_ReadByte[5];//定义读、写字节数组
  uint8_t Flag_ReadWord = 0, Flag_ReadHalfWord = 0, Flag_ReadByte = 0;//定义变量存放字、半字、字节批量读读到数据的个数

  IAP_Unlock();//解锁IAP操作
  IAP_EraseSector((FLASH_USER_ADDR - FLASH_BASE) / 512);//擦除地址所在扇区，每个扇区大小为512Byte
  IAP_WriteCmd(ENABLE);//开启写使能

  /* 单个数据依次写入(注意地址对齐) */
  IAP_ProgramWord(FLASH_USER_ADDR, 0x12345678);//字写入，目标地址FLASH_USER_ADDR
  IAP_ProgramHalfWord(FLASH_USER_ADDR + 4, 0x6666);//半字写入，目标地址（FLASH_USER_ADDR+4）
  IAP_ProgramByte(FLASH_USER_ADDR + 6, 0x99);//字节写入，目标地址（(FLASH_USER_ADDR+4)+2）

  /* 批量数据写入 */
//写入地址请勿重合！写入数据请勿超出扇区范围！
  IAP_ProgramWordArray(FLASH_USER_ADDR + 20, Array_WriteWord, 3);//批量字写入
  IAP_ProgramHalfWordArray(FLASH_USER_ADDR + 40, Array_WriteHalfWord, 5);//批量半字写入
  IAP_ProgramByteArray(FLASH_USER_ADDR + 60, Array_WriteByte, 5);//批量字节写入

  IAP_Lock();//上锁IAP操作，并且复位IAP操作寄存器

  /* 单个数据依次读取 */
  ReadWord = IAP_ReadWord(FLASH_USER_ADDR);//字读，目标地址（0x803FE00）
  ReadHalfWord = IAP_ReadHalfWord(FLASH_USER_ADDR + 4);//半字读，目标地址（0x803FE00+4=0x803FE04）
  ReadByte = IAP_ReadByte(FLASH_USER_ADDR + 6);//字节读，目标地址（0x803FE04+2=0x803FE06）

  /* 批量数据读取 */
  Flag_ReadWord = IAP_ReadWordArray(FLASH_USER_ADDR + 20, Array_ReadWord, 3);//批量字读，返回值为读取成功数据大小
  Flag_ReadHalfWord = IAP_ReadHalfWordArray(FLASH_USER_ADDR + 40, Array_ReadHalfWord, 5);//批量半字读，返回值为读取成功数据大小
  Flag_ReadByte = IAP_ReadByteArray(FLASH_USER_ADDR + 60, Array_ReadByte, 5);//批量字节读，返回值为读取成功数据大小

  if(ReadWord == 0x12345678 && ReadHalfWord == 0x6666 && ReadByte == 0x99)//单个数据读写判断
  {
    if(Flag_ReadWord == 3 && Flag_ReadHalfWord == 5 && Flag_ReadByte == 5)//批量写入成功个数判断
    {
      IAP_Flag = 1;//IAP读写操作完成标志
    }
  }
}

