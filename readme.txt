You have to set the correct memory layout for your device in the linker script.
Please check the FLASH and SRAM length.

e.g.


MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 0x08000   /* 32k */
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 0x02000   /*  8k */
}

Адреса Modbus регистров

текущие обороты (об/мин) - 	0
число пар полюсов - 		1

// параметры запуска
стартовый ШИМ (100...1000) - 	2
стартовые обороты (об/мин) - 	3
скорость разгона (об/сек) - 	4

	
обороты (задание) (об/мин) - 	5