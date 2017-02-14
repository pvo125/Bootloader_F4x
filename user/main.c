#include "stm32f4xx.h"
//#include "stm32f429.h"
#include "CAN.h"



#define FLASHBOOT_SIZE	0x8000				// 32 kB
#define NAMBER_WORK_SECTOR			2						//	первый work сектор 				2
																						//  последний work сектор   	7
#define NAMBER_UPD_SECTOR				8						//	первый update	 сектор 		8
#define NAMBER_SECT_U_END 			12					//  последний update сектор		11

#define FLASHBOOT_SECTOR		0x080000000		//sector 0
#define FLAG_STATUS_SECTOR	0x08004000		//sector 1	
#define FIRM_WORK_SECTOR 		0x08008000			//sector2			firmware work base
#define FIRM_UPD_SECTOR 		0x08080000			//sector12		firmware update base

#define NAMBER_WORK_SECTOR			2						//	первый work сектор 				2
																						//  последний work сектор   	7
#define NAMBER_UPD_SECTOR				8						//	первый update	 сектор 		8
#define NAMBER_SECT_U_END 			12					//  последний update сектор		11

#define NETNAME_INDEX  01   //Core4X9I

extern volatile int size_firmware;
volatile uint8_t get_firmware_size;
extern volatile uint8_t write_flashflag;
const uint32_t crc32_table[256]=
{	
		0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA,
    0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
    0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
    0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
    0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
    0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
    0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
    0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
    0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
    0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086,
    0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4,
    0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
    0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
    0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE,
    0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
    0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252,
    0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60,
    0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
    0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04,
    0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
    0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
    0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E,
    0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C,
    0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
    0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0,
    0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6,
    0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
    0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

/**********************************************************************************************
*																	CRC32 checksum
***********************************************************************************************/
uint32_t crc32_check(const uint8_t *buff,uint32_t count){

	uint32_t crc=0xffffffff;
	while(count--)
		crc=(crc>>8)^crc32_table[(crc^*buff++) & 0xFF];

	return crc^0xffffffff;

}
/**********************************************************************************************
*																	Flash_unlock
***********************************************************************************************/
void Flash_unlock(void){

	FLASH->KEYR=0x45670123;
	FLASH->KEYR=0xCDEF89AB;
}
/**********************************************************************************************
*																	Flash_sect_erase
***********************************************************************************************/
void Flash_sect_erase(uint8_t numsect,uint8_t count){
	uint8_t i;
	while((FLASH->SR & FLASH_SR_BSY)==FLASH_SR_BSY) {}
	if(FLASH->SR & FLASH_SR_EOP)	// если EOP установлен в 1 значит erase/program complete
		FLASH->SR=FLASH_SR_EOP;		// сбросим его для следующей индикации записи
	
	FLASH->CR |=FLASH_CR_EOPIE;					// включим прерывание для индикации флага EOP 
	FLASH->CR |= FLASH_CR_PSIZE_1;			// 10 program/erase x32
	FLASH->CR |=FLASH_CR_SER;																	// флаг  очистки сектора
														
	for(i=numsect;i<numsect+count;i++)
		{
			FLASH->CR &= ~FLASH_CR_SNB;														// очистим биты SNB[3:7] 
			FLASH->CR|=(uint32_t)(i<<3);													// запишем номер сектора для erase
			FLASH->CR |=FLASH_CR_STRT;														// запуск очистки заданного сектора
			while((FLASH->SR & FLASH_SR_EOP)!=FLASH_SR_EOP) {}		// ожидание готовности
			FLASH->SR=FLASH_SR_EOP;	
		}
	FLASH->CR &= ~FLASH_CR_SER;																	// сбросим  флаг  очистки сектора
}
/**********************************************************************************************
*																	Flash_prog
***********************************************************************************************/
void Flash_prog(uint8_t * src,uint8_t * dst,uint32_t nbyte,uint8_t psize){
	uint32_t i;
	while((FLASH->SR & FLASH_SR_BSY)==FLASH_SR_BSY) {}
	if(FLASH->SR & FLASH_SR_EOP)	// если EOP установлен в 1 значит erase/program complete
		FLASH->SR=FLASH_SR_EOP;		// сбросим его для следующей индикации записи
	
	FLASH->CR |=FLASH_CR_EOPIE;					// включим прерывание для индикации флага EOP 
	switch(psize){
		case 1:
			FLASH->CR &= ~FLASH_CR_PSIZE;			// 00 program x8
			FLASH->CR |=FLASH_CR_PG;
			for(i=0;i<nbyte;i++)
			{
				*(uint8_t*)(dst+i)=*(uint8_t*)(src+i);
				while((FLASH->SR & FLASH_SR_EOP)!=FLASH_SR_EOP) {}
				FLASH->SR=FLASH_SR_EOP;	
			}
		break;
		case 2:
			FLASH->CR |= FLASH_CR_PSIZE_0;			// 01 program x16
			FLASH->CR |=FLASH_CR_PG;	
			for(i=0;i<nbyte;i++)
			{
				*(uint16_t*)(dst+i*2)=*(uint16_t*)(src+i*2);
				while((FLASH->SR & FLASH_SR_EOP)!=FLASH_SR_EOP) {}
				FLASH->SR=FLASH_SR_EOP;	
			}
		break;
		case 4:
			FLASH->CR |= FLASH_CR_PSIZE_1;			// 10 program x32
			FLASH->CR |=FLASH_CR_PG;	
			for(i=0;i<nbyte;i++)
			{
				*(uint32_t*)(dst+i*4)=*(uint32_t*)(src+i*4);
				while((FLASH->SR & FLASH_SR_EOP)!=FLASH_SR_EOP) {}
				FLASH->SR=FLASH_SR_EOP;
		  }
		break;	
	}
		
	FLASH->CR &= ~FLASH_CR_PG;

}


/**********************************************************************************************
*			Функция проверки флэш на стертость возвращает 0 если флэш стерта и 1 если проверка на чистоту не прошла
***********************************************************************************************/
uint8_t checkflash_erase(uint32_t start_addr,uint32_t byte){
	uint32_t count;
	
	while(*(uint32_t*)(start_addr+count)==0xFFFFFFFF)
	{
		count+=4;
		if(count>=byte)
			return 0;
	}
	return 1;
}

/**********************************************************************************************
*			Функция  ожидания прошивки и принятия  ее по CAN сети
***********************************************************************************************/
void Bootloader_upd_firmware(uint16_t count){

	uint8_t flag,n;
	GPIO_InitTypeDef GPIO_InitStruct;
	
		bxCAN_Init();
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF	 ,ENABLE);
	
	// PF7 выход push-pull без подтяжки для моргания светодиодом
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_7;
		GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_InitStruct.GPIO_Speed=GPIO_Low_Speed;
		GPIO_Init(GPIOF,&GPIO_InitStruct);	
	
	
	// отправляем запрос по сети на выдачу прошивки
		
	// Настроим SysTick сбросим флаг CLKSOURCE выберем источник тактирования AHB/8
		SysTick->CTRL &=~SysTick_CTRL_CLKSOURCE_Msk;
		SysTick->CTRL |=SysTick_CTRL_ENABLE_Msk;
		
		get_firmware_size=1;										// взводим флаг и ждем когда master пришдет свой запрос с данными размера прошивки 
	
		while(get_firmware_size) 
		{			
			CAN_Data_TX.ID=(NETNAME_INDEX<<8)|0x74;						// 0x174 GET_FIRMWARE			
			CAN_Data_TX.DLC=1;
			CAN_Data_TX.Data[0]=NETNAME_INDEX;
			CAN_Transmit_DataFrame(&CAN_Data_TX);
			
			if(GPIOF->IDR & GPIO_IDR_IDR_7)
				GPIOF->BSRRH=GPIO_BSRR_BS_7;
			else
				GPIOF->BSRRL=GPIO_BSRR_BS_7;
			
			SysTick->LOAD=(2500000*6);
			SysTick->VAL=0;
			while(!(SysTick->CTRL&SysTick_CTRL_COUNTFLAG_Msk)){}
		}
		GPIOF->BSRRH=GPIO_BSRR_BS_7;
		
		Flash_unlock();
		// Проверка flash на стертость
		if(checkflash_erase(FIRM_WORK_SECTOR,size_firmware))	
			{	// Если секторы не чистые (0xFF) 
				if(size_firmware<=96*1024)		
				n=3;															// размер <= 96K стираем только 2,3,4 секторы 
			else if(size_firmware<=224*1024)
				n=4;															// размер <= 224K стираем только 2,3,4,5 секторы 
			else if(size_firmware<=352*1024)	
				n=5;															// размер <= 352K стираем только 2,3,4,5,6 секторы 
			else
				n=6;															// размер <= 480K стираем только 2,3,4,5,6,7 секторы
			
			Flash_sect_erase(NAMBER_WORK_SECTOR,n);		// Очистим n секторов 
		}
		CAN_Data_TX.ID=(NETNAME_INDEX<<8)|0x72;
		CAN_Data_TX.DLC=2;
		CAN_Data_TX.Data[0]=NETNAME_INDEX;
		CAN_Data_TX.Data[1]='g';								// GET_DATA!
		CAN_Transmit_DataFrame(&CAN_Data_TX);
		
		while(write_flashflag==0) {}
			
		// Запишем в сектор FLAG_STATUS флаг 0xA3 по адресу  (FLAG_STATUS_SECTOR+count) стирать предварительно не будем так как там 0xFF
			flag=0xA3;	
			Flash_prog((uint8_t*)&flag,(uint8_t*)(FLAG_STATUS_SECTOR+count),1,1);	// 1 байт в режиме x8
			// Сделаем RESET 
			NVIC_SystemReset();


}
/*
*/
int main (void) {
	uint32_t bin_size,crc;
	uint8_t flag;
	uint16_t count;
	
	CANTX_TypeDef CAN_Data_TX;
	void(*pApplication)(void);		// указатель на функцию запуска приложения
	/*
	0x0800 0000 - 0x0800 3FFF   загрузчик						1 сектор флэш
	0x0800 4000 - 0x0800 7FFF		сектор FLAG_STATUS	2 сектор флэш	
	0x0800 8000 - 0x0807 FFFF		firmware work 	( max 480*1024 B)
	0x0808 0000 - 0x080F FFFF		firmware update ( max 512*1024 B)*/ 
	/*
		Загрузчик должен проверить флаг в секторе FLAG_STATUS во флэши.  
		Если установлен в 0xA7:
		{	
			* проверить crc обновленной прошивки во второй половине флэш
			*	переписать со второй половины флэш в первую(рабочую) 
			* записать в сектор FLAG_STATUS флаг 0xA3 
			* сделать reset для запуска обновленной прошивки из первой половины флэш
		}
		Если установлен в 0xA3:
		{
			* Передвинуть таблицу векторов на FLASHBOOT_SIZE 		(0x8000)
			* Установить указатель стэка SP на FLASHFIRM_W_BASE (0x080008000)
			* Запустить функцию (приложение ) по указателю FIRM_WORK_SECTOR+4
		}
		Иначе
		{	
			* Настроить периферию для работы CAN модуля.
			* Ожидать приема прошивки по CAN сети.
			* Записать принять прошивку по сети во вторую половину флэш
			*	проверить crc обновленной прошивки во второй половине флэш
			*	переписать со второй половины флэш в первую(рабочую) 
			* записать в сектор FLAG_STATUS флаг 0xA3 
			* сделать reset для запуска обновленной прошивки из первой половины флэш
		}
		
		*/
	// проверим флаг  в секторе FLAG_STATUS во флэш.
	count=1;
	while(*(uint8_t*)(FLAG_STATUS_SECTOR+count)!=0xFF)		// Перебираем байты пока не дойдем до неписанного поля 0xFF 
			count++;
	flag=*(uint8_t*)(FLAG_STATUS_SECTOR+count-1);   // значение по адресу (FLAG_STATUS_SECTOR+count-1) // Читаем значение флага на 1 адресс меньше чистого поля.
	
	if(flag==0xA7)	
	{		// установлен флаг обновления firmware равный 0xA7
		// по адресу FIRM_UPD_SECTOR+0x1C считаем 4 байта размера прошивки 
		bin_size=*((uint32_t*)(FIRM_UPD_SECTOR+0x1C));
		
		crc=crc32_check((const uint8_t*)FIRM_UPD_SECTOR,bin_size);
		/* Сравниваем полученный crc c тем что пришел с файлом прошивки */
		if(*(uint32_t*)(FIRM_UPD_SECTOR+bin_size)==crc)
		{
			//Копируем проверенную прошивку в рабочую часть flash  по адресу FIRM_WORK_SECTOR 
				// Подготовим flash для стирания и программирования 
			 // Для доступа к FLASH->CR	
				Flash_unlock();
			// Стирание секторов для записи  в рабочую часть флэш (со 2 по 7)	
				Flash_sect_erase(NAMBER_WORK_SECTOR,6);
			 // Запись обновленной прошивки с адреса FIRM_UPD_SECTOR в FIRM_WORK_SECTOR 
				Flash_prog((uint8_t*)FIRM_UPD_SECTOR,(uint8_t*)FIRM_WORK_SECTOR,bin_size,4);	
			 
			// Запишем в сектор FLAG_STATUS флаг 0xA3 по адресу  (FLAG_STATUS_SECTOR+count) стирать предварительно не будем так как там 0xFF
			flag=0xA3;	
			Flash_prog((uint8_t*)&flag,(uint8_t*)(FLAG_STATUS_SECTOR+count),1,1);	// 1 байт в режиме x8
			// Сделаем RESET 
			NVIC_SystemReset();
		}	
	}
	else if(flag==0xA3)						 
	{				// установлен флаг 0xA3 запуск приложения		
		// по адресу FIRM_WORK_SECTOR+0x1C считаем 4 байта размера прошивки 
		bin_size=*((uint32_t*)(FIRM_WORK_SECTOR+0x1C));	
		crc=crc32_check((const uint8_t*)FIRM_WORK_SECTOR,bin_size);
		/* Сравниваем полученный crc с рабочего сектора  c тем что лежит FIRM_UPD_SECTOR+bin_size и пришло при обновлении */
		if(*(uint32_t*)(FIRM_WORK_SECTOR+bin_size)==crc)
			{			// Если прошивка цела делаем запуск pApplication() с рабочего сектора
			// Передвинем таблицу векторов на FLASHBOOT_SIZE 	
				SCB->VTOR=FLASH_BASE|FLASHBOOT_SIZE;
				pApplication=(void(*)(void))*(__IO uint32_t*)(FIRM_WORK_SECTOR+4);
				__set_MSP(*(__IO uint32_t*)FIRM_WORK_SECTOR);
				
				pApplication();
			}
		else
			{// Иначе настраиваем интерфейс CAN и ждем прошивку по сети.
				Bootloader_upd_firmware(count);
			}
	}
	else
	{
		//Если CRC прошивки не совпадает с подсчитанной настраиваем интерфейс CAN и ждем прошивку по сети.
		Bootloader_upd_firmware(count);
		
	}
	
	while(1){}
}


