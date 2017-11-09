#include "stm32f4xx.h"
#include "CAN.h"

CANTX_TypeDef CAN_Data_TX;
CANRX_TypeDef CAN_Data_RX[2];

extern volatile uint8_t get_firmware_size;
volatile uint32_t countbytes;
volatile int size_firmware;
volatile uint8_t write_flashflag=0;

#define FLAG_STATUS_SECTOR	0x08004000		//sector 1	
#define FIRM_WORK_SECTOR 		0x08008000		//sector2			firmware work base
#define FIRM_UPD_SECTOR 		0x08080000		//sector12		firmware update base

#define NAMBER_WORK_SECTOR			2						//	первый work сектор 				2
																						//  последний work сектор   	7
#define NAMBER_UPD_SECTOR				8						//	первый update	 сектор 		8
#define NAMBER_SECT_U_END 			12					//  последний update сектор		11

#define NETNAME_INDEX  01   //Core4X9I 

extern const uint32_t crc32_table[];
uint32_t crc32_check(const uint8_t *buff,uint32_t nbytes);


/****************************************************************************************************************
*														bxCAN_Init
****************************************************************************************************************/
void bxCAN_Init(void){

	GPIO_InitTypeDef GPIO_InitStruct;
		
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB	 ,ENABLE);
	/*Включаем тактирование CAN в модуле RCC*/	
	RCC->APB1ENR|=RCC_APB1ENR_CAN1EN;
	/*Настройка выводов CAN  CAN1_TX=PB9   CAN1_RX=PB8  */
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin=CAN1_TX|CAN1_RX;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Fast_Speed;
	GPIO_Init(CAN1_PORT,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(CAN1_PORT,CAN1_TX_Source,GPIO_AF_CAN1);
	GPIO_PinAFConfig(CAN1_PORT,CAN1_RX_Source,GPIO_AF_CAN1);
	
	/*Release FIFO*/
	CAN1->RF1R|=CAN_RF1R_RFOM1;
	/*Настройка NVIC для bxCAN interrupt*/
	NVIC_SetPriority(CAN1_RX1_IRQn,  1);

	/*Exit SLEEP mode*/
	CAN1->MCR&=~CAN_MCR_SLEEP;
	/*Enter Init mode bxCAN*/
	CAN1->MCR|=CAN_MCR_INRQ;  /*Initialization Request */
	while((CAN1->MSR&CAN_MSR_INAK)!=CAN_MSR_INAK)		{}   /*while Initialization Acknowledge*/

	CAN1->MCR|=CAN_MCR_DBF;			// CAN работает в режиме отладки//CAN останавливается в режиме отладки
	CAN1->MCR|=CAN_MCR_ABOM;		// Контроллер выходит из состояния «Bus-Off» автоматически 
	CAN1->MCR&=~CAN_MCR_TTCM;
	CAN1->MCR&=~CAN_MCR_AWUM;
	CAN1->MCR&=~CAN_MCR_NART;			// автоматич. ретрансляция включена
	CAN1->MCR&=~CAN_MCR_RFLM;
	CAN1->MCR&=~CAN_MCR_TXFP;	
	
	/*Тестовый режиим работы выключен CAN  SILM=0  LBKM=0 */
	CAN1->BTR&=~CAN_BTR_LBKM;	
	CAN1->BTR&=~CAN_BTR_SILM;	

	CAN1->BTR|=CAN_BTR_BRP&29;														/* tq=(29+1)*tPCLK1=2/3 uS   */
	CAN1->BTR|=CAN_BTR_SJW_0;															/*SJW[1:0]=1  (SJW[1:0]+1)*tCAN=tRJW PROP_SEG =+- 2* tq	*/		
	
	CAN1->BTR|=CAN_BTR_TS1_2;													    /* TS1[3:0]=0X07 */ //tBS1=tq*(7+1)=8*tq
																												/* TS2[2:0]=0X02 */ //tBS2=tq*(2+1)=3*tq
																												// | 1tq | 		8tq 				 |  3tq		| 		T=12*tq=12*2/3=8uS f=125kHz
																												// |-----------------------|---------|		
																												// 								Sample point = 75%		
		/*Init filters*/
			
	CAN1->FM1R|=CAN_FM1R_FBM3|CAN_FM1R_FBM4|CAN_FM1R_FBM5;					// Filters bank 3 4 5  mode ID List		
	CAN1->FS1R&=~(CAN_FS1R_FSC3|CAN_FS1R_FSC4|CAN_FS1R_FSC5);				// Filters bank 3 4 5  scale 16 bits	
	CAN1->FFA1R|=CAN_FFA1R_FFA3|CAN_FFA1R_FFA4|CAN_FFA1R_FFA5;			// Filters bank 3 4 5 FIFO1		

	/*ID filters */
  //FIFO1  
	CAN1->sFilterRegister[3].FR1=0x50D050C0;	//Filters bank 3 fmi 00 ID=0x286 IDE=0 RTR=0	// GET_ALARM_B
																						//							 fmi 01 ID=0x286 IDE=0 RTR=1	// ENABLE ALARM_B
	CAN1->sFilterRegister[3].FR2=0x50F050E0;	//Filters bank 3 fmi 02 ID=0x287 IDE=0 RTR=0	// SET_ALARM_B
																						//							 fmi 03 ID=0x287 IDE=0 RTR=1	// DISABLE ALARM_B
																						
	CAN1->sFilterRegister[4].FR1=0x4E304E20;	//Filters bank 4 fmi 04 ID=0x271 IDE=0 RTR=0	//  SET_FIRMWARE_SIZE 
																						//							 fmi 05 ID=0x271 IDE=0 RTR=1
	CAN1->sFilterRegister[4].FR2=0x4E704E60;	//Filters bank 4 fmi 06 ID=0x273 IDE=0 RTR=0	//	SET_DATA_FIRMWARE
																						//							 fmi 07 ID=0x273 IDE=0 RTR=1	
	
	CAN1->sFilterRegister[5].FR1=0x10F010E0;	//Filters bank 5 fmi 08 ID=0x087 IDE=0 RTR=0	 
																						//							 fmi 09 ID=0x087 IDE=0 RTR=1	// 
	CAN1->sFilterRegister[5].FR2=0x11101100;	//Filters bank 5 fmi 10 ID=0x088 IDE=0 RTR=0	//  
																						//							 fmi 11 ID=0x088 IDE=0 RTR=1	// 	GET_NET_NAME																			
	
	/* Filters activation  */	
	CAN1->FA1R|=CAN_FFA1R_FFA3|CAN_FFA1R_FFA4|CAN_FFA1R_FFA5;		
							
	/*Exit filters init mode*/
	CAN1->FMR&=	~CAN_FMR_FINIT;
	
	/*Разрешение прерываний FIFO0 FIFO1*/
	CAN1->IER|=CAN_IER_FMPIE1;
//	 Exit Init mode bxCAN	
	CAN1->MCR&=~CAN_MCR_INRQ;  														/*Initialization Request */	
	while((CAN1->MSR&CAN_MSR_INAK)==CAN_MSR_INAK)		{}   /*while Initialization Acknowledge*/		
	
	NVIC_EnableIRQ(CAN1_RX1_IRQn);		
}
/*****************************************************************************************************************
*													CAN_Transmit_DataFrame
******************************************************************************************************************/
CAN_TXSTATUS CAN_Transmit_DataFrame(CANTX_TypeDef *Data){
		uint32_t temp=0;
		uint8_t mailbox_index;
	
	if((CAN1->TSR&CAN_TSR_TME0)==CAN_TSR_TME0)
		mailbox_index=0;
	else if((CAN1->TSR&CAN_TSR_TME1)==CAN_TSR_TME1)
		mailbox_index=1;
	else if((CAN1->TSR&CAN_TSR_TME2)==CAN_TSR_TME2)
		mailbox_index=2;
	else
		return CAN_TXBUSY;
	

	CAN1->sTxMailBox[mailbox_index].TIR=(uint32_t)(Data->ID<<21);//&0xffe00000);
	
	CAN1->sTxMailBox[mailbox_index].TDTR&=(uint32_t)0xfffffff0;
	CAN1->sTxMailBox[mailbox_index].TDTR|=Data->DLC;
	
	temp=(Data->Data[3]<<24)|(Data->Data[2]<<16)|(Data->Data[1]<<8)|(Data->Data[0]);
	CAN1->sTxMailBox[mailbox_index].TDLR=temp;
	temp=(Data->Data[7]<<24)|(Data->Data[6]<<16)|(Data->Data[5]<<8)|(Data->Data[4]);
	CAN1->sTxMailBox[mailbox_index].TDHR=temp;
	
	/*Send message*/
	CAN1->sTxMailBox[mailbox_index].TIR|=CAN_TI0R_TXRQ;
	return CAN_TXOK;
		
}	
/*****************************************************************************************************************
*													CAN_Transmit_RemoteFrame
******************************************************************************************************************/

CAN_TXSTATUS CAN_Transmit_RemoteFrame(uint16_t ID){
	
	uint8_t mailbox_index;
	
	if((CAN1->TSR&CAN_TSR_TME0)==CAN_TSR_TME0)
		mailbox_index=0;
	else if((CAN1->TSR&CAN_TSR_TME1)==CAN_TSR_TME1)
		mailbox_index=1;
	else if((CAN1->TSR&CAN_TSR_TME2)==CAN_TSR_TME2)
		mailbox_index=2;
	else
		return CAN_TXBUSY;
	
	CAN1->sTxMailBox[mailbox_index].TIR=(uint32_t)((ID<<21)|0x2);
	CAN1->sTxMailBox[mailbox_index].TDTR&=(uint32_t)0xfffffff0;
	
	/*Send message*/
	CAN1->sTxMailBox[mailbox_index].TIR|=CAN_TI0R_TXRQ;
	return CAN_TXOK;
}
/*****************************************************************************************************************
*													CAN_Receive
******************************************************************************************************************/
void CAN_Receive_IRQHandler(uint8_t FIFONumber){
	
	
	if((CAN1->sFIFOMailBox[FIFONumber].RIR&CAN_RI0R_RTR)!=CAN_RI0R_RTR)
	{
		
		CAN_Data_RX[FIFONumber].Data[0]=(uint8_t)0xFF&(CAN1->sFIFOMailBox[FIFONumber].RDLR);
		CAN_Data_RX[FIFONumber].Data[1]=(uint8_t)0xFF&(CAN1->sFIFOMailBox[FIFONumber].RDLR>>8);
		CAN_Data_RX[FIFONumber].Data[2]=(uint8_t)0xFF&(CAN1->sFIFOMailBox[FIFONumber].RDLR>>16);
		CAN_Data_RX[FIFONumber].Data[3]=(uint8_t)0xFF&(CAN1->sFIFOMailBox[FIFONumber].RDLR>>24);
		
		CAN_Data_RX[FIFONumber].Data[4]=(uint8_t)0xFF&(CAN1->sFIFOMailBox[FIFONumber].RDHR);
		CAN_Data_RX[FIFONumber].Data[5]=(uint8_t)0xFF&(CAN1->sFIFOMailBox[FIFONumber].RDHR>>8);
		CAN_Data_RX[FIFONumber].Data[6]=(uint8_t)0xFF&(CAN1->sFIFOMailBox[FIFONumber].RDHR>>16);
		CAN_Data_RX[FIFONumber].Data[7]=(uint8_t)0xFF&(CAN1->sFIFOMailBox[FIFONumber].RDHR>>24);
		
		CAN_Data_RX[FIFONumber].DLC=(uint8_t)0x0F & CAN1->sFIFOMailBox[FIFONumber].RDTR;
		
	}
		CAN_Data_RX[FIFONumber].ID= (uint16_t)0x7FF & (CAN1->sFIFOMailBox[FIFONumber].RIR>>21);
		CAN_Data_RX[FIFONumber].FMI=(uint8_t)0xFF & (CAN1->sFIFOMailBox[FIFONumber].RDTR>>8);
	
	/*Запрет прерываний FIFO0 FIFO1 на время обработки сообщения*/
		if(FIFONumber)
			CAN1->IER&=~CAN_IER_FMPIE1;	
		else
			CAN1->IER&=~CAN_IER_FMPIE0;
	
		/*Release FIFO*/
	if(FIFONumber)
		CAN1->RF1R|=CAN_RF1R_RFOM1;
	else	
		CAN1->RF0R|=CAN_RF0R_RFOM0;

}


/*****************************************************************************************************************
*													CAN_RXProcess1
******************************************************************************************************************/

void CAN_RXProcess1(void){
	
	uint32_t crc;
	
	switch(CAN_Data_RX[1].FMI) {
		case 4:	//(id=271 SET_FIRMWARE_SIZE)
		// если получили запрос на обновление 
		// * вытащить из CAN_Data_RX[1].Data[0]...CAN_Data_RX[1].Data[3] размер прошивки и записать в size_firmware;
		// * разблокировать flash 
		// * стереть сектора flash 
		// * отправить подтверждение по CAN GET_DATA
		countbytes=0;
		size_firmware=0;
		size_firmware=CAN_Data_RX[1].Data[0];
		size_firmware|=CAN_Data_RX[1].Data[1]<<8;
		size_firmware|=CAN_Data_RX[1].Data[2]<<16;
		size_firmware|=CAN_Data_RX[1].Data[3]<<24;
		
		get_firmware_size=0;
		
		break;		
		case 6:	//(id=273 SET_DATA_FIRMWARE)
			if((size_firmware-countbytes)>=8)
			{
				Flash_prog(&CAN_Data_RX[1].Data[0],(uint8_t*)(FIRM_WORK_SECTOR+countbytes),8,4);		
				countbytes+=8;
				CAN_Data_TX.ID=(NETNAME_INDEX<<8)|0x72;
				CAN_Data_TX.DLC=2;
				CAN_Data_TX.Data[0]=NETNAME_INDEX;
				CAN_Data_TX.Data[1]='g';								// GET_DATA!
				CAN_Transmit_DataFrame(&CAN_Data_TX);
				
				if((countbytes%240)==0)
				{	
					if(LEDPORT->IDR & LEDPIN_IDR)
						LEDPORT->BSRRH=LEDPIN_BSSR;
					else
						LEDPORT->BSRRL=LEDPIN_BSSR;	
				}
			}
			else 
			{
				Flash_prog(&CAN_Data_RX[1].Data[0],(uint8_t*)(FIRM_WORK_SECTOR+countbytes),(size_firmware-countbytes),4);
				countbytes+=(size_firmware-countbytes);
			}
			
			if(size_firmware==countbytes)	
			{
				
				crc=crc32_check((const uint8_t*)FIRM_WORK_SECTOR,(size_firmware-4));
				if(crc==*(uint32_t*)(FIRM_WORK_SECTOR+size_firmware-4))
				{
					CAN_Data_TX.ID=(NETNAME_INDEX<<8)|0x72;
					CAN_Data_TX.DLC=2;
					CAN_Data_TX.Data[0]=NETNAME_INDEX;
					CAN_Data_TX.Data[1]='c';								// CRC OK!	
					CAN_Transmit_DataFrame(&CAN_Data_TX);
					
					write_flashflag=1;
				}
				else
				{
					FLASH->CR |=FLASH_CR_LOCK;
					
					CAN_Data_TX.ID=(NETNAME_INDEX<<8)|0x72;
					CAN_Data_TX.DLC=2;
					CAN_Data_TX.Data[0]=NETNAME_INDEX;
					CAN_Data_TX.Data[1]='e';								// CRC ERROR!		
					CAN_Transmit_DataFrame(&CAN_Data_TX);
				}
			}
			
		break;
		case 11://(id=088 remote get net name)
		//
			CAN_Data_TX.ID=(NETNAME_INDEX<<8)|0x88;
			CAN_Data_TX.DLC=1;
			CAN_Data_TX.Data[0]=NETNAME_INDEX;  // // netname_index для Core4X9I
			CAN_Transmit_DataFrame(&CAN_Data_TX);
						
		break;
		
		default:
		break;
	}
	/*Разрешение прерываний FIFO1*/
	CAN1->IER|=CAN_IER_FMPIE1;
}

