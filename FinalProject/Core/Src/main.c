/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "cantp.h"
#include <stdlib.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

static CANTPCanInit Tester;
static CANTPCanInit ECU;
static CANTPMessage receiveMessage;
static CANTPMessage sendMessage;
static uint8_t isUnlocked = 0; 			// locked: 0; unlocked: 1
static uint8_t seed[4];
static uint8_t key[4];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
 /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CANTP(CANTPMessage *canMessage, CANTPCanInit *Node);


void showResponseFrame(CANTPMessage *receiveMessage){
	int index;
	if(receiveMessage->serviceInfo.SID != 0x7F) {
		printf("Positive data: ");
		printf(" 0x%02X", receiveMessage->serviceInfo.SID);
		printf(" 0x%02X", receiveMessage->serviceInfo.SF);
		index = 2;
	}
	else {
		printf("Negative data: ");
		printf(" 0x%02X", receiveMessage->serviceInfo.SID);
		printf(" 0x%02X", receiveMessage->serviceInfo.DID[0]);
		printf(" 0x%02X", receiveMessage->serviceInfo.DID[1]);
		index = 3;
	}

    for (uint8_t i = index; i < 7; i++)
    {
    	printf(" 0x%02X", 0x55);
    }
    printf("\n");
}

void showData(uint8_t *frame){
    for (uint8_t i = 0; i < 6; i++)
        {
            printf(" 0x%02X", *(frame + i));
        }
    printf("\n");
}

void receiveSingleFrame(CANTPMessage *receiveMessage, uint8_t *CAN_RXData){
	receiveMessage->serviceInfo.SID = CAN_RXData[1];
	uint8_t index = 0;
	if(CAN_RXData[1] == 0x27 || CAN_RXData[1] == 0x67 ){
		receiveMessage->serviceInfo.SF = CAN_RXData[2];
		index = 3;
	}
	else {
		receiveMessage->serviceInfo.DID[0] = CAN_RXData[2];
		receiveMessage->serviceInfo.DID[1] = CAN_RXData[3];
		index = 4;
	}

	receiveMessage->dataLength = 0;
    for (uint8_t i = index; i <= 7; i++)
    {
        if(CAN_RXData[i] != 0x55){
        	receiveMessage->dataLength += 1;
        }
        else{
        	break;
        }
    }
    receiveMessage->data = (uint8_t*)malloc(receiveMessage->dataLength);
    memcpy(receiveMessage->data, &CAN_RXData[index],  receiveMessage->dataLength);
}

void receiveFirstFrame( CANTPMessage *receiveMessage, uint8_t *CANDataRX){
    uint16_t dataLength = (CANDataRX[0] & 0x0F);
    receiveMessage->dataLength = dataLength << 8 | CANDataRX[1];
    receiveMessage->serviceInfo.SID = CANDataRX[2];
    receiveMessage->serviceInfo.DID[0] = CANDataRX[3];
    receiveMessage->serviceInfo.DID[1] = CANDataRX[4];
    receiveMessage->bufferLength = 3;
    receiveMessage->offset = 3;
    receiveMessage->data = (uint8_t*)malloc(receiveMessage->dataLength);
    memcpy(receiveMessage->data, &CANDataRX[5], 3);
}

void receiveConsecutiveFrame(CANTPMessage *receiveMessage, uint8_t *CANDataRX){
    for (uint8_t i = 1; i <= 7; i++)
    {
        if(CANDataRX[i] != 0x55){
        	receiveMessage->data[receiveMessage->bufferLength] = CANDataRX[i];
        	receiveMessage->bufferLength += 1;
        }
    }
}

void sendSingleFrame(CANTPMessage *sendMessage, CANTPCanInit *can){
	CANTPSingleFrame singleFrame;
    CANTPControlInformation PCI_SINGLE = PCI_SINGLE_FRAME;
    uint8_t NAValue[7] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
    singleFrame.indicate = PCI_SINGLE << 4 | sendMessage->bufferLength;
    uint8_t payloadOffset = 0;
    if (sendMessage->serviceInfo.SF != 0x00){
    	singleFrame.payload[0] = sendMessage->serviceInfo.SID;
    	singleFrame.payload[1] = sendMessage->serviceInfo.SF;
    	payloadOffset = 2;
    }
    else {
	singleFrame.payload[0] = sendMessage->serviceInfo.SID;
	singleFrame.payload[1] = sendMessage->serviceInfo.DID[0];
	singleFrame.payload[2] = sendMessage->serviceInfo.DID[1];
	payloadOffset = 3;

    if(sendMessage->dataLength != 0){
    		memcpy(&singleFrame.payload[payloadOffset],&sendMessage->data, sendMessage->dataLength);
    		payloadOffset += sendMessage->dataLength;
		}
    }
    memcpy(&singleFrame.payload[payloadOffset],&NAValue, 7 - payloadOffset);
    memcpy(can->TxData, &singleFrame, sizeof(singleFrame));
	HAL_CAN_AddTxMessage(&(can->hcan), &(can->TxHeader), can->TxData, &(can->TxMailbox));
}

void sendFirstFrame(CANTPMessage *sendMessage, CANTPCanInit *can){
    CANTPFirstFrame firstFrame;
    CANTPControlInformation PCI_FIRST = PCI_FIRST_FRAME;

    firstFrame.indicate = PCI_FIRST | (sendMessage->bufferLength >> 8);
    firstFrame.dataLength = sendMessage->bufferLength;
    firstFrame.SID = sendMessage->serviceInfo.SID;
    memcpy(&firstFrame.DID, &sendMessage->serviceInfo.DID, 2);
    memcpy(&firstFrame.payload, sendMessage->data, 3);
    sendMessage->offset = 3;
    memcpy(can->TxData, &firstFrame, sizeof(firstFrame));
    HAL_CAN_AddTxMessage(&(can->hcan), &(can->TxHeader), can->TxData, &(can->TxMailbox));
}

void sendConsecutiveFrame(CANTPMessage *sendMessage, CANTPCanInit *can){
    CANTPConsecutiveFrame consecutiveFrame;
    CANTPControlInformation PCI_CONSECUTIVE = PCI_CONSECUTIVE_FRAME;
    uint8_t numFrame = (sendMessage->bufferLength + 1)/7 + (((sendMessage->bufferLength + 1)%7 == 0) ? 0:1);
    uint8_t sequenceNumber = 0x01;
    uint8_t *dataTemp = sendMessage->data;

    for (uint8_t i = 0; i < numFrame - 1; i++)
    {

        if(sequenceNumber == 0x0F) sequenceNumber = 0x00;
        consecutiveFrame.indicate = PCI_CONSECUTIVE | sequenceNumber;
        sequenceNumber += 0x01;

        for (uint8_t j = 0; j < 7; j++)
        {
            consecutiveFrame.payload[j] = dataTemp[sendMessage->offset];
            if(sendMessage->offset >= sendMessage->dataLength){
                consecutiveFrame.payload[j] = 0x55;
            }
            sendMessage->offset += 0x01;
        }

        memcpy(&(can->TxData), &consecutiveFrame, sizeof(consecutiveFrame));
        HAL_CAN_AddTxMessage(&(can->hcan), &(can->TxHeader), can->TxData, &(can->TxMailbox));
    }
}

void sendFlowControlFrame(CANTPFlowStatus FLOW_STATUS, uint8_t BLOCK_SIZE, uint8_t STIME, CANTPCanInit *can){
    CANTPFlowControlFrame flowControlFrame;
    CANTPControlInformation PCI_FLOW_CONTROL = PCI_FLOW_CONTROL_FRAME;

    flowControlFrame.indicate = PCI_FLOW_CONTROL | FLOW_STATUS;
    flowControlFrame.blockSize = 0x00;
    flowControlFrame.separationTime = 0x00;
    uint8_t NAValue[] = {0x55, 0x55, 0x55, 0x55, 0x55};

    memcpy(&flowControlFrame.NA, &NAValue, sizeof(NAValue));
    memcpy(can->TxData, &flowControlFrame, sizeof(flowControlFrame));
    HAL_CAN_AddTxMessage(&(can->hcan), &(can->TxHeader), can->TxData, &(can->TxMailbox));

}

void sendresponseFrame(CANTPService *ServiceInformation, CANTPCanInit *can){
	CANTPSingleFrame singleFrame;
	singleFrame.indicate = 0x07;
	singleFrame.payload[0] = ServiceInformation->SID;
	singleFrame.payload[1] = ServiceInformation->DID[0]; // MSB DID
	singleFrame.payload[2] = ServiceInformation->DID[1]; //LSB DID
	singleFrame.payload[3] = 0x55;
	singleFrame.payload[4] = 0x55;
	singleFrame.payload[5] = 0x55;
	singleFrame.payload[6] = 0x55;

    memcpy(&(can->TxData), &singleFrame, sizeof(singleFrame));
    HAL_CAN_AddTxMessage(&(can->hcan), &(can->TxHeader), can->TxData, &(can->TxMailbox));
}

void responseSeedRequest(CANTPService *ServiceInformation, CANTPCanInit *can){
	uint8_t minNumber = 0x00;
	uint8_t maxNumber = 0xFF;
	srand((int)time(0));

	seed[0] = minNumber + rand() % (maxNumber + 1 - minNumber);
	seed[1] = minNumber + rand() % (maxNumber + 1 - minNumber);
	seed[2] = minNumber + rand() % (maxNumber + 1 - minNumber);
	seed[3] = minNumber + rand() % (maxNumber + 1 - minNumber);

	CANTPSingleFrame singleFrame;
	singleFrame.indicate = 0x07;
	singleFrame.payload[0] = ServiceInformation->SID;
	singleFrame.payload[1] = ServiceInformation->SF;
	singleFrame.payload[2] = seed[0];
	singleFrame.payload[3] = seed[1];
	singleFrame.payload[4] = seed[2];
	singleFrame.payload[5] = seed[3];

	singleFrame.payload[6] = 0x55;
    memcpy(&(can->TxData), &singleFrame, sizeof(singleFrame));
    HAL_CAN_AddTxMessage(&(can->hcan), &(can->TxHeader), can->TxData, &(can->TxMailbox));
}

void sendKey(CANTPMessage *receiveMessage, CANTPCanInit *can){
	CANTPSingleFrame singleFrame;
	singleFrame.indicate = 0x07;
	singleFrame.payload[0] = receiveMessage->serviceInfo.SID - 0x40;
	singleFrame.payload[1] = receiveMessage->serviceInfo.SF + 0x01;
	singleFrame.payload[2] = receiveMessage->data[0] + 0x02;
	singleFrame.payload[3] = receiveMessage->data[1] + 0x01;
	singleFrame.payload[4] = receiveMessage->data[2] + 0x01;
	singleFrame.payload[5] = receiveMessage->data[3] + 0x01;
	singleFrame.payload[6] = 0x55;

    memcpy(&(can->TxData), &singleFrame, sizeof(singleFrame));
    HAL_CAN_AddTxMessage(&(can->hcan), &(can->TxHeader), can->TxData, &(can->TxMailbox));
}

void responseKey(CANTPMessage *receiveMessage, CANTPCanInit *can){
	CANTPSingleFrame singleFrame;
	singleFrame.indicate = 0x07;
	key[0] = seed[0] + 0x01;
	key[1] = seed[1] + 0x01;
	key[2] = seed[2] + 0x01;
	key[3] = seed[3] + 0x01;

	if (key[0] == receiveMessage->data[0] && key[1] == receiveMessage->data[1]
		&& key[2] == receiveMessage->data[2] && key[3] == receiveMessage->data[3]) {
		singleFrame.payload[0] = receiveMessage->serviceInfo.SID;
		singleFrame.payload[1] = receiveMessage->serviceInfo.SF;
		singleFrame.payload[2] = 0x55;
		singleFrame.payload[3] = 0x55;
		singleFrame.payload[4] = 0x55;
		singleFrame.payload[5] = 0x55;
		singleFrame.payload[6] = 0x55;

		isUnlocked = 1;
		HAL_GPIO_WritePin(GPIOB, LED0_Pin, isUnlocked);
		memcpy(&(can->TxData), &singleFrame, sizeof(singleFrame));
		HAL_CAN_AddTxMessage(&(can->hcan), &(can->TxHeader), can->TxData, &(can->TxMailbox));
	}
	else {
		receiveMessage->serviceInfo.SID -= 0x40;
		sendNegativeResponse(receiveMessage, can, 0x35);
		printf("---> Invalid key - Delay 10s\n");

		HAL_CAN_Stop(&hcan1);					// stop CAN and delay in 10s
		HAL_Delay(10000);
		HAL_CAN_Start(&hcan1);
		printf("Continue receiving service request\n");

	}
}

void sendNegativeResponse(CANTPMessage *receiveMessage, CANTPCanInit *can, uint8_t NRC){
	CANTPSingleFrame singleFrame;
	singleFrame.indicate = 0x07;
	singleFrame.payload[0] = 0x7F;
	singleFrame.payload[1] = receiveMessage->serviceInfo.SID;
	singleFrame.payload[2] = NRC;			// invalid key negative response code
	singleFrame.payload[3] = 0x55;
	singleFrame.payload[4] = 0x55;
	singleFrame.payload[5] = 0x55;
	singleFrame.payload[6] = 0x55;

	memcpy(&(can->TxData), &singleFrame, sizeof(singleFrame));
	HAL_CAN_AddTxMessage(&(can->hcan), &(can->TxHeader), can->TxData, &(can->TxMailbox));
}

// Handle Single Frame in CallbackFunction
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &ECU.RxHeader, ECU.RxData);
	uint8_t frameType = ECU.RxData[0] >> 4;

	if (ECU.RxHeader.DLC == 8)
	{
		if (ECU.RxData[1] == 0x27 && frameType == 0x0){
			receiveSingleFrame(&receiveMessage, ECU.RxData);
			receiveMessage.serviceInfo.SID += 0x40;

			if(receiveMessage.serviceInfo.SF == 0x01){
				responseSeedRequest(&receiveMessage.serviceInfo, &ECU);
			}
			else if (receiveMessage.serviceInfo.SF == 0x02)
			{
				responseKey(&receiveMessage, &ECU);
			}
		}
		else {
			if (!isUnlocked) {
				if(frameType == 0x00)
					receiveSingleFrame(&receiveMessage, ECU.RxData);
				else
					receiveFirstFrame(&receiveMessage, ECU.RxData);

				sendNegativeResponse(&receiveMessage, &ECU, 0x33);
				printf("*** ECU locked ***\n");
				return;
			}

			switch(frameType){
				case 0x0:
				{
					if(ECU.RxData[1] == 0x22){
						HAL_ADC_Start(&hadc1);
						CANTPMessage message;
						uint16_t ADC_val = HAL_ADC_GetValue(&hadc1);
						uint8_t ADCValue8b[] = {ADC_val >> 8, ADC_val};
						message.serviceInfo.SID = ECU.RxData[1] + 0x40;
						message.serviceInfo.DID[0] = ECU.RxData[2];
						message.serviceInfo.DID[1] = ECU.RxData[3];
						message.serviceInfo.SF = 0x00;
						message.dataLength = sizeof(ADC_val);

						memcpy(&message.data, &ADCValue8b, sizeof(ADCValue8b));
						printf("Send ADC Value 0x%X\n", ADC_val);
						message.bufferLength = sizeof(message.serviceInfo) - 1 + message.dataLength;
						CANTP(&message, &ECU);

					}
					else if (ECU.RxData[1] == 0x2E){
						receiveSingleFrame(&receiveMessage, ECU.RxData);
						receiveMessage.serviceInfo.SID += 0x40;
						sendresponseFrame(&receiveMessage.serviceInfo, &ECU);
					}
				break;
				}
				case 0x1:
				{
					if(ECU.RxData[2] == 0x2E){
						receiveFirstFrame(&receiveMessage, ECU.RxData);
						sendFlowControlFrame(PCI_FLOW_STATUS_CONTINUE, BLOCK_SIZE, SEPARATION_TIME, &ECU);
					}
					else{
						printf("Wrong Frame!\n");
					}
					break;
				}
				case 0x2:
				{

					receiveConsecutiveFrame(&receiveMessage, ECU.RxData);
					printf("Get data at ECU: ");
					showData(receiveMessage.data);
					receiveMessage.serviceInfo.SID += 0x40;
					sendresponseFrame(&receiveMessage.serviceInfo, &ECU);
					break;
				}
				case 0x3:{
					break;
				}
				default:
					break;
				}
		}

	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &Tester.RxHeader, Tester.RxData);
	if (Tester.RxHeader.DLC == 8)
	{
		uint8_t frameType = Tester.RxData[0] >> 4;
		switch(frameType){
			case 0x0:{
				switch (Tester.RxData[1])
				{
					case 0x62:	// Response for Read Data by Identifier
					{
						CANTPMessage receiveMessage;
						receiveSingleFrame(&receiveMessage, Tester.RxData);
						uint16_t ADC_rx = (receiveMessage.data[0] << 8) + receiveMessage.data[1];
						printf("Get ADC from ECU: 0x%02X\n", ADC_rx);
						printf("End session!\n");
						break;
					}
					case 0x6E: // Response for Write Data by Identifier
					{
						printf("End session!\n");
						break;
					}
					case 0x67: // Response for Write Data by Identifier
					{
						CANTPMessage receiveMessage;
						receiveSingleFrame(&receiveMessage, Tester.RxData);
						if(receiveMessage.serviceInfo.SF == 0x01){
							sendKey(&receiveMessage, &Tester);
						}
						else
						{
							printf("*** ECU unlocked *** \n");
						}
						showResponseFrame(&receiveMessage);
						break;
					}
					case 0x7F:
					{
						CANTPMessage receiveMessage;
						receiveSingleFrame(&receiveMessage, Tester.RxData);
						showResponseFrame(&receiveMessage);
						break;
					}
				}
				break;
			}
		case 0x3:
		{
			sendConsecutiveFrame(&sendMessage, &Tester);
			break;
		}

		default:
			break;
		}
	}
}

void CANTP(CANTPMessage *sendMessage, CANTPCanInit *Node){

    uint8_t paddingNumber;
    if(sendMessage->bufferLength <= 7){
        paddingNumber = 7 - (sendMessage->bufferLength % 7);
        sendMessage->bufferLength += paddingNumber;
	}
	else{
        paddingNumber = 7 - (sendMessage->bufferLength % 7) - 1;
        sendMessage->bufferLength += paddingNumber;
	}

	if(sendMessage->bufferLength > 7){
		sendFirstFrame(sendMessage, Node);
	}
	else{
		sendSingleFrame(sendMessage, Node);
	}
}

void writeDataByIdentifier(uint8_t *Data, uint8_t dataLength, uint8_t MSB_DID, uint8_t LSB_DID)
{
	printf("-------------Service $2E---------------\n");
	printf("Write data from Tester: ");
	showData(Data);
	sendMessage.serviceInfo.SID = 0x2E;
	sendMessage.serviceInfo.DID[0] = MSB_DID;
	sendMessage.serviceInfo.DID[1] = LSB_DID;
	sendMessage.dataLength = dataLength;
	sendMessage.bufferLength = sizeof(sendMessage.serviceInfo) + sendMessage.dataLength - 1;
	sendMessage.data = (uint8_t*)malloc(dataLength);
  	memcpy(sendMessage.data, Data, dataLength);
  	CANTP(&sendMessage, &Tester);

}


void readDataByIdentifier(uint8_t MSB_DID, uint8_t LSB_DID)
{
	printf("-------------Service $22---------------\n");
	CANTPMessage sendMessage;
	sendMessage.serviceInfo.SID = 0x22;
	sendMessage.serviceInfo.DID[0] = MSB_DID;
	sendMessage.serviceInfo.DID[1] = LSB_DID;
	sendMessage.serviceInfo.SF = 0x00;

	sendMessage.dataLength = 0;
	sendMessage.bufferLength = sizeof(sendMessage.serviceInfo) + sendMessage.dataLength - 1;
  	CANTP(&sendMessage, &Tester);
}

void sendSeedRequest(uint8_t SubFunction)
{
	printf("-------------Service $27---------------\n");

	sendMessage.serviceInfo.SID = 0x27;
	sendMessage.serviceInfo.SF = SubFunction;

	sendMessage.dataLength = 0;
	sendMessage.bufferLength = sizeof(sendMessage.serviceInfo) + sendMessage.dataLength - 2;
  	CANTP(&sendMessage, &Tester);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_4) { 		// Joystick at Left position
		uint8_t Data[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
		uint8_t dataLength = sizeof(Data);
		writeDataByIdentifier(Data, dataLength, 0xF0, 0x02);

	}
	else if (GPIO_Pin == GPIO_PIN_7) {	// Right
		uint8_t Data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		uint8_t dataLength = sizeof(Data);
		writeDataByIdentifier(Data, dataLength, 0xF0, 0x02);

	}
	else if (GPIO_Pin == GPIO_PIN_13) {	// Middle
		uint8_t Data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		uint8_t dataLength = sizeof(Data);
		writeDataByIdentifier(Data, dataLength, 0xF0, 0x02);
	}
	else if (GPIO_Pin == GPIO_PIN_1) {	// User button
			sendSeedRequest(0x01);
	}
	else if (GPIO_Pin == GPIO_PIN_6) {	// Down
		readDataByIdentifier(0xF0, 0x00);
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  // Response from ECU
  ECU.hcan = hcan1;
  ECU.TxHeader.DLC = 8;
  ECU.TxHeader.IDE = CAN_ID_STD;
  ECU.TxHeader.RTR = CAN_RTR_DATA;
  ECU.TxHeader.StdId = 0x7A2;

  // Request from Tester
  Tester.hcan = hcan2;
  Tester.TxHeader.DLC = 8;
  Tester.TxHeader.IDE = CAN_ID_STD;
  Tester.TxHeader.RTR = CAN_RTR_DATA;
  Tester.TxHeader.StdId = 0x712;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 18;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x712<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x712<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 19;//10;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  canfilterconfig.FilterIdHigh = 0x7A2<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x7A2<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;  // doesn't matter in single can controllers

  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC4 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
 HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);


 return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
