#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


typedef enum {
    PCI_SINGLE_FRAME = 0x00,
    PCI_FIRST_FRAME = 0x10,
    PCI_CONSECUTIVE_FRAME = 0x20,
    PCI_FLOW_CONTROL_FRAME = 0x30
} CANTPControlInformation;

typedef enum {
    PCI_FLOW_STATUS_CONTINUE = 0x00,
    PCI_FLOW_STATUS_WAIT = 0x01,
    PCI_FLOW_STATUS_OVERFLOW = 0x02,
	BLOCK_SIZE = 0x00,
	SEPARATION_TIME = 0x00
} CANTPFlowStatus;

// Store configuration of CAN node
typedef struct {
	CAN_HandleTypeDef hcan;
	CAN_TxHeaderTypeDef TxHeader;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t TxData[8];
	uint8_t RxData[8];
	uint32_t TxMailbox;
} CANTPCanInit;


typedef struct {
    uint8_t SID;					// Service ID - SID
    uint8_t SF;						// Sub-Function
    uint8_t DID[2];					// Data ID - DID
} CANTPService;

typedef struct {
    CANTPService serviceInfo;		// Store SID, MSB_DID, LSB_DID
    uint16_t bufferLength;			// Length of packet to be transmitted/received
    uint16_t dataLength;			// Length of data to be transmitted/received
    uint8_t offset;					// Offset of data array
    uint8_t *data;					// Array data transmitted/received
} CANTPMessage;


/*
* single frame
* +-----------------------------------------+
* |     indicate[0]  |    payload[1:7]      |
* +-----------------------------------------+
* +-----------------------------------+-----+
* |               byte #0             | ... |
* +-----------------------------------+-----+
* | nibble #0[7:4]   | nibble #1[3:0] | ... |
* +-------------+---------------------+ ... +
* |   PCIType = 0    |    SF_DL       | ... |
* +-------------+---------------------+-----+
*/

typedef struct {
    uint8_t indicate;
    uint8_t payload[7];
} CANTPSingleFrame;

/*
* first frame
* +-------------------------+-------------------------------------+
* | indicate[0] | datalength[1] | SID[2] | DID[3:4] | payload[5:7] |
* +-------------------------+-----------+-------------------------+
* +-------------------------+-----------------------+-----+
* | byte #0                 | byte #1               | ... |
* +-------------------------+-----------+-----------+-----+
* | nibble #0   | nibble #1 | nibble #2 | nibble #3 | ... |
* +-------------+-----------+-----------+-----------+-----+
* | PCIType = 1 | FF_DL                             | ... |
* +-------------+-----------+-----------------------+-----+
*/

typedef struct {
    uint8_t indicate;
    uint8_t dataLength;
    uint8_t SID;
    uint8_t DID[2];
    uint8_t payload[3];
} CANTPFirstFrame;

/*
* consecutive frame
* +-----------------------------------------+
* |     indicate[0]  |    payload[1:7]      |
* +-----------------------------------------+
* +-------------------------+-----+
* | byte #0                 | ... |
* +-------------------------+-----+
* | nibble #0   | nibble #1 | ... |
* +-------------+-----------+ ... +
* | PCIType = 0 | SN        | ... |
* +-------------+-----------+-----+
*/

typedef struct {
    uint8_t indicate;
    uint8_t payload[7];
} CANTPConsecutiveFrame;


/*
* flow control frame
* +-------------------------+-------------------------------------+
* | indicate[0] | blockSize[1] | separationTime[2]   |    NA[3:7] |
* +-------------------------+-----------+-------------------------+
* +-------------------------+-----------------------+-----------------------+-----+
* | byte #0                 | byte #1               | byte #2               | ... |
* +-------------------------+-----------+-----------+-----------+-----------+-----+
* | nibble #0   | nibble #1 | nibble #2 | nibble #3 | nibble #4 | nibble #5 | ... |
* +-------------+-----------+-----------+-----------+-----------+-----------+-----+
* | PCIType = 1 | FlowStatus| Block Size            | STmin                 | ... |
* +-------------+-----------+-----------------------+-----------------------+-----+
*/


typedef struct {
    uint8_t indicate;
    uint8_t blockSize;
    uint8_t separationTime;
    uint8_t NA[5];

} CANTPFlowControlFrame;
