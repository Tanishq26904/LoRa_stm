#include "LoRa.h"

/* ----------------------------------------------------------------------------- *\
		name        : newLoRa

		description : it's a constructor for LoRa structure that assign default values
									and pass created object (LoRa struct instanse)

		arguments   : Nothing

		returns     : A LoRa object whith these default values:
											----------------------------------------
										  |   carrier frequency = 433 MHz        |
										  |    spreading factor = 7				       |
											|           bandwidth = 125 KHz        |
											| 		    coding rate = 4/5            |
											----------------------------------------
\* ----------------------------------------------------------------------------- */
LoRa newLoRa(){
	LoRa new_LoRa;

	new_LoRa.frequency             = 433       ;
	new_LoRa.spredingFactor        = SF_7      ;
	new_LoRa.bandWidth			   = BW_125KHz ;
	new_LoRa.crcRate               = CR_4_5    ;
	new_LoRa.power				   = POWER_20db;
	new_LoRa.overCurrentProtection = 100       ;
	new_LoRa.preamble			   = 8         ;

	return new_LoRa;
}
/* ----------------------------------------------------------------------------- *\
		name        : LoRa_reset

		description : reset module

		arguments   :
			LoRa* LoRa --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_reset(LoRa* _LoRa){
	HAL_GPIO_WritePin(_LoRa->reset_port, _LoRa->reset_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(_LoRa->reset_port, _LoRa->reset_pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_gotoMode

		description : set LoRa Op mode

		arguments   :
			LoRa* LoRa    --> LoRa object handler
			mode	        --> select from defined modes

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_gotoMode(LoRa* _LoRa, int mode){
	uint8_t    read;
	uint8_t    data;

	read = LoRa_read(_LoRa, RegOpMode);
	data = read;

	if(mode == SLEEP_MODE){
		data = (read & 0xF8) | 0x00;
		_LoRa->current_mode = SLEEP_MODE;
	}else if (mode == STNBY_MODE){
		data = (read & 0xF8) | 0x01;
		_LoRa->current_mode = STNBY_MODE;
	}else if (mode == TRANSMIT_MODE){
		data = (read & 0xF8) | 0x03;
		_LoRa->current_mode = TRANSMIT_MODE;
	}else if (mode == RXCONTIN_MODE){
		data = (read & 0xF8) | 0x05;
		_LoRa->current_mode = RXCONTIN_MODE;
	}else if (mode == RXSINGLE_MODE){
		data = (read & 0xF8) | 0x06;
		_LoRa->current_mode = RXSINGLE_MODE;
	}else if (mode == CAD_MODE){  // Add this case
		data = (read & 0xF8) | 0x07;
		_LoRa->current_mode = CAD_MODE;
	}

	LoRa_write(_LoRa, RegOpMode, data);
	//HAL_Delay(10);
}


/* ----------------------------------------------------------------------------- *\
		name        : LoRa_readReg

		description : read a register(s) by an address and a length,
									then store value(s) at outpur array.
		arguments   :
			LoRa* LoRa        --> LoRa object handler
			uint8_t* address  -->	pointer to the beginning of address array
			uint16_t r_length -->	detemines number of addresse bytes that
														you want to send
			uint8_t* output		--> pointer to the beginning of output array
			uint16_t w_length	--> detemines number of bytes that you want to read

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_readReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* output, uint16_t w_length){
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_LoRa->hSPIx, address, r_length, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
		;
	HAL_SPI_Receive(_LoRa->hSPIx, output, w_length, RECEIVE_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_writeReg

		description : write a value(s) in a register(s) by an address

		arguments   :
			LoRa* LoRa        --> LoRa object handler
			uint8_t* address  -->	pointer to the beginning of address array
			uint16_t r_length -->	detemines number of addresse bytes that
														you want to send
			uint8_t* output		--> pointer to the beginning of values array
			uint16_t w_length	--> detemines number of bytes that you want to send

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_writeReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* values, uint16_t w_length){
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_LoRa->hSPIx, address, r_length, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
		;
	HAL_SPI_Transmit(_LoRa->hSPIx, values, w_length, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setLowDaraRateOptimization

		description : set the LowDataRateOptimization flag. Is is mandated for when the symbol length exceeds 16ms.

		arguments   :
			LoRa*	LoRa         --> LoRa object handler
			uint8_t	value        --> 0 to disable, otherwise to enable

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setLowDaraRateOptimization(LoRa* _LoRa, uint8_t value){
	uint8_t	data;
	uint8_t	read;

	read = LoRa_read(_LoRa, RegModemConfig3);
	
	if(value)
		data = read | 0x08;
	else
		data = read & 0xF7;

	LoRa_write(_LoRa, RegModemConfig3, data);
	HAL_Delay(10);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setAutoLDO

		description : set the LowDataRateOptimization flag automatically based on the symbol length.

		arguments   :
			LoRa*	LoRa         --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setAutoLDO(LoRa* _LoRa){
	double BW[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0};
	
	LoRa_setLowDaraRateOptimization(_LoRa, (long)((1 << _LoRa->spredingFactor) / ((double)BW[_LoRa->bandWidth])) > 16.0);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setFrequency

		description : set carrier frequency e.g 433 MHz

		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   freq        --> desired frequency in MHz unit, e.g 434

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setFrequency(LoRa* _LoRa, int freq){
	uint8_t  data;
	uint32_t F;
	F = (freq * 524288)>>5;

	// write Msb:
	data = F >> 16;
	LoRa_write(_LoRa, RegFrMsb, data);
	HAL_Delay(5);

	// write Mid:
	data = F >> 8;
	LoRa_write(_LoRa, RegFrMid, data);
	HAL_Delay(5);

	// write Lsb:
	data = F >> 0;
	LoRa_write(_LoRa, RegFrLsb, data);
	HAL_Delay(5);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setSpreadingFactor

		description : set spreading factor, from 7 to 12.

		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   SP          --> desired spreading factor e.g 7

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setSpreadingFactor(LoRa* _LoRa, int SF){
	uint8_t	data;
	uint8_t	read;

	if(SF>12)
		SF = 12;
	if(SF<7)
		SF = 7;

	read = LoRa_read(_LoRa, RegModemConfig2);
	HAL_Delay(10);

	data = (SF << 4) + (read & 0x0F);
	LoRa_write(_LoRa, RegModemConfig2, data);
	HAL_Delay(10);
	
	LoRa_setAutoLDO(_LoRa);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setPower

		description : set power gain.

		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   power       --> desired power like POWER_17db

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setPower(LoRa* _LoRa, uint8_t power){
	LoRa_write(_LoRa, RegPaConfig, power);
	HAL_Delay(10);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setOCP

		description : set maximum allowed current.

		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   current     --> desired max currnet in mA, e.g 120

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setOCP(LoRa* _LoRa, uint8_t current){
	uint8_t	OcpTrim = 0;

	if(current<45)
		current = 45;
	if(current>240)
		current = 240;

	if(current <= 120)
		OcpTrim = (current - 45)/5;
	else if(current <= 240)
		OcpTrim = (current + 30)/10;

	OcpTrim = OcpTrim + (1 << 5);
	LoRa_write(_LoRa, RegOcp, OcpTrim);
	HAL_Delay(10);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setTOMsb_setCRCon

		description : set timeout msb to 0xFF + set CRC enable.

		arguments   :
			LoRa* LoRa        --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setTOMsb_setCRCon(LoRa* _LoRa){
	uint8_t read, data;

	read = LoRa_read(_LoRa, RegModemConfig2);

	data = read | 0x07;
	LoRa_write(_LoRa, RegModemConfig2, data);\
	HAL_Delay(10);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setTOMsb_setCRCon

		description : set timeout msb to 0xFF + set CRC enable.

		arguments   :
			LoRa* LoRa        --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setSyncWord(LoRa* _LoRa, uint8_t syncword){
	LoRa_write(_LoRa, RegSyncWord, syncword);
	HAL_Delay(10);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_read

		description : read a register by an address

		arguments   :
			LoRa*   LoRa        --> LoRa object handler
			uint8_t address     -->	address of the register e.g 0x1D

		returns     : register value
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_read(LoRa* _LoRa, uint8_t address){
	uint8_t read_data;
	uint8_t data_addr;

	data_addr = address & 0x7F;
	LoRa_readReg(_LoRa, &data_addr, 1, &read_data, 1);
	//HAL_Delay(5);

	return read_data;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_write

		description : write a value in a register by an address

		arguments   :
			LoRa*   LoRa        --> LoRa object handler
			uint8_t address     -->	address of the register e.g 0x1D
			uint8_t value       --> value that you want to write

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_write(LoRa* _LoRa, uint8_t address, uint8_t value){
	uint8_t data;
	uint8_t addr;

	addr = address | 0x80;
	data = value;
	LoRa_writeReg(_LoRa, &addr, 1, &data, 1);
	//HAL_Delay(5);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_BurstWrite

		description : write a set of values in a register by an address respectively

		arguments   :
			LoRa*   LoRa        --> LoRa object handler
			uint8_t address     -->	address of the register e.g 0x1D
			uint8_t *value      --> address of values that you want to write

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_BurstWrite(LoRa* _LoRa, uint8_t address, uint8_t *value, uint8_t length){
	uint8_t addr;
	addr = address | 0x80;

	//NSS = 1
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(_LoRa->hSPIx, &addr, 1, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
		;
	//Write data in FiFo
	HAL_SPI_Transmit(_LoRa->hSPIx, value, length, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
		;
	//NSS = 0
	//HAL_Delay(5);
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}
/* ----------------------------------------------------------------------------- *\
		name        : LoRa_isvalid

		description : check the LoRa instruct values

		arguments   :
			LoRa* LoRa --> LoRa object handler

		returns     : returns 1 if all of the values were given, otherwise returns 0
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_isvalid(LoRa* _LoRa){

	return 1;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_transmit

		description : Transmit data

		arguments   :
			LoRa*    LoRa     --> LoRa object handler
			uint8_t  data			--> A pointer to the data you wanna send
			uint8_t	 length   --> Size of your data in Bytes
			uint16_t timeOut	--> Timeout in milliseconds
		returns     : 1 in case of success, 0 in case of timeout
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_transmit(LoRa* _LoRa, uint8_t* data, uint8_t length, uint16_t timeout){
	uint8_t read;

	int mode = _LoRa->current_mode;
	LoRa_gotoMode(_LoRa, STNBY_MODE);
	read = LoRa_read(_LoRa, RegFiFoTxBaseAddr);
	LoRa_write(_LoRa, RegFiFoAddPtr, read);
	LoRa_write(_LoRa, RegPayloadLength, length);
	LoRa_BurstWrite(_LoRa, RegFiFo, data, length);
	LoRa_gotoMode(_LoRa, TRANSMIT_MODE);
	while(1){
		read = LoRa_read(_LoRa, RegIrqFlags);
		if((read & 0x08)!=0){
			LoRa_write(_LoRa, RegIrqFlags, 0xFF);
			LoRa_gotoMode(_LoRa, mode);
			return 1;
		}
		else{
			if(--timeout==0){
				LoRa_gotoMode(_LoRa, mode);
				return 0;
			}
		}
		HAL_Delay(1);
	}
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_startReceiving

		description : Start receiving continuously

		arguments   :
			LoRa*    LoRa     --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_startReceiving(LoRa* _LoRa){
	LoRa_gotoMode(_LoRa, RXCONTIN_MODE);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_Receive

		description : Read received data from module

		arguments   :
			LoRa*    LoRa     --> LoRa object handler
			uint8_t  data			--> A pointer to the array that you want to write bytes in it
			uint8_t	 length   --> Determines how many bytes you want to read

		returns     : The number of bytes received
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length)
{
    uint8_t irq = LoRa_read(_LoRa, RegIrqFlags);
    uint8_t bytes = 0;

    if (irq & 0x40)   // RxDone
    {
        LoRa_write(_LoRa, RegIrqFlags, 0xFF);

        bytes = LoRa_read(_LoRa, RegRxNbBytes);
        uint8_t addr = LoRa_read(_LoRa, RegFiFoRxCurrentAddr);
        LoRa_write(_LoRa, RegFiFoAddPtr, addr);

        if (bytes > length) bytes = length;

        for (uint8_t i = 0; i < bytes; i++)
            data[i] = LoRa_read(_LoRa, RegFiFo);
    }

    return bytes;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_getRSSI

		description : initialize and set the right setting according to LoRa sruct vars

		arguments   :
			LoRa* LoRa        --> LoRa object handler

		returns     : Returns the RSSI value of last received packet.
\* ----------------------------------------------------------------------------- */
int LoRa_getRSSI(LoRa* _LoRa){
	uint8_t read;
	read = LoRa_read(_LoRa, RegPktRssiValue);
	return -164 + read;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setCADMode

		description : set LoRa to CAD mode

		arguments   :
			LoRa* LoRa    --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setCADMode(LoRa* _LoRa){
	LoRa_gotoMode(_LoRa, CAD_MODE);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_startCAD

		description : Start Channel Activity Detection

		arguments   :
			LoRa*    LoRa     --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_startCAD(LoRa* _LoRa){
	LoRa_setCADMode(_LoRa);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_isCADDetected

		description : Check if CAD detected activity on channel

		arguments   :
			LoRa*    LoRa     --> LoRa object handler

		returns     : 1 if activity detected, 0 otherwise
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_isCADDetected(LoRa* _LoRa){
	uint8_t cadFlags = LoRa_read(_LoRa, RegIrqFlags);

	// Check CadDetected flag (bit 0)
	if(cadFlags & 0x01){
		// Clear CadDetected flag
		LoRa_write(_LoRa, RegIrqFlags, cadFlags & 0xFE);
		return 1;
	}

	// Check CadDone flag (bit 2) - optional, depends on your needs
	// if(cadFlags & 0x04){
	//     LoRa_write(_LoRa, RegIrqFlags, cadFlags & 0xFB);
	// }

	return 0;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_isCADDone

		description : Check if CAD operation is complete

		arguments   :
			LoRa*    LoRa     --> LoRa object handler

		returns     : 1 if CAD done, 0 otherwise
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_isCADDone(LoRa* _LoRa){
	uint8_t cadFlags = LoRa_read(_LoRa, RegIrqFlags);

	// Check CadDone flag (bit 2)
	if(cadFlags & 0x04){
		// Clear CadDone flag
		LoRa_write(_LoRa, RegIrqFlags, cadFlags & 0xFB);
		return 1;
	}

	return 0;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_performCAD

		description : Perform CAD and wait for result with timeout

		arguments   :
			LoRa*    LoRa     --> LoRa object handler
			uint16_t timeout  --> Timeout in milliseconds

		returns     : 1 if activity detected, 0 if no activity, 255 if timeout
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_performCAD(LoRa* _LoRa, uint16_t timeout){
	// Start CAD
	LoRa_startCAD(_LoRa);

	// Wait for CAD to complete
	while(timeout--){
		if(LoRa_isCADDone(_LoRa)){
			// Check if activity was detected
			return LoRa_isCADDetected(_LoRa);
		}
		HAL_Delay(1);
	}

	// Timeout
	return 255;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setCADSymbols

		description : Set number of symbols for CAD (default is 1+ symbols)

		arguments   :
			LoRa*    LoRa     --> LoRa object handler
			uint8_t symbols   --> Number of symbols (valid: 1, 2, 4, 8, 16)

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setCADSymbols(LoRa* _LoRa, uint8_t symbols){
	uint8_t regValue = LoRa_read(_LoRa, 0x31);  // RegDetectOptimize
	uint8_t mask;

	// Validate and set CAD symbols
	if(symbols == 1) mask = 0x00;
	else if(symbols == 2) mask = 0x01;
	else if(symbols == 4) mask = 0x02;
	else if(symbols == 8) mask = 0x03;
	else if(symbols == 16) mask = 0x04;
	else mask = 0x00;  // Default to 1 symbol

	regValue = (regValue & 0xF8) | mask;
	LoRa_write(_LoRa, 0x31, regValue);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setCADExitMode

		description : Set what happens after CAD completes

		arguments   :
			LoRa*    LoRa     --> LoRa object handler
			uint8_t mode      --> 0: Return to STDBY, 1: Return to RX mode

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setCADExitMode(LoRa* _LoRa, uint8_t mode){
	uint8_t regValue = LoRa_read(_LoRa, 0x31);  // RegDetectOptimize

	if(mode){
		// Set bit 7 to return to RX mode after CAD
		regValue |= 0x80;
	} else {
		// Clear bit 7 to return to STDBY after CAD
		regValue &= 0x7F;
	}

	LoRa_write(_LoRa, 0x31, regValue);
}
/* ----------------------------------------------------------------------------- *\
		name        : LoRa_enableCRC

		description : Enable or disable CRC checking

		arguments   :
			LoRa*    LoRa     --> LoRa object handler
			uint8_t enable    --> 1 to enable, 0 to disable

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_enableCRC(LoRa* _LoRa, uint8_t enable){
	uint8_t read = LoRa_read(_LoRa, RegModemConfig2);

	if(enable){
		read |= 0x04;  // Set bit 2 (RxPayloadCrcOn)
	} else {
		read &= ~0x04; // Clear bit 2
	}

	LoRa_write(_LoRa, RegModemConfig2, read);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setCRC

		description : Set CRC explicitly (alternative method)

		arguments   :
			LoRa*    LoRa     --> LoRa object handler
			uint8_t crc       --> 0: CRC off, 1: CRC on

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setCRC(LoRa* _LoRa, uint8_t crc){
	uint8_t config2 = LoRa_read(_LoRa, RegModemConfig2);

	if(crc){
		config2 |= 0x04;  // Enable CRC
	} else {
		config2 &= ~0x04; // Disable CRC
	}

	LoRa_write(_LoRa, RegModemConfig2, config2);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_checkCRCError

		description : Check if last received packet had CRC error

		arguments   :
			LoRa*    LoRa     --> LoRa object handler

		returns     : 1 if CRC error, 0 if no error
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_checkCRCError(LoRa* _LoRa){
	uint8_t irqFlags = LoRa_read(_LoRa, RegIrqFlags);

	// Check PayloadCrcError flag (bit 5)
	if(irqFlags & 0x20){
		// Clear the flag
		LoRa_write(_LoRa, RegIrqFlags, irqFlags);
		return 1;
	}

	return 0;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_init

		description : initialize and set the right setting according to LoRa sruct vars

		arguments   :
			LoRa* LoRa        --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
uint16_t LoRa_init(LoRa* _LoRa){
	uint8_t    data;
	uint8_t    read;

	if(LoRa_isvalid(_LoRa)){
		// goto sleep mode:
			LoRa_gotoMode(_LoRa, SLEEP_MODE);
			HAL_Delay(10);

		// turn on LoRa mode:
			read = LoRa_read(_LoRa, RegOpMode);
			HAL_Delay(10);
			data = read | 0x80;
			LoRa_write(_LoRa, RegOpMode, data);
			HAL_Delay(100);

		// set frequency:
			LoRa_setFrequency(_LoRa, _LoRa->frequency);

		// set output power gain:
			LoRa_setPower(_LoRa, _LoRa->power);

		// set over current protection:
			LoRa_setOCP(_LoRa, _LoRa->overCurrentProtection);

		// set LNA gain:
			LoRa_write(_LoRa, RegLna, 0x23);

		// set spreading factor, CRC on, and Timeout Msb:
			LoRa_setTOMsb_setCRCon(_LoRa);
			LoRa_setSpreadingFactor(_LoRa, _LoRa->spredingFactor);

		// set Timeout Lsb:
			LoRa_write(_LoRa, RegSymbTimeoutL, 0xFF);

		// set bandwidth, coding rate and expilicit mode:
			// 8 bit RegModemConfig --> | X | X | X | X | X | X | X | X |
			//       bits represent --> |   bandwidth   |     CR    |I/E|
			data = 0;
			data = (_LoRa->bandWidth << 4) + (_LoRa->crcRate << 1);
			LoRa_write(_LoRa, RegModemConfig1, data);
			LoRa_setAutoLDO(_LoRa);

		// set preamble:
			LoRa_write(_LoRa, RegPreambleMsb, _LoRa->preamble >> 8);
			LoRa_write(_LoRa, RegPreambleLsb, _LoRa->preamble >> 0);

		// DIO mapping:   --> DIO: RxDone
			read = LoRa_read(_LoRa, RegDioMapping1);
			data = read | 0x3F;
			LoRa_write(_LoRa, RegDioMapping1, data);

		// goto standby mode:
			LoRa_gotoMode(_LoRa, STNBY_MODE);
			_LoRa->current_mode = STNBY_MODE;
			HAL_Delay(10);

			read = LoRa_read(_LoRa, RegVersion);
			if(read == 0x12)
				return LORA_OK;
			else
				return LORA_NOT_FOUND;
	}
	else {
		return LORA_UNAVAILABLE;
	}
}
