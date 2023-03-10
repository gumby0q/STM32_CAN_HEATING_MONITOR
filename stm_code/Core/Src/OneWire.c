/*
 * OneWire.c
 *
 */

#include <OneWire.h>

#include <stdio.h>

#define OW_0	0x00
#define OW_1	0xff
#define OW_R_1	0xff

uint8_t ow_buf[8];
uint8_t ow_buf_rx[16];

//#define TIMEOUT_MS (250)
#define OW_RESET_TIMEOUT_VALUE (0x00000100U)
#define OW_DATA_SEND_TIMEOUT_VALUE  (0x00001000U)
#define OW_DATA_SEND_BITS_TIMEOUT_VALUE (0x00001100U)


static void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits);
static uint8_t OW_toByte(uint8_t *ow_bits);
static uint8_t OW_UART_Init(uint32_t baudRate);


HAL_StatusTypeDef HAL_SnglWireFullDuplex_EnableRTX(UART_HandleTypeDef *huart)
{
  __HAL_LOCK(huart);
  huart->gState = HAL_UART_STATE_BUSY;

  /* Enable the USART's transmit and receive interface by setting the TE bit in the USART CR1 register */
  SET_BIT(huart->Instance->CR1, (USART_CR1_TE | USART_CR1_RE));

  huart->gState = HAL_UART_STATE_READY;

  __HAL_UNLOCK(huart);

  return HAL_OK;
}


HAL_StatusTypeDef HAL_SnglWireFullDuplex_DisableRTX(UART_HandleTypeDef *huart)
{
  __HAL_LOCK(huart);
  huart->gState = HAL_UART_STATE_BUSY;

  /* Disable both Receive and transmit by Clearing TE and RE bits */
  CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TE | USART_CR1_RE));

  huart->gState = HAL_UART_STATE_READY;

  __HAL_UNLOCK(huart);

  return HAL_OK;
}



/*----------------------------------------------------------*/
static void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		if (ow_byte & 0x01)
		{
			*ow_bits = OW_1;
		}
		else
		{
			*ow_bits = OW_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

static uint8_t OW_toByte(uint8_t *ow_bits)
{
	uint8_t ow_byte, i;
	ow_byte = 0;
	for (i = 0; i < 8; i++)
	{
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1)
		{
			ow_byte |= 0x80;
		}
		ow_bits++;
	}

	return ow_byte;
}


static uint8_t OW_SendBits(uint8_t num_bits)
{

		// HAL_HalfDuplex_EnableTransmitter(&HUARTx);
	// HAL_UART_Transmit_DMA(&HUARTx, ow_buf, num_bits);
		// HAL_HalfDuplex_EnableReceiver(&HUARTx);
	// HAL_UART_Receive_IT(&HUARTx, ow_buf, num_bits, HAL_MAX_DELAY);
	// HAL_UART_Transmit_IT(&HUARTx, ow_buf, num_bits, HAL_MAX_DELAY);
	// HAL_UART_Receive_IT(&HUARTx, ow_buf_rx, num_bits);
	// HAL_UART_Transmit_IT(&HUARTx, ow_buf, num_bits);

	HAL_UART_Receive_DMA(&HUARTx, ow_buf, num_bits);
	HAL_UART_Transmit_DMA(&HUARTx, ow_buf, num_bits);

//	while (HAL_UART_GetState(&HUARTx) != HAL_UART_STATE_READY)
//	{
//		__NOP();
//	}

		uint32_t tickstart = 0U;
		tickstart = HAL_GetTick();

		while(HAL_UART_GetState(&HUARTx) != HAL_UART_STATE_READY) {
			if ((HAL_GetTick() - tickstart) > OW_DATA_SEND_BITS_TIMEOUT_VALUE)
			{
//				return 0;
				return OW_TIMEOUT;
			}
		}

		// memcpy(ow_buf, ow_buf_rx, num_bits);

		return 0;
}

static uint8_t OW_UART_Init(uint32_t baudRate)
{
    HUARTx.Instance = USARTx;
    HUARTx.Init.BaudRate = baudRate;
    HUARTx.Init.WordLength = UART_WORDLENGTH_8B;
    HUARTx.Init.StopBits = UART_STOPBITS_1;
    HUARTx.Init.Parity = UART_PARITY_NONE;
    HUARTx.Init.Mode = UART_MODE_TX_RX;
    HUARTx.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HUARTx.Init.OverSampling = UART_OVERSAMPLING_16;

	return HAL_HalfDuplex_Init(&HUARTx);
}

uint8_t OW_Reset(void)
{
	uint8_t ow_presence = 0xf0;
	uint8_t _ow_presence = 0;

	OW_UART_Init(9600);


		// HAL_HalfDuplex_EnableTransmitter(&HUARTx);
	// HAL_UART_Transmit_DMA(&HUARTx, &ow_presence, 1);

		// HAL_HalfDuplex_EnableReceiver(&HUARTx);
	// HAL_UART_Receive_DMA(&HUARTx, &ow_presence, 1);
	// HAL_UART_Receive_IT(&HUARTx, &ow_presence, 1, HAL_MAX_DELAY);
	// HAL_UART_Transmit_IT(&HUARTx, &ow_presence, 1, HAL_MAX_DELAY);
	// HAL_UART_Receive_IT(&HUARTx, &_ow_presence, 1);
	// HAL_UART_Transmit_IT(&HUARTx, &ow_presence, 1);

	HAL_UART_Receive_DMA(&HUARTx, &ow_presence, 1);
	HAL_UART_Transmit_DMA(&HUARTx, &ow_presence, 1);


	uint32_t tickstart = 0U;
	tickstart = HAL_GetTick();

	while(HAL_UART_GetState(&HUARTx) != HAL_UART_STATE_READY) {
		if ((HAL_GetTick() - tickstart) > OW_RESET_TIMEOUT_VALUE)
		{

		  return OW_TIMEOUT;
		}
	}

	// ow_presence = _ow_presence;

	OW_UART_Init(115200);

	if (ow_presence != 0xf0)
	{
		return OW_OK;
	}

	return OW_NO_DEVICE;
}

uint8_t OW_Init(void)
{

	// HAL_SnglWireFullDuplex_EnableRTX(&HUARTx);

	return OW_UART_Init(9600);
}

uint8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart)
{
	uint8_t error = 0;
	if (sendReset == OW_SEND_RESET)
	{
		error = OW_Reset();
		if (error != 0)
		{
			// printf("-0-test\n");
			return error;
		}
	}

	// printf("-1-test\n");

	while (cLen > 0)
	{
		OW_toBits(*command, ow_buf);
		command++;
		cLen--;

		char buff[30];

		// printf("-2-test\n");
		// HAL_HalfDuplex_EnableTransmitter(&HUARTx);
		// error = HAL_UART_Transmit_DMA(&HUARTx, ow_buf, sizeof(ow_buf));
		// error = HAL_UART_Receive_IT(&HUARTx, ow_buf, sizeof(ow_buf), HAL_MAX_DELAY);
		// error = HAL_UART_Receive_IT(&HUARTx, ow_buf_rx, sizeof(ow_buf));
		// sprintf(buff, "-3-test error=%d \n", error);
		// printf(buff);
		// HAL_HalfDuplex_EnableReceiver(&HUARTx);
		// error = HAL_UART_Receive_DMA(&HUARTx, ow_buf, sizeof(ow_buf));
		// error = HAL_UART_Transmit_IT(&HUARTx, ow_buf, sizeof(ow_buf), HAL_MAX_DELAY);
		// sprintf(buff, "-4-test error=%d \n", error);
		// printf(buff);

		// error = HAL_UART_Receive_IT(&HUARTx, ow_buf_rx, 8);
		// error = HAL_UART_Transmit_IT(&HUARTx, ow_buf, sizeof(ow_buf));

		error = HAL_UART_Receive_DMA(&HUARTx, ow_buf, sizeof(ow_buf));
		error = HAL_UART_Transmit_DMA(&HUARTx, ow_buf, sizeof(ow_buf));

		// char buf[30];
		// uint8_t tmp_byte = 0;
		// uint8_t tmp_byte2 = 0;
		// uint8_t tmp_byte3 = 0;
		// tmp_byte = OW_toByte(ow_buf);
		// tmp_byte2 = OW_toByte(ow_buf_rx);
		// tmp_byte3 = OW_toByte(&ow_buf_rx[8]);
		// sprintf(buf, "%d - %d - %d \r\n", tmp_byte, tmp_byte2, tmp_byte3);
		// printf(buf);

		uint32_t tickstart = 0U;
		tickstart = HAL_GetTick();

		while(HAL_UART_GetState(&HUARTx) != HAL_UART_STATE_READY) {
			if ((HAL_GetTick() - tickstart) > OW_DATA_SEND_TIMEOUT_VALUE)
			{
				return OW_TIMEOUT;
			}
		}

		// memcpy(ow_buf, ow_buf_rx, sizeof(ow_buf));

		// printf("-5-test\n");

		if (readStart == 0 && dLen > 0)
		{
			*data = OW_toByte(ow_buf);
			data++;
			dLen--;
		}
		else
		{
			if (readStart != OW_NO_READ)
			{
				readStart--;
			}
		}
	}

	return OW_OK;
}

#if ONEWIRE_SEARCH

uint8_t OW_Search(uint8_t *buf, uint8_t num, uint8_t *found)
{

	uint8_t error_status = 0;

	char _test_str[40];
	uint8_t _test_found = 0;

	*found = 0;
	uint8_t *lastDevice = NULL;
	uint8_t *curDevice = buf;
	uint8_t numBit, lastCollision, currentCollision, currentSelection;

	lastCollision = 0;

	// sprintf(_test_str, "check 0 _test_found=%d num=%d\n", _test_found, num);
	// printf(_test_str);

	while (_test_found < num)
	{
		numBit = 1;
		currentCollision = 0;

		error_status = OW_Send(OW_SEND_RESET, (uint8_t*)"\xf0", 1, NULL, 0, OW_NO_READ);

		// sprintf(_test_str, "check 1 error_status=%d num=%d\n", error_status, num);
		// printf(_test_str);

		if (error_status != 0) {
			return error_status;
		}

		for (numBit = 1; numBit <= 64; numBit++)
		{
			OW_toBits(OW_READ_SLOT, ow_buf);

			error_status = OW_SendBits(2);

			if (error_status != 0) {
				return error_status;
			}

			if (ow_buf[0] == OW_R_1)
			{
				if (ow_buf[1] == OW_R_1)
				{
					return 0;
				}
				else
				{
					currentSelection = 1;
				}
			}
			else
			{
				if (ow_buf[1] == OW_R_1)
				{
					currentSelection = 0;
				}
				else
				{
					if (numBit < lastCollision)
					{
							if (lastDevice[(numBit - 1) >> 3] & 1 << ((numBit - 1) & 0x07))
							{
							currentSelection = 1;

								if (currentCollision < numBit)
								{
										currentCollision = numBit;
								}
							}
							else
							{
								currentSelection = 0;
							}
					}
					else
					{
						if (numBit == lastCollision)
						{
								currentSelection = 0;
						}
						else
						{
							currentSelection = 1;

							if (currentCollision < numBit)
							{
									currentCollision = numBit;
							}
						}
					}
				}
			}

			if (currentSelection == 1)
			{
				curDevice[(numBit - 1) >> 3] |= 1 << ((numBit - 1) & 0x07);
				OW_toBits(0x01, ow_buf);
			}
			else
			{
				curDevice[(numBit - 1) >> 3] &= ~(1 << ((numBit - 1) & 0x07));
				OW_toBits(0x00, ow_buf);
			}

			error_status = OW_SendBits(1);

			if (error_status != 0) {
				return error_status;
			}
		}

		_test_found = _test_found + 1;
		// sprintf(_test_str, "_test_found %d \n", _test_found);
		// printf(_test_str);
		// *found = *found + 1;
		*found = _test_found;

		lastDevice = curDevice;
		curDevice += 8;
		if (currentCollision == 0)
		{

			return 0;
		}

		lastCollision = currentCollision;
	}

	return 0;
}

#endif

#if ONEWIRE_CRC
// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

#if ONEWIRE_CRC8_TABLE
// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t dscrc2x16_table[] = {
	0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
	0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
	0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
	0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};

// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (Use tiny 2x16 entry CRC table)
uint8_t OW_crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--)
	{
		crc = *addr++ ^ crc;  // just re-using crc as intermediate
		crc = dscrc2x16_table[crc & 0x0f] ^ dscrc2x16_table[16 + ((crc >> 4) & 0x0f)];
	}

	return crc;
}
#else
//
// Compute a Dallas Semiconductor 8 bit CRC directly.
// this is much slower, but a little smaller, than the lookup table.
//
uint8_t OW_crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--)
	{
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--)
		{
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}
#endif

#if ONEWIRE_CRC16
bool OW_Check_crc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc)
{
    crc = ~OW_crc16(input, len, crc);
    return (crc & 0xFF) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
}

uint16_t OW_crc16(const uint8_t* input, uint16_t len, uint16_t crc)
{
    static const uint8_t oddparity[16] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

    for (uint16_t i = 0 ; i < len ; i++)
    {
      // Even though we're just copying a byte from the input,
      // we'll be doing 16-bit computation with it.
      uint16_t cdata = input[i];
      cdata = (cdata ^ crc) & 0xff;
      crc >>= 8;

      if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
          crc ^= 0xC001;

      cdata <<= 6;
      crc ^= cdata;
      cdata <<= 1;
      crc ^= cdata;
    }

    return crc;
}
#endif

#endif
