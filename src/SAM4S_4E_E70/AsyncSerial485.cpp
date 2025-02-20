
#include "AsyncSerial485.h"
#include <CoreNotifyIndices.h>
#include <asf.h>

#include <cstdlib>
#include <cstring>
#include <algorithm>		// for std::swap

#include <Core.h>
// Constructors ////////////////////////////////////////////////////////////////

AsyncSerial485::AsyncSerial485(Uart* pUart, Pin selectPin, IRQn_Type p_irqn, uint32_t p_id, size_t numTxSlots, size_t numRxSlots, OnBeginFn p_onBegin, OnEndFn p_onEnd) noexcept
	: AsyncSerial(pUart, p_irqn, p_id, numTxSlots, numRxSlots, p_onBegin, p_onEnd),_selectPin(selectPin)
{
}

// Public Methods //////////////////////////////////////////////////////////////

void AsyncSerial485::init(const uint32_t dwBaudRate, const uint32_t modeReg) noexcept
{
	pinMode(_selectPin, OUTPUT_LOW);
	EnableReception();
	AsyncSerial::init(dwBaudRate, modeReg);
}

void AsyncSerial485::EnableTransmission() noexcept
{
	digitalWrite(_selectPin,true);
}
void AsyncSerial485::EnableReception() noexcept
{
	digitalWrite(_selectPin,false);
}


size_t AsyncSerial485::write(uint8_t uc_data) noexcept
{
	return write(&uc_data,1);
}

size_t AsyncSerial485::write(const uint8_t *buffer, size_t buflen) noexcept
{
	const size_t ret = buflen;
	EnableTransmission();
	for (;;)
	{
		buflen -= txBuffer.PutBlock(buffer, buflen);
		if (buflen == 0)
		{
#ifdef RTOS
		    txWaitingTask = RTOSIface::GetCurrentTask();
		    _pUart->UART_IER = UART_IER_TXRDY;
		    TaskBase::TakeIndexed(NotifyIndices::UartTx, 50);	 // The Tx interrupt will wake us up.
		    break;
#endif
		}
		else
		{
			// Error with the buffer.
		}
	}
	EnableReception();
	return ret;
}

void AsyncSerial485::IrqHandler() noexcept
{
	const uint32_t status = _pUart->UART_SR;

	// Did we receive data?
	if ((status & UART_SR_RXRDY) != 0)
	{
		const uint8_t c = _pUart->UART_RHR;
		if (c == interruptSeq[numInterruptBytesMatched])
		{
			++numInterruptBytesMatched;
			if (numInterruptBytesMatched == ARRAY_SIZE(interruptSeq))
			{
				numInterruptBytesMatched = 0;
				if (interruptCallback != nullptr)
				{
					interruptCallback(this);
				}
			}
		}
		else
		{
			numInterruptBytesMatched = 0;
		}

		if (bufferOverrunPending)
		{
			if (rxBuffer.PutItem(0x7F))
			{
				bufferOverrunPending = false;
				(void)rxBuffer.PutItem(c);					// we don't much care whether this succeeds or not
			}
		}
		else if (!rxBuffer.PutItem(c))
		{
			++errors.bufferOverrun;
			bufferOverrunPending = true;
		}
	}

	if ((status & UART_SR_TXEMPTY) != 0 && (_pUart->UART_IMR & UART_IMR_TXEMPTY) != 0)
	{
		_pUart->UART_IDR = UART_IDR_TXEMPTY;			// mask off transmit interrupt so we don't get it anymore
#ifdef RTOS
		if (txWaitingTask != nullptr)
		{
			TaskBase::GiveFromISR(txWaitingTask, NotifyIndices::UartTx);
			txWaitingTask = nullptr;
		}
#endif
	}
	// Do we need to keep sending data?
	if ((status & UART_SR_TXRDY) != 0 && (_pUart->UART_IMR & UART_IMR_TXRDY) != 0)
	{
		uint8_t c;
		if (txBuffer.GetItem(c))
		{
			_pUart->UART_THR = c;
		}
		else
		{
			_pUart->UART_IDR = UART_IDR_TXRDY;			// mask off transmit interrupt so we don't get it anymore
			_pUart->UART_IER = UART_IER_TXEMPTY;
		}
	}

	// Acknowledge errors
	if ((status & (UART_SR_OVRE | UART_SR_FRAME)) != 0)
	{
		if (status & UART_SR_OVRE)
		{
			++errors.uartOverrun;
		}
		if (status & UART_SR_FRAME)
		{
			++errors.framing;
		}
		_pUart->UART_CR = UART_CR_RSTSTA;
		rxBuffer.PutItem(0x7F);
	}
}


// End
