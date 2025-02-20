#ifndef SRC_HARDWARE_SAME4S_4E_E70_ASYNC_SERIAL485_H_
#define SRC_HARDWARE_SAME4S_4E_E70_ASYNC_SERIAL485_H_

#include <Stream.h>
#include <General/RingBuffer.h>

#ifdef RTOS
# include <RTOSIface/RTOSIface.h>
#endif

#if SAM4E || SAME70
#include "component/usart.h"
#include "AsyncSerial.h"
#endif

class AsyncSerial485 : public AsyncSerial
{
public:

	AsyncSerial485(Uart* pUart, Pin selectPin, IRQn_Type p_irqn, uint32_t p_id, size_t numTxSlots, size_t numRxSlots, OnBeginFn p_onBegin, OnEndFn p_onEnd) noexcept;

	size_t write(const uint8_t c) noexcept override;
	size_t write(const uint8_t *buffer, size_t buflen) noexcept override;

	void IrqHandler() noexcept;

protected:
	// We need to initialize the select pin
	void init(const uint32_t dwBaudRate, const uint32_t config) noexcept;

	Pin _selectPin;			// Select Pin to enable tx or rx
private:
	volatile bool doneTx = false;
	void EnableTransmission() noexcept;
	void EnableReception() noexcept;

};

#endif // SRC_HARDWARE_SAME4S_4E_E70_ASYNC_SERIAL485_H_
