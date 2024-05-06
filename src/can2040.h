#pragma once

#include <hardware/structs/pio.h>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iterator>
#include "can2040.h"

class CAN2040;

struct BitUnstuffer {
	uint32_t stuffedBits;
	uint32_t countStuff;
	uint32_t unstuffedBits;
	uint32_t countUnstuff;
};

class CAN2040 {
public:
	struct BusStatistics {
		uint32_t rxTotal;
		uint32_t txTotal;
		uint32_t txAttempt;
		uint32_t parseError;
	};

	struct Message {
		uint32_t id;
		uint32_t dlc;

		union {
			uint8_t data[8];
			uint32_t data32[2];
		};
	};

	struct TransmitQueue {
		Message msg;
		uint32_t crc;
		uint32_t stuffedWords;
		uint32_t stuffedData[5];
	};

	enum TransmitState {
		// Transmit states (stored in tx_state)
		TS_IDLE = 0, TS_QUEUED = 1, TS_ACKING_RX = 2, TS_CONFIRM_TX = 3
	};

	// Report state flags (stored in report_state)
	enum ReportState: uint32_t {
		RS_NEED_EOF_FLAG = 1 << 2,
		// States
		RS_IDLE = 0, RS_NEED_RX_ACK = 1, RS_NEED_TX_ACK = 2,
		RS_NEED_RX_EOF = RS_NEED_RX_ACK | RS_NEED_EOF_FLAG,
		RS_NEED_TX_EOF = RS_NEED_TX_ACK | RS_NEED_EOF_FLAG,
	};

	// Parsing states (stored in parse_state)
	enum ParseState {
		MS_START, MS_HEADER, MS_EXT_HEADER, MS_DATA0, MS_DATA1,
		MS_CRC, MS_ACK, MS_EOF0, MS_EOF1, MS_DISCARD
	};

	enum NotificationType: uint32_t {
		NOTIFY_RX = 1 << 20,
		NOTIFY_TX = 1 << 21,
		NOTIFY_ERROR = 1 << 23,
	};

	typedef std::function<void(CAN2040* cd, NotificationType notify, Message* msg, uint32_t errorCode)> ReceiveCallback;

	static constexpr uint32_t ID_RTR = 1 << 30;
	static constexpr uint32_t ID_EFF = 1 << 31;

	// Setup
	uint32_t pioIndex = 0;
	pio_hw_t* pioHardware = nullptr;
	uint32_t rxGpio = 0;
	uint32_t txGpio = 0;
	uint32_t systemClock = 0;
	ReceiveCallback rxCallback = nullptr;
	BusStatistics stats = {};

	// Bit unstuffing
	BitUnstuffer unstuf = {};
	uint32_t rawBitCount = 0;

	// Input data state
	ParseState parseState = MS_START;
	uint32_t parseCrc = 0;
	uint32_t parseCrcBits = 0;
	uint32_t parseCrcPosition = 0;
	Message parseMessage = {};

	// Reporting
	uint32_t reportState = RS_IDLE;

	// Transmits
	TransmitState txState = TS_IDLE;
	uint32_t txPullPosition = 0;
	uint32_t txPushPosition = 0;
	TransmitQueue txQueue[4] = {};

	/**
	 * API function to initialize can2040 code
	 * @param newPioIndex 0 or 1
	 */
	void setup(uint32_t newPioIndex);

	/**
	 * API function to configure callback
	 * @param rx_cb
	 */
	void callbackConfig(ReceiveCallback rx_cb);

	/**
	 * API function to start CANbus interface
	 * @param bitrate
	 * @param newRxGpio
	 * @param newTxGpio
	 */
	void start(uint32_t bitrate, uint32_t newRxGpio, uint32_t newTxGpio);

	/**
	 * API function to stop can2040 code
	 */
	void stop() const;

	/**
	 * API function to access can2040 statistics
	 * @param stats
	 */
	void getStatistics(BusStatistics* stats);

	/**
	 * Main API irq notification function
	 */
	void pioIrqHandler();

	/**
	 * API function to check if transmit space available
	 * @return
	 */
	[[nodiscard]] bool checkTransmit() const;

	/**
	 * API function to transmit a message
	 * @param msg
	 * @return
	 */
	int transmit(const Message* msg);

protected:
	/**
	 * Setup PIO "sync" state machine (state machine 0)
	 */
	void pioSyncSetup() const;

	/**
	 * Setup PIO "rx" state machine (state machine 1)
	 */
	void pioRxSetup() const;

	/**
	 * Setup PIO "match" state machine (state machine 2)
	 */
	void pioMatchSetup() const;

	/**
	 * Setup PIO "tx" state machine (state machine 3)
	 */
	void pioTxSetup() const;

	/**
	 * Set PIO "sync" machine to signal "may transmit" (sm irq 0) on 11 idle bits
	 */
	void pioSyncNormalStartSignal() const;

	/**
	 * Set PIO "sync" machine to signal "may transmit" (sm irq 0) on 17 idle bits
	 */
	void pioSyncSlowStartSignal() const;

	/**
	 * Test if PIO "rx" state machine has overflowed its fifos
	 */
	[[nodiscard]] int pioRxCheckStAll() const;

	/**
	 * Set PIO "match" state machine to raise a "matched" signal on a bit sequence
	 * @param matchKey
	 */
	void pioMatchCheck(uint32_t matchKey) const;

	/**
	 * Cancel any pending checks on PIO "match" state machine
	 */
	void pioMatchClear() const;

	/**
	 * Flush and halt PIO "tx" state machine
	 */
	void pioTxReset() const;

	/**
	 * Queue a message for transmission on PIO "tx" state machine
	 * @param data
	 * @param count
	 */
	void pioTxSend(const uint32_t* data, uint32_t count) const;

	/**
	 * Set PIO "tx" state machine to inject an ack after a CRC match
	 * @param matchKey
	 */
	void pioTxInjectAck(uint32_t matchKey) const;

	/**
	 * Did PIO "tx" state machine unexpectedly finish a transmit attempt?
	 * @return
	 */
	[[nodiscard]] int pioTxDidFail() const;

	/**
	 * Enable host irqs for state machine signals
	 * @param smIrqs
	 */
	void pioIrqSet(uint32_t smIrqs) const;

	/**
	 * Completely disable host irqs
	 */
	void pioIrqDisable() const;

	/**
	 * Return current host irq mask
	 */
	[[nodiscard]] uint32_t pioIrqGet() const;

	/**
	 * Raise the txpending flag
	 */
	void pioSignalSetTxPending() const;

	/**
	 * Clear the txpending flag
	 */
	void pioSignalClearTxPending() const;

	/**
	 * Setup PIO state machines
	 */
	void pioStateMachineSetup() const;

	/**
	 * Initial setup of gpio pins and PIO state machines
	 * @param bitrate
	 */
	void pioSetup(uint32_t bitrate) const;

	/**
	 * Calculate queue array position from a transmit index
	 * @param pos
	 * @return
	 */
	[[nodiscard]] uint32_t txQueuePosition(uint32_t pos) const;

	/**
	 * Queue the next message for transmission in the PIO
	 * @return
	 */
	uint32_t txScheduleTransmit();

	/**
	 * Setup PIO state for ack injection
	 * @param matchKey
	 */
	void txInjectAck(uint32_t matchKey);

	/**
	 * Check if the current parsed message is feedback from current transmit
	 * @return
	 */
	int txCheckLocalMessage();

	/**
	 * Report error to calling code (via callback interface)
	 * @param errorCode
	 */
	void reportCallbackError(uint32_t errorCode);

	/**
	 * Report a received message to calling code (via callback interface)
	 */
	void reportCallbackRxMsg();

	/**
	 * Report a message that was successfully transmited (via callback interface)
	 */
	void reportCallbackTxMsg();

	/**
	 * EOF phase complete - report message (rx or tx) to calling code
	 */
	void reportHandleEof();

	/**
	 * Check if message being processed is an rx message (not self feedback from tx)
	 * @return
	 */
	[[nodiscard]] int reportIsNotInTx() const;

	/**
	 * Parser found a new message start
	 */
	void reportNoteMessageStart() const;

	/**
	 * Setup for ack injection (if receiving) or ack confirmation (if transmit)
	 */
	int reportNoteCrcStart();

	/**
	 * Parser successfully found matching crc
	 */
	void reportNoteCrcSuccess() const;

	/**
	 * Parser found successful ack
	 */
	void reportNoteAckSuccess();

	/**
	 * Parser found successful EOF
	 */
	void reportNoteEofSuccess();

	/**
	 * Parser found unexpected data on input
	 */
	void reportNoteDiscarding();

	/**
	 * Received PIO rx "ackdone" irq
	 */
	void reportLineAckDone();

	/**
	 * Received PIO "matched" irq
	 */
	void reportLineMatched();

	/**
	 * Received 10+ passive bits on the line (between 10 and 17 bits)
	 */
	void reportLineMaytx();

	/**
	 * Schedule a transmit
	 */
	void reportLineTxPending();

	/**
	 * Reset any bits in the incoming parsing state
	 */
	void dataStateClearBits();

	/**
	 * Transition to the next parsing state
	 * @param state
	 * @param num_bits
	 */
	void dataStateGoNext(ParseState state, uint32_t num_bits);

	/**
	 * Transition to the MS_DISCARD state - drop all bits until 6 passive bits
	 */
	void dataStateGoDiscard();

	/**
	 * Note a data parse error and transition to discard state
	 */
	void dataStateGoError();

	/**
	 * Received six dominant bits on the line
	 */
	void dataStateLineError();

	/**
	 * Received six unexpected passive bits on the line
	 */
	void dataStateLinePassive();

	/**
	 * Transition to MS_CRC state - await 16 bits of crc
	 */
	void dataStateGoCrc();

	/**
	 * Transition to MS_DATA0 state (if applicable) - await data bits
	 * @param id
	 * @param data
	 */
	void dataStateGoData(uint32_t id, uint32_t data);

	/**
	 * Handle reception of first bit of header (after start-of-frame (SOF))
	 * @param data
	 */
	void dataStateUpdateStart(uint32_t data);

	/**
	 * Handle reception of next 17 header bits
	 * @param data
	 */
	void dataStateUpdateHeader(uint32_t data);

	/**
	 * Handle reception of additional 20 bits of "extended header"
	 * @param data
	 */
	void dataStateUpdateExtHeader(uint32_t data);

	/**
	 * Handle reception of first 1-4 bytes of data content
	 * @param data
	 */
	void dataStateUpdateData0(uint32_t data);

	/**
	 * Handle reception of bytes 5-8 of data content
	 * @param data
	 */
	void dataStateUpdateData1(uint32_t data);

	/**
	 * Handle reception of 16 bits of message CRC (15 crc bits + crc delimiter)
	 * @param data
	 */
	void dataStateUpdateCrc(uint32_t data);

	/**
	 * Handle reception of 2 bits of ack phase (ack, ack delimiter)
	 * @param data
	 */
	void dataStateUpdateAck(uint32_t data);

	/**
	 * Handle reception of first four end-of-frame (EOF) bits
	 * @param data
	 */
	void dataStateUpdateEof0(uint32_t data);

	/**
	 * Handle reception of end-of-frame (EOF) bits 5-7 and first two IFS bits
	 * @param data
	 */
	void dataStateUpdateEof1(uint32_t data);

	/**
	 * Handle data received while in MS_DISCARD state
	 * @param data
	 */
	void dataStateUpdateDiscard(uint32_t data);

	/**
	 * Update parsing state after reading the bits of the current field
	 * @param data
	 */
	void dataStateUpdate(uint32_t data);

	/**
	 * Process incoming data from PIO "rx" state machine
	 * @param rxData
	 */
	void processRx(uint32_t rxData);
};
