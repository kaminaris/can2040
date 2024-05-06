// Software CANbus implementation for rp2040
//
// Copyright (C) 2022,2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "can2040.h" // can2040_setup

#include <hardware/clocks.h>
#include <hardware/regs/pads_bank0.h>
#include <hardware/regs/resets.h>
#include <hardware/structs/iobank0.h>
#include <hardware/structs/padsbank0.h>
#include <hardware/structs/resets.h>
#include <RP2040.h> // hw_set_bits

/****************************************************************
 * rp2040 and low-level helper functions
 ****************************************************************/

// Helper compiler definitions
#define barrier() __asm__ __volatile__("": : :"memory")
#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

// Helper functions for writing to "io" memory
static inline void writel(void* addr, uint32_t val) {
	barrier();
	*static_cast<volatile uint32_t*>(addr) = val;
}

static inline uint32_t readl(const void* addr) {
	uint32_t val = *static_cast<volatile const uint32_t*>(addr);
	barrier();
	return val;
}

// rp2040 helper function to clear a hardware reset bit
static void rp2040ClearReset(const uint32_t resetBit) {
	if (resets_hw->reset & resetBit) {
		hw_clear_bits(&resets_hw->reset, resetBit);
		while (!(resets_hw->reset_done & resetBit));
	}
}

// Helper to set the mode and extended function of a pin
static void rp2040GpioPeripheral(const uint32_t gpio, const int func, const int pullUp) {
	padsbank0_hw->io[gpio] = (
		PADS_BANK0_GPIO0_IE_BITS |
		(PADS_BANK0_GPIO0_DRIVE_VALUE_4MA << PADS_BANK0_GPIO0_DRIVE_MSB) |
		(pullUp > 0 ? PADS_BANK0_GPIO0_PUE_BITS : 0) |
		(pullUp < 0 ? PADS_BANK0_GPIO0_PDE_BITS : 0)
	);
	iobank0_hw->io[gpio].ctrl = func << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
}

/****************************************************************
 * rp2040 PIO support
 ****************************************************************/

constexpr uint32_t PIO_CLOCK_PER_BIT = 32;
constexpr uint32_t PIO_RX_WAKE_BITS = 10;

constexpr uint32_t can2040_offset_sync_found_end_of_message = 2u;
constexpr uint32_t can2040_offset_sync_signal_start = 4u;
constexpr uint32_t can2040_offset_sync_entry = 6u;
constexpr uint32_t can2040_offset_sync_end = 13u;
constexpr uint32_t can2040_offset_shared_rx_read = 13u;
constexpr uint32_t can2040_offset_shared_rx_end = 15u;
constexpr uint32_t can2040_offset_match_load_next = 18u;
constexpr uint32_t can2040_offset_tx_conflict = 24u;
constexpr uint32_t can2040_offset_match_end = 25u;
constexpr uint32_t can2040_offset_tx_got_recessive = 25u;
constexpr uint32_t can2040_offset_tx_write_pin = 27u;

static const uint16_t can2040ProgramInstructions[] = {
	0x0085, //  0: jmp    y--, 5
	0x0048, //  1: jmp    x--, 8
	0xe029, //  2: set    x, 9
	0x00cc, //  3: jmp    pin, 12
	0xc000, //  4: irq    nowait 0
	0x00c0, //  5: jmp    pin, 0
	0xc040, //  6: irq    clear 0
	0xe429, //  7: set    x, 9                   [4]
	0xf043, //  8: set    y, 3                   [16]
	0xc104, //  9: irq    nowait 4               [1]
	0x03c5, // 10: jmp    pin, 5                 [3]
	0x0307, // 11: jmp    7                      [3]
	0x0043, // 12: jmp    x--, 3
	0x20c4, // 13: wait   1 irq, 4
	0x4001, // 14: in     pins, 1
	0xa046, // 15: mov    y, isr
	0x01b2, // 16: jmp    x != y, 18             [1]
	0xc002, // 17: irq    nowait 2
	0x40eb, // 18: in     osr, 11
	0x4054, // 19: in     y, 20
	0xa047, // 20: mov    y, osr
	0x8080, // 21: pull   noblock
	0xa027, // 22: mov    x, osr
	0x0098, // 23: jmp    y--, 24
	0xa0e2, // 24: mov    osr, y
	0x6021, // 25: out    x, 1
	0x00df, // 26: jmp    pin, 31
	0xb801, // 27: mov    pins, x                [24]
	0x02d9, // 28: jmp    pin, 25                [2]
	0x0058, // 29: jmp    x--, 24
	0x6021, // 30: out    x, 1
	0x011b, // 31: jmp    27                     [1]
};

// Local names for PIO state machine IRQs
#define SI_MAYTX     PIO_IRQ0_INTE_SM0_BITS
#define SI_MATCHED   PIO_IRQ0_INTE_SM2_BITS
#define SI_ACKDONE   PIO_IRQ0_INTE_SM3_BITS
#define SI_RX_DATA   PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS
#define SI_TXPENDING PIO_IRQ0_INTE_SM1_BITS // Misc bit manually forced

void CAN2040::pioSyncSetup() const {
	pio_sm_hw* sm = &pioHardware->sm[0];
	sm->execctrl = (
		rxGpio << PIO_SM0_EXECCTRL_JMP_PIN_LSB
		| (can2040_offset_sync_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
		| can2040_offset_sync_signal_start << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB
	);
	sm->pinctrl = (
		1 << PIO_SM0_PINCTRL_SET_COUNT_LSB
		| rxGpio << PIO_SM0_PINCTRL_SET_BASE_LSB
	);
	sm->instr = 0xe080; // set pindirs, 0
	sm->pinctrl = 0;
	pioHardware->txf[0] = 9 + 6 * PIO_CLOCK_PER_BIT / 2;
	sm->instr = 0x80a0; // pull block
	sm->instr = can2040_offset_sync_entry; // jmp sync_entry
}

void CAN2040::pioRxSetup() const {
	pio_sm_hw* sm = &pioHardware->sm[1];
	sm->execctrl = (
		(can2040_offset_shared_rx_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
		| can2040_offset_shared_rx_read << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB
	);
	sm->pinctrl = rxGpio << PIO_SM0_PINCTRL_IN_BASE_LSB;
	sm->shiftctrl = 0; // flush fifo on a restart
	sm->shiftctrl = (
		PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS
		| PIO_RX_WAKE_BITS << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB
		| PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS
	);
	sm->instr = can2040_offset_shared_rx_read; // jmp shared_rx_read
}

void CAN2040::pioMatchSetup() const {
	pio_sm_hw* sm = &pioHardware->sm[2];
	sm->execctrl = (
		(can2040_offset_match_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
		| can2040_offset_shared_rx_read << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB
	);
	sm->pinctrl = rxGpio << PIO_SM0_PINCTRL_IN_BASE_LSB;
	sm->shiftctrl = 0;
	sm->instr = 0xe040; // set y, 0
	sm->instr = 0xa0e2; // mov osr, y
	sm->instr = 0xa02a; // mov x, !y
	sm->instr = can2040_offset_match_load_next; // jmp match_load_next
}

void CAN2040::pioTxSetup() const {
	pio_sm_hw* sm = &pioHardware->sm[3];
	sm->execctrl = (
		rxGpio << PIO_SM0_EXECCTRL_JMP_PIN_LSB
		| can2040_offset_tx_conflict << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
		| can2040_offset_tx_conflict << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB
	);
	sm->shiftctrl = (
		PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS
		| PIO_SM0_SHIFTCTRL_AUTOPULL_BITS
	);
	sm->pinctrl = (
		1 << PIO_SM0_PINCTRL_SET_COUNT_LSB
		| 1 << PIO_SM0_PINCTRL_OUT_COUNT_LSB
		| txGpio << PIO_SM0_PINCTRL_SET_BASE_LSB
		| txGpio << PIO_SM0_PINCTRL_OUT_BASE_LSB
	);
	sm->instr = 0xe001; // set pins, 1
	sm->instr = 0xe081; // set pindirs, 1
}

void CAN2040::pioSyncNormalStartSignal() const {
	constexpr uint32_t eom_idx = can2040_offset_sync_found_end_of_message;
	pioHardware->instr_mem[eom_idx] = 0xe12a; // set x, 10 [1]
}

void CAN2040::pioSyncSlowStartSignal() const {
	constexpr uint32_t eom_idx = can2040_offset_sync_found_end_of_message;
	pioHardware->instr_mem[eom_idx] = 0xa127; // mov x, osr [1]
}

int CAN2040::pioRxCheckStAll() const {
	return pioHardware->fdebug & (1 << (PIO_FDEBUG_RXSTALL_LSB + 1));
}

void CAN2040::pioMatchCheck(const uint32_t matchKey) const {
	pioHardware->txf[2] = matchKey;
}

// Calculate pos+bits identifier for PIO "match" state machine
static uint32_t pioMatchCalcKey(const uint32_t rawBits, const uint32_t rxBitPos) {
	return (rawBits & 0x1fffff) | ((-rxBitPos) << 21);
}

void CAN2040::pioMatchClear() const {
	pioMatchCheck(0);
}

void CAN2040::pioTxReset() const {
	pioHardware->ctrl = 0x07 << PIO_CTRL_SM_ENABLE_LSB;
	pioHardware->ctrl = (0x07 << PIO_CTRL_SM_ENABLE_LSB) | (0x08 << PIO_CTRL_SM_RESTART_LSB);
	pioHardware->irq = (SI_MATCHED | SI_ACKDONE) >> 8; // clear PIO irq flags

	// Clear tx fifo
	pio_sm_hw* sm = &pioHardware->sm[3];
	sm->shiftctrl = 0;
	sm->shiftctrl = PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS;
}

void CAN2040::pioTxSend(const uint32_t* data, const uint32_t count) const {
	pioTxReset();
	pioHardware->instr_mem[can2040_offset_tx_got_recessive] = 0x6021; // out x, 1
	for (uint32_t i = 0; i < count; i++) {
		pioHardware->txf[3] = data[i];
	}

	pio_sm_hw* sm = &pioHardware->sm[3];
	sm->instr = 0xe001; // set pins, 1
	sm->instr = 0x6021; // out x, 1
	sm->instr = can2040_offset_tx_write_pin; // jmp tx_write_pin
	sm->instr = 0x20c0; // wait 1 irq, 0
	pioHardware->ctrl = 0x0f << PIO_CTRL_SM_ENABLE_LSB;
}

void CAN2040::pioTxInjectAck(const uint32_t matchKey) const {
	pioTxReset();
	pioHardware->instr_mem[can2040_offset_tx_got_recessive] = 0xc023; // irq wait 3
	pioHardware->txf[3] = 0x7fffffff;
	pio_sm_hw* sm = &pioHardware->sm[3];
	sm->instr = 0xe001; // set pins, 1
	sm->instr = 0x6021; // out x, 1
	sm->instr = can2040_offset_tx_write_pin; // jmp tx_write_pin
	sm->instr = 0x20c2; // wait 1 irq, 2
	pioHardware->ctrl = 0x0f << PIO_CTRL_SM_ENABLE_LSB;

	pioMatchCheck(matchKey);
}

int CAN2040::pioTxDidFail() const {
	// Check for passive/dominant bit conflict without parser noticing
	if (pioHardware->sm[3].addr == can2040_offset_tx_conflict) {
		return !(pioHardware->intr & SI_RX_DATA);
	}

	// Check for unexpected drain of transmit queue without parser noticing
	return (!(pioHardware->flevel & PIO_FLEVEL_TX3_BITS) && (pioHardware->intr & (SI_MAYTX | SI_RX_DATA)) == SI_MAYTX);
}

void CAN2040::pioIrqSet(const uint32_t smIrqs) const {
	pioHardware->inte0 = smIrqs | SI_RX_DATA;
}

void CAN2040::pioIrqDisable() const {
	pioHardware->inte0 = 0;
}

uint32_t CAN2040::pioIrqGet() const {
	return pioHardware->inte0;
}

void CAN2040::pioSignalSetTxPending() const {
	pioHardware->irq_force = SI_TXPENDING >> 8;
}

void CAN2040::pioSignalClearTxPending() const {
	pioHardware->irq = SI_TXPENDING >> 8;
}

void CAN2040::pioStateMachineSetup() const {
	// Reset state machines
	pioHardware->ctrl = PIO_CTRL_SM_RESTART_BITS | PIO_CTRL_CLKDIV_RESTART_BITS;
	pioHardware->fdebug = 0xffffffff;
	pioHardware->irq = 0xff;
	pioSignalSetTxPending();

	// Load pio program
	for (uint32_t i = 0; i < std::size(can2040ProgramInstructions); i++) {
		pioHardware->instr_mem[i] = can2040ProgramInstructions[i];
	}

	// Set initial state machine state
	pioSyncSetup();
	pioRxSetup();
	pioMatchSetup();
	pioTxSetup();

	// Start state machines
	pioHardware->ctrl = 0x07 << PIO_CTRL_SM_ENABLE_LSB;
}

void CAN2040::pioSetup(const uint32_t bitrate) const {
	// Configure pio0 clock
	const uint32_t rb = pioIndex ? RESETS_RESET_PIO1_BITS : RESETS_RESET_PIO0_BITS;
	rp2040ClearReset(rb);

	// Setup and sync pio state machine clocks
	const uint32_t div = (256 / PIO_CLOCK_PER_BIT) * systemClock / bitrate;

	for (int i = 0; i < 4; i++) {
		pioHardware->sm[i].clkdiv = div << PIO_SM0_CLKDIV_FRAC_LSB;
	}

	// Configure state machines
	pioStateMachineSetup();

	// Map Rx/Tx gpios
	const uint32_t pioFunc = pioIndex ? 7 : 6;
	rp2040GpioPeripheral(rxGpio, pioFunc, 1);
	rp2040GpioPeripheral(txGpio, pioFunc, 0);
}

/****************************************************************
 * CRC calculation
 ****************************************************************/

// Calculated 8-bit crc table (see scripts/crc.py)
static const uint16_t crcTable[256] = {
	0x0000, 0x4599, 0x4eab, 0x0b32, 0x58cf, 0x1d56, 0x1664, 0x53fd, 0x7407, 0x319e,
	0x3aac, 0x7f35, 0x2cc8, 0x6951, 0x6263, 0x27fa, 0x2d97, 0x680e, 0x633c, 0x26a5,
	0x7558, 0x30c1, 0x3bf3, 0x7e6a, 0x5990, 0x1c09, 0x173b, 0x52a2, 0x015f, 0x44c6,
	0x4ff4, 0x0a6d, 0x5b2e, 0x1eb7, 0x1585, 0x501c, 0x03e1, 0x4678, 0x4d4a, 0x08d3,
	0x2f29, 0x6ab0, 0x6182, 0x241b, 0x77e6, 0x327f, 0x394d, 0x7cd4, 0x76b9, 0x3320,
	0x3812, 0x7d8b, 0x2e76, 0x6bef, 0x60dd, 0x2544, 0x02be, 0x4727, 0x4c15, 0x098c,
	0x5a71, 0x1fe8, 0x14da, 0x5143, 0x73c5, 0x365c, 0x3d6e, 0x78f7, 0x2b0a, 0x6e93,
	0x65a1, 0x2038, 0x07c2, 0x425b, 0x4969, 0x0cf0, 0x5f0d, 0x1a94, 0x11a6, 0x543f,
	0x5e52, 0x1bcb, 0x10f9, 0x5560, 0x069d, 0x4304, 0x4836, 0x0daf, 0x2a55, 0x6fcc,
	0x64fe, 0x2167, 0x729a, 0x3703, 0x3c31, 0x79a8, 0x28eb, 0x6d72, 0x6640, 0x23d9,
	0x7024, 0x35bd, 0x3e8f, 0x7b16, 0x5cec, 0x1975, 0x1247, 0x57de, 0x0423, 0x41ba,
	0x4a88, 0x0f11, 0x057c, 0x40e5, 0x4bd7, 0x0e4e, 0x5db3, 0x182a, 0x1318, 0x5681,
	0x717b, 0x34e2, 0x3fd0, 0x7a49, 0x29b4, 0x6c2d, 0x671f, 0x2286, 0x2213, 0x678a,
	0x6cb8, 0x2921, 0x7adc, 0x3f45, 0x3477, 0x71ee, 0x5614, 0x138d, 0x18bf, 0x5d26,
	0x0edb, 0x4b42, 0x4070, 0x05e9, 0x0f84, 0x4a1d, 0x412f, 0x04b6, 0x574b, 0x12d2,
	0x19e0, 0x5c79, 0x7b83, 0x3e1a, 0x3528, 0x70b1, 0x234c, 0x66d5, 0x6de7, 0x287e,
	0x793d, 0x3ca4, 0x3796, 0x720f, 0x21f2, 0x646b, 0x6f59, 0x2ac0, 0x0d3a, 0x48a3,
	0x4391, 0x0608, 0x55f5, 0x106c, 0x1b5e, 0x5ec7, 0x54aa, 0x1133, 0x1a01, 0x5f98,
	0x0c65, 0x49fc, 0x42ce, 0x0757, 0x20ad, 0x6534, 0x6e06, 0x2b9f, 0x7862, 0x3dfb,
	0x36c9, 0x7350, 0x51d6, 0x144f, 0x1f7d, 0x5ae4, 0x0919, 0x4c80, 0x47b2, 0x022b,
	0x25d1, 0x6048, 0x6b7a, 0x2ee3, 0x7d1e, 0x3887, 0x33b5, 0x762c, 0x7c41, 0x39d8,
	0x32ea, 0x7773, 0x248e, 0x6117, 0x6a25, 0x2fbc, 0x0846, 0x4ddf, 0x46ed, 0x0374,
	0x5089, 0x1510, 0x1e22, 0x5bbb, 0x0af8, 0x4f61, 0x4453, 0x01ca, 0x5237, 0x17ae,
	0x1c9c, 0x5905, 0x7eff, 0x3b66, 0x3054, 0x75cd, 0x2630, 0x63a9, 0x689b, 0x2d02,
	0x276f, 0x62f6, 0x69c4, 0x2c5d, 0x7fa0, 0x3a39, 0x310b, 0x7492, 0x5368, 0x16f1,
	0x1dc3, 0x585a, 0x0ba7, 0x4e3e, 0x450c, 0x0095
};

// Update a crc with 8 bits of data
static uint32_t crcByte(const uint32_t crc, const uint32_t data) {
	return (crc << 8) ^ crcTable[((crc >> 7) ^ data) & 0xff];
}

// Update a crc with 8, 16, 24, or 32 bits of data
static inline uint32_t crcBytes(uint32_t crc, const uint32_t data, const uint32_t num) {
	switch (num) {
		default: crc = crcByte(crc, data >> 24); /* FALLTHRU */
		case 3: crc = crcByte(crc, data >> 16); /* FALLTHRU */
		case 2: crc = crcByte(crc, data >> 8); /* FALLTHRU */
		case 1: crc = crcByte(crc, data);
	}
	return crc;
}

/****************************************************************
 * Bit unstuffing
 ****************************************************************/

// Add 'count' number of bits from 'data' to the 'bu' unstuffer
static void unstufAddBits(BitUnstuffer* bu, const uint32_t data, const uint32_t count) {
	const uint32_t mask = (1 << count) - 1;
	bu->stuffedBits = (bu->stuffedBits << count) | (data & mask);
	bu->countStuff = count;
}

// Reset state and set the next desired 'num_bits' unstuffed bits to extract
static void unstufSetCount(BitUnstuffer* bu, const uint32_t numBits) {
	bu->unstuffedBits = 0;
	bu->countUnstuff = numBits;
}

// Clear bitstuffing state (used after crc field to avoid bitstuffing ack field)
static void unstufClearState(BitUnstuffer* bu) {
	const uint32_t lb = 1 << bu->countStuff;
	bu->stuffedBits = (bu->stuffedBits & (lb - 1)) | (lb << 1);
}

// Restore raw bitstuffing state (used to undo unstuf_clear_state() )
static void unstufRestoreState(BitUnstuffer* bu, const uint32_t data) {
	const uint32_t cs = bu->countStuff;
	bu->stuffedBits = (bu->stuffedBits & ((1 << cs) - 1)) | (data << cs);
}

// Pull bits from unstuffer (as specified in unstuf_set_count() )
static int unstufPullBits(BitUnstuffer* bu) {
	uint32_t sb = bu->stuffedBits, edges = sb ^ (sb >> 1);
	uint32_t e2 = edges | (edges >> 1), e4 = e2 | (e2 >> 2), rm_bits = ~e4;
	uint32_t cs = bu->countStuff, cu = bu->countUnstuff;
	if (!cs) {
		// Need more data
		return 1;
	}

	for (;;) {
		uint32_t try_cnt = cs > cu ? cu : cs;
		for (;;) {
			const uint32_t try_mask = ((1 << try_cnt) - 1) << (cs + 1 - try_cnt);
			if (likely(!(rm_bits & try_mask))) {
				// No stuff bits in try_cnt bits - copy into unstuffed_bits
				bu->countUnstuff = cu = cu - try_cnt;
				bu->countStuff = cs = cs - try_cnt;
				bu->unstuffedBits |= ((sb >> cs) & ((1 << try_cnt) - 1)) << cu;
				if (!cu) {
					// Extracted desired bits
					return 0;
				}

				break;
			}

			bu->countStuff = cs = cs - 1;
			if (rm_bits & (1 << (cs + 1))) {
				// High bit is a stuff bit
				if (unlikely(rm_bits & (1 << cs))) {
					// Six consecutive bits - a bitstuff error
					if (sb & (1 << cs)) {
						return -1;
					}

					return -2;
				}
				break;
			}

			// High bit not a stuff bit - limit try_cnt and retry
			bu->countUnstuff = cu = cu - 1;
			bu->unstuffedBits |= ((sb >> cs) & 1) << cu;
			try_cnt /= 2;
		}

		if (likely(!cs)) {
			// Need more data
			return 1;
		}
	}
}

// Return most recent raw (still stuffed) bits
static uint32_t unstufGetRaw(const BitUnstuffer* bu) {
	return bu->stuffedBits >> bu->countStuff;
}

/****************************************************************
 * Bit stuffing
 ****************************************************************/

// Stuff 'num_bits' bits in '*pb' - upper bits must already be stuffed
static uint32_t bitstuff(uint32_t* pb, uint32_t num_bits) {
	uint32_t b = *pb, count = num_bits;

	for (;;) {
		uint32_t try_cnt = num_bits, edges = b ^ (b >> 1);
		uint32_t e2 = edges | (edges >> 1), e4 = e2 | (e2 >> 2), add_bits = ~e4;

		for (;;) {
			const uint32_t try_mask = ((1 << try_cnt) - 1) << (num_bits - try_cnt);
			if (!(add_bits & try_mask)) {
				// No stuff bits needed in try_cnt bits
				if (try_cnt >= num_bits) {
					goto done;
				}

				num_bits -= try_cnt;
				try_cnt = (num_bits + 1) / 2;
				continue;
			}

			if (add_bits & (1 << (num_bits - 1))) {
				// A stuff bit must be inserted prior to the high bit
				uint32_t low_mask = (1 << num_bits) - 1, low = b & low_mask;
				const uint32_t high = (b & ~(low_mask >> 1)) << 1;
				b = high ^ low ^ (1 << (num_bits - 1));
				count += 1;
				if (num_bits <= 4) {
					goto done;
				}

				num_bits -= 4;
				break;
			}

			// High bit doesn't need stuff bit - accept it, limit try_cnt, retry
			num_bits--;
			try_cnt /= 2;
		}
	}

done:
	*pb = b;
	return count;
}

// State storage for building bit stuffed transmit messages
struct bitstuffer_s {
	uint32_t prev_stuffed, bitpos,* buf;
};

// Push 'count' bits of 'data' into stuffer without performing bit stuffing
static void bs_pushraw(bitstuffer_s* bs, const uint32_t data, const uint32_t count) {
	const uint32_t bitpos = bs->bitpos;
	uint32_t wp = bitpos / 32, bitused = bitpos % 32, bitavail = 32 - bitused;
	uint32_t* fb = &bs->buf[wp];
	if (bitavail >= count) {
		fb[0] |= data << (bitavail - count);
	} else {
		fb[0] |= data >> (count - bitavail);
		fb[1] |= data << (32 - (count - bitavail));
	}
	bs->bitpos = bitpos + count;
}

// Push 'count' bits of 'data' into stuffer
static void bs_push(bitstuffer_s* bs, uint32_t data, const uint32_t count) {
	data &= (1 << count) - 1;
	uint32_t stuf = (bs->prev_stuffed << count) | data;
	const uint32_t newcount = bitstuff(&stuf, count);
	bs_pushraw(bs, stuf, newcount);
	bs->prev_stuffed = stuf;
}

// Pad final word of stuffer with high bits
static uint32_t bs_finalize(const bitstuffer_s* bs) {
	const uint32_t bitpos = bs->bitpos;
	const uint32_t words = DIV_ROUND_UP(bitpos, 32);
	const uint32_t extra = words * 32 - bitpos;

	if (extra) {
		bs->buf[words - 1] |= (1 << extra) - 1;
	}

	return words;
}

/****************************************************************
 * Transmit state tracking
 ****************************************************************/

uint32_t CAN2040::txQueuePosition(const uint32_t pos) const {
	return pos % std::size(txQueue);
}

uint32_t CAN2040::txScheduleTransmit() {
	if (txState == TS_QUEUED && !pioTxDidFail()) {
		// Already queued or actively transmitting
		return 0;
	}

	uint32_t txPullPos = txPullPosition;
	if (readl(&txPushPosition) == txPullPos) {
		// No new messages to transmit
		txState = TS_IDLE;
		pioSignalClearTxPending();
		__DMB();
		if (likely(readl(&txPushPosition) == txPullPos)) {
			return SI_TXPENDING;
		}

		// Raced with can2040_transmit() - msg is now available for transmit
		pioSignalSetTxPending();
	}

	txState = TS_QUEUED;
	stats.txAttempt++;
	const TransmitQueue* qt = &txQueue[txQueuePosition(txPullPos)];
	pioTxSend(qt->stuffedData, qt->stuffedWords);
	return 0;
}

void CAN2040::txInjectAck(const uint32_t matchKey) {
	txState = TS_ACKING_RX;
	pioTxInjectAck(matchKey);
}

int CAN2040::txCheckLocalMessage() {
	if (txState != TS_QUEUED) {
		return 0;
	}

	TransmitQueue* qt = &txQueue[txQueuePosition(txPullPosition)];
	Message* pm = &parseMessage,* tm = &qt->msg;
	if (tm->id == pm->id) {
		if (
			qt->crc != parseCrc ||
			tm->dlc != pm->dlc ||
			tm->data32[0] != pm->data32[0] ||
			tm->data32[1] != pm->data32[1]
		) {
			// Message with same id that differs in content - an error
			return -1;
		}

		// This is a self transmit
		txState = TS_CONFIRM_TX;
		return 1;
	}
	return 0;
}

/****************************************************************
 * Notification callbacks
 ****************************************************************/

void CAN2040::reportCallbackError(const uint32_t errorCode) {
	Message msg = {};
	rxCallback(this, NOTIFY_ERROR, &msg, errorCode);
}

void CAN2040::reportCallbackRxMsg() {
	stats.rxTotal++;
	rxCallback(this, NOTIFY_RX, &parseMessage, 0);
}

void CAN2040::reportCallbackTxMsg() {
	writel(&txPullPosition, txPullPosition + 1);
	stats.txTotal++;
	rxCallback(this, NOTIFY_TX, &parseMessage, 0);
}

void CAN2040::reportHandleEof() {
	if (reportState & RS_NEED_EOF_FLAG) {
		// RS_NEED_xX_EOF
		// Successfully processed a new message - report to calling code
		pioSyncNormalStartSignal();
		if (reportState == RS_NEED_TX_EOF) {
			reportCallbackTxMsg();
		} else {
			reportCallbackRxMsg();
		}
	}

	reportState = RS_IDLE;
	pioMatchClear();
}

int CAN2040::reportIsNotInTx() const {
	return !(reportState & RS_NEED_TX_ACK);
}

void CAN2040::reportNoteMessageStart() const {
	pioIrqSet(SI_MAYTX);
}

int CAN2040::reportNoteCrcStart() {
	const int ret = txCheckLocalMessage();
	if (ret) {
		if (ret < 0) {
			return -1;
		}

		// This is a self transmit - setup tx eof "matched" signal
		reportState = RS_NEED_TX_ACK;
		const uint32_t bits = (parseCrcBits << 9) | 0x0ff;
		pioMatchCheck(pioMatchCalcKey(bits, parseCrcPosition + 9));
		return 0;
	}

	// Setup for ack inject (after rx fifos fully drained)
	reportState = RS_NEED_RX_ACK;
	pioSignalSetTxPending();
	pioIrqSet(SI_MAYTX | SI_TXPENDING);
	return 0;
}

void CAN2040::reportNoteCrcSuccess() const {
	if (reportState == RS_NEED_TX_ACK) {
		// Enable "matched" irq for fast back-to-back transmit scheduling
		pioIrqSet(SI_MAYTX | SI_MATCHED);
	}
}

void CAN2040::reportNoteAckSuccess() {
	if (reportState == RS_IDLE) {
		// Got "matched" signal already
		return;
	}

	// Transition RS_NEED_xX_ACK to RS_NEED_xX_EOF
	reportState |= RS_NEED_EOF_FLAG;
}

void CAN2040::reportNoteEofSuccess() {
	if (reportState == RS_IDLE) {
		// Got "matched" signal already
		return;
	}

	reportHandleEof();
	pioIrqSet(SI_TXPENDING);
}

void CAN2040::reportNoteDiscarding() {
	if (reportState != RS_IDLE) {
		reportState = RS_IDLE;
		pioMatchClear();
	}

	pioSyncSlowStartSignal();
	pioIrqSet(SI_MAYTX | SI_TXPENDING);
}

void CAN2040::reportLineAckDone() {
	// Setup "matched" irq for fast rx callbacks
	const uint32_t bits = (parseCrcBits << 8) | 0x7f;
	pioMatchCheck(pioMatchCalcKey(bits, parseCrcPosition + 8));
	// Schedule next transmit (so it is ready for next frame line arbitration)
	const uint32_t check_txpending = txScheduleTransmit();
	pioIrqSet(SI_MAYTX | SI_MATCHED | check_txpending);
}

void CAN2040::reportLineMatched() {
	// A match event indicates an ack and eof are present
	if (reportState != RS_IDLE) {
		// Transition RS_NEED_xX_ACK to RS_NEED_xX_EOF (if not already there)
		reportState |= RS_NEED_EOF_FLAG;
		reportHandleEof();
	}

	// Implement fast back-to-back tx scheduling (if applicable)
	const uint32_t check_txpending = txScheduleTransmit();
	pioIrqSet(check_txpending);
}

void CAN2040::reportLineMaytx() {
	// Line is idle - may be unexpected EOF, missed ack injection,
	// or missed "matched" signal.
	if (reportState != RS_IDLE) {
		reportHandleEof();
	}

	const uint32_t check_txpending = txScheduleTransmit();
	pioIrqSet(check_txpending);
}

void CAN2040::reportLineTxPending() {
	const uint32_t pioIrqs = pioIrqGet();
	if (pioIrqs == (SI_MAYTX | SI_TXPENDING | SI_RX_DATA) && reportState == RS_NEED_RX_ACK) {
		// Ack inject request from report_note_crc_start()
		const uint32_t mk = pioMatchCalcKey(parseCrcBits, parseCrcPosition);
		txInjectAck(mk);
		pioIrqSet(SI_MAYTX | SI_ACKDONE);
		return;
	}

	// Tx request from can2040_transmit(), report_note_eof_success(),
	// or report_note_discarding().
	const uint32_t check_txpending = txScheduleTransmit();
	pioIrqSet((pioIrqs & ~SI_TXPENDING) | check_txpending);
}

/****************************************************************
 * Input state tracking
 ****************************************************************/

void CAN2040::dataStateClearBits() {
	rawBitCount = unstuf.stuffedBits = unstuf.countStuff = 0;
}

void CAN2040::dataStateGoNext(const ParseState state, const uint32_t num_bits) {
	parseState = state;
	unstufSetCount(&unstuf, num_bits);
}

void CAN2040::dataStateGoDiscard() {
	if (pioRxCheckStAll()) {
		// CPU couldn't keep up for some read data - must reset pio state
		dataStateClearBits();
		pioStateMachineSetup();
		reportCallbackError(0);
	}

	dataStateGoNext(MS_DISCARD, 32);

	// Clear report state and update hw irqs after transition to MS_DISCARD
	reportNoteDiscarding();
}

void CAN2040::dataStateGoError() {
	stats.parseError++;
	dataStateGoDiscard();
}

void CAN2040::dataStateLineError() {
	if (parseState == MS_DISCARD) {
		dataStateGoDiscard();
	} else {
		dataStateGoError();
	}
}

void CAN2040::dataStateLinePassive() {
	if (parseState != MS_DISCARD && parseState != MS_START) {
		// Bitstuff error
		dataStateGoError();
		return;
	}

	const uint32_t stuffed_bits = unstufGetRaw(&unstuf);
	const uint32_t dom_bits = ~stuffed_bits;
	if (!dom_bits) {
		// Counter overflow in "sync" state machine - reset it
		dataStateClearBits();
		pioStateMachineSetup();
		dataStateGoDiscard();
		return;
	}

	// Look for sof after 10 passive bits (most "PIO sync" will produce)
	if (!(dom_bits & 0x3ff)) {
		dataStateGoNext(MS_START, 1);
		return;
	}

	dataStateGoDiscard();
}

void CAN2040::dataStateGoCrc() {
	parseCrc &= 0x7fff;

	// Calculate raw stuffed bits after crc and crc delimiter
	const uint32_t crcstart_bitpos = rawBitCount - unstuf.countStuff - 1;
	uint32_t crc_bits = (unstufGetRaw(&unstuf) << 15) | parseCrc;
	const uint32_t crc_bitcount = bitstuff(&crc_bits, 15 + 1) - 1;
	parseCrcBits = (crc_bits << 1) | 0x01; // Add crc delimiter
	parseCrcPosition = crcstart_bitpos + crc_bitcount + 1;

	const int ret = reportNoteCrcStart();
	if (ret) {
		dataStateGoError();
		return;
	}
	dataStateGoNext(MS_CRC, 16);
}

void CAN2040::dataStateGoData(uint32_t id, const uint32_t data) {
	if (data & (0x03 << 4)) {
		// Not a supported header
		dataStateGoDiscard();
		return;
	}
	parseMessage.data32[0] = parseMessage.data32[1] = 0;
	uint32_t dlc = data & 0x0f;
	parseMessage.dlc = dlc;
	if (data & (1 << 6)) {
		dlc = 0;
		id |= ID_RTR;
	}
	parseMessage.id = id;
	if (dlc) {
		dataStateGoNext(MS_DATA0, dlc >= 4 ? 32 : dlc * 8);
	} else {
		dataStateGoCrc();
	}
}

void CAN2040::dataStateUpdateStart(const uint32_t data) {
	parseMessage.id = data;
	reportNoteMessageStart();
	dataStateGoNext(MS_HEADER, 17);
}

void CAN2040::dataStateUpdateHeader(uint32_t data) {
	data |= parseMessage.id << 17;
	if ((data & 0x60) == 0x60) {
		// Extended header
		parseMessage.id = data;
		dataStateGoNext(MS_EXT_HEADER, 20);
		return;
	}
	parseCrc = crcBytes(0, data, 3);
	dataStateGoData((data >> 7) & 0x7ff, data);
}

void CAN2040::dataStateUpdateExtHeader(const uint32_t data) {
	const uint32_t hdr1 = parseMessage.id;
	const uint32_t crc = crcBytes(0, hdr1 >> 4, 2);
	parseCrc = crcBytes(crc, ((hdr1 & 0x0f) << 20) | data, 3);
	const uint32_t id = (
		((hdr1 << 11) & 0x1ffc0000) | ((hdr1 << 13) & 0x3e000) | (data >> 7) | ID_EFF
	);
	dataStateGoData(id, data);
}

void CAN2040::dataStateUpdateData0(const uint32_t data) {
	uint32_t dlc = parseMessage.dlc, bits = dlc >= 4 ? 32 : dlc * 8;
	parseCrc = crcBytes(parseCrc, data, dlc);
	parseMessage.data32[0] = __builtin_bswap32(data << (32 - bits));

	if (dlc > 4) {
		dataStateGoNext(MS_DATA1, dlc >= 8 ? 32 : (dlc - 4) * 8);
	} else {
		dataStateGoCrc();
	}
}

void CAN2040::dataStateUpdateData1(const uint32_t data) {
	uint32_t dlc = parseMessage.dlc, bits = dlc >= 8 ? 32 : (dlc - 4) * 8;
	parseCrc = crcBytes(parseCrc, data, dlc - 4);
	parseMessage.data32[1] = __builtin_bswap32(data << (32 - bits));
	dataStateGoCrc();
}

void CAN2040::dataStateUpdateCrc(const uint32_t data) {
	if (((parseCrc << 1) | 1) != data) {
		dataStateGoError();
		return;
	}

	reportNoteCrcSuccess();
	unstufClearState(&unstuf);
	dataStateGoNext(MS_ACK, 2);
}

void CAN2040::dataStateUpdateAck(const uint32_t data) {
	if (data != 0x01) {
		// Undo unstuf_clear_state() for correct SOF detection in
		// data_state_line_passive()
		unstufRestoreState(&unstuf, (parseCrcBits << 2) | data);

		dataStateGoError();
		return;
	}
	reportNoteAckSuccess();
	dataStateGoNext(MS_EOF0, 4);
}

void CAN2040::dataStateUpdateEof0(const uint32_t data) {
	if (data != 0x0f || pioRxCheckStAll()) {
		dataStateGoError();
		return;
	}
	unstufClearState(&unstuf);
	dataStateGoNext(MS_EOF1, 5);
}

void CAN2040::dataStateUpdateEof1(const uint32_t data) {
	if (data == 0x1f) {
		// Success
		reportNoteEofSuccess();
		dataStateGoNext(MS_START, 1);
	} else if (data >= 0x1c || (data >= 0x18 && reportIsNotInTx())) {
		// Message fully transmitted - followed by "overload frame"
		reportNoteEofSuccess();
		dataStateGoDiscard();
	} else {
		dataStateGoError();
	}
}

void CAN2040::dataStateUpdateDiscard(uint32_t data) {
	dataStateGoDiscard();
}

void CAN2040::dataStateUpdate(const uint32_t data) {
	switch (parseState) {
		case MS_START: dataStateUpdateStart(data);
			break;
		case MS_HEADER: dataStateUpdateHeader(data);
			break;
		case MS_EXT_HEADER: dataStateUpdateExtHeader(data);
			break;
		case MS_DATA0: dataStateUpdateData0(data);
			break;
		case MS_DATA1: dataStateUpdateData1(data);
			break;
		case MS_CRC: dataStateUpdateCrc(data);
			break;
		case MS_ACK: dataStateUpdateAck(data);
			break;
		case MS_EOF0: dataStateUpdateEof0(data);
			break;
		case MS_EOF1: dataStateUpdateEof1(data);
			break;
		case MS_DISCARD: dataStateUpdateDiscard(data);
			break;
	}
}

/****************************************************************
 * Input processing
 ****************************************************************/

void CAN2040::processRx(const uint32_t rxData) {
	unstufAddBits(&unstuf, rxData, PIO_RX_WAKE_BITS);
	rawBitCount += PIO_RX_WAKE_BITS;

	// undo bit stuffing
	for (;;) {
		const int ret = unstufPullBits(&unstuf);
		if (likely(ret > 0)) {
			// Need more data
			break;
		} else if (likely(!ret)) {
			// Pulled the next field - process it
			dataStateUpdate(unstuf.unstuffedBits);
		} else {
			if (ret == -1)
				// 6 consecutive high bits
				dataStateLinePassive();
			else
				// 6 consecutive low bits
				dataStateLineError();
		}
	}
}

void CAN2040::pioIrqHandler() {
	uint32_t ints = pioHardware->ints0;
	while (likely(ints & SI_RX_DATA)) {
		const uint32_t rxData = pioHardware->rxf[1];
		processRx(rxData);
		ints = pioHardware->ints0;
		if (likely(!ints))
			return;
	}

	if (ints & SI_ACKDONE) {
		// Ack of received message completed successfully
		reportLineAckDone();
	} else if (ints & SI_MATCHED) {
		// Transmit message completed successfully
		reportLineMatched();
	} else if (ints & SI_MAYTX) {
		// Bus is idle, but not all bits may have been flushed yet
		reportLineMaytx();
	} else if (ints & SI_TXPENDING) {
		// Schedule a transmit
		reportLineTxPending();
	}
}

/****************************************************************
 * Transmit queuing
 ****************************************************************/

bool CAN2040::checkTransmit() const {
	uint32_t tx_pull_posNew = readl(&txPullPosition);
	uint32_t tx_push_posNew = txPushPosition;
	const uint32_t pending = tx_push_posNew - tx_pull_posNew;
	return pending < std::size(txQueue);
}

int CAN2040::transmit(const Message* msg) {
	uint32_t tx_pull_posNew = readl(&txPullPosition);
	uint32_t tx_push_posNew = txPushPosition;
	const uint32_t pending = tx_push_posNew - tx_pull_posNew;
	if (pending >= std::size(txQueue)) {
		// Tx queue full
		return -1;
	}

	// Copy msg into transmit queue
	TransmitQueue* qt = &txQueue[txQueuePosition(tx_push_posNew)];
	const uint32_t id = msg->id;
	if (id & ID_EFF) {
		qt->msg.id = id & ~0x20000000;
	} else {
		qt->msg.id = id & (ID_RTR | 0x7ff);
	}

	qt->msg.dlc = msg->dlc & 0x0f;
	uint32_t data_len = qt->msg.dlc > 8 ? 8 : qt->msg.dlc;
	if (qt->msg.id & ID_RTR) {
		data_len = 0;
	}

	qt->msg.data32[0] = qt->msg.data32[1] = 0;
	memcpy(qt->msg.data, msg->data, data_len);

	// Calculate crc and stuff bits
	uint32_t crc = 0;
	memset(qt->stuffedData, 0, sizeof(qt->stuffedData));
	bitstuffer_s bs = {1, 0, qt->stuffedData};
	const uint32_t edlc = qt->msg.dlc | (qt->msg.id & ID_RTR ? 0x40 : 0);
	if (qt->msg.id & ID_EFF) {
		// Extended header
		const uint32_t id = qt->msg.id;
		const uint32_t h1 = ((id & 0x1ffc0000) >> 11) | 0x60 | ((id & 0x3e000) >> 13);
		const uint32_t h2 = ((id & 0x1fff) << 7) | edlc;
		crc = crcBytes(crc, h1 >> 4, 2);
		crc = crcBytes(crc, ((h1 & 0x0f) << 20) | h2, 3);
		bs_push(&bs, h1, 19);
		bs_push(&bs, h2, 20);
	} else {
		// Standard header
		const uint32_t hdr = ((qt->msg.id & 0x7ff) << 7) | edlc;
		crc = crcBytes(crc, hdr, 3);
		bs_push(&bs, hdr, 19);
	}

	for (uint32_t i = 0; i < data_len; i++) {
		const uint32_t v = qt->msg.data[i];
		crc = crcByte(crc, v);
		bs_push(&bs, v, 8);
	}

	qt->crc = crc & 0x7fff;
	bs_push(&bs, qt->crc, 15);
	bs_pushraw(&bs, 1, 1);
	qt->stuffedWords = bs_finalize(&bs);

	// Submit
	writel(&txPushPosition, tx_push_posNew + 1);

	// Wakeup if in TS_IDLE state
	__DMB();
	pioSignalSetTxPending();

	return 0;
}

/****************************************************************
 * Setup
 ****************************************************************/

void CAN2040::setup(const uint32_t newPioIndex) {
	// memset(0, sizeof(*cd));
	pioIndex = !!newPioIndex;
	pioHardware = (pioIndex ? pio1_hw : pio0_hw);
}

void CAN2040::callbackConfig(const ReceiveCallback newRxCallback) {
	rxCallback = newRxCallback;
}

void CAN2040::start(const uint32_t bitrate, const uint32_t ngpio_rx, const uint32_t ngpio_tx) {
	systemClock = clock_get_hz(clk_sys);
	rxGpio = ngpio_rx;
	txGpio = ngpio_tx;
	dataStateClearBits();
	pioSetup(bitrate);
	dataStateGoDiscard();
}

void CAN2040::stop() const {
	pioIrqDisable();
	pioStateMachineSetup();
}

void CAN2040::getStatistics(BusStatistics* retstats) {
	for (;;) {
		memcpy(retstats, &stats, sizeof(BusStatistics));
		if (memcmp(retstats, &stats, sizeof(BusStatistics)) == 0) {
			// Successfully copied data
			return;
		}

		// Raced with irq handler update - retry copy
	}
}
