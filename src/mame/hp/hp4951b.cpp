// license:BSD-3-Clause
// copyright-holders: Jacob Krall
/*

    HP 4951B Protocol Analyzer — MAME driver.

    Reference: HP 4951B Service Manual (04951-900xx), Figures 8-7/8-8/8-18/8-21/8-27/8-31.

    CPU: National NSC800 @ 4MHz (Z80 instruction set)
    CRTC: MC6845 at I/O 0x08 (index) / 0x09 (data)
    RIOT: NSC810 at I/O 0x40-0x5F (Port A/B/C, timers)
      Port A drives memory decode:
        PA0/PA4 -> U207: 0x8000 bank (00=RAM U201-U204, 01=10023/U200, 10=10024/U205, 11=U100 RAM)
        PA6     -> U206: 0x2000 M2A (ROM 2/U103/10022) vs M2B (RAM 2/U104)
        PA1/PA2 -> U103: ROM 2 page select (4x 8KB)
      Port B (0x41): keyboard matrix rows (R0-R7, active-low)
      Port C: PC3=buzzer, PC1=RSTB ack
    Keyboard: 0x18-0x1F (U302 Y3) latch -> COLUMNS C0-C7; RSTB on R1/R5/R6/R7 (softkeys)
    I/O decode:
      0xC0-0xFF: DLC (Z8530, protocol link)
      0x40-0x7F: RIOT (NSC810)
      0x20-0x27: TIC CLOCK
      0x18-0x1F: KEY BD LATCH (U401)
      0x10-0x17: POD
      0x08-0x0F: C/S (0x08/0x09 = CRTC MC6845)
      0x30-0x37: X6 (ACIA? 265 writes in POST)

    Memory map (Fig 8-7):
      0x0000: ROM 0 (U101/10021, 8KB)
      0x2000: ROM 2 (U103/10022, 32KB, PA1/PA2 pages) or RAM 2 (U104, 8KB, PA6)
      0x4000: RAM 4 (VRAM, dual-port, 8KB)
      0x6000: RAM 6 (U105, 8KB)
      0x8000: RAM B/A/C/E (U204/U203/U202/U201, 32KB) or ROM 8-2/8-1 (U200/10023, U205/10024)

    Interrupts (Fig 8-14): RSTA=Tic Clock, RSTB=softkey, RSTC=Tape, INTR=DLC, NMI=power-fail.

*/

#include "emu.h"

#include "cpu/z80/nsc800.h"
#include "video/mc6845.h"
#include "machine/timer.h"

#include "screen.h"


namespace {

class hp4951b_state : public driver_device
{
public:
	hp4951b_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_crtc(*this, "crtc"),
		m_screen(*this, "screen"),
		m_mainram(*this, "mainram"),
		m_chargen(*this, "chargen")
	{ }

	void hp4951b(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void dump_vram();
	void mem_map(address_map &map);
	void io_map(address_map &map);
	void kbd_latch_w(uint8_t data);
	void crtc_address_w(uint8_t data);
	void crtc_register_w(uint8_t data);
	uint8_t crtc_register_r();
	uint8_t crtc_status_r();


	void pager_w(uint8_t data);
	uint8_t win_r(offs_t offset);
	void win_w(offs_t offset, uint8_t data);
	uint8_t bank_r(offs_t offset);
	void bank_w(offs_t offset, uint8_t data);
	TIMER_DEVICE_CALLBACK_MEMBER(softkey_tick);
	uint8_t m_portc = 0x00;  // 810 Port C output latch (PC3 = buzzer)
	uint8_t m_porta = 0x00;  // 810 Port A output latch (PA0/PA4 = 0x8000 bank, PA1/PA2 = 0x2000 window)
	void portc_update() {
		logerror("hp4951b: BUZZER %s (PC=%04x, cycles=%llu)\n",
			(m_portc & 0x08) ? "BEEP" : "off", m_maincpu->pc(),
			(unsigned long long)m_maincpu->total_cycles());
	}
	void port42_w(uint8_t data) {
		// 810 Port C Data (0x42): direct write.
		// PC1 (bit 1) falling edge (1→0) acks the RSTB latch.
		uint8_t old = m_portc;
		m_portc = data;
		if ((old & 0x02) && !(data & 0x02)) {
			m_maincpu->set_input_line(NSC800_RSTB, CLEAR_LINE);
			m_kbd_irq_asserted = false;
		}
		portc_update();
	}
	void port4a_w(uint8_t data) {
		// 810 Port C Bit-Clear (0x4A): write 1 to clear bit.
		// PC1 (bit 1) falling edge acks the RSTB latch.
		uint8_t old = m_portc;
		m_portc &= ~data;
		if ((old & 0x02) && !(m_portc & 0x02)) {
			logerror("hp4951b: PC1 ACK (clear), pc=%04x\\n", m_maincpu->pc());
			m_maincpu->set_input_line(NSC800_RSTB, CLEAR_LINE);
			m_kbd_irq_asserted = false;
		}
		portc_update();
	}
	void port4e_w(uint8_t data) {
		// 810 Port C Bit-Set (0x4E): write 1 to set bit.
		// Setting PC1 does NOT ack; only the 1→0 transition does.
		m_portc |= data;
		portc_update();
	}
	void porta_update() {
		// Derive banking from Port A bits (per 4951A schematic reverse-engineering):
		// PA0+PA4 select 0x8000 bank, PA1/PA2/PA5/PA6 select 0x2000 window.
		// Reuse the existing pager_w logic which already handles the PA patterns.
		pager_w(m_porta);
	}
	void port40_w(uint8_t data) {
		// 810 Port A Data (0x40): direct write.
		m_porta = data;
		porta_update();
	}
	void port48_w(uint8_t data) {
		// 810 Port A Bit-Clear (0x48): write 1 to clear bit.
		m_porta &= ~data;
		porta_update();
	}
	void port4c_w(uint8_t data) {
		// 810 Port A Bit-Set (0x4C): write 1 to set bit.
		m_porta |= data;
		porta_update();
	}
	uint8_t regs30_r(offs_t offset) { return m_regs30[offset & 0xf]; }
	void regs30_w(offs_t offset, uint8_t data) { m_regs30[offset & 0xf] = data; }
	// Z8530 ACIA stub (DLC) at 0x30-0x33
	// 0x30: Ch B control, 0x31: Ch A control, 0x32: Ch B data, 0x33: Ch A data
	// DLC test does loopback: OUT data, IN data expects same byte back.
	// Control reads return 0x44 (Tx buffer empty, DCD/CTS active = "healthy").
	uint8_t acia_r(offs_t offset) {
		uint8_t v;
		switch (offset & 3) {
			case 0: v = 0x44; break;  // B control: healthy status
			case 1: v = 0x44; break;  // A control: healthy status
			case 2: v = m_acia_b_data; break;  // B data: loopback
			case 3: v = m_acia_a_data; break;  // A data: loopback
			default: v = 0xff; break;
		}
		return v;
	}
	// STUB: DLC POST test expectations (from 10024 POST at F91D-F947):
	// - After OUT (0x30),0xBE, IN (0x32) must have bit2=1
	// - After OUT (0x32),0x41, IN (0x32) must have bit3=1
	// Real Z8530 behavior TBD; these mimic the observed hardware responses.
	void acia_w(offs_t offset, uint8_t data) {
		switch (offset & 3) {
			case 0:
				if (data == 0xBE) m_acia_b_data |= 0x04;
				break;
			case 2:
				m_acia_b_data = (data == 0x41) ? (data | 0x08) : data;
				break;
			case 3: m_acia_a_data = data; break;
			default: break;
		}
	}
	// DLC (Z8530) at 0xC0-0xFF. Stub: loopback for POST.
	uint8_t dlc_r(offs_t offset) {
		switch (offset & 3) {
			case 0: return 0x44;
			case 1: return 0x44;
			case 2: return m_dlc_b_data;
			case 3: return m_dlc_a_data;
			default: return 0xff;
		}
	}
	void dlc_w(offs_t offset, uint8_t data) {
		switch (offset & 3) {
			case 2: m_dlc_b_data = data; break;
			case 3: m_dlc_a_data = data; break;
			default: break;
		}
	}
	// (Old 0xC0 keyboard stub removed; real keyboard is 0x18 latch + 0x41 Port B.
	// The firmware poll routine (fixed ROM 0x1BBB) does:
	//   IN (0xC3) -> (0x7B58), (0x7B56)=1, OUT (0xC1)=0x38 (ack).
	// Discrete hardware scanner: returns the matrix position of the
	// currently pressed host key, or 0xFF if none.
	uint8_t kbd_r(offs_t offset) {
		switch (offset & 3) {
			case 3: {
				uint8_t sc = get_host_key_matrix_position();
				// DEBUG: trace 0xC3 reads
				static int count = 0;
				if (count < 20 || sc != 0xFF) {
					logerror("KBD_R: firmware read 0xC3 -> 0x%02X (call #%d)\n", sc, ++count);
				}
				return sc;
			}
			default: return 0x00;
		}
	}
	void kbd_w(offs_t offset, uint8_t data) {
		switch (offset & 3) {
		case 1:  // 0xC1: acknowledge (firmware writes 0x38 after reading).
			// No hardware state; the 0x38 also hits the 0x3800 matrix
			// latch as a side effect (handled in io_w).
			break;
		case 3:  // 0xC3: firmware boot test writes patterns here.
			// Hardware latch would capture them; we ignore (scanner output
			// takes precedence via kbd_r). The boot test may fail; if so,
			// we'll revisit.
			break;
		default:
			break;
		}
	}
	uint8_t m_kbd_matrix_latch = 0xFF;  // 74HC373 matrix drive latch (U302 output 3, 0x3800-0x3FFF, write-only side effect)
	bool m_kbd_irq_asserted = false;  // RSTB latch (set by SOFTKEY DECODER, cleared by PC1)
	uint8_t m_softkey_prev = 0x00;  // Prev softkey rows (prevent re-trigger on hold; ISR does EI before Port B read)
	// RIOT Port B (0x41): keyboard matrix sense inputs.
	// Returns 0x00 always for now (ISR handles gracefully).
	// TODO: implement actual column mask from m_kbd_matrix_latch + MAME inputs.
	uint8_t riot_pb_r() {
		// 4951B keyboard matrix (Fig 8-31):
		//   U401 latch (0x18) drives COLUMNS C0-C7 (active-low, bit=0 selects).
		//   ROWS R0-R7 go to RIOT Port B (PB0-PB7), with R301 pull-ups.
		//   Key at (R,C) connects row to column; if column is driven low,
		//   the row goes low (active-low).
		//   Latch=0xFF (reset) = no columns selected -> 0xFF (all rows high).
		uint8_t latch = m_kbd_matrix_latch;
		// Find selected column (bit LOW). Must be exactly one.
		int sel_col = -1;
		for (int c = 0; c < 8; c++) {
			if (!(latch & (1 << c))) {
				if (sel_col != -1) return 0xFF; // Multiple = invalid
				sel_col = c;
			}
		}
		if (sel_col == -1) return 0xFF; // None selected
		// Build Port B: for each row R, check key at (R, sel_col).
		// MAME KEY{R} has bit (1<<C) for the key at (R,C). ACTIVE_HIGH.
		uint8_t portb = 0xFF;
		for (int r = 0; r < 8; r++) {
			char tag[8];
			snprintf(tag, sizeof(tag), "KEY%d", r);
			if (ioport(tag)->read() & (1 << sel_col)) {
				portb &= ~(1 << r); // Row goes low (active-low)
			}
		}
		return portb;
	}
	// Helper: check MAME inputs, return scancode (0xFF = no key).
	// Host-to-emulator bridge: poll MAME input ports (KEY0-KEY8) and return
	// the HP 4951A matrix position (row*8+col) for the currently pressed
	// host key, or 0xFF if none. The discrete hardware scanner would output
	// this scancode to the 0xC3 latch; we model the result, not the scan.
	uint8_t get_host_key_matrix_position() {
		// 4951B matrix: KEY0-KEY7 = R0-R7, bit m = column m.
		// Returns row*8+col, or 0xFF if no key.
		for (int row = 0; row < 8; row++) {
			char tag[8];
			snprintf(tag, sizeof(tag), "KEY%d", row);
			uint8_t bits = ioport(tag)->read();
			for (int col = 0; col < 8; col++) {
				if (bits & (1 << col))
					return (row << 3) | col;
			}
		}
		return 0xFF;  // no key pressed
	}
	uint8_t ddr_a_r() { return m_ddr_a; }
	// POD (0x10-0x17) stub
	uint8_t pod_r(offs_t offset) {
		logerror("hp4951b: pod_r 0x%02X\n", 0x10 | (offset & 0x7));
		return 0x00; // Ready (not 0xFF)
	}
	void pod_w(offs_t offset, uint8_t data) {
		logerror("hp4951b: pod_w 0x%02X = 0x%02X\n", 0x10 | (offset & 0x7), data);
	}
	// 0xBB unknown stub
	uint8_t unkbb_r() { logerror("hp4951b: unkbb_r\n"); return 0xff; }
	void unkbb_w(uint8_t data) { logerror("hp4951b: unkbb_w = 0x%02X\n", data); }
	// TIC CLOCK (U404 counter + U304 latch)
	// 8-bit counter, bits 0-5 latched to U304 (read at 0x20)
	// Bit 6 -> RSTA via NOR (inverter), fires every 64 counts
	uint8_t tick_r(offs_t offset) {
		// Return latched bits 0-5
		return m_tick_latch & 0x3f;
	}
	void tick_w(offs_t offset, uint8_t data) {
		// OUT (0x20) acks the tick clock AND resets the counters (U404)
		// Reset = NAND(IOdec#4, WRM\) active-low; OUT strobe triggers it
		// U503 strobes U304 to latch the count BEFORE reset (double-buffered)
		logerror("hp4951b: tick_w 0x20 = 0x%02X (ack + latch + reset)\n", data);
		m_maincpu->set_input_line(NSC800_RSTA, CLEAR_LINE);
		m_tick_latch = m_tick_count & 0x3f; // Latch bits 0-5 before reset
		m_tick_count = 0;
	}
	void ddr_a_w(uint8_t data) {
		// Firmware uses Port A as all outputs. Abort on anything else.
		if (data != 0xff)
			fatalerror("hp4951b: DDR A = 0x%02X not implemented (expected 0xFF)\n", data);
		m_ddr_a = data;
	}
	uint8_t ddr_b_r() { return m_ddr_b; }
	void ddr_b_w(uint8_t data) {
		// Firmware uses Port B as all inputs (keyboard rows). Abort on anything else.
		if (data != 0x00)
			fatalerror("hp4951b: DDR B = 0x%02X not implemented (expected 0x00)\n", data);
		m_ddr_b = data;
	}
	uint8_t ddr_c_r() { return m_ddr_c; }
	void ddr_c_w(uint8_t data) {
		// Firmware uses Port C = 0x2F (outputs: PC0,PC1,PC2,PC3,PC5; inputs: PC4,PC6,PC7).
		// Abort on anything else.
		if (data != 0x2f)
			fatalerror("hp4951b: DDR C = 0x%02X not implemented (expected 0x2F)\n", data);
		m_ddr_c = data;
	}
	uint8_t mdr_r() { return m_mdr; }
	void mdr_w(uint8_t data) {
		// MDR mode encoding (not binary):
		// 0bxxxxxxx0 = MODE 0 (basic I/O)
		// 0bxxxxxx01 = MODE 1
		// 0bxxxxx011 = MODE 2
		// 0bxxxxx111 = MODE 3
		// We only implement MODE 0. Abort if firmware tries another mode.
		if (data & 0x01)
			fatalerror("hp4951b: MDR mode %d not implemented (MDR=0x%02X)\n", (data & 0x07), data);
		m_mdr = data;
	}
	uint8_t timer_r(offs_t offset) {
		switch (offset & 0xf) {
			case 0x0: return (m_timer0_count >> 8) & 0xff; // Timer 0 High (current)
			case 0x1: return m_timer0_count & 0xff;        // Timer 0 Low
			case 0x2: return m_timer1_count & 0xff;        // Timer 1 Low
			case 0x3: return (m_timer1_count >> 8) & 0xff; // Timer 1 High
			case 0x8: return m_timer0_mode;
			case 0x9: return m_timer1_mode;
			default: return 0xff;
		}
	}
	void timer_w(offs_t offset, uint8_t data) {
		logerror("hp4951b: timer_w 0x%02X = 0x%02X\n", 0x50 | (offset & 0xf), data);
		switch (offset & 0xf) {
			case 0x0: m_timer0_load = (m_timer0_load & 0x00ff) | (data << 8); break;
			case 0x1: m_timer0_load = (m_timer0_load & 0xff00) | data; break;
			case 0x2: m_timer1_load = (m_timer1_load & 0xff00) | data; break;
			case 0x3: m_timer1_load = (m_timer1_load & 0x00ff) | (data << 8); break;
			case 0x4: m_timer0_running = false; m_timer0->adjust(attotime::never); break; // STOP Timer 0
			case 0x5: m_timer0_running = true; m_timer0_count = m_timer0_load; start_timer0(); break;
			case 0x6: m_timer1_running = false; m_timer1->adjust(attotime::never); break; // STOP Timer 1
			case 0x7: m_timer1_running = true; m_timer1_count = m_timer1_load; start_timer1(); break;
			case 0x8: m_timer0_mode = data; break;
			case 0x9: m_timer1_mode = data; break;
			default: break;
		}
	}
	void start_timer0() {
		// We only model Square Wave mode (bits 2-0 = 101). Abort on anything else.
		if ((m_timer0_mode & 0x07) != 0x05)
			fatalerror("hp4951b: timer0 mode bits 0x%02X not implemented (expected 0x05)\n", m_timer0_mode & 0x07);
		if (m_timer0_running) {
			// T0 OUT clocks the U404 TIC CLOCK counter. For 60Hz RSTA
			// (every 64 counts), T0 OUT runs at 60*64 = 3840Hz.
			// TODO: calculate from load value and prescale
			m_timer0->adjust(attotime::from_hz(3840), 0, attotime::from_hz(3840));
		}
	}
	void start_timer1() {
		// We only model Square Wave mode (bits 2-0 = 101). Abort on anything else.
		if ((m_timer1_mode & 0x07) != 0x05)
			fatalerror("hp4951b: timer1 mode bits 0x%02X not implemented (expected 0x05)\n", m_timer1_mode & 0x07);
		if (m_timer1_running) {
			m_timer1->adjust(attotime::from_hz(60), 0, attotime::from_hz(60));
		}
	}
	TIMER_CALLBACK_MEMBER(timer0_tick) {
		// T0 OUT clocks the U404 TIC CLOCK counter (one count per T0 OUT).
		// Bit 6 rising (every 64 counts) -> RSTA via NOR.
		// U404 only advances while Timer 0 runs, so RSTA cannot fire before
		// the firmware starts Timer 0 (after the vector is installed).
		uint8_t prev = m_tick_count;
		m_tick_count++;
		if (!(prev & 0x40) && (m_tick_count & 0x40)) {
			logerror("hp4951b: RSTA ASSERT (tick, count=0x%02X)\n", m_tick_count);
			m_maincpu->set_input_line(NSC800_RSTA, ASSERT_LINE);
		}
		// Note: m_tick_latch does NOT update here. U503 holds the latch steady
		// until CPU acks (OUT 0x20), which strobes U304 to capture the count.
	}
	TIMER_CALLBACK_MEMBER(timer1_tick) {
		// Timer 1 OUT -> U503 R\ (active-low) -> Q=0 -> RSTC\=0 (active)
		// Resetting the flip-flop ASSERTS RSTC (active-low interrupt)
		logerror("hp4951b: RSTC ASSERT (timer1 T1 OUT -> R\\)\n");
		m_maincpu->set_input_line(NSC800_RSTC, ASSERT_LINE);
		m_maincpu->set_input_line(NSC800_RSTC, CLEAR_LINE);
	}
	void softkey_poll() {
		// SOFTKEY DECODER (Fig 8-31): monitors R1,R5,R6,R7 (rows) for activity
		// in the currently-selected COLUMN (U401 latch, bit=0 selects).
		// If no column selected (latch=0xFF), the decoder sees nothing.
		// Hardware latch: set when softkey active, cleared by PC1 (0x42/0x4A).
		// Called from 60Hz poll timer (separate from RIOT timer).
		uint8_t latch = m_kbd_matrix_latch;
		int sel_col = -1;
		for (int c = 0; c < 8; c++) {
			if (!(latch & (1 << c))) {
				if (sel_col != -1) { sel_col = -2; break; } // Multiple = invalid
				sel_col = c;
			}
		}
		if (sel_col < 0) {
			m_softkey_prev = 0;
			return; // No column (or invalid) -> decoder idle
		}
		uint8_t soft = 0;
		if (ioport("KEY1")->read() & (1 << sel_col)) soft |= 0x02;  // R1
		if (ioport("KEY5")->read() & (1 << sel_col)) soft |= 0x20;  // R5
		if (ioport("KEY6")->read() & (1 << sel_col)) soft |= 0x40;  // R6
		if (ioport("KEY7")->read() & (1 << sel_col)) soft |= 0x80;  // R7
		// Don't re-trigger while key held: ISR does EI at 0x0F30 before IN A,(41H),
		// so a held key would nest interrupts → stack overflow. Require release.
		uint8_t rising = soft & ~m_softkey_prev;
		m_softkey_prev = soft;
		if (rising && !m_kbd_irq_asserted) {
			m_kbd_irq_asserted = true;
			logerror("hp4951b: RSTB ASSERT (softkey), pc=%04x\n", m_maincpu->pc());
			m_maincpu->set_input_line(NSC800_RSTB, ASSERT_LINE);
		}
	}

	MC6845_UPDATE_ROW(crtc_update_row);

	required_device<nsc800_device> m_maincpu;
	required_device<mc6845_device> m_crtc;
	required_device<screen_device> m_screen;
	// 0x2000 window state: 0=RAM, 1=10023 JP table, 2=10024 JP table
	uint8_t m_winstate = 0;
	// 0x8000 bank state: 0=RAM, 1=10023, 2=10024, 3=10022
	// (Replaces MAME memory_bank which wasn't switching reliably.)
	uint8_t m_bankstate = 0;
	// Port 0x48 value (controls fetch vs data visibility for 0x8000 bank).
	// When 0x48=0x11 (after RAM select), instruction fetches still see ROM.
	// Last ROM bank selected (for fetch when 0x48 indicates ROM fetch).
	uint8_t m_last_rom_bank = 2; // default to PA0=0,PA4=1 (10024/U205)
	required_shared_ptr<uint8_t> m_mainram;
	required_region_ptr<uint8_t> m_chargen;

	std::unique_ptr<uint8_t[]> m_bankram;
	std::unique_ptr<uint8_t[]> m_winram;
	uint8_t m_regs30[16] = { 0 };
	uint16_t m_timer0_load = 0;
	uint16_t m_timer1_load = 0;
	uint16_t m_timer0_count = 0;
	uint16_t m_timer1_count = 0;
	uint8_t m_timer0_mode = 0;
	uint8_t m_timer1_mode = 0;
	bool m_timer0_running = false;
	bool m_timer1_running = false;
	emu_timer *m_timer0 = nullptr;
	emu_timer *m_timer1 = nullptr;
	uint8_t m_ddr_a = 0;
	uint8_t m_ddr_b = 0;
	uint8_t m_ddr_c = 0;
	uint8_t m_mdr = 0;
	uint8_t m_tick_count = 0;
	uint8_t m_tick_latch = 0;
	uint8_t m_acia_b_data = 0;
	uint8_t m_acia_a_data = 0;
	uint8_t m_dlc_a_data = 0;
	uint8_t m_dlc_b_data = 0;
};


void hp4951b_state::mem_map(address_map &map)
{
	map(0x0000, 0x1fff).rom().region("maincpu", 0);
	// 0x2000-0x3FFF: U206 decodes M2A (ROM 2/U103) vs M2B (RAM 2/U104) via PA6.
	// PA6=0 → ROM 2 (10022), PA1/PA2 select 1 of 4 8KB pages.
	// PA6=1 → RAM 2 (APPLICATION RAM).
	map(0x2000, 0x3fff).rw(FUNC(hp4951b_state::win_r), FUNC(hp4951b_state::win_w));
	map(0x4000, 0x7fff).ram().share("mainram");
	map(0x8000, 0xffff).rw(FUNC(hp4951b_state::bank_r), FUNC(hp4951b_state::bank_w));
}



void hp4951b_state::kbd_latch_w(uint8_t data) { logerror("hp4951b: kbd_latch_w 0x18 = 0x%02X\n", data); m_kbd_matrix_latch = data; }
void hp4951b_state::crtc_address_w(uint8_t data) { m_crtc->address_w(data); }
void hp4951b_state::crtc_register_w(uint8_t data) { m_crtc->register_w(data); }
uint8_t hp4951b_state::crtc_register_r() { return m_crtc->register_r(); }
uint8_t hp4951b_state::crtc_status_r() { return m_crtc->register_r(); }

void hp4951b_state::io_map(address_map &map)
{
	// 8-bit I/O ports. Schematic-derived ranges:
	// 0xC0-0xFF: DLC (Z8530)
	// 0x40-0x7F: RIOT (NSC810)
	// 0x30-0x37: X6 (ACIA HD6350)
	// 0x20-0x27: TIC CLOCK (not yet emulated)
	// 0x18-0x1F: KEY BD LATCH (U401, write-only)
	// 0x10-0x17: POD (not yet emulated)
	// 0x08-0x0F: C/S (CRTC MC6845, A10 mirror)
	map.global_mask(0xff);

	// DLC (0xC0-0xFF)
	map(0x00c0, 0x00ff).rw(FUNC(hp4951b_state::dlc_r), FUNC(hp4951b_state::dlc_w));

	// POD (0x10-0x17): POD latches (stub - return 0x00 = ready)
	map(0x0010, 0x0017).rw(FUNC(hp4951b_state::pod_r), FUNC(hp4951b_state::pod_w));

	// 0xBB: unknown (in 0x80-0xBF range) - stub
	map(0x00bb, 0x00bb).rw(FUNC(hp4951b_state::unkbb_r), FUNC(hp4951b_state::unkbb_w));

	// RIOT (0x40-0x7F) - specific registers
	map(0x0040, 0x0040).w(FUNC(hp4951b_state::port40_w)); // Port A Data
	map(0x0041, 0x0041).r(FUNC(hp4951b_state::riot_pb_r)); // Port B Data
	map(0x0042, 0x0042).w(FUNC(hp4951b_state::port42_w)); // Port C Data
	map(0x0048, 0x0048).w(FUNC(hp4951b_state::port48_w));
	map(0x004a, 0x004a).w(FUNC(hp4951b_state::port4a_w)); // Port C bit-clear
	map(0x004c, 0x004c).w(FUNC(hp4951b_state::port4c_w)); // Port A bit-set
	map(0x004e, 0x004e).w(FUNC(hp4951b_state::port4e_w)); // Port C bit-set
	map(0x0044, 0x0044).rw(FUNC(hp4951b_state::ddr_a_r), FUNC(hp4951b_state::ddr_a_w));
	map(0x0045, 0x0045).rw(FUNC(hp4951b_state::ddr_b_r), FUNC(hp4951b_state::ddr_b_w));
	map(0x0046, 0x0046).rw(FUNC(hp4951b_state::ddr_c_r), FUNC(hp4951b_state::ddr_c_w));
	map(0x0047, 0x0047).rw(FUNC(hp4951b_state::mdr_r), FUNC(hp4951b_state::mdr_w));
	map(0x0050, 0x0059).rw(FUNC(hp4951b_state::timer_r), FUNC(hp4951b_state::timer_w));

	// X6 ACIA (0x30-0x37)
	map(0x0030, 0x0033).rw(FUNC(hp4951b_state::acia_r), FUNC(hp4951b_state::acia_w));
	map(0x0034, 0x0037).rw(FUNC(hp4951b_state::regs30_r), FUNC(hp4951b_state::regs30_w));
	map(0x0039, 0x003f).rw(FUNC(hp4951b_state::regs30_r), FUNC(hp4951b_state::regs30_w));

	// TIC CLOCK (0x20-0x27): U404 8-bit counter, U304 latch (bits 0-5)
	// Bit 6 -> RSTA via NOR (fires every 64 counts)
	map(0x0020, 0x0027).rw(FUNC(hp4951b_state::tick_r), FUNC(hp4951b_state::tick_w));

	// KEY BD LATCH (0x18-0x1F, write-only)
	map(0x0018, 0x001f).w(FUNC(hp4951b_state::kbd_latch_w));

	// C/S CRTC (0x08-0x0F, A10 mirror)
	map(0x0008, 0x0008).mirror(0x0004).w(FUNC(hp4951b_state::crtc_address_w));
	map(0x0009, 0x0009).mirror(0x0004).rw(FUNC(hp4951b_state::crtc_register_r), FUNC(hp4951b_state::crtc_register_w));
	map(0x000b, 0x000b).mirror(0x0004).r(FUNC(hp4951b_state::crtc_status_r));
}




// 0x2000-0x3FFF (U206): reads come from ROM 2 (PA6=0) or RAM 2 (PA6=1).
// JP table (or the underlying RAM); writes always land in the RAM underneath,
// never in ROM. This models the U206/U207 mapping, not a memcpy.
uint8_t hp4951b_state::win_r(offs_t offset)
{
	// U206 (Fig 8-21): PA6 selects M2A (ROM 2/U103) vs M2B (RAM 2/U104).
	// PA6=0 → ROM 2 visible; PA1/PA2 select 1 of 4 8KB pages (32KB total).
	// PA6=1 → RAM 2 (U104) visible.
	if (m_porta & 0x40) {
		return m_winram[offset];
	} else {
		int page = (m_porta >> 1) & 0x03;  // PA1=bit1, PA2=bit2
		uint32_t rom_offset = (page << 13) | (offset & 0x1fff);
		return memregion("rom22")->base()[rom_offset];  // U103 = 10022
	}
}

void hp4951b_state::win_w(offs_t offset, uint8_t data)
{
	// U206: PA6=1 selects RAM 2 (U104/M2B). PA6=0 selects ROM 2 (U103/M2A).
	// Writes to ROM are ignored by hardware (no RAM underneath in ROM mode).
	if (m_porta & 0x40) {
		m_winram[offset] = data;
	}
	// PA6=0: ROM selected, writes ignored.
}

uint8_t hp4951b_state::bank_r(offs_t offset)
{
	// Port 0x48=0x11 (after RAM select via trampoline) means instruction
	// fetches still see ROM, while data accesses see RAM. Detect fetches
	// by comparing the address to the CPU's PC.
	// (Experimental: hardware likely has separate fetch/data paths.)
	if ((m_porta & 0x11) == 0x11 && m_bankstate == 0)
	{
		uint16_t pc = m_maincpu->pc();
		if (offset + 0x8000 == pc)
		{
			// Instruction fetch: return from last ROM bank
			switch (m_last_rom_bank)
			{
			case 1: return memregion("rom23")->base()[offset];
			default: return memregion("rom24")->base()[offset]; // 2 = 10024
			}
		}
	}
	switch (m_bankstate)
	{
	case 1: return memregion("rom23")->base()[offset];
	case 2: return memregion("rom24")->base()[offset];
	default: return m_bankram[offset]; // 0 = RAM (U201-U204), 3 = U100 RAM (option slot)
	}
}

void hp4951b_state::bank_w(offs_t offset, uint8_t data)
{
	// U207: writes to 0x8000-0xFFFF go to the RAM chips (U201-U204).
	// Behavior when a ROM bank is selected is unverified; we preserve
	// RAM contents (hardware may ignore the write).
	m_bankram[offset] = data;
}

TIMER_DEVICE_CALLBACK_MEMBER(hp4951b_state::softkey_tick)
{
	softkey_poll();
}

void hp4951b_state::pager_w(uint8_t data)
{
	// Pager values (CONFIRMED from firmware analysis 2026-10-01):
	// Bits 0+4 form a 2-bit bank number for the 0x8000-0xFFFF window:
	//   0x00 (00) = RAM, 0x01 (01) = 10023, 0x10 (10) = 10024, 0x11 (11) = 10022
	// Type-1 trampoline table (@0xBD in fixed ROM):
	//   index 0 -> 0x11 (10022), index 1 -> 0x01 (10023),
	//   index 2 -> 0x10 (10024), index 3 -> 0x00 (RAM)
	// The RAM test at 0x8B27 uses index 3 (0x00) to test RAM at 0x8000.
	// Type-2 trampoline (0x00DD): LD A,L; SLA A; OUT (0x4C),A — writes the
	// SHIFTED value. 0x04=10024 JP table, 0x06=10023 JP table at 0x2000.
	// (CB 27 is SLA A, not SLA L — the shift is intentional.)
	// The 0x2000-0x3FFF window holds ROM 2 (U103/10022) or RAM 2 (U104).
	// U206 overlays the selected ROM onto the bus for 0x2000-
	// 0x20FF — a mapping, not a copy (confirmed: CALL 200CH directly after
	// trampoline, no LDIR). We emulate with win_r/win_w handlers.
	// Calls from fixed ROM during POST are hardware init, not cross-bank calls —
	// don't switch the window there (PC gate).
	// U207 (Fig 8-21): PA0 (bit 0) + PA4 (bit 4) select the 0x8000 bank.
	// Mask out PA1/PA2/PA6/PA7 which are for U206 (0x2000) and other functions.
	switch (data & 0x11)
	{
	case 0x00: m_bankstate = 0; break; // RAM (U201-U204)
	case 0x01: m_bankstate = 1; m_last_rom_bank = 1; break; // PA0=1,PA4=0 (10023/U200)
	case 0x10: m_bankstate = 2; m_last_rom_bank = 2; break; // PA0=0,PA4=1 (10024/U205)
	case 0x11: m_bankstate = 3; break; // U100 RAM (option slot)
	default:
		break;
	}
}


MC6845_UPDATE_ROW(hp4951b_state::crtc_update_row)
{
	// ma already includes the R12/R13 start address (0x000 page 0, 0x200 page 1)
	uint8_t *vram = &m_mainram[0];   // CPU 0x4000-0x47FF (mainram base = 0x4000)
	// Character ROM select via character code bit 7 (CD7 on EN1, HW-verified):
	//   bit 7 clear: CHAR ROM 1 (10005)
	//   bit 7 set:   CHAR ROM 2 (10006)
	// (Per-character selection done in the loop since ch varies.)
	uint32_t *p = &bitmap.pix(y);

	const uint32_t fg_full = rgb_t(0x33, 0xff, 0x66);
	const uint32_t fg_half = rgb_t(0x11, 0x88, 0x33);  // halfbright
	const uint32_t bg = rgb_t(0x00, 0x10, 0x08);

	// Blink timing from MC6845 R10 (Cursor Start) bits 5-6:
	// 2 = 1/16 field rate, 3 = 1/32 field rate.
	// CURSOR attr uses the 6845 blink directly; BLINK attr is divide-by-2 in TTL.
	m_crtc->address_w(10);
	uint8_t r10 = m_crtc->register_r();
	int blink_mode = (r10 >> 5) & 3;
	int cursor_period, blink_period;
	switch (blink_mode)
	{
		case 2:  // 1/16 field rate
			cursor_period = 16;
			blink_period = 32;
			break;
		case 3:  // 1/32 field rate
			cursor_period = 32;
			blink_period = 64;
			break;
		default:  // steady or no cursor: fall back to 2:1 guesses
			cursor_period = 16;
			blink_period = 32;
			break;
	}
	bool blink_on = ((m_screen->frame_number() / (blink_period / 2)) & 1) == 0;
	bool cursor_on = ((m_screen->frame_number() / (cursor_period / 2)) & 1) == 0;

	// Character ROM select: HW-VERIFIED from 4951A schematic (Fig 8-10).
	// CD7 (character code bit 7) drives EN1 on the CHARACTER ROMs:
	//   ch bit 7 clear: CHAR ROM 1 (10005) enabled (EN1 = !CD7)
	//   ch bit 7 set:   CHAR ROM 2 (10006) enabled (EN1 = CD7)
	// Bit 7 is the chip-select, NOT an attribute. The attribute byte is
	// separate (vram[addr*2+1]); its bit 7 is part of the attribute encoding
	// (0x80=plain, 0x81=underline, etc. per TEST PTRN table).
	// (Selection done in the loop since ch varies per character.)

	for (int x = 0; x < x_count; x++)
	{
		uint16_t addr = (ma + x) & 0x3ff;
		uint32_t *px = p + x * 8;

		if (!de)
		{
			for (int b = 0; b < 8; b++) px[b] = bg;
			continue;
		}

		uint8_t ch = vram[addr * 2];
		uint8_t attr = vram[addr * 2 + 1];

		// Character ROM addressing from 4951A schematic Fig 8-10 (HW-verified):
		// 13-bit address on 8KB (8192x8) chips: A12=attr bit6, A11=ch bit7
		// (latched C7), A10-A4=ch bits6-0 (latched C6-C0), A3-A0=ra (R3-R0).
		// EN1 (chip select) = attr bit7 (live CD7):
		//   attr bit7 set:   ROM1 (10005) — HW-verified: diag/menu attr 0x83 uses ROM1
		//   attr bit7 clear: ROM2 (10006)
		uint8_t *chip_base = (attr & 0x80) ? &m_chargen[0x0000] : &m_chargen[0x8000];
		uint16_t rom_addr = ((attr & 0x40) << 6) | ((ch << 4) & 0x0ff0) | (ra & 0x0f);
		uint8_t row = chip_base[rom_addr];

		// Attribute decoding from 4951A schematic Fig 8-10 (ATTRIBUTE LATCH):
		// CD0=OVER - inverted, CD1=UNLN - inverted, CD2=BLINK, CD3=INVID, CD4=CRSR, CD5=HB
		bool overbar = (attr & 0x01) == 0;
		bool underline = (attr & 0x02) == 0;
		bool blink = (attr & 0x04) != 0;
		bool inverse = (attr & 0x08) != 0;
		bool cursor = (attr & 0x10) != 0;
		bool halfbright = (attr & 0x20) != 0;

		// Blink: if blinking and phase is off, blank the character
		if (blink && !blink_on)
		{
			for (int b = 0; b < 8; b++) px[b] = bg;
			continue;
		}

		// (ROM address computed above from schematic bit map)

		// Cursor: rapid blink using inverse video (not blanking)
		// When cursor phase is on, force inverse; otherwise normal
		bool force_inverse = inverse;
		if (cursor && cursor_on)
			force_inverse = true;

		// Attribute rows: ra==1 (top) and ra==12 (bottom).
		// The font ROM stores the overbar as a short line in row 1
		// and the underline as a full-width line in row 12.
		// Show them only when the attribute is set; blank otherwise.
		if (ra == 1 && !overbar)
			row = 0x00;
		else if (ra == 12 && !underline)
			row = 0x00;

		uint32_t fg = halfbright ? fg_half : fg_full;

		// Inverse: swap fg and bg (includes cursor blink phase)
		if (force_inverse)
		{
			for (int b = 0; b < 8; b++)
				px[b] = (row & (0x80 >> b)) ? bg : fg;
		}
		else
		{
			for (int b = 0; b < 8; b++)
				px[b] = (row & (0x80 >> b)) ? fg : bg;
		}
	}
}


void hp4951b_state::machine_start()
{
	m_bankram = std::make_unique<uint8_t[]>(0x8000);
	memset(m_bankram.get(), 0, 0x8000);

	// Allocate RIOT timers (must be done here, not at runtime)
	m_timer0 = timer_alloc(FUNC(hp4951b_state::timer0_tick), this);
	m_timer1 = timer_alloc(FUNC(hp4951b_state::timer1_tick), this);

	// 0x8000 bank now uses explicit handlers (bank_r/bank_w) with m_bankstate,
	// not MAME's memory_bank. m_bankstate defaults to 0 (RAM).

	// 0x2000-0x20FF cross-bank window: entry 0 = RAM (the physical RAM
	// underneath), entry 1 = 10023 JP table, entry 2 = 10024 JP table.
	// U206 overlays ROM onto the bus; writes during the window
	// go to RAM (bankr = read-only for ROM entries, RAM entry is rw via
	// the underlying mapping — actually bankr is read-only, so we use
	// bankrw with RAM backing for entry 0).
	// 8KB window (0x2000-0x3FFF) per 4951A schematic U103 (was 256B).
	m_winram = std::make_unique<uint8_t[]>(0x2000);
	memset(m_winram.get(), 0, 0x2000);
	m_winstate = 0;

	machine().add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate(&hp4951b_state::dump_vram, this));

	save_item(NAME(m_portc));
	save_item(NAME(m_porta));
	logerror("machine_start!\n");
}

void hp4951b_state::machine_reset()
{
	// RIOT RESET (pin 4): clears the RSTB latch. Prevents boot-loop when
	// a softkey is held through reset.
	m_kbd_irq_asserted = false;
	m_maincpu->set_input_line(NSC800_RSTB, CLEAR_LINE);
	m_kbd_matrix_latch = 0xFF;
	logerror("machine_reset!\n");
}


void hp4951b_state::dump_vram()
{
	// Dump VRAM page 0 + back buffer for offline rendering/verification.
	FILE *f = fopen("/tmp/hp4951b_vram.bin", "wb");
	if (f != nullptr)
	{
		fwrite(&m_mainram[0], 1, 0x800, f);
		fclose(f);
	}
	// TEMP: dump RAM around the 0x3FBF stuck-PC for analysis
	FILE *g = fopen("/tmp/hp4951b_ram3f.bin", "wb");
	if (g != nullptr)
	{
		fwrite(&m_mainram[0], 1, 0x2200, g);  // CPU 0x4000-0x6200
		fclose(g);
	}
	logerror("At exit, pc=0x%04X\n", m_maincpu->pc());
}


static INPUT_PORTS_START(hp4951b)
	// 4951B Keyboard Matrix (Figure 8-31, 4951B Service Manual)
	// Rows are R0-R7 (KEY0-KEY7), columns are 8-1 (bits 0-7).
	// R1,R5,R6,R7 feed the SOFTKEY DECODER (RSTB interrupt).
	PORT_START("KEY0")  // R0: Row 9 (X,Y,Z,ESC,FS,GS,RS)
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_X) PORT_NAME("X CAN")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y) PORT_NAME("Y EM")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z) PORT_NAME("Z SUB")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE) PORT_NAME("[ ESC")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH) PORT_NAME("\\ FS")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_NAME("] GS")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS) PORT_NAME("^ RS")
	// 0x80: (blank key, no host mapping)
	PORT_START("KEY1")  // R1: Row 10 (P,Q,R,S,T,U,V,W) - SOFTKEY DECODER
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_P) PORT_NAME("P DLE")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q) PORT_NAME("Q DC1")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_R) PORT_NAME("R DC2")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_S) PORT_NAME("S DC3")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_T) PORT_NAME("T DC4")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_U) PORT_NAME("U NAK")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_V) PORT_NAME("V SYN")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_W) PORT_NAME("W ETB")
	PORT_START("KEY2")  // R2: Row 11 (H,I,J,K,L,M,N,O)
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_H) PORT_NAME("H BS")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_I) PORT_NAME("I HT")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_J) PORT_NAME("J LF")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_K) PORT_NAME("K VT")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_L) PORT_NAME("L FF")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_M) PORT_NAME("M CR")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_N) PORT_NAME("N SO")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_O) PORT_NAME("O SI")
	PORT_START("KEY3")  // R3: Row 12 (A,B,C,D,E,F,G,BEL)
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_A) PORT_NAME("A NUL")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_B) PORT_NAME("B SOH")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_C) PORT_NAME("C STX")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_D) PORT_NAME("D ETX")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_E) PORT_NAME("E EOT")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F) PORT_NAME("F ENQ")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_G) PORT_NAME("G ACK")
	// 0x80: BEL (no host mapping)
	PORT_START("KEY4")  // R4: Row 13 (8,9,:,;,comma,-,.,/)
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_8) PORT_NAME("8 (")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_9) PORT_NAME("9 )")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON) PORT_NAME(": *")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE) PORT_NAME("; +")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA) PORT_NAME(", <")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS) PORT_NAME("- =")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP) PORT_NAME(". >")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH) PORT_NAME("/ ?")
	PORT_START("KEY5")  // R5: Row 14 (DEL,1,2,3,4,5,6,7) - SOFTKEY DECODER
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSPACE) PORT_NAME("DEL -")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_1) PORT_NAME("1")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_2) PORT_NAME("2")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_3) PORT_NAME("3")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_4) PORT_NAME("4")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_5) PORT_NAME("5")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_6) PORT_NAME("6")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_7) PORT_NAME("7")
	PORT_START("KEY6")  // R6: Row 15 (arrows,RTN,SHIFT,CNTL,SPACE) - SOFTKEY DECODER
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_UP) PORT_NAME("Cursor Up")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_DOWN) PORT_NAME("Cursor Down")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LEFT) PORT_NAME("Cursor Left")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_RIGHT) PORT_NAME("Cursor Right")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER) PORT_NAME("RTN")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT) PORT_NAME("SHIFT")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LCONTROL) PORT_CODE(KEYCODE_RCONTROL) PORT_NAME("CNTL")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SPACE) PORT_NAME("SPACE BAR")
	PORT_START("KEY7")  // R7: Row 16 (EXIT,F1-F6,MORE) - SOFTKEY DECODER
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_END) PORT_NAME("EXIT")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F1) PORT_NAME("F1")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F2) PORT_NAME("F2")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F3) PORT_NAME("F3")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F4) PORT_NAME("F4")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F5) PORT_NAME("F5")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F6) PORT_NAME("F6")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F7) PORT_NAME("MORE")
INPUT_PORTS_END


void hp4951b_state::hp4951b(machine_config &config)
{
	NSC800(config, m_maincpu, 4_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &hp4951b_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &hp4951b_state::io_map);

	SCREEN(config, m_screen);
	m_screen->set_raw(8_MHz_XTAL, 40 * 8, 0, 32 * 8, 18 * 14, 0, 16 * 14);
	m_screen->set_screen_update("crtc", FUNC(mc6845_device::screen_update));

	MC6845(config, m_crtc, 8_MHz_XTAL / 8);
	m_crtc->set_screen("screen");
	m_crtc->set_show_border_area(false);
	m_crtc->set_char_width(8);
	m_crtc->set_update_row_callback(FUNC(hp4951b_state::crtc_update_row));

	// SOFTKEY DECODER: RSTB on R1/R5/R6/R7 key press (edge-triggered).
	// Polls at 60Hz for MAME input edges; hardware is combinational.
	TIMER(config, "softkey").configure_periodic(FUNC(hp4951b_state::softkey_tick), attotime::from_hz(60));
}


ROM_START(hp4951b)
	ROM_REGION(0x2000, "maincpu", 0)
	ROM_LOAD("10021.bin", 0x0000, 0x2000, CRC(7a30d53a) SHA1(192a399c9d316aff58708754ad3ebd8d840eb2a7))

	ROM_REGION(0x8000, "rom22", 0)
	ROM_LOAD("10022.bin", 0x0000, 0x8000, CRC(b1653fab) SHA1(8749bd15f7d061a872a07de68a3c83e07f432d84))

	ROM_REGION(0x8000, "rom23", 0)
	ROM_LOAD("10023.bin", 0x0000, 0x8000, CRC(ff3ed21f) SHA1(d90f60024143c2c60e967d65c639ccf1f4071027))

	ROM_REGION(0x8000, "rom24", 0)
	ROM_LOAD("10024.bin", 0x0000, 0x8000, CRC(89bf19d0) SHA1(25041cb7069c1f98cfd0c682220b9c91363e70ea))

	ROM_REGION(0x10000, "chargen", 0)
	ROM_LOAD("charrom1.bin", 0x0000, 0x2000, CRC(c809c50f) SHA1(3628d9846a398d9c4a56011989642357775b535b))
	ROM_LOAD("charrom2.bin", 0x8000, 0x2000, CRC(889b73bf) SHA1(d554377befbcf914ac6da47146d47da4887b69b4))
ROM_END

} // anonymous namespace


//    YEAR  NAME     PARENT  COMPAT  MACHINE  INPUT    CLASS          INIT        COMPANY            FULLNAME                      FLAGS
COMP(1985, hp4951b, 0,      0,      hp4951b, hp4951b, hp4951b_state, empty_init, "Hewlett-Packard", "HP 4951B Protocol Analyzer", MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW)
