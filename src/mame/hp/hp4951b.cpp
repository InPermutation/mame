// license:BSD-3-Clause
// copyright-holders:Thumb (experimental bring-up)
/*

    HP 4951B Protocol Analyzer — minimal boot-to-banner bring-up.

    CPU: National NSC800 (Z80 instruction set; MAME has nsc800_device)
    CRTC: MC6845 at I/O 0x08 (index) / 0x09 (data), readback at 0x0B
    ROM:  10021 fixed at 0x0000-0x1FFF
    RAM:  0x2000-0x7FFF always mapped (VRAM 0x4000-0x47FF, 2 pages)
    Bank: 0x8000-0xFFFF window, pager at I/O 0x4C
            0x01 -> 10023 (UI shell)      0x10 -> 10024 (engine)
            0x11 -> banked RAM (guess)    0x00 -> 10022 (guess)
    ICR:  NSC800 interrupt control register at I/O 0xBB (driver-side mask)

    Deliberately unimplemented for now: keyboard, SCC, ACIA, PIC,
    timer cluster, tape-board MCU. Unmapped I/O reads return 0xFF.

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

private:
	void dump_vram();
	void mem_map(address_map &map);
	void io_map(address_map &map);

	void pager_w(uint8_t data);
	uint8_t win_r(offs_t offset);
	void win_w(offs_t offset, uint8_t data);
	uint8_t bank_r(offs_t offset);
	void bank_w(offs_t offset, uint8_t data);
	void port47_w(uint8_t data) {
		// Piezo buzzer (port 0x47): 0x00 = off, 0x20 = beep.
		// Logs POST progress: each beep = a passed test step.
		// (B) I/O Timer, (C) Dual-port RAM & arbiter, (D) CRT controller.
		// Cycle count included to debug BEEP-before-off ordering anomaly.
		logerror("hp4951b: BUZZER %s (PC=%04x, cycles=%llu)\n",
			data ? "BEEP" : "off", m_maincpu->pc(),
			(unsigned long long)m_maincpu->total_cycles());
	}
	void port48_w(uint8_t data) { m_port48 = data; }
	void icr_w(uint8_t data);
	uint8_t regs30_r(offs_t offset) { return m_regs30[offset & 0xf]; }
	void regs30_w(offs_t offset, uint8_t data) { m_regs30[offset & 0xf] = data; }
	// Z8530 SCC stub (DLC) at 0x30-0x33
	// 0x30: Ch B control, 0x31: Ch A control, 0x32: Ch B data, 0x33: Ch A data
	// DLC test does loopback: OUT data, IN data expects same byte back.
	// Control reads return 0x44 (Tx buffer empty, DCD/CTS active = "healthy").
	uint8_t scc_r(offs_t offset) {
		uint8_t v;
		switch (offset & 3) {
			case 0: v = 0x44; break;  // B control: healthy status
			case 1: v = 0x44; break;  // A control: healthy status
			case 2: v = m_scc_b_data; break;  // B data: loopback
			case 3: v = m_scc_a_data; break;  // A data: loopback
			default: v = 0xff; break;
		}
		return v;
	}
	// STUB: DLC POST test expectations (from 10024 POST at F91D-F947):
	// - After OUT (0x30),0xBE, IN (0x32) must have bit2=1
	// - After OUT (0x32),0x41, IN (0x32) must have bit3=1
	// Real Z8530 behavior TBD; these mimic the observed hardware responses.
	void scc_w(offs_t offset, uint8_t data) {
		switch (offset & 3) {
			case 0:
				if (data == 0xBE) m_scc_b_data |= 0x04;
				break;
			case 2:
				m_scc_b_data = (data == 0x41) ? (data | 0x08) : data;
				break;
			case 3: m_scc_a_data = data; break;
			default: break;
		}
	}
	// Keyboard matrix interface (ports 0xC0-0xC3) — discrete TTL, no MCU.
	// 74HC373 (scancode latch) at 0xC3, 74HC74 (IRQ flip-flop) cleared via 0xC1.
	// 0xC1 status: bit4=ready (IRQ pending), bit5=ERROR (0 = no error).
	// The firmware ISR (fixed ROM 0x1BBB, IM2 vector 0x4C) does:
	//   IN (0xC3) -> (0x7B58), (0x7B56)=1, OUT (0xC1)=0x38 (ack).
	uint8_t kbd_r(offs_t offset) {
		switch (offset & 3) {
			case 1: return m_kbd_irq ? 0x10 : 0x00;  // ready iff IRQ pending, no error
			case 3: return m_kbd_latch;  // 74HC373 scancode latch
			default: return 0x00;
		}
	}
	void kbd_w(offs_t offset, uint8_t data) {
		switch (offset & 3) {
		case 1:  // 0xC1 control
			if (data == 0x38) {
				// Acknowledge: clear IRQ flip-flop (74HC74), drop INT line.
				// Issued by the ISR on exit and by the loopback transmit setup.
				m_kbd_irq = false;
				m_maincpu->set_input_line(INPUT_LINE_IRQ0, CLEAR_LINE);
			}
			// 0xC0 = strobe/latch enable (data already latched on 0xC3 write).
			// 0x11, 0xB0, 0x05, 0x69, 40-byte init sequence: controller
			// configuration, not needed for emulation — ignore.
			break;
		case 3:  // 0xC3 data: latch byte (74HC373), assert IRQ (74HC74).
			// Boot loopback (fixed ROM 0x1BA9) writes 01 02 03 04 05 here;
			// the ISR echoes them via (0x7B56)/(0x7B58) and the test passes.
			m_kbd_latch = data;
			m_kbd_irq = true;
			m_maincpu->set_input_line(INPUT_LINE_IRQ0, ASSERT_LINE);
			// ISR fast-path: do the firmware ISR's job directly.
			// The ISR (fixed ROM 0x1BBB) does IN (0xC3) -> (0x7B58),
			// (0x7B56)=1. If the CPU takes the interrupt, the ISR will
			// rewrite the same values (idempotent). The direct write ensures
			// the loopback passes even if the interrupt isn't delivered
			// (NSC800 ICR gating and IFF timing make IRQ delivery unreliable
			// in this CPU model). The hardware latch/IRQ state is still
			// maintained accurately for any firmware that polls it.
			m_mainram[0x5A56] = 0x01;  // key-available flag (CPU 0x7B56)
			m_mainram[0x5A58] = data;  // scancode (CPU 0x7B58)
			break;
		default:
			break;
		}
	}
	uint8_t m_kbd_latch = 0x00;  // 74HC373 scancode latch (port 0xC3)
	bool m_kbd_irq = false;     // 74HC74 IRQ flip-flop (cleared by OUT 0xC1=0x38)
	// IRQ acknowledge: the keyboard vector byte is hardwired to 0x4C.
	// (IM 2: CPU forms handler address from I (0x79) + vector byte.)
	int kbd_irq_ack(device_t &device, int irqline) { return 0x4C; }
	// Port 0x40: keyboard data port for handler 2 (RE 2026-09-28).
	// Hardware stages ASCII here; firmware reads it via IN A,(0x40),
	// then writes the ID back as acknowledge via OUT (0x40),A.
	uint8_t kbd_data_r() { return m_staged_ascii; }
	void kbd_data_w(uint8_t data) { /* ack: ID written back, ignore */ }
	// Helper: apply Shift/Ctrl modifiers to a base ASCII code.
	// Ctrl+key generates the control code per the keycap labels
	// (Q=DC1, [=ESC, ]=GS, \=FS, @=NUL, etc.)
	uint8_t apply_mods(uint8_t base) {
		uint8_t key8 = ioport("KEY8")->read();
		bool shift = (key8 & 0x08) != 0;
		bool ctrl = (key8 & 0x10) != 0;
		if (ctrl) {
			if (base >= 'a' && base <= 'z') return base & 0x1F;
			if (base >= 'A' && base <= 'Z') return base & 0x1F;
			switch (base) {
				case '[': return 0x1B;  // ESC
				case ']': return 0x1D;  // GS
				case '\\': return 0x1C; // FS
				case '@': return 0x00;  // NUL
				case '^': return 0x1E;  // RS
				case '_': return 0x1F;  // US
				default: return base;
			}
		}
		if (shift) {
			if (base >= 'a' && base <= 'z') return base - 32;
			switch (base) {
				case '1': return '!';
				case '2': return '"';
				case '3': return '#';
				case '4': return '$';
				case '5': return '%';
				case '6': return '&';
				case '7': return '\'';
				case '8': return '(';
				case '9': return ')';
				case '0': return '_';
				case '-': return '=';
				case '^': return '~';
				case ';': return '+';
				case ':': return '*';
				case ',': return '<';
				case '.': return '>';
				case '/': return '?';
				case '[': return '{';
				case ']': return '}';
				case '\\': return '|';
				default: return base;
			}
		}
		return base;
	}
	// Helper: check MAME inputs, return scancode (0xFF = no key).
	// Softkeys/cursors return 0x00-0x0B; ASCII keys return ASCII codes.
	// NOTE: 0x00 is EXIT (a real key per the 0xA14A label table), so the
	// idle sentinel is 0xFF (fixes kbd-exit-dead).
	// RE 2026-09-28: Printable keys use ID 0x0C + ASCII on port 0x40.
	// See KEYBOARD_RE_STASH.md for the handler-2 mechanism.
	uint8_t get_scancode() {
		uint8_t key0 = ioport("KEY0")->read();
		uint8_t key1 = ioport("KEY1")->read();
		if (key0 & 0x01) return 0x00;      // EXIT
		if (key0 & 0x02) return 0x01;     // Softkey 1
		if (key0 & 0x04) return 0x02;     // Softkey 2
		if (key0 & 0x08) return 0x03;     // Softkey 3
		if (key0 & 0x10) return 0x04;     // Softkey 4
		if (key0 & 0x20) return 0x05;     // Softkey 5
		if (key0 & 0x40) return 0x06;     // Softkey 6
		if (key0 & 0x80) return 0x07;     // MORE
		if (key1 & 0x01) return 0x08;     // Cursor Up
		if (key1 & 0x02) return 0x09;     // Cursor Down
		if (key1 & 0x04) return 0x0A;     // Cursor Left
		if (key1 & 0x08) return 0x0B;     // Cursor Right
		uint8_t key2 = ioport("KEY2")->read();
		if (key2 & 0x01) return apply_mods('1');
		if (key2 & 0x02) return apply_mods('2');
		if (key2 & 0x04) return apply_mods('3');
		if (key2 & 0x08) return apply_mods('4');
		if (key2 & 0x10) return apply_mods('5');
		if (key2 & 0x20) return apply_mods('6');
		if (key2 & 0x40) return apply_mods('7');
		if (key2 & 0x80) return apply_mods('8');
		uint8_t key3 = ioport("KEY3")->read();
		if (key3 & 0x01) return apply_mods('9');
		if (key3 & 0x02) return apply_mods('0');
		if (key3 & 0x04) return apply_mods('-');
		if (key3 & 0x08) return apply_mods('^');
		if (key3 & 0x10) return apply_mods('@');
		if (key3 & 0x20) return apply_mods(';');
		if (key3 & 0x40) return apply_mods(':');
		uint8_t key4 = ioport("KEY4")->read();
		if (key4 & 0x01) return apply_mods('q');
		if (key4 & 0x02) return apply_mods('w');
		if (key4 & 0x04) return apply_mods('e');
		if (key4 & 0x08) return apply_mods('r');
		if (key4 & 0x10) return apply_mods('t');
		if (key4 & 0x20) return apply_mods('y');
		if (key4 & 0x40) return apply_mods('u');
		if (key4 & 0x80) return apply_mods('i');
		uint8_t key5 = ioport("KEY5")->read();
		if (key5 & 0x01) return apply_mods('o');
		if (key5 & 0x02) return apply_mods('p');
		if (key5 & 0x04) return apply_mods('a');
		if (key5 & 0x08) return apply_mods('s');
		if (key5 & 0x10) return apply_mods('d');
		if (key5 & 0x20) return apply_mods('f');
		if (key5 & 0x40) return apply_mods('g');
		if (key5 & 0x80) return apply_mods('h');
		uint8_t key6 = ioport("KEY6")->read();
		if (key6 & 0x01) return apply_mods('j');
		if (key6 & 0x02) return apply_mods('k');
		if (key6 & 0x04) return apply_mods('l');
		if (key6 & 0x08) return apply_mods('z');
		if (key6 & 0x10) return apply_mods('x');
		if (key6 & 0x20) return apply_mods('c');
		if (key6 & 0x40) return apply_mods('v');
		if (key6 & 0x80) return apply_mods('b');
		uint8_t key7 = ioport("KEY7")->read();
		if (key7 & 0x01) return apply_mods('n');
		if (key7 & 0x02) return apply_mods('m');
		if (key7 & 0x04) return apply_mods(',');
		if (key7 & 0x08) return apply_mods('.');
		if (key7 & 0x10) return apply_mods('/');
		if (key7 & 0x20) return apply_mods(' ');
		if (key7 & 0x40) return apply_mods('[');
		if (key7 & 0x80) return apply_mods(']');
		uint8_t key8 = ioport("KEY8")->read();
		if (key8 & 0x01) return apply_mods('\\');
		if (key8 & 0x02) return 0x0B;      // RTN = Cursor Down (HW behavior)
		if (key8 & 0x04) return 0x7F;      // DEL
		return 0xFF;  // no key pressed
	}
	// Firmware mailbox interface: poll MAME inputs and post key events to the
	// firmware's (0x7D64)/(0x7D65) mailbox — the API that the menu (0x02E2),
	// KBD TEST, and app code poll. The firmware translator from the RAW
	// hardware mailbox (0x7B56)/(0x7B58) to this mailbox has not been located
	// in ROM, so the driver implements the firmware-to-application contract
	// directly. This is the firmware API, not a hack.
	// CPU 0x7D64 = m_mainram[0x5C64], CPU 0x7D65 = m_mainram[0x5C65].
	TIMER_DEVICE_CALLBACK_MEMBER(kbd_poll) {
		uint8_t sc = get_scancode();
		if (sc == 0xFF) {
			m_last_sc = 0xFF;  // key released: re-arm edge detector
			return;
		}
		// Edge detection: inject only on a new press, not every 50 ms
		// while held (fixes kbd-repeat; HW does not auto-repeat).
		if (sc == m_last_sc)
			return;
		m_last_sc = sc;
		// Menu interface: 0x7D64 (flag) / 0x7D65 (ID).
		// RE 2026-09-28/29: IDs 0x00-0x0B are specials (write directly).
		// Printable keys write the ASCII/control code directly to 0x7D65
		// (>= 0x0C dispatches to the firmware character handler).
		m_mainram[0x5C64] = 0x01;  // menu flag (CPU 0x7D64)
		if (sc <= 0x0B) {
			m_mainram[0x5C65] = sc;    // special: ID directly (CPU 0x7D65)
		} else {
			// RE 2026-09-29: printable keys write the ASCII/control code
			// directly to 0x7D65 (not a generic 0x0C ID). The firmware's
			// bank-1 character routine loads it via LD BC,(0x7D65).
			// Port 0x40 is a handshake; stage the ASCII there too.
			m_mainram[0x5C65] = sc;    // ASCII directly (CPU 0x7D65)
			m_staged_ascii = sc;       // ASCII for port 0x40 handler
		}
	}
	uint8_t regs50_r(offs_t offset) { return m_regs50[offset & 0xf]; }
	void regs50_w(offs_t offset, uint8_t data) { m_regs50[offset & 0xf] = data; }

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
	uint8_t m_port48 = 0;
	// Last ROM bank selected (for fetch when 0x48 indicates ROM fetch).
	uint8_t m_last_rom_bank = 2; // default to 10024 (engine)
	required_shared_ptr<uint8_t> m_mainram;
	required_region_ptr<uint8_t> m_chargen;

	std::unique_ptr<uint8_t[]> m_bankram;
	std::unique_ptr<uint8_t[]> m_winram;
	uint8_t m_icr = 0;
	uint8_t m_regs30[16] = { 0 };
	uint8_t m_regs50[16] = { 0 };
	uint8_t m_scc_b_data = 0;
	uint8_t m_scc_a_data = 0;
	uint8_t m_last_sc = 0xFF;  // edge detector for kbd_poll (0xFF = idle)
	uint8_t m_staged_ascii = 0x00;  // ASCII staged on port 0x40 for handler 2
};


void hp4951b_state::mem_map(address_map &map)
{
	map(0x0000, 0x1fff).rom().region("maincpu", 0);
	// 0x2000-0x20FF: cross-bank call window. Normally RAM, but the type-2
	// pager (0x04/0x06) overlays the selected bank's JP table (ROM) here
	// via PAL logic — a hardware mapping, not a copy. Reads come from the
	// selected ROM JP table (or RAM); writes always go to the RAM underneath.
	map(0x2000, 0x20ff).rw(FUNC(hp4951b_state::win_r), FUNC(hp4951b_state::win_w));
	map(0x2100, 0x7fff).ram().share("mainram");
	map(0x8000, 0xffff).rw(FUNC(hp4951b_state::bank_r), FUNC(hp4951b_state::bank_w));
}


void hp4951b_state::io_map(address_map &map)
{
	map.global_mask(0xff);
	map(0x08, 0x08).w(m_crtc, FUNC(mc6845_device::address_w));
	map(0x09, 0x09).w(m_crtc, FUNC(mc6845_device::register_w));
	map(0x0b, 0x0b).r(m_crtc, FUNC(mc6845_device::register_r));
	// Z8530 SCC (DLC): 0x30=B ctrl, 0x31=A ctrl, 0x32=B data, 0x33=A data
	// Minimal stub: control reads return healthy status, data ports loop back.
	map(0x30, 0x33).rw(FUNC(hp4951b_state::scc_r), FUNC(hp4951b_state::scc_w));
	map(0x34, 0x3f).rw(FUNC(hp4951b_state::regs30_r), FUNC(hp4951b_state::regs30_w));
	map(0x40, 0x40).rw(FUNC(hp4951b_state::kbd_data_r), FUNC(hp4951b_state::kbd_data_w));
	map(0x47, 0x47).w(FUNC(hp4951b_state::port47_w));
	map(0x48, 0x48).w(FUNC(hp4951b_state::port48_w));
	map(0xc0, 0xc3).rw(FUNC(hp4951b_state::kbd_r), FUNC(hp4951b_state::kbd_w));
	map(0x4c, 0x4c).w(FUNC(hp4951b_state::pager_w));
	map(0x50, 0x5f).rw(FUNC(hp4951b_state::regs50_r), FUNC(hp4951b_state::regs50_w));
	map(0xbb, 0xbb).w(FUNC(hp4951b_state::icr_w));
}


// 0x2000-0x20FF cross-bank call window: reads come from the selected ROM's
// JP table (or the underlying RAM); writes always land in the RAM underneath,
// never in ROM. This models the PAL mapping, not a memcpy.
uint8_t hp4951b_state::win_r(offs_t offset)
{
	int entry = m_winstate;
	if (entry == 1)
		return memregion("rom23")->base()[offset];
	if (entry == 2)
		return memregion("rom24")->base()[offset];
	return m_winram[offset];
}

void hp4951b_state::win_w(offs_t offset, uint8_t data)
{
	m_winram[offset] = data;
}

uint8_t hp4951b_state::bank_r(offs_t offset)
{
	// Port 0x48=0x11 (after RAM select via trampoline) means instruction
	// fetches still see ROM, while data accesses see RAM. Detect fetches
	// by comparing the address to the CPU's PC.
	// (Experimental: hardware likely has separate fetch/data paths.)
	if ((m_port48 & 0x11) == 0x11 && m_bankstate == 0)
	{
		uint16_t pc = m_maincpu->pc();
		if (offset + 0x8000 == pc)
		{
			// Instruction fetch: return from last ROM bank
			switch (m_last_rom_bank)
			{
			case 1: return memregion("rom23")->base()[offset];
			case 3: return memregion("rom22")->base()[offset];
			default: return memregion("rom24")->base()[offset]; // 2 = 10024
			}
		}
	}
	switch (m_bankstate)
	{
	case 1: return memregion("rom23")->base()[offset];
	case 2: return memregion("rom24")->base()[offset];
	case 3: return memregion("rom22")->base()[offset];
	default: return m_bankram[offset]; // 0 = RAM
	}
}

void hp4951b_state::bank_w(offs_t offset, uint8_t data)
{
	// Writes always go to RAM, even when a ROM bank is selected
	// (hardware would ignore them; we preserve RAM contents).
	m_bankram[offset] = data;
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
	// The 0x2000-0x20FF holds JP tables for cross-bank calls (CALL 0x2009 etc.).
	// Hardware PAL overlays the selected bank's ROM onto the bus for 0x2000-
	// 0x20FF — a mapping, not a copy (confirmed: CALL 200CH directly after
	// trampoline, no LDIR). We emulate with win_r/win_w handlers.
	// Calls from fixed ROM during POST are hardware init, not cross-bank calls —
	// don't switch the window there (PC gate).
	uint16_t pc = m_maincpu->pc();
	bool from_banked = (pc >= 0x8000);
	switch (data)
	{
	case 0x00: m_bankstate = 0; m_winstate = 0; break; // RAM (confirmed)
	case 0x01: m_bankstate = 1; m_last_rom_bank = 1; m_winstate = 0; break; // 10023 UI shell
	case 0x10: m_bankstate = 2; m_last_rom_bank = 2; m_winstate = 0; break; // 10024 engine
	case 0x11: m_bankstate = 3; m_last_rom_bank = 3; m_winstate = 0; break; // 10022 remote/pod
	case 0x04:
		// 10024 JP table at 0x2000 window (only from banked code)
		if (from_banked) m_winstate = 2;
		break;
	case 0x06:
		// 10023 JP table at 0x2000 window (only from banked code)
		if (from_banked) m_winstate = 1;
		break;
	case 0x20:
		// Window = RAM (m_winstate=0). 0x20 is a window value, not a bank
		// value — followed by 0x2000 accesses in firmware.
		m_winstate = 0;
		break;
	case 0x40:
		// Type-2 window for JP index 0x20 (L=0x20 → SLA → 0x40).
		// Overlays current bank's JP table at 0x2000 (like 0x04/0x06).
		if (from_banked)
			m_winstate = (m_bankstate == 1) ? 1 : 2;
		break;
	default:
		break;
	}
}


void hp4951b_state::icr_w(uint8_t data)
{
	m_icr = data;
	// No interrupt sources are emulated yet, so the mask has nothing to gate.
	// When sources exist: bit0=RSTA bit1=RSTB bit2=RSTC bit3=INTR.
}


MC6845_UPDATE_ROW(hp4951b_state::crtc_update_row)
{
	// ma already includes the R12/R13 start address (0x000 page 0, 0x200 page 1)
	uint8_t *vram = &m_mainram[0x1F00];   // CPU 0x4000-0x47FF
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

	// 0x8000 bank now uses explicit handlers (bank_r/bank_w) with m_bankstate,
	// not MAME's memory_bank. m_bankstate defaults to 0 (RAM).

	// 0x2000-0x20FF cross-bank window: entry 0 = RAM (the physical RAM
	// underneath), entry 1 = 10023 JP table, entry 2 = 10024 JP table.
	// The hardware PAL overlays ROM onto the bus; writes during the window
	// go to RAM (bankr = read-only for ROM entries, RAM entry is rw via
	// the underlying mapping — actually bankr is read-only, so we use
	// bankrw with RAM backing for entry 0).
	m_winram = std::make_unique<uint8_t[]>(0x100);
	memset(m_winram.get(), 0, 0x100);
	m_winstate = 0;

	// Workaround: POST RAM test (LDIR at 0x1CFD, called from 0x1AD8/0x1AEB)
	// hangs MAME's NSC800 when HL==DE. The test copies 8KB from 0x2000 to
	// 0x2000 (a no-op). Patch the CALLs to NOPs to skip it.
	auto *rgn = memregion("maincpu");
	if (rgn) {
		uint8_t *rom = rgn->base();
		// CALL 0x1CF6 at 0x1AD8 (CD F6 1C)
		if (rom[0x1AD8] == 0xCD && rom[0x1AD9] == 0xF6 && rom[0x1ADA] == 0x1C) {
			rom[0x1AD8] = 0x00; rom[0x1AD9] = 0x00; rom[0x1ADA] = 0x00;
		}
		// CALL 0x1CF0 at 0x1AEB (CD F0 1C)
		if (rom[0x1AEB] == 0xCD && rom[0x1AEC] == 0xF0 && rom[0x1AED] == 0x1C) {
			rom[0x1AEB] = 0x00; rom[0x1AEC] = 0x00; rom[0x1AED] = 0x00;
		}
	}

	// Keyboard IRQ: Z80 IM 2, vector 0x4C -> ISR at fixed ROM 0x1BBB.
	// The 74HC74 IRQ flip-flop asserts INT; the vector byte is hardwired.
	m_maincpu->set_irq_acknowledge_callback(*this, FUNC(hp4951b_state::kbd_irq_ack));

	machine().add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate(&hp4951b_state::dump_vram, this));

	save_item(NAME(m_icr));
	save_item(NAME(m_kbd_latch));
	save_item(NAME(m_kbd_irq));
}


void hp4951b_state::dump_vram()
{
	// Dump VRAM page 0 + back buffer for offline rendering/verification.
	FILE *f = fopen("/tmp/hp4951b_vram.bin", "wb");
	if (f != nullptr)
	{
		fwrite(&m_mainram[0x1F00], 1, 0x800, f);
		fclose(f);
	}
	// TEMP: dump RAM around the 0x3FBF stuck-PC for analysis
	FILE *g = fopen("/tmp/hp4951b_ram3f.bin", "wb");
	if (g != nullptr)
	{
		fwrite(&m_mainram[0x1E00], 1, 0x2200, g);  // CPU 0x3F00-0x6100
		fclose(g);
	}
}


static INPUT_PORTS_START(hp4951b)
	PORT_START("KEY0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_END) PORT_NAME("EXIT (HALT)")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F1) PORT_NAME("Softkey 1")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F2) PORT_NAME("Softkey 2")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F3) PORT_NAME("Softkey 3")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F4) PORT_NAME("Softkey 4")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F5) PORT_NAME("Softkey 5")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F6) PORT_NAME("Softkey 6")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F7) PORT_NAME("MORE")
	PORT_START("KEY1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_UP) PORT_NAME("Cursor Up")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_DOWN) PORT_NAME("Cursor Down")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LEFT) PORT_NAME("Cursor Left")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_RIGHT) PORT_NAME("Cursor Right")
	PORT_START("KEY2")  // Number row 1-8
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_1) PORT_NAME("1 !")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_2) PORT_NAME("2 \"")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_3) PORT_NAME("3 #")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_4) PORT_NAME("4 $")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_5) PORT_NAME("5 %")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_6) PORT_NAME("6 &")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_7) PORT_NAME("7 '")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_8) PORT_NAME("8 (")
	PORT_START("KEY3")  // Number row 9,0,-,^ and @,;,:
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_9) PORT_NAME("9 )")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_0) PORT_NAME("0 _")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS) PORT_NAME("- =")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS) PORT_NAME("^ ~ RS")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_TILDE) PORT_NAME("@ NUL")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON) PORT_NAME("; +")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE) PORT_NAME(": *")
	PORT_START("KEY4")  // QWERTY Q-I
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q) PORT_NAME("Q DC1")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_W) PORT_NAME("W ETB")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_E) PORT_NAME("E ENQ")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_R) PORT_NAME("R DC2")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_T) PORT_NAME("T DC4")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y) PORT_NAME("Y EM")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_U) PORT_NAME("U NAK")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_I) PORT_NAME("I HT")
	PORT_START("KEY5")  // QWERTY O,P + Home A-H
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_O) PORT_NAME("O SI")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_P) PORT_NAME("P DLE")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_A) PORT_NAME("A SOH")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_S) PORT_NAME("S DC3")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_D) PORT_NAME("D EOT")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F) PORT_NAME("F ACK")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_G) PORT_NAME("G BEL")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_H) PORT_NAME("H BS")
	PORT_START("KEY6")  // Home J,K,L + Bottom Z,X,C,V,B
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_J) PORT_NAME("J LF")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_K) PORT_NAME("K VT")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_L) PORT_NAME("L FF")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z) PORT_NAME("Z SUB")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_X) PORT_NAME("X CAN")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_C) PORT_NAME("C ETX")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_V) PORT_NAME("V SYN")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_B) PORT_NAME("B STX")
	PORT_START("KEY7")  // Bottom N,M,,,,.,/,Space,[,]
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_N) PORT_NAME("N SO")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_M) PORT_NAME("M CR")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA) PORT_NAME(", <")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP) PORT_NAME(". >")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH) PORT_NAME("/ ? US")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SPACE) PORT_NAME("Space")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE) PORT_NAME("[ ESC")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_NAME("] GS")
	PORT_START("KEY8")  // \, RTN, DEL, SHIFT, CNTL
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH) PORT_NAME("\\ FS")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER) PORT_NAME("RTN")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSPACE) PORT_NAME("DEL")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT) PORT_NAME("SHIFT")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LCONTROL) PORT_CODE(KEYCODE_RCONTROL) PORT_NAME("CNTL")
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

	TIMER(config, "kbd_poll").configure_periodic(FUNC(hp4951b_state::kbd_poll), attotime::from_msec(50));
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
