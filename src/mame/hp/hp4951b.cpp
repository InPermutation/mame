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
		m_bank(*this, "bank"),
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
	void port48_w(uint8_t data) { /* ROM shadow mask, ignore for now */ }
	void icr_w(uint8_t data);
	uint8_t regs30_r(offs_t offset) { return m_regs30[offset & 0xf]; }
	void regs30_w(offs_t offset, uint8_t data) { m_regs30[offset & 0xf] = data; }
	uint8_t regsc0_r(offs_t offset) { return m_regsc0[offset & 3]; }
	void regsc0_w(offs_t offset, uint8_t data) { m_regsc0[offset & 3] = data; }
	uint8_t regs50_r(offs_t offset) { return m_regs50[offset & 0xf]; }
	void regs50_w(offs_t offset, uint8_t data) { m_regs50[offset & 0xf] = data; }

	MC6845_UPDATE_ROW(crtc_update_row);

	required_device<nsc800_device> m_maincpu;
	required_device<mc6845_device> m_crtc;
	required_device<screen_device> m_screen;
	required_memory_bank m_bank;
	required_shared_ptr<uint8_t> m_mainram;
	required_region_ptr<uint8_t> m_chargen;

	std::unique_ptr<uint8_t[]> m_bankram;
	uint8_t m_icr = 0;
	uint8_t m_regs30[16] = { 0 };
	uint8_t m_regs50[16] = { 0 };
	uint8_t m_regsc0[16] = { 0xff, 0xff, 0xff, 0xff };
};


void hp4951b_state::mem_map(address_map &map)
{
	map(0x0000, 0x1fff).rom().region("maincpu", 0);
	map(0x2000, 0x7fff).ram().share("mainram");
	map(0x8000, 0xffff).bankrw("bank");
}


void hp4951b_state::io_map(address_map &map)
{
	map.global_mask(0xff);
	map(0x08, 0x08).w(m_crtc, FUNC(mc6845_device::address_w));
	map(0x09, 0x09).w(m_crtc, FUNC(mc6845_device::register_w));
	map(0x0b, 0x0b).r(m_crtc, FUNC(mc6845_device::register_r));
	map(0x30, 0x3f).rw(FUNC(hp4951b_state::regs30_r), FUNC(hp4951b_state::regs30_w));
	map(0x48, 0x48).w(FUNC(hp4951b_state::port48_w));
	map(0xc0, 0xc3).rw(FUNC(hp4951b_state::regsc0_r), FUNC(hp4951b_state::regsc0_w));
	map(0x4c, 0x4c).w(FUNC(hp4951b_state::pager_w));
	map(0x50, 0x5f).rw(FUNC(hp4951b_state::regs50_r), FUNC(hp4951b_state::regs50_w));
	map(0xbb, 0xbb).w(FUNC(hp4951b_state::icr_w));
}


void hp4951b_state::pager_w(uint8_t data)
{
	// Type-1 trampoline table (@0xBD in fixed ROM):
	//   bank0 -> 0x11, bank1 -> 0x01, bank2 -> 0x10, bank3 -> 0x00
	// bank1 = 10023, bank2 = 10024 (jump-table analysis); bank0/bank3 guessed.
	logerror("hp4951b: pager byte 0x%02x (PC=%04x)\n", data, m_maincpu->pc());
	switch (data)
	{
	case 0x11: m_bank->set_entry(0); break; // banked RAM (guess)
	case 0x01: m_bank->set_entry(1); break; // 10023 UI shell
	case 0x10: m_bank->set_entry(2); break; // 10024 engine
	case 0x00: m_bank->set_entry(3); break; // 10022 remote/pod (guess)
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
	uint8_t *vram = &m_mainram[0x2000];   // CPU 0x4000-0x47FF
	uint8_t *cg = &m_chargen[0x4000];     // CHAR ROM 1, standard bank
	uint32_t *p = &bitmap.pix(y);

	const uint32_t fg = rgb_t(0x33, 0xff, 0x66);
	const uint32_t bg = rgb_t(0x00, 0x10, 0x08);

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
		// uint8_t attr = vram[addr * 2 + 1]; // TODO: attribute semantics
		uint8_t row = cg[ch * 16 + ra];

		// v1 approximation: rows 1 and 12 are attribute rows, blanked
		// unless overline/underline is enabled (matches the font tester).
		if (ra == 1 || ra == 12)
			row = 0;

		for (int b = 0; b < 8; b++)
			px[b] = (row & (0x80 >> b)) ? fg : bg;
	}
}


void hp4951b_state::machine_start()
{
	m_bankram = std::make_unique<uint8_t[]>(0x8000);
	memset(m_bankram.get(), 0, 0x8000);

	m_bank->configure_entry(0, m_bankram.get());
	m_bank->configure_entry(1, memregion("rom23")->base());
	m_bank->configure_entry(2, memregion("rom24")->base());
	m_bank->configure_entry(3, memregion("rom22")->base());
	m_bank->set_entry(0);

	machine().add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate(&hp4951b_state::dump_vram, this));

	save_item(NAME(m_icr));
}


void hp4951b_state::dump_vram()
{
	// Dump VRAM page 0 + back buffer for offline rendering/verification.
	FILE *f = fopen("/tmp/hp4951b_vram.bin", "wb");
	if (f != nullptr)
	{
		fwrite(&m_mainram[0x2000], 1, 0x800, f);
		fclose(f);
	}
}


static INPUT_PORTS_START(hp4951b)
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

	ROM_REGION(0x8000, "chargen", 0)
	ROM_LOAD("charrom1.bin", 0x0000, 0x8000, CRC(b78155f0) SHA1(97bfe16fd1130c0b50c5d6b3136a2011b79016ad))
ROM_END

} // anonymous namespace


//    YEAR  NAME     PARENT  COMPAT  MACHINE  INPUT    CLASS          INIT        COMPANY            FULLNAME                      FLAGS
COMP(1985, hp4951b, 0,      0,      hp4951b, hp4951b, hp4951b_state, empty_init, "Hewlett-Packard", "HP 4951B Protocol Analyzer", MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW)
