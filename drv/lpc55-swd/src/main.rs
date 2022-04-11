// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use drv_lpc55_spi as spi_core;
use drv_lpc55_syscon_api::{Peripheral, Syscon};
use drv_sp_ctrl_api::SpCtrlError;
use idol_runtime::{
    LeaseBufReader, LeaseBufWriter, Leased, LenLimit, RequestError, R, W,
};
use lpc55_pac as device;
use ringbuf::*;
use userlib::*;

#[derive(Copy, Clone, PartialEq)]
enum Trace {
    Idcode(u32),
    Idr(u32),
    MemVal(u32),
    SwdRead(Port, RawSwdReg),
    SwdWrite(Port, RawSwdReg, u32),
    ReadCmd,
    WriteCmd,
    None,
    AckErr(Ack),
}

ringbuf!(Trace, 64, Trace::None);

task_slot!(SYSCON, syscon_driver);
task_slot!(GPIO, gpio_driver);

#[derive(Copy, Clone, PartialEq)]
enum Ack {
    //Ok,
    Wait,
    Fault,
    Protocol,
}

// Several of these bits in the CSW are  implementation defined according to
// the official specification but thankfully RM0433 60.4.2 actually gives us the
// defintions

// Full 32-bit word transfer
const CSW_SIZE32: u32 = 0x00000002;
// Increment by size bytes in the transaction
const CSW_SADDRINC: u32 = 0x00000010;
// AP access enabled
const CSW_DBGSTAT: u32 = 0x00000040;
// Privileged + data access
const CSW_HPROT: u32 = 0x03000000;

const DP_CTRL_CDBGPWRUPREQ: u32 = 1 << 28;
const DP_CTRL_CDBGPWRUPACK: u32 = 1 << 29;

// See Ch5 of ARM ADI for bit pattern
const START_BIT: u8 = 7;
// Stop is bit 1 and always 0
const PARITY_BIT: u8 = 2;
const ADDR_BITS: u8 = 3;

const RDWR_BIT: u8 = 5;
const APDP_BIT: u8 = 6;
const PARK_BIT: u8 = 0;

const START_VAL: u8 = 1 << START_BIT;
const PARK_VAL: u8 = 1 << PARK_BIT;

#[derive(Copy, Clone, PartialEq)]
enum Port {
    DP = 0,
    AP = 1,
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq)]
enum DpRead {
    IDCode = 0x0,
    Ctrl = 0x4,
    //Resend = 0x8,
    //Rdbuf = 0xc,
}

impl DpRead {
    fn addr_bits(&self) -> u8 {
        // we need this to be as bits [2:3]
        match *self {
            DpRead::IDCode => 0b00,
            DpRead::Ctrl => 0b10,
        }
    }
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq)]
enum DpWrite {
    Abort = 0x0,
    Ctrl = 0x4,
    Select = 0x8,
}

impl DpWrite {
    fn addr_bits(&self) -> u8 {
        // we need this to be as bits [2:3]
        match *self {
            DpWrite::Abort => 0b00,
            DpWrite::Ctrl => 0b10,
            DpWrite::Select => 0b01,
        }
    }
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq)]
enum RawSwdReg {
    DpRead(DpRead),
    DpWrite(DpWrite),
    ApRead(ApReg),
    ApWrite(ApReg),
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq)]
enum ApReg {
    CSW = 0x0,
    TAR = 0x4,
    DRW = 0xC,
    //BD0 = 0x10,
    //BD1 = 0x14,
    //BD2 = 0x18,
    //BD3 = 0x1C,
    //ROM = 0xF8,
    IDR = 0xFC,
}

impl ApReg {
    fn addr_bits(&self) -> u8 {
        // we need this to be as bits [2:3]
        match *self {
            ApReg::CSW => 0b00,
            ApReg::TAR => 0b10,
            ApReg::DRW => 0b11,

            ApReg::IDR => 0b11,
        }
    }
}

// represents the port + register
struct ApAddr(u32, ApReg);

fn get_addr_and_rw(reg: RawSwdReg) -> (u8, u8) {
    match reg {
        RawSwdReg::DpRead(v) => (1 << RDWR_BIT, v.addr_bits() << ADDR_BITS),
        RawSwdReg::DpWrite(v) => (0 << RDWR_BIT, v.addr_bits() << ADDR_BITS),
        RawSwdReg::ApRead(v) => (1 << RDWR_BIT, v.addr_bits() << ADDR_BITS),
        RawSwdReg::ApWrite(v) => (0 << RDWR_BIT, v.addr_bits() << ADDR_BITS),
    }
}

// The parity is only over 4 of the bits
fn calc_parity(val: u8) -> u8 {
    let b = val >> 3 & 0xf;

    if b.count_ones() % 2 == 0 {
        0 << PARITY_BIT
    } else {
        1 << PARITY_BIT
    }
}

struct ServerImpl {
    spi: spi_core::Spi,
    init: bool,
}

impl idl::InOrderSpCtrlImpl for ServerImpl {
    fn read(
        &mut self,
        _: &RecvMessage,
        addr: u32,
        dest: LenLimit<Leased<W, [u8]>, 4096>,
    ) -> Result<(), RequestError<SpCtrlError>> {
        ringbuf_entry!(Trace::ReadCmd);
        if !self.init {
            return Err(SpCtrlError::NeedInit.into());
        }
        let cnt = dest.len();
        if cnt % 4 != 0 {
            return Err(SpCtrlError::BadLen.into());
        }
        let mut buf = LeaseBufWriter::<_, 32>::from(dest.into_inner());

        for i in 0..cnt / 4 {
            match self.read_target_addr(addr + ((i * 4) as u32)) {
                Ok(r) => {
                    ringbuf_entry!(Trace::MemVal(r));
                    for b in r.to_le_bytes() {
                        if buf.write(b).is_err() {
                            return Ok(());
                        }
                    }
                }
                Err(_) => return Err(SpCtrlError::Fault.into()),
            }
        }

        Ok(())
    }

    fn write(
        &mut self,
        _: &RecvMessage,
        addr: u32,
        dest: LenLimit<Leased<R, [u8]>, 4096>,
    ) -> Result<(), RequestError<SpCtrlError>> {
        ringbuf_entry!(Trace::WriteCmd);
        if !self.init {
            return Err(SpCtrlError::NeedInit.into());
        }
        let cnt = dest.len();
        if cnt % 4 != 0 {
            return Err(SpCtrlError::BadLen.into());
        }
        let mut buf = LeaseBufReader::<_, 32>::from(dest.into_inner());

        for i in 0..cnt / 4 {
            let mut word: [u8; 4] = [0; 4];
            for j in 0..4 {
                match buf.read() {
                    Some(b) => word[j] = b,
                    None => return Ok(()),
                };
            }
            match self.write_target_addr(
                addr + ((i * 4) as u32),
                u32::from_le_bytes(word),
            ) {
                Err(_) => return Err(SpCtrlError::Fault.into()),
                _ => (),
            }
        }

        Ok(())
    }

    fn setup(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError<SpCtrlError>> {
        match self.swd_setup() {
            Ok(_) => {
                self.init = true;
                Ok(())
            }
            Err(_) => Err(SpCtrlError::Fault.into()),
        }
    }
}

impl ServerImpl {
    fn io_out(&mut self) {
        while !self.spi.mstidle() {
            cortex_m::asm::nop();
        }

        let gpio_driver = GPIO.get_task_id();
        switch_io_out(gpio_driver);
    }

    fn io_in(&mut self) {
        while !self.spi.mstidle() {
            cortex_m::asm::nop();
        }

        let gpio_driver = GPIO.get_task_id();
        switch_io_in(gpio_driver);
    }

    fn read_ack(&mut self) -> Result<(), Ack> {
        // This read includes the turnaround bit which we
        // don't care about.
        let b = self.read_nibble();

        // We configured the SPI controller to give us back 4 bits,
        // if we got more than that something has gone very wrong
        if b & 0xF0 != 0 {
            ringbuf_entry!(Trace::AckErr(Ack::Protocol));
            return Err(Ack::Protocol);
        }

        // Section 5.3 of ADIv5 describes the bit patterns
        match b & 0x7 {
            0b001 => {
                ringbuf_entry!(Trace::AckErr(Ack::Fault));
                Err(Ack::Fault)
            }
            0b010 => {
                ringbuf_entry!(Trace::AckErr(Ack::Wait));
                Err(Ack::Wait)
            }
            0b100 => Ok(()),
            _ => {
                ringbuf_entry!(Trace::AckErr(Ack::Protocol));
                Err(Ack::Protocol)
            }
        }
    }

    fn tx_byte(&mut self, byte: u8) {
        while !self.spi.can_tx() {}
        self.spi.send_u8_no_rx(byte);
    }

    // SW-DP is intended to be used as a bit based protocol.
    // The smallest unit the SPI controller can do is 4 bits
    fn read_nibble(&mut self) -> u8 {
        while !self.spi.can_tx() {}
        self.spi.send_raw_data(0x0, true, false, 4);
        while !self.spi.has_byte() {}
        self.spi.read_u8()
    }

    fn read_byte(&mut self) -> u8 {
        while !self.spi.can_tx() {}
        self.spi.send_raw_data(0x0, true, false, 8);
        while !self.spi.has_byte() {}
        self.spi.read_u8()
    }

    fn read_nine_bits(&mut self) -> u16 {
        while !self.spi.can_tx() {}
        self.spi.send_raw_data(0x0, true, false, 9);
        while !self.spi.has_byte() {}
        self.spi.read_u16()
    }

    fn swd_transfer_cmd(
        &mut self,
        port: Port,
        reg: RawSwdReg,
    ) -> Result<(), Ack> {
        self.io_out();

        // has our start and stop bits set
        let mut byte: u8 = START_VAL | PARK_VAL;

        let (rd, abits) = get_addr_and_rw(reg);

        let port_bit: u8 = match port {
            Port::DP => (0 << APDP_BIT),
            Port::AP => (1 << APDP_BIT),
        };

        byte |= abits | rd | port_bit;

        let p = calc_parity(byte);

        byte |= p;

        self.tx_byte(byte);

        self.io_in();

        self.read_ack()
    }

    fn reset(&mut self) {
        // Spec says hold high for 51 clock cycles, more is okay
        // this gives us 56
        for _ in 0..7 {
            self.tx_byte(0xff);
        }
    }

    fn idle_cycles(&mut self, cnt: usize) {
        // Transmitting one bit = one idle cycle, convert bytes to bits
        // for the correct count.
        //
        // Round up here just to be safe
        let rounded = ((cnt + 7) / 8) * 8;
        for _ in 0..(rounded / 8) {
            self.tx_byte(0x00);
        }
    }

    fn swd_switch(&mut self) {
        // Section B5.2.2 of ADIv6 specifies this
        const JTAG_MAGIC: u16 = 0xe79e;

        self.spi.send_raw_data(JTAG_MAGIC, true, true, 16);
    }

    fn read_word(&mut self) -> u32 {
        let mut result: u32 = 0;

        self.io_in();

        // We need to read exactly 33 bits. We have MOSI disabled so trying to
        // read more results in protocol errors because we can't appropriately
        // drive the line low to treat it as extra idle cycles.
        for i in 0..4 {
            let b = if i == 3 {
                // The last read is 9 bits. Right now we just shift the parity bit
                // away because it's not clear what the appropriate response is if
                // we detect a parity error. "Might have to re-issue original read
                // request or use the RESEND register if a parity or protocol fault"
                // doesn't give much of a hint...
                ((self.read_nine_bits() >> 1).reverse_bits() >> 8) as u32
            } else {
                (self.read_byte().reverse_bits()) as u32
            };
            result |= b << (i * 8);
        }

        return result;
    }

    fn write_word(&mut self, val: u32) {
        let parity: u32 = if val.count_ones() % 2 == 0 { 0 } else { 1 };

        let rev = val.reverse_bits();

        let first: u16 = (rev >> 24 & 0xFF) as u16;
        let second: u16 = (rev >> 16 & 0xFF) as u16;
        let third: u16 = (rev >> 8 & 0xFF) as u16;
        let fourth: u16 = (((rev & 0xFF) << 1) | parity) as u16;

        // We're going to transmit 34 bits: one bit of turnaround (i.e.
        // don't care), 32 bits of data and one bit of parity.
        // Break this up by transmitting 9 bits (turnaround + first byte)
        // 8 bits, 8 bits, 9 bits (last byte + parity)

        self.spi.send_raw_data(first, true, true, 9);
        self.spi.send_raw_data(second, true, true, 8);
        self.spi.send_raw_data(third, true, true, 8);
        self.spi.send_raw_data(fourth, true, true, 9);
    }

    fn swd_read(&mut self, port: Port, reg: RawSwdReg) -> Result<u32, Ack> {
        ringbuf_entry!(Trace::SwdRead(port, reg));
        loop {
            let result = self.swd_transfer_cmd(port, reg);

            match result {
                Ok(_) => (),
                Err(e) => {
                    // Need to handle the turnaround bit
                    self.io_out();
                    self.idle_cycles(8);
                    match e {
                        Ack::Wait => continue,
                        _ => return Err(e),
                    }
                }
            }

            let ret = self.read_word();

            self.io_out();

            self.swd_finish();

            return Ok(ret);
        }
    }

    fn swd_setup(&mut self) -> Result<(), Ack> {
        self.io_out();
        // Section B5.2.2 of ADIv6 specifies this sequence
        self.reset();
        self.swd_switch();
        self.reset();

        self.idle_cycles(16);

        // Must read DP IDCODE register after reset
        let result =
            self.swd_read(Port::DP, RawSwdReg::DpRead(DpRead::IDCode))?;

        ringbuf_entry!(Trace::Idcode(result));

        self.power_up()?;

        // Read the IDR as a basic test for reading from the AP
        let result = self.swd_read_ap_reg(ApAddr(0, ApReg::IDR))?;
        ringbuf_entry!(Trace::Idr(result));

        Ok(())
    }

    fn swd_finish(&mut self) {
        // Allow some idle cycles
        self.idle_cycles(8);
    }

    fn swd_write(
        &mut self,
        port: Port,
        reg: RawSwdReg,
        val: u32,
    ) -> Result<(), Ack> {
        ringbuf_entry!(Trace::SwdWrite(port, reg, val));
        loop {
            let result = self.swd_transfer_cmd(port, reg);

            match result {
                Err(e) => {
                    // Need to account for the turnaround bit before continuing
                    self.io_out();
                    self.idle_cycles(8);
                    match e {
                        Ack::Wait => continue,
                        _ => return Err(e),
                    }
                }
                _ => (),
            }

            self.io_out();
            self.write_word(val);
            self.swd_finish();
            return Ok(());
        }
    }

    fn swd_write_ap_reg(&mut self, addr: ApAddr, val: u32) -> Result<(), Ack> {
        let ap_sel = addr.0 << 24;
        let bank_sel = (addr.1 as u32) & 0xF0;

        self.swd_write(
            Port::DP,
            RawSwdReg::DpWrite(DpWrite::Select),
            ap_sel | bank_sel,
        )?;

        self.swd_write(Port::AP, RawSwdReg::ApWrite(addr.1), val)
    }
    fn swd_read_ap_reg(&mut self, addr: ApAddr) -> Result<u32, Ack> {
        let ap_sel = addr.0 << 24;
        let bank_sel = (addr.1 as u32) & 0xF0;

        self.swd_write(
            Port::DP,
            RawSwdReg::DpWrite(DpWrite::Select),
            ap_sel | bank_sel,
        )?;

        // See section 6.2.5 ADIV5
        // If you require the value from an AP register read, that read must be
        // followed by one of:
        // - A second AP register read, with the appropriate AP selected as the
        //   current AP.
        // - A read of the DP Read Buffer
        let _ = self.swd_read(Port::AP, RawSwdReg::ApRead(addr.1))?;

        let val = self.swd_read(Port::AP, RawSwdReg::ApRead(addr.1))?;

        Ok(val)
    }

    fn read_target_addr(&mut self, addr: u32) -> Result<u32, Ack> {
        self.clear_errors()?;
        self.swd_write_ap_reg(
            ApAddr(0, ApReg::CSW),
            CSW_HPROT | CSW_DBGSTAT | CSW_SADDRINC | CSW_SIZE32,
        )?;

        self.swd_write_ap_reg(ApAddr(0, ApReg::TAR), addr)?;

        let val = self.swd_read_ap_reg(ApAddr(0, ApReg::DRW))?;

        Ok(val)
    }

    fn write_target_addr(&mut self, addr: u32, val: u32) -> Result<(), Ack> {
        self.clear_errors()?;

        self.swd_write_ap_reg(
            ApAddr(0, ApReg::CSW),
            CSW_HPROT | CSW_DBGSTAT | CSW_SADDRINC | CSW_SIZE32,
        )?;

        self.swd_write_ap_reg(ApAddr(0, ApReg::TAR), addr)?;

        self.swd_write_ap_reg(ApAddr(0, ApReg::DRW), val)?;

        Ok(())
    }

    fn clear_errors(&mut self) -> Result<(), Ack> {
        self.swd_write(Port::DP, RawSwdReg::DpWrite(DpWrite::Abort), 0x1F)
    }

    fn power_up(&mut self) -> Result<(), Ack> {
        self.clear_errors()?;
        self.swd_write(Port::DP, RawSwdReg::DpWrite(DpWrite::Select), 0x0)?;
        self.swd_write(
            Port::DP,
            RawSwdReg::DpWrite(DpWrite::Ctrl),
            DP_CTRL_CDBGPWRUPREQ,
        )?;

        loop {
            let r = self.swd_read(Port::DP, RawSwdReg::DpRead(DpRead::Ctrl))?;
            if r & DP_CTRL_CDBGPWRUPACK == DP_CTRL_CDBGPWRUPACK {
                break;
            }
        }

        Ok(())
    }
}

#[export_name = "main"]
fn main() -> ! {
    let syscon = SYSCON.get_task_id();

    let gpio_driver = GPIO.get_task_id();

    setup_pins(gpio_driver).unwrap_lite();

    let mut spi = setup_spi(syscon);

    // This should correspond to SPI mode 0
    spi.initialize(
        device::spi0::cfg::MASTER_A::MASTER_MODE,
        device::spi0::cfg::LSBF_A::STANDARD, // MSB First
        device::spi0::cfg::CPHA_A::CHANGE,
        device::spi0::cfg::CPOL_A::LOW,
        spi_core::TxLvl::Tx7Items,
        spi_core::RxLvl::Rx1Item,
    );

    spi.enable();

    let mut server = ServerImpl { spi, init: false };

    let mut incoming = [0; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use drv_sp_ctrl_api::SpCtrlError;

    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}

include!(concat!(env!("OUT_DIR"), "/pin_config.rs"));
include!(concat!(env!("OUT_DIR"), "/swd.rs"));
