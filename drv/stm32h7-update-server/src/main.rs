// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Functions for writing to flash for updates
//
// This driver is intended to carry as little state as possible. Most of the
// heavy work and decision making should be handled in other tasks.
#![no_std]
#![no_main]

use drv_update_api::UpdateError;
use idol_runtime::{ClientError, Leased, LenLimit, RequestError, R};
use ringbuf::*;
use stm32h7::stm32h753 as device;
use userlib::*;

// Keys constants are defined in RM0433 Rev 7
// Section 4.9.2
const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

// Keys constants are defined in RM0433 Rev 7
// Section 4.9.3
const FLASH_OPT_KEY1: u32 = 0x0819_2A3B;
const FLASH_OPT_KEY2: u32 = 0x4C5D_6E7F;

const BANK_ADDR: u32 = 0x08100000;
const BANK_END: u32 = 0x08200000;

// Writes are indexed by flash words, BANK_ADDR is word 0,
// BANK_ADDR + FLASH_WORD_BYTES is word 1 etc.
const BANK_WORD_LIMIT: usize =
    (BANK_END - BANK_ADDR) as usize / FLASH_WORD_BYTES;

// RM0433 Rev 7 section 4.3.9
// Flash word is defined as 256 bits
const FLASH_WORD_BITS: usize = 256;

// Total length of a word in bytes (i.e. our array size)
const FLASH_WORD_BYTES: usize = FLASH_WORD_BITS / 8;

// number of 32-bit words we will have to write
const FLASH_CPU_WORDS: usize = FLASH_WORD_BYTES / 4;

// Block is an abstract concept here. It represents the size of data the
// driver will process at a time.
const BLOCK_SIZE_BYTES: usize = FLASH_WORD_BYTES * 32;

#[derive(Copy, Clone, PartialEq)]
enum Trace {
    EraseStart,
    EraseEnd,
    WriteStart,
    WriteEnd,
    FinishStart,
    FinishEnd,
    WriteBlock(usize),
    None,
}

enum UpdateState {
    NoUpdate,
    InProgress,
    Finished,
}

ringbuf!(Trace, 64, Trace::None);

const FLASH_TIMEOUT: usize = 10000;

struct ServerImpl<'a> {
    flash: &'a device::flash::RegisterBlock,
    state: UpdateState,
}

impl<'a> ServerImpl<'a> {
    // See RM0433 Rev 7 section 4.3.13
    fn swap_banks(&mut self) -> Result<(), RequestError<UpdateError>> {
        let flash = unsafe { &*device::FLASH::ptr() };

        ringbuf_entry!(Trace::FinishStart);
        if flash.optsr_cur().read().swap_bank_opt().bit() {
            flash
                .optsr_prg()
                .modify(|_, w| w.swap_bank_opt().clear_bit());
        } else {
            flash.optsr_prg().modify(|_, w| w.swap_bank_opt().set_bit());
        }

        flash.optcr().modify(|_, w| w.optstart().set_bit());

        loop {
            if !flash.optsr_cur().read().opt_busy().bit() {
                break;
            }
        }

        ringbuf_entry!(Trace::FinishEnd);
        Ok(())
    }

    fn poll_flash_done(&mut self) -> Result<(), RequestError<UpdateError>> {
        let mut cnt = 0;
        let mut seen = false;

        loop {
            if self.flash.bank2().sr.read().qw().bit() {
                seen = true;
                cnt = 0;
            }

            if seen && !self.flash.bank2().sr.read().qw().bit() {
                break;
            } else {
                cnt += 1;
            }

            if cnt == FLASH_TIMEOUT {
                break;
            }
        }

        if cnt == FLASH_TIMEOUT {
            return Err(UpdateError::Timeout.into());
        }

        if self.flash.bank2().sr.read().dbeccerr().bit() {
            return Err(UpdateError::EccDoubleErr.into());
        }

        if self.flash.bank2().sr.read().sneccerr1().bit() {
            return Err(UpdateError::EccSingleErr.into());
        }

        if self.flash.bank2().sr.read().rdserr().bit() {
            return Err(UpdateError::SecureErr.into());
        }

        if self.flash.bank2().sr.read().rdperr().bit() {
            return Err(UpdateError::ReadProtErr.into());
        }

        if self.flash.bank2().sr.read().operr().bit() {
            return Err(UpdateError::WriteEraseErr.into());
        }

        if self.flash.bank2().sr.read().incerr().bit() {
            return Err(UpdateError::InconsistencyErr.into());
        }

        if self.flash.bank2().sr.read().strberr().bit() {
            return Err(UpdateError::StrobeErr.into());
        }

        if self.flash.bank2().sr.read().pgserr().bit() {
            return Err(UpdateError::ProgSeqErr.into());
        }

        if self.flash.bank2().sr.read().wrperr().bit() {
            return Err(UpdateError::WriteProtErr.into());
        }

        Ok(())
    }

    // RM0433 Rev 7 section 4.3.9
    // Following Single write sequence
    fn write_word(
        &mut self,
        word_number: usize,
        bytes: &[u8],
    ) -> Result<(), RequestError<UpdateError>> {
        ringbuf_entry!(Trace::WriteStart);
        self.flash.bank2().cr.write(|w| w.pg().set_bit());

        if word_number > BANK_WORD_LIMIT {
            return Err(UpdateError::OutOfBounds.into());
        }

        let start = BANK_ADDR + (word_number * FLASH_WORD_BYTES) as u32;

        if bytes.len() != FLASH_WORD_BYTES {
            return Err(UpdateError::BadLength.into());
        }

        for i in 0..FLASH_CPU_WORDS {
            unsafe {
                let byte_offset = i * 4;
                let mut word: [u8; 4] = [0; 4];
                word.clone_from_slice(&bytes[byte_offset..byte_offset + 4]);

                core::ptr::write_volatile(
                    (start + (i * 4) as u32) as *mut u32,
                    u32::from_ne_bytes(word),
                );
            }
        }

        let b = self.poll_flash_done();
        ringbuf_entry!(Trace::WriteEnd);
        b
    }

    // All sequences can be found in RM0433 Rev 7
    fn unlock(&mut self) {
        if !self.flash.bank2().cr.read().lock().bit() {
            return;
        }

        self.flash
            .bank2()
            .keyr
            .write(|w| unsafe { w.keyr().bits(FLASH_KEY1) });
        self.flash
            .bank2()
            .keyr
            .write(|w| unsafe { w.keyr().bits(FLASH_KEY2) });

        self.flash
            .optkeyr()
            .write(|w| unsafe { w.optkeyr().bits(FLASH_OPT_KEY1) });
        self.flash
            .optkeyr()
            .write(|w| unsafe { w.optkeyr().bits(FLASH_OPT_KEY2) });
    }

    fn bank_erase(&mut self) -> Result<(), RequestError<UpdateError>> {
        ringbuf_entry!(Trace::EraseStart);
        self.flash
            .bank2()
            .cr
            .write(|w| w.start().set_bit().ber().set_bit());

        let b = self.poll_flash_done();
        ringbuf_entry!(Trace::EraseEnd);
        b
    }
}

impl idl::InOrderUpdateImpl for ServerImpl<'_> {
    fn prep_image_update(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError<UpdateError>> {
        match self.state {
            UpdateState::InProgress | UpdateState::Finished => {
                return Err(UpdateError::UpdateInProgress.into())
            }
            _ => (),
        }

        self.unlock();
        self.bank_erase()?;
        self.state = UpdateState::InProgress;
        Ok(())
    }

    fn write_one_block(
        &mut self,
        _: &RecvMessage,
        block_num: usize,
        block: LenLimit<Leased<R, [u8]>, BLOCK_SIZE_BYTES>,
    ) -> Result<(), RequestError<UpdateError>> {
        match self.state {
            UpdateState::NoUpdate | UpdateState::Finished => {
                return Err(UpdateError::UpdateInProgress.into())
            }
            _ => (),
        }

        let len = block.len();
        let mut flash_page: [u8; BLOCK_SIZE_BYTES] = [0; BLOCK_SIZE_BYTES];

        block
            .read_range(0..len as usize, &mut flash_page)
            .map_err(|_| RequestError::Fail(ClientError::WentAway))?;

        if len != BLOCK_SIZE_BYTES {
            let mut i = len;

            while len < BLOCK_SIZE_BYTES {
                flash_page[i] = 0;
                i += 1;
            }
        }

        ringbuf_entry!(Trace::WriteBlock(block_num as usize));
        for (i, c) in flash_page.chunks(FLASH_WORD_BYTES).enumerate() {
            const FLASH_WORDS_PER_BLOCK: usize =
                BLOCK_SIZE_BYTES / FLASH_WORD_BYTES;

            self.write_word(block_num * FLASH_WORDS_PER_BLOCK + i, &c)?;
        }

        Ok(())
    }

    fn finish_image_update(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError<UpdateError>> {
        match self.state {
            UpdateState::NoUpdate | UpdateState::Finished => {
                return Err(UpdateError::UpdateInProgress.into())
            }
            _ => (),
        }

        self.swap_banks()?;
        self.state = UpdateState::Finished;
        Ok(())
    }

    fn block_size(
        &mut self,
        _: &RecvMessage,
    ) -> Result<usize, RequestError<UpdateError>> {
        Ok(BLOCK_SIZE_BYTES)
    }
}

#[export_name = "main"]
fn main() -> ! {
    let flash = unsafe { &*device::FLASH::ptr() };

    let mut server = ServerImpl {
        flash,
        state: UpdateState::NoUpdate,
    };
    let mut incoming = [0u8; idl::INCOMING_SIZE];

    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use super::UpdateError;

    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
