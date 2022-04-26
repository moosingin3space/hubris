// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use crate::Config;
use drv_stm32h7_qspi::Qspi;
use drv_stm32xx_sys_api as sys_api;

#[allow(dead_code)]
pub(crate) fn init(qspi: &Qspi, sys: &sys_api::Sys) -> Config {
    let clock = 5; // 200MHz kernel / 5 = 40MHz clock
    qspi.configure(
        clock, 25, // 2**25 = 32MiB = 256Mib
    );
    // Gimletlet pin mapping
    // PF6 SP_QSPI1_IO3
    // PF7 SP_QSPI1_IO2
    // PF8 SP_QSPI1_IO0
    // PF9 SP_QSPI1_IO1
    // PF10 SP_QSPI1_CLK
    //
    // PG6 SP_QSPI1_CS
    //
    // TODO check these if I have a quadspimux board
    // PF4 SP_FLASH_TO_SP_RESET_L
    // PF5 SP_TO_SP3_FLASH_MUX_SELECT <-- low means us
    //
    sys.gpio_configure_alternate(
        sys_api::Port::F.pin(6).and_pin(7).and_pin(10),
        sys_api::OutputType::PushPull,
        sys_api::Speed::VeryHigh,
        sys_api::Pull::None,
        sys_api::Alternate::AF9,
    )
    .unwrap();
    sys.gpio_configure_alternate(
        sys_api::Port::F.pin(8).and_pin(9),
        sys_api::OutputType::PushPull,
        sys_api::Speed::VeryHigh,
        sys_api::Pull::None,
        sys_api::Alternate::AF10,
    )
    .unwrap();
    sys.gpio_configure_alternate(
        sys_api::Port::G.pin(6),
        sys_api::OutputType::PushPull,
        sys_api::Speed::VeryHigh,
        sys_api::Pull::None,
        sys_api::Alternate::AF10,
    )
    .unwrap();

    Config {
        sp_host_mux_select: sys_api::Port::F.pin(5),
        reset: sys_api::Port::F.pin(4),
        flash_select: None,
        clock,
    }
}
