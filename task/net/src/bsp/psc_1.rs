// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use crate::{mgmt, pins};
use drv_spi_api::Spi;
use drv_stm32h7_eth as eth;
use drv_stm32xx_sys_api::{Alternate, Port, Sys};
use userlib::task_slot;

task_slot!(SPI, spi_driver);

// This system wants to be woken periodically to do logging
pub const WAKE_INTERVAL: Option<u64> = Some(500);

////////////////////////////////////////////////////////////////////////////////

/// Stateless function to configure ethernet pins before the Bsp struct
/// is actually constructed
pub fn configure_ethernet_pins(sys: &Sys) {
    pins::RmiiPins {
        refclk: Port::A.pin(1),
        crs_dv: Port::A.pin(7),
        tx_en: Port::G.pin(11),
        txd0: Port::G.pin(13),
        txd1: Port::G.pin(12),
        rxd0: Port::C.pin(4),
        rxd1: Port::C.pin(5),
        af: Alternate::AF11,
    }
    .configure(sys);

    pins::MdioPins {
        mdio: Port::A.pin(2),
        mdc: Port::C.pin(1),
        af: Alternate::AF11,
    }
    .configure(sys);
}

pub struct Bsp(mgmt::Bsp);

pub fn preinit() {
    // Nothing to do here
}

impl Bsp {
    pub fn new(eth: &mut eth::Ethernet, sys: &Sys) -> Self {
        let bsp = mgmt::Config {
            // SP_TO_MGMT_V1P0_EN / SP_TO_MGMT_V2P5_EN
            // (note that the latter also enables the MGMT_PHY_REFCLK)
            power_en: Some(Port::I.pin(10).and_pin(12)),
            slow_power_en: true,
            power_good: None, // TODO
            pll_lock: None,

            // Based on ordering in app.toml
            ksz8463_spi: Spi::from(SPI.get_task_id()).device(0),
            ksz8463_nrst: Port::C.pin(2),
            ksz8463_rst_type: mgmt::Ksz8463ResetSpeed::Normal,

            // SP_TO_MGMT_PHY_COMA_MODE
            vsc85x2_coma_mode: Some(Port::D.pin(7)),

            // SP_TO_MGMT_PHY_RESET
            vsc85x2_nrst: Port::A.pin(8),

            vsc85x2_base_port: 0b11110, // Based on resistor strapping
        }
        .build(sys, eth);

        use crate::miim_bridge::MiimBridge;
        let rw = &mut MiimBridge::new(eth);
        for i in 0..2 {
            use vsc7448_pac::phy;
            let phy = &mut bsp.vsc85x2.phy(i, rw);

            // Errata: this bit must be disabled for loopback mode to work.
            phy.modify(
                phy::EXTENDED_3::MEDIA_SERDES_TX_CRC_ERROR_COUNTER(),
                |r| {
                    let mut v = u16::from(*r);
                    v &= !(1 << 13);
                    *r = v.into();
                },
            )
            .unwrap();

            // Enable far-end loopback
            phy.modify(phy::STANDARD::EXTENDED_PHY_CONTROL(), |r| {
                let mut v = u16::from(*r);
                v |= 1 << 3;
                *r = v.into();
            })
            .unwrap();

            // Disable Rx to avoid DDOSing yourself
            bsp.ksz8463
                .write_masked(ksz8463::Register::PxCR2(i + 1), 0, 1 << 9)
                .unwrap()
        }

        Self(bsp)
    }

    pub fn wake(&self, eth: &mut eth::Ethernet) {
        self.0.wake(eth);
    }
}
