// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use anyhow::Result;
use build_lpc55pins::PinConfig;
use quote::ToTokens;
use serde::Deserialize;
use std::io::Write;

#[derive(Deserialize)]
struct TaskConfig {
    in_cfg: Vec<PinConfig>,
    out_cfg: Vec<PinConfig>,
    pins: Vec<PinConfig>,
    spi_num: usize,
}

fn generate_swd_functions(config: &TaskConfig) -> Result<()> {
    let out_dir = std::env::var("OUT_DIR")?;
    let dest_path = std::path::Path::new(&out_dir).join("swd.rs");
    let mut file = std::fs::File::create(&dest_path)?;

    // The RoT -> SP SWD control requires setting the IO functions at runtime
    // as opposed to just startup.
    writeln!(&mut file, "// io_out = MOSI on, MISO off")?;
    writeln!(&mut file, "fn switch_io_out(task : TaskId) {{")?;
    writeln!(&mut file, "use drv_lpc55_gpio_api::*;")?;
    writeln!(&mut file, "let iocon = Pins::from(task);")?;
    for p in &config.out_cfg {
        writeln!(&mut file, "iocon.iocon_configure(")?;
        writeln!(&mut file, "{}", p.to_token_stream())?;
        writeln!(&mut file, ").unwrap_lite();")?;
    }
    writeln!(&mut file, "}}")?;

    writeln!(&mut file, "// io_in = MOSI off, MISO on")?;
    writeln!(&mut file, "fn switch_io_in(task : TaskId) {{")?;
    writeln!(&mut file, "use drv_lpc55_gpio_api::*;")?;
    writeln!(&mut file, "let iocon = Pins::from(task);")?;
    for p in &config.in_cfg {
        writeln!(&mut file, "iocon.iocon_configure(")?;
        writeln!(&mut file, "{}", p.to_token_stream())?;
        writeln!(&mut file, ").unwrap_lite();")?;
    }
    writeln!(&mut file, "}}")?;

    writeln!(&mut file, "fn setup_spi(task : TaskId) -> spi_core::Spi {{")?;
    writeln!(&mut file, "let syscon = Syscon::from(task);")?;
    writeln!(
        &mut file,
        "syscon.enable_clock(Peripheral::Fc{}).unwrap_lite();",
        config.spi_num
    )?;
    writeln!(
        &mut file,
        "syscon.leave_reset(Peripheral::Fc{}).unwrap_lite();",
        config.spi_num
    )?;
    writeln!(
        &mut file,
        "let flexcomm = unsafe {{ &*device::FLEXCOMM{}::ptr() }};",
        config.spi_num
    )?;
    writeln!(&mut file, "flexcomm.pselid.write(|w| w.persel().spi());")?;
    writeln!(
        &mut file,
        "let registers = unsafe {{ &*device::SPI{}::ptr() }};",
        config.spi_num
    )?;
    writeln!(&mut file, "spi_core::Spi::from(registers)")?;
    writeln!(&mut file, "}}")?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    idol::server::build_server_support(
        "../../idl/sp-ctrl.idol",
        "server_stub.rs",
        idol::server::ServerStyle::InOrder,
    )?;

    let task_config = build_util::task_config::<TaskConfig>()?;

    generate_swd_functions(&task_config)?;
    build_lpc55pins::codegen(task_config.pins)?;

    Ok(())
}
