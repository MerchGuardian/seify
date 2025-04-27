use clap::Parser;
use seify::enumerate_with_args;
use seify::Device;
use seify::Direction::{Rx, Tx};

#[derive(Parser, Debug)]
#[clap(version)]
struct Args {
    /// Device Filters
    #[clap(short, long, default_value = "")]
    args: String,
}

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let cli = Args::parse();

    let devs = enumerate_with_args(cli.args)?;
    println!("Devices");
    println!("=========================================");
    println!("devs: {devs:?}");

    for d in devs {
        let dev = Device::from_args(d)?;

        println!();
        println!("Device ({:?} - {:?}), ", dev.driver(), dev.id()?);
        println!("=========================================");

        println!("driver:       {:?}", dev.driver());
        println!("id:           {:?}", dev.id()?);
        println!("info:         {:?}", dev.info()?);

        println!("RX:");
        println!("  sample rate:  {:?}", dev.sample_rate(Rx, 0)?);
        println!("  frequency:    {:?}", dev.frequency(Rx, 0)?);
        println!("  gain:         {:?}", dev.gain(Rx, 0)?);
        println!("  gain range:   {:?}", dev.gain_range(Rx, 0)?);
        println!(
            "  sample rate range: {:?}",
            dev.get_sample_rate_range(Rx, 0)?
        );

        println!("TX:");
        println!("  sample rate:  {:?}", dev.sample_rate(Tx, 0)?);
        println!("  frequency:    {:?}", dev.frequency(Tx, 0)?);
        println!("  gain:         {:?}", dev.gain(Tx, 0)?);
        println!("  gain range:   {:?}", dev.gain_range(Tx, 0)?);
        println!(
            "  sample rate range: {:?}",
            dev.get_sample_rate_range(Tx, 0)?
        );
    }

    Ok(())
}
