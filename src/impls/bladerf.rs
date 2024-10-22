use std::{
    path::Path,
    sync::Arc,
    time::{Duration, Instant},
};

use bladerf::{Channel, Format, GainMode, Loopback};
use num_complex::{Complex, Complex32};

use crate::{Args, DeviceTrait, Direction, Error, Range, RangeItem};

pub struct BladeRf {
    inner: Arc<BladeRfInner>,
}

impl From<bladerf::BladeRF> for BladeRf {
    fn from(dev: bladerf::BladeRF) -> Self {
        Self {
            inner: Arc::new(BladeRfInner { dev }),
        }
    }
}

struct BladeRfInner {
    dev: bladerf::BladeRF,
}

impl std::ops::Deref for BladeRfInner {
    type Target = bladerf::BladeRF;

    fn deref(&self) -> &Self::Target {
        &self.dev
    }
}

impl BladeRf {
    pub fn probe(_args: &Args) -> Result<Vec<Args>, Error> {
        Ok(bladerf::get_device_list()?
            .into_iter()
            .flat_map(|d| {
                let mut args = Args::new();
                if let Some(bus) = d.usb_bus() {
                    args.set("bus", bus.to_string());
                }
                if let Some(addr) = d.usb_addr() {
                    args.set("address", addr.to_string());
                }
                if args.iter().count() == 0 {
                    log::warn!("Unable to get busnum or address for device: {d:?}");
                    None
                } else {
                    args.set("driver", "bladerf");
                    args.set("manufacturer", d.manufacturer());
                    args.set("product", d.product());
                    Some(args)
                }
            })
            .collect())
    }

    pub fn open<A: TryInto<Args>>(args: A) -> Result<Self, Error> {
        let args: Args = args.try_into().or(Err(Error::ValueError))?;

        if let Ok(fd) = args.get::<i32>("fd") {
            todo!("{fd}");
        }

        let bus: Option<u8> = args.get("bus").ok();
        let address: Option<u8> = args.get("address").ok();
        let dev = match (bus, address) {
            (Some(bus), Some(address)) => Self::open_bus(bus, address)?,
            _ => bladerf::BladeRF::open_first()?,
        };
        let serial = dev.get_serial()?;

        // FIXME: work around `Calibration TIMEOUT (0x16, 0x80)` when re-opening already connected
        // device by resetting it
        log::debug!("Resetting device...");
        let _ = dev
            .device_reset()
            .map_err(|e| println!("Failed to reset device: {e:?}"));

        let start = Instant::now();
        let dev = 'outer: loop {
            for info in bladerf::get_device_list().unwrap_or_default() {
                println!("Found: {:?}", info.serial());
                if info.serial() == serial {
                    if let Ok(dev) = info.open() {
                        break 'outer dev;
                    }
                }
            }
            if start.elapsed().as_secs() > 2 {
                log::error!("Failed to re-discover device `{serial}` after two seconds");
                return Err(Error::NotFound);
            }
            std::thread::sleep(Duration::from_millis(50));
        };

        let fpga_bitstream_path = if let Ok(path) = args.get::<String>("fpga_bitstream_path") {
            Some(path)
        } else if let Ok(path) = std::env::var(bladerf::FPGA_BITSTREAM_VAR_NAME) {
            if Path::new(&path).exists() {
                Some(path)
            } else {
                None
            }
        } else {
            None
        };

        if let Some(path) = fpga_bitstream_path {
            log::info!("Loading fpga bitstream from path: {path}");
            dev.load_fpga(path)?;
        }

        return Ok(dev.into());
    }

    fn open_bus(bus: u8, address: u8) -> Result<bladerf::BladeRF, Error> {
        for info in bladerf::get_device_list()? {
            if info.usb_bus() == Some(bus) && info.usb_addr() == Some(address) {
                log::info!("Opening bladerf device, bus: {bus:?}, address: {address:?}");

                return Ok(info.open()?);
            }
        }

        log::warn!("Failed to find bladerf bus: {bus:?}, address: {address:?}");
        return Err(Error::NotFound);
    }

    fn to_channel(direction: Direction, channel: usize) -> Result<Channel, Error> {
        if channel != 0 && channel != 1 {}
        match (direction, channel) {
            // TODO: enable MIMO support
            (Direction::Rx, 0) => Ok(Channel::Rx1),
            // (Direction::Rx, 1) => Ok(Channel::Rx2),
            (Direction::Tx, 0) => Ok(Channel::Tx1),
            // (Direction::Tx, 1) => Ok(Channel::Tx2),
            _ => return Err(Error::ValueError),
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct StreamParams {
    channel: Channel,
    format: Format,
    num_buffers: u32,
    buffer_size: u32,
    num_transfers: u32,
    stream_timeout: Duration,
}

const TRANSFER_NUM_SAMPLES: usize = 128 * 1024;

impl StreamParams {
    pub fn new(channel: Channel, format: Format) -> Self {
        Self {
            channel,
            format,
            num_buffers: 5,
            buffer_size: TRANSFER_NUM_SAMPLES as u32,
            num_transfers: 2,
            stream_timeout: Duration::from_millis(250),
        }
    }
}

pub struct RxStreamer {
    inner: Arc<BladeRfInner>,
    params: StreamParams,
    buf: Vec<num_complex::Complex<i16>>,
}

impl RxStreamer {
    fn new(inner: Arc<BladeRfInner>, params: StreamParams) -> Self {
        Self {
            inner,
            params,
            buf: vec![],
        }
    }
}

impl crate::RxStreamer for RxStreamer {
    fn mtu(&self) -> Result<usize, Error> {
        Ok(TRANSFER_NUM_SAMPLES)
    }

    fn activate_at(&mut self, _time_ns: Option<i64>) -> Result<(), Error> {
        // TODO: sleep precisely for `time_ns`
        let _ = self
            .inner
            .set_rx_mux(bladerf::RxMux::Baseband)
            .map_err(|e| println!("failed to set rx mux to baseband: {e:?}"));
        self.inner.set_loopback(Loopback::None)?;

        log::info!("Writing config: {:?}", &self.params);

        self.inner
            .sync_config(
                self.params.channel,
                self.params.format,
                self.params.num_buffers,
                self.params.buffer_size,
                self.params.num_transfers,
                self.params.stream_timeout,
            )
            .unwrap();

        self.inner.set_bandwidth(self.params.channel, 250_000)?;
        self.inner
            .set_gain_mode(self.params.channel, GainMode::Default)?;

        self.inner.enable_module(self.params.channel).unwrap();

        Ok(())
    }

    fn deactivate_at(&mut self, _time_ns: Option<i64>) -> Result<(), Error> {
        // TODO: sleep precisely for `time_ns`

        self.inner.disable_module(self.params.channel)?;
        log::info!("Shutting down channel: {:?}", self.params.channel);

        Ok(())
    }

    fn read(
        &mut self,
        buffers: &mut [&mut [num_complex::Complex32]],
        _timeout_us: i64,
    ) -> Result<usize, Error> {
        assert_eq!(buffers.len(), 1);

        if buffers[0].is_empty() {
            return Ok(0);
        }
        assert_eq!(self.params.format, Format::Sc16Q11);

        if self.buf.len() < buffers[0].len() {
            self.buf.resize(buffers[0].len(), Complex::ZERO);
        }

        self.inner
            .sync_rx(&mut self.buf, None, self.params.stream_timeout)
            .unwrap();

        // TODO: make sure assembly is good
        for i in 0..buffers[0].len() {
            buffers[0][i] = Complex::new(
                self.buf[i].re as f32 / 2047.0,
                self.buf[i].im as f32 / 2047.0,
            );
        }

        Ok(buffers[0].len())
    }
}

pub struct TxStreamer {
    inner: Arc<BladeRfInner>,
    params: StreamParams,
    buf: Vec<num_complex::Complex<i16>>,
}

impl TxStreamer {
    fn new(inner: Arc<BladeRfInner>, params: StreamParams) -> Self {
        Self {
            inner,
            params,
            buf: vec![],
        }
    }
}

impl crate::TxStreamer for TxStreamer {
    fn mtu(&self) -> Result<usize, Error> {
        Ok(TRANSFER_NUM_SAMPLES)
    }

    fn activate_at(&mut self, _time_ns: Option<i64>) -> Result<(), Error> {
        // TODO: sleep precisely for `time_ns`

        self.inner.sync_config(
            self.params.channel,
            self.params.format,
            self.params.num_buffers,
            self.params.buffer_size,
            self.params.num_transfers,
            self.params.stream_timeout,
        )?;

        Ok(())
    }

    fn deactivate_at(&mut self, _time_ns: Option<i64>) -> Result<(), Error> {
        // TODO: sleep precisely for `time_ns`

        todo!();
        Ok(())
    }

    fn write(
        &mut self,
        buffers: &[&[num_complex::Complex32]],
        _at_ns: Option<i64>,
        _end_burst: bool,
        _timeout_us: i64,
    ) -> Result<usize, Error> {
        assert_eq!(buffers.len(), 1);

        if buffers[0].is_empty() {
            return Ok(0);
        }
        assert_eq!(self.params.format, Format::Sc16Q11);
        // TODO: possible to relax this in the future, need to know implications
        assert_eq!(buffers[0].len(), TRANSFER_NUM_SAMPLES);

        if self.buf.len() < buffers[0].len() {
            self.buf.resize(buffers[0].len(), Complex::ZERO);
        }

        // TODO: make sure assembly is good
        for i in 0..buffers[0].len() {
            self.buf[i].re = (buffers[0][i].re * 2047.0) as i16;
            self.buf[i].im = (buffers[0][i].im * 2047.0) as i16;
        }
        self.inner
            .sync_tx(&self.buf, None, self.params.stream_timeout)?;

        Ok(buffers[0].len())
    }

    fn write_all(
        &mut self,
        buffers: &[&[num_complex::Complex32]],
        _at_ns: Option<i64>,
        _end_burst: bool,
        _timeout_us: i64,
    ) -> Result<(), Error> {
        debug_assert_eq!(buffers.len(), 1);

        let mut n = 0;
        while n < buffers[0].len() {
            let buf = &buffers[0][n..];
            n += self.write(&[buf], None, false, 0)?;
        }

        Ok(())
    }
}

impl crate::DeviceTrait for BladeRf {
    type RxStreamer = RxStreamer;

    type TxStreamer = TxStreamer;

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }

    fn driver(&self) -> crate::Driver {
        crate::Driver::BladeRf
    }

    fn id(&self) -> Result<String, Error> {
        Ok(self.inner.get_serial()?)
    }

    fn info(&self) -> Result<crate::Args, Error> {
        let mut args = crate::Args::default();
        args.set(
            "firmware version",
            self.inner.firmware_version()?.to_string(),
        );
        args.set("fpga version", self.inner.fpga_version()?.to_string());
        args.set("serial", self.inner.get_serial()?);
        args.set("backend", format!("{:?}", self.inner.info()?.backend()?));
        Ok(args)
    }

    fn num_channels(&self, _: crate::Direction) -> Result<usize, Error> {
        Ok(2)
    }

    fn full_duplex(&self, _direction: Direction, _channel: usize) -> Result<bool, Error> {
        Ok(true)
    }

    fn rx_streamer(&self, channels: &[usize], _args: Args) -> Result<Self::RxStreamer, Error> {
        if channels.len() > 1 {
            todo!("Only only stream allowed per direction, implement interleaved api");
        }
        if channels != [0] && channels != [1] {
            Err(Error::ValueError)
        } else {
            Ok(RxStreamer::new(
                Arc::clone(&self.inner),
                StreamParams::new(Channel::Rx1, Format::Sc16Q11),
            ))
        }
    }

    fn tx_streamer(&self, channels: &[usize], _args: Args) -> Result<Self::TxStreamer, Error> {
        if channels.len() > 1 {
            todo!("Only only stream allowed per direction, implement interleaved api");
        }
        if channels != [0] && channels != [1] {
            Err(Error::ValueError)
        } else {
            Ok(TxStreamer::new(
                Arc::clone(&self.inner),
                StreamParams::new(Channel::Tx1, Format::Sc16Q11),
            ))
        }
    }

    fn antennas(&self, direction: Direction, channel: usize) -> Result<Vec<String>, Error> {
        self.antenna(direction, channel).map(|a| vec![a])
    }

    fn antenna(&self, direction: Direction, channel: usize) -> Result<String, Error> {
        let channel = Self::to_channel(direction, channel)?;
        Ok(if channel.is_tx() { "TX" } else { "RX" }.to_string())
    }

    fn set_antenna(&self, direction: Direction, channel: usize, name: &str) -> Result<(), Error> {
        let channel = Self::to_channel(direction, channel)?;
        if channel.is_rx() && name == "RX" || channel.is_tx() && name == "TX" {
            Ok(())
        } else {
            Err(Error::ValueError)
        }
    }

    fn gain_elements(&self, direction: Direction, channel: usize) -> Result<Vec<String>, Error> {
        let channel = Self::to_channel(direction, channel)?;
        Ok(self.inner.get_gain_stages(channel)?)
    }

    fn supports_agc(&self, direction: Direction, channel: usize) -> Result<bool, Error> {
        let channel = Self::to_channel(direction, channel)?;
        // TODO: bladerf has FastAttackAgc, SlowAttackAgc, and HybridAgc
        // Figure out how to expose these to seify users
        // For now always use hybrid
        Ok(self
            .inner
            .get_gain_modes(channel)?
            .iter()
            .find(|g| g.mode == GainMode::HybridAgc)
            .is_some())
    }

    fn enable_agc(&self, direction: Direction, channel: usize, _agc: bool) -> Result<(), Error> {
        let channel = Self::to_channel(direction, channel)?;
        Ok(self.inner.set_gain_mode(channel, GainMode::HybridAgc)?)
    }

    fn agc(&self, direction: Direction, channel: usize) -> Result<bool, Error> {
        let channel = Self::to_channel(direction, channel)?;
        Ok(match self.inner.get_gain_mode(channel)? {
            GainMode::Default => false,
            GainMode::Manual => false,
            GainMode::FastAttackAgc => true,
            GainMode::SlowAttackAgc => true,
            GainMode::HybridAgc => true,
        })
    }

    fn set_gain(&self, direction: Direction, channel: usize, gain: f64) -> Result<(), Error> {
        self.set_gain_element(direction, channel, "full", gain)
    }

    fn gain(&self, direction: Direction, channel: usize) -> Result<Option<f64>, Error> {
        self.gain_element(direction, channel, "full")
    }

    fn gain_range(&self, direction: Direction, channel: usize) -> Result<Range, Error> {
        self.gain_element_range(direction, channel, "full")
    }

    fn set_gain_element(
        &self,
        direction: Direction,
        channel: usize,
        name: &str,
        gain: f64,
    ) -> Result<(), Error> {
        let r = self.gain_range(direction, channel)?;
        let channel = Self::to_channel(direction, channel)?;

        log::debug!("Set gain {channel:?} {gain}");
        if r.contains(gain) && name == "full" {
            Ok(self.inner.set_gain_stage(channel, name, gain as i32)?)
        } else {
            log::warn!("Gain out of range");
            Err(Error::OutOfRange(r, gain))
        }
    }

    fn gain_element(
        &self,
        direction: Direction,
        channel: usize,
        name: &str,
    ) -> Result<Option<f64>, Error> {
        let channel = Self::to_channel(direction, channel)?;
        Ok(self
            .inner
            .get_gain_stage(channel, name)
            .ok()
            .map(|g| g as f64))
    }

    fn gain_element_range(
        &self,
        direction: Direction,
        channel: usize,
        name: &str,
    ) -> Result<Range, Error> {
        let channel = Self::to_channel(direction, channel)?;
        Ok(self
            .inner
            .get_gain_stage_range(channel, name)
            .unwrap()
            .into())
    }

    fn frequency_range(&self, direction: Direction, channel: usize) -> Result<Range, Error> {
        self.component_frequency_range(direction, channel, "TUNER")
    }

    fn frequency(&self, direction: Direction, channel: usize) -> Result<f64, Error> {
        self.component_frequency(direction, channel, "TUNER")
    }

    fn set_frequency(
        &self,
        direction: Direction,
        channel: usize,
        frequency: f64,
        _args: Args,
    ) -> Result<(), Error> {
        self.set_component_frequency(direction, channel, "TUNER", frequency)
    }

    fn frequency_components(
        &self,
        direction: Direction,
        channel: usize,
    ) -> Result<Vec<String>, Error> {
        let _channel = Self::to_channel(direction, channel)?;
        Ok(vec!["TUNER".to_string()])
    }

    fn component_frequency_range(
        &self,
        direction: Direction,
        channel: usize,
        name: &str,
    ) -> Result<Range, Error> {
        let channel = Self::to_channel(direction, channel)?;
        if name == "TUNER" {
            Ok(self.inner.get_frequency_range(channel)?.into())
        } else {
            Err(Error::ValueError)
        }
    }

    fn component_frequency(
        &self,
        direction: Direction,
        channel: usize,
        name: &str,
    ) -> Result<f64, Error> {
        let channel = Self::to_channel(direction, channel)?;
        if name == "TUNER" {
            Ok(self.inner.get_frequency(channel)? as f64)
        } else {
            Err(Error::ValueError)
        }
    }

    fn set_component_frequency(
        &self,
        direction: Direction,
        channel: usize,
        name: &str,
        frequency: f64,
    ) -> Result<(), Error> {
        let channel = Self::to_channel(direction, channel)?;
        log::debug!(
            "Set frequency {channel:?} {name} = {}MHz",
            frequency / 1_000_000.0
        );
        if name == "TUNER" {
            Ok(self.inner.set_frequency(channel, frequency as u64)?)
        } else {
            Err(Error::ValueError)
        }
    }

    fn sample_rate(&self, direction: Direction, channel: usize) -> Result<f64, Error> {
        let channel = Self::to_channel(direction, channel)?;
        Ok(self.inner.get_sample_rate(channel)? as f64)
    }

    fn set_sample_rate(
        &self,
        direction: Direction,
        channel: usize,
        rate: f64,
    ) -> Result<(), Error> {
        if self
            .get_sample_rate_range(direction, channel)?
            .contains(rate)
        {
            let channel = Self::to_channel(direction, channel)?;
            log::debug!("Set sample rate {channel:?} {}MHz", rate / 1_000_000.0);
            let _actual = self.inner.set_sample_rate(channel, rate as u32)?;

            log::debug!("Sample rate actually {}MHz", _actual as f64 / 1_000_000.0);
            Ok(())
        } else {
            Err(Error::ValueError)
        }
    }

    fn get_sample_rate_range(&self, direction: Direction, channel: usize) -> Result<Range, Error> {
        let channel = Self::to_channel(direction, channel)?;
        Ok(self.inner.get_sample_rate_range(channel)?.into())
    }
}

impl From<bladerf::Range> for Range {
    fn from(range: bladerf::Range) -> Self {
        Self::new(vec![RangeItem::Interval(range.min, range.max)])
    }
}
