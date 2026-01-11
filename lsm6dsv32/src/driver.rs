pub use crate::config::*;

use defmt::*;
use embassy_stm32::{exti::ExtiInput, gpio::Output, mode::Async, spi::Spi};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;

pub const EXPECTED_WHO_AM_I: u8 = 0x70;
pub const G32_SCALE_FACTOR: f32 = 32.0 / 32768.0;

/// encodes bitfields into a 8Bit register
#[macro_export]
macro_rules! encode_reg8 {
    (base: $base:expr, { $($val:expr => $shift:literal, $width:literal),* $(,)? }) => {
        {
            let mut v: u8 = $base;
            $(
                let mask: u8 = ((1u16 << $width) - 1) as u8;
                let shifted_mask: u8 = mask << $shift;
                v = (v & !shifted_mask) | (($val as u8 & mask) << $shift);
            )*
            v
        }
    };

    ({ $($val:expr => $shift:literal, $width:literal),* $(,)? }) => {
        $crate::encode_reg8!(base: 0, { $($val => $shift, $width),* })
    };
}

/// Hardware layer containing peripherals and calibration offset
pub struct Lsm6dsv32HW<'d> {
    spi: &'d Mutex<ThreadModeRawMutex, Spi<'d, Async>>,
    cs: Output<'d>,
    int1: ExtiInput<'d>,
    int2: ExtiInput<'d>,
    bias_accel: [i16; 3],
    bias_accel_ch2: [i16; 3],
    bias_gyro: [i16; 3],
    ts_offset: u32,
}

/// Driver-Object, managing device state, configuration and hardware acces
pub struct Lsm6dsv32<'d, F, I1, I2> {
    pub hw: Lsm6dsv32HW<'d>,
    pub config: ImuConfig<F, I1, I2>,
}

impl<'d> Lsm6dsv32<'d, FifoDisabled, Int1Disabled, Int2Disabled> {
    pub async fn new(
        spi: &'d Mutex<ThreadModeRawMutex, Spi<'d, Async>>,
        cs: Output<'d>,
        int1: ExtiInput<'d>,
        int2: ExtiInput<'d>,
    ) -> Self {
        let imu_config = ImuConfig::default();

        let hw = Lsm6dsv32HW {
            spi,
            cs,
            int1,
            int2,
            bias_accel: [0; 3],
            bias_gyro: [0; 3],
            bias_accel_ch2: [0; 3],
            ts_offset: 0,
        };
        let mut instance = Self {
            hw,
            config: imu_config.clone(),
        };
        instance.reset_timer().await;
        instance.set_config(imu_config.build()).await;
        instance
    }
}

impl<'d, L, I1, I2> Lsm6dsv32<'d, L, I1, I2> {
    /// Converts raw temp data to f32 °C
    pub fn temp_f32(raw: i16) -> f32 {
        (raw as f32 / 256.0) + 25.0
    }
    /// Converts raw gyro data to f32 rad/s
    pub fn gyro_f32(&self, raw: [i16; 3]) -> [f32; 3] {
        raw.map(|val| val as f32 * self.config.gyro.calc_scaling_factor())
    }
    /// Converts dual-channel raw acceleration data to f32 m/s².
    ///
    /// * `raw.0`: Channel 1 raw triplets [x, y, z].
    /// * `raw.1`: Channel 2 raw triplets [x, y, z].
    ///
    /// Returns: `([x, y, z] in m/s², [x, y, z] in m/s²)`
    pub fn accel_dual_f32(&self, raw: ([i16; 3], [i16; 3])) -> ([f32; 3], [f32; 3]) {
        let raw1 = raw.0;
        let raw2 = raw.1;
        (
            raw1.map(|val| val as f32 * self.config.accel.calc_scaling_factor()),
            raw2.map(|val| val as f32 * self.config.accel.calc_scaling_factor_ch2()),
        )
    }

    /// Converts raw acceleration data to f32 m/s²
    pub fn accel_f32(&self, raw: [i16; 3]) -> [f32; 3] {
        raw.map(|val| val as f32 * self.config.accel.calc_scaling_factor())
    }

    /// Converts a raw IMU sample into SI units using f32
    pub fn calc_imusample_f32(&self, raw_sample: ImuSample) -> ImuSampleF32 {
        ImuSampleF32 {
            accel: raw_sample
                .accel
                .map(|val| val as f32 * self.config.accel.calc_scaling_factor()),
            accel_ch2: raw_sample
                .accel_ch2
                .map(|val| val as f32 * self.config.accel.calc_scaling_factor_ch2()),
            gyro: raw_sample
                .gyro
                .map(|val| val as f32 * self.config.gyro.calc_scaling_factor()),
            temp: (raw_sample.temp as f32 / 256.0) + 25.0,
            last_ts: raw_sample.last_ts,
            delta_ts: raw_sample.delta_ts,
        }
    }

    async fn set_config(&mut self, imu_config: ImuConfigRaw) {
        if let Err(e) = self.write_config_registers(&imu_config).await {
            error!("Error writing config registers: {:?}", e);
        }

        if let Err(e) = self.check_who_i_am().await {
            error!("Error checking WHO_AM_I register: {:?}", e);
        }

        //let cfg = &self.config;
        info!("IMU-configuration finished");
    }

    async fn check_who_i_am(&mut self) -> Result<(), Error> {
        let who_am_i_data = self.read_register(Register::WHO_AM_I as u8).await?;
        if who_am_i_data != EXPECTED_WHO_AM_I {
            return Err(Error::WrongWhoAmI(who_am_i_data));
        }
        Ok(())
    }

    async fn write_config_registers(&mut self, imu_config: &ImuConfigRaw) -> Result<(), Error> {
        macro_rules! log_reg {
            ($name:expr, $val:expr) => {
                #[cfg(feature = "debug")]
                debug!("{}: {:08b}", $name, $val);
            };
            ($name:expr, $old:expr, $new:expr) => {
                #[cfg(feature = "debug")]
                debug!("{} old: {:08b}, new: {:08b}", $name, $old, $new);
            };
        }

        let pin_ctrl_old = self.read_register(Register::PIN_CTRL as u8).await?;
        let pin_ctrl = encode_reg8!(base: pin_ctrl_old, {
            1 => 0, 2,
            1 => 5, 1,
            imu_config.general.sdo_pull_up as u8 => 6, 1,
            0 => 7, 1
        });
        log_reg!("PIN_CTRL", pin_ctrl_old, pin_ctrl);
        self.write_register(Register::PIN_CTRL as u8, pin_ctrl)
            .await?;

        let if_ctrl_old = self.read_register(Register::IF_CTRL as u8).await?;
        let if_ctrl = encode_reg8!(base: if_ctrl_old, {
            imu_config.general.sda_pull_up as u8 => 7, 1,
            imu_config.general.anti_spike_filter as u8 => 5, 1,
            imu_config.general.interrupt_lvl as u8 => 4, 1,
            imu_config.general.interrupt_pin_mode as u8 => 3, 1,
            1 => 0, 1
        });
        log_reg!("IF_CTRL", if_ctrl_old, if_ctrl);
        self.write_register(Register::IF_CTRL as u8, if_ctrl)
            .await?;

        let fifo_ctrl1 = encode_reg8!({
            imu_config.fifo.watermark_threshold => 0, 8
        });
        log_reg!("FIFO_CTRL1", fifo_ctrl1);
        self.write_register(Register::FIFO_CTRL1 as u8, fifo_ctrl1)
            .await?;

        let fifo_ctrl3 = encode_reg8!({
            imu_config.fifo.gyro_fifo as u8 => 4, 4,
            imu_config.fifo.accel_fifo as u8 => 0, 4
        });
        log_reg!("FIFO_CTRL3", fifo_ctrl3);
        self.write_register(Register::FIFO_CTRL3 as u8, fifo_ctrl3)
            .await?;

        let fifo_ctrl4 = encode_reg8!({
            imu_config.fifo.ts_fifo as u8 => 6, 2,
            imu_config.fifo.temp_fifo as u8 => 4, 2,
            imu_config.fifo.mode as u8 => 0, 3
        });
        log_reg!("FIFO_CTRL4", fifo_ctrl4);
        self.write_register(Register::FIFO_CTRL4 as u8, fifo_ctrl4)
            .await?;

        let counter_th_89 = ((imu_config.fifo.counter_threshold >> 8) & 0b11) as u8;
        let counter_th_07 = (imu_config.fifo.counter_threshold & 0xFF) as u8;

        let cbdr1 = encode_reg8!({
            imu_config.fifo.counter_trigger as u8 => 5, 2,
            counter_th_89 => 0, 2
        });
        log_reg!("COUNTER_BDR_REG1", cbdr1);
        self.write_register(Register::COUNTER_BDR_REG1 as u8, cbdr1)
            .await?;

        let cbdr2 = encode_reg8!({ counter_th_07 => 0, 8 });
        log_reg!("COUNTER_BDR_REG2", cbdr2);
        self.write_register(Register::COUNTER_BDR_REG2 as u8, cbdr2)
            .await?;

        let int1 = encode_reg8!({
            imu_config.int1.counter_bdr_int as u8 => 6, 1,
            imu_config.int1.fifo_full_int as u8 => 5, 1,
            imu_config.int1.fifo_overrun_int as u8 => 4, 1,
            imu_config.int1.fifo_threshold_int as u8 => 3, 1,
            imu_config.int1.data_ready_gyro as u8 => 1, 1,
            imu_config.int1.data_ready_accel as u8 => 0, 1
        });
        log_reg!("INT1_CTRL", int1);
        self.write_register(Register::INT1_CTRL as u8, int1).await?;

        let int2 = encode_reg8!({
            imu_config.int2.counter_bdr_int as u8 => 6, 1,
            imu_config.int2.fifo_full_int as u8 => 5, 1,
            imu_config.int2.fifo_overrun_int as u8 => 4, 1,
            imu_config.int2.fifo_threshold_int as u8 => 3, 1,
            imu_config.int2.data_ready_gyro as u8 => 1, 1,
            imu_config.int2.data_ready_accel as u8 => 0, 1
        });
        log_reg!("INT2_CTRL", int2);
        self.write_register(Register::INT2_CTRL as u8, int2).await?;

        let c1 = encode_reg8!({
            imu_config.accel.mode as u8 => 4, 3,
            imu_config.accel.odr as u8 => 0, 4
        });
        log_reg!("CTRL1", c1);
        self.write_register(Register::CTRL1 as u8, c1).await?;

        let c2 = encode_reg8!({
            imu_config.gyro.mode as u8 => 4, 3,
            imu_config.gyro.odr as u8 => 0, 4
        });
        log_reg!("CTRL2", c2);
        self.write_register(Register::CTRL2 as u8, c2).await?;

        let c4_old = self.read_register(Register::CTRL4 as u8).await?;
        let c4 = encode_reg8!(base: c4_old, {
            imu_config.int2.temp_ready as u8 => 2, 1
        });
        log_reg!("CTRL4", c4_old, c4);
        self.write_register(Register::CTRL4 as u8, c4).await?;

        let c6 = encode_reg8!({
            imu_config.gyro.lpf1 as u8 => 4, 3,
            imu_config.gyro.full_scale as u8 => 0, 4
        });
        log_reg!("CTRL6", c6);
        self.write_register(Register::CTRL6 as u8, c6).await?;

        let c7 = encode_reg8!({ imu_config.gyro.lpf1_enabled as u8 => 0, 1 });
        log_reg!("CTRL7", c7);
        self.write_register(Register::CTRL7 as u8, c7).await?;

        let c8 = encode_reg8!({
            imu_config.accel.lp_hp_f2 as u8 => 5, 3,
            imu_config.accel.dual_channel as u8 => 3, 1,
            1 => 2, 1,
            imu_config.accel.full_scale as u8 => 0, 2
        });
        log_reg!("CTRL8", c8);
        self.write_register(Register::CTRL8 as u8, c8).await?;

        let c9_old = self.read_register(Register::CTRL9 as u8).await?;
        let c9 = encode_reg8!(base: c9_old, {
            imu_config.accel.hp_reference_mode as u8 => 6, 1,
            imu_config.accel.lp_hp as u8 => 4, 1,
            imu_config.accel.lpf2_enabled as u8 => 3, 1,
            imu_config.accel.user_offset_weight as u8 => 1, 1,
            imu_config.accel.user_offset_en as u8 => 0, 1
        });
        log_reg!("CTRL9", c9_old, c9);
        self.write_register(Register::CTRL9 as u8, c9).await?;

        log_reg!("USER_OFFSET_X", imu_config.accel.user_offset[0] as u8);
        self.write_register(0x73, imu_config.accel.user_offset[0] as u8)
            .await?;
        log_reg!("USER_OFFSET_Y", imu_config.accel.user_offset[1] as u8);
        self.write_register(0x74, imu_config.accel.user_offset[1] as u8)
            .await?;
        log_reg!("USER_OFFSET_Z", imu_config.accel.user_offset[2] as u8);
        self.write_register(0x75, imu_config.accel.user_offset[2] as u8)
            .await?;

        let fe = encode_reg8!({
            imu_config.general.timestamp_enabled as u8 => 6, 1
        });
        log_reg!("FUNCTIONS_ENABLE", fe);
        self.write_register(Register::FUNCTIONS_ENABLE as u8, fe)
            .await?;

        let fc = encode_reg8!({
            imu_config.fifo.batch_dual as u8 => 7,1
        });
        log_reg!("FUNCTIONS_CONFIG", fc);
        self.write_register(Register::EMB_FUNC_CFG as u8, fc)
            .await?;

        if imu_config.high_accuracy_mode.is_some() {
            let haodr_cfg = encode_reg8!({
                imu_config.high_accuracy_mode.unwrap() as u8 => 0,2
            });
            log_reg!("HAODR_CONFIG", haodr_cfg);
            self.write_register(Register::HAODR_CFG as u8, haodr_cfg)
                .await?;
        }

        Ok(())
    }

    async fn write_register(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        const READ_BIT: u8 = 0x80;
        let cmd = reg & !READ_BIT; //Bit 7 = 0 -> Write-Mode -> 0x80: 1000 0000 -> !0x80: 0111 1111

        let mut buffer = [cmd, val];
        self.hw.cs.set_low();
        let mut spi = self.hw.spi.lock().await;
        let spi: &mut Spi<'_, Async> = &mut spi;
        spi.transfer_in_place(&mut buffer)
            .await
            .map_err(|e| Error::Spi(e))?;
        self.hw.cs.set_high();

        Ok(())
    }

    async fn read_register(&mut self, reg: u8) -> Result<u8, Error> {
        const READ_BIT: u8 = 0x80;
        let cmd = reg | READ_BIT;

        let mut buffer = [cmd, 0x00];

        self.hw.cs.set_low();
        let mut spi = self.hw.spi.lock().await;
        let spi: &mut Spi<'_, Async> = &mut spi;

        spi.transfer_in_place(&mut buffer)
            .await
            .map_err(|e| Error::Spi(e))?;
        self.hw.cs.set_high();

        if buffer.len() < 1 {
            return Err(Error::NoValue);
        } else {
            Ok(buffer[1])
        }
    }

    async fn read_multi_registers(&mut self, start_reg: u8, data: &mut [u8]) -> Result<(), Error> {
        const READ_BIT: u8 = 0x80;
        let cmd = start_reg | READ_BIT;

        let mut buffer = [0u8; 141]; //max 141 Bytes data = 20 FifoSamples
        let n = 1 + data.len();
        buffer[0] = cmd;

        self.hw.cs.set_low();
        let mut spi = self.hw.spi.lock().await;
        let spi: &mut Spi<'_, Async> = &mut spi;
        spi.transfer_in_place(&mut buffer[..n])
            .await
            .map_err(Error::Spi)?;
        self.hw.cs.set_high();

        data.copy_from_slice(&buffer[1..n]);
        Ok(())
    }
    /// Reads raw acceleration data from both channels
    /// Applies hardware bias offsets and returns (ch1, ch2) triplets
    pub async fn read_accel_dual_raw(&mut self) -> Result<([i16; 3], [i16; 3]), Error> {
        if self.config.accel.dual_channel == false || self.config.accel.odr == AccelODR::PowerDown {
            error!(
                "Dual channel accel reading requested but dual channel mode is disabled or accel is powered down"
            );
            return Err(Error::WrongConfig);
        }
        let mut data_ch1 = [0u8; 6];
        let mut data_ch2 = [0u8; 6];
        self.read_multi_registers(Register::OUTX_L_A as u8, &mut data_ch1)
            .await?;
        self.read_multi_registers(Register::UI_OUTX_L_A_OIS_DUAL_C as u8, &mut data_ch2)
            .await?;
        let res1 = [
            i16::from_le_bytes([data_ch1[0], data_ch1[1]]) - self.hw.bias_accel[0],
            i16::from_le_bytes([data_ch1[2], data_ch1[3]]) - self.hw.bias_accel[1],
            i16::from_le_bytes([data_ch1[4], data_ch1[5]]) - self.hw.bias_accel[2],
        ];
        let res2 = [
            i16::from_le_bytes([data_ch2[0], data_ch2[1]]) - self.hw.bias_accel_ch2[0],
            i16::from_le_bytes([data_ch2[2], data_ch2[3]]) - self.hw.bias_accel_ch2[1],
            i16::from_le_bytes([data_ch2[4], data_ch2[5]]) - self.hw.bias_accel_ch2[2],
        ];

        Ok((res1, res2))
    }

    /// Reads raw acceleration data and applies hardware bias offset
    pub async fn read_accel_raw(&mut self) -> Result<[i16; 3], Error> {
        if self.config.accel.odr == AccelODR::PowerDown {
            error!("Accel reading requested but accel is powered down");
            return Err(Error::WrongConfig);
        }
        let mut data = [0u8; 6];
        self.read_multi_registers(Register::OUTX_L_A as u8, &mut data)
            .await?;
        let ax = i16::from_le_bytes([data[0], data[1]]) - self.hw.bias_accel[0];
        let ay = i16::from_le_bytes([data[2], data[3]]) - self.hw.bias_accel[1];
        let az = i16::from_le_bytes([data[4], data[5]]) - self.hw.bias_accel[2];
        Ok([ax, ay, az])
    }

    /// Reads raw gyro data and applies hardware bias offset
    pub async fn read_gyro_raw(&mut self) -> Result<[i16; 3], Error> {
        if self.config.gyro.odr == GyroODR::PowerDown {
            error!("Gyro reading requested but gyro is powered down");
            return Err(Error::WrongConfig);
        }
        let mut data = [0u8; 6];
        self.read_multi_registers(Register::OUTX_L_G as u8, &mut data)
            .await?;
        let gx = i16::from_le_bytes([data[0], data[1]]) - self.hw.bias_gyro[0];
        let gy = i16::from_le_bytes([data[2], data[3]]) - self.hw.bias_gyro[1];
        let gz = i16::from_le_bytes([data[4], data[5]]) - self.hw.bias_gyro[2];
        Ok([gx, gy, gz])
    }

    /// Reads raw temperature data
    pub async fn read_temp_raw(&mut self) -> Result<i16, Error> {
        let mut data = [0u8; 2];
        self.read_multi_registers(Register::OUT_TEMP_L as u8, &mut data)
            .await?;
        let temp = i16::from_le_bytes([data[0], data[1]]);
        Ok(temp)
    }

    /// Reads raw timestamp data
    async fn read_timestamp_raw(&mut self) -> Result<u32, Error> {
        let mut data = [0u8; 4];
        self.read_multi_registers(Register::TIMESTAMP0 as u8, &mut data)
            .await?;
        Ok(u32::from_le_bytes(data))
    }

    async fn read_status_reg(&mut self) -> Result<ReadySRC, Error> {
        let status = self.read_register(Register::STATUS_REG as u8).await?;
        Ok(ReadySRC {
            accel: (status & 0b0000_0001) != 0,
            gyro: (status & 0b0000_0010) != 0,
            temp: (status & 0b0000_0100) != 0,
        })
    }
    /// Reads and scales timestamp to ns and applies hardware offset
    pub async fn read_timestamp(&mut self) -> Result<u64, Error> {
        if self.config.general.timestamp_enabled == false {
            error!("Timestamp reading requested but timestamp is disabled");
            return Err(Error::NoValue);
        }
        let raw = self.read_timestamp_raw().await?;
        Ok((raw - self.hw.ts_offset) as u64 * 21750)
    }

    /// Polls the status register until the specified sensor data is ready.
    ///
    /// ### Logic Modes:
    /// - [`LogicOp::AND`]: Waits until **all** selected sensors have new data.
    /// - [`LogicOp::OR`]: Waits until **at least one** of the selected sensors has new data.
    ///
    /// ### Parameters:
    /// - `accel`, `gyro`, `temp`: Selection of sensors to monitor.
    /// - `mode`: The [`LogicOp`] applied to the status bits.
    /// - `delay_us`: Polling interval (µs) to manage bus load and power.
    ///
    /// ### Return Type: [`Option<ReadySRC>`]
    /// Returns `Some` containing a [`ReadySRC`] struct with the **actual** status
    /// of each sensor
    ///
    /// ### Example:
    /// ```rust
    /// // Wait for ANY data to be ready
    /// if let Some(src) = imu.wait_for_data_ready_polling(true, true, false, LogicOp::OR, 500).await {
    ///     if src.gyro {
    ///         let g = imu.read_gyro_raw().await?;
    ///     }
    /// }
    /// ```
    pub async fn wait_for_data_ready_polling(
        &mut self,
        accel: bool,
        gyro: bool,
        temp: bool,
        mode: LogicOp,
        delay_us: u64,
    ) -> Option<ReadySRC> {
        if !accel && !gyro && !temp {
            error!("wait_for_data_ready called with no sensors selected");
            return None;
        }
        if (accel && self.config.accel.odr == AccelODR::PowerDown)
            || (gyro && self.config.gyro.odr == GyroODR::PowerDown)
        {
            error!("wait_for_data_ready called for powered down sensor");
            return None;
        }
        loop {
            let status = match self.read_status_reg().await {
                Ok(s) => s,
                Err(e) => {
                    error!("Status Register konnte nicht gelesen werden: {:?}", e);
                    return None;
                }
            };

            match mode {
                LogicOp::AND => {
                    if (!accel || status.accel) && (!gyro || status.gyro) && (!temp || status.temp)
                    {
                        return Some(ReadySRC {
                            gyro: gyro && status.gyro,
                            accel: accel && status.accel,
                            temp: temp && status.temp,
                        });
                    }
                }
                LogicOp::OR => {
                    if (accel && status.accel) || (gyro && status.gyro) || (temp && status.temp) {
                        return Some(ReadySRC {
                            gyro: gyro && status.gyro,
                            accel: accel && status.accel,
                            temp: temp && status.temp,
                        });
                    }
                }
            }

            Timer::after(embassy_time::Duration::from_micros(delay_us)).await
        }
    }

    /// Orchestrates data polling and execution of a user callback.
    ///
    /// - **Polling**: Uses [`wait_for_data_ready_polling`](Self::wait_for_data_ready_polling) with [`LogicOp`]
    /// - **Auto-Retrieval**: Fetches [`ImuSample`] data based on `data_ready` flags
    /// - **Closure**: Executes `on_sample` with the populated sample
    ///
    /// ### Parameters
    /// - `accel`, `gyro`, `temp`: Sensors to monitor and read.
    /// - `ts`: If `true`, includes a timestamp in the sample via [`read_timestamp`](Self::read_timestamp).
    /// - `delay_us`: Polling interval in microseconds.
    /// - `mode`: The [`LogicOp`] (AND/OR) used to trigger the read.
    /// - `on_sample`: A closure that implements `FnMut(ImuSample)` handeling the resulting data.
    ///
    /// ### Returns
    /// - `Ok(())` if the poll was successful and the callback was executed.
    /// - `Err(Error::NoValue)` if the polling condition was not met or parameters were invalid.
    ///
    /// ### Example: Logging Accel and Gyro data
    /// ```rust
    /// imu.read_data_when_ready_polling(
    ///     true, true, false, // Accel & Gyro
    ///     true,              // Include timestamp
    ///     1000,              // Check every 1ms
    ///     LogicOp::AND,      // Wait until BOTH are ready
    ///     |sample| {
    ///         info!("New Sample at {}: Accel={:?}, Gyro={:?}", sample.last_ts, sample.accel, sample.gyro);
    ///     }
    /// ).await?;
    /// ```
    pub async fn read_data_when_ready_polling<F>(
        &mut self,
        accel: bool,
        gyro: bool,
        temp: bool,
        ts: bool,
        delay_us: u64,
        mode: LogicOp,
        mut on_sample: F,
    ) -> Result<(), Error>
    where
        F: FnMut(ImuSample),
    {
        let rdy = self
            .wait_for_data_ready_polling(accel, gyro, temp, mode, delay_us)
            .await;
        if rdy.is_none() {
            return Err(Error::NoValue);
        }
        if let Some(data_ready) = rdy {
            let mut sample: ImuSample = Default::default();
            if ts {
                sample.last_ts = self.read_timestamp().await.unwrap_or(0) as u64;
            }
            if data_ready.accel
                && self.config.accel.odr != AccelODR::PowerDown
                && !self.config.accel.dual_channel
            {
                sample.accel = self.read_accel_raw().await?;
            }
            if data_ready.accel
                && self.config.accel.odr != AccelODR::PowerDown
                && self.config.accel.dual_channel
            {
                (sample.accel, sample.accel_ch2) = self.read_accel_dual_raw().await?;
            }
            if data_ready.gyro && self.config.gyro.odr != GyroODR::PowerDown {
                sample.gyro = self.read_gyro_raw().await?;
            }
            if data_ready.temp && self.config.general.timestamp_enabled {
                sample.temp = self.read_temp_raw().await?;
            }
            on_sample(sample);
        }

        Ok(())
    }

    /// Syncs the internal time reference to the current sensor timestamp.
    /// Sets [`ts_offset`](Self::hw) to align future reads with this point in time.
    pub async fn reset_timer(&mut self) {
        info!("Resetting timestamp timer");
        self.hw.ts_offset = self.read_timestamp_raw().await.unwrap_or(0);
    }

    /// Finalizes and applies all cached settings to the physical device.
    /// Builds the configuration bitmask and writes it to the sensor registers.
    pub async fn commit_config(&mut self) {
        if let Err(e) = self.write_config_registers(&self.config.build()).await {
            error!("Error committing config: {:?}", e);
        }
    }
}

impl<'d, I1, I2> Lsm6dsv32<'d, FifoDisabled, I1, I2> {
    /// Enables Fifo by consuming `self` and changing state to [`FifoEnabled`]
    pub fn enable_fifo(self) -> Lsm6dsv32<'d, FifoEnabled, I1, I2> {
        Lsm6dsv32 {
            hw: self.hw,
            config: ImuConfig {
                fifo_state: FifoEnabled,
                int1_state: self.config.int1_state,
                int2_state: self.config.int2_state,
                general: self.config.general,
                accel: self.config.accel,
                gyro: self.config.gyro,
                fifo: self.config.fifo,
                int1: self.config.int1,
                int2: self.config.int2,
                high_accuracy_mode: self.config.high_accuracy_mode,
            },
        }
    }
}

impl<'d, I1, I2> Lsm6dsv32<'d, FifoEnabled, I1, I2> {
    /// Returns the current FIFO status as [`FifoStatusSample`]
    pub async fn read_fifo_status(&mut self) -> Result<FifoStatusSample, Error> {
        let fifo_status1 = self.read_register(Register::FIFO_STATUS1 as u8).await?;
        let fifo_status2 = self.read_register(Register::FIFO_STATUS2 as u8).await?;
        Ok(FifoStatusSample::from_registers(fifo_status1, fifo_status2))
    }

    /// Waits until a specific FIFO condition is met.
    ///
    /// This function polls [`read_fifo_status`](Self::read_fifo_status) in a loop until the
    /// hardware signals that the requested [`FifoTrigger`] has been reached.
    ///
    /// ### Triggers:
    /// - [`FifoTrigger::Counter`]: Waits for a specific number of samples of one sensor (set in config).
    /// - [`FifoTrigger::Watermark`]: Waits until the fill level reaches the watermark threshold.
    /// - [`FifoTrigger::Full`]: Waits until the FIFO is completely saturated.
    pub async fn fifo_wait_for_polling(&mut self, trigger: FifoTrigger, delay_us: u64) {
        loop {
            let status = match self.read_fifo_status().await {
                Ok(s) => s,
                Err(e) => {
                    error!("FIFO Status konnte nicht gelesen werden: {:?}", e);
                    return;
                }
            };
            #[cfg(feature = "debug")]
            debug!("{:?}", status);
            match trigger {
                FifoTrigger::Counter => {
                    if self.config.fifo.counter_threshold == 0 {
                        error!("No Counter set");
                    }
                    if status.counter_reached {
                        return;
                    }
                }
                FifoTrigger::Watermark => {
                    if self.config.fifo.watermark_threshold == 0 {
                        error!("No watermark set");
                    }

                    if status.watermark_reached {
                        return;
                    }
                }
                FifoTrigger::Full => {
                    if status.full {
                        return;
                    }
                }
            }
            Timer::after(embassy_time::Duration::from_micros(delay_us)).await
        }
    }

    /// Orchestrates a complete FIFO readout cycle using polling.
    ///
    /// 1. **Wait**: Polls until the [`FifoTrigger`] is reached.
    /// 2. **Burst Read**: Transfers raw data packets via SPI in a single transaction.
    /// 3. **Parsing**: Decodes 7-byte tags (Gyro, Accel, Temp, TS, Dual-CH).
    /// 4. **Callbacks**: Emits [`ImuSample`] via `on_sample` and temperature via `on_temp`.
    ///
    /// ### Example
    /// ```rust
    /// imu.fifo_read_data_polling(FifoTrigger::Watermark, 1000, false,
    ///     |s| info!("Sample: {:?}", s),
    ///     Some(|t| info!("Temp: {}", t))
    /// ).await?;
    /// ```
    pub async fn fifo_read_data_polling<F, G>(
        &mut self,
        trigger: FifoTrigger,
        delay_us: u64,
        dual: bool,
        mut on_sample: F,
        mut on_temp: Option<G>,
    ) -> Result<(), Error>
    where
        F: FnMut(ImuSample),
        G: FnMut(i16),
    {
        let current_ts = self.read_timestamp().await?;
        let mut sampler = Sampler::new(
            dual,
            current_ts,
            self.config.fifo.accel_fifo,
            self.config.fifo.gyro_fifo,
        );
        self.fifo_wait_for_polling(trigger, delay_us).await;
        loop {
            let status = self.read_fifo_status().await?;
            let num_samples = status.unread_samples;

            if num_samples == 0 {
                return Ok(());
            }

            let mut raw_buffer = [0u8; 210];

            self.read_fifo_burst(&mut raw_buffer).await?;

            for chunk in raw_buffer.chunks_exact(7) {
                let tag = chunk[0] >> 3;
                let cnt = chunk[0] & 0b11;
                let data = &chunk[1..7];

                match tag {
                    0x01 => {
                        let raw = [
                            i16::from_le_bytes([data[0], data[1]]) - self.hw.bias_gyro[0],
                            i16::from_le_bytes([data[2], data[3]]) - self.hw.bias_gyro[1],
                            i16::from_le_bytes([data[4], data[5]]) - self.hw.bias_gyro[2],
                        ];
                        sampler.set_gyro(raw, cnt);
                    }
                    0x02 => {
                        let raw = [
                            i16::from_le_bytes([data[0], data[1]]) - self.hw.bias_accel[0],
                            i16::from_le_bytes([data[2], data[3]]) - self.hw.bias_accel[1],
                            i16::from_le_bytes([data[4], data[5]]) - self.hw.bias_accel[2],
                        ];
                        sampler.set_accel(raw, cnt);
                    }
                    0x03 => {
                        if let Some(ref mut callback) = on_temp {
                            let raw_temp = i16::from_le_bytes([data[0], data[1]]);
                            callback(raw_temp);
                        }
                    }
                    0x04 => {
                        let raw_ts = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                        let last_ts = (raw_ts.wrapping_sub(self.hw.ts_offset) as u64) * 21750;
                        sampler.set_last_ts(last_ts);
                    }
                    0x1D => {
                        if dual {
                            let raw = [
                                i16::from_le_bytes([data[0], data[1]]) - self.hw.bias_accel_ch2[0],
                                i16::from_le_bytes([data[2], data[3]]) - self.hw.bias_accel_ch2[1],
                                i16::from_le_bytes([data[4], data[5]]) - self.hw.bias_accel_ch2[2],
                            ];
                            sampler.set_accel_ch2(raw);
                        } else {
                            continue;
                        }
                    }
                    0x00 => {
                        return Ok(());
                    } // FIFO leer
                    _ => {
                        warn!("tag: {}", tag)
                    }
                }
                if let Some(sample) = sampler.complete_sample() {
                    on_sample(sample);
                }
            }
        }
    }

    /// Performs a high-speed SPI burst read from the FIFO data output register.
    ///
    /// - **Efficiency**: Minimizes CS toggling by reading the entire buffer in one go.
    /// - **Buffer**: Expects a mutable data array (e.g., 210 bytes for 30 samples).
    async fn read_fifo_burst(&mut self, data: &mut [u8]) -> Result<(), Error> {
        const READ_BIT: u8 = 0x80;
        const FIFO_DATA_OUT_TAG: u8 = 0x78;
        let cmd = FIFO_DATA_OUT_TAG | READ_BIT;

        self.hw.cs.set_low();
        let mut cmd_buf = [cmd];
        let mut spi = self.hw.spi.lock().await;
        let spi: &mut Spi<'_, Async> = &mut spi;
        spi.transfer_in_place(&mut cmd_buf)
            .await
            .map_err(Error::Spi)?;

        for byte in data.iter_mut() {
            *byte = 0;
        }
        spi.transfer_in_place(data).await.map_err(Error::Spi)?;

        self.hw.cs.set_high();
        Ok(())
    }
    /// Consumes the driver to disable FIFO mode, resetting the state to [`FifoDisabled`]
    pub fn disable_fifo(self) -> Lsm6dsv32<'d, FifoDisabled, I1, I2> {
        Lsm6dsv32 {
            hw: self.hw,
            config: ImuConfig {
                fifo_state: FifoDisabled,
                int1_state: self.config.int1_state,
                int2_state: self.config.int2_state,
                general: self.config.general,
                accel: self.config.accel,
                gyro: self.config.gyro,
                fifo: FifoConfig::default(),
                int1: self.config.int1,
                int2: self.config.int2,
                high_accuracy_mode: self.config.high_accuracy_mode,
            },
        }
    }
}

impl<'d, FI, I2> Lsm6dsv32<'d, FI, Int1Disabled, I2> {
    /// Enables interrupt1 by consuming `self` and changing state to [`Int1Enabled`]
    pub fn enable_interrupt1(self) -> Lsm6dsv32<'d, FI, Int1Enabled, I2> {
        Lsm6dsv32 {
            hw: self.hw,
            config: ImuConfig {
                fifo_state: self.config.fifo_state,
                int1_state: Int1Enabled,
                int2_state: self.config.int2_state,
                general: self.config.general,
                accel: self.config.accel,
                gyro: self.config.gyro,
                fifo: self.config.fifo,
                int1: self.config.int1,
                int2: self.config.int2,
                high_accuracy_mode: self.config.high_accuracy_mode,
            },
        }
    }
}

impl<'d, FI, I1> Lsm6dsv32<'d, FI, I1, Int2Disabled> {
    /// Enables interrupt2 by consuming `self` and changing state to [`Int2Enabled`]
    pub fn enable_interrupt2(self) -> Lsm6dsv32<'d, FI, I1, Int2Enabled> {
        Lsm6dsv32 {
            hw: self.hw,
            config: ImuConfig {
                fifo_state: self.config.fifo_state,
                int1_state: self.config.int1_state,
                int2_state: Int2Enabled,
                general: self.config.general,
                accel: self.config.accel,
                gyro: self.config.gyro,
                fifo: self.config.fifo,
                int1: self.config.int1,
                int2: self.config.int2,
                high_accuracy_mode: self.config.high_accuracy_mode,
            },
        }
    }
}

impl<'d, FI, I2> Lsm6dsv32<'d, FI, Int1Enabled, I2> {
    /// Waits until the INT1 pin signals that data is ready
    ///
    /// - **Hardware-Driven**: Suspends the task until an edge (rising/falling) is detected on INT1
    /// - **Logic**: Returns [`ReadySRC`] once the [`LogicOp`] condition matches the status register
    pub async fn wait_for_data_ready_interrupt1(
        &mut self,
        accel: bool,
        gyro: bool,
        temp: bool,
        mode: LogicOp,
    ) -> Result<ReadySRC, Error> {
        if !accel && !gyro && !temp {
            error!("wait_for_data_ready called with no sensors selected");
            return Err(Error::WrongConfig);
        }
        if (accel && self.config.accel.odr == AccelODR::PowerDown)
            || (gyro && self.config.gyro.odr == GyroODR::PowerDown)
        {
            error!("wait_for_data_ready called for powered down sensor");
            return Err(Error::WrongConfig);
        }
        loop {
            if self.config.general.interrupt_lvl == false {
                self.hw.int1.wait_for_rising_edge().await;
                #[cfg(feature = "debug")]
                debug!("Interrupt erkannt")
            } else {
                self.hw.int1.wait_for_falling_edge().await;
                #[cfg(feature = "debug")]
                debug!("Interrupt erkannt")
            }
            let status = self.read_status_reg().await?;

            match mode {
                LogicOp::AND => {
                    if (!accel || status.accel) && (!gyro || status.gyro) && (!temp || status.temp)
                    {
                        return Ok(ReadySRC {
                            gyro: gyro && status.gyro,
                            accel: accel && status.accel,
                            temp: temp && status.temp,
                        });
                    }
                }
                LogicOp::OR => {
                    if (accel && status.accel) || (gyro && status.gyro) || (temp && status.temp) {
                        return Ok(ReadySRC {
                            gyro: gyro && status.gyro,
                            accel: accel && status.accel,
                            temp: temp && status.temp,
                        });
                    }
                }
            }
        }
    }

    /// High-level orchestrator for interrupt-driven data acquisition.
    ///
    /// - **Wait**: Uses [`wait_for_data_ready_interrupt1`](Self::wait_for_data_ready_interrupt1).
    /// - **Callback**: Reads data into an [`ImuSample`] and triggers the `on_sample` callback.
    ///
    /// ### Example
    /// ```rust
    /// // Wake up on INT1 and process Accel/Gyro data
    /// imu.read_data_when_ready_interrupt1(true, true, false, true, LogicOp::AND, |s| {
    ///     do_something(s);
    /// }).await?;
    /// ```
    pub async fn read_data_when_ready_interrupt1<F>(
        &mut self,
        accel: bool,
        gyro: bool,
        temp: bool,
        ts: bool,
        mode: LogicOp,
        mut on_sample: F,
    ) -> Result<(), Error>
    where
        F: FnMut(ImuSample),
    {
        let data_ready = self
            .wait_for_data_ready_interrupt1(accel, gyro, temp, mode)
            .await?;

        let mut sample: ImuSample = Default::default();
        if ts {
            sample.last_ts = self.read_timestamp().await.unwrap_or(0) as u64;
        }
        if data_ready.accel
            && self.config.accel.odr != AccelODR::PowerDown
            && !self.config.accel.dual_channel
        {
            sample.accel = self.read_accel_raw().await?;
        }
        if data_ready.accel
            && self.config.accel.odr != AccelODR::PowerDown
            && self.config.accel.dual_channel
        {
            (sample.accel, sample.accel_ch2) = self.read_accel_dual_raw().await?;
        }
        if data_ready.gyro && self.config.gyro.odr != GyroODR::PowerDown {
            sample.gyro = self.read_gyro_raw().await?;
        }
        if data_ready.temp && self.config.general.timestamp_enabled {
            sample.temp = self.read_temp_raw().await?;
        }
        on_sample(sample);
        Ok(())
    }
}

impl<'d, FI, I1> Lsm6dsv32<'d, FI, I1, Int2Enabled> {
    /// Waits until the INT2 pin signals that data is ready
    ///
    /// - **Hardware-Driven**: Suspends the task until an edge (rising/falling) is detected on INT2
    /// - **Logic**: Returns [`ReadySRC`] once the [`LogicOp`] condition matches the status register
    pub async fn wait_for_data_ready_interrupt2(
        &mut self,
        accel: bool,
        gyro: bool,
        temp: bool,
        mode: LogicOp,
    ) -> Result<ReadySRC, Error> {
        if !accel && !gyro && !temp {
            error!("wait_for_data_ready called with no sensors selected");
            return Err(Error::WrongConfig);
        }
        if (accel && self.config.accel.odr == AccelODR::PowerDown)
            || (gyro && self.config.gyro.odr == GyroODR::PowerDown)
        {
            error!("wait_for_data_ready called for powered down sensor");
            return Err(Error::WrongConfig);
        }
        loop {
            if self.config.general.interrupt_lvl == false {
                self.hw.int2.wait_for_rising_edge().await;
                #[cfg(feature = "debug")]
                debug!("Interrupt erkannt")
            } else {
                self.hw.int2.wait_for_falling_edge().await;
                #[cfg(feature = "debug")]
                debug!("Interrupt erkannt")
            }
            let status = self.read_status_reg().await?;

            match mode {
                LogicOp::AND => {
                    if (!accel || status.accel) && (!gyro || status.gyro) && (!temp || status.temp)
                    {
                        return Ok(ReadySRC {
                            gyro: gyro && status.gyro,
                            accel: accel && status.accel,
                            temp: temp && status.temp,
                        });
                    }
                }
                LogicOp::OR => {
                    if (accel && status.accel) || (gyro && status.gyro) || (temp && status.temp) {
                        return Ok(ReadySRC {
                            gyro: gyro && status.gyro,
                            accel: accel && status.accel,
                            temp: temp && status.temp,
                        });
                    }
                }
            }
        }
    }

    /// High-level orchestrator for interrupt-driven data acquisition.
    ///
    /// - **Wait**: Uses [`wait_for_data_ready_interrupt2`](Self::wait_for_data_ready_interrupt2).
    /// - **Callback**: Reads data into an [`ImuSample`] and triggers the `on_sample` callback.
    ///
    /// ### Example
    /// ```rust
    /// // Wake up on INT1 and process Accel/Gyro data
    /// imu.read_data_when_ready_interrupt2(true, true, false, true, LogicOp::AND, |s| {
    ///     do_something(s);
    /// }).await?;
    /// ```
    pub async fn read_data_when_ready_interrupt2<F>(
        &mut self,
        accel: bool,
        gyro: bool,
        temp: bool,
        ts: bool,
        mode: LogicOp,
        mut on_sample: F,
    ) -> Result<(), Error>
    where
        F: FnMut(ImuSample),
    {
        let data_ready = self
            .wait_for_data_ready_interrupt2(accel, gyro, temp, mode)
            .await?;

        let mut sample: ImuSample = Default::default();
        if ts {
            sample.last_ts = self.read_timestamp().await.unwrap_or(0) as u64;
        }
        if data_ready.accel
            && self.config.accel.odr != AccelODR::PowerDown
            && !self.config.accel.dual_channel
        {
            sample.accel = self.read_accel_raw().await?;
        }
        if data_ready.accel
            && self.config.accel.odr != AccelODR::PowerDown
            && self.config.accel.dual_channel
        {
            (sample.accel, sample.accel_ch2) = self.read_accel_dual_raw().await?;
        }
        if data_ready.gyro && self.config.gyro.odr != GyroODR::PowerDown {
            sample.gyro = self.read_gyro_raw().await?;
        }
        if data_ready.temp && self.config.general.timestamp_enabled {
            sample.temp = self.read_temp_raw().await?;
        }
        on_sample(sample);
        Ok(())
    }
}
