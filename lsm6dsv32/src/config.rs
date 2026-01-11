use core::f32::consts::PI;

use defmt::error;

/// creates marker structs for the different states of the driver
#[macro_export]
macro_rules! create_state_marker {
    (
        $($name:ident),*
    ) => {
        $(
            #[derive(Clone, Debug, Default, Copy)]
            pub struct $name;
        )*
    };
}

create_state_marker!(
    FifoDisabled,
    FifoEnabled,
    Int1Disabled,
    Int1Enabled,
    Int2Disabled,
    Int2Enabled
);
type SpiError = embassy_stm32::spi::Error;

/// Central error type for the LSM6DSV32 driver.
#[derive(defmt::Format)]
pub enum Error {
    /// SPI bus communication failure (e.g., timeout or hardware issue).
    Spi(SpiError),
    /// Requested data is not available (e.g., sensor is polling but no new data arrives).
    NoValue,
    /// Device ID mismatch: found the value in `u8`, but expected 0x70.
    WrongWhoAmI(u8),
    /// Invalid driver state (e.g., trying to read a sensor that is powered down).
    WrongConfig,
}

/// Register map for the LSM6DSV32 sensor, defining addresses for configuration, status, and output data
#[allow(non_camel_case_types)]
#[repr(u8)]
pub enum Register {
    PIN_CTRL = 0x02,
    IF_CTRL = 0x03,
    FIFO_CTRL1 = 0x07,
    FIFO_CTRL3 = 0x09,
    FIFO_CTRL4 = 0x0A,
    COUNTER_BDR_REG1 = 0x0B,
    COUNTER_BDR_REG2 = 0x0C,
    INT1_CTRL = 0x0D,
    INT2_CTRL = 0x0E,
    WHO_AM_I = 0x0F,
    CTRL1 = 0x10,
    CTRL2 = 0x11,
    CTRL4 = 0x13,
    CTRL6 = 0x15,
    CTRL7 = 0x16,
    CTRL8 = 0x17,
    CTRL9 = 0x18,
    FIFO_STATUS1 = 0x1B,
    FIFO_STATUS2 = 0x1C,
    STATUS_REG = 0x1E,
    OUT_TEMP_L = 0x20,
    OUTX_L_G = 0x22,
    OUTX_L_A = 0x28,
    UI_OUTX_L_G_OIS_EIS = 0x2E,
    UI_OUTX_L_A_OIS_DUAL_C = 0x34,
    TIMESTAMP0 = 0x40,
    FUNCTIONS_ENABLE = 0x50,
    MD1_CFG = 0x5E,
    MD2_CFG = 0x5F,
    EMB_FUNC_CFG = 0x63,
    HAODR_CFG = 0x62,
    FIFO_DATA_OUT_TAG = 0x78,
    FIFO_DATA_OUT_X_L = 0x79,
}

/// Internal storage for all sensor settings without states used for writing hardware registers
#[derive(Clone, Debug)]
pub struct ImuConfigRaw {
    pub(crate) general: GenerelConfig,
    pub(crate) accel: AccelConfig,
    pub(crate) gyro: GyroConfig,
    pub(crate) fifo: FifoConfig,
    pub(crate) int1: Interrupt1Config,
    pub(crate) int2: Interrupt2Config,
    pub(crate) high_accuracy_mode: Option<HighAccuracyODR>,
}

/// Active driver configuration tracking sensor state (using generics) and individual register settings
#[derive(Clone, Debug)]
pub struct ImuConfig<Fifo, Int1, Int2> {
    // Current state marker for Fifo
    pub fifo_state: Fifo,
    // Current state marker for Interrupt 1
    pub int1_state: Int1,
    // Current state marker for Interrupt 2
    pub int2_state: Int2,
    pub general: GenerelConfig,
    pub accel: AccelConfig,
    pub gyro: GyroConfig,
    pub fifo: FifoConfig,
    pub int1: Interrupt1Config,
    pub int2: Interrupt2Config,
    pub high_accuracy_mode: Option<HighAccuracyODR>,
}
impl<F, I1, I2> ImuConfig<F, I1, I2> {
    /// Activates high accuracy mode for both accel and gyro with specific ODR [`HighAccuracyODR`]
    pub fn use_high_accuracy_mode(&mut self, haodr: HighAccuracyODR) {
        self.high_accuracy_mode = Some(haodr);
        self.accel.ha_mode = true;
        self.gyro.ha_mode = true;
    }

    /// Builds [`ImuConfigRaw`] stripping type-state information
    pub fn build(&self) -> ImuConfigRaw {
        ImuConfigRaw {
            general: self.general,
            accel: self.accel,
            gyro: self.gyro,
            fifo: self.fifo,
            int1: self.int1,
            int2: self.int2,
            high_accuracy_mode: self.high_accuracy_mode,
        }
    }
}
/// General hardware settings for the LSM6DSV32.
#[derive(Clone, Debug, Copy)]
pub struct GenerelConfig {
    /// Enables internal pull-up resistor on the SDA/SDI pin.
    pub sda_pull_up: bool,
    /// Enables internal pull-up resistor on the SDO/SA0 pin.
    pub sdo_pull_up: bool,
    /// Activates a filter to suppress short glitches on the I2C/SPI lines.
    pub anti_spike_filter: bool,
    /// Sets interrupt polarity: `false` for Active High, `true` for Active Low.
    pub interrupt_lvl: bool,
    /// `false` for Push-Pull, `true` for Open-Drain (requires external pull-up).
    pub interrupt_pin_mode: bool,
    /// Activates the internal 24-bit timestamp counter.
    pub timestamp_enabled: bool,
}
/// DefaultConfig for the general hardware settings
///
/// - Internal pull-ups and filters disabled
/// - Interrupt pins are Active High and Push-Pull
/// - Timestamp counter enabled
impl Default for GenerelConfig {
    fn default() -> Self {
        Self {
            sda_pull_up: false,
            sdo_pull_up: false,
            anti_spike_filter: false,
            interrupt_lvl: false,
            interrupt_pin_mode: false,
            timestamp_enabled: true,
        }
    }
}
/// Configuration for the Gyroscope, including scaling and filter logic
#[derive(Clone, Debug, Copy)]
pub struct GyroConfig {
    pub(crate) mode: GyroOperatingMode,
    pub(crate) odr: GyroODR,
    pub(crate) lpf1_enabled: bool,
    pub(crate) lpf1: GyroLpf1,
    pub full_scale: GyroFS,
    ha_mode: bool,
}
/// DefaultConfig for the gyro settings
///
/// - High-Performance-Mode
/// - Powered down and filters disabled
/// - 250 dps
impl Default for GyroConfig {
    fn default() -> Self {
        Self {
            mode: GyroOperatingMode::HighPerformance,
            odr: GyroODR::PowerDown,
            lpf1_enabled: false,
            lpf1: GyroLpf1::Medium,
            full_scale: GyroFS::DPS250,
            ha_mode: false,
        }
    }
}

impl GyroConfig {
    /// Sets gyro operating mode
    /// - available when High AccuracyMode is not active
    pub fn set_mode(&mut self, mode: GyroOperatingMode) -> Result<(), Error> {
        if self.ha_mode && !(mode == GyroOperatingMode::HighAccuracy) {
            return Err(Error::WrongConfig);
        } else {
            self.mode = mode;
            Ok(())
        }
    }
    /// Sets gyro output data rate
    ///
    /// - **HighAccuracy**: Min 15Hz
    /// - **LowPower**: Max 240 Hz
    pub fn set_odr(&mut self, odr: GyroODR) -> Result<(), Error> {
        match self.mode {
            GyroOperatingMode::HighPerformance => self.odr = odr,
            GyroOperatingMode::HighAccuracy => {
                if (odr as u8) >= (GyroODR::Hz15 as u8) {
                    self.odr = odr;
                } else {
                    error!("HighAccuracyMode only available with an ODR above 15Hz");
                    return Err(Error::WrongConfig);
                }
            }
            GyroOperatingMode::LowPowerMode => {
                if (odr as u8) <= (GyroODR::Hz240 as u8) {
                    self.odr = odr;
                } else {
                    error!("LowPowerMode only available with an odr equal or less than 240Hz");
                    return Err(Error::WrongConfig);
                }
            }
        }
        Ok(())
    }
    /// Enables Low Pass Filter 1 with desired bandwidth
    /// - only available in High Accuracy Mode
    pub fn activate_lpf1(&mut self, gyro_bw: GyroLpf1) -> Result<(), Error> {
        if self.mode == GyroOperatingMode::HighPerformance {
            self.lpf1_enabled = true;
            self.lpf1 = gyro_bw;
            Ok(())
        } else {
            error!("lpf1 only available in high-performance-mode");
            return Err(Error::WrongConfig);
        }
    }
    /// Calculates the scaling factor to convert raw `i16` to radians per second (rad/s)
    pub fn calc_scaling_factor(&self) -> f32 {
        let fs_value: f32 = match self.full_scale {
            GyroFS::DPS125 => 125.0,
            GyroFS::DPS250 => 250.0,
            GyroFS::DPS500 => 500.0,
            GyroFS::DPS1000 => 1000.0,
            GyroFS::DPS2000 => 2000.0,
            GyroFS::DPS4000 => 4000.0,
        };

        (fs_value / 32768.0) * (PI / 180.0)
    }
}

/// Configuration for Accelerometer including scaling and filters
#[derive(Clone, Debug, Copy)]
pub struct AccelConfig {
    pub(crate) mode: AccelOperatingMode,
    pub(crate) odr: AccelODR,
    pub(crate) lp_hp_f2: AccelFilterBW,
    pub full_scale: AccelFS,
    pub(crate) lp_hp: bool,
    pub(crate) lpf2_enabled: bool,
    /// use the current acceleration as a reference point
    pub hp_reference_mode: bool,
    pub(crate) user_offset_en: bool,
    pub(crate) user_offset_weight: bool,
    pub(crate) user_offset: [i8; 3],
    pub dual_channel: bool,
    ha_mode: bool,
}
/// Default config for accelerometer
///
/// - High Performance Mode
/// - Powered Down, Filters, User offset and dual channel are disabled
/// - Full-scale: 8g
impl Default for AccelConfig {
    fn default() -> Self {
        Self {
            mode: AccelOperatingMode::HighPerformance,
            odr: AccelODR::PowerDown,
            lp_hp_f2: AccelFilterBW::OdrDiv20,
            full_scale: AccelFS::G8,
            lp_hp: false,
            lpf2_enabled: false,
            hp_reference_mode: false,
            user_offset_en: false,
            user_offset_weight: false,
            user_offset: [0i8; 3],
            dual_channel: false,
            ha_mode: false,
        }
    }
}

impl AccelConfig {
    /// Calculates the scaling factor to convert raw `i16` to m/s^2
    pub fn calc_scaling_factor(&self) -> f32 {
        let fs_g = match self.full_scale {
            AccelFS::G4 => 4.0,
            AccelFS::G8 => 8.0,
            AccelFS::G16 => 16.0,
            AccelFS::G32 => 32.0,
        };

        fs_g / 32768.0 * 9.80665
    }

    /// Calculates the scaling factor for second channel to convert raw `i16` to m/s^2
    pub fn calc_scaling_factor_ch2(&self) -> f32 {
        32.0 / 32768.0 * 9.80665
    }
    /// Sets accel operating mode
    /// - available when High AccuracyMode is not active
    pub fn set_mode(&mut self, mode: AccelOperatingMode) -> Result<(), Error> {
        if self.ha_mode && !(mode == AccelOperatingMode::HighAccuracy) {
            return Err(Error::WrongConfig);
        } else {
            self.mode = mode;
            Ok(())
        }
    }
    /// Sets accel output data rate
    /// - **HighPerformance**: Min 7.5Hz
    /// - **Normal**: 7.5Hz < ODR < 1.92kHz
    /// - **HighAccuracy**: Min 15Hz
    /// - **LowPower**: Max 240 Hz
    pub fn set_odr(&mut self, odr: AccelODR) -> Result<(), Error> {
        match self.mode {
            AccelOperatingMode::HighPerformance => {
                if (odr as u8) >= (AccelODR::Hz7_5 as u8) {
                    self.odr = odr;
                } else {
                    error!("HighAccuracyMode only available with an ODR above 7.5Hz");
                    return Err(Error::WrongConfig);
                }
            }
            AccelOperatingMode::Normal => {
                if (odr as u8) >= (AccelODR::Hz7_5 as u8)
                    || (odr as u8) <= (AccelODR::KHz1_92 as u8)
                {
                    self.odr = odr;
                } else {
                    error!("NormalMode only available with an ODR between 7.5Hz and 1.92kHz");
                    return Err(Error::WrongConfig);
                }
            }
            AccelOperatingMode::HighAccuracy => {
                if (odr as u8) >= (GyroODR::Hz15 as u8) {
                    self.odr = odr;
                } else {
                    error!("HighAccuracyMode only available with an ODR above 15Hz");
                    return Err(Error::WrongConfig);
                }
            }
            AccelOperatingMode::LowPowerMode2Mean
            | AccelOperatingMode::LowPowerMode4Mean
            | AccelOperatingMode::LowPowerMode8Mean => {
                if (odr as u8) <= (GyroODR::Hz240 as u8) {
                    self.odr = odr;
                } else {
                    error!("LowPowerMode only available with an odr equal or less than 240Hz");
                    return Err(Error::WrongConfig);
                }
            }
        }
        Ok(())
    }

    /// Enables Low-Pass Filter 2 (LPF2) with selected bandwidth
    pub fn activate_lpf2(&mut self, lpf2_bw: AccelFilterBW) {
        self.lp_hp_f2 = lpf2_bw;
        self.lpf2_enabled = true;
        self.lp_hp = false;
    }
    /// Enables High-Pass (HP) filter with selected bandwidth
    pub fn activate_hp(&mut self, hp_bw: AccelFilterBW) {
        self.lp_hp_f2 = hp_bw;
        self.lpf2_enabled = false;
        self.lp_hp = true;
    }
}
/// Configuration for internal Fifo
#[derive(Clone, Debug, Copy)]
pub struct FifoConfig {
    /// FIFO operation mode
    pub mode: FIFOMode,
    /// Number of samples (fill level) that triggers the watermark interrupt
    pub watermark_threshold: u8,
    pub(crate) counter_threshold: u16,
    pub(crate) counter_trigger: TriggerCounter,
    /// Batch rate for gyro data
    pub gyro_fifo: GyroBatchDataRate,
    /// Batch rate for accelerometer data
    pub accel_fifo: AccelBatchDataRate,
    /// Batch rate for temperature data
    pub temp_fifo: TempatureBatchRate,
    /// Batch rate for timestamps 
    pub ts_fifo: TimeStampBatch,
    /// enables batching for second accelerometer channel
    pub batch_dual: bool,
}
/// Defaul config for FIFO
/// - Bypass-Mode
/// - All batching disabled
impl Default for FifoConfig {
    fn default() -> Self {
        Self {
            mode: FIFOMode::Bypass,
            watermark_threshold: 0,
            counter_threshold: 0,
            counter_trigger: TriggerCounter::Accel,
            gyro_fifo: GyroBatchDataRate::NotBatched,
            accel_fifo: AccelBatchDataRate::NotBatched,
            temp_fifo: TempatureBatchRate::NotBatched,
            ts_fifo: TimeStampBatch::NotBatched,
            batch_dual: false,
        }
    }
}

impl FifoConfig {
    /// Configures the batch data rate (BDR) for all sensors and timestamp
    pub fn set_bdr(
        &mut self,
        gyro: GyroBatchDataRate,
        accel: AccelBatchDataRate,
        temp: TempatureBatchRate,
        ts: TimeStampBatch,
    ) {
        self.gyro_fifo = gyro;
        self.accel_fifo = accel;
        self.temp_fifo = temp;
        self.ts_fifo = ts;
    }
    /// Sets sample counter trigger and threshold 
    /// Counter increments based on the selected [`TriggerCounter`]
    pub fn set_counter(&mut self, trigger: TriggerCounter, threshold: u16) {
        let th = threshold.min(1023);
        self.counter_threshold = th;
        self.counter_trigger = trigger;
    }
}

/// Determines which hardware event triggers a signal on the INT1 pin
#[derive(Clone, Debug, Copy)]
pub struct Interrupt1Config {
    pub counter_bdr_int: bool,
    pub fifo_full_int: bool,
    pub fifo_overrun_int: bool,
    pub fifo_threshold_int: bool,
    pub data_ready_gyro: bool,
    pub data_ready_accel: bool,
}
/// Determines which hardware event triggers a signal on the INT2 pin
#[derive(Clone, Debug, Copy)]
pub struct Interrupt2Config {
    pub counter_bdr_int: bool,
    pub fifo_full_int: bool,
    pub fifo_overrun_int: bool,
    pub fifo_threshold_int: bool,
    pub data_ready_gyro: bool,
    pub data_ready_accel: bool,
    pub temp_ready: bool,
}
/// Configures the output data rate behaviour in High Accuracy Mode refering to user manual p. 33 table 20
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum HighAccuracyODR {
    Standard = 0,
    ShiftedUp = 1,
    ShiftedDown = 2,
}

/// Defines the electrical logic level of the interrupt signal
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum ActivationLevel {
    ActiveHigh = 0,
    ActiveLow = 1,
}
/// Defines the physical drive circuit of the interrupt pins
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum IntMode {
    PushPull = 0,
    OpenDrain = 1,
}

/// Gyro Output-Data-Rate
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum GyroODR {
    PowerDown = 0b0000,
    Hz7_5 = 0b0010,
    Hz15 = 0b0011,
    Hz30 = 0b0100,
    Hz60 = 0b0101,
    Hz120 = 0b0110,
    Hz240 = 0b0111,
    Hz480 = 0b1000,
    Hz960 = 0b1001,
    KHz1_92 = 0b1010,
    KHz3_84 = 0b1011,
    KHz7_68 = 0b1100,
}
/// Selection of gyro Low Pass Filter 1 bandwidths
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum GyroLpf1 {
    ExtraWide = 0b000,
    Wide = 0b001,
    MediumWide = 0b010,
    Medium = 0b011,
    MediumNarrow = 0b100,
    Narrow = 0b101,
    VeryNarrow = 0b110,
    UltraNarrow = 0b111,
}
/// Selection of gyro Full-Scale range in dps
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum GyroFS {
    DPS125 = 0b0000,
    DPS250 = 0b0001,
    DPS500 = 0b0010,
    DPS1000 = 0b0011,
    DPS2000 = 0b0100,
    DPS4000 = 0b1100,
}
/// Gyro power and precision modes
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum GyroOperatingMode {
    HighPerformance,
    HighAccuracy,
    LowPowerMode,
}

/// Frequency at which gyroscope data is pushed into the FIFO
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum GyroBatchDataRate {
    NotBatched = 0b0000,
    Hz7_5 = 0b0010,
    Hz15 = 0b0011,
    Hz30 = 0b0100,
    Hz60 = 0b0101,
    Hz120 = 0b0110,
    Hz240 = 0b0111,
    Hz480 = 0b1000,
    Hz960 = 0b1001,
    KHz1_92 = 0b1010,
    KHz3_84 = 0b1011,
    KHz7_68 = 0b1100,
}

impl GyroBatchDataRate {
    /// Converts the enum variant into its numerical frequency value in Hertz (Hz)
    pub fn to_hz(&self) -> f32 {
        match self {
            Self::NotBatched => 0.0,
            Self::Hz7_5 => 7.5,
            Self::Hz15 => 15.0,
            Self::Hz30 => 30.0,
            Self::Hz60 => 60.0,
            Self::Hz120 => 120.0,
            Self::Hz240 => 240.0,
            Self::Hz480 => 480.0,
            Self::Hz960 => 960.0,
            Self::KHz1_92 => 1920.0,
            Self::KHz3_84 => 3840.0,
            Self::KHz7_68 => 7680.0,
        }
    }
}
/// Accelerometer Output-Data-Rate
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum AccelODR {
    PowerDown = 0b0000,
    Hz1_875 = 0b0001,
    Hz7_5 = 0b0010,
    Hz15 = 0b0011,
    Hz30 = 0b0100,
    Hz60 = 0b0101,
    Hz120 = 0b0110,
    Hz240 = 0b0111,
    Hz480 = 0b1000,
    Hz960 = 0b1001,
    KHz1_92 = 0b1010,
    KHz3_84 = 0b1011,
    KHz7_68 = 0b1100,
}
/// Selection of accelerometer Filter bandwidths for both lp and hp
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum AccelFilterBW {
    OdrDiv4 = 0b000,
    OdrDiv10 = 0b001,
    OdrDiv20 = 0b010,
    OdrDiv45 = 0b011,
    OdrDiv100 = 0b100,
    OdrDiv200 = 0b101,
    OdrDiv400 = 0b110,
    OdrDiv800 = 0b111,
}
/// Selection of accelerometer full-scale range in g
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum AccelFS {
    G4 = 0b00,
    G8 = 0b01,
    G16 = 0b10,
    G32 = 0b11,
}
/// Accelerometer Power and Performance-Modes
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum AccelOperatingMode {
    HighPerformance = 0b000,
    HighAccuracy = 0b001,
    // takes mean value of x samples
    LowPowerMode2Mean = 0b100,
    LowPowerMode4Mean = 0b101,
    LowPowerMode8Mean = 0b110,
    Normal = 0b111,
}
/// Frequency at which gyroscope data is pushed into the FIFO
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum AccelBatchDataRate {
    NotBatched = 0b0000,
    Hz1_875 = 0b0001,
    Hz7_5 = 0b0010,
    Hz15 = 0b0011,
    Hz30 = 0b0100,
    Hz60 = 0b0101,
    Hz120 = 0b0110,
    Hz240 = 0b0111,
    Hz480 = 0b1000,
    Hz960 = 0b1001,
    KHz1_92 = 0b1010,
    KHz3_84 = 0b1011,
    KHz7_68 = 0b1100,
}

impl AccelBatchDataRate {
    /// Converts the enum variant into its numerical frequency value in Hertz (Hz)
    pub fn to_hz(&self) -> f32 {
        match self {
            Self::NotBatched => 0.0,
            Self::Hz1_875 => 1.875,
            Self::Hz7_5 => 7.5,
            Self::Hz15 => 15.0,
            Self::Hz30 => 30.0,
            Self::Hz60 => 60.0,
            Self::Hz120 => 120.0,
            Self::Hz240 => 240.0,
            Self::Hz480 => 480.0,
            Self::Hz960 => 960.0,
            Self::KHz1_92 => 1920.0,
            Self::KHz3_84 => 3840.0,
            Self::KHz7_68 => 7680.0,
        }
    }
}

/// Configures decimation for timestamp batching in FIFO to control the write rate relative to the maximum sensor data rate
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum TimeStampBatch {
    NotBatched = 0b00,
    Every1 = 0b01,
    Every8 = 0b10,
    Every32 = 0b11,
}

/// Rate at which temp data fills the FIFO
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum TempatureBatchRate {
    NotBatched = 0b00,
    Hz1_875 = 0b01,
    Hz15 = 0b10,
    Hz60 = 0b11,
}
/// Selection of the FIFO Operating Mode
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum FIFOMode {
    /// FIFO disabled
    Bypass = 0b000,     
    /// Stops collecting data once the buffer is full            
    FifoMode = 0b001,    
    /// Continuous mode until trigger, then fills buffer to capacity           
    ContinuousToFullMode = 0b010,
    /// Continuous mode until trigger, then switches to FifoMode   
    ContinuousToFifoMode = 0b011,   
    /// Fifo disabled until trigger, then starts continuous data collection
    BypassToContinuousMode = 0b100, 
    /// new samples overwrite the oldest once the fifo is full
    ContinuousMode = 0b110,  
    /// Fifo disabled until trigger, then fills once and stops       
    BypassToFifoMode = 0b111,       
}

/// Specifies which sensor event increments the internal FIFO sample counter
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum TriggerCounter {
    Accel = 0b00,
    Gyro = 0b01,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]

/// Current state of the FIFO buffer, parsed from the hardware status registers
pub struct FifoStatusSample {
    /// Number of unread samples stored in the FIFO
    pub unread_samples: u16,
    /// True if the fill level has reached the configured watermark threshold
    pub watermark_reached: bool,
    /// True if a FIFO overrun occurred (new data overwritten old data)
    pub overrun: bool,
    /// True if the FIFO is completely full (4096 bytes / 512 slots)
    pub full: bool,
    /// True if the Batch Data Rate (BDR) counter has reached its target
    pub counter_reached: bool,
    /// Sticky overrun flag; stays true until the FIFO is cleared or reset
    pub latched_overrun: bool,
}
impl FifoStatusSample {
    /// Decodes the FIFO_STATUS1 and FIFO_STATUS2 registers into a structured sample
    pub fn from_registers(status1: u8, status2: u8) -> Self {
        let unread = ((status2 as u16 & 0x01) << 8) | (status1 as u16);

        Self {
            unread_samples: unread,
            // Bit 7: FIFO_WTM_IA
            watermark_reached: (status2 & 0x80) != 0,
            // Bit 6: FIFO_OVR_IA
            overrun: (status2 & 0x40) != 0,
            // Bit 5: FIFO_FULL_IA
            full: (status2 & 0x20) != 0,
            // Bit 4: COUNTER_BDR_IA
            counter_reached: (status2 & 0x10) != 0,
            // Bit 3: FIFO_OVR_LATCHED
            latched_overrun: (status2 & 0x08) != 0,
        }
    }
}
/// Scaled IMU data in physical units (m/s², rad/s, °C)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSampleF32 {
    pub accel: [f32; 3],
    pub accel_ch2: [f32; 3],
    pub gyro: [f32; 3],
    pub temp: f32,
    pub last_ts: u64,
    pub delta_ts: u64,
}

impl Default for ImuSampleF32 {
    fn default() -> Self {
        Self {
            accel: [0f32; 3],
            accel_ch2: [0f32; 3],
            gyro: [0f32; 3],
            temp: 0f32,
            last_ts: 0,
            delta_ts: 0,
        }
    }
}
/// Raw IMU data as read directly from the sensor registers with offsets
pub struct ImuSample {
    pub accel: [i16; 3],
    pub accel_ch2: [i16; 3],
    pub gyro: [i16; 3],
    pub temp: i16,
    pub last_ts: u64,
    pub delta_ts: u64,
}

impl Default for ImuSample {
    fn default() -> Self {
        Self {
            accel: [0; 3],
            accel_ch2: [0; 3],
            gyro: [0; 3],
            temp: 0,
            last_ts: 0,
            delta_ts: 0,
        }
    }
}
// Aggregates asynchronous sensor data into synchronized [`ImuSample`] packets
// Data of all sensors are buffered until a matching set is complete
pub(crate) struct Sampler {
    accel: Option<[i16; 3]>,
    accel_ch2: Option<[i16; 3]>,
    gyro: Option<[i16; 3]>,
    // timestamp of the last processed sample
    last_ts: u64,
    // true if current data set has received a timestamp
    current_ts: bool,
    // true if Channel2 data is required
    need_ch2: bool,
    // true if Accel ODR is higher than Gyro ODR (used as master clock)
    max_a_g: bool,
    // calculated duration between samples in nanoseconds (based on max ODR)
    sample_intervall: u64,
    // Accumulated time difference to the last timestamp
    delta_ts: u64,
    // hardware counter for accel sample for sync
    cnt_a: u8,
    // hardware counter for gyro sample for sync
    cnt_g: u8,
}

impl Sampler {
    // Initializes a new sampler and determines master clock interval
    pub(crate) fn new(
        dual: bool,
        last_ts: u64,
        bdr_accel: AccelBatchDataRate,
        bdr_gyro: GyroBatchDataRate,
    ) -> Self {
        let max_bdr = bdr_accel.to_hz().max(bdr_gyro.to_hz());
        // identify which sensor dictates the timing
        let who_max = if max_bdr == bdr_accel.to_hz() {
            true
        } else {
            false
        };
        Self {
            accel: None,
            accel_ch2: None,
            gyro: None,
            last_ts: last_ts,
            need_ch2: dual,
            current_ts: false,
            cnt_a: 5,
            cnt_g: 6,
            sample_intervall: (10e9 as f32 / max_bdr) as u64,
            delta_ts: 0,
            max_a_g: who_max,
        }
    }
    // Updated accelerometer data
    pub(crate) fn set_accel(&mut self, data: [i16; 3], cnt: u8) {
        self.accel = Some(data);
        self.cnt_a = cnt;
        // accumlate interval if no fresh timestamp was provided
        if self.max_a_g && !self.current_ts {
            self.delta_ts += self.sample_intervall;
        }
    }
    // Update accelerometer ch2 data
    pub(crate) fn set_accel_ch2(&mut self, data: [i16; 3]) {
        self.accel_ch2 = Some(data);
    }
    // Update gyro data
    pub(crate) fn set_gyro(&mut self, data: [i16; 3], cnt: u8) {
        self.gyro = Some(data);
        self.cnt_g = cnt;
        // accumlate interval if no fresh timestamp was provided
        if !self.max_a_g && !self.current_ts {
            self.delta_ts += self.sample_intervall;
        }
    }
    // injects new timestamp and resets interpolation
    pub(crate) fn set_last_ts(&mut self, data: u64) {
        self.last_ts = data;
        self.delta_ts = 0;
        self.current_ts = true;
    }
    // Attempts to build complete sample with regard to completeness and synchronization
    pub(crate) fn complete_sample(&mut self) -> Option<ImuSample> {
        if self.cnt_a != self.cnt_g {
            return None;
        }

        let accel = self.accel.take()?;
        let gyro = self.gyro.take()?;

        let accel_ch2 = if self.need_ch2 {
            self.accel_ch2.take()?
        } else {
            [0i16; 3]
        };
        self.current_ts = false;
        Some(ImuSample {
            accel,
            accel_ch2,
            gyro,
            temp: 0,
            last_ts: self.last_ts,
            delta_ts: self.delta_ts,
        })
    }
}
/// Represents the 8-bit metadata tag attached to every FIFO dataset
/// Used to identify which sensor produced the data and its sequence position
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct FifoTag {
    /// 2 bit counter used to synchronize multiple data streams
    pub tag_counter: u8,
    /// 5 bit identifier indicating the sensor source
    pub tag_sensor: u8,
}
/// Status flags indicating which sensor has new data available
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ReadySRC {
    pub gyro: bool,
    pub accel: bool,
    pub temp: bool,
}

impl ReadySRC {
    /// Returns `true` if at least one sensor has new data
    pub fn any(&self) -> bool {
        self.gyro || self.accel || self.temp
    }
}
/// Logical Operator for combining multiple interrups or trigger conditions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogicOp {
    AND,
    OR,
}
/// Defines the hardware event that triggers a FIFO interrupt
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum FifoTrigger {
    Watermark,
    Counter,
    Full,
}

impl Default for ImuConfig<FifoDisabled, Int1Disabled, Int2Disabled> {
    fn default() -> Self {
        Self {
            fifo_state: FifoDisabled,
            int1_state: Int1Disabled,
            int2_state: Int2Disabled,
            general: GenerelConfig::default(),
            accel: AccelConfig::default(),
            gyro: GyroConfig::default(),
            fifo: FifoConfig::default(),
            int1: Interrupt1Config {
                counter_bdr_int: false,
                fifo_full_int: false,
                fifo_overrun_int: false,
                fifo_threshold_int: false,
                data_ready_gyro: false,
                data_ready_accel: false,
            },
            int2: Interrupt2Config {
                counter_bdr_int: false,
                fifo_full_int: false,
                fifo_overrun_int: false,
                fifo_threshold_int: false,
                data_ready_gyro: false,
                data_ready_accel: false,
                temp_ready: false,
            },
            high_accuracy_mode: None,
        }
    }
}
