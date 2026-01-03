use defmt::error;

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

#[derive(defmt::Format)]
pub enum Error {
    Spi(SpiError),
    NoValue,
    WrongWhoAmI(u8),
    WrongConfig,
}

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

#[derive(Clone, Debug)]
pub struct ImuConfig<Fifo, Int1, Int2> {
    pub fifo_state: Fifo,
    pub int1_state: Int1,
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
    pub fn use_high_accuracy_mode(&mut self, haodr: HighAccuracyODR) {
        self.high_accuracy_mode = Some(haodr);
        self.accel.ha_mode = true;
        self.gyro.ha_mode = true;
    }
}
#[derive(Clone, Debug, Copy)]
pub struct GenerelConfig {
    pub sda_pull_up: bool,
    pub sdo_pull_up: bool,
    pub anti_spike_filter: bool,
    pub interrupt_lvl: bool,
    pub interrupt_pin_mode: bool,
    pub timestamp_enabled: bool,
}

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
#[derive(Clone, Debug, Copy)]
pub struct GyroConfig {
    pub(crate) mode: GyroOperatingMode,
    pub(crate) odr: GyroODR,
    pub(crate) lpf1_enabled: bool,
    pub(crate) lpf1: GyroLpf1,
    pub full_scale: GyroFS,
    ha_mode:bool,
}

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
    pub fn set_mode(&mut self, mode: GyroOperatingMode) -> Result<(), Error> {
        if self.ha_mode && !(mode == GyroOperatingMode::HighAccuracy) {
            return Err(Error::WrongConfig);
        } else {
            self.mode = mode;
            Ok(())
        }
    }
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

    pub fn calc_scaling_factor(&self, unit_scale: UnitScale) -> f32 {
        let fs_value: f32 = match self.full_scale {
            GyroFS::DPS125 => 125.0,
            GyroFS::DPS250 => 250.0,
            GyroFS::DPS500 => 500.0,
            GyroFS::DPS1000 => 1000.0,
            GyroFS::DPS2000 => 2000.0,
            GyroFS::DPS4000 => 4000.0,
        };

        (fs_value * unit_scale.as_f32()) / 32768.0
    }
}
#[derive(Clone, Debug, Copy)]
pub struct AccelConfig {
    pub(crate) mode: AccelOperatingMode,
    pub(crate) odr: AccelODR,
    pub(crate) lp_hp_f2: AccelFilterBW,
    pub full_scale: AccelFS,
    pub(crate) lp_hp: bool,
    pub(crate) lpf2_enabled: bool,
    pub hp_reference_mode: bool,
    pub(crate) user_offset_en: bool,
    pub(crate) user_offset_weight: bool,
    pub(crate) user_offset: [i8; 3],
    pub dual_channel: bool,
    ha_mode: bool,
}

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
    pub fn calc_scaling_factor(&self, unit_scale: UnitScale) -> f32 {
        let fs_g = match self.full_scale {
            AccelFS::G4 => 4.0,
            AccelFS::G8 => 8.0,
            AccelFS::G16 => 16.0,
            AccelFS::G32 => 32.0,
        };

        (fs_g * unit_scale.as_f32()) / 32768.0
    }

    pub fn set_mode(&mut self, mode: AccelOperatingMode) -> Result<(), Error> {
        if self.ha_mode && !(mode == AccelOperatingMode::HighAccuracy) {
            return Err(Error::WrongConfig);
        } else {
            self.mode = mode;
            Ok(())
        }
    }

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

    pub fn activate_lpf2(&mut self, lpf2_bw: AccelFilterBW) {
        self.lp_hp_f2 = lpf2_bw;
        self.lpf2_enabled = true;
        self.lp_hp = false;
    }

    pub fn activate_hp(&mut self, hp_bw: AccelFilterBW) {
        self.lp_hp_f2 = hp_bw;
        self.lpf2_enabled = false;
        self.lp_hp = true;
    }
}
#[derive(Clone, Debug, Copy)]
pub struct FifoConfig {
    pub mode: FIFOMode,
    pub watermark_threshold: u8,
    pub(crate) counter_threshold: u16,
    pub(crate) counter_trigger: TriggerCounter,
    pub gyro_fifo: GyroBatchDataRate,
    pub accel_fifo: AccelBatchDataRate,
    pub temp_fifo: TempatureBatchRate,
    pub ts_fifo: TimeStampBatch,
    pub batch_dual: bool,
}

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
    pub fn set_bdr(&mut self,gyro: GyroBatchDataRate, accel: AccelBatchDataRate, temp: TempatureBatchRate, ts: TimeStampBatch) {
        self.gyro_fifo = gyro;
        self.accel_fifo = accel;
        self.temp_fifo = temp;
        self.ts_fifo = ts;
    }
    pub fn set_counter(&mut self, trigger: TriggerCounter, threshold: u16) {
        let th = threshold.min(1023);
        self.counter_threshold = th;
        self.counter_trigger = trigger;
    }
}

#[derive(Clone, Debug, Copy)]
pub struct Interrupt1Config {
    pub counter_bdr_int: bool,
    pub fifo_full_int: bool,
    pub fifo_overrun_int: bool,
    pub fifo_threshold_int: bool,
    pub data_ready_gyro: bool,
    pub data_ready_accel: bool,
}
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

#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum HighAccuracyODR {
    Standard = 0,
    ShiftedUp = 1,
    ShiftedDown = 2,
}

#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u32)]
pub enum UnitScale {
    Micro = 1_000_000,
    Milli = 1_000,
    Default = 1,
}

#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum ActivationLevel {
    ActiveHigh = 0,
    ActiveLow = 1,
}

#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum IntMode {
    PushPull = 0,
    OpenDrain = 1,
}

impl UnitScale {
    pub fn as_f32(&self) -> f32 {
        *self as u32 as f32
    }
}
// Gyro Output-Data-Rate
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

#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum GyroOperatingMode {
    HighPerformance,
    HighAccuracy,
    LowPowerMode,
}

// Rate at which gyro data fills the FIFO
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
// Accel Output-Data-Rate
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
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum AccelFS {
    G4 = 0b00,
    G8 = 0b01,
    G16 = 0b10,
    G32 = 0b11,
}
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
// Rate at which accel data fills the FIFO
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

// Configures decimation for timestamp batching in FIFO to control the write rate relative to the maximum sensor data rate
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum TimeStampBatch {
    NotBatched = 0b00,
    Every1 = 0b01,
    Every8 = 0b10,
    Every32 = 0b11,
}

// Rate at which temp data fills the FIFO
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum TempatureBatchRate {
    NotBatched = 0b00,
    Hz1_875 = 0b01,
    Hz15 = 0b10,
    Hz60 = 0b11,
}
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum FIFOMode {
    Bypass = 0b000,                 // FIFO disabled
    FifoMode = 0b001,               // Stops collecting data once the buffer is full
    ContinuousToFullMode = 0b010,   // Continuous mode until trigger, then fills buffer to capacity
    ContinuousToFifoMode = 0b011,   // Continuous mode until trigger, then switches to FifoMode
    BypassToContinuousMode = 0b100, // Fifo disabled until trigger, then starts continuous data collection
    ContinuousMode = 0b110,         // new samples overwrite the oldest once the fifo is full
    BypassToFifoMode = 0b111,       // Fifo disabled until trigger, then fills once and stops
}
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum TriggerCounter {
    Accel = 0b00,
    Gyro = 0b01,
}


#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]

pub struct FifoStatusSample {
    pub unread_samples: u16,
    pub watermark_reached: bool,
    pub overrun: bool,
    pub full: bool,
    pub counter_reached: bool,
    pub latched_overrun: bool,
}
impl FifoStatusSample {
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

impl ImuSample {
    pub fn create_f32(&self, scale: [f32; 3]) -> ImuSampleF32 {
        ImuSampleF32 {
            accel: self.accel.map(|val| val as f32 * scale[0]),
            accel_ch2: self.accel_ch2.map(|val| val as f32 * scale[2]),
            gyro: self.gyro.map(|val| val as f32 * scale[1]),
            temp: (self.temp as f32 / 256.0) + 25.0,
            last_ts: self.last_ts,
            delta_ts: self.delta_ts,
        }
    }
}

pub struct Sampler {
    accel: Option<[i16; 3]>,
    accel_ch2: Option<[i16; 3]>,
    gyro: Option<[i16; 3]>,
    last_ts: u64,

    current_ts: bool,
    need_ch2: bool,
    max_a_g: bool,

    sample_intervall: u64,
    delta_ts: u64,
    cnt_a: u8,
    cnt_g: u8,
}

impl Sampler {
    pub fn new(
        dual: bool,
        last_ts: u64,
        bdr_accel: AccelBatchDataRate,
        bdr_gyro: GyroBatchDataRate,
    ) -> Self {
        let max_bdr = bdr_accel.to_hz().max(bdr_gyro.to_hz());
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
            cnt_g: 5,
            sample_intervall: (10e9 as f32 / max_bdr) as u64,
            delta_ts: 0,
            max_a_g: who_max,
        }
    }
    pub fn set_accel(&mut self, data: [i16; 3], cnt: u8) {
        self.accel = Some(data);
        self.cnt_a = cnt;
        if self.max_a_g && !self.current_ts {
            self.delta_ts += self.sample_intervall;
        }
    }
    pub fn set_accel_ch2(&mut self, data: [i16; 3]) {
        self.accel_ch2 = Some(data);
    }
    pub fn set_gyro(&mut self, data: [i16; 3], cnt: u8) {
        self.gyro = Some(data);
        self.cnt_g = cnt;
        if !self.max_a_g && !self.current_ts {
            self.delta_ts += self.sample_intervall;
        }
    }
    pub fn set_last_ts(&mut self, data: u64) {
        self.last_ts = data;
        self.delta_ts = 0;
        self.current_ts = true;
    }
    pub fn complete_sample(&mut self) -> Option<ImuSample> {
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct FifoTag {
    pub tag_counter: u8,
    pub tag_sensor: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ReadySRC {
    pub gyro: bool,
    pub accel: bool,
    pub temp: bool,
}

impl ReadySRC {
    pub fn any(&self) -> bool {
        self.gyro || self.accel || self.temp
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogicOp {
    AND,
    OR,
}
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

impl<Fi, I1, I2> ImuConfig<Fi, I1, I2> {
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
