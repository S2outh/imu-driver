#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::{Spawner, task};
use embassy_stm32::{
    Config, Peri, bind_interrupts,
    exti::{self, ExtiInput},
    gpio::{Level, Output, Pull, Speed},
    interrupt::typelevel::EXTI9_5,
    mode::Async,
    peripherals, rcc,
    spi::{self, Mode, Phase, Polarity, Spi, mode::Master as Spi_Master},
    time::Hertz,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::Timer;
use lsm6dsv32::driver::*;
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    EXTI9_5 => exti::InterruptHandler<EXTI9_5>;
});

fn get_rcc_config() -> rcc::Config {
    let mut rcc_config = rcc::Config::default();
    rcc_config.hsi = Some(rcc::HSIPrescaler::DIV1);
    rcc_config.sys = rcc::Sysclk::HSI;
    rcc_config.pll1 = Some(rcc::Pll {
        source: rcc::PllSource::HSI,
        prediv: rcc::PllPreDiv::DIV8,
        mul: rcc::PllMul::MUL40,
        divp: None,
        divq: Some(rcc::PllDiv::DIV10),
        divr: Some(rcc::PllDiv::DIV5),
    });
    rcc_config.mux.fdcansel = rcc::mux::Fdcansel::PLL1_Q;
    rcc_config
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = get_rcc_config();
    let p = embassy_stm32::init(config);
    info!("Programm gestartet");

    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(500_000);
    // CPOL = takt im Ruhezustand (0-> Idle = LOW; 1 -> Idle = HIGH)
    // CPHA = an welcher Flanke werden die Daten gelesen (0-> erste Flanke )
    spi_config.mode = Mode {
        polarity: Polarity::IdleLow,            // CPOL=0
        phase: Phase::CaptureOnFirstTransition, // CPHA=0
    }; // => SPI Mode 0

    static SPI: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async, Spi_Master>>> =
        StaticCell::new();

    let spi = Spi::new(
        p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH2, spi_config,
    );

    let spi_mutex = Mutex::new(spi);

    let spi = SPI.init(spi_mutex);

    let cs = Output::new(p.PB1, Level::High, Speed::High);

    let int1 = ExtiInput::new(p.PA9, p.EXTI9, Pull::Down, Irqs);

    let int2 = ExtiInput::new(p.PA8, p.EXTI8, Pull::Down, Irqs);

    let mut lsm = Lsm6dsv32::new(spi, cs, int1, int2).await;
    lsm.config.use_high_accuracy_mode(HighAccuracyODR::Standard);
    lsm.config.accel.dual_channel = true;
    let _ = lsm.config.accel.set_odr(AccelODR::KHz1_92);
    lsm.config.accel.full_scale = AccelFS::G8;
    lsm.config.gyro.full_scale = GyroFS::DPS500;

    loop {
        lsm.commit_config().await;
        if let Err(e) = lsm.check_who_i_am().await {
            defmt::error!("WHO_AM_I Check failed: {:?}", e);

            // Optional: Spezifische Reaktion auf den Fehler
            match e {
                Error::WrongWhoAmI(val) => {
                    defmt::warn!("Sensor responded with 0x{:02x} instead of 0x70", val)
                }
                Error::Spi => defmt::error!("Hardware/Timing problem on SPI bus"),
                _ => {}
            }
            // Hier könntest du z.B. den Task neu starten oder in einen Fehlerzustand gehen
        } else {
            defmt::info!("WHO_AM_I check successful!");
        }
        Timer::after_secs(20).await;
    }

    spawner.spawn(send_iterupt(p.PB5)).unwrap();

    /*
    loop {

        if let Err(e) = lsm
            .read_data_when_ready_polling(true, false, false, true, 10, LogicOp::OR, |s_raw| {
                let s = s_raw.create_f32(scale);
                info!(
                    "IMU DATA -> Accel: {:?}, Accel2: {:?} Gyro: {:?}, TS: {}, deltaTS: {}",
                    s.accel, s.accel_ch2, s.gyro, s.last_ts, s.delta_ts
                );
            })
            .await
        {
            error!("Fehler");
        }

         let status = lsm.read_fifo_status().await.unwrap_or_else(|e|{error!("Fehler beim Lesen {:?}",e); FifoStatusSample::from_registers(0,   0)});
         info!("{}",status.unread_samples)

         let result = lsm.read_timestamp().await.unwrap_or_else(|e|{error!("Fehler beim Lesen {:?}",e); 0});
         info!("{}",result);
         Timer::after_millis(500).await;

         lsm.fifo_read_data_polling(FifoTrigger::Counter, 10, false,
             |se| {

             let s = se.create_f32(scale);
             //info!("IMU DATA -> Accel: {:?}, Accel2: {:?} Gyro: {:?}, TS: {}, deltaTS: {}", s.accel, s.accel_ch2,s.gyro, s.last_ts, s.delta_ts);
             },
             Some(|t| {
             info!("TEMP UPDATE -> {} °C", temp_f32(t));
             })
         ).await.unwrap_or_else(|e| error!("FIFO Error: {:?}", e));

    }
     */
}

#[task()]
pub async fn send_iterupt(pin: Peri<'static, peripherals::PB5>) {
    let mut sim_int_pin = Output::new(pin, Level::Low, Speed::Medium);
    info!("Test");
    loop {
        Timer::after_secs(5).await;
        sim_int_pin.set_high();
        info!("Int-Pin HIGH");
        Timer::after_millis(100).await;
        sim_int_pin.set_low();
        info!("Int-Pin LOW");
    }
}
