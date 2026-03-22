#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::{Spawner, task};
use embassy_stm32::{
    Config, bind_interrupts, dma, exti::{self, ExtiInput}, gpio::{Level, Output, Pull, Speed}, interrupt::typelevel::{EXTI15_10, SPI1}, mode::Async, peripherals, rcc, spi::{self, Mode, Phase, Polarity, Spi, mode::Master as Spi_Master}, time::Hertz
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, with_timeout};
use lsm6dsv32::driver::*;
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    EXTI15_10 => exti::InterruptHandler<EXTI15_10>;
    DMA2_STREAM2 => dma::InterruptHandler<peripherals::DMA2_CH2>;
    DMA2_STREAM3 => dma::InterruptHandler<peripherals::DMA2_CH3>;
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
    spi_config.frequency = Hertz(3_000_000);
    // CPOL = takt im Ruhezustand (0-> Idle = LOW; 1 -> Idle = HIGH)
    // CPHA = an welcher Flanke werden die Daten gelesen (0-> erste Flanke )
    spi_config.mode = Mode {
        polarity: Polarity::IdleLow,            // CPOL=0
        phase: Phase::CaptureOnFirstTransition, // CPHA=0
    }; // => SPI Mode 0

    static SPI: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async, Spi_Master>>> =
        StaticCell::new();

    let spi = Spi::new(
        p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH2, Irqs, spi_config,
    );

    let spi_mutex = Mutex::new(spi);

    let spi = SPI.init(spi_mutex);

    let cs = Output::new(p.PB0, Level::High, Speed::VeryHigh);

    let int1 = ExtiInput::new(p.PA10, p.EXTI10, Pull::Down, Irqs);

    let int2 = ExtiInput::new(p.PA11, p.EXTI11, Pull::Down, Irqs);

    let mut lsm = Lsm6dsv32::new(spi, cs, int1, int2)
        .await
        .enable_interrupt1()
        .enable_interrupt2();
    unwrap!(
        lsm.config
            .accel
            .set_mode(AccelOperatingMode::HighPerformance)
    );
    unwrap!(lsm.config.gyro.set_mode(GyroOperatingMode::LowPowerMode));
    //lsm.config.use_high_accuracy_mode(HighAccuracyODR::Standard);
    lsm.config.accel.dual_channel = false;
    lsm.config.int1.data_ready_accel = true;
    let _ = lsm.config.accel.set_odr(AccelODR::Hz30);
    //let _ = lsm.config.gyro.set_odr(GyroODR::Hz7_5);
    lsm.config.accel.full_scale = AccelFS::G8;
    lsm.config.gyro.full_scale = GyroFS::DPS500;
    lsm.commit_config().await;
    lsm.send_sim_start().await;

    let mut sample_count = 0;
    const MAX_WRONG: usize = 30;
    let mut wrong_samples = [ImuSample::default(); MAX_WRONG];
    let mut wrong_stored_count = 0;
    let mut total_wrong_count = 0;
    let correct_sample: ImuSample = ImuSample {
        accel: [2570, 2570, 2570],
        accel_ch2: [0, 0, 0],
        gyro: [0, 0, 0],
        temp: 0,
        last_ts: 0,
        delta_ts: 0,
    };
    let warm_up_duration = Duration::from_secs(10);

    let _ = with_timeout(warm_up_duration, async {
        loop {
            let _ = lsm
                .read_data_when_ready_interrupt1(true, false, false, false, LogicOp::AND)
                .await;
        }
    })
    .await;
    info!("Starting measurement for 10 seconds...");
    let duration = Duration::from_secs(10);
    let start_time = Instant::now();

    let _ = with_timeout(duration, async {
        loop {
            match lsm
                .read_data_when_ready_interrupt1(true, false, false, false, LogicOp::AND)
                .await
            {
                Ok(sample) => {
                    if sample.check_sample(&correct_sample) {
                        sample_count += 1;
                    } else {
                        total_wrong_count += 1;
                        // Nur speichern, wenn noch Platz im Array ist
                        if wrong_stored_count < MAX_WRONG {
                            wrong_samples[wrong_stored_count] = sample;
                            wrong_stored_count += 1;
                        }
                    }
                }
                Err(e) => {
                    error!("Fehler beim Lesen der IMU: {:?}", e);
                }
            }
        }
    })
    .await;

    let elapsed = start_time.elapsed();
    info!("Finished");
    info!(
        "Result: {} correct and {} wrong samples received in {}ms.",
        sample_count,
        total_wrong_count,
        elapsed.as_millis()
    );
    if wrong_stored_count > 0 {
        info!("First {} faulty samples details:", wrong_stored_count);
        for i in 0..wrong_stored_count {
            let s = &wrong_samples[i];
            info!(
                "Sample {}: Accel: {:?}, Accel2: {:?}",
                i, s.accel, s.accel_ch2
            );
        }
    }
    let hz = (sample_count as f32) / (elapsed.as_millis() as f32 / 1000.0);
    info!("Average rate: {} Hz", hz);
}
