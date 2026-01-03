#![no_std]
#![no_main]

use lsm6dsv32::driver::*;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::{Spawner, task};
use embassy_stm32::{
    Peri, bind_interrupts,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    interrupt,
    mode::Async,
    pac::usb::vals::Stat,
    peripherals,
    spi::{Config, Instance, Mode, Phase, Polarity, Spi},
    time::Hertz,
    usart::{self, Config as UartConfig, Uart},
};
use embassy_time::Timer;
use panic_probe as _;
use static_cell::StaticCell;

type SpiError = embassy_stm32::spi::Error;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Programm gestartet");

    let mut spi_config = Config::default();
    spi_config.frequency = Hertz(500_000);
    // CPOL = takt im Ruhezustand (0-> Idle = LOW; 1 -> Idle = HIGH)
    // CPHA = an welcher Flanke werden die Daten gelesen (0-> erste Flanke )
    spi_config.mode = Mode {
        polarity: Polarity::IdleLow,            // CPOL=0
        phase: Phase::CaptureOnFirstTransition, // CPHA=0
    }; // => SPI Mode 0

    static SPI: StaticCell<Spi<'static, Async>> = StaticCell::new();
    static CS: StaticCell<Output<'static>> = StaticCell::new();
    static INT1: StaticCell<ExtiInput<'static>> = StaticCell::new();
    static INT2: StaticCell<ExtiInput<'static>> = StaticCell::new();

    let spi = SPI.init(Spi::new(
        p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH2, spi_config,
    ));

    let cs = CS.init(Output::new(p.PB0, Level::High, Speed::High));

    let int1 = INT1.init(ExtiInput::new(p.PA9, p.EXTI9, Pull::Down));

    let int2 = INT2.init(ExtiInput::new(p.PA8, p.EXTI8, Pull::Down));

    let mut lsm = Lsm6dsv32::new(spi, cs, int1, int2, true).await;
    lsm.commit_config().await;

    let scale = lsm.calc_scaling(UnitScale::Default);

    spawner.spawn(send_iterupt(p.PB5)).unwrap();

    let mut lsm = lsm.enable_fifo().enable_interrupt1();

    loop {
        lsm.wait_for_data_ready_interrupt1().await;
    }
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
    loop{
        Timer::after_secs(5).await;
        sim_int_pin.set_high();
        info!("Int-Pin HIGH");
        Timer::after_millis(100).await;
        sim_int_pin.set_low();
        info!("Int-Pin LOW");
    }
}
