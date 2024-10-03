#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Channel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{self, Pull};
use embassy_rp::i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler};
use embassy_rp::peripherals::{I2C0, I2C1};
use embassy_time::{Delay, Duration, Timer};
// use embedded_dht_rs::dht20::Dht20;
use dht20::Dht20;
use gpio::{Level, Output};
use mcp9808::reg_conf::Configuration;
use mcp9808::reg_conf::ShutdownMode;
use mcp9808::reg_res::ResolutionVal;
use mcp9808::reg_temp_generic::*;
use mcp9808::MCP9808;

use embassy_rp::watchdog::*;

use {defmt_rtt as _, panic_probe as _};

const REFERENCE_VOLTAGE: f32 = 3.3;
const STEPS_12BIT: f32 = 4096 as f32;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Program start");
    let peripherals = embassy_rp::init(Default::default());

    {
        // I2C0 Setup for MCP9808
        info!("Starting mcp98080 setup on I2C0");

        let sda0 = peripherals.PIN_16;
        let scl0 = peripherals.PIN_17;

        let mut i2c0_config = I2cConfig::default();
        i2c0_config.frequency = 400_000;
        let i2c0 = I2c::new_async(peripherals.I2C0, scl0, sda0, Irqs, i2c0_config);
        // Start MCP9808 Temperature Sensor task
        info!("I2C Initialized");
        unwrap!(spawner.spawn(mcp9808_task(i2c0, "MCP9808")));
    }

    {
        // I2C1 Setup for DHT20 -- uses a 4.7k pull-up resistor to 3.3V
        info!("Starting I2C1 Setup");

        let sda1 = peripherals.PIN_14;
        let scl1 = peripherals.PIN_15;

        let mut i2c1_config = I2cConfig::default();
        i2c1_config.frequency = 400_000;
        let i2c1 = I2c::new_async(peripherals.I2C1, scl1, sda1, Irqs, i2c1_config);
        info!("I2C1 Initialized");

        // Start DHT20 Temperature Sensor task
        unwrap!(spawner.spawn(dht20_task(i2c1, "DHT20")));
    }

    {
        // ADC Setup
        info!("Starting ADC Setup");
        unwrap!(spawner.spawn(tmp36_adc_task(
            peripherals.ADC,
            peripherals.ADC_TEMP_SENSOR,
            peripherals.PIN_26
        )));
    }

    {
        // LED Setup -- Uses an LED on pin 22 with a 4.7k pull-down resistor to ground
        info!("Starting LED Setup");
        let external_led_pin = peripherals.PIN_22;
        unwrap!(spawner.spawn(external_pin_blinky(external_led_pin)));
    }

    {
        // Configure and start the watchdog timer
        info!("Starting watchdog Setup");
        let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
        watchdog.start(Duration::from_millis(1_500));
        info!("Watchdog timer started");
        unwrap!(spawner.spawn(watchdog_task(watchdog)));
    }
}

#[embassy_executor::task]
async fn watchdog_task(mut watchdog: Watchdog) {
    // basic watchdog task that will reset the system if this does not run every 1.5 seconds (based on input to task.)
    // Further logic can be added to check whether each of the tasks are running and if not, reset the system.

    let mut count = 0;
    loop {
        watchdog.feed();
        if count > 0 && count % 10 == 0 {
            info!("Watchdog is pretty hungry {} -- please feed her!", count);
        } else {
            info!("Watchdog count {}", count);
        }
        Timer::after(Duration::from_secs(1)).await;
        count += 1;
    }
}
/// Basic Celsius-to-Fahrenheit conversion
fn c_to_f(c: f32) -> f32 {
    (c * 9.0 / 5.0) + 32.0
}

/// Convert ADC binary value to a float voltage value.
///
/// The ADC has a 12-bit resolution of voltage, meaning that there
/// are 2^12 or 4096 unique levels from OFF (0V) to FULL (3V). This
/// function converts the ADC reading into a float measurement in volts.
fn adc_reading_to_voltage(reading_12bit: u16) -> f32 {
    (reading_12bit as f32 / STEPS_12BIT) * REFERENCE_VOLTAGE
}

/// Convert the voltage from the onboard temp sensor into a temp reading.
///
/// From §4.9.5 from the rp2040-datasheet.pdf, the temperature can be
/// approximated as T = 27 - (ADC_voltage - 0.706) / 0.001721.
fn chip_f(adc_reading: u16) -> f32 {
    let voltage: f32 = adc_reading_to_voltage(adc_reading);
    let c: f32 = 27.0 - ((voltage - 0.706) / 0.001721);
    c_to_f(c)
}

bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
    I2C1_IRQ => I2cInterruptHandler<I2C1>;
    ADC_IRQ_FIFO => AdcInterruptHandler;
});

#[cortex_m_rt::pre_init]
unsafe fn before_main() {
    // Soft-reset doesn't clear spinlocks. Clear the one used by critical-section
    // before we hit main to avoid deadlocks when using a debugger
    embassy_rp::pac::SIO.spinlock(31).write_value(1);
}

#[embassy_executor::task]
async fn external_pin_blinky(pin_22: embassy_rp::peripherals::PIN_22) {
    let mut led = Output::new(pin_22, Level::Low);

    loop {
        info!("LED: led is on.");
        led.set_high();
        Timer::after(Duration::from_secs(1)).await;

        info!("LED: led is off.");
        led.set_low();
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn tmp36_adc_task(
    adc: embassy_rp::peripherals::ADC,
    mut adc_temp_sensor: embassy_rp::peripherals::ADC_TEMP_SENSOR,
    mut pin_26: embassy_rp::peripherals::PIN_26,
) {
    // ADC Setup
    let mut adc = Adc::new(adc, Irqs, AdcConfig::default());
    let mut temp_channel = Channel::new_temp_sensor(&mut adc_temp_sensor);
    let mut adc_channel_pin26 = Channel::new_pin(&mut pin_26, Pull::None);

    loop {
        let chip_voltage_24bit: u16 = adc.read(&mut temp_channel).await.unwrap();
        let tmp36_voltage_24bit: u16 = adc.read(&mut adc_channel_pin26).await.unwrap();

        info!(
            "TMP36: Temperature readings: OnChip: {}°F, TMP36: {}°F",
            chip_f(chip_voltage_24bit),
            tmp36_f(tmp36_voltage_24bit)
        );
        Timer::after(Duration::from_secs(1)).await;
    }
}

/// Convert the voltage from a TMP36 sensor into a temperature reading.
///
/// The sensor returns 0.5V at 0°C and voltage changes ±0.01V for every
/// degree Celcius with higher temps resolting in higher voltages within
/// the range of -40°C to 125°C.
fn tmp36_f(adc_reading: u16) -> f32 {
    let voltage: f32 = adc_reading_to_voltage(adc_reading);
    let c = (100.0 * voltage) - 50.0;
    c_to_f(c)
}

#[embassy_executor::task]
async fn mcp9808_task(i2c: I2c<'static, I2C0, embassy_rp::i2c::Async>, name: &'static str) {
    let mut mcp9808 = MCP9808::new(i2c);
    let mut mcp_9808_conf = mcp9808.read_configuration().unwrap();
    mcp_9808_conf.set_shutdown_mode(ShutdownMode::Continuous);

    // static mut
    loop {
        let mcp9808_reading_c: f32 = mcp9808
            .read_temperature()
            .unwrap()
            .get_celsius(ResolutionVal::Deg_0_0625C);

        info!(
            "{}: MCP9808 Temperature: {}°F",
            name,
            c_to_f(mcp9808_reading_c)
        );

        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn dht20_task(i2c: I2c<'static, I2C1, embassy_rp::i2c::Async>, name: &'static str) {
    let delay = Delay;

    let mut sensor = Dht20::new(i2c /*platform specific i2c driver*/, 0x38, delay);

    loop {
        match sensor.read() {
            Ok(reading) => {
                let temperature = reading.temp;
                let humidity = reading.hum;

                info!(
                    "{}: DHT20 Temperature: {}°F, Humidity: {}%",
                    name,
                    c_to_f(temperature),
                    humidity
                );
            }
            Err(e) => {
                warn!("Error reading DHT20: {:?}", e);
            }
        }

        Timer::after(Duration::from_secs(1)).await;
    }
}
