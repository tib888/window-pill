//! Radiator valve motor sense on A0, A1 (floating or pull down input)
//!
//! Roll up switches A2 (pull down once in each main period if closed)
//! Roll down switches A3 (pull down once in each main period if closed)
//!
//! Ext/Motion alarm on A4 (pull down)
//! Open alarm on A5 (pull down)
//!
//! A6 - ADC6 valve motor driver current sense shunt
//! A7 - ADC7 roll motor hall current sense
//!
//! Optional piezzo speaker on A8 (open drain output)
//!
//! Solid state relay connected to A9 drives the ssr_roll_down (push pull output)
//! Solid state relay connected to A10 drives the ssr_roll_up (push pull output)
//!
//! CAN (RX, TX) on A11, A12
//!
//! Read the NEC IR remote commands on A15 GPIO as input with internal pullup
//!
//! Photoresistor on B0 (ADC8)
//!
//! AC main voltage sense on B1 (ADC9)
//!
//! B3 not used / ITM debug output
//!
//! DS18B20 1-wire temperature sensors connected to B4 GPIO
//! JTAG is removed from B3, B4 to make it work
//!
//! B5 not used, connected to the ground
//!
//! B6, B7 reserved for USART debug (or anything else)
//!
//! B8, B9 reserved for PWM valve motor control if connected with B10 B11
//! Radiator valve motor driver on B10, B11 (push pull output, or pwm from B8, B9)
//!
//! B12 not used, connected to the ground
//!
//! RGB led on PB13, PB14, PB15 as push pull output
//!
//! C13 on board LED
//!
//! C14, C15 used on the bluepill board for 32768Hz xtal
//!

//#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::{entry, exception, ExceptionFrame};

use embedded_hal::{
	digital::v2::{InputPin, OutputPin},	
};

use stm32f1xx_hal;
use stm32f1xx_hal::{
	adc,
	can::*,
	delay::Delay,
	prelude::*,
	pwm::Channel,
	rtc,
	serial::{Config, Serial},
	timer::Timer,	
};

use window_pill::{
	beeper::Beeper,
	shutter,
	shutter::Shutter,
};

use onewire::{ds18x20::*, *};
use room_pill::{
	ac_sense::AcSense,
	ac_switch::*,
	ir,
	ir::NecReceiver,
	ir_remote::*,
	messenger::{Messenger, ID_MOVEMENT, ID_OPEN},
	rgb::{Colors, RgbLed, Rgb},
	timing::{Ticker, TimeExt},
};

use cortex_m_semihosting::hprintln;

static START_MELODY: [(u16, u16); 5] = [(440, 100), (0, 100), (880, 100), (0, 100), (440, 100)];
static OPEN_MELODY: [(u16, u16); 5] = [(440, 100), (0, 100), (440, 100), (0, 100), (880, 1000)];
static MOTION_MELODY: [(u16, u16); 5] = [(880, 100), (0, 100), (800, 100), (0, 100), (440, 1000)];

type Duration = room_pill::timing::Duration<u32, room_pill::timing::MicroSeconds>;

#[entry]
fn main() -> ! {
	window_unit_main();
}

fn window_unit_main() -> ! {
	let device = stm32f1xx_hal::pac::Peripherals::take().unwrap();
	//10 period at 50Hz, 12 period at 60Hz
	let ac_test_period = TimeExt::us(200_000);
	let one_sec = TimeExt::us(1_000_000);
	let mut flash = device.FLASH.constrain();

	//flash.acr.prftbe().enabled();//?? Configure Flash prefetch - Prefetch buffer is not available on value line devices
	//scb().set_priority_grouping(NVIC_PRIORITYGROUP_4);

	let mut rcc = device.RCC.constrain();
	let clocks = rcc
		.cfgr
		.use_hse(8.mhz())
		.sysclk(72.mhz())
		.hclk(72.mhz())
		.pclk1(36.mhz())
		.pclk2(72.mhz())
		.adcclk(9.mhz()) //ADC clock: PCLK2 / 8. User specified value is be approximated using supported prescaler values 2/4/6/8.
		.freeze(&mut flash.acr);

	let mut core = cortex_m::Peripherals::take().unwrap();
	#[cfg(feature = "itm-debug")]
	let _stim = &mut core.ITM.stim[0];

	// real time clock
	let _rtc = {
		let mut pwr = device.PWR;
		let mut backup_domain = rcc.bkp.constrain(device.BKP, &mut rcc.apb1, &mut pwr);
		rtc::Rtc::rtc(device.RTC, &mut backup_domain)
	};

	// A/D converter
	let mut adc1 = adc::Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);

	let mut afio = device.AFIO.constrain(&mut rcc.apb2);

	//let _dma_channels = device.DMA1.split(&mut rcc.ahb);

	//configure pins:
	let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
	let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
	let mut gpioc = device.GPIOC.split(&mut rcc.apb2);

	// Disables the JTAG to free up pb3, pb4 and pa15 for normal use
	let (pa15, _pb3_itm_swo, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

	// -------- Roll control related
	#[cfg(feature = "shutter-switch")]
	let (mut switch_roll_up, mut switch_roll_down) = {
		// Roll up switches A2 (pull down once in each main period if closed)
		(
			AcSwitch::new(gpioa.pa2.into_pull_up_input(&mut gpioa.crl), ac_test_period),
			// Roll down switches A3 (pull down once in each main period if closed)
			AcSwitch::new(gpioa.pa3.into_pull_up_input(&mut gpioa.crl), ac_test_period),
		)
	};

	// Roll motor hall current sense on A7 (ADC7)
	let mut adc7_roll_motor_current_sense = gpioa.pa7.into_analog(&mut gpioa.crl);
	// AC main voltage sense on B1 (ADC9)
	let mut adc9_ac_main_voltage = gpiob.pb1.into_analog(&mut gpiob.crl);
	let mut roll_power_meter = AcSense::<Duration>::new(ac_test_period, true); //0.1sec to be able to wait the first current measurement at motor start

	// Solid state relay connected to A9 drives the ssr_roll_down (push pull output)
	// Solid state relay connected to A10 drives the ssr_roll_up (push pull output)
	let mut shutter = Shutter::new(
		one_sec,
		gpioa.pa9.into_push_pull_output(&mut gpioa.crh),
		gpioa.pa10.into_push_pull_output(&mut gpioa.crh),
	);

	// Photoresistor on B0 (ADC8)
	let mut adc8_photo_resistor = gpiob.pb0.into_analog(&mut gpiob.crl);

	// -------- Alarm related
	// Ext/Motion alarm on A4 (pull down)
	let motion_alarm = gpioa.pa4.into_pull_up_input(&mut gpioa.crl);

	// Open alarm on A5 (pull down)
	let open_alarm = gpioa.pa5.into_pull_up_input(&mut gpioa.crl);

	let mut messenger = {
		// CAN (RX, TX) on A11, A12
		let canrx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
		let cantx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);

		// USB is needed here because it can not be used at the same time as CAN since they share memory:
		Messenger::new(Can::can1(
			device.CAN1,
			(cantx, canrx),
			&mut afio.mapr,
			&mut rcc.apb1,
			device.USB,
		))
	};

	// -------- Heating control related:
	
	#[cfg(feature = "temp-sensor")]
	let mut one_wire = {
		// DS18B20 1-wire temperature sensors connected to B4 GPIO
		let onewire_io = pb4.into_open_drain_output(&mut gpiob.crl);
		let delay = Delay::new(core.SYST, clocks);
		OneWirePort::new(onewire_io, delay).unwrap()
	};

	// -------- Generic user interface related

	let mut piezzo = {
		// Optional piezzo speaker on A8 (open drain output)
		//let mut piezzo_pin = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);
		let piezzo_pin = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh); //TODO into_alternate_open_drain
		let mut piezzo = Timer::tim1(device.TIM1, &clocks, &mut rcc.apb2).pwm(
			piezzo_pin,
			&mut afio.mapr,
			1.khz(),
		); //pwm::<Tim1NoRemap, _, _, _>
		piezzo.set_duty(Channel::C1, piezzo.get_max_duty() / 2);
		//piezzo.set_period(1.ms());
		piezzo.disable(Channel::C1);
		piezzo
	};

	// Read the NEC IR remote commands on A15 GPIO as input with internal pullup
	let ir_receiver = pa15.into_pull_up_input(&mut gpioa.crh);
	let mut receiver = ir::IrReceiver::new();

	// RGB led on PB13, PB14, PB15 as open drain (or push pull) output
	let mut rgb = RgbLed::new(
		gpiob.pb13.into_open_drain_output(&mut gpiob.crh),
		gpiob.pb15.into_open_drain_output(&mut gpiob.crh),
		gpiob.pb14.into_open_drain_output(&mut gpiob.crh),
	);
	rgb.color(Colors::White).unwrap();

	// C13 on board LED^
	let mut led = gpioc.pc13.into_open_drain_output(&mut gpioc.crh);
	led.set_high().unwrap(); //turn off

	let (mut _tx, _rx) = {
		//USART1
		let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
		let rx = gpiob.pb7;
		let serial = Serial::usart1(
			device.USART1,
			(tx, rx),
			&mut afio.mapr,
			Config::default().baudrate(2_000_000.bps()),
			clocks,
			&mut rcc.apb2,
		);
		serial.split()
	};

	// B3 not used, connected to the ground
	#[cfg(not(feature = "itm-debug"))]
	let _b3 = _pb3_itm_swo.into_pull_down_input(&mut gpiob.crl);

	#[cfg(feature = "itm-debug")]
	let _b3 = _pb3_itm_swo.into_push_pull_output(&mut gpiob.crl);

	// B5 not used, connected to the ground
	let _b5 = gpiob.pb5.into_pull_down_input(&mut gpiob.crl);

	// B12 not used, connected to the ground
	let _b12 = gpiob.pb12.into_pull_down_input(&mut gpiob.crh);

	// C14, C15 used on the bluepill board for 32768Hz xtal
	// -------- Init temperature measurement
	let address = {
		//store the addresses of temp sensor, start measurement
		let mut address = [0u8; 8];
		let mut it = RomIterator::new(0);

		#[cfg(feature = "temp-sensor")]
		loop {
			match one_wire.iterate_next(true, &mut it) {
				Ok(None) => {
					break; //no or no more devices found -> stop
				}

				Ok(Some(id)) => {
					if let Some(_device_type) = detect_18x20_devices(id[0]) {
						address = *id; //TODO use this address as unique id on the CAN bus?
						let _ = one_wire.start_temperature_measurement(&address);
						break;
					}
					continue;
				}

				Err(_e) => {
					rgb.color(Colors::Red).unwrap();
					break;
				}
			}
		}

		//not mutable anymore
		address
	};

	let dev_id = stm32_device_signature::device_id();
	let mut chk = 0u8; //"id checksum"
	for b in dev_id {
		chk ^= b;
	}

	hprintln!(
		"Unique ID: {} {} {:?}",
		chk,
		stm32_device_signature::device_id_hex(),
		address
	).unwrap();
	
	// -------- Config finished
	let mut shutter_power = 4000;
	let mut shutter_command = None;
	let mut lux = Option::<u32>::None;

	#[cfg(feature = "beeper")]
	let mut beeper = Beeper::new(&START_MELODY);

	let tick = Ticker::new(core.DWT, core.DCB, clocks);

	let mut last_time = tick.now();

	rgb.color(Colors::Black).unwrap();

	//main update loop
	loop {
		//piezzo_pin.toggle();

		// calculate the time since last execution:
		let now = tick.now();
		let delta = tick.to_us(now - last_time); //in case of 72MHz sysclock this works if less than 59sec passed between two calls
		last_time = now;

		//update the IR receiver statemachine:
		let ir_cmd = receiver.receive(ir_receiver.is_low().unwrap(), now, |last| {
			tick.to_us(now - last).into()
		});

		#[cfg(feature = "beeper")]
		beeper.update(
			delta,
			&|t| room_pill::timing::TimeExt::us(t as u32 * 1000),
			&mut |freq| {
				if freq != 0 {
					piezzo.set_period((freq as u32).hz());
					piezzo.enable(Channel::C1);
				} else {
					piezzo.disable(Channel::C1);
				}
			},
		);
		//process the infrared remote inputs:
		match ir_cmd {
			Ok(ir::NecContent::Repeat) => { led.toggle().unwrap(); }
			Ok(ir::NecContent::Data(data)) => {
				led.toggle().unwrap();
				let ir_command = translate(data);
				match ir_command {
					IrCommands::Up => shutter_command = Some(shutter::Command::Open),
					IrCommands::Down => shutter_command = Some(shutter::Command::Close),
					IrCommands::Ok => shutter_command = Some(shutter::Command::Stop),
					IrCommands::N0 => shutter_command = Some(shutter::Command::Open),
					IrCommands::N1 => shutter_command = Some(shutter::Command::RollTo(100 / 13)),
					IrCommands::N2 => shutter_command = Some(shutter::Command::RollTo(100 / 13)),
					IrCommands::N3 => shutter_command = Some(shutter::Command::RollTo(300 / 13)),
					IrCommands::N4 => shutter_command = Some(shutter::Command::RollTo(400 / 13)),
					IrCommands::N5 => shutter_command = Some(shutter::Command::RollTo(500 / 13)),
					IrCommands::N6 => shutter_command = Some(shutter::Command::RollTo(600 / 13)),
					IrCommands::N7 => shutter_command = Some(shutter::Command::RollTo(700 / 13)),
					IrCommands::N8 => shutter_command = Some(shutter::Command::RollTo(800 / 13)),
					IrCommands::N9 => shutter_command = Some(shutter::Command::RollTo(900 / 13)),
					_ => {}
				};
			}
			_ => {}
		};

		//update the AC switch sensors statemachine:
		#[cfg(feature = "shutter-switch")]
		{
			let _ = switch_roll_up.update(delta);

			//process the AC roll up switch input:
			if switch_roll_up.last_state() == Some(OnOff::Off)
				&& switch_roll_up.state() == Some(OnOff::On)
			{
				//rising edge detected
				shutter_command = if shutter.is_moving() {
					//in case of moving already first click just stops
					Some(shutter::Command::Stop)
				} else {
					Some(shutter::Command::Open)
				};
			};

			let _ = switch_roll_down.update(delta);

			//process the AC roll down switch input:
			if switch_roll_down.last_state() == Some(OnOff::Off)
				&& switch_roll_down.state() == Some(OnOff::On)
			{
				//rising edge detected
				shutter_command = if shutter.is_moving() {
					//in case of moving already first click just stops
					Some(shutter::Command::Stop)
				} else {
					Some(shutter::Command::Close)
				};
			};
		}

		{
			//TODO improve to use multichanel dma adc (or nonblocking)
			//update roll current sensor
			if let Ok(roll_current) = adc1.read(&mut adc7_roll_motor_current_sense) {
				if let Ok(roll_voltage) = adc1.read(&mut adc9_ac_main_voltage) {
					roll_power_meter.update(roll_current, roll_voltage, delta);

					if let Some(stat) = roll_power_meter.state() {
						shutter_power = stat.avg_power;
					}
				}
			}

			//update the shutter statemachine and forward to the executor SSRs:
			shutter.update(shutter_command, delta, shutter_power, 3000, 40000); //350
			shutter_command = None; //processed -> clear
		}

		let mut color = Colors::Green;
		if open_alarm.is_low() == Ok(true) {
			color = Colors::Red;
			beeper = Beeper::new(&OPEN_MELODY);
			let _ = messenger.transmit(ID_OPEN, Payload::new(&address));
		}
		if motion_alarm.is_low() == Ok(true) {
			color = Colors::Blue;
			beeper = Beeper::new(&MOTION_MELODY);
			let _ = messenger.transmit(ID_MOVEMENT, Payload::new(&address));
		}
		
		#[cfg(feature = "lux-sensor")]
		{
			//measure light and send can message in case of change
			let light: u32 = adc1.read(&mut adc8_photo_resistor).unwrap();
			if lux != Some(light) {
				//TODO send can message
				lux = Some(light);
			}
		}
		rgb.color(color).unwrap();
	}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
	panic!("HardFault at {:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
	panic!("Unhandled exception (IRQn = {})", irqn);
}
