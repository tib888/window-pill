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
//! B3 not used, connected to the ground
//!
//! DS18B20 1-wire temperature sensors connected to B4 GPIO
//! JTAG is removed from B3, B4 to make it work
//!
//! B5 not used, connected to the ground
//!
//! Solid state relay or arbitrary unit can be connected to B6, B7, B8, B9
//!
//! Radiator valve motor driver on B10, B11 (push pull output, pwm?)
//!
//! B12 not used, connected to the ground
//!
//! RGB led on PB13, PB14, PB15 as push pull output
//!
//! C13 on board LED
//!
//! C14, C15 used on the bluepill board for 32768Hz xtal
//!

//List of HW problem
//- tapokat elore beallitani 5V-ra!
//- a modulok es a nyak kozott legyen res!
//- berakas elott a csokikat osszerakni!
//- SSR vezerlo tranyokat forditva kell berakni
//- tul fenyes az RGB zold ledje, novelni kell a B15 labon levo ellenallast

//List of PCB problems
//- biztositekoknak nincs eleg hely a csoki mellett
//- SSR labtavot novelni
//- SSR labfurat csokkenteni (mint 100nf)
//- 100nf labfurat csokkenteni (mint az ellenallasok)
//- A C3 (470u 16V) nagyobb atmeroju!

//#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m;

//#[macro_use]
use core::num::Wrapping;
use cortex_m::{iprintln, peripheral::itm::Stim, peripheral::ITM, Peripherals};
use cortex_m_rt::entry;
use embedded_hal::{
	digital::v2::{InputPin, OutputPin},
	watchdog::{Watchdog, WatchdogEnable},
};
use onewire::{ds18x20::*, temperature::Temperature, *};

use panic_halt as _;
use room_pill::{
	ac_sense::AcSense,
	ac_switch::*,
	ir,
	ir::NecReceiver,
	ir_remote::*,
	rgb::{Colors, RgbLed},
	roll,
	timing::{Ticker, TimeExt},
};

use stm32f1xx_hal::{
	adc,
	can::*,
	delay::Delay,
	prelude::*,
	pwm::{Channel, Pwm},
	rtc,
	timer::Timer,
	watchdog::IndependentWatchdog,
};

type Duration = room_pill::timing::Duration<u32, room_pill::timing::MicroSeconds>;

/// pos should be in [0..9] range
fn roll_to(roll: &roll::Roll<Duration>, pos: u32) -> Option<roll::Command<Duration>> {
	if let &Some(max) = roll.bottom() {
		Some(roll::Command::SendTo(max * pos / 13)) //theoretically /9 but about at the last 30% it is closed
	} else {
		None
	}
}

// fn beep(pwm: &mut dyn Pwm) {
// 	pwm.set_period(440.hz());
// 	pwm.enable(Channel::C1);
// 	//delay.delay_us(100_000);
// 	pwm.disable(Channel::C1);
// }

#[entry]
fn main() -> ! {
	window_unit_main();
}

fn window_unit_main() -> ! {
	let dp = stm32f1xx_hal::pac::Peripherals::take().unwrap();
	let mut watchdog = IndependentWatchdog::new(dp.IWDG);
	watchdog.start(stm32f1xx_hal::time::U32Ext::ms(2_000u32));

	//let stim = &mut dp.ITM.stim[0];

	//10 period at 50Hz, 12 period at 60Hz
	let ac_test_period = TimeExt::us(200_000u32);
	let one_sec = TimeExt::us(1_000_000u32);

	let mut flash = dp.FLASH.constrain();

	//flash.acr.prftbe().enabled();//?? Configure Flash prefetch - Prefetch buffer is not available on value line devices
	//scb().set_priority_grouping(NVIC_PRIORITYGROUP_4);

	let mut rcc = dp.RCC.constrain();
	let clocks = rcc
		.cfgr
		.use_hse(8.mhz())
		.sysclk(72.mhz())
		.hclk(72.mhz())
		.pclk1(36.mhz())
		.pclk2(72.mhz())
		.adcclk(9.mhz()) //ADC clock: PCLK2 / 8. User specified value is be approximated using supported prescaler values 2/4/6/8.
		.freeze(&mut flash.acr);
	watchdog.feed();

	let mut pwr = dp.PWR;

	let mut backup_domain = rcc.bkp.constrain(dp.BKP, &mut rcc.apb1, &mut pwr);
	// real time clock
	let _rtc = rtc::Rtc::rtc(dp.RTC, &mut backup_domain);

	// A/D converter
	let mut adc1 = adc::Adc::adc1(dp.ADC1, &mut rcc.apb2, clocks);

	watchdog.feed();

	let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

	//configure pins:
	let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
	let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
	let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

	// Disables the JTAG to free up pb3, pb4 and pa15 for normal use
	let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
	// -------- Roll control related
	watchdog.feed();

	// Roll up switches A2 (pull down once in each main period if closed)
	let mut switch_roll_up =
		AcSwitch::new(gpioa.pa2.into_pull_up_input(&mut gpioa.crl), ac_test_period);

	// Roll down switches A3 (pull down once in each main period if closed)
	let mut switch_roll_down =
		AcSwitch::new(gpioa.pa3.into_pull_up_input(&mut gpioa.crl), ac_test_period);

	// Roll motor hall current sense on A7 (ADC7)
	let mut adc7_roll_motor_current_sense = gpioa.pa7.into_analog(&mut gpioa.crl);
	// AC main voltage sense on B1 (ADC9)
	let mut adc9_ac_main_voltage = gpiob.pb1.into_analog(&mut gpiob.crl);
	// Photoresistor on B0 (ADC8)
	let mut _adc8_photo_resistor = gpiob.pb0.into_analog(&mut gpiob.crl);

	// Solid state relay connected to A9 drives the ssr_roll_down (push pull output)
	let mut ssr_roll_down = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);

	// Solid state relay connected to A10 drives the ssr_roll_up (push pull output)
	let mut ssr_roll_up = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);

	let mut roll = roll::Roll::<Duration>::new();
	let mut roll_power_detector = AcSense::new(ac_test_period, true); //0.1sec to be able to wait the first current measurement at motor start

	// -------- Alarm related
	watchdog.feed();

	// Ext/Motion alarm on A4 (pull down)
	let motion_alarm = gpioa.pa4.into_pull_up_input(&mut gpioa.crl);

	// Open alarm on A5 (pull down)
	let open_alarm = gpioa.pa5.into_pull_up_input(&mut gpioa.crl);
	// CAN (RX, TX) on A11, A12
	let canrx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
	let cantx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
	// USB is needed here because it can not be used at the same time as CAN since they share memory:
	let mut _can = Can::can1(
		dp.CAN1,
		(cantx, canrx),
		&mut afio.mapr,
		&mut rcc.apb1,
		dp.USB,
	);

	// -------- Heating control related:
	watchdog.feed();

	// DS18B20 1-wire temperature sensors connected to B4 GPIO
	let onewire_io = pb4.into_open_drain_output(&mut gpiob.crl);
	let cp = cortex_m::Peripherals::take().unwrap();
	let delay = Delay::new(cp.SYST, clocks);
	let mut one_wire = OneWirePort::new(onewire_io, delay).unwrap();

	// Radiator valve motor sense on A0, A1 (floating or pull down input)
	let _valve_sense_a = gpioa.pa0.into_floating_input(&mut gpioa.crl);
	let _valve_sense_b = gpioa.pa1.into_floating_input(&mut gpioa.crl);
	// A6 - ADC6 valve motor driver current sense shunt
	let mut _adc6_valve_motor_current_sense = gpioa.pa6.into_analog(&mut gpioa.crl);

	// Radiator valve motor driver on B10, B11 (push pull output, pwm?)
	let mut _valve_motor_drive_a = gpiob.pb10.into_push_pull_output(&mut gpiob.crh);
	let mut _valve_motor_drive_b = gpiob.pb11.into_push_pull_output(&mut gpiob.crh);

	// -------- Generic user interface related
	watchdog.feed();

	// Optional piezzo speaker on A8 (open drain output)
	let piezzo_pin = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh); //TODO into_alternate_open_drain
	let mut piezzo =
		Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2).pwm(piezzo_pin, &mut afio.mapr, 1.khz()); //pwm::<Tim1NoRemap, _, _, _>
	piezzo.set_duty(Channel::C1, piezzo.get_max_duty() / 2);
	piezzo.disable(Channel::C1);
	// Read the NEC IR remote commands on A15 GPIO as input with internal pullup
	let ir_receiver = pa15.into_pull_up_input(&mut gpioa.crh);
	let mut receiver = ir::IrReceiver::new();
	// RGB led on PB13, PB14, PB15 as open drain (or push pull) output
	let mut rgb = RgbLed::new(
		gpiob.pb13.into_open_drain_output(&mut gpiob.crh),
		gpiob.pb15.into_open_drain_output(&mut gpiob.crh),
		gpiob.pb14.into_open_drain_output(&mut gpiob.crh),
	);
	rgb.color(Colors::Black).unwrap();

	// C13 on board LED^
	let mut led = gpioc.pc13.into_open_drain_output(&mut gpioc.crh);
	led.set_high().unwrap(); //turn off

	// Solid state relay or arbitrary unit can be connected to B6, B7, B8, B9
	let mut _ssr_0 = gpiob.pb6.into_push_pull_output(&mut gpiob.crl);
	let mut _ssr_1 = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
	let mut _ssr_2 = gpiob.pb8.into_push_pull_output(&mut gpiob.crh);
	let mut _ssr_3 = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);

	// B3 not used, connected to the ground
	let _b3 = pb3.into_pull_down_input(&mut gpiob.crl);

	// B5 not used, connected to the ground
	let _b5 = gpiob.pb5.into_pull_down_input(&mut gpiob.crl);

	// B12 not used, connected to the ground
	let _b12 = gpiob.pb12.into_pull_down_input(&mut gpiob.crh);

	// C14, C15 used on the bluepill board for 32768Hz xtal

	// -------- Init temperature measurement
	// watchdog.feed();

	// const MAX_THERMOMETER_COUNT: usize = 4; //max number of thermometers

	// //store the addresses of temp sensors, start measurement on each:
	// let mut roms = [[0u8; 8]; MAX_THERMOMETER_COUNT];
	// let mut count = 0;

	// let mut it = RomIterator::new(0);

	// loop {
	// 	watchdog.feed();

	// 	match one_wire.iterate_next(true, &mut it) {
	// 		Ok(None) => {
	// 			break; //no or no more devices found -> stop
	// 		}

	// 		Ok(Some(rom)) => {
	// 			if let Some(_device_type) = detect_18x20_devices(rom[0]) {
	// 				//writeln!(hstdout, "rom: {:?}", &rom).unwrap();
	// 				roms[count] = *rom;
	// 				count = count + 1;
	// 				let _ = one_wire.start_temperature_measurement(&rom);
	// 				if count >= MAX_THERMOMETER_COUNT {
	// 					break;
	// 				}
	// 			}
	// 			continue;
	// 		}

	// 		Err(_e) => {
	// 			rgb.color(Colors::White).unwrap();
	// 			break;
	// 		}
	// 	}
	// }
	// //not mutable anymore
	// let roms = roms;
	// let count = count;
	// let mut temperatures = [Option::<Temperature>::None; MAX_THERMOMETER_COUNT];

	// -------- Config finished
	watchdog.feed();

	let tick = Ticker::new(cp.DWT, cp.DCB, clocks);

	//todo beep(piezzo, 440, 100);

	let mut last_time = tick.now();
	let mut last_big_time = last_time;

	rgb.color(Colors::Black).unwrap(); //todo remove

	//main update loop
	loop {
		watchdog.feed();

		// calculate the time since last execution:
		let now = tick.now();
		let delta = tick.to_us(now - last_time); //in case of 72MHz sysclock this works if less than 59sec passed between two calls
		last_time = now;

		//update the IR receiver statemachine:
		let ir_cmd = receiver.receive(now, ir_receiver.is_low().unwrap(), |a, b| tick.to_us(a - b));

		//update the AC switch sensors statemachine:
		switch_roll_up.update(delta).unwrap();
		switch_roll_down.update(delta).unwrap();

		let mut driving_power_detected = true;

		//TODO improve to multichanel dma adc
		//update roll current sensor
		if let Ok(roll_current) = adc1.read(&mut adc7_roll_motor_current_sense) {
			watchdog.feed();

			if let Ok(roll_voltage) = adc1.read(&mut adc9_ac_main_voltage) {
				watchdog.feed();

				roll_power_detector.update(roll_current, roll_voltage, delta);

				if let Some(stat) = roll_power_detector.state() {
					//iprintln!(stim, "{}", stat.avg_power);

					//onboard led shows the power usage of roll motor: //todo remove?
					if stat.avg_power > 350 {
						led.set_low().unwrap();
					} else {
						led.set_high().unwrap();
						driving_power_detected = false;
					}
				}
			}
		}

		//update the roll statemachine and forward to the executor SSRs:
		let roll_state = roll.update(delta, driving_power_detected);
		match roll_state {
			roll::State::DrivingUp => {
				ssr_roll_down.set_low().unwrap();
				ssr_roll_up.set_high().unwrap();
				rgb.color(Colors::Green).unwrap(); //todo remove?
			}
			roll::State::DrivingDown => {
				ssr_roll_up.set_low().unwrap();
				ssr_roll_down.set_high().unwrap();
				rgb.color(Colors::Blue).unwrap(); //todo remove?
			}
			roll::State::Stopped => {
				ssr_roll_up.set_low().unwrap();
				ssr_roll_down.set_low().unwrap();
				rgb.color(Colors::Red).unwrap(); //todo remove?
			}
		}

		let mut roll_command = None;

		//process the infrared remote inputs:
		match ir_cmd {
			Ok(ir::NecContent::Repeat) => {}
			Ok(ir::NecContent::Data(data)) => {
				let ir_command = translate(data);
				//write!(hstdout, "{:x}={:?} ", data, command).unwrap();
				//model.ir_remote_command(command, &MENU);
				//model.refresh_display(&mut display, &mut backlight);

				roll_command = match ir_command {
					IrCommands::Up => Some(roll::Command::SendUp),
					IrCommands::Down => Some(roll::Command::SendDown),
					IrCommands::Ok => Some(roll::Command::Stop),
					IrCommands::N0 => Some(roll::Command::SendUp),
					IrCommands::N1 => roll_to(&roll, 1),
					IrCommands::N2 => roll_to(&roll, 2),
					IrCommands::N3 => roll_to(&roll, 3),
					IrCommands::N4 => roll_to(&roll, 4),
					IrCommands::N5 => roll_to(&roll, 5),
					IrCommands::N6 => roll_to(&roll, 6),
					IrCommands::N7 => roll_to(&roll, 7),
					IrCommands::N8 => roll_to(&roll, 8),
					IrCommands::N9 => roll_to(&roll, 9), //Some(roll::Command::SendDown),
					_ => None,
				}
			}
			_ => {}
		};

		//process the AC switch inputs:
		if switch_roll_up.last_state() == Some(OnOff::Off)
			&& switch_roll_up.state() == Some(OnOff::On)
		{
			//rising edge detected
			roll_command = if roll_state != roll::State::Stopped {
				//in case of moving already first click just stops
				Some(roll::Command::Stop)
			} else {
				Some(roll::Command::SendUp)
			};
		};

		if switch_roll_down.last_state() == Some(OnOff::Off)
			&& switch_roll_down.state() == Some(OnOff::On)
		{
			//rising edge detected
			roll_command = if roll_state != roll::State::Stopped {
				//in case of moving already first click just stops
				Some(roll::Command::Stop)
			} else {
				Some(roll::Command::SendDown)
			};
		};

		if switch_roll_down.last_state() != switch_roll_down.state() {
			rgb.set(
				switch_roll_down.last_state() == Some(OnOff::On),
				false,
				switch_roll_down.state() == Some(OnOff::On),
			)
			.unwrap();
		}

		//execute the user command(s):
		if let Some(command) = roll_command {
			roll.execute(command);
			//this is here to have enough time to measure the current after the start:
			roll_power_detector.reset();
		}

		if motion_alarm.is_low() == Ok(true) {
			//todo send can message
		}

		if open_alarm.is_high() == Ok(true) {
			//todo send can message
		}

		// do not execute the followings too often: (temperature conversion time of the sensors is a lower limit)
		let big_delta = tick.to_us(now - last_big_time);
		if big_delta < one_sec {
			continue;
		}
		last_big_time = now;

		//read sensors and restart temperature measurement
		// for i in 0..count {
		// 	temperatures[i] = match one_wire.read_temperature_measurement_result(&roms[i]) {
		// 		Ok(temperature) => Some(temperature),
		// 		Err(_code) => None,
		// 	};

		// 	let _ = one_wire.start_temperature_measurement(&roms[i]);
		// }

		//todo send can message
		//todo receive can temp control
		//todo control radiator valve?
		// adc6_valve_motor_current_sense;

		//todo receive can ring control
		// let lux: u32 = adc1.read(&mut adc8_photo_resistor).unwrap();
		// piezzo.enable(Channel::C1);
		// piezzo.set_period((lux >> 3).hz());

		// adc7_roll_motor_current_sense;
		// adc8_photo_resistor;

		//led.toggle().unwrap();
	}
}

// #[exception]
// fn HardFault(ef: &ExceptionFrame) -> ! {
//  	panic!("HardFault at {:#?}", ef);
//  }

// #[exception]
// fn DefaultHandler(irqn: i16) {
//  	panic!("Unhandled exception (IRQn = {})", irqn);
// }
