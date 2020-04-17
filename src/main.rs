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
//- egy kicsit tul fenyes az RGB kek ledje, novelni kell a B15 labon levo ellenallast
//- 220as kapcsolo vagy valami nagyon zavarerzekeny... 1M-nal kisebb elenallast v. kondit?
//- ? B10, B11 szelepmotor vezerlot atrakni esetleg pwm pinekre ? pl. rakotni B8..B9-re is, a B10,11 pedig akkor dummy input lesz
//- B6, B7 COM portnak legyen fenntartva...
//- riaszto bementet ne legyen az 5v tapra kotve! akkor mar inkabba 12-re
//- 20k-ra novelni az optocsatolo ledjenek elotetellenallasat!
//- optocsatolo helyett lehet jobb lenne nagy bemeno ellenallasok + 2 vedodioda

//List of PCB problems
//- biztositekoknak nincs eleg hely a csoki mellett
//- SSR labtavot novelni
//- SSR labfurat csokkenteni (mint 100nf)
//- 100nf labfurat csokkenteni (mint az ellenallasok)
//- A C3 (470u 16V) nagyobb atmeroju!
//- pb3-t fel kell szabaditani hogy mukodhessen az itm debugging
//- a CAN lezaras tuajdonkepp lehagyhato a nyakrol
//- SMD ?
//- RGB led labait cikk-cakk-ba?

//Termosztat bekotes
//vastag piros: elem +
//vastag fekete: elem -
//piros -> motor piros
//feher -> motor fekete
//sarga <- vezerles fekete
//zold <- vezerles piros

//Mozgaserzekelo bekotes:
//narancs: +9..+16V
//narance feher : fold
//zoldek: NC mozgas
//etc nincs bekotve, tamper sem 

//#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod valve;

use core::fmt::Write;

#[cfg(not(feature = "itm-debug"))]
use panic_halt as _;
#[cfg(feature = "itm-debug")]
use panic_itm as _;

use cortex_m;

#[cfg(feature = "itm-debug")]
use cortex_m::{iprint, iprintln};
#[cfg(feature = "semihosting-debug")]
use cortex_m_semihosting::hprintln;

use cortex_m_rt::entry; 

//use numtoa::NumToA;

use embedded_hal::{
	digital::v2::{InputPin, OutputPin},
	watchdog::{Watchdog, WatchdogEnable},
};

use stm32f1xx_hal;
use stm32f1xx_hal::{
	adc,
	can::*,
	delay::Delay,
	prelude::*,
	pwm::{Channel, Pwm},
	rtc,
	serial::{Config, Serial},
	//time::{Instant, MicroSeconds, MonoTimer},
	timer::Timer,
	watchdog::IndependentWatchdog,
};

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
use onewire::{ds18x20::*, temperature::Temperature, *};
use valve::Valve;

type Duration = room_pill::timing::Duration<u32, room_pill::timing::MicroSeconds>;

/// pos should be in [0..9] range
fn roll_to(roll: &roll::Roll<Duration>, pos: u32) -> Option<roll::Command<Duration>> {
	if let &Some(max) = roll.maximum() {
		Some(roll::Command::SetPosition(max * pos / 13)) //theoretically /9 but about at the last 30% it is closed
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
	let device = stm32f1xx_hal::pac::Peripherals::take().unwrap();
	let mut watchdog = IndependentWatchdog::new(device.IWDG);
	watchdog.start(stm32f1xx_hal::time::U32Ext::ms(2_000u32));

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
	watchdog.feed();

	let mut core = cortex_m::Peripherals::take().unwrap();
	#[cfg(feature = "itm-debug")]
	let stim = &mut core.ITM.stim[0];

	let mut pwr = device.PWR;
	let mut backup_domain = rcc.bkp.constrain(device.BKP, &mut rcc.apb1, &mut pwr);
	// real time clock
	let _rtc = rtc::Rtc::rtc(device.RTC, &mut backup_domain);

	// A/D converter
	let mut adc1 = adc::Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);

	watchdog.feed();

	let mut afio = device.AFIO.constrain(&mut rcc.apb2);

	//let _dma_channels = device.DMA1.split(&mut rcc.ahb);

	//configure pins:
	let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
	let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
	let mut gpioc = device.GPIOC.split(&mut rcc.apb2);

	// Disables the JTAG to free up pb3, pb4 and pa15 for normal use
	let (pa15, _pb3_itm_swo, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
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
	let mut adc8_photo_resistor = gpiob.pb0.into_analog(&mut gpiob.crl);

	// Solid state relay connected to A9 drives the ssr_roll_down (push pull output)
	let mut ssr_roll_down = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);

	// Solid state relay connected to A10 drives the ssr_roll_up (push pull output)
	let mut ssr_roll_up = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);

	let mut roll = roll::Roll::<Duration>::new();
	let mut roll_power_detector = AcSense::<Duration>::new(ac_test_period, true); //0.1sec to be able to wait the first current measurement at motor start

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
		device.CAN1,
		(cantx, canrx),
		&mut afio.mapr,
		&mut rcc.apb1,
		device.USB,
	);

	// -------- Heating control related:
	watchdog.feed();

	let mut valve = Valve::new(
		one_sec,
		gpiob.pb10.into_push_pull_output(&mut gpiob.crh),
		gpiob.pb11.into_push_pull_output(&mut gpiob.crh),
	);

	// A6 - ADC6 valve motor driver current sense shunt
	let mut valve_motor_current_sense = gpioa.pa6.into_analog(&mut gpioa.crl);

	// Radiator valve motor sense on A0, A1 (floating or pull down input)
	//to see what the thermostat does by itself
	let _sense_a = gpioa.pa0.into_floating_input(&mut gpioa.crl);
	let _sense_b = gpioa.pa1.into_floating_input(&mut gpioa.crl);
	
	// DS18B20 1-wire temperature sensors connected to B4 GPIO
	let onewire_io = pb4.into_open_drain_output(&mut gpiob.crl);
	let delay = Delay::new(core.SYST, clocks);
	let mut one_wire = OneWirePort::new(onewire_io, delay).unwrap();

	// -------- Generic user interface related
	watchdog.feed();

	// Optional piezzo speaker on A8 (open drain output)
	let piezzo_pin = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh); //TODO into_alternate_open_drain
	let mut piezzo =
		Timer::tim1(device.TIM1, &clocks, &mut rcc.apb2).pwm(piezzo_pin, &mut afio.mapr, 1.khz()); //pwm::<Tim1NoRemap, _, _, _>
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
	let (mut tx, rx) = serial.split();

	//let _tx_dma = tx.with_dma(dma_channels.4);

	// arbitrary unit can be connected to B6, B7, B8, B9
	//let mut _b6 = gpiob.pb6.into_push_pull_output(&mut gpiob.crl);
	//let mut _b7 = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
	let mut _b8 = gpiob.pb8.into_push_pull_output(&mut gpiob.crh);
	let mut _b9 = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);

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
	watchdog.feed();

	const MAX_THERMOMETER_COUNT: usize = 2; //max number of thermometers

	//store the addresses of temp sensors, start measurement on each:
	let mut roms = [[0u8; 8]; MAX_THERMOMETER_COUNT];
	let mut count = 0;

	let mut it = RomIterator::new(0);
	let mut lux = Option::<u32>::None;

	loop {
		watchdog.feed();

		match one_wire.iterate_next(true, &mut it) {
			Ok(None) => {
				break; //no or no more devices found -> stop
			}

			Ok(Some(rom)) => {
				if let Some(_device_type) = detect_18x20_devices(rom[0]) {
					//#[cfg(feature = "semihosting-debug")]
					//hprintln!("rom: {:?}", &rom).unwrap();

					//TODO use this address as unique id on the CAN bus!
					roms[count] = *rom;
					count = count + 1;
					let _ = one_wire.start_temperature_measurement(&rom);
					if count >= MAX_THERMOMETER_COUNT {
						break;
					}
				}
				continue;
			}

			Err(_e) => {
				rgb.color(Colors::White).unwrap();
				break;
			}
		}
	}

	//not mutable anymore
	let roms = roms;
	let count = count;
	let mut temperatures = [Option::<Temperature>::None; MAX_THERMOMETER_COUNT];

	rgb.color(Colors::Black).unwrap(); //todo remove

	// -------- Config finished
	watchdog.feed();
	let tick = Ticker::new(core.DWT, core.DCB, clocks);
	//let tick = MonoTimer::new(core.DWT, clocks); //core.DCB,

	//todo beep(piezzo, 440, 100);

	let mut last_time = tick.now();
	let mut last_big_time = last_time;

	//main update loop
	loop {
		watchdog.feed();

		// calculate the time since last execution:
		let now = tick.now();
		let delta = tick.to_us(now - last_time); //in case of 72MHz sysclock this works if less than 59sec passed between two calls
		last_time = now;

		//update the IR receiver statemachine:
		let ir_cmd = receiver.receive(ir_receiver.is_low().unwrap(), now, |last| tick.to_us(now - last).into());

		//update the AC switch sensors statemachine:
		switch_roll_up.update(delta).unwrap();
		switch_roll_down.update(delta).unwrap();
		let mut roll_command = None;
		let mut valve_command = None;

		//process the infrared remote inputs:
		match ir_cmd {
			Ok(ir::NecContent::Repeat) => {}
			Ok(ir::NecContent::Data(data)) => {
				let ir_command = translate(data);
				roll_command = match ir_command {
					IrCommands::Up => Some(roll::Command::Open),
					IrCommands::Down => Some(roll::Command::Close),
					IrCommands::Ok => Some(roll::Command::Stop),
					IrCommands::N0 => Some(roll::Command::Open),
					IrCommands::N1 => roll_to(&roll, 1),
					IrCommands::N2 => roll_to(&roll, 2),
					IrCommands::N3 => roll_to(&roll, 3),
					IrCommands::N4 => roll_to(&roll, 4),
					IrCommands::N5 => roll_to(&roll, 5),
					IrCommands::N6 => roll_to(&roll, 6),
					IrCommands::N7 => roll_to(&roll, 7),
					IrCommands::N8 => roll_to(&roll, 8),
					IrCommands::N9 => roll_to(&roll, 9), //Some(roll::Command::Close),
					_ => None,
				};
				valve_command = match ir_command {
					IrCommands::Left => Some(valve::Command::Regular(roll::Command::Open)),
					IrCommands::Right => Some(valve::Command::Regular(roll::Command::Close)),
					_ => None,
				};
			}
			_ => {}
		};

		//process the AC roll up switch input:
		if switch_roll_up.last_state() == Some(OnOff::Off)
			&& switch_roll_up.state() == Some(OnOff::On)
		{
			//rising edge detected
			roll_command = if roll.state() != roll::State::Stopped {
				//in case of moving already first click just stops
				Some(roll::Command::Stop)
			} else {
				Some(roll::Command::Open)
			};
		};

		//process the AC roll down switch input:
		if switch_roll_down.last_state() == Some(OnOff::Off)
			&& switch_roll_down.state() == Some(OnOff::On)
		{
			//rising edge detected
			roll_command = if roll.state() != roll::State::Stopped {
				//in case of moving already first click just stops
				Some(roll::Command::Stop)
			} else {
				Some(roll::Command::Close)
			};
		};

		//execute the user command(s):
		if let Some(command) = roll_command {
			roll.execute(command);
			//this is here to have enough time to measure the current after the start:
			roll_power_detector.reset();
		}

		if roll.state() != roll::State::Stopped
			|| ssr_roll_up.is_set_high().unwrap()
			|| ssr_roll_down.is_set_high().unwrap()
		{
			let mut driving_power_detected = true;

			//TODO improve to use multichanel dma adc
			//update roll current sensor
			if let Ok(roll_current) = adc1.read(&mut adc7_roll_motor_current_sense) {
				watchdog.feed();

				if let Ok(roll_voltage) = adc1.read(&mut adc9_ac_main_voltage) {
					watchdog.feed();
					roll_power_detector.update(roll_current, roll_voltage, delta);

					if let Some(stat) = roll_power_detector.state() {
						if stat.avg_power > 350 {
							//#[cfg(feature = "itm-debug")]
							//iprint!(stim, "{}\n\r", stat.avg_power);
							//serial.write((stat.avg_power & 255) as u8).unwrap();
							//serial.write(((stat.avg_power >> 8) & 255) as u8).unwrap();
							//writeln!(tx, "{:X}", stat.avg_power as i16).unwrap();
							//let (_, _tx_dma) = tx_dma.write(&mut BUFFER).wait();
							//let mut buffer = [0u8; 12];
							//stat.avg_power.numtoa(10, &mut buffer).iter().for_each(|c| tx.write(*c).unwrap());

							if stat.avg_power > 40000 {
								//overload protection
								ssr_roll_up.set_low().unwrap();
								ssr_roll_down.set_low().unwrap();
								rgb.color(Colors::White).unwrap(); //todo remove?
								roll.execute(roll::Command::Stop);
								// #[cfg(feature = "itm-debug")]
								// iprintln!(stim, "Roll overload detected: {}", stat.avg_power);
								// #[cfg(feature = "semihosting-debug")]
								// hprintln!("Roll overload detected: {}", stat.avg_power).unwrap();
							}

							//onboard led shows the power usage of roll motor: //todo remove?
							led.set_low().unwrap();
						} else {
							//onboard led shows the power usage of roll motor: //todo remove?
							led.set_high().unwrap();
							driving_power_detected = false;
						}
					}
				}
			}

			//update the roll statemachine and forward to the executor SSRs:
			roll.update(delta, driving_power_detected);

			match roll.state() {
				roll::State::Opening => {
					ssr_roll_down.set_low().unwrap();
					ssr_roll_up.set_high().unwrap();
					rgb.color(Colors::Green).unwrap(); //todo remove?
				}
				roll::State::Closing => {
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
		}

		//execute the user command(s):
		if let Some(command) = valve_command {
			valve.command(command);
		}
		let current: u16 = adc1.read(&mut valve_motor_current_sense).unwrap();
		// #[cfg(feature = "itm-debug")]
		// iprint!(stim, "{}", current);
		//#[cfg(feature = "semihosting-debug")]
		//hprintln!("{}", current).unwrap();
		//write!(tx, "{}\n\r", current).unwrap();
		valve.update(delta, current > 300);
		
		if motion_alarm.is_high() == Ok(true) {
			//todo send can message on rising edges
			rgb.color(Colors::Purple).unwrap(); //todo remove?
		}

		if open_alarm.is_high() == Ok(true) {
			//todo send can message on rising edges
			rgb.color(Colors::Cyan).unwrap(); //todo remove?
		}

		// do not execute the followings too often: (temperature conversion time of the sensors is a lower limit)
		let big_delta = tick.to_us(now - last_big_time);
		if big_delta < one_sec {
			continue;
		}
		last_big_time = now;

		//read sensors and restart temperature measurement
		for i in 0..count {
			temperatures[i] = match one_wire.read_temperature_measurement_result(&roms[i]) {
				Ok(temperature) => {
					// #[cfg(feature = "semihosting-debug")]
					// hprintln!(
					// 	"T[{}] = {}.{}",
					// 	i,
					// 	temperature.whole_degrees(),
					// 	temperature.fraction_degrees()
					// )
					// .unwrap();
					// #[cfg(feature = "itm-debug")]
					// iprintln!(
					// 	stim,
					// 	"T[{}] = {}.{}",
					// 	i,
					// 	temperature.whole_degrees(),
					// 	temperature.fraction_degrees()
					// );
					Some(temperature)
				}
				Err(_code) => None,
			};

			let _ = one_wire.start_temperature_measurement(&roms[i]);
		}

		//TODO send can message
		//TODO receive can temp control
		//TODO control radiator valve?
		// adc6_valve_motor_current_sense;

		//TODO receive can ring control
		// piezzo.enable(Channel::C1);
		// piezzo.set_period((lux >> 3).hz());

		//TODO measure light and send can message in case of change
		let light: u32 = adc1.read(&mut adc8_photo_resistor).unwrap();
		if let Some(l) = lux {
			if l != light {
				lux = Some(light);
				//TODO send can message
			}
		} else {
			lux = Some(light);
		}

		//led.toggle().unwrap();

		//#[cfg(feature = "itm-debug")]
		//iprintln!(stim, "The quick brown fox jumped over the lazy dog.");

		//write!(tx, "The quick brown fox jumped over the lazy dog.\n\r").unwrap();
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
