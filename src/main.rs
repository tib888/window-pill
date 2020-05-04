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

//SW TODO
// can lezarasok
// miert villog a mozgaserzekelo mindig? - alacsony volt a fesz

//List of HW problem
//- tapokat elore beallitani 5V-ra!
//- a modulok es a nyak kozott legyen res!
//- berakas elott a csokikat osszerakni!
//- SSR vezerlo tranyokat forditva kell berakni
//- tul fenyes az RGB zold ledje, novelni kell a B15 labon levo ellenallast
//- egy kicsit tul fenyes az RGB kek ledje, novelni kell a B14 labon levo ellenallast
//- 220as kapcsolo vagy valami nagyon zavarerzekeny... 1M-nal kisebb elenallast v. kondit?
//- ? B10, B11 szelepmotor vezerlot atrakni esetleg pwm pinekre ? pl. rakotni B8..B9-re is, a B10,11 pedig akkor dummy input lesz
//- B6, B7 COM portnak legyen fenntartva...
//- riaszto bementet ne legyen az 5v tapra kotve! akkor mar inkabba 12-re
//- 20k-ra novelni az optocsatolo ledjenek elotetellenallasat!
//- optocsatolo helyett lehet jobb lenne nagy bemeno ellenallasok + 2 vedodioda
//- tulfesz vedo diodakat rakni a can busz es a fold koze?
//- A DOOR UNITBAN ROSZ A CAN MEGHAJTO IC ado oldala => CSERE volt!

//door1   Unique ID: 50ff6c065177535424281587 [40, 97, 100, 18, 46, 79, 94, 252]
//window1 Unique ID: 52ff70065065515256301387 [40, 142, 63, 191, 4, 0, 0, 153]

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

pub mod beeper;
pub mod heating;
pub mod roll;
pub mod shutter;

use panic_halt as _;

use cortex_m;
use cortex_m_rt::{entry, exception, ExceptionFrame};

use embedded_hal::{
	digital::v2::{InputPin, OutputPin},
	watchdog::{Watchdog, WatchdogEnable},
	PwmPin,
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
	timer::{Tim4NoRemap, Timer},
	watchdog::IndependentWatchdog,
};

use beeper::Beeper;
use heating::MotorPin;
use heating::Valve;
use onewire::{ds18x20::*, temperature::Temperature, *};
use room_pill::{
	ac_sense::AcSense,
	ac_switch::*,
	ir,
	ir::NecReceiver,
	ir_remote::*,
	messenger::{Messenger, ID_MOVEMENT, ID_OPEN},
	rgb::{Colors, RgbLed},
	timing::{Ticker, TimeExt},
};
use shutter::Shutter;

//use numtoa::NumToA;
//use core::fmt::Write;
use cortex_m_semihosting::hprintln;

#[cfg(feature = "beeper")]
static START_MELODY: [(u16, u16); 5] = [(440, 10), (0, 10), (880, 10), (0, 10), (440, 10)];

type Duration = room_pill::timing::Duration<u32, room_pill::timing::MicroSeconds>;

#[cfg(feature = "pwm-valve")]
impl<T> MotorPin for T
where
	T: PwmPin,
{
	fn on(&mut self) {
		self.enable();
	}
	fn off(&mut self) {
		self.disable();
	}
}

#[cfg(not(feature = "pwm-valve"))]
impl<T> MotorPin for T
where
	T: OutputPin,
{
	fn on(&mut self) {
		self.set_high();
	}
	fn off(&mut self) {
		self.set_low();
	}
}

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
	let _stim = &mut core.ITM.stim[0];

	// real time clock
	let _rtc = {
		let mut pwr = device.PWR;
		let mut backup_domain = rcc.bkp.constrain(device.BKP, &mut rcc.apb1, &mut pwr);
		rtc::Rtc::rtc(device.RTC, &mut backup_domain)
	};

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
	#[cfg(feature = "lux-sensor")]
	let mut adc8_photo_resistor = gpiob.pb0.into_analog(&mut gpiob.crl);

	// -------- Alarm related
	watchdog.feed();

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
	#[cfg(feature = "heating-control")]
	let (mut valve, mut valve_motor_current_sense) = {
		watchdog.feed();

		#[cfg(feature = "pwm-valve")]
		let valve = {
			gpiob.pb10.into_floating_input(&mut gpiob.crh);
			gpiob.pb11.into_floating_input(&mut gpiob.crh);
			let mut valve_motor = Timer::tim4(device.TIM4, &clocks, &mut rcc.apb1)
				.pwm::<Tim4NoRemap, _, _, _>(
					(
						gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh),
						gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh),
					),
					&mut afio.mapr,
					1.khz(),
				);
			valve_motor.set_duty(Channel::C3, valve_motor.get_max_duty() / 2);
			valve_motor.set_duty(Channel::C4, valve_motor.get_max_duty() / 2);
			let (a, b) = valve_motor.split();
			Valve::new(one_sec / 2, a, b)
		};

		#[cfg(not(feature = "pwm-valve"))]
		let valve = {
			gpiob.pb8.into_floating_input(&mut gpiob.crh);
			gpiob.pb9.into_floating_input(&mut gpiob.crh);
			Valve::new(
				one_sec / 2,
				gpiob.pb10.into_push_pull_output(&mut gpiob.crh),
				gpiob.pb11.into_push_pull_output(&mut gpiob.crh),
			)
		};

		// A6 - ADC6 valve motor driver current sense shunt
		let valve_motor_current_sense = gpioa.pa6.into_analog(&mut gpioa.crl);

		// Radiator valve motor sense on A0, A1 (floating or pull down input)
		// to see what the thermostat does by itself
		let _sense_a = gpioa.pa0.into_floating_input(&mut gpioa.crl);
		let _sense_b = gpioa.pa1.into_floating_input(&mut gpioa.crl);

		(valve, valve_motor_current_sense)
	};

	#[cfg(feature = "temp-sensor")]
	let mut one_wire = {
		// DS18B20 1-wire temperature sensors connected to B4 GPIO
		let onewire_io = pb4.into_open_drain_output(&mut gpiob.crl);
		let delay = Delay::new(core.SYST, clocks);
		OneWirePort::new(onewire_io, delay).unwrap()
	};

	// -------- Generic user interface related
	watchdog.feed();

	#[cfg(feature = "beeper")]
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

	#[cfg(feature = "usart-debug")]
	let (mut tx, rx) = {
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
	#[cfg(feature = "temp-sensor")]
	let address = {
		watchdog.feed();

		//store the addresses of temp sensor, start measurement
		let mut address = [0u8; 8];
		let mut it = RomIterator::new(0);
		loop {
			watchdog.feed();

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

	//Unique ID: 52ff70065065515256301387 [40, 142, 63, 191, 4, 0, 0, 153]
	//hprintln!("Unique ID: {} {:?}", stm32_device_signature::device_id_hex(), roms[0]).unwrap();

	// -------- Config finished
	watchdog.feed();
	let mut shutter_power = 4000;
	let mut shutter_command = None;
	let mut temperature = Option::<Temperature>::None;
	let mut lux = Option::<u32>::None;

	#[cfg(feature = "heating-control")]
	let mut heating_command = None;

	#[cfg(feature = "beeper")]
	let mut beeper = Beeper::new(&START_MELODY);

	let tick = Ticker::new(core.DWT, core.DCB, clocks);

	let mut last_time = tick.now();
	let mut last_big_time = last_time;

	rgb.color(Colors::Black).unwrap();

	//main update loop
	loop {
		watchdog.feed();
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
		beeper.update(delta, &|t| room_pill::timing::TimeExt::us(t as u32 * 1000), &mut |freq| {
			if freq != 0 {
				piezzo.set_period((freq as u32).hz());
				piezzo.enable(Channel::C1);
			} else {
				piezzo.disable(Channel::C1);
			}
		});
		
		//process the infrared remote inputs:
		match ir_cmd {
			Ok(ir::NecContent::Repeat) => {}
			Ok(ir::NecContent::Data(data)) => {
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

				#[cfg(feature = "heating-control")]
				match ir_command {
					IrCommands::Left => {
						heating_command = Some(heating::Command::Regular(roll::Command::Open))
					}
					IrCommands::Right => {
						heating_command = Some(heating::Command::Regular(roll::Command::Close))
					}
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

		#[cfg(feature = "heating-control")]
		{
			//execute the user command(s):
			let valve_motor_current: i32 = adc1.read(&mut valve_motor_current_sense).unwrap();
			//writeln!(tx, "{}", valve_motor_current).unwrap();
			valve.update(heating_command, delta, valve_motor_current, 300);
			heating_command = None; //processed -> clear
		}

		// do not execute the followings too often: (temperature conversion time of the sensors is a lower limit)
		if tick.to_us(now - last_big_time) < one_sec {
			continue;
		};
		last_big_time = now;

		let mut color = Colors::Black;
		if open_alarm.is_high() == Ok(true) {
			color = Colors::Green;
			let _ = messenger.transmit(ID_OPEN, Payload::new(&address));
		}
		
		if motion_alarm.is_high() == Ok(true) {
			color = Colors::Blue;
			let _ = messenger.transmit(ID_MOVEMENT, Payload::new(&address));
		}
		
		//messenger.receive_log(&mut tx);
		while let Some((filter_match_index, time_stamp, frame)) = messenger.try_receive() {
			// hprintln!(
			//     "Wnd f:{} t:{} i:{} d:{}",
			//     filter_match_index,
			//     time_stamp,
			//     frame.id().standard(),
			//     frame.data().data_as_u64()
			// ).unwrap();

			//TODO shutter_command = decoded can shutter control commands

			#[cfg(feature = "heating-control")]
			{
				//TODO heating_command = decoded can temp control commands
			}

			#[cfg(feature = "beeper")]
			{
				//TODO receive can ring control
				//piezzo.set_period((lux >> 3).hz());
				//piezzo.enable(Channel::C1);
				//delay(...);
				//piezzo.disable(Channel::C1);
			}
		}

		#[cfg(feature = "temp-sensor")]
		{
			//read sensors and restart temperature measurement
			watchdog.feed();
			if let Ok(temp) = one_wire.read_temperature_measurement_result(&address) {
				if temperature != Some(temp) {
					//TODO send can message
					temperature = Some(temp);
				}
			};

			let _ = one_wire.start_temperature_measurement(&address);
		}

		#[cfg(feature = "lux-sensor")]
		{
			//measure light and send can message in case of change
			let light: u32 = adc1.read(&mut adc8_photo_resistor).unwrap();
			if lux != Some(light) {
				//TODO send can message
				lux = Some(light);
				hprintln!("l {}", light);
			}
		}

		rgb.color(color).unwrap();
		led.toggle().unwrap();
	}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
	//hprintln!("HardFault at {:#?}", ef);
	panic!("HardFault at {:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
	//hprintln!("Unhandled exception (IRQn = {})", irqn);
	panic!("Unhandled exception (IRQn = {})", irqn);
}
