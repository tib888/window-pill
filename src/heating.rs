use crate::roll;
use crate::roll::{Roll, State};
use core::ops::{Div, Mul};
use num_traits::Saturating;

pub trait MotorPin {
    fn on(&mut self);
    fn off(&mut self);
}

pub enum Command<DURATION> {
    Regular(roll::Command<DURATION>),
    Calibrate,
}

enum Mode {
    Normal,
    CalibrationPhase1,
    CalibrationPhase2,
}

pub struct Valve<DA, DB, DURATION>
where
    DA: MotorPin,
    DB: MotorPin,
    DURATION: Default,
{
    mode: Mode,
    state: Roll<DURATION>,
    drive_time: DURATION,
    allowed_overload_duration: DURATION, //during the start only!
    motor_drive_a: DA,
    motor_drive_b: DB,
    //motor_current_sense: CS,
}

impl<DA, DB, DURATION> Valve<DA, DB, DURATION>
where
    DA: MotorPin,
    DB: MotorPin,
    DURATION: Default
        + PartialEq
        + PartialOrd
        + Copy
        + Saturating
        + Div<u32, Output = DURATION>
        + Mul<u32, Output = DURATION>,
{
    pub fn new(allowed_overload_duration: DURATION, motor_drive_a: DA, motor_drive_b: DB) -> Self {
        Self {
            mode: Mode::Normal, //CalibrationPhase1
            state: Roll::<DURATION>::new(),
            drive_time: DURATION::default(),
            allowed_overload_duration: allowed_overload_duration,
            motor_drive_a: motor_drive_a,
            motor_drive_b: motor_drive_b,
            //motor_current_sense: CS,
        }
    }

    pub fn update(
        &mut self,
        command: Option<Command<DURATION>>,
        delta_t: DURATION,
        power: i32,
        overload_level: i32,
    ) {
        if self.state.state() != State::Stopped {
            self.drive_time = self.drive_time.saturating_add(delta_t);
        }

        if let Some(command) = command {
            self.mode = match command {
                Command::Regular(command) => {
                    let translated = if let Some(max) = self.state.maximum() {
                        //to avoid overload at the ends, stay away with 1/16 range
                        match command {
                            crate::roll::Command::Open => {
                                crate::roll::Command::SetPosition(*max / 16u32)
                            }
                            crate::roll::Command::Close => {
                                crate::roll::Command::SetPosition(*max * 15u32 / 16u32)
                            }
                            _ => command,
                        }
                    } else {
                        command
                    };
                    self.state.execute(translated);
                    Mode::Normal
                }
                Command::Calibrate => Mode::CalibrationPhase1,
            }
        }
        //if eats too many then blocked (except we allow some starting transient)
        let moving = power < overload_level || self.drive_time <= self.allowed_overload_duration;

        //update the valve state machine and apply the state on the HW:
        self.state.update(delta_t, moving);
        //render the state on the hardware:
        //a=0 b=1 : closing current: [120 -> 250], block: 300..440
        //a=1 b=0 : opening current: [120..160] avg=143, block: 300..440
        match self.state.state() {
            State::Opening => {
                self.motor_drive_b.off();
                self.motor_drive_a.on();
            }
            State::Closing => {
                self.motor_drive_a.off();
                self.motor_drive_b.on();
            }
            State::Stopped => {
                self.motor_drive_a.off();
                self.motor_drive_b.off();
                self.drive_time = DURATION::default();

                self.mode = match self.mode {
                    Mode::CalibrationPhase1 => {
                        self.state.execute(crate::roll::Command::Open);
                        Mode::CalibrationPhase2
                    }
                    Mode::CalibrationPhase2 => {
                        self.state.execute(crate::roll::Command::Close);
                        Mode::Normal
                    }
                    _ => Mode::Normal,
                }
            }
        }
    }
}
