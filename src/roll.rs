//! Models the roll operation

use core::ops::{Add, Sub};

#[derive(Copy, Clone, PartialEq)]
pub enum Command<DURATION> {
    SendUp,
    SendDown,
    SendTo(DURATION), //can be computed from bottom position if known
    Stop,
}

#[derive(Copy, Clone, PartialEq)]
pub enum State {
    DrivingUp,
    DrivingDown,
    Stopped,
}

pub struct Roll<DURATION> {
    state: State,
    current: DURATION,
    target: Option<DURATION>,
    bottom: Option<DURATION>,
}

impl<DURATION> Roll<DURATION> {
    pub fn new() -> Self
    where
        DURATION: Default,
    {
        Self {
            state: State::Stopped,
            current: DURATION::default(), //recalibrated when possible
            target: None,
            bottom: None, //recalibrated when possible
        }
    }

    pub fn update(&mut self, delta_t: DURATION) -> State
    where
        DURATION: Default
            + PartialOrd
            + PartialEq
            + Add<DURATION, Output = DURATION>
            + Sub<DURATION, Output = DURATION>
            + Copy,
    {
        if delta_t != DURATION::default() {
            let driving_current_detected = false; //TODO... let roll_motor_current: u16 = adc1.read(&mut roll_motor_current_sense).unwrap();

            match self.state {
                State::Stopped => {}
                State::DrivingDown => {
                    if !driving_current_detected {
                        self.state = State::Stopped;
                        self.bottom = Some(self.current);
                    } else {
                        self.current = self.current + delta_t;

                        if let Some(target) = self.target {
                            if self.current >= target {
                                self.target = None;
                                self.state = State::Stopped;
                            }
                        }
                    }
                }
                State::DrivingUp => {
                    if !driving_current_detected {
                        self.state = State::Stopped;
                        self.current = DURATION::default();
                    } else {
                        if self.current > delta_t {
                            self.current = self.current - delta_t;
                        } else {
                            self.current = DURATION::default();
                        }
                        if let Some(target) = self.target {
                            if self.current <= target {
                                self.target = None;
                                self.state = State::Stopped;
                            }
                        }
                    }
                }
            }
        }
        self.state
    }

    pub fn execute(&mut self, command: Command<DURATION>)
    where
        DURATION: Default + PartialOrd + PartialEq + Copy,
    {
        match command {
            Command::Stop => {
                self.state = State::Stopped;
            }
            Command::SendUp => {
                self.target = None;
                self.state = State::DrivingUp;
            }
            Command::SendDown => {
                self.target = None;
                self.state = State::DrivingDown;
            }
            Command::SendTo(target) => {
                //if let Some(bottom) = self.bottom {
                //let target = (bottom * (target_x_256 as Duration)) >> 8;
                self.target = Some(target);

                if self.current < target {
                    self.state = State::DrivingDown;
                } else if self.current > target {
                    self.state = State::DrivingUp;
                } else {
                    self.state = State::Stopped;
                }
                //}
            }
        }
    }
}
