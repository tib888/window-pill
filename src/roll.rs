//! Models the roll operation

type Position = u32;

#[derive(Copy, Clone, PartialEq)]
pub enum Command<DURATION> {
    TimePassed(DURATION),
    SendUp,
    SendDown,
    SendTo(u8), //up=0..255=down
    Stop,
}

#[derive(Copy, Clone, PartialEq)]
pub enum State {
    DrivingUp,
    DrivingDown,
    Stopped,
}

pub struct Roll {
    state: State,
    current_position: Position,
    target_position: Option<Position>,
    bottom_position: Option<Position>,
}

impl Roll {
    pub fn new() -> Self {
        Self {
            state: State::Stopped,
            current_position: Position::default(), //recalibrated when possible
            target_position: None,
            bottom_position: None, //recalibrated when possible
        }
    }

    pub fn update<DURATION>(&mut self, command: Command<DURATION>) -> State
    where
        DURATION: Into<Position>,
    {
        match command {
            Command::TimePassed(delta_t) => {
                let driving_current_detected = false; //TODO... let roll_motor_current: u16 = adc1.read(&mut roll_motor_current_sense).unwrap();

                match self.state {
                    State::Stopped => {}
                    State::DrivingDown => {
                        if !driving_current_detected {
                            self.state = State::Stopped;
                            self.bottom_position = Some(self.current_position);
                        } else {
                            let delta_pos: Position = delta_t.into();
                            self.current_position += delta_pos;

                            if let Some(target_position) = self.target_position {
                                if self.current_position >= target_position {
                                    self.target_position = None;
                                    self.state = State::Stopped;
                                }
                            }
                        }
                    }
                    State::DrivingUp => {
                        if !driving_current_detected {
                            self.state = State::Stopped;
                            self.current_position = Position::default();
                        } else {
                            let delta_pos: Position = delta_t.into();
                            if self.current_position < delta_pos {
                                self.current_position = self.current_position - delta_pos;
                            } else {
                                self.current_position = Position::default();
                            }
                            if let Some(target_position) = self.target_position {
                                if self.current_position <= target_position {
                                    self.target_position = None;
                                    self.state = State::Stopped;
                                }
                            }
                        }
                    }
                }
            }
            Command::Stop => {
                self.state = State::Stopped;
            }
            Command::SendUp => {
                self.target_position = None;
                self.state = State::DrivingUp;
            }
            Command::SendDown => {
                self.target_position = None;
                self.state = State::DrivingDown;
            }
            Command::SendTo(target) => {
                if let Some(bottom_position) = self.bottom_position {
                    let target_position = (bottom_position * (target as Position)) >> 8;
                    self.target_position = Some(target_position);

                    if self.current_position < target_position {
                        self.state = State::DrivingDown;
                    } else if self.current_position > target_position {
                        self.state = State::DrivingUp;
                    } else {
                        self.state = State::Stopped;
                    }
                }
            }
        }

        self.state
    }
}
