//! Models the roll operation:
//! using time based positioning in an autocalibrated range

use num_traits::Saturating;

#[derive(Copy, Clone, PartialEq)]
pub enum Command<DURATION> {
    Open,
    Close,
    SetPosition(DURATION), //can be computed from bottom position if known
    Stop,
}

#[derive(Copy, Clone, PartialEq)]
pub enum State {
    Opening,
    Closing,
    Stopped,
}

pub struct Roll<DURATION> {
    state: State,
    current: DURATION,
    target: Option<DURATION>,
    maximum: Option<DURATION>,
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
            maximum: None, //recalibrated when possible
        }
    }

    pub fn state(&self) -> State {
        self.state
    }

    pub fn update(&mut self, delta_t: DURATION, movement_detected: bool)
    where
        DURATION: Default
            + PartialOrd
            + PartialEq
            + Saturating
            + Copy,
    {
        if delta_t != DURATION::default() {
            match self.state {
                State::Stopped => {}
                State::Closing => {
                    self.current = self.current.saturating_add(delta_t);

                    if !movement_detected {
                        self.state = State::Stopped;
                        self.maximum = Some(self.current);
                        self.target = None;
                    } else {
                        if let Some(pos) = self.target {
                            if self.current >= pos {
                                self.target = None;
                                self.state = State::Stopped;
                            }
                        }
                    }
                }
                State::Opening => {
                    self.current = self.current.saturating_sub(delta_t);
                    
                    if !movement_detected {
                        self.state = State::Stopped;
                        self.current = DURATION::default();
                        self.target = None;
                    } else {
                        if let Some(pos) = self.target {
                            if self.current <= pos {
                                self.target = None;
                                self.state = State::Stopped;
                            }
                        }
                    }
                }
            }
        }        
    }

    pub fn execute(&mut self, command: Command<DURATION>)
    where
        DURATION: Default + PartialOrd + PartialEq + Copy,
    {
        match command {
            Command::Stop => {
                self.state = State::Stopped;
            }
            Command::Open => {
                self.target = None;
                self.state = State::Opening;
            }
            Command::Close => {
                self.target = None;
                self.state = State::Closing;
            }
            Command::SetPosition(pos) => {
                if self.current < pos {
                    self.state = State::Closing;
                } else if self.current > pos {
                    self.state = State::Opening;
                } else {
                    self.state = State::Stopped;
                }
                self.target = Some(pos);
            }
        }
    }

    pub fn maximum<'a>(&'a self) -> &'a Option<DURATION> {
        &self.maximum
    }
}
