use core::ops::{Add, Sub, Mul, Div};
use embedded_hal::digital::v2::{OutputPin};
use room_pill::roll::{Roll, State};

/// RollTo should be in [0..100] range
pub enum Command
{
    Stop,
    Open,
    Close,
    RollTo(u32),//[0..100]%
}

pub struct Shutter<DA, DB, DURATION> 
where     
    DA: OutputPin,
    DB: OutputPin,
    DURATION: Default,
{
    state: Roll::<DURATION>,
    drive_time: DURATION,
    allowed_overload_duration: DURATION,    
    ssr_roll_down: DA,
    ssr_roll_up: DB,
    //motor_current_sense: CS,
}

impl<DA, DB, DURATION> Shutter<DA, DB, DURATION> 
where 
    DA: OutputPin,
    DB: OutputPin,
    DURATION: Default + PartialEq + PartialOrd + Copy + Add<DURATION, Output=DURATION> + Sub<DURATION, Output=DURATION> + Div<u32, Output=DURATION> + Mul<u32, Output=DURATION>
{   
    pub fn new (allowed_overload_duration: DURATION, ssr_roll_down: DA, ssr_roll_up: DB) -> Self {
        Self {
            state: Roll::<DURATION>::new(),
            drive_time: DURATION::default(),
            allowed_overload_duration: allowed_overload_duration,           
            ssr_roll_down: ssr_roll_down,
            ssr_roll_up: ssr_roll_up,
        }
    }

    pub fn update(&mut self, command: Option<Command>, delta_t: DURATION, power: i32, noise_level: i32, overload_level: i32) {        
        if self.state.state() != State::Stopped {
            self.drive_time = self.drive_time + delta_t;
        }
        if let Some(command) = command {
            let translated = 
                match command {
                    Command::Stop => room_pill::roll::Command::Stop,
                    Command::Open => room_pill::roll::Command::Open,
                    Command::Close => room_pill::roll::Command::Close,
                    Command::RollTo(percent) => if let &Some(max) = self.state.maximum() {
                        room_pill::roll::Command::SetPosition((max * percent / 100).into()) 
                    } else {
                        room_pill::roll::Command::Stop
                    },
                };        
            self.state.execute(translated);
        }

        //moves if eats some current; 
        //if eats none or too many then blocked, so stoped (except allowed the starting tranzient)
        let moving = (power > noise_level && power < overload_level) || self.drive_time <= self.allowed_overload_duration;

        //update the valve state machine and apply the state on the HW:            
        self.state.update(delta_t, moving);
    
        //render the state on the hardware:
        match self.state.state() {
            State::Opening => {
                self.ssr_roll_down.set_low();
                self.ssr_roll_up.set_high();
            }
            State::Closing => {
                self.ssr_roll_up.set_low();
                self.ssr_roll_down.set_high();
            }
            State::Stopped => {
                self.ssr_roll_up.set_low();
                self.ssr_roll_down.set_low();
                self.drive_time = DURATION::default();
            }
        }
    }
    
    pub fn is_moving(&self) -> bool {
        self.state.state() != State::Stopped
    }    
}
