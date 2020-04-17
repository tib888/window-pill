use core::ops::{Add, Sub, Mul, Div};
use embedded_hal::digital::v2::{OutputPin};
use room_pill::roll::{Roll, State};

pub enum Command<DURATION>
{
    Regular(room_pill::roll::Command::<DURATION>),
    Calibrate,
}

enum Mode
{
    Normal,
    CalibrationPhase1,
    CalibrationPhase2,
}

pub struct Valve<DA, DB, DURATION> 
where     
    DA: OutputPin,
    DB: OutputPin,
    DURATION: Default,
{
    mode: Mode,
    state: Roll::<DURATION>,
    drive_time: DURATION,
    allowed_overload_duration: DURATION,    
    motor_drive_a: DA,
    motor_drive_b: DB,
    //motor_current_sense: CS,
}

impl<DA, DB, DURATION> Valve<DA, DB, DURATION> 
where 
    DA: OutputPin,
    DB: OutputPin,
    DURATION: Default + PartialEq + PartialOrd + Copy + Add<DURATION, Output=DURATION> + Sub<DURATION, Output=DURATION> + Div<u32, Output=DURATION> + Mul<u32, Output=DURATION>
{   
    pub fn new (allowed_overload_duration: DURATION, motor_drive_a: DA, motor_drive_b: DB) -> Self {
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

    pub fn command(&mut self, command: Command<DURATION>)
    {
        self.mode = match command {
            Command::Regular(command) => {      
                let modified_command = if let Some(max) = self.state.maximum() {
                    //to avoid overload at the ends, stay away with 1/16 range
                    match command {
                        room_pill::roll::Command::Open => room_pill::roll::Command::SetPosition(*max / 16u32),
                        room_pill::roll::Command::Close => room_pill::roll::Command::SetPosition(*max * 15u32 / 16u32),
                        _ => command,
                    }                    
                } else {
                    command
                };
                self.state.execute(modified_command);
                Mode::Normal
            }                
            Command::Calibrate => Mode::CalibrationPhase1
        }
    }

    pub fn update(&mut self, delta_t: DURATION, overload_detected: bool) {
        if self.state.state() != State::Stopped {
            self.drive_time = self.drive_time + delta_t;
        }
        //update the valve state machine and apply the state on the HW:            
        self.state.update(delta_t, !overload_detected || self.drive_time <= self.allowed_overload_duration);        
        self.apply();

        if self.state.state() == State::Stopped {
            self.mode = match self.mode {
                Mode::CalibrationPhase1 => { self.state.execute(room_pill::roll::Command::Open); Mode::CalibrationPhase2 }
                Mode::CalibrationPhase2 => { self.state.execute(room_pill::roll::Command::Close); Mode::Normal }
                _ => Mode::Normal
            }
        }
    }

    //renders the state on the hardware
    fn apply(&mut self)
    {
        //a=0 b=1 : closing current: [120 -> 250], block: 300..440
        //a=1 b=0 : opening current: [120..160] avg=143, block: 300..440
        match self.state.state() {
            State::Opening => {
                self.motor_drive_b.set_low();
                self.motor_drive_a.set_high();
            }
            State::Closing => {
                self.motor_drive_a.set_low();
                self.motor_drive_b.set_high();
            }
            State::Stopped => {
                self.motor_drive_a.set_low();
                self.motor_drive_b.set_low();
                self.drive_time = DURATION::default();
            }
        }
    }
}
