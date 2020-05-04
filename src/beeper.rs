use num_traits::Saturating;

pub struct Beeper<'a, F, T, D> {
    time: D,
    limit: D,
    pos: usize,    
    melody: &'a [(F,T)]
}

impl<'a, F, T, D> Beeper<'a, F, T, D> 
where F: Copy + Default, T: Copy, D: Copy + Ord + Saturating + Default
{
    pub fn update<CT, A>(&mut self, delta_t: D, convert: &CT, apply: &mut A)
        where A: FnMut(F), CT: Fn(T) -> D
    {
        if self.pos > self.melody.len() { 
            return;
        }
            
        if self.pos > 0 {
            self.time = self.time.saturating_add(delta_t);
            if self.time < self.limit { 
                return; 
            }
        }

        if self.melody.len() > self.pos { 
            apply(self.melody[self.pos].0);
            self.limit = self.limit.saturating_add(convert(self.melody[self.pos].1));
        } else {
            apply(F::default());
            return;
        }
        self.pos += 1;
    }

    /// the input array must contain notes: (freq, delay) update will start to play it
    pub fn new(melody: &'a [(F,T)]) -> Beeper<'a, F, T, D>
    {
        Beeper { 
            time: D::default(),
            limit: D::default(),
            pos: 0,
            melody: melody,
        }
    }
}

