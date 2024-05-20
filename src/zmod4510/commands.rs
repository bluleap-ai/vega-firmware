#[derive(Debug, Copy, Clone)]
/// List of SCD4x sensor commands.
pub enum Command {
    ZmodAddrPid,
    ZmodAddrConf,
    ZmodAddrGeneralPupose,
    ZmodAddrCmd,
    ZmodAddrStatus,
    ZmodAddrTracking,
    StatusSequencerRunningMask,
    // StatusSleepTimerEnabledMask,
    // StatusAlarmMask,
    // StatusLastSeqStepMask,
    // StatusPorEventMask,
    // StatusAccessConflictMask,
}

impl Command {
    pub fn as_byte(self) -> u8 {
        match self {
            Self::ZmodAddrPid => 0x00,
            Self::ZmodAddrConf => 0x20,
            Self::ZmodAddrGeneralPupose => 0x26,
            Self::ZmodAddrCmd => 0x93,
            Self::ZmodAddrStatus => 0x94,
            Self::ZmodAddrTracking => 0x3A,
            Self::StatusSequencerRunningMask => 0x80,
            // Self::StatusSleepTimerEnabledMask => 0x40,
            // Self::StatusAlarmMask => 0x20,
            // Self::StatusLastSeqStepMask => 0x1F,
            // Self::StatusPorEventMask => 0x80,
            // Self::StatusAccessConflictMask => 0x40,
        }
    }
}
