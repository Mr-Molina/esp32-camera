//! Safe `#![no_std]` SCCB (Serial Camera Control Bus) abstraction,
//! `embedded-hal` 1.0 adapter, and zero-heap [`MockSccbBus`].

#![no_std]
#![forbid(unsafe_code)]

pub mod adapter;
pub mod bus;
pub mod mock;

pub use crate::adapter::EmbeddedHalSccb;
pub use crate::bus::{is_valid_sccb_addr, with_ov_group_hold, SccbBus, OV_GROUP_HOLD_REG};
pub use crate::mock::{
    MockSccbBus, RegWidth, SccbError, SccbLogEntry, SccbOp, SccbTransaction, UnsetReadPolicy,
    MAX_MOCK_REGS,
};
