//! Compile-fail tests for the ownership rules of the I2S0 DMA API (WP-08):
//! the camera is a singleton that cannot be cloned, and a DMA ring cannot be
//! touched, moved or dropped while a `RunningCapture` borrows it.

// Not run under Miri: trybuild spawns cargo and rustc to compile the cases,
// which an interpreter with filesystem isolation cannot do (`statx` is the
// first thing it trips on). These are compile-time checks; Miri adds nothing
// to them. The ll unit tests, which are what Miri is for, run there in full.
#![cfg(not(miri))]

#[test]
fn ownership_rules_are_enforced_at_compile_time() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/*.rs");
}
