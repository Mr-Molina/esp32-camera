//! Compile-fail tests for the ownership rules of the I2S0 DMA API (WP-08):
//! the camera is a singleton that cannot be cloned, and a DMA ring cannot be
//! touched, moved or dropped while a `RunningCapture` borrows it.

#[test]
fn ownership_rules_are_enforced_at_compile_time() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/*.rs");
}
