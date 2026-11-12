// LL-13: I2S0 has exactly one owner; the camera cannot be cloned.
use esp32_cam_ll::Esp32I2sCamera;

fn main() {
    let cam = Esp32I2sCamera::take().expect("I2S0 is free");
    let second_owner = cam.clone();
    drop(second_owner);
}
