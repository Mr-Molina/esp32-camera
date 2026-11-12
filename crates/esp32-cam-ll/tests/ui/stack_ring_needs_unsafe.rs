// LL-01: the safe `start` only accepts `'static` rings, so leaking the
// `RunningCapture` (`mem::forget`) can never let the DMA write into freed
// stack memory. Borrowed rings need the `unsafe` `start_scoped`.
use esp32_cam_ll::{DmaRing, DmaSizeConfig, Esp32I2sCamera, Lldesc};

#[repr(C, align(4))]
struct Buf([u8; 8192]);

fn main() {
    let mut cam = Esp32I2sCamera::take().expect("I2S0 is free");
    let mut desc = [Lldesc::EMPTY; 4];
    let mut buf = Buf([0; 8192]);
    let mut ring = DmaRing::new(&mut desc, &mut buf.0, 2048).expect("ring");
    let cfg = DmaSizeConfig {
        dma_bytes_per_item: 4,
        dma_buffer_size: 8192,
        dma_half_buffer_size: 4096,
        dma_half_buffer_cnt: 2,
        dma_node_buffer_size: 2048,
        dma_node_cnt: 4,
        lines_per_node: 1,
        nodes_per_line: 1,
        lines_per_half_buffer: 1,
        image_size: 1024,
    };
    let running = cam.start(&cfg, &mut ring).expect("start");
    core::mem::forget(running);
}
