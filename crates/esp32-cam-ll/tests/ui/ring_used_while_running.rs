// LL-01: the CPU cannot touch the ring while the DMA writes into it.
use esp32_cam_ll::{DmaRingStorage, DmaSizeConfig, Esp32I2sCamera};

static RING: DmaRingStorage<4, 8192> = DmaRingStorage::new();

fn main() {
    let mut cam = Esp32I2sCamera::take().expect("I2S0 is free");
    let mut ring = RING.take_ring(2048).expect("ring");
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
    let _ = ring.parts_mut();
    running.stop();
}
