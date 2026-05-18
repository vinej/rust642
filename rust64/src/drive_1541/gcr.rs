// GCR (Group Code Recording) — the 4-to-5 bit encoding used on 1541 tracks.
//
// Each 4-bit nibble is encoded as a 5-bit pattern picked so that no run of
// more than two consecutive zero bits occurs in the bitstream. That gives
// the drive's read-data PLL enough transitions to stay locked, and lets the
// SYNC mark (10+ consecutive '1' bits) be unambiguous.
//
// Two raw bytes (4 nibbles) -> 20 bits = 2.5 GCR bytes, so 4 raw bytes pack
// into exactly 5 GCR bytes. A whole sector = 256 data bytes + 4 framing
// bytes = 260 bytes -> 325 GCR bytes.
//
// SYNC: the bit pattern $FF $FF ... ($FF being a literal 8-bit value, not the
// nibble F's 5-bit code) is illegal in normal GCR data, so the drive uses a
// run of 5+ $FF bytes as a marker between fields.

pub const SYNC_BYTE: u8 = 0xFF;

const NIBBLE_TO_GCR: [u8; 16] = [
    0b01010, 0b01011, 0b10010, 0b10011,
    0b01110, 0b01111, 0b10110, 0b10111,
    0b01001, 0b11001, 0b11010, 0b11011,
    0b01101, 0b11101, 0b11110, 0b10101,
];

/// Encode 4 raw bytes into 5 GCR bytes, packing 8 5-bit nibble codes into
/// 40 bits aligned to byte boundaries.
pub fn encode_4_bytes(input: [u8; 4], output: &mut [u8; 5]) {
    let nibbles: [u8; 8] = [
        input[0] >> 4, input[0] & 0x0F,
        input[1] >> 4, input[1] & 0x0F,
        input[2] >> 4, input[2] & 0x0F,
        input[3] >> 4, input[3] & 0x0F,
    ];

    // Pack 8 5-bit values (MSB-first) into a 40-bit big-endian buffer.
    let mut bits: u64 = 0;
    for n in nibbles.iter() {
        bits = (bits << 5) | (NIBBLE_TO_GCR[*n as usize] as u64);
    }
    output[0] = ((bits >> 32) & 0xFF) as u8;
    output[1] = ((bits >> 24) & 0xFF) as u8;
    output[2] = ((bits >> 16) & 0xFF) as u8;
    output[3] = ((bits >>  8) & 0xFF) as u8;
    output[4] = ( bits        & 0xFF) as u8;
}

/// Encode an arbitrary slice of raw bytes whose length is a multiple of 4.
pub fn encode(input: &[u8]) -> Vec<u8> {
    assert!(input.len() % 4 == 0, "GCR input must be a multiple of 4 bytes (got {})", input.len());
    let mut out = Vec::with_capacity(input.len() / 4 * 5);
    let mut buf = [0u8; 5];
    let mut chunk = [0u8; 4];
    for c in input.chunks(4) {
        chunk.copy_from_slice(c);
        encode_4_bytes(chunk, &mut buf);
        out.extend_from_slice(&buf);
    }
    out
}
