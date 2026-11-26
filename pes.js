// Small MPEG-TS/PES timestamp offset helper.
// It shifts both PTS and DTS by the same 90 kHz tick count while preserving
// every PES/TS packet.  The dock task only needs parsed timestamp objects in
// most tests; the binary path is included for end-to-end regional offsets.

export const VIDEO_CLOCK_OFFSET_ONE_SECOND = 90_000;
export const PTS_MODULUS = 2 ** 33;

export function shiftClock(value, delta = VIDEO_CLOCK_OFFSET_ONE_SECOND) {
  if (typeof value === 'bigint') {
    const mod = BigInt(PTS_MODULUS);
    return ((value + BigInt(Math.trunc(delta))) % mod + mod) % mod;
  }
  if (typeof value !== 'number' || !Number.isFinite(value)) return value;
  return ((Math.trunc(value + delta) % PTS_MODULUS) + PTS_MODULUS) % PTS_MODULUS;
}

function writeTimestampBytes(bytes, offset, value, firstNibble) {
  const pts = Number(shiftClock(value, 0)); // normalize only; caller adds via shiftClock
  bytes[offset] = (firstNibble & 0xf0) | (((pts >>> 30) & 0x07) << 1) | 0x01;
  bytes[offset + 1] = (pts >>> 22) & 0xff;
  bytes[offset + 2] = ((pts >>> 14) & 0xfe) | 0x01;
  bytes[offset + 3] = (pts >>> 7) & 0xff;
  bytes[offset + 4] = ((pts << 1) & 0xfe) | 0x01;
}

export function writePts(bytes, offset, pts) {
  writeTimestampBytes(bytes, offset, pts, 0x20);
}

export function writeDts(bytes, offset, dts) {
  writeTimestampBytes(bytes, offset, dts, 0x10);
}

export function readTimestampBytes(bytes, offset) {
  return (
    ((bytes[offset] & 0x0e) << 29) |
    (bytes[offset + 1] << 22) |
    ((bytes[offset + 2] & 0xfe) << 14) |
    (bytes[offset + 3] << 7) |
    ((bytes[offset + 4] & 0xfe) >>> 1)
  ) >>> 0;
}

function shiftTimestampField(object, key, delta) {
  if (object?.[key] != null) object[key] = shiftClock(object[key], delta);
}

export function shiftParsedPes(pes, delta = 90_000) {
  if (!pes || typeof pes !== 'object') return pes;
  for (const key of ['pts', 'dts', 'PTS', 'DTS', 'presentationTime', 'decodeTime', 'presentationTimestamp', 'decodeTimestamp']) {
    shiftTimestampField(pes, key, delta);
  }
  if (pes.ptsTime != null && typeof pes.ptsTime === 'number') pes.ptsTime += delta / 90_000;
  if (pes.dtsTime != null && typeof pes.dtsTime === 'number') pes.dtsTime += delta / 90_000;
  return pes;
}

export function shiftPesCollection(packets, delta = 90_000) {
  if (!packets) return packets;
  if (Array.isArray(packets)) {
    for (const packet of packets) {
      if (packet?.type === 'pes' || packet?.pts != null || packet?.dts != null) shiftParsedPes(packet, delta);
    }
  } else if (typeof packets === 'object') {
    for (const key of ['video', 'audio', 'pesPackets', 'packets']) {
      if (Array.isArray(packets[key])) shiftPesCollection(packets[key], delta);
    }
  }
  return packets;
}

function isVideoStreamId(id) {
  return id >= 0xe0 && id <= 0xef;
}

function isAudioStreamId(id) {
  return id >= 0xc0 && id <= 0xdf;
}

export function shiftMpegTsTimestamps(data, delta = 90_000, options = {}) {
  const bytes = data instanceof Uint8Array ? data : new Uint8Array(data);
  const packetSize = options.packetSize ?? 188;
  let videoPes = 0;
  let audioPes = 0;
  let pesCount = 0;

  for (let start = 0; start + packetSize <= bytes.length; start += packetSize) {
    if (bytes[start] !== 0x47) continue;
    const payloadUnitStart = (bytes[start + 1] & 0x40) !== 0;
    if (!payloadUnitStart) continue;

    const adaptationControl = (bytes[start + 3] >>> 4) & 0x03;
    let payloadOffset = start + 4;
    if (adaptationControl === 2 || adaptationControl === 3) {
      payloadOffset += 1 + bytes[start + 4];
    }
    if (adaptationControl === 2 || payloadOffset + 9 > start + packetSize) continue;

    const p = payloadOffset;
    if (bytes[p] !== 0x00 || bytes[p + 1] !== 0x00 || bytes[p + 2] !== 0x01) continue;

    const streamId = bytes[p + 3];
    if (isVideoStreamId(streamId)) videoPes++;
    else if (isAudioStreamId(streamId)) audioPes++;
    else continue;
    pesCount++;

    const ptsDtsFlags = (bytes[p + 7] >>> 6) & 0x03;
    const headerDataLength = bytes[p + 8];
    const header = p + 9;
    if (ptsDtsFlags >= 2) {
      const oldPts = readTimestampBytes(bytes, header);
      writePts(bytes, header, shiftClock(oldPts, delta));
      if (ptsDtsFlags === 3) {
        const dtsOffset = header + 5;
        const oldDts = readTimestampBytes(bytes, dtsOffset);
        writeDts(bytes, dtsOffset, shiftClock(oldDts, delta));
      }
    }
    // headerDataLength is parsed for completeness; in-place timestamp writes
    // do not change header or packet lengths.
    void headerDataLength;
  }

  return { bytes, videoPes, audioPes, pesCount, delta };
}

export function shiftRegionalTimestamps(input, delta = 90_000) {
  if (input instanceof Uint8Array || input instanceof ArrayBuffer) {
    return shiftMpegTsTimestamps(input, delta);
  }
  return shiftPesCollection(input, delta);
}

export const offsetTimestamps = shiftRegionalTimestamps;
export const shiftTimestamps = shiftRegionalTimestamps;

export default {
  VIDEO_CLOCK_OFFSET_ONE_SECOND,
  PTS_MODULUS,
  shiftClock,
  readTimestampBytes,
  writePts,
  writeDts,
  shiftParsedPes,
  shiftPesCollection,
  shiftMpegTsTimestamps,
  shiftRegionalTimestamps,
};
