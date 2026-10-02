import {Buffer} from 'buffer';
import {getMqttClient, withPrefix} from './mqttClient';
import {TOPIC} from './openmower';

// OpenMower takes remote driving on "teleop" as bson {vx, vz} (m/s forward, rad/s counterclockwise).
// The xCore stops the wheels when nothing arrived for a second (diff_drive_service.cpp), so this has to be sent
// over and over.
function twist(vx: number, vz: number): Buffer {
  const buf = Buffer.alloc(29);
  let o = 0;
  buf.writeInt32LE(29, o);
  o += 4;
  for (const [key, v] of [
    ['vx', vx],
    ['vz', vz],
  ] as const) {
    buf.writeUInt8(0x01, o++); // double
    o += buf.write(key + '\0', o, 'latin1');
    buf.writeDoubleLE(v, o);
    o += 8;
  }
  buf.writeUInt8(0, o);
  return buf;
}

export function sendDrive(vx: number, vz: number) {
  getMqttClient().publish(withPrefix(TOPIC.teleop), twist(vx, vz), {qos: 0});
}
