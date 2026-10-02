import {describe, expect, it} from 'vitest';
import {parseSystem} from './system';

describe('parseSystem', () => {
  it('reads what system.cgi writes', () => {
    const s = parseSystem('remote 0\nmem_total 4000000000\nmem_available 1000000000\ndisk_total 14000000000\ndisk_free 2500000000\nload 0.52 0.40 0.31\ncpus 4\ncpu_temp 51.3\nuptime 90061\n');
    expect(s).toMatchObject({remote: false, memTotal: 4e9, memAvailable: 1e9, diskFree: 2.5e9, load: 0.52, cpus: 4, cpuTemp: 51.3, uptime: 90061});
    expect(s.dataTotal).toBeUndefined();
  });
  it('says when it runs elsewhere', () => {
    expect(parseSystem('remote 1\n').remote).toBe(true);
  });
});
