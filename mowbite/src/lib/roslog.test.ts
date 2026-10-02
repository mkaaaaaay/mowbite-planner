import {describe, expect, it} from 'vitest';
import {readKeptLog, readRpcLog} from './roslog';

describe('readRpcLog', () => {
  it('keeps the window around the moment, node in front of the text', () => {
    const lines = readRpcLog(
      [
        {t: 90, level: 'WARN', node: '/mower_comms_v2', msg: 'too early'},
        {t: 100, level: 'ERROR', node: '/move_base_flex', msg: 'Timeout in PRE_ROTATE phase.'},
        {t: 110, level: 'INFO', node: '/x', msg: 'not a warning'},
        {t: 120, level: 'fatal', node: '', msg: 'no node'},
        {t: 200, level: 'WARN', node: '/x', msg: 'too late'},
      ],
      95,
      150,
    );
    expect(lines).toEqual([
      {level: 'ERROR', t: 100, text: 'move_base_flex: Timeout in PRE_ROTATE phase.'},
      {level: 'FATAL', t: 120, text: 'no node'},
    ]);
  });

  it('copes with an answer that is not a list', () => {
    expect(readRpcLog({error: 'x'}, 0, 1)).toEqual([]);
  });
});

describe('readKeptLog', () => {
  it('reads the lines logkeeper.sh writes, node like from the rpc', () => {
    expect(
      readKeptLog('1790709655.04 WARN /mower_comms_v2: Firmware version unknown\n1790709700 ERROR /move_base_flex: a: b\ngarbage\n'),
    ).toEqual([
      {level: 'WARN', t: 1790709655.04, text: 'mower_comms_v2: Firmware version unknown'},
      {level: 'ERROR', t: 1790709700, text: 'move_base_flex: a: b'},
    ]);
  });
});
