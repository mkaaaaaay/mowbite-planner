import {tr} from './i18n';
import {RpcError} from './rpc';

// what went wrong with a call to the mower, in words
export function rpcErrorText(e: unknown): string {
  if (e instanceof RpcError) {
    if (e.code === -32601) return tr("Your OpenMower version doesn't offer this ({method}).", {method: e.method});
    if (e.code === 'timeout') return tr("The mower didn't answer.");
  }
  return e instanceof Error ? e.message : tr('failed');
}
