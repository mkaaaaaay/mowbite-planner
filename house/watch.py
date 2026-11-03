#!/usr/bin/env python3
"""The watcher of the MowBite Planner's house computer (remote_planning, mowbite_planner/house.py): runs the house
service with the planner code the mower runs. Which code that is comes from the mower's broker (WANT, retained: the
digest remote.code_version gives there); nothing else of it counts. The code itself comes from REPO only, the planner's
own repository, fixed here: the newest commit there whose package has that digest is exported and the service started
with it, again whenever the mower's code changes or the service ends. Where no commit has it (a fork, a build of one's
own) nothing is started for it: the mower sees no house computer with its code and plans itself.

Environment: the service's (MOWER_MQTT_HOST, MOWER_MQTT_PORT, MOWER_MQTT_USER, MOWER_MQTT_PASSWORD, HOUSE_WORKERS)
and HOUSE_DATA (/data: the clone of REPO and the code exported)."""

import hashlib
import json
import logging
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

REPO = 'https://github.com/mkaaaaaay/mowbite-planner.git'
PACKAGE = 'ros2/mowbite_planner/mowbite_planner'
WANT = 'mowbite/planner/remote/want'
# commits looked at, newest first; s between fetches while no commit has the code wanted, at least between two; codes
# exported kept; s after the service ended before it's started again; bytes of a want at most
COMMITS = 500
FETCH_AGAIN = 600.0
FETCH_LEAST = 60.0
KEPT = 3
RESTART = 10.0
MAX_WANT = 65536

log = logging.getLogger('mowbite_house_watch')


def digest(files: List[Tuple[str, bytes]]) -> str:
    """remote.code_version's digest of the package's files [(name, content)]."""
    sha = hashlib.sha1()
    for name, data in sorted(files):
        if name.endswith('.py'):
            sha.update(name.encode() + b'\0' + data)
    return sha.hexdigest()


def digest_of(folder: str) -> str:
    files = []
    for name in os.listdir(folder):
        if name.endswith('.py'):
            with open(os.path.join(folder, name), 'rb') as f:
                files.append((name, f.read()))
    return digest(files)


def wanted(payload: bytes) -> Optional[str]:
    """The code a want asks for, None: none (empty, not one)."""
    try:
        want = json.loads(payload) if 0 < len(payload) <= MAX_WANT else None
    except ValueError:
        return None
    code = want.get('code') if isinstance(want, dict) else None
    return code if isinstance(code, str) and re.fullmatch(r'[0-9a-f]{40}', code) else None


class Repo:
    """A bare clone of url (REPO) in path, fetched from url itself whatever its config says."""

    def __init__(self, path: str, url: str = REPO) -> None:
        self.path, self.url = path, url
        self.digests: Dict[str, str] = {}  # package tree: its digest

    def git(self, *args: str, data: Optional[bytes] = None) -> bytes:
        return subprocess.run(['git', '-C', self.path] + list(args), input=data, capture_output=True,
                              check=True).stdout

    def fetch(self) -> None:
        if not os.path.isdir(os.path.join(self.path, 'objects')):
            shutil.rmtree(self.path, ignore_errors=True)
            subprocess.run(['git', 'init', '--bare', '--quiet', self.path], capture_output=True, check=True)
        self.git('fetch', '--quiet', '--prune', '--prune-tags', '--force', self.url, '+refs/heads/*:refs/heads/*',
                 '+refs/tags/*:refs/tags/*')

    def find(self, code: str) -> Optional[str]:
        """The newest commit whose package has the digest code (of the last COMMITS), None: none."""
        try:
            commits = self.git('rev-list', '--all', '--date-order', '--max-count=%d' % COMMITS).decode().split()
        except subprocess.CalledProcessError:  # (nothing fetched yet)
            return None
        asked = ''.join('%s:%s\n' % (c, PACKAGE) for c in commits).encode()
        trees = self.git('cat-file', '--batch-check=%(objectname) %(objecttype)', data=asked).decode().splitlines()
        for commit, line in zip(commits, trees):
            tree, _, kind = line.partition(' ')
            if kind != 'tree':
                continue
            if tree not in self.digests:
                self.digests[tree] = self.tree_digest(tree)
            if self.digests[tree] == code:
                return commit
        return None

    def tree_digest(self, tree: str) -> str:
        blobs = []
        for line in self.git('ls-tree', tree).decode().splitlines():
            meta, name = line.split('\t', 1)
            _, kind, sha = meta.split()
            if kind == 'blob' and name.endswith('.py'):
                blobs.append((name, sha))
        out = self.git('cat-file', '--batch', data=''.join(sha + '\n' for _, sha in blobs).encode())
        files, at = [], 0
        for name, _ in blobs:  # ("<sha> blob <size>\n<content>\n" each)
            head, at = out[at:out.index(b'\n', at)], out.index(b'\n', at) + 1
            size = int(head.split()[2])
            files.append((name, out[at:at + size]))
            at += size + 1
        return digest(files)

    def export(self, commit: str, folder: str) -> None:
        """The package of commit as folder/mowbite_planner."""
        tmp = folder + '.tmp'
        shutil.rmtree(tmp, ignore_errors=True)
        os.makedirs(tmp)
        archive = self.git('archive', '--format=tar', commit, PACKAGE)
        subprocess.run(['tar', '-x', '-C', tmp, '--strip-components=%d' % (PACKAGE.count('/'))], input=archive,
                       capture_output=True, check=True)
        shutil.rmtree(folder, ignore_errors=True)
        os.replace(tmp, folder)


class Watcher:
    """What runs: the service on the code the mower wants (step(), every few s on the main thread)."""

    def __init__(self, data: str, repo: Optional[Repo] = None, start=None) -> None:
        self.data = data
        self.repo = repo or Repo(os.path.join(data, 'repo.git'))
        self.start = start or self.popen
        self.code: Optional[str] = None  # the code wanted
        self.running: Optional[str] = None  # the code the service runs on
        self.proc = None
        self.fetched, self.ended = -FETCH_AGAIN, 0.0
        self.missing: Optional[str] = None  # (the code no commit had, said once)
        self.lock = threading.Lock()

    def on_want(self, payload: bytes) -> None:
        code = wanted(payload)
        if code is not None:
            with self.lock:
                self.code = code

    @staticmethod
    def popen(folder: str):
        return subprocess.Popen([sys.executable, '-m', 'mowbite_planner.house'], cwd=folder,
                                env=dict(os.environ, PYTHONPATH=folder))

    def step(self, now: float) -> None:
        with self.lock:
            code = self.code
        ended = self.proc.poll() if self.proc is not None else None
        if ended is not None:
            log.warning('the service on code %s ended (%s)', self.running, ended)
            self.proc, self.running, self.ended = None, None, now
        if code is None or code == self.running or now < self.ended + RESTART:
            return
        folder = self.folder(code, now)
        if folder is None:
            return
        self.stop()
        log.info('the service on code %s (the mower\'s)', code)
        self.proc, self.running = self.start(folder), code

    def folder(self, code: str, now: float) -> Optional[str]:
        """The code exported (fetched and found where it isn't yet), None: no commit has it."""
        folder = os.path.join(self.data, 'code', code)
        if os.path.isdir(folder):
            return folder
        commit = self.repo.find(code)
        if commit is None and now >= self.fetched + (FETCH_AGAIN if self.missing == code else FETCH_LEAST):
            self.fetched = now
            try:
                self.repo.fetch()
            except (OSError, subprocess.CalledProcessError) as e:
                log.warning('%s not fetched: %s', self.repo.url, getattr(e, 'stderr', b'') or e)
            commit = self.repo.find(code)
        if commit is None:
            if self.missing != code:
                log.warning('no commit of %s has code %s: the mower plans itself', self.repo.url, code)
                self.missing = code
            return None
        self.repo.export(commit, folder)
        if digest_of(os.path.join(folder, 'mowbite_planner')) != code:  # (never: the digest is the commit's)
            shutil.rmtree(folder, ignore_errors=True)
            return None
        log.info('code %s is commit %s', code, commit)
        self.tidy(code)
        return folder

    def tidy(self, code: str) -> None:
        """Of the codes exported the last KEPT stay."""
        root = os.path.join(self.data, 'code')
        old = sorted((os.path.getmtime(os.path.join(root, c)), c) for c in os.listdir(root) if c != code)
        for _, c in old[:max(0, len(old) - KEPT + 1)]:
            shutil.rmtree(os.path.join(root, c), ignore_errors=True)

    def stop(self) -> None:
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(15.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc, self.running = None, None


def main() -> None:
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    env = os.environ
    import paho.mqtt.client as mqtt
    watcher = Watcher(env.get('HOUSE_DATA') or '/data')
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    if env.get('MOWER_MQTT_USER'):
        client.username_pw_set(env['MOWER_MQTT_USER'], env.get('MOWER_MQTT_PASSWORD') or None)
    client.on_connect = lambda client, userdata, flags, rc, properties=None: client.subscribe(WANT, 1)
    client.on_message = lambda client, userdata, msg: watcher.on_want(bytes(msg.payload))
    host, port = env.get('MOWER_MQTT_HOST') or '127.0.0.1', int(env.get('MOWER_MQTT_PORT') or 1883)
    log.info('MowBite Planner house computer: the code the mower at %s:%d wants, from %s', host, port, REPO)
    client.connect_async(host, port, keepalive=60)
    client.loop_start()
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))  # (docker stop: the service ends saying so, see stop)
    try:
        while True:
            watcher.step(time.monotonic())
            time.sleep(2.0)
    finally:
        watcher.stop()


if __name__ == '__main__':
    main()
