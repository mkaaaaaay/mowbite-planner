#!/usr/bin/env python3
"""mkay's ranking over bench runs, the first one the reference: the gates, then rank by rank (two-stage bootstrap).
score.py REF [RUN...] [--groups] [--cases] [--md OUT] [--strict] [--exclude CASEFILE] [--cpu-gate] [--cpu-sum]
[--cpu-budget S] [--time-limit S] [--recheck OUT]"""
import argparse
import hashlib
import json
import math
import os
import statistics
import sys
import warnings
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench import case_id  # noqa: E402

FAIL = ('crash', 'timeout', 'refused', 'stuck', 'sim_error', 'aborted', 'gap')
HOME_SPEED = 0.35  # m/s, the drive home in rank 7
BOOT, KEEP = 2000, 0.975
SPIN_M2, PIR_M2 = 0.001, 0.005  # m2 a spin / a pirouette is worth against rank 1
REPLY_MAX = 1_900_000  # bytes (Mosquitto drops from about 2 MB, the app sends 1.9 MB at most)
DEEP_MORE, CPU_MORE = 0.01, 0.10
STRICT = False
TIME_LIMIT: Optional[float] = None  # s of planning a caller waits at most (--time-limit)


def _num(v: Any) -> Optional[float]:
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def rk(rec: Dict[str, Any], key: str) -> Optional[float]:
    return _num((rec.get('rank') or {}).get(key))


def failed(rec: Optional[Dict[str, Any]]) -> bool:
    return rec is None or rk(rec, 'lawn_left_m2') is None or \
        any(a['kind'] in FAIL for a in rec.get('anomalies', []))


def sims(rec: Optional[Dict[str, Any]]) -> Dict[int, Dict[str, Any]]:
    return {s['seed']: s for s in (rec or {}).get('sim') or []}


def group_of(case: str) -> str:
    if case.startswith(('real-', 'export:')):
        return 'echt'
    if case.startswith(('model/', 'torture/')):
        return 'handgemacht'
    return 'erfunden@' + mower_of(case)


def mower_of(case: str) -> str:
    return case.rsplit('@', 1)[-1].split('#')[0].split('+')[0]


class Row:
    """A measure per case (sim rows per seed) with its threshold; how: 'decide', 'check' (a later rank worse) or
    'info'; by: the sim row a plan row must not go against."""

    def __init__(self, rank: int, name: str, get: Callable, sim: bool = False, better: int = -1,
                 tol: Callable[[float], float] = lambda t: 0.0, how: str = 'decide', fail: Optional[str] = None,
                 weigh: bool = False, mean: bool = False, floor: Optional[Callable[[float], float]] = None,
                 by: Optional[str] = None):
        self.rank, self.name, self.get, self.sim, self.better, self.tol = rank, name, get, sim, better, tol
        self.how, self.fail, self.weigh, self.mean, self.floor, self.by = how, fail, weigh, mean, floor, by

    def value(self, rec: Dict[str, Any], sim: Optional[Dict[str, Any]], raw: bool = False) -> Optional[float]:
        v = self.get(rec, sim)
        if v is None or raw or not self.weigh or STRICT:
            return v
        spins = _num(sim.get('spins')) if sim is not None else rk(rec, 'spins_geo')
        return v + SPIN_M2 * (spins or 0.0) + PIR_M2 * ((rk(rec, 'pir_spot') or 0.0) + (rk(rec, 'pir_nothing') or 0.0))

    @property
    def label(self) -> str:
        return self.name + (' samt Drehen und Pirouetten' if self.weigh and not STRICT else '')


def rel(share: float) -> Callable[[float], float]:
    return lambda total: share * abs(total)


def _sum(*keys: str) -> Callable:
    def get(rec, sim):
        vals = [rk(rec, k) for k in keys]
        return None if any(v is None for v in vals) else sum(vals)
    return get


def _sim(key: str) -> Callable:
    return lambda rec, sim: _num(sim.get(key))


def _home(rec, sim):
    s, home = _num(sim.get('s')), rk(rec, 'home_m')
    return None if s is None or home is None else (s + home / HOME_SPEED) / 3600.0


SIM_LEFT = 'Rest Sim m² (Rasen wie gezeichnet, Stücke ≥ 1 cm²)'
SIM_STRIP = 'Randlänge mit Reststreifen ≥ 3 cm Sim m'
ROWS = [
    Row(1, SIM_LEFT, _sim('lawn_left_m2'), sim=True, tol=rel(0.0075), fail='lawn_m2', weigh=True,
        floor=lambda t: 0.15),
    Row(1, 'Rest Plan m²', _sum('lawn_left_m2'), tol=rel(0.0045), fail='lawn_m2', weigh=True,
        floor=lambda t: 0.15, by=SIM_LEFT),
    Row(1, 'Rest Plan, Fahrten ohne Messer m²', _sum('off_left_m2'), tol=rel(0.0045), fail='lawn_m2',
        how='check'),
    Row(2, SIM_STRIP, _sim('strip_m'), sim=True, tol=rel(0.01), fail='lines_m',
        floor=lambda t: max(0.5, 0.02 * abs(t))),
    Row(2, 'Randlänge mit Reststreifen ≥ 3 cm Plan m', _sum('strip_m'), tol=rel(0.01), fail='lines_m',
        floor=lambda t: max(0.5, 0.02 * abs(t)), by=SIM_STRIP),
    Row(2, 'Rest im Randband Sim m²', _sim('band_left_m2'), sim=True, tol=rel(0.01), fail='band_m2', how='check'),
    Row(2, 'Rest im Randband Plan m²', _sum('band_left_m2'), tol=rel(0.01), fail='band_m2', how='check'),
    Row(3, 'Pirouetten (Plan)', _sum('pir_spot', 'pir_nothing'), tol=lambda t: 3.0, floor=lambda t: 3.0),
    Row(3, 'Drehen auf der Stelle Sim', _sim('spins'), sim=True, tol=rel(0.03),
        floor=lambda t: max(10.0, 0.03 * abs(t))),
    Row(3, 'Drehen auf der Stelle Plan (geometrisch)', _sum('spins_geo'), how='info'),
    Row(4, 'Streifenblöcke (Ersatz)', _sum('lane_runs'), how='info'),
    Row(5, 'Knicke > 45° auf 15 cm', _sum('kinks'), tol=rel(0.03)),
    Row(7, 'Sim-Dauer + Heimfahrt h', _home, sim=True, tol=rel(0.005)),
    Row(8, 'Dockwege m', _sum('dock_in_geo_m', 'dock_out_geo_m'), tol=rel(0.05)),
    Row(9, 'Schleifen um nichts (Ersatz)', _sum('pir_nothing'), how='info'),
    Row(10, 'Leerfahrt m (Messerstreifen < 10 % neu)', _sum('idle_m'), tol=rel(0.03)),
    Row(13, 'Mähfaktor (Weg × Messer / gemäht − 1), Mittel', _sum('mow_factor'), tol=lambda t: 0.01, mean=True),
]


# loading ------------------------------------------------------------------------------------------------------------

def load(run: str) -> Tuple[Dict[str, Dict[str, Any]], str]:
    path = run if run.endswith('.json') else os.path.join(run, 'results.json')
    with open(path, 'rb') as f:
        raw = f.read()
    data = json.loads(raw)
    return {r.get('case_id') or case_id(r['case']): r for r in data['cases']}, hashlib.sha256(raw).hexdigest()


def read_cases(path: str) -> List[str]:
    with open(path) as f:
        return [case_id(line.strip()) for line in f if line.strip()]


class Pair:
    """A row's values of a run against the reference: per case and seed, a failed case its fail value."""

    def __init__(self, row: Row, cases: List[str], ref: Dict[str, Dict], new: Dict[str, Dict],
                 fails: Dict[str, Dict[str, float]]):
        self.row, self.cases = row, cases
        seeds = sorted({s for c in cases for s in sims(ref.get(c))}) if row.sim else [None]
        self.seeds = seeds or [None]
        k = len(self.seeds)
        self.a, self.b = np.full((len(cases), k), np.nan), np.full((len(cases), k), np.nan)
        raw = np.full((len(cases), k), np.nan)  # (the reference without the rates: what thresholds are taken of)
        for i, c in enumerate(cases):
            va, vb = self._of(ref.get(c), fails.get(c, {})), self._of(new.get(c), fails.get(c, {}))
            if va is None and vb is None:
                continue
            vr = self._of(ref.get(c), fails.get(c, {}), True) if va is not None else None
            va, vb = va if va is not None else vb, vb if vb is not None else va
            vr = vr if vr is not None else self._of(new.get(c), fails.get(c, {}), True) or {}
            for j, s in enumerate(self.seeds):
                if s in va and s in vb:
                    self.a[i, j], self.b[i, j], raw[i, j] = va[s], vb[s], vr.get(s, va[s])
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            self.ma, self.mb = np.nan_to_num(np.nanmean(self.a, 1)), np.nan_to_num(np.nanmean(self.b, 1))
            self.mr = np.nan_to_num(np.nanmean(raw, 1))
        self.d = self.b - self.a
        self.scale = 1.0 / max(1, len(cases)) if row.mean else 1.0

    def _of(self, rec: Optional[Dict], fail: Dict[str, float], raw: bool = False) -> Optional[Dict[Any, float]]:
        if failed(rec):
            v = fail.get(self.row.fail) if self.row.fail else None
            return None if v is None else {s: v for s in self.seeds}
        if not self.row.sim:
            v = self.row.value(rec, None, raw)
            return None if v is None else {None: v}
        got = {s: self.row.value(rec, sim, raw) for s, sim in sims(rec).items()}
        return {s: v for s, v in got.items() if v is not None}

    def totals(self, idx: Optional[List[int]] = None) -> Tuple[float, float]:
        sel = slice(None) if idx is None else idx
        return float(self.ma[sel].sum()) * self.scale, float(self.mb[sel].sum()) * self.scale

    def per_seed(self, idx: Optional[List[int]] = None) -> List[float]:
        """The paired difference of the total, seed by seed."""
        d = self.d if idx is None else self.d[idx]
        return [float(np.nansum(d[:, j])) * self.scale for j in range(d.shape[1]) if not np.isnan(d[:, j]).all()]

    def boot(self, rng: np.random.Generator) -> np.ndarray:
        """The paired difference of the total, seeds drawn with replacement, then cases."""
        n, k = self.d.shape
        out = np.empty(BOOT)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            whole = np.nan_to_num(np.nanmean(self.d, 1))
            for i in range(BOOT):
                m = np.nanmean(self.d[:, rng.integers(0, k, k)], 1)
                m = np.where(np.isnan(m), whole, m)
                out[i] = m[rng.integers(0, n, n)].sum() * self.scale
        return out


def judge(p: Pair, rng: np.random.Generator) -> Dict[str, Any]:
    ta, tb = p.totals()
    tol = p.row.tol(float(p.mr.sum()) * p.scale)
    delta = tb - ta
    bs = p.boot(rng) * p.row.better  # (> 0: better)
    good, bad = float(np.mean(bs > tol)), float(np.mean(-bs > tol))
    verdict = 'gleich'
    if delta * p.row.better > tol and good >= KEEP:
        verdict = 'BESSER'
    elif -delta * p.row.better > tol and bad >= KEEP:
        verdict = 'SCHLECHTER'
    lo, hi = sorted((float(np.quantile(bs, 1 - KEEP)) * p.row.better, float(np.quantile(bs, KEEP)) * p.row.better))
    return {'ref': ta, 'new': tb, 'delta': delta + 0.0, 'tol': tol, 'lo': lo + 0.0, 'hi': hi + 0.0, 'verdict': verdict}


# gates --------------------------------------------------------------------------------------------------------------

def _more(cases, ref, new, get) -> List[str]:
    """The cases where new has more than the reference (neither failed)."""
    return [c for c in cases if not failed(ref.get(c)) and not failed(new.get(c)) and
            (get(new[c]) or 0) > (get(ref[c]) or 0)]


def _total(cases, recs, get) -> float:
    return sum(get(recs[c]) or 0 for c in cases if not failed(recs.get(c)))


def body_sim(cases, ref, new, n5: str = 'rm_n5', deepest: str = 'rm_worst', what: str = 'Sim-Körper',
             where: str = 'in R_m') -> Tuple[List[str], List[str]]:
    """The simulation's body (E6): (what fails it, what to know). Off the outermost loop (rm_n5, rm_worst; the contact
    with the edge r0_n and the navigation errors there too), or on it (n5 ring_n5, deepest ring_worst: out of the
    recorded space; its contact ring_r0_n). mkay 01.11.: where the body gets in the margins or over the edge is told,
    not a gate (with RTK the margins are the user's buffer); the navigation errors still are."""
    bad, notes, told = [], [], []
    ring = n5 != 'rm_n5'
    both = [c for c in cases if not failed(ref.get(c)) and not failed(new.get(c))]
    seeds = sorted({s for c in both for s in sims(ref.get(c))})
    if not seeds:
        return bad, ['keine Simulation']
    per_seed, cases_up = [], []
    for s in seeds:
        d = 0.0
        for c in both:
            a, b = sims(ref[c]).get(s), sims(new[c]).get(s)
            if a is not None and b is not None:
                d += (b.get(n5) or 0) - (a.get(n5) or 0)
        per_seed.append(d)
    for c in both:
        a, b = sims(ref[c]), sims(new[c])
        common = sorted(set(a) & set(b))
        if common and all((b[s].get(n5) or 0) > (a[s].get(n5) or 0) for s in common):
            cases_up.append(c)
    sd = statistics.stdev(per_seed) if len(per_seed) >= 2 else None
    notes.append(f'{what}: Stellen ≥ 5 cm {where}, Δ je Seed ' + ', '.join(f'{d:+.0f}' for d in per_seed) +
                 (f' (SD {sd:.2f})' if sd is not None else ' (1 Seed: keine SD, nicht gegen das Rauschen geprüft)'))
    if sd is not None and all(d > 0 and d > 2 * sd for d in per_seed):
        told.append(f'{what}: Anstieg in jeder Seed über 2 SD ({", ".join(f"{d:+.0f}" for d in per_seed)})')
    if cases_up:
        told.append(f'{what}: {len(cases_up)} Fälle mit mehr Stellen in jeder Seed ({", ".join(cases_up[:4])})')

    def worst(recs, key):
        return max([s.get(key) or 0.0 for c in both for s in sims(recs[c]).values()] or [0.0])

    wa, wb = worst(ref, deepest), worst(new, deepest)
    if wb > wa + DEEP_MORE:
        told.append(f'{what}: tiefste Stelle {100 * wb:.1f} cm, Referenz {100 * wa:.1f} cm')
    contact = 'ring_r0_n' if ring else 'r0_n'
    ca, cb = (sum(s.get(contact) or 0 for c in both for s in sims(recs[c]).values()) for recs in (ref, new))
    # (as rare events of the same rate both ways, the new run's share of them binomial: how likely this many or more)
    p = sum(math.comb(ca + cb, k) for k in range(cb, ca + cb + 1)) / 2 ** (ca + cb) if ca + cb else 1.0
    notes.append(f'{what}: Kontakt mit der Kante {ca} -> {cb} Stellen über {len(seeds)} Seeds '
                 f'(so viele oder mehr bei gleicher Rate: p = {p:.2f})')
    notes += [f'Bericht (kein Tor, mkay 01.11.): {line}' for line in told]
    if ring:
        return bad, notes
    if cb > ca:
        notes.append(f'Bericht (kein Tor, mkay 01.11.): {what}: Kontakt mit der Kante {ca} -> {cb} Stellen')
    nav = _more(cases, ref, new, lambda r: sum(s.get('nav_errors') or 0 for s in sims(r).values()))
    if nav:
        bad.append(f'Navigationsfehler: {len(nav)} Fälle mehr als die Referenz ({", ".join(nav[:4])})')
    return bad, notes


def gates(cases, ref, new, opts) -> Tuple[List[str], List[str]]:
    bad, notes = [], []
    ok = [c for c in cases if not failed(new.get(c))]
    dup = [c for c in ok if (rk(new[c], 'dup_steps') or 0) > 0]
    if dup:
        bad.append(f'Messvoraussetzung: Schritte < 1e-7 m in {len(dup)} Fällen ({", ".join(dup[:4])})')
    cfg = [c for c in ok if (new[c].get('rank') or {}).get('cfg_diff')]
    if cfg:
        notes.append(f'Plan nennt andere Maße als die Anfrage in {len(cfg)} Fällen (gemessen wird mit der Anfrage)')
    out = _total(cases, new, lambda r: rk(r, 'rm_out'))
    if out:
        bad.append(f'feste Regel: Körper > 1 cm aus R_m im Plan an {out:.0f} Stellen (abseits des Außenrings)')
    out = _total(cases, new, lambda r: rk(r, 'ring_out'))
    if out:
        bad.append(f'feste Regel: Außenring > 1 cm aus dem Aufnahme-Raum im Plan an {out:.0f} Stellen')
    notes.append(f'Außenring: Plankörper > 1 cm aus R_m an {_total(cases, new, lambda r: rk(r, "ring_rm")):.0f} '
                 f'Stellen (Referenz {_total(cases, ref, lambda r: rk(r, "ring_rm")):.0f}; erlaubt bis zum '
                 'Aufnahme-Raum)')
    # (spin_margin: as mkay counts it (31.10.), on the loops turning on the spot only; to know, all of them)
    raw = _more(cases, ref, new, lambda r: rk(r, 'rm_turn_out'))
    notes.append(f'Körper im spin_margin roh (an Ringen auch Bögen): '
                 f'{_total(cases, ref, lambda r: rk(r, "rm_turn_out")):.0f} -> '
                 f'{_total(cases, new, lambda r: rk(r, "rm_turn_out")):.0f} Stellen, {len(raw)} Fälle mehr als die '
                 f'Referenz' + (f' ({", ".join(raw[:4])})' if raw else '') +
                 f'; nach Regel {_total(cases, ref, lambda r: rk(r, "rm_turn_rule")):.0f} -> '
                 f'{_total(cases, new, lambda r: rk(r, "rm_turn_rule")):.0f}')
    for key, what in (('rm_turn_rule', 'Körper im spin_margin an Wenden/Ecken (an Ringen nur Drehen auf der Stelle)'),
                      ('centre_out', 'Mitte außerhalb der erlaubten Flächen'),
                      ('backwards', 'Schritte gegen den Kurs'), ('reverse_segs', 'Rückwärts-Segmente')):
        more = _more(cases, ref, new, lambda r, k=key: rk(r, k))
        if more:
            bad.append(f'feste Regel: {what}, {len(more)} Fälle mehr als die Referenz ({", ".join(more[:4])})')
        ra = _total(cases, ref, lambda r, k=key: rk(r, k))
        if ra:
            notes.append(f'Referenz selbst: {what} {ra:.0f}')
    for args in ((), ('ring_n5', 'ring_worst', 'Sim-Körper Außenring', 'aus dem Aufnahme-Raum')):
        b, n = body_sim(cases, ref, new, *args)
        bad += b
        notes += n
    lost = [c for c in cases if failed(new.get(c)) and not failed(ref.get(c))]
    if lost:
        bad.append(f'Robustheit: {len(lost)} neue Ausfälle ({", ".join(lost[:4])})')
    jumps = _more(cases, ref, new, lambda r: r.get('sections'))
    if jumps:
        bad.append(f'Robustheit: mehr Sektionen in {len(jumps)} Fällen ({", ".join(jumps[:4])})')
    fp = [c for c in ok if (new[c].get('rank') or {}).get('fp_w3') not in (None, (new[c].get('rank') or {}).get('fp'))]
    if fp:
        bad.append(f'Robustheit: mit workers 3 anderer Plan in {len(fp)} Fällen ({", ".join(fp[:4])})')
    # (where the reference's answer passes REPLY_MAX too, no bigger than it)
    big = [c for c in ok if (rk(new[c], 'reply_bytes') or 0) >= REPLY_MAX and
           (failed(ref.get(c)) or (rk(new[c], 'reply_bytes') or 0) > (rk(ref[c], 'reply_bytes') or 0))]
    if big:
        bad.append(f'Robustheit: Antwort ≥ {REPLY_MAX / 1e6:.1f} MB in {len(big)} Fällen ({", ".join(big[:4])})')
    over = [c for c in cases if not failed(ref.get(c)) and (rk(ref[c], 'reply_bytes') or 0) >= REPLY_MAX]
    if over:
        notes.append(f'Referenz selbst: Antwort ≥ {REPLY_MAX / 1e6:.1f} MB in {len(over)} Fällen '
                     f'({", ".join(over[:4])})')
    if opts.cpu_gate:
        ca = [rk(ref[c], 'plan_cpu_s') or 0 for c in cases if not failed(ref.get(c))]
        cb = [rk(new[c], 'plan_cpu_s') or 0 for c in ok]
        if max(cb, default=0) > (1 + CPU_MORE) * max(ca, default=0):
            bad.append(f'CPU: max je Fläche {max(cb):.1f} s, Referenz {max(ca):.1f} s')
        if opts.cpu_sum and sum(cb) > (1 + CPU_MORE) * sum(ca):
            bad.append(f'CPU: Summe {sum(cb):.0f} s, Referenz {sum(ca):.0f} s')
        if opts.cpu_budget is not None and max(cb, default=0) > opts.cpu_budget:
            bad.append(f'CPU: max je Fläche {max(cb):.1f} s über dem Budget {opts.cpu_budget:.1f} s')
    return bad, notes


# ranking ------------------------------------------------------------------------------------------------------------

def fail_values(runs: List[Dict[str, Dict]]) -> Dict[str, Dict[str, float]]:
    """What a failed case leaves: its lawn, lines, band, from any run that planned it (else the outline)."""
    out: Dict[str, Dict[str, float]] = {}
    for recs in runs:
        for c, r in recs.items():
            for k in ('lawn_m2', 'lines_m', 'band_m2'):
                v = rk(r, k)
                if v is not None:
                    out.setdefault(c, {}).setdefault(k, v)
    for recs in runs:
        for c, r in recs.items():
            if 'lawn_m2' not in out.get(c, {}) and _num(r.get('outline_m2')) is not None:
                out.setdefault(c, {})['lawn_m2'] = float(r['outline_m2'])
    return out


def ranking(cases, ref, new, fails) -> List[Tuple[Row, Dict[str, Any]]]:
    rng = np.random.default_rng(0)
    return [(row, judge(Pair(row, cases, ref, new, fails), rng)) for row in ROWS if row.how != 'info']


def decide(judged: List[Tuple[Row, Dict[str, Any]]]) -> Tuple[Optional[Row], Dict[str, Any], List[str]]:
    """The first rank that decides, and the later ones worse beyond their threshold."""
    by_name = {row.name: j for row, j in judged}
    first, got = None, {}
    for row, j in judged:
        if row.how != 'decide' or j['verdict'] == 'gleich':
            continue
        sim = by_name.get(row.by) if row.by else None
        if sim is not None and sim['delta'] * (j['new'] - j['ref']) < 0:
            continue  # (the simulation points the other way)
        first, got = row, j
        break
    later, after = [], False
    for row, j in judged:  # (rows that only check count wherever they are)
        if (after or row.how == 'check') and row is not first and j['verdict'] == 'SCHLECHTER':
            later.append(f'Rang {row.rank} {row.label}: {_fmt(j["ref"])} -> {_fmt(j["new"])}')
        after = after or (first is not None and row is first)
    return first, got, later


def vetoes(cases, ref, new, fails) -> List[str]:
    out = []
    groups = {'echt': [i for i, c in enumerate(cases) if group_of(c) == 'echt']}
    for m in sorted({mower_of(c) for c in cases}):
        groups['Mäher ' + m] = [i for i, c in enumerate(cases) if mower_of(c) == m]
    rows = [r for r in ROWS if r.rank <= 3 and r.how == 'decide']
    pairs = [Pair(r, cases, ref, new, fails) for r in rows]
    for g, idx in groups.items():
        if not idx:
            continue
        for row, p in zip(rows, pairs):
            ta, tb = p.totals(idx)
            seeds = p.per_seed(idx)
            sd = statistics.stdev(seeds) if row.sim and len(seeds) >= 2 else 0.0
            tol = max(row.floor(ta) if row.floor else 0.0, 2 * sd)
            d = (tb - ta) * row.better
            if abs(d) <= tol:
                continue
            if d < 0:
                out.append(f'{g}: Rang {row.rank} {row.label} {ta:.2f} -> {tb:.2f} (Schwelle {tol:.2f})')
            break
    return out


# output -------------------------------------------------------------------------------------------------------------

def _fmt(v: float) -> str:
    return f'{v:.2f}' if abs(v) < 1000 else f'{v:.0f}'


def _seed_mean(rec, key):
    vals = [s.get(key) or 0 for s in sims(rec).values()]
    return sum(vals) / len(vals) if vals else 0.0


def table(names: List[str], runs: List[Dict[str, Dict]], cases: List[str], fails) -> List[str]:
    out = ['| Rang | Messgröße | ' + ' | '.join(names) + ' |', '|---|---|' + '---:|' * len(names)]

    def line(rank, name, cells):
        out.append(f'| {rank} | {name} | ' + ' | '.join(cells) + ' |')

    def ok(recs):
        return [recs[c] for c in cases if not failed(recs.get(c))]

    def tot(key):
        return [_fmt(_total(cases, r, lambda x: rk(x, key))) for r in runs]

    def sim_all(r, key):
        return [s.get(key) or 0 for x in ok(r) for s in sims(x).values()]

    line('fest', 'Körper > 1 cm aus R_m im Plan, abseits des Außenrings (Stellen)', tot('rm_out'))
    line('fest', 'Außenring > 1 cm aus dem Aufnahme-Raum im Plan (Stellen)', tot('ring_out'))
    line('info', 'Außenring > 1 cm aus R_m im Plan (Stellen)', tot('ring_rm'))
    line('fest', 'Körper im spin_margin an Wenden/Ecken nach Regel (an Ringen nur Drehen auf der Stelle), Plan '
                 '(Stellen)', tot('rm_turn_rule'))
    line('info', 'Körper im spin_margin an Wenden/Ecken roh, Plan (Stellen)', tot('rm_turn_out'))
    line('fest', 'Mitte außerhalb erlaubt (Posen)', tot('centre_out'))
    line('fest', 'Schritte gegen den Kurs', tot('backwards'))
    line('Bericht', 'Sim: Stellen ≥ 5 cm in R_m, Mittel der Seeds (Fälle)', [
        f'{sum(_seed_mean(x, "rm_n5") for x in ok(r)):.1f} ({sum(1 for x in ok(r) if _seed_mean(x, "rm_n5") > 0)})'
        for r in runs])
    line('Bericht', 'Sim: tiefste Stelle cm', [f'{100 * max(sim_all(r, "rm_worst") or [0]):.1f}' for r in runs])
    line('Bericht', 'Sim: Kontakt ≥ 2 cm hinter R0 (Stellen, alle Seeds)', [str(sum(sim_all(r, 'r0_n'))) for r in runs])
    line('Bericht', 'Sim Außenring: Stellen ≥ 5 cm aus dem Aufnahme-Raum, Mittel der Seeds (Fälle)', [
        f'{sum(_seed_mean(x, "ring_n5") for x in ok(r)):.1f} ({sum(1 for x in ok(r) if _seed_mean(x, "ring_n5") > 0)})'
        for r in runs])
    line('Bericht', 'Sim Außenring: tiefste Stelle cm',
         [f'{100 * max(sim_all(r, "ring_worst") or [0]):.1f}' for r in runs])
    line('info', 'Sim Außenring: Kontakt ≥ 2 cm hinter R0 (Stellen, alle Seeds)',
         [str(sum(sim_all(r, 'ring_r0_n'))) for r in runs])
    line('fest', 'Sim: Navigationsfehler (alle Seeds)', [str(sum(sim_all(r, 'nav_errors'))) for r in runs])
    line(11, 'Ausfälle (fehlend zählt mit)', [str(sum(failed(r.get(c)) for c in cases)) for r in runs])
    line(11, 'Sektionen', [str(sum(x.get('sections') or 0 for x in ok(r))) for r in runs])
    line(11, 'Antwort max MB', [f'{max([rk(x, "reply_bytes") or 0 for x in ok(r)] or [0]) / 1e6:.2f}' for r in runs])
    for row in ROWS:
        line(row.rank, row.label + (' (info)' if row.how == 'info' else ''),
             [_fmt(Pair(row, cases, runs[0], r, fails).totals()[1]) for r in runs])
    cpu = [[rk(x, 'plan_cpu_s') or 0 for x in ok(r)] for r in runs]
    line(12, 'Planung CPU s: Summe / p95 / max', [
        f'{sum(v):.0f} / {np.percentile(v, 95) if v else 0:.1f} / {max(v or [0]):.1f}' for v in cpu])
    line(12, 'Spitzen-RSS MB (max)', [f'{max([rk(x, "plan_rss_mb") or 0 for x in ok(r)] or [0]):.0f}' for r in runs])
    wall = [[x.get('plan_s') or 0 for x in ok(r)] for r in runs]
    if TIME_LIMIT:
        line(12, f'Wanduhr s: max (Fälle über {TIME_LIMIT:.0f} s)',
             [f'{max(v or [0]):.1f} ({sum(t > TIME_LIMIT for t in v)})' for v in wall])
    else:
        line(12, 'Wanduhr s: max', [f'{max(v or [0]):.1f}' for v in wall])
    return out


def seed_noise(cases, recs, fails) -> List[str]:
    """How much a group's total of the sim rows moves from seed to seed in one run, and the veto threshold that gives
    a candidate as good as it (its paired difference spreads by sqrt 2 of that): the seed thresholds of 2.5."""
    groups = {'alle': list(range(len(cases))), 'echt': [i for i, c in enumerate(cases) if group_of(c) == 'echt']}
    for m in sorted({mower_of(c) for c in cases}):
        groups['Mäher ' + m] = [i for i, c in enumerate(cases) if mower_of(c) == m]
    rows = [r for r in ROWS if r.sim and r.how == 'decide']
    out = ['| Gruppe | ' + ' | '.join(f'{r.label}: SD, Schwelle' for r in rows) + ' |', '|---|' + '---:|' * len(rows)]
    for g, idx in groups.items():
        if not idx:
            continue
        cells = []
        for row in rows:
            p = Pair(row, cases, recs, recs, fails)
            totals = [float(np.nansum(p.a[idx, j])) * p.scale for j in range(p.a.shape[1])
                      if not np.isnan(p.a[idx, j]).all()]
            sd = statistics.stdev(totals) if len(totals) >= 2 else 0.0
            tol = max(row.floor(p.totals(idx)[0]) if row.floor else 0.0, 2 * math.sqrt(2) * sd)
            cells.append(f'{sd:.3f}, {tol:.2f}')
        out.append(f'| {g} ({len(idx)}) | ' + ' | '.join(cells) + ' |')
    return out


def compare(name: str, ref_name: str, cases, ref, new, fails, drop, opts) -> List[str]:
    lines = [f'### {name} gegen {ref_name}', '']
    extra = sorted(set(new) - set(ref) - drop)
    if extra:
        lines.append(f'- {len(extra)} Fälle nur in {name}, nicht gezählt')
    bad, notes = gates(cases, ref, new, opts)
    judged = ranking(cases, ref, new, fails)
    first, got, later = decide(judged)
    veto = vetoes(cases, ref, new, fails)
    lines += [f'- Hinweis: {n}' for n in notes] + [f'- Tor verletzt: {b}' for b in bad]
    lines += ['', '| Rang | Messgröße | Ref | neu | Δ | Schwelle | Bootstrap 2,5 … 97,5 % | Urteil |',
              '|---|---|---:|---:|---:|---:|---:|---|']
    for row, j in judged:
        lines.append(f'| {row.rank} | {row.label}{" (prüft nur)" if row.how == "check" else ""} | {_fmt(j["ref"])} | '
                     f'{_fmt(j["new"])} | {j["delta"]:+.2f} | {j["tol"]:.2f} | {j["lo"]:+.2f} … {j["hi"]:+.2f} | '
                     f'{j["verdict"]} |')
    if bad:
        verdict = 'NO-GO', bad[0]
    elif first is None:
        verdict = 'GLEICH', 'kein Rang über Schwelle und Bootstrap'
    elif got['verdict'] == 'SCHLECHTER':
        verdict = 'NO-GO', f'Rang {first.rank} {first.label}: {_fmt(got["ref"])} -> {_fmt(got["new"])}'
    else:
        verdict = 'VORLAGE' if later or veto else 'GO-VORSCHLAG', \
            f'Rang {first.rank} {first.label} besser: {_fmt(got["ref"])} -> {_fmt(got["new"])}'
    lines += ['', f'- **{verdict[0]}** ({verdict[1]})']
    lines += [f'- später schlechter: {x}' for x in later] + [f'- Gruppenveto: {v}' for v in veto]
    return lines + ['']


def main(argv=None) -> int:
    global STRICT, TIME_LIMIT
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--groups', action='store_true')
    ap.add_argument('--cases', action='store_true', help='per case: ranks 1 and 3 against the reference')
    ap.add_argument('--md')
    ap.add_argument('--strict', action='store_true', help='rank 1 without the rates for spins and pirouettes')
    ap.add_argument('--exclude', action='append', default=[], help="a case file whose cases don't count")
    ap.add_argument('--cpu-gate', action='store_true')
    ap.add_argument('--cpu-sum', action='store_true')
    ap.add_argument('--cpu-budget', type=float)
    ap.add_argument('--time-limit', type=float, help='s of planning a caller waits at most: cases over it')
    ap.add_argument('--recheck', help='write the cases with body places here (to run with seeds 0-9)')
    a = ap.parse_args(argv)
    STRICT, TIME_LIMIT = a.strict, a.time_limit
    loaded = [load(r) for r in a.runs]
    runs = [d for d, _ in loaded]
    names = [os.path.basename(os.path.normpath(r)) for r in a.runs]
    drop = {c for f in a.exclude for c in read_cases(f)}
    cases = sorted(c for c in runs[0] if c not in drop)
    fails = fail_values(runs)
    seeds = sorted({s for c in cases for s in sims(runs[0].get(c))})
    lines = ['Ergebnisse (sha256): ' + ', '.join(f'{n} {h[:16]}' for n, (_, h) in zip(names, loaded)),
             f'{len(cases)} Fälle der Referenz' + (f' ({len(drop & set(runs[0]))} ausgenommen)' if drop else '') +
             f', Sim-Seeds {", ".join(map(str, seeds)) or "keine"}', '']
    lines += table(names, runs, cases, fails) + ['']
    if len(seeds) >= 2:
        lines += [f'Seed-Rauschen in {names[0]} (SD der Gruppensumme über die Seeds; Schwelle für einen gleich guten '
                  'Kandidaten: 2·√2·SD, mindestens der feste Boden)', ''] + seed_noise(cases, runs[0], fails) + ['']
    for name, recs in zip(names[1:], runs[1:]):
        lines += compare(name, names[0], cases, runs[0], recs, fails, drop, a)
    if a.groups:
        for g in sorted({group_of(c) for c in cases}):
            sub = [c for c in cases if group_of(c) == g]
            lines += ['', f'### {g} ({len(sub)} Fälle)', ''] + table(names, runs, sub, fails)
    if a.cases:
        rows = [ROWS[0], ROWS[1], ROWS[7], ROWS[8]]
        lines += ['', '| Fall | ' + ' | '.join(f'{n}: Δ Rest Sim/Plan m², Δ Pir, Δ Drehen Sim' for n in names[1:]) +
                  ' |', '|---|' + '---|' * (len(names) - 1)]
        pairs = [[Pair(r, cases, runs[0], recs, fails) for r in rows] for recs in runs[1:]]
        for i, c in enumerate(cases):
            cells = []
            for ps in pairs:
                d = [p.mb[i] - p.ma[i] for p in ps]
                cells.append(f'{d[0]:+.2f} / {d[1]:+.2f}, {d[2]:+.0f}, {d[3]:+.0f}')
            lines.append(f'| {c} | ' + ' | '.join(cells) + ' |')
    if a.recheck:
        found = sorted({r['case'] for recs in runs for c, r in recs.items() if c in cases and not failed(r) and
                        any((s.get('rm_n5') or 0) > 0 for s in sims(r).values())})
        with open(a.recheck, 'w') as f:
            f.write(''.join(c + '\n' for c in found))
        lines.append(f'{len(found)} Fälle mit Körperstellen nach {a.recheck}')
    text = '\n'.join(lines)
    print(text)
    if a.md:
        with open(a.md, 'w') as f:
            f.write(text + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
