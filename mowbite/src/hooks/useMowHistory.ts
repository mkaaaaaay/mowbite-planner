'use client';

import {callRpc} from '@/lib/rpc';
import {useEffect, useState} from 'react';
import {RPC} from '@/lib/openmower';

export interface TrackSegment {
  attributes: {blades?: boolean; job_id?: string};
  points: [number, number][];
}

export interface JobInfo {
  job_id: string;
  timestamp: number; // unix seconds
}

export interface PastJob extends JobInfo {
  segments: TrackSegment[];
}

// the history doesn't change while you look at it, so everything is loaded once per page load
let listCache: Promise<JobInfo[]> | null = null;
const trackCache = new Map<string, Promise<TrackSegment[]>>();

export function loadJobList(): Promise<JobInfo[]> {
  listCache ??= callRpc<JobInfo[]>(RPC.jobs).then(
    // the mower lists a job again when it's resumed, keep the newest entry
    (list) => list.filter((j, i) => list.findIndex((k) => k.job_id === j.job_id) === i),
    () => {
      listCache = null;
      return [];
    },
  );
  return listCache;
}

export function loadJobTrack(jobId: string): Promise<TrackSegment[]> {
  let p = trackCache.get(jobId);
  if (!p) {
    p = callRpc<{segments?: TrackSegment[]}>(RPC.jobTrack, {job_id: jobId}, 20000).then(
      (h) => h.segments ?? [],
      () => {
        trackCache.delete(jobId);
        return [];
      },
    );
    trackCache.set(jobId, p);
  }
  return p;
}

// all recorded jobs, newest first, null while loading
export function useJobList(): JobInfo[] | null {
  const [jobs, setJobs] = useState<JobInfo[] | null>(null);
  useEffect(() => {
    let alive = true;
    void loadJobList().then((j) => alive && setJobs(j));
    return () => {
      alive = false;
    };
  }, []);
  return jobs;
}

const RECENT = 5;

// the last few jobs with their tracks, newest first, null while loading
export function useMowHistory(): PastJob[] | null {
  const [jobs, setJobs] = useState<PastJob[] | null>(null);
  useEffect(() => {
    let alive = true;
    void loadJobList()
      .then((list) =>
        Promise.all(list.slice(0, RECENT).map(async (j) => ({...j, segments: await loadJobTrack(j.job_id)}))),
      )
      .then((j) => alive && setJobs(j));
    return () => {
      alive = false;
    };
  }, []);
  return jobs;
}
