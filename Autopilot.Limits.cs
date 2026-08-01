using DV.Logic.Job;
using DV.RemoteControls;
using DV.Simulation.Cars;
using DV.Utils;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>Speed limits: what the signs and the geometry allow, what the
    /// whole consist is subject to, and the planned speed that respects every
    /// restriction coming up before the train reaches it.</summary>
    public static partial class Autopilot
    {
        /// Prefers the real sign where one has been indexed, but the route runs
        /// far past streamed terrain where no sign colliders exist, so it also
        /// takes the geometry-derived limit and obeys whichever is tighter.
        /// Direction-aware where the caller knows it (true = toward increasing
        /// span): a 40 posted against the OTHER direction's traffic must not pin
        /// this train to 40 on its own 100 road.</summary>
        // Posted limits are static per track once the signs are indexed, so they are
        // worked out once instead of on every walk of every tick. Keyed by direction
        // too: 0 backward, 1 forward, 2 either.
        private static readonly Dictionary<(RailTrack track, int dir), float> trackLimitCache =
            new Dictionary<(RailTrack, int), float>();

        private static float TrackLimit(RailTrack track, bool? dir = null)
        {
            var key = (track, dir == null ? 2 : dir.Value ? 1 : 0);
            if (trackLimitCache.TryGetValue(key, out var cached))
                return cached;
            float value = ComputeTrackLimit(track, dir);
            // Only cache once a sign has actually been read: before the track is
            // indexed the geometry figure is all there is, and caching it would hide
            // the sign for the rest of the session.
            if (TrackIndexer.MinSpeedLimit(track) > 0f)
                trackLimitCache[key] = value;
            return value;
        }

        private static float ComputeTrackLimit(RailTrack track, bool? dir = null)
        {
            float sign = TrackIndexer.MinSpeedLimit(track, dir); // 0 if no sign indexed
            float geom = GeometryLimit(track);                   // 999 if none
            if (sign > 0f)
                return Mathf.Min(sign, geom);
            return geom;
        }

        /// <summary>How far over a limit we may be while it is still this far off.</summary>
        private static float Allowance(double distance) =>
            distance < BandNear ? 0f : distance < BandMid ? BandOver1 : BandOver2;

        /// <summary>
        /// The same question answered with physics instead of bands, for a train
        /// slowing on the DYNAMIC brake: how much overspeed can still be shed by
        /// `decel` before the limit's at-limit point (BandNear out)? The stepped
        /// bands suit a shunt braking on air; on the road they demanded speed drops
        /// in near-instant steps, which the gentle dynamic could never deliver — so
        /// the air was blended in at every restriction and the dynamic did nothing.
        /// The curve starts the slowing as far out as the dynamic actually needs.
        /// </summary>
        private static float CurveAllowance(double distance, float decel) =>
            distance < BandNear
                ? 0f
                : Mathf.Sqrt(2f * decel * (float)(distance - BandNear)) * 3.6f;

        /// <summary>
        /// The speed to be doing now, given the limit under the train and every limit
        /// ahead, km/h.
        ///
        /// A limit coming up is respected through the overspeed bands: 300 m out we
        /// must already be at it, 600 m out we may still be 7 over, further out 14.
        /// A HIGHER limit ahead earns the same 7 km/h early, so the train starts
        /// pulling away before the last wagon is out of the slower zone rather than
        /// after — but it only gets the full higher speed once the whole train is
        /// inside it, which is what `here` measures.
        /// </summary>
        private static float PlannedLimitKmh(
            RailTrack startTrack, double startSpan, bool towardOut, double ownLead,
            float here, double maxDist, float? curveDecel = null)
        {
            float allowed = 999f;
            bool higherAhead = false;

            float Allow(double d) => curveDecel != null
                ? CurveAllowance(Math.Max(0, d), curveDecel.Value)
                : Allowance(Math.Max(0, d));

            // A geometry restriction binds where the CURVE is, not at the track's
            // door. Taking the whole-track minimum at the entry distance capped the
            // train the moment a kilometre-long track with one tight bend appeared
            // in the walk — kilometres before the bend itself.
            void ScanSegments(RailTrack tr, bool fwd, double entryDist)
            {
                var segs = SpeedZones.SegmentsFor(tr);
                double off = 0;
                for (int i = 0; i < segs.Count; i++)
                {
                    var seg = segs[fwd ? i : segs.Count - 1 - i];
                    if (seg.Length < 3)
                        continue;
                    if (seg[0] > 0f && seg[0] <= here + 0.5f)
                        allowed = Mathf.Min(allowed, seg[0] + Allow(entryDist + off));
                    off += seg[2];
                }
            }

            // The rest of the track we are ON: `here` only covers what is under
            // the train, so a tight bend further along the same track has to be
            // planned for like any other.
            {
                var segs = SpeedZones.SegmentsFor(startTrack);
                double pos = 0;
                foreach (var seg in segs)
                {
                    if (seg.Length < 3)
                        continue;
                    double segLo = pos, segHi = pos + seg[2];
                    pos = segHi;
                    double dist = towardOut ? segLo - startSpan : startSpan - segHi;
                    bool behind = towardOut ? segHi <= startSpan : segLo >= startSpan;
                    if (behind)
                        continue;
                    if (seg[0] > 0f && seg[0] <= here + 0.5f)
                        allowed = Mathf.Min(allowed, seg[0] + Allow(dist - ownLead));
                }
            }

            double cum = -ownLead; // distances are to the front of the train
            RailTrack? t = startTrack;
            double refSpan = startSpan;
            bool outward = towardOut;
            var seen = new HashSet<RailTrack>();

            for (int step = 0; step < 30 && cum < maxDist && t != null && seen.Add(t); step++)
            {
                cum += outward ? (PhysicalSpan(t) - refSpan) : refSpan;
                var branch = outward ? t.GetOutBranch() : t.GetInBranch();
                if (branch?.track == null)
                    break;
                t = branch.track;
                outward = branch.first;
                refSpan = branch.first ? 0.0 : PhysicalSpan(t);

                float lim = TrackLimit(t, outward);
                if (lim > here + 0.5f)
                {
                    higherAhead = true;
                    continue;
                }
                // A posted sign binds its whole track from the entry on.
                float sign = TrackIndexer.MinSpeedLimit(t, outward);
                if (sign > 0f && sign <= here + 0.5f)
                    allowed = Mathf.Min(allowed, sign + Allow(cum));
                ScanSegments(t, outward, cum);
            }

            // Never more than the band allows over what we are standing in.
            float onTrain = higherAhead ? here + BandOver1 : here;
            return Mathf.Min(onTrain, allowed);
        }

        /// <summary>
        /// Tightest speed (km/h) the limits AHEAD allow right now, walking the actual
        /// rail through the set switches. Each restriction is turned into the speed we
        /// may still be doing here and brake down to it in time, so the train slows
        /// BEFORE the sign rather than after it. 999 when nothing ahead binds.
        /// </summary>
        private static float LimitAheadKmh(
            RailTrack startTrack, double startSpan, bool towardOut, double maxDist)
        {
            float best = 999f;
            double cum = 0;
            RailTrack? t = startTrack;
            double refSpan = startSpan;
            bool outward = towardOut;
            var seen = new HashSet<RailTrack>();

            for (int step = 0; step < 30 && cum < maxDist && t != null && seen.Add(t); step++)
            {
                double tspan = t.GetKinkedPointSet().span;
                cum += outward ? (tspan - refSpan) : refSpan;
                var branch = outward ? t.GetOutBranch() : t.GetInBranch();
                if (branch?.track == null)
                    break;
                t = branch.track;
                outward = branch.first;
                refSpan = branch.first ? 0.0 : t.GetKinkedPointSet().span;

                // v² = limit² + 2·a·distance, in m/s, then back to km/h.
                float lim = TrackLimit(t) / 3.6f;
                float allowed = Mathf.Sqrt(lim * lim + 2f * StopDecel * (float)cum) * 3.6f;
                best = Mathf.Min(best, allowed);
            }
            return best;
        }

        /// <summary>Distance (m) along the planned path from the loco to where the
        /// destination track begins, or MaxValue when the loco isn't on the path (a
        /// re-route is due anyway). Zero once it is on the destination itself.</summary>
        private static double DistanceToDest(
            List<RailTrack> path, RailTrack current, double curSpan, bool towardOut) =>
            DistanceToIndex(path, current, curSpan, towardOut, path.Count - 1);

        /// <summary>As above, but to any index on the path — a pull leg has to measure
        /// the run to the track it reverses on, not to the far end of the route.</summary>
        private static double DistanceToIndex(
            List<RailTrack> path, RailTrack current, double curSpan, bool towardOut, int dest)
        {
            int i = IndexNear(path, current, 0);
            if (i < 0 || i > dest)
                return double.MaxValue;
            if (i == dest)
                return 0;
            double d = towardOut ? current.GetKinkedPointSet().span - curSpan : curSpan;
            for (int k = i + 1; k < dest; k++)
                d += path[k].GetKinkedPointSet().span;
            return d;
        }

        /// <summary>Rough distance from the reference bogie to the leading end of the
        /// consist, m — the whole train's length is the safe bound when it is being
        /// shoved, and it costs nothing to be pessimistic here.</summary>
        private static double ConsistLengthAhead(TrainCar loco, Bogie bogie)
        {
            double total = 0;
            foreach (var c in loco.trainset.cars)
                total += c.InterCouplerDistance;
            return total;
        }

        /// <summary>
        /// True while any part of the train is within reach of a junction on our path
        /// that we take on the diverging side. Measured against the junctions we are
        /// actually routed through, NOT the track id: DV's main line is built from
        /// "#Y-#S…#T" segments too, so treating a "#" id as switchwork pinned the
        /// train to yard speed for entire cross-map runs.
        /// </summary>
        private static bool NearDivergingSwitch(TrainCar loco, List<RailTrack> path)
        {
            var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
            if (registry == null)
                return false;

            for (int i = 0; i < path.Count - 1; i++)
            {
                var dj = DivergingJunction(path[i], path[i + 1]);
                // Only a road that BENDS off at the points needs junction speed.
                // Every facing junction used to count, which held a train riding
                // the straight leg of switch after switch to 40 across the map.
                if (dj == null || !TakesDivergingBranch(dj.Value))
                    continue;
                var junction = dj.Value.junction;
                IEnumerable<Junction.Branch> branches = junction.outBranches ?? new List<Junction.Branch>();
                if (junction.inBranch != null)
                    branches = branches.Append(junction.inBranch);

                foreach (var branch in branches)
                {
                    if (branch?.track == null)
                        continue;
                    foreach (var c in loco.trainset.cars)
                        foreach (var b in c.Bogies)
                        {
                            if (b == null || b.track != branch.track)
                                continue;
                            double span = b.traveller.Span;
                            double toJunction = branch.first
                                ? span
                                : PhysicalSpan(branch.track) - span;
                            if (toJunction < SwitchSlowZone)
                                return true;
                        }
                }
            }
            return false;
        }

        /// <summary>
        /// Distance from the leading end of the train to the next junction on the path
        /// that we take on the diverging side, m. MaxValue when the road ahead is
        /// straight through everything within reach.
        /// </summary>
        private static double DistanceToDivergingSwitch(
            List<RailTrack> path, int idx, RailTrack track, double bogieSpan,
            bool towardOut, double ownLead)
        {
            if (idx < 0)
                return double.MaxValue;
            double ahead = (towardOut
                ? track.GetKinkedPointSet().span - bogieSpan
                : bogieSpan) - ownLead;

            for (int i = idx; i < path.Count - 1 && ahead < LimitLookahead; i++)
            {
                var dj = DivergingJunction(path[i], path[i + 1]);
                if (dj != null && TakesDivergingBranch(dj.Value))
                    return Math.Max(0, ahead);
                ahead += path[i + 1].GetKinkedPointSet().span;
            }
            return double.MaxValue;
        }

        /// <summary>
        /// The tightest posted limit under any part of the consist, km/h. The train is
        /// only clear of a restriction once its last wagon is, so the limit is taken
        /// across every track the train is standing on, not just the loco's.
        /// </summary>
        private static float ConsistLimitKmh(TrainCar loco)
        {
            // Where the train actually sits on each track it occupies, and which
            // way it is travelling over each — signs only bind the direction they
            // face. Standing still tells no direction; both then count.
            var extent = new Dictionary<RailTrack, (double lo, double hi, bool? dir)>();
            foreach (var c in loco.trainset.cars)
                foreach (var b in c.Bogies)
                {
                    var t = b?.track;
                    if (t == null)
                        continue;
                    double span = b.traveller.Span;
                    float fs = c.GetForwardSpeed();
                    bool? dir = Mathf.Abs(fs) < 0.5f
                        ? (bool?)null
                        : (fs > 0f) == (b.TrackDirectionSign > 0);
                    if (extent.TryGetValue(t, out var e))
                        extent[t] = (Math.Min(e.lo, span), Math.Max(e.hi, span),
                            e.dir == dir ? dir : null);
                    else
                        extent[t] = (span, span, dir);
                }

            float limit = 999f;
            foreach (var kv in extent)
                limit = Mathf.Min(limit,
                    TrackLimitBetween(kv.Key, kv.Value.lo, kv.Value.hi, kv.Value.dir));
            return limit;
        }

        /// <summary>
        /// The limit over the stretch of a track between two spans, km/h. A track can
        /// be a kilometre long with one tight curve in it; taking the minimum over the
        /// whole thing — as TrackLimit does — then held the train to that curve's speed
        /// for the entire length. Signs still apply track-wide, since a posted limit
        /// does.
        /// </summary>
        private static float TrackLimitBetween(RailTrack track, double lo, double hi, bool? dir = null)
        {
            float sign = TrackIndexer.MinSpeedLimit(track, dir);
            float geometry = 999f;
            double cursor = 0;
            foreach (var seg in SpeedZones.SegmentsFor(track))
            {
                if (seg.Length < 3)
                    continue;
                float speed = seg[0];
                double length = seg[2];
                double segLo = cursor;
                double segHi = cursor + length;
                cursor = segHi;
                // Only segments the train is actually standing on.
                if (segHi < lo || segLo > hi)
                    continue;
                if (speed > 0f)
                    geometry = Mathf.Min(geometry, speed);
            }
            if (geometry > 998f)
                geometry = GeometryLimit(track); // no usable segment data
            return sign > 0f ? Mathf.Min(sign, geometry) : geometry;
        }

        /// <summary>Geometry-derived tightest limit, km/h (999 if none). Works
        /// everywhere, no streaming needed.</summary>
        private static float GeometryLimit(RailTrack track)
        {
            // SegmentsFor returns [speed, grade, length] — speed is index 0.
            var segments = SpeedZones.SegmentsFor(track);
            float min = float.MaxValue;
            foreach (var s in segments)
                if (s.Length >= 1 && s[0] > 0f)
                    min = Mathf.Min(min, s[0]);
            return min == float.MaxValue ? 999f : min;
        }
    }
}
