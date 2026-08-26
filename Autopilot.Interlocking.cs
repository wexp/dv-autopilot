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
    /// <summary>Sharing the rail: track claims between runs, throwing switches
    /// without splitting anyone, who is fouling which junction, and how far the
    /// stock ahead is by rail and by the world.</summary>
    public static partial class Autopilot
    {
        /* -------------------------------------------------- interlocking -- */

        /// <summary>
        /// Hand back the track this run has finished with: anything on the path behind
        /// the train that none of its cars is standing on. Without this a run keeps
        /// every track it has ever claimed, so a loco halfway across the yard still
        /// holds the throat that another train is waiting for.
        /// </summary>
        private static void ReleaseClaimsBehind(Run run, List<RailTrack> path, int pathIdx)
        {
            // Block-signaling release: the most recently CLEARED block stays held
            // as the buffer behind the tail - a bogie leaving a track still leaves
            // the body overhanging it, and handing the block back that instant put
            // a follower's nose right at the tail. Everything behind the buffer
            // block goes back to the pool.
            int newestClear = -1;
            for (int i = 0; i < pathIdx && i < path.Count; i++)
                if (run.claimed.Contains(path[i]) && !OurCarsOn(run.car, path[i]))
                    newestClear = i;
            for (int i = 0; i < newestClear && i < path.Count; i++)
            {
                var track = path[i];
                if (!run.claimed.Contains(track) || OurCarsOn(run.car, track))
                    continue;
                if (claims.TryGetValue(track, out var guid) && guid == run.guid)
                    claims.Remove(track);
                run.claimed.Remove(track);
                Trace.Line("RELEASE", () => $"{run.id} -trk {Trace.Id(track)}"
                    + $" via=release-behind (pathIdx={pathIdx}, cleared)");
            }
        }

        /// <summary>Is the nearest foreign stock along the rail ahead part of the
        /// train standing on <paramref name="destTrack"/>? Decides whether what
        /// the run is closing on is its coupling TARGET or just something parked
        /// in the way.</summary>
        private static bool NextForeignIsOn(
            RailTrack destTrack, TrainCar loco, RailTrack start, double startSpan, bool towardOut)
        {
            var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
            if (registry == null)
                return false;
            var ours = loco.trainset;
            double best = double.MaxValue;
            TrainCar? nearest = null;
            double cum = 0;
            RailTrack? t = start;
            double refSpan = startSpan;
            bool outward = towardOut;
            var seen = new HashSet<RailTrack>();
            for (int step = 0; step < 40 && cum < 400 && t != null && seen.Add(t); step++)
            {
                var logicTrack = t.LogicTrack();
                if (logicTrack != null)
                {
                    foreach (var lc in logicTrack.GetCarsFullyOnTrack().Concat(logicTrack.GetCarsPartiallyOnTrack()))
                    {
                        if (lc == null || !registry.TryGetValue(lc, out var tc) || tc == null
                            || tc.trainset == ours)
                            continue;
                        foreach (var b in tc.Bogies)
                        {
                            if (b == null || b.track != t)
                                continue;
                            double rel = outward ? b.traveller.Span - refSpan : refSpan - b.traveller.Span;
                            if (rel <= 0)
                                continue;
                            if (cum + rel < best)
                            {
                                best = cum + rel;
                                nearest = tc;
                            }
                        }
                    }
                }
                cum += outward ? PhysicalSpan(t) - refSpan : refSpan;
                var branch = outward ? t.GetOutBranch() : t.GetInBranch();
                if (branch?.track == null)
                    break;
                t = branch.track;
                outward = branch.first;
                refSpan = branch.first ? 0.0 : PhysicalSpan(t);
            }
            if (nearest == null)
                return false;
            var cars = nearest.trainset?.cars;
            if (cars == null)
                return false;
            foreach (var c in cars)
                foreach (var b in c.Bogies)
                    if (b?.track == destTrack)
                        return true;
            return false;
        }

        /// <summary>Span of the nearest bogie on this track belonging to a car OUTSIDE
        /// our consist — the train a same-track coupling move should drive toward.
        /// NaN when the track holds nothing but us.</summary>
        private static double NearestForeignCarSpan(TrainCar loco, RailTrack track, double fromSpan)
        {
            var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
            if (registry == null)
                return double.NaN;
            double best = double.MaxValue, span = double.NaN;
            foreach (var other in registry.Values)
            {
                if (other == null || other.trainset == loco.trainset)
                    continue;
                foreach (var b in other.Bogies)
                {
                    if (b?.track != track)
                        continue;
                    double d = Math.Abs(b.traveller.Span - fromSpan);
                    if (d < best)
                    {
                        best = d;
                        span = b.traveller.Span;
                    }
                }
            }
            return span;
        }

        /// <summary>
        /// Give this consist EXACTLY ONE loco with its train brake valve cut in —
        /// `preferred` when it is aboard, otherwise the first loco found. Two valves
        /// open fight each other (each tries to hold the pipe at its own handle's
        /// pressure); none open leaves a lash-up with no train brake at all, which is
        /// what splitting a multi-unit used to do to the detached half. Consists
        /// without a loco are left alone. The direct brakeSystem write covers locos
        /// whose overrider carries no cutout control.
        /// </summary>
        public static void EnsureOneBrakeCutIn(TrainCar? member, TrainCar? preferred = null)
        {
            var cars = member?.trainset?.cars;
            if (cars == null)
                return;
            TrainCar? lead = preferred != null && preferred.trainset == member!.trainset
                ? preferred : null;
            if (lead == null)
                foreach (var c in cars)
                    if (c != null && c.IsLoco)
                    {
                        lead = c;
                        break;
                    }
            if (lead == null)
                return; // wagons only — nothing to cut in
            foreach (var c in cars)
            {
                if (c == null || !c.IsLoco)
                    continue;
                bool cutIn = c == lead;
                var overrider = c.SimController?.controlsOverrider;
                if (overrider?.BrakeCutout != null)
                    overrider.BrakeCutout.Set(cutIn ? 1f : 0f);
                else if (c.brakeSystem != null)
                    c.brakeSystem.trainBrakeCutout = cutIn;
            }
        }

        /// <summary>True while any car of this consist is standing on the track.</summary>
        private static bool OurCarsOn(TrainCar loco, RailTrack track)
        {
            foreach (var c in loco.trainset.cars)
                foreach (var b in c.Bogies)
                    if (b != null && b.track == track)
                        return true;
            return false;
        }

        /// <summary>
        /// Is this foreign car standing on the same leg as us, but FURTHER from the
        /// frog than our own train is? Then it is behind us on the road we are
        /// leaving, and nothing it does can foul our way through the junction.
        ///
        /// Only "further than us" counts. Stock between us and the frog is a real
        /// obstruction and must still refuse the grant — the drive loop's own
        /// proximity guard is what stops short of it.
        /// </summary>
        private static bool ParkedBehindUsAt(Junction junction, TrainCar fouler, Trainset? ours)
        {
            if (ours?.cars == null || fouler.Bogies == null)
                return false;

            RailTrack? leg = null;
            foreach (var candidate in JunctionLegs(junction))
                if (fouler.Bogies.Any(b => b?.track == candidate))
                {
                    leg = candidate;
                    break;
                }
            if (leg == null)
                return false; // on the frog itself, or off the junction entirely

            double foulerGap = BodyDistanceToJunction(junction, fouler);
            foreach (var c in ours.cars)
            {
                if (c?.Bogies == null || !c.Bogies.Any(b => b?.track == leg))
                    continue;
                if (BodyDistanceToJunction(junction, c) <= foulerGap)
                    return true;
            }
            return false;
        }

        /// <summary>Give up every reservation this run holds - tracks and junctions.</summary>
        private static void ReleaseClaims(Run run)
        {
            Trace.Line("RELEASE", () => $"{run.id} ALL via=release-all"
                + $" trk=[{Trace.Ids(run.claimed)}]"
                + $" junc=[{string.Join(",", run.claimedJunctions.Select(Jn))}]");
            foreach (var key in run.claimed)
                if (claims.TryGetValue(key, out var guid) && guid == run.guid)
                    claims.Remove(key);
            run.claimed.Clear();
            foreach (var j in run.claimedJunctions)
                if (junctionClaims.TryGetValue(j, out var guid) && guid == run.guid)
                    junctionClaims.Remove(j);
            run.claimedJunctions.Clear();
            foreach (var tt in run.claimedTurntables)
                if (turntableClaims.TryGetValue(tt, out var guid) && guid == run.guid)
                    turntableClaims.Remove(tt);
            run.claimedTurntables.Clear();
        }

        /// <summary>
        /// Take the ORIENTATION of a turntable, or say who has it. Exclusive and
        /// dispatch-ordered like a junction: whoever wound it first keeps it until
        /// they are finished with it, because the bridge angle IS their route.
        /// Without this a second run at another stall wound the table round while
        /// the first was still standing on the bridge waiting for its own road.
        /// </summary>
        private static bool GrantTurntable(Run run, TurntableRailTrack tt, out string holder)
        {
            holder = "";
            if (turntableClaims.TryGetValue(tt, out var guid))
            {
                if (guid == run.guid)
                    return true;
                // A dead run's claim would lock the table for the session.
                if (runs.TryGetValue(guid, out var other)
                    && (other.state == "driving" || other.state == "waiting"))
                {
                    holder = RunNameOf(guid);
                    return false;
                }
                turntableClaims.Remove(tt);
            }
            if (WantedByOlder(tt, run, out holder))
                return false;
            turntableClaims[tt] = run.guid;
            run.claimedTurntables.Add(tt);
            return true;
        }

        /// <summary>
        /// Wind a turntable on this run's behalf, if the run may have it. False
        /// means somebody else holds the table and named it in <paramref
        /// name="holder"/> — the caller stands still and asks again, exactly as it
        /// would at a junction it could not get.
        /// </summary>
        private static bool TurnFor(Run run, TurntableRailTrack tt, Action turn, out string holder)
        {
            if (!GrantTurntable(run, tt, out holder))
            {
                RegisterWant(tt, run);
                return false;
            }
            turn();
            return true;
        }

        /// <summary>Hand back a turntable once the run has no further use for it:
        /// no car of ours on the bridge, and no bridge left on the road ahead.
        /// Held any longer, one shunt at a roundhouse would lock the table for
        /// every other run until it finished its whole job.</summary>
        private static void ReleaseTurntablesBehind(Run run, List<RailTrack> path, int pathIdx)
        {
            if (run.claimedTurntables.Count == 0)
                return;
            var ahead = new HashSet<TurntableRailTrack>();
            for (int i = Math.Max(0, pathIdx); i < path.Count; i++)
            {
                var tt = Turntables.AsBridge(path[i]);
                if (tt != null)
                    ahead.Add(tt);
            }
            var done = new List<TurntableRailTrack>();
            foreach (var tt in run.claimedTurntables)
            {
                if (ahead.Contains(tt))
                    continue;
                if (tt.Track != null && run.car != null && OurCarsOn(run.car, tt.Track))
                    continue; // still standing on the bridge
                done.Add(tt);
            }
            foreach (var tt in done)
            {
                if (turntableClaims.TryGetValue(tt, out var guid) && guid == run.guid)
                    turntableClaims.Remove(tt);
                run.claimedTurntables.Remove(tt);
            }
        }

        /* ------------------------------------------- movement authority -- */

        private static string RunNameOf(string guid) =>
            runs.TryGetValue(guid, out var r) ? r.id : guid;

        /// <summary>The live run this car belongs to, if any.
        ///
        /// Everything downstream turns on the difference between "a train is in the
        /// way" and "something is parked in the way": the first clears itself and is
        /// waited for, the second never will and the road is written off after thirty
        /// seconds. Refusals used to name the offending CAR — CFF663 — which matches no
        /// run name, so another autopilot fouling a frog read as abandoned stock and
        /// L-006 re-routed round the whole of Harbour rather than wait ten seconds for
        /// L-000 to pull forward.</summary>
        private static Run? RunOwning(TrainCar? car)
        {
            if (car?.trainset == null)
                return null;
            foreach (var r in runs.Values)
                if (r.coroutine != null && r.car != null && r.car.trainset == car.trainset)
                    return r;
            return null;
        }

        /// <summary>The live run that has this track CLAIMED, if any. Ownership in the
        /// ledger, not stock on the ground — the run holding the block ahead of us is
        /// usually nowhere near it yet, which is exactly why we are stopped short of
        /// it. <see cref="RunStandingOn"/> answers the other question.</summary>
        private static Run? RunHolding(RailTrack? track)
        {
            if (track == null || !claims.TryGetValue(track, out var guid))
                return null;
            return runs.TryGetValue(guid, out var run) && run.coroutine != null
                ? run : null;
        }

        /// <summary>The live run with stock standing on this track, if any.</summary>
        private static Run? RunStandingOn(RailTrack track)
        {
            foreach (var r in runs.Values)
                if (r.coroutine != null && r.car != null && OurCarsOn(r.car, track))
                    return r;
            return null;
        }

        /// <summary>A junction as the trace and the run notes name it — the same number
        /// the game's own switch signs carry, so a line can be walked to on the ground.
        /// Lives here rather than in Trace because the index is ours, not the game's.</summary>
        private static string Jn(Junction? junction) =>
            junction == null ? "-" : $"J-{JunctionIndex(junction)}";

        private static float lastLedgerTrace;

        /// <summary>
        /// The whole interlocking, once a second: where every live run is, what it is
        /// doing, and who holds every claimed block. Called from each drive loop and
        /// gated on a shared clock, so one snapshot is written however many trains are
        /// out.
        ///
        /// This is the artifact that makes a lock-up legible. The per-decision lines
        /// above say what happened; a pair of snapshots either side of the moment says
        /// what the state WAS, which is what you need when the deciding line turns out
        /// to be one nobody thought to instrument.
        /// </summary>
        private static void TraceLedger()
        {
            if (!Trace.On)
                return;
            float now = Time.realtimeSinceStartup;
            if (now - lastLedgerTrace < 1f)
                return;
            lastLedgerTrace = now;

            var sb = new System.Text.StringBuilder();
            int live = 0;
            foreach (var r in runs.Values)
                if (r.coroutine != null)
                    live++;
            sb.Append($"runs={live}");
            foreach (var r in runs.Values)
            {
                if (r.coroutine == null)
                    continue;
                Bogie? b = null;
                if (r.car?.Bogies != null)
                    foreach (var x in r.car.Bogies)
                        if (x?.track != null)
                        {
                            b = x;
                            break;
                        }
                sb.Append($"\n  {r.id} seq={r.seq} rank={RankName(r)}"
                    + (r.underway ? " under-way" : " not-started") + $" state={r.state}")
                  .Append($" trk={Trace.Id(b?.track)}")
                  .Append($" span={(b == null ? "-" : Trace.M(b.traveller.Span))}")
                  .Append($" v={(r.car == null ? "-" : Trace.Ms(Mathf.Abs(r.car.GetForwardSpeed()) * 3.6f))}")
                  .Append($" dest={Trace.Id(r.destTrack)} path={r.pathFrom}/{r.path.Count}")
                  .Append($" holds={r.claimed.Count}trk+{r.claimedJunctions.Count}junc")
                  .Append($" detail=\"{r.detail}\"");
            }
            sb.Append("\n  claims:");
            foreach (var kv in claims)
                sb.Append($" {Trace.Id(kv.Key)}={RunNameOf(kv.Value)}");
            sb.Append("\n  junctions:");
            foreach (var kv in junctionClaims)
                sb.Append($" {Jn(kv.Key)}={RunNameOf(kv.Value)}");
            sb.Append("\n  wants:");
            foreach (var kv in blockWants)
                if (now - kv.Value.time <= WantFreshSeconds)
                    sb.Append($" {KeyName(kv.Key)}={RunNameOf(kv.Value.guid)}");
            Trace.Line("LEDGER", () => sb.ToString());
        }

        /// <summary>A block key — track or junction — as one printable name.</summary>
        private static string KeyName(object key) => key switch
        {
            RailTrack t => Trace.Id(t),
            Junction j => Jn(j),
            TurntableRailTrack tt => $"TT-{tt.name}",
            _ => key?.ToString() ?? "-",
        };

        /// <summary>A SENIOR run is stopped waiting for this block: it said so within the
        /// last few seconds. Junior runs must leave the block for it — that is the whole
        /// of the queueing discipline. Seniority by the one rule, Outranks, so the queue
        /// and the reach-ahead reservation can never rank a pair opposite ways.</summary>
        private static bool WantedByOlder(object key, Run run, out string byWho)
        {
            byWho = "";
            if (!Wanter(key, out var wanter) || wanter == run || !Outranks(wanter, run))
                return false;
            byWho = wanter.id;
            return true;
        }

        /// <summary>A senior is queued at this block, and it is not queued behind US.
        ///
        /// The grant sites ask this rather than WantedByOlder because a want is barred
        /// for one more reason than a claim is. L-048 held J-499, L-068 queued for it,
        /// and L-048 was then refused the junction it was already standing on the edge
        /// of — by the queue of the train that was waiting for it to cross. Nothing could
        /// move: the senior wanted the frog, the junior needed the same frog to get out
        /// of the way, and the junior's own claim was no defence because this gate never
        /// looked at who held the block.
        ///
        /// So: rail we already hold is not re-argued (the callers pass that), and a
        /// senior queued behind a train standing on its road does not bar that train from
        /// moving on — the same exception CanTakeOver and ReservedByOlder make, in the
        /// third place it was needed. WantedByOlder itself is left alone:
        /// PreemptWantedBlocks uses it to decide what to hand BACK, and a train in the
        /// way should go on handing back blocks it has not reached.</summary>
        private static bool QueuedAgainstUs(object key, Run run, out string byWho)
        {
            byWho = "";
            if (!Wanter(key, out var wanter) || wanter == run || !Outranks(wanter, run))
                return false;
            if (FoulsPlanAhead(wanter, run))
            {
                var w = wanter;
                Trace.Line("RESV", () => $"{run.id} {KeyName(key)} NOT barred by {w.id}'s"
                    + " queue — it is queued behind us; our moving on is what frees it");
                return false;
            }
            byWho = wanter.id;
            return true;
        }

        /// <summary>The run queued at this block, if one said so recently enough and is
        /// still about. A wanter that has finished leaves its entry to age out.</summary>
        private static bool Wanter(object key, out Run wanter)
        {
            wanter = null!;
            return blockWants.TryGetValue(key, out var want)
                && Time.realtimeSinceStartup - want.time <= WantFreshSeconds
                && runs.TryGetValue(want.guid, out wanter)
                && wanter.coroutine != null;
        }

        /// <summary>
        /// Where a run stands in the yard's order of business. Lower takes the rail.
        ///
        ///   0 — LEAVING a loading track. Highest, and above dispatch order: that track
        ///       cannot be used by anything else until this train is off it, so every
        ///       minute it is held short is a minute the loader stands idle. A train on
        ///       its way IN to the same road is waiting for this one by definition.
        ///   1 — going TO a loading track. The work the yard exists to do.
        ///   2 — everything else: crossing the throat on the way somewhere.
        ///
        /// A run both leaving and heading for a loading road counts as leaving; it is
        /// clearing a berth either way.
        /// </summary>
        private static int LoadingRank(Run run)
        {
            if (run.originTrack?.LogicTrack()?.ID?.trackType == "L")
                return 0;
            if (run.destTrack?.LogicTrack()?.ID?.trackType == "L")
                return 1;
            return 2;
        }

        /// <summary>The loading road this run is clearing, for <see cref="LoadingRank"/>.
        ///
        /// It used to be path[0], and path[0] is bogies[1].track — the LOCO's rail. A
        /// train being pulled OFF a loading road has its wagons on that road and its loco
        /// already out on the lead, so the one move the ranking exists to favour scored
        /// as "through" and gave way to the train coming IN for the berth it was in the
        /// middle of clearing. The whole consist is asked instead.
        ///
        /// <paramref name="carried"/> is the origin of the run this one replaces. A
        /// re-route builds a NEW Run, and the origin is written only at construction, so
        /// a train re-routed after drawing clear of the loading road lost its rank at
        /// exactly the point it still needed it — the berth is not free until the getaway
        /// has finished. An L origin is carried across the replacement; anything else is
        /// re-read from the ground.</summary>
        private static RailTrack? LoadingOrigin(
            TrainCar car, RailTrack? locoTrack, RailTrack? carried)
        {
            var cars = car?.trainset?.cars;
            if (cars != null)
                foreach (var c in cars)
                {
                    if (c == null)
                        continue;
                    foreach (var b in c.Bogies)
                        if (b?.track != null
                            && b.track.LogicTrack()?.ID?.trackType == "L")
                            return b.track;
                }
            if (carried?.LogicTrack()?.ID?.trackType == "L")
                return carried;
            return locoTrack;
        }

        /// <summary>Which of two runs takes the rail. In order:
        ///
        ///   1 — a move that has TURNED A WHEEL, over one that has not. Rank is for
        ///       deciding who goes first, and once one of them has gone there is
        ///       nothing left to decide. Without this the ranking could invert under a
        ///       train that had already taken its road: L-048 was sent off first and
        ///       was across the throat when L-068 was dispatched for a loading road,
        ///       which outranked it from a standing start and left the two of them
        ///       looking at each other across J-499 with no way out.
        ///   2 — between two moves both under way: dispatch order, and nothing else.
        ///       They are past the point where the yard's business can reorder them.
        ///   3 — between two still standing: the order of business — clearing a
        ///       loading road, then filling one — then dispatch order. This is where
        ///       LoadingRank does its work, deciding the pecking order of trains that
        ///       have not yet moved.
        ///
        /// Every test reads a property of ONE run, so this is a lexicographic order on
        /// (under way, order of business, dispatch order) and is transitive by
        /// construction. That matters as much as being fixed both ways round: an order
        /// that can call each of a PAIR the senior is how two trains take turns giving
        /// way and neither arrives, and one that is merely intransitive does the same
        /// thing to three.</summary>
        private static bool Outranks(Run other, Run ours)
        {
            if (other.underway != ours.underway)
                return other.underway;
            if (other.underway)
                return other.seq < ours.seq;
            return LoadingRank(other) != LoadingRank(ours)
                ? LoadingRank(other) < LoadingRank(ours)
                : other.seq < ours.seq;
        }

        /// <summary>The rank as one word, for the trace.</summary>
        private static string RankName(Run run) => LoadingRank(run) switch
        {
            0 => "leaving-L",
            1 => "to-L",
            _ => "through",
        };

        /// <summary>How much rail a run still has to cover before it reaches this block,
        /// m, or -1 if its plan does not reach it inside the priority range. Measured in
        /// whole tracks from the one it is on, which overstates the distance by at most
        /// the track under its wheels — near enough to decide who waits, and it needs no
        /// bogie walk.</summary>
        private static double PlanReach(Run run, object key)
        {
            double d = 0;
            for (int i = run.pathFrom; i < run.path.Count && d <= PriorityReachMeters; i++)
            {
                var t = run.path[i];
                if (t == null)
                    continue;
                if (ReferenceEquals(t, key))
                    return d;
                if (i + 1 < run.path.Count && run.path[i + 1] != null
                    && ReferenceEquals(JunctionBetween(t, run.path[i + 1]), key))
                    return d + PhysicalSpan(t);
                d += PhysicalSpan(t);
            }
            return -1;
        }

        /// <summary>A run that outranks us is close enough to this block to have it.
        ///
        /// Without this, priority was purely reactive: a want is only registered by a
        /// grant that has already FAILED, so the senior train had to arrive at the frog,
        /// be beaten to it, and queue. Whoever got there first won, dispatch order or no.
        /// Reading the senior's plan instead lets it hold the road it is about to need,
        /// and the junior stands at the last block it already holds.
        ///
        /// A senior that has come to a STAND reserves just the same. It used not to, and
        /// that is how a junior drove onto rail a senior was about to need: a runaround
        /// stands still at its reversal by definition, and a shunt stops to brake, to
        /// throw a switch, to recharge the pipe. Every one of those moments handed the
        /// senior's road to whoever happened to be nearest it. Once the junior's stock is
        /// ON the block nothing recovers — ForeignStockOn is absolute and takes no rank —
        /// so the reservation has to survive the senior's stops or it is not a
        /// reservation at all. A senior that then never moves stands there for ever, and
        /// should: that is a fault upstream showing itself, not a case to route around.
        ///
        /// The one exception is a junior ALREADY standing on the senior's road. Barring
        /// it from the block ahead of it as well leaves it nowhere to go, and the junior
        /// pulling forward is the only move that can ever free the senior — the same
        /// reasoning PreemptWantedBlocks applies to claims, applied to the projection.
        /// Rail we are fouling is rail we are entitled to leave by.</summary>
        private static bool ReservedByOlder(object key, Run run, out string byWho)
        {
            byWho = "";
            foreach (var other in runs.Values)
            {
                if (other == run || other.car == null || other.coroutine == null)
                    continue;
                if (!Outranks(other, run))
                    continue;
                if (FoulsPlanAhead(other, run))
                {
                    var senior = other;
                    Trace.Line("RESV", () => $"{run.id} {KeyName(key)} NOT reserved by"
                        + $" {senior.id} — we stand on its road; pulling clear is the"
                        + " only way it moves");
                    continue;
                }
                double reach = PlanReach(other, key);
                if (reach < 0)
                    continue;
                byWho = other.id;
                return true;
            }
            return false;
        }

        /// <summary>The rail a run has still to cover, out to the priority horizon. The
        /// set form of the walk <see cref="PlanReach"/> and <see cref="FoulsPlanAhead"/>
        /// do one track at a time — asked for when the question is not "is this block on
        /// its road" but "where IS its road", which is what a train getting out of the
        /// way has to know.</summary>
        private static HashSet<RailTrack> PlanAhead(Run run, double reach)
        {
            var set = new HashSet<RailTrack>();
            double d = 0;
            for (int i = Math.Max(0, run.pathFrom); i < run.path.Count && d <= reach; i++)
            {
                var t = run.path[i];
                if (t == null)
                    continue;
                set.Add(t);
                d += PhysicalSpan(t);
            }
            return set;
        }

        /// <summary>Every track joined to this one, both ends, whatever the switches are
        /// set to. The road a train backs down is not the road it drove up, and a search
        /// for somewhere to stand must not be limited to the way the points happen to
        /// lie at this instant — they can be thrown.</summary>
        private static IEnumerable<RailTrack> Neighbours(RailTrack track)
        {
            var ins = track.GetAllInBranches();
            if (ins != null)
                foreach (var b in ins)
                    if (b?.track != null)
                        yield return b.track;
            var outs = track.GetAllOutBranches();
            if (outs != null)
                foreach (var b in outs)
                    if (b?.track != null)
                        yield return b.track;
        }

        /// <summary>Is this run's stock standing on the road the other run has still to
        /// cover? Walks the same stretch of plan as <see cref="PlanReach"/> and stops at
        /// the same horizon, so a train is either inside the senior's reach-ahead
        /// reservation or outside it, never counted by one test and not the other.</summary>
        private static bool FoulsPlanAhead(Run other, Run run)
        {
            if (run.car == null)
                return false;
            double d = 0;
            for (int i = other.pathFrom; i < other.path.Count && d <= PriorityReachMeters; i++)
            {
                var t = other.path[i];
                if (t == null)
                    continue;
                if (OurCarsOn(run.car, t))
                    return true;
                d += PhysicalSpan(t);
            }
            return false;
        }

        private static void RegisterWant(object key, Run run)
        {
            if (Wanter(key, out var queued) && queued != run && Outranks(queued, run))
            {
                var q = queued;
                Trace.Line("WANT", () => $"{run.id} {KeyName(key)} NOT queued —"
                    + $" {q.id} already there and senior"
                    + $" ({RankName(q)}/{RankName(run)} seq={q.seq}/{run.seq})");
                return; // a senior run is already queued here
            }
            blockWants[key] = (run.guid, Time.realtimeSinceStartup);
            Trace.Line("WANT", () => $"{run.id} {KeyName(key)} queued"
                + $" ({RankName(run)} seq={run.seq})");
        }

        /// <summary>May we take this block off the run that holds it?
        ///
        /// A reservation is not a freehold. A junior run that reached the frog first and
        /// then STOPPED had, until now, an absolute hold on it: L-046 — a loading move —
        /// stood waiting on L-011, which outranked it in nothing and was going nowhere.
        /// A held block only means something while its holder is using it.
        ///
        /// Never off a MOVING train (it would be brought to a stand in the block it was
        /// promised), never off rail its cars are standing on, and never against the
        /// order in Outranks. A run that has finished holds nothing at all.</summary>
        private static bool CanTakeOver(Run run, string guid, RailTrack? track)
        {
            if (!runs.TryGetValue(guid, out var other) || other == run)
                return true; // nobody there: a stale entry in the ledger
            if (other.coroutine == null)
                return true; // finished or failed — its rail should already be back
            if (other.car == null)
                return true;
            if (Mathf.Abs(other.car.GetForwardSpeed()) * 3.6f > MovingKmh)
                return false;
            if (track != null && OurCarsOn(other.car, track))
                return false;
            // Not the block it is standing AT either. A train held at the end of its
            // authority sits with its nose on the boundary — L-047 logged the wait as
            // "0 m ahead" — and taking the rail from under that nose points a moving
            // train straight at a standing one with nothing between them. Its stock is
            // not on it yet, so the cars test above says nothing.
            if (track != null && other.pathFrom + 1 < other.path.Count
                && other.path[other.pathFrom + 1] == track)
                return false;
            // Rank decides who takes rail BOTH trains want. It has no business barring
            // a train that is standing on the senior's own road from the block it needs
            // to get off it: the senior is stopped (checked above), it is stopped
            // because we are in its way, and our moving on is the only thing that can
            // ever free it. Holding the frog against us there is the senior holding
            // itself up — the same trap PreemptWantedBlocks was written for, one block
            // further on.
            //
            // The two halves are ONE test, not two, and that is the whole point. Written
            // as "rank OR we foul it", each of a pair could take the block off the other
            // whenever each stood on the other's road: L-030 (senior) and L-042 each had
            // cars on the other's plan, so #Y-#S1203#T changed hands about once a second
            // for minutes on end. Every swap dropped the loser's authority to nothing,
            // which is a target of zero, which is a FULL brake application — so the
            // trains alternated notch three and an emergency-looking stop, over and
            // over, and neither got anywhere. An interlocking rule that can call each of
            // a pair the one to give way is the same fault as an order that can call
            // each of them the senior; see Outranks.
            //
            // So: fouling does not ADD to rank, it INVERTS it. The train in the way is
            // the one that may take the block, and the senior it is blocking may not
            // take it back. Both readings are the negation of each other, so exactly one
            // of any pair can ever take rail from the other, and the ledger settles.
            if (Outranks(run, other))
                return !FoulsPlanAhead(run, other); // it needs this rail to pull off ours
            return FoulsPlanAhead(other, run);      // we stand on its road — let us move
        }

        /// <summary>Which test in <see cref="CanTakeOver"/> refused, in words. Built only
        /// for the trace: "denied, somebody holds it" is not an answer when the question
        /// is why a stopped train kept rail it was not using.</summary>
        private static string TakeOverBar(Run run, string guid, RailTrack? track)
        {
            if (!runs.TryGetValue(guid, out var other) || other == run)
                return "stale-entry";
            if (other.coroutine == null)
                return "holder-not-running";
            if (other.car == null)
                return "holder-has-no-car";
            float v = Mathf.Abs(other.car.GetForwardSpeed()) * 3.6f;
            if (v > MovingKmh)
                return $"holder-moving({v:F2}kmh)";
            if (track != null && OurCarsOn(other.car, track))
                return "holder-standing-on-it";
            if (track != null && other.pathFrom + 1 < other.path.Count
                && other.path[other.pathFrom + 1] == track)
                return "holder-standing-at-it";
            if (Outranks(run, other))
                return FoulsPlanAhead(run, other)
                    ? "holder-stands-on-our-road" // junior in the way: it goes first
                    : "we-outrank";
            return FoulsPlanAhead(other, run)
                ? "holder-blocked-by-us"
                : $"holder-senior({RankName(other)}/{RankName(run)}"
                    + $" seq={other.seq}/{run.seq})";
        }

        private static bool GrantTrack(Run run, RailTrack track, out string holder)
        {
            holder = "";
            bool ours = claims.TryGetValue(track, out var guid) && guid == run.guid;
            string? takeFrom = null;
            if (!ours && claims.ContainsKey(track))
            {
                if (!CanTakeOver(run, guid, track))
                {
                    holder = RunNameOf(guid);
                    string who = holder; // an out parameter cannot be captured below
                    Trace.Line("GRANT", () => $"{run.id} trk={Trace.Id(track)} -> DENIED"
                        + $" rule=claim holder={who} {TakeOverBar(run, guid, track)}");
                    return false;
                }
                takeFrom = guid;
            }
            if (!ours && QueuedAgainstUs(track, run, out holder))
            {
                string who = holder;
                Trace.Line("GRANT", () => $"{run.id} trk={Trace.Id(track)} -> DENIED"
                    + $" rule=wanted-by queued={who}");
                return false;
            }
            // Rail we already hold stays ours. Taking a block back off a train that is
            // standing on it, or halfway through it, is not a priority decision — it is
            // a train stopped in the wrong place.
            if (!ours && ReservedByOlder(track, run, out holder))
            {
                string who = holder;
                Trace.Line("GRANT", () => $"{run.id} trk={Trace.Id(track)} -> DENIED"
                    + $" rule=reserved-ahead by={who}");
                return false;
            }
            // A block with somebody's stock STANDING on it does not grant, however
            // the ledger reads - the release-margin behind a train is metres, and
            // authority through a tail is how two locos met on a frog. The
            // destination is the one exception: pulling up short of (or coupling
            // to) what stands there is the drive loop's job.
            if (track != run.destTrack && ForeignStockOn(track, run.car?.trainset))
            {
                // Same again: a run standing there is named, so the wait is a wait.
                holder = RunStandingOn(track)?.id
                    ?? "stock standing on " + (track.LogicTrack()?.ID?.ToString() ?? "it");
                Trace.Line("GRANT", () => $"{run.id} trk={Trace.Id(track)} -> DENIED"
                    + " rule=foreign-stock");
                return false;
            }
            if (takeFrom != null && runs.TryGetValue(takeFrom, out var loser))
                loser.claimed.Remove(track);
            claims[track] = run.guid;
            run.claimed.Add(track);
            Trace.Line("GRANT", () => $"{run.id} trk={Trace.Id(track)} -> OK"
                + (ours ? " (already ours)" : "")
                + (takeFrom != null ? $" TAKEN from={RunNameOf(takeFrom)}" : ""));
            return true;
        }

        /// <summary>Any bogie of a consist that is not ours on this track.</summary>
        private static bool ForeignStockOn(RailTrack track, Trainset? ours)
        {
            var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
            if (registry == null)
                return false;
            var logicTrack = track.LogicTrack();
            if (logicTrack == null)
                return false;
            foreach (var lc in logicTrack.GetCarsFullyOnTrack().Concat(logicTrack.GetCarsPartiallyOnTrack()))
            {
                if (lc == null || !registry.TryGetValue(lc, out var tc) || tc == null)
                    continue;
                if (tc.trainset != ours)
                    return true;
            }
            return false;
        }

        /// <summary>Every track attached to a junction: the stem and each branch.</summary>
        private static IEnumerable<RailTrack> JunctionLegs(Junction junction)
        {
            if (junction.inBranch?.track != null)
                yield return junction.inBranch.track;
            if (junction.outBranches != null)
                foreach (var b in junction.outBranches)
                    if (b?.track != null)
                        yield return b.track;
        }

        /// <summary>
        /// A junction grants to ONE run and takes the WHOLE junction: the node and
        /// every attached leg track. A follower is then denied at the START of a
        /// leg, a full block out, instead of creeping to the frog's edge. Never
        /// grants while foreign stock fouls it from any branch. A leg that another
        /// run merely STANDS on does not fail the grant (that run is behind the
        /// frog and cannot cross - the node is ours), it is simply not taken.
        /// </summary>
        private static bool GrantJunction(Run run, Junction junction, out string holder)
        {
            holder = "";
            bool held = junctionClaims.TryGetValue(junction, out var guid) && guid == run.guid;
            string? takeFrom = null;
            if (!held && junctionClaims.ContainsKey(junction))
            {
                // Decided here, acted on at the bottom: a junction taken off a stopped
                // junior is only actually taken once the fouling and leg tests below
                // have passed, or we would strip its claim and then refuse ourselves.
                if (!CanTakeOver(run, guid, null))
                {
                    holder = RunNameOf(guid);
                    string who = holder;
                    Trace.Line("JUNC", () => $"{run.id} {Jn(junction)} -> DENIED"
                        + $" rule=claim holder={who} {TakeOverBar(run, guid, null)}");
                    return false;
                }
                takeFrom = guid;
            }
            if (!held && QueuedAgainstUs(junction, run, out holder))
            {
                string who = holder;
                Trace.Line("JUNC", () => $"{run.id} {Jn(junction)} -> DENIED"
                    + $" rule=wanted-by queued={who}");
                return false;
            }
            if (!held && ReservedByOlder(junction, run, out holder))
            {
                string who = holder;
                Trace.Line("JUNC", () => $"{run.id} {Jn(junction)} -> DENIED"
                    + $" rule=reserved-ahead by={who}");
                return false;
            }
            var ours = run.car?.trainset;
            foreach (var fouler in CarsAround(junction, null))
            {
                if (fouler == null || fouler.trainset == ours)
                    continue;
                // Clear of the node AND clear of the marked point. The node distance
                // alone let a train held short of this frog — 20 m back, so "clear" by
                // 15 — be swung past by the run that got the junction.
                if (BodyDistanceToJunction(junction, fouler) >= JunctionGrantClearance
                    && !FoulsMarkedClearance(junction, fouler))
                    continue;
                // Stock BEHIND us on the road we are leaving cannot obstruct our
                // way through the frog: we are going out of that leg, not in past
                // it. Refusing anyway stopped a light engine from using the very
                // junction it was standing at, because other locos were parked
                // behind it on the same siding — and the run then re-routed
                // straight into them and gave up. The claims loop below has always
                // drawn this distinction; the car check never did.
                if (ParkedBehindUsAt(junction, fouler, ours))
                    continue;
                // Name the RUN when the fouling stock is being driven by one. The caller
                // waits for a name it recognises and writes the road off for one it does
                // not, so calling another autopilot by its car number is the difference
                // between queueing and touring the valley.
                holder = RunOwning(fouler)?.id ?? fouler.ID;
                var f = fouler;
                Trace.Line("JUNC", () => $"{run.id} {Jn(junction)} -> DENIED rule=fouled"
                    + $" car={f.ID} dNode={Trace.M(BodyDistanceToJunction(junction, f))}"
                    + $" marked={FoulsMarkedClearance(junction, f)}");
                return false;
            }
            // Legs: all free (or standing-only) before any is taken.
            foreach (var leg in JunctionLegs(junction))
            {
                if (!claims.TryGetValue(leg, out var g) || g == run.guid)
                    continue;
                if (runs.TryGetValue(g, out var other)
                    && other.car != null && OurCarsOn(other.car, leg))
                    continue; // standing on its own rail behind the frog
                // An empty leg reserved by a stopped junior goes the way of the node.
                if (CanTakeOver(run, g, leg))
                    continue;
                // A leg claimed by a train that is STANDING STILL is not a reason to
                // refuse the whole junction — the frog is clear (the fouling loop above
                // has already said so) and that train cannot move onto it without a
                // grant we are about to hold. Leave it its leg and go through; that is
                // what this method has always claimed to do for a train standing ON a
                // leg, and the case below it is the same thing one block earlier.
                //
                // Without this, the stopped-at-the-boundary guard in CanTakeOver — added
                // so a moving train could not have rail taken from under a waiting one's
                // nose — became a lock all by itself: L-046 held short of J-495, kept the
                // near leg, and refused it for ever to L-047, which outranked it and was
                // stood still waiting for exactly that junction.
                if (runs.TryGetValue(g, out var stopped) && stopped.car != null
                    && Mathf.Abs(stopped.car.GetForwardSpeed()) * 3.6f <= MovingKmh)
                {
                    var stalled = leg;
                    Trace.Line("JUNC", () => $"{run.id} {Jn(junction)} leg"
                        + $" {Trace.Id(stalled)} left with {RunNameOf(g)} (stopped and"
                        + " clear of the frog) — taking the junction anyway");
                    continue;
                }
                holder = RunNameOf(g);
                string who = holder;
                var barred = leg;
                Trace.Line("JUNC", () => $"{run.id} {Jn(junction)} -> DENIED rule=leg-claim"
                    + $" leg={Trace.Id(barred)} holder={who} {TakeOverBar(run, g, barred)}");
                return false;
            }
            if (takeFrom != null && runs.TryGetValue(takeFrom, out var beaten))
                beaten.claimedJunctions.Remove(junction);
            junctionClaims[junction] = run.guid;
            run.claimedJunctions.Add(junction);
            var took = new List<RailTrack>();
            foreach (var leg in JunctionLegs(junction))
            {
                if (claims.TryGetValue(leg, out var g) && g != run.guid
                    && !CanTakeOver(run, g, leg))
                    continue; // the standing follower keeps its own rail
                if (g != null && g != run.guid && runs.TryGetValue(g, out var lost))
                    lost.claimed.Remove(leg);
                // NEVER take a leg somebody's stock is standing on - taking
                // SW-A2P out from under two queued locos is how the ledger came
                // to disagree with the ground. The frog itself is safe (fouling
                // check above); an occupied leg is defended by the stock on it.
                if (ForeignStockOn(leg, run.car?.trainset))
                    continue;
                claims[leg] = run.guid;
                run.claimed.Add(leg);
                took.Add(leg);
            }
            Trace.Line("JUNC", () => $"{run.id} {Jn(junction)} -> OK"
                + (held ? " (already ours)" : "")
                + (takeFrom != null ? $" TAKEN from={RunNameOf(takeFrom)}" : "")
                + $" legs=[{Trace.Ids(JunctionLegs(junction))}]"
                + $" took=[{Trace.Ids(took)}]");
            return true;
        }

        /// <summary>
        /// Movement authority: hold rail ahead of the train, block by block, up to
        /// `need` metres past its front - a signalman clearing the road ahead of a
        /// train section by section. Returns the metres of held rail ahead of the
        /// FRONT; when the road could not be extended, names what stopped it and
        /// where. The rail under the train is held by physical fact; everything
        /// ahead must be granted. Past the end of the plan (a pull drawing clear
        /// overruns it) the walk follows the live switches, claiming as it goes.
        /// </summary>
        private static double ExtendAuthority(
            Run run, List<RailTrack> path, int idx, RailTrack track, double bogieSpan,
            bool towardOut, double ownLead, double need, double stopFloor,
            out string heldBy, out string atName, out RailTrack? blockedTrack,
            int legEnd = int.MaxValue)
        {
            heldBy = "";
            atName = "";
            blockedTrack = null;

            ForceHold(run, track);
            // The road ahead is set afresh every walk, so the release passes can tell
            // "this junction is behind us" from "this junction is the one we are being
            // authorised over right now, off the end of the plan".
            run.authJunctions.Clear();

            double granted = towardOut ? PhysicalSpan(track) - bogieSpan : bogieSpan;
            var t = track;
            bool outw = towardOut;
            int i = idx;
            var seen = new HashSet<RailTrack> { track };
            // The blocks taken, in order, with the junction each was entered through.
            // Needed to work out where the train would actually COME TO REST if the
            // road runs out — see the clear-stand rule below.
            var chain = new List<(Junction? entry, double span, RailTrack track,
                bool outw, bool onPlan)>();
            // Where the road was shut, as a path index. Everything from here on is rail
            // this train is NOT going to reach, and must therefore not keep.
            int blockedFrom = -1;
            for (int step = 0; step < 60 && granted - ownLead < need; step++)
            {
                RailTrack? next = null;
                bool nextOut = true;
                Junction? junction = null;
                // Past the current LEG the plan is the NEXT leg's business - a
                // pull's road ahead is its live overrun, not the reversal's
                // return hop. Chasing the return hop froze a pull at zero
                // authority because the junction back in was fouled by the very
                // train it was sent to couple to.
                if (i >= legEnd)
                    i = -1;
                if (i >= 0 && i + 1 < path.Count)
                {
                    next = path[i + 1];
                    junction = JunctionBetween(t, next);
                    nextOut = EnterIsIn(next, t);
                }
                else if (Turntables.AsBridge(t) != null)
                {
                    // A turntable bridge is where the road STOPS, off the plan.
                    //
                    // Everywhere else, "follow the switches as they lie" is right: the
                    // train really will go where the points send it. A bridge is not
                    // points. What lies beyond it is whichever stall it happens to be
                    // pointed at this second, and the entire purpose of the move is to
                    // turn it somewhere else — so the rail past it is rail this train
                    // will never touch.
                    //
                    // Walking through it authorised L-009 into HB-T13P, a stall it was
                    // not going to, with somebody's stock parked on it. That block cut
                    // its authority to 18 m, which is a target of zero, which is a full
                    // brake application — and it stuck there for good, because the
                    // bridge is only ever turned by a train that is standing ON it and
                    // L-009 was now forbidden to reach it.
                    //
                    // Authority therefore ends at the bridge, which is exactly where the
                    // train is going: a wind is a dead stand aboard, every time.
                    var bridge = t;
                    Trace.Line("AUTH", () => $"{run.id} walk ends on the turntable bridge"
                        + $" {Trace.Id(bridge)} — past the plan, and the road beyond a"
                        + " bridge is not decided until it is turned");
                    break;
                }
                else
                {
                    // Off (or past) the plan: the pull's overrun rail. Follow the
                    // switches as they lie, like the train will.
                    var branch = outw ? t.GetOutBranch() : t.GetInBranch();
                    if (branch?.track != null)
                    {
                        next = branch.track;
                        junction = outw ? t.outJunction : t.inJunction;
                        nextOut = branch.first;
                    }
                }
                if (next == null || !seen.Add(next))
                {
                    var last = t;
                    bool lastOut = outw;
                    Trace.Line("AUTH", () => $"{run.id} walk ends at {Trace.Id(last)}"
                        + $" — {(next == null ? "no rail beyond" : "the road loops back on itself")}"
                        + $" (going {(lastOut ? "out" : "in")})");
                    break; // rail ends, or loops - authority ends with it
                }

                if (junction != null && !GrantJunction(run, junction, out heldBy))
                {
                    atName = $"J-{JunctionIndex(junction)}";
                    blockedTrack = next;
                    blockedFrom = i >= 0 ? i + 1 : -1;
                    string by = heldBy;
                    var stopAt = next;
                    int n = step;
                    double got = granted - ownLead;
                    Trace.Line("AUTH", () => $"{run.id} step={n} -> BLOCKED at {Jn(junction)}"
                        + $" into {Trace.Id(stopAt)} held-by={by} granted={Trace.M(got)}");
                    RegisterWant(junction, run);
                    break;
                }
                if (!GrantTrack(run, next, out heldBy))
                {
                    atName = next.LogicTrack()?.ID?.ToString() ?? "the next track";
                    blockedTrack = next;
                    blockedFrom = i >= 0 ? i + 1 : -1;
                    string by = heldBy;
                    var stopAt = next;
                    int n = step;
                    double got = granted - ownLead;
                    Trace.Line("AUTH", () => $"{run.id} step={n} -> BLOCKED at"
                        + $" {Trace.Id(stopAt)} held-by={by} granted={Trace.M(got)}");
                    RegisterWant(next, run);
                    break;
                }
                granted += PhysicalSpan(next);
                {
                    var got = next;
                    int n = step;
                    double total = granted - ownLead;
                    bool onPlan = i >= 0;
                    Trace.Line("AUTH", () => $"{run.id} step={n} +{Trace.Id(got)}"
                        + $" span={Trace.M(PhysicalSpan(got))} via={Jn(junction)}"
                        + $" granted={Trace.M(total)} {(onPlan ? "on-plan" : "overrun")}");
                }
                chain.Add((junction, PhysicalSpan(next), next, nextOut, i >= 0));
                if (junction != null)
                    run.authJunctions.Add(junction);
                t = next;
                outw = nextOut;
                i = i >= 0 && i + 1 < path.Count ? i + 1 : -1;
            }

            // Authority may only END somewhere the whole train can stand CLEAR.
            //
            // The walk takes blocks one at a time and the train stops wherever the road
            // runs out. Every unnamed track in this game is a junction leg, and a leg is
            // usually shorter than a consist — so "stop at the end of what you were
            // given" put L-011 to a stand with a wagon lying across J-490. It then held
            // that frog by physical fact, no ledger involved, and L-006 could never have
            // it: no priority rule reaches a train that is already parked on the points.
            //
            // So when the road is blocked, hand back the trailing blocks the train
            // cannot come to rest inside. Refusing the leg is refusing the whole move
            // through it — which is what the block ahead was telling us anyway — and the
            // train waits one stopping place further back, where it is in nobody's way.
            //
            // Overrun rail is exempt: a pull draws its length PAST a pivot and stands
            // there on purpose, and that is how every reversal is made.
            // How far back from the far end of a block the NOSE has to stop, so the
            // train is genuinely clear of whatever junction lies beyond it. The measure
            // is MarkedClearance — the same one GrantJunction judges fouling by. It has
            // to be the same one: the old pull-back here compared the marked inset
            // against SignalMargin and subtracted the difference, which at Harbour is
            // inset 11 against margin 20, i.e. nothing at all. L-046 was then allowed to
            // stand 12.5 m into a 24 m stub and lay across the frog at the end of it,
            // legal by the stopping rule and fouling by the granting rule.
            double NoseInset(RailTrack leg, bool exitOut)
            {
                var beyond = exitOut ? leg.outJunction : leg.inJunction;
                return beyond == null ? 0 : MarkedClearance(leg, !exitOut) + ClearancePad;
            }

            if (atName.Length > 0 && chain.Count > 0)
            {
                double tail = ConsistLength(run.car) + ClearancePad;
                // Standing with the nose at the end of block n-1: how much rail lies
                // between it and the first junction behind. Junctions further back are
                // further still, so the first one decides it.
                //
                // The LAST block only counts the rail up to where the nose may actually
                // stand — its far end less the clearance for the junction beyond.
                bool StandsClear(int n)
                {
                    double acc = 0;
                    for (int k = n - 1; k >= 0; k--)
                    {
                        acc += k == n - 1
                            ? Math.Max(0, chain[k].span
                                - NoseInset(chain[k].track, chain[k].outw))
                            : chain[k].span;
                        if (chain[k].entry != null)
                            return acc >= tail;
                    }
                    return true; // no junction between here and where the train already is
                }

                // A block with no room left for the nose at all — a leg shorter than the
                // clearance it owes the frog beyond it, which is most legs at a throat —
                // is not a stopping place, whatever lies behind it. Entry junction or
                // none: the old test looked only for a junction the walk had entered
                // THROUGH, so a leg reached over a plain joint (via=- in the trace) was
                // waved past without a single pop in six thousand lines of log.
                bool NoRoom(int n) =>
                    n > 0 && chain[n - 1].span
                        - NoseInset(chain[n - 1].track, chain[n - 1].outw) <= 0;

                int keep = chain.Count;
                while (keep > 0 && chain[keep - 1].onPlan
                    && (!StandsClear(keep) || NoRoom(keep)))
                {
                    var give = chain[keep - 1];
                    // Never below the distance the train needs to STOP in.
                    //
                    // This rule picks where a train comes to rest, and at a yard throat
                    // it can find nowhere at all: every block is a junction leg 3 to 50
                    // metres long and no 89 m consist stands clear of a frog anywhere in
                    // them. It then popped the whole road — L-055 went from 198.7 m of
                    // authority to nothing in a single tick, at 25 km/h, with 138 m of
                    // clear rail in front of it. Authority of nothing is a target of
                    // nothing, which is a full brake application: the train stood on its
                    // nose in the middle of the throat for no reason on the ground.
                    //
                    // Where to stop is a question about a train that is going to stop.
                    // It cannot be allowed to answer with less road than the train
                    // needs to stop in, because that answer is not "stop there", it is
                    // "you should already have stopped" — and there is no braking a
                    // train into rail it has passed. Keep those blocks: the train is
                    // going over them whatever the ledger says, and holding rail it will
                    // physically occupy is the honest reading. As it slows the floor
                    // falls away with the square of the speed, and by the time it
                    // matters where the train stands the rule has its full say again.
                    if (granted - give.span - ownLead < stopFloor)
                    {
                        var kept = give.track;
                        double keptSpan = give.span;
                        int k1 = keep;
                        Trace.Line("AUTH", () => $"{run.id} clear-stand: keeping"
                            + $" {Trace.Id(kept)} and everything behind it — popping it"
                            + $" leaves {Trace.M(granted - keptSpan - ownLead)} to stop"
                            + $" in and this train needs {Trace.M(stopFloor)}"
                            + $" (keep={k1}/{chain.Count})");
                        break;
                    }
                    granted -= give.span;
                    // Hand the rail back as well: holding a leg we have decided not to
                    // enter is the same blockage by another name.
                    if (!OurCarsOn(run.car, give.track)
                        && claims.TryGetValue(give.track, out var g) && g == run.guid)
                    {
                        claims.Remove(give.track);
                        run.claimed.Remove(give.track);
                    }
                    int k2 = keep;
                    Trace.Line("AUTH", () => $"{run.id} clear-stand: popped"
                        + $" {Trace.Id(give.track)} (span {Trace.M(give.span)}, consist"
                        + $" {Trace.M(tail)} needs to stand clear of {Jn(give.entry)})"
                        + $" granted={Trace.M(granted + give.span)}->{Trace.M(granted)}"
                        + $" keep={k2 - 1}/{chain.Count}");
                    // The JUNCTION stays ours. We are still rolling towards it — the
                    // fouling test only knows where the train is now, not where it will
                    // be when it stops, and handing a frog to an oncoming train while
                    // closing on it is how they meet on the points. It goes back with
                    // the rest when the plan releases behind us.
                    keep--;
                }
                // The stop is now at the end of whatever is left, so the marked-clearance
                // pull-back below has to be measured against THAT block.
                if (keep < chain.Count)
                {
                    t = keep > 0 ? chain[keep - 1].track : track;
                    outw = keep > 0 ? chain[keep - 1].outw : towardOut;
                }
            }
            // Where the road ends at a JUNCTION, a flat margin off the joint is not
            // clearance. The game marks the clearance point itself — 25 m out on a
            // Harbour throat — so a train held 20 m short of the joint stands 5 m
            // INSIDE the fouling zone of a frog it was never granted. The grant test
            // then read it as clear (it measured 15 m to the node), gave the junction
            // to the other run, and the two met on the points. Hold at the marker
            // whenever the marker is the further of the two; a track whose inset is
            // shorter than the signal margin is unaffected.
            // Where the road ends at a JUNCTION, hold the nose at the point the game
            // itself marks as clear of it — the same distance GrantJunction refuses a
            // fouled frog by. A flat margin off the joint is not clearance and never
            // was: 20 m short of a node whose marked point is 25 m out is 5 m INSIDE
            // the fouling zone, read as clear by the grant test, and that is how two
            // locos came to meet on the points.
            if (atName.Length > 0)
            {
                double pull = NoseInset(t, outw);
                if (pull > 0)
                {
                    granted -= pull;
                    var end = t;
                    Trace.Line("AUTH", () => $"{run.id} marker-pullback at {Trace.Id(end)}"
                        + $" clearance={Trace.M(pull)} (marked"
                        + $" {Trace.M(MarkedClearance(end, !outw))} + pad {ClearancePad})"
                        + $" -> -{Trace.M(pull)}");
                }
            }
            // HOLD-AND-WAIT, which is the whole disease.
            //
            // A train stopped at a shut road kept every block it had been granted on the
            // way to it, including blocks on the FAR side of the thing that stopped it —
            // rail it cannot reach without first passing the block it was just refused.
            // Two trains each holding the other's road is then a lock by construction,
            // and no amount of ranking or waiting can unpick it: L-046 stood at J-495
            // holding #Y-#S146#T, which is beyond J-495, while L-047 — senior, and stood
            // still — needed exactly that to move.
            //
            // So: refused at a block means everything past it goes back. It costs
            // nothing, because the train cannot get there anyway, and it is re-granted
            // block by block as the road opens and the train rolls forward.
            if (blockedFrom >= 0)
            {
                string shutAt = atName; // an out parameter cannot be captured below
                for (int j = blockedFrom; j < path.Count; j++)
                {
                    var far = path[j];
                    if (far == null || !run.claimed.Contains(far))
                        continue;
                    if (OurCarsOn(run.car, far))
                        continue; // rail under our wheels is ours by physical fact
                    if (claims.TryGetValue(far, out var g) && g == run.guid)
                        claims.Remove(far);
                    run.claimed.Remove(far);
                    int idxJ = j;
                    Trace.Line("RELEASE", () => $"{run.id} -trk {Trace.Id(far)}"
                        + $" via=beyond-block (path[{idxJ}], past {shutAt} which we were"
                        + " refused)");
                }
            }
            {
                double result = Math.Max(0, granted - ownLead);
                string at = atName, by = heldBy;
                var blk = blockedTrack;
                Trace.Line("AUTH", () => $"{run.id} result={Trace.M(result)}"
                    + $" ahead-of-nose need={Trace.M(need)} ownLead={Trace.M(ownLead)}"
                    + (at.Length > 0
                        ? $" blocked-at={at} held-by={by} into={Trace.Id(blk)}"
                        : " road clear to the horizon"));
            }
            return Math.Max(0, granted - ownLead);
        }

        /// <summary>How far from a junction a car standing on <paramref name="leg"/>
        /// has to be to be genuinely clear of the points, m — the track's own marked
        /// clearance inset, which the spotting code already reads off the signs. Never
        /// less than the flat grant clearance, so a track whose signs cannot be read
        /// keeps the old behaviour instead of a weaker test.</summary>
        private static double MarkedClearance(RailTrack leg, bool junctionAtInEnd)
        {
            var (lo, hi) = MarkerZone(leg);
            double inset = junctionAtInEnd ? lo : PhysicalSpan(leg) - hi;
            return Math.Max(JunctionGrantClearance, inset);
        }

        /// <summary>True while this car stands inside the MARKED fouling zone of the
        /// junction, on whichever leg it is on. BodyDistanceToJunction measures a
        /// straight line to the node, which says nothing about where the legs are far
        /// enough apart to pass each other: on a shallow turnout that is 25 m out, and
        /// a train "15 m clear" of the node is still in the way of anything swinging
        /// through.</summary>
        private static bool FoulsMarkedClearance(Junction junction, TrainCar car)
        {
            IEnumerable<Junction.Branch> branches = junction.outBranches ?? new List<Junction.Branch>();
            if (junction.inBranch != null)
                branches = branches.Append(junction.inBranch);
            foreach (var branch in branches)
            {
                if (branch?.track == null)
                    continue;
                foreach (var bogie in car.Bogies)
                {
                    if (bogie == null || bogie.track != branch.track)
                        continue;
                    // branch.first: the junction sits at this leg's span-0 end — the
                    // same convention BogieDistanceToJunction measures by.
                    double span = bogie.traveller.Span;
                    double toJunction = branch.first ? span : PhysicalSpan(branch.track) - span;
                    if (toJunction < MarkedClearance(branch.track, branch.first))
                        return true;
                }
            }
            return false;
        }

        /// <summary>Hold the rail the train physically stands on, without asking:
        /// stock on the ground beats any reservation. Never steals a key another
        /// run holds - their claim is already useless with us standing here.</summary>
        private static void ForceHold(Run run, RailTrack track)
        {
            if (!claims.TryGetValue(track, out var guid) || guid == run.guid)
            {
                bool fresh = !run.claimed.Contains(track);
                claims[track] = run.guid;
                run.claimed.Add(track);
                if (fresh)
                    Trace.Line("LEDGER", () => $"{run.id} +trk {Trace.Id(track)}"
                        + " via=force-hold (standing on it)");
            }
            else
            {
                // Somebody else's name on rail we are physically standing on. The
                // ledger and the ground disagree, which is exactly the state that
                // ends with two trains authorised into one block.
                Trace.Line("LEDGER", () => $"{run.id} STANDS ON {Trace.Id(track)}"
                    + $" but it is claimed by {RunNameOf(guid)} — not taken");
            }
        }

        /// <summary>Hand back everything not on the CURRENT plan and not under our
        /// wheels. Runs keep their holdings across legs - releasing everything
        /// between legs opened a gap in which another run could take the rail
        /// under a standing train - and this trims what a re-plan abandoned.</summary>
        private static void ReleaseOffPlan(Run run, List<RailTrack> path)
        {
            var planned = new HashSet<RailTrack>(path);
            foreach (var held in run.claimedJunctions)
                foreach (var leg in JunctionLegs(held))
                    planned.Add(leg); // held junctions keep their legs
            List<RailTrack>? drop = null;
            foreach (var track in run.claimed)
                if (!planned.Contains(track) && !OurCarsOn(run.car, track))
                    (drop ??= new List<RailTrack>()).Add(track);
            if (drop != null)
                foreach (var track in drop)
                {
                    if (claims.TryGetValue(track, out var guid) && guid == run.guid)
                        claims.Remove(track);
                    run.claimed.Remove(track);
                    Trace.Line("RELEASE", () => $"{run.id} -trk {Trace.Id(track)}"
                        + " via=release-off-plan (not on the new plan)");
                }

            var aheadJunctions = new HashSet<Junction>();
            for (int i = 0; i + 1 < path.Count; i++)
            {
                var j = JunctionBetween(path[i], path[i + 1]);
                if (j != null)
                    aheadJunctions.Add(j);
            }
            List<Junction>? dropJ = null;
            foreach (var j in run.claimedJunctions)
            {
                if (aheadJunctions.Contains(j))
                    continue;
                if (run.authJunctions.Contains(j))
                    continue; // the road set ahead of us — see ReleaseJunctionsBehind
                if (run.car != null
                    && JunctionBlocker(j, run.car.trainset, JunctionReleaseMargin) != null)
                    continue;
                (dropJ ??= new List<Junction>()).Add(j);
            }
            if (dropJ != null)
                foreach (var j in dropJ)
                {
                    if (junctionClaims.TryGetValue(j, out var guid) && guid == run.guid)
                        junctionClaims.Remove(j);
                    run.claimedJunctions.Remove(j);
                    // The candidate mechanism for the J-495 pair: a re-plan re-derives
                    // which junctions are "ahead", and one dropped here goes straight
                    // back into the pool while the train is still stood waiting to use
                    // it. If that is what happens, it happens on THIS line.
                    Trace.Line("RELEASE", () => $"{run.id} -junc {Jn(j)}"
                        + " via=release-off-plan (not on the new plan, consist clear)");
                    ReleaseJunctionLegs(run, j);
                }
        }

        /// <summary>Junctions this run holds that its tail has cleared and its plan
        /// no longer crosses: hand them back, block by block, as a train clears
        /// sections behind it.</summary>
        private static void ReleaseJunctionsBehind(Run run, List<RailTrack> path, int pathIdx)
        {
            if (run.claimedJunctions.Count == 0)
                return;
            var aheadJunctions = new HashSet<Junction>();
            for (int i = Math.Max(0, pathIdx); i + 1 < path.Count; i++)
            {
                var j = JunctionBetween(path[i], path[i + 1]);
                if (j != null)
                    aheadJunctions.Add(j);
            }
            var done = new List<Junction>();
            foreach (var j in run.claimedJunctions)
            {
                if (aheadJunctions.Contains(j))
                    continue;
                // Not a junction the movement authority is holding for us this moment.
                // The planned path is not the whole story: past the end of the current
                // leg the walk follows the live switches, and those junctions appear in
                // no path pair at all.
                if (run.authJunctions.Contains(j))
                    continue;
                if (run.car != null
                    && JunctionBlocker(j, run.car.trainset, JunctionReleaseMargin) != null)
                    continue;
                done.Add(j);
            }
            foreach (var j in done)
            {
                if (junctionClaims.TryGetValue(j, out var guid) && guid == run.guid)
                    junctionClaims.Remove(j);
                run.claimedJunctions.Remove(j);
                Trace.Line("RELEASE", () => $"{run.id} -junc {Jn(j)}"
                    + $" via=release-behind (pathIdx={pathIdx}, tail clear)");
                ReleaseJunctionLegs(run, j);
            }
        }

        /// <summary>Release the leg tracks of a junction this run no longer holds -
        /// each leg individually, and only once our own cars are off it.</summary>
        private static void ReleaseJunctionLegs(Run run, Junction junction)
        {
            foreach (var leg in JunctionLegs(junction))
            {
                if (!run.claimed.Contains(leg) || OurCarsOn(run.car, leg))
                    continue;
                // A leg may also belong to a junction we STILL hold, or lie on the
                // road ahead - keep those.
                bool stillNeeded = false;
                foreach (var held in run.claimedJunctions)
                    if (held != junction)
                        foreach (var l in JunctionLegs(held))
                            if (l == leg)
                            {
                                stillNeeded = true;
                                break;
                            }
                if (stillNeeded)
                    continue;
                if (claims.TryGetValue(leg, out var guid) && guid == run.guid)
                    claims.Remove(leg);
                run.claimed.Remove(leg);
                Trace.Line("RELEASE", () => $"{run.id} -trk {Trace.Id(leg)}"
                    + $" via=junction-legs ({Jn(junction)} handed back)");
            }
        }

        /// <summary>
        /// Give unreached blocks to an older run that is stood waiting for them -
        /// the signalman pulling back a road that was set but never used. Only
        /// blocks our train has not reached: rail under our wheels is nobody
        /// else's whatever the ledger says.
        /// </summary>
        private static void PreemptWantedBlocks(Run run)
        {
            if (run.car == null)
                return;
            // The rail this run still has to cover. Anything it holds that is NOT in
            // here is a block it will never drive over — a junction grant takes all
            // three legs, and the third leg is the one the train coming the other way
            // is standing on.
            var ahead = new HashSet<RailTrack>();
            for (int i = Math.Max(0, run.pathFrom); i < run.path.Count; i++)
                if (run.path[i] != null)
                    ahead.Add(run.path[i]);

            List<RailTrack>? giveTracks = null;
            foreach (var track in run.claimed)
            {
                if (OurCarsOn(run.car, track))
                    continue;
                if (WantedByOlder(track, run, out _))
                {
                    (giveTracks ??= new List<RailTrack>()).Add(track);
                    continue;
                }
                // A leg we are not going to use goes to whoever asks, senior or not.
                // Rank decides who takes rail that BOTH trains need; it has no business
                // deciding rail only one of them needs. L-006 outranked L-000 and so
                // kept #Y-#S1637#T, which its own route never touches — and L-000 could
                // not pull forward off the frog that was blocking L-006. Seniority held
                // the senior train up.
                if (!ahead.Contains(track) && Wanter(track, out var asker) && asker != run
                    && asker.coroutine != null)
                    (giveTracks ??= new List<RailTrack>()).Add(track);
            }
            if (giveTracks != null)
                foreach (var track in giveTracks)
                {
                    if (claims.TryGetValue(track, out var guid) && guid == run.guid)
                        claims.Remove(track);
                    run.claimed.Remove(track);
                    string to = Wanter(track, out var asked) ? asked.id : "someone";
                    bool onRoute = ahead.Contains(track);
                    Trace.Line("RELEASE", () => $"{run.id} -trk {Trace.Id(track)}"
                        + $" via=preempt (wanted by {to}"
                        + (onRoute ? ", senior" : ", and not on our road") + ")");
                }

            List<Junction>? giveJunctions = null;
            foreach (var j in run.claimedJunctions)
            {
                if (ConsistFoulsJunction(j, run.car.trainset))
                    continue;
                if (WantedByOlder(j, run, out _))
                    (giveJunctions ??= new List<Junction>()).Add(j);
            }
            if (giveJunctions != null)
                foreach (var j in giveJunctions)
                {
                    if (junctionClaims.TryGetValue(j, out var guid) && guid == run.guid)
                        junctionClaims.Remove(j);
                    run.claimedJunctions.Remove(j);
                    WantedByOlder(j, run, out var to);
                    Trace.Line("RELEASE", () => $"{run.id} -junc {Jn(j)}"
                        + $" via=preempt (wanted by {to})");
                }
        }

        /// <summary>
        /// Walk the planned path ahead and make sure every junction on it still points
        /// our way, throwing any that has drifted. Switches are set once when a leg
        /// starts, which is not enough: another train, the player, or a switch that was
        /// fouled at the time can leave one lying against us, and the drive follows the
        /// live switches, not the plan. Returns false when something ahead is wrong and
        /// cannot be put right from here, with the distance to it so the caller can
        /// slow down and stop short rather than take the wrong road.
        /// </summary>
        private static bool SwitchesAheadOk(
            Run run, List<RailTrack> path, int idx, RailTrack track, double bogieSpan,
            bool towardOut, double ownLead, out double distance, out string reason,
            out RailTrack? blocked)
        {
            distance = double.MaxValue;
            reason = "";
            blocked = null;
            if (idx < 0)
                return true; // off the planned path; the next re-route will fix it

            // Distance to the end of the track we are on: that is where the first
            // junction of the path ahead sits.
            double ahead = towardOut
                ? track.GetKinkedPointSet().span - bogieSpan
                : bogieSpan;

            // Everything below is measured to the LEADING END of the train. When the
            // consist is being shoved the front wagon is a train-length ahead of the
            // loco: measuring from the loco let that wagon roll onto the junction we
            // were waiting to throw, and a switch under a car cannot be thrown — so the
            // train sat there for ever, unable to correct the very thing blocking it.
            ahead -= ownLead;

            // Where the road ahead is barred, the train stops ObstacleBuffer short of
            // this distance — so the distance has to be to a place it can STAND, not to
            // the joint itself. The game marks the clearance point, 25 m out on a
            // Harbour throat, and a couple of metres off the joint is well inside it:
            // L-047 crept to the edge of J-234 and stood buffer to buffer with L-046
            // coming the other way. Same correction the movement authority already
            // makes; this path never had it.
            double ClearOf(int at, double raw)
            {
                var leg = path[at];
                var (lo, hi) = MarkerZone(leg);
                double inset = ExitIsOut(leg, path[at + 1])
                    ? PhysicalSpan(leg) - hi
                    : lo;
                return raw - Math.Max(0, inset + ClearancePad - ObstacleBuffer);
            }

            for (int i = idx; i < path.Count - 1 && ahead < JunctionCheckDistance; i++)
            {
                // Someone else's reservation means it is not ours to set OR to enter —
                // for THIS side of it; entering path[i + 1] from path[i] is a distinct
                // claim from entering it from its other end.
                if (claims.TryGetValue(path[i + 1], out var holder) && holder != run.guid)
                {
                    distance = ClearOf(i, ahead);
                    // Another train's reservation clears by itself; never route round it.
                    reason = runs.TryGetValue(holder, out var other)
                        ? $"{other.id} holds {path[i + 1].LogicTrack()?.ID}"
                        : "track reserved ahead";
                    double d = distance;
                    int at = i;
                    Trace.Line("SWITCH", () => $"{run.id} STOP short of"
                        + $" {Trace.Id(path[at + 1])} at {Trace.M(d)}"
                        + $" (raw {Trace.M(ahead)}) — claimed by {RunNameOf(holder)}");
                    return false;
                }

                var dj = DivergingJunction(path[i], path[i + 1]);
                if (dj != null && dj.Value.junction.selectedBranch != dj.Value.branch)
                {
                    // Try to correct it. This refuses while a car sits on the junction,
                    // which is exactly when it must not be thrown.
                    if (!SetSwitchBetween(
                            path[i], path[i + 1], out var why, run.car?.trainset, out bool byUs))
                    {
                        // Our OWN train on the points is not a reason to stop — it is a
                        // reason to keep going. Nothing can foul a junction our leading
                        // end has not reached (every distance here is measured to that
                        // end), so if one of our cars is on it we are already across and
                        // the road we are taking is settled. Stopping in front of it was
                        // a deadlock with no way out: the switch cannot be thrown until
                        // the train is clear, and the train cannot get clear while it is
                        // standing still waiting for the switch. It then wrote the road
                        // off and went looking for a way round one that was never shut.
                        if (byUs)
                        {
                            ahead += path[i + 1].GetKinkedPointSet().span;
                            continue;
                        }
                        distance = ClearOf(i, ahead);
                        blocked = path[i + 1];
                        var jid = path[i + 1].LogicTrack()?.ID;
                        reason = $"switch to {jid} won't set: {why}";
                        double d = distance;
                        int at = i;
                        string w = why;
                        Trace.Line("SWITCH", () => $"{run.id} STOP short of"
                            + $" {Jn(dj.Value.junction)} at {Trace.M(d)}"
                            + $" (raw {Trace.M(ahead)}) — into {Trace.Id(path[at + 1])}: {w}");
                        return false;
                    }
                    int thrown = i;
                    Trace.Line("SWITCH", () => $"{run.id} threw {Jn(dj.Value.junction)}"
                        + $" to {Trace.Id(path[thrown + 1])} at {Trace.M(ahead)} ahead");
                }
                ahead += path[i + 1].GetKinkedPointSet().span;
            }
            return true;
        }

        /// <summary>Point the junction between a and b at b — but NEVER while a car
        /// sits on it (throwing a switch under a moving train splits the consist).
        /// Returns true once the branch is set (or none was needed).</summary>
        private static bool SetSwitchBetween(RailTrack a, RailTrack b) =>
            SetSwitchBetween(a, b, out _, null, out _);

        private static bool SetSwitchBetween(RailTrack a, RailTrack b, out string why, Trainset? ours) =>
            SetSwitchBetween(a, b, out why, ours, out _);

        /// <summary>As above, reporting WHY a switch could not be set — "it is against
        /// us" is not actionable on its own, and the right response differs completely
        /// between a wagon sitting on the frog and the game refusing the throw.</summary>
        private static bool SetSwitchBetween(
            RailTrack a, RailTrack b, out string why, Trainset? ours, out bool byUs)
        {
            why = "";
            byUs = false;
            var dj = DivergingJunction(a, b);
            if (dj == null)
                return true;
            var (junction, k) = dj.Value;
            if (junction.selectedBranch == k)
                return true;

            // The only throw we must refuse outright is one under our OWN train, which
            // would split the consist. Whether somebody else's wagon standing nearby
            // prevents it is the game's call, not ours to guess at — so try it, and
            // judge by whether the branch actually moved.
            // Our own train gets a wider berth than the bare fouling distance: that is
            // measured to the BOGIE, and a wagon's body overhangs it by a couple of
            // metres at each end. Throwing a switch under our own train splits the
            // consist, so the margin is worth having.
            var mine = JunctionBlocker(junction, ours, OwnSwitchClearance);
            if (mine != null)
            {
                // Report the measured gap, not just the verdict: "on the switch" with no
                // number is unarguable and untestable, and this test has been wrong in
                // both directions before.
                byUs = true;
                why = $"our own {mine.ID} is on the switch "
                    + $"({BodyDistanceToJunction(junction, mine):F1} m of body clearance)";
                return false;
            }
            junction.Switch(Junction.SwitchMode.NO_SOUND, k);
            if (junction.selectedBranch == k)
                return true;
            var blocker = JunctionBlocker(junction, null);
            why = blocker != null ? $"{blocker.ID} is on the switch" : "the game refused the throw";
            return false;
        }

        /// <summary>Physical length of a track, the same measure bogie spans use.
        /// LogicTrack.length is a DIFFERENT number, and mixing the two put cars on
        /// switches they were standing well clear of.</summary>
        private static double PhysicalSpan(RailTrack track) => track.GetKinkedPointSet().span;

        /// <summary>The car fouling this junction, if any — the same test as
        /// <see cref="JunctionOccupied"/> but naming the culprit.</summary>
        private static TrainCar? JunctionBlocker(Junction junction, Trainset? only) =>
            JunctionBlocker(junction, only, FoulingTolerance);

        private static TrainCar? JunctionBlocker(Junction junction, Trainset? only, double tolerance)
        {
            foreach (var car in CarsAround(junction, only))
                if (BodyDistanceToJunction(junction, car) < tolerance)
                    return car;
            return null;
        }

        /// <summary>Every car with a bogie on one of this junction's branches — the
        /// candidates for fouling it. Restricting by branch track is what keeps stock on
        /// a PARALLEL track (four or five metres away in a yard, and so within any
        /// sensible tolerance) from being mistaken for a car on the points.</summary>
        private static IEnumerable<TrainCar> CarsAround(Junction junction, Trainset? only)
        {
            var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
            if (registry == null)
                yield break;
            IEnumerable<Junction.Branch> branches = junction.outBranches ?? new List<Junction.Branch>();
            if (junction.inBranch != null)
                branches = branches.Append(junction.inBranch);

            var seen = new HashSet<TrainCar>();
            foreach (var branch in branches)
            {
                if (branch?.track == null)
                    continue;
                var logicTrack = branch.track.LogicTrack();
                if (logicTrack == null)
                    continue;
                foreach (var car in logicTrack.GetCarsFullyOnTrack().Concat(logicTrack.GetCarsPartiallyOnTrack()))
                {
                    if (car == null || !registry.TryGetValue(car, out var trainCar) || trainCar == null)
                        continue;
                    if (only != null && trainCar.trainset != only)
                        continue; // asking only about our own cars
                    if (seen.Add(trainCar))
                        yield return trainCar;
                }
            }
        }

        /// <summary>
        /// How far this car's BODY is from the junction node, m — zero while it is
        /// standing on the points.
        ///
        /// Every fouling test used to measure to a BOGIE, which is wrong in both
        /// directions and is why no single tolerance ever satisfied both failures:
        ///
        ///   - a car that has PASSED the junction reads as fouling long after it is
        ///     clear, because its bogie is still within the tolerance while the body
        ///     ends metres short of it. A train waiting on that can never clear it —
        ///     it is stopped, waiting for itself;
        ///   - a car STRADDLING the junction reads as clear, because both of its
        ///     bogies sit on tracks either side and each may be outside the tolerance
        ///     while the middle of the car is right over the frog. Throwing the switch
        ///     then splits the consist.
        ///
        /// Measuring the coupler-to-coupler segment against the junction position gets
        /// both right: a straddling car returns ~0, and a car that has gone by returns
        /// the real gap behind it.
        /// </summary>
        private static double BodyDistanceToJunction(Junction junction, TrainCar car)
        {
            var front = car.frontCoupler != null ? car.frontCoupler.transform : null;
            var rear = car.rearCoupler != null ? car.rearCoupler.transform : null;
            if (front == null || rear == null)
                return BogieDistanceToJunction(junction, car);
            Vector3 a = front.position, b = rear.position, p = junction.position;
            Vector3 ab = b - a;
            float len2 = ab.sqrMagnitude;
            float t = len2 <= 1e-4f ? 0f : Mathf.Clamp01(Vector3.Dot(p - a, ab) / len2);
            return Vector3.Distance(p, a + ab * t);
        }

        /// <summary>Fallback for stock whose couplers can't be read: the old
        /// bogie-span measure, along the branch this car sits on.</summary>
        private static double BogieDistanceToJunction(Junction junction, TrainCar car)
        {
            IEnumerable<Junction.Branch> branches = junction.outBranches ?? new List<Junction.Branch>();
            if (junction.inBranch != null)
                branches = branches.Append(junction.inBranch);

            double best = double.MaxValue;
            foreach (var branch in branches)
            {
                if (branch?.track == null)
                    continue;
                foreach (var bogie in car.Bogies)
                {
                    if (bogie == null || bogie.track != branch.track)
                        continue;
                    double span = bogie.traveller.Span;
                    best = Math.Min(best, branch.first ? span : PhysicalSpan(branch.track) - span);
                }
            }
            return best;
        }

        /// <summary>True while any car sits within the fouling zone of the junction —
        /// i.e. it can't safely be thrown. Mirrors dv-hud's car-on-junction test.</summary>
        private static bool JunctionOccupied(Junction junction)
        {
            var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
            if (registry == null)
                return false;

            IEnumerable<Junction.Branch> branches = junction.outBranches ?? new List<Junction.Branch>();
            if (junction.inBranch != null)
                branches = branches.Append(junction.inBranch);

            foreach (var branch in branches)
            {
                if (branch?.track == null)
                    continue;
                var logicTrack = branch.track.LogicTrack();
                if (logicTrack == null)
                    continue;
                foreach (var car in logicTrack.GetCarsFullyOnTrack().Concat(logicTrack.GetCarsPartiallyOnTrack()))
                {
                    if (car == null || !registry.TryGetValue(car, out var trainCar) || trainCar == null)
                        continue;
                    foreach (var bogie in trainCar.Bogies)
                    {
                        if (bogie == null || bogie.track != branch.track)
                            continue;
                        double span = bogie.traveller.Span;
                        double toJunction = branch.first ? span : PhysicalSpan(branch.track) - span;
                        if (toJunction < FoulingTolerance)
                            return true;
                    }
                }
            }
            return false;
        }

        /// <summary>
        /// Distance (m) from the front of our consist to the nearest car that isn't
        /// part of our own train, measured in world space along the travel direction.
        /// Geometry rather than route, so it catches anything the routing logic didn't
        /// think of — but ONLY for cars standing on the rail we are actually going to
        /// run over (<paramref name="corridor"/>). Without that restriction a train
        /// parked on the neighbouring siding reads as "8 m ahead" through the fence
        /// and holds us for nothing. MaxValue if clear.
        /// </summary>
        private static double ObstacleDistanceWorld(
            TrainCar loco, Vector3 motion, HashSet<RailTrack> corridor, double maxDist,
            TrainCar? front = null)
        {
            var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
            if (registry == null)
                return double.MaxValue;
            var ours = loco.trainset;
            // Measure from the LEADING CAR when the caller knows it. From the loco,
            // a long consist around a yard curve put the cars its front was about
            // to hit outside the straight-line lateral window — and the close-range
            // "anything in front" guard protected the LOCO's surroundings while the
            // front ran into a parked rake a hundred metres away.
            Vector3 origin = (front != null ? front : loco).transform.position;

            // Where our own train's leading edge is, projected along the motion axis
            // (the train may extend ahead of the loco when reversing).
            float lead = 0f;
            foreach (var kv in registry)
            {
                var tc = kv.Value;
                if (tc == null || tc.trainset != ours)
                    continue;
                foreach (var b in tc.Bogies)
                    if (b != null)
                        lead = Mathf.Max(lead, Vector3.Dot(b.transform.position - origin, motion));
            }
            lead += 4f; // reach the coupler face rather than the bogie centre

            double best = double.MaxValue;
            foreach (var kv in registry)
            {
                var tc = kv.Value;
                if (tc == null || tc.trainset == ours)
                    continue;
                foreach (var b in tc.Bogies)
                {
                    if (b == null)
                        continue;
                    Vector3 d = b.transform.position - origin;
                    float along = Vector3.Dot(d, motion);
                    if (along <= lead || along > maxDist)
                        continue; // behind our front, or too far to matter yet
                    float lateral = (d - motion * along).magnitude;

                    // Close in, ANY car in front stops us, whatever track the game
                    // thinks it is on. A train standing on a junction is often filed
                    // under a neighbouring track, and the corridor test then filtered
                    // away a car we were about to drive into. The tight lateral limit
                    // is what keeps the neighbouring siding out of it.
                    if (along <= CloseRangeGuard)
                    {
                        if (lateral <= CloseRangeLateral)
                            best = Math.Min(best, along - lead);
                        continue;
                    }

                    // Further out, only what is on the rail we are actually taking,
                    // or a train on a line converging into it.
                    if (b.track == null || !corridor.Contains(b.track))
                        continue;
                    if (lateral > 3.5f)
                        continue; // beside us on a curve, not in front of us
                    best = Math.Min(best, along - lead);
                }
            }
            return best;
        }

        /// <summary>
        /// The car at the LEADING end of the train in the direction of travel, with
        /// its reference bogie and which way along that bogie's track the train is
        /// heading. Walked through the couplers from the loco, flipping orientation
        /// at each joint. The obstacle walk must start HERE: starting at the loco
        /// and following the set switches diverged from where a long consist
        /// actually lay — a junction under the train can be thrown differently
        /// than the wagons crossed it — and the guard then looked down the wrong
        /// road while the front of the train ran into parked stock.
        /// </summary>
        private static (Bogie bogie, bool towardOut, TrainCar car, bool forward)? LeadingEnd(
            TrainCar loco, bool locoForward)
        {
            TrainCar cur = loco;
            bool forward = locoForward;
            var seen = new HashSet<TrainCar> { loco };
            while (true)
            {
                var exit = forward ? cur.frontCoupler : cur.rearCoupler;
                var next = exit?.coupledTo?.train;
                if (next == null || !seen.Add(next))
                    break;
                // Entering the next car by its REAR coupler means travel runs
                // toward its front — its forward axis agrees with the motion.
                forward = exit!.coupledTo == next.rearCoupler;
                cur = next;
            }
            var bogies = cur.Bogies;
            if (bogies == null || bogies.Length < 2 || bogies[1] == null || bogies[1].track == null)
                return null;
            var bogie = bogies[1];
            return (bogie, forward == (bogie.TrackDirectionSign > 0), cur, forward);
        }

        /// <summary>
        /// Distance (m) to the nearest car that isn't ours, walking the ACTUAL rail in
        /// the travel direction (following the set switches). Curve-safe, unlike the
        /// straight-line world check that went blind around the yard throat and let us
        /// hit parked wagons. MaxValue if clear.
        /// </summary>
        private static double ObstacleDistanceRail(
            TrainCar loco, RailTrack startTrack, double startSpan, bool towardOut, double maxDist,
            out HashSet<RailTrack> corridor, out double ownLead, out double railEnd)
        {
            ownLead = 0;
            // Where the connected rail simply STOPS, measured like the obstacle
            // distance. A turntable wound the wrong way, a stub with no buffer —
            // driving is only allowed on rail that exists, and nothing else in
            // this loop knew the difference between "clear ahead" and "no rail
            // ahead": a pull overran its plan onto an open turntable pit and put
            // a DH4 on the ground.
            railEnd = double.MaxValue;
            // The tracks this walk covers are exactly the rail we are about to run
            // over, so the world-space check borrows them to ignore everything parked
            // on neighbouring lines.
            corridor = new HashSet<RailTrack> { startTrack };
            var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
            if (registry == null)
                return double.MaxValue;
            var ours = loco.trainset;
            double foreign = double.MaxValue;
            double ownAhead = 0;
            double cum = 0;
            RailTrack? t = startTrack;
            double refSpan = startSpan;
            bool outward = towardOut;
            var seen = new HashSet<RailTrack>();

            for (int step = 0; step < 120 && cum < maxDist && t != null && seen.Add(t); step++)
            {
                double tspan = t.GetKinkedPointSet().span;
                var logicTrack = t.LogicTrack();
                if (logicTrack != null)
                {
                    foreach (var lc in logicTrack.GetCarsFullyOnTrack().Concat(logicTrack.GetCarsPartiallyOnTrack()))
                    {
                        if (lc == null || !registry.TryGetValue(lc, out var tc) || tc == null)
                            continue;
                        bool mine = tc.trainset == ours;
                        foreach (var b in tc.Bogies)
                        {
                            if (b == null || b.track != t)
                                continue;
                            double cs = b.traveller.Span;
                            double rel = outward ? (cs - refSpan) : (refSpan - cs);
                            if (rel <= 0)
                                continue; // behind our reference point
                            double along = cum + rel;
                            if (mine)
                                ownAhead = Math.Max(ownAhead, along);
                            else
                                foreign = Math.Min(foreign, along);
                        }
                    }
                }

                cum += outward ? (tspan - refSpan) : refSpan;
                var branch = outward ? t.GetOutBranch() : t.GetInBranch();
                if (branch?.track == null)
                {
                    railEnd = cum;
                    break;
                }
                // Stock FOULING the junction we are about to cross, from any of its
                // branches. Its bogies sit on a track this walk never touches, so
                // the per-track scan above is blind to it — that is how a consist
                // sideswiped a parked rake overhanging a frog at SW.
                var crossing = outward ? t.outJunction : t.inJunction;
                if (crossing != null)
                {
                    foreach (var fouler in CarsAround(crossing, null))
                    {
                        if (fouler == null || fouler.trainset == ours)
                            continue;
                        if (BodyDistanceToJunction(crossing, fouler) < FoulingTolerance)
                        {
                            foreign = Math.Min(foreign, cum);
                            break;
                        }
                    }
                }
                t = branch.track;
                outward = branch.first;
                refSpan = branch.first ? 0.0 : t.GetKinkedPointSet().span;
                corridor.Add(t);
                // Everything meeting at this junction counts as our corridor too: a
                // train coming down the other branch converges onto us, and the
                // world-space check is the only thing that can see it.
                foreach (var j in new[] { t.inJunction, t.outJunction })
                {
                    if (j?.inBranch?.track != null)
                        corridor.Add(j.inBranch.track);
                    if (j?.outBranches != null)
                        foreach (var ob in j.outBranches)
                            if (ob?.track != null)
                                corridor.Add(ob.track);
                }
            }

            ownLead = ownAhead;
            // Both distances are to the FRONT of the train, like the foreign one.
            if (railEnd != double.MaxValue)
                railEnd = Math.Max(0, railEnd - ownAhead);
            return foreign == double.MaxValue ? double.MaxValue : Math.Max(0, foreign - ownAhead);
        }

        /// <summary>
        /// What every OTHER run's road costs to route over, by track: reservations
        /// they hold now, and the rail their plans still have to cover. Fed into
        /// the router's edge costs so a new plan bends around live traffic at PLAN
        /// time instead of discovering it nose-to-nose at drive time. Soft costs —
        /// a genuinely shorter road through them can still win, exactly like the
        /// busy-track penalty.
        /// </summary>
        /// <param name="sameWay">Filled with, for each track another run is about to
        /// cover, the tracks its plan goes on to NEXT. A route that takes the same step
        /// is not crossing that run, it is falling in behind it — a following move, not
        /// a conflict, and priced as one.</param>
        internal static Dictionary<RailTrack, double> ForeignRoadCosts(
            TrainCar? ours, out Dictionary<RailTrack, HashSet<RailTrack>> sameWay)
        {
            var costs = new Dictionary<RailTrack, double>();
            sameWay = new Dictionary<RailTrack, HashSet<RailTrack>>();
            string? ourGuid = null;
            if (ours != null)
                foreach (var kv in runs)
                    if (kv.Value.car == ours)
                    {
                        ourGuid = kv.Key;
                        break;
                    }

            void Bump(RailTrack t, double c)
            {
                if (!costs.TryGetValue(t, out var e) || c > e)
                    costs[t] = c;
            }

            foreach (var kv in claims)
                if (kv.Value != ourGuid)
                    Bump(kv.Key, ClaimedRoadPenalty);

            foreach (var kv in runs)
            {
                var run = kv.Value;
                if (kv.Key == ourGuid || run.coroutine == null)
                    continue; // finished runs hold no road
                for (int i = run.pathFrom; i < run.path.Count; i++)
                {
                    if (run.path[i] == null)
                        continue;
                    Bump(run.path[i], PlannedRoadPenalty);
                    // The step it takes out of this track. Path order IS its direction of
                    // travel, so a route making the same step is going the same way.
                    if (i + 1 < run.path.Count && run.path[i + 1] != null)
                    {
                        if (!sameWay.TryGetValue(run.path[i], out var nexts))
                            sameWay[run.path[i]] = nexts = new HashSet<RailTrack>();
                        nexts.Add(run.path[i + 1]);
                    }
                }
            }
            return costs;
        }

        /// <summary>The junction physically connecting tracks a and b (either
        /// orientation), or null if they meet without one.</summary>
        private static Junction? JunctionBetween(RailTrack a, RailTrack b)
        {
            foreach (var j in new[] { a.inJunction, a.outJunction, b.inJunction, b.outJunction })
            {
                if (j == null)
                    continue;
                var brs = j.outBranches;
                bool hasA = j.inBranch?.track == a || (brs != null && brs.Any(x => x?.track == a));
                bool hasB = j.inBranch?.track == b || (brs != null && brs.Any(x => x?.track == b));
                if (hasA && hasB)
                    return j;
            }
            return null;
        }

        /// <summary>
        /// The junctions a pull leg has to draw clear of before the train may reverse:
        /// the ones between consecutive tracks on the leg, PLUS the junction at the end
        /// of the start track we are leaving by.
        ///
        /// That last one matters because JunctionBetween only finds a junction both
        /// tracks are branches of, and a yard track often reaches the throat through a
        /// frog stub that shares no junction object with it. With no junction found the
        /// pull fell back to "the whole train must leave the start track" — on a 359 m
        /// siding that reads as a third of a kilometre of pulling, and the train simply
        /// left the yard.
        /// </summary>
        private static IEnumerable<Junction> JunctionsToClear(
            List<RailTrack> path, int s, int e, RailTrack startTrack)
        {
            var seen = new HashSet<Junction>();
            for (int i = s; i <= e && i < path.Count - 1; i++)
            {
                var junction = JunctionBetween(path[i], path[i + 1]);
                if (junction != null && seen.Add(junction))
                    yield return junction;
            }
            bool exitOut = s + 1 < path.Count && ExitIsOut(path[s], path[s + 1]);
            var edge = exitOut ? startTrack.outJunction : startTrack.inJunction;
            if (edge != null && seen.Add(edge))
                yield return edge;
        }

        /// <summary>
        /// Metres the consist must still travel before this pull leg counts as clear:
        /// the tail off the start track, and every junction on the leg free of our own
        /// cars. Knowing the distance lets the pull be planned to a crawl instead of
        /// discovering the clear point at speed and rolling well past the switch.
        /// </summary>
        private static double PullRemaining(
            TrainCar loco, List<RailTrack> path, int s, int e, RailTrack startTrack)
        {
            double remaining = 0;
            bool anyJunction = false;

            // How much further the tail has to draw forward before every junction the
            // reverse retraces is clear of our own cars. This mirrors the pull-clear
            // test in DriveLeg — the two must agree, or the leg ends at a distance the
            // switch check will not accept, or never ends at all.
            foreach (var junction in JunctionsToClear(path, s, e, startTrack))
            {
                anyJunction = true;
                remaining = Math.Max(remaining, JunctionShortfall(junction, loco.trainset));
            }
            // And the train's own length: every car has to be off the road behind the
            // reversal track, not merely past the frog.
            double behind = BehindPivotShortfall(path, s, e, loco);
            if (behind > 0)
            {
                anyJunction = true;
                remaining = Math.Max(remaining, behind);
            }
            if (anyJunction)
                return remaining;

            // No junction on this leg: the only thing to wait for is the train leaving
            // the track it started on.
            bool exitOut = s + 1 < path.Count && ExitIsOut(path[s], path[s + 1]);
            double startLen = startTrack.GetKinkedPointSet().span;
            foreach (var c in loco.trainset.cars)
                foreach (var b in c.Bogies)
                    if (b?.track == startTrack)
                    {
                        double toEnd = exitOut ? startLen - b.traveller.Span : b.traveller.Span;
                        remaining = Math.Max(remaining, toEnd + OwnSwitchClearance + PullClearMargin);
                    }
            return remaining;
        }

        /// <summary>
        /// How much further the train must draw forward before NO part of it is still
        /// standing on the branch it came in by — the length of train left behind the
        /// pivot, in other words.
        ///
        /// This is the thing the proximity test cannot see. "No car body within 5.5 m
        /// of the frog" is true the moment the last car crawls past it, but a runaround
        /// between two branches of one junction needs the WHOLE consist onto the stem
        /// before it reverses. Stopping 4 m past the frog leaves the train lying back
        /// down the branch it came from; the router still reads that branch as the road
        /// it is on and points the switch back at it, and the reverse then returns the
        /// train exactly where it started. It re-plans the same runaround and does it
        /// again, for ever — which is what J-220 at SW was doing.
        /// </summary>
        private static double BehindPivotShortfall(
            List<RailTrack> path, int s, int e, TrainCar loco)
        {
            var pivot = PivotOf(path, e);
            return pivot == null
                ? 0
                : TrainBehindJunction(pivot.Value.pivot, pivot.Value.onto, loco);
        }

        /// <summary>The junction a pull leg reverses over, and the track it pulls onto —
        /// null on a leg that does not end in a reversal.</summary>
        private static (Junction pivot, RailTrack onto)? PivotOf(List<RailTrack> path, int e)
        {
            if (e + 1 >= path.Count)
                return null;
            var pivot = JunctionBetween(path[e], path[e + 1]);
            return pivot == null ? ((Junction, RailTrack)?)null : (pivot, path[e]);
        }

        /// <summary>Which end of this track sits at the junction — compared by position,
        /// because a yard track reaches the throat through a frog stub and holds no
        /// reference to the junction at all.</summary>
        private static bool JunctionAtInEnd(RailTrack track, Junction junction)
        {
            var points = track.GetKinkedPointSet().points;
            Vector3 move = WorldMover.currentMove;
            float dIn = ((Vector3)points[0].position + move - junction.position).magnitude;
            float dOut = ((Vector3)points[points.Length - 1].position + move - junction.position).magnitude;
            return dIn <= dOut;
        }

        /// <summary>
        /// How much further the train must pull for its LAST car to be past this
        /// junction. Zero means the whole consist is through and the switch can be
        /// thrown behind it.
        ///
        /// Walks the actual rail backwards from the junction, down every leg except the
        /// one being pulled onto, until it has covered a train's length. Every previous
        /// attempt at this measured something that was nearly the same and wrong in one
        /// case: body proximity to the frog said "clear" with a hundred metres of train
        /// still behind it; junction branch membership was blind to anything past the
        /// frog stubs, which are a few tens of metres long; and the planned path drops
        /// the road behind the loco every time the route is re-planned, which is every
        /// leg. Only the rail itself holds still.
        /// </summary>
        private static double TrainBehindJunction(Junction pivot, RailTrack onto, TrainCar loco)
        {
            if (loco.trainset?.cars == null)
                return 0;
            double limit = TrainLength(loco) + OwnSwitchClearance + PullClearMargin;
            var seen = new HashSet<RailTrack> { onto };
            var queue = new Queue<(RailTrack track, bool atIn, double before)>();

            // First ring: every leg of the junction except the one we are pulling onto.
            void Seed(RailTrack? t)
            {
                if (t != null && seen.Add(t))
                    queue.Enqueue((t, JunctionAtInEnd(t, pivot), 0.0));
            }
            Seed(pivot.inBranch?.track);
            if (pivot.outBranches != null)
                foreach (var b in pivot.outBranches)
                    Seed(b?.track);

            double worst = 0;
            while (queue.Count > 0)
            {
                var (track, atIn, before) = queue.Dequeue();
                double span = track.GetKinkedPointSet().span;

                foreach (var c in loco.trainset.cars)
                {
                    if (c?.Bogies == null)
                        continue;
                    foreach (var b in c.Bogies)
                    {
                        if (b?.track != track)
                            continue;
                        // How far back from the junction this bogie is, along the rail.
                        double back = before + (atIn
                            ? b.traveller.Span
                            : span - b.traveller.Span);
                        worst = Math.Max(worst, back + OwnSwitchClearance + PullClearMargin);
                    }
                }

                // Keep walking back until a train's length is covered — beyond that
                // nothing of ours can still be standing.
                double next = before + span;
                if (next >= limit)
                    continue;
                var onward = atIn ? track.GetAllOutBranches() : track.GetAllInBranches();
                if (onward == null)
                    continue;
                foreach (var branch in onward)
                {
                    if (branch?.track == null || !seen.Add(branch.track))
                        continue;
                    queue.Enqueue((branch.track, branch.first, next));
                }
            }
            return worst;
        }

        /// <summary>Length of the whole train, m — every car, not just the wagons that
        /// have to be spotted.</summary>
        private static double TrainLength(TrainCar loco)
        {
            double total = 0;
            if (loco.trainset?.cars == null)
                return 0;
            foreach (var c in loco.trainset.cars)
                if (c != null)
                    total += c.InterCouplerDistance;
            return total;
        }

        /// <summary>How much further our own cars must move to stop fouling this
        /// junction — the same test as <see cref="ConsistFoulsJunction"/>, but as a
        /// distance rather than a yes/no.</summary>
        private static double JunctionShortfall(Junction junction, Trainset ours)
        {
            // Measured against what the SWITCH check will demand, not some looser
            // fouling distance: pulling up short of that is what left the train unable
            // to throw the points it had just cleared.
            double need = OwnSwitchClearance + PullClearMargin;
            double worst = 0;
            foreach (var car in CarsAround(junction, ours))
            {
                double gap = BodyDistanceToJunction(junction, car);
                if (gap < need)
                    worst = Math.Max(worst, need - gap);
            }
            return worst;
        }

        /// <summary>True while one of OUR OWN cars still fouls the junction (so the
        /// tail hasn't cleared it yet). Ignores other parked trains — they don't
        /// gate our pull.</summary>
        private static bool ConsistFoulsJunction(Junction junction, Trainset ours) =>
            // Same measurement and the same margin as JunctionShortfall — see the note
            // there. These two must never disagree.
            JunctionBlocker(junction, ours, OwnSwitchClearance + PullClearMargin) != null;

        /// <summary>
        /// The first switch on this leg that has to be thrown and cannot be, because
        /// our own consist is standing on it.
        ///
        /// Asking BEFORE the leg starts is the whole point. The switch-setting pass
        /// at the top of a leg throws away its result, so a refused throw was
        /// invisible: the train rolled into the junction, took whichever leg the
        /// points already lay on, found itself off-plan, and recovered by going the
        /// long way round. That recovery is what made the move look like it had
        /// chosen a daft road — the road was fine, it simply could not be taken from
        /// where the train was standing.
        ///
        /// Read-only on purpose: this runs before the run has extended its
        /// authority, and must not re-point junctions it does not yet hold.
        /// </summary>
        private static Junction? OwnConsistOnSwitchAhead(
            TrainCar car, List<RailTrack> path, int s, int e)
        {
            var ours = car?.trainset;
            if (ours == null)
                return null;
            for (int i = s; i < e && i + 1 < path.Count; i++)
            {
                var dj = DivergingJunction(path[i], path[i + 1]);
                if (dj == null)
                    continue; // a merge needs no throw — trailing points trail
                var (junction, k) = dj.Value;
                if (junction.selectedBranch == k)
                    continue; // already lying our way, nothing to throw
                if (ConsistFoulsJunction(junction, ours))
                    return junction;
            }
            return null;
        }
    }
}
