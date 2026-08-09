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

        /// <summary>An OLDER (earlier-dispatched) run is stopped waiting for this
        /// block: it said so within the last few seconds. Later runs must leave
        /// the block for it - that is the whole of the queueing discipline.</summary>
        private static bool WantedByOlder(object key, Run run, out string byWho)
        {
            byWho = "";
            if (!blockWants.TryGetValue(key, out var want))
                return false;
            if (Time.realtimeSinceStartup - want.time > WantFreshSeconds)
                return false;
            if (want.seq >= run.seq)
                return false;
            foreach (var r in runs.Values)
                if (r.seq == want.seq)
                {
                    byWho = r.id;
                    return true;
                }
            return false; // the wanter is gone; its entry will age out
        }

        private static void RegisterWant(object key, Run run)
        {
            if (blockWants.TryGetValue(key, out var want)
                && want.seq < run.seq
                && Time.realtimeSinceStartup - want.time <= WantFreshSeconds)
                return; // an older run is already queued here
            blockWants[key] = (run.seq, Time.realtimeSinceStartup);
        }

        private static bool GrantTrack(Run run, RailTrack track, out string holder)
        {
            holder = "";
            if (claims.TryGetValue(track, out var guid) && guid != run.guid)
            {
                holder = RunNameOf(guid);
                return false;
            }
            if (WantedByOlder(track, run, out holder))
                return false;
            // A block with somebody's stock STANDING on it does not grant, however
            // the ledger reads - the release-margin behind a train is metres, and
            // authority through a tail is how two locos met on a frog. The
            // destination is the one exception: pulling up short of (or coupling
            // to) what stands there is the drive loop's job.
            if (track != run.destTrack && ForeignStockOn(track, run.car?.trainset))
            {
                holder = "stock standing on " + (track.LogicTrack()?.ID?.ToString() ?? "it");
                return false;
            }
            claims[track] = run.guid;
            run.claimed.Add(track);
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
            if (junctionClaims.TryGetValue(junction, out var guid) && guid != run.guid)
            {
                holder = RunNameOf(guid);
                return false;
            }
            if (WantedByOlder(junction, run, out holder))
                return false;
            var ours = run.car?.trainset;
            foreach (var fouler in CarsAround(junction, null))
            {
                if (fouler == null || fouler.trainset == ours)
                    continue;
                if (BodyDistanceToJunction(junction, fouler) >= JunctionGrantClearance)
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
                holder = fouler.ID;
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
                holder = RunNameOf(g);
                return false;
            }
            junctionClaims[junction] = run.guid;
            run.claimedJunctions.Add(junction);
            foreach (var leg in JunctionLegs(junction))
            {
                if (claims.TryGetValue(leg, out var g) && g != run.guid)
                    continue; // the standing follower keeps its own rail
                // NEVER take a leg somebody's stock is standing on - taking
                // SW-A2P out from under two queued locos is how the ledger came
                // to disagree with the ground. The frog itself is safe (fouling
                // check above); an occupied leg is defended by the stock on it.
                if (ForeignStockOn(leg, run.car?.trainset))
                    continue;
                claims[leg] = run.guid;
                run.claimed.Add(leg);
            }
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
            bool towardOut, double ownLead, double need,
            out string heldBy, out string atName, out RailTrack? blockedTrack,
            int legEnd = int.MaxValue)
        {
            heldBy = "";
            atName = "";
            blockedTrack = null;

            ForceHold(run, track);

            double granted = towardOut ? PhysicalSpan(track) - bogieSpan : bogieSpan;
            var t = track;
            bool outw = towardOut;
            int i = idx;
            var seen = new HashSet<RailTrack> { track };
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
                    break; // rail ends, or loops - authority ends with it

                if (junction != null && !GrantJunction(run, junction, out heldBy))
                {
                    atName = $"J-{JunctionIndex(junction)}";
                    blockedTrack = next;
                    RegisterWant(junction, run);
                    break;
                }
                if (!GrantTrack(run, next, out heldBy))
                {
                    atName = next.LogicTrack()?.ID?.ToString() ?? "the next track";
                    blockedTrack = next;
                    RegisterWant(next, run);
                    break;
                }
                granted += PhysicalSpan(next);
                t = next;
                outw = nextOut;
                i = i >= 0 && i + 1 < path.Count ? i + 1 : -1;
            }
            return Math.Max(0, granted - ownLead);
        }

        /// <summary>Hold the rail the train physically stands on, without asking:
        /// stock on the ground beats any reservation. Never steals a key another
        /// run holds - their claim is already useless with us standing here.</summary>
        private static void ForceHold(Run run, RailTrack track)
        {
            if (!claims.TryGetValue(track, out var guid) || guid == run.guid)
            {
                claims[track] = run.guid;
                run.claimed.Add(track);
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
            List<RailTrack>? giveTracks = null;
            foreach (var track in run.claimed)
            {
                if (OurCarsOn(run.car, track))
                    continue;
                if (WantedByOlder(track, run, out _))
                    (giveTracks ??= new List<RailTrack>()).Add(track);
            }
            if (giveTracks != null)
                foreach (var track in giveTracks)
                {
                    if (claims.TryGetValue(track, out var guid) && guid == run.guid)
                        claims.Remove(track);
                    run.claimed.Remove(track);
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

            for (int i = idx; i < path.Count - 1 && ahead < JunctionCheckDistance; i++)
            {
                // Someone else's reservation means it is not ours to set OR to enter —
                // for THIS side of it; entering path[i + 1] from path[i] is a distinct
                // claim from entering it from its other end.
                if (claims.TryGetValue(path[i + 1], out var holder) && holder != run.guid)
                {
                    distance = ahead;
                    // Another train's reservation clears by itself; never route round it.
                    reason = runs.TryGetValue(holder, out var other)
                        ? $"{other.id} holds {path[i + 1].LogicTrack()?.ID}"
                        : "track reserved ahead";
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
                        distance = ahead;
                        blocked = path[i + 1];
                        var jid = path[i + 1].LogicTrack()?.ID;
                        reason = $"switch to {jid} won't set: {why}";
                        return false;
                    }
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
        internal static Dictionary<RailTrack, double> ForeignRoadCosts(TrainCar? ours)
        {
            var costs = new Dictionary<RailTrack, double>();
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
                    if (run.path[i] != null)
                        Bump(run.path[i], PlannedRoadPenalty);
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
