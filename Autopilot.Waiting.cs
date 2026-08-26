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
    /// <summary>Giving way between runs: who was dispatched first, whose road
    /// crosses whose, and when standing still is the right move.</summary>
    public static partial class Autopilot
    {
        /// <summary>
        /// Which end of dest this path arrives through: true for the track's in end,
        /// false for its out end, null when the path doesn't end on dest or the last
        /// hop isn't a rail neighbour of it (a re-plan mid-manoeuvre can leave either).
        /// </summary>
        private static bool? EntryEndIsIn(List<RailTrack> path, RailTrack dest)
        {
            if (path.Count < 2 || path[path.Count - 1] != dest)
                return null;
            var from = path[path.Count - 2];
            var ins = dest.GetAllInBranches();
            if (ins != null)
                foreach (var b in ins)
                    if (b?.track == from)
                        return true;
            var outs = dest.GetAllOutBranches();
            if (outs != null)
                foreach (var b in outs)
                    if (b?.track == from)
                        return false;
            return null;
        }

        /// <summary>
        /// True when the train standing in our way belongs to another autopilot that is
        /// itself waiting — i.e. the two are deadlocked and neither will move on its own.
        /// Only counts a run that is actually WAITING: one that is driving will clear in
        /// its own time, and giving way to it would just make two trains dither.
        /// </summary>
        /// <summary>Is anything in the corridor ahead actually moving — either rolling
        /// or being driven by another autopilot? Such a train clears itself, so holding
        /// for it must not count toward giving the road up. Our OWN consist is skipped:
        /// on a shove the wagons ride ahead of the loco, inside this very corridor, and
        /// counting them had the loco standing there "holding for a train ahead to
        /// pass" — waiting for its own wagons — for ever.</summary>
        private static bool MovingStockAhead(HashSet<RailTrack> corridor, Trainset? ours)
        {
            var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
            if (registry == null)
                return false;
            foreach (var other in registry.Values)
            {
                if (other == null || other.trainset == ours)
                    continue;
                // ACTUAL motion only. Counting any autopilot train as "moving"
                // meant two stopped runs each holding for the other reset their
                // give-up clocks for ever - a livelock with both lights green.
                bool moving = Mathf.Abs(other.GetForwardSpeed()) > 0.3f;
                if (!moving)
                    continue;
                foreach (var b in other.Bogies)
                    if (b?.track != null && corridor.Contains(b.track))
                        return true;
            }
            return false;
        }

        /// <summary>Is this trainset being driven by an autopilot that is still working?
        /// Routing asks, so it can price a track a train is passing over as a delay
        /// rather than shutting the road: the train will be gone shortly.</summary>
        /// <summary>Any car of an ACTIVE run standing in this corridor. A stopped
        /// convoy member ahead is someone to queue behind, never something to
        /// park against.</summary>
        private static bool AutopilotStockAhead(HashSet<RailTrack> corridor, Trainset? ours)
        {
            var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
            if (registry == null)
                return false;
            foreach (var other in registry.Values)
            {
                if (other == null || other.trainset == ours || !IsUnderAutopilot(other.trainset))
                    continue;
                foreach (var b in other.Bogies)
                    if (b?.track != null && corridor.Contains(b.track))
                        return true;
            }
            return false;
        }

        public static bool IsUnderAutopilot(Trainset? set)
        {
            if (set == null)
                return false;
            foreach (var run in runs.Values)
            {
                if (run.car == null || run.car.trainset != set)
                    continue;
                if (run.state == "driving" || run.state == "waiting")
                    return true;
            }
            return false;
        }

        /// <summary>May this run back off for that one, and to where?
        ///
        /// Only the JUNIOR gives way. A rule that can call each of a pair the one to move
        /// is how two trains take turns giving way and neither arrives — the same
        /// reasoning that fixed the order in Outranks. The senior stands: it has the road
        /// by rank and it is about to get it.
        ///
        /// A run already standing on the road it came from has nowhere to go. Standing is
        /// then the whole of the answer, and a train that stands for ever there is a
        /// fault showing itself rather than one being papered over.</summary>
        private static bool CanGiveWay(
            Run run, Run blocker, TrainCar car, out RailTrack? back, out string home)
        {
            back = null;
            home = "";
            if (run.retreatTo != null || !Outranks(blocker, run))
                return false;
            back = RetreatTarget(run, blocker, car);
            if (back == null)
                return false;
            home = back.LogicTrack()?.ID?.ToString() ?? "";
            return home.Length > 0 && !AnyCarOnTrack(car, home);
        }

        /// <summary>Every car in the consist end to end, m — locomotives included.
        ///
        /// NOT ConsistLength, which is a spotting measure: CarsToSpot leaves the driven
        /// units out on purpose, because a loco is allowed to hang over the throat when
        /// its wagons are inside the markers. Asked how much rail a train needs to stand
        /// clear on, that answer is short by a locomotive — and short by the whole train
        /// for a light engine, where the fallback measures the loco alone. It picked a
        /// 24 m connector for L-009 and backed one wagon's worth out of the way.
        ///
        /// Getting out of another train's road is not spotting. Every axle has to fit.</summary>
        private static double WholeTrainLength(TrainCar loco)
        {
            var cars = loco.trainset?.cars;
            if (cars == null)
                return loco.InterCouplerDistance;
            double total = 0;
            foreach (var c in cars)
                if (c != null)
                    total += c.InterCouplerDistance;
            return total;
        }

        /// <summary>Where to back off to: the NEAREST road behind us that puts the whole
        /// train out of the senior's way.
        ///
        /// Going all the way home was the first answer and it is a poor one — a shunt
        /// that has worked half way across a yard should not undo the lot to let one
        /// train past, and the longer the retreat the longer the senior waits for the
        /// throat to clear. What is actually needed is the first place we can stand that
        /// is not on its road.
        ///
        /// Breadth first over the rail joined to the train, in every direction, seeded
        /// with the tracks the consist is standing on. Those seeds are never answers,
        /// which is also what stops the search turning round and walking forward through
        /// our own train. The senior's road and anybody's standing stock are not expanded
        /// THROUGH, not merely rejected: rail beyond them is rail we would have to drive
        /// over them to reach. A road only counts if the whole consist fits on it, or the
        /// retreat could never finish, and it has to be one the router will actually take
        /// us to — a backwards move can price out on the shunt length cap, and a retreat
        /// that ends in "no route" would kill the run it was meant to save.
        ///
        /// The road we came from is the fallback, so this can only ever shorten the
        /// retreat, never fail one that used to work.</summary>
        private static RailTrack? RetreatTarget(Run run, Run blocker, TrainCar car)
        {
            var cars = car.trainset?.cars;
            if (cars == null)
                return Routable(run, car, run.originTrack);

            var foul = PlanAhead(blocker, PriorityReachMeters);
            var under = new HashSet<RailTrack>();
            foreach (var c in cars)
                if (c != null)
                    foreach (var b in c.Bogies)
                        if (b?.track != null)
                            under.Add(b.track);
            if (under.Count == 0)
                return Routable(run, car, run.originTrack);

            double need = WholeTrainLength(car) + ClearancePad;
            // Each candidate that FITS costs a full route search, and planning runs on
            // the main thread - the drive loop already times it because a slow plan is
            // something the player feels. Cap the probes: this fires once when a retreat
            // arms, not every tick, but a yard full of roads our train fits on could
            // otherwise buy a dozen searches at once. Out of probes means fall back.
            int probes = 6;
            var seen = new HashSet<RailTrack>(under);
            var queue = new Queue<(RailTrack track, double dist)>();
            foreach (var t in under)
                queue.Enqueue((t, 0.0));

            while (queue.Count > 0)
            {
                var (here, dist) = queue.Dequeue();
                foreach (var next in Neighbours(here))
                {
                    if (!seen.Add(next))
                        continue;
                    if (foul.Contains(next) || ForeignStockOn(next, car.trainset))
                        continue; // not through the senior's road, nor through stock
                    double d = dist + PhysicalSpan(next);
                    if (d > RetreatSearchMeters)
                        continue;
                    if (PhysicalSpan(next) >= need && probes > 0)
                    {
                        probes--;
                        if (Routable(run, car, next) == null)
                        {
                            queue.Enqueue((next, d));
                            continue;
                        }
                        var found = next;
                        double at = d;
                        Trace.Line("RESV", () => $"{run.id} backing off to"
                            + $" {Trace.Id(found)} for {blocker.id} \u2014 {Trace.M(at)} back,"
                            + " nearest road clear of its route");
                        return next;
                    }
                    queue.Enqueue((next, d));
                }
            }
            Trace.Line("RESV", () => $"{run.id} found no nearer road clear of"
                + $" {blocker.id} within {RetreatSearchMeters:F0} m \u2014 going back to where"
                + " it started");
            return Routable(run, car, run.originTrack);
        }

        /// <summary>The track, if the router will take us there from where we stand.
        /// Asked before a retreat is committed to rather than after: Drive ends a run
        /// that cannot reach its destination, and a retreat is supposed to save one.</summary>
        private static RailTrack? Routable(Run run, TrainCar car, RailTrack? track)
        {
            var id = track?.LogicTrack()?.ID?.ToString();
            var from = car.Bogies[1]?.track;
            if (track == null || from == null || id == null || id.Length == 0)
                return null;
            return Routing.FindPath(from, id, car, null, ArriveMode.Auto, null,
                run.reversalPenalty) != null ? track : null;
        }

        /// <summary>Is the run we are giving way to still stood there? Named rather than
        /// held as a reference so a run that has finished, or been stopped by hand, reads
        /// as "gone" instead of keeping a train backing away from nothing. The moment it
        /// is moving again the retreat has served its purpose: the road we wanted is
        /// being cleared by the train that wanted it first.</summary>
        private static bool StillStopped(string id)
        {
            if (string.IsNullOrEmpty(id))
                return false;
            foreach (var r in runs.Values)
                if (r.id == id)
                    return r.coroutine != null && r.car != null
                        && Mathf.Abs(r.car.GetForwardSpeed()) * 3.6f <= MovingKmh;
            return false;
        }

        /// <summary>Another run standing still in our corridor — the train we are
        /// deadlocked with. Named, because giving way means routing round where THAT
        /// train is, and a bool cannot say where that is.</summary>
        private static bool BlockedByWaitingRun(Run ours, HashSet<RailTrack> corridor, out Run? blocker)
        {
            blocker = null;
            foreach (var other in runs.Values)
            {
                if (other == ours || other.car == null)
                    continue;
                // "Waiting" now means stood at the end of its authority, not a
                // dedicated state: a run that is stopped is one that cannot move.
                if (Mathf.Abs(other.car.GetForwardSpeed()) * 3.6f > MovingKmh)
                    continue;
                var trainset = other.car.trainset;
                if (trainset?.cars == null)
                    continue;
                foreach (var c in trainset.cars)
                    foreach (var b in c.Bogies)
                        if (b?.track != null && corridor.Contains(b.track))
                        {
                            blocker = other;
                            return true;
                        }
            }
            return false;
        }

        /// <summary>A live run whose train is in our corridor — the one we are following.
        /// Standing behind a train that is going somewhere is not a blocked road, however
        /// long it takes: it is a following move, and the gap opens again as soon as the
        /// leader draws ahead. Only stock that will never move is worth routing round. A
        /// leader that has itself come to a stand is the deadlock branch's business, and
        /// that fires first.</summary>
        private static bool LiveRunAhead(Run ours, HashSet<RailTrack> corridor, out Run? leader)
        {
            leader = null;
            foreach (var other in runs.Values)
            {
                if (other == ours || other.car == null || other.coroutine == null)
                    continue;
                var trainset = other.car.trainset;
                if (trainset?.cars == null)
                    continue;
                foreach (var c in trainset.cars)
                    foreach (var b in c.Bogies)
                        if (b?.track != null && corridor.Contains(b.track))
                        {
                            leader = other;
                            return true;
                        }
            }
            return false;
        }

    }
}
