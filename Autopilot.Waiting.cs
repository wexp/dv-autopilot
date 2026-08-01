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
        /// True while a run dispatched BEFORE ours is in our way — meaning the rail we
        /// still have to cover and the rail it still has to cover actually meet, or we
        /// are on the point of entering the track it is heading for. Ordering by
        /// dispatch number keeps it one-way: the earlier run never waits for the later
        /// one, so the two cannot sit there politely waiting for each other.
        /// </summary>
        private static bool RunAheadOfUs(Run ours, out string who, out string why)
        {
            who = "";
            why = "";
            foreach (var other in runs.Values)
            {
                if (other == ours || other.seq >= ours.seq)
                    continue;
                if (other.state != "driving" && other.state != "waiting")
                    continue; // arrived, coupled or failed — no longer in the way

                // Standing on the other run's remaining road means waiting for it can
                // never end: it cannot pass through our train, and we will not move
                // until it does. Two couple-shunts nose to tail on one road sat like
                // that for good — the front one crawling to a 2 m stand behind the
                // back one, the back one "waiting" for the front one. Moving away is
                // the only cure, so drive; the claims still serialise the actual rail.
                if (StandsOnRemainingPath(ours.car, other))
                    continue;

                // Do the two roads actually touch? Only the rail EACH STILL HAS TO
                // COVER counts — path is published whole at the top of a leg, and
                // rail already run must not keep reading as a conflict. Sharing a
                // destination is not by itself a reason to stand still either: a
                // runaround out to the far end of the yard and back uses none of the
                // rail the other train is on, and holding for it wasted minutes doing
                // nothing. The destination is left out of this test on purpose — it
                // is in both paths by definition.
                bool conflict = false;
                for (int oi = ours.pathFrom; oi < ours.path.Count && !conflict; oi++)
                {
                    var t = ours.path[oi];
                    if (t == ours.destTrack || t == other.destTrack)
                        continue;
                    int oj = other.path.IndexOf(t, Math.Min(other.pathFrom, other.path.Count));
                    if (oj < 0)
                        continue;

                    // Sharing a track — two cuts dropped on the same long siding, say —
                    // is only a real conflict if the two runs actually meet at one of
                    // its junctions: arriving from the same neighbour, or leaving for
                    // the same one (same direction), or crossing it in OPPOSITE
                    // directions — one's way in is the other's way out, which is a
                    // head-on meeting in the middle. Enter and leave through entirely
                    // different junctions and they are pulling apart across that
                    // track, never needing the same rail.
                    RailTrack? oPrev = oi > 0 ? ours.path[oi - 1] : null;
                    RailTrack? oNext = oi < ours.path.Count - 1 ? ours.path[oi + 1] : null;
                    RailTrack? tPrev = oj > 0 ? other.path[oj - 1] : null;
                    RailTrack? tNext = oj < other.path.Count - 1 ? other.path[oj + 1] : null;
                    bool sameJunction = (oPrev != null && oPrev == tPrev)
                        || (oNext != null && oNext == tNext)
                        || (oPrev != null && oPrev == tNext)
                        || (oNext != null && oNext == tPrev);
                    if (!sameJunction)
                        continue;

                    conflict = true;
                    why = $"our road crosses {other.id}'s";
                    break;
                }

                // The shared destination IS a conflict, but only at the point of going
                // into it — which is the last leg, and not a moment before — and only
                // when both trains come in through the SAME END of it. Two moves
                // spotting one long track from its two ends never need the same rail:
                // a loco sent in via one throat waited on a consist arriving through
                // the other for nothing. Claims already treat the two ends of a track
                // as separate reservations; this rule has to as well. The end is
                // decided by branch membership, not by comparing path neighbours —
                // two ladder roads converging on one frog are different neighbours
                // but the same end, and that is still a real conflict. When either
                // end cannot be worked out, assume the worst and wait.
                if (!conflict && ours.finalLeg && ours.destTrack != null
                    && other.destTrack == ours.destTrack)
                {
                    bool? ourEnd = EntryEndIsIn(ours.path, ours.destTrack);
                    bool? theirEnd = EntryEndIsIn(other.path, other.destTrack);
                    if (ourEnd == null || theirEnd == null || ourEnd == theirEnd)
                    {
                        conflict = true;
                        why = $"{other.id} is going into the same track";
                    }
                }

                if (!conflict)
                    continue;
                who = other.id;
                return true;
            }
            return false;
        }

        /// <summary>True while any car of our consist stands on rail the other run
        /// still has to cover. That other train cannot get past us however long it
        /// tries, so queueing behind IT is a deadlock — we must move first.</summary>
        private static bool StandsOnRemainingPath(TrainCar loco, Run other)
        {
            if (loco?.trainset?.cars == null)
                return false;
            var remaining = new HashSet<RailTrack>();
            for (int i = Math.Max(0, other.pathFrom); i < other.path.Count; i++)
                remaining.Add(other.path[i]);
            if (remaining.Count == 0)
                return false;
            foreach (var c in loco.trainset.cars)
                foreach (var b in c.Bogies)
                    if (b?.track != null && remaining.Contains(b.track))
                        return true;
            return false;
        }

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
                bool moving = Mathf.Abs(other.GetForwardSpeed()) > 0.3f
                    || IsUnderAutopilot(other.trainset);
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

        private static bool BlockedByWaitingRun(Run ours, HashSet<RailTrack> corridor)
        {
            foreach (var other in runs.Values)
            {
                if (other == ours || other.state != "waiting" || other.car == null)
                    continue;
                var trainset = other.car.trainset;
                if (trainset?.cars == null)
                    continue;
                foreach (var c in trainset.cars)
                    foreach (var b in c.Bogies)
                        if (b?.track != null && corridor.Contains(b.track))
                            return true;
            }
            return false;
        }
    }
}
