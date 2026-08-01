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

        private static bool BlockedByWaitingRun(Run ours, HashSet<RailTrack> corridor)
        {
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
                            return true;
            }
            return false;
        }
    }
}
