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
    /// <summary>Reading the rail as a graph: which end of a track meets which,
    /// how a path splits into legs at reversals, the step list a plan compiles
    /// to, and telling a straight ride through a switch from a diverging one.</summary>
    public static partial class Autopilot
    {
        /* --------------------------------------------------------- graph -- */

        /// <summary>True when track b hangs off a's OUT end (else its IN end).</summary>
        private static bool ExitIsOut(RailTrack a, RailTrack b)
        {
            var outs = a.GetAllOutBranches();
            return outs != null && outs.Any(br => br?.track == b);
        }

        /// <summary>True when track b hangs off a's IN end. Together with ExitIsOut
        /// this answers "are these two actually connected right now" — which for a
        /// turntable bridge depends on where the table happens to point.</summary>
        private static bool EnterIsIn(RailTrack a, RailTrack b)
        {
            var ins = a.GetAllInBranches();
            return ins != null && ins.Any(br => br?.track == b);
        }

        /// <summary>Every bogie of the consist on this one track — the whole train
        /// fits within it. The turntable asks before it turns.</summary>
        private static bool AllOnTrack(TrainCar loco, RailTrack track)
        {
            foreach (var c in loco.trainset.cars)
                foreach (var b in c.Bogies)
                    if (b == null || b.track != track)
                        return false;
            return true;
        }

        /// <summary>
        /// Split the path into legs at reversal tracks. On a straight-through track
        /// the train enters one end and leaves the other; where it must enter and
        /// leave the <em>same</em> end it has to stop and reverse, which starts a new
        /// leg. Each returned pair (s, e) is an inclusive track-index range; the last
        /// track of a leg is the reversal track it shares with the next leg.
        /// </summary>
        private static List<(int s, int e)> ComputeLegs(List<RailTrack> path)
        {
            var legs = new List<(int, int)>();
            int start = 0;
            for (int i = 1; i < path.Count - 1; i++)
            {
                bool entryFirst = !ExitIsOut(path[i], path[i - 1]);
                bool exitFirst = !ExitIsOut(path[i], path[i + 1]);
                if (entryFirst == exitFirst)
                {
                    legs.Add((start, i));
                    start = i;
                }
            }
            legs.Add((start, path.Count - 1));
            return legs;
        }

        /// <summary>The junction's number as the map and the console show it.</summary>
        private static int JunctionIndex(Junction? junction)
        {
            if (junction == null)
                return -1;
            var ordered = RailTrackRegistry.Instance?.OrderedJunctions?.ToList();
            return ordered == null ? -1 : ordered.IndexOf(junction);
        }

        /// <summary>
        /// Say what a route MEANS, in moves rather than in tracks: drive here, clear
        /// that junction and come back over it, run round and re-enter by the far end.
        ///
        /// The legs already hold the shape of it — each one ends where the train has
        /// to change ends — so this is a translation, not a second opinion. Keeping it
        /// a translation is the point: the step list can never describe a move the
        /// drive loop is not about to make.
        ///
        /// Only junctions the run must actually DO something at appear. The dozens
        /// rolled over on a set road are not steps, they are the road; a step is a
        /// place the train stops, or a switch it has to be clear of.
        /// </summary>
        private static List<Step> CompileSteps(
            List<RailTrack> path, List<(int s, int e)> legs, bool? arriveEnteredIn, string destId)
        {
            string Id(RailTrack t) => t.LogicTrack()?.ID?.ToString() ?? "?";
            var steps = new List<Step>();

            for (int li = 0; li < legs.Count; li++)
            {
                var (s, e) = legs[li];
                bool last = li == legs.Count - 1;

                if (!last)
                {
                    // A "reversal" onto or off a turntable bridge is no reversal at
                    // all — the leg boundary is where the train stops for the table
                    // to wind round under it.
                    if (Turntables.AsBridge(path[e]) != null && e + 1 < path.Count)
                    {
                        steps.Add(Step.Turntable(Id(path[e + 1])));
                        continue;
                    }
                    // The leg runs out to the reversal track and stops there; the
                    // junction it must clear is the one it will come back over.
                    steps.Add(Step.Drive(Id(path[e])));
                    steps.Add(Step.Reverse(JunctionIndex(JunctionBetween(path[e], path[e + 1]))));
                    continue;
                }

                // Ending on the track we set off from means this was a runaround: the
                // cars in between could not be driven through, so the train has gone
                // out, round, and is coming back in the other end.
                if (arriveEnteredIn != null && path.Count > 1 && path[e] == path[0])
                {
                    steps.Add(Step.TrackReverse(Id(path[e]), arriveEnteredIn.Value));
                    continue;
                }

                // The switch into the destination has to end up behind the whole
                // train, or the consist is left fouling the throat it came in by.
                if (e > s)
                {
                    var throat = DivergingJunction(path[e - 1], path[e])?.junction
                        ?? JunctionBetween(path[e - 1], path[e]);
                    if (throat != null)
                        steps.Add(Step.Cross(JunctionIndex(throat)));
                }
                steps.Add(Step.Drive(destId));
            }

            return steps;
        }

        /// <summary>
        /// A junction on the plan that our OWN stock currently runs through on a
        /// branch the plan does not use — the train's front went down the wrong
        /// road (shoved past a switch set the other way), and the plan cannot be
        /// driven until the front backs off it. Null when the train lies along
        /// its plan.
        /// </summary>
        private static Junction? WrongBranchJunction(TrainCar loco, List<RailTrack> path)
        {
            var ours = new HashSet<RailTrack>();
            var cars = loco.trainset?.cars;
            if (cars == null)
                return null;
            foreach (var c in cars)
                foreach (var b in c.Bogies)
                    if (b?.track != null)
                        ours.Add(b.track);

            for (int i = 0; i + 1 < path.Count; i++)
            {
                var junction = JunctionBetween(path[i], path[i + 1]);
                if (junction == null)
                    continue;
                var branches = new List<RailTrack?>();
                if (junction.inBranch?.track != null)
                    branches.Add(junction.inBranch.track);
                if (junction.outBranches != null)
                    foreach (var ob in junction.outBranches)
                        if (ob?.track != null)
                            branches.Add(ob.track);
                foreach (var t in branches)
                    if (t != null && t != path[i] && t != path[i + 1] && ours.Contains(t))
                        return junction;
            }
            return null;
        }

        /* ------------------------------------------- straight or diverging -- */

        // How far along each leg to look when judging which branch runs straight
        // on. The tangents AT the junction are nearly identical for both legs of
        // a turnout — the curve develops over the following metres — so the test
        // compares where each leg actually IS this far in.
        private const double BranchProbeMeters = 25.0;

        private static readonly Dictionary<Junction, int> straightBranchCache =
            new Dictionary<Junction, int>();

        /// <summary>Direction from a track's given end toward a point
        /// ~<see cref="BranchProbeMeters"/> in, in world space. Zero when the
        /// geometry is unreadable.</summary>
        private static Vector3 BranchProbeDirection(RailTrack track, bool fromFirst)
        {
            var points = track.GetKinkedPointSet()?.points;
            if (points == null || points.Length < 2)
                return Vector3.zero;
            var origin = points[fromFirst ? 0 : points.Length - 1];
            double endSpan = origin.span;
            var probe = origin;
            for (int i = 0; i < points.Length; i++)
            {
                probe = points[fromFirst ? i : points.Length - 1 - i];
                if (Math.Abs(probe.span - endSpan) >= BranchProbeMeters)
                    break;
            }
            var dir = (Vector3)probe.position - (Vector3)origin.position;
            return dir.sqrMagnitude < 0.01f ? Vector3.zero : dir.normalized;
        }

        /// <summary>The out-branch that continues straightest through the
        /// junction, or -1 when it cannot be told (unreadable geometry, or a wye
        /// where both legs bend away alike). Riding the straight leg is not
        /// switchwork and is not held to junction speed.</summary>
        private static int StraightBranch(Junction junction)
        {
            if (straightBranchCache.TryGetValue(junction, out var cached))
                return cached;

            int best = -1;
            var stem = junction.inBranch?.track;
            var outs = junction.outBranches;
            if (stem != null && outs != null && outs.Count >= 2)
            {
                // Through-direction of the stem INTO the junction: from a probe
                // point back toward the junction end.
                var into = -BranchProbeDirection(stem, junction.inBranch.first);
                if (into != Vector3.zero)
                {
                    float bestDot = -2f, secondDot = -2f;
                    for (int k = 0; k < outs.Count; k++)
                    {
                        var t = outs[k]?.track;
                        if (t == null)
                            continue;
                        var away = BranchProbeDirection(t, outs[k].first);
                        if (away == Vector3.zero)
                            continue;
                        float dot = Vector3.Dot(into, away);
                        if (dot > bestDot)
                        {
                            secondDot = bestDot;
                            bestDot = dot;
                            best = k;
                        }
                        else if (dot > secondDot)
                        {
                            secondDot = dot;
                        }
                    }
                    // Both legs equally curved (a wye): neither is straight, treat
                    // every road through it as a diverging move. cos difference of
                    // ~0.0006 is about 2° at these angles.
                    if (best >= 0 && secondDot > -2f && bestDot - secondDot < 0.0006f)
                        best = -1;
                }
            }
            straightBranchCache[junction] = best;
            return best;
        }

        /// <summary>True when taking this branch actually bends off the through
        /// road — the move that needs junction speed. Unknown geometry counts as
        /// diverging, which errs slow, never fast.</summary>
        private static bool TakesDivergingBranch((Junction junction, byte branch) dj)
        {
            int straight = StraightBranch(dj.junction);
            return straight < 0 || straight != dj.branch;
        }

        /// <summary>The junction where track a is the stem and track b diverges off
        /// it, plus b's branch index — null when the two just merge (no switch).</summary>
        private static (Junction junction, byte branch)? DivergingJunction(RailTrack a, RailTrack b)
        {
            foreach (var junction in new[] { a.inJunction, a.outJunction })
            {
                if (junction == null || junction.inBranch?.track != a)
                    continue;
                var branches = junction.outBranches;
                if (branches == null)
                    continue;
                for (byte k = 0; k < branches.Count; k++)
                    if (branches[k].track == b)
                        return (junction, k);
            }
            return null;
        }
    }
}
