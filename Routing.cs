using DV.Utils;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// Shortest-path routing over the rail network — the foundation of the loco
    /// autopilot. The graph is the RailTrack network: two tracks are adjacent when
    /// a junction branch physically connects them. Dijkstra by track length gives
    /// the shortest drivable route between two tracks.
    ///
    /// Stage 1 only computes and returns the route (as track ids) so it can be
    /// drawn on the map for verification; switching and driving come later.
    /// </summary>
    /// <summary>How the train should be facing when it arrives.</summary>
    public enum ArriveMode
    {
        /// <summary>Shove onto an occupied destination, otherwise whatever is shortest.</summary>
        Auto,
        /// <summary>Always arrive wagons first, however far round it has to go.</summary>
        Shove,
        /// <summary>Always arrive loco first.</summary>
        Pull,
    }

    public static class Routing
    {
        // A reversal costs no distance but plenty of time — stop, change ends, wait for
        // the junction to clear of the whole train. Price it so a slightly longer route
        // that rolls straight through beats a short one full of runarounds, m.
        private const double ReversalPenalty = 400.0;

        // What a reversal is worth depends on what kind of move this is, so the two
        // halves of the autopilot price it differently. On the road a runaround means
        // stopping a main-line train and changing ends on a running line, which is
        // worth going a long way round to avoid. In a yard it is the ordinary way of
        // working — nearly every move into a siding is one — and pricing it like a
        // road move sent shunts on absurd laps of the throat to dodge a change of
        // ends that would have taken twenty seconds.
        public const double LineReversalPenalty = 800.0;
        public const double YardReversalPenalty = 60.0;

        // What a track with a MOVING train on it costs, m of equivalent rail. Treating
        // one as impassable is what sent the second of two autopilots bound for the
        // same siding round the whole map: the first was sitting on the only sensible
        // road, so the only "clear" route left was a lap of the valley. It will not be
        // there for long, and standing at the throat until it has gone beats driving
        // fifteen kilometres — but the figure is high enough that a genuinely short way
        // round still wins.
        private const double BusyTrackPenalty = 250.0;
        // Above this a train counts as under way rather than parked, m/s.
        private const float MovingMs = 0.3f;

        // Arriving shoved is a preference, not a mission. Beyond this much extra rail
        // it is not worth it: a lap of the whole map with three reversals to put five
        // wagons away is worse than simply pulling in, m.
        private const double MaxPushDetour = 600.0;

        // How much extra a CLEAN road is worth over one that crosses live traffic
        // (another run's claims or plan, or stock on the move), m of equivalent
        // rail. The soft penalties already tilt Dijkstra away from traffic; this is
        // the second look — probe alternates that dodge the specific conflicted
        // tracks, and take one outright if it costs no more than this on top.
        private const double CleanDetourMax = 500.0;

        /// <summary>
        /// Tracks reachable off one end of this one. "in" is the span-0 end (the
        /// in-junction), "out" the span-max end. Each result also says which end of the
        /// NEXT track we arrive at, so the search knows the direction of travel and not
        /// merely the adjacency.
        /// </summary>
        private static IEnumerable<(RailTrack track, bool enteredIn, double extra)> Neighbors(
            RailTrack track, bool exitIn, double trainLength)
        {
            var branches = exitIn ? track.GetAllInBranches() : track.GetAllOutBranches();
            if (branches != null)
                foreach (var b in branches)
                    if (b?.track != null)
                        yield return (b.track, b.first, 0);
            // A turntable bridge reaches every stall of its table (and a stall the
            // bridge) after a wind of the table — edges the live rail graph only
            // shows for whichever single stall is aligned right now. Priced so
            // plain rail wins where plain rail exists, and withheld entirely when
            // the consist cannot fit on the bridge.
            foreach (var v in Turntables.VirtualNeighbors(track, exitIn, trainLength))
                yield return v;
        }

        /// <summary>
        /// Where every car that isn't ours is sitting, by track. A track with someone
        /// else's stock on it cannot be driven through: the classic case is a light
        /// loco that has just dropped its train on a siding and must leave by the far
        /// end, because the short way out is through the cars it just parked.
        /// </summary>
        private sealed class Occupancy
        {
            private readonly Dictionary<RailTrack, List<double>> spans =
                new Dictionary<RailTrack, List<double>>();
            // Tracks with stock on them that is NOT going to move by itself.
            private readonly HashSet<RailTrack> parked = new HashSet<RailTrack>();

            public static Occupancy Build(Trainset? ours)
            {
                var occupancy = new Occupancy();
                var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
                if (registry == null)
                    return occupancy;
                foreach (var trainCar in registry.Values)
                {
                    if (trainCar == null || trainCar.trainset == ours)
                        continue;
                    // A train under way is a delay, not a wall. Stock that is simply
                    // standing there is a wall: nothing will move it but a shunter.
                    bool willMove = Autopilot.IsUnderAutopilot(trainCar.trainset)
                        || Mathf.Abs(trainCar.GetForwardSpeed()) > MovingMs;
                    foreach (var bogie in trainCar.Bogies)
                    {
                        if (bogie?.track == null)
                            continue;
                        if (!occupancy.spans.TryGetValue(bogie.track, out var list))
                            occupancy.spans[bogie.track] = list = new List<double>();
                        list.Add(bogie.traveller.Span);
                        if (!willMove)
                            occupancy.parked.Add(bogie.track);
                    }
                }
                return occupancy;
            }

            /// <summary>Anyone at all on this track — it can't be driven end to end.</summary>
            public bool Any(RailTrack track) => spans.ContainsKey(track);

            /// <summary>Stock on it that will still be there however long we wait. The
            /// road is genuinely shut, as opposed to merely busy.</summary>
            public bool Parked(RailTrack track) => parked.Contains(track);

            /// <summary>Anyone within (lo, hi) of this track's span — used for the track
            /// the loco starts on, where only the stock between it and the end it is
            /// leaving by matters.</summary>
            public bool AnyBetween(RailTrack track, double lo, double hi)
            {
                if (!spans.TryGetValue(track, out var list))
                    return false;
                foreach (var s in list)
                    if (s > lo && s < hi)
                        return true;
                return false;
            }
        }

        private static bool Matches(RailTrack track, string destId)
        {
            var id = track.LogicTrack()?.ID;
            if (id == null)
                return false;
            return id.ToString() == destId || id.FullDisplayID == destId || id.FullID == destId;
        }

        private static double Length(List<RailTrack> path)
        {
            double total = 0;
            foreach (var t in path)
                total += t.GetKinkedPointSet().span;
            return total;
        }

        /// <summary>Is anyone else's stock standing on this track? An empty destination
        /// can be entered loco-first; one with cars already on it is worth the runaround
        /// to arrive shoving, so the wagons go in behind them and the loco stays out.</summary>
        public static bool HasForeignCars(RailTrack track, Trainset? ours)
        {
            return Occupancy.Build(ours).Any(track);
        }

        /// <summary>Which of these tracks have stock on them that isn't ours — the
        /// reason a route bends away from the short way round. Diagnostic only.</summary>
        public static HashSet<RailTrack> OccupiedTracks(IEnumerable<RailTrack> tracks, Trainset? ours)
        {
            var occupancy = Occupancy.Build(ours);
            var occupied = new HashSet<RailTrack>();
            foreach (var t in tracks)
                if (t != null && occupancy.Any(t))
                    occupied.Add(t);
            return occupied;
        }

        /// <summary>
        /// Shortest drivable route from start to the first track matching destId.
        ///
        /// When the loco is given, the search avoids tracks other stock is standing on
        /// and, on the starting track, avoids leaving by an end the loco can't reach
        /// past its own just-parked train. If everything is blocked it falls back to the
        /// occupancy-blind route, so a plan is still produced (the drive's own obstacle
        /// guard then stops short of whatever is in the way) rather than nothing at all.
        /// </summary>
        /// <param name="arriveEnteredIn">
        /// When set, the route must ARRIVE on the destination through that end of it
        /// (true = the span-0 end), and standing on the destination already does not
        /// count as arrived. That is how "put this loco on the other side of the track
        /// it is on" is expressed: it has to leave, run round, and come back in the
        /// other way, because the cars in between cannot be driven through.
        /// </param>
        /// <param name="preferPush">
        /// Prefer a route that ends with the train being SHOVED onto the destination —
        /// wagons first, loco trailing — which is how stock is put away: the last wagon
        /// goes to the far end and the loco is left on the outside, free to uncouple
        /// and leave. Costs an extra runaround when the short way in would arrive
        /// loco-first, so it is a preference: if no shoving route exists at all, the
        /// ordinary one is used rather than refusing the move.
        /// </param>
        /// <param name="avoid">
        /// Track the caller has learned it cannot use — typically the far side of a
        /// junction that will not throw because a wagon is standing on it. Honoured if
        /// a way round exists; ignored rather than failing the move if not, so the
        /// train falls back to waiting at the blockage instead of giving up.
        /// </param>
        /// <summary>Set by the last FindPath when the only route it could find runs
        /// through stock that is standing in the way — the caller is being handed a road
        /// the train cannot actually drive. Read it immediately after FindPath.</summary>
        public static bool BlockedRoute { get; private set; }

        public static List<RailTrack>? FindPath(
            RailTrack start, string destId, TrainCar? loco = null, bool? arriveEnteredIn = null,
            ArriveMode arrive = ArriveMode.Auto, HashSet<RailTrack>? avoid = null,
            double reversalPenalty = ReversalPenalty)
        {
            BlockedRoute = false;
            if (avoid != null && avoid.Count > 0)
            {
                var around = Plan(start, destId, loco, arriveEnteredIn, arrive, avoid, reversalPenalty);
                if (around != null)
                    return around;
                return Plan(start, destId, loco, arriveEnteredIn, arrive, null, reversalPenalty);
            }
            return Plan(start, destId, loco, arriveEnteredIn, arrive, null, reversalPenalty);
        }

        private static List<RailTrack>? Plan(
            RailTrack start, string destId, TrainCar? loco, bool? arriveEnteredIn,
            ArriveMode arrive, HashSet<RailTrack>? avoid, double reversalPenalty)
        {
            if (loco == null)
                return Search(start, destId, null, null, arriveEnteredIn, null, avoid,
                    reversalPenalty, null, out _);

            var occupancy = Occupancy.Build(loco.trainset);
            // What the other autopilots' roads cost to cross — their claims and the
            // rail their plans still have to cover.
            var foreign = Autopilot.ForeignRoadCosts(loco);
            // The occupancy-blind fallback exists so a plan is always produced — the
            // drive's own guard then stops short of whatever is in the way. Callers can
            // tell the two apart through BlockedRoute, which describes the path actually
            // RETURNED. Setting it from "some search fell back" was wrong: Auto tries a
            // shove route as well, and that attempt falling back condemned a perfectly
            // clean route — every second consist sent to an occupied siding was refused.
            List<RailTrack>? Try(bool? facing)
            {
                var strict = Search(start, destId, loco, occupancy, arriveEnteredIn, facing,
                    avoid, reversalPenalty, foreign, out var cost);
                if (strict != null)
                    return Divert(strict, cost, facing);
                return Search(start, destId, loco, null, arriveEnteredIn, facing,
                    avoid, reversalPenalty, foreign, out _);
            }

            // The tracks of a path that cross live traffic: someone's claim or plan,
            // or stock under way. The destination does not count — meeting others
            // THERE is the drive loop's queueing problem, not a routing one.
            List<RailTrack> Conflicts(List<RailTrack> path)
            {
                var conflicted = new List<RailTrack>();
                for (int i = 1; i < path.Count - 1; i++)
                    if (foreign.ContainsKey(path[i]) || occupancy.Any(path[i]))
                        conflicted.Add(path[i]);
                return conflicted;
            }

            // Second look: the best path under soft costs may still cross other
            // runs' roads. One probe with every conflicted track off limits — if
            // that plans CLEAN and costs no more than CleanDetourMax on top, take
            // it. (A partial dodge would just re-answer the question the soft
            // costs already answered, so there is no ladder to descend.)
            List<RailTrack> Divert(List<RailTrack> path, double cost, bool? facing)
            {
                var conflicted = Conflicts(path);
                if (conflicted.Count == 0)
                    return path;
                var avoidMore = avoid != null
                    ? new HashSet<RailTrack>(avoid)
                    : new HashSet<RailTrack>();
                foreach (var c in conflicted)
                    avoidMore.Add(c);
                var alt = Search(start, destId, loco, occupancy, arriveEnteredIn, facing,
                    avoidMore, reversalPenalty, foreign, out var altCost);
                if (alt != null && Conflicts(alt).Count == 0 && altCost <= cost + CleanDetourMax)
                    return alt;
                return path;
            }

            var plain = Try(null);

            // Asked for outright: honour it whatever the detour, and only fall back to
            // the ordinary route when no such approach exists at all.
            if (arrive == ArriveMode.Shove)
                return Chosen(Try(true) ?? plain, occupancy);
            if (arrive == ArriveMode.Pull)
                return Chosen(Try(false) ?? plain, occupancy);

            // Auto: shoving is worth a runaround only to put wagons behind stock that
            // is already there, and only if the detour is modest.
            bool wagons = (loco.trainset?.cars?.Count ?? 1) > 1;
            if (!wagons || plain == null || !occupancy.Any(plain[plain.Count - 1]))
                return Chosen(plain, occupancy);
            var shove = Try(true);
            return Chosen(
                shove != null && Length(shove) <= Length(plain) + MaxPushDetour ? shove : plain,
                occupancy);
        }

        /// <summary>Records whether the path being handed back has to run THROUGH stock
        /// that is standing in the way. The destination itself does not count — arriving
        /// alongside cars already there is the point of most of these moves; only having
        /// to drive through them is a problem.</summary>
        private static List<RailTrack>? Chosen(List<RailTrack>? path, Occupancy occupancy)
        {
            if (path == null)
                return null;
            for (int i = 1; i < path.Count - 1; i++)
            {
                // Only stock that will still be there counts. A moving train on the
                // road is why we wait, not a reason to refuse the move.
                if (occupancy.Parked(path[i]))
                {
                    BlockedRoute = true;
                    break;
                }
            }
            return path;
        }

        /// <summary>
        /// Dijkstra over DIRECTED states — (track, which end we came in by) — because
        /// which way round a track is entered decides both what counts as a reversal and
        /// which stock is in the way. Cost is track length, plus a penalty per reversal.
        /// </summary>
        private static List<RailTrack>? Search(
            RailTrack start, string destId, TrainCar? loco, Occupancy? occupancy,
            bool? arriveEnteredIn, bool? requireFacing,
            HashSet<RailTrack>? avoid, double reversalPenalty,
            IReadOnlyDictionary<RailTrack, double>? foreignCost, out double totalCost)
        {
            totalCost = 0;
            // How long the train is, for the turntable fit test. No loco (a bare
            // debug route) plans as a light engine would.
            double trainLength = 0;
            if (loco?.trainset?.cars != null)
                foreach (var c in loco.trainset.cars)
                    trainLength += c.InterCouplerDistance;

            // The stretch of the starting track our own consist covers. Leaving by an
            // end means running the gap between our own stock and that end, so only
            // foreign cars inside that gap block the move.
            double startSpan = start.GetKinkedPointSet().span;
            double ownLo = 0, ownHi = startSpan;
            if (loco != null)
            {
                double lo = double.MaxValue, hi = double.MinValue;
                foreach (var c in loco.trainset.cars)
                    foreach (var b in c.Bogies)
                        if (b?.track == start)
                        {
                            lo = System.Math.Min(lo, b.traveller.Span);
                            hi = System.Math.Max(hi, b.traveller.Span);
                        }
                if (lo <= hi)
                {
                    ownLo = lo;
                    ownHi = hi;
                }
            }

            // Two seeds: leaving the start track by its out end (entered "in"), or by
            // its in end (entered "out"). Both cost nothing to be in.
            //
            // The third element of a node says "this is where we started", and it is
            // what makes a runaround expressible: standing on the destination and
            // arriving back on it are then DIFFERENT nodes, so marking the seed visited
            // no longer closes the door on coming back to that same end later.
            // Are the wagons ahead of the loco or behind it? Leaving by the low end
            // with the loco at the high end of the consist means the wagons lead: a
            // shove. Every reversal thereafter swaps it.
            double locoSpan = ownLo;
            if (loco != null)
            {
                var lb = loco.Bogies;
                if (lb != null && lb.Length >= 2 && lb[1] != null)
                    locoSpan = lb[1].traveller.Span;
            }
            double consistMid = (ownLo + ownHi) / 2.0;
            bool pushLeavingLow = locoSpan >= consistMid;   // wagons below us
            bool pushLeavingHigh = locoSpan <= consistMid;  // wagons above us

            // Seed (start, enteredIn: true) leaves by the HIGH end, (…, false) by the low.
            var seeds = new[]
            {
                (start, true, true, pushLeavingHigh),
                (start, false, true, pushLeavingLow),
            };
            var dist = new Dictionary<(RailTrack, bool, bool, bool), double>();
            var prev = new Dictionary<(RailTrack, bool, bool, bool), (RailTrack, bool, bool, bool)?>();
            var queue = new List<(double d, RailTrack t, bool enteredIn, bool seed, bool pushing)>();
            foreach (var seed in seeds)
            {
                dist[seed] = 0;
                prev[seed] = null;
                queue.Add((0, seed.Item1, seed.Item2, true, seed.Item4));
            }
            var visited = new HashSet<(RailTrack, bool, bool, bool)>();
            (RailTrack, bool, bool, bool)? goal = null;

            while (queue.Count > 0)
            {
                // Linear extract-min: the network is only a couple thousand tracks,
                // so a heap isn't worth the complexity.
                int min = 0;
                for (int i = 1; i < queue.Count; i++)
                    if (queue[i].d < queue[min].d)
                        min = i;
                var (d, track, enteredIn, isSeed, pushing) = queue[min];
                queue.RemoveAt(min);
                var node = (track, enteredIn, isSeed, pushing);
                if (!visited.Add(node))
                    continue;
                bool endOk = arriveEnteredIn == null
                    || (enteredIn == arriveEnteredIn.Value && !isSeed);
                if (Matches(track, destId) && endOk
                    && (requireFacing == null || pushing == requireFacing.Value))
                {
                    // Arriving is always allowed even if the destination is occupied —
                    // coupling onto the train standing there is the whole point of most
                    // of these moves. Only driving THROUGH stock is forbidden.
                    goal = node;
                    break;
                }

                // A seed is the start track with the loco standing on it: it has not
                // been driven end to end, so only the gap to the chosen end is run, and
                // its natural exit is the far end from the one it counts as entered by.
                bool seed = isSeed;
                double span = track.GetKinkedPointSet().span;

                foreach (var exitIn in new[] { true, false })
                {
                    double cost;
                    if (seed)
                    {
                        if (exitIn == enteredIn)
                            continue; // the other seed covers this direction
                        if (occupancy != null &&
                            (exitIn ? occupancy.AnyBetween(track, -1, ownLo)
                                    : occupancy.AnyBetween(track, ownHi, span + 1)))
                            continue; // stock stands between us and that end
                        cost = exitIn ? ownLo : span - ownHi;
                    }
                    else
                    {
                        // Standing stock shuts the road; a train under way only makes it
                        // expensive, because it will have gone by the time we get there.
                        double busy = 0;
                        if (occupancy != null && occupancy.Any(track))
                        {
                            if (occupancy.Parked(track))
                                continue;
                            busy = BusyTrackPenalty;
                        }
                        // Another autopilot's claimed or planned rail: dearer to cross,
                        // so a new plan bends around live traffic where a sane detour
                        // exists, instead of meeting it nose to nose and queueing.
                        if (foreignCost != null && foreignCost.TryGetValue(track, out var fc))
                            busy += fc;
                        // Leaving by the end we came in = a reversal: no ground covered,
                        // but a stop and a change of ends.
                        cost = (exitIn == enteredIn ? reversalPenalty : span) + busy;
                    }

                    // Leaving by the end we came in turns the train round, so the end
                    // that leads swaps with it.
                    bool nextPushing = !isSeed && exitIn == enteredIn ? !pushing : pushing;
                    foreach (var hop in Neighbors(track, exitIn, trainLength))
                    {
                        if (avoid != null && avoid.Contains(hop.track))
                            continue; // known blocked — a switch there will not throw
                        var next = (hop.track, hop.enteredIn, false, nextPushing);
                        if (visited.Contains(next))
                            continue;
                        double nd = d + cost + hop.extra;
                        if (!dist.TryGetValue(next, out var old) || nd < old)
                        {
                            dist[next] = nd;
                            prev[next] = node;
                            queue.Add((nd, hop.track, hop.enteredIn, false, nextPushing));
                        }
                    }
                }
            }

            if (goal == null)
                return null;
            totalCost = dist[goal.Value];
            var path = new List<RailTrack>();
            for ((RailTrack, bool, bool, bool)? c = goal; c != null; c = prev[c.Value])
                path.Add(c.Value.Item1);
            path.Reverse();
            return path;
        }

        /// <summary>The track the loco with the given car GUID is sitting on — or the
        /// player's own loco when no GUID is given, since a route can also be asked for
        /// from the cab.</summary>
        private static RailTrack? CurrentLocoTrack(string? locoGuid)
        {
            var car = string.IsNullOrEmpty(locoGuid)
                ? PlayerManager.Car
                : TrainCarRegistry.Instance?.GetTrainCarByCarGuid(locoGuid);
            if (car == null || !car.IsLoco)
                return null;
            var bogies = car.Bogies;
            if (bogies == null || bogies.Length < 2)
                return null;
            return bogies[1].track;
        }

        private static RailTrack? FindTrack(string id)
        {
            return Component.FindObjectsOfType<RailTrack>().FirstOrDefault(t => Matches(t, id));
        }

        /// <summary>
        /// Compute a route to destId. Start is the track named by fromId when given (so
        /// it can be tested without a loco), else the track under the loco named by
        /// locoGuid, else the player's own loco.
        /// </summary>
        public static string GetRouteJson(string destId, string? fromId, string? locoGuid = null)
        {
            return Updater.RunOnMainThread(() =>
            {
                if (!WorldStreamingInit.Instance || !WorldStreamingInit.IsLoaded)
                    return new JObject { ["error"] = "world not loaded" }.ToString(Formatting.None);

                var start = fromId != null ? FindTrack(fromId) : CurrentLocoTrack(locoGuid);
                if (start == null)
                    return new JObject { ["error"] = fromId != null ? "start track not found" : "no loco" }
                        .ToString(Formatting.None);

                var path = FindPath(start, destId);
                if (path == null)
                    return new JObject { ["error"] = "no route" }.ToString(Formatting.None);

                double length = path.Sum(t => t.GetKinkedPointSet().span);
                return new JObject
                {
                    ["from"] = start.LogicTrack().ID.ToString(),
                    ["to"] = destId,
                    ["length"] = System.Math.Round(length),
                    ["tracks"] = new JArray(path.Select(t => t.LogicTrack().ID.ToString())),
                }.ToString(Formatting.None);
            }).Result;
        }
    }
}
