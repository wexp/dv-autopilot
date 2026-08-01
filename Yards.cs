using DV.Utils;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// The yards, the tracks in each, and the running lines that serve them.
    ///
    /// This exists because the two halves of the autopilot ask different questions.
    /// A shunt is "which track in THIS yard", so it needs the tracks grouped by
    /// yard. A road move is "take it to that yard", which has no single answer in
    /// the game's data: yard tracks are all sidings, and the line a train actually
    /// arrives on is a generic <c>#Y-#S…#T</c> track that belongs to no yard at all.
    /// So the running lines are found by walking outward from the yard's own tracks
    /// through the frog stubs of its ladders until a long piece of plain line turns
    /// up. A yard fed from more than one direction has more than one such line —
    /// SM has one along its B tracks and another along its C tracks — and both are
    /// offered rather than guessed between, tagged with the sub-yards they serve.
    /// </summary>
    public static class Yards
    {
        // A piece of plain line this long is a running line, not a bit of ladder.
        // Frog stubs and the connectors between switches are short; the roads that
        // pass a yard are hundreds of metres.
        private const double MainLineSpan = 60.0;
        // Give up looking this far out from the yard's own rails, m. Without it the
        // walk would leave a yard that has no line of its own and adopt the next
        // one it met, miles away.
        private const double MainLineReach = 500.0;
        // Under this a road is a stub that scraped past the length test, not a way into
        // the yard. Deliberately an absolute figure and not a share of the yard's best:
        // CS sits beside a 3.7 km stretch of open line, and judging its other two
        // approaches against that number threw both of them away.
        private const double MainLineFloor = 80.0;
        // Every Nth point of a track is enough to say which yard a loco is standing
        // in — the yards are hundreds of metres apart.
        private const int SampleStride = 4;

        public sealed class YardInfo
        {
            public readonly string id;
            /// <summary>Named tracks in this yard, e.g. SM-B3I.</summary>
            public readonly SortedSet<string> tracks = new SortedSet<string>(StringComparer.Ordinal);
            /// <summary>Running line track id -> the sub-yard letters it serves. A yard
            /// approached from two directions has an entry for each.</summary>
            public readonly Dictionary<string, SortedSet<string>> mains =
                new Dictionary<string, SortedSet<string>>(StringComparer.Ordinal);
            /// <summary>Long tracks the walk only reached by crossing another main —
            /// the parallel road one crossover further out (SM-B's true running
            /// line sits behind its arrival road like this). Candidates only:
            /// PruneMains promotes at most one comparable one per side into mains
            /// and this set is cleared.</summary>
            public readonly Dictionary<string, SortedSet<string>> beyond =
                new Dictionary<string, SortedSet<string>>(StringComparer.Ordinal);

            public YardInfo(string id) => this.id = id;
        }

        private static Dictionary<string, YardInfo>? cache;
        // Sample points of every named track, in point-set coordinates. The world
        // shifts under the player, so WorldMover.currentMove is added at query time
        // rather than baked in — a cached absolute position goes stale on the first
        // shift and would then name the wrong yard.
        private static List<(string yard, Vector3 local)>? samples;

        /// <summary>Tracks either side of this one, whichever end they hang off.</summary>
        private static IEnumerable<RailTrack> Adjacent(RailTrack track)
        {
            var ins = track.GetAllInBranches();
            if (ins != null)
            {
                foreach (var b in ins)
                    if (b?.track != null)
                        yield return b.track;
            }
            var outs = track.GetAllOutBranches();
            if (outs != null)
            {
                foreach (var b in outs)
                    if (b?.track != null)
                        yield return b.track;
            }
        }

        /// <summary>
        /// Walk out of one yard track through generic rail until a running line is
        /// reached, and credit that line with serving this track's sub-yard. The walk
        /// stops at the first long piece of plain line rather than carrying on along
        /// it: past that point it is the same road, and following it would eventually
        /// arrive at the neighbouring yard's.
        ///
        /// Length, not junction structure, decides where the yard ends — which looks
        /// backwards, since a junction is exactly the thing that says "the yard hangs
        /// off here". It was tried. A DV junction has one stem and two branches, and
        /// the stem does NOT reliably face away from the yard: trailing points put the
        /// yard track on the stem side, so a walk that follows branch-to-stem until it
        /// can only diverge stops on a 25 m stub at MFMB and a 30 m one at CW. Where
        /// the points do all face out, the same walk runs past the yard entirely and
        /// lands on the through road — one answer for the whole of HB (2.9 km serving
        /// E, F and G alike) instead of the separate approaches that are the reason
        /// for asking. It collapsed SM to a single line as well, when SM plainly has
        /// two. Length is the cruder test and the one that matches the railway.
        /// </summary>
        private static void WalkToMainLine(RailTrack from, YardInfo yard, string sub, bool collectBeyond)
        {
            var seen = new HashSet<RailTrack> { from };
            var queue = new Queue<(RailTrack track, double dist)>();
            foreach (var n in Adjacent(from))
                if (seen.Add(n))
                    queue.Enqueue((n, 0));

            while (queue.Count > 0)
            {
                var (track, dist) = queue.Dequeue();
                var id = track.LogicTrack()?.ID;
                if (id == null || !id.IsGeneric())
                    continue; // somebody's siding, not a road out

                double span = track.GetKinkedPointSet().span;
                if (span >= MainLineSpan)
                {
                    string key = id.ToString();
                    mainSpans[key] = span;
                    mainTracks[key] = track;
                    if (!collectBeyond)
                    {
                        // First pass: the road the walk meets FIRST on each branch,
                        // and the branch stops there. This is the proven baseline —
                        // whichever additional roads exist further out must never
                        // displace these.
                        if (!yard.mains.TryGetValue(key, out var subs))
                            yard.mains[key] = subs = new SortedSet<string>(StringComparer.Ordinal);
                        if (!string.IsNullOrEmpty(sub))
                            subs.Add(sub);
                        continue;
                    }
                    // Second pass: keep walking THROUGH mains and remember every
                    // other long road in reach — the parallel road one crossover
                    // out (SM-B: the signs' running line sits behind the 490 m
                    // arrival road the first pass stops at). Candidates only;
                    // PruneMains promotes at most one credible sibling per side.
                    if (!yard.mains.ContainsKey(key))
                    {
                        if (!yard.beyond.TryGetValue(key, out var subs))
                            yard.beyond[key] = subs = new SortedSet<string>(StringComparer.Ordinal);
                        if (!string.IsNullOrEmpty(sub))
                            subs.Add(sub);
                    }
                }

                if (dist + span > MainLineReach)
                    continue;
                foreach (var n in Adjacent(track))
                    if (seen.Add(n))
                        queue.Enqueue((n, dist + span));
            }
        }

        /// <summary>
        /// Cut the raw walk down to the roads a dispatcher would actually name. The
        /// walk finds every long-ish piece of plain line a ladder touches, which at a
        /// yard like HB is sixteen of them — mostly consecutive segments of the same
        /// road, split where a yard entry cuts in. What matters is one line per SIDE
        /// of the yard: SM is served from its A side and its B side, and those are two
        /// genuinely different arrivals. So each sub-yard keeps its longest, and
        /// anything far shorter than the yard's best road is a stub that got through
        /// the length test rather than a way in.
        /// </summary>
        private static void PruneMains(YardInfo yard)
        {
            if (yard.mains.Count == 0)
                return;
            var keep = new HashSet<string>(StringComparer.Ordinal);
            var promote = new Dictionary<string, SortedSet<string>>(StringComparer.Ordinal);
            foreach (var sub in yard.mains.Values.SelectMany(s => s).Distinct().ToList())
            {
                string? longest = null;
                double longestSpan = -1;
                foreach (var kv in yard.mains)
                {
                    if (!kv.Value.Contains(sub))
                        continue;
                    double span = SpanOf(kv.Key);
                    if (span > longestSpan)
                    {
                        longestSpan = span;
                        longest = kv.Key;
                    }
                }
                if (longest == null || longestSpan < MainLineFloor)
                    continue;
                keep.Add(longest);

                // The parallel road one crossover further out, if this side has
                // one worth naming: COMPARABLE in length to the near road (half
                // to double — a 4 km open line behind a 160 m approach is the
                // road to somewhere else, not a second way into this yard) and
                // meeting it at no junction (end-to-end contact would make it
                // the same road continuing, not a sibling). SM-B: 490 m arrival
                // road plus the 462 m running line between J-149 and J-150.
                string? sibling = null;
                double siblingSpan = -1;
                // Both pools: a sibling may have been found directly too and merely
                // lost the "longest" contest (SM-B: the 462 m running line against
                // the 490 m arrival road — 28 m decided it).
                foreach (var pool in new[] { yard.mains, yard.beyond })
                foreach (var kv in pool)
                {
                    if (!kv.Value.Contains(sub) || keep.Contains(kv.Key) || kv.Key == longest)
                        continue;
                    double span = SpanOf(kv.Key);
                    if (span < MainLineFloor || span < longestSpan * 0.5 || span > longestSpan * 2.0)
                        continue;
                    if (SharesEndJunction(longest, kv.Key))
                        continue;
                    if (span > siblingSpan)
                    {
                        siblingSpan = span;
                        sibling = kv.Key;
                    }
                }
                if (sibling != null)
                {
                    if (!promote.TryGetValue(sibling, out var subs))
                        promote[sibling] = subs = new SortedSet<string>(StringComparer.Ordinal);
                    subs.Add(sub);
                }
            }
            foreach (var kv in promote)
            {
                keep.Add(kv.Key);
                if (!yard.mains.TryGetValue(kv.Key, out var subs))
                    yard.mains[kv.Key] = subs = kv.Value;
                else
                    foreach (var s in kv.Value)
                        subs.Add(s);
            }
            yard.beyond.Clear();
            // Never leave a yard with no way in at all: if every side fell under the
            // floor, the best road is still the answer.
            if (keep.Count == 0)
                keep.Add(yard.mains.Keys.OrderByDescending(SpanOf).First());

            foreach (var key in yard.mains.Keys.ToList())
                if (!keep.Contains(key))
                    yard.mains.Remove(key);
        }

        /// <summary>
        /// Hand-pinned running lines, by the junction numbers at their ends (the
        /// map's numbering). Discovery keeps getting individual yards wrong in
        /// individual ways — SM's parallel B road, FRS naming an arrival loop —
        /// and where the dispatcher has simply TOLD us the road, that wins: the
        /// yard's discovered mains are REPLACED by the pinned track(s).
        /// </summary>
        // Each pin: "SUB=SPEC" or just "SPEC". SPEC is either "a-b" — the generic
        // road between those two junction numbers — or a literal generic track id,
        // for a road with a junction-less end (FRS-A dead-ends at the turntable).
        // A SUB names the side of the yard the road serves; without one the road
        // carries every side discovery found.
        private static readonly Dictionary<string, string[]> PinnedMains =
            new Dictionary<string, string[]>(StringComparer.Ordinal)
            {
                ["FRS"] = new[] { "B=204-209", "A=#Y-#S93#T" },
            };

        private static void ApplyPinnedMains(YardInfo yard)
        {
            if (!PinnedMains.TryGetValue(yard.id, out var pins))
                return;
            var allSubs = new SortedSet<string>(
                yard.mains.Values.SelectMany(v => v), StringComparer.Ordinal);
            var replacement = new Dictionary<string, SortedSet<string>>(StringComparer.Ordinal);
            foreach (var pin in pins)
            {
                var parts = pin.Split('=');
                string? sub = parts.Length == 2 ? parts[0] : null;
                string spec = parts[parts.Length - 1];

                RailTrack? track = null;
                var dash = spec.Split('-');
                if (dash.Length == 2
                    && int.TryParse(dash[0], out var a) && int.TryParse(dash[1], out var b))
                    track = TrackByEndJunctions(a, b);
                else
                    track = TrackByGenericId(spec);
                if (track == null)
                    continue; // pin names a road that can't be found — keep discovery
                var key = track.LogicTrack().ID.ToString();
                mainTracks[key] = track;
                mainSpans[key] = track.GetKinkedPointSet().span;
                replacement[key] = sub != null
                    ? new SortedSet<string>(StringComparer.Ordinal) { sub }
                    : new SortedSet<string>(allSubs, StringComparer.Ordinal);
            }
            if (replacement.Count == 0)
                return;
            yard.mains.Clear();
            foreach (var kv in replacement)
                yard.mains[kv.Key] = kv.Value;
        }

        private static RailTrack? TrackByGenericId(string id)
        {
            foreach (var track in Component.FindObjectsOfType<RailTrack>())
            {
                var tid = track.LogicTrack()?.ID;
                if (tid != null && tid.IsGeneric() && tid.ToString() == id)
                    return track;
            }
            return null;
        }

        /// <summary>The generic (unnamed) track whose two ends sit at exactly these
        /// junction numbers, or null.</summary>
        private static RailTrack? TrackByEndJunctions(int a, int b)
        {
            foreach (var track in Component.FindObjectsOfType<RailTrack>())
            {
                var id = track.LogicTrack()?.ID;
                if (id == null || !id.IsGeneric())
                    continue;
                var e1 = Autopilot.JunctionIndexAtEnd(track, true);
                var e2 = Autopilot.JunctionIndexAtEnd(track, false);
                if ((e1 == a && e2 == b) || (e1 == b && e2 == a))
                    return track;
            }
            return null;
        }

        /// <summary>Do two discovered mains meet end to end? Consecutive segments of
        /// one road share the junction where a yard entry split them; two genuinely
        /// parallel roads do not. Ends without a junction (plain rail joins,
        /// turntable spurs) never match.</summary>
        private static bool SharesEndJunction(string a, string b)
        {
            if (!mainTracks.TryGetValue(a, out var ta) || !mainTracks.TryGetValue(b, out var tb))
                return false;
            var endsA = new[] {
                Autopilot.JunctionIndexAtEnd(ta, true), Autopilot.JunctionIndexAtEnd(ta, false) };
            var endsB = new[] {
                Autopilot.JunctionIndexAtEnd(tb, true), Autopilot.JunctionIndexAtEnd(tb, false) };
            foreach (var ea in endsA)
                foreach (var eb in endsB)
                    if (ea != null && ea == eb)
                        return true;
            return false;
        }

        /// <summary>Every yard with its tracks and running lines. Built once — the rail
        /// network does not change while the game is running.</summary>
        public static Dictionary<string, YardInfo> All()
        {
            if (cache != null)
                return cache;

            var byId = new Dictionary<string, YardInfo>(StringComparer.Ordinal);
            var named = new List<(RailTrack track, string yard, string sub)>();
            var points = new List<(string yard, Vector3 local)>();

            foreach (var track in Component.FindObjectsOfType<RailTrack>())
            {
                var id = track.LogicTrack()?.ID;
                if (id == null || id.IsGeneric() || string.IsNullOrEmpty(id.yardId))
                    continue;
                if (!byId.TryGetValue(id.yardId, out var info))
                    byId[id.yardId] = info = new YardInfo(id.yardId);
                info.tracks.Add(id.ToString());
                named.Add((track, id.yardId, id.SignIDSubYardPart ?? ""));

                var pts = track.GetKinkedPointSet().points;
                for (int i = 0; i < pts.Length; i += SampleStride)
                    points.Add((id.yardId, (Vector3)pts[i].position));
            }

            foreach (var n in named)
                WalkToMainLine(n.track, byId[n.yard], n.sub, collectBeyond: false);
            foreach (var n in named)
                WalkToMainLine(n.track, byId[n.yard], n.sub, collectBeyond: true);
            foreach (var yard in byId.Values)
                PruneMains(yard);
            foreach (var yard in byId.Values)
                ApplyPinnedMains(yard);

            samples = points;
            cache = byId;
            return byId;
        }

        /// <summary>The yard a world position sits in — nearest named rail wins. Used
        /// for a loco standing out on the main, where the track it is on belongs to no
        /// yard and so cannot answer for itself.</summary>
        private static string? NearestYard(Vector3 position)
        {
            All();
            if (samples == null || samples.Count == 0)
                return null;
            Vector3 local = position - WorldMover.currentMove;
            string? best = null;
            float bestSq = float.MaxValue;
            foreach (var (yard, point) in samples)
            {
                float sq = (point - local).sqrMagnitude;
                if (sq < bestSq)
                {
                    bestSq = sq;
                    best = yard;
                }
            }
            return best;
        }

        /// <summary>Which yard this loco is in: the yard of the track under it, or —
        /// out on the line — the nearest one. Main thread only.</summary>
        public static string? YardOf(string? locoGuid)
        {
            var car = string.IsNullOrEmpty(locoGuid)
                ? PlayerManager.Car
                : SingletonBehaviour<TrainCarRegistry>.Instance?.GetTrainCarByCarGuid(locoGuid);
            if (car == null)
                return null;
            var bogies = car.Bogies;
            var id = bogies != null && bogies.Length >= 2 ? bogies[1].track?.LogicTrack()?.ID : null;
            if (id != null && !id.IsGeneric() && !string.IsNullOrEmpty(id.yardId))
                return id.yardId;
            return NearestYard(car.transform.position);
        }

        /// <summary>
        /// The running line to aim a road move at: whichever of the yard's lines is the
        /// shortest run from where the train stands. A yard served from both ends has
        /// two, and taking the near one saves a lap of the valley — but the dispatcher
        /// can name one instead, which is why they are all published.
        /// </summary>
        public static string? MainLineFor(
            string? yardId, RailTrack start, TrainCar loco, double reversalPenalty, out string error)
        {
            error = "";
            if (string.IsNullOrEmpty(yardId))
            {
                error = "no yard given";
                return null;
            }
            if (!All().TryGetValue(yardId!, out var yard))
            {
                error = $"no yard called {yardId}";
                return null;
            }
            if (yard.mains.Count == 0)
            {
                error = $"{yardId} has no running line of its own — name a track instead";
                return null;
            }

            string? best = null;
            double bestLength = double.MaxValue;
            foreach (var candidate in yard.mains.Keys)
            {
                var path = Routing.FindPath(
                    start, candidate, loco, null, ArriveMode.Auto, null, reversalPenalty);
                if (path == null)
                    continue;
                double length = path.Sum(t => t.GetKinkedPointSet().span);
                if (length < bestLength)
                {
                    bestLength = length;
                    best = candidate;
                }
            }
            if (best == null)
                error = $"no route to the running line at {yardId}";
            return best;
        }

        // Length of every track that has turned up as a running line, so the console
        // can show how much road it is being sent to without asking again.
        private static readonly Dictionary<string, double> mainSpans =
            new Dictionary<string, double>(StringComparer.Ordinal);
        // The running lines themselves, so their end junctions can be named. A
        // dispatcher identifies a road by the switches at its ends far more readily
        // than by a generic #Y-#S…#T id, which is noise.
        private static readonly Dictionary<string, RailTrack> mainTracks =
            new Dictionary<string, RailTrack>(StringComparer.Ordinal);

        private static double SpanOf(string trackId) =>
            mainSpans.TryGetValue(trackId, out var span) ? span : 0;

        /// <summary>Is this track one of the running lines serving that yard? A shunt
        /// may be sent onto its own yard's road — pulling a cut out onto the main is
        /// ordinary yard work — even though the road belongs to no yard itself.</summary>
        public static bool IsMainLineOf(string? yardId, string trackId)
        {
            return !string.IsNullOrEmpty(yardId)
                && All().TryGetValue(yardId!, out var yard)
                && yard.mains.ContainsKey(trackId);
        }

        /// <summary>Yard of a track id, for checking a shunt stays where it was sent.</summary>
        public static string? YardOfTrack(RailTrack track)
        {
            var id = track.LogicTrack()?.ID;
            return id == null || id.IsGeneric() || string.IsNullOrEmpty(id.yardId) ? null : id.yardId;
        }

        /// <summary>
        /// The console's picker data: every yard with its tracks and its running lines,
        /// plus which yard the given loco is standing in so a shunt panel can open on
        /// the right one without the dispatcher having to say.
        /// </summary>
        public static string GetYardsJson(string? locoGuid)
        {
            return Updater.RunOnMainThread(() =>
            {
                if (!WorldStreamingInit.Instance || !WorldStreamingInit.IsLoaded)
                    return new JObject { ["error"] = "world not loaded" }.ToString(Formatting.None);

                var yards = new JObject();
                foreach (var kv in All().OrderBy(k => k.Key, StringComparer.Ordinal))
                {
                    var mains = new JArray();
                    foreach (var m in kv.Value.mains.OrderBy(k => k.Key, StringComparer.Ordinal))
                    {
                        // The switches at each end: how a road is actually named in the
                        // yard — "the one between J-218 and J-221".
                        int? from = null, to = null;
                        if (mainTracks.TryGetValue(m.Key, out var rail))
                        {
                            from = Autopilot.JunctionIndexAtEnd(rail, true);
                            to = Autopilot.JunctionIndexAtEnd(rail, false);
                        }
                        mains.Add(new JObject
                        {
                            ["track"] = m.Key,
                            ["serves"] = new JArray(m.Value),
                            ["length"] = Math.Round(SpanOf(m.Key)),
                            ["from"] = from == null ? JValue.CreateNull() : new JValue(from.Value),
                            ["to"] = to == null ? JValue.CreateNull() : new JValue(to.Value),
                        });
                    }
                    yards[kv.Key] = new JObject
                    {
                        ["tracks"] = new JArray(kv.Value.tracks),
                        ["main"] = mains,
                    };
                }

                var here = YardOf(locoGuid);
                return new JObject
                {
                    ["yards"] = yards,
                    ["here"] = here == null ? JValue.CreateNull() : new JValue(here),
                }.ToString(Formatting.None);
            }).Result;
        }
    }
}
