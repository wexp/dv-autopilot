using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// Writes the rail network out as JSON so the routing and driving code can be
    /// run outside the game. Everything the autopilot reads about the world comes
    /// through a handful of members — track spans, branch lists, junction numbers,
    /// bogie positions — and this captures exactly those, so a simulator can stand
    /// in for the game without anyone having to guess what the graph looks like.
    ///
    /// Two files, because they age differently. The MAP is topology and geometry:
    /// it changes only when the game does, so it is worth committing. The STATE is
    /// where the switches lie and what stock is standing where, which is different
    /// every minute — that is a scenario, not a map.
    /// </summary>
    public static class MapDump
    {
        /// <summary>Bumped whenever the shape of the JSON changes, so a simulator
        /// reading an old file says so instead of misreading it.</summary>
        private const int SchemaVersion = 1;

        /// <summary>Roughly how far apart the dumped centreline points are, m. The
        /// full kinked point set is metre-scale and would run to hundreds of MB
        /// across the whole map; a schematic only needs the shape of the curve.</summary>
        private const double PolyResolution = 25.0;
        /// <summary>However long the track, never more points than this for it.</summary>
        private const int PolyMaxPoints = 48;

        /// <summary>Dump map and state into <paramref name="dir"/> (the mod's own
        /// folder when null). Returns a short JSON report — what was written, how
        /// much of it, and any track the walk could not make sense of.</summary>
        public static string Write(string? dir)
        {
            return Updater.RunOnMainThread(() =>
            {
                if (!WorldStreamingInit.Instance || !WorldStreamingInit.IsLoaded)
                    return new JObject { ["error"] = "world not loaded" }.ToString(Formatting.None);

                string target = string.IsNullOrEmpty(dir)
                    ? (Main.mod?.Path ?? ".")
                    : dir!;
                try
                {
                    Directory.CreateDirectory(target);
                    var tracks = OrderedTracks();
                    var index = new Dictionary<RailTrack, int>();
                    for (int i = 0; i < tracks.Count; i++)
                        index[tracks[i]] = i;
                    var junctions = RailTrackRegistry.Instance?.OrderedJunctions?.ToList()
                        ?? new List<Junction>();

                    var map = BuildMap(tracks, index, junctions);
                    var state = BuildState(tracks, index, junctions);

                    string mapPath = Path.Combine(target, "map.json");
                    string statePath = Path.Combine(target, "state.json");
                    File.WriteAllText(mapPath, map.ToString(Formatting.None));
                    File.WriteAllText(statePath, state.ToString(Formatting.None));

                    return new JObject
                    {
                        ["map"] = mapPath,
                        ["state"] = statePath,
                        ["tracks"] = tracks.Count,
                        ["junctions"] = junctions.Count,
                        ["turntables"] = ((JArray)map["turntables"]!).Count,
                        ["cars"] = ((JArray)state["cars"]!).Count,
                    }.ToString(Formatting.None);
                }
                catch (Exception e)
                {
                    return new JObject { ["error"] = $"{e.GetType().Name}: {e.Message}" }
                        .ToString(Formatting.None);
                }
            }).Result;
        }

        /// <summary>
        /// Every loaded track, in an order that does not depend on the order Unity
        /// happened to hand them over. Track indices are the file's only way of
        /// naming a track — most of the map is generic rail with no id at all — so
        /// an unstable order would make two dumps of the same world incomparable,
        /// and would silently renumber every reference in them.
        /// </summary>
        private static List<RailTrack> OrderedTracks()
        {
            return Component.FindObjectsOfType<RailTrack>()
                .Where(t => t != null)
                .OrderBy(t => IdOf(t), StringComparer.Ordinal)
                .ThenBy(t => Origin(t).x)
                .ThenBy(t => Origin(t).z)
                .ThenBy(t => t.GetKinkedPointSet().span)
                .ToList();
        }

        private static string IdOf(RailTrack track) =>
            track.LogicTrack()?.ID?.ToString() ?? "";

        /// <summary>The track's first centreline point, in the world's own frame.
        /// The streamed origin shift is NOT added back: positions have to mean the
        /// same thing in a dump taken at one end of the map and one taken at the
        /// other, or the two cannot be compared at all.</summary>
        private static Vector3 Origin(RailTrack track)
        {
            var points = track.GetKinkedPointSet()?.points;
            return points == null || points.Length == 0
                ? Vector3.zero
                : (Vector3)points[0].position;
        }

        private static JObject BuildMap(
            List<RailTrack> tracks, Dictionary<RailTrack, int> index, List<Junction> junctions)
        {
            var junctionIndex = new Dictionary<Junction, int>();
            for (int i = 0; i < junctions.Count; i++)
                if (junctions[i] != null)
                    junctionIndex[junctions[i]] = i;

            var trackArray = new JArray();
            foreach (var track in tracks)
                trackArray.Add(TrackJson(track, index, junctionIndex));

            var junctionArray = new JArray();
            for (int i = 0; i < junctions.Count; i++)
                junctionArray.Add(JunctionJson(junctions[i], i, index));

            return new JObject
            {
                ["schema"] = SchemaVersion,
                ["tracks"] = trackArray,
                ["junctions"] = junctionArray,
                ["turntables"] = TurntableJson(index),
            };
        }

        private static JObject TrackJson(
            RailTrack track,
            Dictionary<RailTrack, int> index,
            Dictionary<Junction, int> junctionIndex)
        {
            var id = track.LogicTrack()?.ID;
            var pointSet = track.GetKinkedPointSet();

            return new JObject
            {
                // All three forms, because Routing.Matches accepts all three and a
                // destination named one way must not miss a track named another.
                ["id"] = id?.ToString() ?? "",
                ["fullId"] = id?.FullID ?? "",
                ["displayId"] = id?.FullDisplayID ?? "",
                ["generic"] = id?.IsGeneric() ?? true,
                // The parsed pieces of the name, taken from the game rather than
                // re-split by whoever reads this. The yard grouping and the
                // sign-marker matching both key off them, and two different
                // parsers of "HB-D7L-LO" would eventually disagree.
                ["yard"] = id?.yardId ?? "",
                ["sub"] = id?.SignIDSubYardPart ?? "",
                ["part"] = id?.TrackPartOnly ?? "",
                ["span"] = Round(pointSet?.span ?? 0.0),
                ["in"] = BranchArray(track.GetAllInBranches(), index),
                ["out"] = BranchArray(track.GetAllOutBranches(), index),
                // Null for exactly the tracks that meet a frog side-on, which is why
                // the mod itself walks branches instead of trusting these. Dumped so
                // a simulator can check the walk against them rather than assume.
                ["inJunction"] = JunctionNumber(track.inJunction, junctionIndex),
                ["outJunction"] = JunctionNumber(track.outJunction, junctionIndex),
                ["poly"] = Polyline(pointSet),
            };
        }

        private static JArray BranchArray(
            IEnumerable<Junction.Branch>? branches, Dictionary<RailTrack, int> index)
        {
            var array = new JArray();
            if (branches == null)
                return array;
            foreach (var branch in branches)
            {
                if (branch?.track == null || !index.TryGetValue(branch.track, out var to))
                    continue;
                // Branch ORDER is preserved deliberately. The router's extract-min is
                // first-wins, so equal-cost ties resolve by the order neighbours were
                // enumerated in — reorder these and a simulated route can differ from
                // the game's for no reason the cost model would explain.
                array.Add(new JObject { ["t"] = to, ["first"] = branch.first });
            }
            return array;
        }

        private static int JunctionNumber(Junction? junction, Dictionary<Junction, int> index) =>
            junction != null && index.TryGetValue(junction, out var n) ? n : -1;

        private static JObject JunctionJson(
            Junction? junction, int number, Dictionary<RailTrack, int> index)
        {
            if (junction == null)
                return new JObject { ["n"] = number, ["missing"] = true };

            var outBranches = new JArray();
            if (junction.outBranches != null)
                foreach (var branch in junction.outBranches)
                    outBranches.Add(branch?.track != null && index.TryGetValue(branch.track, out var to)
                        ? new JObject { ["t"] = to, ["first"] = branch.first }
                        : new JObject { ["t"] = -1 });

            var stem = junction.inBranch;
            var json = new JObject
            {
                ["n"] = number,
                ["inBranch"] = stem?.track != null && index.TryGetValue(stem.track, out var si)
                    ? new JObject { ["t"] = si, ["first"] = stem.first }
                    : new JObject { ["t"] = -1 },
                ["outBranches"] = outBranches,
            };

            // Where the frog actually is, for drawing. Taken from the stem's own
            // geometry rather than the junction transform: the stem meets the frog
            // by definition, so this is right even where the transform sits off to
            // one side of the points.
            var at = BranchEnd(stem);
            if (at != null)
                json["pos"] = new JArray(Round(at.Value.x), Round(at.Value.y), Round(at.Value.z));
            return json;
        }

        /// <summary>The end of this branch's track that the junction sits at:
        /// first means the track's span-0 end faces the frog.</summary>
        private static Vector3? BranchEnd(Junction.Branch? branch)
        {
            var points = branch?.track?.GetKinkedPointSet()?.points;
            if (points == null || points.Length == 0)
                return null;
            return (Vector3)points[branch!.first ? 0 : points.Length - 1].position;
        }

        private static JArray TurntableJson(Dictionary<RailTrack, int> index)
        {
            var array = new JArray();
            var controllers = TurntableController.allControllers;
            if (controllers == null)
                return array;
            foreach (var controller in controllers)
            {
                var table = controller?.turntable;
                if (table?.Track == null || !index.TryGetValue(table.Track, out var bridge))
                    continue;
                var ends = new JArray();
                if (table.trackEnds != null)
                    foreach (var end in table.trackEnds)
                        if (end?.track != null && index.TryGetValue(end.track, out var stall))
                            ends.Add(new JObject { ["t"] = stall, ["isFirst"] = end.isFirst });
                array.Add(new JObject
                {
                    ["id"] = string.IsNullOrEmpty(table.uniqueID) ? $"tt-{bridge}" : table.uniqueID,
                    ["bridge"] = bridge,
                    ["bridgeSpan"] = Round(table.Track.GetKinkedPointSet()?.span ?? 0.0),
                    ["ends"] = ends,
                });
            }
            return array;
        }

        /// <summary>
        /// The scenario: switch positions and where every car is standing. Split
        /// from the map because it is true for one moment only — and because the
        /// interesting simulator runs are the ones where these are edited by hand.
        /// </summary>
        private static JObject BuildState(
            List<RailTrack> tracks, Dictionary<RailTrack, int> index, List<Junction> junctions)
        {
            // Which way every switch lies. ReversalRoom and the whole interlocking
            // read the road as it is SET, not merely as it is connected, so a map
            // without these cannot answer whether a move is possible.
            var switches = new JArray();
            for (int i = 0; i < junctions.Count; i++)
                switches.Add(junctions[i]?.selectedBranch ?? -1);

            // Per track, the branch each end is currently joined to. Redundant with
            // the switch list where a junction exists, and the only record of the
            // join where one does not (plain rail, turntable spurs).
            var joins = new JArray();
            foreach (var track in tracks)
            {
                var inBranch = track.GetInBranch();
                var outBranch = track.GetOutBranch();
                joins.Add(new JObject
                {
                    ["in"] = BranchRef(inBranch, index),
                    ["out"] = BranchRef(outBranch, index),
                });
            }

            return new JObject
            {
                ["schema"] = SchemaVersion,
                ["switches"] = switches,
                ["joins"] = joins,
                ["cars"] = CarsJson(index),
            };
        }

        private static JToken BranchRef(Junction.Branch? branch, Dictionary<RailTrack, int> index)
        {
            if (branch?.track == null || !index.TryGetValue(branch.track, out var to))
                return JValue.CreateNull();
            return new JObject { ["t"] = to, ["first"] = branch.first };
        }

        private static JArray CarsJson(Dictionary<RailTrack, int> index)
        {
            var array = new JArray();
            var registry = DV.Utils.SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
            if (registry == null)
                return array;

            // Trainsets are named by a number of our own. The seed logic cares only
            // about which cars share one, never what it is called.
            var consists = new Dictionary<Trainset, int>();
            foreach (var car in registry.Values.Where(c => c != null)
                         .OrderBy(c => c.CarGUID, StringComparer.Ordinal))
            {
                if (car.trainset != null && !consists.ContainsKey(car.trainset))
                    consists[car.trainset] = consists.Count;

                var bogies = new JArray();
                foreach (var bogie in car.Bogies)
                {
                    if (bogie?.track == null || !index.TryGetValue(bogie.track, out var on))
                        continue;
                    bogies.Add(new JObject
                    {
                        ["t"] = on,
                        ["span"] = Round(bogie.traveller.Span),
                        // Which way the car's front points along that track. The
                        // routing seed reads exactly this to decide whether the
                        // wagons lead — every version that inferred it from span
                        // comparisons was a coin flip on short tracks.
                        ["sign"] = bogie.TrackDirectionSign,
                    });
                }

                array.Add(new JObject
                {
                    ["id"] = car.ID ?? "",
                    ["guid"] = car.CarGUID ?? "",
                    ["livery"] = car.carLivery?.id ?? "",
                    ["loco"] = car.IsLoco,
                    ["length"] = Round(car.InterCouplerDistance),
                    ["speed"] = Round(car.GetForwardSpeed()),
                    ["consist"] = car.trainset != null ? consists[car.trainset] : -1,
                    // The couplers, by car id, so a simulator can rebuild the
                    // consist's ORDER — MuGroup and the wagons-lead test both walk
                    // coupler to coupler, and a bare set of cars cannot answer them.
                    ["front"] = car.frontCoupler?.coupledTo?.train?.ID ?? "",
                    ["rear"] = car.rearCoupler?.coupledTo?.train?.ID ?? "",
                    ["bogies"] = bogies,
                });
            }
            return array;
        }

        /// <summary>The centreline, thinned to something a schematic can draw.
        /// First and last points are always kept, so tracks still meet at their
        /// junctions after thinning.</summary>
        private static JArray Polyline(DV.PointSet.EquiPointSet? pointSet)
        {
            var array = new JArray();
            var points = pointSet?.points;
            if (points == null || points.Length == 0)
                return array;

            double span = pointSet!.span;
            double step = Math.Max(PolyResolution, span / PolyMaxPoints);
            double next = 0;
            for (int i = 0; i < points.Length; i++)
            {
                bool edge = i == 0 || i == points.Length - 1;
                if (!edge && points[i].span < next)
                    continue;
                next = points[i].span + step;
                var at = (Vector3)points[i].position;
                // Height is kept as well as plan position: grade is most of what
                // decides how a train is driven, and a 2D centreline cannot say
                // whether a road climbs.
                array.Add(new JArray(Round(at.x), Round(at.y), Round(at.z)));
            }
            return array;
        }

        /// <summary>Centimetre precision. Full doubles tripled the file size to
        /// record noise no route decision could ever turn on.</summary>
        private static double Round(double value) => Math.Round(value, 2);
    }
}
