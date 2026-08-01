using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// The turntables: read their state for the map, and turn them on request.
    /// In the game a turntable is rotated by a player at its lever; here the
    /// same motion is driven by a coroutine at the lever's full-deflection
    /// rate, stepping targetYRotation and calling RotateToTargetRotation just
    /// as TurntableController.FixedUpdate does — so a remote turn moves the
    /// bridge, its track connections and anything standing on it exactly like
    /// a hand-cranked one, only without the walk to the pit.
    /// </summary>
    public static class Turntables
    {

        /// <summary>Raised whenever a table's angle changes — ours or a hand
        /// crank. The UI layer (whatever it turns out to be) subscribes here;
        /// this mod pushes nothing anywhere itself.</summary>
        public static event System.Action? Changed;
        /// <summary>Full-lever rotation rate, deg/s — the game's
        /// MAX_ROTATION_SPEED_DEGREES_PER_SEC.</summary>
        private const float TurnRateDegPerSec = 12f;

        /// <summary>How often a turn in progress pushes its angle to the web
        /// clients, s. At 12 deg/s this is a ~2.4 degree step per push —
        /// smooth enough to watch on the map without spamming the long poll.</summary>
        private const float PushInterval = 0.2f;

        /// <summary>The rotate coroutine per turntable id, so a new request
        /// replaces the old turn instead of fighting it.</summary>
        private static readonly Dictionary<string, Coroutine> turning = new Dictionary<string, Coroutine>();
        /// <summary>Where each running turn is headed, for reporting.</summary>
        private static readonly Dictionary<string, float> goals = new Dictionary<string, float>();

        /// <summary>Every live turntable, via the controller list the game
        /// itself maintains (Awake adds, OnDestroy removes).</summary>
        private static List<TurntableController> All() =>
            TurntableController.allControllers.Where(c => c != null && c.turntable != null).ToList();

        /// <summary>A turntable's stable identity. uniqueID when the scene set
        /// one; otherwise its position, which never changes either.</summary>
        private static string IdOf(TurntableRailTrack tt)
        {
            if (!string.IsNullOrEmpty(tt.uniqueID))
                return tt.uniqueID;
            // Fallback for a table the scene left unnamed: its fixed world
            // position (the streamed origin shift subtracted, or the id would
            // change every time the world moved under the player).
            var at = tt.transform.position - WorldMover.currentMove;
            return $"tt-{Mathf.RoundToInt(at.x)}-{Mathf.RoundToInt(at.z)}";
        }

        private static TurntableController? Find(string id) =>
            All().FirstOrDefault(c => IdOf(c.turntable) == id);

        /// <summary>Public watcher hook for a future UI: starts publishing
        /// Changed on hand-cranked turns.</summary>
        public static void EnsureWatching() => EnsureWatcher();

        /* ------------------------------------------------------- topology -- */

        // The router and the autopilot ask about turntables from the main thread
        // only (Search runs inside RunOnMainThread), so plain dictionaries are
        // fine. Rebuilt whenever the controller list changes size (scene load).
        private static Dictionary<RailTrack, TurntableController>? bridges;
        private static Dictionary<RailTrack, TurntableController>? stalls;
        private static int indexedCount = -1;

        /// <summary>Extra cost, in metres of equivalent rail, for each virtual hop
        /// on or off a turntable bridge — the stop and the wind of the table.</summary>
        public const double CrossPenalty = 200.0;
        /// <summary>Clearance the consist must leave to the bridge ends, m.</summary>
        private const double BridgeFitMargin = 1.5;

        private static void EnsureIndex()
        {
            var all = All();
            if (bridges != null && indexedCount == all.Count)
                return;
            bridges = new Dictionary<RailTrack, TurntableController>();
            stalls = new Dictionary<RailTrack, TurntableController>();
            foreach (var controller in all)
            {
                var tt = controller.turntable;
                if (tt.Track != null)
                    bridges[tt.Track] = controller;
                foreach (var end in tt.trackEnds)
                    if (end?.track != null)
                        stalls[end.track] = controller;
            }
            indexedCount = all.Count;
        }

        /// <summary>The turntable whose BRIDGE this track is, or null.</summary>
        public static TurntableRailTrack? AsBridge(RailTrack? track)
        {
            if (track == null)
                return null;
            EnsureIndex();
            return bridges!.TryGetValue(track, out var c) ? c.turntable : null;
        }

        /// <summary>
        /// True when this train is anywhere near a turntable: a car standing on a
        /// bridge, or a bridge within <paramref name="zone"/> metres along the
        /// connected rail ahead. The autopilot crosses at walking pace — a train
        /// arriving at yard speed cannot stop on a 20 m bridge, and one arriving
        /// at an unaligned table needs to already be crawling when the rail-end
        /// guard brings it to a stand.
        /// </summary>
        public static bool NearBridge(
            TrainCar loco, RailTrack start, double startSpan, bool towardOut, double zone)
        {
            EnsureIndex();
            if (bridges!.Count == 0)
                return false;

            var cars = loco?.trainset?.cars;
            if (cars != null)
                foreach (var c in cars)
                    foreach (var b in c.Bogies)
                        if (b?.track != null && bridges.ContainsKey(b.track))
                            return true;

            double cum = 0;
            RailTrack? t = start;
            double refSpan = startSpan;
            bool outward = towardOut;
            var seen = new HashSet<RailTrack>();
            for (int step = 0; step < 12 && t != null && cum < zone && seen.Add(t); step++)
            {
                cum += outward ? t.GetKinkedPointSet().span - refSpan : refSpan;
                var branch = outward ? t.GetOutBranch() : t.GetInBranch();
                if (branch?.track == null)
                    return false;
                t = branch.track;
                if (bridges.ContainsKey(t))
                    return cum <= zone;
                outward = branch.first;
                refSpan = branch.first ? 0.0 : t.GetKinkedPointSet().span;
            }
            return false;
        }

        /// <summary>
        /// The edges the rail graph does not show: a turntable bridge reaches every
        /// one of its stall tracks after a wind of the table, and a stall track
        /// reaches the bridge the same way. Only offered when the consist fits the
        /// bridge — a train longer than the bridge can never use the table at all.
        /// Each virtual hop carries CrossPenalty so the router prefers plain rail
        /// where plain rail exists.
        /// </summary>
        public static IEnumerable<(RailTrack track, bool enteredIn, double extra)> VirtualNeighbors(
            RailTrack track, bool exitIn, double trainLength)
        {
            EnsureIndex();
            if (bridges!.TryGetValue(track, out var asBridge))
            {
                var tt = asBridge.turntable;
                if (trainLength > tt.Track.GetKinkedPointSet().span - BridgeFitMargin)
                    yield break;
                // Either end of the bridge can be brought to any stall, so the exit
                // end does not constrain the reachable set.
                foreach (var end in tt.trackEnds)
                    if (end?.track != null)
                        yield return (end.track, end.isFirst, CrossPenalty);
                yield break;
            }
            if (stalls!.TryGetValue(track, out var viaStall))
            {
                var tt = viaStall.turntable;
                if (tt.Track == null
                    || trainLength > tt.Track.GetKinkedPointSet().span - BridgeFitMargin)
                    yield break;
                var end = tt.GetTrackEnd(track);
                // Only the pit-facing end of a stall leads to the bridge. isFirst
                // means the stall's IN end faces the pit, and leaving by the in
                // end is exitIn == true.
                if (end != null && exitIn == end.isFirst)
                    yield return (tt.Track, true, CrossPenalty);
            }
        }

        /// <summary>
        /// When the connected rail ahead dead-ends at a turntable pit (the bridge
        /// is wound elsewhere), start winding the bridge round to that stall and
        /// return true. A pull that has run out of room at a table can then buy
        /// the bridge's length instead of standing at the edge for ever. False
        /// when the rail ahead ends anywhere else — a wall, a buffer, nothing to
        /// be done about it here.
        /// </summary>
        public static bool RequestBridgeAhead(
            RailTrack start, double startSpan, bool towardOut, double maxDist)
        {
            EnsureIndex();
            if (stalls!.Count == 0)
                return false;

            double cum = 0;
            RailTrack? t = start;
            double refSpan = startSpan;
            bool outward = towardOut;
            var seen = new HashSet<RailTrack>();
            for (int step = 0; step < 30 && t != null && cum < maxDist && seen.Add(t); step++)
            {
                cum += outward ? t.GetKinkedPointSet().span - refSpan : refSpan;
                var branch = outward ? t.GetOutBranch() : t.GetInBranch();
                if (branch?.track != null)
                {
                    t = branch.track;
                    outward = branch.first;
                    refSpan = branch.first ? 0.0 : t.GetKinkedPointSet().span;
                    continue;
                }
                // Dead end. Only worth a wind when it is the PIT side of a stall
                // — the far end of a roundhouse road is just a wall.
                if (!stalls.TryGetValue(t, out var controller))
                    return false;
                var tt = controller.turntable;
                var end = tt?.GetTrackEnd(t);
                if (tt == null || end == null || outward == end.isFirst)
                    return false;
                RotateBridgeTo(tt, t);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Wind the table so this stall connects, by the SHORTER of the two ways
        /// round (either bridge end may meet the stall). Fire and forget — the
        /// caller polls the rail connection to know when it has landed.
        /// </summary>
        public static void RotateBridgeTo(TurntableRailTrack tt, RailTrack stall)
        {
            var end = tt.GetTrackEnd(stall);
            if (end == null)
                return;
            float current = TurntableRailTrack.AngleRange0To360(tt.currentYRotation);
            float a = TurntableRailTrack.AngleRange0To360(end.angle);
            float b = TurntableRailTrack.AngleRange0To360(end.angle + 180f);
            float goal = Mathf.Abs(TurntableRailTrack.AngleRangeNeg180To180(a - current))
                <= Mathf.Abs(TurntableRailTrack.AngleRangeNeg180To180(b - current)) ? a : b;
            Rotate(IdOf(tt), goal);
        }

        /* ---------------------------------------------------------- watch -- */

        private static Coroutine? watcher;
        private static readonly Dictionary<string, float> lastReported = new Dictionary<string, float>();

        /// <summary>How often the watcher compares angles, s.</summary>
        private const float WatchInterval = 0.25f;
        /// <summary>Angle change worth telling the clients about, deg.</summary>
        private const float ReportDeg = 0.5f;

        /// <summary>
        /// Publish OTHER people's rotations too. Our own coroutine pushes as it
        /// turns, but a table cranked by hand in the game moved silently and the
        /// map showed the old bridge until the next mod-driven turn — never, in
        /// practice. Started lazily by the first state fetch; dies with the mod
        /// on reload and is restarted the same way.
        /// </summary>
        private static void EnsureWatcher()
        {
            watcher ??= Updater.Run(Watch());
        }

        private static IEnumerator Watch()
        {
            while (true)
            {
                yield return new WaitForSecondsRealtime(WatchInterval);
                bool changed = false;
                foreach (var controller in All())
                {
                    var tt = controller.turntable;
                    string id = IdOf(tt);
                    float rot = TurntableRailTrack.AngleRange0To360(tt.currentYRotation);
                    if (!lastReported.TryGetValue(id, out var last)
                        || Mathf.Abs(TurntableRailTrack.AngleRangeNeg180To180(rot - last)) >= ReportDeg)
                    {
                        lastReported[id] = rot;
                        changed = true;
                    }
                }
                if (changed)
                    Changed?.Invoke();
            }
        }

        /// <summary>
        /// Turn a turntable to the given angle (0–360, world Y). Returns at
        /// once with the accepted goal; the bridge takes its time getting
        /// there and the update stream reports it on the way.
        /// </summary>
        public static Task<string> Rotate(string id, float angle) => Updater.RunOnMainThread(() =>
        {
            var controller = Find(id);
            if (controller == null)
                return new JObject { ["error"] = $"no turntable {id}" }.ToString(Formatting.None);

            if (turning.TryGetValue(id, out var previous) && previous != null)
                Updater.StopRun(previous);

            float goal = TurntableRailTrack.AngleRange0To360(angle);
            goals[id] = goal;
            turning[id] = Updater.Run(Turn(controller, id, goal))!;
            Changed?.Invoke();
            return new JObject
            {
                ["id"] = id,
                ["target"] = Math.Round(goal, 2),
                ["rot"] = Math.Round(TurntableRailTrack.AngleRange0To360(
                    controller.turntable.currentYRotation), 2),
            }.ToString(Formatting.None);
        });

        /// <summary>
        /// The turn itself: shortest way round, lever-speed steps every frame,
        /// exact landing on the goal so a snap angle actually connects (the
        /// game checks connection within ANGLE_THRESHOLD of a track end).
        /// </summary>
        private static IEnumerator Turn(TurntableController controller, string id, float goal)
        {
            float sincePush = 0f;
            while (true)
            {
                yield return null;
                if (controller == null || controller.turntable == null)
                    break;
                var tt = controller.turntable;
                float current = TurntableRailTrack.AngleRange0To360(tt.targetYRotation);
                float diff = TurntableRailTrack.AngleRangeNeg180To180(goal - current);
                float step = TurnRateDegPerSec * controller.speedMultiplier * Time.deltaTime;
                if (Mathf.Abs(diff) <= step)
                {
                    tt.targetYRotation = goal;
                    // Force the refresh: the last sliver of rotation may be
                    // rounded to nothing, and the connection check must still
                    // run at the landing angle.
                    tt.RotateToTargetRotation(forceConnectionRefresh: true);
                    break;
                }
                tt.targetYRotation = TurntableRailTrack.AngleRange0To360(current + Mathf.Sign(diff) * step);
                tt.RotateToTargetRotation();
                sincePush += Time.deltaTime;
                if (sincePush >= PushInterval)
                {
                    sincePush = 0f;
                    Changed?.Invoke();
                }
            }
            turning.Remove(id);
            goals.Remove(id);
            Changed?.Invoke();
        }
    }
}
