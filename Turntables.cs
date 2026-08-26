using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;
using UnityModManagerNet;

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
        /// <summary>The game holds a consist on the bridge by its BOGIES — the car
        /// bodies and couplers may overhang the pit (two DH4s, 26 m coupler to
        /// coupler, stand fine on the 24.6 m CS bridge). The outermost axles sit
        /// roughly this far in from each end of the consist, so the fit test gets
        /// that much back before comparing against the bridge.</summary>
        private const double BridgeEndOverhang = 2.5; // per end, m

        private static double FitLength(double trainLength) =>
            Math.Max(0, trainLength - 2 * BridgeEndOverhang);

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
        /// True when any car of this consist stands within <paramref name="clear"/>
        /// metres of the pit, measured along its STALL track to the pit end. A bridge
        /// girder sweeps the rim as it turns, so anything left standing at an
        /// unaligned pit gets hit — which is what happened to L-040 at HB, stopped
        /// with its nose 3.4 m short of the rim while the table was wound under it.
        /// Bogies on the bridge itself do not count: a consist properly aboard turns
        /// with the table.
        /// </summary>
        public static bool FoulingPit(TrainCar? loco, TurntableRailTrack tt, double clear)
        {
            var cars = loco?.trainset?.cars;
            if (cars == null)
                return false;
            EnsureIndex();
            foreach (var c in cars)
            {
                if (c == null)
                    continue;
                foreach (var b in c.Bogies)
                {
                    var t = b?.track;
                    if (t == null || !stalls!.TryGetValue(t, out var controller))
                        continue;
                    if (controller.turntable != tt)
                        continue;
                    var end = tt.GetTrackEnd(t);
                    if (end == null)
                        continue;
                    // isFirst: the stall's IN end is the one facing the pit, so the
                    // pit is at span 0; otherwise it is at the far end of the span.
                    double span = b!.traveller.Span;
                    double toPit = end.isFirst
                        ? span
                        : t.GetKinkedPointSet().span - span;
                    if (toPit < clear)
                        return true;
                }
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
                if (FitLength(trainLength) > tt.Track.GetKinkedPointSet().span - BridgeFitMargin)
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
                    || FitLength(trainLength) > tt.Track.GetKinkedPointSet().span - BridgeFitMargin)
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
            var found = FindBridgeAhead(start, startSpan, towardOut, maxDist);
            if (found == null)
                return false;
            RotateBridgeTo(found.Value.table, found.Value.stall);
            return true;
        }

        /// <summary>
        /// The same search without the wind: which table, and which stall of it,
        /// the rail ahead dead-ends into. Split out so a caller that has to RESERVE
        /// the table before touching it can find out what it is about to ask for —
        /// winding first and asking afterwards is how two runs ended up fighting
        /// over one bridge.
        /// </summary>
        public static (TurntableRailTrack table, RailTrack stall)? FindBridgeAhead(
            RailTrack start, double startSpan, bool towardOut, double maxDist)
        {
            EnsureIndex();
            if (stalls!.Count == 0)
                return null;

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
                    return null;
                var tt = controller.turntable;
                var end = tt?.GetTrackEnd(t);
                if (tt == null || end == null || outward == end.isFirst)
                    return null;
                return (tt, t);
            }
            return null;
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
            {
                // Asked to reach a stall this table does not have. Silent until
                // now: the caller fires and forgets, so a wind that was never
                // ordered looked exactly like one that was ordered and ignored.
                Main.mod?.Logger.Log(
                    $"[TURNTABLE] {IdOf(tt)}: no track end for {stall.LogicTrack()?.ID?.ToString() ?? "?"} — not winding");
                return;
            }
            float current = TurntableRailTrack.AngleRange0To360(tt.currentYRotation);
            float a = TurntableRailTrack.AngleRange0To360(end.angle);
            float b = TurntableRailTrack.AngleRange0To360(end.angle + 180f);
            float goal = Mathf.Abs(TurntableRailTrack.AngleRangeNeg180To180(a - current))
                <= Mathf.Abs(TurntableRailTrack.AngleRangeNeg180To180(b - current)) ? a : b;
            Main.mod?.Logger.Log(
                $"[TURNTABLE] {IdOf(tt)}: winding {current:F1} -> {goal:F1} deg "
                + $"for {stall.LogicTrack()?.ID?.ToString() ?? "?"}");
            Rotate(IdOf(tt), goal);
        }

        /// <summary>Wind the bridge half a turn from where it stands — the other
        /// bridge end comes round to whatever the current end faces. This is how a
        /// facing order (shove/pull) is honoured on a table: the join decides which
        /// end of the CONSIST meets the stall, and half a turn swaps it.</summary>
        public static void RotateBy180(TurntableRailTrack tt)
        {
            float goal = TurntableRailTrack.AngleRange0To360(tt.currentYRotation + 180f);
            Main.mod?.Logger.Log(
                $"[TURNTABLE] {IdOf(tt)}: winding half round, {tt.currentYRotation:F1} -> {goal:F1} deg");
            Rotate(IdOf(tt), goal);
        }

        /* ------------------------------------------- who drives the table -- */

        private static Assembly? rdAssembly;
        private static MethodInfo? rdRotate;

        /// <summary>
        /// RemoteDispatch's own Turntables.Rotate(id, angle), or null when that
        /// mod is absent or disabled. The ids agree: both sides derive them the
        /// same way — uniqueID when the scene set one, else the table's fixed
        /// world position — so an id from here names the same table there.
        /// </summary>
        private static MethodInfo? RemoteDispatchRotate()
        {
            var mod = UnityModManager.FindMod("RemoteDispatch");
            if (mod?.Assembly == null || !mod.Active)
            {
                rdAssembly = null;
                rdRotate = null;
                return null;
            }
            if (!ReferenceEquals(rdAssembly, mod.Assembly))
            {
                rdAssembly = mod.Assembly;
                rdRotate = rdAssembly.GetType("DvMod.RemoteDispatch.Turntables")
                    ?.GetMethod("Rotate", BindingFlags.Public | BindingFlags.Static,
                        null, new[] { typeof(string), typeof(float) }, null);
            }
            return rdRotate;
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
            {
                // Said out loud: the callers fire and forget, so an id that names
                // no table used to end the wind here in silence.
                Main.mod?.Logger.Log($"[TURNTABLE] no turntable {id} — not winding");
                return new JObject { ["error"] = $"no turntable {id}" }.ToString(Formatting.None);
            }

            float goal = TurntableRailTrack.AngleRange0To360(angle);

            // ONE driver per table. RemoteDispatch carries the same turn
            // coroutine this file does — the two were written from the same
            // source — and each keeps its own `turning` and `goals`. Turning the
            // table ourselves therefore left its map with no goal to draw (it
            // reads its own `goals`, which we never touch) and, worse, its
            // Rotate only ever cancels a turn in ITS dictionary: a wind ordered
            // from the web console while we were turning ran BOTH coroutines,
            // each writing targetYRotation toward a different angle every frame.
            // So when it is there, it drives and we ask. Resolved per call: the
            // addon can be hot-reloaded, which swaps the assembly out from under
            // any cached handle.
            var handOver = RemoteDispatchRotate();
            if (handOver != null)
            {
                string? refusal = null;
                try
                {
                    // Its ANSWER, not just the fact that it was called. It reports
                    // a table it cannot find as {"error": ...} and returns
                    // normally, so a wind that never happened was indistinguishable
                    // here from one that did — the bridge simply never moved and
                    // the run stood waiting for a join that was not coming.
                    refusal = Refusal(handOver.Invoke(null, new object[] { id, goal }), id, goal);
                }
                catch (Exception e)
                {
                    // Its signature moved, or it threw. Turning the table matters
                    // more than who does it — fall through and drive it here.
                    refusal = e.GetBaseException().Message;
                }
                if (refusal == null)
                {
                    Changed?.Invoke();
                    return new JObject
                    {
                        ["id"] = id,
                        ["target"] = Math.Round(goal, 2),
                        ["rot"] = Math.Round(TurntableRailTrack.AngleRange0To360(
                            controller.turntable.currentYRotation), 2),
                        ["by"] = "remote-dispatch",
                    }.ToString(Formatting.None);
                }
                Main.mod?.Logger.Log(
                    $"[TURNTABLE] remote-dispatch would not turn {id} ({refusal}) — turning it here");
            }

            DriveHere(controller, id, goal);
            return new JObject
            {
                ["id"] = id,
                ["target"] = Math.Round(goal, 2),
                ["rot"] = Math.Round(TurntableRailTrack.AngleRange0To360(
                    controller.turntable.currentYRotation), 2),
            }.ToString(Formatting.None);
        });

        /// <summary>Drive the table from this mod: our own coroutine, replacing any
        /// turn of ours already running on that table.</summary>
        private static void DriveHere(TurntableController controller, string id, float goal)
        {
            if (turning.TryGetValue(id, out var previous) && previous != null)
                Updater.StopRun(previous);
            goals[id] = goal;
            turning[id] = Updater.Run(Turn(controller, id, goal))!;
            Changed?.Invoke();
        }

        /// <summary>
        /// What remote-dispatch complained about, or null when it took the turn on.
        /// Its Rotate answers with a JSON string wrapped in a Task; called from the
        /// main thread that task is normally already finished, and when it is not,
        /// the answer is chased up asynchronously rather than waited for — blocking
        /// the main thread on it would deadlock its own queue.
        /// </summary>
        private static string? Refusal(object? answer, string id, float goal)
        {
            if (answer is Task<string> task)
            {
                if (task.IsCompleted)
                    return task.IsFaulted
                        ? task.Exception?.GetBaseException().Message ?? "faulted"
                        : Complaint(task.Result);
                task.ContinueWith(finished =>
                {
                    string? late = finished.IsFaulted
                        ? finished.Exception?.GetBaseException().Message ?? "faulted"
                        : Complaint(finished.Result);
                    if (late == null)
                        return;
                    // It answered too late to fall through to, so the recovery has
                    // to be made from here: back onto the main thread and turn the
                    // table ourselves, or the run waits for ever on a bridge that
                    // nobody is moving.
                    Updater.RunOnMainThread(() =>
                    {
                        Main.mod?.Logger.Log(
                            $"[TURNTABLE] remote-dispatch refused {id} after the fact"
                            + $" ({late}) — turning it here");
                        var controller = Find(id);
                        if (controller != null)
                            DriveHere(controller, id, goal);
                    });
                });
                return null;
            }
            return Complaint(answer as string);
        }

        private static string? Complaint(string? said) =>
            said != null && said.Contains("\"error\"") ? said : null;

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
            Main.mod?.Logger.Log(
                $"[TURNTABLE] {id}: stopped at "
                + (controller == null || controller.turntable == null
                    ? "— the table is gone"
                    : $"{TurntableRailTrack.AngleRange0To360(controller.turntable.currentYRotation):F1} deg"));
            Changed?.Invoke();
        }
    }
}
