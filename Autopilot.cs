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
    /// <summary>
    /// Which of the two jobs a run is doing. They are different jobs, driven
    /// differently, and conflating them is what made one set of rules serve both
    /// badly: a road move crawling into a yard for the last kilometre, a shunt
    /// taking a lap of the valley to avoid changing ends.
    /// </summary>
    public enum AutopilotMode
    {
        /// <summary>Yard to yard over the road. The train runs at line speed to the
        /// running line serving the destination yard and stops on it; there is no
        /// spotting, no shoving and no coupling, because none of that belongs on a
        /// through move. Reversals are priced high — changing ends on a main line is
        /// worth a long detour to avoid.</summary>
        Drive,
        /// <summary>Moves within a yard. Speed is held to yard pace whatever the
        /// posted limit, the train is spotted precisely between the markers, and the
        /// dispatcher chooses which end leads, which junction to come in through and
        /// whether to couple onto what is already there. Reversals are cheap: a
        /// runaround is how the job is done.</summary>
        Shunt,
    }

    /// <summary>
    /// Full-route autopilot for a remotely controllable loco: route to a
    /// destination track (Dijkstra, see <see cref="Routing"/>), then drive it there
    /// obeying posted limits and stopping at the end.
    ///
    /// The route can double back on itself — a yard move from one siding to another
    /// off the same junction is a runaround: pull the whole train past the junction,
    /// throw it, then reverse onto the destination. So the path is split into
    /// <em>legs</em> at each reversal point, and between legs the drive waits until
    /// the pivot junction is physically clear of every car (a switch can't be thrown
    /// under a wagon) before throwing it and flipping direction.
    ///
    /// Any number of locos can be under autopilot at once — that is the point of
    /// dispatching from a map: set one train going across the valley and shunt
    /// another meanwhile. Each has its own run, keyed by car guid, so they start,
    /// report and stop independently and one erroring out leaves the rest driving.
    /// The player need not be aboard any of them.
    ///
    /// Each run is a coroutine on the mod's persistent object, so it survives scene
    /// work like the map bake.
    /// </summary>
    public static partial class Autopilot
    {
        /// <summary>One loco under autopilot: its coroutine, the controls it drives
        /// and what it is currently doing. Held so Stop brakes the loco this run
        /// actually took, not whatever car the player happens to be sitting in.</summary>
        private sealed class Run
        {
            public readonly TrainCar car;
            public readonly RemoteControllerModule controller;
            public readonly string guid;
            public readonly string id;
            public Coroutine? coroutine;
            public string state = "driving";
            public string detail = "";
            /// <summary>Buffer up to the cars standing on the destination and couple
            /// to them, instead of parking clear of them.</summary>
            public bool couple;
            /// <summary>The coupling has taken. What follows is an ordinary spot: shove
            /// the whole, now longer, train up the track until all of it is inside the
            /// markers — otherwise adding wagons to a part-full siding leaves half the
            /// consist hanging out over the switch.</summary>
            public bool coupledDone;
            /// <summary>Cars in our consist when the run began — coupling shows up as
            /// this number growing, which needs no guess about which train we met.</summary>
            public int carsAtStart;
            /// <summary>EXACTLY which cars are ours, from the start. Buffers left
            /// touching re-couple on the slightest contact; knowing our own lets
            /// the run cut strangers loose again and carry on.</summary>
            public readonly HashSet<string> members = new HashSet<string>();
            /// <summary>Which end of the destination to come in through: set by a via
            /// junction, and from nothing else. Every re-route must keep the
            /// constraint or the router would simply turn the train back in by the
            /// end it just left.</summary>
            public bool? arriveEnteredIn;
            /// <summary>The destination track.</summary>
            public RailTrack? destTrack;
            /// <summary>Where the run started. Together with the destination it says
            /// whether this is a loading move — which outranks a move that is only
            /// passing through the same rail. See Outranks.</summary>
            public RailTrack? originTrack;
            /// <summary>The arrive-end came from an EXPLICIT via junction — which,
            /// since the same-track guess was removed, is the only way it is ever
            /// set. The constraint therefore holds for the WHOLE run: letting a pull
            /// leg that happened to CROSS the destination clear it sent a
            /// "via J-204" move in through J-209.</summary>
            public bool viaLocked;
            /// <summary>How the dispatcher wants it to arrive: shoved (wagons first),
            /// pulled (loco first), or left to the router to decide.</summary>
            public ArriveMode arrive = ArriveMode.Auto;
            /// <summary>Road move or yard move — see AutopilotMode. Held on the run
            /// because every leg re-routes, and the re-route has to be planned by the
            /// same rules as the first one.</summary>
            public AutopilotMode mode = AutopilotMode.Drive;
            /// <summary>What this run pays for a change of ends, m of equivalent rail.</summary>
            public double reversalPenalty = Routing.LineReversalPenalty;
            /// <summary>The road this run still has to cover, re-planned at the top of
            /// every leg. Other runs read it to find out whether their route and ours
            /// actually touch — which is a different question from whether we happen to
            /// be going to the same place.</summary>
            public List<RailTrack> path = new List<RailTrack>();
            /// <summary>How far along path the train has got, kept up to date every
            /// drive tick. Rail before this index is already covered, and covered rail
            /// must not count as "still has to cover": comparing whole paths made a
            /// run wait on track the other train had left a minute earlier.</summary>
            public int pathFrom;
            /// <summary>We are on the last leg: the next thing we do is go into the
            /// destination. Until then a shared destination costs nobody anything.</summary>
            public bool finalLeg;
            /// <summary>What the run means to do from here, in named moves. Rebuilt
            /// from the live route at the top of every leg, so it always describes the
            /// plan the drive loop is actually working to and never a stale one.</summary>
            public List<Step> steps = new List<Step>();
            /// <summary>Cut the loco (and any MU'd units) off the wagons once the
            /// shunt is done, leaving the cut tied down and the loco free.</summary>
            public bool uncoupleOnFinish;
            /// <summary>Why the run gave up, if a blocked switch defeated it.</summary>
            public string blockedReason = "";
            /// <summary>Track we have found we cannot use: a junction there refuses to
            /// throw because someone has left a wagon on it. Fed back into the router so
            /// the next plan goes round rather than queueing behind it for ever. Every
            /// entry carries a clock — see Avoidance.</summary>
            public readonly Avoidance avoid = new Avoidance();
            /// <summary>Runs this one has given way to, and how many times. Meeting the
            /// same train twice is not bad luck: its road is written off for longer each
            /// time, so the re-plan bends further out instead of walking back into it.</summary>
            public readonly Dictionary<string, int> yielded = new Dictionary<string, int>();
            /// <summary>Track (and the side it is entered from) this run has reserved.
            /// Nobody else may drive over or throw a switch into one of these from the
            /// same side while it holds them — the OPPOSITE side of the same track is a
            /// separate reservation, so two trains meeting a shared track from its two
            /// different ends never block each other.</summary>
            public readonly HashSet<RailTrack> claimed = new HashSet<RailTrack>();
            /// <summary>Junctions this run holds. A junction is ONE exclusive block —
            /// the frog and all its legs — which is what makes two routes through
            /// it on different track pairs an actual conflict.</summary>
            public readonly HashSet<Junction> claimedJunctions = new HashSet<Junction>();

            /// <summary>The junctions the LAST authority walk took — the road standing
            /// set ahead of the train right now. Rebuilt by every walk.
            ///
            /// The release passes work out what is "behind" from the planned path
            /// alone, while the walk runs PAST the end of the current leg onto overrun
            /// rail no path entry covers. The two disagreed three times a second: the
            /// trace caught J-496 granted, dropped as behind, and re-granted on the
            /// next tick, over and over — a hole in the ledger at 3 Hz that another
            /// train could be authorised straight into.</summary>
            public readonly HashSet<Junction> authJunctions = new HashSet<Junction>();

            /// <summary>Turntables this run holds the ORIENTATION of. Separate from
            /// the bridge's track claim: the rail can be ours while the angle is
            /// somebody else's, and the angle is what a turntable move is.</summary>
            public readonly HashSet<TurntableRailTrack> claimedTurntables =
                new HashSet<TurntableRailTrack>();

            /// <summary>How hard this kind of loco may be driven — see LocoProfiles.
            /// Read once at the start: the settings are edited between runs, not during.</summary>
            public readonly LocoProfile profile;

            /// <summary>The speed limit binding the train RIGHT NOW, km/h — posted
            /// limits plus the junction and turntable caps. For the console's
            /// status line; 0 until the first drive tick writes it.</summary>
            public float limitKmh;

            /// <summary>Acceleration this train is currently allowed, m/s². Lives on the
            /// run, not on the leg: what the wheels did on the last leg is exactly what
            /// they will do on the next, so the lesson must survive a reversal.
            /// Seeded on the first tick from the adhesion sum.</summary>
            public float accelBudget = float.MaxValue;

            /// <summary>Ceiling the engine heat puts on the throttle, 0..1. On the
            /// run for the same reason as the budget above: the grade and the load
            /// that cooked the loco on this leg are still there on the next, and a
            /// ceiling that reset at every leg boundary handed full power back to a
            /// train that had just been pulled down for overheating.</summary>
            public float heatCap = 1f;

            /// <summary>Metres of planned route still ahead, and the speed the ETA
            /// is figured from. The speed is a long average, not the current one:
            /// an ETA computed from the needle reads "never" every time the train
            /// is held at a block and "ten minutes" the moment it opens up, which
            /// is no use to anyone watching a screen.</summary>
            public double routeRemaining = double.MaxValue;
            public float avgKmh;

            /// <summary>Trend of the hottest driven unit, °C per second, and the
            /// sampling state behind it. The ceiling moves on the TREND, not on the
            /// reading: a loco sitting stable at 86 °C is carrying its load without
            /// getting worse and has no business being pulled down to half power,
            /// which is exactly what cutting on the reading alone did.</summary>
            public float heatRise;
            public float heatLastTemp = float.NaN;
            public float heatSampleFor;

            /// <summary>Order this run was dispatched in. When two runs want the same
            /// rail, the one asked for first goes first — otherwise they take turns
            /// giving way to each other and neither arrives.</summary>
            public readonly int seq = ++dispatched;

            /// <summary>
            /// The run's own account of itself, newest last. Everything the drive
            /// loop decides used to go to Player.log and nowhere else, which meant
            /// the one thing a dispatcher needed — WHY the train is doing this —
            /// was the one thing no UI could show. These ride along in the status
            /// JSON instead.
            /// </summary>
            public readonly List<RunNote> notes = new List<RunNote>();

            public sealed class RunNote
            {
                public float at;
                public float last;
                public string text = "";
                public int count = 1;
            }

            /// <summary>
            /// Rail this run has written off, each entry with a clock on it. A track
            /// is banned for a while and forgiven again; only rail that beats the run
            /// AvoidStrikes times is banned for the rest of the job. The list used to
            /// be an add-only set, so one shunter standing on a throat for half a
            /// minute barred that road for the whole run — and, where there was no way
            /// round, killed the run outright long after the road had cleared.
            /// </summary>
            public sealed class Avoidance
            {
                private readonly Dictionary<RailTrack, (float until, int strikes)> entries =
                    new Dictionary<RailTrack, (float, int)>();

                /// <summary>Write a track off because it has actually defeated the run.
                /// False once it has failed too often to be worth another look: the
                /// caller then gives up instead of re-planning round it again.</summary>
                public bool Strike(RailTrack track, float seconds)
                {
                    entries.TryGetValue(track, out var entry);
                    int strikes = entry.strikes + 1;
                    entries[track] = (
                        strikes >= AvoidStrikes
                            ? float.MaxValue
                            : Time.realtimeSinceStartup + seconds,
                        strikes);
                    return strikes < AvoidStrikes;
                }

                /// <summary>Ban a track for a while WITHOUT counting it against the
                /// track's strikes. Giving way to another train is a matter of timing,
                /// not a road that is shut: three meets on the same rail must not bar
                /// it for the rest of the job.</summary>
                public void Hold(RailTrack track, float seconds)
                {
                    entries.TryGetValue(track, out var entry);
                    if (entry.until == float.MaxValue)
                        return;
                    entries[track] = (
                        Mathf.Max(Time.realtimeSinceStartup + seconds, entry.until),
                        entry.strikes);
                }

                /// <summary>Written off for the rest of the run — no amount of waiting
                /// will hand this road back.</summary>
                public bool Permanent(RailTrack track) =>
                    entries.TryGetValue(track, out var entry) && entry.until == float.MaxValue;

                public bool Contains(RailTrack track) =>
                    entries.TryGetValue(track, out var entry)
                    && Time.realtimeSinceStartup < entry.until;

                /// <summary>Take a track off the list, keeping its strike count: the
                /// only plan there is needs this road after all, so the drive loop's
                /// own guard has to deal with whatever is standing on it.</summary>
                public void Forgive(RailTrack track)
                {
                    if (entries.TryGetValue(track, out var entry))
                        entries[track] = (0f, entry.strikes);
                }

                /// <summary>The live bans, for the router. Null when there are none —
                /// which is the "no avoid set" the planner is already written for.</summary>
                public HashSet<RailTrack>? Live()
                {
                    float now = Time.realtimeSinceStartup;
                    HashSet<RailTrack>? live = null;
                    foreach (var kv in entries)
                    {
                        if (now >= kv.Value.until)
                            continue;
                        if (live == null)
                            live = new HashSet<RailTrack>();
                        live.Add(kv.Key);
                    }
                    return live;
                }
            }

            /// <summary>Long enough to cover a whole manoeuvre, short enough that a
            /// run stuck for ten minutes does not grow without bound.</summary>
            private const int MaxNotes = 60;
            /// <summary>How many opening notes are never evicted. A run that goes
            /// wrong is explained by how it STARTED as much as by what it is doing
            /// now, and a pure ring buffer throws that away first.</summary>
            private const int KeepHead = 12;
            /// <summary>How far back a repeat is looked for. Two, because the
            /// interesting failure is a CYCLE — "back clear of J-278", "re-plan",
            /// "back clear of J-278" — and matching only the previous line lets an
            /// alternating pair fill the whole buffer while collapsing nothing.</summary>
            private const int CycleWindow = 3;

            public void Note(string text)
            {
                float now = Time.realtimeSinceStartup;
                for (int i = notes.Count - 1; i >= 0 && i >= notes.Count - CycleWindow; i--)
                {
                    if (notes[i].text != text)
                        continue;
                    notes[i].count++;
                    notes[i].last = now;
                    return;
                }

                notes.Add(new RunNote { at = now, last = now, text = text });
                if (notes.Count > MaxNotes)
                    notes.RemoveAt(KeepHead);
                Main.DebugLog(() => $"[{id}] {text}");
            }

            public Run(TrainCar car, RemoteControllerModule controller)
            {
                this.car = car;
                this.controller = controller;
                guid = car.CarGUID;
                id = car.ID;
                if (car.trainset?.cars != null)
                    foreach (var c in car.trainset.cars)
                        if (c != null)
                            members.Add(c.CarGUID);
                profile = Main.settings.autopilot.For(car.carLivery?.id);
            }

            /// <summary>
            /// Everything a dispatcher's screen could want. The first block is the
            /// original contract and its names must not move — remote-dispatch
            /// reads them. The rest is additive: where the train physically is,
            /// what it is holding, what it is waiting for, and its own running
            /// account of why.
            /// </summary>
            public JObject ToJson()
            {
                var bogie = Bogie();
                var json = new JObject
                {
                    ["state"] = state,
                    ["detail"] = detail,
                    ["loco"] = id,
                    ["guid"] = guid,
                    ["mode"] = mode.ToString().ToLowerInvariant(),
                    ["couple"] = couple,
                    ["arrive"] = arrive.ToString().ToLowerInvariant(),
                    ["limit"] = Math.Round(limitKmh),
                    ["steps"] = new JArray(steps.Select(s => s.ToJson())),

                    // Where it is and what it is doing, so a stopped train can be
                    // told apart from a slow one without watching it.
                    ["dest"] = destTrack?.LogicTrack()?.ID?.ToString(),
                    ["track"] = bogie?.track?.LogicTrack()?.ID?.ToString(),
                    ["span"] = bogie == null ? null : (JToken)Math.Round(bogie.traveller.Span, 1),
                    // `car != null` and NOT `car?.`: a loco cleared out from under a
                    // run (sandbox clear, restoration reset) leaves a DESTROYED
                    // TrainCar, which is null only through Unity's own operator. The
                    // null-conditional does a plain reference check, sails past it,
                    // and GetForwardSpeed dies on the destroyed transform — once per
                    // status poll, for as long as the finished run is listed.
                    ["kmh"] = Math.Round(Math.Abs(car != null ? car.GetForwardSpeed() : 0f) * 3.6f, 1),
                    ["cars"] = car?.trainset?.cars?.Count ?? 0,
                    ["seq"] = seq,

                    // How far there is left to go and when it should get there.
                    // "remaining" counts the planned route ahead, so on a shunt
                    // with reversals it reads short — the rail covered twice is
                    // counted once.
                    ["remaining"] = routeRemaining == double.MaxValue
                        ? null : (JToken)Math.Round(routeRemaining),
                    ["avgKmh"] = Math.Round(avgKmh, 1),
                    ["etaSeconds"] = EtaSeconds() is double s ? (JToken)Math.Round(s) : null,
                    ["eta"] = EtaText(),

                    // How far through the plan, and what it still has to cover.
                    ["pathFrom"] = pathFrom,
                    ["pathCount"] = path.Count,
                    ["finalLeg"] = finalLeg,
                    ["path"] = new JArray(path.Skip(pathFrom).Take(12)
                        .Select(t => (JToken)(t.LogicTrack()?.ID?.ToString() ?? "?"))),

                    // Where the CONSIST actually is, which is a different question
                    // from what the run holds: claims are a moving window that
                    // still covers rail the train has left. "Is it clear of J-278
                    // yet" can only be answered from the cars themselves.
                    ["occupies"] = new JArray(OccupiedTracks()
                        .Select(t => (JToken)(t.LogicTrack()?.ID?.ToString() ?? "?"))),
                    // Junctions on the CURRENT plan that the consist is still
                    // standing on. A switch here cannot be thrown, so a plan that
                    // needs one of these is a plan the train cannot execute from
                    // where it lies.
                    ["fouls"] = new JArray(FouledPlanJunctions()
                        .Select(j => (JToken)JunctionIndex(j))),

                    // Why it might not be moving.
                    ["blocked"] = blockedReason,
                    // Only the LIVE bans: an entry whose clock has run out is not
                    // something the dispatcher's screen should still be showing as a
                    // road this train refuses.
                    ["avoid"] = new JArray((avoid.Live() ?? Enumerable.Empty<RailTrack>())
                        .Select(t => (JToken)(t.LogicTrack()?.ID?.ToString() ?? "?"))),
                    ["holdsTracks"] = new JArray(claimed
                        .Select(t => (JToken)(t.LogicTrack()?.ID?.ToString() ?? "?"))),
                    ["holdsJunctions"] = new JArray(claimedJunctions
                        .Select(j => (JToken)JunctionIndex(j))),
                    // Turntables whose ORIENTATION this run owns. Without this on
                    // the screen, two runs at one roundhouse look identical
                    // whether they are queueing politely or fighting.
                    ["holdsTurntables"] = new JArray(claimedTurntables
                        .Select(t => (JToken)(t.Track?.LogicTrack()?.ID?.ToString()
                            ?? t.uniqueID ?? "?"))),

                    // The run's own reasoning, oldest first. "count" above 1 means
                    // the decision was taken again and again — which is the shape a
                    // livelock has, and the thing worth showing loudest.
                    ["notes"] = new JArray(notes.Select(n => new JObject
                    {
                        ["at"] = Math.Round(n.at, 1),
                        ["last"] = Math.Round(n.last, 1),
                        ["count"] = n.count,
                        ["text"] = n.text,
                    })),
                };

                // The handles, so a train that is being braked by the autopilot is
                // distinguishable from one that simply has no power.
                var controls = controller?.controlsOverrider;
                if (controls != null)
                {
                    json["throttle"] = Math.Round(controls.Throttle?.Value ?? 0f, 2);
                    json["brake"] = Math.Round(controls.Brake?.Value ?? 0f, 2);
                    json["dynamic"] = Math.Round(controls.DynamicBrake?.Value ?? 0f, 2);
                    json["reverser"] = Math.Round(controls.Reverser?.Value ?? 0f, 2);
                    json["indep"] = Math.Round(controls.IndependentBrake?.Value ?? 0f, 2);
                }
                // The heat, per driven unit: the throttle backs off against these
                // and there was no way to see it happening from outside the cab.
                if (car != null)
                {
                    var temps = new JArray();
                    foreach (var c in MuGroup(car))
                    {
                        var observer = c?.GetComponentInChildren<MultipleUnitStateObserver>();
                        var simFlow = c?.SimController?.simFlow;
                        if (observer == null || simFlow == null
                            || string.IsNullOrEmpty(observer.temperaturePortId)
                            || !simFlow.TryGetPort(observer.temperaturePortId, out var port, true))
                            continue;
                        temps.Add(new JObject
                        {
                            ["car"] = c!.ID,
                            ["temp"] = Math.Round(port.Value, 1),
                            ["warn"] = Math.Round(observer.overheatStandardThreshold, 1),
                            ["crit"] = Math.Round(observer.overheatCriticalThreshold, 1),
                        });
                    }
                    if (temps.Count > 0)
                        json["temps"] = temps;
                    json["heatCap"] = Math.Round(heatCap, 2);
                    json["heatRise"] = Math.Round(heatRise, 3);
                }
                return json;
            }

            /// <summary>
            /// Seconds to the destination at the average speed made so far, or
            /// null when there is nothing sensible to say: the run is over, the
            /// route length is unknown, or the train has not been moving.
            /// Deliberately built on the average INCLUDING time spent standing —
            /// a train that spends half its journey held at blocks really does
            /// take twice as long, and an ETA that ignores that lies every time.
            /// </summary>
            private double? EtaSeconds()
            {
                if (state != "driving" && state != "waiting")
                    return null;
                if (routeRemaining == double.MaxValue || avgKmh < EtaMinKmh)
                    return null;
                return routeRemaining / (avgKmh / 3.6f);
            }

            /// <summary>The same figure for a human: "4 min", "1 h 12 min".</summary>
            private string EtaText()
            {
                if (!(EtaSeconds() is double seconds))
                    return "";
                int total = (int)Math.Round(seconds);
                int hours = total / 3600;
                int minutes = (total % 3600) / 60;
                if (hours > 0)
                    return $"{hours} h {minutes} min";
                if (minutes > 0)
                    return $"{minutes} min";
                return $"{total} s";
            }

            /// <summary>The bogie the whole mod measures this loco's position by.</summary>
            private Bogie? Bogie()
            {
                if (car == null) // destroyed reads as null only through Unity's operator
                    return null;
                var bogies = car.Bogies;
                return bogies != null && bogies.Length >= 2 ? bogies[1] : null;
            }

            /// <summary>Every track with one of our own bogies on it, in no
            /// particular order — where the train physically LIES.</summary>
            private IEnumerable<RailTrack> OccupiedTracks()
            {
                var seen = new HashSet<RailTrack>();
                var cars = car?.trainset?.cars;
                if (cars == null)
                    return seen;
                foreach (var c in cars)
                {
                    if (c?.Bogies == null)
                        continue;
                    foreach (var b in c.Bogies)
                        if (b?.track != null)
                            seen.Add(b.track);
                }
                return seen;
            }

            /// <summary>Junctions along the current plan that our own consist is
            /// still fouling. The same test the drive loop gates its reversals on,
            /// so what a dispatcher reads here is what the train is waiting for.</summary>
            private IEnumerable<Junction> FouledPlanJunctions()
            {
                var found = new List<Junction>();
                var set = car?.trainset;
                if (set == null)
                    return found;
                for (int i = 0; i + 1 < path.Count; i++)
                {
                    var junction = JunctionBetween(path[i], path[i + 1]);
                    if (junction != null && !found.Contains(junction)
                        && ConsistFoulsJunction(junction, set))
                        found.Add(junction);
                }
                return found;
            }
        }

        // Every run, by car guid. Only ever touched on the main thread (every entry
        // point hops there first), so no locking.
        private static readonly Dictionary<string, Run> runs = new Dictionary<string, Run>();
        /// <summary>Counts dispatches, so runs can be ordered by who asked first.</summary>
        private static int dispatched;

        // Interlocking: which run holds which track, keyed together with the side it is
        // entered from (the previous track on its path, or null for wherever it is
        // sitting right now). Two autopilots converging on the same yard throat used to
        // throw each other's switches and meet on the frog, because each planned as if
        // it were alone. A run may only drive over, and set switches into, a (track,
        // side) it holds; anyone else wanting the SAME side waits — a run approaching
        // the same track from its OTHER end holds a different key and is unaffected.
        private static readonly Dictionary<RailTrack, string> claims =
            new Dictionary<RailTrack, string>();


        // How far ahead to reserve, in path tracks. Enough to cover the switches being
        // set, short enough that a train crossing the map doesn't lock the whole route.
        // The reservation is a moving window: it follows the train and the rail behind
        // is handed straight back, so a long run holds a few tracks, never its route.
        // Junction blocks: exclusive, one holder. Track claims stay keyed by entry
        // side; junctions never are — a frog is a frog whichever way you cross it.
        private static readonly Dictionary<Junction, string> junctionClaims =
            new Dictionary<Junction, string>();

        // Turntable blocks: exclusive, one holder, and NOT covered by the track
        // claim on the bridge. Holding the bridge rail says nothing about which
        // way the bridge POINTS, and pointing it is the whole move — so a second
        // run could stand at another stall and wind the table out from under the
        // first, which is exactly what happened when two runs were dispatched at
        // one roundhouse. The claim covers the ORIENTATION, and it is held from
        // the first wind until the run has no further use for the table.
        private static readonly Dictionary<TurntableRailTrack, string> turntableClaims =
            new Dictionary<TurntableRailTrack, string>();

        // Queueing: a run stopped at a block it could not get says so here every
        // authority pass; a JUNIOR run must not grab that block out from under it.
        // Entries go stale in a few seconds so a run that moved on stops blocking
        // anyone.
        //
        // The WANTER is recorded, not its dispatch number, because seniority is not
        // dispatch order alone — see Outranks. Keeping a bare seq here meant the queue
        // and the reach-ahead reservation ranked the same pair opposite ways: L-046
        // (a loading move) held rail against L-011 by rank while L-011's want held the
        // same rail against L-046 by seq, and each stood waiting for the other with the
        // log saying, of one track, that each of them had it.
        private static readonly Dictionary<object, (string guid, float time)> blockWants =
            new Dictionary<object, (string, float)>();
        private const float WantFreshSeconds = 3f;
        // How far up its own plan a run may hold rail against a lower-ranked one, m.
        // Dispatch order only ever settled arguments between trains ALREADY at the same
        // frog; a senior train two blocks away had no standing at all, so the throat went
        // to whoever arrived first. Far enough that a train approaching a junction takes
        // it, short enough that a run cannot lock a yard from the other end of it.
        private const double PriorityReachMeters = 400.0;

        // Movement authority: how much rail to hold ahead. Distance-based — enough
        // to brake from the current speed with headroom — because a count of tracks
        // means 100 m at a yard throat and 3 km on the main.
        private const double AuthorityMinMeters = 150.0;
        private const double AuthorityMaxMeters = 1200.0;
        private const float AuthorityBrakeFactor = 1.5f;
        private const double AuthorityPad = 50.0;
        /// <summary>
        /// Stop this far short of the end of held rail, m.
        ///
        /// Must exceed JunctionGrantClearance. At 10 m a train that pulled up at
        /// its limit came to a stand INSIDE the 15 m no-grant zone of the frog
        /// ahead — so it fouled a junction it had never been granted, and the run
        /// waiting for that junction could not be given it either. Two trains then
        /// held each other up over rail neither was using.
        /// </summary>
        private const double SignalMargin = 20.0;
        /// <summary>A junction block is only handed back once the tail is THIS far
        /// past it, m of body clearance. The switch-clearance margin (~5 m) was
        /// enough to throw points but not to admit the next train: a follower's
        /// authority extended through a frog the leader's tail had barely left.</summary>
        private const double JunctionReleaseMargin = 15.0;
        /// <summary>No junction grants while anyone's body is within this of the
        /// frog, m. Two legs of a junction converge long before the frog itself:
        /// at the old 7 m a leader was cleared to swing through with a queued
        /// follower's nose just off the points, and they traded paint.</summary>
        private const double JunctionGrantClearance = 15.0;
        /// <summary>Room left beyond a track's MARKED clearance point when a hold is
        /// placed there, m. Covers the coast between the brake command and the stand.
        /// The marker is where the game says the two legs are far enough apart to
        /// pass; stopping a couple of metres short of it is what makes that true of
        /// the car body and not merely of the bogie the span is measured at.</summary>
        private const double ClearancePad = 3.0;

        // A train blocked this long gives its reservations up, so two trains that have
        // each half of a throat cannot sit there holding it against one another, m/s.
        private const float ClaimDeadlockSeconds = 20f;

        // How long a track written off for being fouled stays written off, s. Whatever
        // shut the road — a shunter, a rake left on a lead, another autopilot holding a
        // frog — is usually gone inside a minute, and the ban used to outlive it by the
        // whole job.
        private const float AvoidSeconds = 90f;
        // ...but rail that defeats the run this many times is written off for good.
        // Without the backstop a genuinely shut road is re-tried until the run times
        // out, which is a slower way of failing and says less about why.
        private const int AvoidStrikes = 3;
        // How long the rail under a train we gave way to is written off, s, multiplied
        // by the number of times we have given way to that same train. Shorter than the
        // general ban to start with: the other run is moving, so its road frees itself,
        // and we only need this plan to bend round where it stands NOW.
        private const float GiveWaySeconds = 45f;

        // The longest road a SHUNT may accept, m. Generous — a runaround over a big
        // yard's own 8 km line is real yard work — but it shuts the door on the
        // "re-route the long way round the valley" plans a fouled junction invites.
        private const double ShuntMaxRouteMeters = 8000;

        // Conservative service deceleration used to plan the stop, m/s^2.
        private const float StopDecel = 0.35f;
        // What a road run plans its slowing with, m/s^2. Road moves brake on the
        // DYNAMIC, which pulls far more gently than a service application of air —
        // planning at air rates meant the dynamic could never make the curve, the
        // overspeed grew until the air was blended in anyway, and every slowing
        // ended as an air stop. Plan gentler and the braking starts earlier
        // instead, far enough out that the dynamic alone usually makes it.
        private const float DynamicPlanDecel = 0.18f;
        // ...but the dynamic is SWITCHED OFF for now. On the DH4 the retarder
        // churns the same transmission oil the temperature gauge reads, so
        // slowing on it cooked the loco - and the overheat handling then
        // alternated air and dynamic without ever settling into a real brake
        // application, because the tempWarning branch below only bites while
        // the throttle is still open and during braking it is already shut.
        // Nothing is deleted: flip this back to true to have it all again.
        private const bool UseDynamicBrake = false;

        // Never open the throttle past this — a full-power launch spins the wheels
        // and cooks the traction motors. Notch up/down at these rates per 0.1 s
        // tick; back off hard the instant the wheels slip.
        private const float ThrottleUp = 0.012f;
        private const float ThrottleDown = 0.1f;
        private const float SlipThreshold = 0.06f;
        // Slip this bad is a break-away, not an edge case of adhesion: the wheels
        // are spinning free and a gentle notch-down won't catch them. Cut hard.
        private const float SevereSlip = 0.25f;
        // Once the sander triggers it keeps sanding this long after the last slip,
        // s. The slip signal flickers around the threshold as grip comes and goes,
        // and following it directly clicked the sander on and off every tick.
        private const float SanderHoldSeconds = 4f;
        // Heat management works off a MARGIN below each loco's own serialized
        // overheat threshold (DH4: 90 warning), never off absolute degrees — the
        // thresholds differ per machine. Waiting for the threshold itself is far
        // too late on a hydraulic: by then the oil is on the mark and still
        // climbing on heat already stored in the transmission.
        //
        // Margins below the loco's own warning line. Within 2 °C of it the
        // ceiling eases down; within 1 °C it comes down hard. Anywhere cooler,
        // and not still climbing, it works its way back up.
        //
        // These sit deliberately CLOSE to the line: on a DH4 (warning 90) that
        // is easing off at 88 and cutting hard at 89, so the loco is worked at
        // the top of its usable range instead of being nursed. Backing off at
        // 85 threw away five degrees of perfectly good engine and left a heavy
        // train unable to hold its speed uphill for no thermal benefit — the
        // buzzer is at 90 and nothing below it costs anything.
        private const float PreOverheatMargin = 2f;
        private const float HardOverheatMargin = 1f;
        // Rates the ceiling moves at, per second. Cutting is far faster than
        // recovery on purpose: back off hard, come back gently.
        private const float HeatCapCutFast = 0.5f;
        private const float HeatCapCutSlow = 0.08f;
        private const float HeatCapRecover = 0.02f;
        // Window the temperature trend is measured over, s, and the rise that
        // counts as "still climbing", °C/s. Below it the loco is thermally steady
        // and keeps the power it has, however warm the reading.
        private const float HeatTrendSeconds = 2f;
        private const float HeatRiseEpsilon = 0.02f;
        // Time constant of the average speed the ETA is built on, s. Long enough
        // that signal checks and yard crawls are absorbed rather than swinging
        // the estimate about; short enough to follow a genuine change of pace.
        private const float EtaSmoothingSeconds = 90f;
        // Below this the average is treated as "not moving" and no ETA is given
        // at all, rather than dividing by something near zero and reporting a
        // number of hours that means nothing, km/h.
        private const float EtaMinKmh = 1.5f;
        // The ceiling never goes below this: a cut that forbids movement cools
        // nothing and just reads as a stuck train. Low enough, though, that it
        // can actually arrest a climb — half power on a long bank could not, so
        // the temperature crept past the band to the buzzer anyway and the cap
        // was decoration.
        private const float HotThrottleCapHard = 0.4f;
        // Working hard below this speed is where a hydraulic transmission dumps
        // most of its heat: the engine is at full revs and the converter is
        // slipping against wheels that are barely turning, so the difference goes
        // into the oil rather than into the rail. While warm, cap harder there.
        private const float ConverterSlipKmh = 20f;
        private const float HotThrottleCapLow = 0.75f;
        // Dynamic braking: below this speed a dynamic fades to nothing, km/h.
        private const float DynamicBrakeMinKmh = 8f;
        // The dynamic handle RAMPS instead of snapping to the speed error. Snapping
        // broke wet wheels straight into slide, the slip tick zeroed the handle, and
        // the next tick snapped it right back — continuous wheelslip in rain. Per
        // 0.1 s drive tick.
        private const float DynamicRampUp = 0.04f;
        private const float DynamicRampDown = 0.12f;
        // Each slip also cuts a CAP on the handle (the dynamic's version of the
        // throttle's accel budget), so it settles just under what the rail actually
        // holds instead of retrying full force every time. The cap creeps back up.
        private const float DynamicSlipCut = 0.2f;
        private const float DynamicCapFloor = 0.15f;
        private const float DynamicCapRecover = 0.03f; // per second
        // Overspeed beyond which air is blended in on top of the dynamic straight
        // away rather than waiting to see whether the dynamic can cope, km/h.
        private const float DynamicBrakeAirBackupKmh = 12f;
        // How long the dynamic alone gets to produce ANY deceleration before the
        // air comes in to help, s.
        private const float DynamicBrakeStallSeconds = 5f;
        // Throttle is commanded as a FRACTION of the handle, never in notches. Notch
        // counts differ between locos, and an eighth — fine on a DE6 — quantises down
        // to no power at all on a coarser machine, which is why some locos would hardly
        // move. Fractions land on a sensible notch whatever the loco.
        // Acceleration budget the wheels are given, m/s². It tracks the adhesion sum
        // but is cut on real wheelslip at this rate and eased back at that one, so a
        // wet rail or a wrong μ is corrected by what the train actually does.
        private const float SlipAccelCut = 0.5f;
        private const float AccelRecover = 0.05f;
        // However badly it slips, the train is always allowed to try to move.
        private const float MinAccelBudget = 0.1f;
        // Brake pipe pressure at which the brakes count as off, bar. Below this the
        // train is still being held and no amount of throttle proves anything.
        private const float BrakeReleasedBar = 5.4f;
        // Roughly how long the train should take to close the remaining speed gap, s.
        // This is what makes it ease off as it comes up to the target instead of
        // arriving at it flat out and overshooting.
        private const float ApproachSeconds = 6f;
        // How long the FULL handle may fail to move the train before the run is
        // written off. Counted only at full power: a heavy rake on a grade spends
        // the first seconds working up through the gentle-start ceiling, and calling
        // that "stuck" abandoned trains that had never actually been given power.
        private const float StuckAtFullPowerSeconds = 6f;
        // Absolute backstop, s — nothing may sit trying to start for longer.
        private const float StuckGiveUpSeconds = 60f;
        // How long to hold for stock ahead that is not moving before treating the road
        // as blocked, s. Long enough for another train to pull clear, short enough that
        // a run does not sit in front of a parked rake until somebody notices.
        private const float BlockedGiveUpSeconds = 30f;
        // Below this the coarsest loco has no notch at all, so a command to move that
        // lands under it does nothing.
        private const float MinMoveThrottle = 0.25f;
        // Smoothing applied to the measured acceleration, per tick (about a second of
        // memory), and how far over the adhesion limit counts as really over.
        private const float AccelSmoothing = 0.1f;
        private const float AccelOverBand = 1.3f;
        // Ticks between full re-reads of the speed limits and switch proximity.
        private const int LimitRecheckTicks = 3;
        // The drive loop's tick, s — also the interval acceleration is measured over.
        private const float DriveTick = 0.1f;
        // Steel wheel on dry steel rail. This is the physical constant the adhesion
        // limit is built on, not a tuning knob: the force the driven wheels can put
        // down before they slip is this times the weight on them.
        private const float AdhesionCoefficient = 0.25f;
        private const float Gravity = 9.81f;
        // Service brake ceiling while simply slowing down. A full application at speed
        // locks wheels and flats them just as thoroughly as a slip does; full brake is
        // kept for an actual stop.
        private const float ServiceBrakeMax = 0.6f;

        // Top speed for moving around the yard, km/h (still capped by posted limits
        // and the obstacle guard).
        private const float ManeuverKmh = 25f;
        // Speed a pull creeps at over its last few metres, so the stop that follows is
        // a couple of metres and not half a siding, km/h.
        private const float PullCrawlKmh = 4f;
        // Under this the loco counts as standing still, km/h — the reverser may be
        // moved and its current motion says nothing about which way it is going.
        private const float MovingKmh = 0.6f;
        // Be down to that crawl this far before the clear point, m.
        private const double PullStopMargin = 3.0;
        // How far out from the destination track to come down to approach speed, m.
        // Beyond this the train runs at line speed — the whole point of a long haul.
        // 200 m of 12 km/h was a minute of crawling before every arrival.
        private const double ApproachDistance = 120.0;
        // How far ahead to read posted limits so the train is already slowed when it
        // reaches a restriction instead of discovering it underneath itself, m.
        private const double LimitLookahead = 600.0;
        // Moderate speed on the final leg approaching the destination (before any car
        // is on it), so the slow-down doesn't overshoot, km/h.
        private const float SpotApproachKmh = 15f;
        // Slow crawl once a car is on the destination — accurate spot, minimal coast,
        // km/h. Used for backing onto the marks after an overshoot; the forward spot
        // itself is planned to a stand rather than to a crawl.
        private const float SpotKmh = 5f;
        // Approach speed for a road move, which may have to cover the whole length of
        // a main line to reach its destination. Held to the shunting figure, a train
        // that had already arrived spent minutes crawling up its own destination.
        // There is deliberately no road equivalent of SpotKmh: the final stop is the
        // same physics whatever the move, and giving the road a faster one is what
        // put a 13-car train 25 m past its spot on SM-B3I.
        private const float DriveApproachKmh = 40f;
        // How far to pull into the destination track before stopping, m.
        private const float InsertionDepth = 12f;
        // Once the whole train is on the destination, pull this much DEEPER so the
        // loco's trailing end clears the throat switch's fouling point, m.
        private const float ExtraClearance = 20f;
        // A car this close to a junction (m) still fouls it — can't switch.
        private const double FoulingTolerance = 7.0;
        // How clear of a junction OUR OWN cars must be before we may throw it, m.
        // Bogie distance plus body overhang plus a margin: this one splits our train
        // if it is wrong.
        // Clearance between the junction and the nearest point of our own car's BODY,
        // m (see BodyDistanceToJunction — this is not a bogie distance). A car actually
        // on the points measures ~0, so this only has to cover the coupler swing and a
        // little slack; it does not have to guess at overhang any more.
        private const double OwnSwitchClearance = 2.5;
        // A pull always draws further forward than the switch check demands. If it
        // stopped at exactly that distance the train could come to rest in the gap
        // between "clear enough to finish pulling" and "clear enough to throw the
        // points", which is a deadlock with no way out: the leg is over, the switch
        // will not move, and the router has nowhere else to go. Keep this positive.
        private const double PullClearMargin = 3.0;
        // Keep at least this much clear of any car ahead, m — generous, because a
        // 200 t consist coasts well past where it is commanded to stop.
        private const double ObstacleBuffer = 10.0;
        // Switch-clearance inset from each physical track end to the parking marker, m.
        private const double MarkerClearance = 18.0;
        // Stop this far inside the markers, measured at the bogies — just the car-body
        // overhang past its end bogie (~2 m). The spot speed is low enough that the
        // coast is small, so the car ends land right at the markers, not deep past them.
        private const double ParkMargin = 2.5;
        // Car body past its end bogie, m. Only used to turn a coupler-to-coupler
        // consist length into a bogie-to-bogie one, which is what spans measure.
        private const double CarEndOverhang = 2.0;
        // Slack a cut must have INSIDE the marker window before the spot counts as
        // achievable, m. A 397 m cut in a 398 m window is arithmetically a fit and
        // practically not one: it has to land to the centimetre, the clearance test
        // keeps answering "not yet", and the train hunts up and down the siding for
        // ever. Below this much room it is treated as too long — one end goes to its
        // mark, the tail lies where it falls, and the run STOPS.
        private const double SpotFitSlack = 3.0;
        // Buffering up: close in at walking pace from this far out, m.
        private const double CoupleApproach = 25.0;
        // Speed for the last few metres onto the couplers, km/h. Slow enough not to
        // shove the standing train down the siding.
        private const float CoupleKmh = 3f;
        // How close the obstacle guard may let us get when the "obstacle" is the train
        // we are deliberately coupling to, m.
        private const double CoupleBuffer = 0.4;
        // Deceleration assumed when planning to stop short of something, m/s^2. At the
        // old 0.12 a car 100 m ahead pinned the train to 17 km/h and one 300 m ahead to
        // 30, which with the half-kilometre look-ahead meant creeping past every
        // occupied siding on the route. 0.25 is still well inside what a service
        // application gives, and the guard still holds 10 m short.
        private const float ObstacleDecel = 0.25f;
        // Stop this far short of rail that simply ENDS — an unaligned turntable's
        // pit edge, a stub with no buffer stop. Measured to the leading bogie, and
        // a car body overhangs its end bogie by a couple of metres, so 5 keeps the
        // nose clear of the hole.
        private const double RailEndMargin = 5.0;
        // Boarding a turntable bridge is the one place the nose must go CLOSER to
        // the end of rail than RailEndMargin allows: the whole consist has to get
        // its bogies aboard before the wind, and the game is happy with the body
        // overhanging the pit. Metres from the bridge end the leading bogie stops.
        private const double BridgeBoardMargin = 1.0;
        // Turntables are taken at walking pace: the crossing cap, km/h, and how far
        // out (m, to the front of the train) it starts applying.
        private const float TurntableKmh = 8f;
        // Once aboard, creep on until the consist's middle is over the table's,
        // at this speed and to within this much. "Every bogie is on the bridge"
        // only means the tail has just cleared the pit, which parks the train
        // hard against the near end and swings its far end out over the pit
        // when the table turns.
        private const float BridgeCentreKmh = 3f;
        private const double BridgeCentreTolerance = 0.75;
        private const double TurntableSlowZone = 50.0;
        // What another run's rail costs to route over, m of equivalent rail. A track
        // it has RESERVED is dearer than one merely on its remaining plan: the claim
        // means it is about to be there, the plan means it will be eventually.
        internal const double ClaimedRoadPenalty = 400.0;
        internal const double PlannedRoadPenalty = 120.0;
        // A spot whose remaining distance GROWS has been overshot. This much growth
        // is real movement rather than bogie-span jitter, m.
        private const double OvershootSlack = 0.5;
        // A stop this far past the marks is accepted as spotted; further back it
        // out. Within a couple of metres nobody would touch the handle again;
        // beyond that the spot is simply missed, m.
        private const double OvershootAcceptMeters = 3.0;
        // Floor under the planned approach speed, km/h. The spot is planned to a
        // stand, which alone would ask for zero before the train is there and leave
        // it stalled short; this keeps it walking onto the marks instead.
        private const float FinalCreepKmh = 2f;
        // How long the train may read as off its planned road before the leg is
        // abandoned and re-planned, s. Long enough to ride out the tick or two where a
        // bogie is crossing a frog and belongs to neither track.
        private const float OffPathSeconds = 2f;
        // Wait this long at a switch that will not throw before looking for another
        // way round, s. Long enough that a shunter about to move clear is worth
        // waiting for, short enough not to strand the train.
        private const float SwitchDetourSeconds = 8f;
        // How far ahead the switches on our path are re-checked every tick, m.
        private const double JunctionCheckDistance = 300.0;
        // Speed over a switch taken on the DIVERGING side, km/h. A train can run a
        // curve at line speed and be fine, then put a wagon on the ground taking a
        // frog at the same speed — the posted limit says nothing about it.
        private const float JunctionKmh = 40f;
        // How close to such a switch counts as being on it, m — enough that the whole
        // train is slowed over the frog rather than just the loco.
        private const double SwitchSlowZone = 30.0;
        // Overspeed bands, matching the HUD's colour grading. Being this far over the
        // limit that is coming up is acceptable only from this far out:
        //
        //   under +7 km/h   red 0-300,  orange 300-600, yellow 600-1000
        //   +7 to +15 km/h  red 0-600,  orange 600-1000
        //   over +15 km/h   red at every distance
        //
        // The drive aims to sit in orange or yellow and never red, so the allowance is
        // the most it may be over a limit that is still this far ahead.
        private const double BandNear = 300.0;
        private const double BandMid = 600.0;
        private const float BandOver1 = 7f;
        private const float BandOver2 = 14f; // just under the +15 that is red everywhere
        // Within this distance every car ahead counts, on our route or not: it is too
        // close for a routing question to be worth getting wrong, m.
        private const float CloseRangeGuard = 60f;
        // How far off the line of travel a car may be at that range and still count.
        // Tighter than the ~5 m between parallel tracks, so a neighbouring siding is
        // still ignored.
        private const float CloseRangeLateral = 2.5f;
        // Never scan further ahead than this for obstacles, m. At that decel the
        // braking distance from line speed is otherwise kilometres of rail walking.
        private const double MaxObstacleLookahead = 1500.0;

        private static string Json(string status, string info = "") =>
            new JObject { ["state"] = status, ["detail"] = info }.ToString(Formatting.None);

        /// <summary>Every run, keyed by car guid, so one poll covers however many
        /// locos are out there. A guid absent from the object is simply not under
        /// autopilot.</summary>
        /// <summary>The state of this loco's run ("driving", "arrived", ...) or
        /// null when it has none — for in-game UIs that don't want JSON.</summary>
        internal static string? RunStateOf(string carGuid) =>
            runs.TryGetValue(carGuid, out var run) ? run.state : null;

        /// <summary>The run's live detail line, or null without a run.</summary>
        internal static string? RunDetailOf(string carGuid) =>
            runs.TryGetValue(carGuid, out var run) ? run.detail : null;

        /// <summary>The run's current speed limit, km/h; 0 without a run.</summary>
        internal static float RunLimitOf(string carGuid) =>
            runs.TryGetValue(carGuid, out var run) ? run.limitKmh : 0f;

        /// <summary>The next planned move, as the web console's step list would
        /// read it — "drive to CME", "cross J-217 and clear it" — or null
        /// without a run or a plan. The console shows the whole list; the
        /// tablet has one spare row, so it gets the step at the top.</summary>
        internal static string? RunNextStepOf(string carGuid) =>
            runs.TryGetValue(carGuid, out var run) && run.steps.Count > 0
                ? run.steps[0].Describe() : null;

        public static string GetStatusJson()
        {
            return Updater.RunOnMainThread(() =>
            {
                var all = new JObject();
                foreach (var kv in runs)
                {
                    try
                    {
                        all[kv.Key] = kv.Value.ToJson();
                    }
                    catch (Exception e)
                    {
                        // One run that cannot describe itself used to take the
                        // WHOLE status down: the task faulted, .Result rethrew,
                        // and remote-dispatch turned the lot into "the Autopilot
                        // addon threw — see its log" — with nothing in any log,
                        // because nobody on either side ever printed it. Report
                        // the one broken run in place, name the fault where it
                        // can actually be read, and let the others through.
                        Main.mod?.Logger.Log($"[STATUS] {kv.Value.id} could not be described: {e}");
                        all[kv.Key] = new JObject
                        {
                            ["state"] = "error",
                            ["detail"] = $"status failed: {e.GetType().Name}: {e.Message}",
                            ["loco"] = kv.Value.id,
                            ["guid"] = kv.Value.guid,
                        };
                    }
                }
                return all.ToString(Formatting.None);
            }).Result;
        }

        /// <summary>
        /// The loco to drive: the one with the given car GUID, or — only when no GUID
        /// is supplied — whatever the player is riding. Dispatching a loco from the map
        /// with nobody aboard is the normal case, so the GUID is the real path and the
        /// player fallback just a convenience. Main thread only.
        /// </summary>
        private static TrainCar? ResolveLoco(string? guid, out string error)
        {
            error = "";
            TrainCar? car;
            if (!string.IsNullOrEmpty(guid))
            {
                car = SingletonBehaviour<TrainCarRegistry>.Instance?.GetTrainCarByCarGuid(guid);
                if (car == null)
                {
                    error = "no car with guid " + guid;
                    return null;
                }
            }
            else
            {
                car = PlayerManager.Car;
                if (car == null)
                {
                    error = "no loco given and player is not in one";
                    return null;
                }
            }
            if (!car.IsLoco)
            {
                error = car.ID + " is not a loco";
                return null;
            }
            return car;
        }

        /// <summary>
        /// The junction at one end of a track, as an index into the ordered junction
        /// list — the numbers the map shows.
        ///
        /// A yard track does not touch a junction directly: it runs into a short
        /// "#Y-#S…#T" frog stub, and the junction is beyond that. So this walks a few
        /// hops outward from the end in question rather than reading inJunction /
        /// outJunction off the track itself, which is null for exactly the tracks a
        /// dispatcher actually names.
        /// </summary>
        internal static int? JunctionIndexAtEnd(RailTrack track, bool inEnd, int maxHops = 4)
        {
            var ordered = RailTrackRegistry.Instance?.OrderedJunctions?.ToList();
            if (ordered == null)
                return null;
            RailTrack? t = track;
            bool towardIn = inEnd;
            for (int hop = 0; hop < maxHops && t != null; hop++)
            {
                var junction = towardIn ? t.inJunction : t.outJunction;
                if (junction != null)
                {
                    int index = ordered.IndexOf(junction);
                    return index >= 0 ? index : (int?)null;
                }
                var branch = towardIn ? t.GetInBranch() : t.GetOutBranch();
                if (branch?.track == null)
                    return null;
                t = branch.track;
                towardIn = !branch.first;
            }
            return null;
        }

        /// <summary>The junctions at each end of a track, for the console's "arrive
        /// via" picker: { "in": n, "out": m }, either of which may be null.</summary>
        public static string GetTrackJunctionsJson(string trackId)
        {
            return Updater.RunOnMainThread(() =>
            {
                var track = Component.FindObjectsOfType<RailTrack>()
                    .FirstOrDefault(t =>
                    {
                        var id = t.LogicTrack()?.ID;
                        return id != null && (id.ToString() == trackId
                            || id.FullDisplayID == trackId || id.FullID == trackId);
                    });
                if (track == null)
                    return Json("error", "no such track: " + trackId);
                int? inEnd = JunctionIndexAtEnd(track, true);
                int? outEnd = JunctionIndexAtEnd(track, false);
                return new JObject
                {
                    ["track"] = trackId,
                    ["in"] = inEnd == null ? JValue.CreateNull() : new JValue(inEnd.Value),
                    ["out"] = outEnd == null ? JValue.CreateNull() : new JValue(outEnd.Value),
                }.ToString(Formatting.None);
            }).Result;
        }

        /// <summary>
        /// Resolve "arrive through junction J-n" into which end of the destination the
        /// train must come in by.
        /// </summary>
        private static bool? EndForJunction(int? viaJunction, RailTrack destTrack, out string error)
        {
            error = "";
            if (viaJunction == null)
                return null;
            if (viaJunction == JunctionIndexAtEnd(destTrack, true))
                return true;   // entered by the span-0 end
            if (viaJunction == JunctionIndexAtEnd(destTrack, false))
                return false;  // entered by the span-max end
            error = $"J-{viaJunction} is not at either end of the destination";
            return null;
        }

        public static string Start(string? locoGuid, string destId, float cruiseKmh,
            bool couple = false, ArriveMode arrive = ArriveMode.Auto, int? viaJunction = null,
            AutopilotMode mode = AutopilotMode.Drive, string? yardId = null,
            bool uncoupleOnFinish = false)
        {
            return Updater.RunOnMainThread(() =>
            {
                if (!WorldStreamingInit.Instance || !WorldStreamingInit.IsLoaded)
                    return Json("error", "world not loaded");
                var car = ResolveLoco(locoGuid, out var resolveError);
                if (car == null)
                    return Json("error", resolveError);
                var controller = car.GetComponent<RemoteControllerModule>();
                if (controller == null)
                    return Json("error", car.ID + " is not remote-controllable");
                var bogies = car.Bogies;
                if (bogies == null || bogies.Length < 2 || bogies[1].track == null)
                    return Json("error", "loco not on a track");

                var start = bogies[1].track;

                // The two modes are not the same move with different numbers: each
                // decides its own destination, its own speed and its own idea of what a
                // reversal is worth, and each ignores the other's options outright
                // rather than half-honouring them.
                double reversalPenalty = mode == AutopilotMode.Shunt
                    ? Routing.YardReversalPenalty
                    : Routing.LineReversalPenalty;
                if (mode == AutopilotMode.Drive)
                {
                    // A road move puts the train on the running line and leaves it
                    // there. Shoving, coupling and picking an approach junction are
                    // yard work, so they are dropped rather than obeyed halfway.
                    couple = false;
                    viaJunction = null;
                    arrive = ArriveMode.Auto;
                    if (string.IsNullOrEmpty(destId))
                    {
                        destId = Yards.MainLineFor(yardId, start, car, reversalPenalty, out var mainError);
                        if (destId == null)
                            return Json("error", mainError);
                    }
                }
                else
                {
                    if (string.IsNullOrEmpty(destId))
                        return Json("error", "a shunt needs a track to move to");
                    // Yard pace whatever was asked for: a shunt is measured in metres,
                    // and line speed between two sidings is how stock gets shoved
                    // through the stop blocks.
                    cruiseKmh = cruiseKmh <= 0f ? ManeuverKmh : Mathf.Min(cruiseKmh, ManeuverKmh);
                }

                var path = Routing.FindPath(start, destId, car, null, arrive, null, reversalPenalty);
                if (path == null)
                {
                    return Json("error", arrive switch
                    {
                        ArriveMode.Shove => $"no route to {destId} arriving wagons first — "
                            + "nowhere to swap ends on the way (a turntable only takes "
                            + "a train shorter than its bridge)",
                        ArriveMode.Pull => $"no route to {destId} arriving loco first — "
                            + "nowhere to swap ends on the way",
                        _ => "no route to " + destId,
                    });
                }
                // The only road found runs through somebody's parked stock. That used
                // to be refused outright — but refusing is wrong for how a yard is
                // actually worked: the dispatcher sends this move off and clears the
                // obstruction with the next one, and stock that gets dispatched away
                // counts as busy rather than parked from that moment. So the run
                // starts, drives up, and stands short of the cars; the warning tells
                // the dispatcher what it will be waiting for. If nothing is done
                // about it the drive loop gives up on that road by itself.
                string? warning = null;
                if (Routing.BlockedRoute)
                {
                    var inTheWay = Routing.OccupiedTracks(path, car.trainset)
                        .Select(t => t.LogicTrack()?.ID?.ToString())
                        .Where(id => id != null && id != destId)
                        .ToList();
                    if (inTheWay.Count > 0)
                    {
                        warning = $"the road to {destId} runs through cars standing on "
                            + string.Join(", ", inTheWay) + " — will pull up short of them";
                    }
                }

                var destTrack = path[path.Count - 1];

                // One line per dispatch naming what was actually received — every
                // "the arrive was ignored" hunt so far has died for want of this.
                Main.mod?.Logger.Log($"[PLAN] {car.ID} -> {destId} arrive={arrive} via="
                    + $"{(viaJunction?.ToString() ?? "-")} couple={couple} mode={mode} "
                    + $"tracks={path.Count} cars={car.trainset?.cars?.Count ?? 1}");

                // A shunt panel is opened on one yard and lists only that yard's
                // tracks, so a destination outside it means the request and the picker
                // have gone out of step — say so instead of quietly sending a yard
                // pilot across the valley at 25 km/h.
                if (mode == AutopilotMode.Shunt && !string.IsNullOrEmpty(yardId))
                {
                    // The yard's own running lines count as being in the yard: drawing a
                    // cut out onto the main and putting it back is ordinary yard work,
                    // and the road belongs to no yard of its own to be checked against.
                    var destYard = Yards.YardOfTrack(destTrack);
                    if (destYard != yardId && !Yards.IsMainLineOf(yardId, destId))
                    {
                        return Json("error",
                            $"{destId} is not in {yardId} — send it there on the road first");
                    }
                }

                // "Arrive through J-n" is the same constraint as the runaround uses:
                // come in by that end of the destination.
                bool? viaEnd = EndForJunction(viaJunction, destTrack, out var viaError);
                if (viaError.Length > 0)
                    return Json("error", viaError);
                // Already standing on the destination, on the VIA side of what the
                // move is aimed at: the constraint is satisfied by where the train
                // IS. "Arrive through J-65" is an approach direction, not an order
                // to leave and come back in - manufacturing a runaround out of it
                // sent a coupling move on a pointless lap.
                if (viaEnd != null && destTrack == start)
                {
                    double locoSpan = bogies[1].traveller.Span;
                    double targetSpan = NearestForeignCarSpan(car, start, locoSpan);
                    if (double.IsNaN(targetSpan))
                    {
                        var (lo, hi) = MarkerZone(start);
                        targetSpan = (lo + hi) / 2.0;
                    }
                    bool satisfied = viaEnd.Value
                        ? locoSpan < targetSpan   // in by the span-0 end, target beyond
                        : locoSpan > targetSpan;  // in by the far end, target below
                    if (satisfied)
                        viaEnd = null; // just drive up the track
                }
                if (viaEnd != null)
                {
                    var viaPath = Routing.FindPath(start, destId, car, viaEnd, arrive, null, reversalPenalty);
                    if (viaPath == null)
                        return Json("error", $"no route into {destId} through J-{viaJunction}");
                    path = viaPath;
                    destTrack = path[path.Count - 1];
                }

                var legs = ComputeLegs(path);

                // Re-routing a loco that is already driving replaces its own run and
                // leaves every other loco alone. From here on the old run is gone, so
                // anything that throws must not leave the loco coasting with its
                // throttle open and nobody driving it — hence the catch below.
                StopRun(car.CarGUID, brake: false);
                try
                {
                // Asking a loco to go to the track it is already on means "take it to
                // the other side": out onto the line, round, and back in the far end.
                // Only when no via junction is given — a via picks the end
                // deliberately, and the guess must not override it. A coupling move
                // doesn't go round either: the cars are on this very track, so just
                // drive up to them and hook on.
                // The end to arrive by comes ONLY from a declared via junction. A
                // dispatch to the track the train already stands on re-spots it
                // along that track; it does not guess that a lap of the yard was
                // meant. Guessing was worse than useless: the guess had to hold
                // for the whole run, so it dragged the leftDest/reEntered state
                // machine behind it, and when the road it wanted did not exist the
                // run was left with a one-track plan it could not drive — the leg
                // loop then span through its whole move budget without the train
                // turning a wheel and reported "gave up after too many moves".
                // Somebody who wants the train brought in the other end says so
                // with a via junction, which is unambiguous and already supported.
                bool? arriveEnteredIn = viaEnd;
                var run = new Run(car, controller)
                {
                    arriveEnteredIn = arriveEnteredIn,
                    viaLocked = viaEnd != null,
                    arrive = arrive,
                    destTrack = path[path.Count - 1],
                    originTrack = path[0],
                    detail = $"routing to {destId} ({path.Count} tracks, {legs.Count} leg(s))",
                    couple = couple,
                    carsAtStart = car.trainset?.cars?.Count ?? 1,
                    uncoupleOnFinish = uncoupleOnFinish && mode == AutopilotMode.Shunt,
                    mode = mode,
                    reversalPenalty = reversalPenalty,
                };
                runs[run.guid] = run;
                // Mark the loco as driving itself, so anybody looking at the
                // train from outside can see it without opening a status page.
                Markings.Fit(car);
                LogSpotAtDispatch(car, run.destTrack, car.ID);
                // Never spot against an invented window in silence. If the track's own
                // marks could not be read, the run still goes — the dispatcher may not
                // care, and refusing a move over a missing sign would be worse — but it
                // says so, on the status line and in the log, instead of presenting a
                // guessed stopping point as if it were surveyed.
                if (run.destTrack != null && !MarkersAreReal(run.destTrack))
                {
                    run.detail = "marks for " + destId + " could not be read — "
                        + "spotting against an ESTIMATED window";
                    Main.mod?.Logger.Log(
                        $"[SPOT] {car.ID} WARNING: {destId} has no readable marker signs; "
                        + "using the fallback inset. The spot will be approximate.");
                }
                run.coroutine = Updater.Run(Drive(run, destId, cruiseKmh));
                return new JObject
                {
                    ["state"] = "driving",
                    ["loco"] = car.ID,
                    ["guid"] = car.CarGUID,
                    ["to"] = destId,
                    ["mode"] = mode.ToString().ToLowerInvariant(),
                    ["yard"] = string.IsNullOrEmpty(yardId) ? JValue.CreateNull() : new JValue(yardId!),
                    ["couple"] = couple,
                    ["arrive"] = arrive.ToString().ToLowerInvariant(),
                    ["via"] = viaJunction == null ? JValue.CreateNull() : new JValue(viaJunction.Value),
                    ["tracks"] = path.Count,
                    ["legs"] = legs.Count,
                    ["length"] = Math.Round(path.Sum(t => t.GetKinkedPointSet().span)),
                    ["warning"] = warning == null ? JValue.CreateNull() : new JValue(warning),
                }.ToString(Formatting.None);
                }
                catch (Exception e)
                {
                    // The old run has already been dropped, so there is nothing left
                    // driving this loco. Leave it braked rather than rolling.
                    controller.controlsOverrider.Throttle?.Set(0f);
                    controller.controlsOverrider.Brake?.Set(1f);
                    return Json("error", $"could not start: {e.Message}");
                }
            }).Result;
        }

        /// <summary>Read-only diagnostic: compute the route + leg split and dump how
        /// each track's ends are classified, without moving anything. Lets a wrong
        /// direction be debugged from data instead of by driving into things.</summary>
        public static string GetPlanJson(string? locoGuid, string destId,
            ArriveMode arrive = ArriveMode.Auto)
        {
            return Updater.RunOnMainThread(() =>
            {
                if (!WorldStreamingInit.Instance || !WorldStreamingInit.IsLoaded)
                    return Json("error", "world not loaded");
                var car = ResolveLoco(locoGuid, out var resolveError);
                if (car == null)
                    return Json("error", resolveError);
                var bogies = car.Bogies;
                if (bogies == null || bogies.Length < 2 || bogies[1].track == null)
                    return Json("error", "loco not on a track");

                var start = bogies[1].track;
                var path = Routing.FindPath(start, destId, car, null, arrive);
                if (path == null)
                    return Json("error", "no route to " + destId);
                // Same rule as Start, and it must STAY the same rule: a destination
                // the train already stands on is a re-spot along that track, not a
                // lap of the yard. This once guessed a runaround here while Start
                // guessed separately, so the dry run showed an eleven-track plan for
                // a move the drive loop then refused to make.
                var legs = ComputeLegs(path);

                string Id(RailTrack t) => t.LogicTrack()?.ID?.ToString() ?? "?";

                // Occupied tracks on the route mean the router had to fall back to an
                // occupancy-blind path — nothing else can put one here.
                var occupied = Routing.OccupiedTracks(path, car.trainset);

                var tracks = new JArray();
                for (int i = 0; i < path.Count; i++)
                {
                    bool? entryFirst = i > 0 ? !ExitIsOut(path[i], path[i - 1]) : (bool?)null;
                    bool? exitFirst = i < path.Count - 1 ? !ExitIsOut(path[i], path[i + 1]) : (bool?)null;
                    tracks.Add(new JObject
                    {
                        ["i"] = i,
                        ["id"] = Id(path[i]),
                        ["entryEnd"] = entryFirst == null ? "-" : (entryFirst.Value ? "first" : "last"),
                        ["exitEnd"] = exitFirst == null ? "-" : (exitFirst.Value ? "first" : "last"),
                        ["reversal"] = entryFirst != null && exitFirst != null && entryFirst == exitFirst,
                        ["span"] = Math.Round(path[i].GetKinkedPointSet().span),
                        ["occupied"] = occupied.Contains(path[i]),
                    });
                }

                float reverser = 1f;
                if (legs.Count > 0 && legs[0].s < path.Count - 1)
                    reverser = ExitIsOut(path[0], path[1]) == (bogies[1].TrackDirectionSign > 0) ? 1f : 0f;
                var legArr = new JArray();
                for (int li = 0; li < legs.Count; li++)
                {
                    if (li > 0) reverser = 1f - reverser;
                    var (s, e) = legs[li];
                    var pivot = li < legs.Count - 1 ? DivergingJunction(path[e], path[e + 1]) : null;
                    legArr.Add(new JObject
                    {
                        ["leg"] = li,
                        ["from"] = Id(path[s]),
                        ["to"] = Id(path[e]),
                        ["reverser"] = reverser,
                        ["dir"] = reverser >= 0.5f ? "fwd" : "rev",
                        ["hasPivot"] = pivot != null,
                        ["pivotBranch"] = pivot == null ? JValue.CreateNull() : new JValue(pivot.Value.branch),
                        ["pivotSelected"] = pivot == null ? JValue.CreateNull() : new JValue(pivot.Value.junction.selectedBranch),
                    });
                }

                return new JObject
                {
                    ["loco"] = car.ID,
                    ["from"] = Id(start),
                    ["to"] = destId,
                    ["trackDirectionSign"] = bogies[1].TrackDirectionSign,
                    ["steps"] = new JArray(
                        CompileSteps(path, legs, null, destId).Select(s => s.ToJson())),
                    ["legs"] = legArr,
                    ["tracks"] = tracks,
                }.ToString(Formatting.None);
            }).Result;
        }

        /// <summary>Diagnostic: command the reverser to v and trace its Value for
        /// ~2.5 s while braked, so we can see whether Set latches / how it lerps.
        /// No motion (throttle 0, brake 1 the whole time).</summary>
        public static string RevTest(string? locoGuid, float v)
        {
            var tcs = new System.Threading.Tasks.TaskCompletionSource<string>();
            Updater.RunOnMainThread(() =>
            {
                if (Updater.Run(RevTestCoro(locoGuid, v, tcs)) == null)
                    tcs.TrySetResult(Json("error", "no updater"));
            });
            return tcs.Task.Result;
        }

        private static IEnumerator RevTestCoro(string? locoGuid, float v, System.Threading.Tasks.TaskCompletionSource<string> tcs)
        {
            var car = ResolveLoco(locoGuid, out var resolveError);
            if (car == null)
            {
                tcs.TrySetResult(Json("error", resolveError));
                yield break;
            }
            var controller = car.GetComponent<RemoteControllerModule>();
            if (controller == null)
            {
                tcs.TrySetResult(Json("error", car.ID + " is not remote-controllable"));
                yield break;
            }
            var o = controller.controlsOverrider;
            o.Throttle?.Set(0f);
            o.Brake?.Set(1f);
            float before = o.Reverser?.Value ?? -99f;
            o.Reverser?.Set(v);
            var trace = new JArray();
            for (int i = 0; i < 25; i++)
            {
                trace.Add((double)Math.Round(o.Reverser?.Value ?? -99f, 3));
                yield return WaitForSecondsRealtime(0.1f);
            }
            var sim = car.SimController?.controlsOverrider;
            tcs.TrySetResult(new JObject
            {
                ["set"] = v,
                ["before"] = (double)Math.Round(before, 3),
                ["after"] = (double)Math.Round(o.Reverser?.Value ?? -99f, 3),
                ["trace"] = trace,
                ["sameOverriderAsSim"] = sim != null && ReferenceEquals(o, sim),
            }.ToString(Formatting.None));
        }

        /// <summary>Read-only: find the actual in-game track-sign markers near the
        /// loco's current track and report their exact span on it, plus the consist's
        /// bogie spans — so the parking zone is read from the real markers, not guessed.</summary>
        public static string GetMarkerDebug(string? locoGuid)
        {
            return Updater.RunOnMainThread(() =>
            {
                var car = ResolveLoco(locoGuid, out var resolveError);
                if (car == null)
                    return Json("error", resolveError);
                if (car.trainset == null || car.Bogies == null || car.Bogies.Length < 2)
                    return Json("error", "loco has no bogies");
                var track = car.Bogies[1].track;
                if (track == null)
                    return Json("error", "not on a track");

                var pts = track.GetKinkedPointSet().points;
                Vector3 move = WorldMover.currentMove;
                double physSpan = track.GetKinkedPointSet().span;

                (double span, float dist) Project(Vector3 wp)
                {
                    double best = 0;
                    float bd = float.MaxValue;
                    foreach (var p in pts)
                    {
                        float d = ((Vector3)p.position + move - wp).magnitude;
                        if (d < bd) { bd = d; best = p.span; }
                    }
                    return (best, bd);
                }

                string ReadText(object o, string field)
                {
                    var tmp = o.GetType().GetField(field)?.GetValue(o);
                    return tmp?.GetType().GetProperty("text")?.GetValue(tmp) as string ?? "";
                }

                // Track-sign markers within a few metres of this track.
                var markers = new JArray();
                foreach (var sh in Component.FindObjectsOfType<DV.Signs.TrackSignHover>())
                {
                    var (span, dist) = Project(sh.transform.position);
                    if (dist > 12f)
                        continue;
                    markers.Add(new JObject
                    {
                        ["id"] = ReadText(sh, "trackID"),
                        ["sub"] = ReadText(sh, "subYardID"),
                        ["span"] = Math.Round(span, 1),
                        ["dist"] = Math.Round(dist, 1),
                    });
                }

                var bogies = new JArray();
                foreach (var c in car.trainset.cars)
                    foreach (var b in c.Bogies)
                        if (b?.track == track)
                            bogies.Add(new JObject { ["car"] = c.ID, ["span"] = Math.Round(b.traveller.Span, 1) });

                return new JObject
                {
                    ["track"] = track.LogicTrack()?.ID?.ToString(),
                    ["physSpan"] = Math.Round(physSpan, 1),
                    ["markers"] = markers,
                    ["bogies"] = bogies,
                }.ToString(Formatting.None);
            }).Result;
        }

        /// <summary>
        /// Every named simulation port on a loco, with its live value — a scratch tool
        /// for finding the ID of whatever drives a specific gauge or fuse (e.g. the TM
        /// overheat fuse, which is a separate signal from GetEngineTemperatureState)
        /// so the drive loop can watch it too. Not used by the drive loop itself.
        /// </summary>
        public static string GetPortsDebug(string? locoGuid)
        {
            return Updater.RunOnMainThread(() =>
            {
                var car = ResolveLoco(locoGuid, out var resolveError);
                if (car == null)
                    return Json("error", resolveError);
                var simFlow = car.SimController?.simFlow;
                if (simFlow == null)
                    return Json("error", "loco has no simulation flow");

                var ports = new JArray();
                foreach (var p in simFlow.AllPorts)
                {
                    ports.Add(new JObject
                    {
                        ["id"] = p.id,
                        ["value"] = Math.Round(p.Value, 4),
                        ["type"] = p.type.ToString(),
                        ["valueType"] = p.valueType.ToString(),
                    });
                }

                var wsc = car.SimController?.wheelslipController;
                JObject? wheelslip = null;
                if (wsc != null)
                {
                    var curveKeys = new JArray();
                    foreach (var k in wsc.wheelslipToAdhesionDrop.keys)
                        curveKeys.Add(new JObject { ["time"] = k.time, ["value"] = k.value });
                    wheelslip = new JObject
                    {
                        ["wheelslip"] = wsc.wheelslip,
                        ["isWheelslipping"] = wsc.IsWheelslipping,
                        ["maxWheelslipRpm"] = wsc.maxWheelslipRpm,
                        ["orientedMaxWheelslipRpm"] = wsc.OrientedMaxWheelslipRpm,
                        ["totalForceLimit"] = wsc.TotalForceLimit,
                        ["curveKeys"] = curveKeys,
                    };
                }

                return new JObject { ["ports"] = ports, ["wheelslip"] = wheelslip }.ToString(Formatting.None);
            }).Result;
        }

        /// <summary>
        /// Every run's reserved path and destination, plus the live track claims —
        /// the exact data RunAheadOfUs/BlockedByWaitingRun compare to decide who waits
        /// for whom. A scratch tool for "why is X waiting on Y" questions.
        /// </summary>
        public static string GetPathDebug()
        {
            return Updater.RunOnMainThread(() =>
            {
                var runsJson = new JObject();
                foreach (var kv in runs)
                {
                    var run = kv.Value;
                    runsJson[kv.Key] = new JObject
                    {
                        ["loco"] = run.id,
                        ["state"] = run.state,
                        ["destTrack"] = run.destTrack?.LogicTrack()?.ID?.ToString(),
                        ["finalLeg"] = run.finalLeg,
                        ["path"] = new JArray(run.path.Select(t => t.LogicTrack()?.ID?.ToString())),
                        ["claimed"] = new JArray(run.claimed.Select(t =>
                            $"{t.LogicTrack()?.ID}")),
                        ["junctions"] = new JArray(run.claimedJunctions.Select(j =>
                            $"J-{JunctionIndex(j)}")),
                    };
                }
                var claimsJson = new JObject();
                foreach (var kv in claims)
                    claimsJson[$"{kv.Key.LogicTrack()?.ID}"] = kv.Value;
                foreach (var kv in junctionClaims)
                    claimsJson[$"J-{JunctionIndex(kv.Key)}"] = kv.Value;

                return new JObject { ["runs"] = runsJson, ["claims"] = claimsJson }
                    .ToString(Formatting.None);
            }).Result;
        }

        /// <summary>Stop one loco's autopilot, or every one of them when no guid is
        /// given. Stopping one never disturbs the others.</summary>
        /// <summary>Brake and forget every run, on the CALLER's thread. For the unload
        /// path only, which already runs on the main thread: going through
        /// Updater.RunOnMainThread there deadlocks the game outright, because the wait
        /// blocks the very thread that would have to tick the updater to do the work.
        /// </summary>
        public static void StopAllHere()
        {
            foreach (var guid in runs.Keys.ToList())
                StopRun(guid);
        }

        public static string Stop(string? locoGuid)
        {
            return Updater.RunOnMainThread(() =>
            {
                if (!string.IsNullOrEmpty(locoGuid))
                {
                    bool had = StopRun(locoGuid!);
                    return new JObject
                    {
                        ["state"] = "stopped",
                        ["guid"] = locoGuid,
                        ["wasRunning"] = had,
                    }.ToString(Formatting.None);
                }

                int count = runs.Count;
                foreach (var guid in runs.Keys.ToList())
                    StopRun(guid);
                return new JObject { ["state"] = "stopped", ["stopped"] = count }
                    .ToString(Formatting.None);
            }).Result;
        }

        /// <summary>Halt one run's coroutine, brake the loco it was driving and forget
        /// it. Returns whether there was a run. Main thread only.</summary>
        /// <summary>
        /// Stop every run and leave every loco braked, throttle shut, dynamic off —
        /// called just before a mod reload. The reload kills the drive coroutines
        /// without any of this, so a train reloaded mid-run kept its last throttle
        /// with nobody aboard to take it back off, and crashed.
        /// </summary>
        public static void SecureAllForReload()
        {
            Updater.RunOnMainThread(() =>
            {
                foreach (var guid in runs.Keys.ToList())
                {
                    if (runs.TryGetValue(guid, out var run)
                        && run.car != null && run.controller != null)
                        run.controller.controlsOverrider.DynamicBrake?.Set(0f);
                    StopRun(guid); // throttle shut, full brake
                }
            }).Wait();
        }

        /// <param name="brake">Leave the loco braked. False only when another run is
        /// about to take it over in the same tick: grabbing the brake there would
        /// drag a moving train down just to let the new route start it again.</param>
        private static bool StopRun(string guid, bool brake = true)
        {
            if (!runs.TryGetValue(guid, out var run))
                return false;
            runs.Remove(guid);
            ReleaseClaims(run);
            Markings.Unfit(guid);
            if (run.coroutine != null)
                Updater.StopRun(run.coroutine);
            // The loco can have been despawned since; a destroyed TrainCar compares
            // == null through Unity's operator, so this check covers both cases.
            if (brake && run.car != null && run.controller != null)
            {
                run.controller.controlsOverrider.Throttle?.Set(0f);
                run.controller.controlsOverrider.Brake?.Set(1f);
            }
            return true;
        }
    }
}
