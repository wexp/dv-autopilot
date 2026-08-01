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
    /// <summary>The drive itself: one leg at a time, re-routed from wherever the
    /// train really is, with the 10 Hz tick that sets targets, watches for
    /// obstacles and knows when it has arrived.</summary>
    public static partial class Autopilot
    {
        /* --------------------------------------------------------- drive -- */

        /// <summary>
        /// Drive to the destination one leg at a time, RE-ROUTING from the loco's
        /// actual track after every leg. A shunt pull follows live switches and does
        /// not stay on the original Dijkstra track list, so committing to that whole
        /// plan up front sent later legs the wrong way. Re-routing each time means
        /// every leg is planned from where the train really is, and always uses the
        /// reliable first-leg direction. Switches for a leg are set once at its start
        /// (junctions ahead of the loco, hence clear) and never touched while rolling.
        /// </summary>
        private static IEnumerator Drive(Run run, string destId, float cruiseKmh)
        {
            var car = run.car;
            var controller = run.controller;
            var overrider = controller.controlsOverrider;
            var bogie = car.Bogies[1];
            overrider.IndependentBrake?.Set(0f);
            ReleaseHandbrakes(car);
            float prevReverser = float.NaN; // no leg driven yet

            for (int segment = 0; segment < 200; segment++)
            {
                // Nobody need be aboard, so the loco can be deleted out from under the
                // run (sandbox clear, restoration reset). A destroyed TrainCar reads as
                // null through Unity's operator — bail instead of throwing every tick.
                if (car == null)
                {
                    Finish(run, "error", "loco no longer exists");
                    yield break;
                }
                var track = bogie.track;
                if (track == null)
                {
                    overrider.Brake?.Set(1f);
                    Finish(run, "error", "loco left the rails");
                    yield break;
                }

                // Nothing moves with a handbrake pinned, and pins appear mid-run:
                // our own finishing tie-down leaves one on every cut (including a
                // split-off loco), so every leg starts by winding them all off.
                ReleaseHandbrakes(car);

                // Done when the whole consist is fully within the destination — or,
                // on a coupling run, as soon as our train has grown.
                // A runaround only counts as arrived once the train has actually left
                // the destination and come back in the other way.
                if (!run.viaLocked && run.arriveEnteredIn != null && run.destTrack != null)
                {
                    if (!run.leftDest && AllOffTrack(car, run.destTrack))
                        run.leftDest = true;
                    // Back on it again: the constraint has done its job, and holding it
                    // any longer would send the train out for another lap instead of
                    // letting it pull up inside the markers.
                    if (run.leftDest && !AllOffTrack(car, run.destTrack))
                        run.reEntered = true;
                }
                // A non-couple run that GREW picked something up by buffer contact
                // - an unplanned coupling. Driving on drags a train nobody asked
                // for through the yard; stop dead and say so.
                if (!run.couple && car.trainset?.cars != null
                    && car.trainset.cars.Count > run.carsAtStart)
                {
                    overrider.Throttle?.Set(0f);
                    overrider.Brake?.Set(1f);
                    Finish(run, "error",
                        "coupled onto stock it was never meant to touch - "
                        + "cut it off and start the move again");
                    yield break;
                }

                bool mayArrive;
                if (run.arriveEnteredIn == null)
                    mayArrive = true;
                else if (run.viaLocked)
                    // A via move may legitimately CROSS its destination on the way
                    // round; being fully inside it then is passing through, not
                    // arriving. Arrived = the CURRENT plan ends on the destination
                    // and comes in by the demanded end.
                    mayArrive = run.path.Count > 0
                        && run.path[run.path.Count - 1] == run.destTrack
                        && EntryEndIsIn(run.path, run.destTrack) == run.arriveEnteredIn;
                else
                    mayArrive = run.leftDest;
                if (run.couple && !run.coupledDone && Coupled(run))
                {
                    // Hooked on. From here it is an ordinary spot, with the cars we
                    // just joined being shoved up the track ahead of us.
                    run.coupledDone = true;
                    // The train we just picked up may be tied down — our own
                    // finishing tie-down pins a handbrake on every cut it leaves.
                    ReleaseHandbrakes(car);
                    run.detail = $"coupled; tucking the train into {destId}";
                    // If what we just hooked onto includes locos, they must not
                    // fight the lead unit's brake handle.
                    EnsureOneBrakeCutIn(car, car);
                }
                bool there = run.couple && !run.coupledDone
                    ? false
                    : mayArrive && ConsistClearWithinDest(car, destId, run.destTrack);
                if (there)
                {
                    overrider.Throttle?.Set(0f);
                    overrider.Brake?.Set(1f);
                    Finish(run, run.coupledDone ? "coupled" : "arrived", destId);
                    yield break;
                }

                var path = Routing.FindPath(
                    track, destId, car,
                    run.reEntered && !run.viaLocked ? null : run.arriveEnteredIn,
                    run.arrive, run.avoid, run.reversalPenalty);
                if (path == null)
                {
                    overrider.Brake?.Set(1f);
                    Finish(run, "error", "no route to " + destId);
                    yield break;
                }

                // The router falls back to the blocked road when there is no other, so
                // a plan that still runs through it means there IS no way round. Say so
                // and stop, rather than shuffling between waiting and re-planning until
                // someone notices.
                if (run.avoid.Count > 0 && path.Any(t => run.avoid.Contains(t)))
                {
                    overrider.Throttle?.Set(0f);
                    overrider.Brake?.Set(1f);
                    Finish(run, "error", run.blockedReason.Length > 0
                        ? run.blockedReason + " — and there is no way round; clear it and start again"
                        : "blocked and no way round");
                    yield break;
                }

                // Turntables. The router pretends the bridge reaches every stall;
                // reality needs the table actually wound round before any wheel
                // goes near the pit edge. When the NEXT hop is on or off a bridge
                // and the rail is not connected yet: stop dead, wind the table,
                // wait for the rails to join, and re-plan (the join makes the
                // ordinary leg machinery work from there on).
                if (path.Count >= 2 && !ExitIsOut(path[0], path[1]) && !EnterIsIn(path[0], path[1]))
                {
                    var overBridge = Turntables.AsBridge(path[0]) ?? Turntables.AsBridge(path[1]);
                    if (overBridge != null)
                    {
                        bool onBridge = Turntables.AsBridge(path[0]) != null;
                        overrider.Throttle?.Set(0f);
                        overrider.Brake?.Set(1f);
                        if (onBridge && !AllOnTrack(car, path[0]))
                        {
                            Finish(run, "error",
                                "the consist does not fit on the turntable bridge");
                            yield break;
                        }
                        while (Mathf.Abs(car.GetForwardSpeed()) * 3.6f > 0.3f)
                            yield return WaitForSecondsRealtime(0.2f);
                        // Wind the table to whichever stall the rail must join:
                        // the road onward when we stand on the bridge, our own
                        // track when we are waiting to drive onto it.
                        var stall = onBridge ? path[1] : path[0];
                        run.detail = $"turning the table to reach {path[1].LogicTrack()?.ID}";
                        Turntables.RotateBridgeTo(overBridge, stall);
                        float turned = 0f;
                        while (!ExitIsOut(path[0], path[1]) && !EnterIsIn(path[0], path[1]))
                        {
                            if (GamePaused)
                            {
                                yield return WaitWhilePaused();
                                continue;
                            }
                            yield return WaitForSecondsRealtime(0.25f);
                            turned += 0.25f;
                            if (turned >= 90f)
                            {
                                Finish(run, "error", "the turntable would not line up");
                                yield break;
                            }
                        }
                        continue; // rails joined — plan again and drive across
                    }
                }

                // A shunt is yard work. A re-plan can still hand back a road right
                // out of the yard — a junction fouled (or thrown by hand) at the
                // wrong moment re-routes "the long way round", which at yard speed
                // is a tour of the valley: an 11 km "pull" to the next yard over
                // was sighted. Stopping beats crawling the map.
                if (run.mode == AutopilotMode.Shunt)
                {
                    double len = path.Sum(t => t.GetKinkedPointSet().span);
                    if (len > ShuntMaxRouteMeters)
                    {
                        overrider.Throttle?.Set(0f);
                        overrider.Brake?.Set(1f);
                        Finish(run, "error",
                            $"the only road to {destId} is {len / 1000:F1} km — that is not a "
                            + "yard move; clear what is blocking the short way and start again");
                        yield break;
                    }
                }

                ReleaseOffPlan(run, path);
                var legs = ComputeLegs(path);
                run.steps = CompileSteps(
                    path, legs,
                    run.reEntered && !run.viaLocked ? null : run.arriveEnteredIn, destId);
                var (s, e) = legs[0];
                bool lastLeg = legs.Count == 1;
                // Publish what we still have to cover, so the other runs can tell a
                // genuine conflict from a coincidence of destinations.
                run.path = path;
                run.pathFrom = 0; // re-planned from the loco: nothing covered yet
                run.finalLeg = lastLeg;

                // Direction for this leg from the first hop's geometry — the reliable
                // computation (the one that was always right for the very first leg).
                float reverser = 1f;
                if (path.Count >= 2)
                {
                    reverser = ExitIsOut(path[0], path[1]) == (bogie.TrackDirectionSign > 0) ? 1f : 0f;
                }
                else
                {
                    // Already on the destination but not yet parked inside the markers —
                    // shuffle toward the zone based on the WHOLE consist's extent (not
                    // just the reference bogie): if any bogie is below the low marker,
                    // pull to +span; if any is above the high marker, go -span.
                    var (lo, hi) = MarkerZone(track);
                    double mn = double.MaxValue, mx = double.MinValue;
                    foreach (var c in car.trainset.cars)
                        foreach (var bb in c.Bogies)
                            if (bb?.track == track)
                            {
                                mn = Math.Min(mn, bb.traveller.Span);
                                mx = Math.Max(mx, bb.traveller.Span);
                            }
                    bool wantPlusSpan = mn < lo || mx <= hi; // default deeper unless over
                    if (mx > hi && mn >= lo)
                        wantPlusSpan = false;
                    // Coupling on our own track: head for the nearest cars that are
                    // not ours, wherever they stand — the markers say where to park,
                    // not where the train to pick up is.
                    if (run.couple && !run.coupledDone)
                    {
                        double target = NearestForeignCarSpan(car, track, bogie.traveller.Span);
                        if (!double.IsNaN(target))
                            wantPlusSpan = target > bogie.traveller.Span;
                    }
                    reverser = wantPlusSpan == (bogie.TrackDirectionSign > 0) ? 1f : 0f;
                }

                // The plan may be UNREACHABLE from where the train physically lies:
                // its front already ran through a junction on a branch the plan does
                // not use (shoved past a switch that was set the other way). Driving
                // on just digs deeper down the wrong road — the only move is a
                // shunter's: back up until the front is clear of that junction, throw
                // it, and go again. Detected by our own stock standing on a branch of
                // a planned junction that the plan itself does not touch.
                var wrongJ = WrongBranchJunction(car, path, reverser >= 0.5f);
                if (wrongJ != null)
                {
                    float backRev = ExitIsOut(path[0], path.Count >= 2 ? path[1] : path[0])
                        == (bogie.TrackDirectionSign > 0) ? 0f : 1f;
                    run.detail = $"backing clear of J-{JunctionIndex(wrongJ)} — "
                        + "the front stands on the wrong road";
                    overrider.Throttle?.Set(0f);
                    overrider.DynamicBrake?.Set(0f);
                    overrider.Brake?.Set(1f);
                    while (Mathf.Abs(car.GetForwardSpeed()) * 3.6f > MovingKmh)
                        yield return WaitForSecondsRealtime(0.1f);
                    overrider.Reverser?.Set(backRev);
                    yield return WaitForSecondsRealtime(0.4f);
                    prevReverser = backRev;

                    float bThrottle = 0f, bLast = 0f, bAccel = 0f, bSander = 0f, bDyn = 0f;
                    float backedFor = 0f;
                    bool backedClear = false;
                    while (backedFor < 120f)
                    {
                        yield return WaitForSecondsRealtime(0.1f);
                        backedFor += 0.1f;
                        if (GamePaused)
                            yield return WaitWhilePaused();
                        if (car == null)
                        {
                            Finish(run, "error", "loco no longer exists");
                            yield break;
                        }
                        if (!ConsistFoulsJunction(wrongJ, car.trainset))
                        {
                            backedClear = true;
                            break;
                        }
                        float sp = Mathf.Abs(car.GetForwardSpeed()) * 3.6f;
                        // Guard the way we are backing: cars or rail-end behind.
                        double stopIn = double.MaxValue;
                        var backLead = LeadingEnd(car, backRev >= 0.5f);
                        if (backLead != null)
                        {
                            double clr = ObstacleDistanceRail(
                                car, backLead.Value.bogie.track,
                                backLead.Value.bogie.traveller.Span,
                                backLead.Value.towardOut, 200,
                                out _, out _, out double rEnd);
                            if (clr != double.MaxValue)
                                stopIn = Math.Min(stopIn, clr - ObstacleBuffer);
                            if (rEnd != double.MaxValue)
                                stopIn = Math.Min(stopIn, rEnd - RailEndMargin);
                        }
                        float tgt = Mathf.Min(ManeuverKmh / 2f,
                            stopIn == double.MaxValue
                                ? ManeuverKmh / 2f
                                : Mathf.Sqrt(2f * ObstacleDecel * (float)Math.Max(0, stopIn)) * 3.6f);
                        run.detail = $"backing clear of J-{JunctionIndex(wrongJ)}, "
                            + $"v{sp:F0} tgt{tgt:F0}";
                        ApplyThrottle(controller, ref bThrottle, car, tgt, sp,
                            run.profile, ref bLast, ref bAccel, ref run.accelBudget,
                            ref bSander, ref bDyn, false, false, false);
                        if (tgt < 0.8f && sp < 0.5f && stopIn < 1)
                            break; // pinned against something behind — cannot back further
                    }
                    overrider.Throttle?.Set(0f);
                    overrider.Brake?.Set(1f);
                    while (car != null && Mathf.Abs(car.GetForwardSpeed()) * 3.6f > MovingKmh)
                        yield return WaitForSecondsRealtime(0.1f);
                    if (!backedClear)
                    {
                        Finish(run, "error",
                            $"cannot back clear of J-{JunctionIndex(wrongJ)} — "
                            + "something behind, or the junction never cleared");
                        yield break;
                    }
                    continue; // re-plan: the switch can be thrown now
                }

                // Only stop for a direction change. A same-direction continuation (the
                // pull re-routing forward through the throat) rolls straight through —
                // no full-braking at every junction.
                // First leg: no previous direction to compare against, but a loco that
                // is already rolling has one — read it off its motion rather than
                // braking a moving train to a stand just to start it again the same
                // way. Reverser 1 drives along transform.forward, the sign
                // GetForwardSpeed() reports.
                if (float.IsNaN(prevReverser))
                {
                    float rolling = car.GetForwardSpeed() * 3.6f;
                    if (Mathf.Abs(rolling) > MovingKmh)
                        prevReverser = rolling > 0f ? 1f : 0f;
                }
                bool changeDir = float.IsNaN(prevReverser) || reverser != prevReverser;
                if (changeDir)
                {
                    overrider.Throttle?.Set(0f);
                    // The dynamic is for slowing on the run, never for a reversal
                    // stop: it must be fully off before the reverser moves.
                    overrider.DynamicBrake?.Set(0f);
                    // Say WHY the train is being stopped. Braking a long rake takes the
                    // best part of a minute, and with no message the status sat on the
                    // routing line the whole time, which looked like a hang.
                    string want = reverser >= 0.5f ? "forward" : "reverse";
                    run.detail = $"stopping to set the reverser to {want} "
                        + $"(rolling {car.GetForwardSpeed() * 3.6f:+0;-0;0} km/h)";
                    // Full application: the train must actually STOP before the reverser
                    // moves, and a graduated one just drags the stop out over hundreds of
                    // metres while the leg waits.
                    overrider.Brake?.Set(1f);
                    while (Mathf.Abs(car.GetForwardSpeed()) * 3.6f > MovingKmh)
                        yield return WaitForSecondsRealtime(0.1f);
                    overrider.Reverser?.Set(reverser);
                    yield return WaitForSecondsRealtime(0.4f);
                }
                else
                {
                    overrider.Reverser?.Set(reverser); // same direction — no stop
                    run.detail = $"leg {segment}: rolling on, no direction change";
                }

                // Signaling model: no waiting at the origin. Take whatever rail is
                // grantable right now - the drive loop advances to the end of its
                // authority and creeps forward block by block as the trains ahead
                // release sections. This one pass is only so the switch-setting
                // below has something held to set.
                ExtendAuthority(
                    run, path, s, track, bogie.traveller.Span,
                    (reverser >= 0.5f) == (bogie.TrackDirectionSign > 0),
                    ConsistLengthAhead(car, bogie), AuthorityMinMeters,
                    out _, out _, out _);
                run.state = "driving";

                // Set the switches we HOLD, and only those. Setting the whole route at
                // once is what let a loco bound for C2I re-point junctions under a
                // consist already running to C4S: each train laid out its entire road
                // the moment its leg began, so the second one overwrote the first.
                // Everything beyond the reservation is set later by the rolling check
                // in DriveLeg, once it is close enough to be ours.
                bool HeldByUs(int i) =>
                    !claims.TryGetValue(path[i], out var g) || g == run.guid;
                bool JunctionOurs(int i)
                {
                    var j = JunctionBetween(path[i], path[i + 1]);
                    return j == null
                        || !junctionClaims.TryGetValue(j, out var g) || g == run.guid;
                }
                for (int i = s; i < e; i++)
                    if (HeldByUs(i) && HeldByUs(i + 1) && JunctionOurs(i))
                        SetSwitchBetween(path[i], path[i + 1]);
                if (lastLeg)
                    for (int i = 0; i < path.Count - 1; i++)
                        if (HeldByUs(i) && HeldByUs(i + 1) && JunctionOurs(i))
                            SetSwitchBetween(path[i], path[i + 1]);

                overrider.Brake?.Set(0f);

                bool ok = false;
                yield return DriveLeg(run, bogie, path, s, e, lastLeg, reverser, destId, cruiseKmh, r => ok = r);
                if (!ok)
                    yield break; // DriveLeg reported the error and braked

                // Keep our holdings across legs - a full release opened a gap in
                // which another run could take the rail under our standing train.
                // The next plan trims anything it no longer uses.
                prevReverser = reverser;
            }

            overrider.Brake?.Set(1f);
            Finish(run, "error", "gave up after too many moves");
        }

        /// <summary>
        /// Drive one leg to a stop, world-space and occupancy based rather than by
        /// following an exact track list (which was too brittle through a yard
        /// throat). A pull leg drives forward until the whole consist has left its
        /// start siding — that is "the last wagon cleared the junction". The final
        /// leg drives (usually reversing) until the loco itself sits on the
        /// destination track. The proximity guard runs throughout so nothing gets hit.
        /// </summary>
        private static IEnumerator DriveLeg(
            Run run, Bogie bogie, List<RailTrack> path,
            int s, int e, bool lastLeg, float reverser, string destId, float cruiseKmh, Action<bool> done)
        {
            var car = run.car;
            var controller = run.controller;
            var overrider = controller.controlsOverrider;
            // The remote-control interface reports when the coupler is close enough to
            // take; the overrider has no such notion.
            var remote = car.GetComponent<ILocomotiveRemoteControl>();
            var startTrack = path[s];
            // The junction just behind the pull — the tail must clear it before we can
            // throw it and reverse, else the reverse takes the old branch.
            var startJunction = !lastLeg && s + 1 < path.Count ? JunctionBetween(path[s], path[s + 1]) : null;
            string phase = lastLeg ? "final" : "pull";
            float stuckFor = 0f;
            float switchHold = 0f;
            float blockedFor = 0f;
            float offPathFor = 0f;
            // The front has been as far as the track this leg reverses on. Latched,
            // because pulling the LAST car clear of the junction runs the front out
            // past that track whenever the train is longer than it — which is most of
            // the time in a yard, where the reversal track is a frog stub. Asking
            // "are we still on the planned road" from then on answers no for ever.
            bool reachedPivot = false;
            int limitCountdown = 0;
            float cachedLimit = 999f;
            bool cachedNearSwitch = false;
            double cachedAuthority = double.MaxValue;
            string cachedAuthorityHolder = "";
            string cachedAuthorityAt = "";
            RailTrack? cachedAuthorityBlocked = null;
            float authorityStallFor = 0f;
            float throttle = 0f;
            float fullPowerFor = 0f;
            float lastSpeedMs = Mathf.Abs(run.car.GetForwardSpeed());
            float accelFilter = 0f;
            float sanderHold = 0f;
            float dynStallFor = 0f;
            // One bridge request per leg — the rotate coroutine must not be
            // restarted ten times a second while the train waits at the pit.
            bool bridgeAsked = false;
            // Whether the train is creeping back onto marks it drove past, and
            // which way along the destination the spot was travelling (true =
            // toward increasing span), learned while moving.
            bool backingUp = false;
            bool? spotTowardHigh = null;
            // Road runs slow on the dynamic, which pulls far more gently than air —
            // every planned slowing must start correspondingly earlier or the air
            // ends up doing the work anyway.
            float planDecel = run.mode == AutopilotMode.Drive ? DynamicPlanDecel : StopDecel;

            while (true)
            {
                yield return WaitForSecondsRealtime(0.1f);
                // A paused game means a frozen train, not a stuck one. Every timer
                // below must stop with it.
                if (GamePaused)
                    yield return WaitWhilePaused();

                if (car == null)
                {
                    Finish(run, "error", "loco no longer exists");
                    done(false);
                    yield break;
                }

                var track = bogie.track;
                if (track == null)
                {
                    overrider.Brake?.Set(1f);
                    Finish(run, "error", "loco left the rails");
                    done(false);
                    yield break;
                }

                float speed = Mathf.Abs(car.GetForwardSpeed()) * 3.6f; // km/h
                // How far our own cars reach beyond the reference bogie, so switch and
                // limit distances are measured from the front of the train.
                double ownLeadHint = ConsistLengthAhead(car, bogie);
                Vector3 motion = (reverser >= 0.5f ? 1f : -1f) * car.transform.forward;
                // Which way along the current track's span we are travelling — needed
                // by the distance, look-ahead and obstacle checks alike.
                bool towardOut = (reverser >= 0.5f) == (bogie.TrackDirectionSign > 0);
                // Obey the tightest of: the limits under the WHOLE train, and the ones
                // coming up. A restriction binds every wagon, not the loco — the loco
                // clearing a 40 zone and opening up would drag the tail through it at
                // line speed, which is how you put the last wagon on the ground.
                // The limit the WHOLE train is subject to: no opening up until the last
                // wagon is out of the tighter zone. Re-read a few times a second rather
                // than every tick — it walks hundreds of metres of rail, and a limit
                // cannot move far in 300 ms.
                if (--limitCountdown <= 0)
                {
                    limitCountdown = LimitRecheckTicks;
                    float here = ConsistLimitKmh(car);
                    cachedLimit = PlannedLimitKmh(
                        track, bogie.traveller.Span, towardOut, ownLeadHint, here, LimitLookahead,
                        run.mode == AutopilotMode.Drive ? DynamicPlanDecel : (float?)null);
                    cachedNearSwitch = NearDivergingSwitch(car, path);

                    // Movement authority: hold enough rail ahead to brake in, with
                    // headroom; hand back what the tail has cleared and anything an
                    // older run is stood waiting for. This is the signal aspect the
                    // speed shaping below drives to.
                    double needMeters = Math.Min(AuthorityMaxMeters, Math.Max(
                        AuthorityMinMeters,
                        (speed / 3.6) * (speed / 3.6) / (2f * StopDecel)
                            * AuthorityBrakeFactor + AuthorityPad));
                    int authIdx = IndexNear(path, track, s);
                    cachedAuthority = ExtendAuthority(
                        run, path, authIdx, track, bogie.traveller.Span, towardOut,
                        ownLeadHint, needMeters,
                        out cachedAuthorityHolder, out cachedAuthorityAt,
                        out cachedAuthorityBlocked);
                    ReleaseJunctionsBehind(run, path, Math.Max(0, authIdx));
                    PreemptWantedBlocks(run);

                    // Tripwire: any of our wheels on rail somebody else holds is a
                    // violation of the whole model - name the moment, the rail, the
                    // holder and what we thought we were doing, in the mod log.
                    foreach (var c in car.trainset.cars)
                    {
                        if (c == null)
                            continue;
                        foreach (var b in c.Bogies)
                        {
                            if (b?.track == null)
                                continue;
                            if (claims.TryGetValue(b.track, out var vg) && vg != run.guid)
                                Main.mod?.Logger.Log(
                                    $"[VIOLATION] {run.id} bogie on "
                                    + $"{b.track.LogicTrack()?.ID} held by {RunNameOf(vg)}"
                                    + $" | auth={cachedAuthority:F0} at={cachedAuthorityAt}"
                                    + $" | {run.detail}");
                        }
                    }
                }
                float limit = cachedLimit;

                // Switches: plan down to a safe speed for the next one we take on the
                // diverging side, and hold that speed until the whole train is off the
                // frog — the tail is still on it long after the loco has gone by.
                if (cachedNearSwitch)
                {
                    limit = Mathf.Min(limit, JunctionKmh);
                }
                else
                {
                    double toSwitch = DistanceToDivergingSwitch(
                        path, IndexNear(path, track, s), track, bogie.traveller.Span,
                        towardOut, ownLeadHint);
                    if (toSwitch < double.MaxValue)
                    {
                        limit = Mathf.Min(limit, ApproachSpeedKmh(JunctionKmh, toSwitch, planDecel));
                    }
                }
                float target;
                // The dynamic pulls too little at low speed to land a stop where it
                // was planned — pulling up, spotting and buffering-up get the air,
                // which is what the plan's stopping curve assumes.
                bool airStop = false;

                // Buffering up only makes sense once the destination is close. Looking
                // for the train to couple to from the far side of the map finds nothing
                // — cars that far away are not even streamed in — and the run then spent
                // the whole journey at coupling approach speed, "looking for the train".
                bool nearDest = !run.couple || run.coupledDone
                    || AnyCarOnTrack(car, destId)
                    || DistanceToDest(path, track, bogie.traveller.Span, towardOut) <= ApproachDistance;

                if (!lastLeg)
                {
                    // A pull that has wandered off its own route can never finish: the
                    // clear test ends with "have we reached the reversal track", and a
                    // track that is not on the path has no index, so the answer is no
                    // for ever. The distance terms go to zero at the same time, which
                    // leaves the crawl floor as the only thing setting the speed — and
                    // the train quietly walks out of the yard at 4 km/h looking for a
                    // point on a road it is no longer on. The final leg has always had
                    // this check; the pull leg never did, and that is how a shunt ended
                    // up at GF. Give it a couple of seconds of grace first: the index
                    // reads -1 for a tick or two while a bogie crosses a frog.
                    if (IndexNear(path, track, s) >= e)
                        reachedPivot = true;
                    // Only "lost" until the pivot has been reached. After that, being
                    // off the planned road is the pull doing its job: drawing the tail
                    // clear runs the front past the end of the plan.
                    if (!reachedPivot && IndexNear(path, track, s) < 0)
                    {
                        offPathFor += DriveTick;
                        if (offPathFor > OffPathSeconds)
                        {
                            run.detail = $"{phase}: off the planned route, re-routing";
                            done(true);
                            yield break;
                        }
                    }
                    else
                    {
                        offPathFor = 0f;
                    }
                }

                if (!lastLeg)
                {
                    // Pull forward only until the tail has cleared every junction the
                    // reverse will retrace through (all the throat frogs on this leg,
                    // plus the reversal pivot). Once none of those is fouled by our own
                    // cars, they can all be thrown for the reverse — no need to drag the
                    // whole train out onto the main line just to sit it on one track.
                    // What actually gates the reverse is the junctions it will retrace:
                    // once none of them is fouled by our own cars they can all be
                    // thrown. It is NOT "the whole train is off the track we started
                    // on" — after a re-plan that track is simply whatever the train is
                    // standing on, and if that is a running line the pull never ends and
                    // the train disappears down the main. Only when the leg has no
                    // junction at all does leaving the start track become the condition.
                    bool clear = true;
                    bool anyJunction = false;
                    foreach (var j in JunctionsToClear(path, s, e, startTrack))
                    {
                        anyJunction = true;
                        if (ConsistFoulsJunction(j, car.trainset))
                        {
                            clear = false;
                            break;
                        }
                    }
                    if (!anyJunction)
                        clear = AllOffTrack(car, startTrack);
                    // The whole train has to be onto the reversal track before it can
                    // change ends, or the reverse just takes it back down the branch it
                    // arrived on. Must agree with PullRemaining, which plans the crawl
                    // to this same point.
                    if (clear && BehindPivotShortfall(path, s, e, car) > 0)
                        clear = false;
                    // A pull leg has TWO jobs: get the train to the track it reverses on,
                    // and draw the tail clear of the junctions the reverse retraces.
                    // Testing only the clearance ended the leg on its first tick whenever
                    // the train was not already fouling those junctions — which is the
                    // normal case — so the leg finished, the outer loop re-planned the
                    // same leg, and the train sat there being re-planned two hundred
                    // times without ever being given power.
                    // Reached the reversal track — once, at any point. Requiring the
                    // train to still BE on it is wrong: the last car only clears the
                    // junction when the front has run out beyond, and the test then
                    // never passed, so the leg was abandoned and re-planned into the
                    // same pull for ever.
                    clear = clear && reachedPivot;
                    if (clear)
                    {
                        // Done pulling — return WITHOUT braking. The outer loop re-routes
                        // and only brings the train to a stop if the next leg reverses;
                        // a same-direction continuation just rolls straight through
                        // (no more full-braking at every junction).
                        done(true);
                        yield break;
                    }
                    // Ease down as the clear point comes up. Running flat out at
                    // ManeuverKmh until the check fires meant stopping FROM that speed
                    // afterwards, which dragged the train far past the junction it had
                    // just cleared — the reverse then started from way down the line.
                    // Whichever is further: the run still to make to the reversal track,
                    // or the extra the tail needs to clear the junctions there.
                    double toPivot = DistanceToIndex(path, track, bogie.traveller.Span, towardOut, e);
                    double toClear = Math.Max(
                        toPivot == double.MaxValue ? 0 : toPivot,
                        PullRemaining(car, path, s, e, startTrack));
                    float pullTarget = ApproachSpeedKmh(
                        PullCrawlKmh, toClear - PullStopMargin, planDecel);
                    // The yard-pace cap belongs to SHUNTS. A road move reversing at
                    // a far junction is still a road move on the way there — 9 km
                    // of running line at 25 km/h was most of an hour; the approach
                    // curve above brings it down in good time regardless.
                    float pullCap = run.mode == AutopilotMode.Shunt
                        ? Mathf.Min(run.profile.maneuverKmh, pullTarget)
                        : pullTarget;
                    target = Mathf.Min(Mathf.Min(cruiseKmh, pullCap), limit);
                    airStop = toClear < ApproachDistance;
                    run.detail = $"{phase}: pulling clear, {toClear:F0} m";
                }
                else if (run.couple && !run.coupledDone && nearDest)
                {
                    airStop = true;
                    // Buffer up to whatever is standing on the destination. The train we
                    // are joining is an obstacle to every other part of this class, so
                    // the guard below is told to allow contact instead of holding off.
                    if (Coupled(run))
                    {
                        // Let Drive pick it up as a spot; braking here would only have
                        // to be undone a tick later.
                        done(true);
                        yield break;
                    }

                    bool inRange = remote != null
                        && remote.IsCouplerInRange(ExternalCouplingHandler.COUPLING_RANGE);
                    if (inRange)
                    {
                        // Close enough for the coupler to take: stop shoving and hook on.
                        overrider.Throttle?.Set(0f);
                        overrider.Brake?.Set(1f);
                        throttle = 0f;
                        controller.RemoteControllerCouple();
                        run.detail = $"{phase}: coupling to the train on {destId}";
                        yield return WaitForSecondsRealtime(0.5f);
                        continue;
                    }

                    double toTrain = ObstacleDistanceRail(
                        car, track, bogie.traveller.Span, towardOut, CoupleApproach * 4, out _, out _, out _);
                    // "Couple to the train there" with nothing there: chasing a
                    // train that does not exist drove the spot clean off the far
                    // end of the siding, "looking for the train" for ever. Bare
                    // destination = this is an ordinary spot from here on.
                    if (toTrain == double.MaxValue
                        && (run.destTrack == null
                            || !Routing.HasForeignCars(run.destTrack, car.trainset)))
                    {
                        run.couple = false;
                        run.detail = $"{phase}: nothing on {destId} to couple to — spotting instead";
                        continue;
                    }
                    target = toTrain <= CoupleApproach
                        ? Mathf.Min(cruiseKmh, CoupleKmh)
                        : Mathf.Min(Mathf.Min(cruiseKmh, SpotApproachKmh), limit);
                    run.detail = toTrain == double.MaxValue
                        ? $"{phase}: looking for the train on {destId}"
                        : $"{phase}: buffering up, {toTrain:F0} m";
                }
                else
                {
                    // Stop exactly when the GAME counts the whole consist as fully on
                    // the destination — every car between the track's fouling points,
                    // clear of the switch. That is the in-game clearance marker, not a
                    // guessed distance, so the loco never ends up fouling the throat.
                    if (ConsistClearWithinDest(car, destId, run.destTrack))
                    {
                        target = 0f;
                        if (speed < 0.8f) { done(true); yield break; }
                        run.detail = $"{phase}: clear within {destId}, stopping";
                    }
                    else
                    {
                        // Run at line speed while the destination is still far off, ease
                        // to approach speed over the last stretch, then crawl once a car
                        // is actually on the destination so the spot doesn't overshoot.
                        // Without the distance term every run — including a haul across
                        // the map — was driven at approach speed the whole way.
                        bool onDest = AnyCarOnTrack(car, destId);
                        double toGo = DistanceToDest(path, track, bogie.traveller.Span, towardOut);
                        // Off the planned road — the pull followed live switches
                        // somewhere the plan did not go. Distance-to-destination is
                        // then meaningless (it read as double.MaxValue, and the train
                        // would have run on at line speed until a car happened to
                        // reach the destination), so re-plan from where we actually
                        // are instead of guessing.
                        if (!onDest && toGo == double.MaxValue)
                        {
                            run.detail = $"{phase}: off the planned route, re-routing";
                            done(true);
                            yield break;
                        }
                        // A road move ends by pulling up on a running line, not by
                        // being tucked between two markers, so it is not held to a
                        // shunter's pace over the last stretch. On a 600 m main that
                        // difference is the better part of two minutes.
                        bool road = run.mode == AutopilotMode.Drive;
                        float crawl = road ? DriveSpotKmh : SpotKmh;
                        float approach = road ? DriveApproachKmh : SpotApproachKmh;
                        if (onDest)
                        {
                            // Plan the spot by the distance still to run, not a flat
                            // crawl. Dropping to 5 km/h the moment the first wagon
                            // touched the destination meant creeping the whole length
                            // of a 300 m siding — minutes of it.
                            double toSpot = run.destTrack != null
                                ? DistanceToSpot(car, run.destTrack)
                                : 0;
                            airStop = true;
                            // Overshoot is measured, not inferred: the spotted set
                            // wholly past the marker window. The turntable pockets
                            // put both markers within a metre, so a slightly late
                            // stop is already "past" — full brakes the moment it
                            // reads, then a small miss is left where it stands and
                            // a long one is backed onto the marks at a crawl.
                            // Direction on the destination is read while moving and
                            // remembered — a train stood past the marks is not moving,
                            // but which way it was going decides which marker counts.
                            // FROZEN while backing: the overrun is measured against
                            // the direction the spot came in by, not the creep back.
                            if (run.destTrack != null && !backingUp)
                                spotTowardHigh = SpotDirection(car, run.destTrack) ?? spotTowardHigh;
                            double overrun = run.destTrack != null && spotTowardHigh != null
                                ? SpotOverrun(car, run.destTrack, spotTowardHigh.Value)
                                : 0;
                            if (backingUp)
                            {
                                if (overrun <= 0.1)
                                {
                                    target = 0f;
                                    if (speed < 0.5f)
                                    {
                                        overrider.Throttle?.Set(0f);
                                        overrider.Brake?.Set(1f);
                                        Finish(run, "arrived", $"{destId} — backed onto the marks");
                                        done(false);
                                        yield break;
                                    }
                                    run.detail = $"{phase}: on the marks, stopping";
                                }
                                else
                                {
                                    target = Mathf.Min(SpotKmh,
                                        ApproachSpeedKmh(0f, overrun, StopDecel));
                                    run.detail = $"{phase}: backing onto the marks, {overrun:F0} m";
                                }
                            }
                            else if (overrun > OvershootSlack)
                            {
                                target = 0f; // full brakes, now
                                if (speed < 0.5f)
                                {
                                    if (overrun <= OvershootAcceptMeters)
                                    {
                                        overrider.Throttle?.Set(0f);
                                        overrider.Brake?.Set(1f);
                                        Finish(run, "arrived",
                                            $"{destId} — {overrun:F0} m past the marks, left there");
                                        done(false);
                                        yield break;
                                    }
                                    // Too far past to leave: change ends and creep back.
                                    reverser = 1f - reverser;
                                    overrider.DynamicBrake?.Set(0f);
                                    overrider.Reverser?.Set(reverser);
                                    backingUp = true;
                                }
                                run.detail = $"{phase}: overshot {destId} by {overrun:F0} m — stopping";
                            }
                            else
                            {
                                // Air does this stop, so plan it at the air's rate — the
                                // gentle dynamic curve started the crawl far too early
                                // AND still drifted past the marks it was aimed at.
                                // The last metres go at walking pace: a marker window
                                // barely a metre wide cannot be landed at 5 km/h.
                                float creep = toSpot < FinalCreepZone ? FinalCreepKmh : crawl;
                                float planned = ApproachSpeedKmh(creep, toSpot - ParkMargin, StopDecel);
                                target = Mathf.Min(Mathf.Min(cruiseKmh, approach), planned);
                                run.detail = road
                                    ? $"{phase}: pulling up on {destId}, {toSpot:F0} m"
                                    : $"{phase}: spotting into {destId}, {toSpot:F0} m";
                            }
                        }
                        else if (toGo <= ApproachDistance)
                        {
                            target = Mathf.Min(cruiseKmh, approach);
                            airStop = true;
                            run.detail = $"{phase}: approaching {destId}";
                        }
                        else
                        {
                            // Plan the slow-down: be down to approach speed by the time
                            // the last ApproachDistance metres begin.
                            float planned = ApproachSpeedKmh(
                                approach, toGo - ApproachDistance, planDecel);
                            target = Mathf.Min(cruiseKmh, planned);
                            run.detail = $"{phase}: running to {destId}, {toGo:F0} m";
                        }
                        target = Mathf.Min(target, limit);
                    }
                }

                // Never run into anything. Take the tighter of two independent checks:
                // a straight-line world proximity, and a curve-safe walk along the
                // actual rail ahead (which the world check misses around bends). Cap
                // speed to brake short, hard-stop if close.
                //
                // Look as far ahead as it would take to stop from the speed we are
                // doing. A fixed 80 m was fine while everything crawled at yard speed,
                // but at line speed a train needs the better part of a kilometre — and
                // a stationary train on the line ahead went unseen until far too late.
                float vMs = speed / 3.6f;
                double lookahead = Math.Min(
                    MaxObstacleLookahead,
                    Math.Max(80.0, vMs * vMs / (2f * ObstacleDecel) + ObstacleBuffer * 4));
                // Look ahead from the FRONT CAR of the train, not from the loco:
                // with a long rake leading, a walk from the loco through the SET
                // switches can diverge from where the wagons actually lie and
                // never see the stock they are about to hit.
                var lead = LeadingEnd(car, reverser >= 0.5f);
                Vector3 lookMotion = lead != null
                    ? (lead.Value.forward ? 1f : -1f) * lead.Value.car.transform.forward
                    : motion;
                double railAhead = lead != null
                    ? ObstacleDistanceRail(
                        car, lead.Value.bogie.track, lead.Value.bogie.traveller.Span,
                        lead.Value.towardOut, lookahead,
                        out var corridor, out double ownLead, out double railEnd)
                    : ObstacleDistanceRail(
                        car, track, bogie.traveller.Span, towardOut, lookahead,
                        out corridor, out ownLead, out railEnd);
                double clearAhead = Math.Min(
                    ObstacleDistanceWorld(car, lookMotion, corridor, lookahead,
                        lead != null ? lead.Value.car : null),
                    railAhead);

                // No rail, no driving: brake to a stand short of any end the
                // connected track ahead comes to — an unaligned turntable, a stub
                // with no buffer. This is what actually keeps wheels out of the
                // pit; everything below only knows about CARS ahead.
                if (railEnd != double.MaxValue)
                {
                    float vEnd = Mathf.Sqrt(
                        2f * ObstacleDecel * (float)Math.Max(0, railEnd - RailEndMargin)) * 3.6f;
                    if (vEnd < target)
                        target = vEnd;
                    if (railEnd < RailEndMargin + 3)
                    {
                        // Close enough. A 2 km/h "creep target" here had ApplyThrottle
                        // winding the handle to full against a train it had just braked
                        // — wheels blazing, buffers on the end of the world. Stop means
                        // STOP.
                        target = 0f;

                        // Stopped at the end of the rail ON the destination: that IS
                        // the spot, wherever the markers think the train should be —
                        // the marker depth cannot be bought on a track this short.
                        // A via move must still have come in by its demanded end.
                        if (lastLeg && speed < 0.5f && !run.couple
                            && (!run.viaLocked
                                || (path.Count > 0 && path[path.Count - 1] == run.destTrack
                                    && EntryEndIsIn(path, run.destTrack) == run.arriveEnteredIn))
                            && (LocoOnTrack(car, destId) || AnyCarOnTrack(car, destId)))
                        {
                            overrider.Throttle?.Set(0f);
                            overrider.Brake?.Set(1f);
                            Finish(run, "arrived", $"{destId} — pulled up at the end of the rail");
                            done(false);
                            yield break;
                        }

                        if (speed < 0.5f)
                        {
                            // The rail ran out at a turntable. Two shapes of that:
                            // standing at a pit edge with the bridge wound away —
                            // ask for the bridge; or standing ON the bridge with
                            // the exit not lined up — wind it round to the next
                            // track of the plan. Anywhere else the end is a wall
                            // and holding is all there is.
                            if (!bridgeAsked)
                            {
                                var onBridge = Turntables.AsBridge(track);
                                int i = IndexNear(path, track, s);
                                var leadB = lead; // rail ends ahead of the FRONT
                                // Only turn under our own train when EVERY wheel is
                                // on the bridge — a car hanging over the joint gets
                                // dragged off its rails by the turn.
                                if (onBridge != null && i >= 0 && i + 1 < path.Count
                                    && AllOnTrack(car, track))
                                {
                                    Turntables.RotateBridgeTo(onBridge, path[i + 1]);
                                    bridgeAsked = true;
                                }
                                else if (leadB != null)
                                {
                                    bridgeAsked = Turntables.RequestBridgeAhead(
                                        leadB.Value.bogie.track, leadB.Value.bogie.traveller.Span,
                                        leadB.Value.towardOut, TurntableSlowZone + ownLead);
                                }
                                else
                                {
                                    bridgeAsked = Turntables.RequestBridgeAhead(
                                        track, bogie.traveller.Span, towardOut,
                                        TurntableSlowZone + ownLead);
                                }
                            }
                            run.detail = bridgeAsked
                                ? $"{phase}: waiting for the turntable bridge"
                                : $"{phase}: holding — rail ends {railEnd:F0} m ahead";
                        }
                    }
                }

                // Turntables are crossed at walking pace, aligned or not: within
                // TurntableSlowZone of a bridge (or standing on one), cap hard —
                // a consist arriving at yard speed cannot stop on a 20 m bridge.
                // Drive to the end of held rail like a signal at danger: full
                // speed with authority in hand, braking curve into the boundary,
                // stand at it, roll again the moment the next block grants.
                if (cachedAuthority != double.MaxValue)
                {
                    float vAuth = ApproachSpeedKmh(
                        0f, cachedAuthority - SignalMargin, StopDecel);
                    if (vAuth < target)
                        target = vAuth;
                    if (cachedAuthority <= SignalMargin + 2 && speed < 0.5f
                        && cachedAuthorityAt.Length > 0)
                    {
                        run.detail = $"{phase}: held short of {cachedAuthorityAt}"
                            + (cachedAuthorityHolder.Length > 0
                                ? $" - {cachedAuthorityHolder} has it" : "");
                        // A live run ahead releases its blocks as it moves - wait.
                        // Parked stock fouling the block never will: after a while
                        // write that road off and go round.
                        bool liveHolder = false;
                        foreach (var r in runs.Values)
                            if (r.coroutine != null && r.id == cachedAuthorityHolder)
                            {
                                liveHolder = true;
                                break;
                            }
                        if (!liveHolder)
                        {
                            authorityStallFor += DriveTick * LimitRecheckTicks;
                            if (authorityStallFor >= BlockedGiveUpSeconds
                                && cachedAuthorityBlocked != null)
                            {
                                if (run.avoid.Add(cachedAuthorityBlocked))
                                {
                                    run.detail = $"{phase}: {cachedAuthorityAt} is fouled"
                                        + " - looking for another way round";
                                    done(true);
                                    yield break;
                                }
                                Finish(run, "error",
                                    $"{cachedAuthorityAt} is fouled and there is no way round");
                                done(false);
                                yield break;
                            }
                        }
                    }
                    else
                    {
                        authorityStallFor = 0f;
                    }
                }

                bool nearBridge = lead != null
                    ? Turntables.NearBridge(
                        car, lead.Value.bogie.track, lead.Value.bogie.traveller.Span,
                        lead.Value.towardOut, TurntableSlowZone + ownLead)
                    : Turntables.NearBridge(
                        car, track, bogie.traveller.Span, towardOut, TurntableSlowZone + ownLead);
                if (nearBridge)
                    target = Mathf.Min(target, TurntableKmh);
                // On a coupling run the train ahead is the target, not a hazard, so the
                // guard closes right up to buffer contact instead of holding off.
                double buffer = run.couple && lastLeg ? CoupleBuffer : ObstacleBuffer;
                if (clearAhead != double.MaxValue)
                {
                    float vObstacle = Mathf.Sqrt(2f * ObstacleDecel * (float)Math.Max(0, clearAhead - buffer)) * 3.6f;
                    if (run.couple && lastLeg)
                        vObstacle = Mathf.Max(vObstacle, CoupleKmh); // creep on regardless
                    if (vObstacle < target)
                        target = vObstacle;
                    if (clearAhead < buffer)
                    {
                        overrider.Throttle?.Set(0f);
                        overrider.Brake?.Set(1f);
                        throttle = 0f;
                        stuckFor = 0f;
                        run.detail = $"{phase}: HOLDING - car {clearAhead:F0} m ahead ({blockedFor:F0}s)";
                        // Two autopilots can block each other outright: ours is stopped by
                        // their cars, theirs is stopped waiting for rail we hold. Neither
                        // can ever move, because each is the other's reason for standing
                        // still. Interlocking prevents the collision but has no rule for
                        // this, so one of them must give way — the run that has been
                        // holding longest hands its rail back and looks for another road.
                        blockedFor += DriveTick;
                        if (blockedFor >= ClaimDeadlockSeconds && BlockedByWaitingRun(run, corridor))
                        {
                            ReleaseClaims(run);
                            run.detail = $"{phase}: deadlocked with another autopilot — giving way";
                            done(true); // hand back to Drive, which re-plans from here
                            yield break;
                        }
                        // A train that is simply passing is not an obstruction, and the
                        // clock must not run on it: waiting thirty seconds and then
                        // hunting for another way round is how a shunt that needed to
                        // stand at the throat for a minute ended up crossing the map.
                        if (MovingStockAhead(corridor, car.trainset))
                        {
                            blockedFor = 0f;
                            run.detail = $"{phase}: holding for a train ahead to pass";
                            continue;
                        }

                        // The end of the line for a finished move: on the final leg,
                        // not coupling, stood against stock that is not moving, with
                        // part of the train already on the destination - that IS the
                        // arrival. Park here, short of what stands ahead, and hand the
                        // road back so the queue behind can stack in the same way.
                        bool roadComplete = AnyCarOnTrack(car, destId)
                            || (run.destTrack != null
                                && ForeignStockOn(run.destTrack, car.trainset));
                        // A via move only ever arrives by its demanded end: parking
                        // short from the WRONG side is not arrival, it is the via
                        // being quietly thrown away.
                        bool viaOk = !run.viaLocked
                            || (path.Count > 0 && path[path.Count - 1] == run.destTrack
                                && EntryEndIsIn(path, run.destTrack) == run.arriveEnteredIn);
                        if (lastLeg && !run.couple && speed < 0.5f
                            && blockedFor >= 5f
                            && roadComplete && viaOk
                            && !AutopilotStockAhead(corridor, car.trainset))
                        {
                            overrider.Throttle?.Set(0f);
                            overrider.Brake?.Set(1f);
                            Finish(run, "arrived",
                                destId + " - pulled up behind the stock standing there");
                            done(false);
                            yield break;
                        }

                        // Stock that is not going anywhere. The route was planned through
                        // it — routing falls back to an occupancy-blind path when nothing
                        // clear exists, and the guard then refuses to drive it — so the
                        // run would sit here for ever saying "HOLDING". Try once to go
                        // round; if there is no way round, say so and stop.
                        if (blockedFor >= BlockedGiveUpSeconds)
                        {
                            overrider.Throttle?.Set(0f);
                            overrider.Brake?.Set(1f);
                            var here = track.LogicTrack()?.ID?.ToString() ?? "the line";
                            if (run.avoid.Add(track))
                            {
                                run.detail = $"{phase}: cars ahead on {here} — looking for another way round";
                                done(true);
                                yield break;
                            }
                            Finish(run, "error",
                                $"blocked by cars on {here} and there is no way round — clear them and start again");
                            done(false);
                            yield break;
                        }
                        continue;
                    }
                    blockedFor = 0f;
                }

                // Extend the reservation as we advance: all-or-nothing, so a stretch
                // held by another train simply stays unclaimed and the check below
                // brings us to a stand short of it.
                int pathIdx = IndexNear(path, track, s);
                if (pathIdx >= 0)
                {
                    run.pathFrom = pathIdx; // rail before this is behind us now
                    ReleaseClaimsBehind(run, path, pathIdx);
                }

                // Switches on the road ahead: correct them while they are still far
                // enough away, and stop short of any that cannot be corrected.
                if (!SwitchesAheadOk(run, path, pathIdx, track,
                        bogie.traveller.Span, towardOut, ownLead,
                        out var badDist, out var badReason, out var blockedTrack))
                {
                    float vSwitch = Mathf.Sqrt(
                        2f * ObstacleDecel * (float)Math.Max(0, badDist - ObstacleBuffer)) * 3.6f;
                    if (vSwitch < target)
                        target = vSwitch;
                    if (badDist < ObstacleBuffer)
                    {
                        overrider.Throttle?.Set(0f);
                        overrider.Brake?.Set(1f);
                        throttle = 0f;
                        stuckFor = 0f;
                        switchHold += 0.1f;
                        // Waiting is right while a wagon sits on the switch, but not for
                        // ever: after a while, write that road off and let the router
                        // find another one.
                        if (blockedTrack != null && switchHold > SwitchDetourSeconds)
                        {
                            run.avoid.Add(blockedTrack);
                            run.blockedReason = badReason;
                            run.detail = $"{phase}: {badReason} — looking for another way round";
                            done(true); // hand back to Drive, which re-plans
                            yield break;
                        }
                        run.detail = $"{phase}: HOLDING — {badReason} ({badDist:F0} m)";
                        continue;
                    }
                    run.detail += $" [{badReason} in {badDist:F0} m]";
                }

                // Back off before an engine cooks. Read the temperature ports of EVERY
                // loco in the consist directly: the game's own observer
                // (GetEngineTemperatureState) disables itself unless the player is in
                // that particular cab or a remote is paired to it, so on an autopilot
                // run — and on every trailing MU unit even with the player up front —
                // it reports Nominal while the engine boils.
                var (tempWarning, tempCritical) = EngineTempState(car);
                // The traction motors run their own separate heat budget and blow
                // their own fuse (DeadTractionMotorsController) with no relation to
                // the engine temperature, so a heavy pull could trip that fuse with
                // the throttle logic none the wiser.
                var (tmWarning, tmCritical) = TmOverheatState(car);
                ApplyThrottle(controller, ref throttle, car, target, speed,
                    run.profile, ref lastSpeedMs, ref accelFilter, ref run.accelBudget,
                    ref sanderHold, ref dynStallFor,
                    // The dynamic is a ROAD tool: long descents, speed
                    // restrictions at line speed. A shunt never gets fast enough
                    // for it to bite and just wants the air — and even on the
                    // road, a planned STOP is the air's job (see airStop).
                    run.mode == AutopilotMode.Drive && !airStop,
                    tempWarning || tmWarning,
                    tempCritical || tmCritical);

                run.limitKmh = limit;
                float signedSpeed = car.GetForwardSpeed() * 3.6f;
                run.detail += $" [{track.LogicTrack()?.ID} rev{reverser:F0} v{signedSpeed:+0;-0;0} "
                    + $"tgt{target:F0} lim{limit:F0} th{throttle:F2} clr{(clearAhead == double.MaxValue ? 999 : clearAhead):F0}"
                    + $" auth{(cachedAuthority == double.MaxValue ? 9999 : cachedAuthority):F0}]";

                // Stuck watchdog only while we actually want to move.
                if (speed < 0.5f && target > 2f)
                {
                    stuckFor += 0.1f;
                    // "Stuck" only counts once the train has actually been given the
                    // full handle and STILL will not move. A heavy rake on a grade
                    // spends its first seconds working the throttle up; calling that
                    // stuck abandoned trains that had never been given power at all.
                    //
                    // It also does not count while the brakes are still letting go. A
                    // brake pipe takes its time to recharge after any application — a
                    // stop, or simply a mod reload — and a train being held by its own
                    // brakes looks exactly like one that cannot move.
                    float pipe = car.brakeSystem?.brakePipePressure ?? BrakeReleasedBar;
                    bool brakesHolding = pipe < BrakeReleasedBar;
                    fullPowerFor = throttle >= 0.95f && !brakesHolding ? fullPowerFor + 0.1f : 0f;
                    if (brakesHolding)
                        run.detail = $"{phase}: waiting for the brakes to release ({pipe:F1} bar)";
                    if (run.coupledDone && stuckFor > 10f)
                    {
                        // The train is coupled and being shoved in, but has stopped
                        // moving: the siding has no more room. That is a finished job
                        // with a caveat, not a failure.
                        overrider.Throttle?.Set(0f);
                        overrider.Brake?.Set(1f);
                        Finish(run, "coupled",
                            $"coupled, but {destId} has no room for the whole consist");
                        done(false);
                        yield break;
                    }
                    if (fullPowerFor > StuckAtFullPowerSeconds || stuckFor > StuckGiveUpSeconds)
                    {
                        overrider.Throttle?.Set(0f);
                        overrider.Brake?.Set(1f);
                        Finish(run, "error", fullPowerFor > StuckAtFullPowerSeconds
                            ? "stuck — full power will not move it (handbrake, grade or obstruction)"
                            : "stuck — check direction or obstruction");
                        done(false);
                        yield break;
                    }
                }
                else
                {
                    // Decay rather than reset: a train that crawls a metre and stalls
                    // again on the same grade should keep the power it has worked up.
                    stuckFor = Mathf.Max(0f, stuckFor - 0.2f);
                    fullPowerFor = 0f;
                }
            }
        }
    }
}
