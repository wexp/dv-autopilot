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
    /// <summary>The hands on the controls: throttle, air and dynamic braking,
    /// slip and temperature protection, and how a finished run ties the train
    /// down.</summary>
    public static partial class Autopilot
    {
        /// <summary>Wind off the handbrakes on the wagons of this consist. A parked rake
        /// is left with them on, and no amount of throttle moves a train that is still
        /// tied down — it just reads as "stuck". The locos are left alone: their own
        /// brakes are the overrider's business.</summary>
        private static void ReleaseHandbrakes(TrainCar loco)
        {
            var cars = loco.trainset?.cars;
            if (cars == null)
                return;
            foreach (var c in cars)
            {
                if (c == null || c.IsLoco)
                    continue;
                var brakes = c.brakeSystem;
                if (brakes != null && brakes.hasHandbrake && brakes.handbrakePosition > 0f)
                    brakes.SetHandbrakePosition(0f);
            }
        }

        /// <summary>
        /// The fastest we may be running NOW to be down to endKmh in this many metres,
        /// braking at decel — v = sqrt(v_end² + 2·a·d).
        ///
        /// Every planner used to write endKmh + sqrt(2·a·d), which is not the same
        /// thing and is always too fast: the square root alone is already the most you
        /// can be doing and still make the stop, so adding the end speed on top
        /// guarantees running past it. That is why spots were overshot and why a train
        /// arrived at a junction too fast to take it.
        /// </summary>
        private static float ApproachSpeedKmh(float endKmh, double distance, float decel)
        {
            float endMs = endKmh / 3.6f;
            float d = (float)Math.Max(0, distance);
            return Mathf.Sqrt(endMs * endMs + 2f * decel * d) * 3.6f;
        }

        private static float MaxSafeAccel(TrainCar loco)
        {
            float drivenKg = 0, trainKg = 0;
            foreach (var c in loco.trainset.cars)
            {
                float kg = c.massController?.TotalMass ?? 0f;
                trainKg += kg;
                if (c.IsLoco)
                    drivenKg += kg;
            }
            if (trainKg <= 0f || drivenKg <= 0f)
                return 0.5f;
            return AdhesionCoefficient * Gravity * drivenKg / trainKg;
        }

        /// <summary>
        /// Engine (or transmission — the port is whatever drives the loco's own hot
        /// gauge, coolant.TEMPERATURE on the diesels) temperature state, read off the
        /// simulation ports of EVERY loco in the consist against each one's own
        /// serialized red-line thresholds (DH4: 90 warning / 105 critical). The
        /// TrainCar API for this, GetEngineTemperatureState, cannot be trusted: its
        /// observer disables itself unless the player is in that cab or a remote is
        /// paired to it, and even when it runs it only ever answers for its own car —
        /// a trailing MU unit boils in silence either way.
        /// </summary>
        private static (bool warning, bool critical) EngineTempState(TrainCar loco)
        {
            bool warning = false, critical = false;
            foreach (var c in loco.trainset.cars)
            {
                if (c == null || !c.IsLoco)
                    continue;
                var observer = c.GetComponentInChildren<MultipleUnitStateObserver>();
                var simFlow = c.SimController?.simFlow;
                if (observer == null || simFlow == null
                    || string.IsNullOrEmpty(observer.temperaturePortId)
                    || !simFlow.TryGetPort(observer.temperaturePortId, out var port, true))
                    continue;
                float t = port.Value;
                warning |= t >= observer.overheatStandardThreshold;
                critical |= t >= observer.overheatCriticalThreshold;
            }
            return (warning, critical);
        }

        /// <summary>Worst wheelslip across every loco in the consist. The lead unit
        /// alone is a lie in an MU lash-up: the trailing units get the same throttle
        /// and can break loose first — and did, unseen.</summary>
        private static float MaxSlip(TrainCar loco)
        {
            float slip = 0f;
            foreach (var c in loco.trainset.cars)
                if (c != null && c.IsLoco)
                    slip = Mathf.Max(slip, c.SimController?.wheelslipController?.wheelslip ?? 0f);
            return slip;
        }

        /// <summary>
        /// TM (traction motor) overheat, read straight off the loco's own simulation
        /// ports rather than any TrainCar API — there isn't one. tmOverheatLamp mirrors
        /// the cab warning lamp (pre-fuse, still recoverable); OVERHEAT_POWER_FUSE_OFF
        /// is the fuse having actually blown, cutting power outright. Locos without a
        /// TM (steam, hydraulic) simply lack these ports and read as never overheating.
        /// Checked on every loco of the consist, like the engine temperature.
        /// </summary>
        private static (bool warning, bool critical) TmOverheatState(TrainCar loco)
        {
            bool warning = false, critical = false;
            foreach (var c in loco.trainset.cars)
            {
                if (c == null || !c.IsLoco)
                    continue;
                var simFlow = c.SimController?.simFlow;
                if (simFlow == null)
                    continue;
                warning |= simFlow.TryGetPort("tmOverheatLamp.LAMP_STATE", out var lamp, true)
                    && lamp.Value > 0.5f;
                critical |= simFlow.TryGetPort("tm.OVERHEAT_POWER_FUSE_OFF", out var fuse, true)
                    && fuse.Value > 0.5f;
            }
            return (warning, critical);
        }

        private static void ApplyThrottle(
            RemoteControllerModule controller, ref float throttle, TrainCar car,
            float target, float speed, LocoProfile profile,
            ref float lastSpeedMs, ref float accelFilter, ref float accelBudget,
            ref float sanderHold, ref float dynStallFor, bool allowDynamic,
            bool tempWarning, bool tempCritical)
        {
            var overrider = controller.controlsOverrider;

            // A commanded stop needs a firm brake — the proportional touch below is
            // far too weak to hold 200+ t and just leaves the train creeping.
            if (target < 0.8f)
            {
                throttle = 0f;
                overrider.Throttle?.Set(0f);
                overrider.DynamicBrake?.Set(0f);
                overrider.Brake?.Set(1f);
                return;
            }

            float slip = MaxSlip(car);
            float error = target - speed;
            // Actual acceleration, m/s². Measured over one 0.1 s tick this is mostly
            // noise — couplers surging, the speed itself quantised — and reacting to
            // that noise pinned the throttle around half power for ever. Smooth it.
            float speedMs = speed / 3.6f;
            float sample = (speedMs - lastSpeedMs) / DriveTick;
            lastSpeedMs = speedMs;
            accelFilter += (sample - accelFilter) * AccelSmoothing;
            float accelMs2 = accelFilter;

            // The acceleration this train is allowed to make. It starts at whatever the
            // weight on the driven wheels can hold and is CUT every time the wheels
            // actually slip, then eased back. The adhesion sum is only an estimate — μ
            // is a guess and the rail may be wet — so the slip itself gets the last
            // word. Taken from DVRouteManager's LocoAI, which drives off a measured
            // budget like this rather than a computed one.
            float safeAccel = MaxSafeAccel(car);
            if (slip > SlipThreshold)
                accelBudget = Mathf.Max(MinAccelBudget, accelBudget - SlipAccelCut * DriveTick);
            else
                accelBudget = Mathf.Min(safeAccel, accelBudget + AccelRecover * DriveTick);

            // How hard we actually WANT to accelerate right now: enough to close the
            // remaining speed gap over the next few seconds, and never more than the
            // wheels allow. Without this the handle stayed wide open until the target
            // was already reached — the budget alone says nothing about how much speed
            // is left to gain, so the train blew straight through its limit.
            float wantedAccel = Mathf.Min(accelBudget, (error / 3.6f) / ApproachSeconds);

            // Sand while the wheels are slipping — what a driver's thumb would be
            // doing — and keep it flowing a few seconds past the last slip: the
            // rail that broke them loose is still under the train, and chasing the
            // flickering slip signal clicked the sander on/off every tick.
            if (slip > SlipThreshold)
                sanderHold = SanderHoldSeconds;
            else
                sanderHold = Mathf.Max(0f, sanderHold - DriveTick);
            overrider.Sander?.Set(sanderHold > 0f ? 1f : 0f);

            if (slip > SlipThreshold)
            {
                // A flat SlipBackoff (0.15/tick) cut ~10x faster than the ramp below
                // recovers at (profile.throttleUp, e.g. 0.006-0.025/tick) — any slip
                // lasting more than a few ticks ratcheted the handle to zero and then
                // crawled back up for a second or more, which read as "goes straight to
                // 0" even though each individual cut was modest. Stepping down by the
                // same size the ramp steps up (just doubled, matching the temperature
                // cut below) lets it settle at whatever throttle keeps slip right at
                // the edge instead of overshooting to zero every time.
                //
                // But that gentle step is sized for slip caught at its ONSET. A bad
                // break-away — the DH4's torque converter letting go at a standstill,
                // a wet rail under a heavy pull — spins the wheels far past the
                // threshold, and easing off 0.03/tick left them flailing for seconds.
                // Severe slip gets the full-size cut until it is back under control.
                float cut = slip > SevereSlip ? ThrottleDown : profile.throttleUp * 2f;
                throttle = Mathf.Max(0f, throttle - cut);
                overrider.Throttle?.Set(throttle);
                overrider.DynamicBrake?.Set(0f); // braking slipping wheels locks them
                overrider.Brake?.Set(0f);
            }
            else if (tempCritical)
            {
                // Engine over temperature: come off power whatever the speed error.
                throttle = Mathf.Max(0f, throttle - ThrottleDown);
                overrider.Throttle?.Set(throttle);
                overrider.DynamicBrake?.Set(0f);
                overrider.Brake?.Set(0f);
            }
            else if (tempWarning && throttle > 0f)
            {
                // On the red line but not yet critical: EASE OFF until it cools back
                // under the mark. Holding the handle where it was — the old behaviour
                // — kept a flat-out loco sitting on its red line, still heating,
                // until it crossed into critical and got damaged anyway. Backing off
                // at the ramp rate settles the temperature just under the line and
                // picks the speed back up once it drops.
                throttle = Mathf.Max(0f, throttle - profile.throttleUp);
                overrider.Throttle?.Set(throttle);
                overrider.Brake?.Set(0f);
            }
            else if (error > 1.0f)
            {
                // Gentleness comes from the measured acceleration, not from a cap on the
                // handle. A light engine reaches the budget within a notch or two and is
                // held there; a heavy rake on a grade is allowed the whole handle,
                // because it will never accelerate hard enough to break the wheels
                // loose. The old speed-banded ceiling could not tell those apart, and
                // stalled heavy trains on gradients at a third of the handle.
                float step = profile.throttleUp * (throttle > 0.6f ? 0.5f : 1f);
                if (accelMs2 > wantedAccel * AccelOverBand)
                    throttle = Mathf.Max(0f, throttle - profile.throttleUp * 2f);
                else if (accelMs2 < wantedAccel && !tempWarning)
                    throttle = Mathf.Min(1f, throttle + step);
                // else: inside the band, or held down by engine temperature — hold.
                // A fraction below the loco's coarsest notch does nothing, so at a
                // standstill the floor is allowed — but as a target to ramp TO, not a
                // value to snap to. Setting it outright jumped the handle through the
                // first notches in one tick and only then crawled up.
                if (speed < 1f && throttle < profile.minMoveThrottle)
                    throttle = Mathf.Min(profile.minMoveThrottle, throttle + profile.throttleUp);
                overrider.Throttle?.Set(throttle);
                overrider.DynamicBrake?.Set(0f);
                overrider.Brake?.Set(0f);
            }
            else if (error < -1.0f)
            {
                // Shut off first, then brake — and prefer the DYNAMIC brake where
                // the loco has one: no air to blow off and pump back afterwards, no
                // shoe wear, and it grades far more finely than the pipe. Air still
                // backs it up when the overspeed is serious (a grade the dynamic
                // cannot hold alone) and takes over entirely at crawling speed,
                // where a dynamic fades to nothing.
                throttle = Mathf.Max(0f, throttle - ThrottleDown);
                overrider.Throttle?.Set(throttle);
                float service = Mathf.Min(ServiceBrakeMax, Mathf.Clamp01(-error / 8f));
                var dynamic = overrider.DynamicBrake;
                if (allowDynamic && dynamic != null && speed > DynamicBrakeMinKmh && throttle <= 0.01f)
                {
                    dynamic.Set(Mathf.Clamp01(-error / 10f));
                    // Give the dynamic a fair chance, then stop pretending: if the
                    // train is STILL not slowing after a few seconds of it (heavy
                    // rake, falling grade), blend the air in on top. Air also comes
                    // straight in for a serious overspeed.
                    if (accelMs2 > -0.05f)
                        dynStallFor += DriveTick;
                    else
                        dynStallFor = 0f;
                    bool needAir = -error > DynamicBrakeAirBackupKmh
                        || dynStallFor >= DynamicBrakeStallSeconds;
                    overrider.Brake?.Set(needAir ? service : 0f);
                }
                else
                {
                    dynamic?.Set(0f);
                    dynStallFor = 0f;
                    overrider.Brake?.Set(throttle <= 0.01f ? service : 0f);
                }
            }
            else
            {
                overrider.DynamicBrake?.Set(0f);
                dynStallFor = 0f;
                overrider.Throttle?.Set(throttle);
                overrider.Brake?.Set(0f);
            }
        }

        /// <summary>Metres until the loco has pulled InsertionDepth into the
        /// destination track (leg end e).</summary>
        private static double RemainingToStop(List<RailTrack> path, int idx, int e, RailTrack track, Bogie bogie)
        {
            if (idx >= e)
            {
                // On the destination: how far have we come in from the entry end?
                bool entryFirst = e == 0 || !ExitIsOut(path[e], path[e - 1]);
                double span = bogie.traveller.Span;
                double into = entryFirst ? span : track.GetKinkedPointSet().span - span;
                return Math.Max(0, InsertionDepth - into);
            }

            double curSpan = track.GetKinkedPointSet().span;
            double bogieSpan = bogie.traveller.Span;
            double remaining = ExitIsOut(track, path[idx + 1]) ? (curSpan - bogieSpan) : bogieSpan;
            for (int i = idx + 1; i < e; i++)
                remaining += path[i].GetKinkedPointSet().span;
            remaining += InsertionDepth;
            return remaining;
        }

        /// <summary>Park this run: record how it ended and drop it from the live set,
        /// leaving every other loco's autopilot untouched. The status stays readable
        /// until the next poll by keeping the object itself alive in the caller.</summary>
        private static void Finish(Run run, string status, string info)
        {
            run.state = status;
            run.detail = info;
            run.coroutine = null;
            ReleaseClaims(run); // an arrived or failed run must not hold the junction

            // A finished shunt ties its cut down: handbrake on the first wagon, the
            // way a shunter pins a brake before stepping off. Optionally the loco
            // then cuts itself off, ready for its next job. Deferred until the
            // train actually STANDS — Finish fires the moment the arrival test
            // passes, which is still at rolling speed, and cutting a moving train
            // sent the wagons on without their loco.
            if (run.mode == AutopilotMode.Shunt
                && (status == "arrived" || status == "coupled")
                && run.car != null)
            {
                Updater.Run(TieDownWhenStopped(run, info));
            }
        }

        /// <summary>Wait for the finished shunt to come to a stand, then pin the
        /// first wagon's handbrake and (when asked) cut the loco off.</summary>
        private static IEnumerator TieDownWhenStopped(Run run, string info)
        {
            float waited = 0f;
            while (run.car != null
                && Mathf.Abs(run.car.GetForwardSpeed()) * 3.6f > 0.3f
                && waited < 60f)
            {
                yield return WaitForSecondsRealtime(0.2f);
                waited += 0.2f;
            }
            var car = run.car;
            if (car == null)
                yield break;
            var (wagon, cutAt) = FirstWagonOut(car);
            if (wagon == null)
                yield break;
            var brakes = wagon.brakeSystem;
            if (brakes != null && brakes.hasHandbrake)
                brakes.SetHandbrakePosition(1f);
            if (run.uncoupleOnFinish && cutAt != null)
            {
                // Split the rake wherever the JOB changes before stepping away —
                // the cut just delivered and the strays it was shoved against are
                // different trains the moment the loco leaves, and leaving them
                // coupled welded SU00's wagons to the no-job stock beside them.
                // No-job cars group together; a foreign loco parked in the string
                // is its own cut whatever its job says.
                var cuts = new List<(Coupler cut, TrainCar head)>();
                string? lastJob = JobKeyOf(wagon);
                bool lastLoco = wagon.IsLoco;
                var enter = cutAt.coupledTo;
                int guard = 0;
                while (enter != null && ++guard < 100)
                {
                    var exit = enter.GetOppositeCoupler();
                    var next = exit?.coupledTo?.train;
                    if (exit == null || next == null)
                        break;
                    var job = JobKeyOf(next);
                    if (job != lastJob || next.IsLoco != lastLoco)
                        cuts.Add((exit, next));
                    lastJob = job;
                    lastLoco = next.IsLoco;
                    enter = exit.coupledTo;
                }

                foreach (var (cut, _) in cuts)
                    CouplerLogic.Uncouple(cut);
                CouplerLogic.Uncouple(cutAt);

                // Every piece of the split stands on its own now: one cut-in loco
                // per piece that has one, and a pinned handbrake at the head of
                // each so nothing walks off while unattended.
                EnsureOneBrakeCutIn(car, car);
                EnsureOneBrakeCutIn(wagon);
                foreach (var (_, head) in cuts)
                {
                    var hb = head.brakeSystem;
                    if (hb != null && hb.hasHandbrake)
                        hb.SetHandbrakePosition(1f);
                    EnsureOneBrakeCutIn(head);
                }
                run.detail = info + (cuts.Count > 0
                    ? $" — split into {cuts.Count + 1} cuts by job, loco cut off"
                    : " — handbrake set, loco cut off");
            }
            else
            {
                run.detail = info + " — handbrake set";
            }
        }

        /// <summary>How a car groups for the finishing split: its job id, or null
        /// for stock on no job at all.</summary>
        private static string? JobKeyOf(TrainCar car) => JobData.JobForCar(car)?.ID;

        /// <summary>
        /// The first NON-loco car reachable from this loco, walking the couplers
        /// outward (front side first), and the coupler whose parting separates every
        /// loco unit from the wagons. MU'd units between the loco and the wagons stay
        /// together — the cut lands exactly where the last engine meets the first
        /// wagon.
        /// </summary>
        private static (TrainCar? wagon, Coupler? cutAt) FirstWagonOut(TrainCar loco)
        {
            foreach (var start in new[] { loco.frontCoupler, loco.rearCoupler })
            {
                var coupler = start;
                while (coupler != null && coupler.coupledTo != null)
                {
                    var next = coupler.coupledTo.train;
                    if (next == null)
                        break;
                    if (!next.IsLoco)
                        return (next, coupler);
                    coupler = coupler.coupledTo.GetOppositeCoupler();
                }
            }
            return (null, null);
        }

        /// <summary>Index of track in path, searching outward from a hint so a loop
        /// that revisits a track id still tracks forward progress.</summary>
        private static int IndexNear(List<RailTrack> path, RailTrack track, int hint)
        {
            for (int d = 0; d < path.Count; d++)
            {
                int a = hint + d;
                if (a < path.Count && path[a] == track)
                    return a;
                int b = hint - d;
                if (b >= 0 && path[b] == track)
                    return b;
            }
            return -1;
        }

        /// <summary>Real-time wait so a paused game can't freeze the drive loop.</summary>
        private static WaitForSecondsRealtime WaitForSecondsRealtime(float s) =>
            new WaitForSecondsRealtime(s);

        /// <summary>Is game time stopped (pause menu, alt-tab)? The drive loops wait
        /// in REAL time — a paused game must not freeze the coroutine dead — so
        /// without this every watchdog in them kept counting against a world that
        /// wasn't moving: reads of a frozen train looked "stuck", claims timed out
        /// and were handed away, and runs failed while the player sat in the menu.</summary>
        private static bool GamePaused =>
            DV.AppUtil.Instance != null && DV.AppUtil.Instance.IsTimePaused;

        /// <summary>Idle until game time flows again, without touching any timer.</summary>
        private static IEnumerator WaitWhilePaused()
        {
            while (GamePaused)
                yield return WaitForSecondsRealtime(0.25f);
        }
    }
}
