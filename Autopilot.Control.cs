using DV.Logic.Job;
using DV.MultipleUnit;
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
        /// <summary>Wind off the handbrakes on EVERY car of this consist — wagons
        /// and locos alike. A parked rake is left tied down (our own tie-down pins
        /// one on each delivered cut), and no amount of throttle moves a train
        /// that still has one set — it just reads as "stuck". Checked at every
        /// leg, not once per run: coupling adds cars mid-run and they arrive with
        /// whatever brakes their last shunter left pinned.</summary>
        private static void ReleaseHandbrakes(TrainCar loco)
        {
            var cars = loco.trainset?.cars;
            if (cars == null)
                return;
            foreach (var c in cars)
            {
                if (c == null)
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

        /// <summary>
        /// The locos actually driven from this one: the lead plus every unit
        /// reachable over CONNECTED MU cables. A loco merely coupled into the rake
        /// with no MU cable is freight — its weight puts no grip on the rail for
        /// us, its wheels take no throttle, and its engine stays cold — so every
        /// per-loco read (slip, adhesion mass, temperatures) walks this group, not
        /// IsLoco across the trainset.
        /// </summary>
        internal static HashSet<TrainCar> MuGroup(TrainCar lead)
        {
            var group = new HashSet<TrainCar> { lead };
            var queue = new Queue<TrainCar>();
            queue.Enqueue(lead);
            while (queue.Count > 0)
            {
                var mu = queue.Dequeue().GetComponent<MultipleUnitModule>();
                if (mu == null)
                    continue;
                foreach (var cable in new[] { mu.frontCable, mu.rearCable })
                {
                    var other = cable?.connectedTo?.muModule?.train;
                    if (other != null && group.Add(other))
                        queue.Enqueue(other);
                }
            }
            return group;
        }

        /// <summary>
        /// On this track, do the NON-driven cars (wagons, dead locos) sit toward
        /// its out end relative to the driven units? null when either half has no
        /// bogie on the track. This is what a turntable ride steers by: the bridge
        /// end the wagons occupy is the end a SHOVE must hand to the stall.
        /// </summary>
        private static bool? WagonsTowardOut(TrainCar loco, RailTrack track)
        {
            var driven = MuGroup(loco);
            double wagonSum = 0, drivenSum = 0;
            int wagonN = 0, drivenN = 0;
            foreach (var c in loco.trainset.cars)
            {
                if (c == null)
                    continue;
                foreach (var b in c.Bogies)
                {
                    if (b?.track != track)
                        continue;
                    if (driven.Contains(c)) { drivenSum += b.traveller.Span; drivenN++; }
                    else { wagonSum += b.traveller.Span; wagonN++; }
                }
            }
            if (wagonN == 0 || drivenN == 0)
                return null;
            return wagonSum / wagonN > drivenSum / drivenN;
        }

        /// <summary>Metres from a bogie to the FAR end of the given bridge track,
        /// walking the rail in its direction of travel. MaxValue when the walk
        /// does not reach the bridge within a few tracks.</summary>
        private static double DistanceToBridgeExit(
            RailTrack from, double span, bool towardOut, RailTrack bridge)
        {
            double d = 0;
            RailTrack? t = from;
            bool outw = towardOut;
            double s = span;
            for (int k = 0; k < 4 && t != null; k++)
            {
                double len = t.GetKinkedPointSet().span;
                d += outw ? len - s : s;
                if (t == bridge)
                    return d;
                var br = outw ? t.GetOutBranch() : t.GetInBranch();
                if (br?.track == null)
                    return double.MaxValue;
                t = br.track;
                outw = br.first;
                s = br.first ? 0 : t.GetKinkedPointSet().span;
            }
            return double.MaxValue;
        }

        private static float MaxSafeAccel(TrainCar loco)
        {
            var driven = MuGroup(loco);
            float drivenKg = 0, trainKg = 0;
            foreach (var c in loco.trainset.cars)
            {
                float kg = c.massController?.TotalMass ?? 0f;
                trainKg += kg;
                if (driven.Contains(c))
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
            foreach (var c in MuGroup(loco))
            {
                if (c == null)
                    continue;
                var observer = c.GetComponentInChildren<MultipleUnitStateObserver>();
                var port = Ports.Find(c, observer?.temperaturePortId);
                if (observer == null || port == null)
                    continue;
                float t = port.Value;
                warning |= t >= observer.overheatStandardThreshold;
                critical |= t >= observer.overheatCriticalThreshold;
            }
            return (warning, critical);
        }

        /// <summary>
        /// Update the ceiling the heat puts on the throttle, 0..1, against the
        /// hottest driven unit's own warning threshold.
        ///
        /// This is a SETTLING controller, not a thermostat. Cutting power on the
        /// red line and restoring it as soon as the reading dipped is what made
        /// the loco cycle — buzz, throttle down, cool, throttle up, buzz — for the
        /// whole journey, because full power always leads back to the same line.
        /// Instead the ceiling ratchets down while the temperature is climbing
        /// into the band, then HOLDS across a wide middle window: it comes to rest
        /// at whatever power this train, this grade and this weather can carry
        /// indefinitely. Full power is only earned back once the loco has gone
        /// properly cold, and then only at a crawl.
        /// </summary>
        private static void UpdateHeatCap(Run run, TrainCar loco, float speedKmh)
        {
            float hottest = float.MinValue;
            float margin = float.MaxValue; // °C of headroom below the warning line
            foreach (var c in MuGroup(loco))
            {
                if (c == null)
                    continue;
                var observer = c.GetComponentInChildren<MultipleUnitStateObserver>();
                var port = Ports.Find(c, observer?.temperaturePortId);
                if (observer == null || port == null)
                    continue;
                if (observer.overheatStandardThreshold - port.Value < margin)
                {
                    margin = observer.overheatStandardThreshold - port.Value;
                    hottest = port.Value;
                }
            }
            if (margin == float.MaxValue)
            {
                // Nothing in the consist reports a temperature (steam, or a modded
                // loco without the port). Never invent a restriction from silence.
                run.heatCap = 1f;
                run.heatRise = 0f;
                run.heatLastTemp = float.NaN;
                return;
            }

            // Trend over a couple of seconds. A single tick's difference is below
            // the resolution of anything the gauge does - the oil takes tens of
            // seconds to move a degree - so measuring per tick reads as pure noise.
            run.heatSampleFor += DriveTick;
            if (run.heatSampleFor >= HeatTrendSeconds)
            {
                if (!float.IsNaN(run.heatLastTemp))
                    run.heatRise = (hottest - run.heatLastTemp) / run.heatSampleFor;
                run.heatLastTemp = hottest;
                run.heatSampleFor = 0f;
            }
            bool climbing = run.heatRise > HeatRiseEpsilon;

            if (margin <= HardOverheatMargin)
            {
                // On the line. Get off it whatever the trend says - there is no
                // headroom left to wait and see in.
                run.heatCap -= HeatCapCutFast * DriveTick;
            }
            else if (margin <= PreOverheatMargin && climbing)
            {
                // Warm AND still going up: this power level is not sustainable
                // here. Ease down until the climb stops, then hold there.
                run.heatCap -= HeatCapCutSlow * DriveTick;
            }
            else if (!climbing)
            {
                // Clear of the cut band and not heating: earn power back, slowly.
                //
                // The recovery point used to sit a further CoolOffMargin below
                // the cut point, on the theory that a dead band stops the
                // ceiling chasing the reading. What it actually did was strand
                // it: a DH4 working hard settles around 83.5 C, which is 6.5 C
                // below its line - past the cut band, short of the recovery
                // band - so the ceiling held at whatever it had fallen to for
                // the rest of the journey and the train slowed to a crawl on
                // the first grade. The hysteresis comes from the climbing test
                // instead, which is the honest question anyway.
                run.heatCap += HeatCapRecover * DriveTick;
            }
            // else: warm and still climbing - hold what we have.

            // Working hard while barely moving is where a hydraulic dumps most of
            // its heat, so while warm the ceiling is lower at low speed. Applied as
            // a ceiling rather than baked into heatCap, so it lifts again with
            // speed instead of being remembered as a permanent cut.
            float ceiling = speedKmh < ConverterSlipKmh && margin <= PreOverheatMargin
                ? HotThrottleCapLow : 1f;
            // Never below half power: a ceiling that forbids movement outright does
            // not cool anything, it just reads as "stuck" and abandons the run.
            run.heatCap = Mathf.Clamp(run.heatCap, HotThrottleCapHard, ceiling);
        }

        /// <summary>Worst wheelslip across every loco in the consist. The lead unit
        /// alone is a lie in an MU lash-up: the trailing units get the same throttle
        /// and can break loose first — and did, unseen.</summary>
        private static float MaxSlip(TrainCar loco)
        {
            float slip = 0f;
            foreach (var c in MuGroup(loco))
                if (c != null)
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
        /// <summary>A loco carrying neither port — steam, hydraulic — simply reads
        /// as never overheating, and the DE2 carries one of the pair and not the
        /// other. Asked through Ports, so a machine without them costs nothing and
        /// says nothing in the log.</summary>
        private static (bool warning, bool critical) TmOverheatState(TrainCar loco)
        {
            bool warning = false, critical = false;
            foreach (var c in MuGroup(loco))
            {
                var lamp = Ports.Find(c, "tmOverheatLamp.LAMP_STATE");
                if (lamp != null)
                    warning |= lamp.Value > 0.5f;
                var fuse = Ports.Find(c, "tm.OVERHEAT_POWER_FUSE_OFF");
                if (fuse != null)
                    critical |= fuse.Value > 0.5f;
            }
            return (warning, critical);
        }

        private static void ApplyThrottle(
            RemoteControllerModule controller, ref float throttle, TrainCar car,
            float target, float speed, LocoProfile profile,
            ref float lastSpeedMs, ref float accelFilter, ref float accelBudget,
            ref float sanderHold, ref float dynStallFor,
            ref float dynHandle, ref float dynCap, bool allowDynamic,
            float heatCap, bool tempWarning, bool tempCritical)
        {
            var overrider = controller.controlsOverrider;

            // A commanded stop needs a firm brake — the proportional touch below is
            // far too weak to hold 200+ t and just leaves the train creeping.
            if (target < 0.8f)
            {
                throttle = 0f;
                dynHandle = 0f;
                overrider.Throttle?.Set(0f);
                overrider.DynamicBrake?.Set(0f);
                Brakes.Train(car, overrider, 1f);
                // Standing still is when a gearbox may be worked freely: no load on
                // the dogs, nothing to grind. Calibration and the drop back to first
                // both want to happen here rather than against a moving train.
                Transmissions.HoldsThrottle(car, target, speed);
                return;
            }

            // A machine with a gearbox chooses its ratio before it chooses a
            // throttle, and while it is changing one the handle must be SHUT — the
            // DM3 has no clutch and a shift under power grinds the box. Placed
            // ahead of every other hand on the throttle so no branch below can open
            // it mid-shift.
            if (Transmissions.HoldsThrottle(car, target, speed))
            {
                throttle = 0f;
                dynHandle = 0f;
                overrider.Throttle?.Set(0f);
                overrider.DynamicBrake?.Set(0f);
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
            {
                accelBudget = Mathf.Max(MinAccelBudget, accelBudget - SlipAccelCut * DriveTick);
                // The dynamic caused this slip as much as the throttle could have —
                // cut its cap too, so the retry settles under what the rail holds.
                dynCap = Mathf.Max(DynamicCapFloor, dynCap - DynamicSlipCut);
            }
            else
            {
                accelBudget = Mathf.Min(safeAccel, accelBudget + AccelRecover * DriveTick);
                dynCap = Mathf.Min(1f, dynCap + DynamicCapRecover * DriveTick);
            }

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
                dynHandle = 0f;
                overrider.DynamicBrake?.Set(0f); // braking slipping wheels locks them
                // Slip is a TRACTION event — the loco's wheels breaking loose. The
                // train air acts on every axle of the rake and is what holds the
                // speed on a wet grade; releasing it on each slip tick let L-066 run
                // 11 km/h past its limit downhill and derail on stress. Keep the
                // service air the overspeed asks for; only loco-side handles ease.
                Brakes.Train(car, overrider, error < -1.0f
                    ? Mathf.Min(ServiceBrakeMax, Mathf.Clamp01(-error / 8f))
                    : 0f);
            }
            else if (tempCritical)
            {
                // Engine over temperature: come off power whatever the speed error.
                // Heat condemns only the handles that MAKE heat — throttle and
                // dynamic (the DH4's retarder churns the same transmission oil the
                // gauge reads). The train air heats nothing on the loco, so any
                // overspeed still gets the service application; releasing it here
                // left a hot loco freewheeling down the grade with no brake at all.
                throttle = Mathf.Max(0f, throttle - ThrottleDown);
                overrider.Throttle?.Set(throttle);
                dynHandle = 0f;
                overrider.DynamicBrake?.Set(0f);
                Brakes.Train(car, overrider, error < -1.0f
                    ? Mathf.Min(ServiceBrakeMax, Mathf.Clamp01(-error / 8f))
                    : 0f);
            }
            else if (tempWarning && throttle > 0f)
            {
                // On the red line but not yet critical: EASE OFF until it cools back
                // under the mark. Holding the handle where it was — the old behaviour
                // — kept a flat-out loco sitting on its red line, still heating,
                // until it crossed into critical and got damaged anyway. Backing off
                // at the ramp rate settles the temperature just under the line and
                // picks the speed back up once it drops. Air stays available for
                // overspeed here too — it costs no heat.
                throttle = Mathf.Max(0f, throttle - profile.throttleUp);
                overrider.Throttle?.Set(throttle);
                Brakes.Train(car, overrider, error < -1.0f
                    ? Mathf.Min(ServiceBrakeMax, Mathf.Clamp01(-error / 8f))
                    : 0f);
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
                // Warm but not yet on the red line: come DOWN to what the heat
                // allows rather than snapping there — a step off the top of the
                // handle surges the couplers on a long rake. The loco's own
                // minimum-move throttle still wins, so a heavy train can always
                // start; a cap that forbade movement outright would just read as
                // "stuck" and abandon the run.
                // No relief for being behind on speed. That was tried and it
                // brought the buzzer back: on a long grade the train is behind
                // for the whole climb, so a ceiling that yields to a speed
                // deficit is a ceiling that is never applied, and the engine
                // walked straight up to its warning line every time.
                //
                // Which constraint binds is decided by TEMPERATURE, upstream in
                // UpdateHeatCap: below the band the ceiling is on its way back
                // to 1 and the train may use everything it has; inside the band
                // the heat governs and the train goes as fast as the engine can
                // carry it, which on a steep enough grade is slower than asked.
                // That is the machine, not the policy — a DH4 that cannot hold
                // 60 up a bank at 88 °C also cannot hold it at 95 °C, it just
                // breaks instead.
                float cap = Mathf.Max(profile.minMoveThrottle, heatCap);
                if (throttle > cap)
                    throttle = Mathf.Max(cap, throttle - ThrottleDown);
                overrider.Throttle?.Set(throttle);
                dynHandle = 0f;
                overrider.DynamicBrake?.Set(0f);
                Brakes.Train(car, overrider, 0f);
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
                // !tempWarning matters even though the branch above claims to
                // handle heat: that one only fires while the throttle is still
                // open, and when BRAKING the throttle is already shut, so a hot
                // loco fell straight through to here and put the retarder back
                // on. On the DH4 - whose dynamic churns the very oil the gauge
                // reads - that alternated air and dynamic across the critical
                // line and never settled into a brake application.
                if (allowDynamic && !tempWarning && !tempCritical
                    && dynamic != null && speed > DynamicBrakeMinKmh && throttle <= 0.01f)
                {
                    // Ramp toward what the error asks, capped by what the rail has
                    // shown it will hold. Setting the error straight onto the handle
                    // slid wet wheels, zeroed on the slip tick, then slammed back on
                    // the next — continuous wheelslip all the way down a rainy grade.
                    float want = Mathf.Min(Mathf.Clamp01(-error / 10f), dynCap);
                    dynHandle = want > dynHandle
                        ? Mathf.Min(want, dynHandle + DynamicRampUp)
                        : Mathf.Max(want, dynHandle - DynamicRampDown);
                    dynamic.Set(dynHandle);
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
                    Brakes.Train(car, overrider, needAir ? service : 0f);
                }
                else
                {
                    dynamic?.Set(0f);
                    dynHandle = 0f;
                    dynStallFor = 0f;
                    Brakes.Train(car, overrider, throttle <= 0.01f ? service : 0f);
                }
            }
            else
            {
                overrider.DynamicBrake?.Set(0f);
                dynHandle = 0f;
                dynStallFor = 0f;
                overrider.Throttle?.Set(throttle);
                Brakes.Train(car, overrider, 0f);
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
            // The last line of the run's account, and the one that matters most:
            // every other way of finding out how a run ended meant reading the log.
            run.Note($"{status.ToUpperInvariant()}: {info}");
            run.coroutine = null;
            ReleaseClaims(run); // an arrived or failed run must not hold the junction
            LogSpot(run, status);

            // A finished run ties its train down: handbrake on the first wagon, the
            // way a driver pins a brake before stepping off. Optionally the loco
            // then cuts itself off, ready for its next job. Deferred until the
            // train actually STANDS — Finish fires the moment the arrival test
            // passes, which is still at rolling speed, and cutting a moving train
            // sent the wagons on without their loco.
            //
            // Not just shunts. A road move ends with the train left standing on a
            // running line for exactly the same reasons, and it is the heavier and
            // more consequential train of the two to leave on nothing but air —
            // which leaks off. The uncoupling half stays yard work: dispatch
            // already forces uncoupleOnFinish false outside a shunt.
            if ((status == "arrived" || status == "coupled") && run.car != null)
            {
                // The markings come off in there, once the train STANDS. Finish
                // fires while it is still rolling, and a train still being brought
                // to a stand by the autopilot is still under autopilot.
                Updater.Run(TieDownWhenStopped(run, info));
            }
            else
            {
                // Any other ending: both ends red, then the set comes off a few
                // seconds later. Note that a finished run stays in `runs` so its
                // status can still be read, so this is the ONLY place its markings
                // are concluded — StopRun never sees it.
                Markings.Conclude(run.guid, arrived: false);
            }
        }

        /// <summary>
        /// One line, at the moment a run ends, saying WHERE it actually stopped and
        /// against what. Everything the spot decides on — the marker window, the
        /// window's source (a real sign pair, or the fallback inset used when the
        /// signs are not streamed in), the extent of the cars that had to fit, the
        /// outstanding distance — lived only in run.detail, which nothing writes to
        /// the log. A spot that lands wrong was therefore unarguable after the fact.
        /// </summary>
        private static void LogSpot(Run run, string status)
        {
            if (status != "arrived" && status != "coupled")
                return;
            LogSpot(run.car, run.destTrack, run.id, status);
        }

        /// <summary>The same figures at DISPATCH time. A spot that never finishes
        /// never reaches Finish, so waiting for the arrival line to explain a run
        /// that is still shuffling is waiting for ever.</summary>
        internal static void LogSpotAtDispatch(TrainCar? car, RailTrack? dest, string id) =>
            LogSpot(car, dest, id, "dispatch");

        private static void LogSpot(TrainCar? car, RailTrack? dest, string id, string status)
        {
            if (car == null || dest == null)
                return;
            // The window the run is actually driven to, which is not the sign window
            // when the cut is too long for the signs but still fits the rail.
            var (lo, hi) = SpotZone(car, dest);
            var signZone = MarkerZone(dest);
            bool realSigns = markerZones.ContainsKey(dest);
            bool onRail = lo != signZone.lo || hi != signZone.hi;
            double mn = double.MaxValue, mx = double.MinValue;
            foreach (var c in CarsToSpot(car))
                foreach (var b in c.Bogies)
                    if (b?.track == dest)
                    {
                        mn = Math.Min(mn, b.traveller.Span);
                        mx = Math.Max(mx, b.traveller.Span);
                    }
            string extent = mn > mx ? "none on dest" : $"{mn:F1}..{mx:F1}";
            string destId = dest.LogicTrack()?.ID?.ToString() ?? "";
            string source = onRail
                ? $"RAIL, too long for signs {signZone.lo:F1}..{signZone.hi:F1}"
                : realSigns ? "signs" : "FALLBACK inset";
            // A fallback window means MarkerZone matched fewer than two signs. It is
            // never obvious WHY: the signs may not be streamed, or they are there and
            // the text it matches on is not the text they carry. Print the string we
            // compare against and every sign text actually standing near this track,
            // so the difference between "absent" and "mismatched" is one look.
            string signs = "";
            if (!realSigns)
            {
                var want = dest.LogicTrack()?.ID?.TrackPartOnly ?? "?";
                var pts = dest.GetKinkedPointSet().points;
                var move = WorldMover.currentMove;
                var found = new List<string>();
                foreach (var sh in Component.FindObjectsOfType<DV.Signs.TrackSignHover>())
                {
                    float bd = float.MaxValue;
                    double at = 0;
                    foreach (var p in pts)
                    {
                        float d = ((Vector3)p.position + move - sh.transform.position).magnitude;
                        if (d < bd) { bd = d; at = p.span; }
                    }
                    if (bd > 12f)
                        continue;
                    var tmp = sh.GetType().GetField("trackID")?.GetValue(sh);
                    var text = tmp?.GetType().GetProperty("text")?.GetValue(tmp) as string;
                    found.Add($"'{text}'@{at:F0}({bd:F0}m)");
                }
                signs = $" want='{want}' near=[{string.Join(" ", found)}]";
            }
            Main.mod?.Logger.Log(
                $"[SPOT] {id} {status} on {destId} "
                + $"markers={lo:F1}..{hi:F1} ({source}) "
                + $"physSpan={PhysicalSpan(dest):F1} "
                + $"cut={ConsistLength(car):F1}m over {extent} "
                + $"oversize={Oversize(car, dest)} "
                + $"toSpot={DistanceToSpot(car, dest):F1} "
                + $"clearWithin={ConsistClearWithinDest(car, destId, dest)}"
                + signs);
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
            // Standing still, so it is no longer driving itself: both ends green,
            // and the set comes off a few seconds later. Keyed on the GUID rather
            // than the car, so a loco that was despawned mid-stop still clears its
            // entry instead of leaving one that blocks the next fitting.
            Markings.Conclude(run.guid, arrived: true);

            var car = run.car;
            if (car == null)
                yield break;
            var (wagon, cutAt) = FirstWagonOut(car);
            // A light engine has no wagon to pin, and used to be left secured by
            // nothing at all. It is a train standing on a grade like any other, so
            // it pins its own. Every leg winds the handbrakes off before moving, so
            // this cannot strand the next dispatch.
            var tieDown = wagon ?? car;
            var brakes = tieDown.brakeSystem;
            if (brakes != null && brakes.hasHandbrake)
                brakes.SetHandbrakePosition(1f);
            if (wagon == null)
            {
                run.detail = info + " — handbrake set";
                yield break;
            }
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
        /// The first car outside the driven MU group reachable from this loco,
        /// walking the couplers outward (front side first), and the coupler whose
        /// parting separates the driven units from the rest. MU-cabled units stay
        /// together; a loco that is merely COUPLED — no MU cable — is freight and
        /// the cut lands in front of it like any wagon.
        /// </summary>
        private static (TrainCar? wagon, Coupler? cutAt) FirstWagonOut(TrainCar loco)
        {
            var driven = MuGroup(loco);
            foreach (var start in new[] { loco.frontCoupler, loco.rearCoupler })
            {
                var coupler = start;
                while (coupler != null && coupler.coupledTo != null)
                {
                    var next = coupler.coupledTo.train;
                    if (next == null)
                        break;
                    if (!driven.Contains(next))
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
