using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// The mechanical gearboxes, for the machines that have one.
    ///
    /// Both are driven exactly like a manual car: put it in gear, open the
    /// throttle, and when the engine is near its redline change up; when it is
    /// lugging near idle, change down. The shift points come off the engine's own
    /// tachometer ports, so no per-gear speed table is needed — which is as well,
    /// because none is published for the DM1U at all.
    ///
    /// Neither box has a clutch, so a shift with the throttle open grinds it and
    /// takes powertrain damage. Every shift here is: shut off, move the lever,
    /// wait for the box to report the change complete, power back.
    ///
    /// Two shapes of machine:
    ///
    ///   DM1U — one seven-position selector (neutral + gears I to VI) on
    ///          gearSelect.CONTROL_EXT_IN.
    ///   DM3  — a fluid coupling into TWO three-position boxes in series
    ///          (gearInputA/B), driven up an eight-gear ladder of combinations.
    ///          The ninth combination, A1/B3, the wiki marks "recommended to
    ///          skip" and the community chart omits; it buys nothing.
    ///
    /// What the game wants WRITTEN to a lever port for each position is not
    /// documented anywhere this repo can reach, so it is established at a
    /// standstill by writing a candidate value and reading the GEAR port back —
    /// every lever has one, and it answers with the position that value selected.
    /// A machine whose levers answer to no known encoding is refused with the
    /// readings in the log rather than driven on a guess.
    /// </summary>
    public static class Transmissions
    {
        /* ------------------------------------------------------- the machine -- */

        // Detection is by PORT PRESENCE, not livery id: a CCL loco with the same
        // transmission gets the same treatment, and a DM3 renamed by an update
        // does not quietly fall out.

        // The DM3's two boxes.
        private const string GearInA = "gearInputA.CONTROL_EXT_IN";
        private const string GearInB = "gearInputB.CONTROL_EXT_IN";
        private const string GearOfA = "gearInputA.GEAR";
        private const string GearOfB = "gearInputB.GEAR";
        private const string ShiftingA = "transmissionA.GEAR_CHANGE_IN_PROGRESS";
        private const string ShiftingB = "transmissionB.GEAR_CHANGE_IN_PROGRESS";

        // The DM1U's one selector.
        private const string SelIn = "gearSelect.CONTROL_EXT_IN";
        private const string SelOut = "gearSelect.GEAR";
        private const string SelShifting = "transmission.GEAR_CHANGE_IN_PROGRESS";

        // The tachometer, same ports on both.
        private const string RpmNorm = "de.RPM_NORMALIZED";
        private const string MaxPowerRpm = "de.MAX_POWER_RPM_NORMALIZED";
        private const string IdleRpm = "de.IDLE_RPM_NORMALIZED";

        /// <summary>The DM3's ladder, lowest first: lever positions 0..2 for each
        /// box. Order is by overall ratio, from the community chart and the wiki
        /// (which agree); A0/B2 is deliberately absent, as both sources say.</summary>
        private static readonly (int a, int b)[] DualGears =
        {
            (0, 0), (0, 1), (1, 0), (1, 1), (2, 0), (2, 1), (1, 2), (2, 2),
        };

        /// <summary>A shift is a sequence — shut off, move the lever, wait for the
        /// box, open up — and each step gets at least this long. Also the settling
        /// time between a calibration write and the read that follows it.</summary>
        private const float StepSeconds = 0.2f;

        /// <summary>Nothing is written to a lever above this speed on the theory
        /// that the loco is stationary. Calibration moves the levers through every
        /// position, which is only safe standing still.</summary>
        private const float StandstillKmh = 0.5f;

        /// <summary>The drive loop's tick, which is what HoldsThrottle is counting
        /// down in. Kept here rather than reaching into Autopilot's private
        /// constant so this file stands on its own.</summary>
        private const float DriveTickSeconds = 0.1f;

        /* -------------------------------------------------------- shift rule -- */

        // The two figures the shift rule turns on live in each machine's profile —
        // LocoProfile.upshiftBand and LocoProfile.cruiseThrottle, both editable in
        // the settings panel. What they MEAN is here, where they are used.

        // upshiftBand: change up when the engine is this far from max-power revs
        // toward the redline, as a fraction of that remaining band — past the power
        // peak, revving further is noise, not speed. On the DM3 (peak 0.84) the
        // default 0.5 is 0.92 normalized, about 920 of its 1000 rpm.
        //
        // cruiseThrottle: throttle at or below which the engine is not being asked
        // for power — the drive loop has the speed it wanted and is holding it.
        //
        // Revs alone cannot decide the change up. In gear, road speed and engine
        // speed are the same number twice, so a loco that has REACHED its target
        // speed stops accelerating and its revs stop climbing wherever they are:
        // the DM1U sat at 1900 of its 2200 rpm — exactly its power peak, 0.864 —
        // with the throttle being backed off, and the upshift line at 0.932 was
        // never crossed. It drove whole journeys in first.
        //
        // Screaming at peak revs with the throttle shut is not pulling hard, it is
        // a gear too low: the same road speed is available one gear up with revs
        // in hand. Under load — throttle open above this — the peak is exactly
        // where the engine belongs and it stays put.

        /// <summary>Change down when revs sink to idle plus this margin: the
        /// coupling is barely turning the input and the engine is doing nothing.
        /// The wide gap between here and the upshift point is the hysteresis —
        /// one shift moves the revs by a ratio step, never across both lines.</summary>
        private const float DownshiftAboveIdle = 0.15f;

        /// <summary>Which gear the tachometer wants, one step at a time — skipping
        /// ratios on a clutchless box stalls the engine or over-revs it, and each
        /// step is cheap.</summary>
        private static int Choose(TrainCar car, int gear, int topGear, float speedKmh)
        {
            // Standing: walk back to first, ready for the next launch. No load on
            // the box at a standstill, so the steps cost nothing.
            if (speedKmh <= StandstillKmh)
                return gear > 1 ? gear - 1 : gear;

            float rpm = Read(car, RpmNorm);
            if (float.IsNaN(rpm))
                return gear;
            // Where this machine is asked to change up is the machine's own
            // business — a six-speed shunter and a two-box road loco do not want
            // the same band — so both figures come from its profile.
            var profile = Main.settings.autopilot.For(car.carLivery?.id);
            float maxPower = Read(car, MaxPowerRpm);
            float peak = float.IsNaN(maxPower) || maxPower <= 0.1f ? 0.8f : maxPower;
            float up = peak + (1f - peak) * Mathf.Clamp01(profile.upshiftBand);
            float idle = Read(car, IdleRpm);
            float down = (float.IsNaN(idle) ? 0.3f : idle) + DownshiftAboveIdle;
            float pedal = Read(car, "throttle.EXT_IN");

            // Revving out: more speed is wanted and this gear has none left.
            if (gear < topGear && rpm >= up)
                return gear + 1;
            // Or holding the speed it was asked for at peak revs on a shut
            // throttle — see CruiseThrottle. The gap between peak and the
            // revving-out line is what stops this firing under load. Not while
            // the train brake is on: revs above the peak with the brake applied
            // means the train is being STOPPED, and walking up the box on the way
            // to a stand is a shift ladder that buys nothing — the standstill rule
            // above walks straight back down again.
            if (gear < topGear && rpm >= peak
                && !float.IsNaN(pedal) && pedal <= profile.cruiseThrottle
                && !(Read(car, "brake.EXT_IN") > 0.05f))
                return gear + 1;
            // Lugging only counts with the throttle actually OPEN. With it shut
            // the revs sit at idle whatever the road is doing — which is exactly
            // what they do during our own shifts (the sequence closes the
            // throttle), so an unguarded downshift undid every upshift the moment
            // it completed. Coasting and braking are not lugging either; a lower
            // gear at speed is not grabbed for them.
            if (gear > 1 && rpm <= down && pedal > 0.05f)
                return gear - 1;
            return gear;
        }

        /* ------------------------------------------------------ per-machine -- */

        /// <summary>One geared machine, either shape. The DM3 uses both lever
        /// tables; the DM1U only <see cref="leverA"/>.</summary>
        private sealed class Gearbox
        {
            public bool dual;
            /// <summary>Lever positions incl. any neutral; for the DM1U this is
            /// NUM_OF_GEARS (7: neutral + I..VI), for a DM3 box always 3.</summary>
            public int positions;
            /// <summary>What to write to select position p.</summary>
            public float[] leverA = Array.Empty<float>();
            public float[] leverB = Array.Empty<float>();

            public bool ready;
            /// <summary>Set when calibration failed; the loco is not driven and
            /// this says why. Kept so the reason is reported once, not every
            /// tick.</summary>
            public string? refused;

            /// <summary>Current gear: index into DualGears + 1 for the DM3, the
            /// selector position for the DM1U (neutral is 0, never driven in).
            /// -1 while the lever position is unknown.</summary>
            public int gear = -1;
            public int wantGear = 1;
            public float holdFor;
            public int phase;

            // Calibration state: which step, and the gear numbers the levers'
            // endpoints turned out to select.
            public int calStep;
            public int loA, hiA, loB, hiB;
        }

        /// <summary>One entry per car; the learned livery cache below saves the
        /// sweep on every machine after a livery's first.</summary>
        private static readonly Dictionary<string, Gearbox> boxes =
            new Dictionary<string, Gearbox>();

        private static readonly Dictionary<string, (float[] a, float[] b)> learned =
            new Dictionary<string, (float[], float[])>();

        /* ------------------------------------------------------------ ports -- */

        // Through Ports, not the game's TryGetPort: every one of these is a
        // question asked of machines that mostly answer no, and the game's own
        // lookup logs an error for each no. See Ports.cs.

        private static float Read(TrainCar car, string id) => Ports.Read(car, id);

        private static bool Write(TrainCar car, string id, float value) =>
            Ports.Write(car, id, value);

        private static bool HasPort(TrainCar? car, string id) => Ports.Has(car, id);

        /// <summary>Does this car have a gearbox we have to work?</summary>
        public static bool IsGeared(TrainCar? car) =>
            HasPort(car, GearInA) || HasPort(car, SelIn);

        private static bool Shifting(TrainCar car, bool dual) =>
            dual
                ? Read(car, ShiftingA) > 0.5f || Read(car, ShiftingB) > 0.5f
                : Read(car, SelShifting) > 0.5f;

        /* ------------------------------------------------------ the control -- */

        /// <summary>
        /// Work the gearbox for this tick. Returns true when the throttle must be
        /// held SHUT — during a shift, during calibration, and for good if the box
        /// could not be calibrated at all.
        ///
        /// Called from ApplyThrottle, so every path that drives a locomotive goes
        /// through it and no path can forget to.
        /// </summary>
        public static bool HoldsThrottle(TrainCar? car, float targetKmh, float speedKmh)
        {
            bool dual = HasPort(car, GearInA);
            if (car == null || (!dual && !HasPort(car, SelIn)))
                return false;
            driven = car;

            var box = For(car, dual);
            if (box.refused != null)
                return true;

            if (box.holdFor > 0f)
            {
                box.holdFor -= DriveTickSeconds;
                return true;
            }

            if (!box.ready)
                return Calibrate(car, box, speedKmh);

            // A shift in progress in the game's own model is not ours to interrupt.
            if (Shifting(car, box.dual))
                return true;

            // Levers of unknown position — a machine whose encoding we already
            // knew, so calibration never ran and never touched them. Put it in
            // first before reading anything into what gear it is in.
            if (box.gear < 1)
            {
                box.wantGear = 1;
                return Shift(car, box);
            }

            // A shift already under way finishes on the gear it was STARTED for.
            // Asking again mid-sequence asked a loaded question: phase 0 shuts the
            // throttle, revs fall away with it, and the answer one tick later is
            // "stay where you are" — which phase 1 then dutifully wrote to the
            // lever, selecting the gear the box was already in. The throttle came
            // back, the revs climbed to full, and the whole pantomime repeated;
            // the DM1U changed up only when a tick happened to land on phase 1
            // with the revs still high.
            if (box.phase != 0)
                return Shift(car, box);

            box.wantGear = Choose(car, box.gear, TopGear(box), speedKmh);
            if (box.wantGear == box.gear)
                return false;

            return Shift(car, box);
        }

        private static int TopGear(Gearbox box) =>
            box.dual ? DualGears.Length : box.positions - 1;

        private static void Levers(Gearbox box, int gear, out float a, out float? b)
        {
            if (box.dual)
            {
                var (pa, pb) = DualGears[gear - 1];
                a = box.leverA[pa];
                b = box.leverB[pb];
            }
            else
            {
                a = box.leverA[gear];
                b = null;
            }
        }

        /// <summary>
        /// The shift itself, one phase per call: shut off, move the lever(s), wait
        /// for the box to report the change complete, give the throttle back. On
        /// the DM3 both levers move in the same shift — 4 to 5 is middle/middle to
        /// top/bottom — so they are set together, not one box at a time.
        /// </summary>
        private static bool Shift(TrainCar car, Gearbox box)
        {
            switch (box.phase)
            {
                case 0:
                    // Shut off and let the driveline unload before touching a lever.
                    var overrider = car.SimController?.controlsOverrider;
                    overrider?.Throttle?.Set(0f);
                    overrider?.DynamicBrake?.Set(0f);
                    box.phase = 1;
                    box.holdFor = StepSeconds;
                    return true;
                case 1:
                    Levers(box, box.wantGear, out float a, out float? b);
                    Write(car, box.dual ? GearInA : SelIn, a);
                    if (b != null)
                        Write(car, GearInB, b.Value);
                    int from = box.gear, to = box.wantGear;
                    string who = car.ID;
                    // In Player.log too, not only the trace: a gearbox that will
                    // not change up looks exactly like one that has nothing to
                    // change up to, and the trace is off unless somebody switched
                    // it on before the journey that went wrong.
                    Main.mod?.Logger.Log(
                        $"[{who}] gear {(from < 1 ? 0 : from)} -> {to}"
                        + $" (rpm {Read(car, RpmNorm):F2} throttle {Read(car, "throttle.EXT_IN"):F2})");
                    Trace.Line("GEAR", () => $"{who} shift {(from < 1 ? 0 : from)} -> {to}");
                    box.phase = 2;
                    box.holdFor = StepSeconds;
                    return true;
                default:
                    if (Shifting(car, box.dual))
                        return true;
                    box.gear = box.wantGear;
                    box.phase = 0;
                    return false;
            }
        }

        /* ----------------------------------------------------- calibration -- */

        private static Gearbox For(TrainCar car, bool dual)
        {
            if (boxes.TryGetValue(car.CarGUID, out var box))
                return box;
            box = new Gearbox { dual = dual };
            // A livery already worked out costs nothing to reuse. Knowing the
            // encoding says nothing about where the LEVERS are, though: the player
            // may have parked this one in fifth, or in neutral. Unknown until we
            // have set it ourselves.
            string livery = car.carLivery?.id ?? "";
            // A map kept from an earlier session counts as learned: it was measured
            // off this same livery's ports and nothing about them moves between
            // sessions. Seeded into the session cache so the two agree.
            if (!learned.ContainsKey(livery))
            {
                var saved = Main.settings.autopilot.For(livery);
                // A two-box machine needs both ladders; half a map is no map, and
                // a hand-edited Settings.xml can hold one.
                if (saved.livery == livery && saved.gearLeverA.Count > 0
                    && (!dual || saved.gearLeverB.Count == saved.gearLeverA.Count))
                    learned[livery] = (saved.gearLeverA.ToArray(), saved.gearLeverB.ToArray());
            }
            if (learned.TryGetValue(livery, out var known))
            {
                box.leverA = known.a;
                box.leverB = known.b;
                box.positions = known.a.Length;
                box.ready = true;
                box.gear = -1;
            }
            boxes[car.CarGUID] = box;
            return box;
        }

        /// <summary>
        /// Establish what the lever ports want written, by asking the machine
        /// rather than assuming a count: write the bottom of the range and read
        /// WHICH gear that is, write the top and read that, then prove the middle
        /// lands where a linear lever says it must. Four steps, standing still,
        /// once per livery. The position COUNT is never presumed — it comes back
        /// from the read: the DM1U answers 0..6, seven positions for a port whose
        /// NUM_OF_GEARS reads 7 and a lever that also carries a neutral.
        ///
        /// What IS presumed, and should not be, is that one procedure fits both
        /// machines. On the DM3 this asserts each of the two boxes spans exactly
        /// two steps and moves linearly — an encoding that context/dm3-shifting-
        /// sequence.md says nothing establishes ("NUM_OF_GEARS reads 3 on both
        /// boxes and nothing establishes whether the port runs 0/0.5/1, 0/1/2, or
        /// something else"). That document specifies a different calibration for
        /// the DM3: sweep all nine lever combinations and fit the
        /// OVERALL_GEAR_RATIO readings proportionally against the known ratio
        /// table. This code does not do that yet.
        /// </summary>
        private static bool Calibrate(TrainCar car, Gearbox box, float speedKmh)
        {
            // Levers may only be waggled standing still. Moving, there is nothing
            // to do but hold power off until the train has stopped.
            if (speedKmh > StandstillKmh)
                return true;
            if (Shifting(car, box.dual))
                return true;

            switch (box.calStep)
            {
                case 0:
                    WriteBoth(car, box, 0f);
                    return true;
                case 1:
                    if (!ReadBack(car, box, out box.loA, out box.loB))
                        return Refuse(car, box, "gear read-back is not a number");
                    WriteBoth(car, box, 1f);
                    return true;
                case 2:
                    if (!ReadBack(car, box, out box.hiA, out box.hiB))
                        return Refuse(car, box, "gear read-back is not a number");
                    if (box.hiA <= box.loA || (box.dual && box.hiB <= box.loB))
                        return Refuse(car, box, "lever does not move the box"
                            + Ranges(box));
                    if (box.dual && (box.hiA - box.loA != 2 || box.hiB - box.loB != 2))
                        return Refuse(car, box, "twin boxes are not three-position"
                            + Ranges(box));
                    WriteBoth(car, box, Mid(box) / (float)(box.hiA - box.loA));
                    return true;
                default:
                    if (!ReadBack(car, box, out int midA, out int midB)
                        || midA != box.loA + Mid(box)
                        || (box.dual && midB != box.loB + Mid(box)))
                        return Refuse(car, box,
                            $"lever is not linear (mid read {midA}/{midB})" + Ranges(box));
                    return Finish(car, box);
            }
        }

        /// <summary>The middle position to prove linearity on, as an offset from
        /// the bottom. Both DM3 boxes and the DM1U selector have equal-length
        /// ranges per lever by the time this is used.</summary>
        private static int Mid(Gearbox box) => (box.hiA - box.loA) / 2;

        private static string Ranges(Gearbox box) =>
            $" (A {box.loA}..{box.hiA}" + (box.dual ? $", B {box.loB}..{box.hiB})" : ")");

        private static void WriteBoth(TrainCar car, Gearbox box, float value)
        {
            Write(car, box.dual ? GearInA : SelIn, value);
            if (box.dual)
                Write(car, GearInB, value);
            box.calStep++;
            box.holdFor = StepSeconds;
        }

        private static bool ReadBack(TrainCar car, Gearbox box, out int a, out int b)
        {
            float fa = Read(car, box.dual ? GearOfA : SelOut);
            float fb = box.dual ? Read(car, GearOfB) : 0f;
            a = float.IsNaN(fa) ? int.MinValue : Mathf.RoundToInt(fa);
            b = float.IsNaN(fb) ? int.MinValue : Mathf.RoundToInt(fb);
            return a != int.MinValue && b != int.MinValue;
        }

        private static bool Refuse(TrainCar car, Gearbox box, string why)
        {
            box.refused = why;
            string who = car.ID;
            Main.mod?.Logger.Log($"[{who}] gearbox refused: {why}. Not driving it on a guess.");
            Trace.Line("GEAR", () => $"{who} calibration FAILED: {why}");
            return true;
        }

        private static bool Finish(TrainCar car, Gearbox box)
        {
            int span = box.hiA - box.loA;
            box.positions = span + 1;
            var a = new float[box.positions];
            var b = new float[box.positions];
            for (int p = 0; p <= span; p++)
                a[p] = b[p] = p / (float)span;
            box.leverA = a;
            box.leverB = b;
            box.ready = true;
            string livery = car.carLivery?.id ?? "";
            learned[livery] = (a, b);
            // And into the settings file, so the sweep — the lever dance a machine
            // does before its first move — is once per install rather than once per
            // session. Saved immediately: a crash on the journey that follows should
            // not cost the measurement.
            Main.settings.autopilot.LearnGears(livery, a, b);
            if (Main.mod != null)
                Main.settings.Save(Main.mod);

            // Into the starting gear — bottom of the ladder, never neutral.
            Levers(box, 1, out float la, out float? lb);
            Write(car, box.dual ? GearInA : SelIn, la);
            if (lb != null)
                Write(car, GearInB, lb.Value);
            box.gear = 1;
            box.wantGear = 1;
            box.phase = 0;
            box.holdFor = StepSeconds;

            string id = car.ID;
            string ranges = Ranges(box);
            Main.mod?.Logger.Log($"[{id}] gearbox calibrated{ranges}");
            Trace.Line("GEAR", () => $"{id} calibrated{ranges}, in gear 1");
            return true;
        }

        /* -------------------------------------------------------- recorder -- */

        /// <summary>Everything the gearboxes do, for the trace. Kept alongside the
        /// control: when a shift goes wrong the question is always what the ports
        /// were doing at the time.</summary>
        private static readonly string[] Watched =
        {
            GearInA, GearOfA, GearInB, GearOfB, "gearInputAB.GEAR",
            SelIn, SelOut,
            "gearRatioCalculator.OVERALL_GEAR_RATIO",
            "transmissionAB.MECHANICAL_GEAR_RATIO",
            "transmissionA.GEAR_RATIO", "transmissionB.GEAR_RATIO",
            "transmission.GEAR_RATIO", "transmission.GENERATED_DAMAGE",
            ShiftingA, ShiftingB, SelShifting,
            "transmissionA.GENERATED_DAMAGE", "transmissionB.GENERATED_DAMAGE",
            "fluidCoupler.TRANSMISSION_ENGAGED", "fluidCoupler.SPEED_RATIO",
            "fluidCoupler.TURBINE_RPM_NORMALIZED", "fluidCoupler.GENERATED_DAMAGE",
            "de.RPM", RpmNorm,
            "throttle.EXT_IN", "reverser.REVERSER", "brake.EXT_IN",
        };

        /// <summary>How much a value must move to count as a change. Ratios and
        /// gear indices step; RPM drifts continuously, and logging every tick of
        /// that would bury the shifts themselves.</summary>
        private const float Epsilon = 0.02f;

        private static Coroutine? watcher;

        public static void Start()
        {
            Stop();
            watcher = Updater.Run(Sample());
        }

        public static void Stop()
        {
            if (watcher != null)
            {
                Updater.StopRun(watcher);
                watcher = null;
            }
            boxes.Clear();
        }

        private static IEnumerator Sample()
        {
            var last = new Dictionary<string, float>();
            TrainCar? watching = null;
            var wait = new WaitForSecondsRealtime(DriveTickSeconds);
            while (true)
            {
                yield return wait;
                if (!Trace.On)
                {
                    watching = null;
                    continue;
                }

                var car = Pick();
                var simFlow = car?.SimController?.simFlow;
                if (car == null || simFlow == null)
                {
                    watching = null;
                    continue;
                }

                // A different machine means a different set of readings; carrying
                // the old ones over would report the swap as a shift.
                if (car != watching)
                {
                    watching = car;
                    last.Clear();
                    var attached = car;
                    Trace.Line("GEAR", () =>
                        $"watching {attached.ID} ({attached.carLivery?.id ?? "?"})");
                }

                var line = new StringBuilder();
                foreach (var id in Watched)
                {
                    var port = Ports.Find(simFlow, id);
                    if (port == null)
                        continue;
                    float now = port.Value;
                    if (last.TryGetValue(id, out float was) && Mathf.Abs(now - was) < Epsilon)
                        continue;
                    last[id] = now;
                    if (line.Length > 0)
                        line.Append(' ');
                    line.Append(id).Append('=')
                        .Append(now.ToString("F3", CultureInfo.InvariantCulture));
                }
                if (line.Length == 0)
                    continue;

                // Speed belongs on every line: what a gear ultimately has to
                // answer for is the speed it was holding, and a shift with no
                // speed against it says nothing.
                float kmh = Mathf.Abs(car.GetForwardSpeed()) * 3.6f;
                string changes = line.ToString();
                var sampled = car;
                Trace.Line("GEAR", () =>
                    $"{sampled.ID} v={kmh.ToString("F2", CultureInfo.InvariantCulture)} {changes}");
            }
        }

        /// <summary>Which geared locomotive to record: the one the player is
        /// riding for choice, otherwise the last one the drive loop worked a
        /// gearbox on. One at a time — the point is a clean single-machine record,
        /// not a survey.</summary>
        private static TrainCar? Pick()
        {
            var player = PlayerManager.Car;
            if (IsGeared(player))
                return player;
            return IsGeared(driven) ? driven : null;
        }

        /// <summary>The last geared machine the drive loop asked about, so the
        /// recorder can follow an autopilot DM1U or DM3 with nobody in the
        /// cab.</summary>
        private static TrainCar? driven;
    }
}
