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
    /// <summary>Placing the train at its destination: the marker zones read off
    /// the in-game signs, the distance still to run to the spot, and the checks
    /// for a consist that fits (or does not).</summary>
    public static partial class Autopilot
    {
        private static readonly Dictionary<RailTrack, (double lo, double hi)> markerZones =
            new Dictionary<RailTrack, (double, double)>();
        // Tracks whose signs weren't found yet, and the earliest realtime to look again.
        private static readonly Dictionary<RailTrack, float> markerRetryAt =
            new Dictionary<RailTrack, float>();
        // How long to wait before re-scanning for signs that weren't there, s.
        private const float MarkerRetrySeconds = 5f;

        /// <summary>The track's parking zone [lo, hi] in span, taken from its actual
        /// in-game sign markers (matched by the track's own ID). Falls back to a fixed
        /// switch-clearance inset if the signs can't be found. A real hit is cached for
        /// good; a fallback is only held for a few seconds, because with no player
        /// nearby the sign objects may simply not be streamed in yet and would
        /// otherwise be written off permanently.</summary>
        private static (double lo, double hi) MarkerZone(RailTrack track)
        {
            if (markerZones.TryGetValue(track, out var cached))
                return cached;
            if (markerRetryAt.TryGetValue(track, out var retryAt) && Time.realtimeSinceStartup < retryAt)
                return FallbackZone(track);

            var part = track.LogicTrack()?.ID?.TrackPartOnly;
            var pts = track.GetKinkedPointSet().points;
            var move = WorldMover.currentMove;
            var spans = new List<double>();

            if (part != null)
            {
                foreach (var sh in Component.FindObjectsOfType<DV.Signs.TrackSignHover>())
                {
                    var pos = sh.transform.position;
                    double best = 0;
                    float bd = float.MaxValue;
                    foreach (var p in pts)
                    {
                        float d = ((Vector3)p.position + move - pos).magnitude;
                        if (d < bd) { bd = d; best = p.span; }
                    }
                    if (bd > 8f)
                        continue; // sign belongs to another track
                    // A track sign carries its id in TWO fields: the sub-yard letter
                    // on its own (subYardID, "C") and the rest (trackID, "2I"). This
                    // compared TrackPartOnly — "C2I", the whole thing — against the
                    // trackID field alone, so it never matched a single sign on any
                    // track in a lettered sub-yard. MarkerZone then fell back to a
                    // flat 18 m inset off each physical end and EVERY spot on that
                    // track aimed at an invented window: HB-C2I's real marks are at
                    // 25 and 409 (384 m), the fallback claimed 18 and 416 (398 m),
                    // and a 397 m cut therefore "fitted" by a metre when in truth it
                    // was 13 m too long. The hunting, the overruns and the drive to
                    // the buffer all came off that one number.
                    var id = SignFieldText(sh, "trackID");
                    var sub = SignFieldText(sh, "subYardID");
                    if (id == part || sub + id == part)
                        spans.Add(best);
                }
            }

            if (spans.Count < 2)
            {
                // No signs (yet). MarkerZone runs FindObjectsOfType, far too costly to
                // repeat every tick — that was the spotting lag — so hold the fallback
                // for a few seconds before looking again.
                markerRetryAt[track] = Time.realtimeSinceStartup + MarkerRetrySeconds;
                return FallbackZone(track);
            }

            var result = (spans.Min(), spans.Max());
            markerZones[track] = result;
            markerRetryAt.Remove(track);
            return result;
        }

        /// <summary>
        /// Are this track's marks the REAL ones, read off its signs — or the invented
        /// fallback window?
        ///
        /// MarkerZone answers with a window either way and says nothing about which,
        /// so every caller has been treating a guess as a survey. A fabricated window
        /// is not a small inaccuracy: it decides where the train stops, whether the
        /// cut is judged to fit at all, and when the run calls itself arrived. Callers
        /// that are about to act on it should ask this first and say so out loud.
        /// </summary>
        internal static bool MarkersAreReal(RailTrack track)
        {
            MarkerZone(track); // populates the cache on a hit
            return markerZones.ContainsKey(track);
        }

        /// <summary>The text of one of a track sign's TMP label fields, or "" when the
        /// field is absent. Reflective because DV.Signs.TrackSignHover's fields are not
        /// part of any contract we can reference.</summary>
        private static string SignFieldText(object sign, string field)
        {
            var tmp = sign.GetType().GetField(field)?.GetValue(sign);
            return tmp?.GetType().GetProperty("text")?.GetValue(tmp) as string ?? "";
        }

        /// <summary>Parking zone for a track whose signs can't be read: a fixed
        /// switch-clearance inset from each physical end.</summary>
        private static (double lo, double hi) FallbackZone(RailTrack track)
        {
            double physSpan = track.GetKinkedPointSet().span;
            double inset = Math.Min(MarkerClearance, physSpan / 2.0 - 1.0);
            return (inset, physSpan - inset);
        }

        /// <summary>True when every bogie of the whole consist sits on one and the
        /// same track — the train has pulled fully onto a single lead, clear of every
        /// throat switch behind it.</summary>
        private static bool ConsistOnSingleTrack(TrainCar loco)
        {
            RailTrack? one = null;
            foreach (var c in loco.trainset.cars)
                foreach (var b in c.Bogies)
                {
                    if (b?.track == null)
                        return false;
                    if (one == null)
                        one = b.track;
                    else if (b.track != one)
                        return false;
                }
            return one != null;
        }

        /// <summary>
        /// Metres still to run before the whole consist sits inside the destination's
        /// markers — the distance that actually remains of a spot. Whichever end we
        /// came in by, one of the two conditions is the outstanding one: the tail short
        /// of the near marker, or the front past the far one.
        /// </summary>
        /// <param name="towardHigh">Which way the spot is travelling, as the run
        /// remembers it. Only consulted while part of the train is still off the
        /// destination, and only because the live reading goes blank at a stand —
        /// which is precisely when the train is braking into its spot.</param>
        private static double DistanceToSpot(
            TrainCar loco, RailTrack destTrack, bool? towardHigh = null)
        {
            var (lo, hi) = MarkerZone(destTrack);
            double m = hi - lo > 2 * ParkMargin ? ParkMargin : 0;
            double mn = double.MaxValue, mx = double.MinValue;
            bool off = false;
            foreach (var c in CarsToSpot(loco))
                foreach (var b in c.Bogies)
                {
                    if (b == null)
                        continue;
                    if (b.track != destTrack)
                    {
                        off = true; // still coming in; the tail is outside the track
                        continue;
                    }
                    mn = Math.Min(mn, b.traveller.Span);
                    mx = Math.Max(mx, b.traveller.Span);
                }
            if (mn > mx)
                return double.MaxValue; // nothing on it yet
            // While the cut is still running in, the rearmost bogie ON the track is
            // whichever one last crossed the joint: it climbs a few metres and then
            // the next one crosses and puts the reading back at the entry. Measured
            // that way the outstanding distance reset once per bogie — the run
            // dropped into its final crawl, braked, powered up again and repeated it
            // for every wagon instead of pulling in once and spotting the tail.
            // The tail that matters is the END of the train, so while any of it is
            // still outside, it is measured from the leading bogie back over the
            // length of the cut, which shortens smoothly as the train comes on.
            if (off)
            {
                double bogieSpan = Math.Max(0, ConsistLength(loco) - 2 * CarEndOverhang);
                // Direction used to come from MOTION alone, and motion says nothing at
                // a stand — which is every dispatch, every re-plan, every restart after
                // a block. With no direction the correction below was skipped entirely
                // and the part of the train still OUTSIDE the track was treated as not
                // existing: L-085 stood with 126 m of a 397 m cut on HB-C2I and reported
                // 14.4 m still to run instead of 281. Every speed and stopping decision
                // downstream was then planned against a number nearly 20x too small.
                //
                // It never needed motion. The cut hangs off ONE end of the track, and
                // which end is plain geometry: the end its on-track bogies are bunched
                // against. Travel has to be away from that end, because that is where
                // the rest of the train still is.
                bool? way = towardHigh ?? SpotDirection(loco, destTrack)
                    ?? HangsOffLowEnd(destTrack, mn, mx);
                if (way == true)
                    mn = Math.Min(mn, mx - bogieSpan);
                else if (way == false)
                    mx = Math.Max(mx, mn + bogieSpan);
            }
            double needUp = (lo + m) - mn;   // tail not yet past the low marker
            double needDown = mx - (hi - m); // front beyond the high marker
            // A train longer than the track can never satisfy both markers, so it is
            // driven until its LEADING end reaches the far one and the tail is left
            // where it falls. Whichever end that is, the other term is negative — the
            // marker we have already run past — so the max picks the live one.
            if (Oversize(loco, destTrack))
                return Math.Max(0, Math.Max(-needUp, -needDown));

            // DIRECTIONAL. This used to be Math.Max(0, Math.Max(needUp, needDown)) —
            // an unsigned "how far off the marks am I", handed to a caller that reads
            // it as "how much further do I have to travel". Those agree right up to
            // the moment the train passes the marks, and then they invert: needUp goes
            // negative, needDown goes positive and GROWS with every metre of overshoot,
            // so the reported distance-to-run increases the further past the spot the
            // train gets. ApproachSpeedKmh duly raises the target, the brake comes off,
            // and the train accelerates away down the siding. That is the 60% - 100% -
            // sail past the markers - 60% - accelerate sequence, and a more powerful
            // loco simply leaves faster.
            //
            // Travelling in a known direction there is only ONE live term: the mark we
            // are closing on. Past it, the distance still to run is zero, and it is the
            // overshoot path's business, not this one's.
            bool? going = towardHigh ?? SpotDirection(loco, destTrack)
                ?? HangsOffLowEnd(destTrack, mn, mx);
            double remaining;
            if (going == true)
                remaining = Math.Max(0, needUp);      // closing on the low mark
            else if (going == false)
                remaining = Math.Max(0, needDown);    // closing on the high mark
            else
                remaining = Math.Max(0, Math.Max(needUp, needDown));
            // Cars still off the track have to come on: at minimum the gap above, but
            // never report zero while part of the train is outside.
            return off ? Math.Max(remaining, ParkMargin) : remaining;
        }

        /// <summary>
        /// Which end of the track the part of the cut that is NOT yet on it hangs off:
        /// true when it trails off the LOW end, so travel runs toward increasing span.
        /// Pure geometry, no motion needed — the on-track bogies bunch against the end
        /// the rest of the train is still coming through.
        /// </summary>
        private static bool HangsOffLowEnd(RailTrack destTrack, double mn, double mx)
        {
            double span = PhysicalSpan(destTrack);
            return mn <= span - mx;
        }

        /// <summary>
        /// Which way to shuffle a train that is already ON its destination but not
        /// yet inside the marks: true = toward increasing span, false = toward
        /// decreasing, null = it is where it should be.
        ///
        /// The drive loop used to work this out itself, from the extent of the WHOLE
        /// consist, as `mn &lt; lo || mx &lt;= hi` with an override for `mx &gt; hi &amp;&amp; mn &gt;= lo`.
        /// That is wrong for the one case it most needs to get right: a cut too long
        /// for the window straddles BOTH marks, so `mn &lt; lo` and `mx &gt; hi` hold at the
        /// same time, the first clause answers "deeper", and the override cannot fire
        /// because it demands `mn &gt;= lo`. The train then shoves further in on every
        /// re-plan until the rail-end guard catches it, with the wagons long past both
        /// markers. It also measured a different set of cars than the arrival test
        /// does, so the two could never agree about where "there" was.
        ///
        /// This asks the same question DistanceToSpot answers, over the same cars, so
        /// the direction the train is driven and the test that says it has arrived are
        /// finally the same piece of arithmetic.
        /// </summary>
        private static bool? SpotShuffleDirection(TrainCar loco, RailTrack destTrack)
        {
            var (lo, hi) = MarkerZone(destTrack);
            double m = hi - lo > 2 * ParkMargin ? ParkMargin : 0;
            double mn = double.MaxValue, mx = double.MinValue;
            foreach (var c in CarsToSpot(loco))
                foreach (var b in c.Bogies)
                    if (b?.track == destTrack)
                    {
                        mn = Math.Min(mn, b.traveller.Span);
                        mx = Math.Max(mx, b.traveller.Span);
                    }
            if (mn > mx)
                return null; // none of the spotted set is on it — nothing to shuffle

            if (Oversize(loco, destTrack))
            {
                // It cannot be made to fit, so the job is to put ONE end on its far
                // mark and leave the tail hanging. Pick whichever end is nearer to
                // being right: that terminates, where "always deeper" did not.
                double upMove = (hi - m) - mx;   // + to bring the high end onto its mark
                double downMove = mn - (lo + m); // + to bring the low end onto its mark
                if (Math.Abs(upMove) <= Math.Abs(downMove))
                    return Math.Abs(upMove) <= ParkMargin ? (bool?)null : upMove > 0;
                return Math.Abs(downMove) <= ParkMargin ? (bool?)null : downMove < 0;
            }

            double needUp = (lo + m) - mn;   // tail short of the low mark: go deeper
            double needDown = mx - (hi - m); // front past the high mark: come back
            // The cut fits, so at most one of these can be outstanding.
            if (needUp > ParkMargin)
                return true;
            if (needDown > ParkMargin)
                return false;
            return null;
        }

        /// <summary>
        /// Which way along the destination the spot is travelling: true = toward
        /// increasing span, null while nothing of the spotted set is moving on it.
        /// The overrun test is meaningless without this — "everyone below the low
        /// marker" is an overshoot from one direction and simply NOT THERE YET
        /// from the other, and reading it direction-blind declared a loco still
        /// outside the shed to be 12 m past the marks.
        /// </summary>
        private static bool? SpotDirection(TrainCar loco, RailTrack destTrack)
        {
            foreach (var c in CarsToSpot(loco))
            {
                float fs = c.GetForwardSpeed();
                if (Mathf.Abs(fs) < 0.2f)
                    continue;
                foreach (var b in c.Bogies)
                    if (b != null && b.track == destTrack)
                        return (fs > 0f) == (b.TrackDirectionSign > 0);
            }
            return null;
        }

        /// <summary>
        /// How far the spot has been DRIVEN PAST in the direction it is moving:
        /// the whole spotted set beyond the FAR marker, m. Zero while any of it
        /// is still inside or short of the window. Geometric — "the remaining
        /// distance grew" turned out to fire on bogies crossing joints while the
        /// train was still 20 m out.
        /// </summary>
        private static double SpotOverrun(TrainCar loco, RailTrack destTrack, bool towardHigh)
        {
            if (Oversize(loco, destTrack))
                return 0;
            var (lo, hi) = MarkerZone(destTrack);
            double m = hi - lo > 2 * ParkMargin ? ParkMargin : 0;
            double mn = double.MaxValue, mx = double.MinValue;
            bool off = false;
            foreach (var c in CarsToSpot(loco))
                foreach (var b in c.Bogies)
                {
                    if (b == null)
                        continue;
                    if (b.track != destTrack)
                    {
                        off = true;
                        continue;
                    }
                    mn = Math.Min(mn, b.traveller.Span);
                    mx = Math.Max(mx, b.traveller.Span);
                }
            if (mn > mx)
                return 0;
            // A bogie off the track used to return 0 outright — "part of the train is
            // still outside, so it cannot be past anything". That is true while a cut
            // is running IN, and false for a cut that fills the siding: something is
            // permanently off the end, so the guard returned 0 for ever and the
            // overshoot branch never ran. The train sailed through the marks with
            // nothing watching. Measure the same synthesised extent DistanceToSpot
            // uses instead, so the length hanging off the end is accounted for rather
            // than disabling the check.
            if (off)
            {
                double bogieSpan = Math.Max(0, ConsistLength(loco) - 2 * CarEndOverhang);
                if (towardHigh)
                    mn = Math.Min(mn, mx - bogieSpan);
                else
                    mx = Math.Max(mx, mn + bogieSpan);
            }
            if (towardHigh)
                return mn > hi - m ? mn - (hi - m) : 0; // tail out beyond the high marker
            return mx < lo + m ? (lo + m) - mx : 0;     // tail out beyond the low marker
        }

        /// <summary>
        /// The cars that actually have to end up inside the destination's markers: the
        /// wagons. A loco hanging out over the throat is not a problem — it is leaving
        /// as soon as it has uncoupled, and demanding it fit costs a wagon's worth of
        /// siding on every spot. A light engine has no wagons, so then it is the loco
        /// that must be inside.
        /// </summary>
        private static IEnumerable<TrainCar> CarsToSpot(TrainCar loco)
        {
            // A dead loco hauled in the rake — coupled, no MU cable — is freight
            // and must fit inside the markers with the wagons; only the driven
            // units get to hang out over the throat.
            var driven = MuGroup(loco);
            var wagons = new List<TrainCar>();
            foreach (var c in loco.trainset.cars)
                if (c != null && !driven.Contains(c))
                    wagons.Add(c);
            return wagons.Count > 0 ? wagons : loco.trainset.cars;
        }

        /// <summary>Length of the cars that must be spotted, m.</summary>
        private static double ConsistLength(TrainCar loco)
        {
            double total = 0;
            foreach (var c in CarsToSpot(loco))
                total += c.InterCouplerDistance;
            return total;
        }

        /// <summary>
        /// True when the train is longer than the destination's marker zone, so no
        /// amount of driving will ever get every bogie inside it. Such a move is still
        /// worth making — filling the siding and leaving the tail over the throat is
        /// exactly what a real shunter does — but the arrival tests have to be told,
        /// or the run drives past its destination looking for a fit that cannot happen.
        /// </summary>
        private static bool Oversize(TrainCar loco, RailTrack destTrack)
        {
            var (lo, hi) = MarkerZone(destTrack);
            // Measure the SAME thing ConsistClearWithinDest measures, or the two
            // disagree about whether a spot is even possible.
            //
            // This compared a coupler-to-coupler length against the RAW marker
            // window, while the clearance test compares BOGIE spans against the
            // window inset by ParkMargin at each end. Those differ by about 6 m,
            // and any cut landing in that band was declared "fits" while being
            // impossible to place: L-085's 397 m cut on HB-C2I's 398 m window
            // needed its 393 m of bogie extent inside a 393 m target — zero slack
            // — so Oversize said keep shuffling and the clearance test said not
            // yet, for ever, and the train hunted up and down the siding.
            //
            // Slack on top, because a spot that only succeeds if every bogie lands
            // to the centimetre is not a spot. Below that, treat it as too long:
            // drive one end to its mark, leave the tail, and STOP.
            double m = hi - lo > 2 * ParkMargin ? ParkMargin : 0;
            double usable = Math.Max(0, (hi - m) - (lo + m));
            double bogieExtent = Math.Max(0, ConsistLength(loco) - 2 * CarEndOverhang);
            return bogieExtent > usable - SpotFitSlack;
        }

        /// <summary>True once our train has more cars than it set off with — the
        /// plainest possible test that a coupling actually took, with no guessing about
        /// which train we met or which end we met it by.</summary>
        /// <summary>Did the train we picked up include stock standing on the
        /// DESTINATION? "Couple to the train there" means there - hooking the
        /// first rake met along the way is a mistake to undo, not a success.</summary>
        private static bool GainedTouchesDest(Run run)
        {
            var cars = run.car?.trainset?.cars;
            if (cars == null || run.destTrack == null)
                return false;
            foreach (var c in cars)
            {
                if (c == null || run.members.Contains(c.CarGUID))
                    continue;
                foreach (var b in c.Bogies)
                    if (b?.track == run.destTrack)
                        return true;
            }
            return false;
        }

        private static bool Coupled(Run run)
        {
            int now = run.car?.trainset?.cars?.Count ?? 0;
            return now > run.carsAtStart && GainedTouchesDest(run);
        }

        /// <summary>True when no bogie of our consist is on the given track.</summary>
        private static bool AllOffTrack(TrainCar loco, RailTrack track)
        {
            foreach (var c in loco.trainset.cars)
                foreach (var b in c.Bogies)
                    if (b != null && b.track == track)
                        return false;
            return true;
        }

        /// <summary>True when any car of the consist has a bogie on the destination.</summary>
        private static bool AnyCarOnTrack(TrainCar loco, string destId)
        {
            foreach (var c in loco.trainset.cars)
                foreach (var b in c.Bogies)
                {
                    var id = b?.track?.LogicTrack()?.ID;
                    if (id != null && (id.ToString() == destId || id.FullDisplayID == destId || id.FullID == destId))
                        return true;
                }
            return false;
        }

        /// <summary>True when one of the loco's own bogies sits on the destination.</summary>
        private static bool LocoOnTrack(TrainCar loco, string destId)
        {
            foreach (var b in loco.Bogies)
            {
                var id = b?.track?.LogicTrack()?.ID;
                if (id != null && (id.ToString() == destId || id.FullDisplayID == destId || id.FullID == destId))
                    return true;
            }
            return false;
        }

        /// <summary>True when EVERY bogie of the consist sits within the destination
        /// track's marker zone. The physical rail span runs into the switch approaches
        /// where a car still reads as "on track", so the safe-parking zone is the
        /// shorter logic length (what YardTracksOrganizer reserves against), inset from
        /// each physical end. Whole train inside that inset = last wagon past the
        /// marker, clear of both switches.
        ///
        /// A consist too long for the track is judged the only way it can be: it has
        /// arrived once its leading end is at the far marker, tail wherever it ended up.
        /// </summary>
        private static bool ConsistClearWithinDest(TrainCar loco, string destId, RailTrack? destTrack)
        {
            if (destTrack != null && Oversize(loco, destTrack))
                return DistanceToSpot(loco, destTrack) <= ParkMargin;

            foreach (var c in CarsToSpot(loco))
                foreach (var b in c.Bogies)
                {
                    var track = b?.track;
                    var lt = track?.LogicTrack();
                    var id = lt?.ID;
                    bool onDest = id != null && (id.ToString() == destId || id.FullDisplayID == destId || id.FullID == destId);
                    if (!onDest)
                        return false; // a bogie not (yet) on the destination track

                    var (lo, hi) = MarkerZone(track);
                    // Require a small margin inside the markers so the coast after the
                    // stop command still lands the train within them.
                    double m = hi - lo > 2 * ParkMargin ? ParkMargin : 0;
                    double span = b.traveller.Span;
                    if (span < lo + m || span > hi - m)
                        return false; // this bogie is outside the track markers
                }
            return true;
        }

        /// <summary>
        /// The hardest this train may be accelerated before the driven wheels break
        /// away, m/s². Straight from the weights involved:
        ///
        ///   F_adhesion = μ · g · (weight on driven wheels)
        ///   a_max      = F_adhesion / (weight of the whole train)
        ///
        /// so one loco on a long rake gets a small number and a light engine a large
        /// one, with no invented reference tonnage in between. Spinning the wheels is
        /// what grinds flats onto them, and this is the line not to cross.
        /// </summary>
    }
}
