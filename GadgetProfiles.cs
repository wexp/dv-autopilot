using System.Collections.Generic;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>What kind of gadget a fitting is, which decides how it is lit.</summary>
    internal enum GadgetKind
    {
        /// <summary>A bracket. Nothing to power, nothing to colour.</summary>
        Plain,
        /// <summary>A <c>GadgetLight</c>. Colour lives on its IndicatorEmission.</summary>
        Light,
        /// <summary>A <c>GadgetBeacon</c>. Colour comes from a hue, as the item's does.</summary>
        Beacon,
    }

    /// <summary>One gadget to bolt on: what, where, and what colour.</summary>
    internal readonly struct Fitting
    {
        /// <summary>Prefab name, exactly as it appears in gadgets.txt — e.g.
        /// "LightBar_Gadget". The trailing "(Clone)" forms are instances, never
        /// these.</summary>
        internal readonly string Prefab;
        internal readonly GadgetKind Kind;
        /// <summary>Position in the CAR's frame, metres.</summary>
        internal readonly Vector3 Position;
        /// <summary>Rotation in the car's frame, Euler degrees.</summary>
        internal readonly Vector3 Euler;
        /// <summary>Unused by marker boards, whose colour is chosen by state. Kept
        /// for any fixed-colour light a profile may want later.</summary>
        internal readonly Color Colour;
        /// <summary>Beacons only: the hue the matching shop item carries, degrees.
        /// Saturation stays the prefab's own, as the game does it.</summary>
        internal readonly float Hue;
        /// <summary>
        /// The catalogue item to take the gadget's SPECIFIC MATERIAL from, or null
        /// for the prefab's own. This is the only thing an item's appearance is good
        /// for here: a beacon's glass is a material carried by the item, not by the
        /// prefab — every beacon in the game is the same prefab and the dome is
        /// swapped in when the item is assigned. Ours have no item, so the swap is
        /// made by hand from the named one.
        /// </summary>
        internal readonly string? Item;
        /// <summary>
        /// Which END of the car this marker board is on: true for the +z end, false
        /// for -z, null for anything that is not a marker board. There is one board
        /// per end and it never goes dark — what changes is its COLOUR, and which
        /// colour depends on whether its end is currently the leading one. See
        /// <see cref="GadgetProfiles.MarkerColour"/>.
        /// </summary>
        internal readonly bool? AtFront;

        internal Fitting(string prefab, GadgetKind kind, Vector3 position, Vector3 euler,
            Color colour = default, float hue = 0f, bool? atFront = null,
            string? item = null)
        {
            Prefab = prefab;
            Kind = kind;
            Position = position;
            Euler = euler;
            Colour = colour;
            Hue = hue;
            AtFront = atFront;
            Item = item;
        }
    }

    /// <summary>What the markings are currently saying about the run.</summary>
    internal enum MarkingState
    {
        /// <summary>Under way. Which colour a board shows depends on whether its
        /// end is the leading one.</summary>
        Driving,
        /// <summary>The run gave up. Both ends red, whichever way it faces.</summary>
        Error,
        /// <summary>The run arrived. Both ends green, briefly, then the whole set
        /// comes off.</summary>
        Finished,
    }

    /// <summary>Everything the autopilot puts on one type of locomotive.</summary>
    internal readonly struct LiveryProfile
    {
        internal readonly Fitting[] Fittings;
        /// <summary>
        /// Where to put the headlight switch at each end, as NOTCHES either side of
        /// the switch's own off position. Notches, not port values, because the
        /// number of positions differs from loco to loco: two below off on a
        /// six-position switch is a different port value than on a four-position
        /// one, and the offset is the thing that stays meaningful.
        ///
        /// Positive is toward the white beams, negative toward the markers.
        /// </summary>
        internal readonly int LeadingNotch;
        internal readonly int TrailingNotch;

        internal LiveryProfile(Fitting[] fittings, int leadingNotch, int trailingNotch)
        {
            Fittings = fittings;
            LeadingNotch = leadingNotch;
            TrailingNotch = trailingNotch;
        }
    }

    /// <summary>
    /// Where the autopilot's markings go on each type of locomotive, and what
    /// colour they are.
    ///
    /// Every number here was MEASURED off a loco with the gadgets bolted on by
    /// hand and read back with the "Dump gadgets" button — positions are the
    /// `at car-local` line, colours are the item attributes from the prefab
    /// catalogue. Nothing is estimated, so nothing here should be adjusted by
    /// eye: fit them where you want them in game, dump, and copy the numbers.
    ///
    /// A livery with no profile falls back to a single beacon on the middle of
    /// the roof, worked out from the car's collision bounds. It will be roughly
    /// right and never exactly right, which is the point of measuring.
    /// </summary>
    internal static class GadgetProfiles
    {
        // Item colours, straight out of the catalogue in gadgets.txt.
        internal static readonly Color LightBarRed = new Color(1f, 0f, 0f);
        internal static readonly Color LightBarGreen = new Color(0f, 0.854902f, 0.1803922f);
        internal static readonly Color LightBarWhite = new Color(1f, 1f, 1f);
        // Beacon HUE attributes. Red and blue are the shop items' own values —
        // BeaconRed is 7 rather than 0 because the game's red beacon is slightly
        // orange. No green beacon is sold, so 120 is ours.
        internal const float BeaconRedHue = 7f;
        internal const float BeaconBlueHue = 222f;
        internal const float BeaconGreenHue = 120f;
        // Catalogue items the beacon DOMES come from. Three are sold — amber, blue,
        // red — and that is the whole choice of glass there is: the colour of the
        // dome is a material on the item, not a number, so a hue with no item
        // behind it cannot have matching glass.
        internal const string BeaconBlueItem = "BeaconBlue";
        internal const string BeaconRedItem = "BeaconRed";
        internal const string BeaconAmberItem = "BeaconAmber";

        private const string MountLong = "MountVeryLong_Gadget";
        private const string MountSmall = "MountSmall_Gadget";
        private const string LightBar = "LightBar_Gadget";
        private const string BeaconPrefab = "Beacon_Gadget";

        private static Fitting Mount(string prefab, Vector3 at, Vector3 euler) =>
            new Fitting(prefab, GadgetKind.Plain, at, euler);

        /// <summary>One marker board. <paramref name="atFront"/> says which end of
        /// the car it is on; the colour it shows is decided at run time.</summary>
        private static Fitting Light(Vector3 at, Vector3 euler, bool atFront) =>
            new Fitting(LightBar, GadgetKind.Light, at, euler, default, 0f, atFront);

        /// <summary>
        /// What a marker board shows. One board per end, never dark:
        ///
        ///   driving  — WHITE at the leading end, RED at the trailing one, which is
        ///              how a train is lit anywhere it runs
        ///   error    — RED both ends: stopped and not going anywhere
        ///   finished — GREEN both ends, for the few seconds before the set comes off
        /// </summary>
        internal static Color MarkerColour(MarkingState state, bool atFront, bool forward)
        {
            switch (state)
            {
                case MarkingState.Finished:
                    return LightBarGreen;
                case MarkingState.Error:
                    return LightBarRed;
                default:
                    return atFront == forward ? LightBarWhite : LightBarRed;
            }
        }

        private static Fitting Beacon(Vector3 at, Vector3 euler, float hue,
            string? item = null) =>
            new Fitting(BeaconPrefab, GadgetKind.Beacon, at, euler, default, hue,
                atFront: null, item: item);

        /// <summary>
        /// Keyed on <c>carLivery.id</c> — "LocoDH4", not the car number. Mounts
        /// come before the gadget that sits on them so the bracket is there
        /// first; the game does not enforce that, but reading the list in
        /// build order is easier when something is out of place.
        /// </summary>
        private static readonly Dictionary<string, LiveryProfile> byLivery =
            new Dictionary<string, LiveryProfile>
            {
                // Measured on L-027: one marker board at each end of the hood, and
                // the beacon on the cab roof, offset to the rear.
                //
                // X is ZERO everywhere on purpose. A car's local origin is on its
                // centreline, and all of these belong on it — the measurements came
                // back a few millimetres off (0.001, -0.004, -0.006) because they
                // were placed by hand against a surface, and copying that jitter in
                // would make every DH4 in the game wear a crooked set. Height and
                // position along the car are real and stay exactly as dumped.
                //
                // Each bracket keeps the 1-2 mm standoff its board was measured at,
                // so a board that moves takes its bracket with it.
                ["LocoDH4"] = new LiveryProfile(
                    new[]
                    {
                        // Front (+z).
                        Mount(MountLong, new Vector3(0f, 2.829f, 5.320f), new Vector3(3.2f, 180f, 0f)),
                        Light(new Vector3(0f, 2.831f, 5.347f), new Vector3(3.2f, 180f, 0f), atFront: true),

                        // Rear (-z).
                        Mount(MountLong, new Vector3(0f, 2.840f, -5.307f), new Vector3(3.0f, 359.9f, 0f)),
                        Light(new Vector3(0f, 2.842f, -5.334f), new Vector3(3.0f, 359.9f, 0f), atFront: false),

                        // Cab roof: TWO beacons in the one place, blue and green,
                        // their heads turning independently. One bracket — a second
                        // would only clip through the first.
                        //
                        // Both wear the BLUE dome: the glass comes from an item and
                        // no green beacon is sold, so green is a light shining
                        // through blue glass. A green head under a clear dome is
                        // not on offer either — the prefab's own glass is whatever
                        // the artist left on it, not neutral.
                        Mount(MountSmall, new Vector3(0f, 4.341f, -2.317f), new Vector3(90f, 359.8f, 0f)),
                        Beacon(new Vector3(0f, 4.334f, -2.317f), new Vector3(90f, 359.8f, 0f), BeaconBlueHue, BeaconBlueItem),
                        Beacon(new Vector3(0f, 4.334f, -2.317f), new Vector3(90f, 359.8f, 0f), BeaconGreenHue, BeaconBlueItem),
                    },
                    leadingNotch: 2,
                    trailingNotch: -1),

                // Measured on L-073. Same treatment: X zeroed to the centreline, one
                // board per end.
                ["LocoDE2"] = new LiveryProfile(
                    new[]
                    {
                        // Front (+z).
                        Mount(MountLong, new Vector3(0f, 1.477f, 3.205f), new Vector3(0f, 180f, 0f)),
                        Light(new Vector3(0f, 1.477f, 3.232f), new Vector3(0f, 180f, 0f), atFront: true),

                        // Rear (-z).
                        Mount(MountLong, new Vector3(0f, 1.388f, -3.233f), Vector3.zero),
                        Light(new Vector3(0f, 1.388f, -3.260f), Vector3.zero, atFront: false),

                        // Cab roof, blue and green as on the DH4. The beacons sit a
                        // quarter turn round from that loco's — that is the mount's
                        // own orientation, not jitter, so only the stray 0.6 degrees
                        // came off it.
                        Mount(MountSmall, new Vector3(0f, 3.833f, -0.148f), new Vector3(90f, 90f, 0f)),
                        Beacon(new Vector3(0f, 3.825f, -0.148f), new Vector3(90f, 90f, 0f), BeaconBlueHue, BeaconBlueItem),
                        Beacon(new Vector3(0f, 3.825f, -0.148f), new Vector3(90f, 90f, 0f), BeaconGreenHue, BeaconBlueItem),
                    },
                    leadingNotch: 2,
                    trailingNotch: -1),

                // Measured on L-075, spot by spot: a small mount is parked where
                // each piece goes and dumped, so the POSITIONS are real. X zeroed
                // to the centreline as everywhere (the placements came back
                // 0.010-0.016 off). What is NOT measured is the derived bits, each
                // taken from the measured pairs above and called out where used —
                // if something sits crooked, place the actual piece by hand, dump,
                // and copy the numbers.
                //
                // The beacons sit 8 mm below the mount's own origin: that is the
                // beacon-to-bracket drop BOTH measured pairs above came back with,
                // not a number of ours — only the mount was dumped on the DE6.
                //
                // Rear board: the marker-spot mount read euler 0,0,270 — a small
                // mount's own frame on a vertical wall, not the board's. The board
                // and its long bracket take the rear-facing orientation the DE2's
                // measured rear pair carries (identity), and the 27 mm
                // board-off-bracket standoff both measured liveries agree on.
                ["LocoDE6"] = new LiveryProfile(
                    new[]
                    {
                        // Both ends rounded to 1.5 / 8.8 by request — the dumps
                        // read 1.467-1.503 and 8.785-8.805, so the boards sit
                        // symmetric within 35 mm of every measurement.
                        //
                        // Front (+z).
                        Mount(MountLong, new Vector3(0f, 1.5f, 8.8f), new Vector3(0f, 180f, 0f)),
                        Light(new Vector3(0f, 1.5f, 8.827f), new Vector3(0f, 180f, 0f), atFront: true),

                        // Rear (-z).
                        Mount(MountLong, new Vector3(0f, 1.5f, -8.8f), Vector3.zero),
                        Light(new Vector3(0f, 1.5f, -8.827f), Vector3.zero, atFront: false),

                        Mount(MountSmall, new Vector3(0f, 3.934f, 6.813f), new Vector3(90f, 270f, 0f)),
                        Beacon(new Vector3(0f, 3.926f, 6.813f), new Vector3(90f, 270f, 0f), BeaconBlueHue, BeaconBlueItem),
                        Beacon(new Vector3(0f, 3.926f, 6.813f), new Vector3(90f, 270f, 0f), BeaconGreenHue, BeaconBlueItem),
                    },
                    leadingNotch: 2,
                    trailingNotch: -1),

                // Measured on L-029 (the BE2): one marker spot dumped at the
                // front, the rear is its mirror by request; the beacon spot's
                // height is dumped and the beacons sit centred on the roof both
                // ways. X zeroed throughout, board orientation and 27 mm standoff
                // derived as on the DE6, beacon drop the measured 8 mm. The
                // headlight switch has 4 positions with off at 1, so the notches
                // still fit: +2 is high beam, -1 the markers.
                ["LocoMicroshunter"] = new LiveryProfile(
                    new[]
                    {
                        // Front (+z).
                        Mount(MountLong, new Vector3(0f, 1.448f, 1.520f), new Vector3(0f, 180f, 0f)),
                        Light(new Vector3(0f, 1.448f, 1.547f), new Vector3(0f, 180f, 0f), atFront: true),

                        // Rear (-z).
                        Mount(MountLong, new Vector3(0f, 1.448f, -1.520f), Vector3.zero),
                        Light(new Vector3(0f, 1.448f, -1.547f), Vector3.zero, atFront: false),

                        // Roof centre, blue and green as everywhere.
                        Mount(MountSmall, new Vector3(0f, 2.855f, 0f), new Vector3(90f, 0f, 0f)),
                        Beacon(new Vector3(0f, 2.847f, 0f), new Vector3(90f, 0f, 0f), BeaconBlueHue, BeaconBlueItem),
                        Beacon(new Vector3(0f, 2.847f, 0f), new Vector3(90f, 0f, 0f), BeaconGreenHue, BeaconBlueItem),
                    },
                    leadingNotch: 2,
                    trailingNotch: -1),

                // Measured on L-005, BOTH ends dumped — mirroring the front spot
                // did not fit this body either, so the rear has its own numbers. X
                // zeroed (read -0.011 and 0.024), orientation and standoff derived.
                // Its switch is 3 positions with off at 1, so the +2 clamps to the
                // one white setup it has.
                ["LocoDM1U"] = new LiveryProfile(
                    new[]
                    {
                        // Front (+z).
                        Mount(MountLong, new Vector3(0f, 1.546f, 6.641f), new Vector3(0f, 180f, 0f)),
                        Light(new Vector3(0f, 1.546f, 6.668f), new Vector3(0f, 180f, 0f), atFront: true),

                        // Rear (-z). Distance along the car measured at this end;
                        // height shared with the front boards.
                        Mount(MountLong, new Vector3(0f, 1.546f, -6.471f), Vector3.zero),
                        Light(new Vector3(0f, 1.546f, -6.498f), Vector3.zero, atFront: false),
                    },
                    leadingNotch: 2,
                    trailingNotch: -1),

                // Measured on L-066, BOTH ends dumped — the DM3 is not symmetric:
                // mirroring the front spot put the rear board inside the cab, so
                // the rear got its own measurement, further back and lower on the
                // hood. X zeroed (read 0.003 both times), orientation and standoff
                // derived.
                ["LocoDM3"] = new LiveryProfile(
                    new[]
                    {
                        // Front (+z).
                        Mount(MountLong, new Vector3(0f, 2.760f, 3.402f), new Vector3(0f, 180f, 0f)),
                        Light(new Vector3(0f, 2.760f, 3.429f), new Vector3(0f, 180f, 0f), atFront: true),

                        // Rear (-z). Distance along the car measured at this end;
                        // height shared with the front boards.
                        Mount(MountLong, new Vector3(0f, 2.760f, -3.705f), Vector3.zero),
                        Light(new Vector3(0f, 2.760f, -3.732f), Vector3.zero, atFront: false),
                    },
                    leadingNotch: 2,
                    trailingNotch: -1),
            };

        /// <summary>The markings for this livery, or null when none are measured
        /// yet and the caller should fall back to the estimated beacon.</summary>
        internal static LiveryProfile? For(string? liveryId) =>
            liveryId != null && byLivery.TryGetValue(liveryId, out var profile)
                ? profile
                : (LiveryProfile?)null;

        /// <summary>
        /// The stand-in for an unmeasured livery: one red beacon on the middle of
        /// the roof, from the car's own collision bounds so it lands on the shell
        /// of whatever it is rather than on a number copied off a DH4.
        /// </summary>
        internal static LiveryProfile Estimated(TrainCar car)
        {
            var bounds = car.Bounds;
            // Clear of the roof, so it reads as bolted on rather than sunk in.
            var beacon = Beacon(
                new Vector3(0f, bounds.max.y + 0.02f, bounds.center.z),
                Vector3.zero,
                BeaconBlueHue,
                BeaconBlueItem);
            // Headlights still worth driving: the notches mean the same thing on
            // every switch, whatever the marker boards do.
            return new LiveryProfile(new[] { beacon }, leadingNotch: 2, trailingNotch: -1);
        }
    }
}
