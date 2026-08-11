using System;
using System.Collections.Generic;
using UnityEngine;
using UnityModManagerNet;

namespace DvMod.Autopilot
{
    public class Settings : UnityModManager.ModSettings
    {
        /* --------------------------------------------------------- stored -- */

        /// <summary>Key that hands out a dispatcher tablet. Editable in
        /// Settings.xml; named in the panel so nobody has to guess.</summary>
        public KeyCode tabletKey = KeyCode.F9;

        /// <summary>Top speed a new road command starts at, km/h. Only the
        /// tablet's opening figure — each command can be changed on its own.</summary>
        public float tabletCruiseKmh = 60f;

        /// <summary>Where the hands hold the tablet, written to the item's own
        /// CustomNonVrGrabAnchor. The frame is the game's, so these were tuned
        /// by eye off the map's own values (-0.1, 0.1, 0 / -90, 0, 0).</summary>
        public float tabletHoldX = -0.16f;
        public float tabletHoldY = 0.14f;
        public float tabletHoldZ = -0.08f;
        public float tabletHoldPitch = -90f;
        public float tabletHoldYaw = 0f;
        public float tabletHoldRoll = 0f;

        /// <summary>Whether a held tablet drops out of the way until looked
        /// down at. Off, it stays in the hold pose above the whole time.</summary>
        public bool tabletGlance = true;
        /// <summary>How far the head has to be looking down before the tablet
        /// starts coming up, degrees below the horizon.</summary>
        public float tabletGlanceStart = 20f;
        /// <summary>How far down it has to be for the tablet to be all the way
        /// up, degrees below the horizon.</summary>
        public float tabletGlanceFull = 30f;
        /// <summary>How far below the hold pose the tucked tablet sits.</summary>
        public float tabletTuckDrop = 0.12f;
        /// <summary>How far back (or forward) the tucked tablet sits.</summary>
        public float tabletTuckPush = 0.11f;
        /// <summary>How far it leans away while tucked, degrees.</summary>
        public float tabletTuckLean = 20f;

        // What the autopilot bolts on while it drives. Each part switches off on
        // its own, and its bracket goes with it. The headlights are not part of
        // this: a train running at night unlit is a hazard, not a preference.
        public bool markingBeacons = true;
        public bool markingLightBars = true;

        public LocoProfiles autopilot = new LocoProfiles();

        public bool enableLogging = false;

        // The interlocking trace, kept apart from enableLogging above. That one
        // writes occasional notes into a log the whole game shares; this one
        // writes every metre and every claim into a file of its own, which is
        // the only way a lock-up between two trains can be read back afterwards
        // rather than argued about.
        // ON by default while the interlocking is under investigation: a recorder
        // that has to be remembered is a recorder that is off for the run you
        // needed it for. Untick it when the locks are fixed.
        public bool traceEnabled = true;
        /// <summary>Metres of travel between movement lines. Smaller is finer and
        /// bigger; 1 m keeps a whole Harbour session inside a few tens of MB.</summary>
        public float traceStepMeters = 1.0f;

        public readonly string? version = Main.mod?.Info.Version;

        /* ------------------------------------------------------ panel UI -- */

        // Which sections stand open. Private, so UMM's serializer leaves them
        // out of Settings.xml: panel state is not a setting.
        private bool showTablet = true;
        private bool showMarkings = true;
        private bool showProfiles;
        private bool showTools;

        public void Draw()
        {
            GUILayout.BeginVertical(GUILayout.ExpandWidth(false));

            if (Section("Dispatcher tablet", ref showTablet))
                DrawTablet();
            if (Section("Autopilot markings", ref showMarkings))
                DrawMarkings();
            if (Section("Driving profiles", ref showProfiles))
                autopilot.Draw();
            if (Section("Diagnostics", ref showTools))
                DrawDiagnostics();

            GUILayout.EndVertical();
        }

        /// <summary>A collapsible section header. Everything in this panel is
        /// optional detail except the section you are in, and the profiles alone
        /// are eight boxes of sliders — shown all at once they buried the rest.</summary>
        private static bool Section(string title, ref bool open)
        {
            GUILayout.Space(6);
            if (GUILayout.Button((open ? "▼  " : "►  ") + title,
                    GUILayout.Width(260)))
                open = !open;
            return open;
        }

        private void DrawTablet()
        {
            GUILayout.BeginVertical("box", GUILayout.ExpandWidth(false));
            GUILayout.Label("Press <b>" + tabletKey + "</b> for a tablet — it goes in the "
                + "inventory and is held like any item, and there is only ever one. With "
                + "the cursor showing, point at a window: <b>left click</b> acts on a row, "
                + "the <b>wheel</b> changes the value under the cursor. "
                + "The red button stops every run. "
                + "Key editable in Settings.xml.");
            tabletCruiseKmh = Num("Default road speed",
                tabletCruiseKmh, 20f, 120f, 5f, "0", "km/h");
            GUILayout.Label("What a new road command opens at; shunts are held to yard "
                + "pace whatever this says.");

            GUILayout.Space(8);
            GUILayout.Label("<b>Where it sits in the hands</b> — the game's own hold pose "
                + "for the item, so it still swings in and away on its own. Applies while "
                + "you watch.");
            tabletHoldX = Num("Left / right", tabletHoldX, -0.5f, 0.5f, 0.01f, "0.00", "m");
            tabletHoldY = Num("Down / up", tabletHoldY, -0.5f, 0.5f, 0.01f, "0.00", "m");
            tabletHoldZ = Num("Back / forward", tabletHoldZ, -0.5f, 0.5f, 0.01f, "0.00", "m");
            tabletHoldPitch = Num("Pitch", tabletHoldPitch, -180f, 180f, 1f, "0", "°");
            tabletHoldYaw = Num("Yaw", tabletHoldYaw, -180f, 180f, 1f, "0", "°");
            tabletHoldRoll = Num("Roll", tabletHoldRoll, -180f, 180f, 1f, "0", "°");
            GUILayout.Space(6);
            tabletGlance = GUILayout.Toggle(tabletGlance,
                " Drop the tablet out of the way until glanced at");
            if (tabletGlance)
            {
                GUILayout.Label("Carried low while looking ahead, coming up to the hold "
                    + "pose above as you look down at it.");
                tabletGlanceStart = Num("Starts at",
                    tabletGlanceStart, 0f, 80f, 1f, "0", "° down");
                tabletGlanceFull = Num("Fully up at",
                    tabletGlanceFull, 1f, 89f, 1f, "0", "° down");
                tabletTuckDrop = Num("Tucked drop",
                    tabletTuckDrop, 0f, 0.6f, 0.01f, "0.00", "m");
                tabletTuckPush = Num("Tucked back / fwd",
                    tabletTuckPush, -0.5f, 0.5f, 0.01f, "0.00", "m");
                tabletTuckLean = Num("Tucked lean",
                    tabletTuckLean, -90f, 90f, 1f, "0", "°");
            }
            else
            {
                GUILayout.Label("Always held in the pose above while carried.");
            }

            GUILayout.Space(6);
            if (GUILayout.Button("Reset the hold pose", GUILayout.Width(200)))
            {
                tabletHoldX = -0.16f;
                tabletHoldY = 0.14f;
                tabletHoldZ = -0.08f;
                tabletHoldPitch = -90f;
                tabletHoldYaw = 0f;
                tabletHoldRoll = 0f;
                tabletGlanceStart = 20f;
                tabletGlanceFull = 30f;
                tabletTuckDrop = 0.12f;
                tabletTuckPush = 0.11f;
                tabletTuckLean = 20f;
            }
            GUILayout.EndVertical();
        }

        /// <summary>
        /// A number row: minus, slider, plus, value. Everything is snapped to
        /// the step, so a drag lands on whole values instead of skipping the
        /// one wanted, and the buttons walk to an exact figure the slider
        /// cannot be aimed at.
        /// </summary>
        internal static float Num(string label, float value, float min, float max,
            float step, string format, string unit = "")
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, GUILayout.Width(120));
            if (GUILayout.Button("−", GUILayout.Width(24)))
                value -= step;
            // What the slider was HANDED, not what the value is by the end of
            // the row: the plus button comes after it and moves the value on,
            // and comparing against that made every press look like a drag
            // back to where it started.
            float shown = value;
            float dragged = GUILayout.HorizontalSlider(
                shown, min, max, GUILayout.Width(130));
            if (GUILayout.Button("+", GUILayout.Width(24)))
                value += step;
            // A slider that has not been touched hands back what it was given,
            // so any change to it is a drag and wins over the buttons.
            if (Mathf.Abs(dragged - shown) > step * 0.001f)
                value = dragged;
            value = Mathf.Clamp(Mathf.Round(value / step) * step, min, max);
            GUILayout.Label(value.ToString(format) + " " + unit, GUILayout.Width(60));
            GUILayout.EndHorizontal();
            return value;
        }

        private void DrawMarkings()
        {
            GUILayout.BeginVertical("box", GUILayout.ExpandWidth(false));
            GUILayout.Label("What a self-driving loco wears, so it can be told apart from a "
                + "driven one. Takes effect on the next run; a set already on a loco stays "
                + "until that run ends.");
            markingBeacons = GUILayout.Toggle(markingBeacons,
                " Rotating beacons on the cab roof");
            markingLightBars = GUILayout.Toggle(markingLightBars,
                " Marker boards at each end (white leading, red trailing)");
            GUILayout.EndVertical();
        }

        /// <summary>Last report of each dump, so the button says what it did
        /// instead of leaving the only evidence in Player.log.</summary>
        private string dumpReport = "";
        private string controlReport = "";
        private string gadgetReport = "";

        /// <summary>
        /// Debug logging and the three world dumps. Nothing here changes how a
        /// train is driven — it is all for finding out what the game contains.
        ///
        /// Only tracks the game has STREAMED IN exist to be found, which is why
        /// the map report leads with the count: two dumps taken in different
        /// places that disagree mean the map is partial, and there is no way to
        /// tell that from the file itself.
        /// </summary>
        private void DrawDiagnostics()
        {
            GUILayout.BeginVertical("box", GUILayout.ExpandWidth(false));
            enableLogging = GUILayout.Toggle(enableLogging,
                " Write debug detail to the mod log (Player.log)");

            GUILayout.Space(8);
            bool wantTrace = GUILayout.Toggle(traceEnabled,
                " Trace every movement and every claim to autopilot-trace.log");
            GUILayout.Label("    One file per session in the mod folder: position each "
                + "metre, every block asked for, the rule that refused it, and every "
                + "change of owner. Large — tens of MB an hour with trains out.");
            if (wantTrace != traceEnabled)
            {
                traceEnabled = wantTrace;
                // Opened here rather than only on load, so the switch takes effect
                // in the session it was thrown in — the lock you want to catch is
                // usually already brewing by the time you come looking for it.
                if (wantTrace)
                    Trace.Open();
                else
                    Trace.Close();
            }
            GUILayout.BeginHorizontal();
            GUILayout.Label("    Movement line every", GUILayout.Width(150));
            traceStepMeters = Mathf.Clamp(
                float.TryParse(
                    GUILayout.TextField(
                        traceStepMeters.ToString("F2"), GUILayout.Width(60)),
                    out float step) ? step : traceStepMeters,
                0.1f, 50f);
            GUILayout.Label("m");
            GUILayout.EndHorizontal();

            GUILayout.Space(8);
            DrawDump("Dump map + state",
                "The rail network to map.json / state.json in the mod folder, for running "
                + "the autopilot outside the game.",
                ref dumpReport, () => MapDump.Write(null));

            GUILayout.Space(8);
            DrawDump("Dump loco controls",
                "What handles and simulation ports each locomotive type actually has, to "
                + "controls.txt. The only way to find out what a machine needs that the "
                + "drive loop does not yet touch (a gearbox, a cutoff).",
                ref controlReport, () => ControlDump.Write(null));

            GUILayout.Space(8);
            DrawDump("Dump gadgets",
                "Every gadget fitted to the cars nearby and every gadget prefab the game has "
                + "loaded, to gadgets.txt. The prefab names live in the asset bundles, not in "
                + "the code, so a live world is the only place to read them.",
                ref gadgetReport, () => GadgetDump.Write(null));
            GUILayout.EndVertical();
        }

        private static void DrawDump(
            string button, string what, ref string report, Func<string> write)
        {
            GUILayout.Label(what);
            if (GUILayout.Button(button, GUILayout.Width(200)))
                report = write();
            if (report.Length > 0)
                GUILayout.Label(report);
        }

        public override void Save(UnityModManager.ModEntry entry)
        {
            Save<Settings>(this, entry);
        }
    }

    /// <summary>
    /// How hard the autopilot may drive each kind of locomotive. Throttle is commanded
    /// as a fraction of the handle, but the same fraction does very different things on
    /// different machines: a shunter needs a bigger share to move at all, a road loco
    /// with a heavy train needs a slow hand to keep its feet, and each has its own
    /// notching. One profile per livery, editable here and in Settings.xml.
    /// </summary>
    public class LocoProfile
    {
        /// <summary>Livery id as the game reports it, e.g. "LocoDE6". Empty = default.</summary>
        public string livery = "";
        /// <summary>Opening rate per 0.1 s tick.</summary>
        public float throttleUp = 0.012f;
        /// <summary>Below this the coarsest notch gives nothing, so a start commands
        /// at least this much.</summary>
        public float minMoveThrottle = 0.25f;
        /// <summary>Top speed for yard moves, km/h.</summary>
        public float maneuverKmh = 25f;

        public LocoProfile() { }

        public LocoProfile(string livery, float throttleUp,
            float minMoveThrottle, float maneuverKmh)
        {
            this.livery = livery;
            this.throttleUp = throttleUp;
            this.minMoveThrottle = minMoveThrottle;
            this.maneuverKmh = maneuverKmh;
        }
    }

    public class LocoProfiles
    {
        /// <summary>
        /// Bumped whenever the built-in profiles change. A saved Settings.xml wins over
        /// code defaults, and UMM writes the file on unload — so without this a tuned
        /// default could never reach an existing install: the reload saved the stale
        /// values back over it every time.
        /// </summary>
        private const int CurrentVersion = 4;

        public int version;
        public List<LocoProfile> profiles = new List<LocoProfile>();

        /// <summary>Replace saved profiles with the current defaults when those defaults
        /// have moved on. Hand-tuned values are lost at that point, which is the price
        /// of being able to ship a better starting point at all.</summary>
        public void MigrateDefaults()
        {
            if (version >= CurrentVersion && profiles.Count > 0)
                return;
            profiles.Clear();
            EnsureDefaults();
            version = CurrentVersion;
        }

        /// <summary>Sensible starting points per stock loco. Anything not listed —
        /// including custom (CCL) locos — falls back to the empty-livery default.</summary>
        private static readonly LocoProfile[] Defaults =
        {
            new LocoProfile("", 0.012f, 0.25f, 25f),
            // The DE2 is a light shunter: it gets the gentlest opening of the fleet so
            // it does not throw itself about, but the full handle is available once
            // rolling — the adhesion limiter, not a low ceiling, is what keeps its
            // wheels down.
            new LocoProfile("LocoDE2", 0.006f, 0.22f, 20f),
            new LocoProfile("LocoDE6", 0.012f, 0.25f, 25f),
            new LocoProfile("LocoDH4", 0.015f, 0.30f, 25f),
            new LocoProfile("LocoDM3", 0.015f, 0.30f, 25f),
            new LocoProfile("LocoBE2", 0.025f, 0.40f, 20f),
            new LocoProfile("LocoS060", 0.010f, 0.25f, 20f),
            new LocoProfile("LocoS282A", 0.010f, 0.25f, 20f),
        };

        public LocoProfile For(string? livery)
        {
            EnsureDefaults();
            return profiles.Find(p => p.livery == livery)
                ?? profiles.Find(p => p.livery.Length == 0)
                ?? Defaults[0];
        }

        internal void EnsureDefaults()
        {
            if (profiles.Count > 0)
                return;
            foreach (var d in Defaults)
                profiles.Add(new LocoProfile(d.livery, d.throttleUp,
                    d.minMoveThrottle, d.maneuverKmh));
        }

        /// <summary>"LocoDE6" → "DE6"; the empty livery is the catch-all.</summary>
        private static string FriendlyName(string livery) =>
            livery.Length == 0 ? "All other locos"
            : livery.StartsWith("Loco") ? livery.Substring(4) : livery;

        public void Draw()
        {
            EnsureDefaults();
            GUILayout.Label("How the autopilot handles each locomotive type.");
            GUILayout.Label(
                "Throttle response: how quickly the handle opens (higher = livelier, "
                + "more wheelslip risk).\nThrottle to start: the least that gets that "
                + "machine rolling.\nYard speed: top speed for shunting moves.");
            GUILayout.Space(4);
            foreach (var p in profiles)
            {
                GUILayout.BeginVertical("box", GUILayout.ExpandWidth(false));
                GUILayout.Label("<b>" + FriendlyName(p.livery) + "</b>");
                // Stored per 0.1 s tick; shown as % of the handle per second.
                p.throttleUp = Settings.Num("Throttle response",
                    p.throttleUp * 1000f, 2f, 40f, 1f, "0", "%/s") / 1000f;
                p.minMoveThrottle = Settings.Num("Throttle to start",
                    p.minMoveThrottle * 100f, 5f, 60f, 1f, "0", "%") / 100f;
                p.maneuverKmh = Settings.Num("Yard speed",
                    p.maneuverKmh, 10f, 40f, 1f, "0", "km/h");
                GUILayout.EndVertical();
            }
            GUILayout.Space(4);
            if (GUILayout.Button("Reset all profiles to defaults", GUILayout.Width(200)))
            {
                profiles.Clear();
                EnsureDefaults();
            }
        }
    }
}
