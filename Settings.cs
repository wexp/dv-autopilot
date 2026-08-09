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
        public float tabletCruiseKmh = 80f;

        /// <summary>Whether a held tablet stays tucked low until looked down at.
        /// Off, it sits in the read pose the whole time it is held.</summary>
        public bool tabletGlance = true;

        // What the autopilot bolts on while it drives. Each part switches off on
        // its own, and its bracket goes with it. The headlights are not part of
        // this: a train running at night unlit is a hazard, not a preference.
        public bool markingBeacons = true;
        public bool markingLightBars = true;

        public LocoProfiles autopilot = new LocoProfiles();

        public bool enableLogging = false;

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
                + "inventory and is held like any item. Point at a window: <b>left click</b> "
                + "acts on a row, <b>right click</b> goes back, the <b>wheel</b> changes the "
                + "value under the cursor. The red button stops every run. "
                + "Key editable in Settings.xml.");
            GUILayout.BeginHorizontal();
            GUILayout.Label("Default road speed", GUILayout.Width(140));
            tabletCruiseKmh = Mathf.Round(
                GUILayout.HorizontalSlider(tabletCruiseKmh, 20f, 120f, GUILayout.Width(120)) / 5f) * 5f;
            GUILayout.Label($"{tabletCruiseKmh:0} km/h", GUILayout.Width(70));
            GUILayout.EndHorizontal();
            GUILayout.Label("What a new road command opens at; shunts are held to yard "
                + "pace whatever this says.");
            GUILayout.Space(6);
            tabletGlance = GUILayout.Toggle(tabletGlance,
                " Tuck the tablet away until glanced at");
            GUILayout.Label(tabletGlance
                ? "Held low while looking ahead, rising to the read pose as you look down."
                : "Always held up in the read pose while carried.");
            GUILayout.EndVertical();
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

        /// <summary>A labelled slider with its current value shown in real units.</summary>
        private static float Slider(
            string label, float value, float min, float max, string unit, string format)
        {
            GUILayout.Label(label, GUILayout.Width(120));
            float v = GUILayout.HorizontalSlider(value, min, max, GUILayout.Width(120));
            GUILayout.Label(v.ToString(format) + " " + unit, GUILayout.Width(70));
            return v;
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
                GUILayout.BeginHorizontal();
                // Stored per 0.1 s tick; shown as % of the handle per second.
                p.throttleUp = Slider("Throttle response",
                    p.throttleUp * 1000f, 2f, 40f, "%/s", "0") / 1000f;
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                p.minMoveThrottle = Slider("Throttle to start",
                    p.minMoveThrottle * 100f, 5f, 60f, "%", "0") / 100f;
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                p.maneuverKmh = Mathf.Round(Slider("Yard speed",
                    p.maneuverKmh, 10f, 40f, "km/h", "0"));
                GUILayout.EndHorizontal();
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
