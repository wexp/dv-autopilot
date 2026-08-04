using System.Collections.Generic;
using UnityEngine;
using UnityModManagerNet;

namespace DvMod.Autopilot
{
    public class Settings : UnityModManager.ModSettings
    {
        public bool enableLogging = false;
        public LocoProfiles autopilot = new LocoProfiles();
        // The dispatcher tablet's keys. Editable in Settings.xml; shown in the
        // panel as text so nobody has to guess.
        public KeyCode tabletKey = KeyCode.F9;
        public KeyCode tabletConfirmKey = KeyCode.Return;
        public KeyCode tabletBackKey = KeyCode.Backspace;
        public float tabletCruiseKmh = 80f;

        public readonly string? version = Main.mod?.Info.Version;

        public void Draw()
        {
            GUILayout.BeginVertical(GUILayout.ExpandWidth(false));
            autopilot.Draw();
            GUILayout.Space(8);
            GUILayout.Label("<b>Dispatcher tablet</b> — press " + tabletKey
                + " to receive one; hold or look at it, Shift+wheel moves, "
                + tabletConfirmKey + " confirms, " + tabletBackKey + " goes back. "
                + $"Road runs dispatch at {tabletCruiseKmh:0} km/h. "
                + "Keys editable in Settings.xml.");
            GUILayout.Space(8);
            enableLogging = GUILayout.Toggle(enableLogging,
                " Write debug detail to the mod log (Player.log)");
            GUILayout.Space(8);
            DrawMapDump();
            GUILayout.EndVertical();
        }

        /// <summary>Last dump's report, so the button says what it did instead of
        /// leaving the only evidence in Player.log.</summary>
        private string dumpReport = "";

        /// <summary>
        /// Writes the rail network out for the offline simulator. Only tracks the
        /// game has STREAMED IN exist to be found, so the report leads with the
        /// count: two dumps taken in different places that disagree mean the map is
        /// partial, and there is no way to tell that from the file itself.
        /// </summary>
        private void DrawMapDump()
        {
            GUILayout.Label("<b>Simulator map</b> — dump the rail network to map.json / "
                + "state.json in the mod folder, for running the autopilot outside the game.");
            if (GUILayout.Button("Dump map + state", GUILayout.Width(200)))
                dumpReport = MapDump.Write(null);
            if (dumpReport.Length > 0)
                GUILayout.Label(dumpReport);
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
            GUILayout.Label("<b>Driving profiles</b> — how the autopilot handles each locomotive type.");
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
