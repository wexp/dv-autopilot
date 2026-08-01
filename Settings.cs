using System.Collections.Generic;
using UnityEngine;
using UnityModManagerNet;

namespace DvMod.Autopilot
{
    public class Settings : UnityModManager.ModSettings
    {
        public bool enableLogging = false;
        public LocoProfiles autopilot = new LocoProfiles();

        public readonly string? version = Main.mod?.Info.Version;

        public void Draw()
        {
            GUILayout.BeginVertical(GUILayout.ExpandWidth(false));
            autopilot.Draw();
            enableLogging = GUILayout.Toggle(enableLogging, "Enable logging");
            GUILayout.EndVertical();
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

        private static float Field(string label, float value)
        {
            GUILayout.Label(label, GUILayout.Width(90));
            var text = GUILayout.TextField(value.ToString("0.###"), GUILayout.Width(50));
            return float.TryParse(text, out var parsed) ? parsed : value;
        }

        public void Draw()
        {
            EnsureDefaults();
            GUILayout.Label("Autopilot, per locomotive type:");
            GUILayout.BeginVertical("box", GUILayout.ExpandWidth(false));
            foreach (var p in profiles)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label(p.livery.Length == 0 ? "(default)" : p.livery, GUILayout.Width(90));
                p.throttleUp = Field("open rate", p.throttleUp);
                p.minMoveThrottle = Field("min to move", p.minMoveThrottle);
                p.maneuverKmh = Field("yard km/h", p.maneuverKmh);
                GUILayout.EndHorizontal();
            }
            GUILayout.EndVertical();
        }
    }
}
