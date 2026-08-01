using DV.PointSet;
using DV.Signs;
using HarmonyLib;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// Sign-based speed/grade detection, ported from dv-hud. The game's speed and
    /// grade signs carry a <see cref="SignDebug"/> whose text is the posted value;
    /// as each scene chunk streams in we drop an invisible collider on that sign
    /// (a dedicated layer), then find a track's signs by casting a short ray
    /// forward along the rail at each point and reading the collider it hits.
    ///
    /// This matches what the player sees on the ground, unlike geometry-derived
    /// speed zones whose boundaries are pulled back from the visible signs. The
    /// catch: colliders exist only for chunks streamed while this patch is active,
    /// so a fresh UMM reload won't re-place them for already-loaded terrain (drive
    /// into new terrain or restart to populate). Cached per track.
    /// </summary>
    public static class TrackIndexer
    {
        internal const int SIGN_COLLIDER_LAYER = 30;
        private const float SIMPLIFIED_RESOLUTION = 10f;

        private static readonly Dictionary<RailTrack, List<TrackEvent>> indexedTracks =
            new Dictionary<RailTrack, List<TrackEvent>>();

        public static IEnumerable<TrackEvent> GetTrackEvents(RailTrack track)
        {
            if (!indexedTracks.TryGetValue(track, out var data))
                data = indexedTracks[track] = GenerateTrackEvents(track).ToList();
            return data;
        }

        /// <summary>This track's events re-based onto <paramref name="start"/>,
        /// oriented by travel direction. Where no speed sign has been indexed for
        /// the track at all — its chunk streamed in before the mod, or the sign
        /// raycast missed — the geometry-derived zones stand in, so a 30 km/h
        /// curve doesn't simply vanish from the HUD.</summary>
        public static IEnumerable<TrackEvent> GetTrackEvents(RailTrack track, bool first, double start)
        {
            var events = GetTrackEvents(track);
            if (!events.Any(ev => ev is SpeedLimitEvent))
                events = events.Concat(GeometryEvents(track)).OrderBy(ev => ev.span);
            return events.RelativeFromSpan(start, first);
        }

        /// <summary>Speed events synthesised from the geometry zones: one facing
        /// each way at each constant-speed segment's entry, values rounded the
        /// way the game rounds its signs.</summary>
        private static IEnumerable<TrackEvent> GeometryEvents(RailTrack track)
        {
            double cursor = 0;
            foreach (var seg in SpeedZones.SegmentsFor(track))
            {
                if (seg.Length < 3)
                    continue;
                int limit = Mathf.RoundToInt(seg[0]);
                double length = seg[2];
                if (limit > 0)
                {
                    // Entering forward reads the limit at the segment's start;
                    // entering backward reads it at its far end.
                    yield return new SpeedLimitEvent(cursor, true, limit);
                    yield return new SpeedLimitEvent(cursor + length, false, limit);
                }
                cursor += length;
            }
        }

        /// <summary>Every indexed speed sign on a track: where it stands, which
        /// travel direction it faces (true = toward increasing span), and its
        /// limit. For callers that need the ZONES — a sign binds from where it
        /// stands to the next sign its way, not the whole track.</summary>
        public static IEnumerable<(double span, bool direction, int limit)> SpeedSigns(RailTrack track)
        {
            foreach (var ev in GetTrackEvents(track))
                if (ev is SpeedLimitEvent s && s.limit > 0)
                    yield return (ev.span, s.Direction, s.limit);
        }

        /// <summary>The tightest posted speed limit on a track, km/h — 0 if the
        /// track has no speed sign indexed yet. With <paramref name="direction"/>
        /// given, only signs FACING that travel direction count (true = toward
        /// increasing span): a 40 posted for trains coming the other way must not
        /// pin this one to 40 on its own 100 km/h road.</summary>
        public static float MinSpeedLimit(RailTrack track, bool? direction = null)
        {
            float min = float.MaxValue;
            foreach (var ev in GetTrackEvents(track))
            {
                if (!(ev is SpeedLimitEvent s) || s.limit <= 0)
                    continue;
                if (direction != null && s.Direction != direction.Value)
                    continue;
                min = Mathf.Min(min, s.limit);
            }
            return min == float.MaxValue ? 0f : min;
        }

        /// <summary>Grade at a point as a signed percent, quantised to 0.5.</summary>
        public static float Grade(EquiPointSet.Point point) => Mathf.RoundToInt(point.forward.y * 200) / 2f;

        private static IEnumerable<TrackEvent> ParseSign(string colliderName, bool direction, double span)
        {
            string[] parts = colliderName.Split('\n');
            switch (parts.Length)
            {
                case 1:
                    if (int.TryParse(parts[0], out var limit))
                        yield return new SpeedLimitEvent(span, direction, limit * 10);
                    break;

                case 2:
                    if (int.TryParse(parts[0], out var top))
                    {
                        if (parts[1][0] == '+' || parts[1][0] == '-')
                        {
                            yield return new SpeedLimitEvent(span, direction, top * 10);
                            yield return new GradeEvent(span, direction, float.Parse(parts[1]));
                        }
                        else if (int.TryParse(parts[1], out var bottom))
                        {
                            yield return new DualSpeedLimitEvent(span, direction, top * 10, bottom * 10);
                        }
                    }
                    else if (int.TryParse(parts[1], out var bottom))
                    {
                        yield return new SpeedLimitEvent(span, direction, bottom * 10);
                    }
                    else
                    {
                        Main.DebugLog(() => $"Unable to parse sign: \"{colliderName.Replace("\n", "\\n")}\"");
                    }
                    break;
            }
        }

        private static IEnumerable<TrackEvent> FindSigns(EquiPointSet.Point point)
        {
            // Sign colliders sit on the rail centreline; cast forward along the
            // rail over one point-to-point span and read whatever we pass through.
            var hits = Physics.RaycastAll(
                new Ray((Vector3)point.position + WorldMover.currentMove, point.forward),
                (float)point.spanToNextPoint,
                1 << SIGN_COLLIDER_LAYER);

            foreach (var hit in hits)
            {
                var dp = Vector3.Dot(hit.collider.transform.forward, point.forward);
                bool direction = dp < 0f;
                foreach (var trackEvent in ParseSign(hit.collider.name, direction, point.span + hit.distance))
                    yield return trackEvent;
            }
        }

        private static IEnumerable<TrackEvent> GenerateTrackEvents(RailTrack track)
        {
            var pointSet = track.GetKinkedPointSet();
            EquiPointSet simplified = EquiPointSet.ResampleEquidistant(
                pointSet,
                Mathf.Min(SIMPLIFIED_RESOLUTION, (float)pointSet.span / 3));

            foreach (var point in simplified.points)
                foreach (var trackEvent in FindSigns(point))
                    yield return trackEvent;
        }

        [HarmonyPatch(typeof(Streamer), nameof(Streamer.AddSceneGO))]
        public static class AddSceneGOPatch
        {
            public static void Postfix(GameObject sceneGO)
            {
                var signDebugs = sceneGO.GetComponentsInChildren<SignDebug>();
                bool foundSigns = false;
                foreach (var signDebug in signDebugs)
                {
                    signDebug.gameObject.layer = SIGN_COLLIDER_LAYER;
                    var collider = signDebug.gameObject.AddComponent<CapsuleCollider>();
                    collider.name = signDebug.text;
                    collider.center = new Vector3(2f, 0f, 0f);
                    collider.height = 10f;
                    collider.direction = 1; // along Y-axis
                    collider.isTrigger = true;
                    foundSigns = true;
                }
                // New signs streamed in: drop the cache so tracks near them reindex.
                if (foundSigns)
                    indexedTracks.Clear();
            }
        }
    }
}
