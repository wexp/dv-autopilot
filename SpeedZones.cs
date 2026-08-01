using Newtonsoft.Json;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// Speed limits, derived from track geometry the same way the game places its
    /// speed-limit signs. DV.Signs.SignPlacer.GetSegmentInfos is a static method
    /// that splits a track's bezier curve into constant-speed segments based on
    /// curve radius, so this needs neither the streamed world objects nor the sign
    /// prefab config.
    ///
    /// Everything is done through reflection: SignPlacer.CurveSegmentInfo's
    /// visibility isn't part of the public contract, and reflection degrades to an
    /// empty result instead of a hard failure if a member moves in a game update.
    /// </summary>
    public static class SpeedZones
    {
        private static string? json;

        // The scene's SignPlacer carries tuned values we can't read from the
        // assembly. We prefer a live instance's fields (see ResolveParams) and
        // fall back to these — speeds stay exact, only the segment boundaries
        // shift slightly.
        private const float DefaultError = 1f;
        private const float DefaultMinArcLength = 25f;

        public static string GetSpeedZonesJson()
        {
            if (json != null)
                return json;
            if (!WorldStreamingInit.Instance || !WorldStreamingInit.IsLoaded)
                throw new Exception("World not yet loaded");

            var zones = Updater.RunOnMainThread(Compute).Result;
            var serialized = JsonConvert.SerializeObject(zones);
            // Only cache a real result. An empty dict means the world wasn't ready
            // or the sign reflection failed; caching that would make it stick for
            // the whole session, so leave it recomputable on the next request.
            if (zones.Count > 0)
                json = serialized;
            return serialized;
        }

        private static bool segReady;
        private static MethodInfo? segGetInfos;
        private static PropertyInfo? segCurve;
        private static FieldInfo? segLen;
        private static MethodInfo? segSpeed;
        private static MethodInfo? segGradeAngle;
        private static MethodInfo? segGrade;
        private static float segError = 1f;
        private static float segMinArc = 25f;

        private static bool EnsureSegReflection()
        {
            if (segReady)
                return segGetInfos != null && segCurve != null;
            segReady = true;
            var type = FindLoadedType("DV.Signs.SignPlacer");
            segGetInfos = type?.GetMethod(
                "GetSegmentInfos",
                BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static);
            segCurve = typeof(RailTrack).GetProperty(
                "curve",
                BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            if (type != null)
                (segError, segMinArc) = ResolveParams(type);
            return segGetInfos != null && segCurve != null;
        }

        /// <summary>
        /// Ordered constant-speed segments for one track in curve order, each
        /// [speedKmh, signedGrade, lengthMetres]. Shared with the track-ahead
        /// events walk (Hud). Empty if SignPlacer can't be reached.
        /// </summary>
        // Segment geometry never changes for a given track, but computing it means
        // reflecting into SignPlacer and re-splitting the bezier — far too expensive to
        // repeat, and the autopilot asks for it several times a tick per running loco.
        private static readonly Dictionary<RailTrack, List<float[]>> segmentCache =
            new Dictionary<RailTrack, List<float[]>>();

        public static List<float[]> SegmentsFor(RailTrack track)
        {
            if (track != null && segmentCache.TryGetValue(track, out var cached))
                return cached;
            var list = ComputeSegmentsFor(track);
            if (track != null)
                segmentCache[track] = list;
            return list;
        }

        private static List<float[]> ComputeSegmentsFor(RailTrack track)
        {
            var list = new List<float[]>();
            if (track == null || !EnsureSegReflection())
                return list;
            var curve = segCurve!.GetValue(track);
            if (curve == null)
                return list;
            if (!(segGetInfos!.Invoke(null, new object[] { curve, segError, segMinArc, false }) is IEnumerable infos))
                return list;

            foreach (var info in infos)
            {
                if (segLen == null || segSpeed == null || segGradeAngle == null || segGrade == null)
                {
                    const BindingFlags f =
                        BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
                    var t = info.GetType();
                    segLen = t.GetField("segmentLength", f);
                    segSpeed = t.GetMethod("GetSpeed", f);
                    segGradeAngle = t.GetMethod("GetGradeAngle", f);
                    segGrade = t.GetMethod("GetGrade", f);
                    if (segLen == null || segSpeed == null || segGradeAngle == null || segGrade == null)
                        return list;
                }
                var length = (float)segLen.GetValue(info);
                var speed = (float)segSpeed.Invoke(info, null);
                var mag = (float)segGradeAngle.Invoke(info, null);
                var dir = Convert.ToInt32(segGrade.Invoke(info, null));
                var grade = dir == 1 ? mag : dir == 2 ? -mag : 0f;
                list.Add(new[] { speed, grade, length });
            }
            return list;
        }

        private static Dictionary<string, List<object[]>> Compute()
        {
            var result = new Dictionary<string, List<object[]>>();

            var signPlacerType = FindLoadedType("DV.Signs.SignPlacer");
            if (signPlacerType == null)
            {
                Main.DebugLog(() => "SpeedZones: DV.Signs.SignPlacer type not loaded");
                return result;
            }
            // GetSegmentInfos is a non-public static helper, so NonPublic is
            // required — Public|Static alone silently returned null.
            var getSegmentInfos = signPlacerType.GetMethod(
                "GetSegmentInfos",
                BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static);
            if (getSegmentInfos == null)
            {
                Main.DebugLog(() => "SpeedZones: SignPlacer.GetSegmentInfos not found");
                return result;
            }

            var (error, minArcLength) = ResolveParams(signPlacerType!);

            // RailTrack.curve returns a BezierCurve from an assembly this project
            // doesn't reference; touching it directly would need that reference
            // just to box the value, so it's read reflectively too.
            var curveProp = typeof(RailTrack).GetProperty(
                "curve",
                BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            if (curveProp == null)
            {
                Main.DebugLog(() => "SpeedZones: RailTrack.curve not found");
                return result;
            }

            FieldInfo? lengthField = null;
            MethodInfo? getSpeed = null;
            MethodInfo? getGradeAngle = null;
            MethodInfo? getGrade = null;
            int trackCount = 0, skipped = 0;

            foreach (var track in Component.FindObjectsOfType<RailTrack>())
            {
                if (track == null)
                    continue;
                trackCount++;
                var curve = curveProp.GetValue(track);
                if (curve == null)
                    continue;

                string id;
                try
                {
                    id = track.LogicTrack().ID.ToString();
                }
                catch
                {
                    continue;
                }

                try
                {
                    var infos = getSegmentInfos.Invoke(
                        null,
                        new object[] { curve, error, minArcLength, false }) as IEnumerable;
                    if (infos == null)
                        continue;

                    var zones = new List<object[]>();
                    foreach (var info in infos)
                    {
                        // Resolve the reflected members once, from the first item.
                        if (lengthField == null || getSpeed == null || getGradeAngle == null || getGrade == null)
                        {
                            const BindingFlags f =
                                BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
                            var t = info.GetType();
                            lengthField = t.GetField("segmentLength", f);
                            getSpeed = t.GetMethod("GetSpeed", f);
                            // GetGradeAngle is the magnitude; GetGrade is the
                            // direction (Grade enum: Flat=0, Incline=1, Decline=2).
                            getGradeAngle = t.GetMethod("GetGradeAngle", f);
                            getGrade = t.GetMethod("GetGrade", f);
                            if (lengthField == null || getSpeed == null || getGradeAngle == null || getGrade == null)
                            {
                                Main.DebugLog(() => "SpeedZones: CurveSegmentInfo members not found");
                                return result;
                            }
                        }

                        var length = (float)lengthField.GetValue(info);
                        var speed = (float)getSpeed.Invoke(info, null);
                        var magnitude = (float)getGradeAngle.Invoke(info, null);
                        var direction = Convert.ToInt32(getGrade.Invoke(info, null));
                        // Sign the grade: uphill +, downhill -, flat 0.
                        var grade = direction == 1 ? magnitude : direction == 2 ? -magnitude : 0f;
                        // [lengthMetres, speedKmh, signedGrade] (grade * 100 ~= percent)
                        zones.Add(new object[]
                        {
                            (int)Math.Round(length),
                            (int)Math.Round(speed),
                            (double)Math.Round(grade, 4),
                        });
                    }

                    if (zones.Count > 0)
                        result[id] = zones;
                }
                catch (Exception e)
                {
                    skipped++;
                    Main.DebugLog(() => $"SpeedZones: skipping {id}: {e.Message}");
                }
            }

            Main.DebugLog(() =>
                $"SpeedZones: {trackCount} tracks, {result.Count} with zones, {skipped} errored " +
                $"(error={error}, minArcLength={minArcLength})");
            return result;
        }

        /// <summary>
        /// Resolves a type by full name across every loaded assembly.
        /// Type.GetType with an assembly-qualified name asks the loader to load
        /// the assembly by simple name, which Mono routinely fails to do for an
        /// already-loaded game assembly (this is why the first attempt found
        /// nothing). Scanning the AppDomain finds it as long as it is loaded —
        /// and DV.SignPlacer is, since the world's sign objects use its
        /// SignGenerator component.
        /// </summary>
        private static Type? FindLoadedType(string fullName)
        {
            var direct = Type.GetType(fullName);
            if (direct != null)
                return direct;
            foreach (var assembly in AppDomain.CurrentDomain.GetAssemblies())
            {
                Type? found;
                try
                {
                    found = assembly.GetType(fullName, throwOnError: false);
                }
                catch
                {
                    continue;
                }
                if (found != null)
                    return found;
            }
            return null;
        }

        private static (float error, float minArcLength) ResolveParams(Type signPlacerType)
        {
            // A configured SignPlacer in the scene has the exact tuning; use it
            // when present so the boundaries match the in-game signs.
            try
            {
                var instance = Component.FindObjectOfType(signPlacerType);
                if (instance != null)
                {
                    const BindingFlags flags =
                        BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
                    var error = signPlacerType.GetField("error", flags)?.GetValue(instance);
                    var minArc = signPlacerType.GetField("minArcLength", flags)?.GetValue(instance);
                    if (error is float e && minArc is float m && e > 0f && m > 0f)
                        return (e, m);
                }
            }
            catch (Exception ex)
            {
                Main.DebugLog(() => $"SpeedZones: falling back to default params: {ex.Message}");
            }
            return (DefaultError, DefaultMinArcLength);
        }
    }
}
