using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// A movement and interlocking recorder, written to its own file.
    ///
    /// Three lock-ups in one session could not be explained from Player.log: what that
    /// file holds is a handful of deduped run notes with NO TIMESTAMPS, no positions, no
    /// ledger, and no record of which rule refused a grant or when a claim changed hands.
    /// Every account of those locks was reconstruction from reading the source, which is
    /// exactly the kind of answer that is worth nothing.
    ///
    /// So: every metre covered, every block asked for, every decision that refused one,
    /// and every mutation of the ledger, each stamped with a sequence number and a clock.
    /// Written to Mods/Autopilot/autopilot-trace.log rather than Player.log — the volume
    /// would drown a file the whole game shares, and one file per session greps cleanly.
    ///
    /// Off by default. Every call site passes a lambda, so with the switch down the cost
    /// is one bool test and nothing is built.
    /// </summary>
    public static class Trace
    {
        private static StreamWriter? writer;
        private static int seq;
        private static float lastFlush;
        // Written from the drive coroutines, which are all on the main thread, plus the
        // once-a-second snapshot. The lock is cheap insurance against that ever changing.
        private static readonly object gate = new object();

        public static bool On => Main.settings.traceEnabled && writer != null;

        /// <summary>Start a fresh file. One session per file: an appended log from three
        /// sessions ago is worse than none, because the line you are reading may be from
        /// a build that no longer exists.</summary>
        public static void Open()
        {
            Close();
            if (!Main.settings.traceEnabled)
            {
                // Said out loud, always. A recorder that is silently off looks exactly
                // like a recorder that is on and finding nothing, and an hour of
                // driving was lost to that difference.
                Main.mod?.Logger.Log("Trace is OFF (Diagnostics -> trace to switch it on)");
                return;
            }
            string dir = Main.mod?.Path ?? "";
            if (dir.Length == 0)
                dir = ".";
            string file = Path.Combine(dir, "autopilot-trace.log");
            try
            {
                Directory.CreateDirectory(dir);
                writer = new StreamWriter(file, false, Encoding.UTF8);
                seq = 0;
                lastFlush = Time.realtimeSinceStartup;
                writer.WriteLine($"# autopilot trace, version {Main.mod?.Info.Version}, "
                    + $"build {BuildStamp(dir)}, step {Main.settings.traceStepMeters:F2} m");
                // The path in the game log, so "where is the file" is never a search,
                // and the BUILD STAMP, so a stale assembly is visible at a glance —
                // three reloads today ran a DLL older than the fix being tested.
                Main.mod?.Logger.Log($"Trace ON -> {file} (build {BuildStamp(dir)})");
            }
            catch (Exception e)
            {
                // Dispose, not just drop: the writer may already hold the file open,
                // and a leaked handle means the NEXT open fails too.
                try
                {
                    writer?.Dispose();
                }
                catch
                {
                }
                writer = null;
                Main.mod?.Logger.Log($"Trace file could not be opened: {e.Message}");
            }
        }

        /// <summary>When the DLL now running was built, as a clock time.
        ///
        /// Read off the FILE, never off the assembly: UMM loads mods from bytes, so
        /// Assembly.Location is an empty string and asking it anything throws "Path is
        /// empty" — the same trap ControlDump documents. Worth having at all because a
        /// reload that quietly kept the old assembly wasted three test runs today.</summary>
        private static string BuildStamp(string dir)
        {
            try
            {
                var dll = new FileInfo(Path.Combine(dir, "Autopilot.dll"));
                return dll.Exists ? dll.LastWriteTime.ToString("HH:mm:ss") : "unknown";
            }
            catch
            {
                return "unknown";
            }
        }

        public static void Close()
        {
            lock (gate)
            {
                try
                {
                    writer?.Flush();
                    writer?.Dispose();
                }
                catch
                {
                }
                writer = null;
            }
        }

        /// <summary>One line. The message is built only if the recorder is running.</summary>
        public static void Line(string category, Func<string> message)
        {
            if (!Main.settings.traceEnabled || writer == null)
                return;
            try
            {
                string text = message();
                lock (gate)
                {
                    if (writer == null)
                        return;
                    float now = Time.realtimeSinceStartup;
                    writer.WriteLine(
                        seq++.ToString("D7", CultureInfo.InvariantCulture) + " "
                        + now.ToString("F2", CultureInfo.InvariantCulture) + " "
                        + category.PadRight(7) + " " + text);
                    // Flushed on a timer rather than per line: the tail of the file is
                    // the part that matters when the game is killed mid-lock, and a
                    // second of it is a fair price for not writing to disk at 100 Hz.
                    if (now - lastFlush > 1f)
                    {
                        writer.Flush();
                        lastFlush = now;
                    }
                }
            }
            catch
            {
                // A recorder must never be the reason a train stops being driven.
            }
        }

        /* ------------------------------------------------------- naming -- */

        /// <summary>A track as the game names it. Unnamed rail — every junction leg in
        /// this game — comes back as "#Y-#S1284#T", which is what the plans print, so
        /// the two can be matched by eye.</summary>
        public static string Id(RailTrack? track) =>
            track == null ? "-" : track.LogicTrack()?.ID?.ToString() ?? "?";

        public static string Car(TrainCar? car) => car == null ? "-" : car.ID;

        /// <summary>Metres, to a tenth. Everything in this file is metres unless it says
        /// otherwise, so the unit is left off to keep the lines scannable.</summary>
        public static string M(double v) =>
            v >= 1e6 ? "inf" : v.ToString("F1", CultureInfo.InvariantCulture);

        public static string Ms(float v) => v.ToString("F2", CultureInfo.InvariantCulture);

        /// <summary>A list of tracks, comma separated, for leg and path dumps.</summary>
        public static string Ids(IEnumerable<RailTrack> tracks)
        {
            var sb = new StringBuilder();
            foreach (var t in tracks)
            {
                if (sb.Length > 0)
                    sb.Append(',');
                sb.Append(Id(t));
            }
            return sb.Length == 0 ? "-" : sb.ToString();
        }
    }
}
