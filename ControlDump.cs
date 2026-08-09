using DV.RemoteControls;
using DV.Simulation.Cars;
using DV.Utils;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// What handles a locomotive actually HAS, written out so driving it is a
    /// matter of reading rather than guessing. The drive loop only ever moves
    /// Throttle, Brake, IndependentBrake, DynamicBrake, Reverser and Sander —
    /// every machine that needs something else (a gearbox on a mechanical, a
    /// cutoff on a steamer) is one the autopilot cannot work, and there is no way
    /// to tell WHICH from inside the game.
    ///
    /// Everything here goes through reflection on purpose. The point is to
    /// discover what exists, and code that names the members up front can only
    /// ever find what it already knew about.
    /// </summary>
    internal static class ControlDump
    {
        private const BindingFlags Members =
            BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;

        /// <summary>
        /// Dump one car, or every loco streamed in when null. Writes
        /// controls.txt beside the mod and returns a one-line report.
        /// </summary>
        internal static string Write(TrainCar? only, string? dir = null)
        {
            var cars = new List<TrainCar>();
            if (only != null)
            {
                cars.Add(only);
            }
            else
            {
                var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
                if (registry == null)
                    return "no car registry — is the world loaded?";
                // One per LIVERY, not per car: twenty DE6s describe themselves
                // identically and bury the one machine that is different.
                var seen = new HashSet<string>();
                foreach (var c in registry.Values)
                    if (c != null && c.IsLoco && seen.Add(c.carLivery?.id ?? c.ID))
                        cars.Add(c);
            }
            if (cars.Count == 0)
                return "nothing to dump";

            var text = new StringBuilder();
            foreach (var car in cars)
                DumpCar(car, text);

            // UMM loads the assembly from BYTES, so Location is empty and asking
            // it where it lives throws "Invalid path". The manager knows the
            // folder; ask it, exactly as the map dump does.
            string folder = string.IsNullOrEmpty(dir) ? (Main.mod?.Path ?? ".") : dir!;
            string path = Path.Combine(folder, "controls.txt");
            try
            {
                Directory.CreateDirectory(folder);
                File.WriteAllText(path, text.ToString());
            }
            catch (Exception e)
            {
                Main.mod?.Logger.Log(text.ToString());
                return $"could not write {path} ({e.Message}) — dumped to the log instead";
            }
            Main.mod?.Logger.Log(text.ToString());
            return $"wrote {cars.Count} loco(s) to {path} (and the log)";
        }

        private static void DumpCar(TrainCar car, StringBuilder text)
        {
            text.AppendLine($"=== {car.ID}  livery={car.carLivery?.id ?? "?"}"
                + $"  type={car.carType}  prefab={car.name}");

            var rcm = car.GetComponent<RemoteControllerModule>();
            text.AppendLine($"  RemoteControllerModule: {(rcm == null ? "ABSENT — the autopilot cannot take this loco" : "present")}");

            var sim = car.SimController;
            var overrider = rcm?.controlsOverrider ?? sim?.controlsOverrider;
            if (overrider == null)
                text.AppendLine("  ControlsOverrider: ABSENT");
            else
                DumpOverrider(overrider, text);

            if (sim?.simFlow == null)
                text.AppendLine("  simFlow: ABSENT");
            else
                DumpPorts(sim.simFlow, text);

            text.AppendLine();
        }

        /// <summary>
        /// Every handle the overrider offers, and whether this particular machine
        /// has one. A null property is the loco saying "I do not have that
        /// control" — which is exactly what has to be known before driving it.
        /// </summary>
        private static void DumpOverrider(object overrider, StringBuilder text)
        {
            text.AppendLine("  ControlsOverrider:");
            foreach (var prop in overrider.GetType().GetProperties(Members)
                         .OrderBy(p => p.Name))
            {
                if (prop.GetIndexParameters().Length > 0)
                    continue;
                object? handle;
                try
                {
                    handle = prop.GetValue(overrider);
                }
                catch (Exception e)
                {
                    text.AppendLine($"    {prop.Name,-22} threw: {e.GetBaseException().Message}");
                    continue;
                }
                if (handle == null)
                {
                    text.AppendLine($"    {prop.Name,-22} null");
                    continue;
                }
                // The handle objects expose the live position as Value. Read it
                // if it is there; the type name alone is worth having if not.
                string value = "";
                var valueProp = handle.GetType().GetProperty("Value", Members);
                if (valueProp != null)
                {
                    try { value = $" = {valueProp.GetValue(handle)}"; }
                    catch (Exception e) { value = $" (read threw: {e.GetBaseException().Message})"; }
                }
                text.AppendLine($"    {prop.Name,-22} {handle.GetType().Name}{value}");
            }
        }

        /// <summary>
        /// Every simulation port on the car, by id. This is where a gearbox, a
        /// torque converter or a transmission temperature lives — none of them
        /// are on the overrider, and none of their ids appear as strings in the
        /// game's assemblies, so listing them from a live car is the only way to
        /// learn the names.
        /// </summary>
        private static void DumpPorts(object simFlow, StringBuilder text)
        {
            var listed = new SortedDictionary<string, string>(StringComparer.Ordinal);
            foreach (var field in simFlow.GetType().GetFields(Members))
            {
                object? value;
                try { value = field.GetValue(simFlow); }
                catch { continue; }
                if (!(value is IDictionary map))
                    continue;
                foreach (DictionaryEntry entry in map)
                {
                    string key = entry.Key?.ToString() ?? "?";
                    if (listed.ContainsKey(key))
                        continue;
                    string shown = entry.Value?.GetType().Name ?? "null";
                    var valueProp = entry.Value?.GetType().GetProperty("Value", Members);
                    if (valueProp != null)
                    {
                        try { shown += $" = {valueProp.GetValue(entry.Value)}"; }
                        catch { }
                    }
                    listed[key] = shown;
                }
            }
            if (listed.Count == 0)
            {
                text.AppendLine("  ports: none found (simFlow keeps them somewhere else)");
                return;
            }
            text.AppendLine($"  ports ({listed.Count}):");
            foreach (var kv in listed)
                text.AppendLine($"    {kv.Key,-44} {kv.Value}");
        }
    }
}
