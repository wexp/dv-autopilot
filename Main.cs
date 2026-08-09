using DV.Logic.Job;
using HarmonyLib;
using System;
using System.Linq;
using System.Reflection;
using UnityModManagerNet;

namespace DvMod.Autopilot
{
    [EnableReloading]
    public static class Main
    {
        public static UnityModManager.ModEntry? mod;
        public static Settings settings = new Settings();
        public static bool enabled;
        private static Action<Job>? attachedJobChangedHandler;

        public static bool Load(UnityModManager.ModEntry modEntry)
        {
            mod = modEntry;
            Updater.CaptureMainThread();

            try
            {
                var loaded = Settings.Load<Settings>(modEntry);
                if (loaded.version == modEntry.Info.Version)
                    settings = loaded;
            }
            catch
            {
            }

            // Saved profiles from an older build would otherwise mask improved
            // defaults for ever, since the file is written back on every unload.
            settings.autopilot.MigrateDefaults();

            mod.OnGUI = OnGUI;
            mod.OnSaveGUI = OnSaveGUI;
            mod.OnToggle = OnToggle;
            mod.OnUnload = OnUnload;

            return true;
        }

        private static bool OnUnload(UnityModManager.ModEntry modEntry)
        {
            return OnToggle(modEntry, false);
        }

        private static void OnGUI(UnityModManager.ModEntry modEntry)
        {
            settings.Draw();
        }

        private static void OnSaveGUI(UnityModManager.ModEntry modEntry)
        {
            settings.Save(modEntry);
        }

        private static bool OnToggle(UnityModManager.ModEntry modEntry, bool value)
        {
            Harmony harmony = new Harmony(modEntry.Info.Id);

            if (value)
            {
                PatchAllSurvivingFailures(harmony, modEntry);
                WorldStreamingInit.LoadingFinished += Start;
                UnloadWatcher.UnloadRequested += Stop;
                ConnectToPersistentJobs();
                if (WorldStreamingInit.Instance && WorldStreamingInit.IsLoaded)
                {
                    Start();
                }
            }
            else
            {
                Stop();
                UnloadWatcher.UnloadRequested -= Stop;
                WorldStreamingInit.LoadingFinished -= Start;
                DisconnectFromPersistentJobs();
                // NOT harmony.UnpatchAll: that walks EVERY patched method in the
                // process and resolves every patch entry on each — other mods'
                // included. One entry Harmony cannot resolve (a mod built for a
                // different game version, a hot-swapped assembly) throws and
                // cancelled the whole reload. Touch only methods that carry one
                // of OUR patches, and survive the ones that still blow up.
                string id = modEntry.Info.Id;
                foreach (var original in Harmony.GetAllPatchedMethods().ToList())
                {
                    try
                    {
                        var info = Harmony.GetPatchInfo(original);
                        if (info == null)
                            continue;
                        bool ours = info.Prefixes.Any(p => p.owner == id)
                            || info.Postfixes.Any(p => p.owner == id)
                            || info.Transpilers.Any(p => p.owner == id)
                            || info.Finalizers.Any(p => p.owner == id);
                        if (ours)
                            harmony.Unpatch(original, HarmonyPatchType.All, id);
                    }
                    catch (Exception e)
                    {
                        mod?.Logger.Log(
                            $"Could not unpatch {original?.DeclaringType?.Name}.{original?.Name}: "
                            + $"{e.GetType().Name} — skipped");
                    }
                }
            }
            return true;
        }

        /// <summary>
        /// Apply our patches one class at a time, surviving the ones that blow up.
        ///
        /// NOT harmony.PatchAll: patching a method makes Harmony rebuild that
        /// method's whole wrapper, re-emitting every OTHER mod's patches on it too.
        /// One of those being unpatchable — built for a different game version,
        /// naming an argument the method no longer has — threw out of PatchAll and
        /// took the entire load with it, leaving the mod half-patched and Start()
        /// never called. The failure has nothing to do with the patch we were
        /// applying and everything to do with who else is on that method, so it
        /// must not be fatal to us. Same reasoning as the unpatch loop below.
        /// </summary>
        private static void PatchAllSurvivingFailures(Harmony harmony, UnityModManager.ModEntry modEntry)
        {
            foreach (var type in Assembly.GetExecutingAssembly().GetTypes())
            {
                try
                {
                    new PatchClassProcessor(harmony, type).Patch();
                }
                catch (Exception e)
                {
                    // Name what was lost: a silently skipped patch is a feature
                    // that quietly stops working.
                    modEntry.Logger.Log(
                        $"Could not patch {type.FullName}: {e.GetType().Name} — "
                        + $"{e.InnerException?.Message ?? e.Message}; that feature is off this session");
                }
            }
        }

        private static void DisconnectFromPersistentJobs()
        {
            EventInfo? jobTracksChanged = GetPersistentJobsTrackChangedEvent();
            if (jobTracksChanged != null && attachedJobChangedHandler != null)
            {
                jobTracksChanged.RemoveEventHandler(null, attachedJobChangedHandler);
                attachedJobChangedHandler = null;
                DebugLog(() => "Persistent Jobs event handler removed");
            }
        }

        private static void ConnectToPersistentJobs()
        {
            EventInfo? jobTracksChanged = GetPersistentJobsTrackChangedEvent();
            if (jobTracksChanged != null)
            {
                attachedJobChangedHandler = new Action<Job>(JobData.JobPatches.UpdateJobsFromPersistentJobs);
                jobTracksChanged.AddEventHandler(null, attachedJobChangedHandler);
                DebugLog(() => "Persistent Jobs found and hooked");
            }
        }

        /// <summary>Gets event exposed by Persistent Jobs for notifications when jobs are modified.</summary>
        private static EventInfo? GetPersistentJobsTrackChangedEvent()
        {
            return UnityModManager.FindMod("PersistentJobsMod")
                ?.Assembly
                ?.GetType("PersistentJobsModInteractionFeatures")
                ?.GetEvent("JobTracksChanged");
        }

        private static void Start()
        {
            Updater.Create();
            Tablet.Attach();
            TryAttachCommsRadio();
        }

        /// <summary>The comms radio mode needs two other mods (CommsRadioAPI for
        /// the mode itself, MessageBox for its popups). Without them nothing here
        /// may even TOUCH the CommsRadio class — resolving its method bodies would
        /// throw for the missing assemblies — so the gate lives out here.</summary>
        private static void TryAttachCommsRadio()
        {
            if (UnityModManager.FindMod("CommsRadioAPI")?.Active != true
                || UnityModManager.FindMod("MessageBox")?.Active != true)
            {
                DebugLog(() => "CommsRadioAPI/MessageBox not installed — no radio mode");
                return;
            }
            try
            {
                CommsRadio.Attach();
            }
            catch (System.Exception e)
            {
                mod?.Logger.Log($"Comms radio mode not attached: {e.Message}");
            }
        }

        private static void Stop()
        {
            // Brake anything under autopilot BEFORE the updater goes away. Destroying
            // it kills the drive coroutines where they stand, which would leave a loco
            // rolling with its throttle open.
            Autopilot.StopAllHere();
            // Belt and braces: stopping the runs strips their markings, but a set
            // left behind by a run that died some other way would outlive the mod
            // and could not then be removed by anything.
            Markings.UnfitAll();
            Tablet.Detach();
            Updater.Destroy();
        }

        /// <summary>Hot-reload this mod — what UMM's Ctrl+F10 dialog does, on a
        /// hotkey. The runner lives on a detached object because the reload
        /// unloads this assembly's own scene objects mid-flight.</summary>
        internal static void RequestReload()
        {
            var entry = mod;
            if (entry == null || !entry.CanReload)
            {
                mod?.Logger.Log("Reload not available for this build");
                return;
            }
            mod?.Logger.Log("Reloading by hotkey...");
            ReloadRunner.Begin(entry);
        }

        private sealed class ReloadRunner : UnityEngine.MonoBehaviour
        {
            private UnityModManager.ModEntry? entry;

            public static void Begin(UnityModManager.ModEntry modEntry)
            {
                var host = new UnityEngine.GameObject("AutopilotReload");
                UnityEngine.Object.DontDestroyOnLoad(host);
                host.AddComponent<ReloadRunner>().entry = modEntry;
            }

            private System.Collections.IEnumerator Start()
            {
                yield return null;
                var reload = entry?.GetType().GetMethod(
                    "Reload",
                    BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public);
                reload?.Invoke(entry, null);
                Destroy(gameObject);
            }
        }

        public static void DebugLog(TrainCar car, Func<string> message)
        {
            if (car == PlayerManager.Car)
                DebugLog(message);
        }

        public static void DebugLog(Func<string> message)
        {
            if (settings.enableLogging)
                mod?.Logger.Log(message());
        }
    }
}
