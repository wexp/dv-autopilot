using DV.Simulation.Cars;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// Turning the engines of the driven units over at dispatch and shutting them
    /// down when the run has finished, so a loco is not left idling at a siding
    /// for the rest of the session and a cold one still answers a dispatch.
    ///
    /// Deliberately NOT the game's own <c>StartupHelper.Startup</c>, which is a
    /// sandbox convenience rather than a start: besides the fuses and the starter
    /// it pins the independent brake, winds the handbrakes off, cuts the train
    /// brake in and fills the main reservoir to 9 bar — a dispatch is not licence
    /// to hand the player free air. This does the two things a driver does: the
    /// fuses, then the starter held until it catches.
    /// </summary>
    public static class Engine
    {
        /// <summary>How long the starter is held before giving up, s. The same
        /// five seconds the game's own starter coroutine allows.</summary>
        private const float StarterHold = 5f;
        private const float CrankStep = 0.25f;
        /// <summary>How long a shutdown waits for an engine that is still being
        /// cranked to actually catch, s. The cutoff does nothing to a machine that
        /// is not running yet, and a run can finish the same second it was
        /// dispatched — L-030 arrived on A2P a heartbeat after its own start order
        /// and was left idling because the reader still said off.</summary>
        private const float CatchWait = 8f;
        /// <summary>How many times the cutoff is pulsed before giving up.</summary>
        private const int StopAttempts = 3;

        /// <summary>The crank in flight per car, so a shutdown can call it off
        /// instead of racing it.</summary>
        private static readonly Dictionary<TrainCar, Coroutine> cranking =
            new Dictionary<TrainCar, Coroutine>();

        /// <summary>Start every unit this loco actually drives — the MU group, not
        /// the whole trainset: a loco merely coupled in is freight, and nothing
        /// asked it to run.</summary>
        public static void StartGroup(TrainCar? lead)
        {
            if (lead == null)
                return;
            foreach (var car in Autopilot.MuGroup(lead))
                Start(car);
        }

        public static void StopGroup(TrainCar? lead)
        {
            if (lead == null)
                return;
            foreach (var car in Autopilot.MuGroup(lead))
                Stop(car);
        }

        /// <summary>Fuses in, then crank. A machine with no starter — steam, the
        /// battery shunters, the handcar — gets its fuses and nothing else: there
        /// is no engine there to light.</summary>
        public static void Start(TrainCar? car)
        {
            var sim = car?.SimController;
            var overrider = sim?.controlsOverrider;
            if (sim == null || overrider == null)
                return;
            // A blown or pulled fuse reads exactly like a dead engine from the
            // outside, and the traction-motor fuse is what actually feeds the
            // rail — starting without them in leaves a running loco that cannot
            // move. Same sweep the game's own startup makes.
            var fuses = sim.simFlow?.AllFuses;
            if (fuses != null)
            {
                for (int i = 0; i < fuses.Count; i++)
                    fuses[i].ChangeState(true);
            }
            if (overrider.Starter == null)
                return;
            if (overrider.EngineOnReader != null && overrider.EngineOnReader.IsOn)
                return;
            if (cranking.ContainsKey(car!))
                return;
            Main.DebugLog(() => $"[ENGINE] {car!.ID} starting");
            var routine = Updater.Run(Crank(car!, overrider));
            if (routine != null)
                cranking[car!] = routine;
        }

        /// <summary>Hold the starter until the engine catches, then let go — the
        /// starter is a spring-loaded button, and left pressed it keeps cranking
        /// an engine that is already running.</summary>
        private static IEnumerator Crank(TrainCar car, BaseControlsOverrider overrider)
        {
            overrider.Starter.Set(1f);
            for (float left = StarterHold; left > 0f; left -= CrankStep)
            {
                // Real time: a paused game must not leave the starter engaged for
                // as long as the player sits in the menu.
                yield return new WaitForSecondsRealtime(CrankStep);
                if (overrider == null)
                    break;
                if (overrider.EngineOnReader != null && overrider.EngineOnReader.IsOn)
                    break;
            }
            overrider?.Starter?.Set(0f);
            cranking.Remove(car);
        }

        /// <summary>Shut one unit down, but only one we could start again: the
        /// fuel cutoff exists on a steam loco too, and killing a fire the autopilot
        /// has no way to relight would leave the machine stranded.</summary>
        public static void Stop(TrainCar? car)
        {
            var overrider = car?.SimController?.controlsOverrider;
            if (overrider == null || overrider.PowerOff == null || overrider.Starter == null)
                return;
            // A crank of our own still running is the one case where "the engine is
            // off" must NOT end the shutdown: it is off because it has not caught
            // yet, and it is about to. Call the crank off, drop the starter, and let
            // the shutdown below wait the engine out.
            bool ourCrank = cranking.TryGetValue(car!, out var routine);
            if (ourCrank)
            {
                Updater.StopRun(routine);
                cranking.Remove(car!);
                overrider.Starter.Set(0f);
            }
            if (!ourCrank
                && overrider.EngineOnReader != null && !overrider.EngineOnReader.IsOn)
                return;
            Main.DebugLog(() => $"[ENGINE] {car!.ID} shutting down");
            Updater.Run(Shutdown(overrider));
        }

        /// <summary>
        /// Wait for the engine to be running, then pulse the cutoff until it is not.
        /// One unconditional pulse is not enough: a loco caught mid-start ignores it,
        /// and PowerOffControl's own pulse can land while the engine is still
        /// spinning up.
        /// </summary>
        private static IEnumerator Shutdown(BaseControlsOverrider overrider)
        {
            var reader = overrider.EngineOnReader;
            for (float waited = 0f;
                reader != null && !reader.IsOn && waited < CatchWait;
                waited += CrankStep)
            {
                yield return new WaitForSecondsRealtime(CrankStep);
                if (overrider == null)
                    yield break;
            }
            for (int attempt = 0; attempt < StopAttempts; attempt++)
            {
                overrider?.PowerOff?.Set(1f);
                yield return new WaitForSecondsRealtime(1f);
                if (overrider == null || reader == null || !reader.IsOn)
                    yield break;
            }
        }
    }
}
