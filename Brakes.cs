using DV.Simulation.Cars;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// The train-brake handle, for the machines where it is not a dial.
    ///
    /// On the DE2 or DH4 the lever's POSITION is the application: 0.4 of handle is
    /// 0.4 of brake, and the valve laps itself. The DM3's valve does not — the
    /// lever position is a VERB, not an amount. Release feeds the pipe, lap seals
    /// it where it is, apply vents it, emergency dumps it, and how much brake the
    /// train has is whatever the pipe pressure says by the time the lever comes
    /// back to lap. Setting that lever to "0.4" and walking away parks it in the
    /// apply band and vents the pipe to nothing.
    ///
    /// The bands are the game's own (DV.Simulation.Brake.BrakeSystem,
    /// SimulateTrainBrake): up to 0.1 release, up to 0.5 lap, up to 0.9 service
    /// vent, above that emergency. A self-lapping valve on the same handle chases
    /// pipe = 6 bar at release down to 4.5 bar at full service, so that same line
    /// is what the amount is mapped onto here: every caller keeps saying "this
    /// much brake" and this file works the lever until the pipe agrees.
    /// </summary>
    public static class Brakes
    {
        /* Lever positions, one per band. Mid-band on purpose: the exact band edges
           are the game's, and writing onto an edge trusts a float comparison on
           the other side of it. */
        private const float LeverRelease = 0f;
        private const float LeverLap = 0.3f;
        private const float LeverApply = 0.7f;

        /// <summary>Pipe fully charged — what release feeds toward.</summary>
        private const float FullReleaseBar = 6f;

        /// <summary>Pipe at full service — where a self-lapping valve stops on its
        /// own at full handle. Venting further buys almost no cylinder and costs
        /// the whole recharge time when the train next wants to move.</summary>
        private const float FullServiceBar = 4.5f;

        /// <summary>How far the pipe may sit from its target before the lever
        /// moves. The band is what keeps the lever from sawing between apply and
        /// release around a target the pipe crosses every tick.</summary>
        private const float BandBar = 0.1f;

        /// <summary>
        /// Set the train brake to this much application, 0..1, on whatever valve
        /// the machine has. Self-lapping valves take the amount directly; the
        /// DM3's gets the lever worked — release, lap or apply — until the brake
        /// pipe reaches the pressure a self-lapping valve would have chosen for
        /// the same amount. Callers in a tick loop converge within a few ticks; a
        /// one-shot "park it" call lands the lever in apply and leaves it there,
        /// which only brakes HARDER than asked as the pipe drains.
        /// </summary>
        public static void Train(TrainCar? car, BaseControlsOverrider? overrider, float amount)
        {
            var control = overrider?.Brake;
            if (control == null)
                return;
            var brakes = car != null ? car.brakeSystem : null;
            if (brakes == null || brakes.selfLappingController)
            {
                control.Set(amount);
                return;
            }

            float lever;
            if (amount <= 0.01f)
            {
                // Asked for none: straight to release, no pressure test. The pipe
                // recharging IS the release, and lapping a half-charged pipe would
                // hold the very brake the caller wants gone.
                lever = LeverRelease;
            }
            else
            {
                float target = Mathf.Lerp(
                    FullReleaseBar, FullServiceBar, Mathf.Clamp01(amount));
                float pipe = brakes.brakePipePressure;
                lever = pipe > target + BandBar ? LeverApply
                    : pipe < target - BandBar ? LeverRelease
                    : LeverLap;
            }
            control.Set(lever);
        }
    }
}
