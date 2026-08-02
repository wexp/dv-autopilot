using CommsRadioAPI;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// The radio face of the autopilot: an AUTOPILOT mode on the comms radio.
    /// Point the laser at rolling stock and press the action button — a popup
    /// says what is under the laser (which locomotive, or that it is not a
    /// locomotive at all). The mode only exists when the CommsRadioAPI and
    /// MessageBox mods are installed; Main checks for both before calling into
    /// this class, so the references here never load without them.
    /// </summary>
    public static class CommsRadio
    {
        private static bool created;

        /// <summary>The laser colour of the mode — the console's accent blue.</summary>
        private static readonly Color LaserColor = new Color(0.25f, 0.75f, 1f);

        public static void Attach()
        {
            if (created)
                return;
            try
            {
                CreateMode();
            }
            catch
            {
                // Radio not built yet (fresh session load): wait for the API's
                // ready signal. On a hot reload the radio already exists and the
                // signal will never fire again, hence the try-first.
                ControllerAPI.Ready += CreateMode;
            }
        }

        private static void CreateMode()
        {
            if (created)
                return;
            CommsRadioMode.Create(new Home(null), LaserColor);
            created = true;
            Main.mod?.Logger.Log("Comms radio AUTOPILOT mode attached");
        }

        /// <summary>What the laser is pointing at, if it is a train car.</summary>
        private static TrainCar? CarUnderLaser(CommsRadioUtility utility)
        {
            var origin = utility.SignalOrigin;
            if (origin == null)
                return null;
            if (!Physics.Raycast(origin.position, origin.forward, out var hit, 100f,
                    LayerMask.GetMask("Train_Big_Collider")))
                return null;
            return TrainCar.Resolve(hit.transform.root);
        }

        /// <summary>The one state: follow the laser, and identify on action.</summary>
        private sealed class Home : AStateBehaviour
        {
            private readonly TrainCar? aimed;

            public Home(TrainCar? aimed)
                : base(new CommsRadioState(
                    titleText: "AUTOPILOT",
                    contentText: aimed == null ? "Aim at rolling stock." : aimed.ID,
                    actionText: aimed == null ? "" : "identify",
                    ledState: aimed == null ? LEDState.Off : LEDState.On,
                    buttonBehaviour: DV.ButtonBehaviourType.Regular))
            {
                this.aimed = aimed;
            }

            public override AStateBehaviour OnUpdate(CommsRadioUtility utility)
            {
                var car = CarUnderLaser(utility);
                return car == aimed ? (AStateBehaviour)this : new Home(car);
            }

            public override AStateBehaviour OnAction(CommsRadioUtility utility, InputAction action)
            {
                if (action != InputAction.Activate)
                    return this;
                var car = aimed;
                string message;
                if (car == null)
                {
                    message = "Nothing under the laser.";
                    utility.PlaySound(VanillaSoundCommsRadio.Warning);
                }
                else if (!car.IsLoco)
                {
                    message = $"{car.ID} is not a locomotive.";
                    utility.PlaySound(VanillaSoundCommsRadio.Warning);
                }
                else
                {
                    message = $"Locomotive {car.ID} ({car.carLivery?.id ?? car.carType.ToString()})"
                        + (Autopilot.IsUnderAutopilot(car.trainset) ? "\nUnder autopilot." : "\nIdle.");
                    utility.PlaySound(VanillaSoundCommsRadio.Confirm);
                }
                MessageBox.PopupAPI.ShowOk(message, _ => { }, "Autopilot", "Ok");
                // Activate must transition; a fresh instance re-arms the laser.
                return new Home(null);
            }
        }
    }
}
