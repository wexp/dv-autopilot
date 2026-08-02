using CommsRadioAPI;
using System.Linq;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// The radio face of the autopilot: an AUTOPILOT mode on the comms radio.
    /// Point the laser at a locomotive — it gets the game's cyan hologram box —
    /// and press the action button to pin it to (or remove it from) the
    /// dispatcher tablet. Feedback is a three-second line on the radio's own
    /// LCD, not a popup. Requires the CommsRadioAPI mod; Main checks before
    /// touching this class.
    /// </summary>
    public static class CommsRadio
    {
        private static bool created;

        /// <summary>The laser colour of the mode — the console's accent blue.</summary>
        private static readonly Color LaserColor = new Color(0.25f, 0.75f, 1f);

        private static readonly Vector3 HighlightBoundsExtension = new Vector3(0.25f, 0.8f, 0f);

        /// <summary>The vanilla car-deleter mode: we borrow its laser muzzle, its
        /// hologram highlighter object and its selection material.</summary>
        private static DV.CommsRadioCarDeleter? deleter;

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
            // A hot reload leaves the PREVIOUS assembly's mode on the radio and
            // this one becomes a duplicate — strip stale API modes first. Not
            // FindObjectOfType: a stowed radio is INACTIVE and gets skipped.
            var controller = Resources.FindObjectsOfTypeAll<DV.CommsRadioController>()
                .FirstOrDefault(c => c != null && c.gameObject.scene.IsValid());
            if (controller != null)
            {
                var stale = controller.GetComponents<CommsRadioMode>();
                if (stale.Length > 0)
                {
                    controller.allModes.RemoveAll(m => m is CommsRadioMode);
                    foreach (var s in stale)
                        UnityEngine.Object.Destroy(s);
                    controller.ReactivateModes();
                }
            }
            var mode = CommsRadioMode.Create(new Home(null, "", 0f), LaserColor);
            deleter = ControllerAPI.GetVanillaMode(VanillaMode.Clear) as DV.CommsRadioCarDeleter;
            if (deleter != null && deleter.signalOrigin != null)
                mode.OverrideSignalOrigin(deleter.signalOrigin);
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
            // Not transform.root: cars hang under the world mover, so "root" is
            // the world container and never the car. Walk up to the car itself.
            return hit.collider.GetComponentInParent<TrainCar>();
        }

        /// <summary>The game's own cyan hologram box, borrowed from the deleter
        /// mode (modes are exclusive, so it is idle while ours is active).</summary>
        private static void Highlight(TrainCar? car)
        {
            if (deleter == null || deleter.trainHighlighter == null)
                return;
            var holo = deleter.trainHighlighter;
            if (car == null)
            {
                holo.SetActive(false);
                holo.transform.SetParent(null);
                return;
            }
            var render = holo.GetComponentInChildren<MeshRenderer>(true);
            if (render != null)
                render.material = deleter.selectionMaterial;
            holo.transform.localScale = car.Bounds.size + HighlightBoundsExtension;
            Vector3 up = car.transform.up * (holo.transform.localScale.y / 2f);
            Vector3 fwd = car.transform.forward * car.Bounds.center.z;
            holo.transform.SetPositionAndRotation(
                car.transform.position + up + fwd, car.transform.rotation);
            holo.SetActive(true);
            holo.transform.SetParent(car.transform, worldPositionStays: true);
        }

        /// <summary>The one state: follow the laser (hologram on whatever it
        /// touches), pin on action, flash the result on the LCD for 3 s.</summary>
        private sealed class Home : AStateBehaviour
        {
            private readonly TrainCar? aimed;
            private readonly string flash;
            private readonly float flashUntil;

            public Home(TrainCar? aimed, string flash, float flashUntil)
                : base(new CommsRadioState(
                    titleText: "AUTOPILOT",
                    contentText: flashUntil > Time.time && flash.Length > 0
                        ? flash
                        : aimed == null ? "Aim at a locomotive." : aimed.ID,
                    actionText: aimed == null ? "" : "pin",
                    ledState: aimed == null ? LEDState.Off : LEDState.On,
                    buttonBehaviour: DV.ButtonBehaviourType.Regular))
            {
                this.aimed = aimed;
                this.flash = flash;
                this.flashUntil = flashUntil;
            }

            public override void OnLeave(CommsRadioUtility utility, AStateBehaviour? next)
            {
                if (next == null)
                    Highlight(null); // mode switched away — drop the hologram
            }

            public override AStateBehaviour OnUpdate(CommsRadioUtility utility)
            {
                var car = CarUnderLaser(utility);
                if (car != aimed)
                {
                    Highlight(car);
                    return new Home(car, flash, flashUntil);
                }
                // Flash expired: rebuild so the LCD returns to the aim readout.
                if (flash.Length > 0 && Time.time > flashUntil)
                    return new Home(aimed, "", 0f);
                return this;
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
                    message = TabletPins.Toggle(car);
                    utility.PlaySound(VanillaSoundCommsRadio.Confirm);
                }
                // The result lives on the LCD for three seconds — no popup.
                return new Home(aimed, message, Time.time + 3f);
            }
        }
    }
}
