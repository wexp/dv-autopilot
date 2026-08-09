using DV.CabControls;
using DV.Customization.Gadgets;
using DV.Customization.Gadgets.Implementations;
using DV.Simulation.Cars;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// Marker boards, a rotating beacon and the headlights, set for as long as a
    /// loco is driving itself — so a train under autopilot is recognisable from
    /// outside the cab, and lit the way a driver would leave it. Where everything
    /// goes is <see cref="GadgetProfiles"/>; this is only the fitting and the
    /// switching.
    ///
    /// The gadgets are REAL ones — the same prefabs the game bolts on when you fit
    /// them yourself, linked to the car's <c>Customization</c> and soldered so the
    /// car's electronics fuse powers them. What they deliberately are NOT is owned
    /// ones: the player's copies exist because a <see cref="GadgetItem"/> exists in
    /// storage, and the save is written from those items, not from the car. Ours
    /// have no item, so they are invisible to the save: nothing to load, nothing
    /// left behind if the game quits mid-run, and no way for them to end up in
    /// anybody's inventory.
    ///
    /// Only what this class created is ever removed. Whatever the player has bolted
    /// on — their own light bars, switches, speedometer — is never touched, because
    /// the only gadgets it can see are the ones in its own ledger.
    ///
    /// That ledger is also why our gadgets must never go through <c>Remove()</c>,
    /// which hands a gadget back to its item and logs an error when there isn't
    /// one. Ours unlink and are destroyed.
    /// </summary>
    internal static class Markings
    {
        /// <summary>A gadget we fitted, and the entry that put it there — kept
        /// together because the fitting says whether it lights forward or back.</summary>
        private readonly struct Fitted
        {
            internal readonly GadgetBase Gadget;
            internal readonly Fitting Fitting;

            internal Fitted(GadgetBase gadget, Fitting fitting)
            {
                Gadget = gadget;
                Fitting = fitting;
            }
        }

        /// <summary>Everything we did to one car, so all of it can be undone.</summary>
        private sealed class Set
        {
            internal readonly List<Fitted> Gadgets = new List<Fitted>();
            internal LiveryProfile Profile;
            /// <summary>The car this set belongs to. Held rather than read back off
            /// a gadget, because with every marking switched off there are no
            /// gadgets to read it from and the headlights still need giving back.</summary>
            internal TrainCar? Car;
            /// <summary>The headlight switches as we found them, to be put back.
            /// The player set those; we are only borrowing them.</summary>
            internal float? HeadlightFrontWas;
            internal float? HeadlightRearWas;
            /// <summary>Last direction applied, so a per-tick call is free.</summary>
            internal bool? Forward;
            /// <summary>What the boards are currently saying.</summary>
            internal MarkingState State;
        }

        private static readonly Dictionary<string, Set> fitted = new Dictionary<string, Set>();

        /// <summary>Prefabs by name, filled on demand. A miss is cached too: a
        /// gadget nobody owns may simply not be loaded, and there is no point
        /// sweeping every object in the scene once per dispatch to find that out
        /// again.</summary>
        private static readonly Dictionary<string, GadgetBase?> prefabs =
            new Dictionary<string, GadgetBase?>();

        /// <summary>
        /// Dress this loco. Silent no-op if it is already dressed. Nothing here may
        /// stop a run: markings are cosmetic, and a missing prefab is not a reason
        /// to refuse to drive.
        /// </summary>
        internal static void Fit(TrainCar? car)
        {
            if (car == null || car.Customization == null)
                return;
            if (fitted.TryGetValue(car.CarGUID, out var already))
            {
                // A live set: leave it alone. A set of destroyed objects means the
                // car was despawned and has come back, and the stale entry would
                // otherwise keep it bare for the rest of the session. With every
                // marking switched off there is nothing to judge by, so the car
                // itself is the evidence — a returned one is a different instance.
                bool live = already.Gadgets.Count > 0
                    ? already.Gadgets.Any(f => f.Gadget != null)
                    : already.Car == car;
                if (live)
                    return;
                fitted.Remove(car.CarGUID);
            }

            var known = GadgetProfiles.For(car.carLivery?.id);
            var set = new Set { Profile = known ?? GadgetProfiles.Estimated(car), Car = car };
            var wanted = Wanted(set.Profile.Fittings);

            foreach (var fitting in wanted)
            {
                try
                {
                    var gadget = Build(car, fitting);
                    if (gadget != null)
                        set.Gadgets.Add(new Fitted(gadget, fitting));
                }
                catch (Exception e)
                {
                    // One bad fitting must not cost the rest of the set.
                    Main.mod?.Logger.Log($"[MARKINGS] {car.ID}: {fitting.Prefab} failed: {e}");
                }
            }

            var overrider = car.SimController?.controlsOverrider;
            set.HeadlightFrontWas = overrider?.HeadlightsFront?.Value;
            set.HeadlightRearWas = overrider?.HeadlightsRear?.Value;

            fitted[car.CarGUID] = set;
            Main.DebugLog(() => $"[MARKINGS] {car.ID} ({car.carLivery?.id}): fitted "
                + $"{set.Gadgets.Count}/{wanted.Count}"
                + (wanted.Count < set.Profile.Fittings.Length ? " (some switched off)" : "")
                + (known == null ? " (estimated — no profile)" : ""));
        }

        /// <summary>How far from a gadget a bracket may stand and still count as
        /// the one holding it. Profiles measure the standoff in millimetres, so
        /// this is generous by a wide margin and still nowhere near the next
        /// fitting along the car.</summary>
        private const float MountReach = 0.15f;

        /// <summary>
        /// The fittings that are still switched on. Beacons and marker boards each
        /// have their own setting; a bracket has no setting of its own, because a
        /// bracket holding nothing is not worth bolting on. Which bracket belongs
        /// to which gadget is not written down anywhere — the profiles have no such
        /// link — so it is read off the geometry: a bracket stands within a couple
        /// of millimetres of the thing it carries, and nothing else is near it.
        /// </summary>
        private static List<Fitting> Wanted(Fitting[] all)
        {
            bool beacons = Main.settings.markingBeacons;
            bool bars = Main.settings.markingLightBars;
            if (beacons && bars)
                return all.ToList();

            bool On(Fitting f) =>
                f.Kind == GadgetKind.Beacon ? beacons
                : f.Kind == GadgetKind.Light ? bars
                : false;

            var kept = all.Where(On).ToList();
            // Original order, so a bracket is still built before what sits on it.
            return all.Where(f => f.Kind == GadgetKind.Plain
                    ? kept.Any(k => (k.Position - f.Position).sqrMagnitude
                        < MountReach * MountReach)
                    : On(f))
                .ToList();
        }

        /// <summary>
        /// Which way the train is being driven. Green marks the leading end and red
        /// the trailing one, so everything changes over when the reverser does — and
        /// the headlights go with them. Cheap to call every tick: it does nothing
        /// unless the direction actually changed.
        /// </summary>
        internal static void SetDirection(TrainCar? car, bool forward)
        {
            if (car == null || !fitted.TryGetValue(car.CarGUID, out var set))
                return;
            // A run that has already declared itself finished or failed keeps saying
            // so. The train may still be rolling to a stand, and a board that went
            // back to white while it did would undo the message.
            if (set.State != MarkingState.Driving || set.Forward == forward)
                return;
            set.Forward = forward;
            Show(set);
            Headlights(car, set, forward);
        }

        /// <summary>
        /// How the run ended. Both ends go green for an arrival and red for a
        /// failure, and the whole set comes off a few seconds later — long enough to
        /// be read by somebody watching the train, short enough that a parked loco
        /// is not left wearing autopilot boards.
        /// </summary>
        internal static void Conclude(string guid, bool arrived)
        {
            if (!fitted.TryGetValue(guid, out var set))
                return;
            set.State = arrived ? MarkingState.Finished : MarkingState.Error;
            Show(set);
            Updater.Run(StripAfter(guid, FinishedSeconds));
        }

        /// <summary>How long the closing colour stays up before the set is stripped.</summary>
        private const float FinishedSeconds = 5f;

        private static IEnumerator StripAfter(string guid, float seconds)
        {
            yield return new WaitForSecondsRealtime(seconds);
            // Unfit is idempotent, so a set already taken off by a manual stop or a
            // mod unload in the meantime is simply not there any more.
            Unfit(guid);
        }

        /// <summary>Paint every marker board for the set's current state.</summary>
        private static void Show(Set set)
        {
            bool forward = set.Forward ?? true;
            foreach (var f in set.Gadgets)
            {
                if (f.Gadget == null || f.Fitting.AtFront == null)
                    continue;
                if (!(f.Gadget is GadgetLight board))
                    continue;
                // The boards never go dark; what changes is the colour. Powering one
                // off would make "not the leading end" look the same as "not fitted".
                Paint(board, GadgetProfiles.MarkerColour(
                    set.State, f.Fitting.AtFront.Value, forward));
                board.PowerSwitch = true;
            }
        }

        /// <summary>
        /// Put the headlight switch at each end where the profile asks, converting
        /// its notch offset into this switch's own port value. The number of
        /// positions differs from loco to loco, which is why the profile stores
        /// notches: the controller knows how many it has and where its off position
        /// sits, so the same "two toward the beams" lands correctly on all of them.
        /// </summary>
        private static void Headlights(TrainCar car, Set set, bool forward)
        {
            var overrider = car.SimController?.controlsOverrider;
            if (overrider == null)
                return;
            var lights = car.GetComponentInChildren<HeadlightsMainController>(true);

            int leading = set.Profile.LeadingNotch;
            int trailing = set.Profile.TrailingNotch;
            // "Front" is the car's own front, which is the leading end only while
            // the reverser says forward.
            overrider.HeadlightsFront?.Set(PortValue(lights, true, forward ? leading : trailing));
            overrider.HeadlightsRear?.Set(PortValue(lights, false, forward ? trailing : leading));
        }

        /// <summary>
        /// The 0..1 control value for a switch position given as an offset from off.
        /// Falls back to the neutral value the game itself uses when the controller
        /// cannot be found, which leaves the switch alone rather than guessing.
        /// </summary>
        private static float PortValue(HeadlightsMainController? lights, bool front, int notch)
        {
            const float neutral = 0.4f; // HeadlightsControlBase.NEUTRAL_VALUE
            if (lights == null)
                return neutral;
            int count = lights.GetSetupCount(front);
            if (count < 2)
                return neutral;
            int index = Mathf.Clamp(lights.GetOffIndex(front) + notch, 0, count - 1);
            return index / (float)(count - 1);
        }

        /// <summary>Strip them again, and give the headlights back. Safe for a car
        /// that never had any.</summary>
        internal static void Unfit(TrainCar? car)
        {
            if (car != null)
                Unfit(car.CarGUID);
        }

        internal static void Unfit(string guid)
        {
            if (!fitted.TryGetValue(guid, out var set))
                return;
            fitted.Remove(guid);

            foreach (var f in set.Gadgets)
            {
                try
                {
                    if (f.Gadget == null)
                        continue;
                    // Unlink, never Remove: Remove exists to give the gadget back to
                    // the item it came out of, and ours came out of nothing.
                    if (f.Gadget.IsLinked)
                        f.Gadget.Unlink();
                    UnityEngine.Object.Destroy(f.Gadget.gameObject);
                }
                catch (Exception e)
                {
                    Main.mod?.Logger.Log($"[MARKINGS] could not strip {guid}: {e}");
                }
            }

            // The headlights were the player's before the run and are theirs again
            // after it. Leaving a parked loco lit up is our mess, not theirs.
            try
            {
                var car = set.Car;
                var overrider = car == null ? null : car.SimController?.controlsOverrider;
                if (overrider != null)
                {
                    if (set.HeadlightFrontWas != null)
                        overrider.HeadlightsFront?.Set(set.HeadlightFrontWas.Value);
                    if (set.HeadlightRearWas != null)
                        overrider.HeadlightsRear?.Set(set.HeadlightRearWas.Value);
                }
            }
            catch (Exception e)
            {
                Main.mod?.Logger.Log($"[MARKINGS] could not restore the headlights on {guid}: {e}");
            }
        }

        /// <summary>Strip every set we fitted. For mod unload.</summary>
        internal static void UnfitAll()
        {
            foreach (var guid in fitted.Keys.ToList())
                Unfit(guid);
        }

        private static GadgetBase? Build(TrainCar car, Fitting fitting)
        {
            var source = Prefab(fitting.Prefab);
            if (source == null)
                return null;

            // Instantiating an INACTIVE prefab gives an inactive clone, so Awake has
            // not run yet and the object can be placed before it wakes.
            var clone = UnityEngine.Object.Instantiate(source.gameObject);
            clone.name = "Autopilot" + fitting.Prefab;
            var gadget = clone.GetComponent<GadgetBase>();

            // Where the game parents gadgets: the interior container, a root object
            // that tracks the car and lives exactly as long as it does. Its transform
            // is the car's, which is the frame the profile's numbers are measured in.
            clone.transform.SetParent(car.interior, worldPositionStays: false);
            clone.transform.localPosition = fitting.Position;
            clone.transform.localRotation = Quaternion.Euler(fitting.Euler);

            // Start() on this one reads GadgetItem.SpecificMaterial with no null
            // check, and an item-less gadget has no GadgetItem — so every one of
            // these threw the moment the clone woke. What it does is swap in the
            // material the SHOP item carries: a beacon's coloured dome, which is a
            // material and not a number, so it is the ONLY way to get one. Do its
            // work here with the material from the item the profile names, then
            // take it off. Immediate, not deferred: plain Destroy only takes effect
            // after the current update loop, by which time the component it is
            // meant to stop has already had its Start called and thrown.
            var specific = ItemMaterial(fitting.Item);
            foreach (var applier in clone.GetComponentsInChildren<GadgetItemSpecificMaterialApplier>(true))
            {
                if (specific != null)
                    ApplySpecific(applier, specific);
                UnityEngine.Object.DestroyImmediate(applier);
            }

            clone.SetActive(true);

            // AssignItem is what normally fills this in, and OnAfterLinked walks it
            // with no null check. No item here, so it has to be set by hand or Link
            // throws.
            gadget.nestedInteractables = gadget.GetComponentsInChildren<ControlImplBase>(true);

            car.Customization.Add(gadget);
            // The fuse only reaches a gadget that is soldered. Nobody is coming to do
            // it, so it arrives finished. Brackets are not solderable and this is a
            // no-op on them.
            gadget.MakeSoldered();

            Colour(gadget, fitting);
            // Dark until the first tick says which way the train is going: a board
            // lit the instant it is bolted on would show the wrong colour for as
            // long as it took the drive loop to come round.
            if (fitting.AtFront != null)
                gadget.PowerSwitch = false;
            return gadget;
        }

        /// <summary>
        /// What <c>GadgetItemSpecificMaterialApplier.Start</c> does, on a clone that
        /// has no item to read it from: put the material in the one slot the
        /// component names and leave the rest of the mesh alone. The array from
        /// <c>sharedMaterials</c> is a copy, so it has to be written back.
        /// </summary>
        private static void ApplySpecific(
            GadgetItemSpecificMaterialApplier applier, Material material)
        {
            var renderer = applier.GetComponent<MeshRenderer>();
            if (renderer == null)
                return;
            var slots = renderer.sharedMaterials;
            if (applier.materialIndex < 0 || applier.materialIndex >= slots.Length)
                return;
            slots[applier.materialIndex] = material;
            renderer.sharedMaterials = slots;
        }

        /// <summary>The SPECIFIC MATERIAL a catalogue item carries — a beacon's
        /// coloured dome. Cached by item name, misses included: an item nobody owns
        /// may not be loaded, and there is no point sweeping the scene for it
        /// again on every fitting.</summary>
        private static readonly Dictionary<string, Material?> materials =
            new Dictionary<string, Material?>();

        private static Material? ItemMaterial(string? item)
        {
            if (item == null)
                return null;
            if (materials.TryGetValue(item, out var cached))
                return cached;
            // Same test as Prefab(): a loaded asset, not an instance standing in
            // the world — which keeps a "BeaconRed(Clone)" somebody is carrying out
            // of the result.
            var found = Resources.FindObjectsOfTypeAll<GadgetItem>()
                .FirstOrDefault(i => i != null
                    && i.name == item
                    && !i.gameObject.activeInHierarchy
                    && i.transform.parent == null)
                ?.SpecificMaterial;
            materials[item] = found;
            if (found == null)
                Main.mod?.Logger.Log($"[MARKINGS] no item called {item} is loaded — "
                    + "that gadget keeps the prefab's own material.");
            return found;
        }

        /// <summary>
        /// Paint it. OnItemAssigned is where the colour normally comes from and it
        /// does not run for an item-less gadget, so this repeats what it would have
        /// done — reading the same numbers off the profile that the shop item
        /// carries as attributes.
        /// </summary>
        private static void Colour(GadgetBase gadget, Fitting fitting)
        {
            switch (fitting.Kind)
            {
                case GadgetKind.Light when gadget is GadgetLight light:
                    Paint(light, fitting.Colour);
                    break;

                case GadgetKind.Beacon when gadget is GadgetBeacon beacon:
                    // Hue and the prefab's own saturation, exactly as the item does
                    // it — the game's "red" beacon is a little orange and matching
                    // the item matters more than matching the word.
                    if (beacon.lighting != null)
                        beacon.lighting.color =
                            Color.HSVToRGB(fitting.Hue / 360f, beacon.saturation, 1f);
                    // Bounce the switch so the new colour is pushed through the
                    // lighting controller at the gadget's current power.
                    beacon.PowerSwitch = false;
                    beacon.PowerSwitch = true;
                    break;
            }
        }

        /// <summary>
        /// Set a light bar's colour, at any time. A light bar has no
        /// LightingGadgetController — the dump shows none on the prefab — so its
        /// IndicatorEmission is the whole of it: the lamp, the emissive face and the
        /// glare quad, which reads its own separate colour and so is painted too.
        ///
        /// Brightness is left alone. GadgetLight.Update assigns emission.Value every
        /// frame from the gadget's powered state, and the setter repaints from
        /// whatever emissionColor now says — which is what makes a live recolour
        /// take effect without touching anything else.
        /// </summary>
        private static void Paint(GadgetLight light, Color colour)
        {
            var emission = light.emission;
            if (emission != null)
            {
                emission.emissionColor = colour * light.intensityMultiplier;
                emission.glareColor = colour;
                if (emission.emissionLight != null)
                    emission.emissionLight.color = colour;
            }
            // Nothing on a light bar, but Headlight_Gadget and the modern lamps are
            // the same component with one of these, so paint it when it is there.
            if (light.lighting != null)
                light.lighting.UpdateColor(colour);
        }

        /// <summary>
        /// A gadget prefab by name, found the way loco prefabs are: a loaded object
        /// with no scene — inactive and unparented. The name must match exactly,
        /// which is what keeps a stray "LightBar_Gadget(Clone)" — a gadget somebody
        /// took off, which looks identical to this test — out of the result.
        /// </summary>
        private static GadgetBase? Prefab(string name)
        {
            if (prefabs.TryGetValue(name, out var cached))
                return cached;
            var found = Resources.FindObjectsOfTypeAll<GadgetBase>()
                .FirstOrDefault(g => g != null
                    && g.name == name
                    && !g.gameObject.activeInHierarchy
                    && g.transform.parent == null);
            prefabs[name] = found;
            if (found == null)
                Main.mod?.Logger.Log($"[MARKINGS] no prefab called {name} is loaded — "
                    + "that part of the set will be missing. Dump gadgets to see what is.");
            return found;
        }
    }
}
