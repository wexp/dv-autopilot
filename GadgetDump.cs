using DV.Customization;
using DV.Customization.Gadgets;
using DV.Customization.Gadgets.Implementations;
using DV.Simulation.Cars;
using DV.Utils;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// The customization system, written out from a live world.
    ///
    /// A gadget is a <see cref="GadgetBase"/> — a MonoBehaviour on its own small
    /// prefab — that is LINKED to a <see cref="Customization"/>. On a locomotive
    /// that host is <c>TrainCar.Customization</c>, and everything linked to it is
    /// in <c>Customizers</c>. Gadgets carry no power of their own: a gadget reads
    /// power from the car's electronics fuse, and reads its VALUE from a
    /// <see cref="GadgetSwitch"/> wired to it. A beacon with nothing wired to it
    /// runs whenever the fuse is live.
    ///
    /// This dump exists because none of the prefab names appear as strings in the
    /// game's assemblies — they live in the asset bundles. The only way to learn
    /// what a "lightbar" actually is, is to look at one that is already fitted.
    /// </summary>
    internal static class GadgetDump
    {
        private const BindingFlags Members =
            BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;

        /// <summary>
        /// Write gadgets.txt beside the mod: every gadget fitted to every car
        /// streamed in, plus the catalogue of gadget prefabs the game has loaded.
        /// Returns a one-line report.
        /// </summary>
        internal static string Write(string? dir = null)
        {
            var text = new StringBuilder();
            int fitted = DumpFittedGadgets(text);
            text.AppendLine();
            DumpHeadlights(text);
            text.AppendLine();
            int known = DumpCatalogue(text);

            string folder = string.IsNullOrEmpty(dir) ? (Main.mod?.Path ?? ".") : dir!;
            string path = Path.Combine(folder, "gadgets.txt");
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
            return $"{fitted} fitted gadget(s), {known} prefab(s) → {path} (and the log)";
        }

        /// <summary>
        /// What is bolted to the cars right now. Position is given in the car's
        /// own frame, because that is the number a mod needs in order to put
        /// something in the same place on another loco of the same type.
        /// </summary>
        private static int DumpFittedGadgets(StringBuilder text)
        {
            var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
            if (registry == null)
            {
                text.AppendLine("=== FITTED: no car registry — is the world loaded?");
                return 0;
            }

            int count = 0;
            text.AppendLine("=== FITTED GADGETS (cars streamed in)");
            foreach (var car in registry.Values.OrderBy(c => c?.ID ?? ""))
            {
                var custom = car?.Customization;
                if (car == null || custom == null || custom.Customizers.Count == 0)
                    continue;

                text.AppendLine($"--- {car.ID}  livery={car.carLivery?.id ?? "?"}"
                    + $"  fuse={custom.electronicsFuseID}"
                    + $"  parent={Describe(custom.GetParentingTransform())}");
                foreach (var mod in custom.Customizers)
                {
                    count++;
                    DumpOne(car, mod, text);
                }
            }
            if (count == 0)
                text.AppendLine("  (none — no car nearby has anything fitted)");
            return count;
        }

        private static void DumpOne(TrainCar car, Customization.CustomizerBase mod, StringBuilder text)
        {
            text.AppendLine($"  [{mod.Index}] {mod.GetType().FullName}"
                + $"  object={mod.name}");

            // Where it sits, in the CAR's frame rather than the parenting
            // transform's: the interior container moves with LOD, the car does not.
            var t = mod.transform;
            Vector3 pos = car.transform.InverseTransformPoint(t.position);
            Vector3 rot = (Quaternion.Inverse(car.transform.rotation) * t.rotation).eulerAngles;
            text.AppendLine($"      at car-local {pos.x:F3},{pos.y:F3},{pos.z:F3}"
                + $"  euler {rot.x:F1},{rot.y:F1},{rot.z:F1}");

            if (!(mod is TrainCarCustomization.TrainCarCustomizerBase tcb))
                return;
            text.AppendLine($"      uid={tcb.UID}  power={tcb.PowerState}  switch={tcb.PowerSwitch}"
                + $"  solderable={tcb.IsSolderable}  soldered={tcb.IsSoldered}");

            if (mod is GadgetBase gadget)
            {
                text.AppendLine($"      mountedOn={(gadget.MountedOn == null ? "-" : gadget.MountedOn.name)}"
                    + $"  removal={gadget.GetValidRemovalMethods()}"
                    + $"  item={(gadget.GadgetItem == null ? "-" : gadget.GadgetItem.name)}"
                    + $"  hud={(gadget.HUDPrefab == null ? "-" : gadget.HUDPrefab.name)}");
                DumpWiring(gadget, text);
                DumpState(gadget, text);
                DumpLighting(gadget, text);
            }
        }

        /// <summary>
        /// What this gadget is wired to. A light without a switch is always on
        /// when powered; a light WITH one follows that switch's output, so
        /// driving the light from code means driving the switch, not the light.
        /// </summary>
        private static void DumpWiring(GadgetBase gadget, StringBuilder text)
        {
            if (!gadget.HasAnyWirePorts)
                return;
            var links = new List<GadgetWiringModule.WireLinkPort>();
            foreach (var port in gadget.WireLinkPorts)
            {
                links.Clear();
                port.GetLinks(links);
                string to = links.Count == 0
                    ? "nothing"
                    : string.Join(", ", links.Select(l =>
                        $"{l.owner.name}#{(l.owner as TrainCarCustomization.TrainCarCustomizerBase)?.UID}"));
                text.AppendLine($"      wire port {port.GetType().Name}"
                    + $" passive={port.isPassive} → {to}");
            }
        }

        /// <summary>The live value of the gadget kinds that have one.</summary>
        private static void DumpState(GadgetBase gadget, StringBuilder text)
        {
            switch (gadget)
            {
                case GadgetSwitch sw:
                    text.AppendLine($"      switch raw={sw.RawOutputValue}  default={sw.DefaultOutputValue}");
                    break;
                case GadgetBeacon beacon:
                    text.AppendLine($"      beacon value={beacon.CurrentValue}"
                        + $"  rotationSpeed={beacon.rotationSpeed}  saturation={beacon.saturation}"
                        + $"  head={Describe(beacon.head)}");
                    break;
                case GadgetOscillatingLight osc:
                    text.AppendLine($"      oscillating frequency={osc.frequency}"
                        + $"  amplitude={osc.amplitude}  head={Describe(osc.head)}");
                    break;
                case GadgetLight light:
                    text.AppendLine($"      light value={light.Value}  colour={light.Color}"
                        + $"  intensity={light.IntensityMultiplier}");
                    break;
            }
        }

        /// <summary>
        /// The lighting controller is the piece that actually emits — set its
        /// colour alpha and the lamp, the beam and the glare all follow.
        /// </summary>
        private static void DumpLighting(GadgetBase gadget, StringBuilder text)
        {
            foreach (var lighting in gadget.GetComponentsInChildren<LightingGadgetController>(true))
                text.AppendLine($"      lighting {Describe(lighting.transform)}"
                    + $"  colour={lighting.color}  lights={lighting.lights?.Length ?? 0}"
                    + $"  beams={lighting.beams?.Length ?? 0}  glares={lighting.glares?.Length ?? 0}");
        }

        /// <summary>
        /// What a locomotive's OWN headlights are made of — the parts the game
        /// switches, as opposed to the marker gadgets bolted on beside them.
        ///
        /// Each <see cref="Headlight"/> owns up to four separable things: a lamp
        /// renderer whose material is swapped between lit and unlit, a glare quad, a
        /// volumetric beam, and the real-time Light itself. Whether the surrounding
        /// HOUSING is separable too is the question this answers: if the lamp
        /// renderer is its own object with its own mesh, it can be hidden; if it is
        /// a submesh of the car body, it cannot be, and only the emission can go.
        /// </summary>
        private static void DumpHeadlights(StringBuilder text)
        {
            var registry = SingletonBehaviour<TrainCarRegistry>.Instance?.logicCarToTrainCar;
            if (registry == null)
            {
                text.AppendLine("=== HEADLIGHTS: no car registry");
                return;
            }
            text.AppendLine("=== HEADLIGHTS (one car per livery)");
            var seen = new HashSet<string>();
            foreach (var car in registry.Values.OrderBy(c => c?.ID ?? ""))
            {
                if (car == null || !car.IsLoco || !seen.Add(car.carLivery?.id ?? car.ID))
                    continue;
                var main = car.GetComponentInChildren<HeadlightsMainController>(true);
                if (main == null)
                {
                    text.AppendLine($"--- {car.ID}  livery={car.carLivery?.id}  no HeadlightsMainController");
                    continue;
                }
                text.AppendLine($"--- {car.ID}  livery={car.carLivery?.id}"
                    + $"  setupsFront={main.GetSetupCount(true)} setupsRear={main.GetSetupCount(false)}"
                    + $"  offIndexFront={main.GetOffIndex(true)} offIndexRear={main.GetOffIndex(false)}");

                // EVERYTHING drawn inside the headlight container, not just the
                // lamps the controller knows about. This is what answers "does the
                // bezel go too": if the housing mesh is in here it disappears with
                // the glass, and if it is not, it belongs to the car body and
                // hiding the lamps will leave empty sockets.
                foreach (var root in HeadlightRoots(car))
                {
                    var drawn = root.GetComponentsInChildren<Renderer>(true);
                    text.AppendLine($"  container {root.name}  renderers={drawn.Length}"
                        + $"  lights={root.GetComponentsInChildren<Light>(true).Length}");
                    foreach (var renderer in drawn)
                        text.AppendLine($"      draws {Describe(renderer.transform)}"
                            + $"  mesh={renderer.GetComponent<MeshFilter>()?.sharedMesh?.name ?? renderer.GetType().Name}"
                            + $"  on={renderer.enabled}");
                }

                foreach (var sub in car.GetComponentsInChildren<HeadlightsSubControllerBase>(true))
                {
                    text.AppendLine($"  sub {Describe(sub.transform)}  front={sub.isFront}"
                        + $"  headlights={sub.headlights?.Length ?? 0}"
                        + $"  lightSources={sub.lightSources?.Length ?? 0}");
                    foreach (var light in sub.lightSources ?? Array.Empty<Light>())
                        if (light != null)
                            text.AppendLine($"      lightSource {Describe(light.transform)}");
                }

                foreach (var lamp in car.GetComponentsInChildren<Headlight>(true))
                {
                    text.AppendLine($"  lamp {Describe(lamp.transform)}");
                    // The renderer is the interesting one: its own object means the
                    // lamp can be hidden outright, a shared one means it is part of
                    // the body and only its material can change.
                    var renderer = lamp.headlightRenderer;
                    if (renderer == null)
                        text.AppendLine("      renderer: none");
                    else
                        text.AppendLine($"      renderer {Describe(renderer.transform)}"
                            + $"  mesh={(renderer as MeshRenderer)?.GetComponent<MeshFilter>()?.sharedMesh?.name ?? "?"}"
                            + $"  materials={renderer.sharedMaterials?.Length ?? 0}"
                            + $"  ownObject={(renderer.transform == lamp.transform ? "SHARED with the lamp" : "separate")}"
                            + $"  siblings={renderer.transform.parent?.childCount ?? 0}");
                    text.AppendLine($"      glare={(lamp.glare == null ? "-" : Describe(lamp.glare.transform))}");
                    text.AppendLine($"      beam={(lamp.beamData.beam == null ? "-" : Describe(lamp.beamData.beam.transform))}");
                }
            }
        }

        /// <summary>
        /// Every gadget prefab the game has loaded, whether or not one is fitted
        /// anywhere. Prefabs are the objects with no scene: not active in the
        /// hierarchy and with no parent, exactly as loco prefabs are found.
        /// </summary>
        private static int DumpCatalogue(StringBuilder text)
        {
            text.AppendLine("=== GADGET PREFABS LOADED");

            var items = Resources.FindObjectsOfTypeAll<GadgetItem>()
                .Where(IsPrefab)
                .OrderBy(i => i.name)
                .ToList();
            text.AppendLine($"-- items ({items.Count}) — the thing you carry and place");
            foreach (var item in items)
                text.AppendLine($"  {item.name,-34} → {(item.GadgetPrefab == null ? "?" : item.GadgetPrefab.name)}"
                    + $"  ({item.GadgetPrefab?.GetType().Name ?? "-"})"
                    + $"{Attributes(item)}");

            // The gadget prefabs themselves: some are reachable only from a
            // mount or another gadget, so they never appear in the item list.
            var gadgets = Resources.FindObjectsOfTypeAll<GadgetBase>()
                .Where(IsPrefab)
                .OrderBy(g => g.GetType().Name).ThenBy(g => g.name)
                .ToList();
            text.AppendLine($"-- gadgets ({gadgets.Count}) — the thing that ends up on the car");
            foreach (var gadget in gadgets)
                text.AppendLine($"  {gadget.name,-34} {gadget.GetType().FullName}"
                    + $"  mountPoints={gadget.RequiredMountPoints}"
                    + $"  removal={gadget.RawRemovalMethods}");
            return items.Count + gadgets.Count;
        }

        /// <summary>
        /// The child (or children) of the car that own the headlight assembly —
        /// "[headlights]" on a DE6, "[headlights_dh4]" on a DH4. Found by walking up
        /// from the sub-controllers rather than by name, so it works on any livery.
        /// The same walk Markings uses to hide them.
        /// </summary>
        private static IEnumerable<Transform> HeadlightRoots(TrainCar car)
        {
            var roots = new HashSet<Transform>();
            foreach (var sub in car.GetComponentsInChildren<HeadlightsSubControllerBase>(true))
            {
                var t = sub.transform;
                while (t != null && t.parent != null && t.parent != car.transform)
                    t = t.parent;
                if (t != null && t.parent == car.transform)
                    roots.Add(t);
            }
            return roots;
        }

        /// <summary>An asset, not something already standing in the world.</summary>
        private static bool IsPrefab(Component c) =>
            c != null && !c.gameObject.activeInHierarchy && c.transform.parent == null;

        /// <summary>
        /// The item's custom attributes — HUE picks a beacon's colour, so the
        /// same prefab appears several times in a shop under different names.
        /// </summary>
        private static string Attributes(GadgetItem item)
        {
            var attributes = item.customAttributes;
            if (attributes == null || attributes.Length == 0)
                return "";
            return "  " + string.Join(" ", attributes.Select(a => $"{a.name}={a.value}"));
        }

        private static string Describe(Transform? t)
        {
            if (t == null)
                return "-";
            var parts = new List<string>();
            for (var cur = t; cur != null; cur = cur.parent)
                parts.Add(cur.name);
            parts.Reverse();
            return string.Join("/", parts);
        }
    }
}
