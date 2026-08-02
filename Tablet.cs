using DV.InventorySystem;
using System;
using System.Collections.Generic;
using System.Linq;
using TMPro;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>The locos pinned to the dispatcher tablet, in pin order. The
    /// comms radio's AUTOPILOT mode adds and removes them; every tablet in the
    /// world shows the same board.</summary>
    public static class TabletPins
    {
        public const int Max = 4;
        public static readonly List<string> Guids = new List<string>();

        /// <summary>Pin (or unpin) a loco. Returns the popup message.</summary>
        public static string Toggle(TrainCar car)
        {
            if (Guids.Remove(car.CarGUID))
                return $"{car.ID} removed from the tablet.";
            if (Guids.Count >= Max)
                return $"The tablet is full ({Max} windows) — remove one first.";
            Guids.Add(car.CarGUID);
            return $"{car.ID} pinned to the tablet ({Guids.Count}/{Max}).";
        }
    }

    /// <summary>Marker + screen references on a spawned dispatcher tablet.</summary>
    public class TabletMarker : MonoBehaviour
    {
        public TextMeshPro?[] windows = new TextMeshPro?[TabletPins.Max];
        public TextMeshPro? emptyMsg;
        /// <summary>The visual model child, moved into a reading pose while held.</summary>
        public Transform? model;
        /// <summary>The glass — the piece the cursor-mode mouse ray hits.</summary>
        public Transform? screenPiece;
        public Vector3 homeLocalPos;
        public Quaternion homeLocalRot;
        public int hoverCol = -1;
        public int hoverRow = -1;
    }

    public static class Tablet
    {
        private static GameObject? host;

        public static void Attach()
        {
            if (host != null)
                return;
            host = new GameObject("AutopilotTablet");
            GameObject.DontDestroyOnLoad(host);
            host.AddComponent<TabletManager>();
        }

        public static void Detach()
        {
            if (host != null)
            {
                GameObject.Destroy(host);
                host = null;
            }
        }
    }

    public class TabletManager : MonoBehaviour
    {
        private const float RepaintSeconds = 0.25f;
        /// <summary>Rows in a window, top to bottom. 0 header, 1..7 selectable,
        /// 8 status. Mouse maps the glass to this grid.</summary>
        private const int Rows = 9;
        private static readonly string[] ArriveNames = { "auto", "shove", "pull" };

        /// <summary>Per-loco command drafts, keyed by car guid — they survive
        /// unpinning and tablet respawns within a session.</summary>
        private sealed class Draft
        {
            public bool shunt;
            public int destDrive;
            public int destShunt;
            public bool couple;
            public bool uncouple;
            public int arrive;
            public string lastResult = "";
            public float lastResultUntil;
        }

        private static readonly Dictionary<string, Draft> drafts =
            new Dictionary<string, Draft>();

        private readonly List<TabletMarker> tablets = new List<TabletMarker>();
        private float nextRepaint;
        private float nextScan;
        private List<string> yards = new List<string>();
        private bool sweptStale;
        private static TMP_FontAsset? font;

        public void Update()
        {
            if (!WorldStreamingInit.IsLoaded)
                return;
            if (!sweptStale)
            {
                sweptStale = true;
                SweepStaleTablets();
            }
            if (Input.GetKeyDown(Main.settings.tabletKey))
                GiveTablet();

            if (Cursor.visible)
                HandleMouse();
            else
                foreach (var t in tablets)
                    if (t != null)
                        t.hoverCol = t.hoverRow = -1;

            if (Time.unscaledTime >= nextRepaint)
            {
                nextRepaint = Time.unscaledTime + RepaintSeconds;
                RefreshData();
                foreach (var t in tablets)
                    if (t != null)
                        Repaint(t);
            }

            foreach (var t in tablets)
                if (t != null)
                    ApplyPose(t);
        }

        private static Draft DraftOf(string guid)
        {
            if (!drafts.TryGetValue(guid, out var d))
                drafts[guid] = d = new Draft();
            return d;
        }

        /* ------------------------------------------------------------ pose -- */

        private void ApplyPose(TabletMarker t)
        {
            if (t.model == null)
                return;
            var cam = Camera.main;
            bool held = HeldTablet() == t;
            if (held && cam != null)
            {
                // Looking ahead: tucked low. Looking down at it: close reading
                // pose covering about half the view. Continuous blend.
                float down = Mathf.Clamp01((-cam.transform.forward.y - 0.05f) / 0.30f);
                float dist = Mathf.Lerp(0.26f, 0.11f, down);
                float drop = Mathf.Lerp(0.09f, 0.01f, down);
                var pos = cam.transform.position
                    + cam.transform.forward * dist
                    - cam.transform.up * drop;
                var rot = Quaternion.LookRotation(cam.transform.up, -cam.transform.forward);
                t.model.SetPositionAndRotation(pos, rot);
            }
            else if (t.model.localPosition != t.homeLocalPos)
            {
                t.model.localPosition = t.homeLocalPos;
                t.model.localRotation = t.homeLocalRot;
            }
        }

        /* ------------------------------------------------------------ data -- */

        private void RefreshData()
        {
            if (Time.unscaledTime < nextScan)
                return;
            nextScan = Time.unscaledTime + 2f;
            if (yards.Count == 0)
                yards = Yards.All().Keys.OrderBy(k => k).ToList();
            // Locos that no longer exist fall off the board.
            var registry = DV.Utils.SingletonBehaviour<TrainCarRegistry>.Instance;
            if (registry != null)
                TabletPins.Guids.RemoveAll(g => LocoOf(g) == null);
        }

        private static TrainCar? LocoOf(string guid) =>
            DV.Utils.SingletonBehaviour<TrainCarRegistry>.Instance?
                .GetTrainCarByCarGuid(guid);

        private List<string> ShuntTracks(string guid)
        {
            var yard = Yards.YardOf(guid);
            if (yard == null || !Yards.All().TryGetValue(yard, out var info))
                return new List<string>();
            return info.tracks.ToList();
        }

        /* ----------------------------------------------------------- mouse -- */

        private void HandleMouse()
        {
            var cam = Camera.main;
            if (cam == null)
                return;
            var ray = cam.ScreenPointToRay(Input.mousePosition);
            if (!Physics.Raycast(ray, out var hit, 3f, ~0, QueryTriggerInteraction.Collide))
                return;
            var t = hit.collider != null ? hit.collider.GetComponentInParent<TabletMarker>() : null;
            if (t == null || t.screenPiece == null)
                return;
            var local = t.screenPiece.InverseTransformPoint(hit.point);
            int col = Mathf.Clamp((int)((local.x + 0.5f) * TabletPins.Max), 0, TabletPins.Max - 1);
            int row = Mathf.Clamp((int)((0.5f - local.z) * Rows), 0, Rows - 1);
            if (t.hoverCol != col || t.hoverRow != row)
            {
                t.hoverCol = col;
                t.hoverRow = row;
                Repaint(t);
            }

            if (col >= TabletPins.Guids.Count)
                return;
            string guid = TabletPins.Guids[col];
            int wheel = 0;
            if (Input.mouseScrollDelta.y > 0.01f) wheel = -1;
            else if (Input.mouseScrollDelta.y < -0.01f) wheel = 1;
            bool click = Input.GetMouseButtonDown(0);
            if (click || wheel != 0)
            {
                Act(guid, row, click, wheel);
                Repaint(t);
            }
        }

        /// <summary>One interaction on a window row: click toggles/cycles/fires,
        /// the wheel cycles values in either direction.</summary>
        private void Act(string guid, int row, bool click, int wheel)
        {
            var d = DraftOf(guid);
            int step = wheel != 0 ? wheel : click ? 1 : 0;
            switch (row)
            {
                case 1:
                    if (click)
                        d.shunt = !d.shunt;
                    break;
                case 2:
                    if (d.shunt)
                    {
                        var tracks = ShuntTracks(guid);
                        if (tracks.Count > 0)
                            d.destShunt = Wrap(d.destShunt + step, tracks.Count);
                    }
                    else if (yards.Count > 0)
                    {
                        d.destDrive = Wrap(d.destDrive + step, yards.Count);
                    }
                    break;
                case 3:
                    if (click)
                        d.couple = !d.couple;
                    break;
                case 4:
                    if (click)
                        d.uncouple = !d.uncouple;
                    break;
                case 5:
                    d.arrive = Wrap(d.arrive + step, ArriveNames.Length);
                    break;
                case 6:
                    if (click)
                        Dispatch(guid, d);
                    break;
                case 7:
                    if (click)
                    {
                        Autopilot.Stop(guid);
                        Flash(d, "STOPPED");
                    }
                    break;
            }
        }

        private void Dispatch(string guid, Draft d)
        {
            var arrive = d.arrive == 1 ? ArriveMode.Shove
                : d.arrive == 2 ? ArriveMode.Pull : ArriveMode.Auto;
            string result;
            if (d.shunt)
            {
                var tracks = ShuntTracks(guid);
                if (tracks.Count == 0)
                {
                    Flash(d, "no yard here");
                    return;
                }
                string dest = tracks[Mathf.Clamp(d.destShunt, 0, tracks.Count - 1)];
                result = Autopilot.Start(guid, dest, 0f, d.couple, arrive, null,
                    AutopilotMode.Shunt, Yards.YardOf(guid), d.uncouple);
            }
            else
            {
                if (yards.Count == 0)
                {
                    Flash(d, "no yards known");
                    return;
                }
                string yard = yards[Mathf.Clamp(d.destDrive, 0, yards.Count - 1)];
                result = Autopilot.Start(guid, "", Main.settings.tabletCruiseKmh,
                    false, arrive, null, AutopilotMode.Drive, yard, false);
            }
            Flash(d, result.Contains("\"error\"") ? "REFUSED" : "DISPATCHED");
        }

        private static void Flash(Draft d, string message)
        {
            d.lastResult = message;
            d.lastResultUntil = Time.unscaledTime + 4f;
        }

        private static int Wrap(int v, int count)
        {
            while (v < 0) v += count;
            while (v >= count) v -= count;
            return v;
        }

        /* ---------------------------------------------------------- render -- */

        private void Repaint(TabletMarker t)
        {
            bool empty = TabletPins.Guids.Count == 0;
            if (t.emptyMsg != null)
                t.emptyMsg.text = empty
                    ? "Select a locomotive using the\nComms Radio -> Autopilot" : "";
            for (int i = 0; i < TabletPins.Max; i++)
            {
                var window = t.windows[i];
                if (window == null)
                    continue;
                window.text = i < TabletPins.Guids.Count
                    ? RenderWindow(TabletPins.Guids[i], t.hoverCol == i ? t.hoverRow : -1)
                    : "";
            }
        }

        private string RenderWindow(string guid, int hoverRow)
        {
            var loco = LocoOf(guid);
            if (loco == null)
                return "";
            var d = DraftOf(guid);
            string state = Autopilot.RunStateOf(guid) ?? "idle";
            string dest;
            if (d.shunt)
            {
                var tracks = ShuntTracks(guid);
                dest = tracks.Count > 0
                    ? tracks[Mathf.Clamp(d.destShunt, 0, tracks.Count - 1)] : "-";
            }
            else
            {
                dest = yards.Count > 0
                    ? yards[Mathf.Clamp(d.destDrive, 0, yards.Count - 1)] : "-";
            }

            var rows = new string[Rows];
            rows[0] = $"<b>{loco.ID}</b>";
            rows[1] = $"Mode: {(d.shunt ? "Shunt" : "Drive")}";
            rows[2] = $"Dest: {dest}";
            rows[3] = $"Couple: {(d.couple ? "yes" : "no")}";
            rows[4] = $"Uncpl: {(d.uncouple ? "yes" : "no")}";
            rows[5] = $"Arrive: {ArriveNames[d.arrive]}";
            rows[6] = "GO";
            rows[7] = "STOP";
            rows[8] = Time.unscaledTime < d.lastResultUntil
                ? $"<color=#7fff7f>{d.lastResult}</color>"
                : $"<color=#9fb7c7>{state}</color>";

            for (int i = 0; i < Rows; i++)
                if (i == hoverRow && i >= 1 && i <= 7)
                    rows[i] = $"<color=#ffa500>{rows[i]}</color>";
            return string.Join("\n", rows);
        }

        /* ------------------------------------------------------------ item -- */

        private TabletMarker? HeldTablet()
        {
            var inv = Inventory.Instance;
            if (inv == null)
                return null;
            var held = inv.GetEquippedItemAtSlot(0);
            return held == null ? null : held.GetComponent<TabletMarker>();
        }

        private void SweepStaleTablets()
        {
            var inv = Inventory.Instance;
            foreach (var spec in Resources.FindObjectsOfTypeAll<InventoryItemSpec>())
            {
                if (spec == null || !spec.gameObject.scene.IsValid())
                    continue;
                if (!spec.gameObject.name.StartsWith("AutopilotDispatcherTablet"))
                    continue;
                if (spec.GetComponent<TabletMarker>() != null)
                    continue; // this assembly's live tablet
                spec.isEssential = false;
                spec.ImmuneToDumpster = false;
                if (inv != null)
                    inv.DestroyItem(spec.gameObject);
                else
                    UnityEngine.Object.Destroy(spec.gameObject);
                Main.mod?.Logger.Log("Removed a stale dispatcher tablet");
            }
        }

        private static TMP_FontAsset? Font()
        {
            if (font == null)
                font = Resources.FindObjectsOfTypeAll<TMP_FontAsset>().FirstOrDefault();
            return font;
        }

        private void GiveTablet()
        {
            if (tablets.Any(m => m != null))
            {
                Main.mod?.Logger.Log("A dispatcher tablet already exists");
                return;
            }
            var inv = Inventory.Instance;
            if (inv == null)
                return;
            int slot = inv.GetFirstFreeHotbarSlot();
            if (slot < 0)
                slot = inv.GetFirstFreeBackpackSlot();
            if (slot < 0)
            {
                Main.mod?.Logger.Log("No free inventory slot for the tablet");
                return;
            }

            // The DVBonusTimer recipe: clone the compass — a proven, holdable,
            // stowable item — hide its face, and draw ours where the dial was.
            var spec = Resources.FindObjectsOfTypeAll<InventoryItemSpec>()
                .FirstOrDefault(s => s != null
                    && (s.LocalizationKey == "item/compass" || s.ItemPrefabName == "Compass"));
            if (spec == null)
            {
                Main.mod?.Logger.Log("Compass item spec not found — no tablet");
                return;
            }
            var prefab = Resources.Load(spec.ItemPrefabName) as GameObject;
            if (prefab == null)
            {
                Main.mod?.Logger.Log("Compass prefab not found — no tablet");
                return;
            }

            var go = UnityEngine.Object.Instantiate(prefab);
            go.name = "AutopilotDispatcherTablet";
            var itemSpec = go.GetComponent<InventoryItemSpec>();
            if (itemSpec != null)
            {
                // BelongsToPlayer keeps the world's junk cleanup off it; NOT
                // essential (the compass is, and the flag padlocked the slot).
                itemSpec.BelongsToPlayer = true;
                itemSpec.NameOverride = "Dispatcher Tablet";
                itemSpec.isEssential = false;
            }

            var model = go.transform.Find("model");
            var dial = model != null ? model.Find("CompassDial") : null;
            if (model == null || dial == null)
            {
                Main.mod?.Logger.Log("Compass model/dial not found — no tablet");
                UnityEngine.Object.Destroy(go);
                return;
            }
            int layer = dial.gameObject.layer;
            foreach (var rend in model.GetComponentsInChildren<Renderer>(true))
                rend.enabled = false;

            var pl = model.lossyScale;
            // Landscape, sized like the job booklet (~21 x 15 cm).
            const float S = 1.45f;
            Vector3 World(float x, float y, float z) => new Vector3(
                x * S / Mathf.Max(0.0001f, pl.x),
                y * S / Mathf.Max(0.0001f, pl.y),
                z * S / Mathf.Max(0.0001f, pl.z));
            GameObject Piece(string name, Vector3 size, Vector3 worldOffset, Color color,
                Color? glow = null, bool keepCollider = false,
                PrimitiveType shape = PrimitiveType.Cube)
            {
                var piece = GameObject.CreatePrimitive(shape);
                piece.name = name;
                if (!keepCollider)
                    UnityEngine.Object.Destroy(piece.GetComponent<Collider>());
                piece.transform.SetParent(model, true);
                piece.layer = layer;
                piece.transform.rotation = dial.rotation;
                piece.transform.position = dial.position
                    + dial.right * (worldOffset.x * S) + dial.up * worldOffset.y
                    + dial.forward * (worldOffset.z * S);
                piece.transform.localScale = size;
                var m = new Material(Shader.Find("Standard")) { color = color };
                if (glow != null)
                {
                    m.EnableKeyword("_EMISSION");
                    m.SetColor("_EmissionColor", glow.Value);
                }
                piece.GetComponent<Renderer>().material = m;
                return piece;
            }

            // Rounded body: a cross of two slabs plus quarter cylinders.
            var bodyColor = new Color(0.13f, 0.13f, 0.14f);
            const float cr = 0.009f;
            Piece("TabletBodyA", World(0.145f, 0.004f, 0.105f - 2 * cr), Vector3.zero, bodyColor);
            Piece("TabletBodyB", World(0.145f - 2 * cr, 0.004f, 0.105f), Vector3.zero, bodyColor);
            foreach (var sx in new[] { -1f, 1f })
                foreach (var sz in new[] { -1f, 1f })
                    Piece("TabletCorner", World(2 * cr, 0.002f, 2 * cr),
                        new Vector3(sx * (0.145f / 2 - cr), 0f, sz * (0.105f / 2 - cr)),
                        bodyColor, shape: PrimitiveType.Cylinder);
            var screenPiece = Piece("TabletScreen",
                World(0.118f, 0.002f, 0.082f), new Vector3(0, 0.0022f, 0f),
                new Color(0.02f, 0.03f, 0.05f), new Color(0.02f, 0.05f, 0.09f),
                keepCollider: true);
            // Trigger, not solid — a solid collider 26 cm from the camera shoves
            // the player backwards whenever they glance at the screen.
            screenPiece.GetComponent<Collider>().isTrigger = true;
            Piece("TabletSpeaker", World(0.03f, 0.0015f, 0.003f), new Vector3(0, 0.0022f, 0.0465f),
                new Color(0.05f, 0.05f, 0.05f));
            Piece("TabletHome", World(0.008f, 0.0015f, 0.008f), new Vector3(0, 0.0022f, -0.0465f),
                new Color(0.32f, 0.33f, 0.35f));

            var marker = go.AddComponent<TabletMarker>();
            marker.model = model;
            marker.screenPiece = screenPiece.transform;
            marker.homeLocalPos = model.localPosition;
            marker.homeLocalRot = model.localRotation;

            // Four window texts across the glass plus the empty-board message.
            // TMP, not TextMesh: TMP's shader tests depth, so the text no longer
            // shines through walls.
            TextMeshPro MakeText(string name, float worldX, float worldWidth,
                float fontSize, TextAlignmentOptions align)
            {
                var textGo = new GameObject(name);
                textGo.transform.SetParent(model, true);
                textGo.layer = layer;
                textGo.transform.position = dial.position
                    + dial.up * 0.0035f + dial.right * (worldX * S);
                textGo.transform.rotation = dial.rotation * Quaternion.Euler(90f, 0f, 0f);
                var text = textGo.AddComponent<TextMeshPro>();
                text.font = Font();
                text.fontSize = fontSize;
                text.alignment = align;
                text.richText = true;
                text.color = Color.white;
                text.enableWordWrapping = true;
                // Hard clip at the rect: overflow was billboarding half a
                // sentence across the night sky.
                text.overflowMode = TextOverflowModes.Truncate;
                text.rectTransform.sizeDelta = new Vector2(worldWidth * S, 0.082f * S);
                // World-unit scale regardless of the item's own scaling.
                textGo.transform.localScale = new Vector3(
                    1f / Mathf.Max(0.0001f, pl.x),
                    1f / Mathf.Max(0.0001f, pl.y),
                    1f / Mathf.Max(0.0001f, pl.z));
                return text;
            }

            const float colWidth = 0.118f / TabletPins.Max;
            for (int i = 0; i < TabletPins.Max; i++)
            {
                float x = (i - (TabletPins.Max - 1) / 2f) * colWidth;
                marker.windows[i] = MakeText($"TabletWindow{i}", x, colWidth - 0.004f,
                    0.012f, TextAlignmentOptions.Top);
            }
            marker.emptyMsg = MakeText("TabletEmpty", 0f, 0.112f, 0.04f,
                TextAlignmentOptions.Center);

            tablets.Add(marker);
            int result = inv.AddItemToInventory(go, slot, false);
            Main.mod?.Logger.Log($"Dispatcher tablet added to slot {result}");
        }
    }
}
