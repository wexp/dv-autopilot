using DV.InventorySystem;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// Marker + per-device UI state on a spawned dispatcher tablet. The item is
    /// a gutted compass (the DVBonusTimer recipe: clone a vanilla item, hide its
    /// visuals, draw our own face) whose screen is a TextMesh the manager
    /// repaints. All the interaction state lives here so two tablets don't
    /// fight over one cursor.
    /// </summary>
    public class TabletMarker : MonoBehaviour
    {
        public TextMesh? screen;
        /// <summary>The visual model child, moved into a reading pose while held.</summary>
        public Transform? model;
        /// <summary>The glass — the piece the cursor-mode mouse ray hits.</summary>
        public Transform? screenPiece;
        /// <summary>Rendered line → selection index (-1 = not selectable), refreshed
        /// on every repaint so mouse rows always match what is on screen.</summary>
        public int[] lineMap = System.Array.Empty<int>();
        public Vector3 homeLocalPos;
        public Quaternion homeLocalRot;
        /// <summary>Which page the tablet shows: 0 = loco list, 1 = one loco.</summary>
        public int page;
        public int locoIndex;
        public int actionIndex;
        public int destIndex;
        public string chosenLoco = "";
        public string lastResult = "";
    }

    /// <summary>
    /// The dispatcher tablet: a carryable item that lists every controllable
    /// locomotive with its autopilot state, and can send one on a road run to a
    /// yard or stop it. Interaction while holding it (or looking at it):
    /// Shift + mouse wheel moves the cursor, the confirm key activates, the
    /// back key goes up a page.
    /// </summary>
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
        private const int RowsPerPage = 6;

        private readonly List<TabletMarker> tablets = new List<TabletMarker>();
        private float nextRepaint;
        private float nextScan;
        private List<TrainCar> locos = new List<TrainCar>();
        private List<string> yards = new List<string>();

        public void Start()
        {
            // A reload may find tablets from the previous assembly's session.
            foreach (var m in Resources.FindObjectsOfTypeAll<TabletMarker>())
                if (m != null && m.gameObject.scene.IsValid())
                    tablets.Add(m);
        }

        public void Update()
        {
            // Nothing here may poke game singletons while the world is still
            // loading — an innocent Inventory.Instance mid-load forces creation
            // at the wrong moment.
            if (!WorldStreamingInit.IsLoaded)
                return;
            var settings = Main.settings;
            if (Input.GetKeyDown(settings.tabletKey))
                GiveTablet();

            var target = HeldTablet() ?? LookedAtTablet();
            if (target != null)
                HandleInput(target);

            // Alt (cursor) mode: point at rows, click to confirm, right-click back.
            if (Cursor.visible)
                HandleMouse();

            if (Time.unscaledTime >= nextRepaint)
            {
                nextRepaint = Time.unscaledTime + RepaintSeconds;
                RefreshData();
                foreach (var t in tablets)
                    if (t != null && t.screen != null)
                        Repaint(t);
            }

            foreach (var t in tablets)
                if (t != null)
                    ApplyPose(t);
        }

        /// <summary>Held tablet floats into a reading pose — centred in view, close
        /// enough to read — instead of dangling at the hand anchor. Only the
        /// visual model moves; the item itself stays in the hand.</summary>
        private void ApplyPose(TabletMarker t)
        {
            if (t.model == null)
                return;
            var cam = Camera.main;
            bool held = HeldTablet() == t;
            if (held && cam != null)
            {
                // Two poses blended on camera pitch: looking AHEAD the tablet
                // tucks low and far so the world stays walkable; looking DOWN at
                // it, it rises to a close reading pose covering about half the
                // view. The blend is continuous, so it follows the glance.
                float down = Mathf.Clamp01((-cam.transform.forward.y - 0.15f) / 0.45f);
                float dist = Mathf.Lerp(0.46f, 0.26f, down);
                float drop = Mathf.Lerp(0.18f, 0.02f, down);
                var pos = cam.transform.position
                    + cam.transform.forward * dist
                    - cam.transform.up * drop;
                // Flat to the view — no recline.
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
            nextScan = Time.unscaledTime + 1f;
            var registry = DV.Utils.SingletonBehaviour<TrainCarRegistry>.Instance;
            if (registry == null)
                return;
            locos = registry.logicCarToTrainCar.Values
                .Where(c => c != null && c.IsLoco
                    && c.GetComponent<DV.RemoteControls.RemoteControllerModule>() != null)
                .OrderBy(c => c.ID)
                .ToList();
            if (yards.Count == 0)
                yards = Yards.All().Keys.OrderBy(k => k).ToList();
        }

        /* ----------------------------------------------------------- input -- */

        private void HandleInput(TabletMarker t)
        {
            bool shift = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
            int dir = 0;
            if (shift)
            {
                float wheel = Input.mouseScrollDelta.y;
                if (wheel > 0.01f) dir = -1;
                else if (wheel < -0.01f) dir = 1;
            }

            if (dir != 0)
            {
                if (t.page == 0 && locos.Count > 0)
                    t.locoIndex = Wrap(t.locoIndex + dir, locos.Count);
                else if (t.page == 1)
                    t.actionIndex = Wrap(t.actionIndex + dir, 4);
                Repaint(t);
            }

            if (Input.GetKeyDown(Main.settings.tabletConfirmKey))
            {
                Confirm(t);
                Repaint(t);
            }
            if (Input.GetKeyDown(Main.settings.tabletBackKey) && t.page == 1)
            {
                t.page = 0;
                t.lastResult = "";
                Repaint(t);
            }
        }

        private void HandleMouse()
        {
            var cam = Camera.main;
            if (cam == null)
                return;
            var ray = cam.ScreenPointToRay(Input.mousePosition);
            if (!Physics.Raycast(ray, out var hit, 3f, ~0, QueryTriggerInteraction.Collide))
                return;
            var t = hit.collider != null ? hit.collider.GetComponentInParent<TabletMarker>() : null;
            if (t == null || t.screenPiece == null || t.lineMap.Length == 0)
                return;
            // Local Z runs along the screen's long axis; +0.5 is one end. Map the
            // hit to a rendered line, then to the selection that line stands for.
            var local = t.screenPiece.InverseTransformPoint(hit.point);
            float f = Mathf.Clamp01(0.5f - local.z);
            int line = Mathf.Clamp((int)(f * t.lineMap.Length), 0, t.lineMap.Length - 1);
            int sel = t.lineMap[line];
            if (sel >= 0)
            {
                bool changed = t.page == 0 ? t.locoIndex != sel : t.actionIndex != sel;
                if (t.page == 0) t.locoIndex = sel; else t.actionIndex = sel;
                if (changed)
                    Repaint(t);
                if (Input.GetMouseButtonDown(0))
                {
                    Confirm(t);
                    Repaint(t);
                }
            }
            if (Input.GetMouseButtonDown(1) && t.page == 1)
            {
                t.page = 0;
                t.lastResult = "";
                Repaint(t);
            }
        }

        private void Confirm(TabletMarker t)
        {
            if (t.page == 0)
            {
                if (locos.Count == 0)
                    return;
                t.locoIndex = Mathf.Clamp(t.locoIndex, 0, locos.Count - 1);
                t.chosenLoco = locos[t.locoIndex].CarGUID;
                t.page = 1;
                t.actionIndex = 0;
                t.lastResult = "";
                return;
            }

            var loco = locos.FirstOrDefault(c => c.CarGUID == t.chosenLoco);
            if (loco == null)
            {
                t.page = 0;
                return;
            }
            switch (t.actionIndex)
            {
                case 0: // cycle destination yard
                    if (yards.Count > 0)
                        t.destIndex = Wrap(t.destIndex + 1, yards.Count);
                    break;
                case 1: // GO — road run to the chosen yard's line
                    if (yards.Count > 0)
                    {
                        string yard = yards[Mathf.Clamp(t.destIndex, 0, yards.Count - 1)];
                        string result = Autopilot.Start(
                            loco.CarGUID, "", Main.settings.tabletCruiseKmh, false,
                            ArriveMode.Auto, null, AutopilotMode.Drive, yard, false);
                        t.lastResult = result.Contains("\"error\"")
                            ? "REFUSED - see console" : "DISPATCHED";
                    }
                    break;
                case 2: // STOP
                    Autopilot.Stop(loco.CarGUID);
                    t.lastResult = "STOPPED";
                    break;
                case 3: // back
                    t.page = 0;
                    t.lastResult = "";
                    break;
            }
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
            if (t.screen == null)
                return;
            t.screen.text = t.page == 0 ? RenderList(t) : RenderLoco(t);
        }

        private string RenderList(TabletMarker t)
        {
            var lines = new List<string> { "<b>DISPATCH</b>" };
            var map = new List<int> { -1 };
            if (locos.Count == 0)
            {
                lines.Add("no locomotives");
                map.Add(-1);
            }
            else
            {
                t.locoIndex = Mathf.Clamp(t.locoIndex, 0, locos.Count - 1);
                int first = Mathf.Max(0, Mathf.Min(t.locoIndex - RowsPerPage / 2,
                    locos.Count - RowsPerPage));
                for (int i = first; i < Mathf.Min(first + RowsPerPage, locos.Count); i++)
                {
                    var c = locos[i];
                    string state = Autopilot.RunStateOf(c.CarGUID) ?? "idle";
                    string row = $"{c.ID}  {state}";
                    lines.Add(i == t.locoIndex ? $"<color=#ffa500>> {row}</color>" : $"  {row}");
                    map.Add(i);
                }
            }
            lines.Add("<size=44>shift+wheel / confirm</size>");
            map.Add(-1);
            t.lineMap = map.ToArray();
            return string.Join("\n", lines);
        }

        private string RenderLoco(TabletMarker t)
        {
            var loco = locos.FirstOrDefault(c => c.CarGUID == t.chosenLoco);
            if (loco == null)
            {
                t.page = 0;
                return RenderList(t);
            }
            string state = Autopilot.RunStateOf(loco.CarGUID) ?? "idle";
            string yard = yards.Count > 0
                ? yards[Mathf.Clamp(t.destIndex, 0, yards.Count - 1)] : "-";
            var rows = new[]
            {
                $"Dest: {yard}",
                "GO  (drive there)",
                "STOP",
                "back",
            };
            var lines = new List<string> { $"<b>{loco.ID}</b>  {state}" };
            var map = new List<int> { -1 };
            for (int i = 0; i < rows.Length; i++)
            {
                lines.Add(i == t.actionIndex
                    ? $"<color=#ffa500>> {rows[i]}</color>" : $"  {rows[i]}");
                map.Add(i);
            }
            if (t.lastResult.Length > 0)
            {
                lines.Add($"<color=#7fff7f>{t.lastResult}</color>");
                map.Add(-1);
            }
            t.lineMap = map.ToArray();
            return string.Join("\n", lines);
        }

        /* ----------------------------------------------------------- item -- */

        private TabletMarker? HeldTablet()
        {
            var inv = Inventory.Instance;
            if (inv == null)
                return null;
            var held = inv.GetEquippedItemAtSlot(0);
            return held == null ? null : held.GetComponent<TabletMarker>();
        }

        private TabletMarker? LookedAtTablet()
        {
            var cam = Camera.main;
            if (cam == null)
                return null;
            var ray = cam.ViewportPointToRay(new Vector3(0.5f, 0.5f, 0f));
            if (!Physics.Raycast(ray, out var hit, 4f, ~0, QueryTriggerInteraction.Ignore))
                return null;
            return hit.collider == null ? null : hit.collider.GetComponentInParent<TabletMarker>();
        }

        private void GiveTablet()
        {
            // One is enough — bring an existing one to notice instead of stacking.
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
                // BelongsToPlayer keeps the world's junk cleanup from despawning
                // a tablet that is put down for a moment.
                itemSpec.BelongsToPlayer = true;
                itemSpec.NameOverride = "Dispatcher Tablet";
                // Deliberately NOT dumpster-immune: the dumpster is how a spare
                // tablet is disposed of, F9 always makes another.
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
            foreach (var r in model.GetComponentsInChildren<Renderer>(true))
                r.enabled = false;

            // Screen slab where the dial sat.
            // ------------------------------------------------- tablet body --
            // All primitives, all sized in WORLD units then divided by the
            // parent's scale (localScale multiplies by it).
            var pl = model.lossyScale;
            // Overall tablet size: 1 was a pocket phone; this matches the job
            // booklet (~15 x 21 cm).
            const float S = 1.45f;
            Vector3 World(float x, float y, float z) => new Vector3(
                x * S / Mathf.Max(0.0001f, pl.x),
                y * S / Mathf.Max(0.0001f, pl.y),
                z * S / Mathf.Max(0.0001f, pl.z));
            GameObject Piece(string name, Vector3 size, Vector3 worldOffset, Color color, Color? glow = null, bool keepCollider = false, PrimitiveType shape = PrimitiveType.Cube)
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

            // Body: dark grey slab, the bezel is simply its margin around the
            // screen. Screen: near-black with a faint glow so it reads at night.
            // Landscape: wide side across the hands, like the booklet held open.
            // Rounded corners: the body is a cross of two slabs with a quarter of
            // a cylinder showing at each corner.
            var bodyColor = new Color(0.13f, 0.13f, 0.14f);
            const float cr = 0.009f;
            Piece("TabletBodyA", World(0.145f, 0.004f, 0.105f - 2 * cr), Vector3.zero, bodyColor);
            Piece("TabletBodyB", World(0.145f - 2 * cr, 0.004f, 0.105f), Vector3.zero, bodyColor);
            foreach (var sx in new[] { -1f, 1f })
                foreach (var sz in new[] { -1f, 1f })
                    Piece("TabletCorner",
                        // A cylinder's height is TWICE its Y scale.
                        World(2 * cr, 0.002f, 2 * cr),
                        new Vector3(sx * (0.145f / 2 - cr), 0f, sz * (0.105f / 2 - cr)),
                        bodyColor, shape: PrimitiveType.Cylinder);
            var screenPiece = Piece("TabletScreen",
                World(0.118f, 0.002f, 0.082f), new Vector3(0, 0.0022f, 0f),
                new Color(0.02f, 0.03f, 0.05f), new Color(0.02f, 0.05f, 0.09f),
                keepCollider: true); // the glass takes the cursor-mode mouse ray
            // Speaker slit above the screen, home button below it.
            Piece("TabletSpeaker", World(0.03f, 0.0015f, 0.003f), new Vector3(0, 0.0022f, 0.0465f),
                new Color(0.05f, 0.05f, 0.05f));
            Piece("TabletHome", World(0.008f, 0.0015f, 0.008f), new Vector3(0, 0.0022f, -0.0465f),
                new Color(0.32f, 0.33f, 0.35f));

            var textGo = new GameObject("TabletText");
            textGo.transform.SetParent(model, true);
            textGo.layer = layer;
            textGo.transform.position = dial.position + dial.up * 0.0025f;
            textGo.transform.rotation = dial.rotation * Quaternion.Euler(90f, 0f, 0f);
            var tm = textGo.AddComponent<TextMesh>();
            tm.anchor = TextAnchor.MiddleCenter;
            tm.alignment = TextAlignment.Center;
            tm.fontSize = 64;
            tm.characterSize = 0.00075f * 1.45f;
            // SetParent(worldPositionStays) already compensated this transform's
            // localScale for the parent — forcing it back to world unit keeps the
            // glyph size authored above true regardless of the item's scale.
            textGo.transform.localScale = new Vector3(
                1f / Mathf.Max(0.0001f, model.lossyScale.x),
                1f / Mathf.Max(0.0001f, model.lossyScale.y),
                1f / Mathf.Max(0.0001f, model.lossyScale.z));
            tm.richText = true;
            tm.color = Color.white;
            tm.text = "<b>DISPATCH</b>\nloading...";

            var marker = go.AddComponent<TabletMarker>();
            marker.screen = tm;
            marker.model = model;
            marker.screenPiece = screenPiece.transform;
            marker.homeLocalPos = model.localPosition;
            marker.homeLocalRot = model.localRotation;
            tablets.Add(marker);

            int result = inv.AddItemToInventory(go, slot, false);
            Main.mod?.Logger.Log($"Dispatcher tablet added to slot {result}");
        }
    }
}
