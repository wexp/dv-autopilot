using DV.InventorySystem;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
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
        public TextMesh?[] windows = new TextMesh?[TabletPins.Max];
        /// <summary>Second column of each quadrant: the VALUES, on their own
        /// plane at a fixed x so the board reads as a two-column table. The
        /// label plane keeps anything that spans the full quadrant width
        /// (header, detail lines, GO/STOP).</summary>
        public TextMesh?[] values = new TextMesh?[TabletPins.Max];
        /// <summary>The booklet icon of each quadrant's loco, top-right of the
        /// header row. The renderer sits on a child of an anchor object placed
        /// at the corner, so fitting the sprite is pure local math.</summary>
        public SpriteRenderer?[] icons = new SpriteRenderer?[TabletPins.Max];
        public TextMesh? emptyMsg;
        /// <summary>The visual model child, moved into a reading pose while held.</summary>
        public Transform? model;
        /// <summary>The glass — the piece the cursor-mode mouse ray hits.</summary>
        public Transform? screenPiece;
        /// <summary>The quadrant divider cross — hidden while nothing is pinned.</summary>
        public GameObject? dividerV;
        public GameObject? dividerH;
        /// <summary>The red panic button: stops every autopilot run.</summary>
        public Transform? homeButton;
        public Renderer? homeRenderer;
        public int hoverCol = -1;
        public int hoverRow = -1;
        /// <summary>Whether the cursor is over the value column rather than the
        /// label one. Only the GO/STOP row cares: they share it.</summary>
        public bool hoverRight;
        /// <summary>Full-screen overlay text (a "page"); empty = normal board.</summary>
        public string overlay = "";
        /// <summary>Destination-picker page: the quadrant it is choosing for,
        /// -1 = the picker is closed and the board shows.</summary>
        public int pickerCol = -1;
        /// <summary>Which row opened the picker — a road move picks a yard on one
        /// row and the road to arrive on from another, so the page has to know
        /// which of the two lists it is showing.</summary>
        public int pickerRow = 2;
        /// <summary>First option shown on the picker page (scroll offset).</summary>
        public int pickerOffset;
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
        /// <summary>Rows in a window, top to bottom. 0 header, 1..8 selectable,
        /// 9 status. Mouse maps the glass to this grid.</summary>
        private const int Rows = 10;
        /// <summary>The row GO and STOP share — GO in the label column, STOP in
        /// the value one. Row 1 is left empty to set the header apart, so the
        /// fields run 2..7 and the flash has the last row to itself.</summary>
        private const int GoRow = 8;
        /// <summary>How much status detail fits across half the glass. Longer
        /// than this and the line runs out over the divider.</summary>
        private const int DetailChars = 28;
        /// <summary>Where the value column starts inside a quadrant, in the same
        /// pre-scale x units the window planes use. Labels get this much width.</summary>
        private const float ValueColX = 0.036f;
        /// <summary>Character size of a window row. Ten rows have to fit inside a
        /// quadrant: at 0.00050 the block stood about a row proud of it, so the
        /// last row — the DISPATCHED/STOPPED flash — printed over the divider on
        /// the top pair and off the glass entirely on the bottom pair. It also
        /// walks the text out of the ten mouse bands the quadrant is divided
        /// into, which is why the lower rows answered a click a row late.</summary>
        private const float RowCharSize = 0.00045f;
        private static readonly string[] LeadNames = { "Auto", "Wagons first", "Loco first" };
        /// <summary>Top-speed field limits and step, as the console's number box.</summary>
        private const int SpeedMin = 5;
        private const int SpeedMax = 120;
        private const int SpeedStep = 5;

        /// <summary>Per-loco command drafts, keyed by car guid — they survive
        /// unpinning and tablet respawns within a session.</summary>
        private sealed class Draft
        {
            public bool shunt;
            public int destDrive;
            /// <summary>Which road to pull up on at the destination yard: an index
            /// into Yards.DriveTargets, 0 = whichever running line is nearest.
            /// The console offers this and the tablet did not, so a road move
            /// could only ever end on the main.</summary>
            public int destDriveTrack;
            /// <summary>Top speed of a road move, km/h — the console's own field.
            /// Shunts are held to yard pace by the server whatever this says.</summary>
            public int speed = Mathf.Clamp(
                Mathf.RoundToInt(Main.settings.tabletCruiseKmh), SpeedMin, SpeedMax);
            public int destShunt;
            public bool couple;
            public bool uncouple;
            public int arrive;
            /// <summary>0 = auto, then the entry junctions of the destination.</summary>
            public int via;
            /// <summary>The quadrant's second screen: live run status instead of
            /// the setup rows. The header row toggles it.</summary>
            public bool showStatus;
            public string lastResult = "";
            public float lastResultUntil;
            /// <summary>Whether that result was a refusal or a failed run —
            /// printed red, and left up longer, because it has to be read.</summary>
            public bool lastResultBad;
        }

        private static readonly Dictionary<string, Draft> drafts =
            new Dictionary<string, Draft>();

        private readonly List<TabletMarker> tablets = new List<TabletMarker>();
        private float nextRepaint;
        private float nextScan;
        private List<string> yards = new List<string>();
        private bool sweptStale;

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
            if (Input.GetKeyDown(KeyCode.F10)
                && (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift)))
                Main.RequestReload();

            if (Cursor.visible)
                HandleMouse();
            else
                foreach (var t in tablets)
                    if (t != null)
                    {
                        t.hoverCol = t.hoverRow = -1;
                        SetOverlay(t, "");
                    }

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
                    ApplyHoldPose(t);
        }

        private static Draft DraftOf(string guid)
        {
            if (!drafts.TryGetValue(guid, out var d))
                drafts[guid] = d = new Draft();
            return d;
        }

        /* ------------------------------------------------------------ pose -- */

        // The screen is built as a child of the host item's own face, so the
        // game's hands carry it, raise it and put it away exactly as they do
        // the map itself. The mod used to pin the model in front of the camera
        // every frame instead, which overrode all of that.
        //
        // The one thing set here is the item's OWN hold pose: the two vectors
        // on its CustomNonVrGrabAnchor, which the hands read. Written every
        // frame so the settings sliders show their effect as they move.

        private static FieldInfo? anchorPos;
        private static FieldInfo? anchorRot;

        private static void ApplyHoldPose(TabletMarker t)
        {
            var anchor = t.GetComponents<MonoBehaviour>()
                .FirstOrDefault(c => c != null
                    && c.GetType().Name == "CustomNonVrGrabAnchor");
            if (anchor == null)
                return;
            var ty = anchor.GetType();
            anchorPos = anchorPos ?? ty.GetField("customLocalPosition");
            anchorRot = anchorRot ?? ty.GetField("customLocalRotation");
            var s = Main.settings;
            // The glance, done in the hold pose rather than over it: looking
            // ahead the tablet rides low and leaned away, and it comes up to
            // the tuned pose as the head goes down. Same blend the old
            // camera-pinned pose used, now in the hands' own frame.
            var cam = Camera.main;
            // How far below the horizon the head is looking, in degrees, so the
            // two thresholds read as angles rather than as a height on the view
            // vector. Fully up can never be at or under the start: that would
            // divide by nothing and snap the tablet up with no travel.
            float pitch = cam == null ? 0f
                : Mathf.Asin(Mathf.Clamp(-cam.transform.forward.y, -1f, 1f)) * Mathf.Rad2Deg;
            float full = Mathf.Max(s.tabletGlanceFull, s.tabletGlanceStart + 1f);
            float down = s.tabletGlance && cam != null
                ? Mathf.Clamp01((pitch - s.tabletGlanceStart) / (full - s.tabletGlanceStart))
                : 1f;
            var pos = new Vector3(s.tabletHoldX,
                s.tabletHoldY - Mathf.Lerp(s.tabletTuckDrop, 0f, down),
                s.tabletHoldZ + Mathf.Lerp(s.tabletTuckPush, 0f, down));
            var rot = new Vector3(
                s.tabletHoldPitch + Mathf.Lerp(s.tabletTuckLean, 0f, down),
                s.tabletHoldYaw, s.tabletHoldRoll);

            // The anchor is read ONCE, by GrabHandlerItem.StartInteraction, which
            // does nothing more than parent the item to the hand's attach point
            // and set these two on its transform. So the fields decide the pose
            // the tablet is picked up in, and after that only the transform moves
            // it — same two values, in the same frame, every frame.
            anchorPos?.SetValue(anchor, pos);
            anchorRot?.SetValue(anchor, rot);
            var tf = t.transform;
            if (tf.parent != null)
            {
                tf.localPosition = pos;
                tf.localRotation = Quaternion.Euler(rot);
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

        /// <summary>Per-track via options — the junction walk plus a JSON round
        /// trip is far too dear to repeat four times a second per window, and
        /// track ends never change within a session.</summary>
        private static readonly Dictionary<string, List<(string label, int? junction)>> viaCache =
            new Dictionary<string, List<(string label, int? junction)>>();

        /// <summary>Via choices for a shunt destination: "Auto" plus the
        /// junction at each end of the track, as the web console offers.</summary>
        private List<(string label, int? junction)> ViaOptions(Draft d, string guid)
        {
            var options = new List<(string, int?)> { ("Auto", (int?)null) };
            if (!d.shunt)
                return options;
            var tracks = ShuntTracks(guid);
            if (tracks.Count == 0)
                return options;
            string dest = tracks[Mathf.Clamp(d.destShunt, 0, tracks.Count - 1)];
            if (viaCache.TryGetValue(dest, out var cached))
                return cached;
            try
            {
                var json = Newtonsoft.Json.Linq.JObject.Parse(
                    Autopilot.GetTrackJunctionsJson(dest));
                foreach (var key in new[] { "in", "out" })
                {
                    var v = json[key];
                    if (v != null && v.Type == Newtonsoft.Json.Linq.JTokenType.Integer)
                        options.Add(($"J-{(int)v}", (int?)(int)v));
                }
            }
            catch
            {
            }
            viaCache[dest] = options;
            return options;
        }

        /* ----------------------------------------------------------- mouse -- */

        private void HandleMouse()
        {
            var cam = Camera.main;
            if (cam == null)
                return;
            var ray = cam.ScreenPointToRay(Input.mousePosition);
            if (!Physics.Raycast(ray, out var hit, 3f, ~0, QueryTriggerInteraction.Collide))
            {
                foreach (var other in tablets)
                    if (other != null)
                        SetOverlay(other, "");
                return;
            }
            var t = hit.collider != null ? hit.collider.GetComponentInParent<TabletMarker>() : null;
            if (t == null || t.screenPiece == null)
                return;
            // The red home button: stop everything. Hover = the game's own
            // crosshair prompt plus a brighter glow, like any interactable.
            bool overHome = t.homeButton != null && hit.collider.transform == t.homeButton;
            if (t.homeRenderer != null)
                t.homeRenderer.material.SetColor("_EmissionColor",
                    overHome ? new Color(0.9f, 0.05f, 0.05f) : new Color(0.25f, 0.02f, 0.02f));
            SetOverlay(t, overHome ? "STOP ALL\nAUTOPILOT RUNS" : "");
            if (overHome)
            {
                if (Input.GetMouseButtonDown(0))
                {
                    Autopilot.Stop(null);
                    foreach (var g in TabletPins.Guids)
                        Flash(DraftOf(g), "ALL STOPPED", true);
                    Repaint(t);
                }
                return;
            }
            var local = t.screenPiece.InverseTransformPoint(hit.point);
            // 2 x 2 windows: quadrant first, then the row inside the quadrant.
            int col = (local.x > 0f ? 1 : 0) + (local.z > 0f ? 0 : 2);
            float zq = local.z > 0f ? local.z : local.z + 0.5f; // within-quadrant, 0..0.5
            int row = Mathf.Clamp((int)((0.5f - zq) / 0.5f * Rows), 0, Rows - 1);
            // Which of the two columns, for the one row that has a live cell in
            // each. The value column starts at ValueColX along a quadrant that
            // is half the glass wide; the split sits just short of it so a click
            // on the word itself counts.
            float xq = local.x > 0f ? local.x : local.x + 0.5f;
            bool right = xq > ValueColX / 0.1315f - 0.05f;
            if (t.hoverCol != col || t.hoverRow != row || t.hoverRight != right)
            {
                t.hoverCol = col;
                t.hoverRow = row;
                t.hoverRight = right;
                Repaint(t);
            }

            int wheel = 0;
            if (Input.mouseScrollDelta.y > 0.01f) wheel = -1;
            else if (Input.mouseScrollDelta.y < -0.01f) wheel = 1;
            bool click = Input.GetMouseButtonDown(0);
            bool rightClick = Input.GetMouseButtonDown(1);

            // Destination-picker page: pick, page-scroll, or back.
            if (t.pickerCol >= 0)
            {
                if (t.pickerCol >= TabletPins.Guids.Count)
                {
                    t.pickerCol = -1;
                    Repaint(t);
                    return;
                }
                string pickerGuid = TabletPins.Guids[t.pickerCol];
                var options = PickerOptions(pickerGuid, t.pickerRow);
                int capacity = TabletPins.Max * Rows - 1;
                if (wheel != 0 && options.Count > capacity)
                {
                    t.pickerOffset = Mathf.Clamp(t.pickerOffset + wheel * capacity,
                        0, Mathf.Max(0, options.Count - capacity));
                    Repaint(t);
                }
                int cell = col * Rows + row;
                if (click)
                {
                    if (cell == 0 || rightClick)
                    {
                        t.pickerCol = -1;
                    }
                    else
                    {
                        int index = t.pickerOffset + cell - 1;
                        if (index >= 0 && index < options.Count)
                        {
                            var pd = DraftOf(pickerGuid);
                            if (pd.shunt) { pd.destShunt = index; pd.via = 0; }
                            else if (t.pickerRow == 3) pd.destDriveTrack = index;
                            else { pd.destDrive = index; pd.destDriveTrack = 0; }
                            t.pickerCol = -1;
                        }
                    }
                    Repaint(t);
                }
                else if (rightClick)
                {
                    t.pickerCol = -1;
                    Repaint(t);
                }
                return;
            }

            if (col >= TabletPins.Guids.Count)
                return;
            string guid = TabletPins.Guids[col];
            var hoverDraft = DraftOf(guid);
            bool status = hoverDraft.showStatus;
            var hints = hoverDraft.shunt ? ShuntHints : DriveHints;
            if (row == GoRow)
                Prompt(t.hoverRight ? "Click: stop this locomotive" : "Click: dispatch");
            else if (!status && row >= 1 && row < hints.Length && hints[row].Length > 0)
                Prompt(hints[row]);
            if (row == 0)
                Prompt("Click: switch between setup and status");
            if (click || wheel != 0)
            {
                Act(t, col, guid, row, click, wheel, t.hoverRight);
                Repaint(t);
            }
        }

        /// <summary>What each row does — fed to the game's crosshair prompt
        /// (the "Press F to enter" line) while the cursor hovers it. One set per
        /// job, because the rows between the mode and GO are not the same fields.</summary>
        private static readonly string[] ShuntHints =
        {
            "",
            "",
            "Click: switch between Driving and Shunting",
            "Click or scroll: choose the track to move to",
            "Click or scroll: which end goes in first",
            "Click or scroll: which junction to arrive through",
            "Click: couple onto the train standing there",
            "Click: uncouple the locomotive when done",
            "",
            "",
        };

        private static readonly string[] DriveHints =
        {
            "",
            "",
            "Click: switch between Driving and Shunting",
            "Click or scroll: choose the destination yard",
            "Click or scroll: which road to pull up on there",
            "Scroll: top speed",
            "",
            "",
            "",
            "",
        };

        /// <summary>The game's own crosshair interaction text; it clears itself
        /// 0.1 s after the last call, so hovering keeps feeding it.</summary>
        private static void Prompt(string text)
        {
            var controller = DV.Utils.SingletonBehaviour<InteractionTextControllerNonVr>.Instance;
            if (controller != null)
                controller.DisplayText(text);
        }

        /// <summary>Swap the whole glass to (or from) an overlay page.</summary>
        private void SetOverlay(TabletMarker t, string text)
        {
            if (t.overlay == text)
                return;
            t.overlay = text;
            Repaint(t);
        }

        /// <summary>One interaction on a window row: click toggles/cycles/fires,
        /// the wheel cycles values in either direction.</summary>
        private void Act(TabletMarker t, int col, string guid, int row, bool click,
            int wheel, bool right)
        {
            var d = DraftOf(guid);
            int step = wheel != 0 ? wheel : click ? 1 : 0;

            // Header toggles the quadrant between its two screens.
            if (row == 0)
            {
                if (click)
                    d.showStatus = !d.showStatus;
                return;
            }
            // GO and STOP share one row: dispatch on the label side, stop on the
            // value side. It is the only live row on the status screen.
            if (row == GoRow)
            {
                if (!click)
                    return;
                if (right)
                {
                    Autopilot.Stop(guid);
                    Flash(d, "STOPPED", true);
                    d.showStatus = false; // ...and back to setup on stop
                }
                else
                {
                    Dispatch(guid, d);
                }
                return;
            }
            if (d.showStatus)
                return;

            // Rows 2..7 mean different things in the two jobs, exactly as the two
            // forms of the web console do. Row 1 is the gap under the header.
            switch (row)
            {
                case 2:
                    if (click)
                    {
                        d.shunt = !d.shunt;
                        d.via = 0; // the junction list belongs to the mode's dest
                    }
                    break;
                case 3:
                    if (click)
                    {
                        // The picker page: every destination on one glass instead
                        // of eighty clicks round the wheel.
                        t.pickerCol = col;
                        t.pickerRow = row;
                        t.pickerOffset = 0;
                    }
                    else if (d.shunt)
                    {
                        var tracks = ShuntTracks(guid);
                        if (tracks.Count > 0)
                            d.destShunt = Wrap(d.destShunt + step, tracks.Count);
                        d.via = 0;
                    }
                    else if (yards.Count > 0)
                    {
                        d.destDrive = Wrap(d.destDrive + step, yards.Count);
                        d.destDriveTrack = 0; // roads belong to the yard just left
                    }
                    break;
                case 4:
                    if (d.shunt)
                    {
                        d.arrive = Wrap(d.arrive + step, LeadNames.Length);
                    }
                    else if (click)
                    {
                        t.pickerCol = col;
                        t.pickerRow = row;
                        t.pickerOffset = 0;
                    }
                    else
                    {
                        var targets = DriveTargets(DriveYard(d));
                        d.destDriveTrack = Wrap(d.destDriveTrack + step, targets.Count);
                    }
                    break;
                case 5:
                    if (d.shunt)
                    {
                        var options = ViaOptions(d, guid);
                        if (options.Count > 1)
                            d.via = Wrap(d.via + step, options.Count);
                    }
                    else
                    {
                        // A number box, not a wheel of options: it stops at the
                        // ends rather than snapping from 120 back to 5.
                        d.speed = Mathf.Clamp(d.speed + step * SpeedStep, SpeedMin, SpeedMax);
                    }
                    break;
                case 6:
                    if (d.shunt && click)
                        d.couple = !d.couple;
                    break;
                case 7:
                    if (d.shunt && click)
                        d.uncouple = !d.uncouple;
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
                var options = ViaOptions(d, guid);
                int? via = options[Mathf.Clamp(d.via, 0, options.Count - 1)].junction;
                result = Autopilot.Start(guid, dest, 0f, d.couple, arrive, via,
                    AutopilotMode.Shunt, Yards.YardOf(guid), d.uncouple);
            }
            else
            {
                if (yards.Count == 0)
                {
                    Flash(d, "no yards known");
                    return;
                }
                string yard = DriveYard(d);
                // An empty destination lets the server pick the nearest running
                // line of that yard; a chosen road is sent as the destination.
                result = Autopilot.Start(guid, DriveTarget(d).id, d.speed,
                    false, arrive, null, AutopilotMode.Drive, yard, false);
            }
            bool refused = result.Contains("\"error\"");
            if (refused)
            {
                string why = ErrorReason(result);
                Flash(d, "REFUSED: " + (why.Length > 40 ? why.Substring(0, 40) : why), true);
            }
            else
            {
                Flash(d, "DISPATCHED");
            }
            // A dispatched quadrant flips to its status screen by itself...
            if (!refused)
                d.showStatus = true;
        }

        private static void Flash(Draft d, string message, bool bad = false)
        {
            d.lastResult = message;
            d.lastResultBad = bad;
            // A refusal says WHY, and that takes longer to read than a word.
            d.lastResultUntil = Time.unscaledTime + (bad ? 8f : 4f);
        }

        /// <summary>The flash line: green for what went through, red for what
        /// did not.</summary>
        private static string ResultLine(Draft d) =>
            Time.unscaledTime < d.lastResultUntil
                ? $"<color={(d.lastResultBad ? "#e60d0d" : "#7fff7f")}>{d.lastResult}</color>"
                : "";

        /// <summary>The reason out of a server refusal — {"error":"why"} — so
        /// the tablet can print WHAT was wrong instead of only that something
        /// was. Anything unparseable falls back to the whole reply.</summary>
        private static string ErrorReason(string result)
        {
            const string key = "\"error\"";
            int at = result.IndexOf(key, StringComparison.Ordinal);
            if (at < 0)
                return result;
            int open = result.IndexOf('"', at + key.Length + 1);
            int close = open < 0 ? -1 : result.IndexOf('"', open + 1);
            return open < 0 || close < 0
                ? result : result.Substring(open + 1, close - open - 1);
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
            bool overlayActive = t.overlay.Length > 0;
            bool empty = TabletPins.Guids.Count == 0 || overlayActive;
            // The destination-picker page owns the whole glass.
            bool picker = !overlayActive && t.pickerCol >= 0
                && t.pickerCol < TabletPins.Guids.Count;
            // The divider cross marks the four locomotive windows, and a list of
            // yards or tracks is one page rather than four windows — drawing it
            // over the list only suggests a split that is not there.
            bool grid = !empty && !picker;
            if (t.dividerV != null && t.dividerV.activeSelf != grid)
                t.dividerV.SetActive(grid);
            if (t.dividerH != null && t.dividerH.activeSelf != grid)
                t.dividerH.SetActive(grid);
            if (t.emptyMsg != null)
                SetText(t.emptyMsg, overlayActive
                    ? $"<color=#ff2020><b>{t.overlay}</b></color>"
                    : empty
                        ? "Select a locomotive using the\nComms Radio -> Autopilot" : "");

            for (int i = 0; i < TabletPins.Max; i++)
            {
                var window = t.windows[i];
                if (window == null)
                    continue;
                var value = t.values[i];
                if (picker)
                {
                    // The picker page is a plain list — no value column.
                    SetText(window, RenderPickerColumn(t, i));
                    if (value != null)
                        SetText(value, "");
                }
                else if (!overlayActive && i < TabletPins.Guids.Count)
                {
                    var (labels, vals) = RenderWindow(
                        TabletPins.Guids[i], t.hoverCol == i ? t.hoverRow : -1,
                        t.hoverRight);
                    SetText(window, labels);
                    if (value != null)
                        SetText(value, vals);
                }
                else
                {
                    SetText(window, "");
                    if (value != null)
                        SetText(value, "");
                }
            }

            // The booklet icon beside each header: the livery's own sprite,
            // fitted to header-row height against the top-right anchor.
            for (int i = 0; i < TabletPins.Max; i++)
            {
                var icon = t.icons[i];
                if (icon == null)
                    continue;
                Sprite? sprite = null;
                if (!picker && !overlayActive && i < TabletPins.Guids.Count)
                    sprite = LocoOf(TabletPins.Guids[i])?.carLivery?.icon;
                if (sprite == null)
                {
                    if (icon.enabled)
                        icon.enabled = false;
                    continue;
                }
                if (icon.sprite != sprite)
                {
                    icon.sprite = sprite;
                    // Fit: header-row height, right and top edges on the anchor.
                    const float IconH = 0.005f;
                    var b = sprite.bounds;
                    float k = IconH / Mathf.Max(0.0001f, b.size.y);
                    icon.transform.localScale = Vector3.one * k;
                    icon.transform.localPosition = new Vector3(
                        -k * (b.center.x + b.extents.x),
                        -k * (b.center.y + b.extents.y), 0f);
                }
                if (!icon.enabled)
                    icon.enabled = true;
            }
        }

        /// <summary>Assign only on change: every TextMesh.text set rebuilds the
        /// glyph mesh, changed or not, and doing that five times a screen four
        /// times a second stuttered the game.</summary>
        private static void SetText(TextMesh tm, string value)
        {
            if (tm.text != value)
                tm.text = value;
        }

        /// <summary>Where a road move may pull up in the chosen yard, cached per
        /// yard: the list is built by walking the yard's roads and never changes
        /// within a session, and the picker asks for it four times a second.</summary>
        private static readonly Dictionary<string, List<(string id, string label)>> driveTargetCache =
            new Dictionary<string, List<(string id, string label)>>();

        private static List<(string id, string label)> DriveTargets(string? yard)
        {
            if (string.IsNullOrEmpty(yard))
                return Yards.DriveTargets(null);
            if (!driveTargetCache.TryGetValue(yard!, out var list))
                driveTargetCache[yard!] = list = Yards.DriveTargets(yard);
            return list;
        }

        /// <summary>The destination yard of a road move.</summary>
        private string DriveYard(Draft d) =>
            yards.Count > 0 ? yards[Mathf.Clamp(d.destDrive, 0, yards.Count - 1)] : "";

        /// <summary>The road it is to pull up on there — ("", nearest) unless one
        /// was picked.</summary>
        private (string id, string label) DriveTarget(Draft d)
        {
            var targets = DriveTargets(DriveYard(d));
            return targets[Mathf.Clamp(d.destDriveTrack, 0, targets.Count - 1)];
        }

        /// <summary>Every option the full-glass picker offers, which depends on the
        /// row it was opened from: destination yards and the roads within one for a
        /// road move, the yard's tracks for a shunt.</summary>
        private List<string> PickerOptions(string guid, int row)
        {
            var d = DraftOf(guid);
            if (d.shunt)
                return ShuntTracks(guid);
            return row == 3
                ? DriveTargets(DriveYard(d)).Select(o => o.label).ToList()
                : yards;
        }

        /// <summary>One column of the full-glass destination list. Cell 0 of the
        /// first column is BACK; options fill column by column.</summary>
        private string RenderPickerColumn(TabletMarker t, int col)
        {
            string guid = TabletPins.Guids[t.pickerCol];
            var d = DraftOf(guid);
            var options = PickerOptions(guid, t.pickerRow);
            int chosen = d.shunt ? d.destShunt
                : t.pickerRow == 3 ? d.destDriveTrack : d.destDrive;
            var rows = new string[Rows];
            for (int row = 0; row < Rows; row++)
            {
                int cell = col * Rows + row;
                string label;
                if (cell == 0)
                {
                    label = "< BACK";
                }
                else
                {
                    int index = t.pickerOffset + cell - 1;
                    label = index >= 0 && index < options.Count ? options[index] : "";
                    if (label.Length > 0 && index == chosen)
                        label = $"<b>{label}</b>";
                }
                if (label.Length > 0 && t.hoverCol == col && t.hoverRow == row)
                    label = $"<color=#ffa500>{label}</color>";
                rows[row] = label;
            }
            return string.Join("\n", rows);
        }

        /// <summary>What kind of machine it is, as the livery id says it:
        /// "LocoDE2" is a DE2. Anything not named that way is printed as it
        /// comes, and a car with no livery at all gets nothing.</summary>
        private static string LocoType(TrainCar loco)
        {
            string id = loco.carLivery?.id ?? "";
            return id.StartsWith("Loco", StringComparison.Ordinal)
                ? id.Substring(4) : id;
        }

        private (string labels, string values) RenderWindow(
            string guid, int hoverRow, bool hoverRight)
        {
            var loco = LocoOf(guid);
            if (loco == null)
                return ("", "");
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
                // A road move names the yard unless a particular road in it was
                // chosen, which is the more specific answer.
                string yard = DriveYard(d);
                var target = DriveTarget(d);
                dest = yard.Length == 0 ? "-"
                    : target.id.Length == 0 ? yard : target.id;
            }

            // A finished (or vanished) run sends the quadrant back to setup.
            if (d.showStatus && (state == "idle" || state == "arrived"
                || state == "coupled" || state == "error" || state == "stopped"))
            {
                d.showStatus = false;
                if (state != "idle")
                {
                    // An error carries the run's own last word with it; the
                    // state alone does not say what went wrong.
                    string detail = state == "error"
                        ? (Autopilot.RunDetailOf(guid) ?? "") : "";
                    Flash(d, state.ToUpperInvariant()
                            + (detail.Length > 0 ? ": " + detail : ""),
                        state == "error" || state == "stopped");
                }
            }
            // Two-column table: labels on the window plane, values on their own
            // plane at ValueColX so they line up. Full-width text (header,
            // detail lines, GO/STOP, result flash) stays in the label column
            // with an empty value cell.
            var rows = new string[Rows];
            var vals = new string[Rows];
            // Colour is held aside rather than written into the text, so the
            // hover tint can replace it for one row instead of wrapping it —
            // nested colour tags leave the inner one winning, and the hovered
            // row would keep its own colour and look unhovered.
            var rowCol = new string[Rows];
            var valCol = new string[Rows];
            if (d.showStatus)
            {
                // Second screen of the quadrant: what the run is doing right now.
                string detail = Autopilot.RunDetailOf(guid) ?? "";
                float limit = Autopilot.RunLimitOf(guid);
                rows[0] = $"<b>{loco.ID}</b>  <color=#add8e6>[status]</color>";
                vals[0] = LocoType(loco);
                rows[2] = "State";
                vals[2] = state;
                valCol[2] = state == "error" ? "#e60d0d" : "#add8e6";
                rows[3] = detail.Length > DetailChars
                    ? detail.Substring(0, DetailChars) : detail;
                rows[4] = detail.Length > DetailChars
                    ? detail.Substring(DetailChars,
                        Mathf.Min(DetailChars, detail.Length - DetailChars))
                    : "";
                rows[5] = limit > 0 ? "Limit" : "";
                vals[5] = limit > 0 ? $"{limit:0} km/h" : "";
                rows[6] = "Target";
                vals[6] = dest;
                rows[9] = ResultLine(d);
            }
            else if (d.shunt)
            {
                // Shunting, in the console's own order: where to, which end goes
                // in, which junction through, and what to do on arrival.
                var viaOptions = ViaOptions(d, guid);
                rows[0] = $"<b>{loco.ID}</b>  <color=#ffd700>[setup]</color>";
                vals[0] = LocoType(loco);
                rows[2] = "Job";
                vals[2] = "Shunting";
                valCol[2] = "#f08080";
                rows[3] = "To track";
                vals[3] = dest;
                rows[4] = "Arrive";
                vals[4] = LeadNames[d.arrive];
                rows[5] = "Via";
                vals[5] = viaOptions[Mathf.Clamp(d.via, 0, viaOptions.Count - 1)].label;
                rows[6] = "Couple train";
                vals[6] = d.couple ? "Yes" : "No";
                rows[7] = "Uncouple train";
                vals[7] = d.uncouple ? "Yes" : "No";
                // State lives on the status screen now; here only a fresh
                // dispatch/stop result flashes.
                rows[9] = ResultLine(d);
            }
            else
            {
                // A road move: yard, the road to pull up on there, top speed.
                // Coupling, shoving and via belong to yard work and the server
                // drops them for a drive, so they are not offered here either.
                string yard = DriveYard(d);
                rows[0] = $"<b>{loco.ID}</b>  <color=#ffd700>[setup]</color>";
                vals[0] = LocoType(loco);
                rows[2] = "Job";
                vals[2] = "Driving";
                valCol[2] = "#90ee90";
                rows[3] = "To yard";
                vals[3] = yard.Length > 0 ? yard : "-";
                rows[4] = "Arrive on";
                vals[4] = DriveTarget(d).label;
                rows[5] = "Top speed";
                vals[5] = $"{d.speed} km/h";
                rows[9] = ResultLine(d);
            }

            // GO and STOP share the last row before the flash, one per column,
            // and read as what they do on every screen. STOP takes the red of
            // the home button — the other thing on the tablet that stops a run.
            rows[GoRow] = "GO";
            vals[GoRow] = "STOP";
            rowCol[GoRow] = "#3cb371";
            valCol[GoRow] = "#e60d0d";

            for (int i = 0; i < Rows; i++)
            {
                vals[i] ??= "";
                if (i == hoverRow && i >= 1 && i <= GoRow)
                {
                    // The hover tint covers the whole row — both columns, and
                    // it takes the row's own colour with it. Except on the row
                    // the two buttons share, where only the one under the
                    // cursor lights.
                    bool half = i == GoRow;
                    if (!half || !hoverRight)
                        rows[i] = $"<color=#ffa500>{rows[i]}</color>";
                    else if (rowCol[i] != null)
                        rows[i] = $"<color={rowCol[i]}>{rows[i]}</color>";
                    if (vals[i].Length > 0)
                    {
                        if (!half || hoverRight)
                            vals[i] = $"<color=#ffa500>{vals[i]}</color>";
                        else if (valCol[i] != null)
                            vals[i] = $"<color={valCol[i]}>{vals[i]}</color>";
                    }
                    continue;
                }
                if (rowCol[i] != null && rows[i] != null && rows[i].Length > 0)
                    rows[i] = $"<color={rowCol[i]}>{rows[i]}</color>";
                if (valCol[i] != null && vals[i].Length > 0)
                    vals[i] = $"<color={valCol[i]}>{vals[i]}</color>";
            }
            return (string.Join("\n", rows), string.Join("\n", vals));
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
                // The game renames item objects on pickup, so the name test alone
                // let dead tablets from earlier assemblies survive reloads — the
                // frozen boards whose shadows nothing could fix. Match by marker
                // component NAME across assemblies instead.
                bool isTablet = spec.gameObject.name.StartsWith("AutopilotDispatcherTablet")
                    || spec.GetComponents<Component>().Any(
                        c => c != null && c.GetType().Name == nameof(TabletMarker));
                if (!isTablet)
                    continue;
                if (spec.GetComponent<TabletMarker>() != null)
                    continue; // this assembly's live tablet
                spec.isEssential = false;
                spec.ImmuneToDumpster = false;
                if (inv != null)
                    inv.DestroyItem(spec.gameObject);
                else
                    UnityEngine.Object.Destroy(spec.gameObject);
                Main.mod?.Logger.Log("[TABLET] removed a stale dispatcher tablet");
            }
        }

        /// <summary>The inventory-slot icon: a little landscape tablet, drawn
        /// pixel by pixel like the rest of the device. The dropped variant is
        /// the grey ghost the inventory shows while the item lies in the world.</summary>
        private static Sprite TabletIcon(bool dropped)
        {
            const int W = 128, H = 128;

            // Signed-distance coverage of a rounded rectangle, ~1 px antialiased.
            float RoundRect(float px, float py, float cx, float cy,
                float hw, float hh, float r)
            {
                float dx = Mathf.Abs(px - cx) - (hw - r);
                float dy = Mathf.Abs(py - cy) - (hh - r);
                float outside = Mathf.Sqrt(
                    Mathf.Max(dx, 0f) * Mathf.Max(dx, 0f)
                    + Mathf.Max(dy, 0f) * Mathf.Max(dy, 0f));
                float d = outside + Mathf.Min(Mathf.Max(dx, dy), 0f) - r;
                return Mathf.Clamp01(0.5f - d);
            }

            // The real tablet's palette and proportions: body 145 x 105 with
            // 9 mm corners, screen 131.5 x 93.5 with 7 mm corners centred in
            // it, red round home button on the bottom bezel.
            Color body = dropped
                ? new Color(0.55f, 0.55f, 0.55f, 1f)
                : new Color(0.13f, 0.13f, 0.14f, 1f);
            Color screen = dropped
                ? new Color(0.72f, 0.72f, 0.72f, 1f)
                : new Color(0.04f, 0.07f, 0.10f, 1f);
            Color textCol = dropped
                ? new Color(0.50f, 0.50f, 0.50f, 1f)
                : new Color(0.88f, 0.92f, 0.94f, 1f);
            Color button = dropped
                ? new Color(0.45f, 0.45f, 0.45f, 1f)
                : new Color(0.62f, 0.10f, 0.10f, 1f);

            // A 5 x 7 pixel font, just the letters of "Autopilot". Row 0 is
            // the TOP of the glyph.
            var font = new System.Collections.Generic.Dictionary<char, string[]>
            {
                ['A'] = new[] { "01110", "10001", "10001", "11111", "10001", "10001", "10001" },
                ['u'] = new[] { "00000", "00000", "10001", "10001", "10001", "10011", "01101" },
                ['t'] = new[] { "00100", "00100", "11111", "00100", "00100", "00100", "00011" },
                ['o'] = new[] { "00000", "00000", "01110", "10001", "10001", "10001", "01110" },
                ['p'] = new[] { "00000", "00000", "11110", "10001", "11110", "10000", "10000" },
                ['i'] = new[] { "00100", "00000", "01100", "00100", "00100", "00100", "01110" },
                ['l'] = new[] { "01100", "00100", "00100", "00100", "00100", "00100", "01110" },
            };
            const string label = "Autopilot";
            const int glyphScale = 2;
            int labelW = (label.Length * 6 - 1) * glyphScale;
            int labelX = (W - labelW) / 2;
            int labelY = 64 + 7 * glyphScale / 2; // top edge, vertically centred

            bool LabelPixel(int x, int y)
            {
                int lx = x - labelX;
                int ly = labelY - y; // rows count downwards from the top
                if (lx < 0 || ly < 0 || ly >= 7 * glyphScale)
                    return false;
                int slot = lx / (6 * glyphScale);
                if (slot >= label.Length)
                    return false;
                int gx = lx % (6 * glyphScale) / glyphScale;
                if (gx >= 5)
                    return false; // the 1-glyph gap between letters
                return font[label[slot]][ly / glyphScale][gx] == '1';
            }

            var tex = new Texture2D(W, H, TextureFormat.RGBA32, false);
            var pixels = new Color[W * H];
            for (int y = 0; y < H; y++)
            {
                for (int x = 0; x < W; x++)
                {
                    var c = new Color(0f, 0f, 0f, 0f);
                    // Body: 145 x 105 scaled to 124 x 90, corners 9/145 in.
                    float a = RoundRect(x, y, 64f, 64f, 62f, 45f, 8f);
                    if (a > 0f)
                        c = Color.Lerp(c, body, a);
                    // Screen: 131.5 x 93.5 of the body, centred like the glass.
                    a = RoundRect(x, y, 64f, 64f, 56f, 40f, 6f);
                    if (a > 0f)
                        c = Color.Lerp(c, screen, a);
                    if (LabelPixel(x, y))
                        c = Color.Lerp(c, textCol, 1f);
                    // The home button: bottom bezel, centred, tiny and red.
                    a = RoundRect(x, y, 64f, 21.5f, 2.5f, 2.5f, 2.5f);
                    if (a > 0f)
                        c = Color.Lerp(c, button, a);
                    pixels[y * W + x] = c;
                }
            }
            tex.SetPixels(pixels);
            tex.Apply(false, true);
            return Sprite.Create(tex, new Rect(0, 0, W, H),
                new Vector2(0.5f, 0.5f), 128f);
        }

        /// <summary>The host item's parts, indented, meshes marked. Which part
        /// to draw on and which way its axes point can only be read off a live
        /// prefab, so it goes in the log every time a tablet is made.</summary>
        private static string Tree(Transform t, int depth)
        {
            // Components too: what a held item is MADE of is the whole question
            // when deciding whether to clone one or build one.
            string parts = string.Join(", ", t.GetComponents<Component>()
                .Where(c => c != null && !(c is Transform))
                .Select(c => c.GetType().Name));
            string self = "\n" + new string(' ', depth * 2) + t.name
                + (parts.Length > 0 ? "  {" + parts + "}" : "");
            return depth >= 3
                ? self
                : self + string.Concat(t.Cast<Transform>().Select(c => Tree(c, depth + 1)));
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

            // The DVBonusTimer recipe: clone a real item — holdable, stowable,
            // carried by the game's own hands — hide its mesh, and draw ours in
            // its place. The map is the first choice: it is the flat item the
            // game already holds up to be read, which is exactly the pose a
            // tablet wants. The compass is the fallback the recipe was proven on.
            InventoryItemSpec? FindItem(string key, string prefabName) =>
                Resources.FindObjectsOfTypeAll<InventoryItemSpec>()
                    .FirstOrDefault(s => s != null
                        && (s.LocalizationKey == key || s.ItemPrefabName == prefabName));
            var spec = FindItem("item/map", "Map") ?? FindItem("item/compass", "Compass");
            if (spec == null)
            {
                Main.mod?.Logger.Log("No map or compass item spec found — no tablet");
                return;
            }
            Main.mod?.Logger.Log("Tablet host item: " + spec.ItemPrefabName
                + " (" + spec.LocalizationKey + ")");
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
                itemSpec.NameOverride = "Autopilot Tablet";
                itemSpec.isEssential = false;
                // The clone inherits the compass icons — draw our own.
                var icon = TabletIcon(dropped: false);
                itemSpec.ItemIconSprite = icon;
                itemSpec.ItemIconSpriteSimple = icon;
                itemSpec.ItemIconSpriteDropped = TabletIcon(dropped: true);
            }

            // "model" is the compass's own wrapper; other items name it their
            // own way, so the item root stands in when it is missing. The
            // hierarchy goes in the log either way — the part names are only
            // readable from a live prefab, not from the code.
            var model = go.transform.Find("model") ?? go.transform;
            Main.mod?.Logger.Log("Tablet host hierarchy: " + Tree(go.transform, 0));
            // What the screen could be lettered in. A TextMesh takes a legacy
            // UnityEngine.Font and nothing else, so a TMP asset in the list is
            // only a lead — it carries a source font sometimes, an atlas always.
            // dynamic = the glyphs are rasterised from a font FILE ON THIS
            // MACHINE by name; anything else carries its own atlas and ships
            // with the game.
            Main.mod?.Logger.Log("Fonts loaded: " + string.Join(", ",
                Resources.FindObjectsOfTypeAll<Font>()
                    .Select(f => f.name + (f.dynamic ? " (dynamic)" : ""))));
            Main.mod?.Logger.Log("TMP fonts loaded: " + string.Join(", ",
                Resources.FindObjectsOfTypeAll<ScriptableObject>()
                    .Where(o => o.GetType().Name == "TMP_FontAsset")
                    .Select(o => o.name)));

            // Everything below is drawn in the frame of one anchor transform:
            // the flat face of the host the player already reads. On the map
            // that is MapPaper (the sheet the station markers sit on), on the
            // compass the dial. The root only stands in if neither exists, and
            // then its own axes decide which way the screen points.
            Transform? Part(string name) => go.transform
                .GetComponentsInChildren<Transform>(true)
                .FirstOrDefault(x => x.name == name);
            var face = Part("MapPaper") ?? Part("CompassDial") ?? model;

            // Built in the face's own frame, unrotated. Up is the face normal
            // there: the map's paper measures 0.3 x 0 x 0.3 in its own axes, so
            // its sheet lies in X/Z exactly as the compass dial's does.
            var dial = face;
            int layer = face.gameObject.layer;
            foreach (var rend in model.GetComponentsInChildren<Renderer>(true))
                rend.enabled = false;
            // The map keeps drawing station markers of its own — new objects
            // with new renderers, which the sweep above never sees. Nothing
            // left to mark, so the controller goes.
            foreach (var comp in go.GetComponents<MonoBehaviour>())
                if (comp != null && comp.GetType().Name == "MapMarkersController")
                    UnityEngine.Object.Destroy(comp);
            // Where the hands put the item in non-VR, as data on the item —
            // the game's own hold pose knob, and the only one that does not
            // fight the animation. Logged as the map has it, to tune from.
            foreach (var comp in go.GetComponents<MonoBehaviour>())
                if (comp != null && comp.GetType().Name == "CustomNonVrGrabAnchor")
                {
                    var ty = comp.GetType();
                    Main.mod?.Logger.Log("Grab anchor: pos "
                        + ty.GetField("customLocalPosition")?.GetValue(comp)
                        + " rot " + ty.GetField("customLocalRotation")?.GetValue(comp));
                }

            var pl = dial.lossyScale;
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
                piece.transform.SetParent(dial, true);
                piece.layer = layer;
                piece.transform.rotation = dial.rotation;
                piece.transform.position = dial.position
                    + dial.right * (worldOffset.x * S) + dial.up * worldOffset.y
                    + dial.forward * (worldOffset.z * S);
                piece.transform.localScale = size;
                var m = new Material(Shader.Find("Standard")) { color = color };
                // No shadow interaction AT ALL on any piece: with casting or
                // receiving on, some sun angle always paints a seam or band
                // across the glass. A hand-held slab needs no shadow physics.
                var pr = piece.GetComponent<Renderer>();
                pr.motionVectorGenerationMode = MotionVectorGenerationMode.ForceNoMotion;
                pr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                pr.receiveShadows = false;
                // Fully matte. The glossy default reflected the bright glyphs in
                // the glass as a dark offset copy — the "shadow" that survived
                // every shadow fix, because it never was one.
                m.SetFloat("_Glossiness", 0f);
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
            // EMBEDDED in the body (raised glass parallax-slid off the bezel
            // and read as shadows), and rounded like the body: a cross of two
            // glass slabs with quarter cylinders in the corners. The mouse hits
            // an invisible full-rectangle plate so the click grid keeps its
            // simple math.
            var glassColor = new Color(0.02f, 0.03f, 0.05f);
            var glassGlow = new Color(0.02f, 0.05f, 0.09f);
            const float sr = 0.007f; // screen corner radius
            Piece("TabletGlassA", World(0.1315f, 0.0042f, 0.0935f - 2 * sr),
                Vector3.zero, glassColor, glassGlow);
            Piece("TabletGlassB", World(0.1315f - 2 * sr, 0.0042f, 0.0935f),
                Vector3.zero, glassColor, glassGlow);
            foreach (var gx in new[] { -1f, 1f })
                foreach (var gz in new[] { -1f, 1f })
                    Piece("TabletGlassCorner", World(2 * sr, 0.0021f, 2 * sr),
                        new Vector3(gx * (0.1315f / 2 - sr), 0f, gz * (0.0935f / 2 - sr)),
                        glassColor, glassGlow, shape: PrimitiveType.Cylinder);
            var screenPiece = Piece("TabletScreen",
                World(0.1315f, 0.0042f, 0.0935f), Vector3.zero,
                glassColor, keepCollider: true);
            screenPiece.GetComponent<Renderer>().enabled = false;
            // Window dividers: a thin cross splitting the glass into the four
            // quadrants the windows live in.
            var lineColor = new Color(0.16f, 0.20f, 0.26f);
            var dividerV = Piece("TabletDividerV", World(0.0008f, 0.0002f, 0.0935f),
                new Vector3(0f, 0.0031f, 0f), lineColor);
            var dividerH = Piece("TabletDividerH", World(0.1315f, 0.0002f, 0.0008f),
                new Vector3(0f, 0.0031f, 0f), lineColor);
            // Trigger, not solid — a solid collider 26 cm from the camera shoves
            // the player backwards whenever they glance at the screen.
            screenPiece.GetComponent<Collider>().isTrigger = true;
            // The raised glass was casting its outline onto the bezel. The body
            // still casts the tablet's own shadow; the surface details do not.
            var screenRenderer = screenPiece.GetComponent<Renderer>();
            screenRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            screenRenderer.receiveShadows = false;
            // The home button is the panic switch: red, and clicking it in
            // cursor mode stops EVERY autopilot run at once.
            var homeButton = Piece("TabletHome",
                World(0.005f, 0.0004f, 0.005f), new Vector3(0, 0.0031f, -0.0497f),
                new Color(0.62f, 0.10f, 0.10f), new Color(0.25f, 0.02f, 0.02f),
                keepCollider: true, shape: PrimitiveType.Cylinder);
            homeButton.GetComponent<Collider>().isTrigger = true;
            var homeRenderer = homeButton.GetComponent<Renderer>();

            var marker = go.AddComponent<TabletMarker>();
            marker.model = model;
            marker.screenPiece = screenPiece.transform;
            marker.dividerV = dividerV;
            marker.dividerH = dividerH;
            marker.homeButton = homeButton.transform;
            marker.homeRenderer = homeRenderer;

            // Window texts: the DVBonusTimer recipe — a plain TextMesh. It is
            // the one text setup proven clean on a held DV item; every TMP
            // variant grew new artifacts. The lettering is the game's own
            // though: din1451alt is the face on DV's signs, and it is one of
            // the legacy fonts the game keeps loaded, which is the only kind a
            // TextMesh will take. Anything missing falls back down the list,
            // and past that to the built-in Arial.
            var screenFont = Resources.FindObjectsOfTypeAll<Font>();
            Font? font = null;
            foreach (var want in new[] { "RobotoMono-Bold", "RobotoMono-Regular",
                "COURBD_Console_13", "COURBD", "din1451alt", "Roboto-Bold" })
            {
                font = screenFont.FirstOrDefault(f => f != null && f.name == want);
                if (font != null)
                    break;
            }
            Main.mod?.Logger.Log("Tablet lettering: " + (font?.name ?? "built-in"));

            TextMesh MakeText(string name, float worldX, float worldZ,
                float charSize, TextAnchor anchor)
            {
                var textGo = new GameObject(name);
                textGo.transform.SetParent(dial, true);
                textGo.layer = layer;
                textGo.transform.position = dial.position
                    + dial.up * 0.0034f + dial.right * (worldX * S)
                    + dial.forward * (worldZ * S);
                textGo.transform.rotation = dial.rotation * Quaternion.Euler(90f, 0f, 0f);
                // World-unit scale regardless of the item's own scaling.
                textGo.transform.localScale = new Vector3(
                    1f / Mathf.Max(0.0001f, pl.x),
                    1f / Mathf.Max(0.0001f, pl.y),
                    1f / Mathf.Max(0.0001f, pl.z));
                var tm = textGo.AddComponent<TextMesh>();
                if (font != null)
                    tm.font = font;
                tm.anchor = anchor;
                tm.alignment = anchor == TextAnchor.UpperLeft
                    ? TextAlignment.Left : TextAlignment.Center;
                tm.fontSize = 80;
                tm.characterSize = charSize * S;
                tm.richText = true;
                tm.color = Color.white;
                var mr = textGo.GetComponent<MeshRenderer>();
                if (mr != null)
                {
                    // A TextMesh draws with its font's own material — set the
                    // font and leave the default material and the glyphs come
                    // out of the wrong atlas.
                    if (font != null)
                        mr.sharedMaterial = font.material;
                    mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                    mr.receiveShadows = false;
                    mr.motionVectorGenerationMode = MotionVectorGenerationMode.ForceNoMotion;
                }
                return tm;
            }

            // 2 x 2 grid: each window is anchored at the TOP of its quarter.
            const float halfW = 0.1315f / 2f;
            const float halfH = 0.0935f / 2f;
            for (int i = 0; i < TabletPins.Max; i++)
            {
                // Anchored at the top-LEFT of each quadrant.
                float x = (i % 2 == 0 ? -halfW : 0f) + 0.004f;
                float z = (i < 2 ? 1 : -1) * halfH / 2f + halfH / 2f - 0.002f;
                marker.windows[i] = MakeText($"TabletWindow{i}", x, z,
                    RowCharSize, TextAnchor.UpperLeft);
                // The value column: same rows, fixed x — a two-column table.
                marker.values[i] = MakeText($"TabletValues{i}", x + ValueColX, z,
                    RowCharSize, TextAnchor.UpperLeft);

                // The loco's booklet icon at the header row's right end. The
                // anchor sits at the quadrant's top-RIGHT corner in the text
                // plane (local +x right, +y up-screen); the sprite child is
                // fitted against it in Repaint, where the livery is known.
                float xr = (i % 2 == 0 ? 0f : halfW) - 0.004f;
                var anchor = new GameObject($"TabletIconAnchor{i}");
                anchor.transform.SetParent(model, true);
                anchor.layer = layer;
                anchor.transform.position = dial.position
                    + dial.up * 0.0034f + dial.right * (xr * S)
                    + dial.forward * (z * S);
                anchor.transform.rotation = dial.rotation * Quaternion.Euler(90f, 0f, 0f);
                anchor.transform.localScale = new Vector3(
                    1f / Mathf.Max(0.0001f, pl.x),
                    1f / Mathf.Max(0.0001f, pl.y),
                    1f / Mathf.Max(0.0001f, pl.z));
                var iconGo = new GameObject($"TabletIcon{i}");
                iconGo.transform.SetParent(anchor.transform, false);
                iconGo.layer = layer;
                var sprite = iconGo.AddComponent<SpriteRenderer>();
                sprite.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                sprite.receiveShadows = false;
                sprite.motionVectorGenerationMode = MotionVectorGenerationMode.ForceNoMotion;
                sprite.enabled = false;
                marker.icons[i] = sprite;
            }
            marker.emptyMsg = MakeText("TabletEmpty", 0f, 0f,
                0.00075f, TextAnchor.MiddleCenter);

            tablets.Add(marker);
            int result = inv.AddItemToInventory(go, slot, false);
            Main.mod?.Logger.Log($"Dispatcher tablet added to slot {result}");
        }
    }
}
