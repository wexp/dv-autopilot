using DV.Common;
using DV.InventorySystem;
using DV.ThingTypes;
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

    /// <summary>Which page the glass is showing. The tablet opens on the
    /// menu, and the two boards hang off its tiles: the overview is all four
    /// pinned machines at once, Locomotives is one of them at a time.</summary>
    public enum TabletPage
    {
        Menu,
        Overview,
        Locomotives,
    }

    /// <summary>How many cars the consist strip can draw. Longer trains are
    /// cut off at this, which is already longer than a yard road holds.</summary>
    public static class TabletManagerConsist
    {
        public const int Max = 30;
    }

    /// <summary>Marker + screen references on a spawned dispatcher tablet.</summary>
    public class TabletMarker : MonoBehaviour
    {
        /// <summary>How many tiles the home menu has.</summary>
        public const int Tiles = 2;

        public TabletPage page = TabletPage.Menu;
        /// <summary>The home menu's tiles: a rounded square outline each, drawn
        /// on the window plane and hidden on every other page.</summary>
        public SpriteRenderer?[] tiles = new SpriteRenderer?[Tiles];
        /// <summary>The caption inside each tile.</summary>
        public TextMesh?[] tileLabels = new TextMesh?[Tiles];
        /// <summary>The tile under the cursor, -1 = none.</summary>
        public int hoverTile = -1;
        /// <summary>One-locomotive layout: the pinned loco on the glass, an
        /// index into TabletPins.Guids. Scrolling the header row pages it.</summary>
        public int locoPage;
        /// <summary>The one-locomotive layout's own text planes: the same ten
        /// rows at twice the size, across the whole glass. Its own objects
        /// rather than the quadrant's moved about, so switching layout is a
        /// matter of which pair is written to.</summary>
        /// <summary>ONE TEXT OBJECT PER ROW, placed at the middle of the
        /// click band it belongs to. A ten-line block was tried twice and it
        /// cannot work: a font lays its lines out at its own pitch, the bands
        /// are a fixed pitch, and the two walk apart down the glass — which is
        /// how a row lit up two bands away from the cursor.</summary>
        public const int RowSlots = 10;
        public TextMesh?[] singleRows = new TextMesh?[RowSlots];
        public TextMesh?[] singleVals = new TextMesh?[RowSlots];
        /// <summary>The one-locomotive page's bottom bar: an arrow at each end
        /// and the machine it is showing in the middle.</summary>
        public TextMesh? barLabel;
        public TextMesh? barPrev;
        public TextMesh? barNext;
        /// <summary>The bar's left end on every page but the one-locomotive
        /// board, where the two arrows own both ends.</summary>
        public TextMesh? barBack;
        public SpriteRenderer? barIcon;
        public GameObject? barLine;
        /// <summary>UNPIN, in the bar beside the machine it would drop. The
        /// radio can only reach a locomotive that is in range; this reaches the
        /// one on the glass wherever it has got to.</summary>
        public SpriteRenderer? barUnpinTile;
        public TextMesh? barUnpinLabel;
        /// <summary>The outlines the rest of the bar's buttons sit in: BACK and
        /// the two paging arrows wear the same box UNPIN does.</summary>
        public SpriteRenderer? barBackTile;
        public SpriteRenderer? barPrevTile;
        public SpriteRenderer? barNextTile;
        /// <summary>The outlines round the one-locomotive board's two header
        /// buttons: the way out on the left, the setup/status switch on the
        /// right.</summary>
        public SpriteRenderer? singleBackTile;
        public SpriteRenderer? singleModeTile;
        /// <summary>What in the bar is under the cursor: 0 the left end, 1 the
        /// right one, 2 UNPIN, -1 nothing.</summary>
        public int hoverArrow = -1;
        /// <summary>GO and STOP, one pair per window and one for the
        /// one-locomotive board: a rounded outline with the word centred in it,
        /// filling the half of the row the click lands in.</summary>
        public SpriteRenderer?[] goTiles = new SpriteRenderer?[TabletPins.Max];
        public SpriteRenderer?[] stopTiles = new SpriteRenderer?[TabletPins.Max];
        public TextMesh?[] goLabels = new TextMesh?[TabletPins.Max];
        public TextMesh?[] stopLabels = new TextMesh?[TabletPins.Max];
        public SpriteRenderer? singleGoTile;
        public SpriteRenderer? singleStopTile;
        public TextMesh? singleGoLabel;
        public TextMesh? singleStopLabel;
        /// <summary>The one-locomotive board's [setup] / [status] tag, in the
        /// top-right corner. The machine itself is named in the bottom bar, so
        /// the header row carries nothing but this.</summary>
        public TextMesh? singleMode;
        /// <summary>The one-locomotive board's result flash, on a line of its
        /// own under the buttons. It cannot ride in the row block: ten text
        /// lines do not measure the same as ten click bands, and the last line
        /// came out over the GO button.</summary>
        public TextMesh? singleFlash;
        /// <summary>How many cars the strip could not fit, as "+N".</summary>
        public TextMesh? consistMore;
        /// <summary>The one-locomotive board's way out, in the top-left corner
        /// opposite the mode tag: its bottom bar is both arrows, so BACK has no
        /// end to sit on down there.</summary>
        public TextMesh? singleBack;

        public TextMesh?[][] windowRows = NewGrid();
        /// <summary>Second column of each quadrant: the VALUES, on their own
        /// plane at a fixed x so the board reads as a two-column table. The
        /// label plane keeps anything that spans the full quadrant width
        /// (header, detail lines, GO/STOP).</summary>
        public TextMesh?[][] valueRows = NewGrid();

        private static TextMesh?[][] NewGrid()
        {
            var grid = new TextMesh?[TabletPins.Max][];
            for (int i = 0; i < grid.Length; i++)
                grid[i] = new TextMesh?[RowSlots];
            return grid;
        }

        /// <summary>The booklet icon of each quadrant's loco, top-right of the
        /// header row. The renderer sits on a child of an anchor object placed
        /// at the corner, so fitting the sprite is pure local math.</summary>
        public SpriteRenderer?[] icons = new SpriteRenderer?[TabletPins.Max];
        /// <summary>Each window's status line, under its buttons. Same reason
        /// the one-locomotive board has its own: ten text lines do not measure
        /// the same as ten click bands, and the tenth landed on GO.</summary>
        public TextMesh?[] flashes = new TextMesh?[TabletPins.Max];
        /// <summary>The consist, drawn as the booklet icons of the cars behind
        /// the locomotive, on the one-machine status screen. One renderer per
        /// slot, laid out and sized on the spot — a train is however long it is
        /// and the strip has one glass to fit it on.</summary>
        public SpriteRenderer?[] consist = new SpriteRenderer?[TabletManagerConsist.Max];
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
        /// <summary>An invisible plate over the panic button. The disc itself is
        /// 5 mm across, which is a target the cursor misses more often than it
        /// finds; the ray is meant to land on this instead.</summary>
        public Transform? homePlate;
        /// <summary>The cursor is on the panic button this frame.</summary>
        public bool homeHover;
        /// <summary>Armed until: the first click arms the button, a second one
        /// before this fires it. Stopping every run under the player's hands is
        /// not something to do on a slipped click.</summary>
        public float homeArmedUntil;
        /// <summary>When the disc was last pressed - it sinks and rises again
        /// from here - and how long the glass says it has fired.</summary>
        public float homePressedAt = -100f;
        public float homeStoppedUntil;
        /// <summary>Where the disc sits when nothing is pressing it.</summary>
        public Vector3 homeRest;
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
        /// <summary>Landscape, sized like the job booklet (~21 x 15 cm). Every
        /// figure in this file is written in PRE-SCALE units and multiplied by
        /// this on the way onto the glass — including characterSize, which is
        /// why FitLines has to apply it too.</summary>
        private const float S = 1.45f;
        private const float RepaintSeconds = 0.25f;
        /// <summary>Draws the ten click bands on the glass and prints the row
        /// the cursor is in. On only while the rows are being lined up.</summary>
        private const bool DebugBands = false;
        /// <summary>The status line's four colours — sent, running, finished,
        /// failed — and how long an arrival reads as one before it settles into
        /// the standing "where it is now" line.</summary>
        private const string SentHex = "#add8e6";
        private const string RunningHex = "#e6e6e6";
        private const string DoneHex = "#4cd07a";
        private const string FailedHex = "#e60d0d";
        private const float ArrivedSeconds = 8f;
        /// <summary>The consist strip: how much of the glass it may take, how
        /// tall an icon is, and the widest one car may be drawn.</summary>
        private const float ConsistW = 0.106f;
        private const float ConsistH = 0.013f;
        /// <summary>How much of an icon's own bounds the CAR actually takes.
        /// The booklet art is drawn with a wide margin round it, and laying the
        /// sprites out edge to edge on their raw bounds parks a coupled train a
        /// car-length apart. Measured off the glass, not from the asset.</summary>
        private const float ConsistPack = 0.62f;
        /// <summary>The room kept at the right end for the "+N" when a train is
        /// longer than the strip. Cars are drawn at full height and the ones
        /// that do not fit are counted instead — shrinking thirteen wagons to
        /// fit made a row of dashes.</summary>
        private const float ConsistMoreW = 0.014f;
        /// <summary>How much status detail fits across the WHOLE glass, at the
        /// smaller size the status screen is printed in.</summary>
        private const int WideDetailChars = 46;
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
        /// <summary>The button colours, the same three the text used when GO and
        /// STOP were words in a row: green, red, and the hover orange.</summary>
        private static readonly Color GoColour = new Color(0.235f, 0.702f, 0.443f);
        private static readonly Color StopColour = new Color(0.902f, 0.051f, 0.051f);
        private static readonly Color HoverColour = new Color(1f, 0.647f, 0f);
        /// <summary>The panic button's timings: how long an armed button waits
        /// for its second click, how long the press takes to sink and rise, how
        /// long the glass keeps saying it fired, and how far the disc moves.</summary>
        private const float ConfirmSeconds = 3f;
        private const float PressSeconds = 0.35f;
        private const float StoppedSeconds = 2f;
        private const float HomeTravel = 0.0015f;
        /// <summary>The glass, in the same pre-scale units the pieces are built
        /// in. The mouse ray comes back as a fraction of these.</summary>
        private const float ScreenW = 0.1315f;
        private const float ScreenH = 0.0935f;
        /// <summary>A home-menu tile: a square of this side, its centre this far
        /// either side of the middle. No fill — the outline and the caption are
        /// the whole thing.</summary>
        private const float TileSide = 0.042f;
        private const float TileCentreX = 0.030f;
        private static readonly string[] TileNames = { "OVERVIEW", "LOCOMOTIVES" };
        /// <summary>The one-locomotive page's bottom bar: its height off the
        /// glass, and how much of the width each end arrow answers for. The
        /// rows above it are squeezed by the same fraction, which is why the
        /// scale falls out of the height rather than being its own number.</summary>
        private const float BarH = 0.014f;
        private const float BarArrowZone = 0.18f;
        /// <summary>Where UNPIN sits in the bar, as a fraction of the glass:
        /// between the machine's name and the right-hand arrow.</summary>
        private const float BarUnpinLo = 0.145f;
        private const float BarUnpinHi = 0.5f - BarArrowZone;
        private const float SingleScale = 1f - BarH / ScreenH;
        /// <summary>One quadrant of the rows area — the glass less the bar.</summary>
        private const float QuadH = (ScreenH - BarH) / 2f;
        private static float TileX(int tile) => tile == 0 ? -TileCentreX : TileCentreX;

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
            /// <summary>The standing one-line status and the colour it is
            /// printed in. Kept HERE and not read off the run: a stopped run
            /// leaves the autopilot's own set at once, and the line that says
            /// so has to outlive it.</summary>
            public string line = "";
            public string lineColour = "";
            /// <summary>Where this locomotive was last sent, as the line names
            /// it, and whether it has ever been sent anywhere — until it has,
            /// there is no status to show and the line stays off.</summary>
            public string dest = "";
            public bool dispatched;
            /// <summary>The ending already seen, and when it was first seen. An
            /// arrival reads as an arrival for a while and as a finished job
            /// after that.</summary>
            public string endState = "";
            public float endedAt;
        }

        private static readonly Dictionary<string, Draft> drafts =
            new Dictionary<string, Draft>();

        private readonly List<TabletMarker> tablets = new List<TabletMarker>();
        private float nextRepaint;
        private float nextScan;
        private List<string> yards = new List<string>();
        private bool sweptStale;
        private bool restored;

        public void Update()
        {
            if (!WorldStreamingInit.IsLoaded)
            {
                // Back at the menu. The manager outlives the world — it is on a
                // DontDestroyOnLoad object — so everything read out of a save
                // has to be forgotten here, or the next save loaded in the same
                // session would keep the last one's tablet, pins and drafts.
                restored = false;
                sweptStale = false;
                tablets.Clear();
                TabletPins.Guids.Clear();
                drafts.Clear();
                yards.Clear();
                return;
            }
            if (!sweptStale)
            {
                sweptStale = true;
                SweepStaleTablets();
            }
            // Only once the game has finished putting the saved items back:
            // before that the slot the tablet wants may still be filling up.
            if (!restored
                && DV.Utils.SingletonBehaviour<StartingItemsController>.Instance
                    ?.itemsLoaded == true)
            {
                restored = true;
                RestoreTablet();
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
                        t.hoverCol = t.hoverRow = t.hoverTile = t.hoverArrow = -1;
                        t.homeHover = false;
                        t.homeArmedUntil = 0f;
                        SetOverlay(t, "");
                    }

            if (Time.unscaledTime >= nextRepaint)
            {
                nextRepaint = Time.unscaledTime + RepaintSeconds;
                RefreshData();
                foreach (var g in TabletPins.Guids)
                    TrackStatus(g);
                foreach (var t in tablets)
                    if (t != null)
                        Repaint(t);
            }

            foreach (var t in tablets)
                if (t != null)
                {
                    ApplyHoldPose(t);
                    AnimateHome(t);
                }
        }

        /// <summary>Write a block of rows into their own text objects, one
        /// line to a slot. Splitting HERE rather than letting the font do it is
        /// the whole point: every row then sits in the middle of the band that
        /// answers for it, whatever the font thinks a line is worth.</summary>
        private static void SetRows(TextMesh?[] slots, string block, float charSize)
        {
            var lines = block.Split('\n');
            for (int r = 0; r < slots.Length; r++)
            {
                var slot = slots[r];
                if (slot == null)
                    continue;
                SetText(slot, r < lines.Length ? lines[r] : "");
                float want = charSize * S;
                if (slot.characterSize != want)
                    slot.characterSize = want;
            }
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
            // Only once there ARE cars: the registry exists before the save's
            // own are spawned, and culling against an empty one would wipe the
            // pins just restored from that same save.
            if (registry != null && registry.logicCarToTrainCar.Count > 0)
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
                    {
                        SetOverlay(other, "");
                        other.homeHover = false;
                        other.homeArmedUntil = 0f;
                        if (other.hoverTile != -1 || other.hoverArrow != -1)
                        {
                            other.hoverTile = other.hoverArrow = -1;
                            Repaint(other);
                        }
                    }
                return;
            }
            var t = hit.collider != null ? hit.collider.GetComponentInParent<TabletMarker>() : null;
            if (t == null || t.screenPiece == null)
                return;
            // The red home button: stop everything. Hover = the game's own
            // crosshair prompt plus a brighter glow, like any interactable.
            var hitAt = hit.collider != null ? hit.collider.transform : null;
            bool overHome =
                (t.homeButton != null && hitAt == t.homeButton)
                || (t.homePlate != null && hitAt == t.homePlate);
            t.homeHover = overHome;
            if (overHome)
            {
                bool armed = Time.unscaledTime < t.homeArmedUntil;
                if (Input.GetMouseButtonDown(0))
                {
                    t.homePressedAt = Time.unscaledTime;
                    if (armed)
                    {
                        // Confirmed: everything stops, and the glass says so for
                        // a couple of seconds.
                        t.homeArmedUntil = 0f;
                        t.homeStoppedUntil = Time.unscaledTime + StoppedSeconds;
                        armed = false;
                        Autopilot.Stop(null);
                        foreach (var g in TabletPins.Guids)
                        {
                            var stopped = DraftOf(g);
                            Flash(stopped, "ALL STOPPED", true);
                            if (stopped.dispatched)
                            {
                                stopped.endState = "stopped";
                                SetLine(stopped, "Stopped", FailedHex);
                            }
                        }
                        Repaint(t);
                    }
                    else
                    {
                        t.homeArmedUntil = Time.unscaledTime + ConfirmSeconds;
                        armed = true;
                    }
                }
                Prompt(armed ? "Click again: stop every autopilot run"
                    : "Click: stop every autopilot run");
                SetOverlay(t, Time.unscaledTime < t.homeStoppedUntil
                    ? "ALL AUTOPILOT\nRUNS STOPPED"
                    : armed
                        ? "STOP ALL RUNS?\nCLICK AGAIN TO CONFIRM"
                        : "STOP ALL\nAUTOPILOT RUNS");
                return;
            }
            // Off the button: it forgets it was ever asked.
            t.homeArmedUntil = 0f;
            SetOverlay(t, "");
            int wheel = 0;
            if (Input.mouseScrollDelta.y > 0.01f) wheel = -1;
            else if (Input.mouseScrollDelta.y < -0.01f) wheel = 1;
            bool click = Input.GetMouseButtonDown(0);
            bool rightClick = Input.GetMouseButtonDown(1);

            var local = t.screenPiece.InverseTransformPoint(hit.point);
            // Which board this is: one machine to the page, or all four.
            bool onePerPage = t.page == TabletPage.Locomotives;
            bool onBoard = t.page != TabletPage.Menu && t.pickerCol < 0;
            // How far down the glass, 0 at the top edge and 1 at the bottom. The
            // rows have all but the bottom bar's strip of it, on every page but
            // the menu — the menu is tiles from edge to edge.
            float down = 0.5f - local.z;
            if (t.page != TabletPage.Menu && down >= SingleScale)
            {
                HandleBar(t, local, onePerPage && onBoard, click, rightClick);
                return;
            }
            if (t.hoverArrow != -1)
            {
                t.hoverArrow = -1;
                Repaint(t);
            }
            int col, row;
            bool right;
            if (onePerPage && onBoard)
            {
                // One window over the whole rows area: one column, ten rows, and
                // the quadrant the cursor is in says nothing.
                col = Mathf.Clamp(t.locoPage, 0,
                    Mathf.Max(0, TabletPins.Guids.Count - 1));
                row = Mathf.Clamp((int)(down / SingleScale * Rows), 0, Rows - 1);
                // The header buttons hang into the blank row below them, so a
                // click there is a click on them and not on nothing.
                if (row == 1)
                    row = 0;
                // GO and STOP are two buttons filling the row between them, so
                // that row splits down the middle. Every other row splits at the
                // value column, where its second cell begins.
                right = row == GoRow
                    ? local.x > 0f
                    : local.x + 0.5f
                        > ValueColX * 2f * SingleScale / ScreenW - 0.05f;
            }
            else
            {
                // 2 x 2 windows: quadrant first, then the row inside it. The
                // pairs split the rows area, not the glass, so the bar takes its
                // strip out of the bottom pair rather than off the screen.
                float pair = down / SingleScale * 2f; // 0..2 down the two pairs
                col = (local.x > 0f ? 1 : 0) + (pair < 1f ? 0 : 2);
                row = Mathf.Clamp((int)((pair - Mathf.Floor(pair)) * Rows),
                    0, Rows - 1);
                // Which of the two columns, for the one row that has a live cell
                // in each. The value column starts at ValueColX along a quadrant
                // that is half the glass wide; the split sits just short of it so
                // a click on the word itself counts.
                float xq = local.x > 0f ? local.x : local.x + 0.5f;
                right = row == GoRow
                    ? xq > 0.25f
                    : xq > ValueColX * SingleScale / ScreenW - 0.05f;
            }
            if (t.hoverCol != col || t.hoverRow != row || t.hoverRight != right)
            {
                t.hoverCol = col;
                t.hoverRow = row;
                t.hoverRight = right;
                Repaint(t);
            }

            // The home menu: two tiles, and nothing else on the glass.
            if (t.page == TabletPage.Menu)
            {
                int tile = -1;
                for (int i = 0; i < TabletMarker.Tiles; i++)
                    if (Mathf.Abs(local.x - TileX(i) / ScreenW) <= TileSide / 2f / ScreenW
                        && Mathf.Abs(local.z) <= TileSide / 2f / ScreenH)
                    {
                        tile = i;
                        break;
                    }
                if (t.hoverTile != tile)
                {
                    t.hoverTile = tile;
                    Repaint(t);
                }
                if (tile >= 0)
                {
                    Prompt(tile == 0
                        ? "Click: the locomotive board" : "Click: tablet settings");
                    if (click)
                    {
                        t.page = tile == 0
                            ? TabletPage.Overview : TabletPage.Locomotives;
                        t.hoverTile = -1;
                        Repaint(t);
                    }
                }
                return;
            }

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
                int cell = PickerCell(col, row);
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
                            else if (t.pickerRow == TrackRow) pd.destDriveTrack = index;
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

            // Right click anywhere on the board is the bar's BACK.
            if (rightClick)
            {
                Back(t);
                Repaint(t);
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
                Prompt(t.page == TabletPage.Locomotives && !t.hoverRight
                    ? "Click: back to the menu"
                    : t.page == TabletPage.Locomotives && TabletPins.Guids.Count > 1
                        ? "Click: setup or status.  Scroll: another locomotive"
                        : "Click: switch between setup and status");
            if (click || wheel != 0)
            {
                Act(t, col, guid, row, click, wheel, t.hoverRight);
                Repaint(t);
            }
        }

        /// <summary>The bottom bar under the cursor. On the one-locomotive
        /// board its two ends are the paging arrows; on every other page the
        /// left end is BACK and the right end is nothing.</summary>
        private void HandleBar(TabletMarker t, Vector3 local, bool arrows,
            bool click, bool rightClick)
        {
            bool atLeft = local.x < -0.5f + BarArrowZone;
            bool atRight = local.x > 0.5f - BarArrowZone;
            bool atUnpin = arrows && local.x > BarUnpinLo && local.x < BarUnpinHi;
            int hover = atLeft ? 0 : arrows && atRight ? 1 : atUnpin ? 2 : -1;
            if (t.hoverArrow != hover)
            {
                t.hoverArrow = hover;
                Repaint(t);
            }
            if (rightClick)
            {
                Back(t);
                Repaint(t);
                return;
            }
            if (hover < 0)
                return;
            if (!arrows)
            {
                Prompt("Click: back");
                if (click)
                {
                    Back(t);
                    Repaint(t);
                }
                return;
            }
            if (hover == 2)
            {
                Prompt("Click: drop this locomotive from the tablet");
                if (click)
                    Unpin(t);
                return;
            }
            Prompt(hover == 0
                ? "Click: the locomotive before this one"
                : "Click: the next locomotive");
            if (click && TabletPins.Guids.Count > 0)
            {
                t.locoPage = Wrap(t.locoPage + (hover == 0 ? -1 : 1),
                    TabletPins.Guids.Count);
                Repaint(t);
            }
        }

        /// <summary>Drop the locomotive the one-machine page is showing. The
        /// radio does the same thing, but only within its own range — and a
        /// machine that has driven off is exactly the one worth dropping. Its
        /// setup is kept, as the radio keeps it: what goes is the pin.</summary>
        private void Unpin(TabletMarker t)
        {
            if (TabletPins.Guids.Count == 0)
                return;
            int at = Mathf.Clamp(t.locoPage, 0, TabletPins.Guids.Count - 1);
            string guid = TabletPins.Guids[at];
            var loco = LocoOf(guid);
            TabletPins.Guids.RemoveAt(at);
            t.locoPage = TabletPins.Guids.Count > 0
                ? Mathf.Clamp(at, 0, TabletPins.Guids.Count - 1) : 0;
            t.hoverArrow = -1;
            // The pins are part of what the save carries, and every other
            // tablet is showing the same list.
            WriteSaveState();
            Main.mod?.Logger.Log(
                "[TABLET] unpinned " + (loco != null ? loco.ID : guid));
            foreach (var other in tablets)
                if (other != null)
                    Repaint(other);
        }

        /// <summary>One step back: out of a picker to the board it belongs to,
        /// off any page to the menu.</summary>
        private static void Back(TabletMarker t)
        {
            if (t.pickerCol >= 0)
                t.pickerCol = -1;
            else
                t.page = TabletPage.Menu;
            t.hoverTile = t.hoverArrow = -1;
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

        /// <summary>The panic button's own frame: the disc sinks under a press
        /// and rises again, and it burns while it is armed. Run every frame and
        /// not only under the cursor, so a press the mouse has already moved off
        /// still finishes.</summary>
        private static void AnimateHome(TabletMarker t)
        {
            var disc = t.homeButton;
            if (disc == null)
                return;
            float now = Time.unscaledTime;
            float since = now - t.homePressedAt;
            // 0 at rest, 1 fully down: struck down fast, let back up slowly.
            const float Fall = PressSeconds * 0.25f;
            float press = since < 0f || since > PressSeconds ? 0f
                : since < Fall ? since / Fall
                : 1f - (since - Fall) / (PressSeconds - Fall);
            var parent = disc.parent;
            if (parent != null)
                disc.localPosition = t.homeRest - parent.InverseTransformVector(
                    parent.up * (press * HomeTravel));
            if (t.homeRenderer == null)
                return;
            Color glow;
            if (now < t.homeStoppedUntil)
                glow = new Color(1f, 0.25f, 0.15f);
            else if (now < t.homeArmedUntil)
                // Armed: a pulse, because the button is asking now rather than
                // waiting. It dies with the confirm window.
                glow = Color.Lerp(new Color(0.35f, 0.03f, 0.03f),
                    new Color(1f, 0.15f, 0.10f),
                    0.5f + 0.5f * Mathf.Sin(now * 18f));
            else
                glow = t.homeHover
                    ? new Color(0.9f, 0.05f, 0.05f)
                    : new Color(0.25f, 0.02f, 0.02f);
            // The press flashes white through whatever it is showing.
            t.homeRenderer.material.SetColor("_EmissionColor",
                Color.Lerp(glow, Color.white, press * 0.6f));
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

            // Header toggles the quadrant between its two screens — and, with
            // one locomotive to a page, the wheel is what walks the board.
            if (row == 0)
            {
                // One locomotive to a page: the header row itself is empty and
                // its two corners are the controls — BACK on the left, the
                // setup/status tag on the right.
                if (click && t.page == TabletPage.Locomotives && !right)
                {
                    Back(t);
                    return;
                }
                if (click)
                    d.showStatus = !d.showStatus;
                else if (wheel != 0 && t.page == TabletPage.Locomotives
                    && TabletPins.Guids.Count > 0)
                    t.locoPage = Wrap(t.locoPage + wheel, TabletPins.Guids.Count);
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
                    d.endState = "stopped";
                    SetLine(d, d.dest.Length > 0
                        ? "Stopped short of " + d.dest : "Stopped", FailedHex);
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
                d.dest = dest;
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
                var target = DriveTarget(d);
                d.dest = target.id.Length > 0 ? target.id : yard;
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
                d.dispatched = true;
                d.endState = "";
                SetLine(d, "Dispatched", SentHex);
            }
            // A dispatched quadrant flips to its status screen by itself...
            if (!refused)
                d.showStatus = true;
        }

        /// <summary>Keep a locomotive's one-line status up to date. Read from
        /// the run while there is one; what it leaves behind stands until the
        /// machine is sent somewhere again.</summary>
        private static void TrackStatus(string guid)
        {
            var d = DraftOf(guid);
            string? state = Autopilot.RunStateOf(guid);
            if (state == null)
                return;
            string detail = Autopilot.RunDetailOf(guid) ?? "";
            if (state == "driving")
            {
                // Dispatched, but nothing reported yet: it has been sent and has
                // not moved on it. That is its own line.
                d.endState = "";
                SetLine(d, detail.Length == 0
                        ? "Dispatched"
                        : (d.shunt ? "Shunting" : "Driving") + Doing(guid, detail),
                    detail.Length == 0 ? SentHex : RunningHex);
                return;
            }
            if (state == "arrived" || state == "coupled")
            {
                if (d.endState != state)
                {
                    d.endState = state;
                    d.endedAt = Time.unscaledTime;
                }
                string where = d.dest.Length > 0 ? d.dest : "the destination";
                SetLine(d, Time.unscaledTime - d.endedAt < ArrivedSeconds
                    ? "Arrived at " + where
                    : "Done. On " + where, DoneHex);
                return;
            }
            if (state == "error")
            {
                d.endState = state;
                SetLine(d, "Error: " + detail, FailedHex);
            }
        }

        private static void SetLine(Draft d, string text, string colour)
        {
            d.line = text;
            d.lineColour = colour;
        }

        /// <summary>What a moving locomotive is busy with, in the few words the
        /// line has room for: the phase it is in when that is the whole story,
        /// and otherwise where the next step of its plan takes it.</summary>
        private static string Doing(string guid, string detail)
        {
            string low = detail.ToLowerInvariant();
            if (low.Contains("coupling to"))
                return " to buffer up";
            // A run stops for two quite different reasons and the word for one
            // is not the word for the other: stopping to change ends is
            // reversing, and only stopping ON the destination is parking.
            if (low.Contains("reverser") || low.Contains("throw it")
                || low.Contains("direction change"))
                return ", reversing";
            if (low.Contains("marks") || low.Contains("clear within")
                || low.Contains("overshot") || low.Contains("spotting"))
                return ", parking";
            if (low.Contains("turntable"))
                return " on the turntable";
            if (low.Contains("backing clear"))
                return ", backing clear";
            if (low.Contains("pulling clear"))
                return ", pulling clear";
            if (low.Contains("re-routing"))
                return ", re-routing";
            if (low.Contains("waiting for"))
                return ", waiting";
            string step = Autopilot.RunNextStepOf(guid) ?? "";
            if (step.StartsWith("drive to ", StringComparison.Ordinal))
                return " to " + step.Substring(9);
            if (step.StartsWith("cross ", StringComparison.Ordinal))
                return " to " + step.Substring(6).Split(' ')[0];
            if (step.StartsWith("clear ", StringComparison.Ordinal))
                return ", reversing at " + step.Substring(6).Split(',')[0];
            if (step.StartsWith("run round ", StringComparison.Ordinal))
                return ", running round";
            if (step.StartsWith("ride the turntable", StringComparison.Ordinal))
                return " to the turntable";
            return "";
        }

        /// <summary>The status line as printed: a fresh dispatch or a refusal
        /// flashes over it for a few seconds, and the standing line has it the
        /// rest of the time. Nothing at all until the machine has been sent
        /// somewhere once — a board with no history has nothing to say.</summary>
        private static string StatusLine(Draft d, int chars)
        {
            string flash = ResultLine(d);
            if (flash.Length > 0)
                return flash;
            if (!d.dispatched || d.line.Length == 0)
                return "";
            string text = d.line.Length > chars
                ? d.line.Substring(0, chars - 1) + "-" : d.line;
            return $"<color={d.lineColour}>{text}</color>";
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
            bool menu = !overlayActive && t.page == TabletPage.Menu;
            bool locos = !overlayActive && t.page != TabletPage.Menu;
            // The destination-picker page owns the whole glass.
            bool picker = locos && t.pickerCol >= 0
                && t.pickerCol < TabletPins.Guids.Count;
            bool board = locos && !picker && TabletPins.Guids.Count > 0;
            bool onePerPage = t.page == TabletPage.Locomotives;
            t.locoPage = TabletPins.Guids.Count > 0
                ? Mathf.Clamp(t.locoPage, 0, TabletPins.Guids.Count - 1) : 0;
            // The divider cross marks the four locomotive windows, and nothing
            // else the tablet draws is four windows: not the menu, not the
            // settings list, not a page of yards, not one locomotive over the
            // whole glass — the cross would suggest a split none of them have.
            bool grid = board && !onePerPage;
            if (t.dividerV != null && t.dividerV.activeSelf != grid)
                t.dividerV.SetActive(grid);
            if (t.dividerH != null && t.dividerH.activeSelf != grid)
                t.dividerH.SetActive(grid);
            // The home menu's tiles: a white outline each, dimmed until hovered.
            for (int i = 0; i < TabletMarker.Tiles; i++)
            {
                var tint = t.hoverTile == i ? Color.white : new Color(1f, 1f, 1f, 0.6f);
                var tile = t.tiles[i];
                if (tile != null)
                {
                    if (tile.enabled != menu)
                        tile.enabled = menu;
                    tile.color = tint;
                }
                var caption = t.tileLabels[i];
                if (caption != null)
                {
                    SetText(caption, menu ? TileNames[i] : "");
                    caption.color = tint;
                }
            }
            if (t.emptyMsg != null)
                SetText(t.emptyMsg, overlayActive
                    ? $"<color=#ff2020><b>{t.overlay}</b></color>"
                    // The line that used to BE the home screen. It belongs on
                    // the locomotives page, which is the one it talks about.
                    : locos && !picker && TabletPins.Guids.Count == 0
                        ? "Select a locomotive using the\nComms Radio -> Autopilot" : "");

            // One locomotive to a page: the same ten rows at twice the size,
            // across the whole glass, on their own pair of planes.
            {
                var (oneLabels, oneVals) = board && onePerPage
                    ? RenderWindow(TabletPins.Guids[t.locoPage],
                        t.hoverCol == t.locoPage ? t.hoverRow : -1, t.hoverRight,
                        named: false)
                    : ("", "");
                // The status screen is a read, not a form: it goes on the glass
                // smaller, which leaves the detail line room to say what it
                // means and the consist strip room to sit under it.
                bool small = board && onePerPage
                    && DraftOf(TabletPins.Guids[t.locoPage]).showStatus;
                float wantSize = small
                    ? RowCharSize * 1.22f : RowCharSize * 2f * SingleScale;
                SetRows(t.singleRows, oneLabels, wantSize);
                SetRows(t.singleVals, oneVals, wantSize);
            }
            bool oneStatus = board && onePerPage
                && DraftOf(TabletPins.Guids[t.locoPage]).showStatus;
            RepaintConsist(t, oneStatus ? TabletPins.Guids[t.locoPage] : null);
            if (t.singleFlash != null)
            {
                bool one = board && onePerPage;
                SetText(t.singleFlash, one
                    ? StatusLine(DraftOf(TabletPins.Guids[t.locoPage]), 46) : "");
            }
            {
                bool one = board && onePerPage;
                PaintButton(t.singleBackTile, t.singleBack, one,
                    one && t.hoverRow == 0 && !t.hoverRight, "< BACK",
                    new Color(1f, 1f, 1f, 0.6f));
            }
            {
                // Read AFTER the window: a run that has just finished sends the
                // page back to setup from inside RenderWindow. The switch is a
                // button like the rest now — which screen it would take you to
                // is the word in the box.
                bool one = board && onePerPage;
                bool status = one && DraftOf(TabletPins.Guids[t.locoPage]).showStatus;
                PaintButton(t.singleModeTile, t.singleMode, one,
                    one && t.hoverRow == 0 && t.hoverRight
                        && t.hoverCol == t.locoPage,
                    status ? "STATUS" : "SETUP",
                    status ? new Color(0.678f, 0.847f, 0.902f)
                        : new Color(0.118f, 0.478f, 0.235f));
            }
            RepaintBar(t, !overlayActive && t.page != TabletPage.Menu,
                board && onePerPage);

            // GO and STOP, drawn over the row they used to be words in. Only the
            // half under the cursor lights.
            bool goHover = t.hoverRow == GoRow;
            for (int i = 0; i < TabletPins.Max; i++)
            {
                bool on = board && !onePerPage && i < TabletPins.Guids.Count;
                bool hover = on && goHover && t.hoverCol == i;
                PaintButton(t.goTiles[i], t.goLabels[i], on,
                    hover && !t.hoverRight, "GO", GoColour);
                PaintButton(t.stopTiles[i], t.stopLabels[i], on,
                    hover && t.hoverRight, "STOP", StopColour);
            }
            bool oneOn = board && onePerPage;
            bool oneHover = oneOn && goHover && t.hoverCol == t.locoPage;
            PaintButton(t.singleGoTile, t.singleGoLabel, oneOn,
                oneHover && !t.hoverRight, "GO", GoColour);
            PaintButton(t.singleStopTile, t.singleStopLabel, oneOn,
                oneHover && t.hoverRight, "STOP", StopColour);

            for (int i = 0; i < TabletPins.Max; i++)
            {
                var line = t.flashes[i];
                if (line != null)
                    SetText(line, board && !onePerPage && i < TabletPins.Guids.Count
                        ? StatusLine(DraftOf(TabletPins.Guids[i]), DetailChars)
                        : "");
                string labels, vals;
                if (picker)
                {
                    // The picker page is a plain list — no value column.
                    labels = RenderPickerColumn(t, i);
                    vals = "";
                }
                else if (board && !onePerPage && i < TabletPins.Guids.Count)
                {
                    (labels, vals) = RenderWindow(
                        TabletPins.Guids[i], t.hoverCol == i ? t.hoverRow : -1,
                        t.hoverRight);
                }
                else
                {
                    labels = vals = "";
                }
                SetRows(t.windowRows[i], labels, RowCharSize * SingleScale);
                SetRows(t.valueRows[i], vals, RowCharSize * SingleScale);
            }

            // The booklet icon beside each header: the livery's own sprite,
            // fitted to header-row height against the top-right anchor.
            for (int i = 0; i < TabletPins.Max; i++)
            {
                var icon = t.icons[i];
                if (icon == null)
                    continue;
                Sprite? sprite = null;
                // The booklet icons hang off the quadrant headers; the
                // one-locomotive page has no quadrants to hang them off.
                if (board && !onePerPage && i < TabletPins.Guids.Count)
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

        /// <summary>The consist strip: every car of the train this locomotive
        /// is on, in order, as the booklet icon of its own livery. The machine
        /// under autopilot is drawn full white and what it is dragging a shade
        /// down, so the strip says at a glance which end of it is the engine.
        /// Drawn at full height whatever the length: as many cars as the
        /// strip holds, and the rest counted at the end as "+N".</summary>
        private void RepaintConsist(TabletMarker t, string? guid)
        {
            var loco = guid != null ? LocoOf(guid) : null;
            var cars = loco?.trainset?.cars;
            int have = cars == null ? 0
                : Mathf.Min(cars.Count, TabletManagerConsist.Max);
            // Full height always: a car too small to recognise says nothing.
            // As many as the strip holds are drawn and the rest are counted.
            var widths = new float[TabletManagerConsist.Max];
            int shown = 0;
            float total = 0f;
            for (int i = 0; i < have; i++)
            {
                var art = cars![i].carLivery?.icon;
                if (art == null)
                    break;
                var ab = art.bounds;
                float w = ConsistH / Mathf.Max(0.0001f, ab.size.y)
                    * ab.size.x * ConsistPack;
                float room = ConsistW - (i + 1 < have ? ConsistMoreW : 0f);
                if (shown > 0 && total + w > room)
                    break;
                widths[i] = w;
                total += w;
                shown++;
            }
            int rest = (cars == null ? 0 : cars.Count) - shown;
            // Centred when the whole train is on the glass; pushed left when it
            // is not, so the count sits at the end of the strip.
            float x = rest > 0 ? -ConsistW / 2f : -total / 2f;
            for (int i = 0; i < TabletManagerConsist.Max; i++)
            {
                var slot = t.consist[i];
                if (slot == null)
                    continue;
                var car = i < shown ? cars![i] : null;
                var sprite = car?.carLivery?.icon;
                if (sprite == null)
                {
                    if (slot.enabled)
                        slot.enabled = false;
                    continue;
                }
                if (slot.sprite != sprite)
                    slot.sprite = sprite;
                var b = sprite.bounds;
                float k = ConsistH / Mathf.Max(0.0001f, b.size.y);
                slot.transform.localScale = Vector3.one * k;
                slot.transform.localPosition = new Vector3(
                    x + widths[i] / 2f - k * b.center.x, -k * b.center.y, 0f);
                x += widths[i];
                slot.color = car == loco
                    ? Color.white : new Color(1f, 1f, 1f, 0.78f);
                if (!slot.enabled)
                    slot.enabled = true;
            }
            if (t.consistMore != null)
                SetText(t.consistMore, rest > 0 ? $"+{rest}" : "");
        }

        /// <summary>The one-locomotive page's bottom bar: an arrow at each end
        /// and, in the middle, the machine on the glass — its number, its type
        /// and its booklet icon. Off on every other page.</summary>
        private void RepaintBar(TabletMarker t, bool on, bool arrows)
        {
            var loco = arrows && TabletPins.Guids.Count > 0
                ? LocoOf(TabletPins.Guids[t.locoPage]) : null;
            arrows = arrows && loco != null;
            if (t.barLine != null && t.barLine.activeSelf != on)
                t.barLine.SetActive(on);
            // Both arrows show whenever the one-locomotive board does: with a
            // single locomotive pinned they come back to it, which reads better
            // than a bar that loses its ends. Every other page puts BACK in the
            // left end instead — the arrows are what own it there.
            // Every one of these is the same kind of thing as GO, STOP and
            // UNPIN — a word in a rounded box — and they light the same way.
            var dim = new Color(1f, 1f, 1f, 0.6f);
            PaintButton(t.barBackTile, t.barBack, on && !arrows,
                t.hoverArrow == 0, "< BACK", dim);
            PaintButton(t.barPrevTile, t.barPrev, arrows,
                t.hoverArrow == 0, "<", dim);
            PaintButton(t.barNextTile, t.barNext, arrows,
                t.hoverArrow == 1, ">", dim);
            // UNPIN is the same kind of button as GO and STOP, put where the
            // machine it drops is named.
            PaintButton(t.barUnpinTile, t.barUnpinLabel, arrows,
                t.hoverArrow == 2, "UNPIN", dim);
            if (t.barLabel != null)
                SetText(t.barLabel, loco == null ? ""
                    : DebugBands
                        ? $"{loco.ID}  row {t.hoverRow}"
                        : $"{loco.ID}   {LocoType(loco)}");
            var icon = t.barIcon;
            if (icon != null)
            {
                var sprite = loco?.carLivery?.icon;
                if (sprite == null)
                {
                    if (icon.enabled)
                        icon.enabled = false;
                }
                else
                {
                    if (icon.sprite != sprite)
                    {
                        icon.sprite = sprite;
                        // Fitted to the bar's height, centred on its anchor.
                        const float IconH = 0.009f;
                        var b = sprite.bounds;
                        float k = IconH / Mathf.Max(0.0001f, b.size.y);
                        icon.transform.localScale = Vector3.one * k;
                        icon.transform.localPosition = -k * b.center;
                    }
                    if (!icon.enabled)
                        icon.enabled = true;
                }
            }
        }

        /// <summary>One GO or STOP button: shown or not, lit or not.</summary>
        private static void PaintButton(SpriteRenderer? tile, TextMesh? label,
            bool on, bool hover, string word, Color colour)
        {
            var paint = hover ? HoverColour : colour;
            if (tile != null)
            {
                if (tile.enabled != on)
                    tile.enabled = on;
                tile.color = paint;
            }
            if (label == null)
                return;
            SetText(label, on ? word : "");
            label.color = paint;
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

        /// <summary>The road row of a drive setup — which track in the
        /// destination yard to pull up on. The row above it picks the yard.</summary>
        private const int TrackRow = 4;

        /// <summary>Every option the full-glass picker offers, which depends on the
        /// row it was opened from: destination yards and the roads within one for a
        /// road move, the yard's tracks for a shunt.</summary>
        private List<string> PickerOptions(string guid, int row)
        {
            var d = DraftOf(guid);
            if (d.shunt)
                return ShuntTracks(guid);
            return row == TrackRow
                ? DriveTargets(DriveYard(d)).Select(o => o.label).ToList()
                : yards;
        }

        /// <summary>One column of the full-glass destination list. Cell 0 of the
        /// first column is BACK; options fill column by column.</summary>
        /// <summary>Which entry of the list a quadrant's row holds. The list
        /// runs DOWN the left half of the glass and then down the right one —
        /// a two-column list, the way a printed one reads — and not across the
        /// four quadrants in turn.</summary>
        private static int PickerCell(int col, int row) =>
            (col == 1 ? 2 : col == 2 ? 1 : col) * Rows + row;

        private string RenderPickerColumn(TabletMarker t, int col)
        {
            string guid = TabletPins.Guids[t.pickerCol];
            var d = DraftOf(guid);
            var options = PickerOptions(guid, t.pickerRow);
            int chosen = d.shunt ? d.destShunt
                : t.pickerRow == TrackRow ? d.destDriveTrack : d.destDrive;
            var rows = new string[Rows];
            for (int row = 0; row < Rows; row++)
            {
                int cell = PickerCell(col, row);
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
        /// "LocoDE2" is a DE2. The ids that do not match the name on the
        /// machine's own builder plate are mapped to it — the game calls the BE2
        /// "Microshunter". Anything else is printed as it comes, and a car with
        /// no livery at all gets nothing.</summary>
        private static string LocoType(TrainCar loco)
        {
            string id = loco.carLivery?.id ?? "";
            if (id == "LocoMicroshunter")
                return "BE2";
            return id.StartsWith("Loco", StringComparison.Ordinal)
                ? id.Substring(4) : id;
        }

        /// <summary><paramref name="named"/> is false for the one-locomotive
        /// board, which names its machine in the bottom bar and keeps its mode
        /// tag in the corner — its header row is left empty.</summary>
        private (string labels, string values) RenderWindow(
            string guid, int hoverRow, bool hoverRight, bool named = true)
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
                // The one-machine board prints this small and across the whole
                // glass, so it takes a good deal more of the line than a
                // quadrant does before it has to wrap.
                int width = named ? DetailChars : WideDetailChars;
                rows[3] = detail.Length > width
                    ? detail.Substring(0, width) : detail;
                rows[4] = detail.Length > width
                    ? detail.Substring(width,
                        Mathf.Min(width, detail.Length - width))
                    : "";
                // No limit line on a shunt: the server holds yard work to yard
                // pace whatever the setup says, so the figure means nothing there.
                // Target moves up a row and the limit takes the one under it:
                // the two rows at the bottom of the block are the consist
                // strip's, and a wagon drawn over a word reads as neither.
                rows[5] = "Target";
                vals[5] = dest;
                bool showLimit = limit > 0 && !d.shunt;
                rows[6] = showLimit ? "Limit" : "";
                vals[6] = showLimit ? $"{limit:0} km/h" : "";
            }
            else if (d.shunt)
            {
                // Shunting, in the console's own order: where to, which end goes
                // in, which junction through, and what to do on arrival.
                var viaOptions = ViaOptions(d, guid);
                rows[0] = $"<b>{loco.ID}</b>  <color=#1e7a3c>[setup]</color>";
                vals[0] = LocoType(loco);
                rows[2] = "Job";
                vals[2] = "Shunting";
                valCol[2] = "#1e7a3c";
                rows[3] = "Track";
                vals[3] = dest;
                // "Lead" is the end that goes in first, "Route" the junction it
                // goes in through, and the last two are the two things done on
                // arrival. One word each, to match the road move's rows.
                rows[4] = "Lead";
                vals[4] = LeadNames[d.arrive];
                rows[5] = "Route";
                vals[5] = viaOptions[Mathf.Clamp(d.via, 0, viaOptions.Count - 1)].label;
                rows[6] = "Couple";
                vals[6] = d.couple ? "Yes" : "No";
                rows[7] = "Uncouple";
                vals[7] = d.uncouple ? "Yes" : "No";
                // State lives on the status screen now; here only a fresh
                // dispatch/stop result flashes.
            }
            else
            {
                // A road move: yard, the road to pull up on there, top speed.
                // Coupling, shoving and via belong to yard work and the server
                // drops them for a drive, so they are not offered here either.
                string yard = DriveYard(d);
                rows[0] = $"<b>{loco.ID}</b>  <color=#1e7a3c>[setup]</color>";
                vals[0] = LocoType(loco);
                rows[2] = "Job";
                vals[2] = "Driving";
                valCol[2] = "#90ee90";
                rows[3] = "Yard";
                vals[3] = yard.Length > 0 ? yard : "-";
                rows[4] = "Track";
                vals[4] = DriveTarget(d).label;
                rows[5] = "Top speed";
                vals[5] = $"{d.speed} km/h";
            }

            // GO and STOP are no longer words in this row: they are two
            // buttons drawn over it, each with its word centred in the half the
            // click lands in. The row itself is left empty for them.

            if (!named)
            {
                rows[0] = "";
                vals[0] = "";
                // Drawn on its own line instead - see TabletMarker.singleFlash.
                rows[9] = "";
            }

            for (int i = 0; i < Rows; i++)
            {
                vals[i] ??= "";
                // Nothing on the status screen answers a click, so nothing on
                // it lights under the cursor either.
                if (i == hoverRow && i >= 1 && i < GoRow && !d.showStatus)
                {
                    // The hover tint covers the whole row, both columns, and it
                    // takes the row's own colour with it.
                    rows[i] = $"<color=#ffa500>{rows[i]}</color>";
                    if (vals[i].Length > 0)
                        vals[i] = $"<color=#ffa500>{vals[i]}</color>";
                    continue;
                }
                if (rowCol[i] != null && rows[i] != null && rows[i].Length > 0)
                    rows[i] = $"<color={rowCol[i]}>{rows[i]}</color>";
                if (valCol[i] != null && vals[i].Length > 0)
                    vals[i] = $"<color={valCol[i]}>{vals[i]}</color>";
            }
            return (string.Join("\n", rows), string.Join("\n", vals));
        }

        /* ----------------------------------------------------- persistence -- */

        // The tablet is a clone of the map with its own screen bolted on, and
        // the save file has no way to say that: it stores the prefab name and
        // respawns from the prefab. So the item is kept out of the save
        // (GiveTablet excludes it from every storage) and what IS written is
        // the fact that the player had one, which slot it sat in and what was
        // pinned to it. The board is then rebuilt on load, same as it is when
        // the key is pressed.

        /// <summary>Save keys. Prefixed, as they share the save with the game's
        /// own and with every other mod's.</summary>
        private const string HasKey = "Autopilot_tablet";
        private const string SlotKey = "Autopilot_tablet_slot";
        private const string PinsKey = "Autopilot_tablet_pins";

        private void OnEnable()
        {
            SaveGameManager.AboutToSave += OnAboutToSave;
        }

        private void OnDisable()
        {
            SaveGameManager.AboutToSave -= OnAboutToSave;
        }

        private void OnAboutToSave(SaveType type) => WriteSaveState();

        /// <summary>Put the tablet's state in the live save data. Called on
        /// every save and whenever a tablet is handed out — the data object is
        /// the one the next save writes, so an early write costs nothing.</summary>
        private void WriteSaveState()
        {
            var data = DV.Utils.SingletonBehaviour<SaveGameManager>.Instance?.data;
            if (data == null)
                return;
            var live = tablets.FirstOrDefault(m => m != null);
            data.SetBool(HasKey, live != null);
            // A tablet in the player's hands is in no slot at all, and IndexOf
            // says -1 for it. That is why the "has one" flag is its own key
            // rather than being read off the slot number.
            int slot = -1;
            if (live != null)
                slot = Inventory.Instance?.IndexOf(live.gameObject) ?? -1;
            data.SetInt(SlotKey, slot);
            data.SetStringArray(PinsKey, TabletPins.Guids.ToArray());
        }

        private void RestoreTablet()
        {
            var data = DV.Utils.SingletonBehaviour<SaveGameManager>.Instance?.data;
            if (data == null)
                return;
            var pins = data.GetStringArray(PinsKey);
            if (pins != null)
            {
                TabletPins.Guids.Clear();
                TabletPins.Guids.AddRange(pins
                    .Where(g => !string.IsNullOrEmpty(g)).Take(TabletPins.Max));
            }
            if (data.GetBool(HasKey) != true)
                return;
            Main.mod?.Logger.Log("Restoring the dispatcher tablet from the save");
            GiveTablet(data.GetInt(SlotKey) ?? -1);
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

        /// <summary>Is this item one of our tablets? Three tests, because a
        /// reload leaves the previous assembly's types alive alongside this
        /// one's and none of the three holds on its own: the game renames item
        /// objects on pickup, a marker from another assembly is a DIFFERENT
        /// type to this one however it is spelled, and the screen parts are the
        /// only thing that is ours whatever assembly built them.</summary>
        private static bool IsTablet(Component spec)
        {
            if (spec.gameObject.name.StartsWith("AutopilotDispatcherTablet",
                    StringComparison.Ordinal))
                return true;
            if (spec.GetComponents<Component>().Any(
                    c => c != null && c.GetType().Name == nameof(TabletMarker)))
                return true;
            return spec.GetComponentsInChildren<Transform>(true)
                .Any(x => x != null && x.name == "TabletScreen");
        }

        /// <summary>Take a tablet item out of the world for good: out of the
        /// hands or the slot it sits in first, or the inventory keeps a slot
        /// pointing at a destroyed object.</summary>
        private static void DestroyTablet(GameObject item)
        {
            var spec = item.GetComponent<InventoryItemSpec>();
            if (spec != null)
            {
                spec.isEssential = false;
                spec.ImmuneToDumpster = false;
            }
            Inventory.Instance?.DropItemFromHandsOrInventory(item);
            UnityEngine.Object.Destroy(item);
        }

        /// <summary>Every tablet in the world that this manager did not make.
        /// After a reload that is all of them: the manager is new, its list is
        /// empty, and whatever is still in the bag is last build's dead board.
        /// Testing the marker's TYPE could not see that, because a reload that
        /// does not change the assembly version hands back the same types.</summary>
        private void SweepStaleTablets()
        {
            var live = new HashSet<GameObject>();
            foreach (var m in tablets)
                if (m != null)
                    live.Add(m.gameObject);
            foreach (var spec in Resources.FindObjectsOfTypeAll<InventoryItemSpec>())
            {
                if (spec == null || !spec.gameObject.scene.IsValid())
                    continue;
                if (live.Contains(spec.gameObject) || !IsTablet(spec))
                    continue;
                DestroyTablet(spec.gameObject);
                Main.mod?.Logger.Log("[TABLET] removed a stale dispatcher tablet");
            }
        }

        /// <summary>The mod is going away — take our tablets with it. What the
        /// player had is in the save data, so the next load (or the reload that
        /// is about to happen) puts a fresh one back.</summary>
        private void OnDestroy()
        {
            foreach (var m in tablets)
                if (m != null)
                    DestroyTablet(m.gameObject);
            tablets.Clear();
        }

        /// <summary>A home-menu tile: a rounded square OUTLINE and nothing
        /// inside it. Drawn white, so the renderer's own colour is free to dim
        /// it or bring it up under the cursor. One texture serves every tile on
        /// every tablet.</summary>
        private static Sprite? tileSprite;

        private static Sprite TileSprite()
        {
            if (tileSprite != null)
                return tileSprite;
            const int N = 128;
            // Signed-distance coverage of a rounded square centred in the
            // texture, ~1 px antialiased — the outline is the difference
            // between two of them.
            float Cover(float px, float py, float half, float r)
            {
                float dx = Mathf.Abs(px - N / 2f) - (half - r);
                float dy = Mathf.Abs(py - N / 2f) - (half - r);
                float outside = Mathf.Sqrt(
                    Mathf.Max(dx, 0f) * Mathf.Max(dx, 0f)
                    + Mathf.Max(dy, 0f) * Mathf.Max(dy, 0f));
                return Mathf.Clamp01(0.5f
                    - (outside + Mathf.Min(Mathf.Max(dx, dy), 0f) - r));
            }
            const float Half = 62f;    // a pixel of margin for the antialiasing
            const float Radius = 16f;
            const float Border = 3f;
            var tex = new Texture2D(N, N, TextureFormat.RGBA32, false);
            var pixels = new Color[N * N];
            for (int y = 0; y < N; y++)
                for (int x = 0; x < N; x++)
                    pixels[y * N + x] = new Color(1f, 1f, 1f, Mathf.Clamp01(
                        Cover(x, y, Half, Radius)
                        - Cover(x, y, Half - Border, Mathf.Max(1f, Radius - Border))));
            tex.SetPixels(pixels);
            tex.Apply(false, true);
            // N pixels to the unit: the sprite is one unit across, and the
            // renderer's scale is then the tile's side in tablet units.
            tileSprite = Sprite.Create(tex, new Rect(0, 0, N, N),
                new Vector2(0.5f, 0.5f), N);
            return tileSprite;
        }

        /// <summary>A GO/STOP button: the same rounded outline as a menu tile,
        /// drawn long and low because that is the shape of half a row. Stretched
        /// to fit, so it is generated at roughly the aspect it is used at.</summary>
        /// <summary>Button outlines, one texture per SHAPE. One long texture
        /// stretched to fit was what made the short buttons look wrong: the
        /// squeeze drags the corner radius and the border width with it, so a
        /// wide-and-flat sprite pushed into a stubby button came out a fat
        /// capsule. Each aspect gets its own, drawn at that aspect and cached
        /// by it, so the border is one width all the way round and the corners
        /// are the same radius top and bottom as they are left and right.</summary>
        private static readonly Dictionary<int, Sprite> buttonSprites =
            new Dictionary<int, Sprite>();

        private static Sprite ButtonSprite(float aspect)
        {
            // Quarter-step buckets: a handful of textures covers every button
            // on the glass, and two buttons a hair apart share one.
            int key = Mathf.Clamp(Mathf.RoundToInt(aspect * 4f), 4, 48);
            if (buttonSprites.TryGetValue(key, out var cached) && cached != null)
                return cached;
            int h = 96;
            int w = Mathf.Clamp(Mathf.RoundToInt(h * (key / 4f)), h, 1536);
            float Cover(float px, float py, float hw, float hh, float r)
            {
                float dx = Mathf.Abs(px - w / 2f) - (hw - r);
                float dy = Mathf.Abs(py - h / 2f) - (hh - r);
                float outside = Mathf.Sqrt(
                    Mathf.Max(dx, 0f) * Mathf.Max(dx, 0f)
                    + Mathf.Max(dy, 0f) * Mathf.Max(dy, 0f));
                return Mathf.Clamp01(0.5f
                    - (outside + Mathf.Min(Mathf.Max(dx, dy), 0f) - r));
            }
            const float Border = 4f;
            float radius = h * 0.22f;
            float halfW = w / 2f - 1f;
            float halfH = h / 2f - 1f;
            var tex = new Texture2D(w, h, TextureFormat.RGBA32, false);
            var pixels = new Color[w * h];
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                    pixels[y * w + x] = new Color(1f, 1f, 1f, Mathf.Clamp01(
                        Cover(x, y, halfW, halfH, radius)
                        - Cover(x, y, halfW - Border, halfH - Border,
                            Mathf.Max(1f, radius - Border))));
            tex.SetPixels(pixels);
            tex.Apply(false, true);
            var sprite = Sprite.Create(tex, new Rect(0, 0, w, h),
                new Vector2(0.5f, 0.5f), w);
            buttonSprites[key] = sprite;
            return sprite;
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

        /// <summary>Make a tablet and put it in the player's bag.
        /// <paramref name="wantSlot"/> is the slot a tablet restored from the
        /// save came out of; -1 asks for the first free one.</summary>
        private void GiveTablet(int wantSlot = -1)
        {
            if (tablets.Any(m => m != null))
            {
                Main.mod?.Logger.Log("A dispatcher tablet already exists");
                return;
            }
            var inv = Inventory.Instance;
            if (inv == null)
                return;
            int slot = wantSlot >= 0 && inv.IsSlotEmpty(wantSlot) ? wantSlot : -1;
            if (slot < 0)
                slot = inv.GetFirstFreeHotbarSlot();
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

            // Keep the clone out of the save file. The game stores an item by
            // its PREFAB name and respawns it by loading that prefab, so a
            // saved tablet came back as the plain map it was cloned from; and
            // the map being a basic starting item, the loader then struck one
            // of the two down as an illegal duplicate ("Found a basic item or
            // license item duplicate: Map. Removing it."). That is why the
            // tablet was gone after every restart. The screen is ours to
            // rebuild anyway, so the item is excluded from every storage the
            // serializer walks and put back by RestoreTablet on load.
            var control = go.GetComponent<DV.CabControls.Spec.Item>();
            if (control != null)
                control.excludeFromStorageSerialization =
                    StorageType.Inventory | StorageType.LostAndFound
                    | StorageType.World | StorageType.InstalledGadgets
                    | StorageType.ItemContainers;

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
            //
            // The markers it has ALREADY made go with it. MapMarkersController
            // builds one per fast-travel destination in Awake, which has run by
            // the time Instantiate returns, and every marker carries a DV button
            // that opens the fast travel menu when it is used. Invisible under
            // the board, they were still taking the clicks meant for its own
            // buttons: pressing GO fast-travelled. Destroying the controller
            // alone left them behind, so the marker objects are hunted down the
            // whole hierarchy rather than only on the root.
            foreach (var comp in go.GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (comp == null)
                    continue;
                string kind = comp.GetType().Name;
                if (kind == "MapMarkersController")
                    UnityEngine.Object.Destroy(comp);
                else if (kind == "MapMarker")
                    UnityEngine.Object.Destroy(comp.gameObject);
            }
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
            // 0.2 mm of height and 0.8 mm of width put these lines under a pixel
            // on a screen held at arm's length, and a sub-pixel edge crawls as
            // the hands move — the flicker over the bar. Thicker and further off
            // the glass, they cover a pixel and hold still.
            const float LineT = 0.0007f;
            const float LineY = 0.0033f;
            var dividerV = Piece("TabletDividerV", World(LineT, 0.0003f, 0.0935f),
                new Vector3(0f, LineY, 0f), lineColor);
            var dividerH = Piece("TabletDividerH", World(0.1315f, 0.0003f, LineT),
                new Vector3(0f, LineY, 0.0935f / 2f - QuadH), lineColor);
            // The one-locomotive page's bottom bar is fenced off from the rows
            // above it by a line of its own, the width of the glass.
            var barLine = Piece("TabletBarLine", World(0.1315f, 0.0003f, LineT),
                new Vector3(0f, LineY, -(0.0935f / 2f - BarH)), lineColor);
            barLine.SetActive(false);
            if (DebugBands)
            {
                // A line at every band EDGE, down the left margin, so what the
                // mouse thinks a row is can be read straight off the glass.
                float dbgPitch = (ScreenH - BarH) / Rows;
                for (int r = 0; r <= Rows; r++)
                    Piece($"TabletDebugBand{r}",
                        World(0.02f, 0.0003f, 0.0003f),
                        new Vector3(-0.055f, LineY,
                            0.0935f / 2f - r * dbgPitch),
                        new Color(0.8f, 0.15f, 0.15f));
            }
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
            // What the ray actually lands on: a plate the width of a thumb,
            // filling the bezel strip between the glass edge and the body edge
            // so the button can be hit without hunting for the disc. Invisible,
            // and kept clear of the glass so it never steals a row's click.
            var homePlate = Piece("TabletHomeHit",
                World(0.016f, 0.004f, 0.005f), new Vector3(0, 0.0031f, -0.0497f),
                Color.black, keepCollider: true);
            homePlate.GetComponent<Renderer>().enabled = false;
            homePlate.GetComponent<Collider>().isTrigger = true;

            var marker = go.AddComponent<TabletMarker>();
            marker.model = model;
            marker.screenPiece = screenPiece.transform;
            marker.dividerV = dividerV;
            marker.dividerH = dividerH;
            marker.homeButton = homeButton.transform;
            marker.homeRenderer = homeRenderer;
            marker.homePlate = homePlate.transform;
            marker.homeRest = homeButton.transform.localPosition;
            marker.barLine = barLine;

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
                tm.alignment =
                    anchor == TextAnchor.UpperLeft || anchor == TextAnchor.MiddleLeft
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

            // A flat sprite on the window plane, sized in tablet units: the
            // anchor carries the orientation and undoes the item's own scaling,
            // the child carries the size.
            SpriteRenderer MakeSprite(string name, float worldX, float worldZ,
                float width, float height, Sprite art)
            {
                var spriteAnchor = new GameObject(name + "Anchor");
                spriteAnchor.transform.SetParent(model, true);
                spriteAnchor.layer = layer;
                spriteAnchor.transform.position = dial.position
                    + dial.up * 0.0033f + dial.right * (worldX * S)
                    + dial.forward * (worldZ * S);
                spriteAnchor.transform.rotation =
                    dial.rotation * Quaternion.Euler(90f, 0f, 0f);
                spriteAnchor.transform.localScale = new Vector3(
                    1f / Mathf.Max(0.0001f, pl.x),
                    1f / Mathf.Max(0.0001f, pl.y),
                    1f / Mathf.Max(0.0001f, pl.z));
                var spriteGo = new GameObject(name);
                spriteGo.transform.SetParent(spriteAnchor.transform, false);
                spriteGo.layer = layer;
                var size = art.bounds.size;
                spriteGo.transform.localScale = new Vector3(
                    width * S / Mathf.Max(0.0001f, size.x),
                    height * S / Mathf.Max(0.0001f, size.y), 1f);
                var renderer = spriteGo.AddComponent<SpriteRenderer>();
                renderer.sprite = art;
                renderer.shadowCastingMode =
                    UnityEngine.Rendering.ShadowCastingMode.Off;
                renderer.receiveShadows = false;
                renderer.motionVectorGenerationMode =
                    MotionVectorGenerationMode.ForceNoMotion;
                renderer.enabled = false;
                return renderer;
            }

            // 2 x 2 grid: each window is anchored at the TOP of its quarter.
            const float halfW = 0.1315f / 2f;
            const float halfH = 0.0935f / 2f;
            for (int i = 0; i < TabletPins.Max; i++)
            {
                // Anchored at the top-LEFT of each quadrant. The pairs split
                // the ROWS AREA — the glass less the bottom bar — so the bar
                // takes its strip out of the bottom pair instead of hanging off
                // the screen.
                float x = (i % 2 == 0 ? -halfW : 0f) + 0.004f;
                float z = i < 2 ? halfH : halfH - QuadH;
                // A row per band, each in the MIDDLE of its own, so what the
                // cursor lights and what the glass prints are the same row.
                float quadPitch = QuadH / Rows;
                for (int r = 0; r < Rows; r++)
                {
                    float rz = z - (r + 0.5f) * quadPitch;
                    marker.windowRows[i][r] = MakeText($"TabletWindow{i}_{r}",
                        x, rz, RowCharSize * SingleScale, TextAnchor.MiddleLeft);
                    // The value column: same row, fixed x — a two-column table.
                    marker.valueRows[i][r] = MakeText($"TabletValues{i}_{r}",
                        x + ValueColX * SingleScale, rz,
                        RowCharSize * SingleScale, TextAnchor.MiddleLeft);
                }

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

                // GO and STOP fill the two halves of their row — the same two
                // halves the click is read against — with the word centred in
                // each. The band is measured off the quadrant's own top, not
                // the text's, so the outline sits on the hitbox exactly.
                float rowH = QuadH / Rows;
                float bandZ = (i < 2 ? halfH : halfH - QuadH)
                    - (GoRow + 0.5f) * rowH;
                float x0 = i % 2 == 0 ? -halfW : 0f;
                float bw = halfW / 2f - 0.006f;
                float bh = rowH * 0.8f;
                marker.goTiles[i] = MakeSprite($"TabletGo{i}",
                    x0 + halfW / 4f, bandZ, bw, bh, ButtonSprite(bw / bh));
                marker.stopTiles[i] = MakeSprite($"TabletStop{i}",
                    x0 + halfW * 3f / 4f, bandZ, bw, bh, ButtonSprite(bw / bh));
                marker.flashes[i] = MakeText($"TabletFlash{i}",
                    x0 + halfW / 2f, bandZ - rowH * 1.15f,
                    RowCharSize * 0.95f, TextAnchor.MiddleCenter);
                marker.goLabels[i] = MakeText($"TabletGoLabel{i}",
                    x0 + halfW / 4f, bandZ,
                    RowCharSize * SingleScale, TextAnchor.MiddleCenter);
                marker.stopLabels[i] = MakeText($"TabletStopLabel{i}",
                    x0 + halfW * 3f / 4f, bandZ,
                    RowCharSize * SingleScale, TextAnchor.MiddleCenter);
            }

            // The home menu: two tiles side by side, each a rounded outline with
            // its name in the middle. Same plane as the windows, the outline a
            // hair under the caption so the letters always draw over it.
            for (int i = 0; i < TabletMarker.Tiles; i++)
            {
                float tx = TileX(i);
                var tileAnchor = new GameObject($"TabletTileAnchor{i}");
                tileAnchor.transform.SetParent(model, true);
                tileAnchor.layer = layer;
                tileAnchor.transform.position = dial.position
                    + dial.up * 0.0033f + dial.right * (tx * S);
                tileAnchor.transform.rotation =
                    dial.rotation * Quaternion.Euler(90f, 0f, 0f);
                tileAnchor.transform.localScale = new Vector3(
                    1f / Mathf.Max(0.0001f, pl.x),
                    1f / Mathf.Max(0.0001f, pl.y),
                    1f / Mathf.Max(0.0001f, pl.z));
                var tileGo = new GameObject($"TabletTile{i}");
                tileGo.transform.SetParent(tileAnchor.transform, false);
                tileGo.layer = layer;
                tileGo.transform.localScale = Vector3.one * (TileSide * S);
                var tileRenderer = tileGo.AddComponent<SpriteRenderer>();
                tileRenderer.sprite = TileSprite();
                tileRenderer.shadowCastingMode =
                    UnityEngine.Rendering.ShadowCastingMode.Off;
                tileRenderer.receiveShadows = false;
                tileRenderer.motionVectorGenerationMode =
                    MotionVectorGenerationMode.ForceNoMotion;
                tileRenderer.enabled = false;
                marker.tiles[i] = tileRenderer;
                marker.tileLabels[i] = MakeText($"TabletTileLabel{i}", tx, 0f,
                    RowCharSize, TextAnchor.MiddleCenter);
            }

            // One locomotive to a page: the same two columns over the whole
            // glass at twice the size, less the strip the bottom bar takes.
            float onePitch = (ScreenH - BarH) / Rows;
            for (int r = 0; r < Rows; r++)
            {
                float rz = halfH - (r + 0.5f) * onePitch;
                marker.singleRows[r] = MakeText($"TabletSingleWindow{r}",
                    -halfW + 0.004f, rz,
                    RowCharSize * 2f * SingleScale, TextAnchor.MiddleLeft);
                marker.singleVals[r] = MakeText($"TabletSingleValues{r}",
                    -halfW + 0.004f + ValueColX * 2f * SingleScale, rz,
                    RowCharSize * 2f * SingleScale, TextAnchor.MiddleLeft);
            }

            // The same two buttons across the whole glass, for the page that
            // shows one locomotive on it.
            float singleRowH = (ScreenH - BarH) / Rows;
            float singleBandZ = halfH - (GoRow + 0.5f) * singleRowH;
            // Same box as every other button on the glass: the height of the
            // bar's, the same word size in it, and the same 3 mm margin off the
            // edge that BACK keeps — sized off its own ROW, this pair came out
            // shallower than the rest and its words came out bigger.
            const float SingleGap = 0.005f;
            float singleBw = halfW - 0.003f - SingleGap / 2f;
            float singleCx = SingleGap / 2f + singleBw / 2f;
            float singleBh = BarH * 0.68f;
            marker.singleGoTile = MakeSprite("TabletSingleGo",
                -singleCx, singleBandZ, singleBw, singleBh,
                ButtonSprite(singleBw / singleBh));
            marker.singleStopTile = MakeSprite("TabletSingleStop",
                singleCx, singleBandZ, singleBw, singleBh,
                ButtonSprite(singleBw / singleBh));
            // BACK and the mode tag are buttons, not table cells: each sits in
            // the middle of the half of the header row that answers for it —
            // the same halves the click is read against.
            // The header pair is the same box as the bar's buttons, and it has
            // to be: shrinking it to dodge the rounded corner made it a
            // different button to every other one on the glass. It clears the
            // corner by sitting LOWER instead, over the header row and the
            // blank one under it — which is why both of those bands answer for
            // it in HandleMouse.
            float headZ = halfH - BarH * 0.72f / 2f - 0.0025f;
            // Hard against the left edge, where the way out belongs, with its
            // word centred in the box as every other button's is.
            // The header pair lives in the TOP CORNERS, where the glass is
            // rounded away: a box drawn to the same margin the bar's buttons use
            // hung off the glass and onto the bezel. Sized and inset to what the
            // corner actually leaves — 5.5 mm off the edge, and short enough to
            // stay inside the header's own click band.
            const float BackW = 0.026f;
            const float BackMargin = 0.004f;
            float backH = BarH * 0.72f;
            float backX = -(halfW - BackMargin - BackW / 2f);
            marker.singleBackTile = MakeSprite("TabletSingleBackTile",
                backX, headZ, BackW, backH, ButtonSprite(BackW / backH));
            marker.singleBack = MakeText("TabletSingleBack",
                backX, headZ,
                RowCharSize * 1.4f, TextAnchor.MiddleCenter);
            float modeX = -backX;
            marker.singleModeTile = MakeSprite("TabletSingleModeTile",
                modeX, headZ, BackW, backH, ButtonSprite(BackW / backH));
            marker.singleMode = MakeText("TabletSingleMode", modeX, headZ,
                RowCharSize * 1.4f, TextAnchor.MiddleCenter);
            // The consist strip sits between the last status row and the
            // buttons, on one anchor with a slot per car hanging off it.
            var consistAnchor = new GameObject("TabletConsistAnchor");
            consistAnchor.transform.SetParent(model, true);
            consistAnchor.layer = layer;
            consistAnchor.transform.position = dial.position
                + dial.up * 0.0034f
                + dial.forward * ((singleBandZ + singleRowH * 1.4f) * S);
            consistAnchor.transform.rotation =
                dial.rotation * Quaternion.Euler(90f, 0f, 0f);
            consistAnchor.transform.localScale = new Vector3(
                1f / Mathf.Max(0.0001f, pl.x),
                1f / Mathf.Max(0.0001f, pl.y),
                1f / Mathf.Max(0.0001f, pl.z));
            for (int i = 0; i < TabletManagerConsist.Max; i++)
            {
                var carGo = new GameObject($"TabletConsist{i}");
                carGo.transform.SetParent(consistAnchor.transform, false);
                carGo.layer = layer;
                var carSlot = carGo.AddComponent<SpriteRenderer>();
                carSlot.shadowCastingMode =
                    UnityEngine.Rendering.ShadowCastingMode.Off;
                carSlot.receiveShadows = false;
                carSlot.motionVectorGenerationMode =
                    MotionVectorGenerationMode.ForceNoMotion;
                carSlot.enabled = false;
                marker.consist[i] = carSlot;
            }

            marker.consistMore = MakeText("TabletConsistMore",
                ConsistW / 2f + ConsistMoreW / 2f,
                singleBandZ + singleRowH * 1.4f,
                RowCharSize * 1.3f, TextAnchor.MiddleCenter);
            marker.consistMore.color = new Color(1f, 1f, 1f, 0.78f);

            // Between the buttons and the bar, across the glass.
            marker.singleFlash = MakeText("TabletSingleFlash", 0f,
                (singleBandZ - singleBh / 2f + (-halfH + BarH)) / 2f,
                RowCharSize * 1.6f * SingleScale, TextAnchor.MiddleCenter);
            marker.singleGoLabel = MakeText("TabletSingleGoLabel",
                -singleCx, singleBandZ,
                RowCharSize * 1.4f, TextAnchor.MiddleCenter);
            marker.singleStopLabel = MakeText("TabletSingleStopLabel",
                singleCx, singleBandZ,
                RowCharSize * 1.4f, TextAnchor.MiddleCenter);

            // The bar itself: an arrow hard against each end, and the machine's
            // number and type in the middle with its icon beside them.
            float barZ = -halfH + BarH / 2f;
            // Each end button fills its zone from the zone's inner boundary out
            // to a margin short of the glass. The margin is not decoration: the
            // glass is rounded, and down at the bar the corner has already eaten
            // into the width — a box drawn to the nominal edge hung off it.
            float endInner = (0.5f - BarArrowZone) * ScreenW;
            float endOuter = halfW - 0.007f;
            float barRightX = (endInner + endOuter) / 2f;
            float barLeftX = -barRightX;
            // One box per end, filling most of the zone that end answers for.
            float endW = endOuter - endInner;
            float endH = BarH * 0.72f;
            var endArt = ButtonSprite(endW / endH);
            marker.barPrevTile = MakeSprite("TabletBarPrevTile",
                barLeftX, barZ, endW, endH, endArt);
            marker.barNextTile = MakeSprite("TabletBarNextTile",
                barRightX, barZ, endW, endH, endArt);
            // BACK needs more room than an arrow: the word is six characters
            // wide and an arrow is one, and sharing the arrow's box left the
            // letters hanging out of both ends of it.
            const float BarBackW = 0.026f;
            float barBackX = -halfW + 0.003f + BarBackW / 2f;
            marker.barBackTile = MakeSprite("TabletBarBackTile",
                barBackX, barZ, BarBackW, endH, ButtonSprite(BarBackW / endH));
            marker.barPrev = MakeText("TabletBarPrev", barLeftX, barZ,
                RowCharSize * 2f, TextAnchor.MiddleCenter);
            marker.barNext = MakeText("TabletBarNext", barRightX, barZ,
                RowCharSize * 2f, TextAnchor.MiddleCenter);
            // The name sits left of centre now: the right of the bar is
            // UNPIN's, between the name and the paging arrow.
            marker.barLabel = MakeText("TabletBarLabel", -0.008f, barZ,
                RowCharSize * 1.4f, TextAnchor.MiddleCenter);
            float unpinX = (BarUnpinLo + BarUnpinHi) / 2f * ScreenW;
            float unpinW = (BarUnpinHi - BarUnpinLo) * ScreenW * 0.92f;
            float unpinH = BarH * 0.72f;
            marker.barUnpinTile = MakeSprite("TabletBarUnpin", unpinX, barZ,
                unpinW, unpinH, ButtonSprite(unpinW / unpinH));
            marker.barUnpinLabel = MakeText("TabletBarUnpinLabel", unpinX, barZ,
                RowCharSize * 1.1f, TextAnchor.MiddleCenter);
            marker.barBack = MakeText("TabletBarBack", barBackX, barZ,
                RowCharSize * 1.4f, TextAnchor.MiddleCenter);
            var barIconAnchor = new GameObject("TabletBarIconAnchor");
            barIconAnchor.transform.SetParent(model, true);
            barIconAnchor.layer = layer;
            barIconAnchor.transform.position = dial.position
                + dial.up * 0.0034f + dial.right * (-0.030f * S)
                + dial.forward * (barZ * S);
            barIconAnchor.transform.rotation =
                dial.rotation * Quaternion.Euler(90f, 0f, 0f);
            barIconAnchor.transform.localScale = new Vector3(
                1f / Mathf.Max(0.0001f, pl.x),
                1f / Mathf.Max(0.0001f, pl.y),
                1f / Mathf.Max(0.0001f, pl.z));
            var barIconGo = new GameObject("TabletBarIcon");
            barIconGo.transform.SetParent(barIconAnchor.transform, false);
            barIconGo.layer = layer;
            var barIcon = barIconGo.AddComponent<SpriteRenderer>();
            barIcon.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            barIcon.receiveShadows = false;
            barIcon.motionVectorGenerationMode = MotionVectorGenerationMode.ForceNoMotion;
            barIcon.enabled = false;
            marker.barIcon = barIcon;

            marker.emptyMsg = MakeText("TabletEmpty", 0f, 0f,
                0.00075f, TextAnchor.MiddleCenter);

            tablets.Add(marker);
            int result = inv.AddItemToInventory(go, slot, false);
            Main.mod?.Logger.Log($"Dispatcher tablet added to slot {result}");
            // Written now as well as on save, so a mid-session mod reload —
            // which destroys the item and sweeps what is left — brings the
            // tablet back rather than losing it until the next new one.
            WriteSaveState();
        }
    }
}
