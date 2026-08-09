namespace DvMod.Autopilot
{
    /// <summary>
    /// The surface other mods call — remote-dispatch finds this type by
    /// reflection (UnityModManager.FindMod("Autopilot").Assembly), so every
    /// signature here uses only primitives: enums and custom types would need
    /// the caller to reference this assembly, which would defeat the point of
    /// being an optional addon. Keep names and shapes stable.
    /// </summary>
    public static class Api
    {
        /// <summary>Addon version, so a UI can gate features.</summary>
        public static string Version() => Main.mod?.Info.Version ?? "";

        /// <summary>Start (or re-route) a run. arrive: "shove" | "pull" | anything
        /// = auto. via: junction number as the map shows it, -1 for none. mode:
        /// "shunt" | anything = drive. Returns the run's status JSON.</summary>
        public static string Start(
            string? locoGuid, string dest, float cruiseKmh, bool couple,
            string? arrive, int via, string? mode, string? yard, bool uncouple)
        {
            return Autopilot.Start(
                locoGuid, dest, cruiseKmh, couple, ParseArrive(arrive),
                via < 0 ? (int?)null : via,
                mode == "shunt" ? AutopilotMode.Shunt : AutopilotMode.Drive,
                yard, uncouple);
        }

        /// <summary>Stop one loco's run (guid), or every run when null.</summary>
        public static string Stop(string? locoGuid) => Autopilot.Stop(locoGuid);

        /// <summary>Every run keyed by car guid, as the console shows it.</summary>
        public static string StatusJson() => Autopilot.GetStatusJson();

        /// <summary>Dry-run plan for a move, without driving it.</summary>
        public static string PlanJson(string? locoGuid, string dest, string? arrive) =>
            Autopilot.GetPlanJson(locoGuid, dest, ParseArrive(arrive));

        /// <summary>The junction numbers at either end of a track (for "via" pickers).</summary>
        public static string TrackJunctionsJson(string trackId) =>
            Autopilot.GetTrackJunctionsJson(trackId);

        /// <summary>Picker data: tracks grouped by yard, the running lines serving
        /// each, and which yard the given loco stands in.</summary>
        public static string YardsJson(string? locoGuid) => Yards.GetYardsJson(locoGuid);

        /// <summary>Debug: every run's plan, plus the full block ledger - who holds
        /// which track and junction.</summary>
        public static string PathsJson() => Autopilot.GetPathDebug();

        /// <summary>Brake every run to a stand, synchronously. For anyone about to
        /// do something that would freeze or kill the drive coroutines.</summary>
        public static void SecureAll() => Autopilot.SecureAllForReload();

        /// <summary>Write the rail network and the current scenario to map.json /
        /// state.json in the given folder (the mod's own when null), so the routing
        /// and driving can be run outside the game. Returns a JSON report.</summary>
        public static string DumpMap(string? dir) => MapDump.Write(dir);

        /// <summary>Write every streamed-in loco type's handles and simulation
        /// ports to controls.txt — what a machine has, as opposed to what the
        /// drive loop assumes it has.</summary>
        public static string DumpControls(string? dir) => ControlDump.Write(null, dir);

        /// <summary>Write every gadget fitted to the cars streamed in, and every
        /// gadget prefab loaded, to gadgets.txt.</summary>
        public static string DumpGadgets(string? dir) => GadgetDump.Write(dir);

        private static ArriveMode ParseArrive(string? value) => value switch
        {
            "shove" => ArriveMode.Shove,
            "pull" => ArriveMode.Pull,
            _ => ArriveMode.Auto,
        };
    }
}
