using System.Collections.Generic;
using LocoSim.Implementations;

namespace DvMod.Autopilot
{
    /// <summary>
    /// Reading a simulation port without shouting about the ones that are not
    /// there.
    ///
    /// The game's own SimulationFlow.TryGetPort writes
    ///
    ///     LocoDR4(Clone).[sim]: Port[gearInputA.CONTROL_EXT_IN] not defined!
    ///
    /// to Player.log — as an ERROR, so it also lands in the on-screen console —
    /// every single time a port is asked for and missing. That is fine for the
    /// game, which only asks for ports it knows a loco has, and useless for us:
    /// asking IS how we tell one machine from another (has it a gearbox? a
    /// traction-motor fuse? a temperature port?), and the drive loop asks ten
    /// times a second. One diesel-hydraulic in the consist and the log is
    /// nothing but "not defined!" for the length of the run.
    ///
    /// The lookup underneath it is a plain dictionary, so this goes to that
    /// dictionary directly: same answer, no log line, no per-loco "ports we
    /// already know are missing" caches to keep.
    /// </summary>
    public static class Ports
    {
        /// <summary>The port, or null when this car has no such port (or no
        /// simulation at all — a caboose, a flatbed, a car being unloaded).</summary>
        public static Port? Find(TrainCar? car, string? id) =>
            Find(car?.SimController?.simFlow, id);

        public static Port? Find(SimulationFlow? flow, string? id)
        {
            if (flow == null || id == null || id.Length == 0)
                return null;
            IReadOnlyDictionary<string, Port> byId = flow.fullPortIdToPort;
            return byId != null && byId.TryGetValue(id, out var port) ? port : null;
        }

        public static bool Has(TrainCar? car, string id) => Find(car, id) != null;

        /// <summary>The reading, or NaN when the port is absent — a value that
        /// cannot be mistaken for a measurement, unlike 0.</summary>
        public static float Read(TrainCar? car, string id) =>
            Find(car, id)?.Value ?? float.NaN;

        /// <summary>True when there was a port to write to.</summary>
        public static bool Write(TrainCar? car, string id, float value)
        {
            var port = Find(car, id);
            if (port == null)
                return false;
            port.Value = value;
            return true;
        }
    }
}
