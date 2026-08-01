using Newtonsoft.Json.Linq;

namespace DvMod.Autopilot
{
    /// <summary>
    /// The moves an autopilot run is made of. A route used to be an opaque list of
    /// tracks that the drive loop interpreted as it went, which meant the only way to
    /// find out what it intended was to watch the train do it — and a train that
    /// stopped somewhere odd gave no way to tell a bad plan from bad driving. Naming
    /// the moves splits that question in two: the step list says what was meant, the
    /// run's state says how far it got.
    /// </summary>
    public enum StepKind
    {
        /// <summary>Roll to a track by the shortest road, setting switches ahead.</summary>
        Drive,
        /// <summary>Get the whole consist past a junction — every car clear of it.</summary>
        JunctionCross,
        /// <summary>Clear a junction with the whole consist, stop, throw it, and come
        /// back over it the other way. This is the runaround's pivot.</summary>
        JunctionReverse,
        /// <summary>Leave a track by one end, run round, and come back into the same
        /// track by the other end — the only way to reach the far side of stock we
        /// cannot drive through.</summary>
        TrackReverse,
        /// <summary>Drive fully onto a turntable bridge, stop, wind the table until
        /// the road onward connects, and drive off.</summary>
        Turntable,
    }

    /// <summary>One move in a run's plan. Plain data: the drive loop reports against
    /// these, it does not take orders from them (yet).</summary>
    public sealed class Step
    {
        public StepKind kind;
        /// <summary>Track this step is about, for Drive and TrackReverse.</summary>
        public string track = "";
        /// <summary>Junction number as the map shows it, or -1.</summary>
        public int junction = -1;
        /// <summary>TrackReverse only: come back in by the span-0 end.</summary>
        public bool enterFirst;

        public static Step Drive(string track) =>
            new Step { kind = StepKind.Drive, track = track };

        public static Step Cross(int junction) =>
            new Step { kind = StepKind.JunctionCross, junction = junction };

        public static Step Reverse(int junction) =>
            new Step { kind = StepKind.JunctionReverse, junction = junction };

        public static Step TrackReverse(string track, bool enterFirst) =>
            new Step { kind = StepKind.TrackReverse, track = track, enterFirst = enterFirst };

        public static Step Turntable(string toTrack) =>
            new Step { kind = StepKind.Turntable, track = toTrack };

        /// <summary>What this step would read as on a dispatcher's sheet.</summary>
        public string Describe()
        {
            string j = junction >= 0 ? $"J-{junction}" : "the junction";
            switch (kind)
            {
                case StepKind.Drive:
                    return $"drive to {track}";
                case StepKind.JunctionCross:
                    return $"cross {j} and clear it with the whole train";
                case StepKind.JunctionReverse:
                    return $"clear {j}, stop, throw it, and come back over it";
                case StepKind.TrackReverse:
                    return $"run round and re-enter {track} by its {(enterFirst ? "first" : "last")} end";
                case StepKind.Turntable:
                    return $"ride the turntable round to {track}";
                default:
                    return kind.ToString();
            }
        }

        public JObject ToJson() => new JObject
        {
            ["kind"] = kind.ToString().ToLowerInvariant(),
            ["track"] = track,
            ["junction"] = junction,
            ["text"] = Describe(),
        };
    }
}
