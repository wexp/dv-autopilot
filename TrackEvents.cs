using DV.Logic.Job;
using System.Collections.Generic;
using System.Linq;

namespace DvMod.Autopilot
{
    // Track-ahead events (named tracks, junctions, speed limits, grades), ported
    // from dv-hud so the HUD reads the same posted signs the player sees rather
    // than deriving limits from track geometry. See TrackIndexer / TrackFollower.

    public abstract class TrackEvent
    {
        public readonly double span;

        protected TrackEvent(double span) => this.span = span;

        public abstract TrackEvent WithSpan(double span, bool direction);

        public abstract bool Direction { get; }
    }

    public class TrackChangeEvent : TrackEvent
    {
        public readonly TrackID ID;

        public TrackChangeEvent(double span, TrackID id) : base(span) => ID = id;

        public override TrackEvent WithSpan(double span, bool direction) => new TrackChangeEvent(span, ID);

        public override bool Direction => true;
    }

    public class JunctionEvent : TrackEvent
    {
        public readonly bool direction;
        public readonly Junction junction;

        public JunctionEvent(double span, bool direction, Junction junction) : base(span)
        {
            this.direction = direction;
            this.junction = junction;
        }

        public override TrackEvent WithSpan(double span, bool direction) => new JunctionEvent(span, direction, junction);

        public override bool Direction => direction;
    }

    public class SpeedLimitEvent : TrackEvent
    {
        private readonly bool direction;
        public readonly int limit;

        public SpeedLimitEvent(double span, bool direction, int limit) : base(span)
        {
            this.direction = direction;
            this.limit = limit;
        }

        public override TrackEvent WithSpan(double span, bool direction) => new SpeedLimitEvent(span, direction, limit);

        public override bool Direction => direction;
    }

    public class DualSpeedLimitEvent : SpeedLimitEvent
    {
        public readonly int rightLimit;

        public DualSpeedLimitEvent(double span, bool direction, int limit, int rightLimit)
            : base(span, direction, limit) => this.rightLimit = rightLimit;

        public override TrackEvent WithSpan(double span, bool direction) =>
            new DualSpeedLimitEvent(span, direction, limit, rightLimit);
    }

    public class GradeEvent : TrackEvent
    {
        public override bool Direction { get; }
        public readonly float grade;

        public GradeEvent(double span, bool direction, float grade) : base(span)
        {
            Direction = direction;
            this.grade = grade;
        }

        public override TrackEvent WithSpan(double span, bool direction) => new GradeEvent(span, direction, grade);
    }

    public static class TrackEventsExtension
    {
        /// <summary>Re-base an indexed track's events onto the entry span, dropping
        /// the ones already behind, and mark each as facing travel or not.</summary>
        public static IEnumerable<TrackEvent> RelativeFromSpan(
            this IEnumerable<TrackEvent> events, double startSpan, bool direction)
        {
            foreach (var ev in direction ? events : events.Reverse())
            {
                double relativeSpan = direction ? ev.span - startSpan : startSpan - ev.span;
                if (relativeSpan >= 0)
                    yield return ev.WithSpan(relativeSpan, ev.Direction == direction);
            }
        }

        public static IEnumerable<TrackEvent> Offset(this IEnumerable<TrackEvent> events, double offset)
        {
            foreach (var ev in events)
                yield return ev.WithSpan(ev.span + offset, ev.Direction);
        }

        /// <summary>Pick a junction's dual limit apart onto the branch that is set.</summary>
        public static IEnumerable<TrackEvent> ResolveJunctionSpeedLimits(this IEnumerable<TrackEvent> events)
        {
            int i = 1;
            foreach (var ev in events)
            {
                if (ev is DualSpeedLimitEvent e)
                {
                    if (!e.Direction)
                    {
                        i++;
                        continue;
                    }
                    var nextJunctionEvent = events.Skip(i).OfType<JunctionEvent>().FirstOrDefault(j => j.Direction);
                    if (nextJunctionEvent == null)
                        yield return ev;
                    else if (nextJunctionEvent.junction.selectedBranch == 0)
                        yield return new SpeedLimitEvent(e.span, true, e.limit);
                    else
                        yield return new SpeedLimitEvent(e.span, true, e.rightLimit);
                }
                else
                {
                    yield return ev;
                }
                i++;
            }
        }

        /// <summary>Drop reversed and repeated speed limits.</summary>
        public static IEnumerable<TrackEvent> FilterRedundantSpeedLimits(this IEnumerable<TrackEvent> events)
        {
            int prevSpeedLimit = -1;
            foreach (var ev in events)
            {
                if (ev is SpeedLimitEvent sle)
                {
                    if (sle.Direction && sle.limit != prevSpeedLimit)
                    {
                        prevSpeedLimit = sle.limit;
                        yield return sle;
                    }
                }
                else
                {
                    yield return ev;
                }
            }
        }

        public static IEnumerable<TrackEvent> FilterGradeEvents(this IEnumerable<TrackEvent> events, float prevGrade)
        {
            foreach (var ev in events)
            {
                if (ev is GradeEvent gradeEvent)
                {
                    if (ev.Direction && prevGrade != gradeEvent.grade)
                    {
                        prevGrade = gradeEvent.grade;
                        yield return gradeEvent;
                    }
                }
                else
                {
                    yield return ev;
                }
            }
        }

        public static IEnumerable<TrackEvent> ExceptUnnamedTracks(this IEnumerable<TrackEvent> events) =>
            events.Where(ev => !(ev is TrackChangeEvent tce) || !tce.ID.IsGeneric());
    }
}
