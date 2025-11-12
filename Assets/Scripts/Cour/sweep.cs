using System.Collections.Generic;
using UnityEngine;

public class sweep : MonoBehaviour
{
    [System.Serializable]
    public class Interval
    {
        public string name;
        public float start;
        public float end;

        public Interval(string name, float start, float end)
        {
            this.name = name;
            this.start = start;
            this.end = end;
        }
    }
    public List<Interval> intervals;
    void Start()
    {
        // Example intervals (like I1, I2... from your image)
        intervals = new List<Interval>()
        {
            new Interval("I1", 3, 7),
            new Interval("I2", 7.7f, 18),
            new Interval("I3", 1, 8),
            new Interval("I4", 8.9f, 11.2f),
            new Interval("I5", 7.5f, 11),
            new Interval("I6", 2, 6)
        };

        SweepAndPruneAlgorithm(intervals);
    }

    void SweepAndPruneAlgorithm(List<Interval> intervals)
    {
        // Step 1: Create a list of endpoints
        var endpoints = new List<(float value, bool isStart, Interval interval)>();

        foreach (var interval in intervals)
        {
            endpoints.Add((interval.start, true, interval));
            endpoints.Add((interval.end, false, interval));
        }

        // Step 2: Sort by coordinate value
        endpoints.Sort((a, b) => a.value.CompareTo(b.value));

        // Step 3: Sweep
        List<Interval> active = new List<Interval>();

        foreach (var point in endpoints)
        {
            if (point.isStart)
            {
                // Check overlap with active intervals
                foreach (var activeInterval in active)
                {
                    Debug.Log($"Overlap detected: {point.interval.name} intersects {activeInterval.name}");
                }

                active.Add(point.interval);
            }
            else
            {
                // Remove from active list
                active.Remove(point.interval);
            }
        }
    }
}
