using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Threading.Tasks;
using UnityEngine;

namespace DvMod.Autopilot
{
    /// <summary>
    /// The mod's persistent object: hosts the autopilot's coroutines and a
    /// main-thread work queue. Trimmed from remote-dispatch's Updater — no
    /// player/car polling here, only what driving needs.
    /// </summary>
    public class Updater : MonoBehaviour
    {
        private static Updater? instance;
        private static int mainThreadId = -1;

        public void Start()
        {
            instance = this;
            StartCoroutine(DeferredEventsCoro());
        }

        /// <summary>Called from Main.Load (always the main thread), so the inline
        /// path below works from the first frame — a caller blocking on Result
        /// before the Updater exists would otherwise hang forever.</summary>
        public static void CaptureMainThread()
        {
            mainThreadId = System.Threading.Thread.CurrentThread.ManagedThreadId;
        }

        /// <summary>Already on the game's main thread? Then a queued task would
        /// DEADLOCK any caller that blocks on its Result — the queue only drains
        /// on this very thread. In-game UIs call the same entry points the HTTP
        /// server does; for them the work just runs inline.</summary>
        private static bool OnMainThread =>
            System.Threading.Thread.CurrentThread.ManagedThreadId == mainThreadId;

        /// <summary>Start a coroutine on the mod's persistent object. Returns null
        /// if the Updater hasn't spun up yet.</summary>
        public static Coroutine? Run(IEnumerator routine)
        {
            return instance != null ? instance.StartCoroutine(routine) : null;
        }

        /// <summary>Stop a coroutine previously started with <see cref="Run"/>.</summary>
        public static void StopRun(Coroutine routine)
        {
            if (instance != null && routine != null)
                instance.StopCoroutine(routine);
        }

        private static GameObject? rootObject;

        public static void Create()
        {
            if (rootObject == null)
            {
                rootObject = new GameObject();
                GameObject.DontDestroyOnLoad(rootObject);
                rootObject.AddComponent<Updater>();
            }
        }

        public static void Destroy()
        {
            if (rootObject != null)
            {
                GameObject.Destroy(rootObject);
                rootObject = null;
            }
        }

        private IEnumerator DeferredEventsCoro()
        {
            while (true)
            {
                while (taskQueue.TryDequeue(out var action))
                    action();
                yield return null;
            }
        }

        private static readonly ConcurrentQueue<Action> taskQueue = new ConcurrentQueue<Action>();

        public static Task RunOnMainThread(Action action)
        {
            var tcs = new TaskCompletionSource<bool>();
            if (OnMainThread)
            {
                try
                {
                    action();
                    tcs.SetResult(true);
                }
                catch (Exception e)
                {
                    tcs.SetException(e);
                }
                return tcs.Task;
            }
            taskQueue.Enqueue(() =>
            {
                try
                {
                    action();
                    tcs.SetResult(true);
                }
                catch (Exception e)
                {
                    tcs.SetException(e);
                }
            });
            return tcs.Task;
        }

        public static Task<T> RunOnMainThread<T>(Func<T> func)
        {
            var tcs = new TaskCompletionSource<T>();
            if (OnMainThread)
            {
                try
                {
                    tcs.SetResult(func());
                }
                catch (Exception e)
                {
                    tcs.SetException(e);
                }
                return tcs.Task;
            }
            taskQueue.Enqueue(() =>
            {
                try
                {
                    tcs.SetResult(func());
                }
                catch (Exception e)
                {
                    tcs.SetException(e);
                }
            });
            return tcs.Task;
        }
    }
}
