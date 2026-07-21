/**
 * @file Tasks.h
 * @brief Tasks framework — registry-driven, member-fn-ptr-dispatched task bodies.
 * @author WRO 2026
 *
 * @details
 * The Tasks framework lets Mission steps invoke named, parameterised "tasks"
 * (e.g. ``demoEcho``, eventually ``mosaicPlace`` / ``parkInBay`` / ...) without
 * Mission needing to know anything about each task's implementation. A task is
 * a non-static member function on :class:`TasksClass` with the signature
 *
 * @code
 *   bool TasksClass::<name>(const TaskArgs& args);
 * @endcode
 *
 * It receives a 16-byte ``TaskArgs`` POD union packed by the operator-facing
 * ``Mission.task<Args...>(name, args...)`` template (added in a later T-task)
 * and returns ``true`` on success / ``false`` on "yield without abort".
 *
 * The single source of truth for valid task names is the static
 * ``_registry`` array in :file:`Tasks.cpp`. ``runTask`` performs a ``strcmp``
 * linear scan (registry is bounded by ``Config::MAX_TASKS = 16``, so a hash
 * map would be wasteful) and dispatches through a pointer-to-member-function.
 *
 * Lifecycle invariants:
 *   - ``_inTask`` is owned EXCLUSIVELY by ``runTask`` via an RAII scope guard.
 *     No other method (including ``abort()`` / ``clearAbort()``) touches it.
 *     ``runTask`` also READS it on entry to reject reentrant dispatch (a task
 *     body calling ``runTask`` again) — tasks are non-reentrant leaf
 *     operations; see ADR-026.
 *   - ``_aborted`` is a sticky latch settable via ``abort()`` and readable
 *     via :func:`isAborted`. It is cleared by either ``begin()`` (setup-time)
 *     or ``clearAbort()`` (called from ``Mission.start()`` before each new
 *     mission run) so an abort latched during run N does not bleed into
 *     run N+1.
 *   - There is no persistent active-task FSM state in TasksClass for this
 *     plan — task bodies run to completion inside ``runTask``. Multi-tick
 *     cooperative tasks land in a later iteration; for now ``update()`` is
 *     a deliberate no-op so the singleton can be ticked from ``loop()``
 *     without churn.
 *
 * @note The POD layer (``TaskArgs`` / ``TaskFn`` / ``TaskRegistryEntry``) lives
 *       in :file:`TasksTypes.h` to break the otherwise-cyclic dependency
 *       between :file:`Mission.h` (which needs the arg-union and registry-entry
 *       types to declare the ``TASK`` step variant) and :file:`Tasks.h`
 *       (which adds the class on top of those PODs).
 */

#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#include "TasksTypes.h"

class ManipServo;    // fwd decl only — _waitForServo takes a reference; the full
                     // type is pulled in by Tasks.cpp.

class TasksClass
{
public:
    /**
     * @brief Reset the task-runner state. Call from the sketch's ``setup()``.
     *
     * @return ``true`` always; signature matches the rest of the project's
     *         ``begin()`` convention so failures can be added later without
     *         touching call sites.
     */
    bool begin();

    /**
     * @brief Cooperative tick. Call from ``loop()`` alongside the other modules.
     *
     * Housekeeping only; no active-task FSM in this plan. Advances the
     * non-blocking task-fail beep train armed by ``_signalFail`` (fires the
     * next tone once its gap elapses) and is otherwise idle.
     */
    void update();

    /**
     * @brief Register the cooperative-tick callback driven during blocking task waits.
     *
    * Some task bodies (``servoTest``, ``home``, ``idleWait``) block until an actuator
    * settles or a cooperative timeout elapses. While they spin they must keep every
    * module ticking — otherwise EventTrace stops rendering, the belt's own ``update()``
    * stops advancing, etc. — so the wait invokes this callback each iteration
    * (mirroring ``ManipulatorClass::setTickCallback`` and
    * ``Drive``/``PathTracker``/``LineFollower``). The sketch passes
    * ``cooperativeTick`` here in ``setup()`` after every module's ``begin()`` has run.
     *
     * @param cb Cooperative-tick function pointer, or ``nullptr`` to clear it (the
     *           blocking waits then degrade to ticking only ``Manipulator.update()`` and
     *           warn once).
     *
     * @note ``cooperativeTick()`` calls ``Mission.update()``, which short-circuits while
     *       ``Tasks.inTask()`` is true, so driving it from inside a task wait does NOT
     *       re-enter the task.
     */
    void setTickCallback(void (*cb)());

    /**
     * @brief Look up ``name`` in the static registry and dispatch its body.
     *
     * @param name  Null-terminated task name. Resolved by ``strcmp`` linear
     *              scan over ``_registry``.
     * @param args  Packed argument union. The body owns the unpack ABI for
     *              its specific task (typically ``reinterpret_cast`` to a
     *              per-task ``struct``).
     *
     * @return The body's return value; or ``false`` if ``name`` is unknown
     *         (an ``[Tasks] runTask: unknown name '<name>'`` warning is
     *         printed to Serial); or ``false`` if called reentrantly (see the
     *         reentrancy note below).
     *
     * @note **Non-reentrant.** If ``runTask`` is invoked while a task body is
     *       already in flight (``_inTask`` is ``true``), it refuses: it prints
     *       a ``[Tasks] runTask: refusing reentrant call to '<name>'`` warning
     *       and returns ``false`` WITHOUT resolving ``name`` or arming the
     *       guard. Tasks are leaf operations — share logic between them via
     *       private helpers, not by calling ``runTask`` from inside a task.
     *       See ADR-026.
     *
     * @note ``_inTask`` is set ``true`` for the duration of dispatch via an
     *       RAII scope guard, so it is correctly reset even if the body
     *       returns through every path. ``abort()`` does NOT clear
     *       ``_inTask`` — only the guard does.
     */
    bool runTask(const char* name, const TaskArgs& args);

    /**
     * @brief Linear-scan ``_registry`` for an entry whose name matches ``name``.
     *
     * @param name  Null-terminated task name.
     * @return Pointer to the matching ``TaskRegistryEntry``, or ``nullptr``
     *         when no entry's ``name`` field compares equal under ``strcmp``.
     *
     * @note Exposed publicly so future call sites (e.g. a Mission
     *       pre-validation pass at queue-time) can check name validity
     *       without paying the dispatch cost.
     */
    const TaskRegistryEntry* findTask(const char* name) const;

    /**
     * @brief Latch a request to stop the current task framework.
     *
     * Sets ``_aborted = true``. The flag is sticky for the duration of the
     * current mission and is reset by either ``begin()`` (setup-time) or
     * ``clearAbort()`` (called from ``Mission.start()`` before each new
     * mission run). Does NOT clear ``_inTask`` — that is owned solely by
     * ``runTask``'s scope guard so the in-flight body still gets to unwind
     * cleanly.
     */
    void abort();

    /**
     * @brief Clear the sticky ``_aborted`` latch without otherwise resetting
     *        Tasks state.
     *
     * Called by ``Mission.start()`` so the abort latch from a previous
     * mission does not bleed into the next run. Does NOT touch ``_inTask``.
     */
    void clearAbort();

    /** @brief ``true`` after ``abort()`` has been called and before the next ``begin()`` / ``clearAbort()``. */
    bool isAborted() const;

    /** @brief ``true`` while ``runTask`` is currently dispatching a body. */
    bool inTask() const;

    /**
     * @brief Named colours for the on-board RGB LED pair (see ``_setLeds``).
     *
     * @note Enumerators are mixed-case on purpose: the OLED library ``#define``s
     *       the all-caps ``WHITE``/``BLACK`` macros, which would expand inside an
     *       ``enum class { WHITE }`` and break the build in any TU that includes
     *       ``MatrixMiniR4.h`` before this header (e.g. ``Mission.cpp``).
     */
    enum class LedColor : uint8_t
    {
        Off,     ///< Both LEDs dark.
        White,   ///< R+G+B full.
        Red,     ///< R full.
        Green,   ///< G full.
        Blue,    ///< B full.
        Yellow,  ///< R+G full.
    };

private:
    /**
     * @brief Static registry of all known task names + their member-fn pointers.
     *
     * Defined in ``Tasks.cpp`` — adding a new task is a single-line append to
     * the initializer there. ``_registry_count`` auto-derives from ``sizeof``,
     * and a file-scope ``static_assert`` rejects the build if the count
     * exceeds ``Config::MAX_TASKS``.
     *
     * @note Kept private because the registry is an implementation detail of
     *       ``TasksClass`` lookup and dispatch.
     */
    static const TaskRegistryEntry _registry[];

public:
    /**
    * @brief Number of entries in ``_registry``.
    *
    * Exposed so file-scope validation in ``Tasks.cpp`` can enforce the
    * ``Config::MAX_TASKS`` cap without exposing the array itself.
    */
    static const size_t            _registry_count;

private:
    /**
     * @brief Smoke-test task body — prints the first two ``uint8`` arg slots.
     *
     * @param args  Packed argument union; reads ``args.u8[0]`` and ``args.u8[1]``.
     * @return Always ``true``.
     *
     * @note Serves as the registry's first row so the rest of the plumbing
     *       (Mission's ``TASK`` step, ``runTask`` dispatch, registry lookup)
     *       has a deterministic, side-effect-only target during bring-up.
     */
    bool demoEcho(const TaskArgs& args);

    /**
     * @brief Grab the object to the bin in one call.
     * @param bin  which bin it is sorting to.
     * @note This is NOT a task that can be called at the mission.
     */
    bool _grabToBin(uint8_t bin);

    /**
     * @brief Capture one mosaic observation into the `Vision` latch, with a single retry.
     *
     * Thin wrapper over `Vision.readMosaic()`: on a fault it logs, clears the fault, and
     * retries exactly once. The captured colours are NOT returned — they live in the
     * `Vision` latch (the single source of truth); read them per-cell with
     * `Vision.getColor(cell)` after this returns `true`.
     *
     * @return `true` if a mosaic frame was captured (either attempt); `false` if both the
     *         initial read and the retry faulted, in which case the latch is unchanged.
     * @note The chassis MUST be fully idle before calling — see `Vision::readMosaic`. This is
     *       a command-time BLOCKING call and is NOT a task callable from the mission DSL.
     */
    bool _visionRead();

    /**
     * @brief Drive the sorter servo to an absolute angle and block until it settles.
     *
     * Bring-up helper: sets `Manipulator.sorter` to `args.u8[0]` degrees and polls
     * `Manipulator.update()` until the servo's settle timer reports done. Used to sweep
     * and confirm sorter bin angles by hand at the bench.
     *
     * @param args `u8[0]` = target angle in degrees.
     * @return Always `true`.
     */
    bool servoTest(const TaskArgs& args);

    /// @brief Test-support stub for Mission's TASK YIELD path: prints its
    ///        signature and returns false so a real Mission/Tasks integration can
    ///        exercise the yield (not abort) branch end-to-end.
    bool debugYield(const TaskArgs& args);

    /// @brief Cooperative idle-hold for `args.u32[0]` ms while the chassis is
    ///        stationary; refuses (returns false) if any motion is active.
    bool idleWait(const TaskArgs& args);

    bool testOne(const TaskArgs& args);

    /**
     * @brief Bring-up CTPS demo.
     *
     * @param args Ignored.
     * @return Always `true`.
     */
    bool ctpsExample1(const TaskArgs& args);

    /**
     * @brief Empty no-op test hook.
     *
     * Reserved for quick Mission/task plumbing tests. Does no motion, no manipulator actuation,
     * and simply returns success.
     *
     * @param args Ignored.
     * @return Always ``true``.
     */
    bool emptyTest(const TaskArgs& args);

    /**
     * @brief Belt-only bring-up: start the belt home sequence.
     *
     * Calls `Manipulator.belt.home()` (stall into the LEFT/HOME end-stop, re-zero the encoder,
     * then drive to the MID rest position) so the operator can watch the belt seek home without
     * any grabber / sorter / dumper motion. `belt.home()` is non-blocking, so the sequence
     * continues via `Manipulator.update()` after this returns.
     *
     * @param args Ignored.
     * @return Always ``true``.
     */
    bool beltTest(const TaskArgs& args);

    /**
     * @brief Primitive-handle bench test for every manipulator actuator.
     *
     * Drives the three servos and the belt DIRECTLY through `Manipulator`'s public primitive
     * handles (`grabber` / `sorter` / `dumper` / `belt`) so bring-up can verify each actuator
     * without going through the semantic verbs.
     *
     * @param args u8[0] = 1 to enable belt test; otherwise ignored.
     * @return ``true`` when every primitive movement completed; ``false`` on a belt/servo timeout.
     */
    bool manipDirectTest(const TaskArgs& args);

    /**
     * @brief Go to the place and read the mosaic colors.
     * @param args  Ignored.
     * @return `true` if read successfully; `false` if the read failed (after one retry).
     * @note Read the colors using the `Vision` latch. use .getColor(cell) to read the color of each cell after this returns `true`.
     */
    bool readMosaic(const TaskArgs& args);

    /**
     * @brief Get and put the rectangle shovel
     * @param args  Ignored.
     * @return true when finish. 
     */
    bool rectangleShovel(const TaskArgs& args);

    /**
     * @brief Get and put the triangle shovel
     * @param args  Ignored.
     * @return true when finish.
     */
    bool triangleShovel(const TaskArgs& args);

    /**
     * @brief Get and put the pot
     * @param args  Ignored.
     * @return true when finish.
     */
    bool pot(const TaskArgs& args);

    /**
     * @brief Get and put the mosiac
     * @param args  Ignored.
     * @return true when finish.
     * @note Need the helper _mosiacAlgo to get the mosiac fastest
     */
    bool mosaic(const TaskArgs& args);

    /**
     * @brief Get and put the debries
     * @param args  Ignored.
     * @return true when finish.
     */
    bool debries(const TaskArgs& args);

    /**
     * @brief Unload the carrier onto the mosaic build area, one chassis stop per row.
     *
     * The sorter holds one bin per mosaic COLUMN, each `kMosaicRows` tiles deep, loaded top
     * row first by `mosaic()`. Dumping walks the three row nodes (`dump1` / `dump2` / `dump3`)
     * and at each stop discharges every bin's next tile, so the built grid comes out identical
     * to the pattern read at `CY3`.
     *
     * @param args  Ignored.
     * @return true when finish.
     * @note Run `mosaic` first — this body only unloads what the carrier is already holding.
     */
    bool dumpMosaic(const TaskArgs& args);

    /**
     * @brief Stationary cooperative delay helper for task bodies.
     *
     * @param ms    Timeout in milliseconds.
     * @param site  Short caller tag for diagnostics.
     * @return ``true`` when the timeout elapses; ``false`` if the chassis is moving at
     *         entry or the task framework is aborted mid-wait.
     */
    bool _waitStationaryMs(uint32_t ms,
                          const __FlashStringHelper* site = F("waitStationaryMs"));

    // --- Cooperative-wait helpers -------------------------------------------
    // Shared by the bring-up tasks (``emptyTest`` / ``actuatorSweep`` /
    // ``actuatorStep`` / ``manipDirectTest``) that block until an actuator
    // settles. Each keeps every module ticking during the wait via the
    // registered cooperative-tick callback (falling back to Drive + Manipulator
    // ``update()`` with a one-shot warning when none is set), mirroring
    // ``Manipulator::_drain``.

    /** @brief Drive one cooperative-tick iteration during a blocking wait. */
    void _pumpTick();

    /**
     * @brief Spin ``_pumpTick()`` for @p durationMs, servicing every module.
     * @param durationMs Wall-clock duration to wait, in milliseconds.
     */
    void _waitMs(uint32_t durationMs);

    /**
     * @brief Block until @p servo settles or @p timeoutMs elapses.
     * @param servo     Servo joint to wait on.
     * @param timeoutMs Give-up timeout in milliseconds.
     * @param label     Short site label printed on timeout.
     * @return ``true`` once the servo reports done; ``false`` on timeout.
     */
    bool _waitForServo(ManipServo& servo, uint32_t timeoutMs, const char* label);

    /**
     * @brief Block until the belt settles or @p timeoutMs elapses.
     * @param timeoutMs Give-up timeout in milliseconds.
     * @param label     Short site label printed on timeout.
     * @return ``true`` once the belt reports done; ``false`` on timeout.
     */
    bool _waitForBelt(uint32_t timeoutMs, const char* label);

    // --- Result-feedback helpers --------------------------------------------
    // Non-blocking audible + visual pass/fail cue for a task body. Both light the
    // two on-board RGB LEDs and drive the buzzer via the non-blocking ``Tone()``
    // (hardware timer), so they return immediately without spinning the tick.

    /**
     * @brief Set both on-board RGB LEDs to a named colour.
     * @param color One of ``LedColor`` (``LedColor::Off`` turns both LEDs off).
     * @note Drives both LEDs to the same colour at full brightness, matching the
     *       boot / pass-fail cues. Returns immediately (``setColor`` is non-blocking).
     */
    void _setLeds(LedColor color);

    /**
     * @brief Emit the task-success cue: one short bright chirp + solid green LEDs.
     * @note Non-blocking — returns immediately; the single tone plays out on the
     *       hardware timer. Also cancels any in-flight ``_signalFail`` beep train.
     */
    void _signalSuccess();

    /**
     * @brief Emit the task-fail cue: ``Config::TASK_FAIL_BEEPS`` low tones + solid red LEDs.
     * @note Non-blocking — fires the first tone and returns; ``update()`` emits the
     *       remaining tones as each gap elapses, so the beeps stay audibly separate
     *       without blocking the caller. See ``_failBeepsLeft`` / ``_failPhaseStartMs``.
     */
    void _signalFail();

    /**
     * @brief One planned tile: which wall colour to take it from, and where it goes.
     */
    struct MosaicPick
    {
        Config::Color color;   ///< Wall colour station to take this tile from.
        uint8_t       cell;    ///< Mosaic cell it fills; its row selects the sorter bin.
    };

    /**
     * @brief Plan the shortest station route that satisfies the read mosaic pattern.
     *
     * The pattern is a 4-column x 3-row grid; each column is built top row down, so only the
     * next uncleared cell of each column is "live". The robot sweeps the M wall past the four
     * colour stations in a fixed physical order (up, down, up), and at each station it can
     * satisfy every live cell of that colour at once. Satisfying a cell exposes the one below
     * it, so a colour buried under a station the sweep has already passed is only reachable on
     * the next sweep.
     *
     * This walks that sweep order repeatedly and emits tiles only for the station colours that
     * are actually live, which is what makes the route short: stations with nothing to collect
     * are skipped entirely.
     *
     * Output is one entry per TILE, in collection order. A run of consecutive entries sharing
     * a colour is one stop at that colour's wall station, so `mosaic()` can walk the list
     * straight through. Each entry also carries the mosaic cell the tile fills, whose row is
     * the sorter bin it must be dropped into — the sorter holds one bin per mosaic row so the
     * dumped rows come out identical to the pattern that was read.
     *
     * @param picks    Receives one entry per tile, in collection order. Must hold at least
     *                 @p maxPicks entries.
     * @param maxPicks Capacity of @p picks. `Config::VISION_CELL_COUNT` is always enough —
     *                 one entry per cell of the pattern.
     * @return Number of tiles written to @p picks. `0` when the pattern holds no station
     *         colour at all — including the case where no successful `Vision::readMosaic()`
     *         has happened, since every cell then reads `Config::Color::UNKNOWN`.
     * @note Reads the `Vision` latch only; it does NOT trigger a camera read. Run the
     *       `readMosaic` task first.
     */
    uint8_t _mosaicAlgo(MosaicPick picks[], uint8_t maxPicks) const;

    /**
     * @brief Choose which of a colour's two wall rows to take the next tile from.
     *
     * Prefers staying on the row the chassis is already parked at so the belt can reach the
     * next tile without a drive; when that row is empty (or the robot is at another colour)
     * it enters at whichever remaining row is nearer along the wall chain.
     *
     * @param stock       Tiles left on this colour's two rows, indexed lower / upper.
     * @param colorIdx    Colour index in `[0, 4)` — also the sorter bin index.
     * @param parkedColor Colour index the chassis is parked at, or `0xFF` before the first drive.
     * @param parkedRow   Row index the chassis is parked at, or `0xFF` before the first drive.
     * @param chainPos    Wall-chain position of the chassis (`MWU` = 0 at the top, `MYD` = 7).
     * @return `0` for the lower row (`MxD`), `1` for the upper row (`MxU`), or `0xFF` when
     *         this colour has no tiles left anywhere on the wall.
     */
    uint8_t _mosaicPickRow(
        const uint8_t stock[2],
        uint8_t       colorIdx,
        uint8_t       parkedColor,
        uint8_t       parkedRow,
        uint8_t       chainPos) const;

    bool _inTask  = false;
    bool _aborted = false;

    /**
     * @brief Cooperative-tick callback driven during blocking task waits (``servoTest`` /
     *        ``home``). ``nullptr`` until the sketch registers ``cooperativeTick`` via
     *        ``setTickCallback`` in ``setup()``.
     */
    void (*_tickCallback)() = nullptr;

    /**
     * @brief One-shot latch so the "no tick callback set" warning printed by
     *        ``_pumpTick`` fires at most once per run.
     */
    bool _noCallbackWarned = false;

    // --- Non-blocking task-fail beep train (see _signalFail / update) --------

    /**
     * @brief Task-fail tones still to emit after the first. ``_signalFail`` fires
     *        tone 1 and sets this to ``Config::TASK_FAIL_BEEPS - 1``; ``update()``
     *        decrements it as each gap elapses. ``0`` means idle.
     */
    uint8_t _failBeepsLeft = 0;

    /**
     * @brief ``millis()`` when the current task-fail tone started; ``update()``
     *        fires the next tone once ``TASK_FAIL_BEEP_MS + GAP_MS`` has elapsed.
     */
    uint32_t _failPhaseStartMs = 0;
};

extern TasksClass Tasks;

#endif    // TASKS_H
