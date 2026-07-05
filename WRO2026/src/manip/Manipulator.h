/**
 * @file Manipulator.h
 * @brief Manipulator-subsystem facade owning the four actuators (grabber / sorter / dumper
 *        servos + conveyor belt) and exposing coordinated semantic verbs.
 *
 * @details
 * `ManipulatorClass` is the single entry point mission code uses to actuate the gripping /
 * sorting / dumping mechanism. It owns the three `ManipServo` joints and the one `ManipBelt`
 * by value (constructed from `Config::*` in `Manipulator.cpp`) and presents two API layers:
 *
 *  - **Semantic verbs** (`grab`, `release`, `dump`, `sortTo`, `reset`) — the intended
 *    everyday surface. Each verb FIRES its actuator setpoint(s) immediately in the verb body
 *    (servo writes are fire-and-forget; firing now is correct) and returns a `ManipAction`
 *    fluent builder. The builder is **blocking-by-default**: at end-of-statement it WAITS
 *    (drains) on the actuators the verb touched until they report done. Chain `.async()` to
 *    skip the wait and return immediately — the setpoints have already been issued either way.
 *    Mission scripts then read top-down like Python (`Manipulator.grab();` blocks until the
 *    grabber settles), mirroring the `Drive` builder contract.
 *
 *  - **Primitive actuator handles** (`grabber`, `sorter`, `dumper`, `belt`) — the raw joints
 *    are left PUBLIC so calibration / bring-up code can poke a single actuator directly
 *    (`Manipulator.grabber.setAngle(120)`, `Manipulator.belt.toStation(...)`) without going
 *    through a verb. Production mission code should prefer the verbs.
 *
 * **Completion model.** The three servos have NO position feedback (`ManipServo` fakes
 * completion with a slew-plus-settle timer); the belt has REAL encoder feedback (`ManipBelt`
 * confirms POSITION moves from the rotate-done bit, or times out TIMED runs). `isDone()` is
 * the AND of all four actuators' `isDone()`. `update()` advances only the belt's state machine
 * (`belt.update()`) — the servos are pure timers and need no per-tick work.
 *
 * **`reset()`** homes all three servos AND stall-homes + zeroes the belt; it is the safe-pose
 * verb (e.g. after a fault or before a run).
 *
 * @note The semantic verbs are thin bring-up scaffolding: `grab()` is `grabber.close()`,
 *       `dump()` is `dumper.open()` (a placeholder dump pose), etc. The real multi-actuator
 *       choreography (e.g. lift-then-grip, belt-stage-then-dump) is a TODO to be authored once
 *       the mechanism is built and characterised.
 *
 * @warning A function registered via `setTickCallback()` is invoked from inside `_drain()`'s
 *          blocking spin. It MUST NOT issue a blocking Manipulator verb (one without
 *          `.async()`) — doing so would re-enter `_drain()` from within its own spin and
 *          recurse indefinitely. Same contract as `Drive` (ADR-016): if a future module's tick
 *          needs to issue manipulator verbs, it must use `.async()`.
 */

#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <MatrixMiniR4.h>

#include "../../Config.h"
#include "ManipServo.h"
#include "ManipBelt.h"

// Forward declaration so ManipAction can name the facade before it is defined, and so the
// facade can name-friend the builder below.
class ManipulatorClass;

/**
 * @brief Fluent, blocking-by-default builder returned by every `ManipulatorClass` verb.
 *
 * @details
 * **The verb has ALREADY fired the actuator setpoint(s)** in its body before constructing this
 * builder — servo writes are fire-and-forget and firing immediately is correct, and the belt's
 * non-blocking state machine has been kicked off. This builder's ONLY job is to optionally WAIT
 * for the involved actuators to finish: at end-of-statement its destructor drains (spins until
 * `isDone()` on the masked actuators) unless `.async()` was chained.
 *
 * `.async()` therefore gates the WAIT, NOT the firing — an async action still moves the
 * hardware; it just does not block the caller.
 *
 * The builder is **move-only** (deleted copy + move-assign; a move ctor is kept only so the
 * verb can return it by value on pre-C++17 compilers). `_executed` guards against a double
 * drain if the temporary is moved out of. `[[nodiscard]]` flags a discarded builder, which
 * almost always means a forgotten verb statement.
 *
 * @see ManipulatorClass for the verbs that produce these and the actuator-mask scheme.
 */
class [[nodiscard]] ManipAction
{
public:
    ManipAction(const ManipAction&)            = delete;
    ManipAction& operator=(const ManipAction&) = delete;
    ManipAction& operator=(ManipAction&&)      = delete;

    /**
     * @brief Move constructor — transfers the wait responsibility to the new instance.
     *
     * Required so the verbs can return a builder by value on compilers that do not perform
     * mandatory copy elision (pre-C++17). Copies every member and marks the moved-from
     * instance `_executed` so its destructor becomes a no-op.
     */
    ManipAction(ManipAction&& other) noexcept
        : _m(other._m), _mask(other._mask), _blocking(other._blocking), _executed(other._executed)
    {
        other._executed = true;
    }

    /**
     * @brief Opt this action out of the default blocking wait.
     *
     * Gates the WAIT only — the verb's setpoints were already fired in the verb body, so an
     * async action still actuates the hardware; it simply returns without draining. Poll
     * `Manipulator.isDone()` (or the specific actuator's `isDone()`) to learn when it settles.
     *
     * @return `*this` for chaining.
     * @note The default is blocking. Async is the opt-out for callers that tick the
     *       cooperative loop themselves and do not want each verb to block their tick.
     */
    ManipAction& async()
    {
        _blocking = false;
        return *this;
    }

    /**
     * @brief Destructor — RAII wait. Drains the involved actuators unless `.async()` was set.
     *
     * Fires at end-of-statement. If still responsible (`!_executed`) and blocking, calls
     * `_m._drain(_mask)` to spin until every actuator named in `_mask` reports `isDone()` (or
     * its per-action ms ceiling trips). Does NOT issue any setpoint — that already happened in
     * the verb body; this only waits.
     *
     * @note Idempotent via the `_executed` guard: a moved-out-of builder's destructor is a
     *       no-op.
     * @note Declared here but DEFINED out-of-line below (after `class ManipulatorClass` is
     *       complete): the body calls `_m._drain(_mask)`, and `_m` is a `ManipulatorClass&` that
     *       is only forward-declared at this point. Defining the body inline here would name a
     *       member of an incomplete type, which MSVC rejects (C2027); the out-of-line definition
     *       sees the full class.
     */
    ~ManipAction();

private:
    /**
     * @brief Construct an action bound to facade @p m for the actuators named in @p mask.
     *        Private — only `ManipulatorClass::_action` (and thus the verbs) may build one.
     *
     * @param m    The owning facade — the drain target.
     * @param mask Bitmask of involved actuators (`M_GRAB | M_SORT | M_DUMP | M_BELT`); `0`
     *             means "nothing to wait on" (the drain is an immediate no-op).
     */
    ManipAction(ManipulatorClass& m, uint8_t mask) : _m(m), _mask(mask)
    {
    }

    ManipulatorClass& _m;                  ///< Owning facade; the `_drain` target.
    uint8_t           _mask;               ///< Involved-actuator bitmask to wait on.
    bool              _blocking = true;    ///< Drain at end-of-statement unless `.async()`.
    bool              _executed = false;   ///< Guard so a moved-from builder's dtor is a no-op.

    friend class ManipulatorClass;
};

/**
 * @brief Facade over the manipulator's four actuators with coordinated semantic verbs.
 *
 * @details
 * Owns three `ManipServo` joints (`grabber`, `sorter`, `dumper`) and one `ManipBelt` (`belt`)
 * by value; their construction arguments come from `Config::*` in `Manipulator.cpp`'s ctor.
 * See the file docblock for the verb / primitive split, the blocking-by-default contract, the
 * completion model, and the `setTickCallback` reentrancy warning. A single global instance
 * `Manipulator` is exported (defined in `Manipulator.cpp`).
 */
class ManipulatorClass
{
public:
    /**
     * @brief Construct the facade and its four actuators from `Config::*`.
     *
     * Does NO hardware work — the actuators only capture their config here. Defined in
     * `Manipulator.cpp` so the member initialiser list can name the `Config::GRABBER` /
     * `SORTER` / `DUMPER` / `BELT_STATIONS` tables in one place.
     */
    ManipulatorClass();

    /**
     * @brief Configure and home every actuator. Blocking, setup-time only.
     *
     * Calls each servo's `begin()` (applies hardware direction, seeds the move timer) and the
     * belt's `begin()` (configures the motor and stall-homes it, blocking-bounded). Must be
     * called once from `setup()` after `MiniR4.begin()` has returned.
     *
     * @return `true` if every actuator initialised successfully; `false` if the belt failed to
     *         configure or home within its timeout (the servos have no failable init).
     * @warning The belt physically drives into its HOME end-stop during this call — keep hands
     *          clear at boot (see `ManipBelt::begin`).
     */
    bool begin();

    /**
     * @brief Cooperative tick — must be called every iteration of the main `loop()`.
     *
     * Advances the belt's non-blocking move state machine (`belt.update()`). The three servos
     * are pure software timers with no per-tick work, so they are intentionally not ticked
     * here — their `isDone()` reads the timer directly.
     *
     * @note Contains NO `delay()`. Safe to call at any cadence.
     */
    void update();

    /**
     * @brief Whether every actuator has finished its last commanded motion.
     * @return `grabber.isDone() && sorter.isDone() && dumper.isDone() && belt.isDone()`.
     * @note Servo completion is timer-estimated (no position feedback); belt completion is
     *       encoder/timer-confirmed. See the file docblock.
     */
    bool isDone() const;

    /**
     * @brief Register a function to call each iteration of `_drain()`'s blocking spin.
     *
     * A blocking verb's `_drain()` must keep all cooperative subsystems alive while it waits —
     * otherwise `EventTrace` stops rendering, the belt's own `update()` stops advancing, etc.
     * Register a function that ticks every module (typically `cooperativeTick()` from the
     * sketch); `_drain` invokes it each spin iteration instead of just `update()`.
     *
     * Call once from `setup()`, after every module's `begin()`.
     *
     * @param cb Pointer to the cooperative-tick function. Pass `nullptr` to clear.
     * @note If no callback is registered, `_drain` falls back to ticking only this module's
     *       `update()` and emits a one-time Serial warning at first use — other modules then
     *       freeze during the wait, so register the callback to avoid that.
     * @warning The registered callback's call chain MUST NOT issue a blocking Manipulator verb
     *          (a verb without `.async()`). Doing so re-enters `_drain()` from inside its own
     *          spin and recurses indefinitely — same contract as `Drive` (ADR-016). Today no
     *          `update()` does this; a future tick that needs manipulator motion must use
     *          `.async()`.
     */
    void setTickCallback(void (*cb)());

    // -------------------------------------------------------------------------------------
    // Coordinated semantic verbs.
    //
    // Each verb FIRES its actuator setpoint(s) immediately (servos are fire-and-forget; the
    // belt kicks off its non-blocking machine) and returns a blocking-by-default `ManipAction`
    // carrying the involved-actuator mask. The action's destructor WAITS on that mask unless
    // `.async()` is chained — `.async()` gates the wait, not the firing.
    // -------------------------------------------------------------------------------------

    /**
     * @brief Close the grabber to capture an object.
     * @return A `ManipAction` (blocks on the grabber until settled; chain `.async()` to skip).
     * @note Thin bring-up scaffold (`grabber.close()`); real grip choreography is a TODO once
     *       the mechanism is built.
     */
    ManipAction grab()
    {
        grabber.close();
        return _action(M_GRAB);
    }

    /**
     * @brief Open the grabber to release an object.
     * @return A `ManipAction` (blocks on the grabber until settled; chain `.async()` to skip).
     * @note Thin bring-up scaffold (`grabber.open()`); real release choreography is a TODO.
     */
    ManipAction release()
    {
        grabber.open();
        return _action(M_GRAB);
    }

    /**
     * @brief Move the dumper to its dump pose to discharge the payload.
     * @return A `ManipAction` (blocks on the dumper until settled; chain `.async()` to skip).
     * @note Bring-up scaffold: the dump pose is the dumper's `openDeg` placeholder
     *       (`dumper.open()`); the real dump pose / sequencing is a TODO.
     */
    ManipAction dump()
    {
        dumper.open();
        return _action(M_DUMP);
    }

    /**
     * @brief Swing the sorter to the angle for logical bin @p bin.
     *
     * Bounds-checked: if @p bin is out of range no actuator write happens and an empty-mask
     * action is returned (so a blocking caller does not stall waiting on an actuator that was
     * never commanded).
     *
     * @param bin Bin index in `[0, Config::SORTER_BIN_COUNT)`.
     * @return A `ManipAction` waiting on the sorter on success; an empty-mask (already-done)
     *         action on an out-of-range index.
     * @note On a bad index this prints a Serial warning and performs NO servo write.
     */
    ManipAction sortTo(uint8_t bin)
    {
        if (bin >= Config::SORTER_BIN_COUNT)
        {
            Serial.print(F("[Manipulator] sortTo: bin out of range: "));
            Serial.println((int)bin);
            return _action(0);    // empty mask → drain is an immediate no-op
        }
        sorter.setAngle(Config::SORTER_BIN_ANGLE[bin]);
        return _action(M_SORT);
    }

    /**
     * @brief Return every actuator to its safe rest pose.
     *
     * Homes all three servos (`home()`) and stall-homes + zeroes the belt (`belt.home()`), then
     * waits on all four. The go-to verb for a known-safe state — e.g. after a fault or before a
     * run starts.
     *
     * @return A `ManipAction` waiting on all four actuators (chain `.async()` to skip the wait).
     */
    ManipAction reset()
    {
        grabber.home();
        sorter.home();
        dumper.home();
        belt.home();
        return _action(M_GRAB | M_SORT | M_DUMP | M_BELT);
    }

    // -------------------------------------------------------------------------------------
    // Primitive actuator handles (public for calibration / bring-up).
    //
    // Left public so single-actuator calibration code can drive one joint directly
    // (`Manipulator.grabber.setAngle(...)`, `Manipulator.belt.toStation(...)`) without a verb.
    // Production mission code should prefer the semantic verbs above. Constructed from
    // `Config::*` in the facade ctor (Manipulator.cpp).
    // -------------------------------------------------------------------------------------

    ManipServo grabber;    ///< Grabber joint (`Config::GRABBER`).
    ManipServo sorter;     ///< Sorter joint (`Config::SORTER`; bin angles in `SORTER_BIN_ANGLE`).
    ManipServo dumper;     ///< Dumper joint (`Config::DUMPER`).
    ManipBelt  belt;       ///< Conveyor belt (`Config::BELT_STATIONS`, `BELT_M_ID`).

private:
    // Involved-actuator bitmask — which actuators a verb touched, so `_drain` waits only on
    // those. Combined with `|` (e.g. `reset()` touches all four).
    static constexpr uint8_t M_GRAB = 0x01;    ///< Grabber servo.
    static constexpr uint8_t M_SORT = 0x02;    ///< Sorter servo.
    static constexpr uint8_t M_DUMP = 0x04;    ///< Dumper servo.
    static constexpr uint8_t M_BELT = 0x08;    ///< Conveyor belt.

    /**
     * @brief Build a `ManipAction` for the involved-actuator @p mask. The verb factory.
     * @param mask Bitmask of actuators the verb fired (`0` = nothing to wait on).
     * @return A blocking-by-default action bound to this facade.
     */
    ManipAction _action(uint8_t mask) { return ManipAction(*this, mask); }

    /**
     * @brief Block (spin) until every actuator named in @p mask reports done, or the action
     *        ceiling trips. Defined in `Manipulator.cpp`.
     *
     * Bounded spin: each iteration invokes the registered tick callback (falling back to this
     * module's `update()` plus a one-time Serial warning when no callback is registered) and
     * `delay(2)`, until `_maskDone(mask)` is true or a per-action millisecond ceiling elapses.
     * On the ceiling it stops the belt (a safe brake) and emits a Serial warning so a stalled
     * actuator can never hang the drain indefinitely.
     *
     * @param mask Bitmask of actuators to wait on; `0` returns immediately.
     * @note This is a sanctioned blocking site — the blocking caller is off the cooperative
     *       tick chain by definition (same rationale as `Drive::_drainBlocking`).
     */
    void _drain(uint8_t mask);

    /**
     * @brief Whether every actuator named in @p mask reports `isDone()`. Defined in
     *        `Manipulator.cpp`.
     *
     * Checks ONLY the masked actuators — an unmasked actuator's state is ignored, so a verb
     * that touched one joint does not block on an unrelated in-flight one.
     *
     * @param mask Bitmask of actuators to test (`0` ⇒ vacuously `true`).
     * @return `true` iff all masked actuators are done.
     */
    bool _maskDone(uint8_t mask) const;

    /// @brief Optional cooperative-tick callback invoked from `_drain`'s spin so other modules
    ///        keep running during a blocking verb. Registered via `setTickCallback()`.
    ///        `nullptr` until set; falls back to `update()` alone with a one-time Serial warning.
    void (*_tickCallback)() = nullptr;

    friend class ManipAction;
};

/**
 * @brief Out-of-line definition of the `ManipAction` RAII-wait destructor.
 *
 * Deferred to here (rather than inline in the class) because the body dereferences the
 * `ManipulatorClass&` member `_m` to call `_m._drain(_mask)`, and `ManipulatorClass` is only
 * forward-declared at the point of the class body — naming a member of an incomplete type there
 * is ill-formed (MSVC C2027). With the full class now visible the call type-checks. `inline`
 * keeps it legal to define in a header included by multiple translation units. Semantics are
 * unchanged: idempotent via `_executed`, drain only when blocking.
 */
inline ManipAction::~ManipAction()
{
    if (_executed)
    {
        return;
    }
    _executed = true;
    if (_blocking)
    {
        _m._drain(_mask);
    }
}

/**
 * @brief Single global instance — defined in Manipulator.cpp.
 */
extern ManipulatorClass Manipulator;

#endif    // MANIPULATOR_H
