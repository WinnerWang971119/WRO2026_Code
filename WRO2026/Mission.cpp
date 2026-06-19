/**
 * @file Mission.cpp
 * @brief Implementation of the PR3 Mission DSL.
 * @author WRO 2026
 */

#include "Mission.h"

#include <Arduino.h>
#include <MatrixMiniR4.h>

#include "Config.h"
#include "Drive.h"
#include "EventTrace.h"
#include "FieldMap.h"
#include "LineFollower.h"
#include "Localization.h"
#include "PathTracker.h"
#include "Tasks.h"
#include "routes.h"

#ifdef CTPS_RECORD
#include "ctps_record.h"
#endif

MissionClass Mission;

bool MissionClass::begin()
{
    _count = 0;
    _idx   = 0;
    _state = State::IDLE;
    _stepState = StepState::ENTER;
    _faultBeepIdx       = 0;
    _faultPhaseStart_ms = 0;
    _btnPrev            = false;
    _buildError         = false;
    _lastPushOk         = false;
    return true;
}

// IMPORTANT: `s.text` is stored by pointer and dereferenced when the step
// actually runs (potentially seconds after _push). Callers MUST pass a
// string literal or other static-storage pointer. Never pass
// `String::c_str()`, a stack buffer, or a heap-allocated char* whose
// lifetime ends before the queue executes the step.
bool MissionClass::_push(const Step& s)
{
    if (_count >= Config::MAX_MISSION_STEPS)
    {
        Serial.println(F("[Mission] queue full; step dropped"));
        return false;
    }
    _queue[_count++] = s;
    return true;
}

MissionClass& MissionClass::goTo(const char* nodeName)
{
    Step s{};
    s.type = StepType::GOTO;
    s.text = nodeName;
    s.goal = FieldMap::findNodeByName(nodeName);
    _lastPushOk = _push(s);
    return *this;
}

MissionClass& MissionClass::wait(uint32_t ms)
{
    Step s{};
    s.type        = StepType::WAIT;
    s.duration_ms = ms;
    _lastPushOk = _push(s);
    return *this;
}

MissionClass& MissionClass::print(const char* msg)
{
    Step s{};
    s.type = StepType::PRINT;
    s.text = msg;
    _lastPushOk = _push(s);
    return *this;
}

MissionClass& MissionClass::beep(uint16_t freq_hz, uint32_t duration_ms)
{
    Step s{};
    s.type    = StepType::BEEP;
    s.freq_hz = freq_hz;
    // Clamp at the verb so the stored duration can never exceed the uint16_t
    // range the _beep wrapper / advance gate share. Above the ceiling the
    // audible tone would stop after duration_ms % 65536 ms while the step kept
    // blocking the full duration — see Config::MISSION_BEEP_MAX_MS.
    s.duration_ms =
        duration_ms > Config::MISSION_BEEP_MAX_MS ? Config::MISSION_BEEP_MAX_MS : duration_ms;
    _lastPushOk = _push(s);
    return *this;
}

MissionClass& MissionClass::led(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms)
{
    Step s{};
    s.type        = StepType::LED;
    s.r           = r;
    s.g           = g;
    s.b           = b;
    s.duration_ms = duration_ms;
    _lastPushOk = _push(s);
    return *this;
}

// Modifier verbs — mutate the most-recently-enqueued GOTO step in place.
// Misuse modes (all no-op + Serial warning, queue unchanged):
//   - empty queue (no step pushed yet)
//   - last step is not GOTO (modifier chained on wait/print/beep/led)
//   - preceding push silently overflowed the queue (_lastPushOk == false) —
//     without this third guard, the modifier would target _queue[_count - 1]
//     which is the PREVIOUSLY-enqueued step (often another GOTO), silently
//     rewriting an unrelated goal's direction.

MissionClass& MissionClass::setRouteDirection(Direction d)
{
    if (!_lastPushOk)
    {
        Serial.println(F("[Mission] setRouteDirection: preceding step did not enqueue (ignored)"));
        return *this;
    }
    if (_count == 0 || _queue[_count - 1].type != StepType::GOTO)
    {
        Serial.println(F("[Mission] setRouteDirection: no preceding goTo step (ignored)"));
        return *this;
    }
    _queue[_count - 1].route_direction = d;
    return *this;
}

MissionClass& MissionClass::setArrivalDirection(Direction d)
{
    if (!_lastPushOk)
    {
        Serial.println(F("[Mission] setArrivalDirection: preceding step did not enqueue (ignored)"));
        return *this;
    }
    if (_count == 0 || _queue[_count - 1].type != StepType::GOTO)
    {
        Serial.println(F("[Mission] setArrivalDirection: no preceding goTo step (ignored)"));
        return *this;
    }
    _queue[_count - 1].arrival_direction = d;
    return *this;
}

MissionClass& MissionClass::setStopAtLine(StopAtLine s)
{
    if (!_lastPushOk)
    {
        Serial.println(F("[Mission] setStopAtLine: preceding step did not enqueue (ignored)"));
        return *this;
    }
    if (_count == 0 || _queue[_count - 1].type != StepType::GOTO)
    {
        Serial.println(F("[Mission] setStopAtLine: no preceding goTo step (ignored)"));
        return *this;
    }
    _queue[_count - 1].stop_at_line = s;
    return *this;
}

void MissionClass::start()
{
    if (_buildError)
    {
#ifdef CTPS_RECORD
        ctps::recordBuildError();
#endif
        Serial.println(F("[Mission] start refused: build error (unknown task name at queue time)"));
        EventTrace.pushEvent("MIS BUILD ERR");
        _enterFaultBeeping(millis());
        return;
    }
    if (_count == 0)
    {
        Serial.println(F("[Mission] start with empty queue; no-op"));
        return;
    }
    // Clear the abort latch from any previous mission run so a task body
    // polling Tasks.isAborted() in this run sees a clean state.
    Tasks.clearAbort();
    _idx = 0;
    _stepState = StepState::ENTER;
    _state = State::RUNNING;
    Serial.print(F("[Mission] start; steps="));
    Serial.println(_count);
    EventTrace.pushEvent("MIS START");
}

void MissionClass::abort()
{
    // Real safety stop: cancel every cooperative subsystem that could
    // re-issue motion on the next tick. Drive.stop(true) alone is not
    // enough — PathTracker may be mid-edge (TURNING / LINE_FOLLOW /
    // BLIND_FORWARD) and its next update() would resume the route.
    // LineFollower must be disabled too so the tracer stops driving the
    // motors on the same tick.
    // T7: Tasks.abort() sets the framework-level _aborted flag. In this
    // plan the flag is wired but subsystems do not yet poll it; an
    // in-flight task body unwinds because PathTracker.abort() / Drive.stop
    // below halt the chassis and subsequent subsystem builders return
    // false to the task body.
    Tasks.abort();
    PathTracker.abort();
    LineFollower.setEnabled(false);
    Drive.stop(true);

    _count       = 0;
    _idx         = 0;
    _state       = State::IDLE;
    _stepState   = StepState::ENTER;
    _buildError  = false;
    _lastPushOk  = false;
    Serial.println(F("[Mission] abort"));
}

bool MissionClass::declarePosition(const char* name)
{
    // Immediate recovery action (like abort()) — NOT a queued step. Usable only when
    // PathTracker is NOT mid-hop. The guard is idle-OR-faulted, not strictly-idle:
    // recovering from an OFF_NODE fault is the whole point, and isIdle() returns false
    // while faulted, so a strictly-idle guard would make recovery impossible.
    if (!PathTracker.isIdle() && !PathTracker.hasFault())
    {
        Serial.println(F("[Mission] declarePosition: PathTracker busy (ignored)"));
        return false;
    }

    // Resolve + bounds-check the name the same way the GOTO step validates s.goal.
    const uint8_t id = FieldMap::findNodeByName(name);
    if (id == FieldMap::NODE_NONE || id >= Routes::N_NODES)
    {
        Serial.print(F("[Mission] declarePosition: unknown node '"));
        Serial.print(name == nullptr ? "?" : name);
        Serial.println(F("'"));
        return false;
    }

    // Low-level re-anchor: set node + zero offset (heading offset untouched).
    Localization.declarePosition(id);

    // Clear the latched PathTracker fault if any — this is the "clears the fault" half
    // of the recovery, and it lives HERE (Mission layer) because Localization can't
    // reach up into PathTracker. clearFault() resets the fault and returns the FSM to IDLE.
    if (PathTracker.hasFault())
    {
        PathTracker.clearFault();
    }

    // Also clear Drive's OWN latched fault (#2). A return seek that ran to its max distance
    // latched `Fault::LINE_NOT_FOUND` on Drive and surfaced PathTracker `OFF_NODE`;
    // `PathTracker.clearFault()` does NOT touch Drive and `Drive.stop()` does not clear it
    // either, so without this the very next `goTo()` trips the generic Drive-fault guard
    // (-> `Fault::DRIVE_FAULT`) and the robot is unrecoverable without a reboot. Harmless (a
    // no-op) when Drive has no fault latched.
    Drive.clearFault();

    // Surface the recovery so the operator sees it happened (OLED + Serial). Buffer width
    // tracks EventTrace's ring slot so the truncation rule stays single-sourced. `name` is
    // non-null here: a null name resolves to NODE_NONE above and returns at the unknown-node
    // branch, so the defensive `name == nullptr` ternary on this success path was dead (#11).
    char buf[EventTraceClass::MAX_LINE_LEN];
    snprintf(buf, sizeof(buf), "DECL %s", name);
    EventTrace.pushEvent(buf);
    Serial.print(F("[Mission] declarePosition "));
    Serial.println(name);
    return true;
}

bool MissionClass::isIdle() const
{
    return _state == State::IDLE;
}

bool MissionClass::hasFault() const
{
    return _state == State::FAULT_BEEPING || _state == State::FAULT_AWAIT_RESET;
}

void MissionClass::_setLeds(uint8_t r, uint8_t g, uint8_t b)
{
    // MatrixMiniR4 exposes the on-board RGB pair via the LED module. The two
    // on-board RGB LEDs are fixed at indices 1 and 2 — MiniR4LED::setColor
    // rejects any idx outside 1..2 (`if (idx < 1 || idx > 2) return false;`),
    // so idx 0 is invalid and would be silently dropped.
    MiniR4.LED.setColor(1, r, g, b);
    MiniR4.LED.setColor(2, r, g, b);
}

void MissionClass::_beep(uint16_t freq_hz, uint16_t duration_ms)
{
    // MatrixMiniR4 owns the on-board buzzer (pin 6 internally). The library
    // exposes the non-blocking tone as MiniR4BUZZER::Tone(freq, ms).
    MiniR4.Buzzer.Tone(freq_hz, duration_ms);
}

void MissionClass::_enterFaultBeeping(uint32_t now)
{
    // Every mission-halting path lands here so the robot ends in the same
    // known-safe state: chassis braked, queue cleared, LEDs red, first
    // beep emitted, FSM in FAULT_BEEPING. The remaining queue is dropped
    // immediately so isIdle() can never return true while a stale step
    // sits behind a latched fault.
    Drive.stop(true);
    _count = 0;
    _idx   = 0;
    _stepState = StepState::ENTER;
    _setLeds(255, 0, 0);
    _faultLedOn         = true;
    _state              = State::FAULT_BEEPING;
    _faultBeepIdx       = 0;
    _faultPhaseStart_ms = now;
    _beep(Config::MISSION_FAULT_BEEP_HZ, Config::MISSION_FAULT_BEEP_MS);
}

void MissionClass::_emitFault()
{
    if (!PathTracker.hasFault())
    {
        return;
    }
    const auto info = PathTracker.getFault();
    Serial.print(F("[Mission] FAULT "));
    Serial.print(PathTrackerClass::faultMnemonic(info.kind));
    Serial.print(F(" from="));
    Serial.print(info.fromNode);
    Serial.print(F(" to="));
    Serial.println(info.destNode);

    // Push a short tag to OLED (NAV FAULT <mn>). Size is shared with the
    // EventTrace ring slot so a future change to MAX_LINE_LEN flows here
    // without an out-of-sync truncation.
    char buf[EventTraceClass::MAX_LINE_LEN];
    snprintf(buf, sizeof(buf), "NAV FAULT %s",
             PathTrackerClass::faultMnemonic(info.kind));
    EventTrace.pushEvent(buf);
}

void MissionClass::_advanceStep()
{
    _idx++;
    _stepState = StepState::ENTER;
}

void MissionClass::update()
{
    // T6: short-circuit when called from inside a blocking task body
    // (the body re-enters cooperativeTick() while waiting on PathTracker
    // / Drive sync drains; without this guard the Mission FSM would
    // recurse into the same TASK step). Tasks.inTask() is true between
    // runTask() entry and return (RAII scope-exit guard inside Tasks).
    if (Tasks.inTask()) { return; }

    switch (_state)
    {
        case State::IDLE:
        {
            return;
        }

        case State::RUNNING:
        {
            // Drain-time PathTracker fault check.
            if (PathTracker.hasFault())
            {
                _emitFault();
                _enterFaultBeeping(millis());
                return;
            }

            if (_idx >= _count)
            {
                Serial.println(F("[Mission] done"));
                EventTrace.pushEvent("MIS DONE");
                _count = 0;
                _idx   = 0;
                _state = State::IDLE;
                return;
            }

            Step& s = _queue[_idx];
            const uint32_t now = millis();
            switch (s.type)
            {
                case StepType::GOTO:
                {
                    if (_stepState == StepState::ENTER)
                    {
                        if (s.goal == FieldMap::NODE_NONE || s.goal >= Routes::N_NODES)
                        {
                            Serial.print(F("[Mission] unknown node '"));
                            Serial.print(s.text == nullptr ? "?" : s.text);
                            Serial.println(F("'"));
                            EventTrace.pushEvent("NAV FAULT BAD");
                            _enterFaultBeeping(now);
                            return;
                        }
                        Serial.print(F("[Mission] goTo "));
                        Serial.println(s.text == nullptr ? "?" : s.text);
                        PathTracker.goTo(s.goal)
                            .setRouteDirection(s.route_direction)
                            .setArrivalDirection(s.arrival_direction)
                            .setStopAtLine(s.stop_at_line)
                            .async();
                        _stepState    = StepState::ACTIVE;
                        _stepStart_ms = now;
                    }
                    else if (PathTracker.isIdle())
                    {
                        Serial.print(F("[Mission] arrived "));
                        Serial.println(s.text == nullptr ? "?" : s.text);
                        // Buffer width tracks EventTrace's ring slot so the
                        // OLED truncation rule stays single-sourced. snprintf
                        // truncates long node names cleanly within MAX_LINE_LEN.
                        char buf[EventTraceClass::MAX_LINE_LEN];
                        snprintf(buf, sizeof(buf), "ARR %s",
                                 s.text == nullptr ? "?" : s.text);
                        EventTrace.pushEvent(buf);
                        _advanceStep();
                    }
                    break;
                }

                case StepType::WAIT:
                {
                    if (_stepState == StepState::ENTER)
                    {
#ifdef CTPS_RECORD
                        ctps::recordWait(s.duration_ms);
#endif
                        _stepStart_ms = now;
                        _stepState    = StepState::ACTIVE;
                    }
                    if (now - _stepStart_ms >= s.duration_ms)
                    {
                        _advanceStep();
                    }
                    break;
                }

                case StepType::PRINT:
                {
#ifdef CTPS_RECORD
                    ctps::recordPrint(s.text ? s.text : "");
#endif
                    Serial.print(F("[Mission] "));
                    Serial.println(s.text == nullptr ? "" : s.text);
                    EventTrace.pushEvent(s.text == nullptr ? "" : s.text);
                    _advanceStep();
                    break;
                }

                case StepType::BEEP:
                {
                    if (_stepState == StepState::ENTER)
                    {
#ifdef CTPS_RECORD
                        ctps::recordBeep(s.freq_hz, s.duration_ms);
#endif
                        _beep(s.freq_hz, (uint16_t) s.duration_ms);
                        _stepStart_ms = now;
                        _stepState    = StepState::ACTIVE;
                    }
                    if (now - _stepStart_ms >= s.duration_ms)
                    {
                        _advanceStep();
                    }
                    break;
                }

                case StepType::LED:
                {
                    if (_stepState == StepState::ENTER)
                    {
#ifdef CTPS_RECORD
                        ctps::recordLed(s.r, s.g, s.b, s.duration_ms);
#endif
                        _setLeds(s.r, s.g, s.b);
                        if (s.duration_ms == 0)
                        {
                            _advanceStep();
                            return;
                        }
                        _stepStart_ms = now;
                        _stepState    = StepState::ACTIVE;
                    }
                    if (now - _stepStart_ms >= s.duration_ms)
                    {
                        _setLeds(0, 0, 0);
                        _advanceStep();
                    }
                    break;
                }

                case StepType::TASK:
                {
                    if (_stepState == StepState::ENTER)
                    {
                        const TaskRegistryEntry* entry =
                            Tasks.findTask(s.task_name == nullptr ? "" : s.task_name);
                        if (entry == nullptr)
                        {
                            Serial.print(F("[Mission] unknown task '"));
                            Serial.print(s.task_name == nullptr ? "?" : s.task_name);
                            Serial.println(F("'"));
                            EventTrace.pushEvent("NV TASK");
                            _enterFaultBeeping(millis());
                            return;
                        }

                        // Boundary event before dispatch. Tags are kept short
                        // (3 chars + space) so the OLED's MAX_LINE_LEN=20 budget
                        // leaves 15 chars for the task name — fits Phase-5/6
                        // names like "mosaicPlace" / "parkInBay" / "debugYield"
                        // without truncation.
                        char buf[EventTraceClass::MAX_LINE_LEN];
                        snprintf(buf, sizeof(buf), "T> %s",
                                 s.task_name == nullptr ? "?" : s.task_name);
                        EventTrace.pushEvent(buf);

#ifdef CTPS_RECORD
                        ctps::recordTaskStart(s.task_name ? s.task_name : "",
                                              s.task_args.bytes,
                                              sizeof(s.task_args.bytes));
#endif

                        const bool ok = Tasks.runTask(s.task_name, s.task_args);

                        // Post-runTask fault check — a PathTracker fault that latched
                        // mid-task did NOT trigger the top-of-RUNNING check at line 248
                        // because the inTask() guard short-circuited Mission.update().
                        // Check here so a task-internal fault triggers the fault path
                        // BEFORE we advance past the step (mirrors how GOTO never
                        // advances on a faulted leg). Pass millis() fresh — `now`
                        // captured at the top of update() is stale by however long
                        // the (potentially multi-second) task body ran, and using
                        // the stale value would collapse the FAULT_BEEPING beep
                        // schedule.
                        if (PathTracker.hasFault())
                        {
                            _emitFault();
                            _enterFaultBeeping(millis());
                            return;
                        }

                        // If the task body called Mission.abort() mid-flight,
                        // _state is now IDLE and _count/_idx have been reset.
                        // Bail BEFORE pushing a misleading "TASK OK" event or
                        // advancing _idx past _count — the operator's abort
                        // should not be papered over by a "task completed" log.
                        if (_state != State::RUNNING)
                        {
                            return;
                        }

                        snprintf(buf, sizeof(buf), ok ? "T+ %s" : "T- %s",
                                 s.task_name == nullptr ? "?" : s.task_name);
                        EventTrace.pushEvent(buf);
                        if (!ok)
                        {
                            Serial.print(F("[Mission] task '"));
                            Serial.print(s.task_name == nullptr ? "?" : s.task_name);
                            Serial.println(F("' yielded"));
                        }
#ifdef CTPS_RECORD
                        ctps::recordTaskEnd(s.task_name ? s.task_name : "", ok);
#endif
                        _advanceStep();
                    }
                    // No ACTIVE branch — runTask is fake-blocking; the in-flight period
                    // is fully covered by Tasks.inTask() short-circuiting Mission.update()
                    // at the top. _stepState stays in ENTER until _advanceStep() resets it.
                    break;
                }
            }
            return;
        }

        case State::FAULT_BEEPING:
        {
            const uint32_t now = millis();
            const uint32_t half_ms  = Config::MISSION_FAULT_BEEP_MS;
            const uint32_t cycle_ms = half_ms * 2u;
            const uint32_t elapsed = now - _faultPhaseStart_ms;
            const uint8_t  desiredIdx = (uint8_t) (elapsed / cycle_ms);

            // LEDs flash in sync with the beep: red during the first
            // half-cycle (while the tone plays), off during the gap.
            const bool ledOnPhase = (elapsed % cycle_ms) < half_ms;
            if (ledOnPhase != _faultLedOn)
            {
                _faultLedOn = ledOnPhase;
                _setLeds(ledOnPhase ? 255 : 0, 0, 0);
            }

            if (desiredIdx > _faultBeepIdx)
            {
                _faultBeepIdx = desiredIdx;
                if (_faultBeepIdx >= Config::MISSION_FAULT_BEEPS)
                {
                    // Hold LEDs solid red while waiting for the ack so the
                    // operator can still see the chassis is in fault.
                    _setLeds(255, 0, 0);
                    _faultLedOn = true;
                    Serial.println(F("[Mission] press BTN_DOWN to clear fault"));
                    EventTrace.pushEvent("PRESS BTN_DOWN");
                    _state = State::FAULT_AWAIT_RESET;
                    _btnPrev = MiniR4.BTN_DOWN.getState();
                }
                else
                {
                    _beep(Config::MISSION_FAULT_BEEP_HZ, Config::MISSION_FAULT_BEEP_MS);
                }
            }
            return;
        }

        case State::FAULT_AWAIT_RESET:
        {
            const bool btnNow = MiniR4.BTN_DOWN.getState();
            if (btnNow && !_btnPrev)
            {
                _setLeds(0, 0, 0);
                _faultLedOn = false;
                if (PathTracker.hasFault())
                {
                    PathTracker.clearFault();
                }
                // Queue was cleared on fault entry; resetting indices is
                // belt-and-braces in case a stale path slipped through.
                _count       = 0;
                _idx         = 0;
                _state       = State::IDLE;
                _buildError  = false;    // clear so a new buildMission can re-arm
                Serial.println(F("[Mission] fault cleared; mission ended"));
                EventTrace.pushEvent("MIS RESET");
            }
            _btnPrev = btnNow;
            return;
        }
    }
}
