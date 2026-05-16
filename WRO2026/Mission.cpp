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
#include "PathTracker.h"
#include "routes.h"

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
    return true;
}

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
    _push(s);
    return *this;
}

MissionClass& MissionClass::wait(uint32_t ms)
{
    Step s{};
    s.type        = StepType::WAIT;
    s.duration_ms = ms;
    _push(s);
    return *this;
}

MissionClass& MissionClass::print(const char* msg)
{
    Step s{};
    s.type = StepType::PRINT;
    s.text = msg;
    _push(s);
    return *this;
}

MissionClass& MissionClass::beep(uint16_t freq_hz, uint32_t duration_ms)
{
    Step s{};
    s.type        = StepType::BEEP;
    s.freq_hz     = freq_hz;
    s.duration_ms = duration_ms;
    _push(s);
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
    _push(s);
    return *this;
}

void MissionClass::start()
{
    if (_count == 0)
    {
        Serial.println(F("[Mission] start with empty queue; no-op"));
        return;
    }
    _idx = 0;
    _stepState = StepState::ENTER;
    _state = State::RUNNING;
    Serial.print(F("[Mission] start; steps="));
    Serial.println(_count);
    EventTrace.pushEvent("MIS START");
}

void MissionClass::abort()
{
    Drive.stop(true);
    _count = 0;
    _idx   = 0;
    _state = State::IDLE;
    _stepState = StepState::ENTER;
    Serial.println(F("[Mission] abort"));
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
    // MatrixMiniR4 exposes the on-board RGB pair via the LED module. The
    // exact method name may differ across library revisions; this is the
    // setColor(idx, r, g, b) signature in the current header.
    MiniR4.LED.setColor(0, r, g, b);
    MiniR4.LED.setColor(1, r, g, b);
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
    Serial.println(info.goalNode);

    // Push a short tag to OLED (NAV FAULT <mn>).
    char buf[20];
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
                Drive.stop(true);
                _emitFault();
                _setLeds(255, 0, 0);
                _state = State::FAULT_BEEPING;
                _faultBeepIdx       = 0;
                _faultPhaseStart_ms = millis();
                tone(Config::BUZZER_PIN,
                     Config::MISSION_FAULT_BEEP_HZ,
                     Config::MISSION_FAULT_BEEP_MS);
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
                            // Synthesise a fault path the same way PathTracker would.
                            _setLeds(255, 0, 0);
                            _state = State::FAULT_BEEPING;
                            _faultBeepIdx       = 0;
                            _faultPhaseStart_ms = now;
                            tone(Config::BUZZER_PIN,
                                 Config::MISSION_FAULT_BEEP_HZ,
                                 Config::MISSION_FAULT_BEEP_MS);
                            return;
                        }
                        Serial.print(F("[Mission] goTo "));
                        Serial.println(s.text == nullptr ? "?" : s.text);
                        PathTracker.goTo(s.goal).async();
                        _stepState    = StepState::ACTIVE;
                        _stepStart_ms = now;
                    }
                    else if (PathTracker.isIdle())
                    {
                        Serial.print(F("[Mission] arrived "));
                        Serial.println(s.text == nullptr ? "?" : s.text);
                        char buf[20];
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
                        tone(Config::BUZZER_PIN, s.freq_hz, (uint32_t) s.duration_ms);
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
            }
            return;
        }

        case State::FAULT_BEEPING:
        {
            const uint32_t now = millis();
            const uint32_t cycle_ms = (uint32_t) Config::MISSION_FAULT_BEEP_MS * 2u;
            const uint32_t elapsed = now - _faultPhaseStart_ms;
            const uint8_t  desiredIdx = (uint8_t) (elapsed / cycle_ms);

            if (desiredIdx > _faultBeepIdx)
            {
                _faultBeepIdx = desiredIdx;
                if (_faultBeepIdx >= Config::MISSION_FAULT_BEEPS)
                {
                    Serial.println(F("[Mission] press BTN_DOWN to clear fault"));
                    EventTrace.pushEvent("PRESS BTN_DOWN");
                    _state = State::FAULT_AWAIT_RESET;
                    _btnPrev = MiniR4.BTN_DOWN.getState();
                }
                else
                {
                    tone(Config::BUZZER_PIN,
                         Config::MISSION_FAULT_BEEP_HZ,
                         Config::MISSION_FAULT_BEEP_MS);
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
                if (PathTracker.hasFault())
                {
                    PathTracker.clearFault();
                }
                _count = 0;
                _idx   = 0;
                _state = State::IDLE;
                Serial.println(F("[Mission] fault cleared; mission ended"));
                EventTrace.pushEvent("MIS RESET");
            }
            _btnPrev = btnNow;
            return;
        }
    }
}
