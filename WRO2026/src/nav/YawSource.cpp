/**
 * @file YawSource.cpp
 * @brief Implementation of the `YawSource` singleton (see YawSource.h).
 * @author WRO 2026
 *
 * @details
 * Owns the external MXColor-protocol gyro board (I2C addr 0x22 on `Wire1`), the tick-scoped yaw
 * cache, and the per-read dead-byte gate. The pure wire decode + dead-byte math lives in
 * `GyroDecodePolicy.h` (scale/sign passed as params, ADR-074); this file is only the transport,
 * the cache, and the health flag around that math. The SDK (`MatrixColor`, `Wire1`) is
 * confined here so `YawSource.h` stays `<stdint.h>`-only for `Drive.h` / `Localization.cpp`.
 */

#include "YawSource.h"

#include <MatrixMiniR4.h>

#include "GyroDecodePolicy.h"

#include "../../Config.h"

namespace
{
/// The external gyro board. File-static (not a `YawSourceClass` member) so `<MatrixMiniR4.h>`
/// stays out of the header. `begin()` wires `_pWire` / `_ch`; every read runs through it.
MatrixColor _gyro;

// ---- Tick-scoped cache — written ONLY by update(), served by readYawDeg -----------------------
/// Whether the cache holds a value from a successful device read.
bool     _cacheValid   = false;
/// Last cached app-frame heading (degrees, wrapped to (-180, 180]).
double   _cacheYawDeg  = 0.0;
/// `millis()` at which the cache was last written by `update()`.
uint32_t _cacheStampMs = 0;

// ---- Health -----------------------------------------------------------------------------------
/// `true` iff the last DEVICE read passed the dead-byte gate (i.e. was not all-zero).
bool _healthy = false;

/**
 * @brief Read one raw (G,R) sample off the gyro, decode it, and run the dead-byte gate.
 *
 * Mutates only the file-static `_healthy` flag — never the cache, which is `update()`'s alone.
 * Safe to call from the `const` `readYawDeg` because that state is not a `YawSourceClass` member.
 *
 * Reads are gated ONLY by dead-byte detection: an all-zero (G,R) pair is the vendor's only
 * distinguishable dead-bus signature, and accepting it would inject the `-32768`-derived garbage
 * heading `decodeGyroYawDeg(0, 0, ...)` yields. A time-normalized plausibility gate (a torn-read
 * guard the known-good reference omits) is deferred to bench verification.
 *
 * @param[out] appDegOut Accepted app-frame heading (degrees) — written ONLY on `true`.
 * @return `true` on an accepted sample (@p appDegOut written); `false` on a dead read
 *         (@p appDegOut untouched).
 */
bool sampleGyro(double& appDegOut)
{
    // G is the HIGH byte, R the LOW byte. Read them BACK-TO-BACK: each getColor() re-runs an
    // internal i2cMUXSelect(), so anything touching the bus between the two lands the second read
    // on the wrong board. Nothing may tick / delay / log between these two lines.
    const uint8_t hiG = _gyro.getColor(G);
    const uint8_t loR = _gyro.getColor(R);

    // Dead bus — an all-zero pair is the vendor's only distinguishable dead-bus signature.
    if (gyroBytesDead(hiG, loR))
    {
        _healthy = false;
        return false;
    }

    // Decode to the app frame. GYRO_YAW_SIGN is applied HERE, inside the policy, and NOWHERE
    // else — a second application would double-negate (the ADR-045 one-place rule, now per-frame).
    const float decoded = decodeGyroYawDeg(hiG, loR, Config::GYRO_SCALE, Config::GYRO_YAW_SIGN);

    _healthy  = true;
    appDegOut = (double) decoded;
    return true;
}
}    // namespace

YawSourceClass YawSource;    // single global instance

bool YawSourceClass::begin()
{
    // Wire the I2C transport BEFORE any read: MatrixColor caches the bus pointer + mux channel
    // and uses them on every transaction. Port label != mux channel — the 0-based PCA9548A
    // channel is (PORT - 1): I2C1 -> ch 0 ... I2C4 -> ch 3. Getting this wrong reads nothing.
    _gyro._pWire = &Wire1;
    _gyro._ch    = static_cast<uint8_t>(Config::GYRO_I2C_PORT - 1);

    // Clean bring-up: drop any cache / health state a prior begin() (or a re-arm) left behind so
    // the first post-boot read seeds a fresh cache rather than serving a stale one.
    _cacheValid = false;
    _healthy    = false;

    // Attempt the vendor begin(), but treat its result as INCONCLUSIVE (see the @note in the
    // header): it ID-checks 0x43 and writes a CONFIG reset the gyro board may not implement, so a
    // false here does NOT mean the hardware is dead. The return is deliberately discarded.
    (void) _gyro.begin();

    // Authoritative liveness gate: one G/R pair, read back-to-back. Pass unless it is all-zero (a
    // dead bus / no device on this mux channel). Only that makes begin() fail (AC-6).
    const uint8_t hiG = _gyro.getColor(G);
    const uint8_t loR = _gyro.getColor(R);
    if (gyroBytesDead(hiG, loR))
    {
        return false;
    }

    // Zero the hardware once (getColor(K) side effect) so the app-frame heading starts near 0,
    // then seed the cache from a fresh post-zero read.
    zeroHardware();

    double seedYaw = 0.0;
    if (sampleGyro(seedYaw))
    {
        _cacheValid   = true;
        _cacheYawDeg  = seedYaw;
        _cacheStampMs = millis();
    }
    // The liveness gate already proved a live device, so report success even if the single seed
    // read was gated out — the inline refresh in readYawDeg recovers on the next tick.
    return true;
}

void YawSourceClass::update()
{
    // One device read per tick. Refresh the cache with a fresh, gated sample + timestamp; a
    // gated-out read leaves the prior good value untouched (it ages out via GYRO_CACHE_MAX_AGE_MS),
    // so a transient miss never overwrites a good cache with garbage. update() is the SOLE writer.
    double yaw = 0.0;
    if (sampleGyro(yaw))
    {
        _cacheValid   = true;
        _cacheYawDeg  = yaw;
        _cacheStampMs = millis();
    }
}

void YawSourceClass::zeroHardware()
{
    // getColor(K) (register 10, Device_BLACK) is a READ WITH A SIDE EFFECT: it triggers the gyro
    // board's zero-heading reset. The returned byte carries no heading and is discarded.
    (void) _gyro.getColor(K);
}

bool YawSourceClass::isHealthy() const
{
    return _healthy;
}

bool YawSourceClass::readYawDeg(double& yawOut) const
{
    // Serve the tick-scoped cache when it is present and no older than GYRO_CACHE_MAX_AGE_MS: this
    // is what collapses N consumer reads in one tick (after an update()) into a SINGLE device
    // transaction (AC-5). The subtraction is unsigned, so it is rollover-safe.
    if (_cacheValid
        && (uint32_t) (millis() - _cacheStampMs) <= Config::GYRO_CACHE_MAX_AGE_MS)
    {
        yawOut = _cacheYawDeg;
        return true;
    }

    // Cache absent or stale: refresh INLINE with one gated device read. This deliberately does NOT
    // write the cache — update() is the sole cache writer — so a consumer polling readYawDeg
    // without a driving update() (or under a frozen clock, as the host suites do) always observes
    // a live value rather than one pinned by its own first read. On a gated-out read leave yawOut
    // UNTOUCHED and return false (the existing contract: a transient miss must never inject a
    // 0.0-derived heading).
    double yaw = 0.0;
    if (!sampleGyro(yaw))
    {
        return false;
    }
    yawOut = yaw;
    return true;
}

double YawSourceClass::readYawDegOr(double fallback) const
{
    double y = 0.0;
    return readYawDeg(y) ? y : fallback;
}
