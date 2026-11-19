// Pure input-validity policy. OpenXR poses retain their original right-handed axes.
#pragma once
#include <cmath>
#include <cstdint>

namespace tg3 {
constexpr uint64_t kRequiredLocationFlags=0x1|0x2|0x4|0x8;
inline bool Tracked(uint64_t flags, bool actionActive, bool focused) {
    return focused && actionActive && (flags & kRequiredLocationFlags)==kRequiredLocationFlags;
}
inline bool FinitePose(float x,float y,float z,float qx,float qy,float qz,float qw) {
    float norm=qx*qx+qy*qy+qz*qz+qw*qw;
    return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z)&&std::isfinite(norm)&&
        std::fabs(norm-1.f)<=.02f&&std::fabs(x)<=20&&std::fabs(y)<=20&&std::fabs(z)<=20;
}
inline float Analog(float value) { return std::isfinite(value) ? std::fmax(0.f,std::fmin(1.f,value)) : 0.f; }
}
