#pragma once

namespace NKSwerve {

class IMutex {
public:
    virtual ~IMutex() = default;

    virtual void Lock() = 0;
    virtual void Unlock() = 0;
};

// NKSwerve's own RAII helper, mirroring std::lock_guard, depending on nothing
// but IMutex.
class LockGuard {
public:
    explicit LockGuard(IMutex& mutex) : m_mutex(mutex) { m_mutex.Lock(); }
    ~LockGuard() { m_mutex.Unlock(); }

    LockGuard(const LockGuard&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;

private:
    IMutex& m_mutex;
};

}  // namespace NKSwerve
