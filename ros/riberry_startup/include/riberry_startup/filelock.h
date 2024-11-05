#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/file.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <thread>


// The code has been adapted to ensure consistency with the FileLock behavior in a Python program.
// https://github.com/tox-dev/filelock/blob/main/src/filelock/_unix.py
class FileLock {
public:
    FileLock(const std::string& filename, int timeoutSecs = 10) : lockFile(filename), timeoutSecs(timeoutSecs), lockFileFd(-1), mode(0666) {}

    void acquire() {
        ensureDirectoryExists();
        int openFlags = O_RDWR | O_TRUNC;
        if (!std::filesystem::exists(lockFile)) {
            openFlags |= O_CREAT;
        }
        lockFileFd = open(lockFile.c_str(), openFlags, mode);
        if (lockFileFd == -1) {
            throw std::runtime_error("Failed to open lock file");
        }
        try {
            fchmod(lockFileFd, mode);
        } catch (...) {
            // Ignore PermissionError: This locked is not owned by this UID
        }

        auto start = std::chrono::steady_clock::now();
        while (true) {
            if (flock(lockFileFd, LOCK_EX | LOCK_NB) != -1) {
                break;
            }
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(timeoutSecs)) {
                close(lockFileFd);
                lockFileFd = -1;
                throw std::runtime_error("Failed to acquire file lock: timeout");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void release() {
        if (lockFileFd != -1) {
            flock(lockFileFd, LOCK_UN);
            close(lockFileFd);
            lockFileFd = -1;
        }
    }

    ~FileLock() {
        if (lockFileFd != -1) {
            release();
        }
    }

private:
    std::string lockFile;
    int timeoutSecs;
    int lockFileFd;
    mode_t mode;

    void ensureDirectoryExists() {
        std::filesystem::path dir = std::filesystem::path(lockFile).parent_path();
        if (!std::filesystem::exists(dir)) {
            std::filesystem::create_directories(dir);
        }
    }
};
