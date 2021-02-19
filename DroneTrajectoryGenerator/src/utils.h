#ifndef UTILS_H_
#define UTILS_H_

#include <chrono>
#include <mutex>

using namespace msr::airlib;
namespace utils {

double ctime() {
    return std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
}

}
#endif
