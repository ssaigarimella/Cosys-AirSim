#ifndef msr_AirLibUnitTests_TestBase_hpp
#define msr_AirLibUnitTests_TestBase_hpp

#include <string>
#include <exception>

namespace msr { namespace airlib {

class TestBase
{
public:
    virtual void run() = 0;

    void testAssert(double lhs, double rhs, const std::string& message) {
        testAssert(lhs == rhs, message);
    }

    void testAssert(bool condition, const std::string& message) {
        if (!condition) {
            Utils::DebugBreak();
            throw std::runtime_error(message.c_str());
        }
    }
};

} }

#endif