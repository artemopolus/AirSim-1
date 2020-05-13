
#include "SettingsTest.hpp"
#include "PixhawkTest.hpp"
#include "SimpleFlightTest.hpp"
#include "WorkerThreadTest.hpp"
#include "QuaternionTest.hpp"
#include "CelestialTests.hpp"
#include "PlaneTest.hpp" /* наш тест */

int main()
{
	std::cout << "Start unit test";
    using namespace msr::airlib;

    std::unique_ptr<TestBase> tests[] = {
        std::unique_ptr<TestBase>(new QuaternionTest()),
       // std::unique_ptr<TestBase>(new CelestialTest()),
        std::unique_ptr<TestBase>(new SettingsTest()),
        //std::unique_ptr<TestBase>(new SimpleFlightTest())
		std::unique_ptr<TestBase>(new PlaneTest()) /* тест самолета */
        //,
        //std::unique_ptr<TestBase>(new PixhawkTest()),
        //std::unique_ptr<TestBase>(new WorkerThreadTest())
    };

    for (auto& test : tests)
        test->run();

	std::cout << "done!"<< std::endl;
    return 0;
}

