#ifndef msr_AirLibUnitTests_SettingsTest_hpp
#define msr_AirLibUnitTests_SettingsTest_hpp

#include "TestBase.hpp"
#include "common/Settings.hpp"

namespace msr { namespace airlib {

class SettingsTest : public TestBase {
public:
	SettingsTest()
	{
		std::cout << "Setting test"<< std::endl;
	}
    virtual void run() override
    {
        Settings& settings = Settings::loadJSonFile("settings.json");
        unused(settings);
    }
};


}}
#endif