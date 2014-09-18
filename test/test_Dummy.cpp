#include <boost/test/unit_test.hpp>
#include <rosa_localization/Dummy.hpp>

using namespace rosa_localization;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    rosa_localization::DummyClass dummy;
    dummy.welcome();
}
