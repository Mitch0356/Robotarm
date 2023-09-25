
#include "driver.hpp"

int main([[maybe_unused]]int argc, [[maybe_unused]]char const *argv[])
{
    auto flunk = [](){while(1){/*hee hee haw*/}};
    std::thread t(flunk);
    driver d("config.json","/dev/ttyUSB0");    
    d.move_arm_pos(-45, d.get_joint(0), 5000);
    t.join();
    return 0;
}

//Straight up #0 P1500 #1 P1500 #2 P700 #3 P1500 #4 P1500 #5 P1500 T1000
//Ready #0 P1500 #1 P1900 #2 P1900 #3 P1500 #4 P1500 #5 P1500 T5000
//Park  #0 P1500 #1 P2100 #2 P2100 #3 P1000 #4 P1500 #5 P1500 T1000
//Init  #0 P1500 #1 P1500 #2 P1500 #3 P1500 #4 P1500 #5 P1500 T1000