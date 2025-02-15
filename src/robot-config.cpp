#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftback = motor(PORT8, ratio18_1, false);
motor leftcenter = motor(PORT9, ratio18_1, true);
motor leftfront = motor(PORT10, ratio18_1, true);
motor rightback = motor(PORT1, ratio18_1, true);
motor rightcenter = motor(PORT2, ratio18_1, false);
motor rightfront = motor(PORT3, ratio18_1, false);
motor intake = motor(PORT6, ratio18_1, false);
motor hook = motor(PORT5, ratio18_1, true);
motor ladybrown = motor(PORT19, ratio18_1, false);
digital_out blocker = digital_out(Brain.ThreeWirePort.B);
digital_out clamp = digital_out(Brain.ThreeWirePort.A);
digital_out intk = digital_out(Brain.ThreeWirePort.C);
inertial inertials = inertial(PORT4);
rotation spin = rotation(PORT18, false);
distance distancer = distance(PORT15);



// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void)
{
    // nothing to initialize
}