using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor leftback;
extern motor leftcenter;
extern motor leftfront;
extern motor rightback;
extern motor rightcenter;
extern motor rightfront;
extern motor intake;
extern motor hook;
extern motor ladybrown;
extern digital_out blocker;
extern digital_out clamp;
extern digital_out intk;
extern inertial inertials;
extern rotation spin;
extern distance distancer;
extern vision tyler;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );