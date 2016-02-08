#include "settings.h"
#include "eeprom.h"

unsigned short settings[SETTINGS_CNT];

uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777, 0x8888,0x9999, 0xAAAA};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0, 0, 0, 0};

extern unsigned char polepairs;
extern unsigned short alignmentdc;
extern unsigned short holdrpm;
extern unsigned short startuprpmpersecond;
extern volatile unsigned short minstep;

extern unsigned short rpmGoal;

#define POLE_PAIRS_MAX                  64
#define POLE_PAIRS_MIN                  1
#define ALIGNMENTDC_MAX                 600
#define ALIGNMENTDC_MIN                 100
#define HOLD_RPM_MAX                    1000
#define HOLD_RPM_MIN                    100
#define STARTUP_RPM_PER_SECOND_MAX      1000
#define STARTUP_RPM_PER_SECOND_MIN      5
#define RPM_GOAL_MAX                    20000
#define RPM_GOAL_MIN                    300



unsigned short getParam(unsigned short addr)
{
    if((addr>=0)&&(addr<SETTINGS_CNT)) return settings[addr];
    return 0;
}

void setParam(unsigned short addr, unsigned short value)
{
    if((addr>=0)&&(addr<SETTINGS_CNT)) {
        switch(addr) {
            case 0: settings[addr] = value;break;
            case 1:
                if((value>=POLE_PAIRS_MIN)&&(value<=POLE_PAIRS_MAX)) {
                    settings[addr] = VarDataTab[0] = polepairs = value;
                    EE_WriteVariable(VirtAddVarTab[0], VarDataTab[0]);
                }
                break;
            case 2:
                if((value>=ALIGNMENTDC_MIN)&&(value<=ALIGNMENTDC_MAX)) {
                    settings[addr] = VarDataTab[1] = alignmentdc = value;
                    EE_WriteVariable(VirtAddVarTab[1], VarDataTab[1]);
                }
                break;
            case 3:
                if((value>=HOLD_RPM_MIN)&&(value<=HOLD_RPM_MAX)) {
                    settings[addr] = VarDataTab[2] = holdrpm = value;
                    EE_WriteVariable(VirtAddVarTab[2], VarDataTab[2]);
                }
                break;
            case 4:
                if((value>=STARTUP_RPM_PER_SECOND_MIN)&&(value<=STARTUP_RPM_PER_SECOND_MAX)) {
                    settings[addr] = VarDataTab[3] = startuprpmpersecond = value;
                    EE_WriteVariable(VirtAddVarTab[3], VarDataTab[3]);
                }
                break;
            case 5:
                if((value>=RPM_GOAL_MIN)&&(value<=RPM_GOAL_MAX)) {
                    settings[addr] = VarDataTab[4] = rpmGoal = value;
                    EE_WriteVariable(VirtAddVarTab[4], VarDataTab[4]);
                }
                break;
        }
        minstep = 200000/polepairs/holdrpm;
    }

}

void read_settings(void)
{
    uint16_t status = EE_ReadVariable(VirtAddVarTab[0], &VarDataTab[0]);
    if(status!=0) {EE_WriteVariable(VirtAddVarTab[0],polepairs);}
    else if((VarDataTab[0]>=POLE_PAIRS_MIN)&&(VarDataTab[0]<=POLE_PAIRS_MAX)) polepairs = VarDataTab[0];
    settings[1] = polepairs;

    status = EE_ReadVariable(VirtAddVarTab[1], &VarDataTab[1]);
    if(status!=0) {EE_WriteVariable(VirtAddVarTab[1],alignmentdc);}
    else if((VarDataTab[1]>=ALIGNMENTDC_MIN)&&(VarDataTab[1]<=ALIGNMENTDC_MAX)) alignmentdc = VarDataTab[1];
    settings[2] = alignmentdc;

    status = EE_ReadVariable(VirtAddVarTab[2], &VarDataTab[2]);
    if(status!=0) {EE_WriteVariable(VirtAddVarTab[2],holdrpm);}
    else if((VarDataTab[2]>=HOLD_RPM_MIN)&&(VarDataTab[2]<=HOLD_RPM_MAX)) holdrpm = VarDataTab[2];
    settings[3] = holdrpm;

    status = EE_ReadVariable(VirtAddVarTab[3], &VarDataTab[3]);
    if(status!=0) {EE_WriteVariable(VirtAddVarTab[3],startuprpmpersecond);}
    else if((VarDataTab[3]>=STARTUP_RPM_PER_SECOND_MIN)&&(VarDataTab[3]<=STARTUP_RPM_PER_SECOND_MAX)) startuprpmpersecond = VarDataTab[3];
    settings[4] = startuprpmpersecond;

    status = EE_ReadVariable(VirtAddVarTab[4], &VarDataTab[4]);
    if(status!=0) {EE_WriteVariable(VirtAddVarTab[4],rpmGoal);}
    else if((VarDataTab[4]>=RPM_GOAL_MIN)&&(VarDataTab[4]<=RPM_GOAL_MAX)) rpmGoal = VarDataTab[4];
    settings[5] = rpmGoal;

    minstep = 200000/polepairs/holdrpm;
}
