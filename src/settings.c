#include "settings.h"
#include "eeprom.h"

unsigned short settings[SETTINGS_CNT];

uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777, 0x8888,0x9999};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0, 0, 0};

extern unsigned char polepairs;
extern unsigned short alignmentdc;
extern unsigned short holdrpm;
extern unsigned short startuprpmpersecond;
extern volatile unsigned short minstep;

extern unsigned short rpmGoal;

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
                if((value>0)&&(value<=64)) {
                    settings[addr] = VarDataTab[0] = polepairs = value;
                    EE_WriteVariable(VirtAddVarTab[0], VarDataTab[0]);
                }
                break;
            case 2:
                if((value>0)&&(value<=1100)) {
                    settings[addr] = VarDataTab[1] = alignmentdc = value;
                    EE_WriteVariable(VirtAddVarTab[1], VarDataTab[1]);
                }
                break;
            case 3:
                if((value>100)&&(value<=5000)) {
                    settings[addr] = VarDataTab[2] = holdrpm = value;
                    EE_WriteVariable(VirtAddVarTab[2], VarDataTab[2]);
                }
                break;
            case 4:
                if((value>0)&&(value<=1000)) {
                    settings[addr] = VarDataTab[3] = startuprpmpersecond = value;
                    EE_WriteVariable(VirtAddVarTab[3], VarDataTab[3]);
                }
                break;
            case 5:
                if((value>=0)&&(value<=5000)) {
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
    else if((VarDataTab[0]>0)&&(VarDataTab[0]<=64)) polepairs = VarDataTab[0];
    settings[1] = polepairs;

    status = EE_ReadVariable(VirtAddVarTab[1], &VarDataTab[1]);
    if(status!=0) {EE_WriteVariable(VirtAddVarTab[1],alignmentdc);}
    else if((VarDataTab[1]>0)&&(VarDataTab[1]<=1100)) alignmentdc = VarDataTab[1];
    settings[2] = alignmentdc;

    status = EE_ReadVariable(VirtAddVarTab[2], &VarDataTab[2]);
    if(status!=0) {EE_WriteVariable(VirtAddVarTab[2],holdrpm);}
    else if((VarDataTab[2]>100)&&(VarDataTab[2]<=5000)) holdrpm = VarDataTab[2];
    settings[3] = holdrpm;

    status = EE_ReadVariable(VirtAddVarTab[3], &VarDataTab[3]);
    if(status!=0) {EE_WriteVariable(VirtAddVarTab[3],startuprpmpersecond);}
    else if((VarDataTab[3]>0)&&(VarDataTab[3]<=1000)) startuprpmpersecond = VarDataTab[3];
    settings[4] = startuprpmpersecond;

    status = EE_ReadVariable(VirtAddVarTab[4], &VarDataTab[4]);
    if(status!=0) {EE_WriteVariable(VirtAddVarTab[4],rpmGoal);}
    else if((VarDataTab[4]>=0)&&(VarDataTab[4]<=5000)) rpmGoal = VarDataTab[4];
    settings[5] = rpmGoal;

    minstep = 200000/polepairs/holdrpm;
}
