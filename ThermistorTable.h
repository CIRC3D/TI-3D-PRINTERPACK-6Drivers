#ifndef THERMISTORTABLE_H_
#define THERMISTORTABLE_H_

// Thermistor lookup table for RepRap Temperature Sensor Boards (http://make.rrrf.org/ts)
// See this page:  
// http://dev.www.reprap.org/bin/view/Main/Thermistor
// for details of what goes in this table.
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4700 --beta=4066 --max-adc=1023
// r0: 100000
// t0: 25
// r1: 0
// r2: 4700
// beta: 4066
// max adc: 4095

/* Modified by Ryan Brown for Texas Instruments
        Original project is Tonokip, repository: https://github.com/tonokip/Tonokip-Firmware
	Works with Energia for MSP430F5529 now, not Arduino
	December 2014
        This code is licensed as GPLv2
   For use with the CIRC3D BoosterPack 
*/

#define NUMTEMPS 108
const short temptable[NUMTEMPS][2] = {
   {1, 1760},
   {39, 433},
   {77, 355},
   {115, 317},
   {153, 292},
   {191, 274},
   {229, 260},
   {267, 249},
   {305, 239},
   {343, 231},
   {381, 223},
   {419, 217},
   {457, 211},
   {495, 206},
   {533, 201},
   {571, 197},
   {609, 192},
   {647, 189},
   {685, 185},
   {723, 181},
   {761, 178},
   {799, 175},
   {837, 172},
   {875, 169},
   {913, 167},
   {951, 164},
   {989, 162},
   {1027, 159},
   {1065, 157},
   {1103, 155},
   {1141, 153},
   {1179, 151},
   {1217, 149},
   {1255, 147},
   {1293, 145},
   {1331, 143},
   {1369, 141},
   {1407, 139},
   {1445, 138},
   {1483, 136},
   {1521, 134},
   {1559, 132},
   {1597, 131},
   {1635, 129},
   {1673, 128},
   {1711, 126},
   {1749, 125},
   {1787, 123},
   {1825, 122},
   {1863, 120},
   {1901, 119},
   {1939, 117},
   {1977, 116},
   {2015, 114},
   {2053, 113},
   {2091, 112},
   {2129, 110},
   {2167, 109},
   {2205, 108},
   {2243, 106},
   {2281, 105},
   {2319, 103},
   {2357, 102},
   {2395, 101},
   {2433, 99},
   {2471, 98},
   {2509, 97},
   {2547, 95},
   {2585, 94},
   {2623, 93},
   {2661, 91},
   {2699, 90},
   {2737, 89},
   {2775, 87},
   {2813, 86},
   {2851, 84},
   {2889, 83},
   {2927, 81},
   {2965, 80},
   {3003, 79},
   {3041, 77},
   {3079, 76},
   {3117, 74},
   {3155, 72},
   {3193, 71},
   {3231, 69},
   {3269, 68},
   {3307, 66},
   {3345, 64},
   {3383, 62},
   {3421, 60},
   {3459, 58},
   {3497, 56},
   {3535, 54},
   {3573, 52},
   {3611, 50},
   {3649, 48},
   {3687, 45},
   {3725, 42},
   {3763, 39},
   {3801, 36},
   {3839, 33},
   {3877, 29},
   {3915, 24},
   {3953, 19},
   {3991, 12},
   {4029, 3},
   {4067, -12}
};
#endif

