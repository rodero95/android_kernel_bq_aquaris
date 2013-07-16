#include <mach/mt6577_typedefs.h>

#define FGAUGE_VOLTAGE_FACTOR           2.44 // mV
#define FGAUGE_CURRENT_FACTOR           6.25 // uV/Rsns
#define FGAUGE_CURRENT_OFFSET_FACTOR    1.56 // uV/Rsns
#define FGAUGE_CAR_FACTOR               6.25 // uV/Rsns
#define FGAUGE_RSNS_FACTOR              0.02 // Ohm

//#define COMPASATE_OCV                   80 // mV for evb
#define COMPASATE_OCV                   40 // mV for phone

#define BATTERY_VOLTAGE_MINIMUM         3400
#define BATTERY_VOLTAGE_MAXIMUM         4200

#define BATTERY_CAPACITY_MAXIMUM        1668 //zhaoshaopeng from 2561 for v970 20120720

#define TEMPERATURE_T0                  110
#define TEMPERATURE_T1                  0
#define TEMPERATURE_T2                  25
#define TEMPERATURE_T3                  50
#define TEMPERATURE_T                   255 // This should be fixed, never change the value

//#define BATT_CAPACITY                   1280
#define BATT_CAPACITY                   1655 //zhaoshaopeng from 2561 for aq 20120720

#define ENABLE_SW_COULOMB_COUNTER       0 // 1 is enable, 0 is disable
//#define ENABLE_SW_COULOMB_COUNTER       1 // 1 is enable, 0 is disable

//#define FG_CURRENT_OFFSET_DISCHARGING 	31
#define FG_CURRENT_OFFSET_DISCHARGING 	0

#define FG_RESISTANCE 	20

#define FG_METER_RESISTANCE 	0
//#define FG_METER_RESISTANCE 	540 // current meter

//#define MAX_BOOTING_TIME_FGCURRENT	5*6 // 5 seconds, 6 points = 1s
#define MAX_BOOTING_TIME_FGCURRENT	1*10 // 10s

#if defined(CONFIG_POWER_EXT)
//#define OCV_BOARD_COMPESATE	32 //mV 
#define OCV_BOARD_COMPESATE	72 //mV 
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
#else
//#define OCV_BOARD_COMPESATE	0 //mV 
//#define OCV_BOARD_COMPESATE	48 //mV 
//#define OCV_BOARD_COMPESATE	25 //mV 
#define OCV_BOARD_COMPESATE	0 //mV 
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
//#define R_FG_BOARD_SLOPE	1057 //slope
//#define R_FG_BOARD_SLOPE	1075 //slope
#endif

#if 0//zhaoshaopeng for aq 20120720
#define Q_MAX_POS_50	2561
#define Q_MAX_POS_25	2561
#define Q_MAX_POS_0		2561
#define Q_MAX_NEG_10	2561

#define Q_MAX_POS_50_H_CURRENT	2534
#define Q_MAX_POS_25_H_CURRENT	2534
#define Q_MAX_POS_0_H_CURRENT	2534
#define Q_MAX_NEG_10_H_CURRENT	2534
#else
#define Q_MAX_POS_50	1668
#define Q_MAX_POS_25	1655
#define Q_MAX_POS_0		1520
#define Q_MAX_NEG_10	1087

#define Q_MAX_POS_50_H_CURRENT	1653
#define Q_MAX_POS_25_H_CURRENT	1640
#define Q_MAX_POS_0_H_CURRENT	1439
#define Q_MAX_NEG_10_H_CURRENT	961
#endif

#define R_FG_VALUE 				20 // mOhm, base is 20
#define CURRENT_DETECT_R_FG	100  //10mA

#define OSR_SELECT_7			0

#define CAR_TUNE_VALUE			100 //1.00

/////////////////////////////////////////////////////////////////////
// <DOD, Battery_Voltage> Table
/////////////////////////////////////////////////////////////////////
typedef struct _BATTERY_PROFILE_STRUC
{
    kal_int32 percentage;
    kal_int32 voltage;
} BATTERY_PROFILE_STRUC, *BATTERY_PROFILE_STRUC_P;

typedef enum
{
    T1_0C,
    T2_25C,
    T3_50C
} PROFILE_TEMPERATURE;

// T0 -10C
BATTERY_PROFILE_STRUC battery_profile_t0[] =
{
	{0   , 4173},
	{3   , 4151},
	{6   , 4126},
	{8   , 4081},
	{11  , 4041},
	{14  , 4014},
	{16  , 3991},
	{19 , 3969},
	{22 , 3950},
	{25 , 3932},
	{27 , 3916},
	{30 , 3902},
	{33 , 3891},
	{36 , 3879},
	{38 , 3869},
	{41 , 3860},
	{44 , 3851},
	{47 , 3843},
	{49 , 3834},
	{52 , 3827},
	{55 , 3820},
	{58 , 3811},
	{60 , 3803},
	{63 , 3795},
	{66 , 3785},
	{69 , 3773},
	{71 , 3761},
	{74 , 3747},  
	{77 , 3733}, 
	{80 , 3721},
	{82 , 3713},
	{85 , 3707},  
	{88 , 3701},
	{91 , 3695},
	{93 , 3670},
	{96 , 3600},
	{98 , 3533},
	{98 , 3507},
	{99 , 3495},
	{99 , 3488},
	{99 , 3483},
	{99 , 3481},
	{99 , 3479},
	{99 , 3477},
	{100, 3476},
	{100, 3475},
	{100, 3474},
	{100, 3472},
        {100, 3471},
        {100, 3471},
        {100, 3470},
        {100, 3469},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3468},
        {100, 3400}








};      
        
// T1 0C
BATTERY_PROFILE_STRUC battery_profile_t1[] =
{
	{0  , 4183},
	{2  , 4139},
	{4  , 4093},
	{6  , 4069},
	{8  , 4051},
	{10 , 4035},
	{12 , 4020},
	{14 , 4005},
	{16 , 3992},
	{18 , 3979},
	{20 , 3967},
	{22 , 3955},
	{24 , 3944},
	{26 , 3933},
	{28 , 3923},
	{29 , 3912},
	{31 , 3900},
	{33 , 3887},
	{35 , 3871},
	{37 , 3854},
	{39 , 3840},
	{41 , 3829},
	{43 , 3821},
	{45 , 3814},
	{47 , 3808},
	{49 , 3802},
	{51 , 3798},
	{53 , 3794},  
	{55 , 3790}, 
	{57 , 3787},
	{59 , 3783},
	{61 , 3782},  
	{63 , 3779},
	{65 , 3777},
	{67 , 3776},
	{69 , 3774},
	{71 , 3772},
	{73 , 3769},
	{75 , 3764},
	{77 , 3759},
	{79 , 3751},
	{81 , 3740},
	{83 , 3727},
	{84 , 3712},
	{86 , 3695},
	{88 , 3687},
	{90 , 3681},
	{92 , 3675},
        {94 , 3667},
        {96 , 3643},
        {98 , 3582},
        {98 , 3542},
        {99 , 3516},
        {99 , 3497},
        {99 , 3481},
        {99 , 3467},
        {100, 3456},
        {100, 3446},
        {100, 3438},
        {100, 3431},
        {100, 3424},
        {100, 3420},
        {100, 3416},
        {100, 3412},
        {100, 3408},
        {100, 3406},
        {100, 3403},
        {100, 3400},
        {100, 3400}
 
 
 
 
 
 
  

}; 

// T2 25C
BATTERY_PROFILE_STRUC battery_profile_t2[] =
{
	{0  , 4183},
	{2  , 4162},
	{4  , 4142},
	{5  , 4125},
	{7  , 4107},
	{9  , 4089},
	{11 , 4073},
	{13 , 4058},
	{14 , 4042},
	{16 , 4027},
	{18 , 4013},
	{20 , 3999},
	{22 , 3986},
	{23 , 3974},
	{25 , 3961},
	{27 , 3950},
	{29 , 3939},
	{31 , 3928},
	{32 , 3918},
	{34 , 3909},
	{36 , 3899},
	{38 , 3890},
	{40 , 3880},
	{41 , 3867},
	{43 , 3847},
	{45 , 3830},
	{47 , 3819},
	{49 , 3812},  
	{51 , 3805}, 
	{52 , 3800},
	{54 , 3794},
	{56 , 3791},  
	{58 , 3786},
	{60 , 3782},
	{61 , 3778},
	{63 , 3776},
	{65 , 3774},
	{67 , 3771},
	{69 , 3769},
	{70 , 3768},
	{72 , 3765},
	{74 , 3762},
	{76 , 3757},
	{78 , 3751},
	{79 , 3743},
	{81 , 3735},
	{83 , 3724},
	{85 , 3709},
        {87 , 3694},
        {88 , 3676},
        {90 , 3670},
        {92 , 3666},
        {94 , 3660},
        {96 , 3651},
        {97 , 3604},
        {99 , 3492},
        {101, 3323},
        {101, 3287},
        {101, 3274},
        {101, 3268},
        {101, 3265},
        {101, 3264},
        {101, 3260},
        {101, 3260},
        {101, 3260},
        {101, 3259},
        {101, 3257},
        {101, 3257},
        {101, 3257}

}; 

// T3 50C
BATTERY_PROFILE_STRUC battery_profile_t3[] =
{
	{0  , 4188},
	{2  , 4167},
	{4  , 4149},
	{5  , 4130},
	{7  , 4113},
	{9  , 4096},
	{11 , 4079},
	{13 , 4063},
	{14 , 4047},
	{16 , 4031},
	{18 , 4017},
	{20 , 4003},
	{21 , 3990},
	{23 , 3977},
	{25 , 3964},
	{27 , 3953},
	{29 , 3940},
	{30 , 3929},
	{32 , 3920},
	{34 , 3909},
	{36 , 3900},
	{38 , 3891},
	{39 , 3882},
	{41 , 3872},
	{43 , 3854},
	{45 , 3831},
	{47 , 3820},
	{48 , 3813},  
	{50 , 3806}, 
	{52 , 3800},
	{54 , 3794},
	{55 , 3789},  
	{57 , 3784},
	{59 , 3780},
	{61 , 3777},
	{63 , 3773},
	{64 , 3770},
	{66 , 3768},
	{68 , 3765},
	{70 , 3763},
	{72 , 3755},
	{73 , 3745},
	{75 , 3742},
	{77 , 3736},
	{79 , 3729},
	{81 , 3721},
	{82 , 3713},
	{84 , 3701},
        {86 , 3687},
        {88 , 3668},
        {89 , 3661},
        {91 , 3657},
        {93 , 3652},
        {95 , 3645},
        {97 , 3619},
        {98 , 3532},
        {100, 3382},
        {101, 3296},
        {101, 3280},
        {101, 3269},
        {101, 3261},
        {101, 3256},
        {101, 3253},
        {101, 3251},
        {101, 3250},
        {101, 3249},
        {101, 3249},
        {101, 3247},
        {101, 3247}

  





      
};              

// battery profile for actual temperature. The size should be the same as T1, T2 and T3
BATTERY_PROFILE_STRUC battery_profile_temperature[] =
{
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
	  {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
	  {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}








};    

/////////////////////////////////////////////////////////////////////
// <Rbat, Battery_Voltage> Table
/////////////////////////////////////////////////////////////////////
typedef struct _R_PROFILE_STRUC
{
    kal_int32 resistance; // Ohm
    kal_int32 voltage;
} R_PROFILE_STRUC, *R_PROFILE_STRUC_P;

// T0 -10C
R_PROFILE_STRUC r_profile_t0[] =
{
	{333    ,  4173},
	{333    ,  4151},
	{340    ,  4126},
	{350    ,  4081},
	{530    ,  4041},
	{645    ,  4014},
	{683    ,  3991},
	{683    ,  3969},
	{678    ,  3950},
	{673    ,  3932},
	{670    ,  3916},
	{665    ,  3902},
	{668    ,  3891},
	{670    ,  3879},
	{673    ,  3869},
	{678    ,  3860},
	{680    ,  3851},
	{685    ,  3843},
	{685    ,  3834},
	{695    ,  3827},
	{700    ,  3820},
	{703    ,  3811},
	{708    ,  3803},
	{710    ,  3795},
	{713    ,  3785},
	{710    ,  3773},
	{710    ,  3761},
	{708    ,  3747},  
	{703    ,  3733}, 
	{700    ,  3721},
	{705    ,  3713},
	{718    ,  3707},  
	{743    ,  3701},
	{783    ,  3695},
	{823    ,  3670},
	{848    ,  3600},
	{833    ,  3533},
	{770    ,  3507},
	{740    ,  3495},
	{725    ,  3488},
	{713    ,  3483},
	{705    ,  3481},
	{703    ,  3479},
	{693    ,  3477},
	{695    ,  3476},
	{693    ,  3475},
	{688    ,  3474},
	{683    ,  3472},
        {685    ,  3471},
        {678    ,  3471},
        {680    ,  3470},
        {678    ,  3469},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {675    ,  3468},
        {505    ,  3400}

};

// T1 0C
R_PROFILE_STRUC r_profile_t1[] =
{
	{178    ,  4183},
	{178    ,  4139},
	{283    ,  4093},
	{358    ,  4069},
	{373    ,  4051},
	{380    ,  4035},
	{390    ,  4020},
	{395    ,  4005},
	{400    ,  3992},
	{403    ,  3979},
	{410    ,  3967},
	{415    ,  3955},
	{418    ,  3944},
	{423    ,  3933},
	{428    ,  3923},
	{430    ,  3912},
	{428    ,  3900},
	{423    ,  3887},
	{408    ,  3871},
	{388    ,  3854},
	{378    ,  3840},
	{373    ,  3829},
	{370    ,  3821},
	{370    ,  3814},
	{370    ,  3808},
	{373    ,  3802},
	{375    ,  3798},
	{380    ,  3794},  
	{380    ,  3790}, 
	{385    ,  3787},
	{385    ,  3783},
	{393    ,  3782},  
	{393    ,  3779},
	{393    ,  3777},
	{398    ,  3776},
	{400    ,  3774},
	{400    ,  3772},
	{403    ,  3769},
	{405    ,  3764},
	{408    ,  3759},
	{410    ,  3751},
	{410    ,  3740},
	{410    ,  3727},
	{413    ,  3712},
	{415    ,  3695},
	{430    ,  3687},
	{458    ,  3681},
	{508    ,  3675},
        {615    ,  3667},
        {813    ,  3643},
        {958    ,  3582},
        {858    ,  3542},
        {790    ,  3516},
        {745    ,  3497},
        {710    ,  3481},
        {678    ,  3467},
        {650    ,  3456},
        {630    ,  3446},
        {605    ,  3438},
        {585    ,  3431},
        {578    ,  3424},
        {555    ,  3420},
        {543    ,  3416},
        {553    ,  3412},
        {533    ,  3408},
        {528    ,  3406},
        {528    ,  3403},
        {533    ,  3400},
        {528    ,  3400}
}; 

// T2 25C
R_PROFILE_STRUC r_profile_t2[] =
{
	{168    ,  4183},
	{168    ,  4162},
	{168    ,  4142},
	{175    ,  4125},
	{175    ,  4107},
	{175    ,  4089},
	{178    ,  4073},
	{183    ,  4058},
	{183    ,  4042},
	{185    ,  4027},
	{190    ,  4013},
	{190    ,  3999},
	{193    ,  3986},
	{198    ,  3974},
	{200    ,  3961},
	{203    ,  3950},
	{208    ,  3939},
	{213    ,  3928},
	{215    ,  3918},
	{220    ,  3909},
	{228    ,  3899},
	{233    ,  3890},
	{233    ,  3880},
	{225    ,  3867},
	{198    ,  3847},
	{178    ,  3830},
	{170    ,  3819},
	{170    ,  3812},  
	{170    ,  3805}, 
	{173    ,  3800},
	{168    ,  3794},
	{175    ,  3791},  
	{175    ,  3786},
	{173    ,  3782},
	{175    ,  3778},
	{178    ,  3776},
	{180    ,  3774},
	{178    ,  3771},
	{180    ,  3769},
	{183    ,  3768},
	{183    ,  3765},
	{180    ,  3762},
	{178    ,  3757},
	{178    ,  3751},
	{173    ,  3743},
	{173    ,  3735},
	{170    ,  3724},
	{173    ,  3709},
        {175    ,  3694},
        {173    ,  3676},
        {173    ,  3670},
        {180    ,  3666},
        {190    ,  3660},
        {215    ,  3651},
        {223    ,  3604},
        {253    ,  3492},
        {315    ,  3323},
        {223    ,  3287},
        {188    ,  3274},
        {170    ,  3268},
        {163    ,  3265},
        {160    ,  3264},
        {163    ,  3260},
        {158    ,  3260},
        {163    ,  3260},
        {160    ,  3259},
        {148    ,  3257},
        {160    ,  3257},
        {165    ,  3257}

};

// T3 50C
R_PROFILE_STRUC r_profile_t3[] =
{
	{128    ,  4188},
	{128    ,  4167},
	{130    ,  4149},
	{130    ,  4130},
	{130    ,  4113},
	{133    ,  4096},
	{133    ,  4079},
	{135    ,  4063},
	{135    ,  4047},
	{135    ,  4031},
	{140    ,  4017},
	{140    ,  4003},
	{143    ,  3990},
	{145    ,  3977},
	{145    ,  3964},
	{153    ,  3953},
	{148    ,  3940},
	{153    ,  3929},
	{160    ,  3920},
	{160    ,  3909},
	{168    ,  3900},
	{175    ,  3891},
	{183    ,  3882},
	{190    ,  3872},
	{170    ,  3854},
	{138    ,  3831},
	{133    ,  3820},
	{133    ,  3813},  
	{133    ,  3806}, 
	{133    ,  3800},
	{135    ,  3794},
	{135    ,  3789},  
	{135    ,  3784},
	{140    ,  3780},
	{143    ,  3777},
	{145    ,  3773},
	{148    ,  3770},
	{150    ,  3768},
	{150    ,  3765},
	{153    ,  3763},
	{143    ,  3755},
	{133    ,  3745},
	{140    ,  3742},
	{140    ,  3736},
	{138    ,  3729},
	{138    ,  3721},
	{138    ,  3713},
	{138    ,  3701},
        {138    ,  3687},
        {135    ,  3668},
        {138    ,  3661},
        {143    ,  3657},
        {150    ,  3652},
        {155    ,  3645},
        {163    ,  3619},
        {168    ,  3532},
        {225    ,  3382},
        {240    ,  3296},
        {203    ,  3280},
        {175    ,  3269},
        {158    ,  3261},
        {145    ,  3256},
        {135    ,  3253},
        {133    ,  3251},
        {128    ,  3250},
        {130    ,  3249},
        {128    ,  3249},
        {125    ,  3247},
        {123    ,  3247}
};

// r-table profile for actual temperature. The size should be the same as T1, T2 and T3
R_PROFILE_STRUC r_profile_temperature[] =
{
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
	  {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
	  {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},   
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}
    
};    


int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUC_P fgauge_get_profile(kal_uint32 temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUC_P fgauge_get_profile_r_table(kal_uint32 temperature);
