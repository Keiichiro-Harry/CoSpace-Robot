////////////////////////////////////////
//
//	File : ai.c
//	CoSpace Robot
//	Version 1.0.0
//	OCT 1 2021
//	Copyright (C) 2021 CoSpace Robot. All Rights Reserved
//
//////////////////////////////////////
//
// ONLY C Code can be compiled.
//
/////////////////////////////////////

#define CsBot_AI_H//DO NOT delete this line
#ifndef CSBOT_REAL
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#define DLL_EXPORT extern "C" __declspec(dllexport)
#define false 0
#define true 1
#endif
//The robot ID : six chars unique CID.
//Find it from your CoSpace Robot label or CoSpace program download GUI.
//Don't write the below line into two lines or more lines.
char AI_MyID[6] = {'1','2','3','4','5','6'};

int Duration = 0;
int SuperDuration = 0;
int bGameEnd = false;
int CurAction = -1;
int CurGame = 0;
int SuperObj_Num = 0;
int SuperObj_X = 0;
int SuperObj_Y = 0;
int Teleport = 0;
int LoadedObjects = 0;
int RedObjects = 0;
int CyanObjects = 0;
int BlackObjects = 0;
int US_Front = 0;
int US_Left = 0;
int US_Right = 0;
int CSLeft_R = 0;
int CSLeft_G = 0;
int CSLeft_B = 0;
int CSRight_R = 0;
int CSRight_G = 0;
int CSRight_B = 0;
int PositionX = 0;
int PositionY = 0;
int TM_State = 0;
int Compass = 0;
int Time = 0;
int WheelLeft = 0;
int WheelRight = 0;
int LED_1 = 0;
int MyState = 0;
int T2 = 0;
int AI_TeamID = 1;   //Robot Team ID. 1:Blue Ream; 2:Red Team.
int MySMS; //A integer value which you want to send to the other robot.
           //In Super Team mode, you can use this value to inform your another robot of your status.
           //In Indiviual Mode, you should keep this value to 0.  

//The following three variables save the information of the other robot.
int OtherRob_SMS; //Keep the recieved value of the other robot MySMS value.
            //In Super Team mode, this value is sent by your another robot.
            //In Indiviual Mode, this value has no meaning.
int OtherRob_PositionX;//The X coordinate of the other robot. 
int OtherRob_PositionY;//The Y coordinate of the other robot. 

//The following four variables keep the information of the last state-changed object.
int ObjState;    //The state (0: Disappear; 1:Appear.) of the last state changed object.
int ObjPositionX;//The X coordinate of the last state-changed object.
int ObjPositionY;//The Y coordinate of the last state-changed object.
int ObjDuration; //The duration(seconds) of the object maintains the current state;


#define CsBot_AI_C//DO NOT delete this line

DLL_EXPORT void SetGameID(int GameID)
{
    if(CurGame != GameID) LoadedObjects = 0;
    CurGame = GameID;
    bGameEnd = 0;
}


DLL_EXPORT void SetTeamID(int TeamID)
{
    AI_TeamID = TeamID;
}

DLL_EXPORT int GetGameID()
{
    return CurGame;
}

//Only Used by CsBot Dance Platform
DLL_EXPORT int IsGameEnd()
{
    return bGameEnd;
}

#ifndef CSBOT_REAL

char info[3000];
DLL_EXPORT char* GetDebugInfo()
{
    sprintf(info, "Duration=%d;SuperDuration=%d;bGameEnd=%d;CurAction=%d;CurGame=%d;SuperObj_Num=%d;SuperObj_X=%d;SuperObj_Y=%d;Teleport=%d;LoadedObjects=%d;RedObjects=%d;CyanObjects=%d;BlackObjects=%d;US_Front=%d;US_Left=%d;US_Right=%d;CSLeft_R=%d;CSLeft_G=%d;CSLeft_B=%d;CSRight_R=%d;CSRight_G=%d;CSRight_B=%d;PositionX=%d;PositionY=%d;TM_State=%d;Compass=%d;Time=%d;WheelLeft=%d;WheelRight=%d;LED_1=%d;MyState=%d;",Duration,SuperDuration,bGameEnd,CurAction,CurGame,SuperObj_Num,SuperObj_X,SuperObj_Y,Teleport,LoadedObjects,RedObjects,CyanObjects,BlackObjects,US_Front,US_Left,US_Right,CSLeft_R,CSLeft_G,CSLeft_B,CSRight_R,CSRight_G,CSRight_B,PositionX,PositionY,TM_State,Compass,Time,WheelLeft,WheelRight,LED_1,MyState);
    return info;
}
 
DLL_EXPORT char* GetTeamName()
{
     return "Sky Sea";
}

DLL_EXPORT int GetCurAction()
{
    return CurAction;
}

//Only Used by CsBot Rescue Platform
DLL_EXPORT int GetTeleport()
{
    return Teleport;
}

//Only Used by CsBot Rescue Platform
DLL_EXPORT void SetSuperObj(int X, int Y, int num)
{
    SuperObj_X = X;
    SuperObj_Y = Y;
    SuperObj_Num = num;
}
//Only Used by CsBot Rescue Platform
DLL_EXPORT void GetSuperObj(int *X, int *Y, int *num)
{
    *X = SuperObj_X;
    *Y = SuperObj_Y;
    *num = SuperObj_Num;
}
//Used by CoSpace Rescue Simulation. 
///Called each time frame by CoSpaceRobot Studio to update the other robot information.
DLL_EXPORT void UpdateRobInfo(int sms, int X, int Y)
{
	OtherRob_SMS = sms;
	OtherRob_PositionX = X;
	OtherRob_PositionY = Y;
}

//Used by CsBot Rescue Platform
DLL_EXPORT void UpdateObjectInfo(int X, int Y, int state, int duration)
{
	ObjState = state;
	ObjPositionX = X;
	ObjPositionY = Y;
	ObjDuration = duration;
}
DLL_EXPORT int GetMySMS()
{
	return MySMS;
}

#endif ////CSBOT_REAL

DLL_EXPORT void SetDataAI(volatile int* packet, volatile int *AI_IN)
{

    int sum = 0;

    US_Front = AI_IN[0]; packet[0] = US_Front; sum += US_Front;
    US_Left = AI_IN[1]; packet[1] = US_Left; sum += US_Left;
    US_Right = AI_IN[2]; packet[2] = US_Right; sum += US_Right;
    CSLeft_R = AI_IN[3]; packet[3] = CSLeft_R; sum += CSLeft_R;
    CSLeft_G = AI_IN[4]; packet[4] = CSLeft_G; sum += CSLeft_G;
    CSLeft_B = AI_IN[5]; packet[5] = CSLeft_B; sum += CSLeft_B;
    CSRight_R = AI_IN[6]; packet[6] = CSRight_R; sum += CSRight_R;
    CSRight_G = AI_IN[7]; packet[7] = CSRight_G; sum += CSRight_G;
    CSRight_B = AI_IN[8]; packet[8] = CSRight_B; sum += CSRight_B;
    PositionX = AI_IN[9]; packet[9] = PositionX; sum += PositionX;
    PositionY = AI_IN[10]; packet[10] = PositionY; sum += PositionY;
    TM_State = AI_IN[11]; packet[11] = TM_State; sum += TM_State;
    Compass = AI_IN[12]; packet[12] = Compass; sum += Compass;
    Time = AI_IN[13]; packet[13] = Time; sum += Time;
    packet[14] = sum;

}
DLL_EXPORT void GetCommand(int *AI_OUT)
{
    AI_OUT[0] = WheelLeft;
    AI_OUT[1] = WheelRight;
    AI_OUT[2] = LED_1;
    AI_OUT[3] = MyState;
}
void TurnTo(int curRot, int targetRot)
{
    int p0 = targetRot;
    int p3 = (targetRot + 3) % 360;
    int p15 = (targetRot + 15) % 360;
    int n3 = (targetRot - 3 + 360) % 360;
    int n15 = (targetRot - 15 + 360) % 360;
    int p180 = (targetRot + 180) % 360;
    int l = 0, r = 0;
    Duration = 6;
    //Within(-3,+3)deg, stop turing.
    l = n3; r = p3;
    if ((l < r && curRot > l && curRot < r) ||
    (l > r && (curRot > l || curRot < r)))
    {
        WheelLeft = 0;
        WheelRight = 0;
        Duration = 0;
        return;
    }
    //Within[3,15]deg,Turn Slowly
    l = p3; r = p15;
    if ((l < r && curRot >= l && curRot <= r) ||
        (l > r && (curRot >= l || curRot <= r)))
    {
        WheelLeft = 10;
        WheelRight = -10;
        return;
    }
    //Within[15,180]deg,Turn Faast
    l = p15; r = p180;
    if ((l < r && curRot >= l && curRot <= r) ||
       (l > r && (curRot >= l || curRot <= r)))
    {
        WheelLeft = 30;
        WheelRight = -30;
        return;
    }
    //Within[-15,-3]deg,Turn Slowly
    l = n15; r = n3;
    if ((l < r && curRot >= l && curRot <= r) ||
    (l > r && (curRot >= l || curRot <= r)))
    {
        WheelLeft = -10;
        WheelRight = 10;
        return;
    }
    //Within[-180,-15]deg,Turn Fast
    l = p180; r = n15;
    if ((l < r && curRot >= l && curRot <= r) ||
    (l > r && (curRot >= l || curRot <= r)))
    {
        WheelLeft = -30;
        WheelRight = 30;
        return;
    }
}

void motor(int left,int right){
    WheelLeft = left;
    WheelRight= right;
}

void setOnLED()
{
    LED_1 = 2;
}
void setOnOffLED()
{
    LED_1 = 1;
}
void setOffLED()
{
    LED_1 = 0;
}

//編集するのはここから

int SPON = 0;
int SPOX[10];
int SPOY[10];
int SPOF1 = 0;//こっちは一番新しく出たSuperObjectについて。
int SPOF2 = 0;//こっちは今追っているSuperObjectについて。二つ出現したときに、遠くても古い方を目指す恐れあり
// 0~359
void goToAngle(int angle)
{
    //行きたい方向と向いている方向の差を調べる
    angle = angle - Compass;
    //angleは行きたい方向。θ（シータ）を求めてる。サイン・コサイン・タンジェントのθ！
    //座標に点を取って、その点と原点を結んだ線をr（rは１）として、x座標をコサイン、y座標をサイン、
    //サイン/コサイン（傾き）をタンジェント、原点のところの角度がθであるのだ
    if (angle > 180)
    {
        angle = angle - 360;
    }
    //θが180より大きければ、反対周りで行きたい方向を向いたほうが速い
    else if (angle < -180)
    {
        angle = angle + 360;
    }
    int US = 0;//2進数を使ってる

    if (US_Front < 10)
    {
        motor(-30, -80);
    }
    if (US_Left < 10)
    {
        motor(-30, -80);
    }
    if (US_Right < 10)
    {
        motor(-80, -30);
    }
    if (US_Front < 40)
    {
        US += 1;
    }
    if (US_Left < 40)
    {
        US += 2;
    }
    if (US_Right < 40)
    {
        US += 4;
    }

    int big_motor = 80;
    int middle_motor = 50;
    int small_motor = 20;
    //ここは最適化の余地あり
    if (US == 1 && abs(angle) < 30)//前方に障害物があって30°以内の回転
    {
        if (angle < 0)//右に行きたいとき
        {
            motor(big_motor, small_motor);
        }
        else//左に行きたいとき
        {
            motor(small_motor, big_motor);
        }
    }
    else if (US == 2 && angle > 0)//左に障害物があるけど左に行きたい
    {
        if (angle < 120)//120°以下なら若干右に行って曲がれるとこを探す
        {
            motor(big_motor, middle_motor);
        }
        else//それ以上ならその場で向きを変える
        {
            motor(middle_motor, -middle_motor);
        }
    }
    else if (US == 3 && angle > -30)//前と左に障害物
    {
        if (angle < 120)//右へ
        {
            motor(big_motor, small_motor);
        }
        else
        {
            motor(middle_motor, -middle_motor);//正負逆に改造するべき？
        }
    }
    else if (US == 4 && angle < 0)//右に障害物があるけど右に行きたい
    {
        if (angle > -120)//左に行って右に曲がれる場所を探す
        {
            motor(middle_motor, big_motor);
        }
        else
        {
            motor(-middle_motor, middle_motor);
        }
    }
    else if (US == 5 && angle < 30)
    {
        if (angle < -120)
        {
            motor(middle_motor, -middle_motor);
        }
        else
        {
            motor(small_motor, big_motor);
        }
    }
    else if (US == 6 && abs(angle) > 30)
    {
        if (angle > 0)
        {
            if (angle < 120)
            {
                motor(middle_motor, middle_motor);
            }
            else
            {
                motor(-middle_motor, middle_motor);
            }
        }
        else
        {
            if (angle > -120)
            {
                motor(middle_motor, middle_motor);
            }
            else
            {
                motor(middle_motor, -middle_motor);
            }
        }
    }
    else if (US == 7 && abs(angle) < 120)
    {
        if (angle > 0)
        {
            motor(-middle_motor, middle_motor);
        }
        else
        {
            motor(middle_motor, -middle_motor);
        }
    }
    else
    {
        if(SPON==0||LoadedObjects==6||(SPON!=0&&pow((PositionX-SPOX[SPOF2]),2)+pow((PositionY-SPOY[SPOF2]),2)>100))
        {
            if (abs(angle) < 10)
            {
                motor(90, 90);
            }
            //行きたい方向±20（絶対値20）だったらゆっくり回る
            else if (abs(angle) < 25)
            {
                if (angle < 0)
                {
                    motor(90, 70);
                }
                else
                {
                    motor(70, 90);
                }
            }
            else if (abs(angle) < 40)
            {
                if (angle < 0)
                {
                    motor(90, 30);
                }
                else
                {
                    motor(30, 90);
                }
            }
            else
            {
                if (angle < 0)
                {
                    motor(90, 0);
                }
                else
                {
                    motor(0, 90);
                }
            }   
        }
        else
        {
            if (abs(angle) < 5)
            {
                motor(90, 90);
            }
            //行きたい方向±20（絶対値20）だったらゆっくり回る
            else if (abs(angle) < 15)
            {
                if (angle < 0)
                {
                    motor(10, -20);
                }
                else
                {
                    motor(-20, 10);
                }
            }
            else if (abs(angle) < 30)
            {
                if (angle < 0)
                {
                    motor(20, -40);
                }
                else
                {
                    motor(-40, 20);
                }
            }
            else
            {
                if (angle < 0)
                {
                    motor(40, -80);
                }
                else
                {
                    motor(-80, 40);
                }
            }   
        }
    }  
}

int goToPosition(int x, int y, int x_wide, int y_wide, int arrive_wide)
{
    // 本当の目標地点(x, y)の+-(x_wide, y_wide)の中にある
    // xとyの中からランダムに選んだ場所がターゲットx,y
    static int target_x = 0;
    static int target_y = 0;

    // target_xとtarget_yが、目標エリアの中にいない場合
    if (!(x - x_wide <= target_x && target_x <= x + x_wide &&
          y - y_wide <= target_y && target_y <= y + y_wide))
    {
        double randtheta=rand()%6283/1000;
        //target_x = x - x_wide + rand() % (x_wide * 2);
        //target_y = y - y_wide + rand() % (y_wide * 2);
        target_x=x+rand()%x_wide*cos(randtheta);
        target_y=y+rand()%y_wide*sin(randtheta);
    }

    x = target_x - PositionX;
    y = target_y - PositionY;
    float theta = atan2(y, x);//
    int angle = theta / 3.14 * 180 + 270;
    if (angle >= 360)
    {
        angle - 360;
    }
    goToAngle(angle);
    if (target_x - arrive_wide <= PositionX &&
        PositionX <= target_x + arrive_wide &&
        target_y - arrive_wide <= PositionY &&
        PositionY <= target_y + arrive_wide)
    {
        printf("arrived\n");
        return 1;
    }
    else
    {
        return 0;
    }
}

int processpoint[43][5] = {
    {130, 30, 10, 10, 10}, // 0
    {235, 60, 10, 10, 10},
    {120, 90, 10, 10, 10},
    {240, 130, 10, 10, 10},
    {340, 40, 10, 10, 10},
    {235, 60, 10, 10, 10}, // 5
    {340, 90, 10, 10, 10},
    {240, 115, 10, 10, 10},
    {340, 180, 10, 10, 10},
    {280, 185, 10, 10, 10},
    {195, 70, 10, 10, 10}, // 10
    {110, 130, 10, 10, 10},
    {190, 90, 10, 10, 10},
    {190, 170, 10, 10, 10},
    {110, 180, 10, 10, 10},
    {30, 180, 10, 10, 10}, // 15
    {90, 120, 10, 10, 10},
    {30, 80, 10, 10, 10},
    {180, 70, 10, 10, 10},
    {50, 25, 10, 10, 10},
    {130, 50, 10, 10, 10}, // 20
    {190, 20, 10, 5, 10},
    {260, 50, 10, 5, 10},
    {315, 25, 10, 5, 10},
    /*{240, 25, 10, 10, 10},
    {340, 25, 5, 10, 10},
    {290, 250,  10,  10,  10},//25
    {260, 235,  10,   5,  10},
    {200, 245,  10,  10,  10},
    {150, 210,  10,  10,  10},
    {100, 170,   3,  10,   3},
    {140, 150,   3,  10,   3},//30
    { 90, 110,   5,   5,   5},
    { 80, 160,  10,  10,  10},
    { 45, 160,  10,  10,  10},
    { 25, 210,  10,  10,  10},
    { 45, 245,  10,  10,  10},//35
    { 20, 140,   1,   1,   3},
    { 25,  80,   5,  10,   5},
    { 30,  40,  10,  10,  10},
    { 60,  30,  10,  10,  10},
    {150,  80,  10,  10,  10},//40
    {210, 120,  10,  10,  10},
    {210, 170,  10,  10,  10}*/
};


int NearestProcess(int currentX, int currentY){
    int min_distance=300000;
    int answer=0;
    for(int i=0;i<26;i++){
        if(pow(currentX-processpoint[i][0],2)+pow(currentY-processpoint[i][1],2)<min_distance){
            min_distance=pow(currentX-processpoint[i][0],2)+pow(currentY-processpoint[i][1],2);
            answer = i;
        }
    }
    return answer;
}

// int color_black[3] = {0, 0, 0};
// void isLeftBlack()
// {
//     if (isLeftBlack)
// }
// void isRightBlack()
// {
// }

/*
    0: 何もない場所
    1: 沼地
    2: 罠
    3: 壁
*/
enum
{
    MAP_WHITE = 0,     // 何もない場所
    MAP_SWAMPLAND = 1, // 沼地
    MAP_TRAP = 2,      // 罠
    MAP_WALL = 3,      // 壁
};

int map[36][27] = {

    //  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26
    {3, 3, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3},  // 0
    {3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3},  // 1
    {3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3},  // 2
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3},  // 3
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3},  // 4
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3},  // 5
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 6
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 7
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 8
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 9
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 10
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},  // 11
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1},  // 12
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},  // 13
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},  // 14
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},  // 15
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},  // 16
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},  // 17
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},  // 18
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},  // 19
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},  // 20
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},  // 21
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},  // 22
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1},  // 23
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},  // 24
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 25
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 26
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 27
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 28
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 29
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3},  // 30
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3},  // 31
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3},  // 32
    {3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3},  // 33
    {3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3},  // 34
    {3, 3, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3}}; // 35


/*
    0: 左下
    1: 左
    2: 左上
    3: 下
    4: なし
    5: 上
    6: 右下
    7: 右
    8: 右上
*/
int from[36][27];

int dijkstra(int goal_x, int goal_y, int rand_x, int rand_y, int tolerance)
{
    // 1.コストが一番小さいマス目で、印がついていないマス目を探す
    // 2.確定済みの印をつける
    // 3.隣接するマス目のコストを計算する
    // 4.隣接する先のマス目のコストと比較する
    // 5.コストが小さい方を採用する
    // 6.1-5を、マス目が見つからなくなるまで繰り返す

    // true or false
    int sign[36][27];
    int cost[36][27];
    // 初期化
    for (int i = 0; i < 36; i++)
    {
        for (int j = 0; j < 27; j++)
        {
            sign[i][j] = 0;
            cost[i][j] = 100000;
        }
    }
    // 現在地の位置を把握する
    int dijkstraX = PositionX / 10;
    int dijkstraY = PositionY / 10;

    // 現在地のコストを設定する
    cost[dijkstraX][dijkstraY] = 0;
    // すべてのマス目への最短の行き方を調べる
    while (1)
    {
        // min_costを大きくして(cost[x][y] < min_cost)に当てはまるようにする
        int min_cost = 100000;
        //x座標は360Y座標は270までだから、マス目はmin_xで36個まで, min_yで27個までにしている
        int min_x = -1, min_y;
        //for (最初の１度だけ実行する;○○である限り繰り返す;繰り返すたび○○する)
        for (int x = 0; x < 36; x++)
        {
            for (int y = 0; y < 27; y++)
            {
                //既に印がついていたら一番近いfor (yのとこ）に戻る
                if (sign[x][y] == 1)
                {
                    continue;
                }
                //既に表記されたコストより新しく計算したコストのほうが低ければ、
                //コストと座標を(min_cost,min_x,min_y)保存する
                if (cost[x][y] < min_cost)
                {
                    min_cost = cost[x][y];
                    min_x = x;
                    min_y = y;
                }
            }
        }
        // 一度も、コストの上書き処理がされなかった場合
        // つまり、すべて探し終わった時
        if (min_x == -1)
        {
            printf("out!\n");
            break;
        }

        // その座標に印をつける
        sign[min_x][min_y] = 1;
        // 隣接するマス目のコストを計算し、もし新しいコストのほうが小さければ上書きする
        for (int x = min_x - 1; x <= min_x + 1; x++)
        {
            if (x == -1 || x == 36)
            {
                continue;
            }
            for (int y = min_y - 1; y <= min_y + 1; y++)
            {
                if (y == -1 || y == 27)
                {
                    continue;
                }
                // その場所の隣接するマス目のコストを計算し、もし新しいコストのほうが小さければ上書きする
                // コストの計算式：普通は３コスト、沼は1000コスト、壁800コスト、トラップ2000
                // (min_x, min_y)のコスト:cost[min_x][min_y]
                // 隣接するマス目の座標: (x, y)
                // 隣接するマス目の座標のコスト:cost[x][y]

                // 隣接するマス目のコストを計算して、変数に保存する
                int new_cost;
                if (map[x][y] == MAP_SWAMPLAND)
                {
                    new_cost = cost[min_x][min_y] + 1000;
                }
                else if (map[x][y] == MAP_TRAP)
                {
                    new_cost = cost[min_x][min_y] + 1000;
                }
                else if (map[x][y] == MAP_WALL)
                {
                    new_cost = cost[min_x][min_y] + 2000;
                }
                else
                {
                    new_cost = cost[min_x][min_y] + 3;
                }
                // 元々保存されているコストと比較して、小さい方を保存する
                if (new_cost < cost[x][y])
                {
                    cost[x][y] = new_cost;
                    from[x][y] = (x - (min_x - 1)) * 3 + y - (min_y - 1);
                }
            }
        }
    }

    // ゴールからさかのぼって計算する
    goal_x /= 10;
    goal_y /= 10;

    int go_to_x = goal_x, go_to_y = goal_y;
    int next_x = goal_x, next_y = goal_y;
    int now_x = goal_x, now_y = goal_y;

    while (next_x != dijkstraX || next_y != dijkstraY)
    {
        go_to_x = now_x;
        go_to_y = now_y;
        now_x = next_x;
        now_y = next_y;

        printf("%d %d\n", now_x, now_y);

        next_x = now_x + (8 - from[now_x][now_y]) % 3 - 1;//ここ変えた
        next_y = now_y + (8 - from[now_x][now_y]) % 3 - 1;
    }
    printf("robot position %d %d\n", PositionX, PositionY);
    printf("go to %d %d\n", go_to_x * 10, go_to_y * 10);
    rand_x=5, rand_y=5, tolerance=5;
    // 次の場所に移動する by gotoposition
    goToPosition(go_to_x * 10, go_to_y * 10, rand_x, rand_y, tolerance);//ここ変えた
    if (goal_x * 10 - tolerance <= PositionX && PositionX <= goal_x * 10 + tolerance &&
        goal_y * 10 - tolerance <= PositionY && PositionY <= goal_y * 10 + tolerance)
    {
        return true;
    }
    else
    {
        return false;
    }
    return true;
}

//本題はここから
int log_x, log_y;
int process = 0;
int ObjectBLACK = 2;
int ObjectRED = 2;
int ObjectBLUE = 2;
int ObjectSuper=0;
//ObjectSuperは持ってる数、それ以外は収集していい数

void Game0()
{
    if(Duration>0)
    {
        Duration--;
    }
    else if(CSLeft_R>=250 && CSLeft_G>=190 && CSLeft_G<=210 && CSLeft_B<=10 && CSRight_R>=250 && CSRight_G>=190 && CSRight_G<=210 && CSRight_B<=10 && Time>=180&&(LoadedObjects>0))
    {
        Duration = 71;
        CurAction =1;
    }
    else if(CSLeft_R>=250 && CSLeft_G>=190 && CSLeft_G<=210 && CSLeft_B<=10 && CSRight_R>=250 && CSRight_G>=190 && CSRight_G<=210 && CSRight_B<=10 && Time<=180&&(((BlackObjects>0&&CyanObjects>0&&RedObjects>0)/*||LoadedObjects>=5*/)||Time- T2 > 20))
    {
        Duration = 71;
        CurAction =2;
    }
    else if(CSRight_R>=250 && CSRight_G>=190 && CSRight_G<=210 && CSRight_B<=10&&(((BlackObjects>0&&CyanObjects>0&&RedObjects>0)/*||LoadedObjects>=5*/)||Time- T2 > 20))
    {
        Duration = 0;
        CurAction =3;
    }
    else if(CSLeft_R>=250 && CSLeft_G>=190 && CSLeft_G<=210 && CSLeft_B<=10&&(((BlackObjects>0&&CyanObjects>0&&RedObjects>0)/*||LoadedObjects>=5*/)||Time- T2 > 20))
    {
        Duration = 4;
        CurAction =4;
    }
    else if(CSRight_R>=250 && CSRight_G>=250 && CSRight_B<=10&&(LoadedObjects>0))
    {
        Duration = 4;
        CurAction =5;
    }
    else if(CSLeft_R>=250 && CSLeft_G>=250 && CSLeft_B<=10&&(LoadedObjects>0))
    {
        Duration = 0;
        CurAction =6;
    }
    else if(CSRight_R<=10 && CSRight_G<=10 && CSRight_B<=10&&(LoadedObjects<6&&BlackObjects<2)/*&&Time <= 60*/)//これはマップによって変える
    {
        Duration = 65;
        CurAction =7;
    }
    /*else if(CSRight_R<=10 && CSRight_G<=10 && CSRight_B<=10&&(LoadedObjects<5&&BlackObjects<3)&&Time >= 60)
    {
        Duration = 65;
        CurAction =7;
    }
    else if(CSRight_R<=10 && CSRight_G<=10 && CSRight_B<=10&&(LoadedObjects==5&&RedObjects!=0&&CyanObjects!=0)&&Time >= 60)
    {
        Duration = 65;
        CurAction =7;
    }*/
    else if(CSLeft_R<=10 && CSLeft_G<=10 && CSLeft_B<=10&&(LoadedObjects<6&&BlackObjects<2)/*&&Time <= 60*/)
    {
        Duration = 65;
        CurAction =8;
    }
    /*else if(CSLeft_R<=10 && CSLeft_G<=10 && CSLeft_B<=10&&(LoadedObjects<5&&BlackObjects<3)&&Time >= 60)
    {
        Duration = 65;
        CurAction =8;
    }
    else if(CSLeft_R<=10 && CSLeft_G<=10 && CSLeft_B<=10&&(LoadedObjects==5&&RedObjects!=0&&CyanObjects!=0)&&Time >= 60)
    {
        Duration = 65;
        CurAction =8;
    }*/
    else if(CSRight_R<=10 && CSRight_G>=250 && CSRight_B>=250&&(LoadedObjects<6&&CyanObjects<2)/*&&Time <= 60*/)
    {
        Duration = 65;
        CurAction =9;
    }
    /*else if(CSRight_R<=10 && CSRight_G>=250 && CSRight_B>=250&&(LoadedObjects<5&&CyanObjects<3)&&Time >= 60)
    {
        Duration = 65;
        CurAction =9;
    }
    else if(CSRight_R<=10 && CSRight_G>=250 && CSRight_B>=250&&(LoadedObjects==5&&RedObjects!=0&&BlackObjects!=0)&&Time >= 60)
    {
        Duration = 65;
        CurAction =9;
    }*/
    else if(CSLeft_R<=10 && CSLeft_G>=250 && CSLeft_B>=250&&(LoadedObjects<6&&CyanObjects<2)/*&&Time <= 60*/)
    {
        Duration = 65;
        CurAction =10;
    }
    /*else if(CSLeft_R<=10 && CSLeft_G>=250 && CSLeft_B>=250&&(LoadedObjects<5&&CyanObjects<3)&&Time >= 60)
    {
        Duration = 65;
        CurAction =10;
    }
    else if(CSLeft_R<=10 && CSLeft_G>=250 && CSLeft_B>=250&&(LoadedObjects==5&&RedObjects!=0&&BlackObjects!=0)&&Time >= 60)
    {
        Duration = 65;
        CurAction =10;
    }*/
    else if(CSRight_R>=250 && CSRight_G<=10 && CSRight_B<=10&&(LoadedObjects<6&&RedObjects<2)/*&&Time <= 60*/)
    {
        Duration = 65;
        CurAction =11;
    }
    /*else if(CSRight_R>=250 && CSRight_G<=10 && CSRight_B<=10&&(LoadedObjects<5&&RedObjects<3)&&Time >= 60)
    {
        Duration = 65;
        CurAction =11;
    }
    else if(CSRight_R>=250 && CSRight_G<=10 && CSRight_B<=10&&(LoadedObjects==5&&CyanObjects!=0&&BlackObjects!=0)&&Time >= 60)
    {
        Duration = 65;
        CurAction =11;
    }*/
    else if(CSLeft_R>=250 && CSLeft_G<=10 && CSLeft_B<=10&&(LoadedObjects<6&&RedObjects<2)/*&&Time <= 60*/)
    {
        Duration = 65;
        CurAction =12;
    }
    /*else if(CSLeft_R>=250 && CSLeft_G<=10 && CSLeft_B<=10&&(LoadedObjects<5&&RedObjects<3)&&Time >= 60)
    {
        Duration = 65;
        CurAction =12;
    }
    else if(CSLeft_R>=250 && CSLeft_G<=10 && CSLeft_B<=10&&(LoadedObjects==5&&CyanObjects!=0&&BlackObjects!=0)&&Time >= 60)
    {
        Duration = 65;
        CurAction =12;
    }*/
    else if(US_Front>=20 && US_Left<=10 && US_Right<=10)
    {
        Duration = 2;
        CurAction =23;
    }
    else if(US_Front<=8 && US_Left<=8 && US_Right<=8)
    {
        Duration = 2;
        CurAction =13;
    }
    else if((CSLeft_R>=250 && CSLeft_G>=190 && CSLeft_G<=210 && CSLeft_B<=10 && CSRight_R>=250 && CSRight_G>=190 && CSRight_G<=210 && CSRight_B<=10)&&(US_Front<=30 && US_Left<=30 && US_Right<=30)&&(LoadedObjects==0||(BlackObjects==0||CyanObjects==0||RedObjects==0))&&(Compass>350||Compass<10)){
        Duration = 10;
        CurAction = 30;
    }
    else if(US_Front<=10 && US_Left>=50 && US_Right>=50)
    {
        Duration = 2;
        CurAction =14;
    }
    else if(US_Front>=230 && US_Left>=230 && US_Right>=230)
    {
        Duration = 1;
        CurAction =15;
    }
    else if(US_Front<=10)
    {
        Duration = 0;
        CurAction =16;
        if(US_Left>US_Right
)
        {
            if((CSLeft_R<=10 && CSLeft_G>=180 && CSLeft_G<=190 && CSLeft_B>=250) || (CSRight_R<=10 && CSRight_G>=180 && CSRight_G<=190 && CSRight_B>=250)){
                Duration = 0;
                CurAction =17;
            }else if(((BlackObjects>0&&CyanObjects>0&&RedObjects>0)/*||LoadedObjects>=5*/)||Time- T2 > 20){
                Duration = 0;
                CurAction = 28;
            }else{
                Duration = 0;
                CurAction =17;
            }
        }
        else if(US_Left<US_Right
)
        {
            if((CSLeft_R<=10 && CSLeft_G>=180 && CSLeft_G<=190 && CSLeft_B>=250) || (CSRight_R<=10 && CSRight_G>=180 && CSRight_G<=190 && CSRight_B>=250)){
                Duration = 0;
                CurAction =18;
            }else if(((BlackObjects>0&&CyanObjects>0&&RedObjects>0)/*||LoadedObjects>=5*/)||Time- T2 > 20){
                Duration = 0;
                CurAction = 29;
            }else{
                Duration = 0;
                CurAction =18;
            }
        }
    }
    else if(US_Right<=8)
    {
        if(((BlackObjects>0&&CyanObjects>0&&RedObjects>0)/*||LoadedObjects>=5*/)||Time- T2 > 20){
            Duration = 0;
            CurAction = 28;
        }else{
            Duration = 0;
            CurAction =19;
        }
    }
    else if(US_Left<=8)
    {
        if(((BlackObjects>0&&CyanObjects>0&&RedObjects>0)/*||LoadedObjects>=5*/)||Time- T2 > 20){
            Duration = 0;
            CurAction = 29;
        }else{
            Duration = 0;
            CurAction =20;
        }
    }
    else if(US_Left < 40/*これを大きくすると真ん中を徘徊???、小さくすると壁沿い*/&&(((BlackObjects>0&&CyanObjects>0&&RedObjects>0)/*||LoadedObjects>=5*/)||Time - T2 > 20)){
        if(US_Left < 25){
            Duration = 0;
            CurAction = 23;
        }else if(US_Left < 15){
            Duration = 0;
            CurAction = 24;
        }else{
            Duration = 0;
            CurAction = 25;
        }
    }
    else if(US_Right < 40/*これを大きくすると真ん中を徘徊???、小さくすると壁沿い*/&&(((BlackObjects>0&&CyanObjects>0&&RedObjects>0)/*||LoadedObjects>=5*/)||Time - T2 > 20)){
        if(US_Right < 25){
            Duration = 0;
            CurAction = 23;
        }else if(US_Right < 15){
            Duration = 0;
            CurAction = 25;
        }else{
            Duration = 0;
            CurAction = 24;
        }
    }
    /*else if((BlackObjects==0||CyanObjects==0||RedObjects==0)&&US_Left>100&&US_Front>80){
        Duration = 0;
        CurAction = 27;
    }
    else if((BlackObjects==0||CyanObjects==0||RedObjects==0)&&US_Right>100&&US_Front>80){
        Duration = 0;
        CurAction = 26;
    }*/
    /*else if(Time>=180 && Time<=210&&(LoadedObjects==0))
    {
        Duration = 59;
        CurAction =22;
    }
    else if(Time>=210)
    {
        Duration = 59;
        CurAction =21;
    }*/
    else if(true)
    {
        /*if(Time%2 == 0){
            Duration = 0;
            CurAction = 24;
        }else{
            Duration = 0;
            CurAction = 25;
        }*/
        Duration = 0;
        CurAction = 23;
    }
    switch(CurAction)
    {
        case 1:
            motor(0,0);
            LED_1=2;
            MyState=0;
            if(Duration < 12)
            {
                motor(-100,-100);
            }
            LoadedObjects=0;     
            BlackObjects=0;      
            CyanObjects=0;       
            RedObjects=0;
            T2 = Time;  
            break;
        case 2:
            motor(0,0);
            LED_1=2;
            MyState=0;
            if(Duration < 12)
            {
                motor(-75,-75);
            }
            LoadedObjects=0;
            BlackObjects=0;
            CyanObjects=0;        
            RedObjects=0;
            T2 = Time;
                    
            break;
        case 3:
            motor(80,0);
            LED_1=0;
            MyState=0;
            break;
        case 4:
            motor(0,80);
            LED_1=0;
            MyState=0;
            break;
        case 5:
            motor(-80,-20);
            LED_1=0;
            MyState=0;
            break;
        case 6:
            motor(-20,-80);
            LED_1=0;
            MyState=0;
            break;
        case 7:
            motor(0,0);
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                motor(40,40);
            }
            if(Duration==1){
 BlackObjects++;
 T2 = Time;
                    
}
            break;
        case 8:
            motor(0,0);
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                motor(40,40);
            }
            if(Duration==1){
 BlackObjects++;
 T2 = Time;
                    
}
            break;
        case 9:
            motor(0,0);
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                motor(40,40);
            }
            if(Duration==1){
 CyanObjects++;
 T2 = Time;
                    
}
            break;
        case 10:
            motor(0,0);
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                motor(40,40);
            }
            if(Duration==1){
 CyanObjects++;
 T2 = Time;
                    
}

            break;
        case 11:
            motor(0,0);
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                motor(40,40);
            }
            if(Duration==1){
 RedObjects++;
 T2 = Time;
                    
}
            break;
        case 12:
            motor(0,0);
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                motor(40,40);
            }
            if(Duration==1){
 RedObjects++;
 T2 = Time;
                    
}

            break;
        case 13:
            motor(-20,-75);
            LED_1=0;
            MyState=0;
            break;
        case 14:
            motor(-75,-20);
            LED_1=0;
            MyState=0;
            break;
        case 15:
            motor(-80,-20);
            LED_1=0;
            MyState=0;
            break;
        case 16:
            motor(-100,-80);
            LED_1=0;
            MyState=0;
            break;
        case 17:
            motor(-80,70);
            LED_1=0;
            MyState=0;
            break;
        case 18:
            motor(70,-80);
            LED_1=0;
            MyState=0;
            break;
        case 19:
            motor(-40,40);
            LED_1=0;
            MyState=0;
            break;
        case 20:
            motor(40,-40);
            LED_1=0;
            MyState=0;
            break;
        case 21:
            motor(0,0);
            LED_1=0;
            MyState=0;
             Teleport =1;  
            LoadedObjects = 0;
            motor(0,0);  LED_1=0;
            Duration = 0;   SuperDuration = 0;
            break;
        case 22:
            motor(0,0);
            LED_1=0;
            MyState=0;
             Teleport =1;  
            LoadedObjects = 0;
            motor(0,0);  LED_1=0;
            Duration = 0;   SuperDuration = 0;
            break;
        case 23:
            motor(75,75);
            LED_1=0;
            MyState=0;
            break;
        case 24://右
            motor(75,70);
            LED_1=0;
            MyState=0;
            break;
        case 25://左
            motor(70,75);
            LED_1=0;
            MyState=0;
            break;
        case 26://右
            motor(80,40);
            LED_1=0;
            MyState=0;
            break;
        case 27://左
            motor(40,80);
            LED_1=0;
            MyState=0;
            break;
        case 28://左
            motor(-20,20);
            LED_1=0;
            MyState=0;
            break;
        case 29://右
            motor(20,-20);
            LED_1=0;
            MyState=0;
            break;
        case 30:
            motor(60,-60);
            LED_1=0;
            MyState=0;
            break;
        default:
            break;
    }

}

void Game1()
{
    printf("%d %d %d %d\n ", ObjectBLACK, ObjectBLUE, ObjectRED,ObjectSuper, LoadedObjects);
    if (SuperObj_Num >= 1)
    {
        SPOX[SPOF1] = SuperObj_X;
        SPOY[SPOF1] = SuperObj_Y;
        SPON = SuperObj_Num;
        SuperObj_Num = 0;
        ++SPOF1;
    }
    if (PositionX == 0 && PositionY == 0)
    {
        PositionX = log_x;
        PositionY = log_y;
    }
    log_x = PositionX;
    log_y = PositionY;

    if (Duration > 0)
    {
        Duration--;
    }
    else if (CSLeft_R <= 10 && CSLeft_G <= 10 && CSLeft_B <= 10 && (ObjectBLACK > 0 && LoadedObjects < 6))//左黒
    {
        Duration = 65;
        CurAction = 1;
    }
    else if (CSRight_R <= 10 && CSRight_G <= 10 && CSRight_B <= 10 && (ObjectBLACK > 0 && LoadedObjects < 6))//右黒
    {
        Duration = 65;
        CurAction = 2;
    }
    else if (CSLeft_R <= 10 && CSLeft_G >= 245 && CSLeft_B >= 245 && (ObjectBLUE > 0 && LoadedObjects < 6))//左シアン
    {
        Duration = 65;
        CurAction = 3;
    }
    else if (CSRight_R <= 10 && CSRight_G >= 245 && CSRight_B >= 245 && (ObjectBLUE > 0 && LoadedObjects < 6))//右シアン
    {
        Duration = 65;
        CurAction = 4;
    }
    else if (CSLeft_R >= 245 && CSLeft_G <= 10 && CSLeft_B <= 10 && (ObjectRED > 0 && LoadedObjects < 6))//左赤
    {
        Duration = 65;
        CurAction = 5;
    }
    else if (CSRight_R >= 245 && CSRight_G <= 10 && CSRight_B <= 10 && (ObjectRED > 0 && LoadedObjects < 6))//右赤
    {
        Duration = 65;
        CurAction = 6;
    }
    else if (CSLeft_R >= 245 && CSLeft_G >= 193 && CSLeft_G <= 213 && CSLeft_B <= 10 &&
             CSRight_R >= 245 && CSRight_G >= 190 && CSRight_G <= 213 && CSRight_B <= 10 && (LoadedObjects >= 6 || Time >= 450))//デポジット
    {
        Duration = 75;
        CurAction = 7;
    }
    /*else if (CSLeft_R >= 245 && CSLeft_G >= 245 && CSLeft_B <= 10 &&
             CSRight_R >= 245 && CSRight_G >= 245 && CSRight_B <= 10 && (LoadedObjects != 0))//トラップ
    {
        Duration = 10;
        CurAction = 8;
    }
    else if (CSLeft_R >= 245 && CSLeft_G >= 245 && CSLeft_B <= 10 && (LoadedObjects != 0))
    {
        Duration = 10;
        CurAction = 10;
    }
    else if (CSRight_R >= 245 && CSRight_G >= 245 && CSRight_B <= 10 && (LoadedObjects != 0))
    {
        Duration = 10;
        CurAction = 9;
    }
    else if (CSLeft_R >= 245 && CSLeft_G >= 150 && CSLeft_G <= 170 && CSLeft_B <= 10 && (LoadedObjects >= 5))
    {
        Duration = 0;
        CurAction = 11;
    }
    else if (CSRight_R >= 245 && CSRight_G >= 150 && CSRight_G <= 170 && CSRight_B <= 10 && (LoadedObjects >= 5))
    {
        Duration = 0;
        CurAction = 12;
    }*/
    else if (CSLeft_R >= 245 && CSLeft_G <= 10 && CSLeft_B >= 235 && LoadedObjects<6/*うっかりSuperObjectを取ってしまうのは許す&& (ObjectBLACK == 0 || ObjectRED == 0 || ObjectBLUE == 0)*/)
    {
        Duration = 65;
        CurAction = 13;
    }
    else if (CSRight_R >= 245 && CSRight_G <= 10 && CSRight_B >= 235 && LoadedObjects<6 /*&& (ObjectBLACK == 0 || ObjectRED == 0 || ObjectBLUE == 0) && Time<460*/)
    {
        Duration = 65;
        CurAction = 14;
    }
    else if (LoadedObjects >= 6)
    {
        CurAction = 0;
        if (PositionX >= 180)//ここ変える
        {
            goToPosition(290, 245, 1, 1, 10);//ここ変える
        }
        else
        {
            goToPosition(70, 245, 1, 1, 10);//ここ変える
        }
    }
    else if (SPON >= 1&&(LoadedObjects<=3||ObjectSuper!=0))//まだ3つしかとっていないときまたは既に一つ以上SuperObjectを持っているとき
    {
        if (PositionX - SPOX[SPOF2] <= 20 && PositionX - SPOX[SPOF2] >= -20 && PositionY - SPOY[SPOF2] <= 20 && PositionY - SPOY[SPOF2] >= -20)
        {
            goToPosition(SPOX[SPOF2], SPOY[SPOF2], 3, 3, 5);
            CurAction = 0;
        }
        else
        {
            CurAction = 0;
            goToPosition(SPOX[SPOF2], SPOY[SPOF2], 10, 10, 10);
        }
    }
    else if (US_Front < 5 || US_Left < 5)
    {
        CurAction = 15;
        Duration = 5;
        printf("kabeForL");
    }
    else if (US_Right < 3)
    {
        CurAction = 16;
        Duration = 5;
        printf("kabeR\n");
    }
    else if (US_Front < 10 || US_Left < 10)
    {
        CurAction = 17;
        Duration = 5;
        printf("kabeFL\n");
    }
    else if (US_Right < 5)
    {
        CurAction = 18;
        Duration = 5;
        printf("kabeRda\n");
    }
    else if (true)
    {
        CurAction = 0;
        printf("process %d\n", process);
        if (process == 0)
        {
            if (goToPosition(processpoint[0][0], processpoint[0][1], processpoint[0][2], processpoint[0][3], processpoint[0][4]))
            {
                ++process;
            }
        }
        else if (process == 1)
        {
            if (goToPosition(processpoint[1][0], processpoint[1][1], processpoint[1][2], processpoint[1][3], processpoint[1][4]))
            {
                ++process;
            }
        }
        else if (process == 2)
        {
            if (goToPosition(processpoint[2][0], processpoint[2][1], processpoint[2][2], processpoint[2][3], processpoint[2][4]))
            {
                ++process;
            }
        }

        else if (process == 3)
        {
            if (goToPosition(processpoint[3][0], processpoint[3][1], processpoint[3][2], processpoint[3][3], processpoint[3][4]))
            {
                ++process;
            }
        }
        else if (process == 4)
        {
            if (ObjectBLACK>1)
            {
                process=11;
            }
            if (goToPosition(processpoint[4][0], processpoint[4][1], processpoint[4][2], processpoint[4][3], processpoint[4][4]))
            {
                ++process;
            }
        }
        else if (process == 5)
        {
            if (ObjectBLACK>1)
            {
                process=4;
            }
            if (goToPosition(processpoint[5][0], processpoint[5][1], processpoint[5][2], processpoint[5][3], processpoint[5][4]))
            {
                ++process;
            }
        }
        else if (process == 6)
        {
            /*if (ObjectBLACK>1)
            {
                process=35;
            }*/
            if (goToPosition(processpoint[6][0], processpoint[6][1], processpoint[6][2], processpoint[6][3], processpoint[6][4]))
            {
                ++process;
            }
        }
        else if (process == 7)
        {
            /*if (ObjectBLACK>1)
            {
                process=26;
            }*/
            if (goToPosition(processpoint[7][0], processpoint[7][1], processpoint[7][2], processpoint[7][3], processpoint[7][4]))
            {
                ++process;
            }
        }
        else if (process == 8)
        {
            if (goToPosition(processpoint[8][0], processpoint[8][1], processpoint[8][2], processpoint[8][3], processpoint[8][4]))
            {
                ++process;
            }
        }
        else if (process == 9)
        {
            if (goToPosition(processpoint[9][0], processpoint[9][1], processpoint[9][2], processpoint[9][3], processpoint[9][4]))
            {
                ++process;
            }
        }
        else if (process == 10)
        {
            if (goToPosition(processpoint[10][0], processpoint[10][1], processpoint[10][2], processpoint[10][3], processpoint[10][4]))
            {
                ++process;
            }
        }
        else if (process == 11)
        {
            if (goToPosition(processpoint[11][0], processpoint[11][1], processpoint[11][2], processpoint[11][3], processpoint[11][4]))
            {
                ++process;
            }
        }
        else if (process == 12)
        {
            if (goToPosition(processpoint[12][0], processpoint[12][1], processpoint[12][2], processpoint[12][3], processpoint[12][4]))
            {
                ++process;
            }
        }
        else if (process == 13)
        {
            if (goToPosition(processpoint[13][0], processpoint[13][1], processpoint[13][2], processpoint[13][3], processpoint[13][4]))
            {
                ++process;
            }
        }
        else if (process == 14)
        {
            if (goToPosition(processpoint[14][0], processpoint[14][1], processpoint[14][2], processpoint[14][3], processpoint[14][4]))
            {
                ++process;
            }
        }
        else if (process == 15)
        {
            if (ObjectBLACK>1)
            {
                process=19;
            }
            if (goToPosition(processpoint[15][0], processpoint[15][1], processpoint[15][2], processpoint[15][3], processpoint[15][4]))
            {
                ++process;
            }
        }
        else if (process == 16)
        {
            if (ObjectBLACK>1)
            {
                process=19;
            }
            if (goToPosition(processpoint[16][0], processpoint[16][1], processpoint[16][2], processpoint[16][3], processpoint[16][4]))
            {
                ++process;
            }
        }
        else if (process == 17)
        {
            /*if (ObjectBLACK>1)
            {
                process=31;
            }*/
            if (goToPosition(processpoint[17][0], processpoint[17][1], processpoint[17][2], processpoint[17][3], processpoint[17][4]))
            {
                ++process;
            }
        }
        else if (process == 18)
        {
            if (goToPosition(processpoint[18][0], processpoint[18][1], processpoint[18][2], processpoint[18][3], processpoint[18][4]))
            {
                ++process;
            }
        }
        else if (process == 19)
        {
            if (goToPosition(processpoint[19][0], processpoint[19][1], processpoint[19][2], processpoint[19][3], processpoint[19][4]))
            {
                ++process;
            }
        }
        else if (process == 20)
        {
            if (goToPosition(processpoint[20][0], processpoint[20][1], processpoint[20][2], processpoint[20][3], processpoint[20][4]))
            {
                ++process;
            }
        }
        else if (process == 21)
        {
            if (goToPosition(processpoint[21][0], processpoint[21][1], processpoint[21][2], processpoint[21][3], processpoint[21][4]))
            {
                ++process;
            }
        }
        else if (process == 22)
        {
            if (goToPosition(processpoint[22][0], processpoint[22][1], processpoint[22][2], processpoint[22][3], processpoint[22][4]))
            {
                ++process;
            }
        }
        else if (process == 23)
        {
            if (goToPosition(processpoint[23][0], processpoint[23][1], processpoint[23][2], processpoint[23][3], processpoint[23][4]))
            {
                ++process;
            }
        }
        /*else if (process == 24)
        {
            if (goToPosition(processpoint[24][0], processpoint[24][1], processpoint[24][2], processpoint[24][3], processpoint[24][4]))
            {
                ++process;
            }
        }
        else if (process == 25)
        {
            if (goToPosition(processpoint[25][0], processpoint[25][1], processpoint[25][2], processpoint[25][3], processpoint[25][4]))
            {
                ++process;
            }
        }
        else if (process == 26)
        {
            if (goToPosition(processpoint[26][0], processpoint[26][1], processpoint[26][2], processpoint[26][3], processpoint[26][4]))
            {
                ++process;
            }
        }
        else if (process == 27)
        {
            if (goToPosition(processpoint[27][0], processpoint[27][1], processpoint[27][2], processpoint[27][3], processpoint[27][4]))
            {
                ++process;
            }
        }
        else if (process == 28)
        {
            if (ObjectBLACK>1)
            {
                process=42;
            }
            if (goToPosition(processpoint[28][0], processpoint[28][1], processpoint[28][2], processpoint[28][3], processpoint[28][4]))
            {
                ++process;
            }
        }
        else if (process == 29)
        {
            if (ObjectBLACK>1)
            {
                process=32;
            }
            if (goToPosition(processpoint[29][0], processpoint[29][1], processpoint[29][2], processpoint[29][3], processpoint[29][4]))
            {
                ++process;
            }
        }
        else if (process == 30)
        {
            if (ObjectBLACK>1)
            {
                process=18;
            }
            if (goToPosition(processpoint[30][0], processpoint[30][1], processpoint[30][2], processpoint[30][3], processpoint[30][4]))
            {
                ++process;
            }
        }
        else if (process == 31)
        {
            if (goToPosition(processpoint[31][0], processpoint[31][1], processpoint[31][2], processpoint[31][3], processpoint[31][4]))
            {
                ++process;
            }
        }
        else if (process == 32)
        {
            if (goToPosition(processpoint[32][0], processpoint[32][1], processpoint[32][2], processpoint[32][3], processpoint[32][4]))
            {
                ++process;
            }
        }
        else if (process == 33)
        {
            if (goToPosition(processpoint[33][0], processpoint[33][1], processpoint[33][2], processpoint[33][3], processpoint[33][4]))
            {
                ++process;
            }
        }
        else if (process == 34)
        {
            if (goToPosition(processpoint[34][0], processpoint[34][1], processpoint[34][2], processpoint[34][3], processpoint[34][4]))
            {
                ++process;
            }
        }
        else if (process == 35)
        {
            if (goToPosition(processpoint[35][0], processpoint[35][1], processpoint[35][2], processpoint[35][3], processpoint[35][4]))
            {
                ++process;
            }
        }
        else if (process == 36)
        {
            if (goToPosition(processpoint[36][0], processpoint[36][1], processpoint[36][2], processpoint[36][3], processpoint[36][4]))
            {
                ++process;
            }
        }
        else if (process == 37)
        {
            if (goToPosition(processpoint[37][0], processpoint[37][1], processpoint[37][2], processpoint[37][3], processpoint[37][4]))
            {
                ++process;
            }
        }
        else if (process == 38)
        {
            if (goToPosition(processpoint[38][0], processpoint[38][1], processpoint[38][2], processpoint[38][3], processpoint[38][4]))
            {
                ++process;
            }
        }
        else if (process == 39)
        {
            if (ObjectBLACK>1)
            {
                process=17;
            }
            if (goToPosition(processpoint[39][0], processpoint[39][1], processpoint[39][2], processpoint[39][3], processpoint[39][4]))
            {
                ++process;
            }
        }
        else if (process == 40)
        {
            if (goToPosition(processpoint[40][0], processpoint[40][1], processpoint[40][2], processpoint[40][3], processpoint[40][4]))
            {
                ++process;
            }
        }
        else if (process == 41)
        {
            if (goToPosition(processpoint[41][0], processpoint[41][1], processpoint[41][2], processpoint[41][3], processpoint[41][4]))
            {
                ++process;
            }
        }
        else if (process == 42)
        {
            if (goToPosition(processpoint[42][0], processpoint[42][1], processpoint[42][2], processpoint[42][3], processpoint[42][4]))
            {
                ++process;
            }
        }*/
        else
        {
            process = 0;
        }
    }
    switch(CurAction)
    {
    case 0:
        break;
    case 1:
        WheelLeft = 0;
        WheelRight = 0;
        LED_1 = 1;
        MyState = 0;
        if (Duration == 1)
        {
            LoadedObjects++;
            ObjectBLACK--;
        }
        if (Duration < 6)
        {
            WheelLeft = 40;
            WheelRight = 40;
        }
        break;
    case 2:
        WheelLeft = 0;
        WheelRight = 0;
        LED_1 = 1;
        MyState = 0;
        if (Duration == 1)
        {
            LoadedObjects++;
            ObjectBLACK--;
        }
        if (Duration < 6)
        {
            WheelLeft = 40;
            WheelRight = 40;
        }
        break;
    case 3:
        WheelLeft = 0;
        WheelRight = 0;
        LED_1 = 1;
        MyState = 0;
        if (Duration == 1)
        {
            LoadedObjects++;
            ObjectBLUE--;
        }
        if (Duration < 6)
        {
            WheelLeft = 40;
            WheelRight = 40;
        }
        break;
    case 4:
        WheelLeft = 0;
        WheelRight = 0;
        LED_1 = 1;
        MyState = 0;
        if (Duration == 1)
        {
            LoadedObjects++;
            ObjectBLUE--;
        }
        if (Duration < 6)
        {
            WheelLeft = 40;
            WheelRight = 40;
        }
        break;
    case 5:
        WheelLeft = 0;
        WheelRight = 0;
        LED_1 = 1;
        MyState = 0;
        if (Duration == 1)
        {
            LoadedObjects++;
            ObjectRED--;
        }
        if (Duration < 6)
        {
            WheelLeft = 40;
            WheelRight = 40;
        }
        break;
    case 6:
        WheelLeft = 0;
        WheelRight = 0;
        LED_1 = 1;
        MyState = 0;
        if (Duration == 1)
        {
            LoadedObjects++;
            ObjectRED--;
        }
        if (Duration < 6)
        {
            WheelLeft = 40;
            WheelRight = 40;
        }
        break;
    case 7:
        WheelLeft = 0;
        WheelRight = 0;
        LED_1 = 2;
        MyState = 0;
        if (Duration == 1)
        {
            LoadedObjects = 0;
            ObjectBLACK = 2;
            ObjectBLUE = 2;
            ObjectRED = 2;
            ObjectSuper = 0;
            process = NearestProcess(PositionX, PositionY);
            ;
        }
        if (Duration < 16)
        {
            WheelLeft = -50;
            WheelRight = -50;
        }
        if (Duration < 8)
        {
            WheelLeft = -50;
            WheelRight = +50;
        }
        break;
    case 8:
        WheelLeft = -40;
        WheelRight = -60;
        LED_1 = 0;
        MyState = 0;
        break;
    case 9:
        WheelLeft = -40;
        WheelRight = -60;
        LED_1 = 0;
        MyState = 0;
        break;
    case 10:
        WheelLeft = -60;
        WheelRight = -40;
        LED_1 = 0;
        MyState = 0;
        break;
    case 11:
        WheelLeft = 0;
        WheelRight = 70;
        LED_1 = 0;
        MyState = 0;
        break;
    case 12:
        WheelLeft = 70;
        WheelRight = 0;
        LED_1 = 0;
        MyState = 0;
        break;
    case 13:
        WheelLeft = 0;
        WheelRight = 0;
        LED_1 = 1;
        MyState = 0;
        if (Duration == 1)
            LoadedObjects++;
            ObjectSuper++;
            SPON = 0;
            ++SPOF2;
            ObjectBLACK = 6;
            ObjectBLUE = 6;
            ObjectRED = 6;
            process = NearestProcess(PositionX, PositionY);
        if (Duration < 6)
        {
            WheelLeft = 40;
            WheelRight = 40;
        }
        break;
    case 14:
        WheelLeft = 0;
        WheelRight = 0;
        LED_1 = 1;
        MyState = 0;
        if (Duration == 1)
            LoadedObjects++;
            ObjectSuper++;
            SPON = 0;
            ++SPOF2;
            ObjectBLACK = 6;
            ObjectBLUE = 6;
            ObjectRED = 6;
            process = NearestProcess(PositionX, PositionY);
        if (Duration < 6)
        {
            WheelLeft = 40;
            WheelRight = 40;
        }
        break;
    case 15:
        WheelLeft = -20;
        WheelRight = -30;
        LED_1 = 0;
        MyState = 0;
        break;
    case 16:
        WheelLeft = -30;
        WheelRight = -20;
        LED_1 = 0;
        MyState = 0;
        break;
    case 17:
        WheelLeft = 80;
        WheelRight = 20;
        LED_1 = 0;
        MyState = 0;
        break;
    case 18:
        WheelLeft = 20;
        WheelRight = 80;
        LED_1 = 0;
        MyState = 0;
        break;
    }
}


DLL_EXPORT void OnTimer()
{
    switch (CurGame)
    {
        case 9:
            break;
        case 10:
            WheelLeft=0;
            WheelRight=0;
            LED_1=0;
            MyState=0;
            break;
        case 0:
            Game0();
            break;
        case 1:
            Game1();
            break;
        default:
            break;
    }
}

