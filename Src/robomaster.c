#include "robomaster.h"
RoboMaster robomaster[4];
PID_Struct Robomaster_Speed_PID[4];    // = {1.4,0.9,0.6,0,0,5000,0,0.005};
PID_Struct Robomaster_Position_PID[4]; // = {0.13,0,0.082,0,0,5000,0,0.005};
int robomaster_flag = 0;
int ver_slide_error = 0;

static int cnt = 0;
void can_robomaster_rcv(can_msg *msg)
{
    //pRxMsg->StdId
    //这里id还没改
    int id = rx_id - 0x201;
    
    if (id > 4 || id < 0)
        return;
    static int first_flag[4] = {1, 1, 1, 1};
    if (first_flag[id] == 1)
    {
        robomaster[id].angle = (uint16_t)(msg->ui8[0] << 8 | msg->ui8[1]);
        robomaster[id].offset_angle = robomaster[id].angle;
        first_flag[id] = 0;
        robomaster[id].round_cnt = 0;
        return;
    }
    robomaster[id].last_angle = robomaster[id].angle;
    robomaster[id].angle = (uint16_t)(msg->ui8[0] << 8 | msg->ui8[1]);
    robomaster[id].speed_rpm = (int16_t)(msg->ui8[2] << 8 | msg->ui8[3]);
    robomaster[id].real_current = (msg->ui8[4] << 8 | msg->ui8[5]) * 5.f / 16384.f;

    if (robomaster[id].angle - robomaster[id].last_angle > 4096)
        robomaster[id].round_cnt--;
    else if (robomaster[id].angle - robomaster[id].last_angle < -4096)
        robomaster[id].round_cnt++;
    robomaster[id].total_angle = robomaster[id].round_cnt * 8192 + robomaster[id].angle - robomaster[id].offset_angle;
    cnt++;
    
    if(cnt%10 == 0){
        //uprintf("%d\r\n",robomaster[id].speed_rpm);
        send_wave((float)robomaster[id].total_angle,(float)robomaster[id].target_position,0,0);
        //send_wave((float)robomaster[id].speed_rpm,0,0,0);
    }
}

void RoboconMaster_Control()
{
    float speed_out[4];
    for (int i = 0; i < 4; i++)
    {
        speed_out[i] = robomaster_pid_control(i);
    }
    robomaster_set_current((int16_t)speed_out[0], (int16_t)speed_out[1], (int16_t)speed_out[2], (int16_t)speed_out[3]);
}

float robomaster_pid_control(int id)
{
    float speed_out = 0;
    float position_out = 0;
    position_out = PID_Release(&Robomaster_Position_PID[id],
                               (float)robomaster[id].target_position + ver_slide_error, (float)robomaster[id].total_angle);
    if (robomaster[id].type == _M2006)
    {
        //if(position_out > 19000) position_out = 19000;
        //if(position_out < -19000) position_out = -19000;
        if (position_out > 4000)
            position_out = 4000;
        if (position_out < -10000)
            position_out = -10000;
    }
    else if (robomaster[id].type == _M3508)
    {
        if (position_out > 9100)
            position_out = 9100;
        if (position_out < -9100)
            position_out = -9100;
        //if(position_out > 2000) position_out = 2000;
        //if(position_out < -2000) position_out = -2000;
    }
    else
    {
        position_out = 0;
    }
    //speed_out=PID_Release(&Robomaster_Speed_PID[id],position_out,(float)robomaster[id].speed_rpm);
    speed_out = PID_Release(&Robomaster_Speed_PID[id], (float)robomaster[id].target_speed, (float)robomaster[id].speed_rpm);
    return position_out;
}

void robomaster_set_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    can_msg Data;
    Data.ui8[0] = (iq1 >> 8);
    Data.ui8[1] = iq1;
    Data.ui8[2] = (iq2 >> 8);
    Data.ui8[3] = iq2;
    Data.ui8[4] = iq3 >> 8;
    Data.ui8[5] = iq3;
    Data.ui8[6] = iq4 >> 8;
    Data.ui8[7] = iq4;
    // for(int i = 0; i < 8; i++)
    //     uprintf("%d ",(int)Data.ui8[i]);
    // uprintf("\r\n");
    can_send_msg(0x200, &Data);
}

void M2006_init(int id)
{
    reset_PID(&Robomaster_Speed_PID[id]);
    reset_PID(&Robomaster_Position_PID[id]);
    Robomaster_Speed_PID[id].KP = 1.4;
    Robomaster_Speed_PID[id].KI = 0.9;
    Robomaster_Speed_PID[id].KD = 0.6;
    Robomaster_Speed_PID[id].i_max = 5000;
    Robomaster_Speed_PID[id].I_TIME = 0.005;

    Robomaster_Position_PID[id].KP = 0.13;
    Robomaster_Position_PID[id].KI = 0;
    Robomaster_Position_PID[id].KD = 0.082;
    Robomaster_Position_PID[id].i_max = 5000;
    Robomaster_Position_PID[id].I_TIME = 0.005;
}

void M3508_init(int id)
{
    reset_PID(&Robomaster_Speed_PID[id]);
    reset_PID(&Robomaster_Position_PID[id]);
    Robomaster_Speed_PID[id].KP = 1.6;
    Robomaster_Speed_PID[id].KI = 0.52;
    Robomaster_Speed_PID[id].KD = 0.6;
    Robomaster_Speed_PID[id].i_max = 5000;
    Robomaster_Speed_PID[id].I_TIME = 0.005;

    Robomaster_Position_PID[id].KP = 1.5;
    Robomaster_Position_PID[id].KI = 0;
    Robomaster_Position_PID[id].KD = 10;
    Robomaster_Position_PID[id].i_max = 5000;
    Robomaster_Position_PID[id].I_TIME = 0.005;
}

can_msg Data;
float Helm_robomaster_pid_control()
{
    float speed_out = 0;
    float position_out = 0;
    position_out = PID_Release(&Robomaster_Position_PID[0], (float)robomaster[0].target_position, (float)robomaster[0].total_angle);
    if (position_out > 8000)
        position_out = 8000;
    if (position_out < -8000)
        position_out = -8000;
    speed_out = PID_Release(&Robomaster_Speed_PID[0], position_out, (float)robomaster[0].speed_rpm);
    return speed_out;
}
void Helm_RoboconMaster_Control()
{
    int16_t speed = (int16_t)Helm_robomaster_pid_control();
    Data.ui8[0] = (speed >> 8);
    Data.ui8[1] = speed;
    can_send_msg(0x200, &Data);
}
