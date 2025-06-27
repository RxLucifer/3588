/****************************************************************************/
/*  ��Ŀ���ƣ����ʿ�ݳ��Զ���ʻ����ϵͳ                                        */
/*  �ļ����ƣ�control_HY_0528.cpp                                            */
/*  ����ʱ�䣺2025-05-13                                                     */
/*  ����޸ģ�2025-05-28                                                     */
/*  ��ǰ�汾��v2.0.0-beta                                                    */
/*  ������Ա��������                                                          */
/*  ��Ŀ�����������ձ���ͳ�����ϵͳ��д�����ʿ�ݳ��Զ���ʻ���Ƴ���              */
/****************************************************************************/

/****************************************************************************/
/*                              �汾��ʷ��¼                                 */
/****************************************************************************/

/*  �汾��v1.0.0-alpha                                                      */
/*  ���ڣ�2025-05-13                                                        */
/*  �޸��ˣ�������                                                           */
/*  ������ͣ�[�¹���] ��ʼ�汾����                                           */
/*  ��Ҫ�����                                                               */
/*    1. [�ع�] ɾ��createCanFrame��һ���ĺ�����ʵ��createEightCanFrames      */
/*       8����ģʽ������ͨ��Э���׼                                          */
/*    2. [�Ż�] ��д��ʱ��������ͳһ��������Ϊ20ms                            */
/*    3. [����] ʵ�ֿ��Ʋ��ԣ�����˫״̬��ͻ�жϻ���                           */
/*    4. [����] �������-����-��ʻ-ͣ���������̲���                           */
/*    5. [��֤] ��ͣ���ܲ���ͨ��                                              */
/*    6. [����] �Ż�����ṹ������CAN���Ľ����ͷ���ģ��                       */
/****************************************************************************/

/*  �汾��v1.1.0-alpha                                                      */
/*  ���ڣ�2025-05-22                                                        */
/*  �޸��ˣ����������ʤ                                                    */
/*  ������ͣ�[�¹���] GPS����ϵͳ����                                        */
/*  ��Ҫ�����                                                               */
/*    1. [����] ����INS821 GPS����ϵͳ��ʵ�ָ߾��ȶ�λ                       */
/*    2. [�ع�] ����readSerial������֧��ʵʱGPS���ݶ�ȡ                      */
/*    3. [�Ż�] �Ľ�parseData����������GPS���ݽ�������                       */
/*    4. [�޸�] ���ѭ���㷨Ԥ��㶨λ��������                                */
/*    5. [�Ľ�] ���GPS������������ƣ����������ȶ���                         */
/*    6. [����] �ʤ��ӳ������Ľ����������ֱ����4A2��441��411��431��      */
/*       471��473�Լ�451���ģ����Ƴ���״̬��ع���                          */
/****************************************************************************/

/*  �汾��v1.2.0-beta                                                       */
/*  ���ڣ�2025-05-23                                                        */
/*  �޸��ˣ�������                                                           */
/*  ������ͣ�[�ܹ��ع�] ���߳�CANͨ�żܹ�                                    */
/*  ��Ҫ�����                                                               */
/*    1. [����] �ں�ѭ���㷨����ɹ켣���ٹ��ܲ���                            */
/*    2. [�ع�] ʵ�ֶ��߳�CANͨ�żܹ���                                       */
/*       - ���̣߳���������߼���ָ������                                     */
/*       - CAN�����̣߳�ר�Ŵ���CAN���ķ��ͣ�20ms����                        */
/*       - ȫ�ֱ�־λflag_canSend���Ʒ���ʱ��                                */
/*    3. [�Ż�] ����vector<can_frame>�����ά����洢                        */
/*    4. [�Ľ�] ʵ������CAN���ķ��ͻ���                                       */
/*    5. [��ǿ] �̼߳�����ͬ���ͳ�ͻ�������                                  */
/****************************************************************************/

/*  �汾��v2.0.0-beta                                                       */
/*  ���ڣ�2025-05-28                                                        */
/*  �޸��ˣ�������                                                           */
/*  ������ͣ�[�ش���] RDCԶ�̼�ʻ����ϵͳ                                  */
/*  ��Ҫ�����                                                               */
/*    1. [����] RDC_IN���ݽṹ��֧��Զ�̼�ʻ����ָ�����                      */
/*       - ����/ɲ��/ת���/��λ���Ʋ���                                     */
/*       - �����ƶ���ʱ�����Ϣ                                              */
/*    2. [ʵ��] 8002�˿�UDP��������ר����RDC���ݰ�����                       */
/*       - ʵʱ���ݽ����ʹ洢                                                */
/*       - ControlState=2Զ�̿���ģʽ�л�                                   */
/*    3. [����] send_candata����RDC�汾��                                    */
/*       - �򻯿������̣��Ƴ����ӽ�ѹ/��������                               */
/*       - 30֡��ֱ�ӽ���RDC����ģʽ                                         */
/*       - ֱ��ӳ��RDC��������������                                         */
/*       - ������ȫ���ͼ�ͣ����                                            */
/*    4. [�ܹ�] ˫ģʽ����ϵͳ��                                              */
/*       - �˿�8000/8001�����߿���ģʽ(ControlState=1)                      */
/*       - �˿�8002��RDCԶ�̿���ģʽ(ControlState=2)                        */
/*    5. [�Ż�] RDC�����߼�����                                            */
/*       - �Ƴ����ӵ�λ���ƺ;�̬����                                        */
/*       - �����ſ��϶ȿ����㷨                                            */
/*       - ֱ��ʹ��RDC_steering_angleת�����                               */
/****************************************************************************/

/****************************************************************************/
/*                            �汾����淶˵��                               */
/****************************************************************************/
/*                                                                          */
/*  �汾�Ź淶��                                                             */
/*    ��ʽ��vX.Y.Z-[alpha|beta|rc|stable]                                   */
/*    X�����汾�ţ��ش�ܹ������                                             */
/*    Y���ΰ汾�ţ��¹�����ӣ�                                               */
/*    Z���޶��汾�ţ�bug�޸���С�Ż���                                        */
/*    ��׺�������׶α�ʶ                                                      */
/*                                                                          */
/*  ������ͱ�ǩ��                                                           */
/*    [�¹���] - ����µĹ���ģ��                                            */
/*    [�ع�] - ����ṹ������֯                                              */
/*    [�Ż�] - ���ܻ��߼��Ż�                                                */
/*    [�޸�] - bug�޸�                                                       */
/*    [����] - �������                                                      */
/*    [��֤] - ������֤                                                      */
/*    [����] - ģ�鼯��                                                      */
/*    [�ܹ��ع�] - �ش�ܹ����                                              */
/*    [�ش���] - ��Ҫ�������                                              */
/*                                                                          */
/*  ��¼��ʽҪ��                                                           */
/*    1. ÿ���汾������¼����ʱ�䵹������                                     */
/*    2. �������Ҫ������ȷ������ģ������                                     */
/*    3. ʹ��ͳһ�������Ͷ����ʽ                                             */
/*    4. ��Ҫ���Ҫ��ϸ˵������ϸ��                                           */
/*    5. ���ּ�¼�������Ժ�������                                             */
/*                                                                          */
/****************************************************************************/

#include <string>
#include <cstring>
#include <cstdlib>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
// �Ƴ��ظ���pthread.hͷ�ļ�����,��Ϊ�ں����Ѿ���������
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <cstdio>
#include <sys/ioctl.h>
#include <sys/socket.h> 
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <arpa/inet.h>  // for inet_ntop
#include <ctype.h>      // for toupper
#include <netinet/in.h> // for sockaddr_in, htonl, htons, ntohs, INADDR_ANY
#include <cstdint>
#include <map>
#include <GeographicLib/Geodesic.hpp>
#include <algorithm> // ȡ��ע����֧��std::clamp����
#include <cmath>
#include <iostream>
#include <chrono> // ��Ӷ�std::chrono��֧��
#include <vector>
#include <pthread.h>
#include <atomic>
#include <iomanip> // ������������ľ���
#include <fstream>
#include <sstream>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <ctime>
#include <string>
#include <chrono>
#include <utility>
#include <thread>
#include <mutex>
#include <cstring>
#include <sys/select.h>

//=======================
using namespace std;

// ============================================================================
// ��������
// ============================================================================
#define _USE_MATH_DEFINES
#define M_PI 3.14159265358979323846
#define NUM_THREADS 5
#define BUFSIZE 800
#define SERV_PORT 8000
#define SERIAL_PORT "/dev/ttyS7"     // �����豸·��
#define BAUD_RATE B115200            // ����������Ϊ 115200

// ============================================================================
// ȫ�ֱ������� - CANͨ�����
// ============================================================================
unsigned char rev_data[8];           // CAN�������ݻ�����
unsigned char send_data[8];          // CAN�������ݻ�����
std::vector<can_frame> eightFrames;  // CAN֡����
std::mutex framesMutex;              // CAN֡������
std::atomic<bool> flag_canSend{false}; // CAN���ͱ�־
int can_sockfd = -1;                 // CAN�׽����ļ�������
int cout_can;                        // CAN��ʼ��������
                                     // ���ܣ�0-��Ҫ��ʼ����-1-����������-���ƻ�ɲ��֡

// ============================================================================
// ȫ�ֱ������� - ϵͳ�������
// ============================================================================
bool running = true;                 // �����߳����б�־
bool enable_gps = false;              // GPS���ܿ��أ�true-����GPS��false-ʹ��ģ������
int ControlState = 0;                // ����״̬��0-δ�л���1-���߿��ƣ�2-Զ�̿���
double _target_ind;                  // Ŀ������
int rev_ok, rev_save_ok, rev_first, begin_run; // ����״̬��־
unsigned long rev_count, frame_count; // ������

// ============================================================================
// ȫ�ֱ������� - ��־���
// ============================================================================
std::ofstream logFile;               // ��־�ļ���
int recordCounter = 0;               // ��¼������
FILE *fp;                           // �ļ�ָ��

// ============================================================================
// �ṹ�嶨�� - �������
// ============================================================================

/**
 * @brief ��������ṹ��
 * @details ����ģ�����������ģ���״̬��Ϣ
 */
typedef struct controlOutput
{
    int controlOut_gear;             // ��λ״̬
    double vehicle_speed;            // ��ǰ���� (km/h)
    float Soc_LOW;                   // SOC��ѹ���״̬ (%)
    float Soc_HIGH;                  // SOC��ѹ���״̬ (%)
    int Emgy_brk_En;                 // ��ɲʹ��״̬��0-���ã�1-����
    int controlOut_ErrorCode;        // ������
} CONTROL_OUT;

/**
 * @brief ��������ṹ��
 * @details ����ģ������Ŀ���ָ�����
 */
typedef struct decisionInput
{
    // ��������
    int bStart;                      // ������ǩ��0-��������1-����
    
    // �ƶ����� (CAN ID: 0x210)
    float brake_bar;                 // ɲ��ѹ�� (bar)
    double breaking_dis;             // ɲ������ (m)
    
    // פ������ (CAN ID: 0x220)
    int EPB_park;                    // פ��ʹ�ܣ�0-�Ƴ���1-פ��
    
    // �������� (CAN ID: 0x240)
    int gear;                        // ��λ��1-P����3-R����5-N����9-D��
    double endSpeed;                 // ĩ�ٶ� (km/h)
    int controlMode;                 // ����ģʽ��1-�ٶ�ģʽ��2-Ť��ģʽ��3-��������ģʽ
    double PedposReq;                // ��������ٷֱ� (%)
    
    // �������� (CAN ID: 0x251)
    int Emgy_brk_En;                 // ��ɲʹ�ܣ�0-�Ƴ���1-��ɲ
    int Emgy_brk_ReqRmv;             // ��ɲ�Ƴ���0-�Ƴ���1-��ɲ�Ƴ�
    int Emgy_FtCrashRemove;          // ǰ�����Ƴ���0-���Ƴ���1-�Ƴ�
    int Emgy_RrrCrashRemove;         // �󴥱��Ƴ���0-���Ƴ���1-�Ƴ�
    int Emgy_LeftCrashRemove;        // �󴥱��Ƴ���0-���Ƴ���1-�Ƴ�
    int Emgy_RightCrashRemove;       // �Ҵ����Ƴ���0-���Ƴ���1-�Ƴ�
    
    // �������� (CAN ID: 0x260)
    int ADU_Hom;                     // ���ȿ���
    int ADU_BackLamp;                // �����ƿ���
    int ADU_TurnRLamp;               // ��ת�ƿ���
    int ADU_TurnLLamp;               // ��ת�ƿ���
    int ADU_DblFlashLamp;            // ˫���ƿ���
    int ADU_LowBeamLamp;             // ����ƿ���
    int ADU_WidthLamp;               // ʾ��ƿ���
    int ADU_HighBeamLamp;            // Զ��ƿ���
    int ADU_FogLamp;                 // ��ƿ���
    int ADU_BrkLamp;                 // �ƶ��ƿ���
} DECISION_IN;

/**
 * @brief Զ�̼�ʻ����ָ��ṹ��
 * @details RDCԶ�̼�ʻ����������Ϣ
 */
typedef struct RemoteDriveCommand
{
    // ʱ���
    long long RDC_timestamp;         // ָ��ʱ�����Unix��������ȷ��ʵʱ��
    
    // �ƶ����� (CAN ID: 0x210)
    double RDC_brake;                // ɲ�����ȣ�0-100��Χ (%)
    
    // פ������ (CAN ID: 0x220)
    int RDC_Park;                    // פ��ʹ�ܣ�0-�Ƴ���1-פ��
    
    // ת����� (CAN ID: 0x230)
    double RDC_steering_angle;       // Ŀ�귽����ת�� (��)
    
    // �������� (CAN ID: 0x240)
    double RDC_throttle;             // ���ſ��ƣ�0-100��Χ (%)
    
    // �������� (CAN ID: 0x251)
    int RDC_gear;                    // ��λ��1-P����3-R����5-N����9-D��
    int RDC_Emgy_brk_En;             // ��ɲʹ�ܣ�0-�Ƴ���1-��ɲ
    int RDC_Emgy_brk_ReqRmv;         // ��ɲ�Ƴ���0-�Ƴ���1-��ɲ�Ƴ�
    
    // �������� (CAN ID: 0x260)
    int RDC_ADU_Hom;                 // ���ȿ���
    int RDC_ADU_BackLamp;            // �����ƿ���
    int RDC_ADU_TurnRLamp;           // ��ת�ƿ���
    int RDC_ADU_TurnLLamp;           // ��ת�ƿ���
    int RDC_ADU_DblFlashLamp;        // ˫���ƿ���
    int RDC_ADU_LowBeamLamp;         // ����ƿ���
    int RDC_ADU_WidthLamp;           // ʾ��ƿ���
    int RDC_ADU_HighBeamLamp;        // Զ��ƿ���
    int RDC_ADU_FogLamp;             // ��ƿ���
    int RDC_ADU_BrkLamp;             // �ƶ��ƿ���
} RDC_IN;

// ============================================================================
// �ṹ�嶨�� - ��������������
// ============================================================================

/**
 * @brief �������Ʋ����ṹ��
 * @details ����ʵʱ״̬�Ϳ��Ʋ���
 */
typedef struct _vehicle_params
{
    double condition_time_stamp;     // ״̬ʱ���
    double gps_time_stamp;           // GPSʱ���
    double vehicle_speed;            // �����ٶ� (km/h)
    float steer_angle;               // ת��� (��)
    unsigned steer_angle_speed;      // ת����ٶ� (��/��)
    float steer_cmd_callback;        // ת������ص�
    double engine_rpm;               // ������ת�� (rpm)
    char target_gear;                // Ŀ�굵λ
    double acc_pos;                  // ����λ�� (%)
    double brake_press;              // �ƶ�ѹ�� (bar)
    char current_gear;               // ��ǰ��λ��'0'-'6', 'P', 'N'
    double latitude;                 // γ�� (��)
    double longitude;                // ���� (��)
    double heading_angle;            // ����� (��)
} VPARAMS, *PVPARAMS;

/**
 * @brief CAN���ݽṹ��
 * @details CAN���߿������� (��ʱ����)
 */
struct CanData
{
    int handshake_control;           // ���ֿ��ƣ�0-�Ͽ���1-����
    int gear_control;                // ��λ����
    int park_control;                // פ�����ƣ�0-�ͷţ�1-פ��
    double brake_deceleration;       // ɲ�����ٶȿ��� (m/s2)
    double steering_angle;           // �����̽Ƕȿ��� (��)
    double steering_speed;           // �����̽��ٶȿ��� (��/��)
    double vehicle_speed;            // ���ٿ��� (km/h)
};

/**
 * @brief λ�ò�ֵ�ṹ��
 * @details ��ά����λ����Ϣ
 */
typedef struct tagPosition
{
    double x;                        // X���� (m)
    double y;                        // Y���� (m)
    
    // ���캯��
    tagPosition(double _x, double _y) : x(_x), y(_y) {}
    tagPosition() : x(0.0), y(0.0) {}
    
    // �Ƚ������
    bool operator==(const tagPosition &pt) { return (x == pt.x && y == pt.y); }
} CPosition;

/**
 * @brief ��ͼ���������ݽṹ��
 * @details ��ͼģ�鴫�ݵ�·���Ϳ�����Ϣ
 */
struct MapToControl
{
    long version;                    // �汾��/ʱ���
    int frameNum;                    // ֡����
    int frameId;                     // ��ǰ֡��
    int validNumInFrame;             // ��֡��Ч��������
    CPosition info[50];              // ·�������� (�̶�����50)
};

// ============================================================================
// CAN���Ľ����ṹ�嶨�� (Author: LKS)
// ============================================================================

/**
 * @brief ������̬״̬�ṹ��
 * @details CAN ID: 0x4A2 - ������̬״̬��Ϣ
 */
struct VCU_VehDynStatus
{
    double vehicle_speed;            // ������ǰ�ٶ� (km/h)
    float Veh_Ramp;                  // �����¶� (��)
    uint8_t VehDynStat_RollCnt;      // ѭ������
    uint8_t VehDyncStat_CheckSum;    // У���
};
struct VCU_VehDynStatus revin_4a2;   // ȫ��ʵ��

/**
 * @brief ����״̬�ṹ��
 * @details CAN ID: 0x441 - ����ϵͳ״̬��Ϣ
 */
struct VCU_DriveStatus
{
    uint8_t Drv_RunDir;              // ����ʵ���˶�����0-ǰ����1-����
    uint8_t Drv_DrvCtrlMode;         // ��ǰ��������ģʽ
    uint8_t Drv_WorkMode;            // ����ϵͳ����ģʽ
    int Drv_GearAct;                 // ʵ�ʵ�λ
    float Drv_MotTq;                 // ���ʵ��ת�� (Nm)
    uint16_t Drv_MotorSpeed;         // ���ʵ��ת�� (rpm)
    uint8_t DrvStat_RollCnt;         // ѭ������
    uint8_t Drv_ErrLevel;            // ����ϵͳ���ϵȼ�
    uint8_t DrvStat_CheckSum;        // У���
};
struct VCU_DriveStatus revin_441;    // ȫ��ʵ��

/**
 * @brief �ƶ�״̬�ṹ��
 * @details CAN ID: 0x411 - �ƶ�ϵͳ״̬��Ϣ
 */
struct VCU_BrakeStatus
{
    uint8_t Brk_BrkCtrlMode;         // ��ǰ�ƶ�����ģʽ
    uint8_t Brk_WorkMode;            // �г��ƶ�����ģʽ
    float Brk_BrkPres;               // ��ǰʵ���ƶ�ѹ�� (bar)
    uint8_t BrkStat_RollCnt;         // ѭ������
    uint8_t Brk_ErrLevel;            // �г��ƶ�ϵͳ���ϵȼ�
    uint8_t BrkStat_CheckSum;        // У���
};
struct VCU_BrakeStatus revin_411;    // ȫ��ʵ��

/**
 * @brief ת��״̬�ṹ��
 * @details CAN ID: 0x431 - ת��ϵͳ״̬��Ϣ
 */
struct VCU_SteeringStatus
{
    uint8_t Str_StrCtrlMode;         // ��ǰת�����ģʽ
    uint8_t Str_WorkMode;            // ת����ģʽ
    float Str_StrWhlAngle;           // ʵ�ʷ�����ת�� (��)
    float Str_StrWhlAngleSpd;        // ʵ�ʷ�����ת���ٶ� (��/��)
    uint8_t StrStat_RollCnt;         // ѭ������
    uint8_t Str_ErrLevel;            // ת��ϵͳ���ϵȼ�
    uint8_t StrStat_CheckSum;        // У���
};
struct VCU_SteeringStatus revin_431; // ȫ��ʵ��

/**
 * @brief ���״̬01�ṹ��
 * @details CAN ID: 0x471 - ��ػ���״̬��Ϣ
 */
struct VCU_BatStatus01
{
    float LVBat_Volt;                // 12V��ѹ��ص�ѹ (V)
    float Bat_BatCur;                // ������ص��� (A)
    float Bat_BatVolt;               // ������ص�ѹ (V)
    float Bat_BatTemp;               // ��������¶� (��C)
    uint8_t BatStat1_RollCnt;        // ѭ������
    uint8_t Bat_ChrgSt;              // ������س��״̬
    uint8_t BatStat1_CheckSum;       // У���
};
struct VCU_BatStatus01 revin_471;    // ȫ��ʵ��

/**
 * @brief ���״̬02�ṹ��
 * @details CAN ID: 0x473 - ��ؽ���״̬��Ϣ
 */
struct VCU_BatStatus02
{
    float Bat_BatSOC;                // �������SOC (%)
    float Bat_BatSOH;                // �������SOH (%)
    uint8_t BatStat2_RollCnt;        // ѭ������
    uint8_t Bat_ErrLevel;            // ��Դϵͳ���ϵȼ�
    uint8_t BatStat2_CheckSum;       // У���
};
struct VCU_BatStatus02 revin_473;    // ȫ��ʵ��

/**
 * @brief ����״̬�ṹ��
 * @details CAN ID: 0x451 - ����ϵͳ״̬��Ϣ
 */
struct VCU_EmrgStatus
{
    uint8_t Emrg_Sw_St;              // ����ͣ״̬��0-������1-����
    uint8_t RrCrashTrg_St;           // ����ײ����״̬��0-������1-����
    uint8_t FrCrashTrg_St;           // ǰ��ײ����״̬��0-������1-����
    uint8_t LeCrashTrg_St;           // ����ײ����״̬��0-������1-����
    uint8_t RiCrashTrg_St;           // ����ײ����״̬��0-������1-����
    uint8_t Emrg_VehEmrgStopErr;     // ����ͣ��������
    uint8_t Emrg_ADLEmrgStopErr;     // Զ�̽������ܹ���
    uint8_t Emrg_BckCrashSwErr;      // �󴥱���ײ����
    uint8_t Emrg_FrntCrashSwErr;     // ǰ������ײ����
    uint8_t Emrg_LeftCrashSwErr;     // �󴥱���ײ����
    uint8_t Emrg_RightCrashSwErr;    // �Ҵ�����ײ����
    uint8_t Emrg_EmrgyCmdOfflErr;    // ��������ָ��ĵ���
    uint8_t Emrg_BckSlipWarn;        // �ﳵ���棺0-������1-����
    uint8_t EmrgStat_RollCnt;        // ѭ������
    uint8_t Emrg_ErrLevel;           // ����ϵͳ���ϵȼ�
    uint8_t EmrgStat_CheckSum;       // У���
};
struct VCU_EmrgStatus revin_451;     // ȫ��ʵ��

// ============================================================================
// ȫ�ֱ���ʵ����
// ============================================================================

// ��������ʵ��
VPARAMS cur_params;

// ��ͼ��������
vector<double> map_latitude_v;           // ��ͼγ������
vector<double> map_longitude_v;          // ��ͼ��������

// �߳�ͬ��
pthread_mutex_t my_mutex = PTHREAD_MUTEX_INITIALIZER;

// ============================================================================
// UDPͨ�����ݽṹ��
// ============================================================================

/**
 * @brief UDP�������ݽṹ��
 * @details UDPͨ�Ž��յĳ����͵�ͼ����
 */
typedef struct udp_data
{
    unsigned long id;                    // ���ݰ�ID
    double end_speed;                    // Ŀ���ٶ� (km/h)
    double car_latitude;                 // ����γ�� (��)
    double car_longitude;                // �������� (��)
    double car_heading_angle;            // ��������� (��)
    double map_latitude[10];             // ��ͼγ������ (��)
    double map_longitude[10];            // ��ͼ�������� (��)
} UDP_DATA;

/**
 * @brief UDP�������ݽṹ��
 * @details UDPͨ�ŷ��͵Ŀ�������
 */
typedef struct udp_senddata
{
    double keep_Angle;                   // ���ֽǶ� (��)
} UDP_SENDDATA;

// ============================================================================
// PID�������ṹ��
// ============================================================================

/**
 * @brief PID�������ṹ��
 * @details ����ɲ���ȿ��Ƶ�PID�㷨ʵ��
 */
struct PIDController
{
    // PID����
    double kp, ki, kd, dt;               // ���������֡�΢��ϵ��������ʱ��
    
    // ����޷�
    double output_min, output_max;       // �����Сֵ�����ֵ
    
    // ״̬����
    double integral;                     // �����ۻ�ֵ
    double prev_error;                   // ��һ�����ֵ
    bool first_call;                     // �״ε��ñ�־

    /**
     * @brief ���캯��
     * @param _kp ����ϵ��
     * @param _ki ����ϵ��
     * @param _kd ΢��ϵ��
     * @param _dt ����ʱ�� (��)
     * @param _min �����Сֵ
     * @param _max ������ֵ
     */
    PIDController(double _kp, double _ki, double _kd, double _dt,
                  double _min = 0.0, double _max = 100.0)
        : kp(_kp),
          ki(_ki * _dt),
          kd(_kd / _dt),
          dt(_dt),
          output_min(_min),
          output_max(_max),
          integral(0.0),
          prev_error(0.0),
          first_call(true)
    {
    }

    /**
     * @brief ����PID������״̬
     * @details ������������ʷ���
     */
    void reset()
    {
        integral = 0.0;
        prev_error = 0.0;
        first_call = true;
    }

    // ���� PID ��������޷��� [output_min, output_max]
    double update(double setpoint, double measurement)
    {
        double error = setpoint - measurement;
        integral += error;
        double derivative = first_call ? 0.0 : (error - prev_error);
        first_call = false;
        prev_error = error;

        double out = kp * error + ki * integral + kd * derivative;
        return std::clamp(out, output_min, output_max);
    }
};

typedef struct _gis_info
{
    double latitude;
    double longitude;
    double heading_angle;
} GIS, *PGIS;

GIS GIS_map;

// ����֡�ṹ
struct CanFrame
{
    std::string name; // ֡���ƣ���̬����
    uint8_t data[8];  // ����֡����
};
// ����һ�� CAN ֡��������
CanFrame canFrameData[8];
std::vector<can_frame> can_frame_log;
// 8��CAN ID
uint32_t can_ids[8] = {0x210, 0x220, 0x230, 0x240, 0x251, 0x260, 0x262, 0x272};

// log��¼
static std::mutex log_mutex;              // ��־�ļ�������
static std::string current_log_timestamp; // ��ǰ��־�ļ�ʱ���
static int log_counter = 0;               // ��ǰ�ļ���¼����
static std::ofstream log_stream;          // �ļ�������
static const int MAX_LOG_ENTRIES = 5000;  // ÿ���ļ�����¼��
static const int FLUSH_INTERVAL = 100;    // ÿ100��ˢ��һ�λ�����

int Map_ind = 0;                 // Ԥ����±� ȫ�ֱ���
DECISION_IN decision_control_in; // ��������
CONTROL_OUT control_out;         // �������
RDC_IN rdc_in;                   // Զ�̼�ʻ����ָ����Ϣ

UDP_SENDDATA send_data_udp; // ����

uint8_t udp_monitor = 0; // udp��ؾ��߼�����

// ʮ����תʮ�����ƺ���
/**
 * @brief ������ֵת��Ϊʮ�������ַ���
 * @param value Ҫת��������ֵ
 * @param width �������С��ȣ�������ǰ׺������������0���
 * @param prefix �Ƿ����"0x"ǰ׺
 * @param uppercase �Ƿ�ʹ�ô�д��ĸ
 * @return ��ʽ�����ʮ�������ַ���
 *
 * @example
 *   decToHex(255)          // ���� "FF"
 *   decToHex(255, 4)       // ���� "00FF"
 *   decToHex(255, 4, true) // ���� "0x00FF"
 *   decToHex(255, 0, true, false) // ���� "0xff"
 *   decToHex(0x410)        // ���� "410" (����CAN ID��)
 */
std::string decToHex(uint32_t value, int width = 0, bool prefix = true, bool uppercase = true)
{
    std::stringstream ss;

    // ���ý��ƺ����
    ss << std::hex;
    ss << std::setfill('0');

    // ���ÿ��
    if (width > 0)
    {
        ss << std::setw(width);
    }

    // ���ô�Сд
    if (uppercase)
    {
        ss << std::uppercase;
    }

    // ���ֵ
    ss << value;

    // ���ǰ׺
    if (prefix)
    {
        return "0x" + ss.str();
    }

    return ss.str();
}

// PIDɲ�����ĺ�������������ǰ�ٶȣ�Ŀ���ٶ�
// ����ֵ����Ҫ��ɲ��ѹ��ֵ��0-100��
// ע�⣺�˺�����Ҫ����ʵ��������е������Ż�
double calculateBrakePressure(double current_speed, double endSpeed)
{
    static PIDController pid(5.0, 0.5, 1.0, 0.1); // PID ������kp, ki, kd, dt

    // ǰ���ֵ�
    double a_des;
    if (current_speed <= 10.0)
        a_des = 1.0;
    else if (current_speed <= 20.0)
        a_des = 2.5;
    else if (current_speed <= 30.0)
        a_des = 4.0;
    else
        a_des = 5.0;
    double ff_bar = (a_des / 5.0) * 100.0;

    // PID ����
    double fb_bar = pid.update(endSpeed, current_speed);

    // �ϲ����޷�
    double brake_bar = ff_bar + fb_bar;
    return std::clamp(brake_bar, 0.0, 100.0);
}

// ��ȡ��ǰʱ�������ȷ�����룩
std::string getCurrentTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    std::time_t time = value.count() / 1000;
    struct tm *ptm = localtime(&time);
    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", ptm);
    return std::string(buffer) + "." + std::to_string(value.count() % 1000);
}

typedef struct tagPlanRoadToControl
{
    double latitude;
    double longitude;
    double headingAngle;
    int status; // ??42
} planRoadToControl;

double inline degreeToRadian(double angle)
{
    return (angle * 3.1415926 / 180.0);
}
double inline radianToDegree(double radian)
{
    double degree = fmod(radian, (2 * 3.1415926));
    return degree * 180 / 3.1415926;
}
double aTr(double angle)
{
    if (angle < -180)
    {
        angle += 360;
    }
    if (angle > 180)
    {
        angle -= 360;
    }
    return angle;
}
double distance(double lat1, double lon1, double lat2, double lon2)
{

    double dLat = (lat2 - lat1) * 3.1415926 / 180.0;
    double dLon = (lon2 - lon1) * 3.1415926 / 180.0;

    lat1 = lat1 * 3.1415926 / 180.0;
    lat2 = lat2 * 3.1415926 / 180.0;

    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return 6378.140 * c * 1000;
}
// ????????????????????????? [0,360]
double bearing(double latitude1, double longitude1, double latitude2, double longitude2)
{
    double radLatitude1 = degreeToRadian(latitude1);
    double radLatitude2 = degreeToRadian(latitude2);
    double radLongitude1 = degreeToRadian(longitude1);
    double radLongitude2 = degreeToRadian(longitude2);

    double a = sin(radLongitude2 - radLongitude1) * cos(radLatitude2);
    double b = cos(radLatitude1) * sin(radLatitude2) - sin(radLatitude1) * cos(radLatitude2) * cos(radLongitude2 - radLongitude1);
    double ret = radianToDegree(atan2(a, b));
    ret = (ret < 0) ? ret + 360 : ret;
    return ret;
}
CPosition getTargetLngLat(double lat, double lng, double angleDegree, double dist)
{
    double a = 6378137;
    double b = 6356752.3142;
    double f = 1.0 / 298.257223563;

    double alpha1 = degreeToRadian(angleDegree);
    double sinAlpha1 = sin(alpha1);
    double cosAlpha1 = cos(alpha1);

    double tanU1 = (1 - f) * tan(degreeToRadian(lat));
    double cosU1 = 1 / sqrt((1 + tanU1 * tanU1));
    double sinU1 = tanU1 * cosU1;
    double sigma1 = atan2(tanU1, cosAlpha1);
    double sinAlpha = cosU1 * sinAlpha1;
    double cosSqAlpha = 1 - sinAlpha * sinAlpha;
    double uSq = cosSqAlpha * (a * a - b * b) / (b * b);
    double A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
    double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));

    double sigma = dist / (b * A);
    double sigmaP = 2 * 3.1415926;
    double sinSigma = 0.0;
    double cosSigma = 0.0;
    double cos2SigmaM = 0.0;
    while (fabs(sigma - sigmaP) > 0.000000000001)
    {
        cos2SigmaM = cos(2 * sigma1 + sigma);
        sinSigma = sin(sigma);
        cosSigma = cos(sigma);
        double deltaSigma = B * sinSigma * (cos2SigmaM + B / 4 * (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) - B / 6 * cos2SigmaM * (-3 + 4 * sinSigma * sinSigma) * (-3 + 4 * cos2SigmaM * cos2SigmaM)));
        sigmaP = sigma;
        sigma = dist / (b * A) + deltaSigma;
    }

    double tmp = sinU1 * sinSigma - cosU1 * cosSigma * cosAlpha1;
    double lat2 = atan2(sinU1 * cosSigma + cosU1 * sinSigma * cosAlpha1, (1 - f) * sqrt(sinAlpha * sinAlpha + tmp * tmp));
    double lambdA = atan2(sinSigma * sinAlpha1, cosU1 * cosSigma - sinU1 * sinSigma * cosAlpha1);
    double C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
    double L = lambdA - (1 - C) * f * sinAlpha * (sigma + C * sinSigma * (cos2SigmaM + C * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM)));
    double lng2 = lng + radianToDegree(L);
    return CPosition(radianToDegree(lat2), lng2);
}
CPosition interpolatePosition(CPosition p1, CPosition p2, float duration, float t)
{
    float k = t / duration;
    k = k > 0 ? k : 0;
    k = k > 1 ? 1 : k;
    CPosition ret;
    ret.x = p1.x + k * (p2.x - p1.x);
    ret.y = p1.y + k * (p2.y - p1.y);
    return ret;
}
double F03(double t)
{
    return 1.0 / 6 * (-t * t * t + 3 * t * t - 3 * t + 1);
}
double F13(double t)
{
    return 1.0 / 6 * (3 * t * t * t - 6 * t * t + 4);
}
double F23(double t)
{
    return 1.0 / 6 * (-3 * t * t * t + 3 * t * t + 3 * t + 1);
}
double F33(double t)
{
    return 1.0 / 6 * t * t * t;
}
void ThreeOrderBSplineInterpolatePt(CPosition *&pt, int &Num, int *InsertNum)
{
    if (pt == NULL || InsertNum == NULL)
        return;
    int InsertNumSum = 0;
    for (int i = 0; i < Num - 1; i++)
        InsertNumSum += InsertNum[i];
    CPosition *temp = new CPosition[Num + 2];
    for (int i = 0; i < Num; i++)
        temp[i + 1] = pt[i];
    temp[0].x = 2 * temp[1].x - temp[2].x;
    temp[0].y = 2 * temp[1].y - temp[2].y;
    temp[Num + 1].x = 2 * temp[Num].x - temp[Num - 1].x;
    temp[Num + 1].y = 2 * temp[Num].y - temp[Num - 1].y;
    CPosition NodePt1, NodePt2, NodePt3, NodePt4;
    double t;
    delete[] pt;
    pt = new CPosition[Num + InsertNumSum];
    int totalnum = 0;
    for (int i = 0; i < Num - 1; i++)
    {
        NodePt1 = temp[i];
        NodePt2 = temp[i + 1];
        NodePt3 = temp[i + 2];
        NodePt4 = temp[i + 3];
        double dt = 1.0 / (InsertNum[i] + 1);
        for (int j = 0; j < InsertNum[i] + 1; j++)
        {
            t = dt * j;
            pt[totalnum].x = F03(t) * NodePt1.x + F13(t) * NodePt2.x + F23(t) * NodePt3.x + F33(t) * NodePt4.x;
            pt[totalnum].y = F03(t) * NodePt1.y + F13(t) * NodePt2.y + F23(t) * NodePt3.y + F33(t) * NodePt4.y;
            totalnum++;
        }
        if (i == Num - 2)
        {
            t = 1;
            pt[totalnum].x = F03(t) * NodePt1.x + F13(t) * NodePt2.x + F23(t) * NodePt3.x + F33(t) * NodePt4.x;
            pt[totalnum].y = F03(t) * NodePt1.y + F13(t) * NodePt2.y + F23(t) * NodePt3.y + F33(t) * NodePt4.y;
            totalnum++;
        }
    }
    delete[] temp;
    Num = Num + InsertNumSum;
}

vector<planRoadToControl> insertTargetPoints(vector<CPosition> iPts)
{

    vector<planRoadToControl> ret;
    vector<CPosition> tmp;
    if (iPts.empty() || iPts.size() <= 1)
        return ret;
    tmp.emplace_back(iPts[0]);
    for (int i = 1; i < iPts.size() - 1; i++)
    {
        double angle1 = bearing(iPts[i - 1].x, iPts[i - 1].y, iPts[i].x, iPts[i].y);
        double angle2 = bearing(iPts[i].x, iPts[i].y, iPts[i + 1].x, iPts[i + 1].y);
        if (fabs(aTr(angle1 - angle2)) > 50)
        {
            CPosition t1 = getTargetLngLat(iPts[i].x, iPts[i].y, angle1, -12);
            CPosition t2 = getTargetLngLat(iPts[i].x, iPts[i].y, angle2, 12);
            if (distance(iPts[i - 1].x, iPts[i - 1].y, iPts[i].x, iPts[i].y) > 12)
                tmp.emplace_back(t1);
            tmp.emplace_back(iPts[i]);
            if (distance(iPts[i].x, iPts[i].y, iPts[i + 1].x, iPts[i + 1].y) > 12)
                tmp.emplace_back(t2);
        }
        else
        {
            tmp.emplace_back(iPts[i]);
        }
    }
    tmp.emplace_back(iPts.back());
    iPts.swap(tmp);

    int num = iPts.size();
    CPosition *testpt = new CPosition[num];
    for (int i = 0; i < num; i++)
    {
        testpt[i] = iPts[i];
    }
    int num2 = num;
    int *Intnum = new int[num - 1];
    for (int i = 0; i < num - 1; i++)
    {
        Intnum[i] = (int)(distance(testpt[i].x, testpt[i].y, testpt[i + 1].x, testpt[i + 1].y) / 0.1); //  ?????????????????n????
    }

    ThreeOrderBSplineInterpolatePt(testpt, num2, Intnum); //  ????B????????

    vector<CPosition> newPts;
    for (int i = 0; i < num2 - 1; i++)
    {
        newPts.push_back(testpt[i]);
        double d = distance(testpt[i].x, testpt[i].y, testpt[i + 1].x, testpt[i + 1].y);
        if (d >= 0.5)
        {
            int n = d / 0.1;
            for (int j = 1; j < n; j++)
            {
                CPosition pt = interpolatePosition(CPosition(testpt[i].x, testpt[i].y), CPosition(testpt[i + 1].x, testpt[i + 1].y), n, j);
                newPts.emplace_back(pt);
            }
        }
    }

    for (int i = 0; i < newPts.size() - 1; i++)
    {
        planRoadToControl info;
        info.latitude = newPts[i].x;
        info.longitude = newPts[i].y;
        info.headingAngle = bearing(newPts[i].x, newPts[i].y, newPts[i + 1].x, newPts[i + 1].y);
        ret.emplace_back(info);
    }

    delete[] testpt;
    delete[] Intnum;
    return ret;
}

// �������� 8 ֡ CAN ����
bool batch_send_frames(int sockfd, struct can_frame frames[], int frame_count)
{
    if (sockfd < 0 || frames == NULL || frame_count <= 0 || frame_count > 8)
    {
        fprintf(stderr, "�������Ͳ�������: sockfd=%d, frames=%p, count=%d\n",
                sockfd, frames, frame_count);
        return false;
    }

    // ʹ��sendmmsg�������Ͷ��CAN֡
    struct mmsghdr msgs[8];
    struct iovec iov[8];

    memset(msgs, 0, sizeof(msgs));
    memset(iov, 0, sizeof(iov));

    // ׼���������͵����ݽṹ
    for (int i = 0; i < frame_count; ++i)
    {
        iov[i].iov_base = &frames[i];
        iov[i].iov_len = sizeof(struct can_frame);

        msgs[i].msg_hdr.msg_iov = &iov[i];
        msgs[i].msg_hdr.msg_iovlen = 1;
    }

    // ִ����������
    int sent = sendmmsg(sockfd, msgs, frame_count, 0);

    if (sent < 0)
    {
        perror("sendmmsgʧ��");
        return false;
    }
    else if (sent != frame_count)
    {
        fprintf(stderr, "�������Ͳ�����: �ѷ���%d/%d֡\n", sent, frame_count);
        return false;
    }

    return true;
}

// ��֡���ͺ��� - ������������ʧ��ʱ�Ļ��˷���
bool send_single_frame(int sockfd, struct can_frame *frame)
{
    if (sockfd < 0 || frame == NULL)
    {
        return false;
    }

    int ret = write(sockfd, frame, sizeof(struct can_frame));
    return (ret == sizeof(struct can_frame));
}

/*
===================================���򣺽���CAN����===================================
author��lks
*/
// ID��4A2
VCU_VehDynStatus parseVCU_VehDynStatus(const uint8_t data[8]) // ID:4A2  ����̬״̬
{
    VCU_VehDynStatus result;
    uint16_t rawSpd = (data[2] << 8) | data[3];
    result.vehicle_speed = rawSpd * 0.003906f;
    // std::cout << "���������ٶȣ�" << std::fixed << std::setprecision(2) << result.vehicle_speed << " km/h | ";
    int16_t rawRamp = ((data[5] & 0x0F) << 8) | data[4];
    if (rawRamp & 0x800)
    {
        rawRamp |= 0xF000; // ��չ����λ
    }
    result.Veh_Ramp = rawRamp * 0.05f;
    // std::cout << "���������¶ȣ�" << std::fixed << std::setprecision(2) << result.Veh_Ramp << " % | ";
    result.VehDynStat_RollCnt = (data[6] >> 0) & 0x0F;
    // std::cout << "ѭ��������" << (int)result.VehDynStat_RollCnt << " | "<<endl;
    result.VehDyncStat_CheckSum = data[7];
    return result;
}

VCU_DriveStatus parseVCU_DriveStatus(const uint8_t data[8]) // ID:441  ����״̬
{
    VCU_DriveStatus result;
    result.Drv_RunDir = (data[0] >> 0) & 0x03;
    // std::cout << "����ʵ���˶�����";
    switch (result.Drv_RunDir)
    {
    case 0x0:
        // std::cout << "Invalid(��Ч)";
        break;
    case 0x1:
        // std::cout << "Forward(ǰ��)";
        break;
    case 0x2:
        // std::cout << "Backward(����)";
        break;
    case 0x3:
        // std::cout << "Stopped(ֹͣ)";
        break;
    default:
        // std::cout << "error";
        break;
    }
    // std::cout << " | ";
    result.Drv_DrvCtrlMode = (data[0] >> 2) & 0x03;
    // std::cout << "��������ģʽ��";
    switch (result.Drv_DrvCtrlMode)
    {
    case 0x0:
        // std::cout << "Invalid";
        break;
    case 0x1:
        // std::cout << "Speed Control";
        break;
    case 0x2:
        // std::cout << "Torque Control";
        break;
    case 0x3:
        // std::cout << "Acceleration Pedal Control";
        break;
    default:
        // std::cout << "error";
        break;
    }
    // std::cout << " | ";
    result.Drv_WorkMode = (data[0] >> 4) & 0x0F;
    result.Drv_GearAct = (data[1] >> 0) & 0x0F;
    // std::cout << "ʵ�ʵ�λ��";
    switch (result.Drv_GearAct)
    {
    case 0x0:
        // std::cout << "Invalid";
        break;
    case 0x1:
        // std::cout << "Parking(P)";
        break;
    case 0x3:
        // std::cout << "Reverse(R)";
        break;
    case 0x5:
        // std::cout << "Neutral(N)";
        break;
    case 0x9:
        // std::cout << "Drive(D)";
        break;
    default:
        // std::cout << "δ֪��λ";
        break;
    }
    // std::cout << " | ";
    int16_t rawTorque = (data[2] << 8) | data[3];
    result.Drv_MotTq = rawTorque * 0.1f;
    // std::cout << "���ʵ��ת�أ�" << result.Drv_MotTq << " Nm | ";
    result.Drv_MotorSpeed = (data[4] << 8) | data[5];
    // std::cout << "���ʵ��ת�٣�" << result.Drv_MotorSpeed << " rpm | ";
    result.DrvStat_RollCnt = (data[6] >> 0) & 0x0F;
    // std::cout << "ѭ��������" << (int)result.DrvStat_RollCnt << " | ";
    result.Drv_ErrLevel = (data[6] >> 4) & 0x0F;
    result.DrvStat_CheckSum = data[7];
    return result;
}
VCU_BrakeStatus parseVCU_BrakeStatus(const uint8_t data[8]) // ID411 �ƶ�״̬
{
    VCU_BrakeStatus result;
    result.Brk_BrkCtrlMode = (data[0] >> 0) & 0x03;
    // std::cout << "��ǰ�ƶ�����ģʽ��";
    switch (result.Brk_BrkCtrlMode)
    {
    case 0x0:
        // std::cout << "Invalid";
        break;
    case 0x1:
        // std::cout << "Brake Pressure Mode";
        break;
    default:
        // std::cout << "δ֪ģʽ";
        break;
    }
    // std::cout << " | ";
    result.Brk_WorkMode = (data[0] >> 4) & 0x0F;
    result.Brk_BrkPres = data[1];
    // std::cout << "��ǰʵ���ƶ�ѹ����" << result.Brk_BrkPres << " bar | ";
    result.BrkStat_RollCnt = (data[6] >> 0) & 0x0F;
    // std::cout << "ѭ��������" << (int)result.BrkStat_RollCnt << " | ";
    result.Brk_ErrLevel = (data[6] >> 4) & 0x0F;
    result.BrkStat_CheckSum = data[7];
    return result;
}

VCU_SteeringStatus parseVCU_SteeringStatus(const uint8_t data[8]) /// ID:431 ?ת��״̬
{
    VCU_SteeringStatus result;
    result.Str_StrCtrlMode = (data[0] >> 2) & 0x03;
    // std::cout << "����ת�����ģʽ��";
    switch (result.Str_StrCtrlMode)
    {
    case 0x0:
        // std::cout << "Invalid";
        break;
    case 0x1:
        // std::cout << "AngleControlMode";
        break;
    case 0x2:
        // std::cout << "DirectiveAngleControlMode";
        break;
    default:
        // std::cout << "error";
        break;
    }
    // std::cout << " | ";
    result.Str_WorkMode = (data[0] >> 4) & 0x0F;
    int16_t rawAngle = (data[2] << 8) | data[3];
    result.Str_StrWhlAngle = rawAngle * 0.05f;
    // std::cout << "�����Ҹ���" << result.Str_StrWhlAngle << " deg | ";
    uint16_t rawAngleSpd = (data[4] << 8) | data[5];
    result.Str_StrWhlAngleSpd = rawAngleSpd * 0.1f - 1800;
    // std::cout << "ʵ�ʷ�����ת���ٶȣ�" << result.Str_StrWhlAngleSpd << " degree/s | ";
    result.StrStat_RollCnt = (data[6] >> 0) & 0x0F;
    result.Str_ErrLevel = (data[6] >> 4) & 0x0F;
    result.StrStat_CheckSum = data[7];
    return result;
}

VCU_BatStatus01 parseVCU_BatStatus01(const uint8_t data[8]) // ID:471  ���״̬01
{
    VCU_BatStatus01 result;
    uint16_t rawLVVolt = (data[0] << 2) | (data[1] >> 6);
    result.LVBat_Volt = rawLVVolt * 0.1f;
    // std::cout << "12V��ѹ��ص�ѹ��" << result.LVBat_Volt << " V | ";
    int16_t rawBatCur = ((data[1] & 0x3F) << 8) | data[2];
    if (rawBatCur & 0x2000)
    {
        rawBatCur |= 0xC000; // ��չ����λ
    }
    result.Bat_BatCur = rawBatCur * 0.1f;
    // std::cout << "������ص�����" << result.Bat_BatCur << " A | ";
    uint16_t rawBatVolt = (data[3] << 8) | data[4];
    result.Bat_BatVolt = rawBatVolt * 0.1f;
    // std::cout << "������ص�ѹ��" << result.Bat_BatVolt << " V | ";
    int8_t rawBatTemp = data[5];
    result.Bat_BatTemp = rawBatTemp - 50; // ƫ����Ϊ-50
    // std::cout << "��������¶ȣ�" << result.Bat_BatTemp << " ��C | ";
    result.BatStat1_RollCnt = (data[6] >> 0) & 0x0F;
    // std::cout << "ѭ��������" << (int)result.BatStat1_RollCnt << " | ";
    result.Bat_ChrgSt = (data[6] >> 6) & 0x03;
    // std::cout << "������س��״̬��";
    switch (result.Bat_ChrgSt)
    {
    case 0x0:
        // std::cout << "Not Chrg(δ���)";
        break;
    case 0x1:
        // std::cout << "Charging(�����)";
        break;
    default:
        // std::cout << "error";
        break;
    }
    // std::cout << " | ";
    result.BatStat1_CheckSum = data[7];
    return result;
}
VCU_BatStatus02 parseVCU_BatStatus02(const uint8_t data[8]) // ID:473 ?���״̬02
{
    VCU_BatStatus02 result;
    uint16_t rawSOC = (data[1] >> 4) | (data[0] << 4);
    result.Bat_BatSOC = rawSOC * 0.1f;
    // std::cout << "�������SOC��" << result.Bat_BatSOC << " % | ";
    uint16_t rawSOH = ((data[1] & 0x0F) << 8) | data[2];
    result.Bat_BatSOH = rawSOH * 0.1f;
    // std::cout << "�������SOH��" << result.Bat_BatSOH << " % | ";
    result.BatStat2_RollCnt = (data[6] >> 0) & 0x0F;
    // std::cout << "ѭ��������" << (int)result.BatStat2_RollCnt << " | ";
    result.Bat_ErrLevel = (data[6] >> 4) & 0x0F;
    result.BatStat2_CheckSum = data[7];
    return result;
}
VCU_EmrgStatus parseVCU_EmrgStatus(const uint8_t data[8]) // ID:451  ����״̬
{
    VCU_EmrgStatus result;
    result.Emrg_Sw_St = (data[0] >> 0) & 0x03;
    // std::cout << "����ͣ״̬��";
    switch (result.Emrg_Sw_St)
    {
    case 0x0:
        // std::cout << "Released(���ܿ��عر�)";
        break;
    case 0x1:
        // std::cout << "Enabled(���ܿ��ش�)";
        break;
    default:
        // std::cout << "error";
        break;
    }
    // std::cout << " | ";
    result.RrCrashTrg_St = (data[0] >> 4) & 0x01;
    // std::cout << "����ײ����״̬��";
    // if(result.RrCrashTrg_St == 0x0)
    // std::cout << "False(δ����)";
    // else
    // std::cout << "True(����)";
    // std::cout << " | ";
    result.FrCrashTrg_St = (data[0] >> 5) & 0x01;
    // std::cout << "ǰ��ײ����״̬��";
    // if(result.FrCrashTrg_St == 0x0)
    // std::cout << "False(δ����)";
    // else
    // std::cout << "True(����)";
    // std::cout << " | ";
    result.LeCrashTrg_St = (data[0] >> 6) & 0x01;
    // std::cout << "����ײ����״̬��";
    // if(result.LeCrashTrg_St == 0x0)
    // std::cout << "False(δ����)";
    // std::cout << "True(����)";
    // std::cout << " | ";
    result.RiCrashTrg_St = (data[0] >> 7) & 0x01;
    // std::cout << "����ײ����״̬��";
    // if(result.RiCrashTrg_St == 0x0)
    // std::cout << "False(δ����)";
    // else
    //     std::cout << "True(����)";
    // std::cout << " | ";

    result.Emrg_VehEmrgStopErr = (data[1] >> 0) & 0x01;
    // std::cout << "����ͣ�������ϣ�";
    // if(result.Emrg_VehEmrgStopErr == 0x0)
    // std::cout << "Normal(����)";
    // else
    // std::cout << "Fault(����)";
    // std::cout << " | ";
    result.Emrg_ADLEmrgStopErr = (data[1] >> 1) & 0x01;
    // std::cout << "Զ�̼�ͣ�������ϣ�";
    // if(result.Emrg_ADLEmrgStopErr == 0x0)
    // std::cout << "Normal(����)";
    // else
    // std::cout << "Fault(����)";
    // std::cout << " | ";
    result.Emrg_BckCrashSwErr = (data[1] >> 4) & 0x01;
    // std::cout << "�󴥱���ײ���ϣ�";
    // if(result.Emrg_BckCrashSwErr == 0x0)
    // std::cout << "Normal(����)";
    // else
    // std::cout << "Fault(����)";
    // std::cout << " | ";
    result.Emrg_FrntCrashSwErr = (data[1] >> 5) & 0x01;
    // std::cout << "ǰ������ײ���ϣ�";
    // if(result.Emrg_FrntCrashSwErr == 0x0)
    // std::cout << "Normal(����)";
    // else
    //     std::cout << "Fault(����)";
    // std::cout << " | ";
    result.Emrg_LeftCrashSwErr = (data[1] >> 6) & 0x01;
    // std::cout << "�󴥱���ײ����";
    // if(result.Emrg_LeftCrashSwErr == 0x0)
    // std::cout << "Normal(����)";
    // else
    //     std::cout << "Fault(����)";
    // std::cout << " | ";
    result.Emrg_RightCrashSwErr = (data[1] >> 7) & 0x01;
    // std::cout << "�Ҵ�����ײ���ϣ�";
    // if(result.Emrg_RightCrashSwErr == 0x0)
    // std::cout << "Normal(����)";
    // else
    //     std::cout << "Fault(����)";
    // std::cout << " | ";
    result.Emrg_EmrgyCmdOfflErr = (data[2] >> 3) & 0x01;
    // std::cout << "��������ָ��ĵ��ߣ�";
    // if(result.Emrg_EmrgyCmdOfflErr == 0x0)
    // std::cout << "Normal(����)";
    // else
    // std::cout << "Fault(����)";
    // std::cout << " | ";
    result.Emrg_BckSlipWarn = (data[3] >> 0) & 0x03;
    // std::cout << "�ﳵ���棺";
    switch (result.Emrg_BckSlipWarn)
    {
    case 0x0:
        // std::cout << "Normal(����)";
        break;
    case 0x1:
        // std::cout << "1��";
        break;
    case 0x2:
        // std::cout << "2��";
        break;
    case 0x3:
        // std::cout << "3��";
        break;
    default:
        // std::cout << "error";
        break;
    }
    // std::cout << " | ";
    result.EmrgStat_RollCnt = (data[6] >> 0) & 0x0F;
    // std::cout << "ѭ��������" << (int)result.EmrgStat_RollCnt << " | ";
    result.Emrg_ErrLevel = (data[6] >> 4) & 0x0F;
    result.EmrgStat_CheckSum = data[7];
    return result;
}
int create_udp_socket(int port)
{
    int sockfd;
    struct sockaddr_in addr;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    return sockfd;
}

//=================================================================================

// ��ʼ��CAN socket��װ
// ��ʼ��CAN socket����������ʱ����һ��
int init_can_socket(const char *ifname)
{
    struct sockaddr_can addr;
    struct ifreq ifr;
    int sockfd;

    // ����socket
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0)
    {
        perror("socket");
        return -1;
    }

    // ��ȡ�ӿ�����
    strcpy(ifr.ifr_name, ifname);
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl");
        close(sockfd);
        return -1;
    }

    // ��socket��CAN�ӿ�
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        close(sockfd);
        return -1;
    }

    // ����Ϊ������ģʽ
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    return sockfd;
}
// ��ʱ���źŴ�������ֻ���ñ�־��udp��ؼ���
void handle_sigalrm(int sig)
{
    std::cout << "SIGALRM: set flag_canSend\n";
    flag_canSend = true;
    // udp����������س���1�����û�յ�udb����ָ����ͣ����ʵ��ʹ��ʱ�򿪣�����ʱע�͵� test
    // if (udp_monitor > 50) {
    //     begin_run = 0;
    //     fprintf(stderr, "udp Error\n");
    // } else {
    //     udp_monitor += 1;
    // }
}

// ���ļ��ж�ȡ��γ�����ݲ�����vector
void readMapDataFromFile(const string &filePath)
{
    ifstream file(filePath);
    string line;

    if (!file.is_open())
    {
        cerr << "Error opening file: " << filePath << endl;
        return;
    }

    // ���֮ǰ������
    map_latitude_v.clear();
    map_longitude_v.clear();

    // ���ж�ȡ�ļ�
    while (getline(file, line))
    {
        // ����Latitude��Longitude����ʼλ��
        size_t lat_pos = line.find("Latitude:");
        size_t lon_pos = line.find("Longitude:");

        if (lat_pos != string::npos && lon_pos != string::npos)
        {
            try
            {
                // ��ȡLatitude��Longitude��ֵ
                size_t lat_start = lat_pos + 9;  // "Latitude: "���λ��
                size_t lon_start = lon_pos + 10; // "Longitude: "���λ��

                // Ѱ��Latitude��Longitude�Ľ���λ��
                size_t lat_end = line.find(",", lat_start);
                size_t lon_end = line.find(",", lon_start);

                if (lat_end == string::npos)
                    lat_end = line.find(" ", lat_start);
                if (lon_end == string::npos)
                    lon_end = line.find(" ", lon_start);

                // ��ȡγ�Ⱥ;��Ȳ�ת��Ϊdouble
                double latitude = stod(line.substr(lat_start, lat_end - lat_start));
                double longitude = stod(line.substr(lon_start, lon_end - lon_start));

                // ����vector
                map_latitude_v.push_back(latitude);
                map_longitude_v.push_back(longitude);

                // �������8λ���ȵľ�γ��
                cout << fixed << setprecision(8)
                     << "Latitude: " << latitude << ", Longitude: " << longitude << endl;
            }
            catch (const exception &e)
            {
                cerr << "Error parsing line: " << line << " Error: " << e.what() << endl;
            }
        }
        else
        {
            cerr << "Skipping invalid line: " << line << endl;
        }
    }

    file.close();
    cout << "Successfully read " << map_latitude_v.size() << " GIS_map points." << endl;
}

// ���� $GPCHC ���Ĳ���ȡ�����ֶ�
bool parseGPCHC(const std::string &sentence, double &latitude, double &longitude, double &heading, int &status)
{
    if (sentence.find("$GPCHC") != 0)
    {
        return false;
    }

    std::vector<std::string> fields;
    std::stringstream ss(sentence);
    std::string field;

    while (std::getline(ss, field, ','))
    {
        fields.push_back(field);
    }

    if (fields.size() < 24)
    {
        std::cerr << "Incomplete $GPCHC sentence: " << sentence << std::endl;
        return false;
    }
    try
    {
        {
            heading = std::stod(fields[3]);    // ƫ����
            latitude = std::stod(fields[12]);  // γ��
            longitude = std::stod(fields[13]); // ����
            status = std::stoi(fields[21]);    // ״̬��
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error parsing $GPCHC sentence: " << e.what() << std::endl;
        return false;
    }

    return true;
}

// ���ô���
int configureSerialPort(const std::string &port)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY); // ȥ�� O_NDELAY��ʹ������ģʽ
    if (fd == -1)
    {
        std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
        return -1;
    }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0)
    {
        std::cerr << "Error getting serial attributes: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // ����λ��8
    tty.c_cflag &= ~PARENB;                     // ��У��λ
    tty.c_cflag &= ~CSTOPB;                     // ֹͣλ��1
    tty.c_cflag |= CREAD | CLOCAL;              // ���ý����������Ե��ƽ������·״̬

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // ���ù淶ģʽ
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // �������������
    tty.c_oflag &= ~OPOST;                          // ���ԭʼģʽ

    tty.c_cc[VMIN] = 1;  // ��С��ȡ�ַ���
    tty.c_cc[VTIME] = 1; // ��ʱ��0.1��

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error setting serial attributes: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    // ��մ��ڻ�����
    tcflush(fd, TCIOFLUSH); // �����������������
    std::cout << "Serial port buffer flushed successfully." << std::endl;

    return fd;
}

// ��ȡʵʱGPS�� ���¶�ѡһ
//*********************
/*����CGI610*/
/*
std::vector<double> readSerialData(int fd) {
    char buffer[256];
    std::string sentence;
    double latitude = 0.0, longitude = 0.0, heading = 0.0;
    int status = 0;
    std::vector<double> res;

    // ����������ȣ�γ�Ⱥ;��ȱ���С����� 8 λ
    std::cout << std::fixed << std::setprecision(8);

    while (true) {
        memset(buffer, 0, sizeof(buffer));
        int bytesRead = read(fd, buffer, sizeof(buffer) - 1);

        if (bytesRead < 0) {
            if (errno == EAGAIN) {
                continue; // ������ʱ�����ȴ�
            }
            std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
            break;
        }
        else if (bytesRead > 0) {
            sentence += std::string(buffer, bytesRead);

            // ���ұ���ͷ "$GPCHC"
            size_t startPos = sentence.find("$GPCHC");
            if (startPos != std::string::npos) {
                // ����ҵ��� "$GPCHC"�����Ǿ���Ϊ����һ����Ч�ı��Ŀ�ʼ
                size_t endPos = sentence.find("\n", startPos); // ���ұ��ĵĽ�β�����з���
                if (endPos != std::string::npos) {
                    std::cout << "---------------------" << endl;
                    // ��ȡ�������ݣ�ȥ����ʼ��"$GPCHC"��ĩβ�Ļ��з���
                    std::string fullSentence = sentence.substr(startPos, endPos - startPos);
                    sentence.erase(0, endPos + 1); // ɾ���Ѿ�������Ĳ���

                    // ��������ȡ����
                    if (parseGPCHC(fullSentence, latitude, longitude, heading, status)) {
                        // ʵʱ���¾�γ�ȱ�������ӡ
                        std::cout << "Updated Latitude: " << latitude
                                  << ", Longitude: " << longitude
                                  << ", Heading: " << heading
                                  << ", Status: " << status << std::endl;
                        res.clear();
                        res.push_back(latitude);
                        res.push_back(longitude);
                        res.push_back(heading);
                        res.push_back((double)status);

                        return res; // ���ؾ�γ������
                    } else {
                        std::cerr << "��������Invalid $GPCHC sentence: " << fullSentence << std::endl;
                        res.at(0) = 999.99;

                        return res; // ���س�ʼֵ
                    }
                }
            }
        }
    }
}


*/
#if 0
//INS821
std::vector<double> parseData(std::vector<uint8_t>& data) {
    double headingAngle = 0.0, longitudeData = 0.0, latitudeData = 0.0;
    int status = -1;
    std::vector<double> res(4, 0.0);  // ����һ������4��Ԫ�ص�vector���ֱ�洢latitude, longitude, heading_angle, GPS_status

    // ����֡ͷ BD DB 0B (����Э��ͼƬ)
    size_t frameStartIndex = 0;
    bool frameFound = false;
    
    // �������в���֡ͷ���� BD DB 0B
    for (size_t i = 0; i < data.size() - 2; i++) {
        if (data[i] == 0xBD && data[i+1] == 0xDB && data[i+2] == 0x0B) {
            frameStartIndex = i;
            frameFound = true;
            break;
        }
    }
    
    // ����ҵ�֡ͷ���Һ������㹻������(����92�ֽ�)
    if (frameFound && (frameStartIndex + 92) <= data.size()) {
        // ��ȡ������92�ֽ�����֡
        std::vector<uint8_t> frameData(data.begin() + frameStartIndex, data.begin() + frameStartIndex + 92);
        
        // ��������� (ƫ����3������ǣ�����Э��ͼƬ)
        // ����Э��ͼƬ���������ƫ����3��������Ϊ2�ֽڣ�LSB_first
        if (frameData.size() > 8) {
            uint16_t headingRaw = (frameData[8] << 8) | frameData[7];  // LSB_first��ʽ
            headingAngle = (headingRaw * 360.0) / 32768.0;  // ����ϵ�� (360/32768)
        }
        
        // ����γ�� (ƫ����21������Э��ͼƬ)
        // ����Э��ͼƬ��γ����ƫ����21��������Ϊ8�ֽڣ�LSB_first
        if (frameData.size() > 28) {
            uint64_t latitudeRaw = (static_cast<uint64_t>(frameData[28]) << 56) |
                                  (static_cast<uint64_t>(frameData[27]) << 48) |
                                  (static_cast<uint64_t>(frameData[26]) << 40) |
                                  (static_cast<uint64_t>(frameData[25]) << 32) |
                                  (static_cast<uint64_t>(frameData[24]) << 24) |
                                  (static_cast<uint64_t>(frameData[23]) << 16) |
                                  (static_cast<uint64_t>(frameData[22]) << 8) |
                                  static_cast<uint64_t>(frameData[21]);  // LSB_first ƴ��

            latitudeData = latitudeRaw * 1.00E-08;  // ����ϵ�� (1.00E-08)
        }
        
        // �������� (ƫ����29������Э��ͼƬ)
        // ����Э��ͼƬ��������ƫ����29��������Ϊ8�ֽڣ�LSB_first
        if (frameData.size() > 36) {
            uint64_t longitudeRaw = (static_cast<uint64_t>(frameData[36]) << 56) |
                                   (static_cast<uint64_t>(frameData[35]) << 48) |
                                   (static_cast<uint64_t>(frameData[34]) << 40) |
                                   (static_cast<uint64_t>(frameData[33]) << 32) |
                                   (static_cast<uint64_t>(frameData[32]) << 24) |
                                   (static_cast<uint64_t>(frameData[31]) << 16) |
                                   (static_cast<uint64_t>(frameData[30]) << 8) |
                                   static_cast<uint64_t>(frameData[29]);  // LSB_first ƴ��
            longitudeData = longitudeRaw * 1.00E-08;  // ����ϵ�� (1.00E-08)
        }
        
        // ����GPS״̬ (ƫ����47������Э��ͼƬ)
        // ����Э��ͼƬ��״̬��ƫ����47��������Ϊ1�ֽ�
        if (frameData.size() > 47) {
            status = frameData[47];  // ֱ�ӻ�ȡ״̬�ֽ�
        }
        
        // �������������vector
        res[0] = latitudeData;
        res[1] = longitudeData;
        res[2] = headingAngle;
        res[3] = static_cast<double>(status);
        
        // ��ӡ���������Ϣ
        std::cout << "Parsed Info: "
                  << "Latitude: " << std::fixed << std::setprecision(8) << latitudeData
                  << ", Longitude: " << std::fixed << std::setprecision(8) << longitudeData
                  << ", Heading: " << std::fixed << std::setprecision(8) << headingAngle
                  << ", Status: " << status << std::endl;
    } else {
        if (!frameFound) {
            std::cout << "Error: Frame header (BD DB 0B) not found." << std::endl;
        } else {
            std::cout << "Error: Incomplete GPS message after frame header." << std::endl;
        }
    }
    
    return res;
}
std::vector<double> readSerialData(int serialPort) {
    static std::vector<uint8_t> dataBuffer;             // �����������ڴ�Ŵ��ڽ��յ���ԭʼ����
    static std::vector<double> lastParsedFrame(0, 0.0); // ��һ֡����ֵ����γ�ȡ�����ǡ�״̬��
    static auto lastReadTime = std::chrono::steady_clock::now();
    static int emptyFrameCount = 0;
    static int totalFrameCount = 0;
    
    // ��������ϴζ�ȡ��ʱ����
    auto currentTime = std::chrono::steady_clock::now();
    auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastReadTime).count();
    
    // ÿ�ζ�ȡǰ��մ��ڻ�������ȷ����ȡ��������
    tcflush(serialPort, TCIFLUSH);
    
    // ��ȡ��������
    char buffer[1024];
    ssize_t bytesRead = read(serialPort, buffer, sizeof(buffer));
    
    // ���¶�ȡʱ��
    lastReadTime = currentTime;
    
    if (bytesRead > 0) {
        std::cout << "Read " << bytesRead << " bytes from serial port" << std::endl;
        for (ssize_t i = 0; i < bytesRead; ++i) {
            dataBuffer.push_back(static_cast<uint8_t>(buffer[i]));
        }

        // ���ƻ�������С��������������
        if (dataBuffer.size() > 1000) {
            dataBuffer.erase(dataBuffer.begin(), dataBuffer.begin() + (dataBuffer.size() - 1000));
        }
    } else {
        std::cout << "No data read from serial port" << std::endl;
    }

    // ���Ҳ�������������֡��92�ֽڣ�
    bool frameFound = false;
    for (size_t i = 0; i + 92 <= dataBuffer.size(); ++i) {
        if (dataBuffer[i] == 0xBD && dataBuffer[i+1] == 0xDB && dataBuffer[i+2] == 0x0B) {
            std::vector<uint8_t> frame(dataBuffer.begin() + i, dataBuffer.begin() + i + 92);
            std::vector<double> parsed = parseData(frame);
            
            totalFrameCount++;
            
            // ���»���֡
            lastParsedFrame = parsed;

            // ����Ѵ��������
            dataBuffer.erase(dataBuffer.begin(), dataBuffer.begin() + i + 92);
            
            frameFound = true;
            std::cout << "Found valid GPS frame, total frames: " << totalFrameCount << std::endl;
            break;
        }
    }
    
    if (!frameFound) {
        emptyFrameCount++;
        std::cout << "No valid GPS frame found, empty count: " << emptyFrameCount 
                  << ", empty rate: " << (double)emptyFrameCount / (totalFrameCount + 1) * 100 << "%" << std::endl;
    }

    // ���û����֡��������һ֡����ֵ
    return lastParsedFrame;
}
#elif 0
// ����GPS
std::vector<double> readSerialData(int fd)
{
    char buffer[256];
    std::string sentence;
    double latitude = 0.0, longitude = 0.0, heading = 0.0;
    int status = 0;
    std::vector<double> res;

    // ����������ȣ�γ�Ⱥ;��ȱ���С����� 8 λ
    std::cout << std::fixed << std::setprecision(8);

    int maxRetries = 1000; // ������Դ���
    int retryCount = 0;

    while (true)
    {
        memset(buffer, 0, sizeof(buffer));
        int bytesRead = read(fd, buffer, sizeof(buffer) - 1);

        if (bytesRead < 0)
        {
            if (errno == EAGAIN)
            {
                retryCount++;
                if (retryCount >= maxRetries)
                {
                    std::cerr << "Max retries reached. Exiting." << std::endl;
                    break; // �ﵽ������Դ����˳�
                }
                continue; // ������ʱ�����ȴ�
            }
            std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
            break;
        }
        else if (bytesRead > 0)
        {
            retryCount = 0; // �������Լ���
            sentence += std::string(buffer, bytesRead);

            // ���ұ���ͷ "$GPCHC"
            size_t startPos = sentence.find("$GPCHC");
            if (startPos != std::string::npos)
            {
                // ����ҵ��� "$GPCHC"�����Ǿ���Ϊ����һ����Ч�ı��Ŀ�ʼ
                size_t endPos = sentence.find("\n", startPos); // ���ұ��ĵĽ�β�����з���
                if (endPos != std::string::npos)
                {
                    // ��ȡ�������ݣ�ȥ����ʼ��"$GPCHC"��ĩβ�Ļ��з���
                    std::string fullSentence = sentence.substr(startPos, endPos - startPos);
                    sentence.erase(0, endPos + 1); // ɾ���Ѿ�������Ĳ���

                    // ��������ȡ����
                    if (parseGPCHC(fullSentence, latitude, longitude, heading, status))
                    {
                        // ʵʱ���¾�γ�ȱ�������ӡ
                        std::cout << "Updated Latitude: " << latitude
                                  << ", Longitude: " << longitude
                                  << ", Heading: " << heading
                                  << ", Status: " << status << std::endl;

                        res.push_back(latitude);
                        res.push_back(longitude);
                        res.push_back(heading);
                        res.push_back((double)status);

                        return res; // ���ؾ�γ������
                    }
                    else
                    {
                        // �������ʧ�ܵ���ϸ����
                        std::cerr << "��������Invalid $GPCHC sentence: " << fullSentence << std::endl;
                        res.push_back(999.99); // ���һ��Ĭ��ֵ������Խ�磩
                        return res;            // ���س�ʼֵ
                    }
                }
            }
            else
            {
                std::cerr << "δ�ҵ�����ͷ��Invalid data or missing expected header '$GPCHC'. Full sentence: " << sentence << std::endl;
            }
        }
    }

    // ���ѭ����������ʾû���ҵ���Ч����
    std::cerr << "No valid data received. Exiting..." << std::endl;
    return res; // ����һ���յĽ��
}
#elif 1
// poly 3000P
//  �� GPNAV �ַ����н��������ĸ�ֵ
//  ���� true ��ʾ�����ɹ�
bool parseGPNAV(const std::string &sentence,
                double &latitude,
                double &longitude,
                double &heading,
                int &status)
{
    // ȥ����ʼ��"$"��ĩβ��У��Σ���"*41"��
    size_t asterisk = sentence.find('*');
    std::string body = sentence.substr(1, (asterisk == std::string::npos ? sentence.size() : asterisk) - 1);

    std::vector<std::string> tokens;
    std::istringstream ss(body);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        tokens.push_back(item);
    }

    // GPNAV һ���� 24+ ���ֶΣ�����Ҫ�� index 23
    if (tokens.size() < 24 || tokens[0] != "GPNAV")
    {
        return false;
    }

    try
    {
        // �ֶ�������ͼƬ��Ӧ��
        // tokens[3]  ƫ���� (heading)
        // tokens[12] γ��
        // tokens[13] ����
        // tokens[23] ����״̬ (1 ����, 2 ����)
        heading = std::stod(tokens[3]);
        latitude = std::stod(tokens[12]);
        longitude = std::stod(tokens[13]);
        status = std::stoi(tokens[23]);
    }
    catch (...)
    {
        return false;
    }

    return true;
}

std::vector<double> readSerialData(int fd)
{
    char buffer[256];
    std::string sentence_acc;
    double latitude = 0.0, longitude = 0.0, heading = 0.0;
    int status = 0;
    std::vector<double> res;

    // ����������ȣ������ڵ��Դ�ӡ��
    std::cout << std::fixed << std::setprecision(8);

    const int maxRetries = 1000;
    int retryCount = 0;

    while (true)
    {
        memset(buffer, 0, sizeof(buffer));
        int bytesRead = read(fd, buffer, sizeof(buffer) - 1);

        if (bytesRead < 0)
        {
            if (errno == EAGAIN && retryCount++ < maxRetries)
            {
                continue;
            }
            std::cerr << "Serial read error: " << strerror(errno) << std::endl;
            break;
        }
        if (bytesRead == 0)
        {
            if (++retryCount >= maxRetries)
            {
                std::cerr << "No data after retries. Exiting.\n";
                break;
            }
            continue;
        }
        retryCount = 0;
        sentence_acc.append(buffer, bytesRead);

        // ���� $GPNAV ��ͷ
        size_t start = sentence_acc.find("$GPNAV");
        if (start == std::string::npos)
        {
            // �ӵ���������Чǰ׺�������ڴ���������
            if (sentence_acc.size() > 1024)
                sentence_acc.erase(0, sentence_acc.size() - 512);
            continue;
        }

        // �ҵ����л�س���Ϊ���Ľ�����־
        size_t end = sentence_acc.find_first_of("\r\n", start);
        if (end == std::string::npos)
        {
            // ���Ļ�û��ȫ��������һ�ζ�
            continue;
        }

        // ��ȡһ��������
        std::string fullSentence = sentence_acc.substr(start, end - start);
        // ɾ���Ѵ�����
        sentence_acc.erase(0, end + 1);

        // ����
        if (parseGPNAV(fullSentence, latitude, longitude, heading, status))
        {
            std::cout << "Lat: " << latitude
                      << ", Lon: " << longitude
                      << ", Head: " << heading
                      << ", Status: " << status
                      << std::endl;

            res.push_back(latitude);
            res.push_back(longitude);
            res.push_back(heading);
            res.push_back(static_cast<double>(status));
            return res;
        }
        else
        {
            std::cerr << "Invalid GPNAV sentence: " << fullSentence << std::endl;
            // �������ʧ�ܣ��ɷ���һ������ֵ�����ѭ��
            res = {0, 0, 0, 0};
            return res;
        }
    }

    // ���˳�ѭ��δ�ɹ������ؿ� vector
    return res;
}
#endif

//**********************

// 0.�Ƕȼ���calculate_delta_angle
double calculate_delta_angle(double angle1, double angle2)
{
    while (angle1 > 360)
    {
        angle1 -= 360;
    }
    while (angle1 < 0)
    {
        angle1 += 360;
    }
    while (angle2 > 360)
    {
        angle2 -= 360;
    }
    while (angle2 < 0)
    {
        angle2 += 360;
    }
    double delta = angle1 - angle2;
    if (delta > 180)
    {
        delta = delta - 360;
    }
    else if (delta < -180)
    {
        delta = 360 + delta;
    }
    return delta;
}
// 3.����Ǽ���
double Azimuth_withGeo(double lat1, double lon1, double lat2, double lon2)
{
    double s12(0), azi1(0), azi2(0);
    const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();
    geod.Inverse(lat1, lon1, lat2, lon2, s12, azi1, azi2);
    // cout << "geodesic lib:" << azi1 << endl;
    // cout << "c++:" << bearing(lat1, lon1, lat2, lon2) << endl;
    return azi1;
}

double calc_Lf(double v)
{
    double k = 1.5, b = 0;
    double Ld = k * v + b;
    if (Ld < 2.5)
    {
        Ld = 2.5;
    }

    // Ld = 2.85; //5kph
    // Ld = 5.7; //10kph 15kph
    return Ld;
}

// 1.������루������յ㾭γ�ȣ�
double Distance_withGeo(double lat1, double lon1, double lat2, double lon2)
{
    double s12(0);
    const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();
    geod.Inverse(lat1, lon1, lat2, lon2, s12);
    return s12;
}

int findWayPoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y)
{
    int x_size = maps_x.size(), y_size = maps_y.size();
    int n = x_size <= y_size ? x_size : y_size;
    std::vector<double> disList(n, 0);
    for (int i = 0; i != n; ++i)
    {
        disList.at(i) = Distance_withGeo(x, y, maps_x.at(i), maps_y.at(i));
    }

    int closest = min_element(disList.begin(), disList.end()) - disList.begin();
    return closest;
}

// 2.���ҵ�ͼ��ص�
// cx, cy -- ·������γ������
int calc_target_index(double lat, double lon, double Ld, vector<double> cx, vector<double> cy, int &Map_ind)
{
    Map_ind = findWayPoint(lat, lon, cx, cy);
    cout << "������±�: " << std::dec << Map_ind << "/" << std::dec << cx.size() << endl;
    double L = 0;
    while ((Ld > L) && ((Map_ind + 1) < cx.size()))
    {
        L += Distance_withGeo(cx[Map_ind + 1], cy[Map_ind + 1], cx[Map_ind], cy[Map_ind]);
        Map_ind += 1;
    }
    return Map_ind;
}

double obtainMapData()
{
    double Ld;
    int ind;
    Ld = calc_Lf(cur_params.vehicle_speed / 3.6);

    // ÿ�ζ����ڵ�ǰλ�ü�������㣬������֮ǰ��Ԥ�������
    Map_ind = findWayPoint(cur_params.latitude, cur_params.longitude, map_latitude_v, map_longitude_v);
    std::cout << "��ǰλ�����������: " << Map_ind << "/" << map_latitude_v.size() << std::endl;

    // ������������Ԥ���
    ind = calc_target_index(cur_params.latitude, cur_params.longitude, Ld, map_latitude_v, map_longitude_v, Map_ind);
    std::cout << "�����Ԥ�������: " << ind << "/" << map_latitude_v.size() << std::endl;

    // �Ƴ���֮ǰԤ����������ȷ��ÿ�ζ�ʹ���¼����Ԥ���
    // if (_target_ind >= ind)
    // {
    //     ind = _target_ind;
    // }

    if (ind < map_latitude_v.size())
    {
        GIS_map.latitude = map_latitude_v[ind];
        GIS_map.longitude = map_longitude_v[ind];
        // GIS_map.heading_angle = map_heading_angle_v[ind];
        std::cout << "ʹ�ü����Ԥ���: (" << GIS_map.latitude << ", " << GIS_map.longitude << ")" << std::endl;
    }
    else
    {
        // ���Ԥ��㳬����ͼ��Χ��ʹ�õ�ͼ�����һ���㣬���������
        GIS_map.latitude = map_latitude_v.back();
        GIS_map.longitude = map_longitude_v.back();
        // GIS_map.heading_angle = map_heading_angle_v.back();
        ind = map_latitude_v.size() - 1;
        std::cout << "���棺Ԥ��㳬����ͼ��Χ��ʹ�����һ����: (" << GIS_map.latitude << ", " << GIS_map.longitude << ")" << std::endl;
    }

    _target_ind = ind;
    return Ld;
}

// vehicle_speed, heading_angle -- ��ǰ�ٶȡ������
// lat,lon -- ��ǰ����
// map_lat,lon -- Ŀ������
// map_heading_angle_v--Ŀ�꺽���
// wheelbase--���
// Ԥ����루Ld��Խ��������Ч����Խƽ����Ԥ�����Խ�̣�����Ч����Խ��ȷ��ͬʱҲ�����һ�����𵴣�
//  GPS2Steer��������ת��ǲ�����
double GPS2Steer(double vehicle_speed, double latitude, double longitude, double heading_angle,
                 double map_latitude_v, double map_longitude_v, double wheelbase, double &Ld, double &alpha)
{
    double steer_value = 0;

    // ���ݳ��ٵ���Ԥ�����
    if (vehicle_speed < 0)
    {
        Ld = 4.5 + 0.78; // �ٶ�С��0ʱ�趨��СԤ�����
    }
    else if (vehicle_speed >= 0 && vehicle_speed < 30)
    {
        Ld = calc_Lf(cur_params.vehicle_speed / 2.8); // ����Ԥ����� Ld
    }
    else if (vehicle_speed >= 30 && vehicle_speed < 45)
    {
        Ld = calc_Lf(cur_params.vehicle_speed / 2.8); // ����Ԥ����� Ld
        Ld -= 9;                                      // ����Ԥ�����
    }
    else if (vehicle_speed >= 45 && vehicle_speed < 60)
    {
        Ld = calc_Lf(cur_params.vehicle_speed / 2.8); // ����Ԥ����� Ld
        Ld -= 10.5;                                   // ����Ԥ�����
    }
    // printf("\nԤ�鳵�� vehicle_speed = %f\n",vehicle_speed);
    // printf("\nԤ����� Ld = %f\n",Ld);

    alpha = calculate_delta_angle(Azimuth_withGeo(latitude, longitude, map_latitude_v, map_longitude_v), heading_angle);

    // printf("\nalpha = %f\n",alpha);

    double delta = atan2(2.0 * wheelbase * sin(alpha * M_PI / 180.0) / Ld, 1.0) * 180.0 / M_PI;

    // ����ת�Ƿ�Χ
    double bound = 33; // ǰ��ת����
    if (delta > bound)
    {
        delta = bound;
    }
    if (delta < -bound)
    {
        delta = -bound;
    }

    steer_value = -delta; // ��Ӧ���������Ҹ�
    // if (steer_value < 0) {
    //     steer_value *= 1.068;
    // }
    // printf("\n steer_value = %f\n",steer_value);
    return steer_value;
}

void *can_receiving_thread(void *arg)
{
    // CAN�����̺߳��� - ���ղ�����CAN��������
    printf("\ncan_receiving_Thread [%lu] is running\n", pthread_self());
    struct ifreq ifr = {0};
    struct sockaddr_can can_addr = {0};
    struct can_frame frame = {0};
    struct CanFrame frame4A2; // ���ڴ洢0x4A2�������ݣ���������״̬��
    struct CanFrame frame441; // ���ڴ洢0x441�������ݣ���λ״̬��
    struct CanFrame frame411; // ���ڴ洢0x411�������ݣ��ƶ�״̬��
    struct CanFrame frame431; // ���ڴ洢0x441�������ݣ�ת��״̬��
    struct CanFrame frame471; // ���ڴ洢0x471�������ݣ����״̬1��
    struct CanFrame frame473; // ���ڴ洢0x473�������ݣ����״̬2��
    struct CanFrame frame451; // ���ڴ洢0x451�������ݣ�����״̬��

    int sockfd = -1;
    int i;
    int ret;

    /* ���׽��� */
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (0 > sockfd)
    {
        perror("CAN socket error");
        return NULL; // ʹ��return������exit���������������˳�
    }

    /* ָ��can0�豸 */
    strcpy(ifr.ifr_name, "can0");
    ioctl(sockfd, SIOCGIFINDEX, &ifr);
    can_addr.can_family = AF_CAN;
    can_addr.can_ifindex = ifr.ifr_ifindex;

    /* ��can0���׽��ֽ��а� */
    ret = bind(sockfd, (struct sockaddr *)&can_addr, sizeof(can_addr));
    if (0 > ret)
    {
        perror("CAN bind error");
        close(sockfd);
        return NULL; // ʹ��return������exit���������������˳�
    }

    /* ���ù��˹��� - ֻ���չ��˺�ı��� */
    struct can_filter rfilter[7]; // ����������10���������������СΪ10
    // �����˹���ֻ����ָ��ID�ı���
    rfilter[0].can_id = 0x4A2;
    rfilter[0].can_mask = 0x7FF;
    rfilter[1].can_id = 0x441;
    rfilter[1].can_mask = 0x7FF;
    rfilter[2].can_id = 0x411;
    rfilter[2].can_mask = 0x7FF;
    rfilter[3].can_id = 0x431;
    rfilter[3].can_mask = 0x7FF;
    rfilter[4].can_id = 0x471;
    rfilter[4].can_mask = 0x7FF;
    rfilter[5].can_id = 0x473;
    rfilter[5].can_mask = 0x7FF;
    rfilter[6].can_id = 0x451;
    rfilter[6].can_mask = 0x7FF;
    // ����setsockopt���ù��˹���
    if (setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0)
    {
        perror("����CAN������ʧ��");
        close(sockfd);
        return NULL;
    }

    /* ��������ѭ�� */
    while (1)
    {
        // ��ȡCAN֡����
        if (0 > read(sockfd, &frame, sizeof(struct can_frame)))
        {
            perror("CAN read error");
            break;
        }

        // printf("\nCAN_receive is running!\n");

        /* У���Ƿ���յ�����֡ */
        if (frame.can_id & CAN_ERR_FLAG)
        {
            printf("Error frame!\n");
            continue; // �޸�Ϊcontinue������break�������򵥸�����֡���˳�ѭ��
        }

        // /* У��֡��ʽ����ӡ */
        // if (frame.can_id & CAN_EFF_FLAG) {
        // 	// ��չ֡
        // 	printf("��չ֡ <0x%08x> ", frame.can_id & CAN_EFF_MASK);
        // } else {
        // 	// ��׼֡
        // 	printf("��׼֡ <0x%03x> ", frame.can_id & CAN_SFF_MASK);
        // }

        /* У��֡���ͣ�����֡����Զ��֡ */
        if (frame.can_id & CAN_RTR_FLAG)
        {
            printf("remote request\n");
            continue;
        }

        /* ��ӡ���ݳ��Ⱥ�֡���� */
        printf("[%d] [%lu] ", frame.can_dlc, frame_count++);

        /* ���沢��ӡ���� */
        for (i = 0; i < frame.can_dlc; i++)
        {
            rev_data[i] = frame.data[i];
            // printf("%02x ", rev_data[i]);
        }
        // printf("\n"); // ��ӻ��У�ʹ���������
        rev_ok = 1;

        // ����CAN ID�ֱ���ͬ�ı���
        uint32_t can_id = frame.can_id & CAN_SFF_MASK; // ��ȡ��׼֡ID

        // ����������IDΪ0x4A2�ı��� - ������̬״̬
        if (can_id == 0x4A2)
        {
            // ��frame���ݸ��Ƶ�frameA��
            memcpy(frame4A2.data, frame.data, sizeof(frame.data));
            // �������������
            revin_4a2 = parseVCU_VehDynStatus(frame4A2.data);
        }
        // ����������IDΪ0x441�ı��� - ����״̬
        else if (can_id == 0x441)
        { // ʹ��else if�����ظ��ж�
            // ��frame���ݸ��Ƶ�frameB��
            memcpy(frame441.data, frame.data, sizeof(frame.data));
            // �������������
            revin_441 = parseVCU_DriveStatus(frame441.data);
        }
        // ����������IDΪ0x411�ı��� - �ƶ�״̬
        else if (can_id == 0x411)
        { // ʹ��else if�����ظ��ж�
            // ��frame���ݸ��Ƶ�frameC��
            memcpy(frame411.data, frame.data, sizeof(frame.data));
            // �������������
            revin_411 = parseVCU_BrakeStatus(frame411.data);
        }
        // ����������IDΪ0x431�ı��� - ת��״̬
        else if (can_id == 0x431)
        { // ʹ��else if�����ظ��ж�
            // ��frame���ݸ��Ƶ�frameD��
            memcpy(frame431.data, frame.data, sizeof(frame.data));
            // �������������
            revin_431 = parseVCU_SteeringStatus(frame431.data);
        }
        // ����������IDΪ0x471�ı��� - ���״̬01
        else if (can_id == 0x471)
        { // ʹ��else if�����ظ��ж�
            // ��frame���ݸ��Ƶ�frameE��
            memcpy(frame471.data, frame.data, sizeof(frame.data));
            // �������������
            revin_471 = parseVCU_BatStatus01(frame471.data);
        }
        // ����������IDΪ0x473�ı��� - ���״̬02
        else if (can_id == 0x473)
        { // ʹ��else if�����ظ��ж�
            // ��frame���ݸ��Ƶ�frameF��
            memcpy(frame473.data, frame.data, sizeof(frame.data));
            // �������������
            revin_473 = parseVCU_BatStatus02(frame473.data);
        }
        // ����������IDΪ0x451�ı��� - ����״̬
        else if (can_id == 0x451)
        { // ʹ��else if�����ظ��ж�
            // ��frame���ݸ��Ƶ�frameG��
            memcpy(frame451.data, frame.data, sizeof(frame.data));
            // �������������
            revin_451 = parseVCU_EmrgStatus(frame451.data);
        }
    }

    // �ر��׽��ֲ��˳�
    close(sockfd);
    printf("CAN�����߳����˳�\n");
    return NULL;
}

// ��ʼ������ CAN RAW �׽���
// int init_can_socket(const char* ifname) {
//     int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//     if (s < 0) { perror("socket"); return -1; }
//     struct ifreq ifr{};
//     std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
//     if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); close(s); return -1; }
//     struct sockaddr_can addr{};
//     addr.can_family = AF_CAN;
//     addr.can_ifindex = ifr.ifr_ifindex;
//     if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); close(s); return -1; }
//     // ������ģʽ����ѡ��
//     int flags = fcntl(s, F_GETFL, 0);
//     fcntl(s, F_SETFL, flags | O_NONBLOCK);
//     return s;
// }

// ���� timerfd�������ļ�������
// int setup_timerfd(int interval_ms) {
//     int fd = timerfd_create(CLOCK_MONOTONIC, 0);
//     if (fd < 0) { perror("timerfd_create"); return -1; }
//     struct itimerspec its{};
//     its.it_interval.tv_sec  = interval_ms / 1000;
//     its.it_interval.tv_nsec = (interval_ms % 1000) * 1000000;
//     its.it_value = its.it_interval;
//     if (timerfd_settime(fd, 0, &its, nullptr) < 0) { perror("timerfd_settime"); close(fd); return -1; }
//     return fd;
// }

/**
 * @brief �������յ���UDP���ݲ����¾��߿�������
 * @param buffer ���յ������ݻ�����
 * @param client_addr �ͻ��˵�ַ��Ϣ��������־��ʾ��
 */
void process_udp_receive(char *buffer, struct sockaddr_in client_addr)
{
    // printf("Received buffer (hex): ");
    // for (size_t i = 0; i < BUFSIZE; ++i) {
    //     printf("%02X ", (unsigned char)buffer[i]);
    // }
    printf("\n");
    // Expected size of the data based on DECISION_IN struct
    const size_t expected_size = sizeof(DECISION_IN);

    // Check if the buffer size is sufficient
    // Note: In a real UDP scenario, you would need the actual received buffer size.
    // Assuming BUFSIZE is the max possible size and the sender sends exactly expected_size bytes.
    // A more robust solution would pass the actual received length to this function.
    // For now, we assume the buffer contains at least expected_size bytes.
    // A proper implementation should handle variable length messages or add a size prefix.
    if (BUFSIZE < expected_size)
    {
        fprintf(stderr, "Error: Received buffer is too small to contain DECISION_IN data.\n");
        return;
    }

    // Manually parse the buffer to avoid potential issues with memcpy and struct packing
    pthread_mutex_lock(&my_mutex);
    char *ptr = buffer;

    // Read bStart (int)
    decision_control_in.bStart = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read brake_bar (float)
    decision_control_in.brake_bar = *(reinterpret_cast<float *>(ptr));
    ptr += sizeof(float);

    // Read breaking_dis (double)
    decision_control_in.breaking_dis = *(reinterpret_cast<double *>(ptr));
    ptr += sizeof(double);

    // Read EPB_park (int)
    decision_control_in.EPB_park = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read gear (int)
    decision_control_in.gear = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read endSpeed (double)
    decision_control_in.endSpeed = *(reinterpret_cast<double *>(ptr));
    ptr += sizeof(double);

    // Read Emgy_brk_En (int)
    decision_control_in.Emgy_brk_En = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read Emgy_brk_ReqRmv (int)
    decision_control_in.Emgy_brk_ReqRmv = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read Emgy_FtCrashRemove (int)
    decision_control_in.Emgy_FtCrashRemove = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read Emgy_RrrCrashRemove (int)
    decision_control_in.Emgy_RrrCrashRemove = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read Emgy_LeftCrashRemove (int)
    decision_control_in.Emgy_LeftCrashRemove = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read Emgy_RightCrashRemove (int)
    decision_control_in.Emgy_RightCrashRemove = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_Hom (int)
    decision_control_in.ADU_Hom = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_BackLamp (int)
    decision_control_in.ADU_BackLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_TurnRLamp (int)
    decision_control_in.ADU_TurnRLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_TurnLLamp (int)
    decision_control_in.ADU_TurnLLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_DblFlashLamp (int)
    decision_control_in.ADU_DblFlashLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_LowBeamLamp (int)
    decision_control_in.ADU_LowBeamLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_WidthLamp (int)
    decision_control_in.ADU_WidthLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_HighBeamLamp (int)
    decision_control_in.ADU_HighBeamLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_FogLamp (int)
    decision_control_in.ADU_FogLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_BrkLamp (int)
    decision_control_in.ADU_BrkLamp = *(reinterpret_cast<int *>(ptr));
    // ptr += sizeof(int); // No need to advance ptr after the last member

    pthread_mutex_unlock(&my_mutex);

    _target_ind = 0;
    udp_monitor = 0;

    // Optional: Print received data for verification
    char clientIpStr[INET_ADDRSTRLEN];
    int clientPort;
    inet_ntop(AF_INET, &client_addr.sin_addr, clientIpStr, sizeof(clientIpStr));
    clientPort = ntohs(client_addr.sin_port);

    std::string timestamp_udp = getCurrentTimestamp();
    cout << "\nʱ�䣺" << timestamp_udp << endl;
    printf("=== decision_control_in (parsed from bytes) ===\n");
    printf("bStart = %d, brake_bar = %.2f, breaking_dis = %.2lf\n",
           decision_control_in.bStart, decision_control_in.brake_bar, decision_control_in.breaking_dis);
    printf("EPB_park = %d, gear = %d, endSpeed = %.2lf\n",
           decision_control_in.EPB_park, decision_control_in.gear, decision_control_in.endSpeed);
    printf("Emgy_brk_En = %d, Emgy_brk_ReqRmv = %d, Emgy_FtCrashRemove = %d\n",
           decision_control_in.Emgy_brk_En, decision_control_in.Emgy_brk_ReqRmv, decision_control_in.Emgy_FtCrashRemove);
    printf("Emgy_RrrCrashRemove = %d, Emgy_LeftCrashRemove = %d, Emgy_RightCrashRemove = %d\n",
           decision_control_in.Emgy_RrrCrashRemove, decision_control_in.Emgy_LeftCrashRemove, decision_control_in.Emgy_RightCrashRemove);
    printf("ADU_Hom = %d, ADU_BackLamp = %d, ADU_TurnRLamp = %d, ADU_TurnLLamp = %d\n",
           decision_control_in.ADU_Hom, decision_control_in.ADU_BackLamp, decision_control_in.ADU_TurnRLamp, decision_control_in.ADU_TurnLLamp);
    printf("ADU_DblFlashLamp = %d, ADU_LowBeamLamp = %d, ADU_WidthLamp = %d, ADU_HighBeamLamp = %d\n",
           decision_control_in.ADU_DblFlashLamp, decision_control_in.ADU_LowBeamLamp, decision_control_in.ADU_WidthLamp, decision_control_in.ADU_HighBeamLamp);
    printf("ADU_FogLamp = %d, ADU_BrkLamp = %d\n",
           decision_control_in.ADU_FogLamp, decision_control_in.ADU_BrkLamp);
    printf("recvfrom %s at PORT %d\n", clientIpStr, clientPort);
}

/**
 * @brief ���Ϳ���������ݵ��ͻ���
 * @param sockfd �׽���������
 * @param client_addr �ͻ��˵�ַ��Ϣ
 * @param buffer ���ݻ�����
 * @param buffer_size ��������С
 * @return ���ͽ�����ɹ����ط��͵��ֽ�����ʧ�ܷ���-1
 */
int send_udp_response(int sockfd, struct sockaddr_in client_addr, char *buffer, size_t buffer_size)
{
    int ret;

    // ���ûظ��˿�
    client_addr.sin_port = htons(8383);

    // ��ջ�����
    memset(buffer, '\0', buffer_size);

    // ����������ṹ�帴�Ƶ�������
    memcpy(buffer, &control_out, sizeof(CONTROL_OUT));

    // ��������
    ret = sendto(sockfd, buffer, sizeof(CONTROL_OUT), 0,
                 (struct sockaddr *)&client_addr, sizeof(client_addr));

    // ���ط��ͽ��
    return ret;
}

/**
 * @brief ����RDC���ݲ���ֵ��rdc_in
 * @param buffer ���յ������ݻ�����
 * @param size ���ݴ�С
 * @return �����Ƿ�ɹ�
 */
bool parseRDCData(const char *buffer, size_t size)
{
    if (size != sizeof(RDC_IN))
    {
        printf("RDC data size mismatch: expected %zu, got %zu\n", sizeof(RDC_IN), size);
        return false;
    }

    try
    {
        // ֱ�ӿ����ṹ������
        memcpy(&rdc_in, buffer, sizeof(RDC_IN));

        printf("RDC data parsed successfully:\n");
        printf("  Timestamp: %lld\n", rdc_in.RDC_timestamp);
        printf("  Brake: %.2f\n", rdc_in.RDC_brake);
        printf("  Throttle: %.2f\n", rdc_in.RDC_throttle);
        printf("  Steering: %.2f\n", rdc_in.RDC_steering_angle);
        printf("  Gear: %d\n", rdc_in.RDC_gear);

        return true;
    }
    catch (...)
    {
        printf("Error parsing RDC data\n");
        return false;
    }
}

/**
 * @brief UDPͨ���̺߳���
 * @param arg �̲߳�����δʹ�ã�
 * @return �̷߳���ֵ
 */
void *udp_thread(void *arg)
{

    printf("\nudp_Thread [%lu] is running\n", pthread_self());

    int sockfd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t client_len;
    int ret;
    char buffer[128];

    struct sockaddr_in server_addr_insert;
    struct sockaddr_in client_addr_insert;
    char buffer_insert[1024];
    MapToControl mapData;

    // ���RDC socket��ر���
    int sockfd_rdc;
    struct sockaddr_in server_addr_rdc;
    struct sockaddr_in client_addr_rdc;
    char buffer_rdc[1024];
    socklen_t client_len_rdc;

    // ����socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        perror("UDP socket creation failed");
        return NULL;
    }

    int sockfd_insert = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_insert < 0)
    {
        perror("UDP socket_insert creation failed");
        return NULL;
    }

    // ����RDC socket (�˿�8002)
    sockfd_rdc = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_rdc < 0)
    {
        perror("UDP socket_rdc creation failed");
        return NULL;
    }

    int maxfd = std::max({sockfd, sockfd_insert, sockfd_rdc}) + 1;
    // �����������ַ
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(8000);

    bzero(&server_addr_insert, sizeof(server_addr));
    server_addr_insert.sin_family = AF_INET;
    server_addr_insert.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr_insert.sin_port = htons(8001);

    // ����RDC��������ַ (�˿�8002)
    bzero(&server_addr_rdc, sizeof(server_addr_rdc));
    server_addr_rdc.sin_family = AF_INET;
    server_addr_rdc.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr_rdc.sin_port = htons(8002);

    // ��socket����ַ
    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("UDP socket bind failed");
        close(sockfd);
        return NULL;
    }
    if (bind(sockfd_insert, (struct sockaddr *)&server_addr_insert, sizeof(server_addr_insert)) < 0)
    {
        perror("UDP socket_insert bind failed");
        close(sockfd_insert);
        return NULL;
    }
    if (bind(sockfd_rdc, (struct sockaddr *)&server_addr_rdc, sizeof(server_addr_rdc)) < 0)
    {
        perror("UDP socket_rdc bind failed");
        close(sockfd_rdc);
        return NULL;
    }
    client_len = sizeof(client_addr);
    auto client_len_insert = sizeof(client_addr_insert);
    client_len_rdc = sizeof(client_addr_rdc);
    // ��ѭ�����������ݲ���Ӧ
    while (1)
    {
        // ��ս��ջ�����
        memset(buffer, '\0', sizeof(buffer));
        memset(buffer_insert, '\0', sizeof(buffer_insert));
        memset(buffer_rdc, '\0', sizeof(buffer_rdc));
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(sockfd, &readfds);
        FD_SET(sockfd_insert, &readfds);
        FD_SET(sockfd_rdc, &readfds);

        int activity = select(maxfd, &readfds, nullptr, nullptr, nullptr);
        if (activity < 0)
        {
            perror("select error");
            break;
        }

        if (FD_ISSET(sockfd, &readfds))
        {
            // ��������
            ret = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                           (struct sockaddr *)&client_addr, &client_len);

            if (ret > 0)
            {
                begin_run = 1;
                // ������յ�������
                process_udp_receive(buffer, client_addr);

                // ������Ӧ����
                ret = send_udp_response(sockfd, client_addr, buffer, sizeof(buffer));

                // 8000�˿����ݽ��ճɹ�������ControlStateΪ1
                ControlState = 1;
                printf("Port 8000 data received successfully, ControlState set to 1\n");
            }
            else
            {
                // ����ʧ�ܴ�����ǰ����¼�����жϣ�
                perror("UDP send failed");
            }
        }
        if (FD_ISSET(sockfd_insert, &readfds))
        {
            ret = recvfrom(sockfd_insert, buffer_insert, sizeof(buffer_insert), 0,
                           (struct sockaddr *)&client_addr_insert, &client_len);
            if (ret > 0)
            {
                begin_run = 1;
                memcpy(&mapData, buffer_insert, sizeof(MapToControl));
                // ׼������insertTargetPoints�����Ĳ���
                std::vector<CPosition> points;
                for (int i = 0; i < mapData.validNumInFrame; i++)
                {
                    points.push_back(mapData.info[i]);
                    // printf("��[%d]: x=%.6f, y=%.6f\n", i, mapData.info[i].x, mapData.info[i].y);
                }
                vector<planRoadToControl> ret1 = insertTargetPoints(points);
                // ��ӡ����ֵ
                for (const auto &road : ret1)
                {
                    std::cout << "UDP_lat: " << road.latitude << std::endl;
                    std::cout << "UDP_lon: " << road.longitude << std::endl;
                    std::cout << "UDP_heading: " << road.headingAngle << std::endl;
                    std::cout << "UDP_status: " << road.status << std::endl;
                }
                // if (mapData.validNumInFrame > 0) {
                //     }
            }
            else
            {
                // ����ʧ�ܴ�����ǰ����¼�����жϣ�
                perror("UDP send failed");
            }
        }

        // ����8002�˿ڵ�RDC����
        if (FD_ISSET(sockfd_rdc, &readfds))
        {
            ret = recvfrom(sockfd_rdc, buffer_rdc, sizeof(buffer_rdc), 0,
                           (struct sockaddr *)&client_addr_rdc, &client_len_rdc);
            if (ret > 0)
            {
                begin_run = 1;
                printf("Received RDC data from port 8002, size: %d bytes\n", ret);

                // ����RDC����
                if (parseRDCData(buffer_rdc, ret))
                {
                    // RDC���ݽ����ɹ�������ControlStateΪ2
                    ControlState = 2;
                    printf("Port 8002 RDC data received and parsed successfully, ControlState set to 2\n");
                }
                else
                {
                    printf("Failed to parse RDC data from port 8002\n");
                }
            }
            else
            {
                // ����ʧ�ܴ���
                perror("UDP RDC receive failed");
            }
        }

        std::cout << "UDP" << std::endl;
    }

    // �ر�socket��ʵ������Զ����ִ�е��������whileѭ������ϣ�
    close(sockfd);
    return NULL;
}

/*
date:20241224
author:rxl
���ӽ�����ͳ�ָ��ĳ���
*/

// ��װ ����CAN ���ģ����߽������������
CanFrame createCanFrame(int handshake, int gear, int park, double brake, double angle, double steer_speed, double vehicle_speed)
{
    CanFrame frame = {};

    // Byte 0: ���֣�0Ϊ�����֡�5Ϊ���֣�����λ��1N��2R��3D����פ����1�ͷţ�2פ����
    frame.data[0] = (handshake & 0b00000111) | ((gear & 0b00000111) << 3) | ((park & 0b00000011) << 6);

    // Byte 1-2: ɲ�����ٶȿ��ƣ���8λ��ǰ����8λ�ں� brake ���Ϊ10
    int16_t brake_raw = static_cast<int16_t>((15 - brake) * 2048);
    frame.data[1] = brake_raw & 0xFF;
    frame.data[2] = (brake_raw >> 8) & 0xFF;

    // Byte 3-4: ������ת�ǣ���8λ��ǰ����8λ�ں�������
    int16_t angle_raw = static_cast<int16_t>((1575 + angle));
    frame.data[3] = angle_raw & 0xFF;
    frame.data[4] = (angle_raw >> 8) & 0xFF;

    // Byte 5: �����̽��ٶ�
    frame.data[5] = static_cast<uint8_t>(steer_speed / 10);

    // Byte 6-7: ���٣���4λ�� byte7����8λ�� byte6��
    uint16_t vehicle_speed_raw = static_cast<uint16_t>(vehicle_speed / 0.05);
    frame.data[6] = vehicle_speed_raw & 0xFF;        // ��8λ
    frame.data[7] = (vehicle_speed_raw >> 8) & 0x0F; // ��4λ

    // ��ӡ��������
    std::cout << "Created CAN Frame: [ ";
    for (int i = 0; i < 8; ++i)
    {
        std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
    }
    std::cout << "]" << std::endl;

    return frame;
}

void sendCanFrame(const CanFrame &frame)
{
    // ģ�ⷢ�� CAN ����
    std::cout << "Sending CAN  : [ ";
    for (int i = 0; i < 8; ++i)
    {
        std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
    }
    std::cout << "]" << std::endl;
}

// ���ߣ����ٶ�ֵת��Ϊ 0.05 �ı���������ӡת������
double convert_and_print_speed(double &speed)
{ // ʹ�����ô��ݣ�����ֱ���޸�speed��ֵ
    // std::cout << "ԭʼ�ٶȣ�" << speed;
    double multiple = speed / 0.05;
    double rounded_multiple = std::round(multiple);
    speed = rounded_multiple * 0.05;
    return speed;
}
// ���ߣ����ٶ�
double convert_and_print_brake_bar(double &brake_bar)
{
    // std::cout << "ԭʼ���ٶȣ�" << brake_bar;

    // �� 0.00048828125 ��Ϊ����
    double multiple = brake_bar / 0.01;             // ��������ٸ�0.01
    double rounded_multiple = std::round(multiple); // ��������
    brake_bar = rounded_multiple * 0.01;            // ����ת����ļ��ٶ�

    // std::cout << std::fixed << std::setprecision(4); // �����������
    // std::cout << "ת����ļ��ٶ�Ϊ��" << brake_bar << std::endl;

    return brake_bar;
}
// ���ߣ���ȫ8λʮ�����Ʊ���
std::string formatCanData(const CanFrame &frame)
{
    std::ostringstream oss;
    for (int i = 0; i < 8; ++i)
    { // ���� CAN ����֡����Ϊ 8
        oss << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(frame.data[i]) << " ";
    }
    return oss.str();
}

// �������У��ͺ�����XOR��
uint8_t calculateChecksum(uint8_t *data, size_t length)
{
    uint8_t xor_sum = 0;
    for (size_t i = 0; i < length; i++)
    {
        xor_sum ^= data[i];
    }
    return xor_sum;
}

/*
brake_bar����Ӧ��ʽ���ֵ��ʵ��ɲ��ѹ��0-100����ʽ��800(����)*0.01�����ȣ�=80 bar�������
EPB_Park��פ��״̬ 0�ͷ� 1����;��Ҫ���������ͷ�
steer_angle����Ӧ��ʽ���ֵ��������ת�ǣ������Ҹ������ֵ540deg��Ӧ�������33�㣬��ʽ��10800�����룩*0.1�����ȣ�=540 deg�������
steer_speed����Ӧ��ʽ���ֵ�������̽��ٶȣ����520 deg/s����ʽ��1000�����룩*0.2�����ȣ�=200 deg/s�������
endspeed����Ӧ��ʽ���ֵ�����40 km/h����ʽ��800�����룩*0.05�����ȣ�=40 km/h�������
Emgy_brk_En����ɲʹ�ܣ�0�Ƴ���1��ɲ
Emgy_brk_Ft��ǰ�����Ƴ���0���Ƴ���1�Ƴ�
Emgy_brk_Rr���󴥱��Ƴ���0���Ƴ���1�Ƴ�
Emgy_brk_Left���󴥱��Ƴ���0���Ƴ���1�Ƴ�
Emgy_brk_Right���Ҵ����Ƴ���0���Ƴ���1�Ƴ�
ADU_Hom �����ȿ���
ADU_BackLamp�������ƿ���
ADU_TurnRLamp����ת�ƿ���
ADU_TurnLLamp����ת�ƿ���
ADU_DblFlashLamp��˫���ƿ���
ADU_LowBeamLamp������ƿ���
ADU_HighBeamLamp��Զ��ƿ���
ADU_WidthLamp��ʾ��ƿ���
ADU_FogLamp����ƿ���
ADU_BrkLamp���ƶ��ƿ���
ADU_DoorUnlcking�����Ž���
ADU_DoorLocking����������
ADU_TricolourRedLamp����ɫ�ƿ���-��
ADU_TricolourYellowLamp����ɫ�ƿ���-��
ADU_TricolourGreenLamp����ɫ�ƿ���-��
ADU_FANRunCtrl�����ȿ���

Ĭ�ϲ�����
brake_bar��0    ��ɲ��ѹ��
EPB_Park��0     �ͷ�
steer_angle��0  ������0
steer_speed��200    200 deg/s
endspeed��0     ���ٶ�
gear��5         N��
Emgy_brk_En��0  �޼�ɲ
Emgy_brk_Ft��0  ��ǰ�����Ƴ�
Emgy_brk_Rr��0  �޺󴥱��Ƴ�
Emgy_brk_Left��0  ���󴥱��Ƴ�
Emgy_brk_Right��0  ���Ҵ����Ƴ�
ADU_Hom ��0  ���ȹر�
ADU_BackLamp��0  �����ƹر�
ADU_TurnRLamp��0  ��ת�ƹر�
ADU_TurnLLamp��0  ��ת�ƹر�
ADU_DblFlashLamp��0  ˫���ƹر�
ADU_LowBeamLamp��0  ����ƹر�
ADU_HighBeamLamp��0  Զ��ƹر�
ADU_WidthLamp��0  ʾ��ƹر�
ADU_FogLamp��0  ��ƹر�
ADU_BrkLamp��0  �ƶ��ƹر�
ADU_DoorUnlcking��0  ���Ž���
ADU_DoorLocking��0  ��������
ADU_TricolourRedLamp��0  ��ɫ�ƿ���-��
ADU_TricolourYellowLamp��0  ��ɫ�ƿ���-��
ADU_TricolourGreenLamp��0  ��ɫ�ƿ���-��
ADU_FANRunCtrl��0  ���ȿ���
*/

void createEightCanFrames(std::vector<can_frame> &eightFrames, int step,
                          float brake_bar = 0,
                          int EPB_Park = 1,
                          double steer_angle = 0,
                          double steer_speed = 200,
                          double endspeed = 0,
                          int gear = 5,
                          double PedalposReq = 0, // 0-100�������� Զ��ʹ��
                          int Emgy_brk_En = 0,
                          int Emgy_brk_ReqRmv = 0,
                          int Emgy_brk_Ft = 0,
                          int Emgy_brk_Rr = 0,
                          int Emgy_brk_Left = 0,
                          int Emgy_brk_Right = 0,
                          int ADU_Hom = 0,
                          int ADU_BackLamp = 0,
                          int ADU_TurnRLamp = 0,
                          int ADU_TurnLLamp = 0,
                          int ADU_DblFlashLamp = 0,
                          int ADU_LowBeamLamp = 0,
                          int ADU_WidthLamp = 0,
                          int ADU_HighBeamLamp = 0,
                          int ADU_FogLamp = 0,
                          int ADU_BrkLamp = 0,
                          int ADU_DoorUnlcking = 0,
                          int ADU_DoorLocking = 0,
                          int ADU_TricolourRedLamp = 0,
                          int ADU_TricolourYellowLamp = 0,
                          int ADU_TricolourGreenLamp = 0,
                          int ADU_FANRunCtrl = 0)
{
    std::lock_guard<std::mutex> lock(framesMutex);
    eightFrames.clear();

    static uint8_t frameCounter = 0;

    auto push_frame = [&](uint32_t id, const std::array<uint8_t, 8> &data)
    {
        can_frame frame{};
        frame.can_id = id;
        frame.can_dlc = 8;
        std::copy(data.begin(), data.end(), frame.data);
        eightFrames.push_back(frame);
    };

    // 210
    std::array<uint8_t, 8> data210{};
    if (step == 0)
    {
        data210[0] = 0x80;
    }
    else if (step == 1 || step == 2)
    {
        data210[0] = 0xA0;
        data210[2] = 0x07;
        data210[3] = 0xD0;
    }
    else if (step == 3)
    {
        data210[0] = 0x80;
    }
    else
    {
        data210[0] = 0xA0;
        int16_t brake_barValue = static_cast<int16_t>(brake_bar / 0.01);
        data210[2] = (brake_barValue >> 8) & 0xFF;
        data210[3] = brake_barValue & 0xFF;
    }
    data210[6] = frameCounter & 0x0F;
    data210[7] = calculateChecksum(data210.data(), 7);
    push_frame(0x210, data210);

    // 220
    std::array<uint8_t, 8> data220{};
    data220[0] = 0x80 | ((EPB_Park & 0b01) << 6);
    data220[6] = frameCounter & 0x0F;
    data220[7] = calculateChecksum(data220.data(), 7);
    push_frame(0x220, data220);

    // 230
    std::array<uint8_t, 8> data230{};
    if (step == 0)
    {
        data230[0] = 0x80;
        data230[6] = frameCounter & 0x0F;
    }
    else
    {
        data230[0] = 0xA0;
        int16_t angleValue = static_cast<int16_t>(steer_angle / 0.05);
        data230[1] = (angleValue >> 8) & 0xFF;
        data230[2] = angleValue & 0xFF;
        int16_t steer_speedValue = static_cast<int16_t>(steer_speed / 0.2);
        data230[5] = (steer_speedValue >> 4) & 0xFF;
        data230[6] = ((steer_speedValue & 0x0F) << 4) | (frameCounter & 0x0F);
    }
    data230[7] = calculateChecksum(data230.data(), 7);
    push_frame(0x230, data230);

    // 240
    std::array<uint8_t, 8> data240{};
    if (step == 0 || step == 1 || step == 3)
    {
        data240[0] = 0x80;
        data240[1] = 0x07;
        data240[2] = 0xD0;
        data240[3] = 0x27;
        data240[4] = 0x10;
    }
    else if (ControlState == 1) // ����ģʽ���ٶȻ�����
    {
        data240[0] = 0xA0 | (gear & 0x0F);
        double clamped_speed = std::clamp(endspeed, -100.0, 104.75);
        uint16_t speed_raw = static_cast<uint16_t>((clamped_speed + 100) / 0.05);
        speed_raw = std::clamp(speed_raw, static_cast<uint16_t>(0), static_cast<uint16_t>(4095));
        data240[1] = (speed_raw >> 8) & 0x0F;
        data240[2] = speed_raw & 0xFF;
        data240[3] = 0x27;
        data240[4] = 0x10;
    }
    else if (ControlState == 2) // Զ��ģʽ������̤�����
    {
        data240[0] = 0xE0 | (gear & 0x0F);

        // ����̤��������� (��5�ֽڣ�40bitλ��ռ8bit)
        uint8_t pedal_raw = static_cast<uint8_t>(PedalposReq / 0.5); // ת���ٷֱ�
        data240[5] = pedal_raw;
    }
    data240[6] = frameCounter & 0x0F;
    data240[7] = calculateChecksum(data240.data(), 7);
    push_frame(0x240, data240);

    // 251
    std::array<uint8_t, 8> data251{};
    data251[0] = ((Emgy_brk_Right & 1) << 5) | ((Emgy_brk_Left & 1) << 4) | ((Emgy_brk_Rr & 1) << 3) |
                 ((Emgy_brk_Ft & 1) << 2) | ((Emgy_brk_ReqRmv & 1) << 1) | (Emgy_brk_En & 1);
    data251[6] = frameCounter & 0x0F;
    data251[7] = calculateChecksum(data251.data(), 7);
    push_frame(0x251, data251);

    // 260
    std::array<uint8_t, 8> data260{};
    data260[1] = ((ADU_DblFlashLamp & 1) << 6) | ((ADU_TurnLLamp & 1) << 5) | ((ADU_TurnRLamp & 1) << 4) |
                 ((ADU_BackLamp & 1) << 3) | ((ADU_Hom & 1) << 1);
    data260[2] = ((ADU_BrkLamp & 1) << 7) | ((ADU_FogLamp & 1) << 6) | ((ADU_WidthLamp & 1) << 5) |
                 ((ADU_HighBeamLamp & 1) << 4) | ((ADU_LowBeamLamp & 1) << 3);
    data260[6] = frameCounter & 0x0F;
    data260[7] = calculateChecksum(data260.data(), 7);
    push_frame(0x260, data260);

    // 262
    std::array<uint8_t, 8> data262{};
    data262[6] = frameCounter & 0x0F;
    data262[7] = calculateChecksum(data262.data(), 7);
    push_frame(0x262, data262);

    // 272
    std::array<uint8_t, 8> data272{};
    data272[6] = frameCounter & 0x0F;
    data272[7] = calculateChecksum(data272.data(), 7);
    push_frame(0x272, data272);

    frameCounter = (frameCounter + 1) & 0x0F;
}

/*
program������can���Ŀ��Ƴ���
steer_angle:ѭ�������ķ�����ת��
CONTROL_OUT c_out���������
DECISION_IN &din����������
recvData revin������310�õ��Ľṹ�壬������ǰ���ٵ���Ϣ
unsigned char *orders������ָ��
int &canFrameCount��can��ʼ���Լ����ƹ��ܼ�����
int Map_ind��Ԥ����ͼ�±�
*/
void send_candata(double steer_angle, CONTROL_OUT &c_out, DECISION_IN &din, VCU_VehDynStatus revin_4a2, VCU_DriveStatus revin_441, int &canFrameCount)
{
    // ��ǰ�������
    static int currentStep = 0;

    // Step Emy: ��ͣ����֡ �ж��Ƿ���Ҫ��ͣ
    if (din.Emgy_brk_En == 1 || begin_run == 0) // ��ͣ
    {
        std::cout << "Step Emy: ��ͣ" << ",begin_run=" << begin_run << std::endl;
        // �����������Ʊ���
        createEightCanFrames(eightFrames, 10, 100, 0, 0, 200, 0, 5, 1, 0, 0, 0, 0, 0, 1);
        canFrameCount = -1;
        currentStep = -1;
    }
    else if (din.Emgy_brk_ReqRmv == 1) // �Ƴ���ͣ
    {
        std::cout << "Step Emy����Rmv: �Ƴ���ͣ" << std::endl;
        // �Ƴ���ͣ����
        createEightCanFrames(eightFrames, 10, 100, 0, 0, 200, 0, 5, 0, 1, 1, 1, 1, 1);
        canFrameCount = 0;
        currentStep = 0;
    }
    // Step 0: ��ʼ������
    if (canFrameCount >= 0 && canFrameCount < 30 && begin_run == 1)
    {
        std::cout << "Step 0: ��ʼ�����ֽ׶�" << std::endl;
        // �������ֱ��ģ�210��220��230��240��Ҫ���֣�
        createEightCanFrames(eightFrames, 0);
        ++canFrameCount;
        currentStep = 0;
    }
    // Step 1: ��ѹ�׶�
    else if ((currentStep == 0 || currentStep == 1) && canFrameCount < 40)
    {
        std::cout << "Step 1: ��ѹ�׶�" << std::endl;
        createEightCanFrames(eightFrames, 1);
        ++canFrameCount;
        currentStep = 1;
    }
    // Step 2: �����׶�
    else if ((currentStep == 1 || currentStep == 2) && canFrameCount < 60)
    {
        std::cout << "Step 2: �����׶�" << std::endl;
        createEightCanFrames(eightFrames, 2);
        ++canFrameCount;
        currentStep = 2;
    }
    // Step 3: �����ѹ�׶�
    else if ((currentStep == 2 || currentStep == 3) && canFrameCount < 80)
    {
        std::cout << "Step 3: �����ѹ�׶�" << std::endl;
        createEightCanFrames(eightFrames, 3);
        ++canFrameCount;
        currentStep = 3;
    }
    // Step 4: ʵ�ʿ���֡
    else if (currentStep >= 3 && canFrameCount >= 80 && begin_run == 1)
    {
        std::cout << "Step 4: ���Ʊ��ٽ׶�" << std::endl;
        static double ctrl_speed = 0;       // ʵ���·�ĩ�ٶ�
        static bool flag = false;           // ����false������true
        static int gear_count = 0;          // ��λ������
        static float tmp_din_brake_bar = 0; // ָ��ɲ��ѹ��

        /* test ģ�����*/
        // if (canFrameCount > 130)
        // {
        //     decision_control_in.endSpeed=0;
        //     decision_control_in.brake_bar=100;
        //     decision_control_in.gear=1;
        //     decision_control_in.Emgy_brk_En=0;
        //     decision_control_in.ADU_DblFlashLamp=1;
        // }

        /* ָ������ж� */
        // 1. ɲ���Լ����ų�ͻ
        if (din.endSpeed != 0 && din.brake_bar != 0) // ͬʱ����ɲ��������ָ���ͻ����
        {
            cout << "ɲ��������ָ���ͻ���޷��·�" << endl;
            c_out.controlOut_ErrorCode = 1;
            return;
        }
        // 2. פ�����Լ����ٳ�ͻ
        if (din.gear == 1 && c_out.vehicle_speed != 0) // ͬʱ����ɲ��������ָ���ͻ����
        {
            cout << "פ�����Լ����ٳ�ͻ���޷��·�" << endl;
            c_out.controlOut_ErrorCode = 2;
            return;
        }
        //========================== 1.���жϵ�λ ==========================*/
        if (din.gear != revin_441.Drv_GearAct && din.brake_bar == 0)
        {
            // 0.�洢ԭ��ָ��ѹ��
            tmp_din_brake_bar = din.brake_bar;
            // 1.�ٽ�ѹ��ֵ
            din.brake_bar = 50;
            cout << "----��λ�л�:din.gear=" << din.gear << ",revin_441.Drv_GearAct=" << revin_441.Drv_GearAct << "----" << endl;
            gear_count += 1;
        }
        else if ((tmp_din_brake_bar != din.brake_bar) && (gear_count != 0))
        {
            // 2. �����ѹ��ֵ ��ԭԭ��ָ��0ѹ��
            din.brake_bar = 0;
            gear_count = 0;
        }
        //========================== 2.�ٿ����ٶ� ==========================*/

        // ��;ͣ��������¸�ֵ - �Ż������ж�
        if (fabs(ctrl_speed - c_out.vehicle_speed) >= 6 && ctrl_speed > c_out.vehicle_speed)
        {
            ctrl_speed = c_out.vehicle_speed;
            std::cout << "��⵽�����쳣�½������ÿ����ٶ�Ϊ: " << ctrl_speed << std::endl;
        }

        // ����Ŀ���ٶȺ͵�ǰ�ٶȹ�ϵ��ȷ���Ӽ���ģʽ
        if (din.endSpeed >= c_out.vehicle_speed) // ��Ҫ����
        {
            flag = false;
            std::cout << "Ŀ���ٶȴ��ڵ�ǰ�ٶȣ��������ģʽ" << std::endl;
        }
        else if (din.endSpeed < c_out.vehicle_speed) // ��Ҫ����
        {
            flag = true;
            std::cout << "Ŀ���ٶ�С�ڵ�ǰ�ٶȣ��������ģʽ" << std::endl;
        }
        // ���ٹ���
        if (!flag) // ����ģʽ
        {
            if (c_out.vehicle_speed <= din.endSpeed)
            {
                // ����Ӧ���٣����ݵ�ǰ�ٶȺ�Ŀ���ٶȵĲ�ֵ����������
                double speed_diff = din.endSpeed - ctrl_speed;
                double speed_unit;

                if (speed_diff > 10) // ���ϴ�ʱ��ʹ�ýϴ���ٶ�
                {
                    speed_unit = 0.05 * 4; // 4����С��λ
                }
                else if (speed_diff > 5) // �еȲ��
                {
                    speed_unit = 0.05 * 2; // 2����С��λ
                }
                else // �ӽ�Ŀ���ٶ�
                {
                    speed_unit = 0.05; // 1����С��λ��ƽ������
                }

                ctrl_speed += speed_unit;

                // ȷ��������ٹ���
                if (ctrl_speed > din.endSpeed)
                {
                    ctrl_speed = din.endSpeed;
                }

                std::cout << "����Ӧ����: " << speed_unit << " ��λ/����, Ŀ���ٶ�: "
                          << din.endSpeed << ", ��ǰ�����ٶ�: " << ctrl_speed << std::endl;
            }
            else
            {
                ctrl_speed = din.endSpeed; // �ﵽĿ���ٶȻ����С�����ֲ���
                std::cout << "�ѴﵽĿ���ٶ�: " << din.endSpeed << ",�·��ٶ�ctrl:" << ctrl_speed << std::endl;
            }
        }
        // ���ٹ���
        else // ����ģʽ
        {
            // ������Ҫ�ļ��ٶ�
            double speed_diff = c_out.vehicle_speed - din.endSpeed;

            if (speed_diff > 8) // ��Ҫ�ϴ����
            {
                // PID����ɲ��ѹ����ʹ��ǰ��������
                din.brake_bar = static_cast<float>(calculateBrakePressure(revin_4a2.vehicle_speed, din.endSpeed));
                std::cout << "������� - PID����ɲ��ѹ��: " << din.brake_bar << std::endl;
                if (ctrl_speed < din.endSpeed)
                {
                    ctrl_speed = din.endSpeed;
                }
            }
            else if (speed_diff > 3) // �еȼ���
            {
                // �е�ɲ��ѹ��
                din.brake_bar = static_cast<float>(calculateBrakePressure(revin_4a2.vehicle_speed, din.endSpeed) * 0.7);
                std::cout << "�еȼ��� - PID����ɲ��ѹ��: " << din.brake_bar << std::endl;

                if (ctrl_speed < din.endSpeed)
                {
                    ctrl_speed = din.endSpeed;
                }
            }
            else // ΢������
            {
                // ��΢ɲ��ѹ��
                din.brake_bar = static_cast<float>(calculateBrakePressure(revin_4a2.vehicle_speed, din.endSpeed) * 0.5);
                std::cout << "΢������ - PID����ɲ��ѹ��: " << din.brake_bar << std::endl;

                // ����������Ŀ���ٶ�
                ctrl_speed = din.endSpeed;
            }

            // ������Ч��
            if (revin_4a2.vehicle_speed <= din.endSpeed + 2)
            {
                // �Ѿ��ӽ�Ŀ���ٶȣ�ȡ��ɲ��
                din.brake_bar = 0;
                ctrl_speed = din.endSpeed;
                std::cout << "������ɣ�����Ŀ���ٶ�: " << din.endSpeed << std::endl;
            }
        }
        // ȷ�������ٶ���0.05��������
        ctrl_speed = convert_and_print_speed(ctrl_speed);
        // ȷ������ѹ����0.01��������
        double tmp_brake_bar = din.brake_bar;
        din.brake_bar = convert_and_print_brake_bar(tmp_brake_bar);

        // ȷ�����в�������Ч��Χ��
        if (ctrl_speed < 0)
            ctrl_speed = 0;
        if (din.brake_bar < 0)
            din.brake_bar = 0;

        // ʹ��memset���eightFrames���飬����ʹ��δ��ʼ�����ڴ�
        // memset(eightFrames, 0, sizeof(unsigned char) * 8 * 8);
        eightFrames.clear();

        createEightCanFrames(eightFrames, 4, din.brake_bar, din.EPB_park, steer_angle, 200, ctrl_speed, din.gear,
                             din.Emgy_brk_En, din.Emgy_brk_ReqRmv, din.Emgy_FtCrashRemove, din.Emgy_RrrCrashRemove, din.Emgy_LeftCrashRemove, din.Emgy_RightCrashRemove,
                             din.ADU_Hom, din.ADU_BackLamp, din.ADU_TurnRLamp, din.ADU_TurnLLamp, din.ADU_DblFlashLamp, din.ADU_LowBeamLamp, din.ADU_WidthLamp, din.ADU_HighBeamLamp, din.ADU_FogLamp, din.ADU_BrkLamp);
        ++canFrameCount;
    }
    // Step 5: ����֡
    else if (currentStep >= 4 && canFrameCount > 0 && c_out.vehicle_speed == 0 && din.EPB_park == 1 && din.endSpeed == 0)
    {
        std::cout << "Step 5: ����֡" << std::endl;
        currentStep = -2;

        // �ڵ���createEightCanFramesǰ����ڴ�
        // memset(eightFrames, 0, sizeof(unsigned char) * 8 * 8);
        eightFrames.clear();

        createEightCanFrames(eightFrames, 1); // ��ѹ

        // �ٴ�����ڴ棬����ʹ�ÿ����ѱ��ͷŵ��ڴ�
        // memset(eightFrames, 0, sizeof(unsigned char) * 8 * 8);
        eightFrames.clear();

        createEightCanFrames(eightFrames, 5, 20, 1, 0, 50, 0, 1); // ��P��

        canFrameCount = -2;
    }
    else if (currentStep < 0)
    {
        if (currentStep == -1)
        {
            std::cout << "��ͣδ�Ƴ�" << std::endl;
        }
        if (currentStep == -2)
        {
            std::cout << "����״̬������" << std::endl;
        }
    }
    else
    {
        std::cout << "δ֪״̬��currentStep=" << currentStep
                  << ", canFrameCount=" << canFrameCount
                  << ", begin_run=" << begin_run
                  << ", vehicle_speed=" << revin_4a2.vehicle_speed
                  << ", EPB_park=" << din.EPB_park
                  << ", endSpeed=" << din.endSpeed
                  << std::endl;
    }
}

/*
program������can���Ŀ��Ƴ�����RDC���ذ汾��
steer_angle:ѭ�������ķ�����ת��
CONTROL_OUT c_out���������
RDC_IN rdc_in��Զ�̼�ʻ��������
recvData revin������310�õ��Ľṹ�壬������ǰ���ٵ���Ϣ
unsigned char *orders������ָ��
int &canFrameCount��can��ʼ���Լ����ƹ��ܼ�����
*/
void send_candata(RDC_IN rdc_in, CONTROL_OUT &c_out, VCU_VehDynStatus revin_4a2, VCU_DriveStatus revin_441, int &canFrameCount)
{
    // ��ǰ�������
    static int currentStep = 0;

    printf("RDC����ģʽ - ɲ��: %.2f, ����: %.2f, ת��: %.2f, ��λ: %d\n",
           rdc_in.RDC_brake, rdc_in.RDC_throttle, rdc_in.RDC_steering_angle, rdc_in.RDC_gear);

    // Step Emy: ��ͣ����֡ �ж��Ƿ���Ҫ��ͣ
    if (rdc_in.RDC_Emgy_brk_En == 1 || begin_run == 0) // ��ͣ
    {
        std::cout << "Step Emy: ��ͣ" << ",begin_run=" << begin_run << std::endl;
        // �����������Ʊ���
        createEightCanFrames(eightFrames, 10, 100, 0, 0, 200, 0, 5, 1, 0, 0, 0, 0, 0, 1);
        canFrameCount = -1;
        currentStep = -1;
    }
    else if (rdc_in.RDC_Emgy_brk_ReqRmv == 1) // �Ƴ���ͣ
    {
        std::cout << "Step Emy����Rmv: �Ƴ���ͣ" << std::endl;
        // �Ƴ���ͣ����
        createEightCanFrames(eightFrames, 10, 100, 0, 0, 200, 0, 5, 0, 1, 1, 1, 1, 1);
        canFrameCount = 0;
        currentStep = 0;
    }
    // Step 0: ��ʼ������
    if (canFrameCount >= 0 && canFrameCount < 30 && begin_run == 1)
    {
        std::cout << "Step 0: ��ʼ�����ֽ׶�" << std::endl;
        // �������ֱ��ģ�210��220��230��240��Ҫ���֣�
        createEightCanFrames(eightFrames, 0);
        ++canFrameCount;
        currentStep = 1;
    }
    // Step 1: RDCֱ�ӿ���ģʽ��������ѹ�������������ѹ�׶Σ�
    else if (currentStep > 0 && canFrameCount >= 30 && begin_run == 1)
    {
        std::cout << "Step 4: RDCֱ�ӿ���ģʽ" << std::endl;

        // RDCģʽ��ֱ��ʹ��Զ��ָ��������ſ��϶ȿ���

        float current_throttle = rdc_in.RDC_throttle;
        float current_brake = rdc_in.RDC_brake;

        // 1. ɲ���Լ����ų�ͻ���
        if (current_throttle != 0 && current_brake != 0)
        {
            cout << "RDCģʽ��ɲ��������ָ���ͻ������ִ��ɲ��" << endl;
            current_throttle = 0; // �������ָ��
            c_out.controlOut_ErrorCode = 1;
        }

        // 2. פ�����Լ����ٳ�ͻ���
        if (rdc_in.RDC_gear == 1 && c_out.vehicle_speed != 0)
        {
            cout << "RDCģʽ��פ�����Լ����ٳ�ͻ��ǿ��ɲ��" << endl;
            current_brake = 50; // ǿ��ɲ��
            c_out.controlOut_ErrorCode = 2;
        }

        // ȷ�����Ʋ�������Ч��Χ��
        if (current_throttle < 0)
            current_throttle = 0;
        if (current_throttle > 100)
            current_throttle = 100;
        if (current_brake < 0)
            current_brake = 0;
        if (current_brake > 100)
            current_brake = 100;

        // ת��ɲ��ѹ��Ϊ0.01��������
        double tmp_brake_bar = current_brake;
        current_brake = convert_and_print_brake_bar(tmp_brake_bar);

        // ��ղ�����CAN֡
        eightFrames.clear();

        // ֱ��ʹ��RDC�ṹ���Ա���п���
        createEightCanFrames(eightFrames, 4, current_brake, rdc_in.RDC_Park, rdc_in.RDC_steering_angle, 200, 0, rdc_in.RDC_gear, current_throttle,
                             rdc_in.RDC_Emgy_brk_En, rdc_in.RDC_Emgy_brk_ReqRmv, 0, 0, 0, 0, rdc_in.RDC_ADU_Hom, rdc_in.RDC_ADU_BackLamp,
                             rdc_in.RDC_ADU_TurnRLamp, rdc_in.RDC_ADU_TurnLLamp, rdc_in.RDC_ADU_DblFlashLamp,
                             rdc_in.RDC_ADU_LowBeamLamp, rdc_in.RDC_ADU_WidthLamp, rdc_in.RDC_ADU_HighBeamLamp, rdc_in.RDC_ADU_FogLamp);

        ++canFrameCount;
    }
    // Step 5: ����֡
    else if (currentStep >= 4 && canFrameCount > 0 && c_out.vehicle_speed == 0 && rdc_in.RDC_Park == 1)
    {
        std::cout << "Step 5: RDC����֡" << std::endl;
        currentStep = -2;

        eightFrames.clear();
        createEightCanFrames(eightFrames, 1); // ��ѹ

        eightFrames.clear();
        createEightCanFrames(eightFrames, 5, 20, 1, 0, 50, 0, 1); // ��P��

        canFrameCount = -2;
    }
    else if (currentStep < 0)
    {
        if (currentStep == -1)
        {
            std::cout << "RDCģʽ����ͣδ�Ƴ�" << std::endl;
        }
        if (currentStep == -2)
        {
            std::cout << "RDCģʽ������״̬������" << std::endl;
        }
    }
    else
    {
        std::cout << "RDCģʽδ֪״̬��currentStep=" << currentStep
                  << ", canFrameCount=" << canFrameCount
                  << ", begin_run=" << begin_run
                  << ", vehicle_speed=" << revin_4a2.vehicle_speed
                  << ", EPB_park=" << rdc_in.RDC_Park
                  << std::endl;
    }
}

// ��ʼ����־�ļ�
void initLogFile()
{
    // ��ȡ��ǰʱ����������ļ���
    std::string timestamp = getCurrentTimestamp();
    std::string logFilePath = "/home/ztl/log/log_" + timestamp.substr(0, 19) + ".txt";

    // ����Ѿ��д򿪵��ļ����ȹر�
    if (logFile.is_open())
    {
        logFile.close();
    }

    // �����ļ�
    logFile.open(logFilePath, std::ios::out);
    if (!logFile.is_open())
    {
        std::cerr << "Error opening log file: " << logFilePath << std::endl;
        exit(EXIT_FAILURE);
    }

    // д���ͷ
    logFile << "ʱ���, γ��, ����, �����, ��ǰ���� , �����̽Ƕ�, ��ͼγ��, ��ͼ����, ������ǩ, ɲ��ѹ��, ɲ������, EPB����פ��, ��λ, Ŀ��ĩ�ٶ�, ��ɲʹ��, ��ɲ�Ƴ�����, ǰ�����Ƴ�����, �󴥱��Ƴ�����, �󴥱��Ƴ�����, �Ҵ����Ƴ�����, ����, ������, ��ת��, ��ת��, ˫��, �����, ʾ����, Զ���, ���, ɲ����, Ԥ�����, �������";

    // ΪCAN֡������ӱ�ͷ
    for (int i = 0; i < 8; i++)
    {
        logFile << ", CAN֡" << i + 1 << "_ID, CAN֡" << i + 1 << "_����";
    }

    // ��ͷ����������
    logFile << std::endl;

    std::cout << "Created new log file: " << logFilePath << std::endl;
    recordCounter = 0;                               // ���ü�¼����
    current_log_timestamp = timestamp.substr(0, 19); // ���浱ǰ��־�ļ���ʱ���
}

// ������־���ļ�
void logToFile(DECISION_IN decision_control_in, VPARAMS cur_params, GIS GIS_map, double Ld, double alpha, std::vector<can_frame> can_frame_log)
{
    // ��ȡ��ǰʱ���
    std::string timestamp = getCurrentTimestamp();

    // �����־�ļ�δ�򿪻��߼�¼���Ѵﵽ���ޣ���ʼ�����ļ�
    if (!logFile.is_open() || recordCounter >= MAX_LOG_ENTRIES)
    {
        initLogFile();
    }

    // д����־����
    logFile << timestamp << ", "
            /**** ��ǰ����״̬ ****/
            << std::fixed << std::setprecision(8) << cur_params.latitude << ", "
            << std::fixed << std::setprecision(8) << cur_params.longitude << ", "
            << cur_params.heading_angle << ", "
            << cur_params.vehicle_speed << ", "
            << cur_params.steer_angle << ", "
            /**** ��ͼĿ��� ****/
            << GIS_map.latitude << ", "
            << GIS_map.longitude << ", "
            /**** �������루ȫ��22�������� ****/
            << decision_control_in.bStart << ", "
            /* �ƶ����� 210 */
            << decision_control_in.brake_bar << ", "
            << decision_control_in.breaking_dis << ", "
            /* פ������ 220 */
            << decision_control_in.EPB_park << ", "
            /* �������� 240 */
            << decision_control_in.gear << ", "
            << decision_control_in.endSpeed << ", "
            /* �������� 251 */
            << decision_control_in.Emgy_brk_En << ", "
            << decision_control_in.Emgy_brk_ReqRmv << ", "
            << decision_control_in.Emgy_FtCrashRemove << ", "
            << decision_control_in.Emgy_RrrCrashRemove << ", "
            << decision_control_in.Emgy_LeftCrashRemove << ", "
            << decision_control_in.Emgy_RightCrashRemove << ", "
            /* �������� 260 */
            << decision_control_in.ADU_Hom << ", "
            << decision_control_in.ADU_BackLamp << ", "
            << decision_control_in.ADU_TurnRLamp << ", "
            << decision_control_in.ADU_TurnLLamp << ", "
            << decision_control_in.ADU_DblFlashLamp << ", "
            << decision_control_in.ADU_LowBeamLamp << ", "
            << decision_control_in.ADU_WidthLamp << ", "
            << decision_control_in.ADU_HighBeamLamp << ", "
            << decision_control_in.ADU_FogLamp << ", "
            << decision_control_in.ADU_BrkLamp << ", "
            /**** ����ѭ�� */
            << Ld << ", "
            << alpha;

    // ��¼����CAN֡
    for (size_t i = 0; i < can_frame_log.size(); i++)
    {
        logFile << ", " << can_frame_log[i].can_id << ", ";

        // ��¼��ǰCAN֡��8�ֽ�����
        for (int j = 0; j < 8; ++j)
        {
            logFile << std::setw(2) << std::setfill('0') << std::hex << (int)can_frame_log[i].data[j];
            if (j < 7)
                logFile << " ";
        }
    }

    // ����
    logFile << std::endl;
    // ���Ӽ�¼����
    recordCounter++;
    // ÿ100����¼ˢ��һ�λ�������ȷ������д�����
    if (recordCounter % FLUSH_INTERVAL == 0)
    {
        logFile.flush();
    }
}

void *main_thread(void *arg)
{
    /****************************
     * ��һ���֣���ʼ��
     ****************************/
    printf("\nMain_Thread [%lu] is running\n", pthread_self());

    // ��ʼ������
    double Ld = 0.0, alpha = 0.0;
    double steer_error = 0.0;  // ��������ٳ�������Ƕ�
    double steer_angle = 0.0;  // ʵ�ʷ����̽Ƕ�
    float steer_ratio = 16.36; // ת��ȸ�ֵ��540 deg�������̣�/33�㣨����=16.36
    double wheelbase = 2.0;    // ���

    // ��ʼ�����ڣ�����GPS����ʱ��
    int serialFd = -1;
    if (enable_gps)
    {
        serialFd = configureSerialPort(SERIAL_PORT);
        if (serialFd == -1)
        {
            printf("\nSerialPort configure error!\n");
            return NULL;
        }
        std::cout << "Serial port " << SERIAL_PORT << " configured successfully. Listening for data..." << std::endl;
    }
    else
    {
        std::cout << "GPS�����ѽ��ã���ʹ��ģ�����ݽ��е���" << std::endl;
    }

    cout_can = 0; // CAN��ʼ��������

    // ��ʼ��GPS���ݴ�����ر���
    static auto lastProcessTime = std::chrono::steady_clock::now();
    static int emptyDataCount = 0;
    static int totalDataCount = 0;

    // ��ѭ��
    while (1)
    {
        /****************************
         * �ڶ����֣�GPS���ݻ�ȡ
         ****************************/
        // ��CAN֡�л�ȡ���٣����ڷ��͸����߽ṹ��control_out
        control_out.vehicle_speed = revin_4a2.vehicle_speed;
        // ��CAN֡�л�ȡSOC�����ڷ��͸����߽ṹ��control_out
        control_out.Soc_HIGH = revin_473.Bat_BatSOC;
        control_out.Soc_LOW = revin_471.LVBat_Volt;
        control_out.Emgy_brk_En = revin_451.Emrg_Sw_St;

        // ���ٻ�ȡ�����ڸ��³���״̬�ṹ��cur_params
        cur_params.vehicle_speed = control_out.vehicle_speed;

        std::vector<double> real_gps;

        // ��������ϴδ����ʱ����
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastProcessTime).count();

        if (enable_gps)
        {
            // ÿ�ζ�ȡǰ��¼ʱ�䣬ȷ������������GPS����ͨ����10Hz����100msһ�Σ�
            if (elapsedMs >= 95)
            { // ��΢��ǰһ�㣬ȷ������������
                lastProcessTime = currentTime;

                // �Ӵ��ڶ�ȡʵ��GPS����
                real_gps = readSerialData(serialFd);
                totalDataCount++;

                if (real_gps.empty())
                {
                    emptyDataCount++;
                    std::cerr << "��GPS���ݻ��������No valid data received. ��������: "
                              << (double)emptyDataCount / totalDataCount * 100 << "%" << std::endl;
                    // ����޷���ȡGPS���ݣ�����ʹ����һ�ε���Ч���ݻ�Ĭ��ֵ
                }
                else
                {
                    // ���³�������
                    cur_params.latitude = real_gps[0];
                    cur_params.longitude = real_gps[1];
                    cur_params.heading_angle = real_gps[2];
                    std::cout << "GPS���ݴ�����: " << elapsedMs << "ms" << std::endl;
                    printf("\n��ǰGPS��Ԫ����Ϣ��latitude = %f, longitude = %f, heading_angle = %f��",
                           cur_params.latitude, cur_params.longitude, cur_params.heading_angle);
                    printf("��ǰ���٣�vehicle_speed = %f\n", cur_params.vehicle_speed);
                }
            }
        }
        else
        {
            // ʹ��ģ�����ݽ��е���
            // ����������ù̶���ģ��ֵ���߸�����Ҫ���ɶ�̬ģ������
            cur_params.latitude = 39.20842796;   // ģ��γ��
            cur_params.longitude = 117.22737372; // ģ�⾭��
            cur_params.heading_angle = 160.0;    // ģ�⺽���
            // ��CAN֡�л�ȡ����
            control_out.vehicle_speed = revin_4a2.vehicle_speed;
            cur_params.vehicle_speed = control_out.vehicle_speed;

            printf("\nʹ��ģ��GPS���ݣ�latitude = %f, longitude = %f, heading_angle = %f��",
                   cur_params.latitude, cur_params.longitude, cur_params.heading_angle);
            printf("��ǰ���٣�vehicle_speed = %f\n", cur_params.vehicle_speed);
        }

        /****************************
         * �������֣�·���滮����
         ****************************/
        pthread_mutex_lock(&my_mutex);

        // ��ȡ��ͼ���ݺ�Ԥ�����
        double Ld = obtainMapData();

        // ģ������ʱ������ת��
        if (cur_params.vehicle_speed < 0.5 || enable_gps == false)
        {
            steer_error = 0;
        }
        else
        {
            // ����ת��Ƕ�
            steer_error = GPS2Steer(cur_params.vehicle_speed, cur_params.latitude, cur_params.longitude,
                                    cur_params.heading_angle, GIS_map.latitude, GIS_map.longitude,
                                    wheelbase, Ld, alpha);
        }

        // ����ʵ�ʷ����̽Ƕ�
        steer_angle = steer_error * steer_ratio; // ʵ�ʽǶ�=����Ƕ�*ת���
        /* test */
        // steer_angle = 0;

        cur_params.steer_angle = steer_angle;

        pthread_mutex_unlock(&my_mutex);

        /****************************
         * ���Ĳ��֣����߿��Ƹ�ֵ test
         ****************************/
        printf("\n\nStart Can \n\n");
        int test = 1;
        if (test == 1)
        {
            // Ϊ���߽ṹ������г�Ա��ֵ - ������������
            // 1. ������ǩ
            decision_control_in.bStart = 1; // ������ǩ��1��ʾ����

            // 2. �ƶ����� (210)
            decision_control_in.brake_bar = 0;        // ����ɲ��ѹ��Ϊ0bar
            decision_control_in.breaking_dis = 200.0; // ����ɲ������Ϊ200m

            // 3. פ������ (220)
            decision_control_in.EPB_park = 0; // ������פ��

            // 4. �������� (240)
            decision_control_in.gear = 9;       // ���õ�λΪ1P 3R 5N 9D
            decision_control_in.endSpeed = 0; // ����Ŀ���ٶ�Ϊ2km/h

            // 5. �������� (251)
            decision_control_in.Emgy_brk_En = 0;           // �����ü�ɲ
            decision_control_in.Emgy_brk_ReqRmv = 0;       // ���Ƴ���ɲ
            decision_control_in.Emgy_FtCrashRemove = 0;    // ���Ƴ�ǰ����
            decision_control_in.Emgy_RrrCrashRemove = 0;   // ���Ƴ��󴥱�
            decision_control_in.Emgy_LeftCrashRemove = 0;  // ���Ƴ��󴥱�
            decision_control_in.Emgy_RightCrashRemove = 0; // ���Ƴ��Ҵ���

            // 6. �������� (260)
            decision_control_in.ADU_Hom = 0;          // ����������
            decision_control_in.ADU_BackLamp = 0;     // �����õ�����
            decision_control_in.ADU_TurnRLamp = 0;    // ��������ת��
            decision_control_in.ADU_TurnLLamp = 0;    // ������ת��
            decision_control_in.ADU_DblFlashLamp = 1; // ������˫����
            decision_control_in.ADU_LowBeamLamp = 0;  // ���ý����
            decision_control_in.ADU_WidthLamp = 1;    // ����ʾ���
            decision_control_in.ADU_HighBeamLamp = 0; // ������Զ���
            decision_control_in.ADU_FogLamp = 1;      // ���������
            decision_control_in.ADU_BrkLamp = 0;      // �������ƶ���

            // ��ӡ���Բ���
            printf("\n���Բ���:\n");
            printf("ת��Ƕ�: %.2f deg\n", steer_angle);
            printf("������ǩ: %d\n", decision_control_in.bStart);
            printf("ɲ��ѹ��: %.2f bar\n", decision_control_in.brake_bar);
            printf("ɲ������: %.2f m\n", decision_control_in.breaking_dis);
            printf("פ��ʹ��: %d\n", decision_control_in.EPB_park);
            printf("��λ����: %d\n", decision_control_in.gear);
            printf("Ŀ���ٶ�: %.2f km/h\n", decision_control_in.endSpeed);
            printf("��ɲʹ��: %d\n", decision_control_in.Emgy_brk_En);
            printf("��ɲ�����Ƴ�: %d\n", decision_control_in.Emgy_brk_ReqRmv);
            printf("ǰ�����Ƴ�: %d\n", decision_control_in.Emgy_FtCrashRemove);
            printf("�󴥱��Ƴ�: %d\n", decision_control_in.Emgy_RrrCrashRemove);
            printf("�󴥱��Ƴ�: %d\n", decision_control_in.Emgy_LeftCrashRemove);
            printf("�Ҵ����Ƴ�: %d\n", decision_control_in.Emgy_RightCrashRemove);
            printf("����: %d\n", decision_control_in.ADU_Hom);
            printf("������: %d\n", decision_control_in.ADU_BackLamp);
            printf("��ת��: %d\n", decision_control_in.ADU_TurnLLamp);
            printf("��ת��: %d\n", decision_control_in.ADU_TurnRLamp);
            printf("˫����: %d\n", decision_control_in.ADU_DblFlashLamp);
            printf("�����: %d\n", decision_control_in.ADU_LowBeamLamp);
            printf("Զ���: %d\n", decision_control_in.ADU_HighBeamLamp);
            printf("���: %d\n", decision_control_in.ADU_FogLamp);
            printf("�ƶ���: %d\n", decision_control_in.ADU_BrkLamp);
            printf("ʾ���: %d\n", decision_control_in.ADU_WidthLamp);
        }

        convert_and_print_speed(decision_control_in.endSpeed); // תΪ�ٶ���С���ȵı���

        /****************************
         * ���岿�֣�CANͨ��
         ****************************/
        // ������ǩ����
        if (cout_can == -2 && decision_control_in.bStart == 1)
        {
            std::cout << "-------------------RESETTING--------------------" << std::endl;
            cout_can = 0;
        }

        // ����CAN���� - ����ControlStateѡ��ͬ�Ŀ���ģʽ
        if (ControlState == 2)
        {
            // RDCԶ�̿���ģʽ
            // send_candata(rdc_in, control_out, revin_4a2, revin_441, cout_can);
            printf("RDCģʽ - can��������cout_can= %d\n", cout_can);
        }
        else
        {
            // ���߿���ģʽ
            // send_candata(steer_angle, control_out, decision_control_in, revin_4a2, revin_441, cout_can);
            printf("����ģʽ - can��������cout_can= %d\n", cout_can);
        }

        // ��¼��־
        logToFile(decision_control_in, cur_params, GIS_map, Ld, alpha, can_frame_log);

        // ����20����
        usleep(20000);
    }

    return NULL;
}

// CAN�׽��ֽ�����麯��
bool check_can_socket_health(int sockfd)
{
    if (sockfd < 0)
        return false;

    // ���Ի�ȡ�ӿ�״̬������׽����Ƿ���Ч
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strcpy(ifr.ifr_name, "can0");

    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl in health check");
        return false;
    }

    return true;
}

// CAN�׽��ּ���̺߳���
void *can_monitor_thread(void *arg)
{
    printf("CAN����߳�������\n");

    while (1)
    {
        // ÿ5����һ��CAN�׽��ֽ���״̬
        sleep(5);

        // ʹ�û���������ȫ���׽��ַ���
        pthread_mutex_lock(&my_mutex);

        if (!check_can_socket_health(can_sockfd))
        {
            fprintf(stderr, "CAN�׽��ֽ������ʧ�ܣ��������³�ʼ��...\n");

            // �رվ��׽���
            if (can_sockfd >= 0)
            {
                close(can_sockfd);
            }

            // ���³�ʼ��
            can_sockfd = init_can_socket("can0");
            if (can_sockfd < 0)
            {
                fprintf(stderr, "CAN�׽������³�ʼ��ʧ��!\n");
            }
            else
            {
                fprintf(stderr, "CAN�׽������³�ʼ���ɹ�\n");
            }
        }

        pthread_mutex_unlock(&my_mutex);
    }

    return NULL;
}

/*can�����̴߳���*/
// �����̺߳���
void canSendThreadFunc(int socket_fd)
{
    while (running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        std::vector<can_frame> framesToSend;
        {
            std::lock_guard<std::mutex> lock(framesMutex);
            framesToSend = eightFrames;
        }
        can_frame_log = framesToSend; // can�ṹ�壺��־��¼ʹ��
        if (!framesToSend.empty())
        {
            if (flag_canSend)
            {
                batch_send_frames(socket_fd, framesToSend.data(), framesToSend.size());
                flag_canSend = false;
            }
            else
            {
                batch_send_frames(socket_fd, framesToSend.data(), framesToSend.size());
            }
        }
    }
}

// �̼߳��ݺ�����װ�� pthread_create ����
void *can_send_thread(void *arg)
{
    int socket_fd = *reinterpret_cast<int *>(arg);
    canSendThreadFunc(socket_fd);
    return nullptr;
}

int main()
{
    pthread_t threads[NUM_THREADS]; // ����һ���߳�����CAN���
    int rc;
    long t;

    rev_ok = 0;
    rev_save_ok = 1;
    rev_first = 1;
    begin_run = 0;

    rev_count = 0;
    frame_count = 0;

    _target_ind = 0;

    // ��ʼ��ȫ��CAN socket
    can_sockfd = init_can_socket("can0");
    if (can_sockfd < 0)
    {
        fprintf(stderr, "��ʼCAN�׽��ֳ�ʼ��ʧ�ܣ�����5�������...\n");
        sleep(5); // �ȴ�5�������

        can_sockfd = init_can_socket("can0");
        if (can_sockfd < 0)
        {
            fprintf(stderr, "����CAN�׽��ֳ�ʼ����Ȼʧ�ܣ�����Ӳ������!\n");
            exit(EXIT_FAILURE);
        }
    }

    // �����׽���Ϊ������ģʽ
    int flags = fcntl(can_sockfd, F_GETFL, 0);
    fcntl(can_sockfd, F_SETFL, flags | O_NONBLOCK);

    printf("CAN�׽��ֳ�ʼ���ɹ����׽���������: %d\n", can_sockfd);

    // ����CAN����߳�
    rc = pthread_create(&threads[0], NULL, can_monitor_thread, (void *)t);
    if (rc)
    {
        printf("ERROR; return code from pthread_create() is %d\n", rc);
        exit(-1);
    }
    /*
    // �����Բ�test �ر�udp
    */
   // ����UDP�߳�
   rc = pthread_create(&threads[1], NULL, udp_thread, (void *)t);
   if(rc){
       printf("ERROR; return code from pthread_create() is %d\n", rc);
       exit(-1);
   }

    string filePath = "/home/ztl/gps_path/path.txt";

    // ���ļ��ж�ȡ���ݲ�����map_latitude_v��map_longitude_v
    readMapDataFromFile(filePath);

    for (size_t i = 0; i < map_latitude_v.size(); ++i)
    {
        // ʹ��fixed��setprecision��ȷ�����8λС��
        cout << fixed << setprecision(8)
             << "Latitude: " << map_latitude_v[i]
             << ", Longitude: " << map_longitude_v[i] << endl;
    }
    begin_run = 1; // �����Բ� test
    cout << "No Decision Data: begin_run =" << begin_run << endl;
    while (begin_run == 0)
        ;

    // ���������߳�
    printf("Creating CAN receiving thread\n");
    rc = pthread_create(&threads[2], NULL, can_receiving_thread, (void *)2);
    if (rc)
    {
        printf("ERROR; return code from pthread_create() is %d\n", rc);
        exit(-1);
    }

    printf("Creating main control thread\n");
    rc = pthread_create(&threads[3], NULL, main_thread, (void *)3);
    if (rc)
    {
        printf("ERROR; return code from pthread_create() is %d\n", rc);
        exit(-1);
    }

    printf("Creating CAN send thread\n");
    rc = pthread_create(&threads[4], NULL, can_send_thread, &can_sockfd);
    if (rc)
    {
        printf("ERROR; return code from pthread_create() is %d\n", rc);
        exit(-1);
    }

    printf("�����߳��Ѵ�����ϵͳ������...\n");

    // ��ӡ�߳�״̬
    printf("�߳�״̬:\n");
    printf("- �߳�0: CAN����߳�\n");
    printf("- �߳�1: UDPͨ���߳�\n");
    printf("- �߳�2: CAN�����߳�\n");
    printf("- �߳�3: �������߳�\n");
    printf("- �߳�4: CAN�����߳�\n");

    struct itimerval tick;
    signal(SIGALRM, handle_sigalrm); // �����źŴ�����

    // ��ʼ����ʱ��
    tick.it_value.tv_sec = 0;
    tick.it_value.tv_usec = 20 * 1000; // 20����
    tick.it_interval = tick.it_value;  // �����ظ�����

    // ������ʱ��
    if (setitimer(ITIMER_REAL, &tick, NULL) < 0)
    {
        perror("setitimer");
        return 1;
    }

    // �ȴ������߳����
    // ע�⣺������NUM_THREADS+1���̣߳�������CAN����̣߳�
    for (t = 0; t < NUM_THREADS; t++)
    {
        printf("�ȴ��߳� %ld ����...\n", t);
        pthread_join(threads[t], NULL);
        printf("�߳� %ld �ѽ���\n", t);
    }

    // �������ǰ������Դ
    // 1. �ر�CAN socket
    if (can_sockfd >= 0)
    {
        close(can_sockfd);
        can_sockfd = -1;
    }

    // 2. �ر���־�ļ�
    if (logFile.is_open())
    {
        logFile.close();
    }

    // 3. ���ȫ�����飬�����˳�ʱ���ڴ�����
    eightFrames.clear();

    // 4. �������
    can_frame_log.clear();
    map_latitude_v.clear();
    map_longitude_v.clear();

    std::cout << "���������˳�����Դ������" << std::endl;
    return 0;
}
