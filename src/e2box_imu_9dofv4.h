#ifndef E2BOX_IMU_9DOFV4_H
#define E2BOX_IMU_9DOFV4_H

#include "t_serial.h"
#include "stdio.h"

class e2box_imu_9dofv4
{

public:
    e2box_imu_9dofv4();
    virtual ~e2box_imu_9dofv4();

    t_serial serial;

    bool data_acquisition;

    bool m_boolUpdateDataFlag;
    void SetStatusUpdateData(bool boolOnOff) { m_boolUpdateDataFlag = boolOnOff; }
    bool GetStatusUpdateData(void) { return m_boolUpdateDataFlag; }

    unsigned int m_dwordCounterChecksumPass;
    unsigned int m_dwordCounterChecksumFail;

    bool m_boolHeaderDetectFlag;
    void SetStatusHeaderDetect(bool boolOnOff) { m_boolHeaderDetectFlag = boolOnOff; }
    bool GetStatusHeaderDetect(void) { return m_boolHeaderDetectFlag; }

    int m_iRawDataIndex;

    bool HandlingDataIMU();

    //double m_dAngle[3];
    double m_dQuaternion[4];
    double m_dAccel[3];
    double m_dAngRate[3];

    BYTE m_abyteRawData[90];
    char m_acCopiedRawData[90];

    int iChecksum ;
    int iCandisum ;

    void Initialize(void);

    void InterpretGeneral(void);
    BOOL CalcCheckSum(void);
    void ExtractData(unsigned char byteCurrent);

    //QString m_sPacket;

    DWORD CheckBYTEXOR(BYTE byteCompared1, BYTE byteCompared2);
};

#endif // SENSOR_GPS_H
