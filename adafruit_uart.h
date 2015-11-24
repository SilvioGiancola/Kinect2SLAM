#ifndef ADAFRUIT_UART_H
#define ADAFRUIT_UART_H
/******************/
/*** ADAFRUIT DRIVERS ***/
/******************/
/******************/
#include <QSerialPort>
#include <QSerialPortInfo>
#include <iostream>
#include <iomanip>

#define SUCCESS 0
#define ERROR 1
#define ERR_DATABITS 2
#define ERR_PARITY 3
#define ERR_STOPBITS 4
#define ERR_FLOWCONTROL 5
#define ERR_OPEN 6
#define ERR_PLUG 7
#define ERR_MESSAGE 8
#define ERR_BAUDRATE 9



#define REG_PAGE_ID         0x07
#define REG_START_RAW_DATA  0x08
#define REG_START_ACC_DATA  0x08
#define REG_START_MAG_DATA  0x0E
#define REG_START_GYR_DATA  0x14
#define REG_START_EUL_DATA  0x1A
#define REG_START_QUA_DATA  0x20
#define REG_START_LIA_DATA  0x28
#define REG_START_GRV_DATA  0x2E
#define REG_START_TMP_DATA  0x34
#define REG_OPR_MODE        0x3D
#define REG_PWR_MODE        0x3E
#define REG_SYS_TRIGGER     0x3F


#include <QWidget>
#include <QThread>
#include <eigen3/Eigen/Geometry>

class Adafruit_Data {

public:
    explicit Adafruit_Data(){}

    Eigen::Vector3f	m_accelerometer;
    Eigen::Vector3f	m_magnetometer;
    Eigen::Vector3f	m_gyroscope;
    Eigen::Vector3f	m_euler_angles;
    Eigen::Quaternionf	m_quaternion;
    Eigen::Vector3f	m_linear_acceleration;
    Eigen::Vector3f	m_gravity;
    float	m_temperature;
};

class Adafruit_UART : public QSerialPort
{
    Q_OBJECT
public:

    // Constructor
    explicit Adafruit_UART(QWidget *parent = 0) : QSerialPort(parent) {}


    // Open the Serial connection
    int Open()
    {
        foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        {
            if (info.description() == QString("USB-Serial Controller"))
                if (info.manufacturer() == QString("Prolific Technology Inc."))
                    if(info.vendorIdentifier() == 0x067b)
                        if(info.productIdentifier() == 0x2303)
                            setPortName(info.portName());

        }

        plugged = true;
        if (this->portName().isEmpty())
            plugged = false;

        if (!plugged)
            return ERR_PLUG;

        if (!open(QIODevice::ReadWrite))
            return ERR_OPEN;

        if (!setBaudRate(QSerialPort::Baud115200))
            return ERR_BAUDRATE;

        if (!setDataBits(QSerialPort::Data8))
            return ERR_DATABITS;

        if (!setParity(QSerialPort::NoParity))
            return ERR_PARITY;

        if (!setStopBits(QSerialPort::OneStop))
            return ERR_STOPBITS;

        if (!setFlowControl(QSerialPort::NoFlowControl))
            return ERR_FLOWCONTROL;

        return SUCCESS;
    }


    // Try first communication and initilize the device
    int Init()
    {
        std::cout << "  --> INIT <--  " << std::endl;
        if ( !isOpen() )
        {
            std::cout << "  -> Error Device not opened" << std::endl;
            return ERROR;
        }


        std::cout << "Checking device ID, FW and BootLoader" << std::endl;
        if ( CheckDevice() != SUCCESS )
        {
            std::cout << "  -> Error Checking Device" << std::endl;
            return ERROR;
        }


        std::cout << "Set OPR_MODE to CONFIG MODE" << std::endl;
        if ( WriteRegister((quint8) REG_OPR_MODE, (quint8)0x00) != SUCCESS )
        {
            std::cout << "  -> Error entering in config mode" << std::endl;
            return ERROR;
        }


        std::cout << "Set REG_SYS_TRIGGER to 20" << std::endl;
        if ( WriteRegister((quint8) REG_SYS_TRIGGER, (quint8)0x20, true) != SUCCESS )
        {
            std::cout << "  -> Error setting REG_SYS_TRIGGER to 20" << std::endl;
            return ERROR;
        }


        QThread::usleep(1000*1000);


        std::cout << "Checking device ID, FW and BootLoader" << std::endl;
        if ( CheckDevice() != SUCCESS )
        {
            std::cout << "  -> Error Checking Device" << std::endl;
            return ERROR;
        }


        std::cout << "Set REG_POWER_MODE to 00" << std::endl;
        if ( WriteRegister((quint8) REG_PWR_MODE, (quint8)0x00) != SUCCESS )
        {
            std::cout << "  -> Error setting REG_POWER_MODE to 00" << std::endl;
            return ERROR;
        }


        QThread::usleep(1000*1000);


        std::cout << "Set REG_PAGE_ID to 00" << std::endl;
        if ( WriteRegister((quint8) REG_PAGE_ID, (quint8)0x00) != SUCCESS )
        {
            std::cout << "  -> Error setting REG_PAGE_ID to 00" << std::endl;
            return ERROR;
        }


        std::cout << "Set REG_SYS_TRIGGER to 80" << std::endl;
        if ( WriteRegister((quint8) REG_SYS_TRIGGER, (quint8)0x80) != SUCCESS )
        {
            std::cout << "  -> Error setting REG_SYS_TRIGGER to 20" << std::endl;
            return ERROR;
        }


        std::cout << "Set OPR_MODE to FUSION MODE" << std::endl;
        if ( WriteRegister((quint8) REG_OPR_MODE, (quint8)0x0C) != SUCCESS )
        {
            std::cout << "  -> Error entering in FUSION MODE" << std::endl;
            return ERROR;
        }


        QThread::usleep(1000*1000);


        return SUCCESS;

    }


    // Read all the 45 data available
    int ReadAllData(Adafruit_Data * result)
    {

        if ( !isOpen() )
        {
            std::cout << "  -> Error Device not opened" << std::endl;
            return ERROR;
        }


        QByteArray input ;
        ReadRegister(input, (quint8) REG_START_RAW_DATA, 0x2d);

        if( !(((quint8)input[0] == (quint8)0xBB) &&
              ((quint8)input[1] == (quint8)0x2d)) )
        {
            std::cout << "error" << std::endl;
            return ERROR;
        }


        int16_t s[4];
        int off = 2;



        // plain accelerometer, assumes [a] = m/s^2
        result->m_accelerometer =  convertQByteArrayInVector3f(input.mid(2,6));

        // magnetometer, always [B] = uT
        result->m_magnetometer = convertQByteArrayInVector3f(input.mid(8,6));

        // gyroscope, assumes [w] = deg/s
        result->m_gyroscope = convertQByteArrayInVector3f(input.mid(14,6));

        // euler angles, assumes [e] = deg/s
        result->m_euler_angles = convertQByteArrayInVector3f(input.mid(20,6));

        // quaternion, no unit
        result->m_quaternion = convertQByteArrayInQuaternionf(input.mid(26,8));

        // linear acceleration, assumes [a] = m/s^2
        result->m_linear_acceleration = convertQByteArrayInVector3f(input.mid(34,6));

        // gravity, assumes [a] = m/s^2
        result->m_gravity = convertQByteArrayInVector3f(input.mid(40,6));

        // temperature, assuming [T] = C
        result->m_temperature = (float) ((int16_t)static_cast<const quint8>(input[46]));



        return SUCCESS;
    }

    // Read the accelerometer;
    int GetAcc(Eigen::Vector3f * acc)
    {
        if ( !isOpen() )
        {
            std::cout << "  -> Error Device not opened" << std::endl;
            return ERROR;
        }

        QByteArray input ;
        ReadRegister(input, (quint8) REG_START_ACC_DATA, 6);

        if( !(((quint8)input[0] == (quint8)0xBB) &&
              ((quint8)input[1] == (quint8)6)) )
        {
            std::cout << "error" << std::endl;
            return ERROR;
        }

        *acc =  convertQByteArrayInVector3f(input.mid(2,6));

        return SUCCESS;
    }

    // Read the magnetometer;
    int GetMag(Eigen::Vector3f * mag)
    {
        if ( !isOpen() )
        {
            std::cout << "  -> Error Device not opened" << std::endl;
            return ERROR;
        }

        QByteArray input ;
        ReadRegister(input, (quint8) REG_START_MAG_DATA, 6);

        if( !(((quint8)input[0] == (quint8)0xBB) &&
              ((quint8)input[1] == (quint8)6)) )
        {
            std::cout << "error" << std::endl;
            return ERROR;
        }

        *mag =  convertQByteArrayInVector3f(input.mid(2,6));

        return SUCCESS;
    }

    // Read the linear_acceleration;
    int GetLia(Eigen::Vector3f * lia)
    {
        if ( !isOpen() )
        {
            std::cout << "  -> Error Device not opened" << std::endl;
            return ERROR;
        }

        QByteArray input ;
        ReadRegister(input, (quint8) REG_START_LIA_DATA, 6);

        if( !(((quint8)input[0] == (quint8)0xBB) &&
              ((quint8)input[1] == (quint8)6)) )
        {
            std::cout << "error" << std::endl;
            return ERROR;
        }

        *lia =  convertQByteArrayInVector3f(input.mid(2,6));

        return SUCCESS;
    }

    // Read the gyroscope;
    int GetGyr(Eigen::Vector3f * gyr)
    {
        if ( !isOpen() )
        {
            std::cout << "  -> Error Device not opened" << std::endl;
            return ERROR;
        }

        QByteArray input ;
        ReadRegister(input, (quint8) REG_START_GYR_DATA, 6);

        if( !(((quint8)input[0] == (quint8)0xBB) &&
              ((quint8)input[1] == (quint8)6)) )
        {
            std::cout << "error" << std::endl;
            return ERROR;
        }

        *gyr =  convertQByteArrayInVector3f(input.mid(2,6));

        return SUCCESS;
    }

    // Read the euler_angles;
    int GetEul(Eigen::Vector3f * eul)
    {
        if ( !isOpen() )
        {
            std::cout << "  -> Error Device not opened" << std::endl;
            return ERROR;
        }

        QByteArray input ;
        ReadRegister(input, (quint8) REG_START_EUL_DATA, 6);

        if( !(((quint8)input[0] == (quint8)0xBB) &&
              ((quint8)input[1] == (quint8)6)) )
        {
            std::cout << "error" << std::endl;
            return ERROR;
        }

        *eul =  convertQByteArrayInVector3f(input.mid(2,6));

        return SUCCESS;
    }

    // Read the quaternion
    int GetQuat(Eigen::Quaternionf * quat)
    {

        if ( !isOpen() )
        {
            std::cout << "  -> Error Device not opened" << std::endl;
            return ERROR;
        }


        QByteArray input ;
        ReadRegister(input, (quint8) REG_START_QUA_DATA, 8);

        if( !(((quint8)input[0] == (quint8)0xBB) &&
              ((quint8)input[1] == (quint8)8)) )
        {
            std::cout << "error in getting Quat" << std::endl;
            return ERROR;
        }

        *quat = convertQByteArrayInQuaternionf(input.mid(2,8));

        return SUCCESS;
    }

    // Read the gravity;
    int GetGrv(Eigen::Vector3f * grv)
    {
        if ( !isOpen() )
        {
            std::cout << "  -> Error Device not opened" << std::endl;
            return ERROR;
        }

        QByteArray input ;
        ReadRegister(input, (quint8) REG_START_GRV_DATA, 6);

        if( !(((quint8)input[0] == (quint8)0xBB) &&
              ((quint8)input[1] == (quint8)6)) )
        {
            std::cout << "error" << std::endl;
            return ERROR;
        }

        *grv =  convertQByteArrayInVector3f(input.mid(2,6));

        return SUCCESS;
    }


private:
    void PlotAnswer(QByteArray answer)
    {

        std::cout << "Total Answer is: " <<std::endl;
        int line = 0;
        std::cout << std::hex << "    " << (static_cast<quint16>(answer[0])&0xFF) << std::endl;
        std::cout << std::hex << "    " << (static_cast<quint16>(answer[1])&0xFF) << std::endl;

        for (int i = 2; i < answer.length(); i++)
        {
            std::cout << std::hex << "    " << (static_cast<quint16>(answer[i])&0xFF);

            line ++;
            if (line > 15)
            {
                std::cout << std::endl;
                line = 0;
            }
        }
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;

        return;
    }

    int CheckDevice()
    {
        QByteArray answer;
        ReadRegister(answer, (quint8) 0x00, 8);
        if ( !(((quint8)answer[0] == (quint8)0xBB) &&
               ((quint8)answer[1] == (quint8)0x08) &&
               ((quint8)answer[2] == (quint8)0xA0) &&
               ((quint8)answer[3] == (quint8)0xFB) &&
               ((quint8)answer[4] == (quint8)0x32) &&
               ((quint8)answer[5] == (quint8)0x0F)) )
        {
            PlotAnswer(answer);
            std::cout << "  -> Error in device ID" << std::endl;
            return ERROR;
        }
        return SUCCESS;
    }

    // Data Conversion
    Eigen::Vector3f convertQByteArrayInVector3f(QByteArray m_bytearray)
    {
        Eigen::Vector3f m_vec;
        int16_t m_temp;

        m_temp = (int16_t)static_cast<const quint8>(m_bytearray[0]) |
                (((int16_t)static_cast<const quint8>(m_bytearray[1]))<<8);
        m_vec.data()[0] = ( (float) m_temp ) / 100.0f;

        m_temp = (int16_t)static_cast<const quint8>(m_bytearray[2]) |
                (((int16_t)static_cast<const quint8>(m_bytearray[3]))<<8);
        m_vec.data()[1] = ( (float) m_temp ) / 100.0f;

        m_temp = (int16_t)static_cast<const quint8>(m_bytearray[4]) |
                (((int16_t)static_cast<const quint8>(m_bytearray[5]))<<8);
        m_vec.data()[2] = ( (float) m_temp ) / 100.0f;

        return m_vec;
    }

    Eigen::Quaternionf convertQByteArrayInQuaternionf(QByteArray m_bytearray)
    {
        Eigen::Quaternionf m_quat;
        int16_t m_temp;



        m_temp = (int16_t)static_cast<const quint8>(m_bytearray[0]) |
                (((int16_t)static_cast<const quint8>(m_bytearray[1]))<<8);
        m_quat.w() = ((float)m_temp);

        m_temp = (int16_t)static_cast<const quint8>(m_bytearray[2]) |
                (((int16_t)static_cast<const quint8>(m_bytearray[3]))<<8);
        m_quat.x() = ((float)m_temp);

        m_temp = (int16_t)static_cast<const quint8>(m_bytearray[4]) |
                (((int16_t)static_cast<const quint8>(m_bytearray[5]))<<8);
        m_quat.y() = ((float)m_temp);

        m_temp = (int16_t)static_cast<const quint8>(m_bytearray[6]) |
                (((int16_t)static_cast<const quint8>(m_bytearray[7]))<<8);
        m_quat.z() = ((float)m_temp);

        double norm = m_quat.w()*m_quat.w() +
                m_quat.x()*m_quat.x() +
                m_quat.y()*m_quat.y() +
                m_quat.z()*m_quat.z();
        norm = (norm>=0.0 ? 1.0/sqrt(norm) : 0.0);
        m_quat.w()*=float(norm);
        m_quat.x()*=float(norm);
        m_quat.y()*=float(norm);
        m_quat.z()*=float(norm);

        return m_quat;
    }


    // Read in the register
    int ReadRegister(QByteArray& receive, quint8 reg, quint8 size = 0x01)
    {
        QThread::usleep(1*1000);
        int pause = 0;
        //send a message in hexa
        char cmdBuf[] = { (char)0xAA,(char)0x01,(char)reg ,(char) size};
        QByteArray data = QByteArray::fromRawData(cmdBuf, sizeof(cmdBuf));

        int timeout= 20;


        //empty the buffer
        //   std::cout << "Buffer was still : " << readAll().size() << std::endl;
        readAll();
        clear();


        // write the message
        write(data);
        if (!waitForBytesWritten(4*timeout))
            std::cout << "All the byte has not been written!!!" << std::endl;


        QThread::usleep(pause*1000);


        //QByteArray receive;
        int excepted_data = ( static_cast<int16_t>(data[3]) ) + 2;

        while (receive.size() < excepted_data)
        {
            if (!waitForReadyRead(timeout))
                break;

            receive.append(readAll());
        }

        if ( (quint8) receive[0] == (quint8) 0xEE &&
             (quint8) receive[1] == (quint8) 0x07 )
        {
            std::cout << " overrun error" << std::endl;
            return ERROR;
        }
        else if ( !((quint8)receive[0] == (quint8)0xBB) )
        {
            std::cout << " error reading" << std::endl;
            return ERROR;
        }

        return SUCCESS;
    }

    // Write in the register
    int WriteRegister(quint8 reg, quint8 value, bool simple = false)
    {
        int pause = 0;
        //send a message in hexa
        char cmdBuf[] = { (char)0xAA,(char)0x00,(char)reg , (char)0x01, (char) value};
        QByteArray data = QByteArray::fromRawData(cmdBuf, sizeof(cmdBuf));

        int timeout = 1000;


        //empty the buffer
        // std::cout << "Buffer was still : " << readAll().size() << std::endl;
        readAll();
        clear();

        // write the message
        write(data);
        if (!waitForBytesWritten(4*timeout))
            std::cout << "All the byte has not been written!!!" << std::endl;

        if (simple)
            return SUCCESS;



        QThread::usleep(pause*1000);


        QByteArray receive;
        // int excepted_data = ( static_cast<int16_t>(data[3]) ) + 2;

        while (receive.size() < 2)
        {
            if (!waitForReadyRead(timeout))
                break;

            receive.append(readAll());
        }

        QThread::usleep(pause*1000);

        if ( !((receive.size() == 2 ) &&
               ((quint8)receive[0] == (quint8)0xEE) &&
               ((quint8)receive[1] == (quint8)0x01)) )
        {
            std::cout << "error in writing" << std::endl;
            PlotAnswer(receive);
            return ERROR;
        }

        return SUCCESS;
    }

    bool plugged;

};

#endif // ADAFRUIT_UART_H
