#ifndef OURCONTROLERRORINFO_H
#define OURCONTROLERRORINFO_H

#include <string.h>

//错误信息的长度
#define ERROR_MSG_COUNT 60


//错误号
typedef enum
{
    OUR_CONTROL_SUCCESS = 0,
    OUR_CONTROL_PARAM_ERROR = 1,
    OUR_CONTROL_DISCONNECT,
    OUR_CONTROL_NOT_LOGIN,

    NO_ERROR_MSG,
    OUR_CONTROL_ERROR_COUNT,

}our_control_errorno;


typedef struct
{
    our_control_errorno error_no;       //错误号
    char error_msg[ERROR_MSG_COUNT];
}our_control_error;


class ourControlErrorInfo
{
    public:
        ourControlErrorInfo();

    private:
        int  m_currentErrorCode;

        char m_currentDetailErrorMsg[ERROR_MSG_COUNT];        //详细错误消息

        static ourControlErrorInfo * m_errorInfoHandle;

    public:

        //设置错误号
        void setCurrentErrorCode(int errorCode);

        //获取错误号
        int getCurrentErrorCode();

        //根据错误号从错误列表中获取错误信息
        char *getErrorMsgByErrorCode(int errorCode);

        //设置当前实时详细错误信息
        void setCurrentDetailErrorMsg(const char *errorMsg);

        //获取当前实时详细错误信息
        void getCurrentDetailErrorMsg(char *buf, unsigned int len);

        //初始化函数   用于创建一个本类的对象
        static void  InitErrorInfoHandle();

        //获取由init创建的本类的对象的指针
        static ourControlErrorInfo*  getErrorInfoHandle();
};

#endif // OURCONTROLERRORINFO_H
