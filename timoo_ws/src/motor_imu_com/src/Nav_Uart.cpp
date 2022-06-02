#include <iostream>
#include <boost/asio.hpp>             // boost库头文件
#include <boost/bind.hpp>
#include <string>

using namespace std;
using namespace boost::asio;


bool UART_Init(boost::asio::serial_port &sp1)
{   
    if(!sp1.is_open())
    {
        cout << "Open false" << endl;
        return false;
    }
        
    sp1.set_option(serial_port::baud_rate(460800));                              // 设置波特率
    sp1.set_option(serial_port::flow_control(serial_port::flow_control::none));  // 设置控制方式
    sp1.set_option(serial_port::parity(serial_port::parity::none));              // 设置奇偶校验
    sp1.set_option(serial_port::stop_bits(serial_port::stop_bits::one));         // 设置停止位
    sp1.set_option(serial_port::character_size(8));                              // 设置字母位数为8位
    cout << "Init OK" << endl;
    return true;
}

int main(int argc , char ** argv)
{
    char sent_buf[5] = {0};
    char Read_Buff[5] = {0};

    io_service m_iosev;
    serial_port sp(m_iosev, "/dev/ttyUSB0"); 
    UART_Init(sp); 

    write(sp, buffer(sent_buf, 5));
    cout << "Sent OK" << endl;

    read(sp, buffer(buffer(Read_Buff,5)));  

    for (int i = 0 ;i <5 ;i++)
    {
        cout << Read_Buff[i] << endl;
    }
    //read(*pSerialPort,buffer(Read_Buff,date_length),m_ec);
    m_iosev.run();
    return 0;
}
