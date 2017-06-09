/*
 * serial_port.cpp
 *
 *  Created on: 2012-4-8
 *      Author: startar
 */

#include "serial_port.h"
#include <vector>
#include <boost/make_shared.hpp>

using namespace std;
using namespace boost;

namespace BO_SerialNode {

SerialPort::SerialPort()
{
	cout << "SerialPort Object created!" << endl;

	m_tempBuf.resize(1024, 0);
}

SerialPort::~SerialPort()
{
	m_pios->stop();
	m_thread.join();
}

void SerialPort::start_a_read()
{
	cout << "SerialPort::start_a_read called !!" << endl;

	mutex::scoped_lock lock(m_serialMutex);

	// 启动一次异步读
	m_pSerial->async_read_some(buffer(m_tempBuf), boost::bind(&SerialPort::readHandler,
			this,
			placeholders::error,
			placeholders::bytes_transferred
			));
}

void SerialPort::start_a_write()
{
	mutex::scoped_lock lock(m_serialMutex);

	// 启动一次异步写
	async_write(*m_pSerial, buffer(*(m_writeQueue.front())),
			bind(&SerialPort::writeHandler, this, placeholders::error));
}

void SerialPort::mainRun()
{
	cout << "SerialPort mainThread STARTED!" << endl;

	// 初始化工作状态
	m_state = WAITING_FF;

	// 设置Header存储区的大小为4字节
	m_currentHeader.resize(4, 0);

	// 启动一次异步读
	start_a_read();

	// 开始跑io_service::run()
	m_pios->run();

	cout << "SerialPort mainThread EXITED!" << endl;
}

void SerialPort::readHandler(const system::error_code &ec, size_t bytesTransferred)
{
	size_t vec_size=14;
	double v_l=0.0,v_r=0.0;
	uint8_t CLC_H=0,CLC_L=0;
	if (ec)
	{
		// TODO: 报错
		cout << "SerialPort read error !!" << endl;
		return;
	}
//	for (size_t i=0; i<vec_size; i++)
//	{
//		printf("%02X ", m_tempBuf.at(i));
//	}
//	cout << endl;
	// 处理每个读到的字符


	if(m_tempBuf.at(0)==(uint8_t)0x01&&m_tempBuf.at(1)==(uint8_t)0x04&&m_tempBuf.at(2)==(uint8_t)0x00&&m_tempBuf.at(3)==(uint8_t)0x08)
	{
		m_ptimer.reset(new deadline_timer(*m_pios,
				posix_time::milliseconds(m_timeOut)));
		m_ptimer->async_wait(bind(
					&SerialPort::timeoutHandler,
					this,
					placeholders::error));
		unsigned short l=((0+m_tempBuf.at(6))<<8)+m_tempBuf.at(7);
		unsigned short r=((0+m_tempBuf.at(10))<<8)+m_tempBuf.at(11);
	//	cout<<l<<" "<<r<<"i="<<i<<endl;
		v_l= 0.001*l;
		v_r= 0.001*r;
		if(m_tempBuf.at(5))v_l=v_l*-1;
		if(m_tempBuf.at(9))v_r=v_r*-1;
		CLC_H=m_tempBuf.at(13);
		CLC_L=m_tempBuf.at(12);

		m_ptimer->cancel();
		m_ptimer.reset();
		unsigned short xda , xdapoly;
		uint8_t i,j, xdabit;
		uint8_t calculate_CRC_L,calculate_CRC_H;
		xda = 0xFFFF;
		xdapoly = 0xA001;
		// (X**16 + X**15 + X**2 + 1)
		for(i=0;i<vec_size-2;i++)
		{
			xda ^= m_tempBuf[i];
			for(j=0;j<8;j++)
			{
			xdabit = (uint8_t )(xda & 0x01);
			xda >>= 1;
			if( xdabit ) xda ^= xdapoly;
			}
		//CtrlWatchdogReset( );
		}
		calculate_CRC_L = (uint8_t)(xda & 0xFF);
		calculate_CRC_H = (uint8_t)(xda>>8);
		if (calculate_CRC_L=CLC_L && calculate_CRC_H==CLC_H)
		{
			// 如果校验通过, 就构建一个bo_DataGram, 通过回调函数传递出去
			bo_msgs::bo_DataGramPtr pNewDiagram =
				make_shared<bo_msgs::bo_DataGram>();
			pNewDiagram->left_vel= v_l;
			pNewDiagram->right_vel= v_r;

			cout << "A new datagram received !!" << endl;
			cout<< v_l<<" "<<v_r<<endl;
			m_dataCallbackFunc(pNewDiagram);
		}
		else {
			printf("checksum :0x%X 0x%X \n",calculate_CRC_L,calculate_CRC_H );
			printf("real checksum :0x%X 0x%X \n",CLC_L,CLC_H );
			cout << "checksum error !!" << endl;
	
			}
	}
	start_a_read();
}

void SerialPort::writeHandler(const system::error_code &ec)
{
	if (ec)
	{
		// TODO: 报错
	}

	{
		mutex::scoped_lock lock(m_writeQueueMutex);
		m_writeQueue.pop();
		if (m_writeQueue.empty()==false)
		{
			start_a_write();
		}
	}
}

void SerialPort::timeoutHandler(const system::error_code &ec)
{
	if (!ec)
	{
		cout << "Time Out !" << endl;
		m_state = WAITING_FF;
	}
}

void SerialPort::setSerialParams(const SerialParams &params)
{
	m_serialParams = params;
}

void SerialPort::setTimeOut(int timeout)
{
	m_timeOut = timeout;
}

bool SerialPort::startThread()
{
	cout << "SerialPort::startThread() called!" << endl;

	// 创建io_service对象
	m_pios = make_shared<io_service>();

	try {
		// 创建一个serial_port对象, 替换掉原来的
		m_pSerial = make_shared<serial_port>(ref(*m_pios), m_serialParams.serialPort);

		// 设置串口通信参数
		// PS: 这里的强制转换不是什么好习惯... 我只是图省事
		m_pSerial->set_option(
				serial_port::baud_rate(m_serialParams.baudRate));
		m_pSerial->set_option(
				serial_port::flow_control((serial_port::flow_control::type)m_serialParams.flowControl));
		m_pSerial->set_option(
				serial_port::parity((serial_port::parity::type)m_serialParams.parity));
		m_pSerial->set_option(
				serial_port::stop_bits((serial_port::stop_bits::type)m_serialParams.stopBits));
		m_pSerial->set_option(serial_port::character_size(8));
	}
	catch (std::exception &e) {
		cout << "Failed to open serial port !" << endl;
		cout << "Error Info: " << e.what() << endl;
		return false;
	}

	try {
		// 创建线程
		m_thread = boost::thread(boost::bind(&SerialPort::mainRun, this));
	}
	catch (std::exception &e) {
		cout << "Failed to create thread !" << endl;
		cout << "Error Info: " << e.what() << endl;
		return false;
	}
	return true;
}

bool SerialPort::stopThread()
{
	m_pios->stop();
	return true;
}

void SerialPort::setCallbackFunc(const function<void(bo_msgs::bo_DataGramPtr)> &func)
{
	m_dataCallbackFunc = func;
}

bool SerialPort::writeDataGram(const bo_msgs::bo_DataGram &datagram)
{
	ByteVector bufToSend(16, 0);
	bufToSend[0] = (uint8_t)0x01;
	bufToSend[1] = (uint8_t)0x16;
	bufToSend[2] = (uint8_t)0x00;
	bufToSend[3] = (uint8_t)0x38;
	bufToSend[4] = (uint8_t)0x00;
	bufToSend[5] = (uint8_t)0x04;

	bufToSend[6] = (uint8_t)0x00;
	if(datagram.left_vel>0)
		bufToSend[7] = (uint8_t)0x00;
	else
		bufToSend[7] = (uint8_t)0x01;

	unsigned short vel =floor(abs(datagram.left_vel)*1000);
	bufToSend[9] = (uint8_t)(vel&0xFF);
	bufToSend[8] = (uint8_t)(vel>>8);

	if(datagram.right_vel>0)
		bufToSend[10] = (uint8_t)0x00;
	else
		bufToSend[11] = (uint8_t)0x01;

	vel =floor(abs(datagram.right_vel)*1000);
	bufToSend[13] = (uint8_t)(vel&0xFF);
	bufToSend[12] = (uint8_t)(vel>>8);
	

	unsigned short xda , xdapoly;
	uint8_t i,j, xdabit;
	xda = 0xFFFF;
	xdapoly = 0xA001;
	// (X**16 + X**15 + X**2 + 1)
	for(i=0;i<14;i++)
	{
		xda ^= bufToSend[i];
		for(j=0;j<8;j++)
		{
		xdabit = (uint8_t )(xda & 0x01);
		xda >>= 1;
		if( xdabit ) xda ^= xdapoly;
		}
	//CtrlWatchdogReset( );
	}
	bufToSend[14] = (uint8_t)(xda & 0xFF);
	bufToSend[15] = (uint8_t)(xda>>8);

	// 调用writeRaw
	return writeRaw(bufToSend);
}

bool SerialPort::writeRaw(const ByteVector &rawData)
{
//	for (size_t i=0; i<rawData.size(); i++)
//	{
//		printf("%02X ", rawData.at(i));
//	}
//	cout << endl;
	 mutex::scoped_lock lock(m_writeQueueMutex);	// 上锁

	bool writeIdle = m_writeQueue.empty();			// 检测当前队列是否为空
	pByteVector data(new ByteVector(rawData));
	m_writeQueue.push(data);						// 将数据拷贝一份, 并加入队列里去

	if (writeIdle) start_a_write();					// 如果没有在写数据, 则启动一次异步写过程

	return true;
}

//校验程序
/*void ModBus_CRC16_Calculate(unsigned char *aStr , unsigned char alen)
{
	//unsigned int xda , xdapoly;
	unsigned short xda , xdapoly;
	unsigned char i,j, xdabit;
	xda = 0xFFFF;
	xdapoly = 0xA001;
	// (X**16 + X**15 + X**2 + 1)
	for(i=0;i<alen;i++)
	{
		xda ^= aStr[i];
		for(j=0;j<8;j++)
		{
			xdabit = (unsigned char )(xda & 0x01);
			xda >>= 1;
			if( xdabit ) xda ^= xdapoly;
		}
	//CtrlWatchdogReset( );
	}
	calculate_CRC_L = (unsigned char)(xda & 0xFF);
	calculate_CRC_H = (unsigned char)(xda>>8);
}*/	

} /* namespace BO_SerialNode */
