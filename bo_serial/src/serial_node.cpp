/*
 * serial_node.cpp
 *
 *  Created on: 2012-4-8
 *      Author: startar
 */

#include "serial_node.h"

namespace BO_SerialNode {

SerialNode::SerialNode(const ros::NodeHandle &node, const ros::NodeHandle &prinode):
		m_node(node), m_privateNode(prinode)
{
	cout << "SerialNode Object created!" << endl;
	loadParams();			// 载入参数
	SPEED_CODE.push_back(0x01);
	SPEED_CODE.push_back(0x04);
	SPEED_CODE.push_back(0x00);
	SPEED_CODE.push_back(0x38);
	SPEED_CODE.push_back(0x00);
	SPEED_CODE.push_back(0x04);
	SPEED_CODE.push_back(0x70);
	SPEED_CODE.push_back(0x04);
	// 创建SerialPort对象
	m_pSerialPort = make_shared<SerialPort>();
	m_pSerialPort->setSerialParams(m_serialParams);
	m_pSerialPort->setTimeOut(m_timeOut);
	m_pSerialPort->setCallbackFunc(bind(&SerialNode::serialCallback, this, _1));

	// 订阅两个串口数据Topic
	m_sub_bomsg = m_node.subscribe("SendSerialData", 1000,
			&SerialNode::BO_DataGram_Callback, this);
	m_sub_raw = m_node.subscribe("SendSerialData_raw", 1000,
			&SerialNode::BO_RawData_Callback, this);
	// 公布一个Topic, 发布从串口中读到的数据报文
	m_pub_Allrecv = m_node.advertise<bo_msgs::bo_DataGram>("base_vel", 1000);


	// 启动串口线程
	m_pSerialPort->startThread();

	

}

SerialNode::~SerialNode()
{
	m_pSerialPort->stopThread();
}

void SerialNode::loadParams()
{
	m_serialParams.serialPort 	= "/dev/ttyUSB0";
	m_serialParams.baudRate 	= 115200;
	m_serialParams.flowControl	= 0;
	m_serialParams.parity		= 0;
	m_serialParams.stopBits		= 0;
	m_timeOut = 1000;

	m_privateNode.getParam("serialPort", m_serialParams.serialPort);
	m_privateNode.getParam("baudRate", (int&)(m_serialParams.baudRate));
	m_privateNode.getParam("flowControl", (int&)(m_serialParams.flowControl));
	m_privateNode.getParam("parity", (int&)(m_serialParams.parity));
	m_privateNode.getParam("stopBits", (int&)(m_serialParams.stopBits));
	m_privateNode.getParam("timeout", m_timeOut);
}

void SerialNode::BO_DataGram_Callback(const bo_msgs::bo_DataGram::ConstPtr &msg)
{
	cout << "Sending new datagram !" << endl;
	m_pSerialPort->writeDataGram(*msg);
}

void SerialNode::BO_RawData_Callback(const bo_msgs::bo_Uint8Array::ConstPtr &msg)
{
	cout << "Sending new datagram !" << endl;	
	m_pSerialPort->writeRaw(msg->data);
}

void SerialNode::serialCallback(bo_msgs::bo_DataGramPtr pDatagram)
{
	// 向AllRecvData中发布一个msg
	speed=*pDatagram;
	m_pub_Allrecv.publish(pDatagram);
}

bo_msgs::bo_DataGram SerialNode::getSpeed()
{
	return speed;
}

void SerialNode::updateSpeed()
{
	m_pSerialPort->writeRaw(SPEED_CODE);
}

} /* namespace BO_SerialNode */
