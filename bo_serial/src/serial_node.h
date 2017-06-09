/*
 * serial_node.h
 *
 *  Created on: 2012-4-8
 *      Author: startar
 */

#ifndef SERIALNODE_H_
#define SERIALNODE_H_

#include <ros/ros.h>
#include <map>
#include "serial_port.h"
#include "bo_msgs/bo_DataGram.h"
#include "bo_msgs/bo_Uint8Array.h"

namespace BO_SerialNode {


class SerialNode {
private:
	SerialParams 		m_serialParams; 		// 串口的配置数据
	int 				m_timeOut; 				// 数据报超时时间
	ByteVector SPEED_CODE;
	shared_ptr<SerialPort>	m_pSerialPort;

	ros::NodeHandle 	m_node, m_privateNode;	// 当前NodeHandle与私有NodeHandle

	// Note: 按照ROS的文档, ROS提供的对象统统是thread-safe的, 所以不用管互斥了
	ros::Subscriber		m_sub_bomsg;			// SendSerialData的ROS订阅对象
	ros::Subscriber		m_sub_raw;				// SendSerialData_raw的ROS订阅对象
	ros::Publisher		m_pub_Allrecv;			// AllRecvData的ROS发布对象
	//std::map<uint8_t, ros::Publisher>	m_pubsByRecvID;		// 各receiveID对应的ROS发布对象

	void loadParams();

	bo_msgs::bo_DataGram speed;

	void BO_DataGram_Callback(const bo_msgs::bo_DataGram::ConstPtr &msg);
	void BO_RawData_Callback(const bo_msgs::bo_Uint8Array::ConstPtr &msg);
	void serialCallback(bo_msgs::bo_DataGramPtr pDatagram);

public:
	SerialNode(const ros::NodeHandle &node, const ros::NodeHandle &prinode);
	void updateSpeed();
	bo_msgs::bo_DataGram getSpeed();
	virtual ~SerialNode();
};

} /* namespace BO_SerialNode */
#endif /* SERIALNODE_H_ */
