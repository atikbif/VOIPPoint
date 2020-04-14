/*
 * can_cmd.c
 *
 *  Created on: 23 ���. 2019 �.
 *      Author: User
 */

#include "can_cmd.h"
#include "can_tx_stack.h"
#include "main.h"

extern uint8_t group_id;			// ����� ������/����� � ������� ����������� �����
extern uint8_t pos_in_group;		// ����� ����� � ������� �����
extern uint8_t test_2_5_kHz_res;	// ��������� �������� ���������

// ����� ������ �� ��������, ����� _pr ����� ����� ������� ���������
extern tx_stack can1_tx_stack;
extern tx_stack can2_tx_stack;
extern tx_stack can1_tx_stack_pr;
extern tx_stack can2_tx_stack_pr;

extern uint16_t discrete_state;		// ������� ��������� �����
// ������� ����:
// ��� 0 - ��������� �������� ���������/���������
// ��� 1  - ���� �� ��������
// ��� 2 - ��������� ���������
//������� ����:
// ��� 0 - ���� 1 �������
// ��� 1 - ���� 1 �����
// ��� 2 - ���� 1 ��
// ��� 3 - ���� 2 �������
// ��� 4 - ���� 2 �����
// ��� 5 - ���� 2 ��
// ��� 6 - ����� 1
// ��� 7 - ����� 2

extern uint8_t pow_data[2]; // ���������� ������������ � ������� 1 �� - 0.1�

extern uint64_t gain; // ����������� ���������� ���������
// 0 - ������������ ���������
// 1 - ���� � 2 ����
// 2 - ���� � 4 ����
// 3 - ���� � 8 ���

extern uint8_t limit_switch; // �������� ����������� (��� - ����� ��������� � ����)

// �������� ����� ��������� �����
void send_point_state(uint8_t can_num) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = pos_in_group;
	p_id->group_addr = group_id;
	p_id->cmd = POINT_STATE;
	p_id->param = 0;
	packet.priority = HIGH_PACKET_PRIORITY;	// ��������� ��������� ������� �� ������������ �������
	packet.length = 6;
	packet.data[0] = discrete_state & 0xFF;
	packet.data[1] = discrete_state >> 8;
	packet.data[2] = pow_data[0];
	packet.data[3] = pow_data[1];
	packet.data[4] = 1;	// ������
	packet.data[5] = gain;
	if(can_num==1) add_tx_can_packet(&can1_tx_stack_pr,&packet);
	else add_tx_can_packet(&can2_tx_stack_pr,&packet);
}

// ����� ��������� � ������� �����
// t - �������� ������������ ��� �������
// 1 - ������ � ��������� ����� (������������ � can2)
// ����� ������ �������� - ������������ ����� ��� ���������� �����
void next_point(uint8_t t) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = pos_in_group;
	p_id->group_addr = group_id;
	p_id->cmd = FIND_NEXT_POINT;
	p_id->param = t;
	packet.priority = HIGH_PACKET_PRIORITY;
	packet.length = 1;
	packet.data[0] = pos_in_group;
	if(t==1) add_tx_can_packet(&can2_tx_stack_pr,&packet);	// request
	else add_tx_can_packet(&can1_tx_stack_pr,&packet);	// answer
}

// ��������� ��� ����� ��������� � ����
// 2 ��������:
// ����� ������� ���� ��������� ��-�� ����������� ��������� ����������� (������� �����)
// ����� ������������� �������� ��������� � ���� (�� ������ ���������� ��������� �����)

void last_point() {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	if(limit_switch) p_id->type = NORMAL_FINISH;
	else p_id->type = BREAK_FINISH;
	p_id->point_addr = pos_in_group;
	p_id->group_addr = group_id;
	p_id->cmd = LAST_POINT;
	p_id->param = 0;
	packet.priority = HIGH_PACKET_PRIORITY;
	packet.length = 0;
	add_tx_can_packet(&can1_tx_stack_pr,&packet);
}
