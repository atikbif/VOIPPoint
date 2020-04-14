/*
 * can_cmd.c
 *
 *  Created on: 23 авг. 2019 г.
 *      Author: User
 */

#include "can_cmd.h"
#include "can_tx_stack.h"
#include "main.h"

extern uint8_t group_id;			// номер группы/шлюза к которой принадлежит точка
extern uint8_t pos_in_group;		// номер точки в цепочке шлюза
extern uint8_t test_2_5_kHz_res;	// результат проверки динамиков

// стеки команд на передачу, стеки _pr имеют более высокий приоритет
extern tx_stack can1_tx_stack;
extern tx_stack can2_tx_stack;
extern tx_stack can1_tx_stack_pr;
extern tx_stack can2_tx_stack_pr;

extern uint16_t discrete_state;		// битовое состо€ние точки
// младший байт:
// бит 0 - результат проверки микрофона/динамиков
// бит 1  - была ли проверка
// бит 2 - состо€ние концевика
//старший байт:
// бит 0 - вход 1 замкнут
// бит 1 - вход 1 обрыв
// бит 2 - вход 1 кз
// бит 3 - вход 2 замкнут
// бит 4 - вход 2 обрыв
// бит 5 - вход 2 кз
// бит 6 - выход 1
// бит 7 - выход 2

extern uint8_t pow_data[2]; // напр€жение аккумул€тора и питани€ 1 ед - 0.1¬

extern uint64_t gain; // коэффициент ослаблени€ громкости
// 0 - максимальна€ громкость
// 1 - тише в 2 раза
// 2 - тише в 4 раза
// 3 - тише в 8 раз

extern uint8_t limit_switch; // концевой выключатель (вкл - точка последн€€ в цепи)

// передать шлюзу состо€ние точки
void send_point_state(uint8_t can_num) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = pos_in_group;
	p_id->group_addr = group_id;
	p_id->cmd = POINT_STATE;
	p_id->param = 0;
	packet.priority = HIGH_PACKET_PRIORITY;	// приоритет оказывает вли€ние на ретрансл€цию пакетов
	packet.length = 6;
	packet.data[0] = discrete_state & 0xFF;
	packet.data[1] = discrete_state >> 8;
	packet.data[2] = pow_data[0];
	packet.data[3] = pow_data[1];
	packet.data[4] = 1;	// верси€
	packet.data[5] = gain;
	if(can_num==1) add_tx_can_packet(&can1_tx_stack_pr,&packet);
	else add_tx_can_packet(&can2_tx_stack_pr,&packet);
}

// поиск следующей в цепочке точки
// t - параметр определ€ющий тип посылки
// 1 - запрос к следующей точке (отправл€етс€ в can2)
// любое другое значение - сформировать ответ дл€ предыдущей точки
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

// сообщение что точка последн€€ в цепи
// 2 варианта:
// точка считает себ€ последней из-за включенного концевого выключател€ (штатный режим)
// точка действительно €вл€етс€ последней в цепи (не удаЄтс€ обнаружить следующую точку)

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
