
/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "include.h"

/*********************************************/

/*********************************************/
/*          Variable                         */
/*********************************************/


void (*transition[5][8])(void) = {

	//Pending
	{STTStop				, STTPendingStateVelocity	, STTPendingStateLineFollow		, STTPendingStatePathPlan		,
	 STTPendingStateUser 	, STTPendingStateParam		, STTPendingStateEnq			, INVALID													}	,

	 //Velocity
	{STTStop				, STTVelocityStateVelocity	, STTVelocityStateLineFollow	, STTVelocityStatePathPlan		,
	 STTVelocityStateUser	, STTVelocityStateParam		, STTVelocityStateEnq			, INVALID													}	,

	 //Line Follow
	{STTStop				, INVALID					, INVALID						, INVALID						,
	 INVALID				, STTLineFollowStateParam	, STTLineFollowStateEnq			, STTStop													}	,

	 //Path Planning
	{STTPathPlanStateStop	, INVALID					, INVALID						, INVALID						,
	 INVALID 				, STTPathPlanStateParam		, STTPathPlanStateEnq			, STTPathPlanStateStop										} ,

	//User
	{STTUserStateStop		, INVALID					, INVALID						, INVALID						,
	 INVALID				, STTUserStateParam			, STTUserStateEnq				, STTUserStateFinish										}
};


/*********************************************/

void STTStateInit(void)
{
	state = RNS_S_PENDING;
}

void STTEventChecker(void)
{
	if (main_board_1_data_receive.common_instruction != RNS_PENDING){

		sys.activate = 0;

		if (main_board_1_data_receive.common_instruction < RNS_INS_RESET)
			event = RNS_E_STOP;
		else if (main_board_1_data_receive.common_instruction < RNS_INS_VELOCITY)
			event = RNS_E_VELOCITY;
		else if (main_board_1_data_receive.common_instruction < RNS_INS_LINE_FOLLOW)
			event = RNS_E_LINE_FOLLOW;
		else if (main_board_1_data_receive.common_instruction < RNS_INS_PATH_PLAN)
			event = RNS_E_PATH_PLAN;
		else if (main_board_1_data_receive.common_instruction < RNS_INS_PARAM)
			event = RNS_E_PARAM;
		else if (main_board_1_data_receive.common_instruction < RNS_INS_ENQ)
			event = RNS_E_ENQ;
		else if (main_board_1_data_receive.common_instruction < RNS_INS_USER)
			event = RNS_E_USER;

		if(transition[state][event] != 0)
			(transition[state][event])();

		sys.activate = 1;
		main_board_1_data_receive.common_instruction = RNS_PENDING;

	}

	if (state == RNS_S_LINE_FOLLOW || state == RNS_S_PATH_PLAN ){
		if (!APPBusy(&ins)){
			event = RNS_E_FINISH;
			(*transition[state][event])();
			main_board_1_data_receive.common_instruction = RNS_PENDING;
		}
	}

	if (state == RNS_S_USER){
		if (UF.user == 0){
			event = RNS_E_FINISH;
			(*transition[state][event])();
			main_board_1_data_receive.common_instruction = RNS_PENDING;
		}
	}

}

void STTStop(void)
{
	APPStop();
	state = RNS_S_PENDING;

	insData_send[0] = 1;
	insData_send[1] = RNS_PENDING;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTPendingStateVelocity(void)
{
	ins.instruction = main_board_1_data_receive.common_instruction;
	ins.ins_buffer[0].data = main_board_1_data_receive.common_buffer[0].data;
	ins.ins_buffer[1].data = main_board_1_data_receive.common_buffer[1].data;
	ins.ins_buffer[2].data = main_board_1_data_receive.common_buffer[2].data;
	ins.ins_buffer[3].data = main_board_1_data_receive.common_buffer[3].data;

	APPApply(&ins);
	APPStart(&ins);
	state = RNS_S_VELOCITY;

	insData_send[0] = 1;
	insData_send[1] = RNS_PENDING;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif



}

void STTPendingStateLineFollow(void)
{
	ins.instruction = main_board_1_data_receive.common_instruction;
	ins.ins_buffer[0].data = main_board_1_data_receive.common_buffer[0].data;
	ins.ins_buffer[1].data = main_board_1_data_receive.common_buffer[1].data;
	ins.ins_buffer[2].data = main_board_1_data_receive.common_buffer[2].data;
	ins.ins_buffer[3].data = main_board_1_data_receive.common_buffer[3].data;


	APPApply(&ins);
	APPStart(&ins);
	state = RNS_S_LINE_FOLLOW;

	insData_send[0] = 1;
	insData_send[1] = RNS_BUSY;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTPendingStatePathPlan(void)
{
	ins.instruction = main_board_1_data_receive.common_instruction;
	ins.ins_buffer[0].data = main_board_1_data_receive.common_buffer[0].data;
	ins.ins_buffer[1].data = main_board_1_data_receive.common_buffer[1].data;
	ins.ins_buffer[2].data = main_board_1_data_receive.common_buffer[2].data;
	ins.ins_buffer[3].data = main_board_1_data_receive.common_buffer[3].data;

	APPApply(&ins);
	APPStart(&ins);
	state = RNS_S_PATH_PLAN;

	insData_send[0] = 1;
	insData_send[1] = RNS_BUSY;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTPendingStateUser(void)
{
	ins.instruction = main_board_1_data_receive.common_instruction;
	ins.ins_buffer[0].data = main_board_1_data_receive.common_buffer[0].data;
	ins.ins_buffer[1].data = main_board_1_data_receive.common_buffer[1].data;
	ins.ins_buffer[2].data = main_board_1_data_receive.common_buffer[2].data;
	ins.ins_buffer[3].data = main_board_1_data_receive.common_buffer[3].data;

	APPApply(&ins);
	APPStart(&ins);

	state = RNS_S_USER;

	insData_send[0] = 1;
	insData_send[1] = RNS_BUSY;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTPendingStateParam(void)
{
	param.parameter = main_board_1_data_receive.common_instruction;
	param.param_buffer[0].data = main_board_1_data_receive.common_buffer[0].data;
	param.param_buffer[1].data = main_board_1_data_receive.common_buffer[1].data;
	param.param_buffer[2].data = main_board_1_data_receive.common_buffer[2].data;
	param.param_buffer[3].data = main_board_1_data_receive.common_buffer[3].data;
	param.param_buffer[4].data = main_board_1_data_receive.common_buffer[4].data;
	param.param_buffer[5].data = main_board_1_data_receive.common_buffer[5].data;
	param.param_buffer[6].data = main_board_1_data_receive.common_buffer[6].data;

//	param.param_buffer[7].data = main_board_1_data_receive.common_buffer[7].data;

//	sprintf(uartbuff,"%f %f %f %f %f %f %f\r\n",param.param_buffer[0].data,param.param_buffer[1].data,param.param_buffer[2].data,
//					param.param_buffer[3].data,param.param_buffer[4].data,param.param_buffer[5].data,param.param_buffer[6].data);

//	sprintf(uartbuff,"%f %f %f %f %f %f %f\r\n",main_board_1_data_receive.common_buffer[0].data,main_board_1_data_receive.common_buffer[1].data,main_board_1_data_receive.common_buffer[2].data,
//			main_board_1_data_receive.common_buffer[3].data,main_board_1_data_receive.common_buffer[4].data,main_board_1_data_receive.common_buffer[5].data,main_board_1_data_receive.common_buffer[6].data);
//
//			UARTPrintString(UART5,uartbuff);

	APPSet(&param);
	state = RNS_S_PENDING;

	insData_send[0] = 1;
	insData_send[1] = RNS_PENDING;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTPendingStateEnq(void)
{
	enq.enquiry = main_board_1_data_receive.common_instruction;
	APPEnquire(&enq);
	state = RNS_S_PENDING;

	enq.enquiry = RNS_PENDING;
	insData_send[0] = 17;
	insData_send[1] = RNS_PENDING;


	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
		CAN_TxMsg(&hcan1,RNS_TO_mainboard_buf1,&enq.enq_buffer[0],8);
		CAN_TxMsg(&hcan1,RNS_TO_mainboard_buf2,&enq.enq_buffer[2],8);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
		CAN_TxMsg(&hcan2,RNS_TO_mainboard_buf1,&enq.enq_buffer[0],8);
		CAN_TxMsg(&hcan2,RNS_TO_mainboard_buf2,&enq.enq_buffer[2],8);
	#endif

}

void STTVelocityStateVelocity(void)
{
	ins.instruction = main_board_1_data_receive.common_instruction;
	ins.ins_buffer[0].data = main_board_1_data_receive.common_buffer[0].data;
	ins.ins_buffer[1].data = main_board_1_data_receive.common_buffer[1].data;
	ins.ins_buffer[2].data = main_board_1_data_receive.common_buffer[2].data;
	ins.ins_buffer[3].data = main_board_1_data_receive.common_buffer[3].data;

	APPApply(&ins);
	APPStart(&ins);
	state = RNS_S_VELOCITY;
	insData_send[0] = 1;
	insData_send[1] = RNS_PENDING;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif
}

void STTVelocityStateLineFollow(void)
{
	ins.instruction = main_board_1_data_receive.common_instruction;
	ins.ins_buffer[0].data = main_board_1_data_receive.common_buffer[0].data;
	ins.ins_buffer[1].data = main_board_1_data_receive.common_buffer[1].data;
	ins.ins_buffer[2].data = main_board_1_data_receive.common_buffer[2].data;
	ins.ins_buffer[3].data = main_board_1_data_receive.common_buffer[3].data;

	APPApply(&ins);
	APPStart(&ins);

	state = RNS_S_LINE_FOLLOW;

	insData_send[0] = 1;
	insData_send[1] = RNS_BUSY;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTVelocityStatePathPlan(void)
{
	ins.instruction = main_board_1_data_receive.common_instruction;
	ins.ins_buffer[0].data = main_board_1_data_receive.common_buffer[0].data;
	ins.ins_buffer[1].data = main_board_1_data_receive.common_buffer[1].data;
	ins.ins_buffer[2].data = main_board_1_data_receive.common_buffer[2].data;
	ins.ins_buffer[3].data = main_board_1_data_receive.common_buffer[3].data;

	APPApply(&ins);
	APPStart(&ins);
	state = RNS_S_PATH_PLAN;

	insData_send[0] = 1;
	insData_send[1] = RNS_BUSY;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTVelocityStateUser(void)
{
	ins.instruction = main_board_1_data_receive.common_instruction;
	ins.ins_buffer[0].data = main_board_1_data_receive.common_buffer[0].data;
	ins.ins_buffer[1].data = main_board_1_data_receive.common_buffer[1].data;
	ins.ins_buffer[2].data = main_board_1_data_receive.common_buffer[2].data;
	ins.ins_buffer[3].data = main_board_1_data_receive.common_buffer[3].data;

	APPApply(&ins);
	APPStart(&ins);
	state = RNS_S_USER;

	insData_send[0] = 1;
	insData_send[1] = RNS_BUSY;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTVelocityStateParam(void)
{
	//arrange structure
	param.parameter = main_board_1_data_receive.common_instruction;
	param.param_buffer[0].data = main_board_1_data_receive.common_buffer[0].data;
	param.param_buffer[1].data = main_board_1_data_receive.common_buffer[1].data;
	param.param_buffer[2].data = main_board_1_data_receive.common_buffer[2].data;
	param.param_buffer[3].data = main_board_1_data_receive.common_buffer[3].data;
	APPSet(&param);

	state = RNS_S_VELOCITY;

	insData_send[0] = 1;
	insData_send[1] = RNS_PENDING;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTVelocityStateEnq(void)
{
	//arrange structure
	enq.enquiry = main_board_1_data_receive.common_instruction;
	APPEnquire(&enq);
	state = RNS_S_VELOCITY;

	enq.enquiry = RNS_PENDING;

	insData_send[0] = 17;
	insData_send[1] = RNS_PENDING;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
		CAN_TxMsg(&hcan1,RNS_TO_mainboard_buf1,&enq.enq_buffer[0],8);
		CAN_TxMsg(&hcan1,RNS_TO_mainboard_buf2,&enq.enq_buffer[2],8);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
		CAN_TxMsg(&hcan2,RNS_TO_mainboard_buf1,&enq.enq_buffer[0],8);
		CAN_TxMsg(&hcan2,RNS_TO_mainboard_buf2,&enq.enq_buffer[2],8);
	#endif

}

void STTLineFollowStateParam(void)
{
	//arrange structure
	param.parameter = main_board_1_data_receive.common_instruction;
	param.param_buffer[0].data = main_board_1_data_receive.common_buffer[0].data;
	param.param_buffer[1].data = main_board_1_data_receive.common_buffer[1].data;
	param.param_buffer[2].data = main_board_1_data_receive.common_buffer[2].data;
	param.param_buffer[3].data = main_board_1_data_receive.common_buffer[3].data;
	APPSet(&param);

	state = RNS_S_LINE_FOLLOW;

	insData_send[0] = 1;
	insData_send[1] = RNS_BUSY;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTLineFollowStateEnq(void)
{
	//arrange structure
	enq.enquiry = main_board_1_data_receive.common_instruction;
	APPEnquire(&enq);
	state = RNS_S_LINE_FOLLOW;

	enq.enquiry = RNS_BUSY;

	insData_send[0] = 17;
	insData_send[1] = RNS_BUSY;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,&enq.enq_buffer[0],8);
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,&enq.enq_buffer[2],8);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,&enq.enq_buffer[0],8);
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,&enq.enq_buffer[2],8);
	#endif

}

void STTPathPlanStateStop(void)
{

	PP_stop(&pp);
	APPStop();
	// put function to stop path plan and clear the path plan related variable

	state = RNS_S_PENDING;

	insData_send[0] = 1;
	insData_send[1] = RNS_PENDING;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTPathPlanStateParam(void)
{
	//arrange structure
	param.parameter = main_board_1_data_receive.common_instruction;
	param.param_buffer[0].data = main_board_1_data_receive.common_buffer[0].data;
	param.param_buffer[1].data = main_board_1_data_receive.common_buffer[1].data;
	param.param_buffer[2].data = main_board_1_data_receive.common_buffer[2].data;
	param.param_buffer[3].data = main_board_1_data_receive.common_buffer[3].data;
	//param.param_buffer[4].data = main_board_1_data_receive.common_buffer[4].data;//
	APPSet(&param);

	state = RNS_S_PATH_PLAN;

	insData_send[0] = 1;
	insData_send[1] = RNS_BUSY;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTPathPlanStateEnq(void)
{
	//arrange structure
	enq.enquiry = main_board_1_data_receive.common_instruction;
	APPEnquire(&enq);
	state = RNS_S_PATH_PLAN;

	enq.enquiry = RNS_BUSY;

	insData_send[0] = 17;
	insData_send[1] = RNS_BUSY;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
		CAN_TxMsg(&hcan1,RNS_TO_mainboard_buf1,&enq.enq_buffer[0],8);
		CAN_TxMsg(&hcan1,RNS_TO_mainboard_buf2,&enq.enq_buffer[2],8);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
		CAN_TxMsg(&hcan2,RNS_TO_mainboard_buf1,&enq.enq_buffer[0],8);
		CAN_TxMsg(&hcan2,RNS_TO_mainboard_buf2,&enq.enq_buffer[2],8);
	#endif

}


void STTUserStateStop(void)
{
	UF.flag = 0;
	APPStop();
	// put function to stop path plan and clear the path plan related variable

	state = RNS_S_PENDING;

	insData_send[0] = 1;
	insData_send[1] = RNS_PENDING;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTUserStateParam(void)
{
	//arrange structure
	param.parameter = main_board_1_data_receive.common_instruction;
	param.param_buffer[0].data = main_board_1_data_receive.common_buffer[0].data;
	param.param_buffer[1].data = main_board_1_data_receive.common_buffer[1].data;
	param.param_buffer[2].data = main_board_1_data_receive.common_buffer[2].data;
	param.param_buffer[3].data = main_board_1_data_receive.common_buffer[3].data;
//	param.param_buffer[4].data = main_board_1_data_receive.common_buffer[4].data;//
	APPSet(&param);

	state = RNS_S_USER;

	insData_send[0] = 1;
	insData_send[1] = RNS_BUSY;
	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}

void STTUserStateEnq(void)
{
	//arrange structure
	enq.enquiry = main_board_1_data_receive.common_instruction;
	APPEnquire(&enq);
	state = RNS_S_PATH_PLAN;

	enq.enquiry = RNS_BUSY;

	insData_send[0] = 17;
	insData_send[1] = RNS_BUSY;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
		CAN_TxMsg(&hcan1,RNS_TO_mainboard_buf1,&enq.enq_buffer[0],8);
		CAN_TxMsg(&hcan1,RNS_TO_mainboard_buf2,&enq.enq_buffer[2],8);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
		CAN_TxMsg(&hcan2,RNS_TO_mainboard_buf1,&enq.enq_buffer[0],8);
		CAN_TxMsg(&hcan2,RNS_TO_mainboard_buf2,&enq.enq_buffer[2],8);
	#endif

}

void STTUserStateFinish(void)
{

	APPStop();

	state = RNS_S_PENDING;

	insData_send[1] = RNS_PENDING;

	#if defined USED_CAN1
		CAN_TxMsg(&hcan1,RNS_TO_mainboard,insData_send,2);
	#elif defined USED_CAN2
		CAN_TxMsg(&hcan2,RNS_TO_mainboard,insData_send,2);
	#endif

}



