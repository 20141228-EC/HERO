
/* Includes ------------------------------*/
#include "auto_buy_bullet.h"
extern UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/

/* Exported variables --------------------*/
client_interactive_frame_t client_interactive_frame={
	.std_frame_header_.sof=SOF,
	.std_frame_header_.data_length=CLIENT_INTERACTIVE_DATA_LENGTH,
	.cmd_id=KEY_MOUSE_DATA_CMD_ID,
};
Interactive_Class_t Interactive_Class={
	.client_interactive_frame=&client_interactive_frame,
};
uint8_t auto_buy_bullet_data_buffer[100];
/* Function  body --------------------------------------------------------*/
void test ()
{

	client_interactive_frame.std_frame_header_.seq++;
	client_interactive_frame.custom_client_data.key_value1=79;
	client_interactive_frame.custom_client_data.key_value2=73;
	client_interactive_frame.custom_client_data.x_position=500;
	client_interactive_frame.custom_client_data.y_position=500;
	client_interactive_frame.custom_client_data.mouse_left=0;
	client_interactive_frame.custom_client_data.mouse_right=0;
	memcpy(auto_buy_bullet_data_buffer,&client_interactive_frame,sizeof(client_interactive_frame_t));
	Append_CRC8_Check_Sum(auto_buy_bullet_data_buffer, CRC8_INDEX);                  
    Append_CRC16_Check_Sum(auto_buy_bullet_data_buffer, sizeof(client_interactive_frame_t)); 
	
	HAL_UART_Transmit(&huart2,auto_buy_bullet_data_buffer,sizeof(client_interactive_frame_t),0xff);
}
/**
* @brief 鼠标移动点击
* @note  鼠标移动点击，键盘全部松开;
* @parament uint8_t* command_start_flag 给其传入1，自动执行一次，执行完自动置0； 
*/
void Mouse_movement_work (client_interactive_frame_t *client_interactive_frame, uint16_t x_position,uint16_t y_position, \
					 uint8_t left_state, uint8_t right_state,uint8_t* command_start_flag)
{
	const uint16_t delay_time=35;
	static uint8_t mouse_movement_step;//步骤
	
	if(*command_start_flag==0)
	{
		//复位
		mouse_movement_step=0;
		return;
	}
	if(mouse_movement_step==0)
	{
		client_interactive_frame->std_frame_header_.seq++;
		client_interactive_frame->custom_client_data.key_value1=0;
		client_interactive_frame->custom_client_data.key_value2=0;
		client_interactive_frame->custom_client_data.x_position=x_position;
		client_interactive_frame->custom_client_data.y_position=y_position;
		client_interactive_frame->custom_client_data.mouse_left=0;
		client_interactive_frame->custom_client_data.mouse_right=0;
		memcpy(auto_buy_bullet_data_buffer,client_interactive_frame,sizeof(client_interactive_frame_t));
		Append_CRC8_Check_Sum(auto_buy_bullet_data_buffer, CRC8_INDEX);                  
		Append_CRC16_Check_Sum(auto_buy_bullet_data_buffer, sizeof(client_interactive_frame_t)); 
		HAL_UART_Transmit(&huart2,auto_buy_bullet_data_buffer,sizeof(client_interactive_frame_t),0xff);
		
		mouse_movement_step++;
		HAL_Delay(delay_time);
	}
	else if(mouse_movement_step==1)
	{
		client_interactive_frame->std_frame_header_.seq++;
		client_interactive_frame->custom_client_data.key_value1=0;
		client_interactive_frame->custom_client_data.key_value2=0;
		client_interactive_frame->custom_client_data.x_position=x_position;
		client_interactive_frame->custom_client_data.y_position=y_position;
		client_interactive_frame->custom_client_data.mouse_left=left_state;
		client_interactive_frame->custom_client_data.mouse_right=right_state;
		memcpy(auto_buy_bullet_data_buffer,client_interactive_frame,sizeof(client_interactive_frame_t));
		Append_CRC8_Check_Sum(auto_buy_bullet_data_buffer, CRC8_INDEX);                  
		Append_CRC16_Check_Sum(auto_buy_bullet_data_buffer, sizeof(client_interactive_frame_t)); 
		HAL_UART_Transmit(&huart2,auto_buy_bullet_data_buffer,sizeof(client_interactive_frame_t),0xff);
		
		mouse_movement_step++;
		HAL_Delay(delay_time);
	}
	else if(mouse_movement_step==2)
	{
		client_interactive_frame->std_frame_header_.seq++;
		client_interactive_frame->custom_client_data.key_value1=0;
		client_interactive_frame->custom_client_data.key_value2=0;
		client_interactive_frame->custom_client_data.x_position=x_position;
		client_interactive_frame->custom_client_data.y_position=y_position;
		client_interactive_frame->custom_client_data.mouse_left=0;
		client_interactive_frame->custom_client_data.mouse_right=0;
		memcpy(auto_buy_bullet_data_buffer,client_interactive_frame,sizeof(client_interactive_frame_t));
		Append_CRC8_Check_Sum(auto_buy_bullet_data_buffer, CRC8_INDEX);                  
		Append_CRC16_Check_Sum(auto_buy_bullet_data_buffer, sizeof(client_interactive_frame_t)); 
		HAL_UART_Transmit(&huart2,auto_buy_bullet_data_buffer,sizeof(client_interactive_frame_t),0xff);
		
		
		mouse_movement_step=0;//复位
		*command_start_flag=0;//结束执行
		HAL_Delay(delay_time);
	}
}

/**
* @brief 按下松开一次键盘,两键无冲
* @note  鼠标默认在（0，0）;
* @parament uint8_t* command_start_flag 给其传入1，自动执行一次，执行完自动置0； 
*			uint8_t key_value 传入通用键值； 
*/
void Keyboard_movement_work (client_interactive_frame_t *client_interactive_frame,uint8_t key1_value,uint8_t key2_value,uint8_t* command_start_flag)
{
	const uint16_t delay_time=35;
	static uint8_t Keyboard_movement_step;//步骤
	
	if(*command_start_flag==0)
	{
		//复位
		Keyboard_movement_step=0;
		return;
	}
	if(Keyboard_movement_step==0)
	{
		client_interactive_frame->std_frame_header_.seq++;
		client_interactive_frame->custom_client_data.key_value1=key1_value;
		client_interactive_frame->custom_client_data.key_value2=key2_value;
		client_interactive_frame->custom_client_data.x_position=0;
		client_interactive_frame->custom_client_data.y_position=0;
		client_interactive_frame->custom_client_data.mouse_left=0;
		client_interactive_frame->custom_client_data.mouse_right=0;
		memcpy(auto_buy_bullet_data_buffer,client_interactive_frame,sizeof(client_interactive_frame_t));
		Append_CRC8_Check_Sum(auto_buy_bullet_data_buffer, CRC8_INDEX);                  
		Append_CRC16_Check_Sum(auto_buy_bullet_data_buffer, sizeof(client_interactive_frame_t)); 
		HAL_UART_Transmit(&huart2,auto_buy_bullet_data_buffer,sizeof(client_interactive_frame_t),0xff);
		
		Keyboard_movement_step++;
		HAL_Delay(delay_time);
	}
	else if(Keyboard_movement_step==1)
	{
		client_interactive_frame->std_frame_header_.seq++;
		client_interactive_frame->custom_client_data.key_value1=0;
		client_interactive_frame->custom_client_data.key_value2=0;
		client_interactive_frame->custom_client_data.x_position=0;
		client_interactive_frame->custom_client_data.y_position=0;
		client_interactive_frame->custom_client_data.mouse_left=0;
		client_interactive_frame->custom_client_data.mouse_right=0;
		memcpy(auto_buy_bullet_data_buffer,client_interactive_frame,sizeof(client_interactive_frame_t));
		Append_CRC8_Check_Sum(auto_buy_bullet_data_buffer, CRC8_INDEX);                  
		Append_CRC16_Check_Sum(auto_buy_bullet_data_buffer, sizeof(client_interactive_frame_t)); 
		HAL_UART_Transmit(&huart2,auto_buy_bullet_data_buffer,sizeof(client_interactive_frame_t),0xff);
		
		Keyboard_movement_step=0;
		*command_start_flag=0;
		HAL_Delay(delay_time);
	}
	
}
/**
* @brief 长按2s后松开按键
* @note  鼠标默认在（0，0）;
* @parament uint8_t* command_start_flag 给其传入1，自动执行一次，执行完自动置0； 
*			uint8_t key_value 传入通用键值； 
*/
void Keyboard_Long_Press_work (client_interactive_frame_t *client_interactive_frame,uint8_t key1_value,uint8_t key2_value,uint8_t* command_start_flag)
{
	const uint16_t delay_time=35;
	static uint8_t Keyboard_movement_step;//步骤
	
	if(*command_start_flag==0)
	{
		//复位
		Keyboard_movement_step=0;
		return;
	}
	if(Keyboard_movement_step==0)
	{
		client_interactive_frame->std_frame_header_.seq++;
		client_interactive_frame->custom_client_data.key_value1=key1_value;
		client_interactive_frame->custom_client_data.key_value2=key2_value;
		client_interactive_frame->custom_client_data.x_position=0;
		client_interactive_frame->custom_client_data.y_position=0;
		client_interactive_frame->custom_client_data.mouse_left=0;
		client_interactive_frame->custom_client_data.mouse_right=0;
		memcpy(auto_buy_bullet_data_buffer,client_interactive_frame,sizeof(client_interactive_frame_t));
		Append_CRC8_Check_Sum(auto_buy_bullet_data_buffer, CRC8_INDEX);                  
		Append_CRC16_Check_Sum(auto_buy_bullet_data_buffer, sizeof(client_interactive_frame_t)); 
		HAL_UART_Transmit(&huart2,auto_buy_bullet_data_buffer,sizeof(client_interactive_frame_t),0xff);
		
		Keyboard_movement_step++;
		HAL_Delay(2000);
	}
	else if(Keyboard_movement_step==1)
	{
		client_interactive_frame->std_frame_header_.seq++;
		client_interactive_frame->custom_client_data.key_value1=0;
		client_interactive_frame->custom_client_data.key_value2=0;
		client_interactive_frame->custom_client_data.x_position=0;
		client_interactive_frame->custom_client_data.y_position=0;
		client_interactive_frame->custom_client_data.mouse_left=0;
		client_interactive_frame->custom_client_data.mouse_right=0;
		memcpy(auto_buy_bullet_data_buffer,client_interactive_frame,sizeof(client_interactive_frame_t));
		Append_CRC8_Check_Sum(auto_buy_bullet_data_buffer, CRC8_INDEX);                  
		Append_CRC16_Check_Sum(auto_buy_bullet_data_buffer, sizeof(client_interactive_frame_t)); 
		HAL_UART_Transmit(&huart2,auto_buy_bullet_data_buffer,sizeof(client_interactive_frame_t),0xff);
		
		Keyboard_movement_step=0;
		*command_start_flag=0;
		HAL_Delay(delay_time);
	}
	
}

uint16_t x_position_0=1150;//第三挡x1150 y580 第四档（最多弹）x1200  第二档x1100 第一档x1050

/**
* @brief 一键买弹动作，含有清标志位
* @parament uint8_t bullet_num_level 1~4  传入4最多子弹
*/
uint16_t x_position_1=960;//第一次确定
	uint16_t y_position_1=680;
	uint16_t x_position_2=900;//第二次确定
	uint16_t y_position_2=580;
	uint16_t bullet_x_position;
void Buy_bullet(uint8_t bullet_num_level)
{
	/*初始化坐标 Start*/
	 
	const uint16_t bullet_y_position=580;
	if(bullet_num_level==4)
	{
		bullet_x_position=1200;
	}
	else if(bullet_num_level==3)
	{
		bullet_x_position=1150;
	}
	else if(bullet_num_level==2)
	{
		bullet_x_position=1100;
	}
	else if(bullet_num_level==1)
	{
		bullet_x_position=1050;
	}
	else
	{
		Interactive_Class.command_running_state=COMMAND_SLEEP;
	}
	/*初始化坐标 End*/
	
	if(Interactive_Class.auto_buy_bullet_state==auto_buy_bullet_SLEEP)
	{
		Interactive_Class.auto_buy_bullet_state=KEY_MOVING;
	}
	if(Interactive_Class.auto_buy_bullet_state==KEY_MOVING)//按下按键
	{
		Interactive_Class.key_running_state=1;
		Keyboard_movement_work(Interactive_Class.client_interactive_frame,O_KEY_VALUE,I_KEY_VALUE,&Interactive_Class.key_running_state);
		if(Interactive_Class.key_running_state==0)
		{
			Interactive_Class.auto_buy_bullet_state=MOUSE_MOVING_SELECT;
		}
	}
	
	if(Interactive_Class.auto_buy_bullet_state==MOUSE_MOVING_SELECT)//移动鼠标并点击
	{
		Interactive_Class.mouse_running_state=1;
		Mouse_movement_work(Interactive_Class.client_interactive_frame,bullet_x_position,bullet_y_position,1,0,&Interactive_Class.mouse_running_state);
		if(Interactive_Class.mouse_running_state==0)
		{
			Interactive_Class.auto_buy_bullet_state=MOUSE_MOVING_YES1;
		}
		
		
	}
	if(Interactive_Class.auto_buy_bullet_state==MOUSE_MOVING_YES1)//移动鼠标并点击
	{
		Interactive_Class.mouse_running_state=1;
		Mouse_movement_work(Interactive_Class.client_interactive_frame,x_position_1,y_position_1,1,0,&Interactive_Class.mouse_running_state);
		if(Interactive_Class.mouse_running_state==0)
		{
			Interactive_Class.auto_buy_bullet_state=MOUSE_MOVING_YES2;
		}
	}
	if(Interactive_Class.auto_buy_bullet_state==MOUSE_MOVING_YES2)//移动鼠标并点击
	{
		Interactive_Class.mouse_running_state=1;
		Mouse_movement_work(Interactive_Class.client_interactive_frame,x_position_2,y_position_2,1,0,&Interactive_Class.mouse_running_state);
		if(Interactive_Class.mouse_running_state==0)
		{
			Interactive_Class.auto_buy_bullet_state=KEY_CLOSE_BUY;
		}
	}
	
	if(Interactive_Class.auto_buy_bullet_state==KEY_CLOSE_BUY)//按下按键
	{
		Interactive_Class.key_running_state=1;
		Keyboard_movement_work(Interactive_Class.client_interactive_frame,O_KEY_VALUE,I_KEY_VALUE,&Interactive_Class.key_running_state);
		if(Interactive_Class.key_running_state==0)
		{
			Interactive_Class.auto_buy_bullet_state=auto_buy_bullet_SLEEP;
		}
	}
	if(Interactive_Class.auto_buy_bullet_state==auto_buy_bullet_SLEEP)
	{
		Interactive_Class.command_running_state=COMMAND_SLEEP;//等待触发
	}
}
/**
* @brief 一键进入部署模式，同时按下KL
* @parament uint8_t bullet_num_level 1~4  传入4最多子弹
*/
void Deploy(void)
{
	Interactive_Class.key_running_state=1;
	Keyboard_Long_Press_work(Interactive_Class.client_interactive_frame,L_KEY_VALUE,K_KEY_VALUE,&Interactive_Class.key_running_state);
	Interactive_Class.command_running_state=COMMAND_SLEEP;
}







