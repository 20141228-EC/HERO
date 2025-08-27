
#include "Limit.h"


/* Private function prototypes -----------------------------------------------*/
/** 
 *	@brief 复位时总角度复位到0度方便处理
 */
void Limit_reset_0_anglesum()
{
	rm_motor[LIMIT].info->angle_sum=0;
}


