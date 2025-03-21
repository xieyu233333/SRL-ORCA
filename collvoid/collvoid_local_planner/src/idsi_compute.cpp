//-----------------------------------------------------------------------------
//	Version 1.0
//-----------------------------------------------------------------------------
//	File Name: idsi_compute_1.cpp
//-----------------------------------------------------------------------------
//	CopyRight (C) 2023, TigherLab.USTC
//-----------------------------------------------------------------------------
//	Modifier	Date		Detail
//
//	qinjianmin		20230307	create
//-----------------------------------------------------------------------------
//	Note:   IDSI威胁评估程序
//-----------------------------------------------------------------------------

#include <iostream>
#include <vector>
#include <cmath>
//#include "IDSI.h"
#include "visual_field.h"
using namespace std;
using std::vector;

//最小Pr值
float MinPr_c = 0.1;

//单向观测，Interact_Pr值缩小倍数
float MinPr_e = 0.05;

//IDSI相关超参数
//以下参数--参照清华大学王建强的论文
float SPE_K = 0.5;

//障碍物和机器人有相同的SPE_R
float SPE_R = 1.0;

float SPE_k1 = 1.0;

float SPE_k3 = 45.0;

float SPE_a = 0.06;//英文论文是0.06是否过小？？？

//需要自己设定
float SPE_k4 = 2.0;//暂定1.5-2.0之间较好？？根据公式？？王建强论文中是 k2 = 1.2



//等价的相当于main函数
void main{

	//更新IDSI数值
	if (Timer() = true)
	{
		//更新机器人优先级--从ros发送数据中读取
		idsi_Lidar_Update();

		//计算机器人IDSI数据
		IDSI_Compute();
	}
}


//idsi相关接口的初始化
void idsi_init()
{
	//数据初始化--开始
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//激光雷达的真实数量--由ros输入--初始化
	Num_Lidar = 10;//关注的雷达线数--------------------------------------------------------暂定10个，总40
		
	//初始化
    //安全机器人雷达数据：横--序号，0-9  竖--一定角度范围内，的机器人激光雷达数据，建议所有机器人的全部统一
	SafeyLidar_Data[Num_Lidar] = { 1.0, 1.0, 1.0, 1.0, 1.0,    1.0, 1.0, 1.0, 1.0, 1.0};
	//S_AHV的雷达探测长度-----------------------------------------------------------暂定1.0，待调整


	//真实雷达数据初始化--赋为0
	//对于第i个机器人
    for (int i = 0; i < 10; i++)//Num_Robot
	{
		//遍历激光雷达数据
		for (int j = 0; j < Num_Lidar; j++)
		{
			//第i个机器人，第j个激光数据
			//从ros发送数据中读取
			idsi_Lidar_Data[i][j] = 0;//ros
		}
	}



	Base_Pr[10] = { 0.2, 0.2, 0.2, 0.2, 0.2,   0.2, 0.2, 0.2, 0.2, 0.2 };//基础优先级赋初值------------------------------------------------暂定0.2，待调整
	
	RealTime_Pr[10] = { 0.2, 0.2, 0.2, 0.2, 0.2,   0.2, 0.2, 0.2, 0.2, 0.2 };//优先级赋初值--------------------------------------------暂时全部统一和Base_Pr[10]一样？？


	//机器人0-9 实时优先级
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			Interact_Pr[i][j] = 0.2;//实时优先级赋初值---------------------------------------------------------赋值为0？？还是和基础优先级一样？？
		}
	}


	//机器人safety potential energy--安全势能-数组,float
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			SPE_V[i][j] = 0;//---------------------------------------------------------赋初始值为0f??？ 是否可行？？？
		}
	}


	//机器人--交互场功率-数组：横--机器人序号，0-9,float
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			Power_I[i][j] = 0;//---------------------------------------------------------赋初始值为0f??？ 是否可行？？？
		}
	}


	//机器人interactive driving safety index--交互驾驶安全指数-数组：横--机器人序号，0-9,float
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			IDSI_T[i][j] = 0;//---------------------------------------------------------赋初始值为0f??？ 是否可行？？？
		}
	}

	//机器人观测范围障碍物数量，根据Observe_index[10][10]检索而来
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		Observe_Num[i] = 0;//---------------------------------------------------------赋初始值为0??？ 是否可行？？？是否有用？？		
	}


	//IDSI顺序数组----根据IDSI对障碍物威胁程度进行排序
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			Rank_IDSI_Index[i][j] = -1;//---------------------------------------------------------赋初始值为 -1 ??？ 是否可行？？？
		}
	}


	//数据初始化--结束
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
}



//计算机器人IDSI数据
void IDSI_Compute()
{
	//更新机器人实时优先级
	RealTime_Pr_Update();

	//更新机器人交互优先级
	Interact_Pr_Update();

	//逐个计算机器人IDSI----interactive driving safety index--交互驾驶安全指数--
	//对于第i个机器人
	for (int i = 0; i < Num_Robot; i++)
	{
		//对于第j个障碍物
		for (int j = 0; j < Num_Robot; j++)
		{
			//如果第i个机器人，能够观测到第j个障碍物
			if (Observe_index[i][j] == true)
			{
				//机器人interactive driving safety index--交互驾驶安全指数--赋值
				//int Index_R, int Index_O, float _R_IAPr, float _O_IAPr, float _R_Px, float _R_Py, float _R_Vx, float _R_Vy, float _O_Px, float _O_Py, float _O_Vx, float _O_Vy
				IDSI_T[i][j] = IDSI_Update(i, j, Interact_Pr[i][j], Interact_Pr[j][i], Robot_Data[i][0], Robot_Data[i][1], Robot_Data[i][2], Robot_Data[i][3], Robot_Data[j][0], Robot_Data[j][1], Robot_Data[j][2], Robot_Data[j][3]);
			}
			else
			{
				IDSI_T[i][j] = 0;
			}

		}
	}

	//IDSI排序----根据IDSI对障碍物威胁程度进行排序
	IDSI_Sort();	
}


//更新机器人优先级--从ros发送数据中读取
void idsi_Lidar_Update()
{
	//对于第i个机器人
	for (int i = 0; i < Num_Robot; i++)
	{
		//遍历激光雷达数据
		for (int j = 0; j < Num_Lidar; j++)
		{
			//第i个机器人，第j个激光数据
			//从ros发送数据中读取
			idsi_Lidar_Data[i][j] = //ros;
		}
	}
}


//更新机器人实时优先级
void RealTime_Pr_Update()
{
	//对于第i个机器人
	for (int i = 0; i < Num_Robot; i++)
	{
		//不安全线数量
		int UnsafeLidar_Num = 0;

		//遍历激光雷达数据
		for (int j = 0; j < Num_Lidar; j++)
		{
			//激光雷达数据小于安全数据--判断
			if (idsi_Lidar_Data[i][j] < SafeyLidar_Data[j])
			{
				UnsafeLidar_Num++;
			}
		}

		//根据不安全线数量，计算优先级Pr，论文公式4
		RealTime_Pr[i] = (Base_Pr[i] - MinPr_c)*(1 - UnsafeLidar_Num / Num_Lidar) + MinPr_c;
	}

}

//更新机器人交互优先级
void Interact_Pr_Update()
{
	//对于第i个机器人
	for (int i = 0; i < Num_Robot; i++)
	{
		//对于第j个障碍物
		for (int j = 0; j < Num_Robot; j++)
		{
			//如果第i个机器人，能够观测到第j个障碍物
			if (Observe_index[i][j] == true)
			{
				//如果第j个障碍物，能够反向观测到第i个机器人
				if (Observe_index[j][i] == true)
				{
					//可互相观测，实时优先级=优先级RealTime_Pr
					Interact_Pr[i][j] = RealTime_Pr[i];
				}
				else
				{
					//单向观测，实时优先级=优先级RealTime_Pr
					Interact_Pr[i][j] = MinPr_e * Base_Pr[i];
				}
			}
			else
			{
				//如果第j个障碍物，可以反向观测到第i个机器人
				if (Observe_index[j][i] == true)
				{
					//i对j不可观测，但j可以观测到i ---i保持自己的，实时优先级=优先级RealTime_Pr ？？？
					Interact_Pr[i][j] = RealTime_Pr[i];
				}
				else
				{
					//双向无法观测，应当怎么样处理？？？？---设定为，实时优先级=优先级RealTime_Pr ？？？实事求是地处理，保障避障安全？？？
					Interact_Pr[i][j] = RealTime_Pr[i];
				}
			}
		}
	}

}


//更新机器人----interactive driving safety index--交互驾驶安全指数
//Index_R表示机器人的编号，_R_IAPr表示机器人的实时优先级
//R_Px代表Robot的x坐标位置，O_Px代表Robot的x坐标位置，以此类推
void float IDSI_Update(int Index_R, int Index_O, float _R_IAPr, float _O_IAPr, float _R_Px, float _R_Py, float _R_Vx, float _R_Vy,  float _O_Px, float _O_Py, float _O_Vx, float _O_Vy)
{

	float RL_ji = 0;

	//相对位置半径R_ji的模长
	RL_ji = sqrt( pow( _R_Px - _O_Px, 2.0) + pow(_R_Py - _O_Py, 2.0) );//障碍物j相对于机器人i的距离,先平方和再开方


	float VL_i = 0;

	//R机器人i速度VL_i的模长
	VL_i = sqrt(pow(_R_Vx, 2.0) + pow(_R_Vy, 2.0));//先平方和再开方




	float VL_j = 0;

	//O障碍物j速度VL_j的模长
	VL_j = sqrt(pow(_O_Vx, 2.0f) + pow(_O_Vy, 2.0));//先平方和再开方

	float O_Qj = 0;

	//O_Qj,表示O障碍物j的速度方向V_j和相对位置半径R_ji的夹角，论文公式8
	O_Qj = [ (_R_Px - _O_Px)*_O_Vx + (_R_Py - _O_Py)*_O_Vy ]/( RL_ji*VL_j );



	float SPE_Vji_aa = 0;

	//动能场的SPE的后半部分，为了简化计算，论文公式8
	SPE_Vji_aa = pow( SPE_k3 - VL_j * cos(O_Qj), 1.0 - SPE_k1 ) / (SPE_k3 - VL_j);

	float SPE_Vji = 0;

	//动能场的SPE，论文公式8
	SPE_Vji = (SPE_K* SPE_R*SPE_R * _R_IAPr*_O_IAPr * SPE_k3) * pow(SPE_Vji_aa, 1.0 / SPE_k1) / [ (SPE_k1 - 1.0)*pow(RL_ji, SPE_k1 - 1.0) ];

	//传参到，机器人safety potential energy--安全势能-数组
	SPE_V[Index_R][Index_O] = SPE_Vji;



	float P_Iji_aa = 0;

	//机器人--交互场功率P_I的后半部分，为了简化计算，论文公式13
	P_Iji_aa = (_R_Px - _O_Px) * (_O_Vx - _R_Vx) + (_R_Py - _O_Py) * (_O_Vy - _R_Vy);

	float P_Iji = 0;

	//机器人--交互场功率P_I，论文公式13
	P_Iji = (SPE_K* SPE_R*SPE_R * _R_IAPr*_O_IAPr) * P_Iji_aa / pow(RL_ji, SPE_k4 + 1.0);

	//传参到，机器人--交互场功率-数组
	Power_I[Index_R][Index_O] = P_Iji;

	

	float IDSI_ji = 0;

	//机器人interactive driving safety index--交互驾驶安全指数,，论文公式10
	IDSI_ji = -1.0 * SPE_a * SPE_Vji + (1.0 - SPE_a) * P_Iji;


	//输出
	return IDSI_ji;
}




//更新机器人观测范围障碍物数量，根据Observe_index[10][10]检索而来
void ObserveNum_Update()
{
	////机器人观测范围障碍物数量，根据Observe_index[10][10]检索而来
	////先清零上一组数据
	//Observe_Num = { 0,0,0,0,0,  0,0,0,0,0 };

	//对于第i个机器人
	for (int i = 0; i < Num_Robot; i++)
	{
		//观测数量计数
		int _Obs_num = 0;

		//对于第j个障碍物
		for (int j = 0; j < Num_Robot; j++)
		{
			//如果可观测
			if (Observe_index[i][j] == true )
			{
				_Obs_num++;
			}
		}

		//赋值给观测数量
		Observe_Num[i] = _Obs_num;
	}
}


//IDSI排序----根据IDSI对障碍物威胁程度进行排序
void IDSI_Sort()
{
	//根据各个IDSI值
	//对于第i个机器人
	for (int i = 0; i < Num_Robot; i++)
	{
		//观测障碍物IDSI数值表--长度由Observe_Num[i]确定
		float _Obs_IDSI[ Observe_Num[i] ];

		//观测障碍物 编号列表--长度由Observe_Num[i]确定
		int _Obs_Index[Observe_Num[i]];


		//_Obs_Index障碍物编号
		int _Obs_Num = 0;

		//对于第j个障碍物
		for (int j = 0; j < Num_Robot; j++)
		{
			//如果可观测
			if (Observe_index[i][j] == true)
			{
				//赋值为对应的--交互驾驶安全指数IDSI数值
				_Obs_IDSI[_Obs_Num] = IDSI_T[i][j];

				//观测障碍物 编号列表--赋值为对应编号
				_Obs_Index[Observe_Num[i]] = j;

				_Obs_Num++;
			}
		}


		//冒泡排序-上浮 根据IDSI的数值大小，从大到小排序，输出排序结果
		//vector<int>& nums, vector<int>& index
		vector <int> _Index_aa = bubbleSort(vector<int>& nums, vector<int>& index);


		//_Obs_Num_X障碍物编号，对应于 _Obs_Index
		int _Obs_Num_X = _Obs_Num;

		//IDSI顺序数组----根据IDSI对障碍物威胁程度,逆向进行排序赋值
		//对于第j个障碍物
		for (int j = 0; j < Num_Robot; j++)
		{
			//如果可观测
			if (Observe_index[i][j] == true)
			{
				//赋值到---IDSI顺序数组----
				Rank_IDSI_Index[i][j] = _Index_aa[_Obs_Num_X];

				_Obs_Num_X--;
			}
			else
			{
				//赋值到---IDSI顺序数组----观测不到即为-1
				Rank_IDSI_Index[i][j] = -1;
			}
		}


	}
}



//冒泡排序-上浮 从大到小排序
vector <int> bubbleSort(vector<int>& nums, vector<int>& index)//nums必须和index有相同的长度，index为动态障碍物编号
{
	//i记录参与比较的起始位置
	for (int i = 0; i < nums.size(); ++i)
	{
		bool isSwap = false;//如果isSwap为true，则循环有交换操作
		for (int j = nums.size() - 1; j > i; --j)
		{
			//逆序则交换
			if (nums[j] > nums[j - 1]) {
				swap(nums[j], nums[j - 1]);

				//同步交换index编号
				swap(index[j], index[j - 1]);

				isSwap = true;
			}
		}
		//此轮循环已经有序，不需要再遍历
		if (isSwap == false) 
		{
			break;
		}
	}

	//返回数组编号的值
	return index;
}


