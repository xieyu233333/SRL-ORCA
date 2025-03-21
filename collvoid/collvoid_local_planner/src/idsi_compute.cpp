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
//	Note:   IDSI��в��������
//-----------------------------------------------------------------------------

#include <iostream>
#include <vector>
#include <cmath>
//#include "IDSI.h"
#include "visual_field.h"
using namespace std;
using std::vector;

//��СPrֵ
float MinPr_c = 0.1;

//����۲⣬Interact_Prֵ��С����
float MinPr_e = 0.05;

//IDSI��س�����
//���²���--�����廪��ѧ����ǿ������
float SPE_K = 0.5;

//�ϰ���ͻ���������ͬ��SPE_R
float SPE_R = 1.0;

float SPE_k1 = 1.0;

float SPE_k3 = 45.0;

float SPE_a = 0.06;//Ӣ��������0.06�Ƿ��С������

//��Ҫ�Լ��趨
float SPE_k4 = 2.0;//�ݶ�1.5-2.0֮��Ϻã������ݹ�ʽ��������ǿ�������� k2 = 1.2



//�ȼ۵��൱��main����
void main{

	//����IDSI��ֵ
	if (Timer() = true)
	{
		//���»��������ȼ�--��ros���������ж�ȡ
		idsi_Lidar_Update();

		//���������IDSI����
		IDSI_Compute();
	}
}


//idsi��ؽӿڵĳ�ʼ��
void idsi_init()
{
	//���ݳ�ʼ��--��ʼ
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//�����״����ʵ����--��ros����--��ʼ��
	Num_Lidar = 10;//��ע���״�����--------------------------------------------------------�ݶ�10������40
		
	//��ʼ��
    //��ȫ�������״����ݣ���--��ţ�0-9  ��--һ���Ƕȷ�Χ�ڣ��Ļ����˼����״����ݣ��������л����˵�ȫ��ͳһ
	SafeyLidar_Data[Num_Lidar] = { 1.0, 1.0, 1.0, 1.0, 1.0,    1.0, 1.0, 1.0, 1.0, 1.0};
	//S_AHV���״�̽�ⳤ��-----------------------------------------------------------�ݶ�1.0��������


	//��ʵ�״����ݳ�ʼ��--��Ϊ0
	//���ڵ�i��������
    for (int i = 0; i < 10; i++)//Num_Robot
	{
		//���������״�����
		for (int j = 0; j < Num_Lidar; j++)
		{
			//��i�������ˣ���j����������
			//��ros���������ж�ȡ
			idsi_Lidar_Data[i][j] = 0;//ros
		}
	}



	Base_Pr[10] = { 0.2, 0.2, 0.2, 0.2, 0.2,   0.2, 0.2, 0.2, 0.2, 0.2 };//�������ȼ�����ֵ------------------------------------------------�ݶ�0.2��������
	
	RealTime_Pr[10] = { 0.2, 0.2, 0.2, 0.2, 0.2,   0.2, 0.2, 0.2, 0.2, 0.2 };//���ȼ�����ֵ--------------------------------------------��ʱȫ��ͳһ��Base_Pr[10]һ������


	//������0-9 ʵʱ���ȼ�
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			Interact_Pr[i][j] = 0.2;//ʵʱ���ȼ�����ֵ---------------------------------------------------------��ֵΪ0�������Ǻͻ������ȼ�һ������
		}
	}


	//������safety potential energy--��ȫ����-����,float
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			SPE_V[i][j] = 0;//---------------------------------------------------------����ʼֵΪ0f??�� �Ƿ���У�����
		}
	}


	//������--����������-���飺��--��������ţ�0-9,float
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			Power_I[i][j] = 0;//---------------------------------------------------------����ʼֵΪ0f??�� �Ƿ���У�����
		}
	}


	//������interactive driving safety index--������ʻ��ȫָ��-���飺��--��������ţ�0-9,float
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			IDSI_T[i][j] = 0;//---------------------------------------------------------����ʼֵΪ0f??�� �Ƿ���У�����
		}
	}

	//�����˹۲ⷶΧ�ϰ�������������Observe_index[10][10]��������
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		Observe_Num[i] = 0;//---------------------------------------------------------����ʼֵΪ0??�� �Ƿ���У������Ƿ����ã���		
	}


	//IDSI˳������----����IDSI���ϰ�����в�̶Ƚ�������
	for (int i = 0; i < 10; i++)//Num_Robot
	{
		for (int j = 0; j < 10; j++)
		{
			Rank_IDSI_Index[i][j] = -1;//---------------------------------------------------------����ʼֵΪ -1 ??�� �Ƿ���У�����
		}
	}


	//���ݳ�ʼ��--����
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
}



//���������IDSI����
void IDSI_Compute()
{
	//���»�����ʵʱ���ȼ�
	RealTime_Pr_Update();

	//���»����˽������ȼ�
	Interact_Pr_Update();

	//������������IDSI----interactive driving safety index--������ʻ��ȫָ��--
	//���ڵ�i��������
	for (int i = 0; i < Num_Robot; i++)
	{
		//���ڵ�j���ϰ���
		for (int j = 0; j < Num_Robot; j++)
		{
			//�����i�������ˣ��ܹ��۲⵽��j���ϰ���
			if (Observe_index[i][j] == true)
			{
				//������interactive driving safety index--������ʻ��ȫָ��--��ֵ
				//int Index_R, int Index_O, float _R_IAPr, float _O_IAPr, float _R_Px, float _R_Py, float _R_Vx, float _R_Vy, float _O_Px, float _O_Py, float _O_Vx, float _O_Vy
				IDSI_T[i][j] = IDSI_Update(i, j, Interact_Pr[i][j], Interact_Pr[j][i], Robot_Data[i][0], Robot_Data[i][1], Robot_Data[i][2], Robot_Data[i][3], Robot_Data[j][0], Robot_Data[j][1], Robot_Data[j][2], Robot_Data[j][3]);
			}
			else
			{
				IDSI_T[i][j] = 0;
			}

		}
	}

	//IDSI����----����IDSI���ϰ�����в�̶Ƚ�������
	IDSI_Sort();	
}


//���»��������ȼ�--��ros���������ж�ȡ
void idsi_Lidar_Update()
{
	//���ڵ�i��������
	for (int i = 0; i < Num_Robot; i++)
	{
		//���������״�����
		for (int j = 0; j < Num_Lidar; j++)
		{
			//��i�������ˣ���j����������
			//��ros���������ж�ȡ
			idsi_Lidar_Data[i][j] = //ros;
		}
	}
}


//���»�����ʵʱ���ȼ�
void RealTime_Pr_Update()
{
	//���ڵ�i��������
	for (int i = 0; i < Num_Robot; i++)
	{
		//����ȫ������
		int UnsafeLidar_Num = 0;

		//���������״�����
		for (int j = 0; j < Num_Lidar; j++)
		{
			//�����״�����С�ڰ�ȫ����--�ж�
			if (idsi_Lidar_Data[i][j] < SafeyLidar_Data[j])
			{
				UnsafeLidar_Num++;
			}
		}

		//���ݲ���ȫ���������������ȼ�Pr�����Ĺ�ʽ4
		RealTime_Pr[i] = (Base_Pr[i] - MinPr_c)*(1 - UnsafeLidar_Num / Num_Lidar) + MinPr_c;
	}

}

//���»����˽������ȼ�
void Interact_Pr_Update()
{
	//���ڵ�i��������
	for (int i = 0; i < Num_Robot; i++)
	{
		//���ڵ�j���ϰ���
		for (int j = 0; j < Num_Robot; j++)
		{
			//�����i�������ˣ��ܹ��۲⵽��j���ϰ���
			if (Observe_index[i][j] == true)
			{
				//�����j���ϰ���ܹ�����۲⵽��i��������
				if (Observe_index[j][i] == true)
				{
					//�ɻ���۲⣬ʵʱ���ȼ�=���ȼ�RealTime_Pr
					Interact_Pr[i][j] = RealTime_Pr[i];
				}
				else
				{
					//����۲⣬ʵʱ���ȼ�=���ȼ�RealTime_Pr
					Interact_Pr[i][j] = MinPr_e * Base_Pr[i];
				}
			}
			else
			{
				//�����j���ϰ�����Է���۲⵽��i��������
				if (Observe_index[j][i] == true)
				{
					//i��j���ɹ۲⣬��j���Թ۲⵽i ---i�����Լ��ģ�ʵʱ���ȼ�=���ȼ�RealTime_Pr ������
					Interact_Pr[i][j] = RealTime_Pr[i];
				}
				else
				{
					//˫���޷��۲⣬Ӧ����ô������������---�趨Ϊ��ʵʱ���ȼ�=���ȼ�RealTime_Pr ������ʵ�����ǵش������ϱ��ϰ�ȫ������
					Interact_Pr[i][j] = RealTime_Pr[i];
				}
			}
		}
	}

}


//���»�����----interactive driving safety index--������ʻ��ȫָ��
//Index_R��ʾ�����˵ı�ţ�_R_IAPr��ʾ�����˵�ʵʱ���ȼ�
//R_Px����Robot��x����λ�ã�O_Px����Robot��x����λ�ã��Դ�����
void float IDSI_Update(int Index_R, int Index_O, float _R_IAPr, float _O_IAPr, float _R_Px, float _R_Py, float _R_Vx, float _R_Vy,  float _O_Px, float _O_Py, float _O_Vx, float _O_Vy)
{

	float RL_ji = 0;

	//���λ�ð뾶R_ji��ģ��
	RL_ji = sqrt( pow( _R_Px - _O_Px, 2.0) + pow(_R_Py - _O_Py, 2.0) );//�ϰ���j����ڻ�����i�ľ���,��ƽ�����ٿ���


	float VL_i = 0;

	//R������i�ٶ�VL_i��ģ��
	VL_i = sqrt(pow(_R_Vx, 2.0) + pow(_R_Vy, 2.0));//��ƽ�����ٿ���




	float VL_j = 0;

	//O�ϰ���j�ٶ�VL_j��ģ��
	VL_j = sqrt(pow(_O_Vx, 2.0f) + pow(_O_Vy, 2.0));//��ƽ�����ٿ���

	float O_Qj = 0;

	//O_Qj,��ʾO�ϰ���j���ٶȷ���V_j�����λ�ð뾶R_ji�ļнǣ����Ĺ�ʽ8
	O_Qj = [ (_R_Px - _O_Px)*_O_Vx + (_R_Py - _O_Py)*_O_Vy ]/( RL_ji*VL_j );



	float SPE_Vji_aa = 0;

	//���ܳ���SPE�ĺ�벿�֣�Ϊ�˼򻯼��㣬���Ĺ�ʽ8
	SPE_Vji_aa = pow( SPE_k3 - VL_j * cos(O_Qj), 1.0 - SPE_k1 ) / (SPE_k3 - VL_j);

	float SPE_Vji = 0;

	//���ܳ���SPE�����Ĺ�ʽ8
	SPE_Vji = (SPE_K* SPE_R*SPE_R * _R_IAPr*_O_IAPr * SPE_k3) * pow(SPE_Vji_aa, 1.0 / SPE_k1) / [ (SPE_k1 - 1.0)*pow(RL_ji, SPE_k1 - 1.0) ];

	//���ε���������safety potential energy--��ȫ����-����
	SPE_V[Index_R][Index_O] = SPE_Vji;



	float P_Iji_aa = 0;

	//������--����������P_I�ĺ�벿�֣�Ϊ�˼򻯼��㣬���Ĺ�ʽ13
	P_Iji_aa = (_R_Px - _O_Px) * (_O_Vx - _R_Vx) + (_R_Py - _O_Py) * (_O_Vy - _R_Vy);

	float P_Iji = 0;

	//������--����������P_I�����Ĺ�ʽ13
	P_Iji = (SPE_K* SPE_R*SPE_R * _R_IAPr*_O_IAPr) * P_Iji_aa / pow(RL_ji, SPE_k4 + 1.0);

	//���ε���������--����������-����
	Power_I[Index_R][Index_O] = P_Iji;

	

	float IDSI_ji = 0;

	//������interactive driving safety index--������ʻ��ȫָ��,�����Ĺ�ʽ10
	IDSI_ji = -1.0 * SPE_a * SPE_Vji + (1.0 - SPE_a) * P_Iji;


	//���
	return IDSI_ji;
}




//���»����˹۲ⷶΧ�ϰ�������������Observe_index[10][10]��������
void ObserveNum_Update()
{
	////�����˹۲ⷶΧ�ϰ�������������Observe_index[10][10]��������
	////��������һ������
	//Observe_Num = { 0,0,0,0,0,  0,0,0,0,0 };

	//���ڵ�i��������
	for (int i = 0; i < Num_Robot; i++)
	{
		//�۲���������
		int _Obs_num = 0;

		//���ڵ�j���ϰ���
		for (int j = 0; j < Num_Robot; j++)
		{
			//����ɹ۲�
			if (Observe_index[i][j] == true )
			{
				_Obs_num++;
			}
		}

		//��ֵ���۲�����
		Observe_Num[i] = _Obs_num;
	}
}


//IDSI����----����IDSI���ϰ�����в�̶Ƚ�������
void IDSI_Sort()
{
	//���ݸ���IDSIֵ
	//���ڵ�i��������
	for (int i = 0; i < Num_Robot; i++)
	{
		//�۲��ϰ���IDSI��ֵ��--������Observe_Num[i]ȷ��
		float _Obs_IDSI[ Observe_Num[i] ];

		//�۲��ϰ��� ����б�--������Observe_Num[i]ȷ��
		int _Obs_Index[Observe_Num[i]];


		//_Obs_Index�ϰ�����
		int _Obs_Num = 0;

		//���ڵ�j���ϰ���
		for (int j = 0; j < Num_Robot; j++)
		{
			//����ɹ۲�
			if (Observe_index[i][j] == true)
			{
				//��ֵΪ��Ӧ��--������ʻ��ȫָ��IDSI��ֵ
				_Obs_IDSI[_Obs_Num] = IDSI_T[i][j];

				//�۲��ϰ��� ����б�--��ֵΪ��Ӧ���
				_Obs_Index[Observe_Num[i]] = j;

				_Obs_Num++;
			}
		}


		//ð������-�ϸ� ����IDSI����ֵ��С���Ӵ�С�������������
		//vector<int>& nums, vector<int>& index
		vector <int> _Index_aa = bubbleSort(vector<int>& nums, vector<int>& index);


		//_Obs_Num_X�ϰ����ţ���Ӧ�� _Obs_Index
		int _Obs_Num_X = _Obs_Num;

		//IDSI˳������----����IDSI���ϰ�����в�̶�,�����������ֵ
		//���ڵ�j���ϰ���
		for (int j = 0; j < Num_Robot; j++)
		{
			//����ɹ۲�
			if (Observe_index[i][j] == true)
			{
				//��ֵ��---IDSI˳������----
				Rank_IDSI_Index[i][j] = _Index_aa[_Obs_Num_X];

				_Obs_Num_X--;
			}
			else
			{
				//��ֵ��---IDSI˳������----�۲ⲻ����Ϊ-1
				Rank_IDSI_Index[i][j] = -1;
			}
		}


	}
}



//ð������-�ϸ� �Ӵ�С����
vector <int> bubbleSort(vector<int>& nums, vector<int>& index)//nums�����index����ͬ�ĳ��ȣ�indexΪ��̬�ϰ�����
{
	//i��¼����Ƚϵ���ʼλ��
	for (int i = 0; i < nums.size(); ++i)
	{
		bool isSwap = false;//���isSwapΪtrue����ѭ���н�������
		for (int j = nums.size() - 1; j > i; --j)
		{
			//�����򽻻�
			if (nums[j] > nums[j - 1]) {
				swap(nums[j], nums[j - 1]);

				//ͬ������index���
				swap(index[j], index[j - 1]);

				isSwap = true;
			}
		}
		//����ѭ���Ѿ����򣬲���Ҫ�ٱ���
		if (isSwap == false) 
		{
			break;
		}
	}

	//���������ŵ�ֵ
	return index;
}


