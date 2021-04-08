/*
	�����ӿ�
	���Ľ�20191001
	ע�⣺�������ڲ��洢���ú���ô˽ӿ�
*/

#pragma once

#include <stdint.h>
#include "mavlink.h"
#include "Basic.hpp"

enum PR_RESULT
{
	PR_OK = 0 ,
	PR_Existed ,
	PR_NewParam ,
	PR_TimeOut ,
	PR_ERR ,
};

#define MAX_VOLATILE_PARAMS 65536

/*
	�ɱ䳤������ע�ᣨ��������С�ڵ���16�ֽڣ�
	ע�⣺�����ڲ������ʼ�����ǰ����
	name������������
	param_version�������汾
	param_length��������������
	ram_params_count����ram�пɴ�ŵĲ�������n

	����ֵ��
	PR_ERR���������ʼ��δ���
	PR_Existed: ������������ͬ������
	PR_TimeOut�����ʲ����������ʱ
	PR_NewParam��ע��Ĳ������²������洢����û�У�
	PR_OK��ע��Ĳ����ڴ洢�����ҵ���������param
*/
PR_RESULT VolatileParamGroupRegister( SName name, uint16_t version, 
															uint16_t param_length, uint32_t ram_params_count );

/*
	������ע�ᣨ��������С�ڵ���16�ֽڣ�
	ע�⣺�����ڳ�ʼ�����ǰ���ã�setInitializationCompletedǰ��
	name������������
	param_version�������汾
	params_count����������

	����ֵ��
	PR_ERR���������ʼ��δ���
	PR_Existed: ������������ͬ������
	PR_TimeOut�����ʲ����������ʱ
	PR_NewParam��ע��Ĳ������²������洢����û�У�
	PR_OK��ע��Ĳ����ڴ洢�����ҵ���������param
*/
PR_RESULT ParamGroupRegister( SName name, uint16_t version, uint32_t params_count,
															const MAV_PARAM_TYPE param_types[], const SName param_names[], const uint64_t initial_params[] );

/*
	������ע�ᣨ��������С�ڵ���16�ֽڣ�
	ע�⣺�˺���ֻ��ע�����ڴ洢���еĲ���
	ע�⣺�����ڲ������ʼ�����ǰ����
	name������������
	param_version�������汾
	params_count����������n

	param_names����������*n������ΪNULL������Ҫ��������б�

	����ֵ��
	PR_ERR���������ʼ��δ��ɻ�����ڴ洢���в�����
	PR_Existed: ������������ͬ������
	PR_TimeOut�����ʲ����������ʱ
	PR_NewParam��ע��Ĳ������²������洢����û�У�
	PR_OK��ע��Ĳ����ڴ洢�����ҵ���������param
*/
PR_RESULT ParamGroupRegister( SName name, uint16_t version, uint32_t params_count, const SName param_names[] );

/*
	��ȡ�������������
	ע�⣺�����ڲ������ʼ����ɺ����
	count�����ز�������
����ֵ��
	PR_ERR��δ��ɳ�ʼ��
	PR_OK����ȡ�ɹ�
*/
PR_RESULT GetParametersCount( uint32_t* count );
/*
	���ò������ȡ������
	ע�⣺�����ڲ������ʼ����ɺ����
	count�����ز�������
����ֵ��
	PR_ERR��δ��ɳ�ʼ��
	PR_OK�������ɹ�
*/
PR_RESULT ResetParametersIterator();
/*
	����������
	ע�⣺�����ڲ������ʼ����ɺ����
����ֵ��
	PR_ERR��δ��ɳ�ʼ�����Ѿ���ĩβ
	PR_OK�������ɹ�
*/
PR_RESULT ParameterIteratorMoveNext();
/*
	����ǰ����
	ע�⣺�����ڲ������ʼ����ɺ����	
	name����ȡ�Ĳ�������
	index����ǰ�������
	type����ȡ�Ĳ�������
	value����ȡ�Ĳ�����ֵ
	is_new��������Ϊ�£����ڴ洢����
����ֵ��
	PR_ERR��δ��ɳ�ʼ�����Ѿ���ĩβ
	PR_TimeOut�����ʲ����������ʱ
	PR_OK�������ɹ�
*/
PR_RESULT ReadCurrentParameter( SName* name, uint32_t* index, MAV_PARAM_TYPE* type, uint64_t* value, bool* is_new, double TIMEOUT = -1 );

/*
	��ȡָ�����Ʋ���
	ע�⣺�����ڲ������ʼ����ɺ����
	name����������
	type����ȡ�Ĳ�������
	value��Ҫ��ȡ�Ĳ�����ֵ
	is_new��������Ϊ�£����ڴ洢����

	����ֵ��
	PR_ERR�����������޴˲���
	PR_TimeOut�����ʲ����������ʱ
	PR_OK����ȡ�ɹ�
*/
PR_RESULT ReadParam( SName name, uint32_t* index, MAV_PARAM_TYPE* type, uint64_t* value, bool* is_new, double TIMEOUT = -1 );
/*
	��ȡָ����Ų�������Ч�ʣ�
	ע�⣺�����ڲ������ʼ����ɺ����
	index���������
	name����������
	type����ȡ�Ĳ�������
	value��Ҫ��ȡ�Ĳ�����ֵ
	is_new��������Ϊ�£����ڴ洢����

	����ֵ��
	PR_ERR�����������޴˲���
	PR_TimeOut�����ʲ����������ʱ
	PR_OK����ȡ�ɹ�
*/
PR_RESULT ReadParam( uint32_t index, SName* name, MAV_PARAM_TYPE* type, uint64_t* value, bool* is_new, double TIMEOUT = -1 );

/*
	��ȡָ��������ͨ������
	ע�⣺�����ڲ������ʼ����ɺ���ã�setInitializationCompleted��
	name������������
	data��Ҫ��ȡ�Ĳ����������ֵ

	����ֵ��
	PR_ERR�����������޴˲���
	PR_TimeOut�����ʲ����������ʱ
	PR_OK����ȡ�ɹ�
*/
PR_RESULT ReadParamGroup( SName name, uint64_t data[], bool* is_new, double TIMEOUT = -1 );
/*
	��ȡָ��������ͨ������
	ע�⣺�����ڲ������ʼ����ɺ���ã�setInitializationCompleted��
	name������������
	data��Ҫ��ȡ�Ĳ����������ֵ
	start�������start��ʼ��ȡ��0��ʼ��
	read_count����ȡ��Ŀ

	����ֵ��
	PR_ERR�����������޴˲������ȡ�Ĳ�������������Χ
	PR_TimeOut�����ʲ����������ʱ
	PR_OK����ȡ�ɹ�
*/
PR_RESULT ReadParamGroup( SName name, uint64_t data[], bool* is_new, uint16_t start, uint16_t read_count, double TIMEOUT = -1 );

/*
	��ȡָ�����Ʋ������������
	ע�⣺�����ڲ������ʼ����ɺ����
	name������������
	count����ȡ�Ĳ����鳤��
	TIMEOUT����ʱʱ��

	����ֵ��
	PR_ERR�����������޴˲������ȡ�Ĳ�������������Χ
	PR_TimeOut�����ʲ����������ʱ
	PR_OK���ɹ�
*/
PR_RESULT GetVolatileParamGroupParamCount( SName name, uint16_t* count, double TIMEOUT = -1 );

/*
	��ȡָ�����Ʋ�����
	ע�⣺�����ڲ������ʼ����ɺ����
	name������������
	data��Ҫ��ȡ�Ĳ����������ֵ
	start�������start��ʼ���£�0��ʼ��
	read_count����ȡ��Ŀ
	TIMEOUT����ʱʱ��

	����ֵ��
	PR_ERR�����������޴˲������ȡ�Ĳ�������������Χ
	PR_TimeOut�����ʲ����������ʱ
	PR_OK���ɹ�
*/
PR_RESULT ReadVolatileParamGroup( SName name, void* data, uint16_t start, uint16_t read_count, double TIMEOUT = -1 );

/*
	����ָ�����ƿɱ������
	ע�⣺�����ڲ������ʼ����ɺ����
	name������������
	TIMEOUT����ʱʱ��

	����ֵ��
	PR_ERR�����������޴˲������ȡ�Ĳ�������������Χ
	PR_TimeOut�����ʲ����������ʱ
	PR_OK���ɹ�
*/
PR_RESULT TruncateVolatileParamGroup( SName name, uint16_t size, double TIMEOUT = -1 );

/*
	����ָ�����ƿɱ������
	ע�⣺�����ڲ������ʼ����ɺ����
	name������������
	TIMEOUT����ʱʱ��

	����ֵ��
	PR_ERR�����������޴˲������ȡ�Ĳ�������������Χ
	PR_TimeOut�����ʲ����������ʱ
	PR_OK���ɹ�
*/
PR_RESULT SaveVolatileParamGroup( SName name, double TIMEOUT = -1 );

/*
	����ָ�����ƿɱ������
	ע�⣺�����ڲ������ʼ����ɺ����
	name������������
	data��Ҫ���µĲ����������ֵ
	start�������start��ʼ���£�0��ʼ��
	write_count��������Ŀ
	st���Ƿ�д��洢��
	TIMEOUT����ʱʱ��

	����ֵ��
	PR_ERR�����������޴˲������ȡ�Ĳ�������������Χ
	PR_TimeOut�����ʲ����������ʱ
	PR_OK���ɹ�
*/
PR_RESULT WriteVolatileParamGroup( SName name, const void* data, uint16_t start, uint16_t write_count, bool st, double TIMEOUT = -1 );

/*
	����ָ�����Ʋ�����
	ע�⣺�����ڲ������ʼ����ɺ����
	name������������
	data��Ҫ���µĲ����������ֵ
	start�������start��ʼ���£�0��ʼ��
	write_count��������Ŀ
	st���Ƿ�д��洢��
	TIMEOUT����ʱʱ��

	����ֵ��
	PR_ERR�����������޴˲������ȡ�Ĳ�������������Χ
	PR_TimeOut�����ʲ����������ʱ
	PR_OK���ɹ�
*/
PR_RESULT UpdateParamGroup( SName name, const uint64_t data[], uint16_t start, uint16_t write_count, bool st = true, double TIMEOUT = -1 );

/*
	��ָ�����Ʋ��������ƵĲ������浽�洢��
	ע�⣺�����ڲ������ʼ����ɺ����
	name������������
	TIMEOUT����ʱʱ��

	����ֵ��
	PR_ERR�����������޴˲������ȡ�Ĳ�������������Χ
	PR_TimeOut�����ʲ����������ʱ
	PR_OK������ɹ�
*/
PR_RESULT SaveParamGroup( SName name, double TIMEOUT = -1 );

/*
	����ָ�����Ʋ���
	ע�⣺�����ڲ������ʼ����ɺ���ã�setInitializationCompleted��
	name������������
	data��Ҫ���µĲ����������ֵ
	TIMEOUT����ʱʱ��

	����ֵ��
	PR_ERR�����������޴˲���
	PR_TimeOut�����ʲ����������ʱ
	PR_OK�����³ɹ�
*/
PR_RESULT UpdateParam( SName name, const uint64_t data, double TIMEOUT = -1 );

void init_Parameters();