#ifndef FFT_H
#define FFT_H
/**
  ******************************************************************************
  * @file           : fft.h
  * @brief          : �����źŲ�����FFT������AMFM�������
	* @author					: wjx 2219
	* @date						: 2024_7_23	
  ******************************************************************************
	* @attention
  *	Ϊ���FFT���� ʹ��si5351�ⲿʱ��ģ���ṩ����ʱ�ӣ�FFTΪ4096��
  *
  ******************************************************************************
ʹ��ʾ����
	fft_set_fs(1.024*MHz)�����ò���Ƶ��Ϊ1024KHz
	fft_run_sig_measure()�����е���fft���� ����ӡfft���
	
	AMFM���������ʹ��ǰ����fft_measure()����fft�������������з�����ֵ��ſ���ʹ�����º���
	fft_FM_decoder(400,4)�������ķ���400�㡢����Ϊ4���fft�����������FM��� ���ص���ϵ��
	fft_AM_decoder(400,4)�������ķ���400�㡢����Ϊ4���fft�����������AM��� ���ص������
*/
#include "main.h"
#include "usart.h"
#include "si5351.h"
#include "tim.h"
#include "adc.h"
#include "delay.h"

#include <math.h>
#include "arm_math.h"
#include "arm_const_structs.h"

void fft_set_fs(float32_t fs);
uint32_t fft_run_sig_measure();
void fft_measure();
float32_t fft_FM_decoder(uint32_t center_index,uint32_t peak_gap);
float32_t fft_AM_decoder(uint32_t center_index,uint32_t peak_gap);

//������������� ���ڼ���FM����ϵ��
static float32_t bessel_chart[51][4]={{1.7388857357447036,1.0,0.26111426425529627,0.0444570570211851},
{1.5281765953469804,1.0,0.29000522283483754,0.054564446672136654},
{1.3468743374398338,1.0,0.31979232922683254,0.06597443075610875},
{1.1878512932757463,1.0,0.35061024518579237,0.07880075441782267},
{1.0459590580478253,1.0,0.3826123705236036,0.09317820149601026},
{0.9173582737095044,1.0,0.4159750596238293,0.10926682566354445},
{0.799097062221145,1.0,0.4509029377788548,0.12725734444713707},
{0.6888349068580414,1.0,0.4876356813772527,0.1473780738288297},
{0.5846543424579886,1.0,0.5264567686531223,0.1699039303402719},
{0.4849266620099259,1.0,0.5677049169374421,0.19516824618408918},
{0.38821076556779555,1.0,0.611789234432204,0.22357846886440838},
{0.2931713635453504,1.0,0.6592095888356022,0.2556373120678136},
{0.19850647800800947,1.0,0.7105844310828999,0.29197169287799934},
{0.10287573657004076,1.0,0.7666894808212636,0.3333730101239371},
{0.0048207503184552695,1.0,0.8285125830148784,0.38085430502479695},
{0.09733323374456088,1.0,0.897333233744561,0.43573317399129785},
{0.2056100225529888,1.0,0.9748407917837583,0.4997550642827043},
{0.3225745588259733,1.0,1.0633152995667141,0.5752819252840207},
{0.45162767202834886,1.0,1.1659133863140632,0.6655905518772333},
{0.5974830204455841,1.0,1.2871381928593766,0.7753630246336234},
{0.7669815185904922,1.0,1.433648185257159,0.9115309136762115},
{0.9705677518920357,1.0,1.6157290422146167,1.0848116673736987},
{1.2251633480153168,1.0,1.8501633480153172,1.3127041850191454},
{1.5602776796115565,1.0,2.166338285672163,1.625864588693529},
{2.0326063084324004,1.0,2.6208416025500494,2.0833430618235855},
{2.767030003274795,1.0,3.3384585747033673,2.815381228232419},
{4.103773510781151,1.0,4.659329066336706,4.177032295929677},
{7.415950779251894,1.0,7.956491319792433,7.6016122376134385},
{31.398199694599676,1.0,31.92451548407336,32.60475314112985},
{14.749134874299365,1.0,14.236314361478852,15.60134806305524},
{6.013473605119274,1.0,5.513473605119274,6.513473605119272},
{3.7635074976209903,1.0,3.2757026195722103,4.1958074337289855},
{2.715942007759776,1.0,2.2397515315693,3.1330966967326668},
{2.100164871078648,1.0,1.6350485920088806,2.520975434426865},
{1.687860481176345,1.0,1.2333150266308903,2.1211954787553546},
{1.387266986029987,1.0,0.9428225415855425,1.8380644814093712},
{1.1542956261129051,1.0,0.7195130174172529,1.625663493406307},
{0.9650640650845363,1.0,0.5395321501909193,1.4591762980348248},
{0.805445365441329,1.0,0.38877869877466237,1.3239822489788853},
{0.6664819804819695,1.0,0.2583187151758471,1.210872420551712},
{0.5421492120045363,1.0,0.14214921200453617,1.113719369603629},
{0.42816951964108535,1.0,0.03601265689598741,1.028245221094892},
{0.32133754994309055,1.0,0.06327783467229402,0.951324742559774},
{0.21910894002858783,1.0,0.15824955053744996,0.8805663769528681},
{0.11933031088137422,1.0,0.25104005948899616,0.8140444003785214},
{0.020044239656124438,1.0,0.3435921239802392,0.7501148189234625},
{0.0806707620595154,1.0,0.4378136192023725,0.6872759862840196},
{0.18485404456031132,1.0,0.5357312375427674,0.6240482543559526},
{0.2948372589076906,1.0,0.6396648451145871,0.5588518309554573},
{0.41347273756191594,1.0,0.7524557884093735,0.4898604824343231},
{0.5444670978295446,1.0,0.877800431162878,0.41479971255808135}};
#endif