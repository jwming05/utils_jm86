
/*!
 **************************************************************************************
 * \file
 *    parset.c
 * \brief
 *    Picture and Sequence Parameter set generation and handling
 *	  ͼ������в��������ɺͿ���
 *  \date 25 November 2002
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details) 
 *      - Stephan Wenger        <stewe@cs.tu-berlin.de>
 *
 **************************************************************************************
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <malloc.h>
#include <string.h>
 
#include "global.h"
#include "contributors.h"
#include "parsetcommon.h"
#include "nalu.h"
#include "parset.h"
#include "fmo.h"
#include "vlc.h"
#include "mbuffer.h"

// Local helpers
static int IdentifyProfile();						//��֤����
static int IdentifyLevel();							//��֤����
static int IdentifyNumRefFrames();					//��֤�ο�֡��
static int GenerateVUISequenceParameters();			//����VUI���в���

extern ColocatedParams *Co_located;


/*! 
 *************************************************************************************
 * \brief
 *    generates a sequence and picture parameter set and stores these in global
 *    active_sps and active_pps
 *    ����sps��pps���ұ�����Щȫ�ֻ�ı���
 * \return
 *    A NALU containing the Sequence ParameterSet
 *
 *************************************************************************************
*/
void GenerateParameterSets ()
{
  seq_parameter_set_rbsp_t *sps = NULL;			//spsָ��
  pic_parameter_set_rbsp_t *pps = NULL;			//ppsָ��

  sps = AllocSPS();								//����sps�ռ�
  pps = AllocPPS();								//����pps�ռ�

  FillParameterSetStructures (sps, pps);		//�������
  
  active_sps = sps;
  active_pps = pps;
}

/*! 
*************************************************************************************
* \brief
*    frees global parameter sets active_sps and active_pps
*
* \return
*    A NALU containing the Sequence ParameterSet
*
*************************************************************************************
*/
void FreeParameterSets ()
{
  FreeSPS (active_sps);
  FreePPS (active_pps);
}

/*! 
*************************************************************************************
* \brief
*    int GenerateSeq_parameter_set_NALU ();
*	 ����sps������SODB��������RBSP, ���nalu��Ϣ��RBSP2EBSP��Ϣ������nalu->buf��
* \note
*    Uses the global variables through FillParameterSetStructures()
*
* \return
*    A NALU containing the Sequence ParameterSet
*	 ����һ��sps��ص�nal��Ԫ
*************************************************************************************
*/

NALU_t *GenerateSeq_parameter_set_NALU ()
{
  NALU_t *n = AllocNALU(64000);
  int RBSPlen = 0;
  int NALUlen;
  byte rbsp[MAXRBSPSIZE];

  RBSPlen = GenerateSeq_parameter_set_rbsp (active_sps, rbsp);									//����sps������rbsp��������RBSP����
  NALUlen = RBSPtoNALU (rbsp, n, RBSPlen, NALU_TYPE_SPS, NALU_PRIORITY_HIGHEST, 0, 1);			//��rbsp��Ϣ��ת��nalu����NAL��Ԫ���ȡ�(NALU = NALH+RBSP)
  n->startcodeprefix_len = 4;																	//

  return n;
}


/*! 
*************************************************************************************
* \brief
*    NALU_t *GeneratePic_parameter_set_NALU ();
*
* \note
*    Uses the global variables through FillParameterSetStructures()
*
* \return
*    A NALU containing the Picture Parameter Set
*
*************************************************************************************
*/

NALU_t *GeneratePic_parameter_set_NALU()
{
  NALU_t *n = AllocNALU(64000);
  int RBSPlen = 0;
  int NALUlen;
  byte rbsp[MAXRBSPSIZE];

  RBSPlen = GeneratePic_parameter_set_rbsp (active_pps, rbsp);
  NALUlen = RBSPtoNALU (rbsp, n, RBSPlen, NALU_TYPE_PPS, NALU_PRIORITY_HIGHEST, 0, 1);
  n->startcodeprefix_len = 4;

  return n;
}

/*!
 ************************************************************************
 * \brief
 *    FillParameterSetStructures: extracts info from global variables and
 *    generates a picture and sequence parameter set structure
 *    ��ȫ�ֱ�������ȡ��Ϣ������sps��pps�ṹ��
 *   
 * \param sps
 *    Sequence parameter set to be filled
 * \param pps
 *    Picture parameter set to be filled
 * \par
 *    Function reads all kinds of values from several global variables,
 *    including input-> and image-> and fills in the sps and pps.  Many
 *    values are current hard-coded to defaults, especially most of the
 *    VUI stuff.  Currently, the sps and pps structures are fixed length
 *    with the exception of the fully flexible FMO map (mode 6).  This
 *    mode is not supported.  Hence, the function does not need to
 *    allocate memory for the FMOmap, the pointer slice_group_id is
 *    always NULL.  If one wants to implement FMO mode 6, one would need
 *    to malloc the memory for the map here, and the caller would need
 *    to free it after use.
 *    �ú������Ӽ���ȫ�ֱ���(����input��image)�ж�ȡ���е����ݣ�������sps��pps�����
 *	  
 * \par 
 *    Limitations
 *    Currently, the encoder does not support multiple parameter sets,
 *    primarily because the config file does not support it.  Hence the
 *    pic_parameter_set_id and the seq_parameter_set_id are always zero.
 *    If one day multiple parameter sets are implemented, it would
 *    make sense to break this function into two, one for the picture and
 *    one for the sequence.
 *    Currently, FMO is not supported
 *    The following pps and sps elements seem not to be used in the encoder 
 *    or decoder and, hence, a guessed default value is conveyed:
 *
 *    pps->num_ref_idx_l0_active_minus1 = img->num_ref_pic_active_fwd_minus1;
 *    pps->num_ref_idx_l1_active_minus1 = img->num_ref_pic_active_bwd_minus1;
 *    pps->chroma_qp_index_offset = 0;
 *    sps->required_frame_num_update_behaviour_flag = FALSE;
 *    sps->direct_temporal_constrained_flag = FALSE;
 *
 * \par
 *    Regarding the QP
 *    The previous software versions coded the absolute QP only in the 
 *    slice header.  This is kept, and the offset in the PPS is coded 
 *    even if we could save bits by intelligently using this field.
 *
 ************************************************************************
 */

void FillParameterSetStructures (seq_parameter_set_rbsp_t *sps, 
                                 pic_parameter_set_rbsp_t *pps)
{
  unsigned i;
  // *************************************************************************
  // Sequence Parameter Set ���в�����
  // *************************************************************************
  assert (sps != NULL);
  assert (pps != NULL);
  // Profile and Level should be calculated using the info from the config
  // file.  Calculation is hidden in IndetifyProfile() and IdentifyLevel()
  // ͨ���������ݣ����㵵�κ͵ȼ���������IndetifyProfile()��IdentifyLevel()�н���
  sps->profile_idc = IdentifyProfile();
  sps->level_idc = IdentifyLevel();

  // needs to be set according to profile
  // H.264��׼�е���Լ������=0ʱ��ʾ������ѭ
  sps->constrained_set0_flag = 0;
  sps->constrained_set1_flag = 0;
  sps->constrained_set2_flag = 0;

  // Parameter Set ID hardcoded to zero
  sps->seq_parameter_set_id = 0;				// sps��id����

  //! POC stuff:
  //! The following values are hard-coded in init_poc().  Apparently,
  //! the poc implementation covers only a subset of the poc functionality.
  //! Here, the same subset is implemented.  Changes in the POC stuff have
  //! also to be reflected here
  sps->log2_max_frame_num_minus4 = log2_max_frame_num_minus4;								//ָ��frame_num�����ֵ
  sps->log2_max_pic_order_cnt_lsb_minus4 = log2_max_pic_order_cnt_lsb_minus4;				//ָ��POC��LSB�����ֵ
  
  sps->pic_order_cnt_type = input->pic_order_cnt_type;										//ָ��POC�ļ���ģʽ
  sps->num_ref_frames_in_pic_order_cnt_cycle = img->num_ref_frames_in_pic_order_cnt_cycle;	//���ڲο�֡��POC��ֵ��ѭ������
  sps->delta_pic_order_always_zero_flag = img->delta_pic_order_always_zero_flag;			//����delta_pic_order_cnt[]�ĳ���
  sps->offset_for_non_ref_pic = img->offset_for_non_ref_pic;								//��������ǲο�֡�򳡵�POC				
  sps->offset_for_top_to_bottom_field = img->offset_for_top_to_bottom_field;				//ģʽ2ʱ����֪����POC������׳�POC

  //���ڲο�֡POC��ֵ�ĸ�ֵ
  for (i=0; i<img->num_ref_frames_in_pic_order_cnt_cycle; i++)
  {
    sps->offset_for_ref_frame[i] = img->offset_for_ref_frame[i];
  }
  // End of POC stuff

  // Number of Reference Frames
  sps->num_ref_frames = IdentifyNumRefFrames();												//��֤�ο�֡���Ƿ����

  //required_frame_num_update_behaviour_flag hardcoded to zero
  // double check 
  sps->gaps_in_frame_num_value_allowed_flag = FALSE;										//�Ƿ�����frame_num������(ͨ�������ϰ�ʱ)		

  sps->frame_mbs_only_flag = !(input->PicInterlace || input->MbInterlace);					//Ϊ1ʱ������������ͼ��ı���ģʽ����֡�����򣬲�һ��

  // Picture size, finally a simple one :-)
  sps->pic_width_in_mbs_minus1 = (input->img_width/16) -1;									//ͼ���ȣ���λ�Ǻ��
  sps->pic_height_in_map_units_minus1 = ((input->img_height/16)/ (2 - sps->frame_mbs_only_flag)) - 1;		//ͼ��߶ȣ���λ�Ǻ��

  // a couple of flags, simple
  sps->mb_adaptive_frame_field_flag = (FRAME_CODING != input->MbInterlace);					//
  sps->direct_8x8_inference_flag = input->directInferenceFlag;								//

  // Sequence VUI not implemented, signalled as not present
  sps->vui_parameters_present_flag = FALSE;													//�Ƿ���VUI��Ϣ
  
  {
    int PicWidthInMbs, PicHeightInMapUnits, FrameHeightInMbs;
    int width, height;
    PicWidthInMbs = (sps->pic_width_in_mbs_minus1 +1);
    PicHeightInMapUnits = (sps->pic_height_in_map_units_minus1 +1);
    FrameHeightInMbs = ( 2 - sps->frame_mbs_only_flag ) * PicHeightInMapUnits;
    
    width = PicWidthInMbs * MB_BLOCK_SIZE;
    height = FrameHeightInMbs * MB_BLOCK_SIZE;
    
    Co_located = alloc_colocated (width, height,sps->mb_adaptive_frame_field_flag);			//�����ڴ�
    
  }
  // *************************************************************************
  // Picture Parameter Set ͼ�������
  // *************************************************************************
  // 
  pps->seq_parameter_set_id = 0;											//pps��id����
  pps->pic_parameter_set_id = 0;											//��pps������sps����
  pps->entropy_coding_mode_flag = (input->symbol_mode==UVLC?0:1);			//��pps���õ��ر��루0��cavlc 1��cabac��

  // JVT-Fxxx (by Stephan Wenger, make this flag unconditional
  pps->pic_order_present_flag = img->pic_order_present_flag;				//ָ��Ƭͷ�Ƿ�����﷨Ԫ��ָ��POC����ʱ��Ҫ�õ��Ĳ���


  // Begin FMO stuff
  pps->num_slice_groups_minus1 = input->num_slice_groups_minus1;			//���õ���Ƭ�����

	
  //! Following set the parameter for different slice group types
  if (pps->num_slice_groups_minus1 > 0)
	  //slice_group_map_type ����ָ��Ƭ��ָ�����
    switch (input->slice_group_map_type)
    {
    case 0:
			
      pps->slice_group_map_type = 0;
      for(i=0; i<=pps->num_slice_groups_minus1; i++)
      {
        pps->run_length_minus1[i]=input->run_length_minus1[i];
      }
			
      break;
    case 1:
      pps->slice_group_map_type = 1;
      break;
    case 2:
      // i loops from 0 to num_slice_groups_minus1-1, because no info for background needed
      pps->slice_group_map_type = 2;
      for(i=0; i<pps->num_slice_groups_minus1; i++)
      {
        pps->top_left[i] = input->top_left[i];
        pps->bottom_right[i] = input->bottom_right[i];      
      }
     break;
    case 3:
    case 4:
    case 5:
      pps->slice_group_map_type = input->slice_group_map_type;
			
      pps->slice_group_change_direction_flag = input->slice_group_change_direction_flag;
      pps->slice_group_change_rate_minus1 = input->slice_group_change_rate_minus1;
      break;
    case 6:
      pps->slice_group_map_type = 6;   
      pps->pic_size_in_map_units_minus1 = 
				((input->img_height/MB_BLOCK_SIZE)/(2-sps->frame_mbs_only_flag))
				*(input->img_width/MB_BLOCK_SIZE) -1;
			
      for (i=0;i<=pps->pic_size_in_map_units_minus1; i++)
        pps->slice_group_id[i] = input->slice_group_id[i];
			
      break;
    default:
      printf ("Parset.c: slice_group_map_type invalid, default\n");
      assert (0==1);
    }
// End FMO stuff

  //�ڶ����У���ǰʵ�ʵģ��Ѵ��ڵĲο�֡��Ŀ
  pps->num_ref_idx_l0_active_minus1 = sps->frame_mbs_only_flag ? (sps->num_ref_frames-1) : (2 * sps->num_ref_frames - 1) ;   // set defaults
  pps->num_ref_idx_l1_active_minus1 = sps->frame_mbs_only_flag ? (sps->num_ref_frames-1) : (2 * sps->num_ref_frames - 1) ;   // set defaults
  //pps->num_ref_idx_l1_active_minus1 = sps->frame_mbs_only_flag ? 0 : 1 ;   // set defaults

  pps->weighted_pred_flag = input->WeightedPrediction;						//ָ���Ƿ�����p��spƬ�ļ�ȨԤ��
  pps->weighted_bipred_idc = input->WeightedBiprediction;					//ָ���Ƿ�����BƬ�ļ�ȨԤ��

  //��ʼ��������
  pps->pic_init_qp_minus26 = 0;         // hard coded to zero, QP lives in the slice header
  pps->pic_init_qs_minus26 = 0;

  //ɫ�ȷ�����������������ʱ���õ��Ĳ���
  pps->chroma_qp_index_offset = input->chroma_qp_index_offset;      // double check: is this chroma fidelity thing already implemented???

  //�Ƿ���ʾ�Ŀ����˲���ǰ�ȡ�0�����������������˲�ǿ�� 1�����﷨ǿ�п����˲�ǿ��
  pps->deblocking_filter_control_present_flag = input->LFSendParameters;
  pps->constrained_intra_pred_flag = input->UseConstrainedIntraPred;		//1��֡�ڱ��벻������֡�����ĺ����Ϊ�Լ���Ԥ�⡣0������
  
  pps->redundant_pic_cnt_present_flag = 0;									//ָ���Ƿ����redundant_pic_cnt�﷨

  // the picture vui consists currently of the cropping rectangle, which cannot
  // used by the current decoder and hence is never sent.
  sps->frame_cropping_flag = FALSE;
};



/*! 
 *************************************************************************************
 * \brief
 *    int GenerateSeq_parameter_set_rbsp (seq_parameter_set_rbsp_t *sps, char *rbsp);
 *
 * \param sps
 *    sequence parameter structure
 *	  sps�ṹ
 * \param rbsp
 *    buffer to be filled with the rbsp, size should be at least MAXIMUMPARSETRBSPSIZE
 *    ���rbsp�Ļ���������СӦ������ΪMAXIMUMPARSETRBSPSIZE
 * \return
 *    size of the RBSP in bytes
 *	  ����rbsp�Ĵ�С���ֽڵ�λ��
 * \note
 *    Sequence Parameter VUI function is called, but the function implements
 *    an exit (-1)
 *************************************************************************************
 */

//byte_buf, ��ǰ����д�����ֽ����ݣ�������ݲ�û���ֽڶ��䣬bits_to_goָ���˸����ݻ��ж���bitû��д��
//bits_to_go, ָ���˵�ǰ����д����ֽ����ݣ����ж���bitû��д��
//byte_pos, ָ����ǰ����д���ֽ����ݵ�λ��
 
int GenerateSeq_parameter_set_rbsp (seq_parameter_set_rbsp_t *sps, char *rbsp)
{
  DataPartition *partition;			//���ݷ���
  int len = 0, LenInBytes;			
  unsigned i;

  assert (rbsp != NULL);
  // In order to use the entropy coding functions from golomb.c we need 
  // to allocate a partition structure.  It will be freed later in this
  // function
  if ((partition=calloc(1,sizeof(DataPartition)))==NULL) no_mem_exit("SeqParameterSet:partition");				//Ϊpartition����ռ�
  if ((partition->bitstream=calloc(1, sizeof(Bitstream)))==NULL) no_mem_exit("SeqParameterSet:bitstream");		//Ϊpartion->bitstream����ռ�
  // .. and use the rbsp provided (or allocated above) for the data
  
  //����partition�Ļ���������rbsp����,partition��¼rbsp�����ѶϢ�����Ҷ�patition�Ļ������������Ƕ�rbsp�Ĳ���
  partition->bitstream->streamBuffer = rbsp;																	// bitstream->streamBuffer��rbsp����
  partition->bitstream->bits_to_go = 8;																			// 

  // ������д��partition�������Ƕ�rbspд�����ݣ�
  // �漰���ַ�����trace�ļ�����Ҫ��
  len+=u_v  (8, "SPS: profile_idc",                             sps->profile_idc,                               partition);

  len+=u_1  ("SPS: constrained_set0_flag",                      sps->constrained_set0_flag,    partition);
  len+=u_1  ("SPS: constrained_set1_flag",                      sps->constrained_set1_flag,    partition);
  len+=u_1  ("SPS: constrained_set2_flag",                      sps->constrained_set2_flag,    partition);
  len+=u_v  (5, "SPS: reserved_zero",                           0,                             partition);

  len+=u_v  (8, "SPS: level_idc",                               sps->level_idc,                                 partition);

  len+=ue_v ("SPS: seq_parameter_set_id",                    sps->seq_parameter_set_id,                      partition);
  len+=ue_v ("SPS: log2_max_frame_num_minus4",               sps->log2_max_frame_num_minus4,                 partition);
  len+=ue_v ("SPS: pic_order_cnt_type",                      sps->pic_order_cnt_type,                        partition);
  // POC200301
  if (sps->pic_order_cnt_type == 0)
    len+=ue_v ("SPS: log2_max_pic_order_cnt_lsb_minus4",     sps->log2_max_pic_order_cnt_lsb_minus4,         partition);
  else if (sps->pic_order_cnt_type == 1)
  {
    len+=u_1  ("SPS: delta_pic_order_always_zero_flag",        sps->delta_pic_order_always_zero_flag,          partition);
    len+=se_v ("SPS: offset_for_non_ref_pic",                  sps->offset_for_non_ref_pic,                    partition);
    len+=se_v ("SPS: offset_for_top_to_bottom_field",          sps->offset_for_top_to_bottom_field,            partition);
    len+=ue_v ("SPS: num_ref_frames_in_pic_order_cnt_cycle",   sps->num_ref_frames_in_pic_order_cnt_cycle,     partition);
    for (i=0; i<sps->num_ref_frames_in_pic_order_cnt_cycle; i++)
      len+=se_v ("SPS: offset_for_ref_frame",                  sps->offset_for_ref_frame[i],                      partition);
  }
  len+=ue_v ("SPS: num_ref_frames",                          sps->num_ref_frames,                            partition);
  len+=u_1  ("SPS: gaps_in_frame_num_value_allowed_flag",    sps->gaps_in_frame_num_value_allowed_flag,      partition);
  len+=ue_v ("SPS: pic_width_in_mbs_minus1",                 sps->pic_width_in_mbs_minus1,                   partition);
  len+=ue_v ("SPS: pic_height_in_map_units_minus1",          sps->pic_height_in_map_units_minus1,            partition);
  len+=u_1  ("SPS: frame_mbs_only_flag",                     sps->frame_mbs_only_flag,                       partition);
  if (!sps->frame_mbs_only_flag)
  {
    len+=u_1  ("SPS: mb_adaptive_frame_field_flag",            sps->mb_adaptive_frame_field_flag,              partition);
  }
  len+=u_1  ("SPS: direct_8x8_inference_flag",               sps->direct_8x8_inference_flag,                 partition);

  len+=u_1  ("SPS: frame_cropping_flag",                      sps->frame_cropping_flag,                       partition);
  if (sps->frame_cropping_flag)
  {
    len+=ue_v ("SPS: frame_cropping_rect_left_offset",          sps->frame_cropping_rect_left_offset,           partition);
    len+=ue_v ("SPS: frame_cropping_rect_right_offset",         sps->frame_cropping_rect_right_offset,          partition);
    len+=ue_v ("SPS: frame_cropping_rect_top_offset",           sps->frame_cropping_rect_top_offset,            partition);
    len+=ue_v ("SPS: frame_cropping_rect_bottom_offset",        sps->frame_cropping_rect_bottom_offset,         partition);
  }

  len+=u_1  ("SPS: vui_parameters_present_flag",             sps->vui_parameters_present_flag,               partition);
  if (sps->vui_parameters_present_flag)
    len+=GenerateVUISequenceParameters();    // currently a dummy, asserting

  //д��partition->bitstream(Ҳ����rbsp�������ڳ���ʼǰrbsp��bitstream����)�е����ݣ���ʵ��ֻ��sodb���ú�����sodb׼ȷת��Ϊ������rbsp
  //��SODB��������rbsp_trailing_bits�͵õ�RBSP
  SODBtoRBSP(partition->bitstream);			// copies the last couple of bits into the byte buffer
  
  LenInBytes=partition->bitstream->byte_pos;

  //�ͷ�partionָ�룬��ʱstreamBuffer�����Ľ��������rbspָ��(partition->bitstream->streamBuffer = rbsp)
  free (partition->bitstream);
  free (partition);
  
  return LenInBytes;
}


/*! 
 *************************************************************************************
 * \brief
 *    int GeneratePic_parameter_set_rbsp (pic_parameter_set_rbsp_t *sps, char *rbsp);
 *
 * \param pps
 *    picture parameter structure
 * \param rbsp
 *    buffer to be filled with the rbsp, size should be at least MAXIMUMPARSETRBSPSIZE
 *
 * \return
 *    size of the RBSP in bytes, negative in case of an error
 *
 * \note
 *    Picture Parameter VUI function is called, but the function implements
 *    an exit (-1)
 *************************************************************************************
 */
 // �ú����ο�GenerateSeq_parameter_set_rbsp���ע��
int GeneratePic_parameter_set_rbsp (pic_parameter_set_rbsp_t *pps, char *rbsp)
{
  DataPartition *partition;
  int len = 0, LenInBytes;
  unsigned i;
  unsigned NumberBitsPerSliceGroupId;

  assert (rbsp != NULL);

  // In order to use the entropy coding functions from golomb.c we need 
  // to allocate a partition structure.  It will be freed later in this
  // function
  if ((partition=calloc(1,sizeof(DataPartition)))==NULL) no_mem_exit("PicParameterSet:partition");
  if ((partition->bitstream=calloc(1, sizeof(Bitstream)))==NULL) no_mem_exit("PicParameterSet:bitstream");
  // .. and use the rbsp provided (or allocated above) for the data
  partition->bitstream->streamBuffer = rbsp;
  partition->bitstream->bits_to_go = 8;
  //sw paff
  pps->pic_order_present_flag = img->pic_order_present_flag;

  len+=ue_v ("PPS: pic_parameter_set_id",                    pps->pic_parameter_set_id,                      partition);
  len+=ue_v ("PPS: seq_parameter_set_id",                    pps->seq_parameter_set_id,                      partition);
  len+=u_1  ("PPS: entropy_coding_mode_flag",                pps->entropy_coding_mode_flag,                  partition);
  len+=u_1  ("PPS: pic_order_present_flag",                  pps->pic_order_present_flag,                    partition);
  len+=ue_v ("PPS: num_slice_groups_minus1",                 pps->num_slice_groups_minus1,                   partition);

  // FMO stuff
  if(pps->num_slice_groups_minus1 > 0 )
  {
    len+=ue_v ("PPS: slice_group_map_type",                 pps->slice_group_map_type,                   partition);
    if (pps->slice_group_map_type == 0)
      for (i=0; i<=pps->num_slice_groups_minus1; i++)
        len+=ue_v ("PPS: run_length_minus1[i]",                           pps->run_length_minus1[i],                             partition);
    else if (pps->slice_group_map_type==2)
      for (i=0; i<pps->num_slice_groups_minus1; i++)
      {

        len+=ue_v ("PPS: top_left[i]",                          pps->top_left[i],                           partition);
        len+=ue_v ("PPS: bottom_right[i]",                      pps->bottom_right[i],                       partition);
      }
    else if (pps->slice_group_map_type == 3 ||
             pps->slice_group_map_type == 4 ||
             pps->slice_group_map_type == 5) 
    {
      len+=u_1  ("PPS: slice_group_change_direction_flag",         pps->slice_group_change_direction_flag,         partition);
      len+=ue_v ("PPS: slice_group_change_rate_minus1",            pps->slice_group_change_rate_minus1,            partition);
    } 
    else if (pps->slice_group_map_type == 6)
    {
      if (pps->num_slice_groups_minus1>=4)
        NumberBitsPerSliceGroupId=3;
      else if (pps->num_slice_groups_minus1>=2)
        NumberBitsPerSliceGroupId=2;
      else if (pps->num_slice_groups_minus1>=1)
        NumberBitsPerSliceGroupId=1;
      else
        NumberBitsPerSliceGroupId=0;
        
      len+=ue_v ("PPS: pic_size_in_map_units_minus1",          pps->pic_size_in_map_units_minus1,             partition);
      for(i=0; i<=pps->pic_size_in_map_units_minus1; i++)
        len+= u_v  (NumberBitsPerSliceGroupId, "PPS: >slice_group_id[i]",   pps->slice_group_id[i],           partition);			
    }
  }
  // End of FMO stuff

  len+=ue_v ("PPS: num_ref_idx_l0_active_minus1",             pps->num_ref_idx_l0_active_minus1,              partition);
  len+=ue_v ("PPS: num_ref_idx_l1_active_minus1",             pps->num_ref_idx_l1_active_minus1,              partition);
  len+=u_1  ("PPS: weighted_pred_flag",                       pps->weighted_pred_flag,                        partition);
  len+=u_v  (2, "PPS: weighted_bipred_idc",                   pps->weighted_bipred_idc,                       partition);
  len+=se_v ("PPS: pic_init_qp_minus26",                      pps->pic_init_qp_minus26,                       partition);
  len+=se_v ("PPS: pic_init_qs_minus26",                      pps->pic_init_qs_minus26,                       partition);
  len+=se_v ("PPS: chroma_qp_index_offset",                   pps->chroma_qp_index_offset,                    partition);
  len+=u_1  ("PPS: deblocking_filter_control_present_flag",   pps->deblocking_filter_control_present_flag,    partition);
  len+=u_1  ("PPS: constrained_intra_pred_flag",              pps->constrained_intra_pred_flag,               partition);
  len+=u_1  ("PPS: redundant_pic_cnt_present_flag",           pps->redundant_pic_cnt_present_flag,            partition);

  SODBtoRBSP(partition->bitstream);     // copies the last couple of bits into the byte buffer
  
  LenInBytes=partition->bitstream->byte_pos;

  // Get rid of the helper structures
  free (partition->bitstream);
  free (partition);

  return LenInBytes;
}



/*! 
 *************************************************************************************
 * \brief
 *    Returns the Profile
 *
 * \return
 *    Profile according to Annex A
 *
 * \note
 *    Function is currently a dummy.  Should "calculate" the profile from those
 *    config file parameters.  E.g.
 *
 *    Profile = Baseline;
 *    if (CABAC Used || Interlace used) Profile=Main;
 *    if (!Cabac Used) && (Bframes | SPframes) Profile = Streaming;
 *
 *************************************************************************************
 */
int IdentifyProfile()
{
  return input->ProfileIDC;
};

/*! 
 *************************************************************************************
 * \brief
 *    Returns the Level
 *
 * \return
 *    Level according to Annex A
 *
 * \note
 *    This function is currently a dummy, but should calculate the level out of 
 *    the config file parameters (primarily the picture size)
 *************************************************************************************
 */
int IdentifyLevel()
{
  return input->LevelIDC;
};


/*! 
 *************************************************************************************
 * \brief
 *    Returns the number of reference frame buffers
 *
 * \return
 *    Number of reference frame buffers used
 *
 * \note
 *    This function currently maps to input->num_reference_frames.  With all this interlace
 *    stuff this may or may not be correct.  If you determine a problem with the
 *    memory management for Interlace, then this could be one possible problem.
 *    However, so far no problem have been determined by my limited testing of
 *    a stupid 1950's technology :-)  StW, 11/27/02
 *************************************************************************************
 */

int IdentifyNumRefFrames()
{
  if(input->num_reference_frames > 16)error("no ref frames too large",-100);
  
  return input->num_reference_frames;
}


/*! 
 *************************************************************************************
 * \brief
 *    Function body for VUI Parameter generation (to be done)
 *
 * \return
 *    exits with error message
 *************************************************************************************
 */
static int GenerateVUISequenceParameters()
{
  printf ("Sequence Parameter VUI not yet implemented, this should never happen, exit\n");
  exit (-1);
}

