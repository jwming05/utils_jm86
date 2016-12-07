
/*!
 **************************************************************************************
 * \file
 *    parset.c
 * \brief
 *    Picture and Sequence Parameter set generation and handling
 *	  图像和序列参数集生成和控制
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
static int IdentifyProfile();						//验证档次
static int IdentifyLevel();							//验证级别
static int IdentifyNumRefFrames();					//验证参考帧数
static int GenerateVUISequenceParameters();			//生成VUI序列参数

extern ColocatedParams *Co_located;


/*! 
 *************************************************************************************
 * \brief
 *    generates a sequence and picture parameter set and stores these in global
 *    active_sps and active_pps
 *    生成sps和pps并且保存这些全局活动的变量
 * \return
 *    A NALU containing the Sequence ParameterSet
 *
 *************************************************************************************
*/
void GenerateParameterSets ()
{
  seq_parameter_set_rbsp_t *sps = NULL;			//sps指针
  pic_parameter_set_rbsp_t *pps = NULL;			//pps指针

  sps = AllocSPS();								//分配sps空间
  pps = AllocPPS();								//分配pps空间

  FillParameterSetStructures (sps, pps);		//参数填充
  
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
*	 根据sps，生成SODB，再生成RBSP, 最后将nalu信息和RBSP2EBSP信息保存在nalu->buf中
* \note
*    Uses the global variables through FillParameterSetStructures()
*
* \return
*    A NALU containing the Sequence ParameterSet
*	 返回一个sps相关的nal单元
*************************************************************************************
*/

NALU_t *GenerateSeq_parameter_set_NALU ()
{
  NALU_t *n = AllocNALU(64000);
  int RBSPlen = 0;
  int NALUlen;
  byte rbsp[MAXRBSPSIZE];

  RBSPlen = GenerateSeq_parameter_set_rbsp (active_sps, rbsp);									//根据sps，生成rbsp，并返回RBSP长度
  NALUlen = RBSPtoNALU (rbsp, n, RBSPlen, NALU_TYPE_SPS, NALU_PRIORITY_HIGHEST, 0, 1);			//将rbsp信息，转入nalu，并NAL单元长度。(NALU = NALH+RBSP)
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
 *    从全局变量中提取信息，生成sps和pps结构体
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
 *    该函数，从几个全局变量(比如input和image)中读取所有的数据，并且往sps和pps中填充
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
  // Sequence Parameter Set 序列参数集
  // *************************************************************************
  assert (sps != NULL);
  assert (pps != NULL);
  // Profile and Level should be calculated using the info from the config
  // file.  Calculation is hidden in IndetifyProfile() and IdentifyLevel()
  // 通过配置内容，计算档次和等级，计算在IndetifyProfile()和IdentifyLevel()中进行
  sps->profile_idc = IdentifyProfile();
  sps->level_idc = IdentifyLevel();

  // needs to be set according to profile
  // H.264标准中的制约条件，=0时表示不必遵循
  sps->constrained_set0_flag = 0;
  sps->constrained_set1_flag = 0;
  sps->constrained_set2_flag = 0;

  // Parameter Set ID hardcoded to zero
  sps->seq_parameter_set_id = 0;				// sps的id号码

  //! POC stuff:
  //! The following values are hard-coded in init_poc().  Apparently,
  //! the poc implementation covers only a subset of the poc functionality.
  //! Here, the same subset is implemented.  Changes in the POC stuff have
  //! also to be reflected here
  sps->log2_max_frame_num_minus4 = log2_max_frame_num_minus4;								//指定frame_num的最大值
  sps->log2_max_pic_order_cnt_lsb_minus4 = log2_max_pic_order_cnt_lsb_minus4;				//指定POC的LSB的最大值
  
  sps->pic_order_cnt_type = input->pic_order_cnt_type;										//指定POC的计算模式
  sps->num_ref_frames_in_pic_order_cnt_cycle = img->num_ref_frames_in_pic_order_cnt_cycle;	//相邻参考帧的POC差值的循环周期
  sps->delta_pic_order_always_zero_flag = img->delta_pic_order_always_zero_flag;			//控制delta_pic_order_cnt[]的出现
  sps->offset_for_non_ref_pic = img->offset_for_non_ref_pic;								//用来计算非参考帧或场的POC				
  sps->offset_for_top_to_bottom_field = img->offset_for_top_to_bottom_field;				//模式2时，已知顶场POC，计算底场POC

  //相邻参考帧POC差值的赋值
  for (i=0; i<img->num_ref_frames_in_pic_order_cnt_cycle; i++)
  {
    sps->offset_for_ref_frame[i] = img->offset_for_ref_frame[i];
  }
  // End of POC stuff

  // Number of Reference Frames
  sps->num_ref_frames = IdentifyNumRefFrames();												//验证参考帧数是否过大

  //required_frame_num_update_behaviour_flag hardcoded to zero
  // double check 
  sps->gaps_in_frame_num_value_allowed_flag = FALSE;										//是否允许frame_num不连续(通信遇到障碍时)		

  sps->frame_mbs_only_flag = !(input->PicInterlace || input->MbInterlace);					//为1时，本序列所有图像的编码模式都是帧。否则，不一定

  // Picture size, finally a simple one :-)
  sps->pic_width_in_mbs_minus1 = (input->img_width/16) -1;									//图像宽度，单位是宏块
  sps->pic_height_in_map_units_minus1 = ((input->img_height/16)/ (2 - sps->frame_mbs_only_flag)) - 1;		//图像高度，单位是宏块

  // a couple of flags, simple
  sps->mb_adaptive_frame_field_flag = (FRAME_CODING != input->MbInterlace);					//
  sps->direct_8x8_inference_flag = input->directInferenceFlag;								//

  // Sequence VUI not implemented, signalled as not present
  sps->vui_parameters_present_flag = FALSE;													//是否有VUI信息
  
  {
    int PicWidthInMbs, PicHeightInMapUnits, FrameHeightInMbs;
    int width, height;
    PicWidthInMbs = (sps->pic_width_in_mbs_minus1 +1);
    PicHeightInMapUnits = (sps->pic_height_in_map_units_minus1 +1);
    FrameHeightInMbs = ( 2 - sps->frame_mbs_only_flag ) * PicHeightInMapUnits;
    
    width = PicWidthInMbs * MB_BLOCK_SIZE;
    height = FrameHeightInMbs * MB_BLOCK_SIZE;
    
    Co_located = alloc_colocated (width, height,sps->mb_adaptive_frame_field_flag);			//分配内存
    
  }
  // *************************************************************************
  // Picture Parameter Set 图像参数集
  // *************************************************************************
  // 
  pps->seq_parameter_set_id = 0;											//pps的id号码
  pps->pic_parameter_set_id = 0;											//该pps所属的sps号码
  pps->entropy_coding_mode_flag = (input->symbol_mode==UVLC?0:1);			//该pps所用的熵编码（0：cavlc 1：cabac）

  // JVT-Fxxx (by Stephan Wenger, make this flag unconditional
  pps->pic_order_present_flag = img->pic_order_present_flag;				//指定片头是否会有语法元素指明POC计算时需要用到的参数


  // Begin FMO stuff
  pps->num_slice_groups_minus1 = input->num_slice_groups_minus1;			//所用到的片组个数

	
  //! Following set the parameter for different slice group types
  if (pps->num_slice_groups_minus1 > 0)
	  //slice_group_map_type 用于指定片组分割类型
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

  //在队列中，当前实际的，已存在的参考帧数目
  pps->num_ref_idx_l0_active_minus1 = sps->frame_mbs_only_flag ? (sps->num_ref_frames-1) : (2 * sps->num_ref_frames - 1) ;   // set defaults
  pps->num_ref_idx_l1_active_minus1 = sps->frame_mbs_only_flag ? (sps->num_ref_frames-1) : (2 * sps->num_ref_frames - 1) ;   // set defaults
  //pps->num_ref_idx_l1_active_minus1 = sps->frame_mbs_only_flag ? 0 : 1 ;   // set defaults

  pps->weighted_pred_flag = input->WeightedPrediction;						//指定是否允许p和sp片的加权预测
  pps->weighted_bipred_idc = input->WeightedBiprediction;					//指明是否允许B片的加权预测

  //初始量化参数
  pps->pic_init_qp_minus26 = 0;         // hard coded to zero, QP lives in the slice header
  pps->pic_init_qs_minus26 = 0;

  //色度分量的量化参数计算时，用到的参数
  pps->chroma_qp_index_offset = input->chroma_qp_index_offset;      // double check: is this chroma fidelity thing already implemented???

  //是否显示的控制滤波器前度。0：解码器独立计算滤波强度 1：有语法强行控制滤波强度
  pps->deblocking_filter_control_present_flag = input->LFSendParameters;
  pps->constrained_intra_pred_flag = input->UseConstrainedIntraPred;		//1：帧内编码不能用用帧间编码的宏块作为自己的预测。0：可以
  
  pps->redundant_pic_cnt_present_flag = 0;									//指明是否出现redundant_pic_cnt语法

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
 *	  sps结构
 * \param rbsp
 *    buffer to be filled with the rbsp, size should be at least MAXIMUMPARSETRBSPSIZE
 *    填充rbsp的缓冲区，大小应该至少为MAXIMUMPARSETRBSPSIZE
 * \return
 *    size of the RBSP in bytes
 *	  返回rbsp的大小（字节单位）
 * \note
 *    Sequence Parameter VUI function is called, but the function implements
 *    an exit (-1)
 *************************************************************************************
 */

//byte_buf, 当前正在写尔的字节数据，这个数据并没有字节对其，bits_to_go指明了该数据还有多少bit没有写入
//bits_to_go, 指明了当前正在写入的字节数据，还有多少bit没有写入
//byte_pos, 指明当前正在写的字节数据的位置
 
int GenerateSeq_parameter_set_rbsp (seq_parameter_set_rbsp_t *sps, char *rbsp)
{
  DataPartition *partition;			//数据分区
  int len = 0, LenInBytes;			
  unsigned i;

  assert (rbsp != NULL);
  // In order to use the entropy coding functions from golomb.c we need 
  // to allocate a partition structure.  It will be freed later in this
  // function
  if ((partition=calloc(1,sizeof(DataPartition)))==NULL) no_mem_exit("SeqParameterSet:partition");				//为partition分配空间
  if ((partition->bitstream=calloc(1, sizeof(Bitstream)))==NULL) no_mem_exit("SeqParameterSet:bitstream");		//为partion->bitstream分配空间
  // .. and use the rbsp provided (or allocated above) for the data
  
  //这里partition的缓冲区，和rbsp联立,partition记录rbsp的相关讯息，并且对patition的缓冲区操作就是对rbsp的操作
  partition->bitstream->streamBuffer = rbsp;																	// bitstream->streamBuffer和rbsp联立
  partition->bitstream->bits_to_go = 8;																			// 

  // 将数据写入partition（核心是对rbsp写入数据）
  // 涉及的字符串是trace文件所需要的
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

  //写入partition->bitstream(也就是rbsp变量，在程序开始前rbsp和bitstream联立)中的内容，其实还只是sodb，该函数将sodb准确转换为真正的rbsp
  //对SODB的最后填充rbsp_trailing_bits就得到RBSP
  SODBtoRBSP(partition->bitstream);			// copies the last couple of bits into the byte buffer
  
  LenInBytes=partition->bitstream->byte_pos;

  //释放partion指针，这时streamBuffer操作的结果，仅由rbsp指定(partition->bitstream->streamBuffer = rbsp)
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
 // 该函数参考GenerateSeq_parameter_set_rbsp相关注释
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

