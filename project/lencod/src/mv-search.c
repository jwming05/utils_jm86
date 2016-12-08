
/*!
 *************************************************************************************
 * \file mv-search.c
 *
 * \brief
 *    Motion Vector Search, unified for B and P Pictures
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *      - Inge Lille-Lang鴜               <inge.lille-langoy@telenor.com>
 *      - Rickard Sjoberg                 <rickard.sjoberg@era.ericsson.se>
 *      - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *      - Jani Lainema                    <jani.lainema@nokia.com>
 *      - Detlev Marpe                    <marpe@hhi.de>
 *      - Thomas Wedi                     <wedi@tnt.uni-hannover.de>
 *      - Heiko Schwarz                   <hschwarz@hhi.de>
 *
 *************************************************************************************
*/

#include "contributors.h"

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <limits.h>

#include "global.h"
#include "image.h"
#include "mv-search.h"
#include "refbuf.h"
#include "memalloc.h"
#include "mb_access.h"
#include "fast_me.h"

#include <time.h>
#include <sys/timeb.h>

// These procedure pointers are used by motion_search() and one_eigthpel()
static pel_t  (*PelY_14)     (pel_t**, int, int, int, int);
static pel_t *(*PelYline_11) (pel_t *, int, int, int, int);

// Statistics, temporary
int     max_mvd;
int*    spiral_search_x;
int*    spiral_search_y;
int*    mvbits;
int*    refbits;
int*    byte_abs;
int**** motion_cost;


void SetMotionVectorPredictor (int  pmv[2],
                               int  ***refPic,
                               int  ****tmp_mv,
                               int  ref_frame,
                               int  list,
                               int  block_x,
                               int  block_y,
                               int  blockshape_x,
                               int  blockshape_y);

#ifdef _FAST_FULL_ME_

/*****
 *****  static variables for fast integer motion estimation
 *****
 */
static int  **search_setup_done;  //!< flag if all block SAD's have been calculated yet
static int  **search_center_x;    //!< absolute search center for fast full motion search
static int  **search_center_y;    //!< absolute search center for fast full motion search
static int  **pos_00;             //!< position of (0,0) vector
static int  *****BlockSAD;        //!< SAD for all blocksize, ref. frames and motion vectors
static int  **max_search_range;

extern ColocatedParams *Co_located;

/*!
 ***********************************************************************
 * \brief
 *    function creating arrays for fast integer motion estimation
 ***********************************************************************
 */
void
InitializeFastFullIntegerSearch ()
{
  int  i, j, k, list;
  int  search_range = input->search_range;
  int  max_pos      = (2*search_range+1) * (2*search_range+1);

  if ((BlockSAD = (int*****)malloc (2 * sizeof(int****))) == NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");

  for (list=0; list<2;list++)
  {
    if ((BlockSAD[list] = (int****)malloc ((img->max_num_references+1) * sizeof(int***))) == NULL)
      no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
    for (i = 0; i <= img->max_num_references; i++)
    {
      if ((BlockSAD[list][i] = (int***)malloc (8 * sizeof(int**))) == NULL)
        no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
      for (j = 1; j < 8; j++)
      {
        if ((BlockSAD[list][i][j] = (int**)malloc (16 * sizeof(int*))) == NULL)
          no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
        for (k = 0; k < 16; k++)
        {
          if ((BlockSAD[list][i][j][k] = (int*)malloc (max_pos * sizeof(int))) == NULL)
            no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
        }
      }
    }
  }

  if ((search_setup_done = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_setup_done");
  if ((search_center_x = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_x");
  if ((search_center_y = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_y");
  if ((pos_00 = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: pos_00");
  if ((max_search_range = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: max_search_range");

  for (list=0; list<2; list++)
  {
  if ((search_setup_done[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_setup_done");
  if ((search_center_x[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_x");
  if ((search_center_y[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_y");
  if ((pos_00[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: pos_00");
  if ((max_search_range[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: max_search_range");
  }

  // assign max search ranges for reference frames
  if (input->full_search == 2)
  {
    for (list=0;list<2;list++)
      for (i=0; i<=img->max_num_references; i++)  
        max_search_range[list][i] = search_range;
  }
  else
  {
    for (list=0;list<2;list++)
    {
      max_search_range[list][0] = max_search_range[list][img->max_num_references] = search_range;
      for (i=1; i< img->max_num_references; i++)  max_search_range[list][i] = search_range / 2;
    }
  }

}



/*!
 ***********************************************************************
 * \brief
 *    function for deleting the arrays for fast integer motion estimation
 ***********************************************************************
 */
void
ClearFastFullIntegerSearch ()
{
  int  i, j, k, list;

  for (list=0; list<2; list++)
  {
    for (i = 0; i <= img->max_num_references; i++)
    {
      for (j = 1; j < 8; j++)
      {
        for (k = 0; k < 16; k++)
        {
          free (BlockSAD[list][i][j][k]);
        }
        free (BlockSAD[list][i][j]);
      }
      free (BlockSAD[list][i]);
    }
    free (BlockSAD[list]);
  }
  free (BlockSAD);

  for (list=0; list<2; list++)
  {
    free (search_setup_done[list]);
    free (search_center_x[list]);
    free (search_center_y[list]);
    free (pos_00[list]);
    free (max_search_range[list]);
  }
  free (search_setup_done);
  free (search_center_x);
  free (search_center_y);
  free (pos_00);
  free (max_search_range);

}


/*!
 ***********************************************************************
 * \brief
 *    function resetting flags for fast integer motion estimation
 *    (have to be called in start_macroblock())
 ***********************************************************************
 */
void
ResetFastFullIntegerSearch ()
{
  int i,list;

  for (list=0; list<2; list++)
    for (i = 0; i <= img->max_num_references; i++)
      search_setup_done [list][i] = 0;
}

/*!
 ***********************************************************************
 * \brief
 *    calculation of SAD for larger blocks on the basis of 4x4 blocks
 *	  基于BlockSAD[list][refindex][7](4x4块的sad)，计算所有大块(4x8 8x4 8x8 8x16 16x8 16x16)的sad
 *    保存在BlockSAD[list][refindex][type]中type，是不同的分割类型
 ***********************************************************************
 */
void
SetupLargerBlocks (int list, int refindex, int max_pos)
{
#define ADD_UP_BLOCKS()   _o=*_bo; _i=*_bi; _j=*_bj; for(pos=0;pos<max_pos;pos++) _o[pos] = _i[pos] + _j[pos];
#define INCREMENT(inc)    _bo+=inc; _bi+=inc; _bj+=inc;

  int    pos, **_bo, **_bi, **_bj;
  register int *_o,   *_i,   *_j;

  //--- blocktype 6 ---4x8模式
  //_bo由两个上下相邻的4x4块求和得出，上下相邻的4x4块sad的指针分别是_bi, _bj
  _bo = BlockSAD[list][refindex][6];			//_bo待求出，_bo中存放宏块中，所有4x8分块的sad，计算sad的时候，也是4x8块的所有搜寻位置pos的sad
  _bi = BlockSAD[list][refindex][7];			//_bi中存放的是宏块中，所有4x4分块sad(即前面求出的block_sad)，这个是用来求出4x8块sad的
  _bj = _bi + 4;								//指向_bi相邻下边的4x4块_bj.
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(5);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS();

  //--- blocktype 5 ---8x4模式
  //_bo由两个左右相邻的4x4块求和得出，左右相邻的4x4块sad的指针分别是_bi, _bj
  _bo = BlockSAD[list][refindex][5];			//_bo待求出，_bo中存放宏块中，所有8x4分块的sad，计算sad的时候，也是8x4块的所有搜寻位置pos的sad
  _bi = BlockSAD[list][refindex][7];			//_bi中存放的是宏块中，所有4x4分块sad(即前面求出的block_sad)，这个是用来求出8x4块sad的
  _bj = _bi + 1;								//指向_bi相邻右边的4x4块_bj.
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 4 --- 8x8模式
  //_bo由左右相邻的4x8块求和得出，相邻左右的4x8块sad的指针分别是_bi, _bj
  _bo = BlockSAD[list][refindex][4];			//_bo待求出，_bo中存放宏块中，所有8x8分块的sad，计算sad的时候，也是8x8块的所有搜寻位置pos的sad
  _bi = BlockSAD[list][refindex][6];			//_bi中存放的是宏块中，所有4x8分块sad(即前面求出的block_sad)，这个是用来求出8x8块sad的
  _bj = _bi + 1;								//指向_bi相邻右边的4x8块_bj.
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(6);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 3 ---8x16
  //_bo由上下相邻的8x8块求和得出，相邻上下的8x8块sad的指针分别是_bi, _bj
  _bo = BlockSAD[list][refindex][3];			//_bo待求出，_bo中存放宏块中，所有8x16分块的sad，计算sad的时候，也是8x16块的所有搜寻位置pos的sad
  _bi = BlockSAD[list][refindex][4];			//_bi中存放的是宏块中，所有8x8分块sad(即前面求出的block_sad)，这个是用来求出8x16块sad的
  _bj = _bi + 8;								//指向_bi相邻右边的8x8块_bj.
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 2 ---16x8模式
  //_bo由左右相邻的8x8块求和得出，相邻的8x左右8块sad的指针分别是_bi, _bj
  _bo = BlockSAD[list][refindex][2];			//_bo待求出，_bo中存放宏块中，所有16x8分块的sad，计算sad的时候，也是8x16块的所有搜寻位置pos的sad
  _bi = BlockSAD[list][refindex][4];			//_bi中存放的是宏块中，所有8x8分块sad(即前面求出的block_sad)，这个是用来求出16x8块sad的
  _bj = _bi + 2;								//指向_bi相邻右边的8x8块_bj.
  ADD_UP_BLOCKS(); INCREMENT(8);
  ADD_UP_BLOCKS();

  //--- blocktype 1 ---16x16模式
  _bo = BlockSAD[list][refindex][1];
  _bi = BlockSAD[list][refindex][3];
  _bj = _bi + 2;
  ADD_UP_BLOCKS();
}


/*!
 ***********************************************************************
 * \brief
 *    Setup the fast search for an macroblock
 ***********************************************************************
 */
void SetupFastFullPelSearch (int ref, int list)  // <--  reference frame parameter, list0 or 1
{
  int     pmv[2];
  pel_t   orig_blocks[256], *orgptr=orig_blocks, *refptr;
  int     offset_x, offset_y, x, y, range_partly_outside, ref_x, ref_y, pos, abs_x, abs_y, bindex, blky;
  int     LineSadBlk0, LineSadBlk1, LineSadBlk2, LineSadBlk3;
  int     max_width, max_height;
  int     img_width, img_height;
  
  //ref_picture是用于保存指定参考帧信息的，包括像素信息，picnum，等等
  //ref_pic仅仅是ref_picture中的像素信息
  StorablePicture *ref_picture;
  pel_t   *ref_pic;

  //block_sad实际上是BlockSAD[list][ref][7]的指针，即保存所有4x4块，所有搜寻点的sad
  int**   block_sad     = BlockSAD[list][ref][7];

  int     search_range  = max_search_range[list][ref];
  int     max_pos       = (2*search_range+1) * (2*search_range+1);

  int     list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  int     apply_weights = ( (active_pps->weighted_pred_flag && (img->type == P_SLICE || img->type == SP_SLICE)) ||
                            (active_pps->weighted_bipred_idc && (img->type == B_SLICE)));

  
  //指定参考帧图像！ 其中缓存了参考帧的整幅图像的像素
  ref_picture     = listX[list+list_offset][ref];

  //缓存参考帧像素！！！
  if (apply_weights)
    ref_pic       = ref_picture->imgY_11_w;
  else
    ref_pic       = ref_picture->imgY_11;

  max_width     = ref_picture->size_x - 17;
  max_height    = ref_picture->size_y - 17;
  
  img_width     = ref_picture->size_x;
  img_height    = ref_picture->size_y;

  //===== get search center: predictor of 16x16 block =====
  SetMotionVectorPredictor (pmv, enc_picture->ref_idx, enc_picture->mv, ref, list, 0, 0, 16, 16);
  search_center_x[list][ref] = pmv[0] / 4;
  search_center_y[list][ref] = pmv[1] / 4;

  if (!input->rdopt)
  {
    //--- correct center so that (0,0) vector is inside ---
    search_center_x[list][ref] = max(-search_range, min(search_range, search_center_x[list][ref]));
    search_center_y[list][ref] = max(-search_range, min(search_range, search_center_y[list][ref]));
  }

  search_center_x[list][ref] += img->opix_x;
  search_center_y[list][ref] += img->opix_y;

  //搜索中心，其实就是当前宏块在图像中的绝对坐标（像素单位）
  offset_x = search_center_x[list][ref];
  offset_y = search_center_y[list][ref];

  //===== copy original block for fast access =====
  for   (y = img->opix_y; y < img->opix_y+16; y++)
    for (x = img->opix_x; x < img->opix_x+16; x++)
      *orgptr++ = imgY_org [y][x];


  //===== check if whole search range is inside image =====
  //判断是否有超出范围(offset_x, offset_y就是宏块的左上角坐标。)
  //offset_x >= search_range, 则左方有足够的空间
  //offset_y >= search_range, 则上方有足够的空间
  //offset_x <= max_width-search_range， 则右方有足够的空间
  //offset_y <= max_heigth-search_range, 则下方有足够的空间
  //上述均以像素为单位
  if (offset_x >= search_range && offset_x <= max_width  - search_range &&
      offset_y >= search_range && offset_y <= max_height - search_range   )
  {
    range_partly_outside = 0; PelYline_11 = FastLine16Y_11;
  }
  else
  {
    range_partly_outside = 1;
  }

  //===== determine position of (0,0)-vector =====
  if (!input->rdopt)
  {
    ref_x = img->opix_x - offset_x;
    ref_y = img->opix_y - offset_y;

    for (pos = 0; pos < max_pos; pos++)
    {
      if (ref_x == spiral_search_x[pos] &&
          ref_y == spiral_search_y[pos])
      {
        pos_00[list][ref] = pos;
        break;
      }
    }
  }

  //===== loop over search range (spiral search): get blockwise SAD =====
  //遍历max_pos个搜索点，pos实际上指定了运动矢量，得到了参考帧的绝对坐标abs_y, abs_x（像素单位）
  for (pos = 0; pos < max_pos; pos++)
  {
	//参考帧的绝对坐标
	//spiral_search, 在主函数中的Init_Motion_Search_Module进行了初始化，保存的是运动矢量的螺旋搜索坐标
    abs_y = offset_y + spiral_search_y[pos];
    abs_x = offset_x + spiral_search_x[pos];

	//先判断参考帧中所用到的坐标abs_x, abs_y是否超过了搜索范围
	//没有超过搜索范围，使用FastLine16Y_11读取像素
	//超过范围，使用UMLine16Y_l1
    if (range_partly_outside)
    {
      if (abs_y >= 0 && abs_y <= max_height &&
          abs_x >= 0 && abs_x <= max_width    )
      {
        PelYline_11 = FastLine16Y_11;
      }
      else
      {
        PelYline_11 = UMVLine16Y_11;
      }
    }

    orgptr = orig_blocks;			//将orgptr复位到宏块像素起始位置
    bindex = 0;						//新的搜寻位置pos，复位bindex

	//blky从0到3，将宏块分成了4行4x4块
    for (blky = 0; blky < 4; blky++)
    {
      LineSadBlk0 = LineSadBlk1 = LineSadBlk2 = LineSadBlk3 = 0;
	  //累积4行，4个部分的预测误差
	  //当前行(4x4块单位)又要分成4行(像素)单位来分别进行，并累积当前行(4x4单位)第i个块预测误差
	  //如LineSadBlk0缓存了当前行(4x4块)的第1个4x4块的预测误差
      for (y = 0; y < 4; y++)
      {
        refptr = PelYline_11 (ref_pic, abs_y++, abs_x, img_height, img_width);	//读取指定abs_y abs_x为头的像素, 其实是为了读取一行元素，后面好减
		//下面的refptr和orgptr分别自加16次，也对应相减了16次，也就是1个宏块的1行。
		//当前行(4x4块单位)又要分成4行(像素)单位来分别进行，并累积当前行(4x4单位)第i个块预测误差
		//LineSadBlk0是当前行(4x4块单位)第1个4x4块的预测误差
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];
        //LineSadBlk1是当前行(4x4块单位)第2个4x4块的预测误差
		LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
		//LineSadBlk2是当前行(4x4块单位)第3个4x4块的预测误差
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
		//LineSadBlk3是当前行(4x4块单位)第4个4x4块的预测误差
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
      }
	  //block_sad[bindex][pos] 是指定搜索点pos下，指定bindex/4行(4x4块单位)，指定bindex%4的4x4块的预测误差
	  //bindex可以直接对某个4x4块的预测误差进行定位，所以bindex可以简单理解为4x4块的编号，编号的方法为：
	  /*************************************************************************************************
	  bindex对4x4块的编号顺序：																		  **
	  **							00 | 01 | 02 | 03												  **
	  **							-----------------												  **
	  **							04 | 05 | 06 | 07												  **
	  **							-----------------												  **
	  **							08 | 09 | 10 | 11												  **
	  *8							-----------------												  **
	  **							12 | 13 | 14 | 15												  **
	  ***************************************************************************************************/
	  //block_sad实际上是BlockSAD[list][ref][7]的指针
      block_sad[bindex++][pos] = LineSadBlk0;
      block_sad[bindex++][pos] = LineSadBlk1;
      block_sad[bindex++][pos] = LineSadBlk2;
      block_sad[bindex++][pos] = LineSadBlk3;
    }
  }


  //===== combine SAD's for larger block types =====
  SetupLargerBlocks (list, ref, max_pos);


  //===== set flag marking that search setup have been done =====
  search_setup_done[list][ref] = 1;
}
#endif // _FAST_FULL_ME_

/*!
 ************************************************************************
 * \brief
 *    Set motion vector predictor
 *	  预测指定模式下，指定搜索块，指定参考帧中的运动矢量
 ************************************************************************
 */
void SetMotionVectorPredictor (int  pmv[2],					//pred_mv
                               int  ***refPic,				//enc_picture->ref_idx		
                               int  ****tmp_mv,				//enc_picture->mv
                               int  ref_frame,				//ref						指定的参考帧
                               int  list,					//list						指定的参考帧列表
                               int  block_x,				//block_x
                               int  block_y,				//block_y
                               int  blockshape_x,			//bsx
                               int  blockshape_y)			//bsy
{
  int mb_x                 = 4*block_x;
  int mb_y                 = 4*block_y;
  int mb_nr                = img->current_mb_nr;

  int mv_a, mv_b, mv_c, pred_vec=0;
  int mvPredType, rFrameL, rFrameU, rFrameUR;
  int hv;

  PixelPos block_a, block_b, block_c, block_d;

  int SAD_a=0, SAD_b=0, SAD_c=0, SAD_d=0;
  int temp_pred_SAD[2];

  if (input->FMEnable) pred_SAD_space=0;
  
  //下面读取当前搜寻块邻近像素的信息，为了相邻像素所属搜寻块(即相邻搜寻块)的运动矢量。
  getLuma4x4Neighbour(mb_nr, block_x, block_y,           -1,  0, &block_a);		//搜寻块左方  4x4块的信息(a块)
  getLuma4x4Neighbour(mb_nr, block_x, block_y,            0, -1, &block_b);		//搜寻块上方  4x4块的信息(b块)
  getLuma4x4Neighbour(mb_nr, block_x, block_y, blockshape_x, -1, &block_c);		//搜寻块右上方4x4块的信息(c块)
  getLuma4x4Neighbour(mb_nr, block_x, block_y,           -1, -1, &block_d);		//搜寻块左上方4x4块的信息(d块) 主要在c块不可用时，代替c块的信息

  if (mb_y > 0)
  {
    if (mb_x < 8)  // first column of 8x8 blocks
    {
      if (mb_y==8)
      {
        if (blockshape_x == 16)      block_c.available  = 0;
        else                         block_c.available &= 1;
      }
      else
      {
        if (mb_x+blockshape_x != 8)  block_c.available &= 1;
        else                         block_c.available  = 0;
      }
    }
    else
    {
      if (mb_x+blockshape_x != 16)   block_c.available &= 1;
      else                           block_c.available  = 0;
    }
  }

  //如果c块没用，那么用d块的信息代替c块
  if (!block_c.available)
  {
    block_c=block_d;
  }

  //运动矢量的测试模式(a b c块间的中值预测)
  mvPredType = MVPRED_MEDIAN;

  //缓存各块使用的参考帧
  //rFrameL(reference frame left)、rFrameU(reference frame up)、rFrameUR(reference frame up right)
  if (!img->MbaffFrameFlag)
  {
    rFrameL    = block_a.available    ? refPic[list][block_a.pos_x][block_a.pos_y] : -1;			//a块使用的参考帧
    rFrameU    = block_b.available    ? refPic[list][block_b.pos_x][block_b.pos_y] : -1;			//b块使用的参考帧
    rFrameUR   = block_c.available    ? refPic[list][block_c.pos_x][block_c.pos_y] : -1;			//c块使用的参考帧
  }
  else
  {
    if (img->mb_data[img->current_mb_nr].mb_field)
    {
      rFrameL    = block_a.available    ? 
        img->mb_data[block_a.mb_addr].mb_field ? 
        refPic[list][block_a.pos_x][block_a.pos_y]:
        refPic[list][block_a.pos_x][block_a.pos_y] * 2: 
        -1;
      rFrameU    = block_b.available    ? 
        img->mb_data[block_b.mb_addr].mb_field ? 
        refPic[list][block_b.pos_x][block_b.pos_y]:
        refPic[list][block_b.pos_x][block_b.pos_y] * 2: 
        -1;
      rFrameUR    = block_c.available    ? 
        img->mb_data[block_c.mb_addr].mb_field ? 
        refPic[list][block_c.pos_x][block_c.pos_y]:
        refPic[list][block_c.pos_x][block_c.pos_y] * 2: 
        -1;
    }
    else
    {
      rFrameL    = block_a.available    ? 
        img->mb_data[block_a.mb_addr].mb_field ? 
        refPic[list][block_a.pos_x][block_a.pos_y] >>1:
        refPic[list][block_a.pos_x][block_a.pos_y] : 
        -1;
      rFrameU    = block_b.available    ? 
        img->mb_data[block_b.mb_addr].mb_field ? 
        refPic[list][block_b.pos_x][block_b.pos_y] >>1:
        refPic[list][block_b.pos_x][block_b.pos_y] : 
        -1;
      rFrameUR    = block_c.available    ? 
        img->mb_data[block_c.mb_addr].mb_field ? 
        refPic[list][block_c.pos_x][block_c.pos_y] >>1:
        refPic[list][block_c.pos_x][block_c.pos_y] : 
        -1;
    }
  }


  /* Prediction if only one of the neighbors uses the reference frame
   * we are checking
   */
  //如果只有一个邻块使用的参考帧和我们当前指定的参考帧一样，则用该块进行预测
  if(rFrameL == ref_frame && rFrameU != ref_frame && rFrameUR != ref_frame)       mvPredType = MVPRED_L;
  else if(rFrameL != ref_frame && rFrameU == ref_frame && rFrameUR != ref_frame)  mvPredType = MVPRED_U;
  else if(rFrameL != ref_frame && rFrameU != ref_frame && rFrameUR == ref_frame)  mvPredType = MVPRED_UR;
  // Directional predictions 
  if(blockshape_x == 8 && blockshape_y == 16)
  {
    if(mb_x == 0)
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;
    }
    else
    {
      if( rFrameUR == ref_frame)
        mvPredType = MVPRED_UR;
    }
  }
  else if(blockshape_x == 16 && blockshape_y == 8)
  {
    if(mb_y == 0)
    {
      if(rFrameU == ref_frame)
        mvPredType = MVPRED_U;
    }
    else
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;
    }
  }

  //hv,运动矢量的两个方向.0为x方向，1为y方向
  for (hv=0; hv < 2; hv++)
  {
    if (!img->MbaffFrameFlag || hv==0)
    {	//将可用块的运动矢量缓存，不可用的，置为0
      mv_a = block_a.available  ? tmp_mv[list][block_a.pos_x][block_a.pos_y][hv] : 0;
      mv_b = block_b.available  ? tmp_mv[list][block_b.pos_x][block_b.pos_y][hv] : 0;
      mv_c = block_c.available  ? tmp_mv[list][block_c.pos_x][block_c.pos_y][hv] : 0;
    }
    else
    {
      if (img->mb_data[img->current_mb_nr].mb_field)
      {
        mv_a = block_a.available  ? img->mb_data[block_a.mb_addr].mb_field?
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv]:
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv] / 2: 
          0;
        mv_b = block_b.available  ? img->mb_data[block_b.mb_addr].mb_field?
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv]:
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv] / 2: 
          0;
        mv_c = block_c.available  ? img->mb_data[block_c.mb_addr].mb_field?
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv]:
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv] / 2: 
          0;
      }
      else
      {
        mv_a = block_a.available  ? img->mb_data[block_a.mb_addr].mb_field?
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv] * 2:
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv]: 
          0;
        mv_b = block_b.available  ? img->mb_data[block_b.mb_addr].mb_field?
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv] * 2:
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv]: 
          0;
        mv_c = block_c.available  ? img->mb_data[block_c.mb_addr].mb_field?
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv] * 2:
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv]: 
          0;
      }
    }

  if(input->FMEnable)
  {
    SAD_a = block_a.available ? ((list==1) ? all_bwmincost[block_a.pos_x][block_a.pos_y][0][FME_blocktype][0]:all_mincost[block_a.pos_x][block_a.pos_y][ref_frame][FME_blocktype][0]):0;
    SAD_b = block_b.available ? ((list==1) ? all_bwmincost[block_b.pos_x][block_b.pos_y][0][FME_blocktype][0]:all_mincost[block_b.pos_x][block_b.pos_y][ref_frame][FME_blocktype][0]):0;
    SAD_d = block_d.available ? ((list==1) ? all_bwmincost[block_d.pos_x][block_d.pos_y][0][FME_blocktype][0]:all_mincost[block_d.pos_x][block_d.pos_y][ref_frame][FME_blocktype][0]):0;
    SAD_c = block_c.available ? ((list==1) ? all_bwmincost[block_c.pos_x][block_c.pos_y][0][FME_blocktype][0]:all_mincost[block_c.pos_x][block_c.pos_y][ref_frame][FME_blocktype][0]):SAD_d;
  }

	//该预测方式，是通过邻近块所用的参考帧是否为当前指定参考帧来进行预测方式选择的。
    switch (mvPredType)
    {
	//通过中值预测。
    case MVPRED_MEDIAN:
	   // b块和c块都不可用时， 使用a块运动矢量作为预测
      if(!(block_b.available || block_c.available))
      {
        pred_vec = mv_a;
        if(input->FMEnable) temp_pred_SAD[hv] = SAD_a;
      }
      else	//中值=a+b+c-最大-最小
      {
        pred_vec = mv_a+mv_b+mv_c-min(mv_a,min(mv_b,mv_c))-max(mv_a,max(mv_b,mv_c));
      }
      if(input->FMEnable)
      {
         if (pred_vec == mv_a && SAD_a != 0) temp_pred_SAD[hv] = SAD_a;
         else if (pred_vec == mv_b && SAD_b!=0) temp_pred_SAD[hv] = SAD_b;
              else temp_pred_SAD[hv] = SAD_c;
      }
      break;
	//通过左方块预测
    case MVPRED_L:
      pred_vec = mv_a;
      if(input->FMEnable) temp_pred_SAD[hv] = SAD_a;
      break;
	//通过上方块预测
    case MVPRED_U:
      pred_vec = mv_b;
      if(input->FMEnable) temp_pred_SAD[hv] = SAD_b;
      break;
	////通过左上方块预测
    case MVPRED_UR:
      pred_vec = mv_c;
      if(input->FMEnable) temp_pred_SAD[hv] = SAD_c;
      break;
    default:
      break;
    }

    pmv[hv] = pred_vec;				//记录预测的运动矢量

  }

  if(input->FMEnable) pred_SAD_space = temp_pred_SAD[0]>temp_pred_SAD[1]?temp_pred_SAD[1]:temp_pred_SAD[0];
}

/*!
 ************************************************************************
 * \brief
 *    Initialize the motion search
 *	  初始化运动估计
 ************************************************************************
 */
void
Init_Motion_Search_Module ()
{
  int bits, i, imin, imax, k, l;

  int search_range               = input->search_range;						//寻找范围
  int number_of_reference_frames = img->max_num_references;					//参考帧数
  int max_search_points          = (2*search_range+1)*(2*search_range+1);	//最多搜索点
  int max_ref_bits               = 1 + 2 * (int)floor(log(max(16,number_of_reference_frames+1)) / log(2) + 1e-10);	//最大的参考帧(bits)
  int max_ref                    = (1<<((max_ref_bits>>1)+1))-1;			//最大的参考帧
  int number_of_subpel_positions = 4 * (2*search_range+3);
  int max_mv_bits                = 3 + 2 * (int)ceil (log(number_of_subpel_positions+1) / log(2) + 1e-10);			//最大运动适量残差(bits)
  max_mvd                        = (1<<( max_mv_bits >>1)   )-1;													//最大运动适量残差


  //=====   CREATE ARRAYS   =====
  //-----------------------------
  if ((spiral_search_x = (int*)calloc(max_search_points, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: spiral_search_x");
  if ((spiral_search_y = (int*)calloc(max_search_points, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: spiral_search_y");
  if ((mvbits = (int*)calloc(2*max_mvd+1, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: mvbits");
  if ((refbits = (int*)calloc(max_ref, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: refbits");
  if ((byte_abs = (int*)calloc(512, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: byte_abs");

  get_mem4Dint (&motion_cost, 8, 2, img->max_num_references+1, 4);

  //--- set array offsets ---
  mvbits   += max_mvd;
  byte_abs += 256;


  //=====   INIT ARRAYS   =====
  //---------------------------
  //--- init array: motion vector bits ---
  mvbits[0] = 1;
  for (bits=3; bits<=max_mv_bits; bits+=2)
  {
    imax = 1    << (bits >> 1);
    imin = imax >> 1;

    for (i = imin; i < imax; i++)   mvbits[-i] = mvbits[i] = bits;
  }
  //--- init array: reference frame bits ---
  refbits[0] = 1;
  for (bits=3; bits<=max_ref_bits; bits+=2)
  {
    imax = (1   << ((bits >> 1) + 1)) - 1;
    imin = imax >> 1;

    for (i = imin; i < imax; i++)   refbits[i] = bits;
  }
  //--- init array: absolute value ---
  byte_abs[0] = 0;
  for (i=1; i<256; i++)   byte_abs[i] = byte_abs[-i] = i;
  //--- init array: search pattern ---
  //运动搜索时，是按照螺旋结构搜索，该处保存指定pos所对应的运动矢量
  spiral_search_x[0] = spiral_search_y[0] = 0;
  for (k=1, l=1; l<=max(1,search_range); l++)
  {
    for (i=-l+1; i< l; i++)
    {
      spiral_search_x[k] =  i;  spiral_search_y[k++] = -l;
      spiral_search_x[k] =  i;  spiral_search_y[k++] =  l;
    }
    for (i=-l;   i<=l; i++)
    {
      spiral_search_x[k] = -l;  spiral_search_y[k++] =  i;
      spiral_search_x[k] =  l;  spiral_search_y[k++] =  i;
    }
  }

#ifdef _FAST_FULL_ME_
  if(!input->FMEnable)
    InitializeFastFullIntegerSearch ();
#endif
}


/*!
 ************************************************************************
 * \brief
 *    Free memory used by motion search
 ************************************************************************
 */
void
Clear_Motion_Search_Module ()
{
  //--- correct array offset ---
  mvbits   -= max_mvd;
  byte_abs -= 256;

  //--- delete arrays ---
  free (spiral_search_x);
  free (spiral_search_y);
  free (mvbits);
  free (refbits);
  free (byte_abs);
  free_mem4Dint (motion_cost, 8, 2);

#ifdef _FAST_FULL_ME_
  if(!input->FMEnable)
    ClearFastFullIntegerSearch ();
#endif
}



/*!
 ***********************************************************************
 * \brief
 *    Full pixel block motion search
 ***********************************************************************
 */
int                                               //  ==> minimum motion cost after search
FullPelBlockMotionSearch (pel_t**   orig_pic,     // <--  original pixel values for the AxB block
                          int       ref,          // <--  reference frame (0... or -1 (backward))
                          int       list,
                          int       pic_pix_x,    // <--  absolute x-coordinate of regarded AxB block
                          int       pic_pix_y,    // <--  absolute y-coordinate of regarded AxB block
                          int       blocktype,    // <--  block type (1-16x16 ... 7-4x4)
                          int       pred_mv_x,    // <--  motion vector predictor (x) in sub-pel units
                          int       pred_mv_y,    // <--  motion vector predictor (y) in sub-pel units
                          int*      mv_x,         // <--> in: search center (x) / out: motion vector (x) - in pel units
                          int*      mv_y,         // <--> in: search center (y) / out: motion vector (y) - in pel units
                          int       search_range, // <--  1-d search range in pel units
                          int       min_mcost,    // <--  minimum motion cost (cost for center or huge value)
                          double    lambda)       // <--  lagrangian parameter for determining motion cost
{
  int   pos, cand_x, cand_y, y, x4, mcost;
  
  pel_t *orig_line, *ref_line;
  pel_t *(*get_ref_line)(int, pel_t*, int, int, int, int);

  int   list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;
  pel_t *ref_pic			= listX[list+list_offset][ref]->imgY_11;
  int   img_width     = listX[list+list_offset][ref]->size_x;
  int   img_height    = listX[list+list_offset][ref]->size_y;

  int   best_pos      = 0;                                        // position with minimum motion cost
  int   max_pos       = (2*search_range+1)*(2*search_range+1);    // number of search positions
  int   lambda_factor = LAMBDA_FACTOR (lambda);                   // factor for determining lagragian motion cost
  int   blocksize_y   = input->blc_size[blocktype][1];            // vertical block size
  int   blocksize_x   = input->blc_size[blocktype][0];            // horizontal block size
  int   blocksize_x4  = blocksize_x >> 2;                         // horizontal block size in 4-pel units
  int   pred_x        = (pic_pix_x << 2) + pred_mv_x;       // predicted position x (in sub-pel units)
  int   pred_y        = (pic_pix_y << 2) + pred_mv_y;       // predicted position y (in sub-pel units)
  int   center_x      = pic_pix_x + *mv_x;                        // center position x (in pel units)
  int   center_y      = pic_pix_y + *mv_y;                        // center position y (in pel units)
  int   check_for_00  = (blocktype==1 && !input->rdopt && img->type!=B_SLICE && ref==0);

  //===== set function for getting reference picture lines =====
  if ((center_x > search_range) && (center_x < img->width -1-search_range-blocksize_x) &&
      (center_y > search_range) && (center_y < img->height-1-search_range-blocksize_y)   )
  {
     get_ref_line = FastLineX;
  }
  else
  {
     get_ref_line = UMVLineX;
  }


  //===== loop over all search positions =====
  for (pos=0; pos<max_pos; pos++)
  {
    //--- set candidate position (absolute position in pel units) ---
    cand_x = center_x + spiral_search_x[pos];
    cand_y = center_y + spiral_search_y[pos];

    //--- initialize motion cost (cost for motion vector) and check ---
    mcost = MV_COST (lambda_factor, 2, cand_x, cand_y, pred_x, pred_y);
    if (check_for_00 && cand_x==pic_pix_x && cand_y==pic_pix_y)
    {
      mcost -= WEIGHTED_COST (lambda_factor, 16);
    }
    if (mcost >= min_mcost)   continue;

    //--- add residual cost to motion cost ---
    for (y=0; y<blocksize_y; y++)
    {
      ref_line  = get_ref_line (blocksize_x, ref_pic, cand_y+y, cand_x, img_height, img_width);
      orig_line = orig_pic [y];

      for (x4=0; x4<blocksize_x4; x4++)
      {
        mcost += byte_abs[ *orig_line++ - *ref_line++ ];
        mcost += byte_abs[ *orig_line++ - *ref_line++ ];
        mcost += byte_abs[ *orig_line++ - *ref_line++ ];
        mcost += byte_abs[ *orig_line++ - *ref_line++ ];
      }

      if (mcost >= min_mcost)
      {
        break;
      }
    }

    //--- check if motion cost is less than minimum cost ---
    if (mcost < min_mcost)
    {
      best_pos  = pos;
      min_mcost = mcost;
    }
  }


  //===== set best motion vector and return minimum motion cost =====
  if (best_pos)
  {
    *mv_x += spiral_search_x[best_pos];
    *mv_y += spiral_search_y[best_pos];
  }
  return min_mcost;
}


#ifdef _FAST_FULL_ME_
/*!
 ***********************************************************************
 * \brief
 *    Fast Full pixel block motion search
 ***********************************************************************
 */
int                                                   //  ==> minimum motion cost after search
FastFullPelBlockMotionSearch (pel_t**   orig_pic,     // <--  not used
                              int       ref,          // <--  reference frame (0... or -1 (backward))
                              int       list,
                              int       pic_pix_x,    // <--  absolute x-coordinate of regarded AxB block
                              int       pic_pix_y,    // <--  absolute y-coordinate of regarded AxB block
                              int       blocktype,    // <--  block type (1-16x16 ... 7-4x4)
                              int       pred_mv_x,    // <--  motion vector predictor (x) in sub-pel units
                              int       pred_mv_y,    // <--  motion vector predictor (y) in sub-pel units
                              int*      mv_x,         //  --> motion vector (x) - in pel units				<-------------记录运动矢量
                              int*      mv_y,         //  --> motion vector (y) - in pel units				<-------------记录运动矢量
                              int       search_range, // <--  1-d search range in pel units
                              int       min_mcost,    // <--  minimum motion cost (cost for center or huge value)
                              double    lambda)       // <--  lagrangian parameter for determining motion cost
{
  int   pos, offset_x, offset_y, cand_x, cand_y, mcost;

  int   max_pos       = (2*search_range+1)*(2*search_range+1);              // number of search positions
  int   lambda_factor = LAMBDA_FACTOR (lambda);                             // factor for determining lagragian motion cost
  int   best_pos      = 0;                                                  // position with minimum motion cost
  int   block_index;                                                        // block index for indexing SAD array
  int*  block_sad;                                                          // pointer to SAD array

  //这里，block_index是当前宏块，指定参考帧，指定搜寻块的编号
  block_index   = (pic_pix_y-img->opix_y)+((pic_pix_x-img->opix_x)>>2); //block index for indexing SAD array
  //这里，block_sad[pos]是当前宏块，指定参考帧指，指定搜寻块，在参考帧中的指定位置(pos)的sad
  block_sad     = BlockSAD[list][ref][blocktype][block_index];         // pointer to SAD array

  //===== set up fast full integer search if needed / set search center =====
  //search_setup_done, 当前搜寻块中，指定参考帧是否进行过搜索的标记。
  //若为0，则需要对当前宏块的所有搜寻块模式(4x4 4x8 8x4 8x8 16x8 8x16 16x16)进行运动矢量搜寻
  //并将所有sad保存在BlockSad[list][ref][type][block][pos]中，type指定分割模式, block指定当前模式下的搜寻块编号, pos指的参考帧中的搜寻起始点
  //pos可以换算成运动矢量
  //若为1，则对当前宏块，指定参考帧，已经进行过了搜索，所以就不用搜索了，直接当前宏块，指定参考帧，指定分割模式下，指定搜寻块的最佳pos
  if (!search_setup_done[list][ref])
  {
    SetupFastFullPelSearch (ref, list);
  }
  
  offset_x = search_center_x[list][ref] - img->opix_x;
  offset_y = search_center_y[list][ref] - img->opix_y;

  //===== cost for (0,0)-vector: it is done before, because MVCost can be negative =====
  if (!input->rdopt)
  {
    mcost = block_sad[pos_00[list][ref]] + MV_COST (lambda_factor, 2, 0, 0, pred_mv_x, pred_mv_y);

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos_00[list][ref];
    }
  }

  //===== loop over all search positions =====
  //取出在当前宏块，指定参考帧，指定搜寻块中所有pos对应代价最小的pos，转换为运动矢量，并记录保存
  for (pos=0; pos<max_pos; pos++, block_sad++)
  {
    //--- check residual cost ---
    if (*block_sad < min_mcost)
    {
      //--- get motion vector cost ---
      cand_x = offset_x + spiral_search_x[pos];
      cand_y = offset_y + spiral_search_y[pos];
      mcost  = *block_sad;
      mcost += MV_COST (lambda_factor, 2, cand_x, cand_y, pred_mv_x, pred_mv_y);

      //--- check motion cost ---
      if (mcost < min_mcost)
      {
        min_mcost = mcost;
        best_pos  = pos;
      }
    }
  }

  //===== set best motion vector and return minimum motion cost =====
  //当前宏块，指定参考帧，指定搜寻块，对应的最佳运动矢量
  *mv_x = offset_x + spiral_search_x[best_pos];
  *mv_y = offset_y + spiral_search_y[best_pos];
  return min_mcost;
}
#endif


/*!
 ***********************************************************************
 * \brief
 *    Calculate SA(T)D
 ***********************************************************************
 */
int
SATD (int* diff, int use_hadamard)
{
  int k, satd = 0, m[16], dd, *d=diff;
  
  if (use_hadamard)
  {
    /*===== hadamard transform =====*/
    m[ 0] = d[ 0] + d[12];
    m[ 4] = d[ 4] + d[ 8];
    m[ 8] = d[ 4] - d[ 8];
    m[12] = d[ 0] - d[12];
    m[ 1] = d[ 1] + d[13];
    m[ 5] = d[ 5] + d[ 9];
    m[ 9] = d[ 5] - d[ 9];
    m[13] = d[ 1] - d[13];
    m[ 2] = d[ 2] + d[14];
    m[ 6] = d[ 6] + d[10];
    m[10] = d[ 6] - d[10];
    m[14] = d[ 2] - d[14];
    m[ 3] = d[ 3] + d[15];
    m[ 7] = d[ 7] + d[11];
    m[11] = d[ 7] - d[11];
    m[15] = d[ 3] - d[15];
    
    d[ 0] = m[ 0] + m[ 4];
    d[ 8] = m[ 0] - m[ 4];
    d[ 4] = m[ 8] + m[12];
    d[12] = m[12] - m[ 8];
    d[ 1] = m[ 1] + m[ 5];
    d[ 9] = m[ 1] - m[ 5];
    d[ 5] = m[ 9] + m[13];
    d[13] = m[13] - m[ 9];
    d[ 2] = m[ 2] + m[ 6];
    d[10] = m[ 2] - m[ 6];
    d[ 6] = m[10] + m[14];
    d[14] = m[14] - m[10];
    d[ 3] = m[ 3] + m[ 7];
    d[11] = m[ 3] - m[ 7];
    d[ 7] = m[11] + m[15];
    d[15] = m[15] - m[11];
    
    m[ 0] = d[ 0] + d[ 3];
    m[ 1] = d[ 1] + d[ 2];
    m[ 2] = d[ 1] - d[ 2];
    m[ 3] = d[ 0] - d[ 3];
    m[ 4] = d[ 4] + d[ 7];
    m[ 5] = d[ 5] + d[ 6];
    m[ 6] = d[ 5] - d[ 6];
    m[ 7] = d[ 4] - d[ 7];
    m[ 8] = d[ 8] + d[11];
    m[ 9] = d[ 9] + d[10];
    m[10] = d[ 9] - d[10];
    m[11] = d[ 8] - d[11];
    m[12] = d[12] + d[15];
    m[13] = d[13] + d[14];
    m[14] = d[13] - d[14];
    m[15] = d[12] - d[15];
    
    d[ 0] = m[ 0] + m[ 1];
    d[ 1] = m[ 0] - m[ 1];
    d[ 2] = m[ 2] + m[ 3];
    d[ 3] = m[ 3] - m[ 2];
    d[ 4] = m[ 4] + m[ 5];
    d[ 5] = m[ 4] - m[ 5];
    d[ 6] = m[ 6] + m[ 7];
    d[ 7] = m[ 7] - m[ 6];
    d[ 8] = m[ 8] + m[ 9];
    d[ 9] = m[ 8] - m[ 9];
    d[10] = m[10] + m[11];
    d[11] = m[11] - m[10];
    d[12] = m[12] + m[13];
    d[13] = m[12] - m[13];
    d[14] = m[14] + m[15];
    d[15] = m[15] - m[14];
    
    /*===== sum up =====*/
    for (dd=diff[k=0]; k<16; dd=diff[++k])
    {
      satd += (dd < 0 ? -dd : dd);
    }
    satd >>= 1;
  }
  else
  {
    /*===== sum up =====*/
    for (k = 0; k < 16; k++)
    {
      satd += byte_abs [diff [k]];
    }
  }
  
  return satd;
}



/*!
 ***********************************************************************
 * \brief
 *    Sub pixel block motion search
 ***********************************************************************
 */
int                                               //  ==> minimum motion cost after search
SubPelBlockMotionSearch (pel_t**   orig_pic,      // <--  original pixel values for the AxB block
                         int       ref,           // <--  reference frame (0... or -1 (backward))
                         int       list,          // <--  reference picture list 
                         int       pic_pix_x,     // <--  absolute x-coordinate of regarded AxB block
                         int       pic_pix_y,     // <--  absolute y-coordinate of regarded AxB block
                         int       blocktype,     // <--  block type (1-16x16 ... 7-4x4)
                         int       pred_mv_x,     // <--  motion vector predictor (x) in sub-pel units
                         int       pred_mv_y,     // <--  motion vector predictor (y) in sub-pel units
                         int*      mv_x,          // <--> in: search center (x) / out: motion vector (x) - in pel units
                         int*      mv_y,          // <--> in: search center (y) / out: motion vector (y) - in pel units
                         int       search_pos2,   // <--  search positions for    half-pel search  (default: 9)		半像素搜索位置数
                         int       search_pos4,   // <--  search positions for quarter-pel search  (default: 9)		1/4像素搜索位置数
                         int       min_mcost,     // <--  minimum motion cost (cost for center or huge value)		最小代价
                         double    lambda         // <--  lagrangian parameter for determining motion cost
                         )
{
  int   diff[16], *d;
  int   pos, best_pos, mcost, abort_search;
  int   y0, x0, ry0, rx0, ry;
  int   cand_mv_x, cand_mv_y;
  int   max_pos_x4, max_pos_y4;
  pel_t *orig_line;
  pel_t **ref_pic;      
  StorablePicture *ref_picture;
  int   lambda_factor   = LAMBDA_FACTOR (lambda);
  int   mv_shift        = 0;
  int   check_position0 = (blocktype==1 && *mv_x==0 && *mv_y==0 && input->hadamard && !input->rdopt && img->type!=B_SLICE && ref==0);
  int   blocksize_x     = input->blc_size[blocktype][0];
  int   blocksize_y     = input->blc_size[blocktype][1];
  // 1/4像素时，图像的坐标位置
  int   pic4_pix_x      = (pic_pix_x << 2);
  int   pic4_pix_y      = (pic_pix_y << 2);
  int   min_pos2        = (input->hadamard ? 0 : 1);
  int   max_pos2        = (input->hadamard ? max(1,search_pos2) : search_pos2);
  int   list_offset     = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  int  apply_weights = ( (active_pps->weighted_pred_flag && (img->type == P_SLICE || img->type == SP_SLICE)) ||
                         (active_pps->weighted_bipred_idc && (img->type == B_SLICE)));  

  int   img_width, img_height;
  
  //设置参考帧!!但其实并没有用ref_picture，因为这个只保存了整数级的像素，而该函数是子像素级的运动搜素
  ref_picture     = listX[list+list_offset][ref];

  //ref_pic是升采样后的图像像素，适合用于子像素级的运动搜索
  if (apply_weights)
  {
    ref_pic = listX[list+list_offset][ref]->imgY_ups_w;
  }
  else
    ref_pic = listX[list+list_offset][ref]->imgY_ups;

  img_width  = ref_picture->size_x;
  img_height = ref_picture->size_y;

  max_pos_x4      = ((ref_picture->size_x - blocksize_x+1)<<2);			//x方向上的最大搜索位置
  max_pos_y4      = ((ref_picture->size_y - blocksize_y+1)<<2);			//y方向上的最大搜索位置
  
  /*********************************
   *****                       *****
   *****  HALF-PEL REFINEMENT  *****
   *****                       *****
   *********************************/
  //===== convert search center to quarter-pel units =====
  // 1/4像素时的运动矢量
  *mv_x <<= 2;
  *mv_y <<= 2;
  //===== set function for getting pixel values =====
  if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 2) &&
      (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 2)   )
  {
    PelY_14 = FastPelY_14;
  }
  else
  {
    PelY_14 = UMVPelY_14;
  }
  //===== loop over search positions =====
  //以mv_x, mv_y为中心，遍历包括中心在内的周围9个pos，以选取最佳的pos
  for (best_pos = 0, pos = min_pos2; pos < max_pos2; pos++)
  {
	//指定pos的运动矢量, 1/2像素级别
    cand_mv_x = *mv_x + (spiral_search_x[pos] << 1);    // quarter-pel units
    cand_mv_y = *mv_y + (spiral_search_y[pos] << 1);    // quarter-pel units

    //----- set motion vector cost -----
	//计算mv_cost, 实际上是没有算sad(satd)的mv_cost.
    mcost = MV_COST (lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);
    if (check_position0 && pos==0)
    {
      mcost -= WEIGHTED_COST (lambda_factor, 16);
    }

	//如果没有算sad(satd)时，mv_cost都超过了min_mcost, 则忽略这个pos，直接看下一个....
    if (mcost >= min_mcost) continue;

    //----- add up SATD -----
	//y0是整数级像素单位， blocksize_y是当前搜索块y方向上的长度(像素单位)，y0+=4是让y0步进1个4x4子块
	//也就是说该循环将搜寻块分成了blocksize_y/4行(4x4单位)
	//++其实下面这两个循环，就是将当前搜寻块，分成了很多的4x4子块
	//++外层的循环，是按行遍历这些子块
	//++里层的循环，是按列遍历这些子块，以计算所有子块的satd
    for (y0=0, abort_search=0; y0<blocksize_y && !abort_search; y0+=4)
    {
	  //pic_pix_y+y0是当前搜寻块，当前4x4分块的左上角坐标. <<2 将其转换为了子像素级
	  //+cand_mv_y，将其转换为了当前搜寻块，当前4x4分块的左上角坐标,子像素级别
      ry0 = ((pic_pix_y+y0)<<2) + cand_mv_y;							//ry0, 当前搜寻块，当前4x4分块，指定pos在参考帧中4x4块的左上角坐标,子像素级别

	  //x0是整数级像素单位, blocksize_x是当前搜索块的x方向长度(像素单位)，x0+=4是让x0步进1个4x4子块
	  //也就是说这个循环是计算当前行(4x4子块单位)所有4x4子块的sad(satd)之和
      for (x0=0; x0<blocksize_x; x0+=4)
      {
		//单个循环，是计算一个4x4子块的satd之和
        rx0 = ((pic_pix_x+x0)<<2) + cand_mv_x;							//rx0, 当前搜寻块，当前4x4分块，指定pos在参考帧中4x4块的左上角坐标,子像素级别
        d   = diff;																		//因为要重新进行satd的计算，d指针恢复头位置

		//从宏块中读取第y0行的数据
		//注意，对rx、ry进行移动时，都是进行4的倍数移动!
		//这是因为宏块中相邻像素时整数级的位置差异(即在子像素中，是4倍的位置差异)，所以对应相减时，参考帧中的像素应该移动4的倍数.
        orig_line = orig_pic [y0  ];    ry=ry0;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);


		//从宏块中读取第y0+1行的数据
        orig_line = orig_pic [y0+1];    ry=ry0+4;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

		//从宏块中读取第y0+2行的数据
        orig_line = orig_pic [y0+2];    ry=ry0+8;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

		//从宏块中读取第y0+4行的数据
        orig_line = orig_pic [y0+3];    ry=ry0+12;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d        = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

		// 计算当前4x4子块的satd，累积。当satd超过了最小的satd时间，则退出循环，直接看下一个pos
		// 因为这是该模式已经无需再计算下去, 反正不会选这儿模式的
        if ((mcost += SATD (diff, input->hadamard)) > min_mcost)		//diff即前面进行计算的d, 保存的预测误差矩阵，这里计算预测误差的satd
        {
          abort_search = 1;
          break;
        }
      }
    }

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }
  }
  if (best_pos)
  {
    *mv_x += (spiral_search_x [best_pos] << 1);
    *mv_y += (spiral_search_y [best_pos] << 1);
  }


  /************************************
   *****                          *****
   *****  QUARTER-PEL REFINEMENT  *****
   *****                          *****
   ************************************/
  //跟上述1/2子像素过程差不多。
  //===== set function for getting pixel values =====
  if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 1) &&
      (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 1)   )
  {
    PelY_14 = FastPelY_14;
  }
  else
  {
    PelY_14 = UMVPelY_14;
  }
  //===== loop over search positions =====
  for (best_pos = 0, pos = 1; pos < search_pos4; pos++)
  {
    cand_mv_x = *mv_x + spiral_search_x[pos];    // quarter-pel units
    cand_mv_y = *mv_y + spiral_search_y[pos];    // quarter-pel units

    //----- set motion vector cost -----
    mcost = MV_COST (lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);

    if (mcost >= min_mcost) continue;

    //----- add up SATD -----
    for (y0=0, abort_search=0; y0<blocksize_y && !abort_search; y0+=4)
    {
      ry0 = ((pic_pix_y+y0)<<2) + cand_mv_y;

      for (x0=0; x0<blocksize_x; x0+=4)
      {
        rx0 = ((pic_pix_x+x0)<<2) + cand_mv_x;
        d   = diff;

        orig_line = orig_pic [y0  ];    ry=ry0;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+1];    ry=ry0+4;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+2];    ry=ry0+8;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+3];    ry=ry0+12;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d        = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        if ((mcost += SATD (diff, input->hadamard)) > min_mcost)
        {
          abort_search = 1;
          break;
        }
      }
    }

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }
  }
  if (best_pos)
  {
    *mv_x += spiral_search_x [best_pos];
    *mv_y += spiral_search_y [best_pos];
  }

  //===== return minimum motion cost =====
  return min_mcost;
}



/*!
 ***********************************************************************
 * \brief
 *    Block motion search
 *	  块的运动矢量搜寻
 ***********************************************************************
 */
int                                         //!< minimum motion cost after search
BlockMotionSearch (int       ref,           //!< reference idx
                   int       list,          //!< reference pciture list
                   int       mb_x,          //!< x-coordinate inside macroblock
                   int       mb_y,          //!< y-coordinate inside macroblock
                   int       blocktype,     //!< block type (1-16x16 ... 7-4x4)
                   int       search_range,  //!< 1-d search range for integer-position search
                   double    lambda         //!< lagrangian parameter for determining motion cost
                   )
{
  static pel_t   orig_val [256];
  static pel_t  *orig_pic  [16] = {orig_val,     orig_val+ 16, orig_val+ 32, orig_val+ 48,
                                   orig_val+ 64, orig_val+ 80, orig_val+ 96, orig_val+112,
                                   orig_val+128, orig_val+144, orig_val+160, orig_val+176,
                                   orig_val+192, orig_val+208, orig_val+224, orig_val+240};

  int       pred_mv_x, pred_mv_y, mv_x, mv_y, i, j;

  int       max_value = (1<<20);
  int       min_mcost = max_value;

  int       block_x   = (mb_x>>2);						//将像素为单位的块位置，转换为4x4块为单位的块位置(该位置是指的宏块内部的搜寻块位置)
  int       block_y   = (mb_y>>2);						//将像素为单位的块位置，转换为4x4块为单位的块位置(该位置是指的宏块内部的搜寻块位置)
  
  int       bsx       = input->blc_size[blocktype][0];	//搜寻块大小
  int       bsy       = input->blc_size[blocktype][1];	//搜寻块大小

  int       pic_pix_x = img->opix_x + mb_x;
  int       pic_pix_y = img->opix_y + mb_y;

  int*      pred_mv;

  int***    mv_array  = enc_picture->mv[list];			//运动适量mv_array[blk_x][blk_y][0/1]

  int****** all_mv    = img->all_mv;					//所有运动矢量all_mv

#ifdef WIN32
  struct _timeb tstruct1;
  struct _timeb tstruct2;
#else
  struct timeb tstruct1;
  struct timeb tstruct2;
#endif
  
  int me_tmp_time;

  int  N_Bframe=0, n_Bframe=0;

  //input->FMEnable 是 FastMotionEstimate的意思
  if(input->FMEnable)
  {
    N_Bframe = input->successive_Bframe;
    n_Bframe =(N_Bframe) ? ((Bframe_ctr%N_Bframe)+1) : 0 ;
  }

  // 预测运动适量
   pred_mv = img->pred_mv[block_x][block_y][list][ref][blocktype];

  //==================================
  //=====   GET ORIGINAL BLOCK   =====
  //==================================
  //得到当前块的像素值。pic_pix_y, pic_pix_x是当前块在图像中的位置(像素单位)
  for (j = 0; j < bsy; j++)
  {
    for (i = 0; i < bsx; i++)
    {
      orig_pic[j][i] = imgY_org[pic_pix_y+j][pic_pix_x+i];
    }
  }

//快速运动估计*****************************************************************************************************************************************
  if(input->FMEnable)
  {
    
    if(blocktype>6)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][5][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][5][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][5][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][5][0]);
      pred_SAD_uplayer   /= 2; 
      
    }
    else if(blocktype>4)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][4][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][4][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][4][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][4][0]);
      pred_SAD_uplayer   /= 2; 
      
    }
    else if(blocktype == 4)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][2][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][2][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][2][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][2][0]);
      pred_SAD_uplayer   /= 2; 
    }
    else if(blocktype > 1)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][1][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][1][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][1][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][1][0]);
      pred_SAD_uplayer   /= 2; 
    }
    
    if ((img->type==B_SLICE)&& (img->nal_reference_idc>0))
    {
      if(blocktype>6)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][5][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][5][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][5][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][5][0]);
        pred_SAD_uplayer   /= 2; 
      }
      else if(blocktype>4)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][4][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][4][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][4][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][4][0]);
        pred_SAD_uplayer   /= 2; 
      }
      else if(blocktype == 4)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][2][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][2][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][2][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][2][0]);
        pred_SAD_uplayer   /= 2; 
      }
      else if(blocktype > 1)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][1][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][1][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][1][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][1][0]);
        pred_SAD_uplayer   /= 2; 
      }
    }
    
    pred_SAD_uplayer = flag_intra_SAD ? 0 : pred_SAD_uplayer;// for irregular motion
    
    //Coordinate prediction
    if (img->number > ref+1)
    {
      pred_SAD_time = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][0];
      pred_MV_time[0] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][1];
      pred_MV_time[1] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][2];
    }
    
    if(list==1 && (Bframe_ctr%N_Bframe) > 1) 
    {
      pred_SAD_time = all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][0];
      pred_MV_time[0] = (int)(all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][1] * ((n_Bframe==1) ? (N_Bframe) : (N_Bframe-n_Bframe+1.0)/(N_Bframe-n_Bframe+2.0)) );//should add a factor
      pred_MV_time[1] = (int)(all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][2] *((n_Bframe==1) ? (N_Bframe) : (N_Bframe-n_Bframe+1.0)/(N_Bframe-n_Bframe+2.0)) );//should add a factor
    }
    
    if (input->PicInterlace == FIELD_CODING) 
    {
      if (img->type == P_SLICE && ref > 1)
      {
        pred_SAD_ref = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-2)][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-2)][blocktype][1];
        pred_MV_ref[0] = (int)(pred_MV_ref[0]*((ref>>1)+1)/(float)((ref>>1)));
        pred_MV_ref[1] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-2)][blocktype][2];
        pred_MV_ref[1] = (int)(pred_MV_ref[1]*((ref>>1)+1)/(float)((ref>>1)));
      }
      if (img->type == B_SLICE && list==0 && (ref==0 || ref==1) )
      {
        pred_SAD_ref = all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] =(int) (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][1]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); //should add a factor
        pred_MV_ref[1] =(int) ( all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][2]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); 
      }
    }
    else //frame case
    {
      if (ref > 0)
      {//field_mode top_field
        pred_SAD_ref = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-1)][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-1)][blocktype][1];
        pred_MV_ref[0] = (int)(pred_MV_ref[0]*(ref+1)/(float)(ref));
        pred_MV_ref[1] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-1)][blocktype][2];
        pred_MV_ref[1] = (int)(pred_MV_ref[1]*(ref+1)/(float)(ref));
      }
      if (img->type == B_SLICE && (list==0 && ref==0)) //B frame forward prediction, first ref
      {
        pred_SAD_ref = all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] =(int) (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][1]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); //should add a factor
        pred_MV_ref[1] =(int) ( all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][2]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); 
      }
    }
 }
//上面为快速运动估计所需要的***************************************************************************************************************************
//***************************************************************************************************************************************************** 

//=====================================================================================================================================================
//================================================== 1).GET MOTION VECTOR PREDICTOR   =================================================================
//=====================================================================================================================================================

  if (input->FMEnable) 
    FME_blocktype=blocktype;

  //邻近已编码块的参考帧信息 和 运动矢量信息，由enc_picture提供
  SetMotionVectorPredictor (pred_mv, enc_picture->ref_idx, enc_picture->mv, ref, list, block_x, block_y, bsx, bsy);

  //注意，这个运动矢量是亚像素级的
  pred_mv_x = pred_mv[0];
  pred_mv_y = pred_mv[1];

//=====================================================================================================================================================
//==================================================== 2). INTEGER-PEL SEARCH   =======================================================================
//=====================================================================================================================================================
#ifdef WIN32
  _ftime( &tstruct1 );    // start time ms 计时器打开
#else
  ftime(&tstruct1);
#endif

  // 整数像素运动矢量搜索
  if(input->FMEnable)
  {
	//转换为整数级的运动矢量
	//mv初始化为预测mv
    mv_x = pred_mv_x / 4;
    mv_y = pred_mv_y / 4;
    if (!input->rdopt)
    {
      //--- adjust search center so that the (0,0)-vector is inside ---
      mv_x = max (-search_range, min (search_range, mv_x));
      mv_y = max (-search_range, min (search_range, mv_y));
    }
    
	//mv初始化为预测mv	 
	min_mcost = FastIntegerPelBlockMotionSearch(orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
      pred_mv_x, pred_mv_y, &mv_x, &mv_y, search_range,
      min_mcost, lambda);
    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3
	//bsx bsy是像素级的搜索块大小
	//该处将最小代价保存起来。是按4x4块为单位进行保存的
    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        if(list == 0) 
          all_mincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][0] = min_mcost;
        else
          all_bwmincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][0] = min_mcost; 
      }
    }
  }
  else
  {
#ifndef _FAST_FULL_ME_

    //--- set search center ---
    mv_x = pred_mv_x / 4;
    mv_y = pred_mv_y / 4;
    if (!input->rdopt)
    {
      //--- adjust search center so that the (0,0)-vector is inside ---
      mv_x = max (-search_range, min (search_range, mv_x));
      mv_y = max (-search_range, min (search_range, mv_y));
    }
    
    //--- perform motion search ---
    min_mcost = FullPelBlockMotionSearch     (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                              pred_mv_x, pred_mv_y, &mv_x, &mv_y, search_range,
                                              min_mcost, lambda);

#else

    // comments:   - orig_pic is not used  -> be careful
    //             - search center is automatically determined
	//运动搜索，将当前宏块，指定参考帧，指定搜寻块的运动矢量保存在mv_x, mv_y中
    min_mcost = FastFullPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                              pred_mv_x, pred_mv_y, &mv_x, &mv_y, search_range,
                                              min_mcost, lambda);

#endif
  }

// 关闭定时器，计算所用时间
#ifdef WIN32
      _ftime(&tstruct2);   // end time ms
#else
      ftime(&tstruct2);    // end time ms
#endif
      
      me_tmp_time=(tstruct2.time*1000+tstruct2.millitm) - (tstruct1.time*1000+tstruct1.millitm); 
      me_tot_time += me_tmp_time;
      me_time += me_tmp_time;

//=====================================================================================================================================================
//==================================================== 3). SUB-PEL SEARCH   ===========================================================================
//=====================================================================================================================================================
  if (input->hadamard)
  {
    min_mcost = max_value;
  }

  if(input->FMEnable)
  {
    if(blocktype >3)
    {
      min_mcost =  FastSubPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                                pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                                min_mcost, lambda, /*useABT*/0);
    }
    else
    {
	  //以整数搜索点mv_x, mv_y作为半像素图像的搜索中心
	  //以半像素搜索点mv_x, mv_y作为1/4像素图像的搜索中心
	  //子像素搜索时，通过input->hadamard来指定使用satd或是sad.
      min_mcost =  SubPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                            pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                            min_mcost, lambda);
    }


    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        if(list == 0)
        {
          all_mincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][1] = mv_x;
          all_mincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][2] = mv_y;
        }
        else
        {
          all_bwmincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][1] = mv_x;
          all_bwmincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][2] = mv_y;
          
        }
      }
    }
  }
  else//if(input->FMEnable)
  {
    min_mcost =  SubPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                          pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                          min_mcost, lambda);
  }


  if (!input->rdopt)
  {
    // Get the skip mode cost
    if (blocktype == 1 && (img->type == P_SLICE||img->type == SP_SLICE))
    {
      int cost;

      FindSkipModeMotionVector ();

      cost  = GetSkipCostMB (lambda);
      cost -= (int)floor(8*lambda+0.4999);

      if (cost < min_mcost)
      {
        min_mcost = cost;
        mv_x      = img->all_mv [0][0][0][0][0][0];
        mv_y      = img->all_mv [0][0][0][0][0][1];
      }
    }
  }

//=====================================================================================================================================================
//============================================== 4). SET MV'S AND RETURN MOTION COST   ================================================================
//=====================================================================================================================================================

// mv_x = 0;
//  mv_y = 0;
  if(input->FMEnable)
  {
    int h4x4blkno = (img->pix_x>>2)+block_x;	//4x4块为单位
    int v4x4blkno = (img->pix_y>>2)+block_y;
    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        all_mv[block_x+i][block_y+j][list][ref][blocktype][0] = mv_x;
        all_mv[block_x+i][block_y+j][list][ref][blocktype][1] = mv_y;
        mv_array[h4x4blkno+i][v4x4blkno+j][0] = mv_x;
        mv_array[h4x4blkno+i][v4x4blkno+j][1] = mv_y;
      }
    }
  }
  else
  {
    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
		//all_mv保存了当前宏块各个4x4子块的运动矢量和参考帧
		//block_x, block_y都是宏块内部的各4x4分块位置
        all_mv[block_x+i][block_y+j][list][ref][blocktype][0] = mv_x;
        all_mv[block_x+i][block_y+j][list][ref][blocktype][1] = mv_y;
      }
    }
  }

  return min_mcost;
}


/*!
 ***********************************************************************
 * \brief
 *    Motion Cost for Bidirectional modes
 ***********************************************************************
 */
int BIDPartitionCost (int   blocktype,
                      int   block8x8,
                      int   fw_ref,
                      int   bw_ref,
                      int   lambda_factor)
{
  static int  bx0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,2,0,2}};
  static int  by0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,0,0,0}, {0,0,2,2}};

  int   diff[16];
  int   pic_pix_x, pic_pix_y, block_x, block_y;
  int   v, h, mcost, i, j, k;
  int   mvd_bits  = 0;
  int   parttype  = (blocktype<4?blocktype:4);
  int   step_h0   = (input->blc_size[ parttype][0]>>2);
  int   step_v0   = (input->blc_size[ parttype][1]>>2);
  int   step_h    = (input->blc_size[blocktype][0]>>2);
  int   step_v    = (input->blc_size[blocktype][1]>>2);
  int   bxx, byy;                               // indexing curr_blk

  int   ******all_mv = img->all_mv;
  int   ******  p_mv = img->pred_mv;

  //----- cost for motion vector bits -----
  for (v=by0[parttype][block8x8]; v<by0[parttype][block8x8]+step_v0; v+=step_v)
  for (h=bx0[parttype][block8x8]; h<bx0[parttype][block8x8]+step_h0; h+=step_h)
  {
    mvd_bits += mvbits[ all_mv [h][v][LIST_0][fw_ref][blocktype][0] - p_mv[h][v][LIST_0][fw_ref][blocktype][0] ];
    mvd_bits += mvbits[ all_mv [h][v][LIST_0][fw_ref][blocktype][1] - p_mv[h][v][LIST_0][fw_ref][blocktype][1] ];

    mvd_bits += mvbits[ all_mv [h][v][LIST_1][bw_ref][blocktype][0] - p_mv[h][v][LIST_1][bw_ref][blocktype][0] ];
    mvd_bits += mvbits[ all_mv [h][v][LIST_1][bw_ref][blocktype][1] - p_mv[h][v][LIST_1][bw_ref][blocktype][1] ];
  }

  mcost = WEIGHTED_COST (lambda_factor, mvd_bits);

  //----- cost of residual signal -----
  for (byy=0, v=by0[parttype][block8x8]; v<by0[parttype][block8x8]+step_v0; byy+=4, v++)
  {
    pic_pix_y = img->opix_y + (block_y = (v<<2));

    for (bxx=0, h=bx0[parttype][block8x8]; h<bx0[parttype][block8x8]+step_h0; bxx+=4, h++)
    {
      pic_pix_x = img->opix_x + (block_x = (h<<2));

      LumaPrediction4x4 (block_x, block_y, 2, blocktype, blocktype, fw_ref, bw_ref);

      for (k=j=0; j<4; j++)
      for (  i=0; i<4; i++, k++)
      {
        diff[k] = imgY_org[pic_pix_y+j][pic_pix_x+i] - img->mpr[i+block_x][j+block_y];
      }
      mcost += SATD (diff, input->hadamard);
    }
  }
  return mcost;
}

/*!
 ************************************************************************
 * \brief
 *    Get cost for skip mode for an macroblock
 ************************************************************************
 */
int GetSkipCostMB (double lambda)
{
  int block_y, block_x, pic_pix_y, pic_pix_x, i, j, k;
  int diff[16];
  int cost = 0;

  for (block_y=0; block_y<16; block_y+=4)
  {
    pic_pix_y = img->opix_y + block_y;

    for (block_x=0; block_x<16; block_x+=4)
    {
      pic_pix_x = img->opix_x + block_x;

      //===== prediction of 4x4 block =====
      LumaPrediction4x4 (block_x, block_y, 0, 0, 0, 0, 0);

      //===== get displaced frame difference ======                
      for (k=j=0; j<4; j++)
        for (i=0; i<4; i++, k++)
        {
          diff[k] = imgY_org[pic_pix_y+j][pic_pix_x+i] - img->mpr[i+block_x][j+block_y];
        }
      cost += SATD (diff, input->hadamard);
    }
  }

  return cost;
}

/*!
 ************************************************************************
 * \brief
 *    Find motion vector for the Skip mode
 ************************************************************************
 */
void FindSkipModeMotionVector ()
{
  int bx, by;
  int ******all_mv = img->all_mv;

  int pmv[2];

  int zeroMotionAbove;
  int zeroMotionLeft;
  PixelPos mb_a, mb_b;
  int      a_mv_y = 0;
  int      a_ref_idx = 0;
  int      b_mv_y = 0;
  int      b_ref_idx = 0;

  Macroblock *currMB = &img->mb_data[img->current_mb_nr];
  
  getLuma4x4Neighbour(img->current_mb_nr,0,0,-1, 0,&mb_a);
  getLuma4x4Neighbour(img->current_mb_nr,0,0, 0,-1,&mb_b);
  
  if (mb_a.available)
  {
    a_mv_y    = enc_picture->mv[LIST_0][mb_a.pos_x][mb_a.pos_y][1];
    a_ref_idx = enc_picture->ref_idx[LIST_0][mb_a.pos_x][mb_a.pos_y];
    
    if (currMB->mb_field && !img->mb_data[mb_a.mb_addr].mb_field)
    {
      a_mv_y    /=2;
      a_ref_idx *=2;
    }
    if (!currMB->mb_field && img->mb_data[mb_a.mb_addr].mb_field)
    {
      a_mv_y    *=2;
      a_ref_idx >>=1;
    }
  }
  
  if (mb_b.available)
  {
    b_mv_y    = enc_picture->mv[LIST_0][mb_b.pos_x][mb_b.pos_y][1];
    b_ref_idx = enc_picture->ref_idx[LIST_0][mb_b.pos_x][mb_b.pos_y];
    
    if (currMB->mb_field && !img->mb_data[mb_b.mb_addr].mb_field)
    {
      b_mv_y    /=2;
      b_ref_idx *=2;
    }
    if (!currMB->mb_field && img->mb_data[mb_b.mb_addr].mb_field)
    {
      b_mv_y    *=2;
      b_ref_idx >>=1;
    }
  }
  
  zeroMotionLeft  = !mb_a.available ? 1 : a_ref_idx==0 && enc_picture->mv[LIST_0][mb_a.pos_x][mb_a.pos_y][0]==0 && a_mv_y==0 ? 1 : 0;
  zeroMotionAbove = !mb_b.available ? 1 : b_ref_idx==0 && enc_picture->mv[LIST_0][mb_b.pos_x][mb_b.pos_y][0]==0 && b_mv_y==0 ? 1 : 0;
  
  if (zeroMotionAbove || zeroMotionLeft)
  {
    for (by = 0;by < 4;by++)
      for (bx = 0;bx < 4;bx++)
      {
        all_mv [bx][by][0][0][0][0] = 0;
        all_mv [bx][by][0][0][0][1] = 0;
      }
  }
  else
  {
    SetMotionVectorPredictor (pmv, enc_picture->ref_idx, enc_picture->mv, 0, LIST_0, 0, 0, 16, 16);
    for (by = 0;by < 4;by++)
      for (bx = 0;bx < 4;bx++)
      {
        all_mv [bx][by][0][0][0][0] = pmv[0];
        all_mv [bx][by][0][0][0][1] = pmv[1];
      }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Get cost for direct mode for an 8x8 block
 ************************************************************************
 */
int Get_Direct_Cost8x8 (int block, double lambda)
{
  int block_y, block_x, pic_pix_y, pic_pix_x, i, j, k;
  int diff[16];
  int cost  = 0;
  int mb_y  = (block/2)<<3;
  int mb_x  = (block%2)<<3;

  for (block_y=mb_y; block_y<mb_y+8; block_y+=4)
  {
    pic_pix_y = img->opix_y + block_y;

    for (block_x=mb_x; block_x<mb_x+8; block_x+=4)
    {
      pic_pix_x = img->opix_x + block_x;

      if (direct_pdir[pic_pix_x>>2][pic_pix_y>>2]<0)
      {
        return (1<<30); //mode not allowed
      }

      //===== prediction of 4x4 block =====
      LumaPrediction4x4 (block_x, block_y, direct_pdir[pic_pix_x>>2][pic_pix_y>>2], 0, 0, 
                         direct_ref_idx[LIST_0][pic_pix_x>>2][pic_pix_y>>2], 
                         direct_ref_idx[LIST_1][pic_pix_x>>2][pic_pix_y>>2]);

      //===== get displaced frame difference ======                
      for (k=j=0; j<4; j++)
        for (i=0; i<4; i++, k++)
        {
          diff[k] = imgY_org[pic_pix_y+j][pic_pix_x+i] - img->mpr[i+block_x][j+block_y];

        }
      cost += SATD (diff, input->hadamard);
    }
  }

  return cost;
}



/*!
 ************************************************************************
 * \brief
 *    Get cost for direct mode for an macroblock
 ************************************************************************
 */
int Get_Direct_CostMB (double lambda)
{
  int i;
  int cost = 0;
  
  for (i=0; i<4; i++)
  {
    cost += Get_Direct_Cost8x8 (i, lambda);
    if (cost >= (1<<30)) return cost;
  }
  return cost;
}


/*!
 ************************************************************************
 * \brief
 *    Motion search for a partitin
 *	  对一个分块的运动矢量搜索
 *	  blocktype类型，block8x8分块编号，lambda拉格朗日系数。
 ************************************************************************
 */
void
PartitionMotionSearch (int    blocktype,
                       int    block8x8,
                       double lambda)
{
	// 各个模式下搜索块的起始坐标,以4x4块为坐标单位
	//(bx0[type][block], by0[type][block]) = (x,y)， 这就是指定类型下，指定块编号的，分块起始坐标。
  static int  bx0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,2,0,2}};
  static int  by0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,0,0,0}, {0,0,2,2}};

  int   **ref_array, ***mv_array;
  int   ref, v, h, mcost, search_range, i, j;
  int   pic_block_x, pic_block_y;
  int   bslice    = (img->type==B_SLICE);
  int   parttype  = (blocktype<4?blocktype:4);					//分块模式，只能是1,2,3,4. 大于4的都为4(1:16x16, 2:16x8, 3:8x16, 4:8x8)
  // input->blc_size[type][xy] 是指定模式下 指定xy分量的分块长度，像素为单位
  // 当blocktype>4时，步进step_h和step_v会以亚宏块的格式存在，而step_h0和step_v0最小也是8x8
  int   step_h0   = (input->blc_size[ parttype][0]>>2);			//当前搜索块所属分块模式的宽(x方向长度)，4个像素为单位
  int   step_v0   = (input->blc_size[ parttype][1]>>2);			//当前搜索块所属分块模式的高(y方向长度)，4个像素为单位
  int   step_h    = (input->blc_size[blocktype][0]>>2);			//x方向步进, 4个像素为单位
  int   step_v    = (input->blc_size[blocktype][1]>>2);			//y方向步进, 4个像素为单位
  int   list;													//参考帧列表的序号
  int   numlists;												//参考帧列表的表数
  int   list_offset;											//

  if (img->mb_data[img->current_mb_nr].mb_field)
  {
    if(img->current_mb_nr%2)
      list_offset = 4; // bottom field mb
    else
      list_offset = 2; // top field mb
  }
  else
  {
    list_offset = 0;  // no mb aff or frame mb
  }

  //参考帧列表的数量
  numlists=bslice?2:1;

  //===== LOOP OVER REFERENCE FRAMES =====
  //遍历所有参考帧，选取每个参考帧作为当前块的运动估计参考帧。
  for (list=0; list<numlists;list++)
  {
    for (ref=0; ref < listXsize[list+list_offset]; ref++)
    {
        //----- set search range ---
#ifdef _FULL_SEARCH_RANGE_
        if      (input->full_search == 2) search_range = input->search_range;
        else if (input->full_search == 1) search_range = input->search_range /  (min(ref,1)+1);
        else                              search_range = input->search_range / ((min(ref,1)+1) * min(2,blocktype));
#else
        search_range = input->search_range / ((min(ref,1)+1) * min(2,blocktype));
#endif
        
        //----- set arrays -----
        ref_array = enc_picture->ref_idx[list];			//参考帧矩阵，用于缓存4x4子块使用的参考帧
        mv_array  = enc_picture->mv[list];				//运动矢量矩阵，用于缓存4x4子块使用的运动适量
        
        //----- init motion cost -----
		//初始化指定块类型，指定参考帧列表，指定参考帧，指定块的代价
        motion_cost[blocktype][list][ref][block8x8] = 0;
        
        //===== LOOP OVER SUB MACRO BLOCK partitions
		// 遍历所有块的子块分区（1-4类型时无子块分区）
		// h和v，表明了当前搜索块在宏块中的位置(4x4单位的位置)

		// step_v0是当前搜索块在y方向上的长度，单位4x4块
        for (v=by0[parttype][block8x8]; v<by0[parttype][block8x8]+step_v0; v+=step_v)
        {
          pic_block_y = img->block_y + v;			//当前块在图像中的行位置，4x4块为单位
          
		  // step_h0是当前搜索块在x方向上的长度，单位4x4块
          for (h=bx0[parttype][block8x8]; h<bx0[parttype][block8x8]+step_h0; h+=step_h)
          {
            pic_block_x = img->block_x + h;			//当前块在图像中的列位置，4x4块为单位
            
            //--- motion search for block ---
			// 子块的运动矢量搜寻(传入的位置是像素为单位在宏块中的位置，所以h<<2, v<<2都是x4)
			// 对指定宏块类型，指定的搜索块，指定参考帧，进行运动搜索，并返回代价
            mcost = BlockMotionSearch     (ref, list, h<<2, v<<2, blocktype, search_range, lambda);		//当前搜寻块的mv	
            motion_cost[blocktype][list][ref][block8x8] += mcost;										//指定宏块类型，指定搜索块，指定参考帧的代价
            
			Event_MvSearch(h, v, list, ref, blocktype, img->all_mv);

            //--- set motion vectors and reference frame (for motion vector prediction) ---
			// 设置运动矢量在mv_array和ref_array(其实就是设置在enc_picture中)
			// mv_array[i][j], ref_array[i][j]都是以4x4子块作为坐标单位的
            for (j=0; j<step_v; j++)
              for (i=0; i<step_h; i++)
              {
				//all_mv保存了当前宏块各个4x4子块的运动矢量和参考帧
				//这里好像只是把all_mv
                mv_array  [pic_block_x+i][pic_block_y+j][0] = img->all_mv[h][v][list][ref][blocktype][0];	//enc_picture->mv[list][x][y][0]
                mv_array  [pic_block_x+i][pic_block_y+j][1] = img->all_mv[h][v][list][ref][blocktype][1];	//enc_picture->mv[list][x][y][1]
                ref_array [pic_block_x+i][pic_block_y+j]    = ref;											//enc_picture->ref_idx[list]
              }
          }
        }
    }
  }
}





extern int* last_P_no;
/*********************************************
 *****                                   *****
 *****  Calculate Direct Motion Vectors  *****
 *****                                   *****
 *********************************************/
void Get_Direct_Motion_Vectors ()
{

  int  block_x, block_y, pic_block_x, pic_block_y, opic_block_x, opic_block_y;
  int  ******all_mvs = img->all_mv;
  int  mv_scale;
  byte **    moving_block;
  int ****   co_located_mv;
  int ***    co_located_ref_idx;
  int64 ***    co_located_ref_id;
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  if ((img->MbaffFrameFlag)&&(currMB->mb_field))
  {
    if(img->current_mb_nr%2)
    {
      moving_block = Co_located->bottom_moving_block;
      co_located_mv = Co_located->bottom_mv;
      co_located_ref_idx = Co_located->bottom_ref_idx;
      co_located_ref_id = Co_located->bottom_ref_pic_id;
    }
    else
    {
      moving_block = Co_located->top_moving_block;
      co_located_mv = Co_located->top_mv;
      co_located_ref_idx = Co_located->top_ref_idx;
      co_located_ref_id = Co_located->top_ref_pic_id;
    }
  }
  else
  {
    moving_block = Co_located->moving_block;
    co_located_mv = Co_located->mv;
    co_located_ref_idx = Co_located->ref_idx;
    co_located_ref_id = Co_located->ref_pic_id;
  }

  if (img->direct_type)  //spatial direct mode copy from decoder
  {
    
    int fw_rFrameL, fw_rFrameU, fw_rFrameUL, fw_rFrameUR;
    int bw_rFrameL, bw_rFrameU, bw_rFrameUL, bw_rFrameUR; 
    int fw_rFrame,bw_rFrame;
    int pmvfw[2]={0,0},pmvbw[2]={0,0};

    PixelPos mb_left, mb_up, mb_upleft, mb_upright;              
    
    getLuma4x4Neighbour(img->current_mb_nr,0,0,-1, 0,&mb_left);
    getLuma4x4Neighbour(img->current_mb_nr,0,0, 0,-1,&mb_up);
    getLuma4x4Neighbour(img->current_mb_nr,0,0,16, -1,&mb_upright);
    getLuma4x4Neighbour(img->current_mb_nr,0,0, -1,-1,&mb_upleft);

    if (!img->MbaffFrameFlag)
    {
      fw_rFrameL = mb_left.available ? enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] : -1;
      fw_rFrameU = mb_up.available ? enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] : -1;
      fw_rFrameUL = mb_upleft.available ? enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] : -1;
      fw_rFrameUR = mb_upright.available ? enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] : fw_rFrameUL;      
      
      bw_rFrameL = mb_left.available ? enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] : -1;
      bw_rFrameU = mb_up.available ? enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] : -1;
      bw_rFrameUL = mb_upleft.available ? enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] : -1;
      bw_rFrameUR = mb_upright.available ? enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] : bw_rFrameUL;      
    }
    else
    {
      if (currMB->mb_field)
      {
        fw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field  || enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] < 0? 
          enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] : 
          enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] * 2: -1;

        fw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] < 0? 
          enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] : 
        enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] * 2: -1;

        fw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] < 0?         
          enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] : 
        enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] *2: -1;      

        fw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] < 0?
          enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] : 
        enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] * 2: fw_rFrameUL;      
        
        bw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] < 0? 
          enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] * 2: -1;

        bw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] < 0? 
          enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] * 2: -1;

        bw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] < 0?         
          enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] *2: -1;      

        bw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] < 0?         
          enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] * 2: bw_rFrameUL;              
      }
      else
      {
        fw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y]  < 0 ?
          enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] >> 1 : 
        enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y]: -1;
        
        fw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] >> 1 :  
        enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] : -1;
        
        fw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y]>> 1 : 
        enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] : -1;      
        
        fw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] < 0 ? 
          enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] >> 1 :  
        enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] : fw_rFrameUL;      
        
        bw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] >> 1 :  
        enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] : -1;
        
        bw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] >> 1 : 
        enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] : -1;
        
        bw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] >> 1 : 
        enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] : -1;      
        
        bw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] >> 1: 
        enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] : bw_rFrameUL;      
      }
    }
    
    fw_rFrame = (fw_rFrameL >= 0 && fw_rFrameU >= 0) ? min(fw_rFrameL,fw_rFrameU): max(fw_rFrameL,fw_rFrameU);
    fw_rFrame = (fw_rFrame >= 0 && fw_rFrameUR >= 0) ? min(fw_rFrame,fw_rFrameUR): max(fw_rFrame,fw_rFrameUR);
    
    bw_rFrame = (bw_rFrameL >= 0 && bw_rFrameU >= 0) ? min(bw_rFrameL,bw_rFrameU): max(bw_rFrameL,bw_rFrameU);
    bw_rFrame = (bw_rFrame >= 0 && bw_rFrameUR >= 0) ? min(bw_rFrame,bw_rFrameUR): max(bw_rFrame,bw_rFrameUR);        
    
    if (fw_rFrame >=0)
      SetMotionVectorPredictor (pmvfw, enc_picture->ref_idx, enc_picture->mv, fw_rFrame, LIST_0, 0, 0, 16, 16);
    
    if (bw_rFrame >=0)
      SetMotionVectorPredictor (pmvbw, enc_picture->ref_idx, enc_picture->mv, bw_rFrame, LIST_1, 0, 0, 16, 16);

    for (block_y=0; block_y<4; block_y++)
    {
      pic_block_y  = (img->pix_y>>2) + block_y;
      opic_block_y = (img->opix_y>>2) + block_y;
      
      for (block_x=0; block_x<4; block_x++)
      {
        pic_block_x  = (img->pix_x>>2) + block_x;
        opic_block_x = (img->opix_x>>2) + block_x;

        if (fw_rFrame >=0)
        {
          if (!fw_rFrame  && !moving_block[opic_block_x][opic_block_y])
          {
            all_mvs [block_x][block_y][LIST_0][0][0][0] = 0;
            all_mvs [block_x][block_y][LIST_0][0][0][1] = 0;            
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y]=0;       
          }
          else
          {
            all_mvs [block_x][block_y][LIST_0][fw_rFrame][0][0] = pmvfw[0];
            all_mvs [block_x][block_y][LIST_0][fw_rFrame][0][1] = pmvfw[1];
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y]=fw_rFrame;              
          }
        }
        else
        {
          all_mvs [block_x][block_y][LIST_0][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_0][0][0][1] = 0;
          direct_ref_idx[LIST_0][pic_block_x][pic_block_y]=-1;          
        }

        if (bw_rFrame >=0)
        {
          if(bw_rFrame==0 && !moving_block[opic_block_x][opic_block_y])
          {                  
            all_mvs [block_x][block_y][LIST_1][0][0][0] = 0;
            all_mvs [block_x][block_y][LIST_1][0][0][1] = 0;
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y]=bw_rFrame;     
          }
          else
          {
            all_mvs [block_x][block_y][LIST_1][bw_rFrame][0][0] = pmvbw[0];
            all_mvs [block_x][block_y][LIST_1][bw_rFrame][0][1] = pmvbw[1];
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y]=bw_rFrame;
          }               
        }
        else
        {      
          direct_ref_idx[LIST_1][pic_block_x][pic_block_y]=-1;

          all_mvs [block_x][block_y][LIST_1][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_1][0][0][1] = 0;
        }
        
        if (fw_rFrame < 0 && bw_rFrame < 0)
        {
          direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = 
          direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = 0;
        }

        if      (direct_ref_idx[LIST_1][pic_block_x][pic_block_y]==-1) direct_pdir[pic_block_x][pic_block_y] = 0;
        else if (direct_ref_idx[LIST_0][pic_block_x][pic_block_y]==-1) direct_pdir[pic_block_x][pic_block_y] = 1;
        else                                                           direct_pdir[pic_block_x][pic_block_y] = 2;
      }
    }
  }
  else
  {
    //temporal direct mode copy from decoder
    for (block_y=0; block_y<4; block_y++)
    {
      pic_block_y  = (img->pix_y>>2) + block_y;
      opic_block_y = (img->opix_y>>2) + block_y;
      
      for (block_x=0; block_x<4; block_x++)
      {
        int refList; 
        int ref_idx; 

        int list_offset = ((img->MbaffFrameFlag)&&(currMB->mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

        pic_block_x  = (img->pix_x>>2) + block_x;
        opic_block_x = (img->opix_x>>2) + block_x;
        
        refList = (co_located_ref_idx[LIST_0][opic_block_x][opic_block_y]== -1 ? LIST_1 : LIST_0);
        ref_idx = co_located_ref_idx[refList][opic_block_x][opic_block_y];
              
        // next P is intra mode
        if (ref_idx==-1)
        {
          all_mvs [block_x][block_y][LIST_0][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_0][0][0][1] = 0;
          all_mvs [block_x][block_y][LIST_1][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_1][0][0][1] = 0;
          direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = 0;
          direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = 0;
          direct_pdir[pic_block_x][pic_block_y] = 2;
        }
        // next P is skip or inter mode
        else 
        {
          int mapped_idx=INVALIDINDEX;
          int iref; 

          {
            for (iref=0;iref<min(img->num_ref_idx_l0_active,listXsize[LIST_0+list_offset]);iref++)
            {
              if (enc_picture->ref_pic_num[LIST_0 +list_offset][iref]==co_located_ref_id[refList ][opic_block_x][opic_block_y])
              {
                mapped_idx=iref;
                break;
              }
              else //! invalid index. Default to zero even though this case should not happen
              {                        
                mapped_idx=INVALIDINDEX;
              }
            }
          }

          if (mapped_idx !=INVALIDINDEX)
          {
            mv_scale = img->mvscale[LIST_0+list_offset][mapped_idx];

            if (mv_scale==9999)
            {
              // forward
              all_mvs [block_x][block_y][LIST_0][0][0][0] = co_located_mv[refList][opic_block_x][opic_block_y][0];
              all_mvs [block_x][block_y][LIST_0][0][0][1] = co_located_mv[refList][opic_block_x][opic_block_y][1];
              // backward
              all_mvs [block_x][block_y][LIST_1][       0][0][0] = 0;
              all_mvs [block_x][block_y][LIST_1][       0][0][1] = 0;
            }else
            {
              // forward
              all_mvs [block_x][block_y][LIST_0][mapped_idx][0][0] = (mv_scale * co_located_mv[refList][opic_block_x][opic_block_y][0] + 128) >> 8;
              all_mvs [block_x][block_y][LIST_0][mapped_idx][0][1] = (mv_scale * co_located_mv[refList][opic_block_x][opic_block_y][1] + 128) >> 8;
              // backward
              all_mvs [block_x][block_y][LIST_1][       0][0][0] = ((mv_scale - 256)* co_located_mv[refList][opic_block_x][opic_block_y][0] + 128) >> 8;
              all_mvs [block_x][block_y][LIST_1][       0][0][1] = ((mv_scale - 256)* co_located_mv[refList][opic_block_x][opic_block_y][1] + 128) >> 8;
            }
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = mapped_idx;
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = 0;
            direct_pdir[pic_block_x][pic_block_y] = 2;
          }
          else
          {
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = -1;
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = -1;
            direct_pdir[pic_block_x][pic_block_y] = -1;
          }
        }
      }
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    control the sign of a with b
 *		当b>0，a>0.否则 a<0
 ************************************************************************
 */
int sign(int a,int b)
{
  int x;
  x=absm(a);
  if (b >= 0)
    return x;
  else
    return -x;
}

