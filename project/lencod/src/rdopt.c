
/*!
 ***************************************************************************
 * \file rdopt.c
 *
 * \brief
 *    Rate-Distortion optimized mode decision
 *
 * \author
 *    Heiko Schwarz <hschwarz@hhi.de>
 *
 * \date
 *    12. April 2001
 **************************************************************************
 */

#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include <assert.h>
#include "rdopt_coding_state.h"
#include "elements.h"
#include "refbuf.h"
#include "intrarefresh.h"
#include "vlc.h"
#include "mbuffer.h"
#include "image.h"
#include "mb_access.h"
#include "fast_me.h"
#include "ratectl.h"            // head file for rate control
#include "cabac.h"            // head file for rate control

//Rate control

int QP,QP2;
int DELTA_QP,DELTA_QP2;
int diffy[16][16];
static int pred[16][16];

extern       int  QP2QUANT  [40];

//mpr8x8 cofAC8x8 recmbY8x8
//==== MODULE PARAMETERS ====
int   best_mode;
int   rec_mbY[16][16], rec_mbU[8][8], rec_mbV[8][8], rec_mbY8x8[16][16];    // reconstruction values
int   mpr8x8[16][16];
int   ****cofAC=NULL, ****cofAC8x8=NULL;        // [8x8block][4x4block][level/run][scan_pos]
int   ***cofDC=NULL;                       // [yuv][level/run][scan_pos]
int   **cofAC4x4=NULL, ****cofAC4x4intern=NULL; // [level/run][scan_pos]
int   cbp, cbp8x8, cnt_nonz_8x8;
int   cbp_blk, cbp_blk8x8;
int   frefframe[4][4], brefframe[4][4], b8mode[4], b8pdir[4];
int   best8x8mode [4];                // [block]
int   best8x8pdir [MAXMODE][4];       // [mode][block]
int   best8x8fwref  [MAXMODE][4];       // [mode][block]
int   b8_ipredmode[16], b8_intra_pred_modes[16];
CSptr cs_mb=NULL, cs_b8=NULL, cs_cm=NULL, cs_imb=NULL, cs_ib8=NULL, cs_ib4=NULL, cs_pc=NULL;
int   best_c_imode;
int   best_i16offset;

int   best8x8bwref     [MAXMODE][4];       // [mode][block]
int   abp_typeframe[4][4];

/*!
 ************************************************************************
 * \brief
 *    delete structure for RD-optimized mode decision
 ************************************************************************
 */
void clear_rdopt ()
{
  free_mem_DCcoeff (cofDC);
  free_mem_ACcoeff (cofAC);
  free_mem_ACcoeff (cofAC8x8);
  free_mem_ACcoeff (cofAC4x4intern);

  // structure for saving the coding state
  delete_coding_state (cs_mb);
  delete_coding_state (cs_b8);
  delete_coding_state (cs_cm);
  delete_coding_state (cs_imb);
  delete_coding_state (cs_ib8);
  delete_coding_state (cs_ib4);
  delete_coding_state (cs_pc);
}


/*!
 ************************************************************************
 * \brief
 *    create structure for RD-optimized mode decision
 ************************************************************************
 */
void init_rdopt ()
{
  get_mem_DCcoeff (&cofDC);				//初始化DC系数缓冲区
  get_mem_ACcoeff (&cofAC);				//初始化AC系数缓冲区
  get_mem_ACcoeff (&cofAC8x8);
  get_mem_ACcoeff (&cofAC4x4intern);
  cofAC4x4 = cofAC4x4intern[0][0];

  // structure for saving the coding state
  cs_mb  = create_coding_state ();
  cs_b8  = create_coding_state ();
  cs_cm  = create_coding_state ();
  cs_imb = create_coding_state ();
  cs_ib8 = create_coding_state ();
  cs_ib4 = create_coding_state ();
  cs_pc  = create_coding_state ();
}



/*! 
 *************************************************************************************
 * \brief
 *    Updates the pixel map that shows, which reference frames are reliable for
 *    each MB-area of the picture.
 *
 * \note
 *    The new values of the pixel_map are taken from the temporary buffer refresh_map
 *
 *************************************************************************************
 */
void UpdatePixelMap()
{
  int mx,my,y,x,i,j;
  if (img->type==I_SLICE)
  {
    for (y=0; y<img->height; y++)
    for (x=0; x<img->width; x++)
    {
      pixel_map[y][x]=1;
    }
  }
  else
  {
    for (my=0; my<img->height/8; my++)
    for (mx=0; mx<img->width/8;  mx++)
    {
      j = my*8 + 8;
      i = mx*8 + 8;
      if (refresh_map[my][mx])
      {
        for (y=my*8; y<j; y++)
        for (x=mx*8; x<i; x++)  pixel_map[y][x] = 1;
      }
      else
      {
        for (y=my*8; y<j; y++)
        for (x=mx*8; x<i; x++)  pixel_map[y][x] = min(pixel_map[y][x]+1, input->num_reference_frames+1);
      }
    }
  }
}

/*! 
 *************************************************************************************
 * \brief
 *    Checks if a given reference frame is reliable for the current 
 *    macroblock, given the motion vectors that the motion search has 
 *    returned.
 *	  检查制定的参考帧是否使用在当前宏块中。
 * \return
 *    If the return value is 1, the reference frame is reliable. If it 
 *    is 0, then it is not reliable.
 *	  返回1，参考帧使用，否则没使用
 * \note
 *    A specific area in each reference frame is assumed to be unreliable
 *    if the same area has been intra-refreshed in a subsequent frame.
 *    The information about intra-refreshed areas is kept in the pixel_map.
 *
 *************************************************************************************
 */
int CheckReliabilityOfRef (int block, int list_idx, int ref, int mode)
{
  int y,x, block_y, block_x, dy, dx, y_pos, x_pos, yy, xx, pres_x, pres_y;
  int maxold_x  = img->width-1;
  int maxold_y  = img->height-1;
  int ref_frame = ref+1;

  int by0 = (mode>=4?2*(block/2):mode==2?2*block:0);
  int by1 = by0 + (mode>=4||mode==2?2:4);
  int bx0 = (mode>=4?2*(block%2):mode==3?2*block:0);
  int bx1 = bx0 + (mode>=4||mode==3?2:4);

  for (block_y=by0; block_y<by1; block_y++)
    for (block_x=bx0; block_x<bx1; block_x++)
    {
      y_pos  = img->all_mv[block_x][block_y][list_idx][ref][mode][1];
      y_pos += (img->block_y+block_y) * BLOCK_SIZE * 4;
      x_pos  = img->all_mv[block_x][block_y][list_idx][ref][mode][0];
      x_pos += (img->block_x+block_x) * BLOCK_SIZE * 4;
      
      /* Here we specify which pixels of the reference frame influence
         the reference values and check their reliability. This is
         based on the function Get_Reference_Pixel */
      
      dy = y_pos & 3;
      dx = x_pos & 3;

      y_pos = (y_pos-dy)/4;
      x_pos = (x_pos-dx)/4;

      if (dy==0 && dx==0) //full-pel
      {
        for (y=0 ; y < BLOCK_SIZE ; y++)
          for (x=0 ; x < BLOCK_SIZE ; x++)
            if (pixel_map[max(0,min(maxold_y,y_pos+y))][max(0,min(maxold_x,x_pos+x))] < ref_frame)
              return 0;
      }
      else  /* other positions */
      {
        if (dy == 0) 
        {
          for (y=0 ; y < BLOCK_SIZE ; y++)
            for (x=0 ; x < BLOCK_SIZE ; x++)
            {
              pres_y = max(0,min(maxold_y,y_pos+y));
              for(xx=-2;xx<4;xx++) {
                pres_x = max(0,min(maxold_x,x_pos+x+xx));
                if (pixel_map[pres_y][pres_x] < ref_frame)
                  return 0;
              }
            }
        }

        else if (dx == 0) 
        {
          for (y=0 ; y < BLOCK_SIZE ; y++)
            for (x=0 ; x < BLOCK_SIZE ; x++)
            {
              pres_x = max(0,min(maxold_x,x_pos+x));
              for(yy=-2;yy<4;yy++) {
                pres_y = max(0,min(maxold_y,y_pos+yy+y));
                if (pixel_map[pres_y][pres_x] < ref_frame)
                  return 0;
              }
            }
        }
        else if (dx == 2) 
        {
          for (y=0 ; y < BLOCK_SIZE ; y++)
            for (x=0 ; x < BLOCK_SIZE ; x++)
            {
              for(yy=-2;yy<4;yy++) {
                pres_y = max(0,min(maxold_y,y_pos+yy+y));
                for(xx=-2;xx<4;xx++) {
                  pres_x = max(0,min(maxold_x,x_pos+xx+x));
                  if (pixel_map[pres_y][pres_x] < ref_frame)
                    return 0;
                }
              }
            }
        }
        else if (dy == 2) 
        {
          for (y=0 ; y < BLOCK_SIZE ; y++)
            for (x=0 ; x < BLOCK_SIZE ; x++)
            {
              for(xx=-2;xx<4;xx++) {
                pres_x = max(0,min(maxold_x,x_pos+xx+x));
                for(yy=-2;yy<4;yy++) {
                  pres_y = max(0,min(maxold_y,y_pos+yy+y));
                  if (pixel_map[pres_y][pres_x] < ref_frame)
                    return 0;
                }
              }
            }
        }
        else 
        {
          for (y=0 ; y < BLOCK_SIZE ; y++)
            for (x=0 ; x < BLOCK_SIZE ; x++)
            {
              pres_y = dy == 1 ? y_pos+y : y_pos+y+1;
              pres_y = max(0,min(maxold_y,pres_y));

              for(xx=-2;xx<4;xx++) {
                pres_x = max(0,min(maxold_x,x_pos+xx+x));
                if (pixel_map[pres_y][pres_x] < ref_frame)
                  return 0;
              }

              pres_x = dx == 1 ? x_pos+x : x_pos+x+1;
              pres_x = max(0,min(maxold_x,pres_x));

              for(yy=-2;yy<4;yy++) {
                pres_y = max(0,min(maxold_y,y_pos+yy+y));
                if (pixel_map[pres_y][pres_x] < ref_frame)
                  return 0;
              }
            }
        }

      }
    }

  return 1;
}



/*! 
 *************************************************************************************
 * \brief
 *    R-D Cost for an 4x4 Intra block
 *	  计算4x4帧内预测块的率失真代价
 *************************************************************************************
 */
double RDCost_for_4x4IntraBlocks (int*    nonzero,				//非0标识
                           int     b8,							//指定8x8块
                           int     b4,							//指定4x4子块
                           int    ipmode,						//指定帧内预测模式
                           double  lambda,						//率失真计算参数
                           double  min_rdcost,					//
                           int mostProbableMode)
{
  double  rdcost;
  int     dummy, x, y, rate;
  int     distortion  = 0;
  //4x4子块在宏块中的位置, 像素单位
  int     block_x     = 8*(b8%2)+4*(b4%2);						
  int     block_y     = 8*(b8/2)+4*(b4/2);
  //4x4子块在图像中的位置，像素单位
  int     pic_pix_x   = img->pix_x+block_x;
  int     pic_pix_y   = img->pix_y+block_y;

  int     pic_opix_y  = img->opix_y+block_y;
  byte    **imgY      = enc_picture->imgY;										//编解码图像的像素，该处用于保存解码后的图像像素

  Slice          *currSlice    =  img->currentSlice;							//当前片
  Macroblock     *currMB       = &img->mb_data[img->current_mb_nr];				//当前宏块数据
  SyntaxElement  *currSE       = &img->MB_SyntaxElements[currMB->currSEnr];		//
  const int      *partMap      = assignSE2partition[input->partition_mode];		//
  DataPartition  *dataPart;

  //===== perform DCT, Q, IQ, IDCT, Reconstruction =====
  //对预测误差执行dct，量化，反量化，反dct
  //重建值保存在enc_picture->imgY中
  //并将变换系数，保存于img->cofAC[b8][b4][level/run]中
  dummy = 0;
  *nonzero = dct_luma (block_x, block_y, &dummy, 1);

  //===== get distortion (SSD) of 4x4 block =====
  for (y=0; y<4; y++)
  for (x=pic_pix_x; x<pic_pix_x+4; x++)  
    distortion += img->quad [imgY_org[pic_opix_y+y][x] - imgY[pic_pix_y+y][x]];	//误差=累计[ (实际值-解码值)平方 ], 说白了就是计算MSE，用于PSNR

  //===== RATE for INTRA PREDICTION MODE  (SYMBOL MODE MUST BE SET TO UVLC) =====
  //最佳预测模式的预测 == 最佳预测模式时，返回-1，否则。。。。
  currSE->value1 = (mostProbableMode == ipmode) ? -1 : ipmode < mostProbableMode ? ipmode : ipmode-1;

  //--- set position and type ---
  currSE->context = 4*b8 + b4;
  currSE->type    = SE_INTRAPREDMODE;

  //--- set function pointer ----
  if (input->symbol_mode != UVLC)    
  currSE->writing = writeIntraPredMode_CABAC;

  //--- choose data partition ---
  dataPart = &(currSlice->partArr[partMap[SE_INTRAPREDMODE]]);
  //--- encode and update rate ---
  if (input->symbol_mode == UVLC)    writeSyntaxElement_Intra4x4PredictionMode(currSE, dataPart);
  else                               dataPart->writeSyntaxElement (currSE, dataPart);

  rate = currSE->len;
  currSE++;
  currMB->currSEnr++;

  //===== RATE for LUMINANCE COEFFICIENTS =====
  if (input->symbol_mode == UVLC)
  {
    rate  += writeCoeff4x4_CAVLC (LUMA, b8, b4, 0);
  }
  else
  {
    rate  += writeLumaCoeff4x4_CABAC (b8, b4, 1);
  }
  rdcost = (double)distortion + lambda*(double)rate;

  return rdcost;
}

/*! 
 *************************************************************************************
 * \brief
 *    Mode Decision for an 4x4 Intra block
 *    帧内4x4块预测, 并寻到帧内4x4预测最佳模式，以及对应的重建值，对应的预测值，对应的变换系数
 *************************************************************************************
 */
// b8代指了宏块中的第b8个块
// b4代指了b8块中的b4子块
// lambda是率失真计算的参数
// cost4x4是4x4帧内预测的率失真
int Mode_Decision_for_4x4IntraBlocks (int  b8,  int  b4,  double  lambda,  int*  min_cost)
{
  int     ipmode, best_ipmode = 0, i, j, k, x, y, cost, dummy;
  int     c_nz, nonzero = 0, rec4x4[4][4], diff[16];
  double  rdcost;
  int     block_x     = 8*(b8%2)+4*(b4%2);						//该4x4子块在宏块中的位置，像素为单位
  int     block_y     = 8*(b8/2)+4*(b4/2);						//该4x4子块在宏块中的位置，像素为单位
  int     pic_pix_x   = img->pix_x+block_x;						//该4x4子块在图像中的位置, 像素为单位
  int     pic_pix_y   = img->pix_y+block_y;						//该4x4子块在图像中的位置, 像素为单位
  int     pic_opix_x   = img->opix_x+block_x;					//该4x4子块在图像中的位置, 像素为单位
  int     pic_opix_y   = img->opix_y+block_y;					//该4x4子块在图像中的位置, 像素为单位
  int     pic_block_x = pic_pix_x/4;							//该4x4子块在图像中的位置, 4x4块为单位
  int     pic_block_y = pic_pix_y/4;							//该4x4子块在图像中的位置, 4x4块为单位
  double  min_rdcost  = 1e30;

  int left_available, up_available, all_available;

  int     upMode;
  int     leftMode;
  int     mostProbableMode;
  
  PixelPos left_block;
  PixelPos top_block;

  //输入的block_x/y 需要转化为4x4单位的位置，而不能用像素单位位置
  //这里得到的虽然是邻近像素的信息，但是邻近像素的可用性，已经代表了所在块的可用性了。
  //在实际进行预测时，会提取所有邻近的像素信息。
  getLuma4x4Neighbour(img->current_mb_nr, block_x/4, block_y/4, -1,  0, &left_block);	//获得左方邻像素的信息
  getLuma4x4Neighbour(img->current_mb_nr, block_x/4, block_y/4,  0, -1, &top_block);	//获得上方邻像素的信息


  // constrained intra pred. 是否现在帧内预测
  if (input->UseConstrainedIntraPred)
  {
    left_block.available = left_block.available ? img->intra_block[left_block.mb_addr] : 0;
    top_block.available  = top_block.available  ? img->intra_block[top_block.mb_addr]  : 0;
  }

  //不可用的相邻块，将该相邻块的预测模式值设为-1
  upMode            = top_block.available ? img->ipredmode[top_block.pos_x ][top_block.pos_y ] : -1;
  leftMode          = left_block.available ? img->ipredmode[left_block.pos_x][left_block.pos_y] : -1;
  
  /*********************************************************************
		  A
		B C
		如上，当前块为C，左上角相邻块为A, B，利用A B的预测模式，预测C的预测模式
		mostProbableMode = DC_PRED,				若A B中某个块不可用
						 = min(Amode, Bmode),	其他
  **********************************************************************/
  mostProbableMode  = (upMode < 0 || leftMode < 0) ? DC_PRED : upMode < leftMode ? upMode : leftMode;

  *min_cost = (1<<20);

  //===== INTRA PREDICTION FOR 4x4 BLOCK =====
  //利用函数intrapred_luma计算出了9中预测模式下的预测值保存在img->mprr中
  intrapred_luma (pic_pix_x, pic_pix_y, &left_available, &up_available, &all_available);

  //===== LOOP OVER ALL 4x4 INTRA PREDICTION MODES =====
  // 遍历9中4x4帧内预测模式, 计算出4x4帧内模式中的最佳模式
  for (ipmode=0; ipmode<NO_INTRA_PMODE; ipmode++)
  {
    int available_mode =  (ipmode==DC_PRED) ||
        ((ipmode==VERT_PRED||ipmode==VERT_LEFT_PRED||ipmode==DIAG_DOWN_LEFT_PRED) && up_available ) ||
        ((ipmode==HOR_PRED||ipmode==HOR_UP_PRED) && left_available ) ||(all_available);
    
	//non rdo:	MODE_Cost = sad + 4 * lambda_mde
	//rdo		MODE_Cost = ssd + lambda_mode * BLOCKbits
	//sad:是实际值与“预测值”的误差.这里也有可能使用的是satd
	//ssd:是实际值与“重建值”的误差.这里也有可能使用的是sstd
	if( available_mode)
    {
	  //==============================================================非RDO模式====================================================================
	  if (!input->rdopt)
      {
		//遍历4x4块中的所有元素
        for (k=j=0; j<4; j++)
          for (i=0; i<4; i++, k++)
          {
			//imgY_org是整副图像的像素缓存
			//img->mprr是当前4x4块，指定模式的图像像素缓存
            diff[k] = imgY_org[pic_opix_y+j][pic_opix_x+i] - img->mprr[ipmode][j][i];		//计算指定帧内预测模式ipomde的残差
          }
        cost  = (ipmode == mostProbableMode) ? 0 : (int)floor(4 * lambda );
        cost += SATD (diff, input->hadamard);												//累计误差,根据配置(input->hadamard)，选择使用SAD或者SATD
		//记录4x4帧内预测的最佳模式，并记录对应的代价
        if (cost < *min_cost)
        {
          best_ipmode = ipmode;
          *min_cost   = cost;
        }
      }
	  //==============================================================RDO模式=====================================================================
      else
      {
        // get prediction and prediction error
        for (j=0; j<4; j++)
        for (i=0; i<4; i++)
        {
		  //mpr指定模式的像素预测值(img->mpr是整幅图像的预测值)
		  //m7是预测误差
          img->mpr[block_x+i][block_y+j]  = img->mprr[ipmode][j][i];
          img->m7[i][j]                   = imgY_org[pic_opix_y+j][pic_opix_x+i] - img->mprr[ipmode][j][i];
        }

        //===== store the coding state =====
        store_coding_state (cs_cm);
        // get and check rate-distortion cost
		// 获得指定块，指定分块，指定模式的率失真代价，并检查其是否需要记录
		// 该函数对预测误差进行dct变换，量化，反dct变换，累计并量化重建图像的误差，以计算率失真
		// 重建图像的像素已经保存于enc_pictuire->imgY中
		// 误差变换的系数，已经存在cofAC中
        if ((rdcost = RDCost_for_4x4IntraBlocks (&c_nz, b8, b4, ipmode, lambda, min_rdcost, mostProbableMode)) < min_rdcost)
        {
          //--- set coefficients ---
		  // 当前模式下，率失真比之前的模式还小，则更新预测误差的变换参数
          for (j=0; j<2; j++)
          for (i=0; i<18;i++)  cofAC4x4[j][i]=img->cofAC[b8][b4][j][i];

          //--- set reconstruction ---
		  // 当前模式下，率失真比之前的模式还小，则更新重建图像的像素
          for (y=0; y<4; y++)
          for (x=0; x<4; x++)  rec4x4[y][x] = enc_picture->imgY[pic_pix_y+y][pic_pix_x+x];

          //--- flag if dct-coefficients must be coded ---
          // dct系数编码标识
		  nonzero = c_nz;

          //--- set best mode update minimum cost ---设置最佳预测模式，以及对应的率失真
          min_rdcost  = rdcost;
          best_ipmode = ipmode;
        }
        reset_coding_state (cs_cm);
      }
    }
  }

  //===== set intra mode prediction =====
  img->ipredmode[pic_block_x][pic_block_y] = best_ipmode;		//指定4x4块的最佳预测模式
  //这里保存着使用传输使用的值,在解码器端根据该值和预测模式预测值可以确定具体最佳预测模式******************************************************
		  /**************************************************
		   *	intra_pred_modes[i]的4x4块顺序如下：	*
		   *											*
		   *			00 01 | 04 05					*
		   *			02 03 | 06 07					*
		   *			-------------					*
		   *			08 09 | 12 13					*
		   *			10 11 | 14 15					*
		   **************************************************/
  img->mb_data[img->current_mb_nr].intra_pred_modes[4*b8+b4] = (mostProbableMode == best_ipmode) ? -1 : best_ipmode < mostProbableMode ? best_ipmode : best_ipmode-1;
  //********************************************************************************************************************************************

  if (!input->rdopt)
  {
    // get prediction and prediction error
    for (j=0; j<4; j++)
      for (i=0; i<4; i++)
      {
        img->mpr[block_x+i][block_y+j]  = img->mprr[best_ipmode][j][i];
        img->m7[i][j]                   = imgY_org[pic_opix_y+j][pic_opix_x+i] - img->mprr[best_ipmode][j][i];
      }
    nonzero = dct_luma (block_x, block_y, &dummy, 1);
  }
  else
  {
    //===== restore coefficients =====
	// 保存最佳预测模式下的，预测误差dct系数
    for (j=0; j<2; j++)
    for (i=0; i<18;i++)  img->cofAC[b8][b4][j][i]=cofAC4x4[j][i];
  
    //===== restore reconstruction and prediction (needed if single coeffs are removed) =====
	// 保存最佳预测模式下的，重建图像像素 和 预测图像像素
    for (y=0; y<4; y++)
    for (x=0; x<4; x++)
    {
      enc_picture->imgY[pic_pix_y+y][pic_pix_x+x] = rec4x4[y][x];			//最佳重建图像像素
      img->mpr[block_x+x][block_y+y] = img->mprr[best_ipmode][y][x];		//最佳预测图像像素
    }
  }

  return nonzero;
}


/*! 
 *************************************************************************************
 * \brief
 *    Mode Decision for an 8x8 Intra block
 *************************************************************************************
 */
int Mode_Decision_for_8x8IntraBlocks(int b8,double lambda,int *cost)
{
  int  nonzero=0, b4;
  int  cost4x4;
  
  *cost = (int)floor(6.0 * lambda + 0.4999);

  //将8x8的块，分成4个4x4的子块，进行帧内预测
  for (b4=0; b4<4; b4++)
  {
	// b8代指了宏块中的第b8个块
	// b4代指了b8块中的b4子块
	// lambda是率失真计算的参数
	// cost4x4是4x4帧内预测的率失真
    if (Mode_Decision_for_4x4IntraBlocks (b8, b4, lambda, &cost4x4))		//找到4x4帧内模式的最佳预测模式，以及相应的预测、重建、误差值，返回非零标志
    {
      nonzero        = 1;
    }
    *cost += cost4x4;														//当前8x8块中，所有4x4块的代价累计
  }

  return nonzero;
}

/*! 
 *************************************************************************************
 * \brief
 *    4x4 Intra mode decision for an macroblock
 *	  1个宏块的4x4帧内模式
 *************************************************************************************
 */
int Mode_Decision_for_Intra4x4Macroblock (double lambda,  int* cost)

{
  int  cbp=0, b8, cost8x8;

  //将16x16的宏块，分为4个8x8的块，分别进行4x4的帧内预测
  for (*cost=0, b8=0; b8<4; b8++)
  {
    if (Mode_Decision_for_8x8IntraBlocks (b8, lambda, &cost8x8))		//
    {
      cbp |= (1<<b8);
    }
    *cost += cost8x8;													//当前宏块所有8x8代价的累计
  }

  return cbp;
}


/*! 
 *************************************************************************************
 * \brief
 *    R-D Cost for an 8x8 Partition
 *	  8x8亚宏块分割，计算亚宏块的rdcost
 *************************************************************************************
 */
double RDCost_for_8x8blocks (int*    cnt_nonz,   // --> number of nonzero coefficients
                             int*    cbp_blk,    // --> cbp blk
                             double  lambda,     // <-- lagrange multiplier
                             int     block,      // <-- 8x8 block number
                             int     mode,       // <-- partitioning mode
                             int     pdir,       // <-- prediction direction
                             int     ref,        // <-- reference frame
                             int     bwd_ref)    // <-- abp type
{
  int  i, j, k;
  int  rate=0, distortion=0;
  int  dummy, mrate;
  int  fw_mode, bw_mode;
  int  cbp     = 0;
  int  pax     = 8*(block%2);
  int  pay     = 8*(block/2);
  int  i0      = pax/4;
  int  j0      = pay/4;
  int  bframe  = (img->type==B_SLICE);
  int  direct  = (bframe && mode==0);
  int  b8value = B8Mode2Value (mode, pdir);

  Macroblock    *currMB    = &img->mb_data[img->current_mb_nr];
  SyntaxElement *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
  Slice         *currSlice = img->currentSlice;
  DataPartition *dataPart;
  const int     *partMap   = assignSE2partition[input->partition_mode];

  EncodingEnvironmentPtr eep_dp;

  //======================================================================================================
  //==============  GET COEFFICIENTS, RECONSTRUCTIONS, CBP 获得预测误差系数、重建图像、cbp================
  //======================================================================================================
  if (direct)
  {
    if (direct_pdir[img->block_x+i0][img->block_y+j0]<0) // mode not allowed
    {
      return (1e20);
    }
    else
    {
	  //计算预测误差，保存在img->m7；重建图像， 保存在enc_picture->imgY
      *cnt_nonz = LumaResidualCoding8x8 (&cbp, cbp_blk, block, direct_pdir[img->block_x+i0][img->block_y+j0], 0, 0, max(0,direct_ref_idx[LIST_0][img->block_x+i0][img->block_y+j0]), direct_ref_idx[LIST_1][img->block_x+i0][img->block_y+j0]);
    }
  }
  else
  {
    fw_mode   = (pdir==0||pdir==2 ? mode : 0);
    bw_mode   = (pdir==1||pdir==2 ? mode : 0);
    *cnt_nonz = LumaResidualCoding8x8 (&cbp, cbp_blk, block, pdir, fw_mode, bw_mode, ref, bwd_ref);
  }

  //===== get residue =====
  if (input->rdopt==2 && img->type!=B_SLICE)
  {
    // We need the reconstructed prediction residue for the simulated decoders.
    compute_residue_b8block (block, -1);
  }

  //=====
  //=====   GET DISTORTION
  //=====
  if (input->rdopt==2 && img->type!=B_SLICE)
  {
    for (k=0; k<input->NoOfDecoders ;k++)
    {
      decode_one_b8block (k, P8x8, block, mode, ref);
      for (j=img->opix_y+pay; j<img->opix_y+pay+8; j++)
      for (i=img->opix_x+pax; i<img->opix_x+pax+8; i++)
      {
        distortion += img->quad[imgY_org[j][i] - decs->decY[k][j][i]];
      }
    }
    distortion /= input->NoOfDecoders;
  }
  else
  {
    for (j=pay; j<pay+8; j++)
    for (i=img->pix_x+pax; i<img->pix_x+pax+8; i++)
    {
      distortion += img->quad [imgY_org[img->opix_y+j][i] - enc_picture->imgY[img->pix_y+j][i]];		//distortion = ssd
    }
  }

  //=====
  //=====   GET RATE
  //=====
  //----- block 8x8 mode -----
  if (input->symbol_mode == UVLC)
  {
    ue_linfo (b8value, dummy, &mrate, &dummy);
    rate += mrate;
  }
  else
  {
    currSE->value1  = b8value;
    currSE->writing = writeB8_typeInfo_CABAC;
    currSE->type    = SE_MBTYPE;
    dataPart = &(currSlice->partArr[partMap[currSE->type]]);
    dataPart->writeSyntaxElement (currSE, dataPart);
    rate += currSE->len;
    currSE++;
    currMB->currSEnr++;
  }

  //----- motion information -----
  if (!direct)
  {
    if ((img->num_ref_idx_l0_active > 1 ) && (pdir==0 || pdir==2))
      rate  += writeReferenceFrame (mode, i0, j0, 1, ref);
    if(img->num_ref_idx_l1_active > 1 && img->type== B_SLICE)
    {
      if (pdir==1 || pdir==2)
      {
        rate  += writeReferenceFrame (mode, i0, j0, 0, bwd_ref);
      }
    }

    if (pdir==0 || pdir==2)
    {
      rate  += writeMotionVector8x8 (i0, j0, i0+2, j0+2, ref, LIST_0, mode);
    }
    if (pdir==1 || pdir==2)
    {
      rate  += writeMotionVector8x8 (i0, j0, i0+2, j0+2, bwd_ref, LIST_1, mode);
    }
  }

  //----- coded block pattern (for CABAC only) -----
  if (input->symbol_mode == CABAC)
  {
    dataPart = &(currSlice->partArr[partMap[SE_CBP_INTER]]);
    eep_dp   = &(dataPart->ee_cabac);
    mrate    = arienco_bits_written (eep_dp);
    writeCBP_BIT_CABAC (block, ((*cnt_nonz>0)?1:0), cbp8x8, currMB, 1, eep_dp);
    mrate    = arienco_bits_written (eep_dp) - mrate;
    rate    += mrate;
  }

  //----- luminance coefficients -----
  if (*cnt_nonz)
  {
    rate += writeLumaCoeff8x8 (block, 0);
  }

  return (double)distortion + lambda * (double)rate;
}


/*! 
 *************************************************************************************
 * \brief
 *    Gets mode offset for intra16x16 mode
 *************************************************************************************
 */
int I16Offset (int cbp, int i16mode)
{
  return (cbp&15?13:1) + i16mode + ((cbp&0x30)>>2);
}


/*! 
 *************************************************************************************
 * \brief
 *    Sets modes and reference frames for an macroblock
 *	  为宏块设置模式和参考帧
 *************************************************************************************
 */
void SetModesAndRefframeForBlocks (int mode)
{
  int i,j,k,l;
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];												//当前宏块
  int  bframe  = (img->type==B_SLICE);																	//b帧标志

  int list_offset   = ((img->MbaffFrameFlag)&&(currMB->mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  //--- macroblock type ---
  currMB->mb_type = mode;																				//宏块预测类型
  
  //--- block 8x8 mode and prediction direction ---
  switch (mode)
  {
  case 0:
    for(i=0;i<4;i++)
    {
      currMB->b8mode[i] = 0;
      currMB->b8pdir[i] = (bframe?direct_pdir[img->block_x+(i%2)*2][img->block_y+(i/2)*2]:0);
    }
    break;
  case 1:
  case 2:
  case 3:
    for(i=0;i<4;i++)
    {
      currMB->b8mode[i] = mode;
      currMB->b8pdir[i] = best8x8pdir[mode][i];
    }
    break;
  case P8x8:
    for(i=0;i<4;i++)
    {
      currMB->b8mode[i]   = best8x8mode[i];
      currMB->b8pdir[i]   = best8x8pdir[mode][i];
    }
    break;
  case I4MB:
    for(i=0;i<4;i++)
    {
      currMB->b8mode[i] = IBLOCK;
      currMB->b8pdir[i] = -1;
    }
    break;
  case I16MB:
    for(i=0;i<4;i++)
    {
      currMB->b8mode[i] =  0;
      currMB->b8pdir[i] = -1;
    }
    break;
  default:
    printf ("Unsupported mode in SetModesAndRefframeForBlocks!\n");
    exit (1);
  }
  
#define IS_FW ((best8x8pdir[mode][k]==0 || best8x8pdir[mode][k]==2) && (mode!=P8x8 || best8x8mode[k]!=0 || !bframe))
#define IS_BW ((best8x8pdir[mode][k]==1 || best8x8pdir[mode][k]==2) && (mode!=P8x8 || best8x8mode[k]!=0))
  //--- reference frame arrays ---
  // 参考帧矩阵
  if (mode==0 || mode==I4MB || mode==I16MB)
  {
    if (bframe)
    { 
      for (j=0;j<4;j++)
        for (i=0;i<4;i++)
        {
          if(!mode)
          {     //direct mode
            enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = direct_ref_idx[LIST_0][img->block_x+i][img->block_y+j];
            enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = direct_ref_idx[LIST_1][img->block_x+i][img->block_y+j];
          }
          else
          {   //intra  帧内预测，没有使用参考帧
            enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = -1;
            enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = -1;
            
          }
        }
    }
	//非bframe
    else
    {
      for (j=0;j<4;j++)
        for (i=0;i<4;i++)
        {
          enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = (mode==0?0:-1);
        }
    }
  }
  else
  {
    if (bframe)
    {
      for (j=0;j<4;j++)
        for (i=0;i<4;i++)
        {
          k = 2*(j/2)+(i/2);
          l = 2*(j%2)+(i%2);
          
          if(mode == P8x8 && best8x8mode[k]==0)
          {           
            enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = direct_ref_idx[LIST_0][img->block_x+i][img->block_y+j];
            enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = direct_ref_idx[LIST_1][img->block_x+i][img->block_y+j];
          }
          else
          {
            enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = (IS_FW ? best8x8fwref[mode][k] : -1);
            enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = (IS_BW ? best8x8bwref[mode][k] : -1);
          }
        }
    }
    else
    {
      for (j=0;j<4;j++)
        for (i=0;i<4;i++)
        {
          k = 2*(j/2)+(i/2);
          l = 2*(j%2)+(i%2);
		  //enc_picture->ref_idx中是按4x4块为单位存储参考帧的
          enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = (IS_FW ? best8x8fwref[mode][k] : -1);
        }
    }
  }

  for (j=0;j<4;j++)
  {
    for (i=0;i<4;i++)
    {
      enc_picture->ref_pic_id [LIST_0][img->block_x+i][img->block_y+j] = 
        (enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j]>=0 ? 
         enc_picture->ref_pic_num[LIST_0 + list_offset][enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j]]:
         -1);
    }
  }
  if (bframe)
  {
    for (j=0;j<4;j++)
    {
      for (i=0;i<4;i++)
      {
        enc_picture->ref_pic_id [LIST_1][img->block_x+i][img->block_y+j] = 
          (enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j]>=0 ? 
           enc_picture->ref_pic_num[LIST_1 + list_offset][enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j]]:
           -1);
      }
      
    }
  }


#undef IS_FW
#undef IS_BW
}


/*! 
 *************************************************************************************
 * \brief
 *    Intra 16x16 mode decision
 *************************************************************************************
 */
void
Intra16x16_Mode_Decision (Macroblock* currMB, int* i16mode)
{
  //intrapred_luma_16x16, 将帧内16x16所有模式的预测值保存在img->mprr_2[mode][][]中
  intrapred_luma_16x16 ();   /* make intra pred for all 4 new modes */		//,调用intrapred_luma_16x16函数,计算I16MB的4种模式下的预测值,保存在img->mprr_2中
  find_sad_16x16 (i16mode);   /* get best new intra mode */					//通过4种预测模式的预测值，得到帧内16x16的最佳模式(使用的是Hadamard变换)
  currMB->cbp = dct_luma_16x16 (*i16mode);									//得到了16x16的最佳预测模式，要进行预测误差的dct变换，并在enc_picture->imgY中保存重建值！
}



/*! 
 *************************************************************************************
 * \brief
 *    Sets Coefficients and reconstruction for an 8x8 block
 *	  设置系数和重建图像
 *************************************************************************************
 */
void SetCoeffAndReconstruction8x8 (Macroblock* currMB)
{
  int block, k, j, i;
  int block_y = img->block_y;
  int **ipredmodes = img->ipredmode;

  //--- restore coefficients ---
  //释放系数
  for (block=0; block<6; block++)
  for (k=0; k<4; k++)
  for (j=0; j<2; j++)
  for (i=0; i<65; i++)
    img->cofAC[block][k][j][i] = cofAC8x8[block][k][j][i];

  //恢复重建图像
  //cnt_nonz表示预测残差中不为0的个数，当小于5时，就直接用预测值做重建值enc_pictuire->imgY
  //当预测残差数大于5时，用重建值做enc_picture->imgY
  if (cnt_nonz_8x8<=5 && img->type!=SP_SLICE)
  {
    currMB->cbp     = 0;
    currMB->cbp_blk = 0;
    for (j=0; j<16; j++)
    for (i=0; i<16; i++)  enc_picture->imgY[img->pix_y+j][img->pix_x+i] = mpr8x8[j][i];
  }
  //保存重建图像
  else
  {
    currMB->cbp     = cbp8x8;
    currMB->cbp_blk = cbp_blk8x8;
    for (j=0; j<16; j++)
    for (i=0; i<16; i++)  enc_picture->imgY[img->pix_y+j][img->pix_x+i] = rec_mbY8x8[j][i];
  }

  //===== restore intra prediction modes for 8x8+ macroblock mode =====
  //恢复帧内预测模式
  for (k=0, j=block_y; j<block_y+4; j++)
  for (     i=img->block_x; i<img->block_x+4; i++, k++)
  {
    ipredmodes    [i][j] = b8_ipredmode       [k];
    currMB->intra_pred_modes[k] = b8_intra_pred_modes[k];
  }
}


/*! 
 *************************************************************************************
 * \brief
 *    Sets motion vectors for an macroblock
 *************************************************************************************
 */
void SetMotionVectorsMB (Macroblock* currMB, int bframe)
{
  int i, j, k, l, m, mode8, pdir8, ref, by, bx, bxr;
  int ******all_mv  = img->all_mv;
  int ******pred_mv = img->pred_mv;
  int  bw_ref;

  // 以4x4块为单位，进行保存
  for (j=0; j<4; j++)
    for (i=0; i<4; i++)
    {
      mode8 = currMB->b8mode[k=2*(j/2)+(i/2)];				//当前4x4块的最佳模式
      l     = 2*(j%2)+(i%2);
      by    = img->block_y+j;
      bxr   = img->block_x+i;
      bx    = img->block_x+i+4;


      
        pdir8 = currMB->b8pdir[k];							//当前4x4块的预测方向
        ref    = enc_picture->ref_idx[LIST_0][bxr][by];		//当前4x4块的参考帧
        bw_ref = enc_picture->ref_idx[LIST_1][bxr][by];
      
      
      if (!bframe)
      {
        if (pdir8>=0) //(mode8!=IBLOCK)&&(mode8!=I16MB))  // && ref != -1)
        {
          enc_picture->mv[LIST_0][bxr][by][0] = all_mv [i][j][LIST_0][ ref][mode8][0];
          enc_picture->mv[LIST_0][bxr][by][1] = all_mv [i][j][LIST_0][ ref][mode8][1];
        }
        else
        {
          enc_picture->mv[LIST_0][bxr][by][0] = 0;
          enc_picture->mv[LIST_0][bxr][by][1] = 0;
        }
      }
      else
      {
        if (pdir8==-1) // intra
        {
          enc_picture->mv[LIST_0][bxr][by][0] = 0;
          enc_picture->mv[LIST_0][bxr][by][1] = 0;
          enc_picture->mv[LIST_1][bxr][by][0] = 0;
          enc_picture->mv[LIST_1][bxr][by][1] = 0;
        }
        else if (pdir8==0) // forward
        {
          enc_picture->mv[LIST_0][bxr][by][0] = all_mv [i][j][LIST_0][ ref][mode8][0];
          enc_picture->mv[LIST_0][bxr][by][1] = all_mv [i][j][LIST_0][ ref][mode8][1];
          enc_picture->mv[LIST_1][bxr][by][0] = 0;
          enc_picture->mv[LIST_1][bxr][by][1] = 0;
        }
        else if (pdir8==1) // backward
        {
          enc_picture->mv[LIST_0][bxr][by][0] = 0;
          enc_picture->mv[LIST_0][bxr][by][1] = 0;
          
          enc_picture->mv[LIST_1][bxr][by][0] = all_mv[i][j][LIST_1][bw_ref][mode8][0];
          enc_picture->mv[LIST_1][bxr][by][1] = all_mv[i][j][LIST_1][bw_ref][mode8][1];
        }
        else if (pdir8==2) // bidir
        {
          enc_picture->mv[LIST_0][bxr][by][0] = all_mv [i][j][LIST_0][ ref][mode8][0];
          enc_picture->mv[LIST_0][bxr][by][1] = all_mv [i][j][LIST_0][ ref][mode8][1];
          

          enc_picture->mv[LIST_1][bxr][by][0] = all_mv[i][j][LIST_1][bw_ref][mode8][0];
          enc_picture->mv[LIST_1][bxr][by][1] = all_mv[i][j][LIST_1][bw_ref][mode8][1];
        }
        else 
        {
          error("invalid direction mode", 255);
        }
      }
  }
  
  // copy all the motion vectors into rdopt structure
  // Can simplify this by copying the MV's of the best mode (TBD)
  if(img->MbaffFrameFlag)
  {
    for(i=0;i<4;i++)
      for(j=0;j<4;j++)
        for (k=0;k<2;k++)
          for(l=0;l<img->max_num_references;l++)
            for(m=0;m<9;m++)
            {
              rdopt->all_mv [i][j][k][l][m][0]  = all_mv [i][j][k][l][m][0];
              rdopt->pred_mv[i][j][k][l][m][0]  = pred_mv[i][j][k][l][m][0];
              
              rdopt->all_mv [i][j][k][l][m][1]  = all_mv [i][j][k][l][m][1];
              rdopt->pred_mv[i][j][k][l][m][1]  = pred_mv[i][j][k][l][m][1];
            }
  }
}



/*! 
 *************************************************************************************
 * \brief
 *    R-D Cost for a macroblock
 *	  宏块的率失真代价
 *************************************************************************************
 */
int RDCost_for_macroblocks (double   lambda,      // <-- lagrange multiplier
                        int      mode,        // <-- modus (0-COPY/DIRECT, 1-16x16, 2-16x8, 3-8x16, 4-8x8(+), 5-Intra4x4, 6-Intra16x16)
                        double*  min_rdcost)  // <-> minimum rate-distortion cost
{
  int         i, j, k; //, k, ****ip4;
  int         i16mode, rate=0, distortion=0;
  double      rdcost;
  Macroblock  *currMB   = &img->mb_data[img->current_mb_nr];
  Macroblock  *prevMB   = img->current_mb_nr ? &img->mb_data[img->current_mb_nr-1] : NULL;
  int         bframe    = (img->type==B_SLICE);
  int         tmp_cc;
  int         use_of_cc =  (img->type!=I_SLICE &&  input->symbol_mode!=CABAC);
  int         cc_rate, dummy;
  
  //=====
  //=====  SET REFERENCE FRAMES AND BLOCK MODES
  //=====
  SetModesAndRefframeForBlocks (mode);

  //=====
  //=====  GET COEFFICIENTS, RECONSTRUCTIONS, CBP
  //=====
  if (bframe && mode==0)
  {
    int block_x=img->pix_x>>2;
    int block_y=img->pix_y>>2;
    for (i=0;i<4;i++)
      for (j=0;j<4;j++)
        if (direct_pdir[block_x+i][block_y+j]<0)
          return 0;
  }

  //宏块级的的rdo模式代价计算
  if (mode<P8x8)
  {
	//将宏块分成4个8x8块，再将每个8x8块分成4个4x4子块
	//对4x4子块，进行运动补偿，得到预测图像，保存在img->mpr中,再计算预测误差, 并计算对应的预测误差系数
	//这个函数中，已经执行了dct_luma，对每个4x4子块的预测误差进行了变换，并且将重建值保存在了enc_picture->imgY中
	//虽然在之前，已经对mode<P8x8的进行了最佳模式，最佳参考帧，运动矢量的计算，但是该处重复，主要是计算了指定模式的重建值，以及残差系数编码
	//这里的计算根据是MODE_COST = ssd + lambda_mode*rate
    LumaResidualCoding ();
  }
  else if (mode==P8x8)
  {
    SetCoeffAndReconstruction8x8 (currMB);
  }
  //帧内4x4
  else if (mode==I4MB)
  {		
	  //在里面计算出宏块的distortion，以及MODE_COST_4x4
	  //这个函数中，已经执行了dct_luma，对每个4x4子块的预测误差进行了变换，并且将重建值保存在了enc_picture->imgY中
	  //non rdo, distortion = sad, MODE_COST_4x4 = distortion + 4*lambda_mode
	  //  s  rdo, distortion = ssd, MODE_COST_4x4 = distortion + lambda_mode*BLOCKbits
    currMB->cbp = Mode_Decision_for_Intra4x4Macroblock (lambda, &dummy);		//cbp（code_block_patern） 指定一个宏块6个8x8残差的编码方案
  }
  //帧内16x16
  else if (mode==I16MB)
  {
	  //需要注意的是该函数中计算distortion的方法，non rdo和rdo都是一样的，使用的是sad
	  //只不过这里是rdo模式，在计算出distortion后，还会使用lambda_mode*BLOCKbits
	  //但是这里个函数，调用dct_luma_16x16计算了预测误差系数，并且将重建值保存在了enc_picture->imgY中
	  //non rdo, distortion = sad, MODE_COST_16x16 = distortion
	  //    rdo, distortion = ssd, MODE_COST_16x16 = distortion
    Intra16x16_Mode_Decision  (currMB, &i16mode);								//在该函数中，获得当前宏块cbp
  }

  if (input->rdopt==2 && img->type!=B_SLICE)
  {
    // We need the reconstructed prediction residue for the simulated decoders.
    compute_residue_mb (mode==I16MB?i16mode:-1);
  }

  //Rate control
  if (mode == I16MB)
  {
	  for(i=0; i<16; i++)
	  for(j=0; j<16; j++)
	    pred[j][i] = img->mprr_2[i16mode][j][i];
  }else
  {
	  for(i=0; i<16; i++)
	  for(j=0; j<16; j++)
	    pred[j][i] = img->mpr[i][j];
  }

  img->i16offset = 0;
  dummy = 0;
  ChromaResidualCoding (&dummy);
  if (mode==I16MB)     img->i16offset = I16Offset  (currMB->cbp, i16mode);


  //=====
  //=====   GET DISTORTION
  //=====
  // LUMA
  if (input->rdopt==2 && img->type!=B_SLICE)
  {
    for (k=0; k<input->NoOfDecoders ;k++)
    {
      decode_one_mb (k, currMB);
      for (j=0; j<MB_BLOCK_SIZE; j++)
      for (i=img->opix_x; i<img->opix_x+MB_BLOCK_SIZE; i++)
      {
        distortion += img->quad [imgY_org[img->opix_y+j][i] - decs->decY[k][img->opix_y+j][i]];
      }
    }
    distortion /= input->NoOfDecoders;
  }
  else
  {
    for (j=0; j<16; j++)
    for (i=img->opix_x; i<img->opix_x+16; i++)
    {
      distortion += img->quad [imgY_org[j+img->opix_y][i] - enc_picture->imgY[img->pix_y+j][i]];		//重建误差：ssd
    }
  }

  // CHROMA
  for (j=0; j<8; j++)
  for (i=img->opix_c_x; i<img->opix_c_x+8; i++)
  {
    distortion += img->quad [imgUV_org[0][j+img->opix_c_y][i] - enc_picture->imgUV[0][img->pix_c_y+j][i]];
    distortion += img->quad [imgUV_org[1][j+img->opix_c_y][i] - enc_picture->imgUV[1][img->pix_c_y+j][i]];
  }


  //=====   S T O R E   C O D I N G   S T A T E   =====
  //---------------------------------------------------
  store_coding_state (cs_cm);
  

  //=====
  //=====   GET RATE
  //=====
  //----- macroblock header -----
  if (use_of_cc)
  {
    if (currMB->mb_type!=0 || (bframe && currMB->cbp!=0))
    {
      // cod counter and macroblock mode are written ==> do not consider code counter
      tmp_cc = img->cod_counter;
      rate   = writeMBHeader (1); 
      ue_linfo (tmp_cc, dummy, &cc_rate, &dummy);
      rate  -= cc_rate;
      img->cod_counter = tmp_cc;
    }
    else
    {
      // cod counter is just increased  ==> get additional rate
      ue_linfo (img->cod_counter+1, dummy, &rate,    &dummy);
      ue_linfo (img->cod_counter,   dummy, &cc_rate, &dummy);
      rate -= cc_rate;
    }
  }
  else
  {
    rate = writeMBHeader (1); 
  }
  if (mode)
  {
    //----- motion information -----
    rate  += writeMotionInfo2NAL  ();
  }
  if (mode || (bframe && (currMB->cbp!=0 || input->symbol_mode==CABAC)))
  {
    rate  += writeCBPandLumaCoeff ();

    rate  += writeChromaCoeff     ();
  }


  //=====   R E S T O R E   C O D I N G   S T A T E   =====
  //-------------------------------------------------------
  reset_coding_state (cs_cm);

//计算率失真! 当率失真比min_rdcost还大时，说明当前这个模式，不是最好的预测模式，直接返回0**********************************************************
  rdcost = (double)distortion + lambda * (double)rate;

  if (rdcost >= *min_rdcost)
  {
    return 0;
  }
//*************************************************************************************************************************************************

  if ((img->MbaffFrameFlag) && (mode ? 0: ((img->type == B_SLICE) ? !currMB->cbp:1)))  // AFF and current is skip
  {
    if (img->current_mb_nr%2) //bottom
    {
      if (prevMB->mb_type ? 0:((img->type == B_SLICE) ? !prevMB->cbp:1)) //top is skip
      {
        if (!(field_flag_inference() == currMB->mb_field)) //skip only allowed when correct inference
          return 0;
      }
    }
  }

 
  //=====   U P D A T E   M I N I M U M   C O S T   =====
  //-----------------------------------------------------
  *min_rdcost = rdcost;
  return 1;
}





/*! 
 *************************************************************************************
 * \brief
 *    Store macroblock parameters
 *************************************************************************************
 */
void store_macroblock_parameters (int mode)
{
  int  i, j, k, ****i4p, ***i3p;
  Macroblock *currMB  = &img->mb_data[img->current_mb_nr];
  int        bframe   = (img->type==B_SLICE);

  //--- store best mode ---
  // 1). 保存当前宏块最佳预测模式
  best_mode = mode;
  best_c_imode = currMB->c_ipred_mode;
  best_i16offset = img->i16offset;		//img->i16offset是帧内16x16预测模式相关的值，缓存起来
  for (i=0; i<4; i++)					//缓存4个8x8块模式各自的最佳分割模式
  {
    b8mode[i]   = currMB->b8mode[i];
    b8pdir[i]   = currMB->b8pdir[i];
  }

  //--- reconstructed blocks ----
  // 2).保存当前宏块重建值
  for (j=0; j<16; j++)
  for (i=0; i<16; i++)
  {
    rec_mbY[j][i] = enc_picture->imgY[img->pix_y+j][img->pix_x+i];
  }
  for (j=0; j<8; j++)
  for (i=0; i<8; i++)
  {
    rec_mbU[j][i] = enc_picture->imgUV[0][img->pix_c_y+j][img->pix_c_x+i];
    rec_mbV[j][i] = enc_picture->imgUV[1][img->pix_c_y+j][img->pix_c_x+i];
  }


  //--- store results of decoders ---
  // 3).保存解码的结果(rdopt==2且B片时才进行)
  if (input->rdopt==2 && img->type!=B_SLICE)
  {
    for (k=0; k<input->NoOfDecoders; k++)
    {
      for (j=img->pix_y; j<img->pix_y+16; j++)
      for (i=img->pix_x; i<img->pix_x+16; i++)
      {
        // Keep the decoded values of each MB for updating the ref frames
        decs->decY_best[k][j][i] = decs->decY[k][j][i];
      }
    }
  }

  //--- coeff, cbp, kac ---
  //预测误差变换系数、cbp、kac保存
  if (mode || bframe)
  {
	  //i4p作为中间变量，交换cofAC和img->cofAC的指针，这样后面对img->cofAC的操作，便不是对原最佳img->cofAC的操作，最佳预测误差系数，由cofAC指定
	  //i3p同上
    i4p=cofAC; cofAC=img->cofAC; img->cofAC=i4p;
    i3p=cofDC; cofDC=img->cofDC; img->cofDC=i3p;
    cbp     = currMB->cbp;
    cbp_blk = currMB->cbp_blk;
  }
  else
  {
    cbp = cbp_blk = 0;
  }

  //每个4x4块进行参考帧保存
  for (j=0; j<4; j++)
  for (i=0; i<4; i++)
  {  
    frefframe[j][i] = enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j];

    if (bframe)
    {
      brefframe[j][i] = enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j];
    }
  }

}


/*! 
 *************************************************************************************
 * \brief
 *    Set stored macroblock parameters
 *	  找到最佳模式后，恢复宏块的参数
 *************************************************************************************
 */
void set_stored_macroblock_parameters ()
{
  int  i, j, k, ****i4p, ***i3p,l;
  Macroblock  *currMB  = &img->mb_data[img->current_mb_nr];		//当前宏块数据
  int         mode     = best_mode;								//当前宏块最优模式
  int         bframe   = (img->type==B_SLICE);					//
  int         **ipredmodes = img->ipredmode;
  
  byte        **imgY  = enc_picture->imgY;						//保存重建像素
  byte       ***imgUV = enc_picture->imgUV;						//

  int        list_offset   = ((img->MbaffFrameFlag)&&(currMB->mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  //===== 1).reconstruction values =====================================================================================================================
  //Y分量重建像素
  for (j=0; j<16; j++)
  for (i=0; i<16; i++)
  {
    imgY[img->pix_y+j][img->pix_x+i] = rec_mbY[j][i];

    if(img->MbaffFrameFlag)
      rdopt->rec_mbY[j][i]       = rec_mbY[j][i]; 
  }

  //UV分量重建像素
  for (j=0; j<8; j++)
  for (i=0; i<8; i++)
  {
    imgUV[0][img->pix_c_y+j][img->pix_c_x+i] = rec_mbU[j][i];
    imgUV[1][img->pix_c_y+j][img->pix_c_x+i] = rec_mbV[j][i];

    if(img->MbaffFrameFlag)
    {
      rdopt->rec_mbU[j][i]           = rec_mbU[j][i];   
      rdopt->rec_mbV[j][i]           = rec_mbV[j][i];   
    }
  }

  //===== 2). coefficients and cbp ======================================================================================================================
  i4p=cofAC; cofAC=img->cofAC; img->cofAC=i4p;
  i3p=cofDC; cofDC=img->cofDC; img->cofDC=i3p;
  currMB->cbp      = cbp;
  currMB->cbp_blk = cbp_blk;
  //==== 3). macroblock type ============================================================================================================================
  currMB->mb_type = mode;


  if(img->MbaffFrameFlag)
  {
    rdopt->mode = mode;
    rdopt->i16offset = img->i16offset;
    rdopt->cbp = cbp;
    rdopt->cbp_blk = cbp_blk;
    rdopt->mb_type  = mode;

    rdopt->prev_qp=currMB->prev_qp;
    rdopt->prev_delta_qp=currMB->prev_delta_qp;
    rdopt->qp=currMB->qp;

    for(i=0;i<6;i++)
      for(j=0;j<4;j++)
        for(k=0;k<2;k++)
          for(l=0;l<18;l++)
            rdopt->cofAC[i][j][k][l] = img->cofAC[i][j][k][l];

    for(i=0;i<3;i++)
        for(k=0;k<2;k++)
          for(l=0;l<18;l++)
            rdopt->cofDC[i][k][l] = img->cofDC[i][k][l];
  }


  //保存8x8块为单位的预测模式和预测方向
  for (i=0; i<4; i++)
  {
    currMB->b8mode[i]   = b8mode[i];
    currMB->b8pdir[i]   = b8pdir[i];
    if(img->MbaffFrameFlag)
    {
      rdopt->b8mode[i]  = b8mode[i];                  
      rdopt->b8pdir[i]  = b8pdir[i];                  
    }
  }

  if (input->rdopt==2 && img->type!=B_SLICE)
  {
    //! save the MB Mode of every macroblock
    decs->dec_mb_mode[img->mb_x][img->mb_y] = mode;
  }

  //==== reference frames =====
  //保存参考帧
  for (j=0; j<4; j++)
  for (i=0; i<4; i++)
  {
    // backward prediction or intra
	// 后向预测或帧内预测，LIST0的所有相关数据都不可用
    if ((currMB->b8pdir[i/2+(j/2)*2] == 1) || IS_INTRA(currMB))
    {
      enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = -1;
      enc_picture->ref_pic_id [LIST_0][img->block_x+i][img->block_y+j] = -1;

      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][0] =0;
      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][1] = 0;
      if(img->MbaffFrameFlag)
        rdopt->refar[LIST_0][j][i] = -1;
    }
	// 及非后向预测，亦非帧内预测，LIST0相关数据(参考帧，运动矢量)保存
    else
    {
      enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = frefframe[j][i]; 
      enc_picture->ref_pic_id [LIST_0][img->block_x+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_0 + list_offset][enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j]];

      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][0] = img->all_mv[i][j][LIST_0][frefframe[j][i]][currMB->b8mode[i/2+(j/2)*2]][0];
      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][1] = img->all_mv[i][j][LIST_0][frefframe[j][i]][currMB->b8mode[i/2+(j/2)*2]][1];
      if(img->MbaffFrameFlag)
        rdopt->refar[LIST_0][j][i] = frefframe[j][i];
    }

    // forward prediction or intra
	// 前向预测或帧内预测，LIST1的所有相关数据都不可用
    if ((currMB->b8pdir[i/2+(j/2)*2] == 0) || IS_INTRA(currMB))
    {
      enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = -1;
      enc_picture->ref_pic_id [LIST_1][img->block_x+i][img->block_y+j] = -1;  
      enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][0] =0;
      enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][1] = 0;
      if(img->MbaffFrameFlag)
        rdopt->refar[LIST_1][j][i] = -1;
    }
    
  }

  if (bframe)
  {
    for (j=0; j<4; j++)
      for (i=0; i<4; i++)
      {
        
        // forward
        if (IS_INTRA(currMB)||(currMB->b8pdir[i/2+(j/2)*2] == 0))
        {
          enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = -1;
          enc_picture->ref_pic_id [LIST_1][img->block_x+i][img->block_y+j] = -1;
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][0] = 0;
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][1] = 0;
          if(img->MbaffFrameFlag)
            rdopt->refar[LIST_1][j][i] = -1;
        }
        else
        {
          enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = brefframe[j][i];
          enc_picture->ref_pic_id [LIST_1][img->block_x+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_1 + list_offset][enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j]];
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][0] = img->all_mv[i][j][LIST_1][brefframe[j][i]][currMB->b8mode[i/2+(j/2)*2]][0];
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][1] = img->all_mv[i][j][LIST_1][brefframe[j][i]][currMB->b8mode[i/2+(j/2)*2]][1];
          if(img->MbaffFrameFlag)
            rdopt->refar[LIST_1][j][i] = brefframe[j][i];
        }
      }
  }

  //==== intra prediction modes ====
  currMB->c_ipred_mode = best_c_imode;
  img->i16offset = best_i16offset;				//img->i16offset是帧内16x16预测模式相关的值, 恢复
  if (mode==P8x8)
  {
    for (k=0, j=img->block_y; j<img->block_y+4; j++)
      for (   i=img->block_x; i<img->block_x+4; i++, k++)
      {
        ipredmodes           [i][j] = b8_ipredmode       [k];
        currMB->intra_pred_modes[k] = b8_intra_pred_modes[k];
      }
  }
  else if (mode!=I4MB)
  {
    for (k=0, j=img->block_y; j<img->block_y+4; j++)
      for (   i=img->block_x; i<img->block_x+4; i++, k++)
      {
        ipredmodes           [i][j] = DC_PRED;
        currMB->intra_pred_modes[k] = DC_PRED;
      }
  }

  if(img->MbaffFrameFlag)
  {
    for (k=0, j=img->block_y; j<img->block_y+4; j++)
      for (   i=img->block_x; i<img->block_x+4; i++, k++)
      {
        rdopt->ipredmode[i][j]     = ipredmodes[i][j];
        rdopt->intra_pred_modes[k] = currMB->intra_pred_modes[k];
      }
    rdopt->c_ipred_mode = currMB->c_ipred_mode;
    rdopt->i16offset = img->i16offset;  // DH
  }

  //==== motion vectors =====
  //将运动矢量，保存在enc_picture->mv中
  SetMotionVectorsMB (currMB, bframe);
}



/*! 
 *************************************************************************************
 * \brief
 *    Set reference frames and motion vectors
 *************************************************************************************
 */
void SetRefAndMotionVectors (int block, int mode, int pdir, int fwref, int bwref)
{
  int     i, j=0;
  int     bslice  = (img->type==B_SLICE);
  int     pmode   = (mode==1||mode==2||mode==3?mode:4);
  int     j0      = ((block/2)<<1);
  int     i0      = ((block%2)<<1);
  int     j1      = j0 + (input->blc_size[pmode][1]>>2);
  int     i1      = i0 + (input->blc_size[pmode][0]>>2);

  int list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;


  if (pdir<0)
  {
    for (j=j0; j<j1; j++)
    for (i=i0; i<i1; i++)
    {
      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][0] = 0;
      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][1] = 0;
      enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][0] = 0;
      enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][1] = 0;
      enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = -1;
      enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = -1;
      enc_picture->ref_pic_id[LIST_0][img->block_x+i][img->block_y+j] = -1;
      enc_picture->ref_pic_id[LIST_1][img->block_x+i][img->block_y+j] = -1;
    }
    return;
  }

  if (!bslice)
  {
    for (j=j0; j<j1; j++)
    for (i=i0; i<i1; i++)
    {
      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][0] = img->all_mv[i][j][LIST_0][fwref][mode][0];
      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][1] = img->all_mv[i][j][LIST_0][fwref][mode][1];
      enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = fwref;
      enc_picture->ref_pic_id[LIST_0][img->block_x+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_0+list_offset][enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j]];
    }
  }
  else
  {
    for (j=j0; j<j1; j++)
      for (i=i0; i<i1; i++)
      {
        if (mode==0)
        {
          pdir  =direct_pdir[img->block_x+i][img->block_y+j];
          fwref =direct_ref_idx[LIST_0][img->block_x+i][img->block_y+j];
          bwref =direct_ref_idx[LIST_1][img->block_x+i][img->block_y+j];
        }
        
        if ((pdir==0 || pdir==2))
        {
          enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][0] = img->all_mv[i][j][LIST_0][fwref][mode][0];
          enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][1] = img->all_mv[i][j][LIST_0][fwref][mode][1];
          enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = fwref;
          enc_picture->ref_pic_id[LIST_0][img->block_x+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_0+list_offset][enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j]];
        }
        else
        {
          enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][0] = 0;
          enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][1] = 0;
          enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = -1;
          enc_picture->ref_pic_id[LIST_0][img->block_x+i][img->block_y+j] = -1;
        }

        if ((pdir==1 || pdir==2))
        {
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][0] = img->all_mv[i][j][LIST_1][bwref][mode][0];
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][1] = img->all_mv[i][j][LIST_1][bwref][mode][1];
          enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = bwref;
          enc_picture->ref_pic_id[LIST_1][img->block_x+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_1+list_offset][enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j]];
        }
        else
        {
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][0] = 0;
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][1] = 0;
          enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = -1;
          enc_picture->ref_pic_id[LIST_1][img->block_x+i][img->block_y+j] = -1;
        }
      }
  }
}

/*! 
 *************************************************************************************
 * \brief
 *    skip macroblock field inference
 * \return
 *    inferred field flag
 *************************************************************************************
 */
int field_flag_inference()
{
  int mb_field;
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  if (currMB->mbAvailA)
  {
    mb_field = img->mb_data[currMB->mbAddrA].mb_field;
  }
  else
  {
    // check top macroblock pair
    if (currMB->mbAvailB)
    {
      mb_field = img->mb_data[currMB->mbAddrB].mb_field;
    }
    else
      mb_field = 0;
  }

  return mb_field;
}

/*! 
 *************************************************************************************
 * \brief
 *    Mode Decision for a macroblock
 *************************************************************************************
 */
 void encode_one_macroblock ()
 {
   static const int  b8_mode_table[6]  = {0, 4, 5, 6, 7};         // DO NOT CHANGE ORDER !!!
   static const int  mb_mode_table[7]  = {0, 1, 2, 3, P8x8, I16MB, I4MB}; // DO NOT CHANGE ORDER !!!
   
   int         valid[MAXMODE];
   int         rerun, block, index, mode, i0, i1, j0, j1, pdir, ref, i, j, k, ctr16x16, dummy;
   double      qp, lambda_mode, lambda_motion, min_rdcost, rdcost = 0, max_rdcost=1e30;
   int         lambda_motion_factor;
   int         fw_mcost, bw_mcost, bid_mcost, mcost, max_mcost=(1<<30);
   int         curr_cbp_blk, cnt_nonz = 0, best_cnt_nonz = 0, best_fw_ref = 0, best_pdir;
   int         cost=0;
   int         min_cost = max_mcost, min_cost8x8, cost8x8, cost_direct=0, have_direct=0, i16mode;
   int         intra1 = 0;
   
   int         intra       = (((img->type==P_SLICE||img->type==SP_SLICE) && img->mb_y==img->mb_y_upd && img->mb_y_upd!=img->mb_y_intra) || img->type==I_SLICE);
   int         spframe     = (img->type==SP_SLICE);
   int         siframe     = (img->type==SI_SLICE);
   int         bframe      = (img->type==B_SLICE);
   int         runs        = (input->RestrictRef==1 && input->rdopt==2 && (img->type==P_SLICE || img->type==SP_SLICE || (img->type==B_SLICE && img->nal_reference_idc>0)) ? 2 : 1);
   
   int         checkref    = (input->rdopt && input->RestrictRef && (img->type==P_SLICE || img->type==SP_SLICE));
   Macroblock* currMB      = &img->mb_data[img->current_mb_nr];
   Macroblock* prevMB      = img->current_mb_nr ? &img->mb_data[img->current_mb_nr-1]:NULL ;
   
   int     **ipredmodes = img->ipredmode;
   int     best_bw_ref = -1;
   int     ******allmvs = img->all_mv;				//该宏块的所有运动矢量

   
   int  l,list_offset;

   int curr_mb_field = ((img->MbaffFrameFlag)&&(currMB->mb_field));

   // find out the correct list offsets
   if (curr_mb_field)
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

   if(input->FMEnable)
     decide_intrabk_SAD();
   
   intra |= RandomIntra (img->current_mb_nr);    // Forced Pseudo-Random Intra， 是否对宏块进行帧内预测。

   //===== SET VALID MODES =====
   valid[I4MB]   = 1;										// 帧内4x4预测			mode == 9
   valid[I16MB]  = 1;										// 帧内16x16预测		mode == 10
   
   valid[0]      = (!intra );								// 帧间预测				mode == 0
   valid[1]      = (!intra && input->InterSearch16x16);		// 帧间16x16预测		mode == 1
   valid[2]      = (!intra && input->InterSearch16x8);		// 帧间16x8预测			mode == 2
   valid[3]      = (!intra && input->InterSearch8x16);		// 帧间8x16预测			mode == 3
   valid[4]      = (!intra && input->InterSearch8x8);		// 帧间8x8预测			mode == 4
   valid[5]      = (!intra && input->InterSearch8x4);		// 帧间8x4预测			mode == 5
   valid[6]      = (!intra && input->InterSearch4x8);		// 帧间4x8预测			mode == 6
   valid[7]      = (!intra && input->InterSearch4x4);		// 帧间4x4预测			mode == 7
   valid[P8x8]   = (valid[4] || valid[5] || valid[6] || valid[7]);	//				mode == 8
   valid[12]     = (siframe);										//				mode == 12

   if (!img->MbaffFrameFlag)
   {
     for (l=0+list_offset;l<(2+list_offset);l++)
     {
       for(k = 0; k < listXsize[l]; k++)
       {
         listX[l][k]->chroma_vector_adjustment= 0;
         if(img->structure == TOP_FIELD && img->structure != listX[l][k]->structure)
           listX[l][k]->chroma_vector_adjustment = -2;
         if(img->structure == BOTTOM_FIELD && img->structure != listX[l][k]->structure)
           listX[l][k]->chroma_vector_adjustment = 2;
       }
     }
   }
   else
   {
     if (curr_mb_field)
     {
       for (l=0+list_offset;l<(2+list_offset);l++)
       {
         for(k = 0; k < listXsize[l]; k++)
         {
           listX[l][k]->chroma_vector_adjustment= 0;
           if(img->current_mb_nr % 2 == 0 && listX[l][k]->structure == BOTTOM_FIELD)
             listX[l][k]->chroma_vector_adjustment = -2;
           if(img->current_mb_nr % 2 == 1 && listX[l][k]->structure == TOP_FIELD)
             listX[l][k]->chroma_vector_adjustment = 2;
         }
       }
     }
     else
     {
       for (l=0+list_offset;l<(2+list_offset);l++)
       {
         for(k = 0; k < listXsize[l]; k++)
         {
           listX[l][k]->chroma_vector_adjustment= 0;
         }
       }
     }
     
   }
   //===== SET LAGRANGE PARAMETERS =====
   if (input->rdopt)
   {
     qp = (double)img->qp - SHIFT_QP;

     if (input->successive_Bframe>0)
       lambda_mode   = 0.68 * pow (2, qp/3.0) * (img->type==B_SLICE? max(2.00,min(4.00,(qp / 6.0))):spframe?max(1.4,min(3.0,(qp / 12.0))):1.0);  
     else
       lambda_mode   = 0.85 * pow (2, qp/3.0) * (img->type==B_SLICE? 4.0:spframe?max(1.4,min(3.0,(qp / 12.0))):1.0);  

     lambda_motion = sqrt (lambda_mode);
   }
   else
   {
     lambda_mode = lambda_motion = QP2QUANT[max(0,img->qp-SHIFT_QP)];
   }
   lambda_motion_factor = LAMBDA_FACTOR (lambda_motion);

//****************************************************************************************************************************************************   
//****************************************************************************************************************************************************
//******************************************************************下面为帧间编码********************************************************************

   for (rerun=0; rerun<runs; rerun++)		//runs=2是针对loss rdo模式，其余的情况runs=1
   {
     if (runs==2)
     {
       if (rerun==0)   input->rdopt=1;
       else            input->rdopt=2;
     }
     
     // reset chroma intra predictor to default
     currMB->c_ipred_mode = DC_PRED_8;

//=================================================================================================================================================
//==================================================================下面为帧间编码=================================================================
//=================================================================================================================================================
	 // 帧间预测
     if (!intra)
     {
       //===== set direct motion vectors =====
		// b帧
       if (bframe)
       {
         Get_Direct_Motion_Vectors ();		//
       }
//*****************************************************************************************************************************************************
//下面在3种预测模式中，进行最佳预测模式选择，并选择相应的最佳参考帧************************************************************************************

       //===== MOTION ESTIMATION FOR 16x16, 16x8, 8x16 BLOCKS =====
	   //帧间3种模式(16x16 16x8 8x16)进行循环运动估计
	   //初始化best_mode=1(16x16), min_cost是最大值
       for (min_cost=1<<20, best_mode=1, mode=1; mode<4; mode++)
       {
		 // 判断当前宏块的mode是否可以使用
         if (valid[mode])
         {
			// 模式1时，仅有1个block(16x16)，循环两次
			// 模式!=1时，有2个block（8x18或16x8），循环两次
			// 计算宏块的cost
           for (cost=0, block=0; block<(mode==1?1:2); block++)
           {
			 // 对指定模式和搜索块块，对所有的参考帧进行运动搜索
			 // 得到该模式下的每个最佳mv（每个参考帧都有最佳mv），保存在img->all_mv中。得到该模式下的最优代价motion_cost[mode][LIST_0][ref][block]
			 // 还得到了对应的分块，所有参考帧的预测矢量：img->pred_mv[block_x][block_y][list][ref][blocktype]
             // 这里没有记录预测值，只记录了运动矢量
			 PartitionMotionSearch (mode, block, lambda_motion);

             //--- set 4x4 block indizes (for getting MV) ---
             j = (block==1 && mode==2 ? 2 : 0);
             i = (block==1 && mode==3 ? 2 : 0);

             //--- get cost and reference frame for forward prediction ---
			 // 获得代价 和 前向预测的参考帧
			 // 遍历所有参考帧
             for (fw_mcost=max_mcost, ref=0; ref<listXsize[LIST_0+list_offset]; ref++)			//参考帧选择
             {
               if (!checkref || ref==0 || CheckReliabilityOfRef (block, LIST_0, ref, mode))
               {
				 // 参考帧索引号编码所需要的bit数，是编码代价的一部分
				 // 该处直接区分rdo和非rdo
                 // rdo:		refcost = SAD + lambda_motion_factor*[ REFbits相关量 + MVbits相关量 ]
				 // non_rdo:	refcost = SAD + 2*lambda_motion*ref + lambda_motion*MVbits相关量
				 //SAD+*lambda_motion*MVbits相关量 已经保存于 motion_cost[mode][list][ref][block]中
				 mcost  = (input->rdopt ? REF_COST (lambda_motion_factor, ref, LIST_0 + list_offset) : (int)(2*lambda_motion*min(ref,1)));
                 mcost += motion_cost[mode][LIST_0][ref][block];		
				 //其实上面已经进行跟进rdo或者非rdo模式的运动矢量、参考帧、p16x16, p16x8, p8x16的模式选择
				 //后面还会和帧内模式一次再进行一次选择

                 if (mcost < fw_mcost)
                 {
                   fw_mcost    = mcost;					//指定模式下， 指定搜索块的，最佳参考帧对应的最小代价
                   best_fw_ref = ref;					//指定模式下， 指定搜索块的，最佳参考帧
                 }
               }
             }
			 
			 //p帧：cost      += fw_mcost;			//cost是指定宏块模式下，所有搜索块块编码的代价和

			 //b帧
             if (bframe)
             {
               //--- get cost for bidirectional prediction ---
               for (bw_mcost=max_mcost, ref=0; ref<listXsize[LIST_1 + list_offset]; ref++)
               {
                 mcost  = (input->rdopt ? REF_COST (lambda_motion_factor, ref, LIST_1 + list_offset) : (int)(2*lambda_motion*min(ref,1)));
                 mcost += motion_cost[mode][LIST_1][ref][block];
                 if (mcost < bw_mcost)
                 {
                   bw_mcost    = mcost;
                   best_bw_ref = ref;
                 }
               }

               // search bidirectional between best forward and ref_idx=0 backward
               bid_mcost  = (input->rdopt ? (REF_COST (lambda_motion_factor, best_fw_ref,LIST_0+list_offset)+REF_COST (lambda_motion_factor, 0,LIST_1+list_offset)) : (int)(2*lambda_motion*min(best_fw_ref,1)));
               bid_mcost += BIDPartitionCost (mode, block, best_fw_ref, 0, lambda_motion_factor);

               //--- get prediction direction ----
               if (fw_mcost<=bw_mcost && fw_mcost<=bid_mcost)
               {
                 best_pdir = 0;
                 best_bw_ref = 0;
                 cost += fw_mcost;
               }
               else if (bw_mcost<=fw_mcost && bw_mcost<=bid_mcost)
               {
                 best_pdir = 1;
                 cost += bw_mcost;
                 best_fw_ref = 0;
               }
               else
               {
                 best_pdir = 2;
                 cost += bid_mcost;
                 best_bw_ref = 0;
               }
             }
             else // if (bframe)
             {
			   //B帧要从三种方式(前向,后向,双向)中选择一个,pdir指定方向，从上面if中的代码中我们就可以看出来
               best_pdir  = 0;					//对于p帧，best_pdir直接设置为0
               cost      += fw_mcost;			//cost是指定宏块模式下，所有搜索块块编码的代价和
             }

			 // 下面进行最佳参考帧和相应运动矢量的存储，保存在enc_picture中。
			 // 每个元素是按照4x4子块排序的
			 // 模式1时的每个4x4子块最佳参考帧和相应运动矢量排序
             if (mode==1)
             {
			   //对于b帧，才会有best_pdir!=0
			   //best_pdir==1时，采用后向预测，于是LIST0的参考帧，运动矢量全部禁止
               if (best_pdir==1)
               {
                 for (j=0; j<4; j++)
                 {
                   for (i=0; i<4; i++)
                   {
                     enc_picture->ref_idx[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = -1;
                     enc_picture->ref_pic_id [LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = -1; 
                     enc_picture->mv[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][0] = 0;
                     enc_picture->mv[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][1] = 0;
                   }
                 }
               }
			   //best_pdir==0 或 2，采用前向或双向，总之LIST0是使用了的
               else
               {
                 for (j=0; j<4; j++)
                 {
                   for (i=0; i<4; i++)
                   {
                     enc_picture->ref_idx[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = best_fw_ref;
                     enc_picture->ref_pic_id [LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = enc_picture->ref_pic_num[LIST_0 + list_offset][enc_picture->ref_idx[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j]];  
                     enc_picture->mv[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][0] = img->all_mv[i][j][LIST_0][best_fw_ref][mode][0];
                     enc_picture->mv[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][1] = img->all_mv[i][j][LIST_0][best_fw_ref][mode][1];
                   }
                 }
               }

			   //B帧时，进一步设置enc_picture->ref_idx/ref_pic_id/mv的信息
               if (bframe)
               {
                 if (best_pdir==0)
                 {
                   for (j=0; j<4; j++)
                   {
                     for (i=0; i<4; i++)
                     {
                       enc_picture->ref_idx[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = -1;
                       enc_picture->ref_pic_id [LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = -1;   
                       enc_picture->mv[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][0] = 0;
                       enc_picture->mv[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][1] = 0;
                     }
                   }
                 }
                 else
                 {
                   for (j=0; j<4; j++)
                   {
                     for (i=0; i<4; i++)
                     {
                       enc_picture->ref_idx[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = best_bw_ref;
                       enc_picture->ref_pic_id [LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = enc_picture->ref_pic_num[LIST_1 + list_offset][enc_picture->ref_idx[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j]];
                       if(best_bw_ref>=0)
                       {
                         enc_picture->mv[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][0] = img->all_mv[i][j][LIST_1][best_bw_ref][mode][0];
                         enc_picture->mv[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][1] = img->all_mv[i][j][LIST_1][best_bw_ref][mode][1];
                       }
                     }
                   }
                 }
               }
             }
			 // 模式2时，搜索块中的每个4x4子块最佳参考帧和相应运动矢量排序
             else if (mode==2)
             {
			   // img->block_x+i，				为相应4x4子块在整个图像中的x方向上的位置(单位是4x4子块)
			   // img->block_y+block*2+j		为相应4x4子块在整个图像中的y方向上的位置(单位是4x4子块)
			   // block*2是因为mode=2时，搜索块的分割模式是上下两个16x8的块，每个搜索块在y方向上都只有2个4x4块
			   // block == 1时，为了跳过头顶的搜索块，所以应该有block*2的偏置
               for (j=0; j<2; j++)
               {
                 for (i=0; i<4; i++)
                 {
                   if (best_pdir==1)
                   {
                     enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+block*2+j] = -1;
                     enc_picture->ref_pic_id [LIST_0][img->block_x+i][img->block_y+block*2+j] = -1;
                     enc_picture->mv[LIST_0][img->block_x+i][img->block_y+block*2+j][0] = 0;
                     enc_picture->mv[LIST_0][img->block_x+i][img->block_y+block*2+j][1] = 0;
                   }
                   else
                   {
                     enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+block*2+j] = best_fw_ref;
                     enc_picture->ref_pic_id [LIST_0][img->block_x+i][img->block_y+block*2+j] = enc_picture->ref_pic_num[LIST_0 + list_offset][enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+block*2+j]];
                     enc_picture->mv[LIST_0][img->block_x+i][img->block_y+block*2+j][0] = img->all_mv[i][j+block*2][LIST_0][best_fw_ref][mode][0];
                     enc_picture->mv[LIST_0][img->block_x+i][img->block_y+block*2+j][1] = img->all_mv[i][j+block*2][LIST_0][best_fw_ref][mode][1];
                   }

                   if (bframe)
                   {
                     if (best_pdir==0)
                     {
                       enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+block*2+j] = -1;
                       enc_picture->ref_pic_id [LIST_1][img->block_x+i][img->block_y+block*2+j] = -1;
                       enc_picture->mv[LIST_1][img->block_x+i][img->block_y+block*2+j][0] = 0;
                       enc_picture->mv[LIST_1][img->block_x+i][img->block_y+block*2+j][1] = 0;
                     }
                     else
                     {
                       enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+block*2+j] = best_bw_ref;
                       enc_picture->ref_pic_id [LIST_1][img->block_x+i][img->block_y+block*2+j] = enc_picture->ref_pic_num[LIST_1 + list_offset][enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+block*2+j]];
                       if(best_bw_ref>=0)
                       {
                         enc_picture->mv[LIST_1][img->block_x+i][img->block_y+block*2+j][0] = img->all_mv[i][j][LIST_1][best_bw_ref][mode][0];
                         enc_picture->mv[LIST_1][img->block_x+i][img->block_y+block*2+j][1] = img->all_mv[i][j][LIST_1][best_bw_ref][mode][1];
                       }
                     }
                   }
                 }
               }
             }
			 // 模式3时，搜索块中的每个4x4子块最佳参考帧和相应运动矢量排序
             else
             {
			   // img->block_x+block*2+i，				为相应4x4子块在整个图像中的x方向上的位置(单位是4x4子块)
			   // img->block_y+j						为相应4x4子块在整个图像中的y方向上的位置(单位是4x4子块)
			   // block*2是因为mode=2时，搜索块的分割模式是左右两个8x16的块，每个搜索块在x方向上都只有2个4x4块
			   // block == 1时，为了跳过左方的搜索块，所以应该有block*2的偏置
               for (j=0; j<4; j++)
               {
                 for (i=0; i<2; i++)
                 {
                   if (best_pdir==1)
                   {
                     enc_picture->ref_idx[LIST_0][img->block_x+block*2+i][img->block_y+j] = -1;
                     enc_picture->ref_pic_id [LIST_0][img->block_x+block*2+i][img->block_y+j] = -1;
                     enc_picture->mv[LIST_0][img->block_x+block*2+i][img->block_y+j][0] = 0;
                     enc_picture->mv[LIST_0][img->block_x+block*2+i][img->block_y+j][1] = 0;
                   }
                   else
                   {
                     enc_picture->ref_idx[LIST_0][img->block_x+block*2+i][img->block_y+j] = best_fw_ref;
                     enc_picture->ref_pic_id [LIST_0][img->block_x+block*2+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_0 + list_offset][enc_picture->ref_idx[LIST_0][img->block_x+block*2+i][img->block_y+j]];
                     enc_picture->mv[LIST_0][img->block_x+block*2+i][img->block_y+j][0] = img->all_mv[i][j][LIST_0][best_fw_ref][mode][0];
                     enc_picture->mv[LIST_0][img->block_x+block*2+i][img->block_y+j][1] = img->all_mv[i][j][LIST_0][best_fw_ref][mode][1];
                   }

                   if (bframe)
                   {
                     if (best_pdir==0)
                     {
                       enc_picture->ref_idx[LIST_1][img->block_x+block*2+i][img->block_y+j] = -1;
                       enc_picture->ref_pic_id [LIST_1][img->block_x+block*2+i][img->block_y+j] = -1;
                       enc_picture->mv[LIST_1][img->block_x+block*2+i][img->block_y+j][0] = 0;
                       enc_picture->mv[LIST_1][img->block_x+block*2+i][img->block_y+j][1] = 0;
                     }
                     else
                     {
                       enc_picture->ref_idx[LIST_1][img->block_x+block*2+i][img->block_y+j] = best_bw_ref;
                       enc_picture->ref_pic_id [LIST_1][img->block_x+block*2+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_1 + list_offset][enc_picture->ref_idx[LIST_1][img->block_x+block*2+i][img->block_y+j]];
                       if(best_bw_ref>=0)
                       {
                         enc_picture->mv[LIST_1][img->block_x+block*2+i][img->block_y+j][0] = img->all_mv[i][j][LIST_1][best_bw_ref][mode][0];
                         enc_picture->mv[LIST_1][img->block_x+block*2+i][img->block_y+j][1] = img->all_mv[i][j][LIST_1][best_bw_ref][mode][1];
                       }
                     }
                   }
                 }
               }
             }

             //----- set reference frame and direction parameters -----
			 //这里记录参考帧和预测方向，是按8x8为单位的
			 //best8x8fwref[mode][k], 保存了指定模式，指定8x8块的最佳参考帧，在进行RDCost_for_macroblocks前，会将当前mode的参考帧从中取出
			 /************************************
			  *		 0 | 1    *8x8宏块分割序号如左图
			  *		 --|--    *所示
			  *		 2 | 3    *
			 *************************************/
             if (mode==3)		//(0, 2)相同, (1, 3)相同
             {
               best8x8fwref   [3][block] = best8x8fwref   [3][block+2] = best_fw_ref;			// 指定block的最佳前向预测参考帧
               best8x8pdir    [3][block] = best8x8pdir    [3][block+2] = best_pdir;
               best8x8bwref   [3][block] = best8x8bwref   [3][block+2] = best_bw_ref;			// 指定block的最佳后向预测参考帧
             }
             else if (mode==2)	//(0, 1)相同, (1, 3)相同
             {
               best8x8fwref   [2][2*block] = best8x8fwref   [2][2*block+1] = best_fw_ref;
               best8x8pdir    [2][2*block] = best8x8pdir    [2][2*block+1] = best_pdir;
               best8x8bwref   [2][2*block] = best8x8bwref   [2][2*block+1] = best_bw_ref;
             }
             else				////(0, 1, 2, 3)相同
             {
               best8x8fwref   [1][0] = best8x8fwref   [1][1] = best8x8fwref   [1][2] = best8x8fwref   [1][3] = best_fw_ref;
               best8x8pdir    [1][0] = best8x8pdir    [1][1] = best8x8pdir    [1][2] = best8x8pdir    [1][3] = best_pdir;
               best8x8bwref   [1][0] = best8x8bwref   [1][1] = best8x8bwref   [1][2] = best8x8bwref   [1][3] = best_bw_ref;
             }

             //--- set reference frames and motion vectors ---
			 // 设置参考帧和运动适量
			 // 这里，只对16x8 8x16进行参考帧和运动矢量保存，这是因为第二个搜寻块的矢量预测会用到第一个搜寻块的。
			 // 另外，实际上之前的操作，已经保存了相关的参考帧和运动矢量信息，这里是重复操作！
             if (mode>1 && block==0) 
               SetRefAndMotionVectors (block, mode, best_pdir, best_fw_ref, best_bw_ref);

          } // for (block=0; block<(mode==1?1:2); block++)

		  // 记录代价最小的模式best_mode，和相应的最小代价min_cost
          if (cost < min_cost)
          {
            best_mode = mode;
            min_cost  = cost;
          }
        } // if (valid[mode])
	   } // for (mode=1; mode<4; mode++)
//上面在3种预测模式中，进行最佳预测模式选择，并选择相应的最佳参考帧************************************************************************************
//*****************************************************************************************************************************************************
// 以下是帧间预测8x8模式，8x8还会继续分为亚宏块模式****************************************************************************************************
      if (valid[P8x8])
      {
       cost8x8 = 0;
        
        //===== store coding state of macroblock =====
        store_coding_state (cs_mb);
        
        //=====  LOOP OVER 8x8 SUB-PARTITIONS  (Motion Estimation & Mode Decision) =====
		//block=0-3, 遍历4个8x8块
        for (cbp8x8=cbp_blk8x8=cnt_nonz_8x8=0, block=0; block<4; block++)
        {
          //--- set coordinates ---
		  //当前8x8块在宏块中的位置.
		  //i0 j0 像素单位， i1 j1 4x4子块单位
          j0 = ((block/2)<<3);    j1 = (j0>>2);
          i0 = ((block%2)<<3);    i1 = (i0>>2);
          
          //=====  LOOP OVER POSSIBLE CODING MODES FOR 8x8 SUB-PARTITION  =====
		  //8x8 8x4 4x8 4x4 4种模式遍历，选择当前8x8亚宏块的best_mode
		  //这里面使用代价，主要就是为了当前亚宏块选择最佳的模式
          for (min_cost8x8=(1<<20), min_rdcost=1e30, index=(bframe?0:1); index<5; index++)
          {
            if (valid[mode=b8_mode_table[index]])
            {
              curr_cbp_blk = 0;
              
			  
	 //---------------------------------------------------------------------------------------------------------------------------------------------------
	 //下面进行了指定亚宏块，指定模式的运动矢量搜索以及参考帧选择-----------------------------------------------------------------------------------------
              if (mode==0)
              {
                //--- Direct Mode ---
                if (!input->rdopt)
                {
                  cost_direct += (cost = Get_Direct_Cost8x8 ( block, lambda_mode ));
                  if (cost==1<<30)
                    cost_direct = (1<<30);
                  have_direct ++;
                }
                best_fw_ref = direct_ref_idx[LIST_0][img->block_x+(block&1)*2][img->block_y+(block&2)];
                best_bw_ref = direct_ref_idx[LIST_1][img->block_x+(block&1)*2][img->block_y+(block&2)];
                best_pdir   = direct_pdir[img->block_x+(block&1)*2][img->block_y+(block&2)];
              } // if (mode==0)
              else
              {
                //--- motion estimation for all reference frames ---
				//计算出指定分割模式，指定8x8亚宏块中的所有分块的所有运动矢量
				//比如当前分割模式4x2, 则在该函数计算出指定8x8块中，两个8x4分割模式的所有运动矢量以及参考帧(使用的是相同的参考帧)
				//也就是说，该函数的最小计算的单位是8x8.
				//该函数计算了当前宏块，指定亚宏块，指定分割模式的所有运动向量img->all_mv，以及预测向量img->pred_mv
                PartitionMotionSearch (mode, block, lambda_motion);
                
                //--- get cost and reference frame for forward prediction ---
				// 遍历所有参考帧，选择指定8x8亚宏块 指定模式的 最佳参考帧
				// 注意这里，一个16x16宏块，里面的不同搜索块(16x8 8x16 8x8)可以使用不同的参考帧！
				// 一个8x8亚宏块, 里面不同的搜索块(8x4,4x8,4x4)只能使用的是同一个参考帧！
                for (fw_mcost=max_mcost, ref=0; ref<listXsize[LIST_0+list_offset]; ref++)
                {
                  if (!checkref || ref==0 || CheckReliabilityOfRef (block, LIST_0, ref, mode))
                  {
                    mcost  = (input->rdopt ? REF_COST(lambda_motion_factor,ref,LIST_0+list_offset) : (int)(2*lambda_motion*min(ref,1)));
                    
                    mcost += motion_cost[mode][LIST_0][ref][block];
					//记录指定亚宏块，指定搜索块模式的最佳参考帧
                    if (mcost < fw_mcost)
                    {
                      fw_mcost    = mcost;
                      best_fw_ref = ref;
                    }
                  }
                }
                
                //store forward reference index for every block
				//保存最佳参考帧
                for (j=0; j<2; j++)
                  for (i=0; i<2; i++)
                  {
                    enc_picture->ref_idx[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = best_fw_ref;
                    enc_picture->ref_pic_id [LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = enc_picture->ref_pic_num[LIST_0 + list_offset][enc_picture->ref_idx[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j]];
                  }

                if (bframe)
                {
                  for (bw_mcost=max_mcost, ref=0; ref<listXsize[LIST_1+list_offset]; ref++)
                  {
                    mcost  = (input->rdopt ? REF_COST(lambda_motion_factor,ref,LIST_1+list_offset) : (int)(2*lambda_motion*min(ref,1)));
                    
                    mcost += motion_cost[mode][LIST_1][ref][block];
                    if (mcost < bw_mcost)
                    {
                      bw_mcost    = mcost;
                      best_bw_ref = ref;
                    }
                  }
                  
                  // bidirectional uses best forward and zero backward reference
                  bid_mcost  = (input->rdopt ? (REF_COST (lambda_motion_factor, best_fw_ref, LIST_0 + list_offset)+REF_COST (lambda_motion_factor, 0, LIST_1 + list_offset)) : (int)(2*lambda_motion*min(best_fw_ref,1)));
                  bid_mcost += BIDPartitionCost (mode, block, best_fw_ref, 0, lambda_motion_factor );
                  
                  //--- get prediction direction ----
                  if      (fw_mcost<=bw_mcost && fw_mcost<=bid_mcost)
                  {
                    best_pdir = 0;
                    cost = fw_mcost;
                    best_bw_ref = -1;
                  }
                  else if (bw_mcost<=fw_mcost && bw_mcost<=bid_mcost)
                  {
                    best_pdir = 1;
                    cost = bw_mcost;
                    best_fw_ref = -1;
                  }
                  else
                  {
                    best_pdir = 2;
                    cost = bid_mcost;
                    best_bw_ref = 0;
                  }
                    //store backward reference index for every block
                  for (j=0; j<2; j++)
                    for (i=0; i<2; i++)
                    {
                      enc_picture->ref_idx[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = best_fw_ref;
                      enc_picture->ref_idx[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = best_bw_ref;
                      //enc_picture->ref_pic_id [LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = enc_picture->ref_pic_num[LIST_1 + list_offset][enc_picture->ref_idx[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j]];  
                    }
                } // if (bframe)
                else
                {
                  best_pdir = 0;
				  //指定8x8亚宏块中，指定mode（8x8,8x4,4x8,4x4）的最小误差
				  //这里不采用累积，是因为两个8x4 4x8 四个4x4的子块误差，在PartitionMotionSearch中已经进行了累积
                  cost      = fw_mcost;
                }
              } // if (mode!=0)
     //上面进行了指定亚宏块，指定模式的运动矢量搜索以及参考帧选择-----------------------------------------------------------------------------------------
	 //---------------------------------------------------------------------------------------------------------------------------------------------------
         
			  //--- store coding state before coding with current mode ---
              store_coding_state (cs_cm);
              
			  // 指定亚宏块，指定模式，使用最佳参考帧，进行rdcost或cost计算
			  //	 rdo: J = ssd + lambda*BLOCKbits
			  // non rdo: J = 
              if (input->rdopt)
              {
                //--- get and check rate-distortion cost ---
				/*******************************************************************************************************************************
				由于在函数PartitionMotionSearch函数中进行运动搜索时已经计算出了运动矢量mv保存在了img->all_mv中,所以这个函数里面，就可以根据mv
				直接找到预测值,进而在函数LumaPrediction4x4中完成预测值计算,保存在img->mpr中,然后在函数LumaResidualCoding8x8中计算得到残差,保
				存在img->m7中接着再dct_luma函数中完成DCT变换,量化, 反DCT,反量化,然后将得到的重建值存在enc_picture->imgY中同时也计算出了coef_cost, 
				这个coef_cost作为RDCost_for_8x8blocks函数中的cnt_nonz
				*********************************************************************************************************************************/
				//RDCost_for_8x8blocks只是在RDO方式下为一个8x8块在4种亚宏块模式中一种计算其代价，并和之前的比较，选择最佳模式,并且不考虑色度
				//该函数中，对指定亚宏块，指定模式，指定参考帧，进行了像素预测(img->mpr)，预测误差变换，以及重建(enc_picture->imgY)
				//该rdcost用于选择当前8x8亚宏块的最佳预测模式
                rdcost = RDCost_for_8x8blocks (&cnt_nonz, &curr_cbp_blk, lambda_mode,
                                               block, mode, best_pdir, best_fw_ref, best_bw_ref);
              }
              else
              {
                cost += (REF_COST (lambda_motion_factor, B8Mode2Value (mode, best_pdir), list_offset + (best_pdir<1?0:1)) - 1);
              }
              
              //--- set variables if best mode has changed ---
			  //记录最优rdcost或cost所对应的最佳模式，并且保存相关参数
			  // 记录的是当前8x8亚宏块的最佳分割模式以及参考帧
              if (( input->rdopt && rdcost < min_rdcost) ||
                  (!input->rdopt && cost   < min_cost8x8  )   )
              {
                min_cost8x8                  = cost;					//记录指定8x8亚宏块，最佳模式的cost
                min_rdcost                   = rdcost;					//记录指定8x8亚宏块，最佳模式的rdcost
                best8x8mode          [block] = mode;					//记录指定8x8亚宏块, 最佳模式mode
                best8x8pdir    [P8x8][block] = best_pdir;				//记录指定8x8亚宏块，最佳模式的pdir
                best8x8fwref   [P8x8][block] = best_fw_ref;				//记录指定8x8亚宏块, 最佳模式的fw_ref
                best8x8bwref   [P8x8][block] = best_bw_ref;				//记录指定8x8亚宏块，最佳模式的bw_ref

                
                //--- store number of nonzero coefficients ---
                best_cnt_nonz  = cnt_nonz;								//记录指定8x8亚宏块, 最佳模式下的非零系数个数
                
                if (input->rdopt)
                {
                  //--- store block cbp ---
                  cbp_blk8x8    &= (~(0x33 << (((block>>1)<<3)+((block%2)<<1)))); // delete bits for block
                  cbp_blk8x8    |= curr_cbp_blk;
                  
                  //--- store coefficients ---
                  for (k=0; k< 4; k++)
                    for (j=0; j< 2; j++)
                      for (i=0; i<65; i++)  cofAC8x8[block][k][j][i] = img->cofAC[block][k][j][i]; // 18->65 for ABT
                      
                      //--- store reconstruction and prediction ---
                  for (j=j0; j<j0+8; j++)
                    for (i=i0; i<i0+8; i++)
                    {
                      rec_mbY8x8[j][i] = enc_picture->imgY[img->pix_y+j][img->pix_x+i];
                      mpr8x8    [j][i] = img->mpr[i][j];
                    }
                }
                
                //--- store coding state ---
                store_coding_state (cs_b8);
              } // if (rdcost <= min_rdcost)
              
              //--- re-set coding state as it was before coding with current mode was performed ---
              reset_coding_state (cs_cm);
            } // if (valid[mode=b8_mode_table[index]])
          } // for (min_rdcost=1e30, index=(bframe?0:1); index<6; index++)
          
		  //cost8x8也是非rdo用的代价
		  //min_cost8x8 保存的是8x8亚宏块的最小代价
		  //这里，通过cost8x8计算该宏块的总代价
          cost8x8 += min_cost8x8;

//          if (!input->rdopt) cost8x8+= min_cost8x8;
//          else cost8x8 += min_rdcost;          
          
          if (!input->rdopt)
          {
            mode = best8x8mode[block];
            pdir = best8x8pdir[P8x8][block];
            
            curr_cbp_blk  = 0;
            best_cnt_nonz = LumaResidualCoding8x8 (&dummy, &curr_cbp_blk, block, pdir,
                                                   (pdir==0||pdir==2?mode:0),
                                                   (pdir==1||pdir==2?mode:0),
                                                   (best8x8fwref[P8x8][block]),
                                                   (best8x8bwref[P8x8][block]));
            cbp_blk8x8   &= (~(0x33 << (((block>>1)<<3)+((block%2)<<1)))); // delete bits for block
            cbp_blk8x8   |= curr_cbp_blk;
            
            //--- store coefficients ---
            for (k=0; k< 4; k++)
              for (j=0; j< 2; j++)
                for (i=0; i<65; i++)  cofAC8x8[block][k][j][i] = img->cofAC[block][k][j][i]; // 18->65 for ABT
                
                //--- store reconstruction and prediction ---
            for (j=j0; j<j0+8; j++)
              for (i=i0; i<i0+8; i++)
              {
                rec_mbY8x8[j][i] = enc_picture->imgY[img->pix_y+j][img->pix_x+i];
                mpr8x8    [j][i] = img->mpr[i][j];
              }
          }
          
          //----- set cbp and count of nonzero coefficients ---
          if (best_cnt_nonz)
          {
            cbp8x8        |= (1<<block);
            cnt_nonz_8x8  += best_cnt_nonz;
          }
          
          mode=best8x8mode[block];
          
	//===== reset intra prediction modes (needed for prediction, must be stored after 8x8 mode dec.) =====
		  // i0 j0是当前宏块，指定8x8块的左上角坐标
          j0 = img->block_y+2*(block/2);
          i0 = img->block_x+2*(block%2);
		  //遍历当前8x8块的4个4x4块，设置初试预测模式DC_PRED
          for (j=j0; j<j0+2; j++)
            for (i=i0; i<i0+2; i++) 
              ipredmodes[i][j]         = DC_PRED;
          // 设置当前8x8块的4个4x4块的intra_pred_modes，这个是使用帧内4x4时，传输的值
		  i0 = 4*block;
          for (i=i0; i<i0+4; i++)    currMB->intra_pred_modes[i]  = DC_PRED;
            
          if (block<3)
          {
            //===== re-set reconstructed block =====
            j0   = 8*(block/2);
            i0   = 8*(block%2);
            for (j=j0; j<j0+8; j++)
              for (i=i0; i<i0+8; i++)  
                enc_picture->imgY[img->pix_y+j][img->pix_x+i] = rec_mbY8x8[j][i];
          } // if (block<3)
            
          //===== set motion vectors and reference frames (prediction) =====
		  //和宏块级一样，设置参考帧和运动矢量
          SetRefAndMotionVectors (block, mode, best8x8pdir[P8x8][block], best8x8fwref[P8x8][block], best8x8bwref[P8x8][block]);
          //===== set the coding state after current block =====
          reset_coding_state (cs_b8);
        } // for (cbp8x8=cbp_blk8x8=cnt_nonz_8x8=0, block=0; block<4; block++)
        
        //===== store intra prediction modes for 8x8+ macroblock mode =====
        for (k=0, j=img->block_y; j<img->block_y+4; j++)
        {
          for (     i=img->block_x; i<img->block_x+4; i++, k++)
          {
            b8_ipredmode       [k] = ipredmodes    [i][j];
            b8_intra_pred_modes[k] = currMB->intra_pred_modes[k];
          }
        }
        
        //--- re-set coding state (as it was before 8x8 block coding) ---
        reset_coding_state (cs_mb);
        for (i=0; i<16; i++)
          for(j=0; j<16; j++)
            diffy[j][i] = imgY_org[img->opix_y+j][img->opix_x+i]-img->mpr[j][i]; 

		// 记录8x8模式是否为最佳模式
        if(cost8x8 < min_cost)
        {
           best_mode = P8x8;
           min_cost = cost8x8;
        }
      }
	  // 如果不能使能8x8模式，则对应的cost8x8为最大值
      else // if (valid[P8x8])
      {
        cost8x8 = (1<<20);
      }
// 以上是帧间预测8x8模式，8x8还会继续分为亚宏块模式****************************************************************************************************
// ****************************************************************************************************************************************************

      // Find a motion vector for the Skip mode
      if((img->type == P_SLICE)||(img->type == SP_SLICE))
        FindSkipModeMotionVector ();
    }
	else // if (!intra)
    {
      min_cost = (1<<20);
    }
//=================================================================================================================================================
//==================================================================上面为帧间编码=================================================================
//=================================================================================================================================================
	
	//rdopt 根据rdo代价进行选择
	if (input->rdopt)
    {
      int mb_available_up;
      int mb_available_left;
      int mb_available_up_left;
      
      min_rdcost = max_rdcost;
      
      // precompute all new chroma intra prediction modes
      // 对4种色度模式，进行RDO计算.(每个色度模式，分别进行亮度的各种模式计算)
	  // 在后面的循环中，亮度和色度一同计算cost，选择最佳最佳的模式
	  // 色预测数据保存在img->mprr_c[uv][chroma_mode][i][j]中
	  IntraChromaPrediction8x8(&mb_available_up, &mb_available_left, &mb_available_up_left);
      
	  //当前色度预测模式缓存在currMB->c_ipred_mode中
      for (currMB->c_ipred_mode=DC_PRED_8; currMB->c_ipred_mode<=PLANE_8; currMB->c_ipred_mode++)
      {
        
        // bypass if c_ipred_mode is not allowed
		  // 如果当前c_ipred_mode不被允许，则考虑下一个c_ipred_mode
        if ((currMB->c_ipred_mode==VERT_PRED_8 && !mb_available_up) ||
          (currMB->c_ipred_mode==HOR_PRED_8 && !mb_available_left) ||
          (currMB->c_ipred_mode==PLANE_8 && (!mb_available_left || !mb_available_up || !mb_available_up_left)))
          continue;
        
        
        //===== GET BEST MACROBLOCK MODE =====
		// 获得宏块最佳模式
        for (ctr16x16=0, index=0; index<7; index++)
        {
          mode = mb_mode_table[index];									//从8种模式中(0, 1, 2, 3, P8x8, I16MB, I4MB)，选择当前索引的模式
          
          //--- for INTER16x16 check all prediction directions ---
		  //帧间16x16且为双向预测片
          if (mode==1 && img->type==B_SLICE)
          {
            best8x8pdir[1][0] = best8x8pdir[1][1] = best8x8pdir[1][2] = best8x8pdir[1][3] = ctr16x16;
            if (ctr16x16 < 2) index--;
            ctr16x16++;
          }
          
          img->NoResidueDirect = 0;
          

		  //判断当前模式mode是否使能
          if (valid[mode])
          {
            // bypass if c_ipred_mode not used
            //该函数在下面的RDCost_for_macroblocks 函数内再次调用，进行了重复操作
            //这里设置了当前宏块的8x8分割方式和预测方向，以及每个4x4块的参考帧索引，在RDCost_for_macroblock的帧间预测时中将会用到
			//指定分割模式mode的所有8x8块的参考帧保存在best8x8fwref中,将其读出保存在enc_picture->ref_idx中
			//enc_picture->ref_idx中是按4x4块为单位存储参考帧的
			SetModesAndRefframeForBlocks (mode);

			//色度的帧内预测模式为DC，或者当前宏块是帧内预测宏块，则向下执行
			//利用这个条件限制来实现inter 模式时只计算一次RDO.因为当inter模式时，只有在色度块为DC预测模式时，才能进来计算RDO
            if (currMB->c_ipred_mode == DC_PRED_8 ||
              (IS_INTRA(currMB) ))
            {
			  // 在0-3, P8x8, Intra4x4, Intera16x16之间，根据rdo选择最佳的模式.(前面已经选出了rdo或者非rdo模式下，0-3的最佳模式，这里要重新选择)
			  // RDCost_for_macroblocks，若当前指定模式mode的率失真>min_rdocst, 则返回0, 否则返回1
			  // RDCost_for_macroblocks是为一整个宏块(16x16)在(0,1-3,P8x8,I4MB,I16MB)计算其RDO代价,然后和之前的比较选择一个最优模式,并且是考虑色度块的
			  // 这里P8x8的4中亚宏块模式，已经在前面中计算好了最佳的亚宏块模式.
			  // 比较的模式是：p16x16, p16x8, p8x16, 最佳p8x8, 最佳i4mb, 最佳i16mb，在其中选择最佳
              if (RDCost_for_macroblocks (lambda_mode, mode, &min_rdcost))
			  // 该函数存在重复计算的情况，因为色度选择和亮度选择无关，每次选择色度模式的时候，都会重新进行亮度选择，且每次亮度选择都是一样滴
              {
					//Rate control
					//计算预测误差
					if(mode == P8x8){
					  for (i=0; i<16; i++)
						for(j=0; j<16; j++)
						  diffy[j][i] = imgY_org[img->opix_y+j][img->opix_x+i] - mpr8x8[j][i];
					}
					else{
					  for (i=0; i<16; i++)
						for(j=0; j<16; j++)
						  diffy[j][i] = imgY_org[img->opix_y+j][img->opix_x+i] - pred[j][i];
					}
					//有更好的编码模式时，便保存宏块信息
					//包括最佳预测模式，重建像素rec_mbY/U/V(来自enc_picture)，预测误差变换系数，cbp，参考帧等
					store_macroblock_parameters (mode);
              }
            }
          }

		  // 
          if (valid[0] && bframe && mode == 0 && currMB->cbp && (currMB->cbp&15) != 15) //g050
          {
            img->NoResidueDirect = 1;
            if (RDCost_for_macroblocks (lambda_mode, mode, &min_rdcost))
            {
              //Rate control
              for (i=0; i<16; i++)
                for(j=0; j<16; j++)
                  diffy[j][i] = imgY_org[img->opix_y+j][img->opix_x+i] - pred[j][i];
                store_macroblock_parameters (mode);
            }
          }
        }
      }
    }
    else //if (input->rdopt) 下面执行的是非rdo模式的计算
    {
      if (valid[0] && bframe) // check DIRECT MODE
      {
        cost  = (have_direct?cost_direct:Get_Direct_CostMB (lambda_mode));
        cost -= (int)floor(16*lambda_motion+0.4999);
        if (cost <= min_cost)
        {
          //Rate control
          for (i=0; i<16; i++)
            for(j=0; j<16; j++)
              diffy[j][i] = imgY_org[img->pix_y+j][img->pix_x+i]-img->mpr[i][j];
            
            min_cost  = cost;
            best_mode = 0;
        }
      }
      if (valid[I4MB]) // check INTRA4x4
      {
		//这里面计算rdocast时，采用的是non rdopt方式
		// non rdopt = sad + 4*lambda_mode
        currMB->cbp = Mode_Decision_for_Intra4x4Macroblock (lambda_mode, &cost);
        if (cost <= min_cost)
        {
          //Rate control
          for (i=0; i<16; i++)
            for(j=0; j<16; j++)
              diffy[j][i] = imgY_org[img->pix_y+j][img->pix_x+i]-img->mpr[i][j];
            
            min_cost  = cost;
            best_mode = I4MB;
        }
      }
      if (valid[I16MB]) // check INTRA16x16
      {
		//调用intrapred_luma_16x16函数,计算I16MB的4种模式下的预测值,保存在img->mprr_2中
		//16x16的帧内预测模式，注意non rdo是将sad作为最终代价的，即便在rdo中，也是使用sad作为distortion，而非使用ssd
		//并且需要注意，sad使用的是实际值和预测值的误差，ssd使用的是实际值和重建值之间的误差
        intrapred_luma_16x16 ();
        cost = find_sad_16x16 (&i16mode);
        if (cost < min_cost)
        {
          //Rate control
          for (i=0; i<16; i++)
            for(j=0; j<16; j++)
              diffy[j][i] = imgY_org[img->pix_y+j][img->pix_x+i]-img->mprr_2[i16mode][j][i];
            
            best_mode   = I16MB;
            currMB->cbp = dct_luma_16x16 (i16mode);
        }
      }
    }
    
    if (rerun==0)
    {
      intra1 = (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
    }
  } // for (rerun=0; rerun<runs; rerun++)

//******************************************************帧内和帧间完成****************************************************************************
//************************************************************************************************************************************************
//************************************************************************************************************************************************

 //+++下面的要进行存储编码参数,分RDO和非RDO

//如果用RDO代价函数,因为计算代价时已经编码,择优后仅需对编码进行存储;
  if (input->rdopt)
  {
    
    if ((cbp!=0 || best_mode==I16MB ))			
      currMB->prev_cbp = 1;
    else if (cbp==0 && !input->RCEnable)
    {
      currMB->delta_qp = 0;
      currMB->qp = currMB->prev_qp;
      img->qp = currMB->qp;
      currMB->prev_cbp = 0;
    }

	//+++存储编码参数
    set_stored_macroblock_parameters ();
	//下面很多内容，都在enc_picture中进行赋值
	//+++    (1)保存重建图像									(3)宏块模式
	//+++    enc_picture->imgY[j][i] = rec_mbY[j][i];			currMB->mb_type = mode(best_mode);
	//+++    enc_picture->imgUV[j][i][0] = rec_mbU[j][i];		(4)参考帧保存
	//+++    enc_picture->imgUV[j][i][1] = rec_mbV[j][i];		rdopt.c 1511行,对前向参考,后向参考帧分别进行了保存
	//+++    (2)变换系数和cbp									(5)帧内预测模式
	//+++    img->cofAC=i4p(cofAC);								currMB->c_ipred_mode = best_c_imode;
	//+++    img->cofDC=i3p(cofDC);								(6)为当前块保存运动向量
	//+++    currMB->cbp      = cbp;							SetMotionVectorsMB (currMB, bframe);
	//+++    currMB->cbp_blk = cbp_blk;							(7)保存帧间8x8块的分割模式和预测方向(b8mode)
	//+++    从上面其实可以看到"="右边的量在rdopt.c中都是全局变量,在完成encode_one_macroblock之前,对这些参数进行一下保存.
  }
//如果用非RDO模式，则择优后还要进行编码过程,编码后存储参数.
//这是因为，RDO模式自身就已经进行了编码了（RDO的cost计算，就已经用到了编码了）
  else// if (input->rdopt)
  {
    //===== set parameters for chosen mode =====
    SetModesAndRefframeForBlocks (best_mode);
    if (best_mode==P8x8)
    {
      SetCoeffAndReconstruction8x8 (currMB);
    }
    else
    {
      if (best_mode!=I4MB)
      {
        for (k=0, j=img->block_y; j<img->block_y+4; j++)
          for (     i=img->block_x; i<img->block_x+4; i++, k++)
          {
            ipredmodes    [i][j] = DC_PRED;
            currMB->intra_pred_modes[k] = DC_PRED;
          }
          if (best_mode!=I16MB)
          {
            LumaResidualCoding ();
            //Rate control
            for (i=0; i<16; i++)
              for(j=0; j<16; j++)
                diffy[j][i] = imgY_org[img->pix_y+j][img->pix_x+i]-img->mpr[i][j];
          }
      }
    }
    // precompute all chroma intra prediction modes
    IntraChromaPrediction8x8(NULL, NULL, NULL);
    img->i16offset = 0;
    dummy = 0;
    ChromaResidualCoding (&dummy);
    if (best_mode==I16MB)
    {
      img->i16offset = I16Offset  (currMB->cbp, i16mode);
    }
    SetMotionVectorsMB (currMB, bframe);
    
    //===== check for SKIP mode =====
    if ((img->type==P_SLICE || img->type==SP_SLICE) && best_mode==1 && currMB->cbp==0 &&
      enc_picture->ref_idx[LIST_0][img->block_x][img->block_y]==0 &&
      enc_picture->mv[LIST_0][img->block_x][img->block_y][0]==allmvs[0][0][0][0][0][0] &&
      enc_picture->mv[LIST_0][img->block_x][img->block_y][1]==allmvs[0][0][0][0][0][1]               )
    {
      currMB->mb_type=currMB->b8mode[0]=currMB->b8mode[1]=currMB->b8mode[2]=currMB->b8mode[3]=0;
    }
    
    if(img->MbaffFrameFlag)
      set_mbaff_parameters();
  }
  
  // Rate control
  if(input->RCEnable)
  {   
    if(img->type==P_SLICE)
    {
      img->MADofMB[img->current_mb_nr] = calc_MAD();
      
      if(input->basicunit<img->Frame_Total_Number_MB)
      {
        img->TotalMADBasicUnit +=img->MADofMB[img->current_mb_nr];
        
        /* delta_qp is present only for non-skipped macroblocks*/
        if ((cbp!=0 || best_mode==I16MB))
          currMB->prev_cbp = 1;
        else
        {
          img->qp -= currMB->delta_qp;
          currMB->delta_qp = 0;
          currMB->qp = img->qp;
          currMB->prev_cbp = 0;
        }
        /* When MBAFF is used, delta_qp is only present for the first non-skipped macroblock of each 
        macroblock pair*/
        if (input->MbInterlace)
        {
          if(!currMB->mb_field)
          {
            DELTA_QP = currMB->delta_qp;
            QP      = currMB->qp;
          }
          else
          {
            DELTA_QP2 = currMB->delta_qp;
            QP2      = currMB->qp;
          }
        }       
      }
    }
  }
  
    
  if(input->rdopt)
    rdopt->min_rdcost = min_rdcost;
  else
    rdopt->min_rdcost = min_cost; 

  if(img->MbaffFrameFlag)
  {
    if (img->current_mb_nr%2) //bottom
    {
      if ((currMB->mb_type ? 0:((img->type == B_SLICE) ? !currMB->cbp:1))  // bottom is skip
        &&(prevMB->mb_type ? 0:((img->type == B_SLICE) ? !prevMB->cbp:1))) // top is skip
      {
        if (!(field_flag_inference() == curr_mb_field))
        {
          rdopt->min_rdcost = 1e30;  // don't allow coding of an MB pair as skip if wrong inference
        }
      }
    }
  }
  
  //===== Decide if this MB will restrict the reference frames =====
  if (input->RestrictRef==1)
  {
    if (input->rdopt==1)
    {
      refresh_map[2*img->mb_y  ][2*img->mb_x  ] = (intra ? 1 : 0);
      refresh_map[2*img->mb_y  ][2*img->mb_x+1] = (intra ? 1 : 0);
      refresh_map[2*img->mb_y+1][2*img->mb_x  ] = (intra ? 1 : 0);
      refresh_map[2*img->mb_y+1][2*img->mb_x+1] = (intra ? 1 : 0);
    }
    else if (input->rdopt==2)
    {
      refresh_map[2*img->mb_y  ][2*img->mb_x  ] = (intra1==0 && (currMB->mb_type==I16MB || currMB->mb_type==I4MB) ? 1 : 0);
      refresh_map[2*img->mb_y  ][2*img->mb_x+1] = (intra1==0 && (currMB->mb_type==I16MB || currMB->mb_type==I4MB) ? 1 : 0);
      refresh_map[2*img->mb_y+1][2*img->mb_x  ] = (intra1==0 && (currMB->mb_type==I16MB || currMB->mb_type==I4MB) ? 1 : 0);
      refresh_map[2*img->mb_y+1][2*img->mb_x+1] = (intra1==0 && (currMB->mb_type==I16MB || currMB->mb_type==I4MB) ? 1 : 0);
    }
  }
  else if (input->RestrictRef==2)
  {
    refresh_map[2*img->mb_y  ][2*img->mb_x  ] = (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
    refresh_map[2*img->mb_y  ][2*img->mb_x+1] = (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
    refresh_map[2*img->mb_y+1][2*img->mb_x  ] = (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
    refresh_map[2*img->mb_y+1][2*img->mb_x+1] = (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
  }
  
  if(input->FMEnable)
    skip_intrabk_SAD(best_mode, listXsize[LIST_0+list_offset]);
}


void set_mbaff_parameters()
{
  int  i, j, k, l;
  Macroblock  *currMB  = &img->mb_data[img->current_mb_nr];
  int         mode     = best_mode;
  int         bframe   = (img->type==B_SLICE);
  int     **ipredmodes = img->ipredmode;

  if (!img->MbaffFrameFlag)
    return;

  //===== reconstruction values =====
  for (j=0; j<16; j++)
    for (i=0; i<16; i++)
      rdopt->rec_mbY[j][i]           = enc_picture->imgY[img->pix_y+j][img->pix_x+i]; 

  for (j=0; j<8; j++)
    for (i=0; i<8; i++)
    {
      rdopt->rec_mbU[j][i]           = enc_picture->imgUV[0][img->pix_c_y+j][img->pix_c_x+i];   
      rdopt->rec_mbV[j][i]           = enc_picture->imgUV[1][img->pix_c_y+j][img->pix_c_x+i];    
    }

  //===== coefficients and cbp =====
  rdopt->mode = mode;
  rdopt->i16offset = img->i16offset;  // For MBINTLC  -Rajeev
  rdopt->cbp = currMB->cbp;
  rdopt->cbp_blk = currMB->cbp_blk;
  rdopt->mb_type  =currMB->mb_type;

  if(rdopt->mb_type == 0 && mode != 0)
  {
    mode=0;
    rdopt->mode=0;
  }

  for(i=0;i<6;i++)
    for(j=0;j<4;j++)
      for(k=0;k<2;k++)
        for(l=0;l<18;l++)
          rdopt->cofAC[i][j][k][l] = img->cofAC[i][j][k][l];

  for(i=0;i<3;i++)
    for(k=0;k<2;k++)
      for(l=0;l<18;l++)
        rdopt->cofDC[i][k][l] = img->cofDC[i][k][l];


  for (i=0; i<4; i++)
  {
    rdopt->b8mode[i]  = currMB->b8mode[i];                  
    rdopt->b8pdir[i]  = currMB->b8pdir[i];                  
  }

  //==== reference frames =====
  for (j=0; j<4; j++)
    for (i=0; i<4; i++)
    {
      rdopt->refar[LIST_0][j][i]       = enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j];
    }

  if (bframe)
  {
    for (j=0; j<4; j++)
      for (i=0; i<4; i++)
      {
        rdopt->refar[LIST_1][j][i]     = enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j];
      }
  }


  for   (k=0, j=img->block_y; j<img->block_y+4; j++)
    for (     i=img->block_x; i<img->block_x+4; i++, k++)
    {
      rdopt->ipredmode[i][j]     = ipredmodes[i][j];
      rdopt->intra_pred_modes[k] = currMB->intra_pred_modes[k];
    }
}
