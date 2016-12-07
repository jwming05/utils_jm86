
/*!
 **************************************************************************************
 * \file
 *    nal.c
 * \brief
 *    Handles the operations on converting String of Data Bits (SODB)
 *    to Raw Byte Sequence Payload (RBSP), and then 
 *    onto Encapsulate Byte Sequence Payload (EBSP).
 *  \date 14 June 2002
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details) 
 *      - Shankar Regunathan                  <shanre@microsoft.de>
 *      - Stephan Wenger                      <stewe@cs.tu-berlin.de>
 ***************************************************************************************
 */


#include "contributors.h"

#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "defines.h"
#include "global.h"

 /*!
 ************************************************************************
 * \brief
 *    Converts String Of Data Bits (SODB) to Raw Byte Sequence 
 *    Packet (RBSP)
 * \param currStream
 *        Bitstream which contains data bits.
 * \return None
 * \note currStream is byte-aligned at the end of this function
 *    
 ************************************************************************
*/

static byte *NAL_Payload_buffer;

//SODBtoRBSP是将SODB转换为RBSP数据，目的是字节对齐(因为SODB的比特数可能不是8的倍数)。转换的方法是：
//在已有的比特数据后面，添加首先填充的比特数据是1，后面全是0比特
//byte_buf, 当前正在写尔的字节数据，这个数据并没有字节对其，bits_to_go指明了该数据还有多少bit没有写入
//bits_to_go, 指明了当前正在写入的字节数据，还有多少bit没有写入
//byte_pos, 指明当前正在写的字节数据的位置
void SODBtoRBSP(Bitstream *currStream)
{
  currStream->byte_buf <<= 1;
  currStream->byte_buf |= 1;
  currStream->bits_to_go--;
  currStream->byte_buf <<= currStream->bits_to_go;
  currStream->streamBuffer[currStream->byte_pos++] = currStream->byte_buf;
  currStream->bits_to_go = 8;
  currStream->byte_buf = 0;
}


/*!
************************************************************************
*  \brief
*     This function converts a RBSP payload to an EBSP payload
*     
*  \param streamBuffer
*       pointer to data bits
*  \param begin_bytepos
*            The byte position after start-code, after which stuffing to
*            prevent start-code emulation begins.
*  \param end_bytepos
*           Size of streamBuffer in bytes.
*  \param min_num_bytes
*           Minimum number of bytes in payload. Should be 0 for VLC entropy
*           coding mode. Determines number of stuffed words for CABAC mode.
*  \return 
*           Size of streamBuffer after stuffing.
*  \note
*      NAL_Payload_buffer is used as temporary buffer to store data.
*
*
************************************************************************
*/

//EBSP　扩展字节序列载荷
//为了使NALU主体中不包括与开始码相冲突的，在编码时，每遇到两个字节连续为0，就插入一个字节的0x03。解码时将0x03去掉。也称为脱壳操作。
int RBSPtoEBSP(byte *streamBuffer, int begin_bytepos, int end_bytepos, int min_num_bytes)
{
  
  int i, j, count;

  for(i = begin_bytepos; i < end_bytepos; i++)
    NAL_Payload_buffer[i] = streamBuffer[i];

  count = 0;
  j = begin_bytepos;
  for(i = begin_bytepos; i < end_bytepos; i++) 
  {
    if(count == ZEROBYTES_SHORTSTARTCODE && !(NAL_Payload_buffer[i] & 0xFC)) 
    {
      streamBuffer[j] = 0x03;
      j++;
      count = 0;   
    }
    streamBuffer[j] = NAL_Payload_buffer[i];
    if(NAL_Payload_buffer[i] == 0x00)      
      count++;
    else 
      count = 0;
    j++;
  }
  while (j < begin_bytepos+min_num_bytes) {
    streamBuffer[j] = 0x00; // cabac stuffing word
    streamBuffer[j+1] = 0x00;
    streamBuffer[j+2] = 0x03;
    j += 3;
    stat->bit_use_stuffingBits[img->type]+=16;
  }
  return j;
}

 /*!
 ************************************************************************
 * \brief
 *    Initializes NAL module (allocates NAL_Payload_buffer)
 ************************************************************************
*/

void AllocNalPayloadBuffer()
{
  //缓冲空间大小，宽*高，4倍是为了安全起见。
  const int buffer_size = (input->img_width * input->img_height * 4); // AH 190202: There can be data expansion with 
                                                          // low QP values. So, we make sure that buffer 
                                                          // does not everflow. 4 is probably safe multiplier.
  FreeNalPayloadBuffer();												//尝试释放nal缓冲

  NAL_Payload_buffer = (byte *) calloc(buffer_size, sizeof(byte));		//申请空间
  assert (NAL_Payload_buffer != NULL);									//断言
}


 /*!
 ************************************************************************
 * \brief
 *   Finits NAL module (frees NAL_Payload_buffer)
 ************************************************************************
*/

void FreeNalPayloadBuffer()
{
  if(NAL_Payload_buffer)
  {
    free(NAL_Payload_buffer);
    NAL_Payload_buffer=NULL;
  }
}
