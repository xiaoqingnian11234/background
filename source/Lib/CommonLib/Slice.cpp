/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     Slice.cpp
    \brief    slice header and SPS class
*/

#include "CommonDef.h"
#include "Unit.h"
#include "Slice.h"
#include "Picture.h"
#include "dtrace_next.h"
#include "fstream"
using namespace std;


//! \ingroup CommonLib
//! \{

Slice::Slice()
: m_iPPSId                        ( -1 )
, m_PicOutputFlag                 ( true )
, m_iPOC                          ( 0 )
, m_iLastIDR                      ( 0 )
, m_iAssociatedIRAP               ( 0 )
, m_iAssociatedIRAPType           ( NAL_UNIT_INVALID )
, m_pRPS                          ( 0 )
, m_localRPS                      ( )
, m_rpsIdx                        ( 0 )
, m_RefPicListModification        ( )
, m_eNalUnitType                  ( NAL_UNIT_CODED_SLICE_IDR_W_RADL )
, m_eSliceType                    ( I_SLICE )
, m_iSliceQp                      ( 0 )
#if HEVC_DEPENDENT_SLICES
, m_dependentSliceSegmentFlag     ( false )
#endif
, m_ChromaQpAdjEnabled            ( false )
, m_deblockingFilterDisable       ( false )
, m_deblockingFilterOverrideFlag  ( false )
, m_deblockingFilterBetaOffsetDiv2( 0 )
, m_deblockingFilterTcOffsetDiv2  ( 0 )
, m_pendingRasInit                ( false )
, m_bCheckLDC                     ( false )
, m_iSliceQpDelta                 ( 0 )
, m_iDepth                        ( 0 )
#if HEVC_VPS
, m_pcVPS                         ( NULL )
#endif
, m_pcSPS                         ( NULL )
, m_pcPPS                         ( NULL )
, m_pcPic                         ( NULL )
, m_colFromL0Flag                 ( true )
, m_noOutputPriorPicsFlag         ( false )
, m_noRaslOutputFlag              ( false )
, m_handleCraAsBlaFlag            ( false )
, m_colRefIdx                     ( 0 )
, m_maxNumMergeCand               ( 0 )
, m_uiTLayer                      ( 0 )
, m_bTLayerSwitchingFlag          ( false )
, m_sliceMode                     ( NO_SLICES )
, m_sliceArgument                 ( 0 )
, m_sliceCurStartCtuTsAddr        ( 0 )
, m_sliceCurEndCtuTsAddr          ( 0 )
, m_independentSliceIdx           ( 0 )
#if HEVC_DEPENDENT_SLICES
, m_sliceSegmentIdx               ( 0 )
, m_sliceSegmentMode              ( NO_SLICES )
, m_sliceSegmentArgument          ( 0 )
, m_sliceSegmentCurStartCtuTsAddr ( 0 )
, m_sliceSegmentCurEndCtuTsAddr   ( 0 )
#endif
, m_nextSlice                     ( false )
#if HEVC_DEPENDENT_SLICES
, m_nextSliceSegment              ( false )
#endif
, m_sliceBits                     ( 0 )
#if HEVC_DEPENDENT_SLICES
, m_sliceSegmentBits              ( 0 )
#endif
, m_bFinalized                    ( false )
, m_bTestWeightPred               ( false )
, m_bTestWeightBiPred             ( false )
, m_substreamSizes                ( )
, m_cabacInitFlag                 ( false )
, m_cabacWinUpdateMode            ( 0 )
, m_bLMvdL1Zero                   ( false )
, m_temporalLayerNonReferenceFlag ( false )
, m_LFCrossSliceBoundaryFlag      ( false )
, m_enableTMVPFlag                ( true )
, m_encCABACTableIdx              (I_SLICE)
, m_iProcessingStartTime          ( 0 )
, m_dProcessingTime               ( 0 )
, m_uiMaxBTSize                   ( 0 )
{
  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i] = 0;
  }

  for (UInt component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_lambdas            [component] = 0.0;
    m_iSliceChromaQpDelta[component] = 0;
  }

  initEqualRef();

  for ( Int idx = 0; idx < MAX_NUM_REF; idx++ )
  {
    m_list1IdxToList0Idx[idx] = -1;
  }

  for(Int iNumCount = 0; iNumCount < MAX_NUM_REF; iNumCount++)
  {
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_apcRefPicList [i][iNumCount] = NULL;
      m_aiRefPOCList  [i][iNumCount] = 0;
    }
  }

  resetWpScaling();
  initWpAcDcParam();

  for(Int ch=0; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_saoEnabledFlag[ch] = false;
  }

}

Slice::~Slice()
{

}


Void Slice::initSlice()
{
  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i]      = 0;
  }
  m_colFromL0Flag = true;

  m_colRefIdx = 0;
  initEqualRef();

  m_bCheckLDC = false;

  for (UInt component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_iSliceChromaQpDelta[component] = 0;
  }

  m_maxNumMergeCand = MRG_MAX_NUM_CANDS;

  m_bFinalized=false;

  m_substreamSizes.clear();
  m_cabacInitFlag        = false;
  m_cabacWinUpdateMode   = 0;
  m_enableTMVPFlag       = true;

}

Void Slice::setDefaultClpRng( const SPS& sps )
{
  m_clpRngs.comp[COMPONENT_Y].min = m_clpRngs.comp[COMPONENT_Cb].min  = m_clpRngs.comp[COMPONENT_Cr].min = 0;
  m_clpRngs.comp[COMPONENT_Y].max                                     = (1<< sps.getBitDepth(CHANNEL_TYPE_LUMA))-1;
  m_clpRngs.comp[COMPONENT_Y].bd  = sps.getBitDepth(CHANNEL_TYPE_LUMA);
  m_clpRngs.comp[COMPONENT_Y].n   = 0;
  m_clpRngs.comp[COMPONENT_Cb].max = m_clpRngs.comp[COMPONENT_Cr].max = (1<< sps.getBitDepth(CHANNEL_TYPE_CHROMA))-1;
  m_clpRngs.comp[COMPONENT_Cb].bd  = m_clpRngs.comp[COMPONENT_Cr].bd  = sps.getBitDepth(CHANNEL_TYPE_CHROMA);
  m_clpRngs.comp[COMPONENT_Cb].n   = m_clpRngs.comp[COMPONENT_Cr].n   = 0;
  m_clpRngs.used = m_clpRngs.chroma = false;
}


Bool Slice::getRapPicFlag() const
{
  return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA;
}


Void  Slice::sortPicList        (PicList& rcListPic)
{
  Picture*    pcPicExtract;
  Picture*    pcPicInsert;

  PicList::iterator    iterPicExtract;
  PicList::iterator    iterPicExtract_1;
  PicList::iterator    iterPicInsert;

  for (Int i = 1; i < (Int)(rcListPic.size()); i++)
  {
    iterPicExtract = rcListPic.begin();
    for (Int j = 0; j < i; j++)
    {
      iterPicExtract++;
    }
    pcPicExtract = *(iterPicExtract);

    iterPicInsert = rcListPic.begin();
    while (iterPicInsert != iterPicExtract)
    {
      pcPicInsert = *(iterPicInsert);
      if (pcPicInsert->getPOC() >= pcPicExtract->getPOC())
      {
        break;
      }

      iterPicInsert++;
    }

    iterPicExtract_1 = iterPicExtract;    iterPicExtract_1++;

    //  swap iterPicExtract and iterPicInsert, iterPicExtract = curr. / iterPicInsert = insertion position
    rcListPic.insert( iterPicInsert, iterPicExtract, iterPicExtract_1 );
    rcListPic.erase( iterPicExtract );
  }
}

Picture* Slice::xGetRefPic (PicList& rcListPic, Int poc)
{
  PicList::iterator  iterPic = rcListPic.begin();
  Picture*           pcPic   = *(iterPic);

  while ( iterPic != rcListPic.end() )
  {
    if(pcPic->getPOC() == poc)
    {
      break;
    }
    iterPic++;

    pcPic = *(iterPic);
  }
  return  pcPic;
}


Picture* Slice::xGetLongTermRefPic( PicList& rcListPic, Int poc, Bool pocHasMsb)
{
  PicList::iterator  iterPic = rcListPic.begin();
  Picture*           pcPic   = *(iterPic);
  Picture*           pcStPic = pcPic;

  Int pocCycle = 1 << getSPS()->getBitsForPOC();
  if (!pocHasMsb)
  {
    poc = poc & (pocCycle - 1);
  }

  while ( iterPic != rcListPic.end() )
  {
    pcPic = *(iterPic);
    if (pcPic && pcPic->getPOC()!=this->getPOC() && pcPic->referenced)
    {
      Int picPoc = pcPic->getPOC();
      if (!pocHasMsb)
      {
        picPoc = picPoc & (pocCycle - 1);
      }

      if (poc == picPoc)
      {
        if(pcPic->longTerm)
        {
          return pcPic;
        }
        else
        {
          pcStPic = pcPic;
        }
        break;
      }
    }

    iterPic++;
  }

  return pcStPic;
}

Void Slice::setRefPOCList       ()
{
  for (Int iDir = 0; iDir < NUM_REF_PIC_LIST_01; iDir++)
  {
    for (Int iNumRefIdx = 0; iNumRefIdx < m_aiNumRefIdx[iDir]; iNumRefIdx++)
    {
      m_aiRefPOCList[iDir][iNumRefIdx] = m_apcRefPicList[iDir][iNumRefIdx]->getPOC();
    }
  }

}

Void Slice::setList1IdxToList0Idx()
{
  Int idxL0, idxL1;
  for ( idxL1 = 0; idxL1 < getNumRefIdx( REF_PIC_LIST_1 ); idxL1++ )
  {
    m_list1IdxToList0Idx[idxL1] = -1;
    for ( idxL0 = 0; idxL0 < getNumRefIdx( REF_PIC_LIST_0 ); idxL0++ )
    {
      if ( m_apcRefPicList[REF_PIC_LIST_0][idxL0]->getPOC() == m_apcRefPicList[REF_PIC_LIST_1][idxL1]->getPOC() )
      {
        m_list1IdxToList0Idx[idxL1] = idxL0;
        break;
      }
    }
  }
}

Void Slice::setRefPicList( PicList& rcListPic, Bool checkNumPocTotalCurr, Bool bCopyL0toL1ErrorCase )
{
  if ( m_eSliceType == I_SLICE)
  {
    ::memset( m_apcRefPicList, 0, sizeof (m_apcRefPicList));
    ::memset( m_aiNumRefIdx,   0, sizeof ( m_aiNumRefIdx ));

    if (!checkNumPocTotalCurr)
    {
      return;
    }
  }

  Picture*  pcRefPic= NULL;
  static const UInt MAX_NUM_NEGATIVE_PICTURES=16;
  Picture*  RefPicSetStCurr0[MAX_NUM_NEGATIVE_PICTURES];
  Picture*  RefPicSetStCurr1[MAX_NUM_NEGATIVE_PICTURES];
  Picture*  RefPicSetLtCurr[MAX_NUM_NEGATIVE_PICTURES];
  UInt NumPicStCurr0 = 0;
  UInt NumPicStCurr1 = 0;
  UInt NumPicLtCurr = 0;
  Int i;



  for (i = 0; i < m_pRPS->getNumberOfNegativePictures(); i++)
  {
	  if (m_pRPS->getUsed(i))
	  {
		  pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i));
		  pcRefPic->longTerm = false;
		  pcRefPic->extendPicBorder();
		  RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;
		  NumPicStCurr0++;
		  //cout << "in Negative getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << endl;
		
	  }
  }

  

  for(; i < m_pRPS->getNumberOfNegativePictures()+m_pRPS->getNumberOfPositivePictures(); i++)
  {
    if(m_pRPS->getUsed(i))
    {
      pcRefPic = xGetRefPic(rcListPic, getPOC()+m_pRPS->getDeltaPOC(i));
      pcRefPic->longTerm = false;
      pcRefPic->extendPicBorder();
      RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;
      NumPicStCurr1++;
	 // cout << "in Positive getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << endl;
    }
  }



  for(i = m_pRPS->getNumberOfNegativePictures()+m_pRPS->getNumberOfPositivePictures()+m_pRPS->getNumberOfLongtermPictures()-1; i > m_pRPS->getNumberOfNegativePictures()+m_pRPS->getNumberOfPositivePictures()-1 ; i--)
  {
    if(m_pRPS->getUsed(i))
    {
      pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
      pcRefPic->longTerm = true;
      pcRefPic->extendPicBorder();
      RefPicSetLtCurr[NumPicLtCurr] = pcRefPic;
      NumPicLtCurr++;
    }
    if(pcRefPic==NULL)
    {
      pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
    }
  }

  // ref_pic_list_init
  Picture*  rpsCurrList0[MAX_NUM_REF+1];
  Picture*  rpsCurrList1[MAX_NUM_REF+1];
  Int numPicTotalCurr = NumPicStCurr0 + NumPicStCurr1 + NumPicLtCurr;

  if (checkNumPocTotalCurr)
  {
    // The variable NumPocTotalCurr is derived as specified in subclause 7.4.7.2. It is a requirement of bitstream conformance that the following applies to the value of NumPocTotalCurr:
    // - If the current picture is a BLA or CRA picture, the value of NumPocTotalCurr shall be equal to 0.
    // - Otherwise, when the current picture contains a P or B slice, the value of NumPocTotalCurr shall not be equal to 0.
    if (getRapPicFlag())
    {
      CHECK(numPicTotalCurr != 0, "Invalid state");
    }

    if (m_eSliceType == I_SLICE)
    {
      return;
    }

    CHECK(numPicTotalCurr == 0, "Invalid state");
    // general tier and level limit:
    CHECK(numPicTotalCurr > 8, "Invalid state");
  }

  Int cIdx = 0;
  for ( i=0; i<NumPicStCurr0; i++, cIdx++)
  {
    rpsCurrList0[cIdx] = RefPicSetStCurr0[i];
  }
  for ( i=0; i<NumPicStCurr1; i++, cIdx++)
  {
    rpsCurrList0[cIdx] = RefPicSetStCurr1[i];
  }
  for ( i=0; i<NumPicLtCurr;  i++, cIdx++)
  {
    rpsCurrList0[cIdx] = RefPicSetLtCurr[i];
  }
  CHECK(cIdx != numPicTotalCurr, "Invalid state");

  if (m_eSliceType==B_SLICE)
  {
    cIdx = 0;
    for ( i=0; i<NumPicStCurr1; i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetStCurr1[i];
    }
    for ( i=0; i<NumPicStCurr0; i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetStCurr0[i];
    }
    for ( i=0; i<NumPicLtCurr;  i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetLtCurr[i];
    }
    CHECK(cIdx != numPicTotalCurr, "Invalid state");
  }


  ::memset(m_bIsUsedAsLongTerm, 0, sizeof(m_bIsUsedAsLongTerm));

  for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_0]; rIdx ++)
  {
	  //cout << "m_aiNumRefIdx[REF_PIC_LIST_0]" << m_aiNumRefIdx[REF_PIC_LIST_0] << endl;
    cIdx = m_RefPicListModification.getRefPicListModificationFlagL0() ? m_RefPicListModification.getRefPicSetIdxL0(rIdx) : rIdx % numPicTotalCurr;
    CHECK(cIdx < 0 || cIdx >= numPicTotalCurr, "Invalid state");
    m_apcRefPicList[REF_PIC_LIST_0][rIdx] = rpsCurrList0[ cIdx ];
    m_bIsUsedAsLongTerm[REF_PIC_LIST_0][rIdx] = ( cIdx >= NumPicStCurr0 + NumPicStCurr1 );
  }
  if ( m_eSliceType != B_SLICE )
  {
    m_aiNumRefIdx[REF_PIC_LIST_1] = 0;
    ::memset( m_apcRefPicList[REF_PIC_LIST_1], 0, sizeof(m_apcRefPicList[REF_PIC_LIST_1]));
  }
  else
  {
    for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_1]; rIdx ++)
    {
	  //cout << "m_aiNumRefIdx[REF_PIC_LIST_1]" << m_aiNumRefIdx[REF_PIC_LIST_1] << endl;
      cIdx = m_RefPicListModification.getRefPicListModificationFlagL1() ? m_RefPicListModification.getRefPicSetIdxL1(rIdx) : rIdx % numPicTotalCurr;
	  //cout << "cIdx" << cIdx << endl;
      CHECK(cIdx < 0 || cIdx >= numPicTotalCurr, "Invalid state");
      m_apcRefPicList[REF_PIC_LIST_1][rIdx] = rpsCurrList1[ cIdx ];
      m_bIsUsedAsLongTerm[REF_PIC_LIST_1][rIdx] = ( cIdx >= NumPicStCurr0 + NumPicStCurr1 );
    }
  }

    // For generalized B
  // note: maybe not existed case (always L0 is copied to L1 if L1 is empty)
  if( bCopyL0toL1ErrorCase && isInterB() && getNumRefIdx(REF_PIC_LIST_1) == 0)
  {
    Int iNumRefIdx = getNumRefIdx(REF_PIC_LIST_0);
    setNumRefIdx( REF_PIC_LIST_1, iNumRefIdx );

    for (Int iRefIdx = 0; iRefIdx < iNumRefIdx; iRefIdx++)
    {
      m_apcRefPicList[REF_PIC_LIST_1][iRefIdx] = m_apcRefPicList[REF_PIC_LIST_0] [iRefIdx];
    }
  }
}
#if BG_REFERENCE_SUBSTITUTION

Void Slice::setRefPicListaddbg(PicList& rcListPic,Picture* bgPicYuv, Picture* rcTempPicYuv,Int& j,Bool checkNumPocTotalCurr, Bool bCopyL0toL1ErrorCase)
{
	//put rcList to rcTempicYuv,put bgPicYuv to rcList
	if (m_eSliceType == I_SLICE)
	{
		::memset(m_apcRefPicList, 0, sizeof(m_apcRefPicList));
		::memset(m_aiNumRefIdx, 0, sizeof(m_aiNumRefIdx));

		if (!checkNumPocTotalCurr)
		{
			return;
		}
	}

	Picture*  pcRefPic = NULL;
	static const UInt MAX_NUM_NEGATIVE_PICTURES = 16;
	Picture*  RefPicSetStCurr0[MAX_NUM_NEGATIVE_PICTURES];
	Picture*  RefPicSetStCurr1[MAX_NUM_NEGATIVE_PICTURES];
	Picture*  RefPicSetLtCurr[MAX_NUM_NEGATIVE_PICTURES];
	UInt NumPicStCurr0 = 0;
	UInt NumPicStCurr1 = 0;
	UInt NumPicLtCurr = 0;
	Int i;

	for (i = 0; i < m_pRPS->getNumberOfNegativePictures()-1; i++)  //设置向前参考图像
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i));
			pcRefPic->longTerm = false;
			pcRefPic->extendPicBorder();
			RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;
			NumPicStCurr0++;
			//cout << "in Negative getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << endl;
		}
	}

	if (m_pRPS->getUsed(i))   //lowdelay 
	{
		j = m_pRPS->getDeltaPOC(i);
		pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i)); //找到参考帧
		//cout << "getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << "j:" << j << endl;
		pcRefPic->Copy2Temp(rcTempPicYuv, pcRefPic); //第一个等于第二个
		//pcRefPic->CopyOrg(pcRefPic,rcTempPicYuv);//----
		pcRefPic->CopyBGYuv(bgPicYuv, pcRefPic); //第二个等于第一个
		//pcRefPic->CopyOrg(bgPicYuv, pcRefPic);//----
		pcRefPic->longTerm = false;//LT flag 设置为0
		pcRefPic->extendPicBorder();
		RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;
		NumPicStCurr0++;
		i++;
	}

	
	for (; i < m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures(); i++)//
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i));
			pcRefPic->longTerm = false;
			pcRefPic->extendPicBorder();
			RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;
			NumPicStCurr1++;
			//cout << "in Positive getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << endl;
		}
	}

	/*Int jj;
	for (jj = -16; jj < 16; jj++)
	{
		if (jj == 0)
			continue;
		cout << "j:" << jj << " " << xGetRefPic(rcListPic, getPOC() + jj) << endl;
		if (xGetRefPic(rcListPic, getPOC() + jj))
		{
			cout << "jj:" << jj << " " << xGetRefPic(rcListPic, getPOC() + jj) << endl;
		}
		if (m_pRPS->getUsed(jj))
		{
			cout << "jjj:" << jj << " " << xGetRefPic(rcListPic, getPOC() + +m_pRPS->getDeltaPOC(jj)) << endl;
		}
		cout << "m_pRPS->getDeltaPOC(jj)" << m_pRPS->getDeltaPOC(jj) << endl;
		if (xGetRefPic(rcListPic, getPOC() + jj) && xGetRefPic(rcListPic, getPOC() + jj)->referenced != NULL)
		{
			cout << "reference" << xGetRefPic(rcListPic, getPOC() + jj)->referenced << endl;
		}
		else
			cout << "reference NULL" << endl;
		if (xGetRefPic(rcListPic, getPOC() + jj) && xGetRefPic(rcListPic, getPOC() + jj)->reconstructed != NULL)
		{
			cout << "reconstructed" << xGetRefPic(rcListPic, getPOC() + jj)->reconstructed << endl;
		}
		else
		{
			cout << "reconstructed NULL" << endl;
		}
	}*/

#if addbg3ref
	Int k;
	Bool f;
	for (j = -1; j > -16; j--)
	{
		f = true;
		for (k = 0; k < m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures(); k++)
		{
			if (j == m_pRPS->getDeltaPOC(k))
			{
				f = false;
				break;
			}
		}
		if (f && xGetRefPic(rcListPic, getPOC() + j) && xGetRefPic(rcListPic, getPOC() + j)->reconstructed)
		{
			pcRefPic = xGetRefPic(rcListPic, getPOC() + j); //找到参考帧	
			pcRefPic->Copy2Temp(rcTempPicYuv, pcRefPic); //第一个等于第二个
			pcRefPic->CopyBGYuv(bgPicYuv, pcRefPic); //第二个等于第一个
			pcRefPic->longTerm = false;//LT flag 设置为0
			pcRefPic->extendPicBorder();
			/*if (m_pRPS->getNumberOfNegativePictures() >= 3)  
			{
				NumPicStCurr0 = NumPicStCurr0 - (m_pRPS->getNumberOfNegativePictures() - 3);
				RefPicSetStCurr0[--NumPicStCurr0] = pcRefPic;//Negative 换
				NumPicStCurr0++;
				NumPicStCurr0 = NumPicStCurr0 + (m_pRPS->getNumberOfNegativePictures() - 3);
				RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;//Positive 加
				NumPicStCurr1++;
			}
			else if(m_pRPS->getNumberOfPositivePictures() >= 3)
			{
				NumPicStCurr1 = NumPicStCurr1 - (m_pRPS->getNumberOfPositivePictures() - 3);
				RefPicSetStCurr1[--NumPicStCurr1] = pcRefPic;//Positive 换
				NumPicStCurr1++;
				NumPicStCurr1 = NumPicStCurr1 + (m_pRPS->getNumberOfPositivePictures() - 3);
				RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;//Negative 加
				NumPicStCurr0++;
			}
			else  //都不足三个
			{
				RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;//Negative 加
				NumPicStCurr0++;
				RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;//Positive 加
				NumPicStCurr1++;
			}*/
			cout << "setbg jjjjjjjj" << j << "getPOC() + j!!!!!!" << getPOC() + j << endl;
			break;
		}
	}
#endif
	
	
	

	for (i = m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures() + m_pRPS->getNumberOfLongtermPictures() - 1; i > m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures() - 1; i--)//
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
			pcRefPic->longTerm = true;
			pcRefPic->extendPicBorder();
			RefPicSetLtCurr[NumPicLtCurr] = pcRefPic;
			NumPicLtCurr++;
		}
		if (pcRefPic == NULL)
		{
			pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
		}
	}

	// ref_pic_list_init
	Picture*  rpsCurrList0[MAX_NUM_REF + 1];
	Picture*  rpsCurrList1[MAX_NUM_REF + 1];
	Int numPicTotalCurr = NumPicStCurr0 + NumPicStCurr1 + NumPicLtCurr;

	if (checkNumPocTotalCurr)
	{
		// The variable NumPocTotalCurr is derived as specified in subclause 7.4.7.2. It is a requirement of bitstream conformance that the following applies to the value of NumPocTotalCurr:
		// - If the current picture is a BLA or CRA picture, the value of NumPocTotalCurr shall be equal to 0.
		// - Otherwise, when the current picture contains a P or B slice, the value of NumPocTotalCurr shall not be equal to 0.
		if (getRapPicFlag())
		{
			CHECK(numPicTotalCurr != 0, "Invalid state");
		}

		if (m_eSliceType == I_SLICE)
		{
			return;
		}

		CHECK(numPicTotalCurr == 0, "Invalid state");
		// general tier and level limit:
		CHECK(numPicTotalCurr > 8, "Invalid state");
	}

	Int cIdx = 0;
	for (i = 0; i<NumPicStCurr0; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetStCurr0[i];
	}
	for (i = 0; i<NumPicStCurr1; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetStCurr1[i];
	}
	for (i = 0; i<NumPicLtCurr; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetLtCurr[i];
	}
	CHECK(cIdx != numPicTotalCurr, "Invalid state");
#if addbg3ref  //直接加到参考帧的最后一帧
	rpsCurrList0[2] = pcRefPic;
	cIdx++;
#endif
	if (m_eSliceType == B_SLICE)
	{
		cIdx = 0;
		for (i = 0; i<NumPicStCurr1; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetStCurr1[i];
		}
		for (i = 0; i<NumPicStCurr0; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetStCurr0[i];
		}
		for (i = 0; i<NumPicLtCurr; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetLtCurr[i];
		}
		CHECK(cIdx != numPicTotalCurr, "Invalid state");
#if addbg3ref
		rpsCurrList1[2] = pcRefPic;
		cIdx++;
#endif
	}

	::memset(m_bIsUsedAsLongTerm, 0, sizeof(m_bIsUsedAsLongTerm));

	for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_0]; rIdx++)
	{
		//cout << "m_aiNumRefIdx[REF_PIC_LIST_0]" << m_aiNumRefIdx[REF_PIC_LIST_0] << endl;
		cIdx = m_RefPicListModification.getRefPicListModificationFlagL0() ? m_RefPicListModification.getRefPicSetIdxL0(rIdx) : rIdx % numPicTotalCurr;
		CHECK(cIdx < 0 || cIdx >= numPicTotalCurr, "Invalid state");
		m_apcRefPicList[REF_PIC_LIST_0][rIdx] = rpsCurrList0[cIdx];
		m_bIsUsedAsLongTerm[REF_PIC_LIST_0][rIdx] = (cIdx >= NumPicStCurr0 + NumPicStCurr1);
	}

	if (m_eSliceType != B_SLICE)
	{
		m_aiNumRefIdx[REF_PIC_LIST_1] = 0;
		::memset(m_apcRefPicList[REF_PIC_LIST_1], 0, sizeof(m_apcRefPicList[REF_PIC_LIST_1]));
	}
	else
	{
		for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_1]; rIdx++)
		{
			//cout << "m_aiNumRefIdx[REF_PIC_LIST_1]" << m_aiNumRefIdx[REF_PIC_LIST_1] << endl;
			cIdx = m_RefPicListModification.getRefPicListModificationFlagL1() ? m_RefPicListModification.getRefPicSetIdxL1(rIdx) : rIdx % numPicTotalCurr;
			CHECK(cIdx < 0 || cIdx >= numPicTotalCurr, "Invalid state");
			m_apcRefPicList[REF_PIC_LIST_1][rIdx] = rpsCurrList1[cIdx];
			m_bIsUsedAsLongTerm[REF_PIC_LIST_1][rIdx] = (cIdx >= NumPicStCurr0 + NumPicStCurr1);
		}
	}

	// For generalized B
	// note: maybe not existed case (always L0 is copied to L1 if L1 is empty)
	if (bCopyL0toL1ErrorCase && isInterB() && getNumRefIdx(REF_PIC_LIST_1) == 0)
	{
		Int iNumRefIdx = getNumRefIdx(REF_PIC_LIST_0);
		setNumRefIdx(REF_PIC_LIST_1, iNumRefIdx);

		for (Int iRefIdx = 0; iRefIdx < iNumRefIdx; iRefIdx++)
		{
			m_apcRefPicList[REF_PIC_LIST_1][iRefIdx] = m_apcRefPicList[REF_PIC_LIST_0][iRefIdx];
		}
	}
	cout << "setbg over" << endl;
}

Void Slice::resetRefPicList(PicList& rcListPic, Picture* TempPicYuv,Int j)
{
	//将TempPicYuv加入rcListPic
	Picture*  pcRefPic = NULL;
	Int i = m_pRPS->getNumberOfNegativePictures()-1;
	
	if (xGetRefPic(rcListPic, getPOC() + j))
	{
		//cout << "reset jjjj" << j << endl;
		pcRefPic = xGetRefPic(rcListPic, getPOC() + j);
		pcRefPic->CopyBack(TempPicYuv, pcRefPic); //第二个等于第一个
		//pcRefPic->CopyOrg(TempPicYuv, pcRefPic);
	}
}
Void Slice::resetRefPicListRec(PicList& rcListPic, Picture* TempPicYuv, Int j)
{
	//将TempPicYuv加入rcListPic
	Picture*  pcRefPic = NULL;
	Int i = 3;

	if (xGetRefPic(rcListPic, getPOC() + j))
	{
		//cout << "reset jjjj" << j << endl;
		pcRefPic = xGetRefPic(rcListPic, getPOC() + j);
		pcRefPic->CopyBack(TempPicYuv, pcRefPic); //第二个等于第一个
												  //pcRefPic->CopyOrg(TempPicYuv, pcRefPic);
	}
}
#if BLOCK_ENCODE
Void Slice::setRefPicListaddbgBlockRec(PicList& rcListPic, Picture* bgPicYuv, Picture* rcTempPicYuv, Int& j, Int BgBlock[], Bool checkNumPocTotalCurr, Bool bCopyL0toL1ErrorCase)
{
	//put rcList to rcTempicYuv,put bgPicYuv to rcList
	if (m_eSliceType == I_SLICE)
	{
		::memset(m_apcRefPicList, 0, sizeof(m_apcRefPicList));
		::memset(m_aiNumRefIdx, 0, sizeof(m_aiNumRefIdx));

		if (!checkNumPocTotalCurr)
		{
			return;
		}
	}

	Picture*  pcRefPic = NULL;
	static const UInt MAX_NUM_NEGATIVE_PICTURES = 16;
	Picture*  RefPicSetStCurr0[MAX_NUM_NEGATIVE_PICTURES];
	Picture*  RefPicSetStCurr1[MAX_NUM_NEGATIVE_PICTURES];
	Picture*  RefPicSetLtCurr[MAX_NUM_NEGATIVE_PICTURES];
	UInt NumPicStCurr0 = 0;
	UInt NumPicStCurr1 = 0;
	UInt NumPicLtCurr = 0;
	Int i;

	for (i = 0; i < m_pRPS->getNumberOfNegativePictures()-1; i++)  //设置向前参考图像
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i));
			pcRefPic->longTerm = false;
			pcRefPic->extendPicBorder();
			RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;
			NumPicStCurr0++;
			//cout << "in Negative getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << endl;
		}
	}
	//在50帧之前
	if (m_pRPS->getUsed(i))   //lowdelay 
	{
		j = m_pRPS->getDeltaPOC(i);
		pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i)); //找到参考帧
		//cout << "getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << "j:" << j << endl;
		pcRefPic->Copy2Temp(rcTempPicYuv, pcRefPic); //第一个等于第二个
		Int num_block = 0;
		for (Int i = 0; i < pcRefPic->getRecoBuf().Y().height; i += BLOCK_GEN_LEN)
		{
			for (Int j = 0; j < pcRefPic->getRecoBuf().Y().width; j += BLOCK_GEN_LEN)
			{
				if (BgBlock[num_block] >0 &&BgBlock[num_block]<1500)
				{
					pcRefPic->CopyReco2Block(pcRefPic, j, i, bgPicYuv); //第一个等于第二个
					//pcRefPic->CopyOrg2Block(pcRefPic, j, i, bgPicYuv); //第一个等于第二个
				}
				num_block++;
			}
		}
		//pcRefPic->CopyBGYuv(bgPicYuv, pcRefPic); //第二个等于第一个
		pcRefPic->longTerm = false;//LT flag 设置为0
		pcRefPic->extendPicBorder();
		RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;
		NumPicStCurr0++;
		i++;
	}


	for (; i < m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures(); i++)//
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i));
			pcRefPic->longTerm = false;
			pcRefPic->extendPicBorder();
			RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;
			NumPicStCurr1++;
			//cout << "in Positive getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << endl;
		}
	}



#if addbg3ref
	Int k;
	Bool f;
	for (j = -1; j > -16; j--)
	{
		f = true;
		for (k = 0; k < m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures(); k++)
		{
			if (j == m_pRPS->getDeltaPOC(k))
			{
				f = false;
				break;
			}
		}
		if (f && xGetRefPic(rcListPic, getPOC() + j) && xGetRefPic(rcListPic, getPOC() + j)->reconstructed)
		{
			pcRefPic = xGetRefPic(rcListPic, getPOC() + j); //找到参考帧	
			pcRefPic->Copy2Temp(rcTempPicYuv, pcRefPic); //第一个等于第二个
			pcRefPic->CopyBGYuv(bgPicYuv, pcRefPic); //第二个等于第一个
			pcRefPic->longTerm = false;//LT flag 设置为0
			pcRefPic->extendPicBorder();
			/*if (m_pRPS->getNumberOfNegativePictures() >= 3)
			{
			NumPicStCurr0 = NumPicStCurr0 - (m_pRPS->getNumberOfNegativePictures() - 3);
			RefPicSetStCurr0[--NumPicStCurr0] = pcRefPic;//Negative 换
			NumPicStCurr0++;
			NumPicStCurr0 = NumPicStCurr0 + (m_pRPS->getNumberOfNegativePictures() - 3);
			RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;//Positive 加
			NumPicStCurr1++;
			}
			else if(m_pRPS->getNumberOfPositivePictures() >= 3)
			{
			NumPicStCurr1 = NumPicStCurr1 - (m_pRPS->getNumberOfPositivePictures() - 3);
			RefPicSetStCurr1[--NumPicStCurr1] = pcRefPic;//Positive 换
			NumPicStCurr1++;
			NumPicStCurr1 = NumPicStCurr1 + (m_pRPS->getNumberOfPositivePictures() - 3);
			RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;//Negative 加
			NumPicStCurr0++;
			}
			else  //都不足三个
			{
			RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;//Negative 加
			NumPicStCurr0++;
			RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;//Positive 加
			NumPicStCurr1++;
			}*/
			cout << "setbg jjjjjjjj" << j << "getPOC() + j!!!!!!" << getPOC() + j << endl;
			break;
		}
	}
#endif




	for (i = m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures() + m_pRPS->getNumberOfLongtermPictures() - 1; i > m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures() - 1; i--)//
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
			pcRefPic->longTerm = true;
			pcRefPic->extendPicBorder();
			RefPicSetLtCurr[NumPicLtCurr] = pcRefPic;
			NumPicLtCurr++;
		}
		if (pcRefPic == NULL)
		{
			pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
		}
	}

	// ref_pic_list_init
	Picture*  rpsCurrList0[MAX_NUM_REF + 1];
	Picture*  rpsCurrList1[MAX_NUM_REF + 1];
	Int numPicTotalCurr = NumPicStCurr0 + NumPicStCurr1 + NumPicLtCurr;

	if (checkNumPocTotalCurr)
	{
		// The variable NumPocTotalCurr is derived as specified in subclause 7.4.7.2. It is a requirement of bitstream conformance that the following applies to the value of NumPocTotalCurr:
		// - If the current picture is a BLA or CRA picture, the value of NumPocTotalCurr shall be equal to 0.
		// - Otherwise, when the current picture contains a P or B slice, the value of NumPocTotalCurr shall not be equal to 0.
		if (getRapPicFlag())
		{
			CHECK(numPicTotalCurr != 0, "Invalid state");
		}

		if (m_eSliceType == I_SLICE)
		{
			return;
		}

		CHECK(numPicTotalCurr == 0, "Invalid state");
		// general tier and level limit:
		CHECK(numPicTotalCurr > 8, "Invalid state");
	}

	Int cIdx = 0;
	for (i = 0; i<NumPicStCurr0; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetStCurr0[i];
	}
	for (i = 0; i<NumPicStCurr1; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetStCurr1[i];
	}
	for (i = 0; i<NumPicLtCurr; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetLtCurr[i];
	}
	CHECK(cIdx != numPicTotalCurr, "Invalid state");
#if addbg3ref  //直接加到参考帧的最后一帧
	rpsCurrList0[2] = pcRefPic;
	cIdx++;
#endif
	if (m_eSliceType == B_SLICE)
	{
		cIdx = 0;
		for (i = 0; i<NumPicStCurr1; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetStCurr1[i];
		}
		for (i = 0; i<NumPicStCurr0; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetStCurr0[i];
		}
		for (i = 0; i<NumPicLtCurr; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetLtCurr[i];
		}
		CHECK(cIdx != numPicTotalCurr, "Invalid state");
#if addbg3ref
		rpsCurrList1[2] = pcRefPic;
		cIdx++;
#endif
	}

	::memset(m_bIsUsedAsLongTerm, 0, sizeof(m_bIsUsedAsLongTerm));

	for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_0]; rIdx++)
	{
		//cout << "m_aiNumRefIdx[REF_PIC_LIST_0]" << m_aiNumRefIdx[REF_PIC_LIST_0] << endl;
		cIdx = m_RefPicListModification.getRefPicListModificationFlagL0() ? m_RefPicListModification.getRefPicSetIdxL0(rIdx) : rIdx % numPicTotalCurr;
		CHECK(cIdx < 0 || cIdx >= numPicTotalCurr, "Invalid state");
		m_apcRefPicList[REF_PIC_LIST_0][rIdx] = rpsCurrList0[cIdx];
		m_bIsUsedAsLongTerm[REF_PIC_LIST_0][rIdx] = (cIdx >= NumPicStCurr0 + NumPicStCurr1);
	}

	if (m_eSliceType != B_SLICE)
	{
		m_aiNumRefIdx[REF_PIC_LIST_1] = 0;
		::memset(m_apcRefPicList[REF_PIC_LIST_1], 0, sizeof(m_apcRefPicList[REF_PIC_LIST_1]));
	}
	else
	{
		for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_1]; rIdx++)
		{
			//cout << "m_aiNumRefIdx[REF_PIC_LIST_1]" << m_aiNumRefIdx[REF_PIC_LIST_1] << endl;
			cIdx = m_RefPicListModification.getRefPicListModificationFlagL1() ? m_RefPicListModification.getRefPicSetIdxL1(rIdx) : rIdx % numPicTotalCurr;
			CHECK(cIdx < 0 || cIdx >= numPicTotalCurr, "Invalid state");
			m_apcRefPicList[REF_PIC_LIST_1][rIdx] = rpsCurrList1[cIdx];
			m_bIsUsedAsLongTerm[REF_PIC_LIST_1][rIdx] = (cIdx >= NumPicStCurr0 + NumPicStCurr1);
		}
	}

	// For generalized B
	// note: maybe not existed case (always L0 is copied to L1 if L1 is empty)
	if (bCopyL0toL1ErrorCase && isInterB() && getNumRefIdx(REF_PIC_LIST_1) == 0)
	{
		Int iNumRefIdx = getNumRefIdx(REF_PIC_LIST_0);
		setNumRefIdx(REF_PIC_LIST_1, iNumRefIdx);

		for (Int iRefIdx = 0; iRefIdx < iNumRefIdx; iRefIdx++)
		{
			m_apcRefPicList[REF_PIC_LIST_1][iRefIdx] = m_apcRefPicList[REF_PIC_LIST_0][iRefIdx];
		}
	}
}
Void Slice::setRefPicListaddbgBlock(PicList& rcListPic, Picture* bgPicYuv, Picture* rcTempPicYuv, Int& j, Int BgBlock[], Bool checkNumPocTotalCurr, Bool bCopyL0toL1ErrorCase)
{
	//put rcList to rcTempicYuv,put bgPicYuv to rcList
	if (m_eSliceType == I_SLICE)
	{
		::memset(m_apcRefPicList, 0, sizeof(m_apcRefPicList));
		::memset(m_aiNumRefIdx, 0, sizeof(m_aiNumRefIdx));

		if (!checkNumPocTotalCurr)
		{
			return;
		}
	}

	Picture*  pcRefPic = NULL;
	static const UInt MAX_NUM_NEGATIVE_PICTURES = 16;
	Picture*  RefPicSetStCurr0[MAX_NUM_NEGATIVE_PICTURES];
	Picture*  RefPicSetStCurr1[MAX_NUM_NEGATIVE_PICTURES];
	Picture*  RefPicSetLtCurr[MAX_NUM_NEGATIVE_PICTURES];
	UInt NumPicStCurr0 = 0;
	UInt NumPicStCurr1 = 0;
	UInt NumPicLtCurr = 0;
	Int i;

	for (i = 0; i < m_pRPS->getNumberOfNegativePictures() - 1; i++)  //设置向前参考图像
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i));
			pcRefPic->longTerm = false;
			pcRefPic->extendPicBorder();
			RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;
			NumPicStCurr0++;
			//cout << "in Negative getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << endl;
		}
	}
	//在50帧之前
	if (m_pRPS->getUsed(i))   //lowdelay 
	{
		j = m_pRPS->getDeltaPOC(i);
		pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i)); //找到参考帧
		//cout << "getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << "j:" << j << endl;
		pcRefPic->Copy2Temp(rcTempPicYuv, pcRefPic); //第一个等于第二个
		//pcRefPic->CopyOrg(pcRefPic, rcTempPicYuv);//第二个等于第一个
		//参考帧中加入已编码的块。
		Int num_block = 0;
		for (Int i = 0; i < pcRefPic->getRecoBuf().Y().height; i += BLOCK_GEN_LEN)
		{
			for (Int j = 0; j < pcRefPic->getRecoBuf().Y().width; j += BLOCK_GEN_LEN)
			{
				if (BgBlock[num_block] >= 2000)
				{
					pcRefPic->CopyReco2Block(pcRefPic, j, i, bgPicYuv); //第一个等于第二个
					//pcRefPic->CopyOrg2Block(pcRefPic, j, i, bgPicYuv); //第一个等于第二个
				}
				num_block++;
			}
		}
		//pcRefPic->CopyBGYuv(bgPicYuv, pcRefPic); //第二个等于第一个
		pcRefPic->longTerm = false;//LT flag 设置为0
		pcRefPic->extendPicBorder();
		RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;
		NumPicStCurr0++;
		i++;
	}


	for (; i < m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures(); i++)//
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i));
			pcRefPic->longTerm = false;
			pcRefPic->extendPicBorder();
			RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;
			NumPicStCurr1++;
			//cout << "in Positive getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << endl;
		}
	}

	

#if addbg3ref
	Int k;
	Bool f;
	for (j = -1; j > -16; j--)
	{
		f = true;
		for (k = 0; k < m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures(); k++)
		{
			if (j == m_pRPS->getDeltaPOC(k))
			{
				f = false;
				break;
			}
		}
		if (f && xGetRefPic(rcListPic, getPOC() + j) && xGetRefPic(rcListPic, getPOC() + j)->reconstructed)
		{
			pcRefPic = xGetRefPic(rcListPic, getPOC() + j); //找到参考帧	
			pcRefPic->Copy2Temp(rcTempPicYuv, pcRefPic); //第一个等于第二个
			pcRefPic->CopyBGYuv(bgPicYuv, pcRefPic); //第二个等于第一个
			pcRefPic->longTerm = false;//LT flag 设置为0
			pcRefPic->extendPicBorder();
			/*if (m_pRPS->getNumberOfNegativePictures() >= 3)
			{
			NumPicStCurr0 = NumPicStCurr0 - (m_pRPS->getNumberOfNegativePictures() - 3);
			RefPicSetStCurr0[--NumPicStCurr0] = pcRefPic;//Negative 换
			NumPicStCurr0++;
			NumPicStCurr0 = NumPicStCurr0 + (m_pRPS->getNumberOfNegativePictures() - 3);
			RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;//Positive 加
			NumPicStCurr1++;
			}
			else if(m_pRPS->getNumberOfPositivePictures() >= 3)
			{
			NumPicStCurr1 = NumPicStCurr1 - (m_pRPS->getNumberOfPositivePictures() - 3);
			RefPicSetStCurr1[--NumPicStCurr1] = pcRefPic;//Positive 换
			NumPicStCurr1++;
			NumPicStCurr1 = NumPicStCurr1 + (m_pRPS->getNumberOfPositivePictures() - 3);
			RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;//Negative 加
			NumPicStCurr0++;
			}
			else  //都不足三个
			{
			RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;//Negative 加
			NumPicStCurr0++;
			RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;//Positive 加
			NumPicStCurr1++;
			}*/
			cout << "setbg jjjjjjjj" << j << "getPOC() + j!!!!!!" << getPOC() + j << endl;
			break;
		}
	}
#endif




	for (i = m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures() + m_pRPS->getNumberOfLongtermPictures() - 1; i > m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures() - 1; i--)//
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
			pcRefPic->longTerm = true;
			pcRefPic->extendPicBorder();
			RefPicSetLtCurr[NumPicLtCurr] = pcRefPic;
			NumPicLtCurr++;
		}
		if (pcRefPic == NULL)
		{
			pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
		}
	}

	// ref_pic_list_init
	Picture*  rpsCurrList0[MAX_NUM_REF + 1];
	Picture*  rpsCurrList1[MAX_NUM_REF + 1];
	Int numPicTotalCurr = NumPicStCurr0 + NumPicStCurr1 + NumPicLtCurr;

	if (checkNumPocTotalCurr)
	{
		// The variable NumPocTotalCurr is derived as specified in subclause 7.4.7.2. It is a requirement of bitstream conformance that the following applies to the value of NumPocTotalCurr:
		// - If the current picture is a BLA or CRA picture, the value of NumPocTotalCurr shall be equal to 0.
		// - Otherwise, when the current picture contains a P or B slice, the value of NumPocTotalCurr shall not be equal to 0.
		if (getRapPicFlag())
		{
			CHECK(numPicTotalCurr != 0, "Invalid state");
		}

		if (m_eSliceType == I_SLICE)
		{
			return;
		}

		CHECK(numPicTotalCurr == 0, "Invalid state");
		// general tier and level limit:
		CHECK(numPicTotalCurr > 8, "Invalid state");
	}

	Int cIdx = 0;
	for (i = 0; i<NumPicStCurr0; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetStCurr0[i];
	}
	for (i = 0; i<NumPicStCurr1; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetStCurr1[i];
	}
	for (i = 0; i<NumPicLtCurr; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetLtCurr[i];
	}
	CHECK(cIdx != numPicTotalCurr, "Invalid state");
#if addbg3ref  //直接加到参考帧的最后一帧
	rpsCurrList0[2] = pcRefPic;
	cIdx++;
#endif
	if (m_eSliceType == B_SLICE)
	{
		cIdx = 0;
		for (i = 0; i<NumPicStCurr1; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetStCurr1[i];
		}
		for (i = 0; i<NumPicStCurr0; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetStCurr0[i];
		}
		for (i = 0; i<NumPicLtCurr; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetLtCurr[i];
		}
		CHECK(cIdx != numPicTotalCurr, "Invalid state");
#if addbg3ref
		rpsCurrList1[2] = pcRefPic;
		cIdx++;
#endif
	}

	::memset(m_bIsUsedAsLongTerm, 0, sizeof(m_bIsUsedAsLongTerm));

	for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_0]; rIdx++)
	{
		//cout << "m_aiNumRefIdx[REF_PIC_LIST_0]" << m_aiNumRefIdx[REF_PIC_LIST_0] << endl;
		cIdx = m_RefPicListModification.getRefPicListModificationFlagL0() ? m_RefPicListModification.getRefPicSetIdxL0(rIdx) : rIdx % numPicTotalCurr;
		CHECK(cIdx < 0 || cIdx >= numPicTotalCurr, "Invalid state");
		m_apcRefPicList[REF_PIC_LIST_0][rIdx] = rpsCurrList0[cIdx];
		m_bIsUsedAsLongTerm[REF_PIC_LIST_0][rIdx] = (cIdx >= NumPicStCurr0 + NumPicStCurr1);
	}

	if (m_eSliceType != B_SLICE)
	{
		m_aiNumRefIdx[REF_PIC_LIST_1] = 0;
		::memset(m_apcRefPicList[REF_PIC_LIST_1], 0, sizeof(m_apcRefPicList[REF_PIC_LIST_1]));
	}
	else
	{
		for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_1]; rIdx++)
		{
			//cout << "m_aiNumRefIdx[REF_PIC_LIST_1]" << m_aiNumRefIdx[REF_PIC_LIST_1] << endl;
			cIdx = m_RefPicListModification.getRefPicListModificationFlagL1() ? m_RefPicListModification.getRefPicSetIdxL1(rIdx) : rIdx % numPicTotalCurr;
			CHECK(cIdx < 0 || cIdx >= numPicTotalCurr, "Invalid state");
			m_apcRefPicList[REF_PIC_LIST_1][rIdx] = rpsCurrList1[cIdx];
			m_bIsUsedAsLongTerm[REF_PIC_LIST_1][rIdx] = (cIdx >= NumPicStCurr0 + NumPicStCurr1);
		}
	}

	// For generalized B
	// note: maybe not existed case (always L0 is copied to L1 if L1 is empty)
	if (bCopyL0toL1ErrorCase && isInterB() && getNumRefIdx(REF_PIC_LIST_1) == 0)
	{
		Int iNumRefIdx = getNumRefIdx(REF_PIC_LIST_0);
		setNumRefIdx(REF_PIC_LIST_1, iNumRefIdx);

		for (Int iRefIdx = 0; iRefIdx < iNumRefIdx; iRefIdx++)
		{
			m_apcRefPicList[REF_PIC_LIST_1][iRefIdx] = m_apcRefPicList[REF_PIC_LIST_0][iRefIdx];
		}
	}
}
#endif 
#endif // BG_REFERENCE_SUBSTITUTION
#if ENCODE_BGPIC
Void Slice::setRefPicListaddRecbg(PicList& rcListPic, Picture* bgPicYuv, Picture* rcTempPicYuv, Int& j, Bool checkNumPocTotalCurr, Bool bCopyL0toL1ErrorCase)
{
	//put rcList to rcTempicYuv,put bgPicYuv to rcList
	if (m_eSliceType == I_SLICE)
	{
		::memset(m_apcRefPicList, 0, sizeof(m_apcRefPicList));
		::memset(m_aiNumRefIdx, 0, sizeof(m_aiNumRefIdx));

		if (!checkNumPocTotalCurr)
		{
			return;
		}
	}

	Picture*  pcRefPic = NULL;
	static const UInt MAX_NUM_NEGATIVE_PICTURES = 16;
	Picture*  RefPicSetStCurr0[MAX_NUM_NEGATIVE_PICTURES];
	Picture*  RefPicSetStCurr1[MAX_NUM_NEGATIVE_PICTURES];
	Picture*  RefPicSetLtCurr[MAX_NUM_NEGATIVE_PICTURES];
	UInt NumPicStCurr0 = 0;
	UInt NumPicStCurr1 = 0;
	UInt NumPicLtCurr = 0;
	Int i;

	for (i = 0; i < m_pRPS->getNumberOfNegativePictures() - 1; i++)  //设置向前参考图像
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i));
			pcRefPic->longTerm = false;
			pcRefPic->extendPicBorder();
			RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;
			NumPicStCurr0++;


		}
	}
	if (m_pRPS->getUsed(i))
	{
		j = m_pRPS->getDeltaPOC(i);
		pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i)); //找到参考帧		 

		//pcRefPic->Copy2Temp(rcTempPicYuv, pcRefPic); //第一个等于第二个

		pcRefPic->longTerm = false;//LT flag 设置为0
		pcRefPic->extendPicBorder();
		//--------------
		//pcRefPic->CopyBGYuv(bgPicYuv, pcRefPic); //第二个等于第一个

		pcRefPic->extendPicBorder();
		RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;
		NumPicStCurr0++;
		//m_pRPS->setCheckLTMSBPresent(NumPicStCurr0,false);
		i++;
	}


	for (; i < m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures(); i++)
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i));
			pcRefPic->longTerm = false;
			pcRefPic->extendPicBorder();
			RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;
			NumPicStCurr1++;
		}
	}

	for (i = m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures() + m_pRPS->getNumberOfLongtermPictures() - 1; i > m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures() - 1; i--)
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
			pcRefPic->longTerm = true;
			pcRefPic->extendPicBorder();
			RefPicSetLtCurr[NumPicLtCurr] = pcRefPic;
			NumPicLtCurr++;
		}
		if (pcRefPic == NULL)
		{
			pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
		}
	}

	// ref_pic_list_init
	Picture*  rpsCurrList0[MAX_NUM_REF + 1];
	Picture*  rpsCurrList1[MAX_NUM_REF + 1];
	Int numPicTotalCurr = NumPicStCurr0 + NumPicStCurr1 + NumPicLtCurr;

	if (checkNumPocTotalCurr)
	{
		// The variable NumPocTotalCurr is derived as specified in subclause 7.4.7.2. It is a requirement of bitstream conformance that the following applies to the value of NumPocTotalCurr:
		// - If the current picture is a BLA or CRA picture, the value of NumPocTotalCurr shall be equal to 0.
		// - Otherwise, when the current picture contains a P or B slice, the value of NumPocTotalCurr shall not be equal to 0.
		if (getRapPicFlag())
		{
			CHECK(numPicTotalCurr != 0, "Invalid state");
		}

		if (m_eSliceType == I_SLICE)
		{
			return;
		}

		CHECK(numPicTotalCurr == 0, "Invalid state");
		// general tier and level limit:
		CHECK(numPicTotalCurr > 8, "Invalid state");
	}

	Int cIdx = 0;
	for (i = 0; i<NumPicStCurr0; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetStCurr0[i];
	}
	for (i = 0; i<NumPicStCurr1; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetStCurr1[i];
	}
	for (i = 0; i<NumPicLtCurr; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetLtCurr[i];
	}
	CHECK(cIdx != numPicTotalCurr, "Invalid state");

	if (m_eSliceType == B_SLICE)
	{
		cIdx = 0;
		for (i = 0; i<NumPicStCurr1; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetStCurr1[i];
		}
		for (i = 0; i<NumPicStCurr0; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetStCurr0[i];
		}
		for (i = 0; i<NumPicLtCurr; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetLtCurr[i];
		}
		CHECK(cIdx != numPicTotalCurr, "Invalid state");
	}

	::memset(m_bIsUsedAsLongTerm, 0, sizeof(m_bIsUsedAsLongTerm));

	for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_0]; rIdx++)
	{
		cIdx = m_RefPicListModification.getRefPicListModificationFlagL0() ? m_RefPicListModification.getRefPicSetIdxL0(rIdx) : rIdx % numPicTotalCurr;
		CHECK(cIdx < 0 || cIdx >= numPicTotalCurr, "Invalid state");
		m_apcRefPicList[REF_PIC_LIST_0][rIdx] = rpsCurrList0[cIdx];
		m_bIsUsedAsLongTerm[REF_PIC_LIST_0][rIdx] = (cIdx >= NumPicStCurr0 + NumPicStCurr1);
	}
	if (m_eSliceType != B_SLICE)
	{
		m_aiNumRefIdx[REF_PIC_LIST_1] = 0;
		::memset(m_apcRefPicList[REF_PIC_LIST_1], 0, sizeof(m_apcRefPicList[REF_PIC_LIST_1]));
	}
	else
	{
		for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_1]; rIdx++)
		{
			cIdx = m_RefPicListModification.getRefPicListModificationFlagL1() ? m_RefPicListModification.getRefPicSetIdxL1(rIdx) : rIdx % numPicTotalCurr;
			CHECK(cIdx < 0 || cIdx >= numPicTotalCurr, "Invalid state");
			m_apcRefPicList[REF_PIC_LIST_1][rIdx] = rpsCurrList1[cIdx];
			m_bIsUsedAsLongTerm[REF_PIC_LIST_1][rIdx] = (cIdx >= NumPicStCurr0 + NumPicStCurr1);
		}
	}

	// For generalized B
	// note: maybe not existed case (always L0 is copied to L1 if L1 is empty)
	if (bCopyL0toL1ErrorCase && isInterB() && getNumRefIdx(REF_PIC_LIST_1) == 0)
	{
		Int iNumRefIdx = getNumRefIdx(REF_PIC_LIST_0);
		setNumRefIdx(REF_PIC_LIST_1, iNumRefIdx);

		for (Int iRefIdx = 0; iRefIdx < iNumRefIdx; iRefIdx++)
		{
			m_apcRefPicList[REF_PIC_LIST_1][iRefIdx] = m_apcRefPicList[REF_PIC_LIST_0][iRefIdx];
		}
	}
}
#if BLOCK_ENCODE
Void Slice::setRefPicListaddBlockRecbg(PicList& rcListPic, Picture* bgPicYuv, Picture* rcTempPicYuv, Int& j, Int BgBlock[], Bool checkNumPocTotalCurr, Bool bCopyL0toL1ErrorCase)
{
	//put rcList to rcTempicYuv,put bgPicYuv to rcList
	if (m_eSliceType == I_SLICE)
	{
		::memset(m_apcRefPicList, 0, sizeof(m_apcRefPicList));
		::memset(m_aiNumRefIdx, 0, sizeof(m_aiNumRefIdx));

		if (!checkNumPocTotalCurr)
		{
			return;
		}
	}

	Picture*  pcRefPic = NULL;
	static const UInt MAX_NUM_NEGATIVE_PICTURES = 16;
	Picture*  RefPicSetStCurr0[MAX_NUM_NEGATIVE_PICTURES];
	Picture*  RefPicSetStCurr1[MAX_NUM_NEGATIVE_PICTURES];
	Picture*  RefPicSetLtCurr[MAX_NUM_NEGATIVE_PICTURES];
	UInt NumPicStCurr0 = 0;
	UInt NumPicStCurr1 = 0;
	UInt NumPicLtCurr = 0;
	Int i;

	for (i = 0; i < m_pRPS->getNumberOfNegativePictures() - 1; i++)  //设置向前参考图像
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i));
			pcRefPic->longTerm = false;
			pcRefPic->extendPicBorder();
			RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;
			NumPicStCurr0++;
			//cout << "in Negative getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << endl;
		}
	}

	if (m_pRPS->getUsed(i))   //lowdelay 
	{
		j = m_pRPS->getDeltaPOC(i);
		pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i)); //找到参考帧
																			 //cout << "getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << "j:" << j << endl;
		//pcRefPic->Copy2Temp(rcTempPicYuv, pcRefPic); //第一个等于第二个
		pcRefPic->CopyOrg(pcRefPic, rcTempPicYuv);//第二个等于第一个
		//参考帧中加入生成的BlocksRec
		Int num_block = 0;
		for (Int i = 0; i < pcRefPic->getRecoBuf().Y().height; i += BLOCK_GEN_LEN)
		{
			for (Int j = 0; j < pcRefPic->getRecoBuf().Y().width; j += BLOCK_GEN_LEN)
			{
				if (BgBlock[num_block] == 1)
				{
					pcRefPic->CopyReco2Block(pcRefPic, j, i, bgPicYuv); //第一个等于第二个
				}
				num_block++;
			}
		}
		//pcRefPic->CopyBGYuv(bgPicYuv, pcRefPic); //第二个等于第一个
		pcRefPic->longTerm = false;//LT flag 设置为0
		pcRefPic->extendPicBorder();
		RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;
		NumPicStCurr0++;
		i++;
	}


	for (; i < m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures(); i++)//
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetRefPic(rcListPic, getPOC() + m_pRPS->getDeltaPOC(i));
			pcRefPic->longTerm = false;
			pcRefPic->extendPicBorder();
			RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;
			NumPicStCurr1++;
			//cout << "in Positive getPOC() + m_pRPS->getDeltaPOC(i)" << getPOC() + m_pRPS->getDeltaPOC(i) << endl;
		}
	}


	for (i = m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures() + m_pRPS->getNumberOfLongtermPictures() - 1; i > m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures() - 1; i--)//
	{
		if (m_pRPS->getUsed(i))
		{
			pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
			pcRefPic->longTerm = true;
			pcRefPic->extendPicBorder();
			RefPicSetLtCurr[NumPicLtCurr] = pcRefPic;
			NumPicLtCurr++;
		}
		if (pcRefPic == NULL)
		{
			pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
		}
	}

	// ref_pic_list_init
	Picture*  rpsCurrList0[MAX_NUM_REF + 1];
	Picture*  rpsCurrList1[MAX_NUM_REF + 1];
	Int numPicTotalCurr = NumPicStCurr0 + NumPicStCurr1 + NumPicLtCurr;

	if (checkNumPocTotalCurr)
	{
		// The variable NumPocTotalCurr is derived as specified in subclause 7.4.7.2. It is a requirement of bitstream conformance that the following applies to the value of NumPocTotalCurr:
		// - If the current picture is a BLA or CRA picture, the value of NumPocTotalCurr shall be equal to 0.
		// - Otherwise, when the current picture contains a P or B slice, the value of NumPocTotalCurr shall not be equal to 0.
		if (getRapPicFlag())
		{
			CHECK(numPicTotalCurr != 0, "Invalid state");
		}

		if (m_eSliceType == I_SLICE)
		{
			return;
		}

		CHECK(numPicTotalCurr == 0, "Invalid state");
		// general tier and level limit:
		CHECK(numPicTotalCurr > 8, "Invalid state");
	}

	Int cIdx = 0;
	for (i = 0; i<NumPicStCurr0; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetStCurr0[i];
	}
	for (i = 0; i<NumPicStCurr1; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetStCurr1[i];
	}
	for (i = 0; i<NumPicLtCurr; i++, cIdx++)
	{
		rpsCurrList0[cIdx] = RefPicSetLtCurr[i];
	}
	CHECK(cIdx != numPicTotalCurr, "Invalid state");
#if addbg3ref  //直接加到参考帧的最后一帧
	rpsCurrList0[2] = pcRefPic;
	cIdx++;
#endif
	if (m_eSliceType == B_SLICE)
	{
		cIdx = 0;
		for (i = 0; i<NumPicStCurr1; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetStCurr1[i];
		}
		for (i = 0; i<NumPicStCurr0; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetStCurr0[i];
		}
		for (i = 0; i<NumPicLtCurr; i++, cIdx++)
		{
			rpsCurrList1[cIdx] = RefPicSetLtCurr[i];
		}
		CHECK(cIdx != numPicTotalCurr, "Invalid state");
#if addbg3ref
		rpsCurrList1[2] = pcRefPic;
		cIdx++;
#endif
	}

	::memset(m_bIsUsedAsLongTerm, 0, sizeof(m_bIsUsedAsLongTerm));

	for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_0]; rIdx++)
	{
		//cout << "m_aiNumRefIdx[REF_PIC_LIST_0]" << m_aiNumRefIdx[REF_PIC_LIST_0] << endl;
		cIdx = m_RefPicListModification.getRefPicListModificationFlagL0() ? m_RefPicListModification.getRefPicSetIdxL0(rIdx) : rIdx % numPicTotalCurr;
		CHECK(cIdx < 0 || cIdx >= numPicTotalCurr, "Invalid state");
		m_apcRefPicList[REF_PIC_LIST_0][rIdx] = rpsCurrList0[cIdx];
		m_bIsUsedAsLongTerm[REF_PIC_LIST_0][rIdx] = (cIdx >= NumPicStCurr0 + NumPicStCurr1);
	}

	if (m_eSliceType != B_SLICE)
	{
		m_aiNumRefIdx[REF_PIC_LIST_1] = 0;
		::memset(m_apcRefPicList[REF_PIC_LIST_1], 0, sizeof(m_apcRefPicList[REF_PIC_LIST_1]));
	}
	else
	{
		for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_1]; rIdx++)
		{
			//cout << "m_aiNumRefIdx[REF_PIC_LIST_1]" << m_aiNumRefIdx[REF_PIC_LIST_1] << endl;
			cIdx = m_RefPicListModification.getRefPicListModificationFlagL1() ? m_RefPicListModification.getRefPicSetIdxL1(rIdx) : rIdx % numPicTotalCurr;
			CHECK(cIdx < 0 || cIdx >= numPicTotalCurr, "Invalid state");
			m_apcRefPicList[REF_PIC_LIST_1][rIdx] = rpsCurrList1[cIdx];
			m_bIsUsedAsLongTerm[REF_PIC_LIST_1][rIdx] = (cIdx >= NumPicStCurr0 + NumPicStCurr1);
		}
	}

	// For generalized B
	// note: maybe not existed case (always L0 is copied to L1 if L1 is empty)
	if (bCopyL0toL1ErrorCase && isInterB() && getNumRefIdx(REF_PIC_LIST_1) == 0)
	{
		Int iNumRefIdx = getNumRefIdx(REF_PIC_LIST_0);
		setNumRefIdx(REF_PIC_LIST_1, iNumRefIdx);

		for (Int iRefIdx = 0; iRefIdx < iNumRefIdx; iRefIdx++)
		{
			m_apcRefPicList[REF_PIC_LIST_1][iRefIdx] = m_apcRefPicList[REF_PIC_LIST_0][iRefIdx];
		}
	}
	cout << "setbg over" << endl;
}
#endif
#endif // ENCODE_BGPIC


Int Slice::getNumRpsCurrTempList() const
{
  Int numRpsCurrTempList = 0;

  if (m_eSliceType == I_SLICE)
  {
    return 0;
  }
  for(UInt i=0; i < m_pRPS->getNumberOfNegativePictures()+ m_pRPS->getNumberOfPositivePictures() + m_pRPS->getNumberOfLongtermPictures(); i++)
  {
    if(m_pRPS->getUsed(i))
    {
      numRpsCurrTempList++;
    }
  }
  return numRpsCurrTempList;
}

Void Slice::initEqualRef()
{
  for (Int iDir = 0; iDir < NUM_REF_PIC_LIST_01; iDir++)
  {
    for (Int iRefIdx1 = 0; iRefIdx1 < MAX_NUM_REF; iRefIdx1++)
    {
      for (Int iRefIdx2 = iRefIdx1; iRefIdx2 < MAX_NUM_REF; iRefIdx2++)
      {
        m_abEqualRef[iDir][iRefIdx1][iRefIdx2] = m_abEqualRef[iDir][iRefIdx2][iRefIdx1] = (iRefIdx1 == iRefIdx2? true : false);
      }
    }
  }
}

Void Slice::checkColRefIdx(UInt curSliceSegmentIdx, const Picture* pic)
{
  Int i;
  Slice* curSlice = pic->slices[curSliceSegmentIdx];
  Int currColRefPOC =  curSlice->getRefPOC( RefPicList(1 - curSlice->getColFromL0Flag()), curSlice->getColRefIdx());

  for(i=curSliceSegmentIdx-1; i>=0; i--)
  {
    const Slice* preSlice = pic->slices[i];
    if(preSlice->getSliceType() != I_SLICE)
    {
      const Int preColRefPOC  = preSlice->getRefPOC( RefPicList(1 - preSlice->getColFromL0Flag()), preSlice->getColRefIdx());
      if(currColRefPOC != preColRefPOC)
      {
        THROW("Collocated_ref_idx shall always be the same for all slices of a coded picture!");
      }
      else
      {
        break;
      }
    }
  }
}

Void Slice::checkCRA(const ReferencePictureSet *pReferencePictureSet, Int& pocCRA, NalUnitType& associatedIRAPType, PicList& rcListPic)
{
  for(Int i = 0; i < pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures(); i++)
  {
    if(pocCRA < MAX_UINT && getPOC() > pocCRA)
    {
      CHECK(getPOC()+pReferencePictureSet->getDeltaPOC(i) < pocCRA, "Invalid state");
    }
  }
  for(Int i = pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures(); i < pReferencePictureSet->getNumberOfPictures(); i++)
  {
    if(pocCRA < MAX_UINT && getPOC() > pocCRA)
    {
      if (!pReferencePictureSet->getCheckLTMSBPresent(i))
      {
        CHECK(xGetLongTermRefPic(rcListPic, pReferencePictureSet->getPOC(i), false)->getPOC() < pocCRA, "Invalid state");
      }
      else
      {
        CHECK(pReferencePictureSet->getPOC(i) < pocCRA, "Invalid state");
      }
    }
  }
  if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP ) // IDR picture found
  {
    pocCRA = getPOC();
    associatedIRAPType = getNalUnitType();
  }
  else if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ) // CRA picture found
  {
    pocCRA = getPOC();
    associatedIRAPType = getNalUnitType();
  }
  else if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
         || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
         || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP ) // BLA picture found
  {
    pocCRA = getPOC();
    associatedIRAPType = getNalUnitType();
  }
}

/** Function for marking the reference pictures when an IDR/CRA/CRANT/BLA/BLANT is encountered.
 * \param pocCRA POC of the CRA/CRANT/BLA/BLANT picture
 * \param bRefreshPending flag indicating if a deferred decoding refresh is pending
 * \param rcListPic reference to the reference picture list
 * This function marks the reference pictures as "unused for reference" in the following conditions.
 * If the nal_unit_type is IDR/BLA/BLANT, all pictures in the reference picture list
 * are marked as "unused for reference"
 *    If the nal_unit_type is BLA/BLANT, set the pocCRA to the temporal reference of the current picture.
 * Otherwise
 *    If the bRefreshPending flag is true (a deferred decoding refresh is pending) and the current
 *    temporal reference is greater than the temporal reference of the latest CRA/CRANT/BLA/BLANT picture (pocCRA),
 *    mark all reference pictures except the latest CRA/CRANT/BLA/BLANT picture as "unused for reference" and set
 *    the bRefreshPending flag to false.
 *    If the nal_unit_type is CRA/CRANT, set the bRefreshPending flag to true and pocCRA to the temporal
 *    reference of the current picture.
 * Note that the current picture is already placed in the reference list and its marking is not changed.
 * If the current picture has a nal_ref_idc that is not 0, it will remain marked as "used for reference".
 */
Void Slice::decodingRefreshMarking(Int& pocCRA, Bool& bRefreshPending, PicList& rcListPic, const bool bEfficientFieldIRAPEnabled)
{
  Picture* rpcPic;
  Int      pocCurr = getPOC();

  if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP )  // IDR or BLA picture
  {
    // mark all pictures as not used for reference
    PicList::iterator        iterPic       = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic);
      if (rpcPic->getPOC() != pocCurr)
      {
        rpcPic->referenced = false;
      }
      iterPic++;
    }
    if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
    {
      pocCRA = pocCurr;
    }
    if (bEfficientFieldIRAPEnabled)
    {
      bRefreshPending = true;
    }
  }
  else // CRA or No DR
  {
    if(bEfficientFieldIRAPEnabled && (getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP || getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL))
    {
      if (bRefreshPending==true && pocCurr > m_iLastIDR) // IDR reference marking pending
      {
        PicList::iterator        iterPic       = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          rpcPic = *(iterPic);
          if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != m_iLastIDR)
          {
            rpcPic->referenced = false;
          }
          iterPic++;
        }
        bRefreshPending = false;
      }
    }
    else
    {
      if (bRefreshPending==true && pocCurr > pocCRA) // CRA reference marking pending
      {
        PicList::iterator iterPic = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          rpcPic = *(iterPic);
          if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != pocCRA)
          {
            rpcPic->referenced = false;
          }
          iterPic++;
        }
        bRefreshPending = false;
      }
    }
    if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ) // CRA picture found
    {
      bRefreshPending = true;
      pocCRA = pocCurr;
    }
  }
}

Void Slice::copySliceInfo(Slice *pSrc, bool cpyAlmostAll)
{
  CHECK(!pSrc, "Source is NULL");

  Int i, j, k;

  m_iPOC                 = pSrc->m_iPOC;
  m_eNalUnitType         = pSrc->m_eNalUnitType;
  m_eSliceType           = pSrc->m_eSliceType;
  m_iSliceQp             = pSrc->m_iSliceQp;
  m_iSliceQpBase         = pSrc->m_iSliceQpBase;
  m_ChromaQpAdjEnabled              = pSrc->m_ChromaQpAdjEnabled;
  m_deblockingFilterDisable         = pSrc->m_deblockingFilterDisable;
  m_deblockingFilterOverrideFlag    = pSrc->m_deblockingFilterOverrideFlag;
  m_deblockingFilterBetaOffsetDiv2  = pSrc->m_deblockingFilterBetaOffsetDiv2;
  m_deblockingFilterTcOffsetDiv2    = pSrc->m_deblockingFilterTcOffsetDiv2;

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i]     = pSrc->m_aiNumRefIdx[i];
  }

  for (i = 0; i < MAX_NUM_REF; i++)
  {
    m_list1IdxToList0Idx[i] = pSrc->m_list1IdxToList0Idx[i];
  }

  m_bCheckLDC             = pSrc->m_bCheckLDC;
  m_iSliceQpDelta        = pSrc->m_iSliceQpDelta;
  for (UInt component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_iSliceChromaQpDelta[component] = pSrc->m_iSliceChromaQpDelta[component];
  }
  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    for (j = 0; j < MAX_NUM_REF; j++)
    {
      m_apcRefPicList[i][j]  = pSrc->m_apcRefPicList[i][j];
      m_aiRefPOCList[i][j]   = pSrc->m_aiRefPOCList[i][j];
      m_bIsUsedAsLongTerm[i][j] = pSrc->m_bIsUsedAsLongTerm[i][j];
    }
    m_bIsUsedAsLongTerm[i][MAX_NUM_REF] = pSrc->m_bIsUsedAsLongTerm[i][MAX_NUM_REF];
  }
  if( cpyAlmostAll ) m_iDepth = pSrc->m_iDepth;

  // access channel
  if( cpyAlmostAll ) m_pRPS   = pSrc->m_pRPS;
  m_iLastIDR             = pSrc->m_iLastIDR;

  if( cpyAlmostAll ) m_pcPic  = pSrc->m_pcPic;

  m_colFromL0Flag        = pSrc->m_colFromL0Flag;
  m_colRefIdx            = pSrc->m_colRefIdx;

  if( cpyAlmostAll ) setLambdas(pSrc->getLambdas());

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    for (j = 0; j < MAX_NUM_REF; j++)
    {
      for (k =0; k < MAX_NUM_REF; k++)
      {
        m_abEqualRef[i][j][k] = pSrc->m_abEqualRef[i][j][k];
      }
    }
  }

  m_uiTLayer                      = pSrc->m_uiTLayer;
  m_bTLayerSwitchingFlag          = pSrc->m_bTLayerSwitchingFlag;

  m_sliceMode                     = pSrc->m_sliceMode;
  m_sliceArgument                 = pSrc->m_sliceArgument;
  m_sliceCurStartCtuTsAddr        = pSrc->m_sliceCurStartCtuTsAddr;
  m_sliceCurEndCtuTsAddr          = pSrc->m_sliceCurEndCtuTsAddr;
  m_independentSliceIdx           = pSrc->m_independentSliceIdx;
#if HEVC_DEPENDENT_SLICES
  m_sliceSegmentIdx               = pSrc->m_sliceSegmentIdx;
  m_sliceSegmentMode              = pSrc->m_sliceSegmentMode;
  m_sliceSegmentArgument          = pSrc->m_sliceSegmentArgument;
  m_sliceSegmentCurStartCtuTsAddr = pSrc->m_sliceSegmentCurStartCtuTsAddr;
  m_sliceSegmentCurEndCtuTsAddr   = pSrc->m_sliceSegmentCurEndCtuTsAddr;
#endif
  m_nextSlice                     = pSrc->m_nextSlice;
#if HEVC_DEPENDENT_SLICES
  m_nextSliceSegment              = pSrc->m_nextSliceSegment;
#endif
  m_clpRngs                       = pSrc->m_clpRngs;
  m_pendingRasInit                = pSrc->m_pendingRasInit;

  for ( UInt e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( UInt n=0 ; n<MAX_NUM_REF ; n++ )
    {
      memcpy(m_weightPredTable[e][n], pSrc->m_weightPredTable[e][n], sizeof(WPScalingParam)*MAX_NUM_COMPONENT );
    }
  }

  for( UInt ch = 0 ; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_saoEnabledFlag[ch] = pSrc->m_saoEnabledFlag[ch];
  }

  m_cabacInitFlag                 = pSrc->m_cabacInitFlag;
  m_cabacWinUpdateMode            = pSrc->m_cabacWinUpdateMode;

  m_bLMvdL1Zero                   = pSrc->m_bLMvdL1Zero;
  m_LFCrossSliceBoundaryFlag      = pSrc->m_LFCrossSliceBoundaryFlag;
  m_enableTMVPFlag                = pSrc->m_enableTMVPFlag;
  m_maxNumMergeCand               = pSrc->m_maxNumMergeCand;
  if( cpyAlmostAll ) m_encCABACTableIdx  = pSrc->m_encCABACTableIdx;
  m_uiMaxBTSize                   = pSrc->m_uiMaxBTSize;
}


/** Function for checking if this is a switching-point
*/
Bool Slice::isTemporalLayerSwitchingPoint(PicList& rcListPic) const
{
  // loop through all pictures in the reference picture buffer
  PicList::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    const Picture* pcPic = *(iterPic++);
    if( pcPic->referenced && pcPic->poc != getPOC())
    {
      if( pcPic->layer >= getTLayer())
      {
        return false;
      }
    }
  }
  return true;
}

/** Function for checking if this is a STSA candidate
 */
Bool Slice::isStepwiseTemporalLayerSwitchingPointCandidate(PicList& rcListPic) const
{
  PicList::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    const Picture* pcPic = *(iterPic++);
    if( pcPic->referenced &&  pcPic->usedByCurr && pcPic->poc != getPOC())
    {
      if( pcPic->layer >= getTLayer())
      {
        return false;
      }
    }
  }
  return true;
}


Void Slice::checkLeadingPictureRestrictions(PicList& rcListPic) const
{
  Int nalUnitType = this->getNalUnitType();

  // When a picture is a leading picture, it shall be a RADL or RASL picture.
  if(this->getAssociatedIRAPPOC() > this->getPOC())
  {
    // Do not check IRAP pictures since they may get a POC lower than their associated IRAP
    if(nalUnitType < NAL_UNIT_CODED_SLICE_BLA_W_LP ||
       nalUnitType > NAL_UNIT_RESERVED_IRAP_VCL23)
    {
      CHECK( nalUnitType != NAL_UNIT_CODED_SLICE_RASL_N &&
             nalUnitType != NAL_UNIT_CODED_SLICE_RASL_R &&
             nalUnitType != NAL_UNIT_CODED_SLICE_RADL_N &&
             nalUnitType != NAL_UNIT_CODED_SLICE_RADL_R, "Invalid NAL unit type");
    }
  }

  // When a picture is a trailing picture, it shall not be a RADL or RASL picture.
  if(this->getAssociatedIRAPPOC() < this->getPOC())
  {
    CHECK( nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
           nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R ||
           nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
           nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R, "Invalid NAL unit type" );
  }

  // No RASL pictures shall be present in the bitstream that are associated
  // with a BLA picture having nal_unit_type equal to BLA_W_RADL or BLA_N_LP.
  if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
     nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
  {
    CHECK (this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
           this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_N_LP, "Invalid NAL unit type");
  }

  // No RASL pictures shall be present in the bitstream that are associated with
  // an IDR picture.
  if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
     nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
  {
    CHECK( this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
           this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL, "Invalid NAL unit type");
  }

  // No RADL pictures shall be present in the bitstream that are associated with
  // a BLA picture having nal_unit_type equal to BLA_N_LP or that are associated
  // with an IDR picture having nal_unit_type equal to IDR_N_LP.
  if(nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
     nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R)
  {
    CHECK (this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_N_LP   ||
           this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP, "Invalid NAL unit type");
  }

  // loop through all pictures in the reference picture buffer
  PicList::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    Picture* pcPic = *(iterPic++);
    if( ! pcPic->reconstructed)
    {
      continue;
    }
    if( pcPic->poc == this->getPOC())
    {
      continue;
    }
    const Slice* pcSlice = pcPic->slices[0];

    // Any picture that has PicOutputFlag equal to 1 that precedes an IRAP picture
    // in decoding order shall precede the IRAP picture in output order.
    // (Note that any picture following in output order would be present in the DPB)
    if(pcSlice->getPicOutputFlag() == 1 && !this->getNoOutputPriorPicsFlag())
    {
      if(nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP    ||
         nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP    ||
         nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL  ||
         nalUnitType == NAL_UNIT_CODED_SLICE_CRA         ||
         nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP    ||
         nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL)
      {
        CHECK(pcPic->poc >= this->getPOC(), "Invalid POC");
      }
    }

    // Any picture that has PicOutputFlag equal to 1 that precedes an IRAP picture
    // in decoding order shall precede any RADL picture associated with the IRAP
    // picture in output order.
    if(pcSlice->getPicOutputFlag() == 1)
    {
      if((nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
          nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R))
      {
        // rpcPic precedes the IRAP in decoding order
        if(this->getAssociatedIRAPPOC() > pcSlice->getAssociatedIRAPPOC())
        {
          // rpcPic must not be the IRAP picture
          if(this->getAssociatedIRAPPOC() != pcPic->poc)
          {
            CHECK( pcPic->poc >= this->getPOC(), "Invalid POC");
          }
        }
      }
    }

    // When a picture is a leading picture, it shall precede, in decoding order,
    // all trailing pictures that are associated with the same IRAP picture.
      if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
         nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R ||
         nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
         nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R)
      {
        if(pcSlice->getAssociatedIRAPPOC() == this->getAssociatedIRAPPOC())
        {
          // rpcPic is a picture that preceded the leading in decoding order since it exist in the DPB
          // rpcPic would violate the constraint if it was a trailing picture
          CHECK( pcPic->poc > this->getAssociatedIRAPPOC(), "Invalid POC");
        }
      }

    // Any RASL picture associated with a CRA or BLA picture shall precede any
    // RADL picture associated with the CRA or BLA picture in output order
    if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
       nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
    {
      if((this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_N_LP   ||
          this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_W_LP   ||
          this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
          this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA)       &&
          this->getAssociatedIRAPPOC() == pcSlice->getAssociatedIRAPPOC())
      {
        if(pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N ||
           pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R)
        {
          CHECK( pcPic->poc <= this->getPOC(), "Invalid POC");
        }
      }
    }

    // Any RASL picture associated with a CRA picture shall follow, in output
    // order, any IRAP picture that precedes the CRA picture in decoding order.
    if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
       nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
    {
      if(this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA)
      {
        if(pcSlice->getPOC() < this->getAssociatedIRAPPOC() &&
           (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP   ||
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP   ||
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA))
        {
          CHECK(this->getPOC() <= pcSlice->getPOC(), "Invalid POC");
        }
      }
    }
  }
}



/** Function for applying picture marking based on the Reference Picture Set in pReferencePictureSet.
*/
Void Slice::applyReferencePictureSet( PicList& rcListPic, const ReferencePictureSet *pReferencePictureSet) const
{
  Int i, isReference;

  checkLeadingPictureRestrictions(rcListPic);

  // loop through all pictures in the reference picture buffer
  PicList::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    Picture* pcPic = *(iterPic++);

    if( ! pcPic->referenced)
    {
      continue;
    }

    isReference = 0;
    // loop through all pictures in the Reference Picture Set
    // to see if the picture should be kept as reference picture
    for(i=0;i<pReferencePictureSet->getNumberOfPositivePictures()+pReferencePictureSet->getNumberOfNegativePictures();i++)
    {
      if( ! pcPic->longTerm && pcPic->poc == this->getPOC() + pReferencePictureSet->getDeltaPOC(i))
      {
        isReference = 1;
        pcPic->usedByCurr = pReferencePictureSet->getUsed(i);
        pcPic->longTerm = false;
      }
    }
    for(;i<pReferencePictureSet->getNumberOfPictures();i++)
    {
      if(pReferencePictureSet->getCheckLTMSBPresent(i)==true)
      {
        if( pcPic->longTerm && pcPic->poc == pReferencePictureSet->getPOC(i))
        {
          isReference = 1;
          pcPic->usedByCurr = pReferencePictureSet->getUsed(i);
        }
      }
      else
      {
        Int pocCycle = 1 << pcPic->cs->sps->getBitsForPOC();
        Int curPoc = pcPic->poc & (pocCycle-1);
        Int refPoc = pReferencePictureSet->getPOC(i) & (pocCycle-1);
        if( pcPic->longTerm && curPoc == refPoc)
        {
          isReference = 1;
          pcPic->usedByCurr = pReferencePictureSet->getUsed(i);
        }
      }
    }
    // mark the picture as "unused for reference" if it is not in
    // the Reference Picture Set
    if( pcPic->poc != this->getPOC() && isReference == 0)
    {
      pcPic->referenced = false;
      pcPic->usedByCurr = false;
      pcPic->longTerm   = false;
    }

    // sanity checks
    if( pcPic->referenced)
    {
      //check that pictures of higher temporal layers are not used
      CHECK( pcPic->usedByCurr && !(pcPic->layer<=this->getTLayer()), "Invalid state");
      //check that pictures of higher or equal temporal layer are not in the RPS if the current picture is a TSA picture
      if( this->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_R || this->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N)
      {
        CHECK( !(pcPic->layer<this->getTLayer()), "Invalid state");
      }
      //check that pictures marked as temporal layer non-reference pictures are not used for reference
      if( pcPic->poc != this->getPOC() && (pcPic->layer == this->getTLayer()))
      {
        CHECK( pcPic->usedByCurr && pcPic->slices[0]->getTemporalLayerNonReferenceFlag(), "Invalid state");
      }
    }
  }
}

/** Function for applying picture marking based on the Reference Picture Set in pReferencePictureSet.
*/
Int Slice::checkThatAllRefPicsAreAvailable( PicList& rcListPic, const ReferencePictureSet *pReferencePictureSet, Bool printErrors, Int pocRandomAccess, Bool bUseRecoveryPoint) const
{
  Int atLeastOneUnabledByRecoveryPoint = 0;
  Int atLeastOneFlushedByPreviousIDR = 0;
  Picture* rpcPic;
  Int i, isAvailable;
  Int atLeastOneLost = 0;
  Int atLeastOneRemoved = 0;
  Int iPocLost = 0;

  // loop through all long-term pictures in the Reference Picture Set
  // to see if the picture should be kept as reference picture
  for(i=pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures();i<pReferencePictureSet->getNumberOfPictures();i++)
  {
    isAvailable = 0;
    // loop through all pictures in the reference picture buffer
    PicList::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      if(pReferencePictureSet->getCheckLTMSBPresent(i)==true)
      {
        if(rpcPic->longTerm && (rpcPic->getPOC()) == pReferencePictureSet->getPOC(i) && rpcPic->referenced)
        {
          if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
          {
            isAvailable = 0;
          }
          else
          {
            isAvailable = 1;
          }
        }
      }
      else
      {
        Int pocCycle = 1<<rpcPic->cs->sps->getBitsForPOC();
        Int curPoc = rpcPic->getPOC() & (pocCycle-1);
        Int refPoc = pReferencePictureSet->getPOC(i) & (pocCycle-1);
        if(rpcPic->longTerm && curPoc == refPoc && rpcPic->referenced)
        {
          if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
          {
            isAvailable = 0;
          }
          else
          {
            isAvailable = 1;
          }
        }
      }
    }
    // if there was no such long-term check the short terms
    if(!isAvailable)
    {
      iterPic = rcListPic.begin();
      while ( iterPic != rcListPic.end())
      {
        rpcPic = *(iterPic++);

        Int pocCycle = 1 << rpcPic->cs->sps->getBitsForPOC();
        Int curPoc = rpcPic->getPOC();
        Int refPoc = pReferencePictureSet->getPOC(i);
        if (!pReferencePictureSet->getCheckLTMSBPresent(i))
        {
          curPoc = curPoc & (pocCycle - 1);
          refPoc = refPoc & (pocCycle - 1);
        }

        if (rpcPic->referenced && curPoc == refPoc)
        {
          if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
          {
            isAvailable = 0;
          }
          else
          {
            isAvailable = 1;
            rpcPic->longTerm = true;
            break;
          }
        }
      }
    }
    // report that a picture is lost if it is in the Reference Picture Set
    // but not available as reference picture
    if(isAvailable == 0)
    {
      if (this->getPOC() + pReferencePictureSet->getDeltaPOC(i) >= pocRandomAccess)
      {
        if(!pReferencePictureSet->getUsed(i) )
        {
          if(printErrors)
          {
            msg( ERROR, "\nLong-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneRemoved = 1;
        }
        else
        {
          if(printErrors)
          {
            msg( ERROR, "\nLong-term reference picture with POC = %3d is lost or not correctly decoded!", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneLost = 1;
          iPocLost=this->getPOC() + pReferencePictureSet->getDeltaPOC(i);
        }
      }
      else if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess)
      {
        atLeastOneUnabledByRecoveryPoint = 1;
      }
      else if(bUseRecoveryPoint && (this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_N_LP || this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_W_RADL))
      {
        atLeastOneFlushedByPreviousIDR = 1;
      }
    }
  }
  // loop through all short-term pictures in the Reference Picture Set
  // to see if the picture should be kept as reference picture
  for(i=0;i<pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures();i++)
  {
    isAvailable = 0;
    // loop through all pictures in the reference picture buffer
    PicList::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);

      if( ! rpcPic->longTerm && rpcPic->getPOC() == this->getPOC() + pReferencePictureSet->getDeltaPOC(i) && rpcPic->referenced)
      {
        if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
        {
          isAvailable = 0;
        }
        else
        {
          isAvailable = 1;
        }
      }
    }
    // report that a picture is lost if it is in the Reference Picture Set
    // but not available as reference picture
    if(isAvailable == 0)
    {
      if (this->getPOC() + pReferencePictureSet->getDeltaPOC(i) >= pocRandomAccess)
      {
        if(!pReferencePictureSet->getUsed(i) )
        {
          if(printErrors)
          {
            msg( ERROR, "\nShort-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneRemoved = 1;
        }
        else
        {
          if(printErrors)
          {
            msg( ERROR, "\nShort-term reference picture with POC = %3d is lost or not correctly decoded!", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneLost = 1;
          iPocLost=this->getPOC() + pReferencePictureSet->getDeltaPOC(i);
        }
      }
      else if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess)
      {
        atLeastOneUnabledByRecoveryPoint = 1;
      }
      else if(bUseRecoveryPoint && (this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_N_LP || this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_W_RADL))
      {
        atLeastOneFlushedByPreviousIDR = 1;
      }
    }
  }

  if(atLeastOneUnabledByRecoveryPoint || atLeastOneFlushedByPreviousIDR)
  {
    return -1;
  }
  if(atLeastOneLost)
  {
    return iPocLost+1;
  }
  if(atLeastOneRemoved)
  {
    return -2;
  }
  else
  {
    return 0;
  }
}

/** Function for constructing an explicit Reference Picture Set out of the available pictures in a referenced Reference Picture Set
*/
Void Slice::createExplicitReferencePictureSetFromReference( PicList& rcListPic, const ReferencePictureSet *pReferencePictureSet, Bool isRAP, Int pocRandomAccess, Bool bUseRecoveryPoint, const Bool bEfficientFieldIRAPEnabled)
{
  Picture* rpcPic;
  Int i, j;
  Int k = 0;
  Int nrOfNegativePictures = 0;
  Int nrOfPositivePictures = 0;
  ReferencePictureSet* pLocalRPS = this->getLocalRPS();
  (*pLocalRPS)=ReferencePictureSet();

  Bool irapIsInRPS = false; // Used when bEfficientFieldIRAPEnabled==true

  // loop through all pictures in the Reference Picture Set
  for(i=0;i<pReferencePictureSet->getNumberOfPictures();i++)
  {
    j = 0;
    // loop through all pictures in the reference picture buffer
    PicList::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())
    {
      j++;
      rpcPic = *(iterPic++);

      if(rpcPic->getPOC() == this->getPOC() + pReferencePictureSet->getDeltaPOC(i) && rpcPic->referenced)
      {
        // This picture exists as a reference picture
        // and should be added to the explicit Reference Picture Set
        pLocalRPS->setDeltaPOC(k, pReferencePictureSet->getDeltaPOC(i));
        pLocalRPS->setUsed(k, pReferencePictureSet->getUsed(i) && (!isRAP));
        if (bEfficientFieldIRAPEnabled)
        {
          pLocalRPS->setUsed(k, pLocalRPS->getUsed(k) && !(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess) );
        }

        if(pLocalRPS->getDeltaPOC(k) < 0)
        {
          nrOfNegativePictures++;
        }
        else
        {
          if(bEfficientFieldIRAPEnabled && rpcPic->getPOC() == this->getAssociatedIRAPPOC() && this->getAssociatedIRAPPOC() == this->getPOC()+1)
          {
            irapIsInRPS = true;
          }
          nrOfPositivePictures++;
        }
        k++;
      }
    }
  }

  Bool useNewRPS = false;
  // if current picture is complimentary field associated to IRAP, add the IRAP to its RPS.
  if(bEfficientFieldIRAPEnabled && m_pcPic->fieldPic && !irapIsInRPS)
  {
    PicList::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
	  
      if(rpcPic->getPOC() == this->getAssociatedIRAPPOC() && this->getAssociatedIRAPPOC() == this->getPOC()+1)
      {
        pLocalRPS->setDeltaPOC(k, 1);
        pLocalRPS->setUsed(k, true);
        nrOfPositivePictures++;
        k ++;
        useNewRPS = true;
      }
    }
  }
  pLocalRPS->setNumberOfNegativePictures(nrOfNegativePictures);
  pLocalRPS->setNumberOfPositivePictures(nrOfPositivePictures);
  pLocalRPS->setNumberOfPictures(nrOfNegativePictures+nrOfPositivePictures);
  // This is a simplistic inter rps example. A smarter encoder will look for a better reference RPS to do the
  // inter RPS prediction with.  Here we just use the reference used by pReferencePictureSet.
  // If pReferencePictureSet is not inter_RPS_predicted, then inter_RPS_prediction is for the current RPS also disabled.
  if (!pReferencePictureSet->getInterRPSPrediction() || useNewRPS )
  {
    pLocalRPS->setInterRPSPrediction(false);
    pLocalRPS->setNumRefIdc(0);
  }
  else
  {
    Int rIdx =  this->getRPSidx() - pReferencePictureSet->getDeltaRIdxMinus1() - 1;
    Int deltaRPS = pReferencePictureSet->getDeltaRPS();
    const ReferencePictureSet* pcRefRPS = this->getSPS()->getRPSList()->getReferencePictureSet(rIdx);
    Int iRefPics = pcRefRPS->getNumberOfPictures();
    Int iNewIdc=0;
    for(i=0; i<= iRefPics; i++)
    {
      Int deltaPOC = ((i != iRefPics)? pcRefRPS->getDeltaPOC(i) : 0);  // check if the reference abs POC is >= 0
      Int iRefIdc = 0;
      for (j=0; j < pLocalRPS->getNumberOfPictures(); j++) // loop through the  pictures in the new RPS
      {
        if ( (deltaPOC + deltaRPS) == pLocalRPS->getDeltaPOC(j))
        {
          if (pLocalRPS->getUsed(j))
          {
            iRefIdc = 1;
          }
          else
          {
            iRefIdc = 2;
          }
        }
      }
      pLocalRPS->setRefIdc(i, iRefIdc);
      iNewIdc++;
    }
    pLocalRPS->setInterRPSPrediction(true);
    pLocalRPS->setNumRefIdc(iNewIdc);
    pLocalRPS->setDeltaRPS(deltaRPS);
    pLocalRPS->setDeltaRIdxMinus1(pReferencePictureSet->getDeltaRIdxMinus1() + this->getSPS()->getRPSList()->getNumberOfReferencePictureSets() - this->getRPSidx());
  }

  this->setRPS(pLocalRPS);
  this->setRPSidx(-1);
}

//! get AC and DC values for weighted pred
Void  Slice::getWpAcDcParam(const WPACDCParam *&wp) const
{
  wp = m_weightACDCParam;
}

//! init AC and DC values for weighted pred
Void  Slice::initWpAcDcParam()
{
  for(Int iComp = 0; iComp < MAX_NUM_COMPONENT; iComp++ )
  {
    m_weightACDCParam[iComp].iAC = 0;
    m_weightACDCParam[iComp].iDC = 0;
  }
}

//! get tables for weighted prediction
Void  Slice::getWpScaling( RefPicList e, Int iRefIdx, WPScalingParam *&wp ) const
{
  CHECK(e>=NUM_REF_PIC_LIST_01, "Invalid picture reference list");
  wp = (WPScalingParam*) m_weightPredTable[e][iRefIdx];
}

//! reset Default WP tables settings : no weight.
Void  Slice::resetWpScaling()
{
  for ( Int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( Int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( Int yuv=0 ; yuv<MAX_NUM_COMPONENT ; yuv++ )
      {
        WPScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        pwp->bPresentFlag      = false;
        pwp->uiLog2WeightDenom = 0;
        pwp->uiLog2WeightDenom = 0;
        pwp->iWeight           = 1;
        pwp->iOffset           = 0;
      }
    }
  }
}

//! init WP table
Void  Slice::initWpScaling(const SPS *sps)
{
  const Bool bUseHighPrecisionPredictionWeighting = sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag();
  for ( Int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( Int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( Int yuv=0 ; yuv<MAX_NUM_COMPONENT ; yuv++ )
      {
        WPScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        if ( !pwp->bPresentFlag )
        {
          // Inferring values not present :
          pwp->iWeight = (1 << pwp->uiLog2WeightDenom);
          pwp->iOffset = 0;
        }

        const Int offsetScalingFactor = bUseHighPrecisionPredictionWeighting ? 1 : (1 << (sps->getBitDepth(toChannelType(ComponentID(yuv)))-8));

        pwp->w      = pwp->iWeight;
        pwp->o      = pwp->iOffset * offsetScalingFactor; //NOTE: This value of the ".o" variable is never used - .o is set immediately before it gets used
        pwp->shift  = pwp->uiLog2WeightDenom;
        pwp->round  = (pwp->uiLog2WeightDenom>=1) ? (1 << (pwp->uiLog2WeightDenom-1)) : (0);
      }
    }
  }
}


void Slice::startProcessingTimer()
{
  m_iProcessingStartTime = clock();
}

void Slice::stopProcessingTimer()
{
  m_dProcessingTime += (double)(clock()-m_iProcessingStartTime) / CLOCKS_PER_SEC;
  m_iProcessingStartTime = 0;
}


unsigned Slice::getMinPictureDistance() const
{
  int minPicDist = MAX_INT;
  if( ! isIntra() )
  {
    const Int currPOC  = getPOC();
    for (Int refIdx = 0; refIdx < getNumRefIdx(REF_PIC_LIST_0); refIdx++)
    {
      minPicDist = std::min( minPicDist, std::abs(currPOC - getRefPic(REF_PIC_LIST_0, refIdx)->getPOC()));
    }
    if( getSliceType() == B_SLICE )
    {
      for (Int refIdx = 0; refIdx < getNumRefIdx(REF_PIC_LIST_1); refIdx++)
      {
        minPicDist = std::min( minPicDist, std::abs(currPOC - getRefPic(REF_PIC_LIST_0, refIdx)->getPOC()));
      }
    }
  }
  return (unsigned) minPicDist;
}

#if HEVC_VPS
// ------------------------------------------------------------------------------------------------
// Video parameter set (VPS)
// ------------------------------------------------------------------------------------------------
VPS::VPS()
: m_VPSId                     (  0)
, m_uiMaxTLayers              (  1)
, m_uiMaxLayers               (  1)
, m_bTemporalIdNestingFlag    (false)
, m_numHrdParameters          (  0)
, m_maxNuhReservedZeroLayerId (  0)
, m_hrdParameters             ()
, m_hrdOpSetIdx               ()
, m_cprmsPresentFlag          ()
{

  for( Int i = 0; i < MAX_TLAYER; i++)
  {
    m_numReorderPics[i] = 0;
    m_uiMaxDecPicBuffering[i] = 1;
    m_uiMaxLatencyIncrease[i] = 0;
  }
}

VPS::~VPS()
{
}
#endif

// ------------------------------------------------------------------------------------------------
// Sequence parameter set (SPS)
// ------------------------------------------------------------------------------------------------
SPSRExt::SPSRExt()
 : m_transformSkipRotationEnabledFlag   (false)
 , m_transformSkipContextEnabledFlag    (false)
// m_rdpcmEnabledFlag initialized below
 , m_extendedPrecisionProcessingFlag    (false)
 , m_intraSmoothingDisabledFlag         (false)
 , m_highPrecisionOffsetsEnabledFlag    (false)
 , m_persistentRiceAdaptationEnabledFlag(false)
 , m_cabacBypassAlignmentEnabledFlag    (false)
{
  for (UInt signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++)
  {
    m_rdpcmEnabledFlag[signallingModeIndex] = false;
  }
}


SPSNext::SPSNext( SPS& sps )
  : m_SPS                       ( sps )
  , m_NextEnabled               ( false )
  // disable all tool enabling flags by default
  , m_QTBT                      ( false )
  , m_LargeCTU                  ( false )
  , m_DisableMotionCompression  ( false )
  , m_MTTEnabled                ( false )
#if ENABLE_WPP_PARALLELISM
  , m_NextDQP                   ( false )
#endif

  // default values for additional parameters
  , m_CTUSize                   ( 0 )
  , m_minQT                     { 0, 0 }
  , m_maxBTDepth                { MAX_BT_DEPTH, MAX_BT_DEPTH_INTER, MAX_BT_DEPTH_C }
  , m_maxBTSize                 { MAX_BT_SIZE,  MAX_BT_SIZE_INTER,  MAX_BT_SIZE_C }
  , m_MTTMode                   ( 0 )
  // ADD_NEW_TOOL : (sps extension) add tool enabling flags here (with "false" as default values)
{
}


SPS::SPS()
: m_SPSId                     (  0)
#if HEVC_VPS
, m_VPSId                     (  0)
#endif
, m_chromaFormatIdc           (CHROMA_420)
, m_uiMaxTLayers              (  1)
// Structure
, m_picWidthInLumaSamples     (352)
, m_picHeightInLumaSamples    (288)
, m_log2MinCodingBlockSize    (  0)
, m_log2DiffMaxMinCodingBlockSize(0)
, m_uiMaxCUWidth              ( 32)
, m_uiMaxCUHeight             ( 32)
, m_uiMaxCodingDepth          (  3)
, m_bLongTermRefsPresent      (false)
, m_uiQuadtreeTULog2MaxSize   (  0)
, m_uiQuadtreeTULog2MinSize   (  0)
, m_uiQuadtreeTUMaxDepthInter (  0)
, m_uiQuadtreeTUMaxDepthIntra (  0)
// Tool list
, m_usePCM                    (false)
, m_pcmLog2MaxSize            (  5)
, m_uiPCMLog2MinSize          (  7)
, m_bPCMFilterDisableFlag     (false)
, m_uiBitsForPOC              (  8)
, m_numLongTermRefPicSPS      (  0)
, m_uiMaxTrSize               ( 32)
, m_bUseSAO                   (false)
, m_bTemporalIdNestingFlag    (false)
#if HEVC_USE_SCALING_LISTS
, m_scalingListEnabledFlag    (false)
#endif
#if HEVC_USE_INTRA_SMOOTHING_T32 || HEVC_USE_INTRA_SMOOTHING_T64
, m_useStrongIntraSmoothing   (false)
#endif
, m_vuiParametersPresentFlag  (false)
, m_vuiParameters             ()
, m_spsNextExtension          (*this)
{
  for(Int ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_bitDepths.recon[ch] = 8;
    m_pcmBitDepths[ch] = 8;
    m_qpBDOffset   [ch] = 0;
  }

  for ( Int i = 0; i < MAX_TLAYER; i++ )
  {
    m_uiMaxLatencyIncreasePlus1[i] = 0;
    m_uiMaxDecPicBuffering[i] = 1;
    m_numReorderPics[i]       = 0;
  }

  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_usedByCurrPicLtSPSFlag, 0, sizeof(m_usedByCurrPicLtSPSFlag));
}

SPS::~SPS()
{
  m_RPSList.destroy();
}

Void  SPS::createRPSList( Int numRPS )
{
  m_RPSList.destroy();
  m_RPSList.create(numRPS);
}



const Int SPS::m_winUnitX[]={1,2,2,1};
const Int SPS::m_winUnitY[]={1,2,1,1};

PPSRExt::PPSRExt()
: m_log2MaxTransformSkipBlockSize      (2)
, m_crossComponentPredictionEnabledFlag(false)
, m_diffCuChromaQpOffsetDepth          (0)
, m_chromaQpOffsetListLen              (0)
// m_ChromaQpAdjTableIncludingNullEntry initialized below
// m_log2SaoOffsetScale initialized below
{
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CbOffset = 0; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0. This is initialised here and never subsequently changed.
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CrOffset = 0;
  for(Int ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_log2SaoOffsetScale[ch] = 0;
  }
}

PPS::PPS()
: m_PPSId                            (0)
, m_SPSId                            (0)
, m_picInitQPMinus26                 (0)
, m_useDQP                           (false)
, m_bConstrainedIntraPred            (false)
, m_bSliceChromaQpFlag               (false)
, m_uiMaxCuDQPDepth                  (0)
, m_chromaCbQpOffset                 (0)
, m_chromaCrQpOffset                 (0)
, m_numRefIdxL0DefaultActive         (1)
, m_numRefIdxL1DefaultActive         (1)
, m_TransquantBypassEnabledFlag      (false)
, m_useTransformSkip                 (false)
#if HEVC_DEPENDENT_SLICES
, m_dependentSliceSegmentsEnabledFlag(false)
#endif
#if HEVC_TILES_WPP
, m_tilesEnabledFlag                 (false)
, m_entropyCodingSyncEnabledFlag     (false)
, m_loopFilterAcrossTilesEnabledFlag (true)
, m_uniformSpacingFlag               (false)
, m_numTileColumnsMinus1             (0)
, m_numTileRowsMinus1                (0)
#endif
#if HEVC_USE_SIGN_HIDING
, m_signDataHidingEnabledFlag        (false)
#endif
, m_cabacInitPresentFlag             (false)
, m_sliceHeaderExtensionPresentFlag  (false)
, m_loopFilterAcrossSlicesEnabledFlag(false)
, m_listsModificationPresentFlag     (0)
, m_numExtraSliceHeaderBits          (0)
, m_ppsRangeExtension                ()
, pcv                                (NULL)
{
}

PPS::~PPS()
{
  delete pcv;
}

ReferencePictureSet::ReferencePictureSet()
: m_numberOfPictures (0)
, m_numberOfNegativePictures (0)
, m_numberOfPositivePictures (0)
, m_numberOfLongtermPictures (0)
, m_interRPSPrediction (0)
, m_deltaRIdxMinus1 (0)
, m_deltaRPS (0)
, m_numRefIdc (0)
{
  ::memset( m_deltaPOC, 0, sizeof(m_deltaPOC) );
  ::memset( m_POC, 0, sizeof(m_POC) );
  ::memset( m_used, 0, sizeof(m_used) );
  ::memset( m_refIdc, 0, sizeof(m_refIdc) );
  ::memset( m_bCheckLTMSB, 0, sizeof(m_bCheckLTMSB) );
  ::memset( m_pocLSBLT, 0, sizeof(m_pocLSBLT) );
  ::memset( m_deltaPOCMSBCycleLT, 0, sizeof(m_deltaPOCMSBCycleLT) );
  ::memset( m_deltaPocMSBPresentFlag, 0, sizeof(m_deltaPocMSBPresentFlag) );
}

ReferencePictureSet::~ReferencePictureSet()
{
}

Void ReferencePictureSet::setUsed(Int bufferNum, Bool used)
{
  m_used[bufferNum] = used;
}

Void ReferencePictureSet::setDeltaPOC(Int bufferNum, Int deltaPOC)
{
  m_deltaPOC[bufferNum] = deltaPOC;
}

Void ReferencePictureSet::setNumberOfPictures(Int numberOfPictures)
{
  m_numberOfPictures = numberOfPictures;
}

Int ReferencePictureSet::getUsed(Int bufferNum) const
{
  return m_used[bufferNum];
}

Int ReferencePictureSet::getDeltaPOC(Int bufferNum) const
{
  return m_deltaPOC[bufferNum];
}

Int ReferencePictureSet::getNumberOfPictures() const
{
  return m_numberOfPictures;
}

Int ReferencePictureSet::getPOC(Int bufferNum) const
{
  return m_POC[bufferNum];
}

Void ReferencePictureSet::setPOC(Int bufferNum, Int POC)
{
  m_POC[bufferNum] = POC;
}

Bool ReferencePictureSet::getCheckLTMSBPresent(Int bufferNum) const
{
  return m_bCheckLTMSB[bufferNum];
}

Void ReferencePictureSet::setCheckLTMSBPresent(Int bufferNum, Bool b)
{
  m_bCheckLTMSB[bufferNum] = b;
}

//! set the reference idc value at uiBufferNum entry to the value of iRefIdc
Void ReferencePictureSet::setRefIdc(Int bufferNum, Int refIdc)
{
  m_refIdc[bufferNum] = refIdc;
}

//! get the reference idc value at uiBufferNum
Int  ReferencePictureSet::getRefIdc(Int bufferNum) const
{
  return m_refIdc[bufferNum];
}

/** Sorts the deltaPOC and Used by current values in the RPS based on the deltaPOC values.
 *  deltaPOC values are sorted with -ve values before the +ve values.  -ve values are in decreasing order.
 *  +ve values are in increasing order.
 * \returns Void
 */
Void ReferencePictureSet::sortDeltaPOC()
{
  // sort in increasing order (smallest first)
  for(Int j=1; j < getNumberOfPictures(); j++)
  {
    Int deltaPOC = getDeltaPOC(j);
    Bool used = getUsed(j);
    for (Int k=j-1; k >= 0; k--)
    {
      Int temp = getDeltaPOC(k);
      if (deltaPOC < temp)
      {
        setDeltaPOC(k+1, temp);
        setUsed(k+1, getUsed(k));
        setDeltaPOC(k, deltaPOC);
        setUsed(k, used);
      }
    }
  }
  // flip the negative values to largest first
  Int numNegPics = getNumberOfNegativePictures();
  for(Int j=0, k=numNegPics-1; j < numNegPics>>1; j++, k--)
  {
    Int deltaPOC = getDeltaPOC(j);
    Bool used = getUsed(j);
    setDeltaPOC(j, getDeltaPOC(k));
    setUsed(j, getUsed(k));
    setDeltaPOC(k, deltaPOC);
    setUsed(k, used);
  }
}

/** Prints the deltaPOC and RefIdc (if available) values in the RPS.
 *  A "*" is added to the deltaPOC value if it is Used bu current.
 * \returns Void
 */
Void ReferencePictureSet::printDeltaPOC() const
{
  DTRACE( g_trace_ctx, D_RPSINFO, "DeltaPOC = { " );
  for(Int j=0; j < getNumberOfPictures(); j++)
  {
    DTRACE( g_trace_ctx, D_RPSINFO, "%d%s ", getDeltaPOC( j ), ( getUsed( j ) == 1 ) ? "*" : "" );
  }
  if (getInterRPSPrediction())
  {
    DTRACE( g_trace_ctx, D_RPSINFO, "}, RefIdc = { " );
    for(Int j=0; j < getNumRefIdc(); j++)
    {
      DTRACE( g_trace_ctx, D_RPSINFO, "%d ", getRefIdc( j ) );
    }
  }
  DTRACE( g_trace_ctx, D_RPSINFO, "}\n" );
}

RefPicListModification::RefPicListModification()
: m_refPicListModificationFlagL0 (false)
, m_refPicListModificationFlagL1 (false)
{
  ::memset( m_RefPicSetIdxL0, 0, sizeof(m_RefPicSetIdxL0) );
  ::memset( m_RefPicSetIdxL1, 0, sizeof(m_RefPicSetIdxL1) );
}

RefPicListModification::~RefPicListModification()
{
}

#if HEVC_USE_SCALING_LISTS
ScalingList::ScalingList()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < SCALING_LIST_NUM; listId++)
    {
      m_scalingListCoef[sizeId][listId].resize(std::min<Int>(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId]));
    }
  }
}

/** set default quantization matrix to array
*/
Void ScalingList::setDefaultScalingList()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId=0;listId<SCALING_LIST_NUM;listId++)
    {
      processDefaultMatrix(sizeId, listId);
    }
  }
}
/** check if use default quantization matrix
 * \returns true if use default quantization matrix in all size
*/
Bool ScalingList::checkDefaultScalingList()
{
  UInt defaultCounter=0;

  for( UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++ )
  {
    for(UInt listId=0;listId<SCALING_LIST_NUM;listId++)
    {
      if( !::memcmp(getScalingListAddress(sizeId,listId), getScalingListDefaultAddress(sizeId, listId),sizeof(Int)*std::min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId])) // check value of matrix
     && ((sizeId < SCALING_LIST_16x16) || (getScalingListDC(sizeId,listId) == 16))) // check DC value
      {
        defaultCounter++;
      }
    }
  }

  return (defaultCounter == (SCALING_LIST_NUM * SCALING_LIST_SIZE_NUM )) ? false : true;
}

/** get scaling matrix from RefMatrixID
 * \param sizeId    size index
 * \param listId    index of input matrix
 * \param refListId index of reference matrix
 */
Void ScalingList::processRefMatrix( UInt sizeId, UInt listId , UInt refListId )
{
  ::memcpy(getScalingListAddress(sizeId, listId),((listId == refListId)? getScalingListDefaultAddress(sizeId, refListId): getScalingListAddress(sizeId, refListId)),sizeof(Int)*std::min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId]));
}

Void ScalingList::checkPredMode(UInt sizeId, UInt listId)
{
  Int predListStep = (sizeId == SCALING_LIST_32x32? (SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) : 1); // if 32x32, skip over chroma entries.

  for(Int predListIdx = (Int)listId ; predListIdx >= 0; predListIdx-=predListStep)
  {
    if( !::memcmp(getScalingListAddress(sizeId,listId),((listId == predListIdx) ?
      getScalingListDefaultAddress(sizeId, predListIdx): getScalingListAddress(sizeId, predListIdx)),sizeof(Int)*std::min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId])) // check value of matrix
     && ((sizeId < SCALING_LIST_16x16) || (getScalingListDC(sizeId,listId) == getScalingListDC(sizeId,predListIdx)))) // check DC value
    {
      setRefMatrixId(sizeId, listId, predListIdx);
      setScalingListPredModeFlag(sizeId, listId, false);
      return;
    }
  }
  setScalingListPredModeFlag(sizeId, listId, true);
}

static Void outputScalingListHelp(std::ostream &os)
{
  os << "The scaling list file specifies all matrices and their DC values; none can be missing,\n"
         "but their order is arbitrary.\n\n"
         "The matrices are specified by:\n"
         "<matrix name><unchecked data>\n"
         "  <value>,<value>,<value>,....\n\n"
         "  Line-feeds can be added arbitrarily between values, and the number of values needs to be\n"
         "  at least the number of entries for the matrix (superfluous entries are ignored).\n"
         "  The <unchecked data> is text on the same line as the matrix that is not checked\n"
         "  except to ensure that the matrix name token is unique. It is recommended that it is ' ='\n"
         "  The values in the matrices are the absolute values (0-255), not the delta values as\n"
         "  exchanged between the encoder and decoder\n\n"
         "The DC values (for matrix sizes larger than 8x8) are specified by:\n"
         "<matrix name>_DC<unchecked data>\n"
         "  <value>\n";

  os << "The permitted matrix names are:\n";
  for(UInt sizeIdc = 0; sizeIdc < SCALING_LIST_SIZE_NUM; sizeIdc++)
  {
    for(UInt listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      if ((sizeIdc!=SCALING_LIST_32x32) || (listIdc%(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) == 0))
      {
        os << "  " << MatrixType[sizeIdc][listIdc] << '\n';
      }
    }
  }
}

Void ScalingList::outputScalingLists(std::ostream &os) const
{
  for(UInt sizeIdc = 0; sizeIdc < SCALING_LIST_SIZE_NUM; sizeIdc++)
  {
    const UInt size = std::min(8,4<<(sizeIdc));
    for(UInt listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      if ((sizeIdc!=SCALING_LIST_32x32) || (listIdc%(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) == 0))
      {
        const Int *src = getScalingListAddress(sizeIdc, listIdc);
        os << (MatrixType[sizeIdc][listIdc]) << " =\n  ";
        for(UInt y=0; y<size; y++)
        {
          for(UInt x=0; x<size; x++, src++)
          {
            os << std::setw(3) << (*src) << ", ";
          }
          os << (y+1<size?"\n  ":"\n");
        }
        if(sizeIdc > SCALING_LIST_8x8)
        {
          os << MatrixType_DC[sizeIdc][listIdc] << " = \n  " << std::setw(3) << getScalingListDC(sizeIdc, listIdc) << "\n";
        }
        os << "\n";
      }
    }
  }
}

Bool ScalingList::xParseScalingList(const std::string &fileName)
{
  static const Int LINE_SIZE=1024;
  FILE *fp = NULL;
  TChar line[LINE_SIZE];

  if (fileName.empty())
  {
    msg( ERROR, "Error: no scaling list file specified. Help on scaling lists being output\n");
    outputScalingListHelp(std::cout);
    std::cout << "\n\nExample scaling list file using default values:\n\n";
    outputScalingLists(std::cout);
    return true;
  }
  else if ((fp = fopen(fileName.c_str(),"r")) == (FILE*)NULL)
  {
    msg( ERROR, "Error: cannot open scaling list file %s for reading\n", fileName.c_str());
    return true;
  }

  for(UInt sizeIdc = SCALING_LIST_FIRST_CODED; sizeIdc < SCALING_LIST_SIZE_NUM; sizeIdc++)
  {
    const UInt size = std::min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeIdc]);

    for(UInt listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      Int * const src = getScalingListAddress(sizeIdc, listIdc);

      if ((sizeIdc==SCALING_LIST_32x32) && (listIdc%(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) != 0)) // derive chroma32x32 from chroma16x16
      {
        const Int *srcNextSmallerSize = getScalingListAddress(sizeIdc-1, listIdc);
        for(UInt i=0; i<size; i++)
        {
          src[i] = srcNextSmallerSize[i];
        }
        setScalingListDC(sizeIdc,listIdc,(sizeIdc > SCALING_LIST_8x8) ? getScalingListDC(sizeIdc-1, listIdc) : src[0]);
      }
      else
      {
        {
          fseek(fp, 0, SEEK_SET);
          Bool bFound=false;
          while ((!feof(fp)) && (!bFound))
          {
            TChar *ret = fgets(line, LINE_SIZE, fp);
            TChar *findNamePosition= ret==NULL ? NULL : strstr(line, MatrixType[sizeIdc][listIdc]);
            // This could be a match against the DC string as well, so verify it isn't
            if (findNamePosition!= NULL && (MatrixType_DC[sizeIdc][listIdc]==NULL || strstr(line, MatrixType_DC[sizeIdc][listIdc])==NULL))
            {
              bFound=true;
            }
          }
          if (!bFound)
          {
            msg( ERROR, "Error: cannot find Matrix %s from scaling list file %s\n", MatrixType[sizeIdc][listIdc], fileName.c_str());
            return true;
          }
        }
        for (UInt i=0; i<size; i++)
        {
          Int data;
          if (fscanf(fp, "%d,", &data)!=1)
          {
            msg( ERROR, "Error: cannot read value #%d for Matrix %s from scaling list file %s at file position %ld\n", i, MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          if (data<0 || data>255)
          {
            msg( ERROR, "Error: QMatrix entry #%d of value %d for Matrix %s from scaling list file %s at file position %ld is out of range (0 to 255)\n", i, data, MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          src[i] = data;
        }

        //set DC value for default matrix check
        setScalingListDC(sizeIdc,listIdc,src[0]);

        if(sizeIdc > SCALING_LIST_8x8)
        {
          {
            fseek(fp, 0, SEEK_SET);
            Bool bFound=false;
            while ((!feof(fp)) && (!bFound))
            {
              TChar *ret = fgets(line, LINE_SIZE, fp);
              TChar *findNamePosition= ret==NULL ? NULL : strstr(line, MatrixType_DC[sizeIdc][listIdc]);
              if (findNamePosition!= NULL)
              {
                // This won't be a match against the non-DC string.
                bFound=true;
              }
            }
            if (!bFound)
            {
              msg( ERROR, "Error: cannot find DC Matrix %s from scaling list file %s\n", MatrixType_DC[sizeIdc][listIdc], fileName.c_str());
              return true;
            }
          }
          Int data;
          if (fscanf(fp, "%d,", &data)!=1)
          {
            msg( ERROR, "Error: cannot read DC %s from scaling list file %s at file position %ld\n", MatrixType_DC[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          if (data<0 || data>255)
          {
            msg( ERROR, "Error: DC value %d for Matrix %s from scaling list file %s at file position %ld is out of range (0 to 255)\n", data, MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          //overwrite DC value when size of matrix is larger than 16x16
          setScalingListDC(sizeIdc,listIdc,data);
        }
      }
    }
  }
//  std::cout << "\n\nRead scaling lists of:\n\n";
//  outputScalingLists(std::cout);

  fclose(fp);
  return false;
}


/** get default address of quantization matrix
 * \param sizeId size index
 * \param listId list index
 * \returns pointer of quantization matrix
 */
const Int* ScalingList::getScalingListDefaultAddress(UInt sizeId, UInt listId)
{
  const Int *src = 0;
  switch(sizeId)
  {
    case SCALING_LIST_2x2:
    case SCALING_LIST_4x4:
      src = g_quantTSDefault4x4;
      break;
    case SCALING_LIST_8x8:
    case SCALING_LIST_16x16:
    case SCALING_LIST_32x32:
    case SCALING_LIST_64x64:
    case SCALING_LIST_128x128:
      src = (listId < (SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) ) ? g_quantIntraDefault8x8 : g_quantInterDefault8x8;
      break;
    default:
      THROW( "Invalid scaling list" );
      src = NULL;
      break;
  }
  return src;
}

/** process of default matrix
 * \param sizeId size index
 * \param listId index of input matrix
 */
Void ScalingList::processDefaultMatrix(UInt sizeId, UInt listId)
{
  ::memcpy(getScalingListAddress(sizeId, listId),getScalingListDefaultAddress(sizeId,listId),sizeof(Int)*std::min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId]));
  setScalingListDC(sizeId,listId,SCALING_LIST_DC);
}

/** check DC value of matrix for default matrix signaling
 */
Void ScalingList::checkDcOfMatrix()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < SCALING_LIST_NUM; listId++)
    {
      //check default matrix?
      if(getScalingListDC(sizeId,listId) == 0)
      {
        processDefaultMatrix(sizeId, listId);
      }
    }
  }
}
#endif

ParameterSetManager::ParameterSetManager()
#if HEVC_VPS
: m_vpsMap(MAX_NUM_VPS)
, m_spsMap(MAX_NUM_SPS)
#else
: m_spsMap(MAX_NUM_SPS)
#endif
, m_ppsMap(MAX_NUM_PPS)
#if HEVC_VPS
, m_activeVPSId(-1)
#endif
, m_activeSPSId(-1)
{
}


ParameterSetManager::~ParameterSetManager()
{
}

//! activate a SPS from a active parameter sets SEI message
//! \returns true, if activation is successful
//Bool ParameterSetManager::activateSPSWithSEI(Int spsId)
//{
//  SPS *sps = m_spsMap.getPS(spsId);
//  if (sps)
//  {
//    Int vpsId = sps->getVPSId();
//    VPS *vps = m_vpsMap.getPS(vpsId);
//    if (vps)
//    {
//      m_activeVPS = *(vps);
//      m_activeSPS = *(sps);
//      return true;
//    }
//    else
//    {
//     msg( WARNING, "Warning: tried to activate SPS using an Active parameter sets SEI message. Referenced VPS does not exist.");
//    }
//  }
//  else
//  {
//    msg( WARNING, "Warning: tried to activate non-existing SPS using an Active parameter sets SEI message.");
//  }
//  return false;
//}

#if HEVC_VPS
//! activate a PPS and depending on isIDR parameter also SPS and VPS
#else
//! activate a PPS and depending on isIDR parameter also SPS
#endif
//! \returns true, if activation is successful
Bool ParameterSetManager::activatePPS(Int ppsId, Bool isIRAP)
{
  PPS *pps = m_ppsMap.getPS(ppsId);
  if (pps)
  {
    Int spsId = pps->getSPSId();
    if (!isIRAP && (spsId != m_activeSPSId ))
    {
      msg( WARNING, "Warning: tried to activate PPS referring to a inactive SPS at non-IDR.");
    }
    else
    {
      SPS *sps = m_spsMap.getPS(spsId);
      if (sps)
      {

#if HEVC_VPS
        Int vpsId = sps->getVPSId();
        if (!isIRAP && (vpsId != m_activeVPSId ))
        {
          msg( WARNING, "Warning: tried to activate PPS referring to a inactive VPS at non-IDR.");
        }
        else
        {
#endif
          m_spsMap.setActive(spsId);
#if HEVC_VPS
          VPS *vps =m_vpsMap.getPS(vpsId);
          if (vps)
          {
            m_activeVPSId = vpsId;
            m_activeSPSId = spsId;
            m_ppsMap.setActive(ppsId);
            return true;
          }
          else
          {
            msg( WARNING, "Warning: tried to activate PPS that refers to a non-existing VPS.");
          }
        }
#else
        m_activeSPSId = spsId;
        m_ppsMap.setActive(ppsId);
        return true;
#endif
      }
      else
      {
        msg( WARNING, "Warning: tried to activate a PPS that refers to a non-existing SPS.");
      }
    }
  }
  else
  {
    msg( WARNING, "Warning: tried to activate non-existing PPS.");
  }

  // Failed to activate if reach here.
  m_activeSPSId=-1;
#if HEVC_VPS
  m_activeVPSId=-1;
#endif
  return false;
}

template <>
Void ParameterSetMap<PPS>::setID(PPS* parameterSet, const Int psId)
{
  parameterSet->setPPSId(psId);
}

template <>
Void ParameterSetMap<SPS>::setID(SPS* parameterSet, const Int psId)
{
  parameterSet->setSPSId(psId);
}

ProfileTierLevel::ProfileTierLevel()
  : m_profileSpace    (0)
  , m_tierFlag        (Level::MAIN)
  , m_profileIdc      (Profile::NONE)
  , m_levelIdc        (Level::NONE)
  , m_progressiveSourceFlag  (false)
  , m_interlacedSourceFlag   (false)
  , m_nonPackedConstraintFlag(false)
  , m_frameOnlyConstraintFlag(false)
{
  ::memset(m_profileCompatibilityFlag, 0, sizeof(m_profileCompatibilityFlag));
}

PTL::PTL()
{
  ::memset(m_subLayerProfilePresentFlag, 0, sizeof(m_subLayerProfilePresentFlag));
  ::memset(m_subLayerLevelPresentFlag,   0, sizeof(m_subLayerLevelPresentFlag  ));
}

Void calculateParameterSetChangedFlag(Bool &bChanged, const std::vector<UChar> *pOldData, const std::vector<UChar> *pNewData)
{
  if (!bChanged)
  {
    if ((pOldData==0 && pNewData!=0) || (pOldData!=0 && pNewData==0))
    {
      bChanged=true;
    }
    else if (pOldData!=0 && pNewData!=0)
    {
      // compare the two
      if (pOldData->size() != pNewData->size())
      {
        bChanged=true;
      }
      else
      {
        const UChar *pNewDataArray=&(*pNewData)[0];
        const UChar *pOldDataArray=&(*pOldData)[0];
        if (memcmp(pOldDataArray, pNewDataArray, pOldData->size()))
        {
          bChanged=true;
        }
      }
    }
  }
}

//! \}

UInt PreCalcValues::getValIdx( const Slice &slice, const ChannelType chType ) const
{
  return slice.isIntra() ? ( ISingleTree ? 0 : ( chType << 1 ) ) : 1;
}

UInt PreCalcValues::getMaxBtDepth( const Slice &slice, const ChannelType chType ) const
{
  return maxBtDepth[getValIdx( slice, chType )];
}

UInt PreCalcValues::getMinBtSize( const Slice &slice, const ChannelType chType ) const
{
  return minBtSize[getValIdx( slice, chType )];
}

UInt PreCalcValues::getMaxBtSize( const Slice &slice, const ChannelType chType ) const
{
  return ( !slice.isIntra() || isLuma( chType ) || ISingleTree ) ? slice.getMaxBTSize() : MAX_BT_SIZE_C;
}

UInt PreCalcValues::getMinTtSize( const Slice &slice, const ChannelType chType ) const
{
  return minTtSize[getValIdx( slice, chType )];
}

UInt PreCalcValues::getMaxTtSize( const Slice &slice, const ChannelType chType ) const
{
  return maxTtSize[getValIdx( slice, chType )];
}
UInt PreCalcValues::getMinQtSize( const Slice &slice, const ChannelType chType ) const
{
  return minQtSize[getValIdx( slice, chType )];
}

#if ENABLE_TRACING
#if HEVC_VPS
void xTraceVPSHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Video Parameter Set     ===========\n" );
}
#endif

void xTraceSPSHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Sequence Parameter Set  ===========\n" );
}

void xTracePPSHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Picture Parameter Set  ===========\n" );
}

void xTraceSliceHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Slice ===========\n" );
}

void xTraceAccessUnitDelimiter()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Access Unit Delimiter ===========\n" );
}
#endif
