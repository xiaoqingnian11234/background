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

/** \file     EncSlice.cpp
    \brief    slice encoder class
*/

#include "EncSlice.h"

#include "EncLib.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/Picture.h"

#if ENABLE_WPP_PARALLELISM
#include <mutex>
extern recursive_mutex g_cache_mutex;
#endif

#include <math.h>

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncSlice::EncSlice()
 : m_encCABACTableIdx(I_SLICE)
{
}

EncSlice::~EncSlice()
{
  destroy();
}

Void EncSlice::create( Int iWidth, Int iHeight, ChromaFormat chromaFormat, UInt iMaxCUWidth, UInt iMaxCUHeight, UChar uhTotalDepth )
{
}

Void EncSlice::destroy()
{
  // free lambda and QP arrays
  m_vdRdPicLambda.clear();
  m_vdRdPicQp.clear();
  m_viRdPicQp.clear();
}

Void EncSlice::init( EncLib* pcEncLib, const SPS& sps )
{
  m_pcCfg             = pcEncLib;
  m_pcLib             = pcEncLib;
  m_pcListPic         = pcEncLib->getListPic();

  m_pcGOPEncoder      = pcEncLib->getGOPEncoder();
  m_pcCuEncoder       = pcEncLib->getCuEncoder();
  m_pcInterSearch     = pcEncLib->getInterSearch();
  m_CABACWriter       = pcEncLib->getCABACEncoder()->getCABACWriter   (&sps);
  m_CABACEstimator    = pcEncLib->getCABACEncoder()->getCABACEstimator(&sps);
  m_pcTrQuant         = pcEncLib->getTrQuant();
  m_pcRdCost          = pcEncLib->getRdCost();

  // create lambda and QP arrays
  m_vdRdPicLambda.resize(m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_vdRdPicQp.resize(    m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_viRdPicQp.resize(    m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_pcRateCtrl        = pcEncLib->getRateCtrl();
}

Void
EncSlice::setUpLambda( Slice* slice, const Double dLambda, Int iQP)
{
  // store lambda
  m_pcRdCost ->setLambda( dLambda, slice->getSPS()->getBitDepths() );

  // for RDO
  // in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
  Double dLambdas[MAX_NUM_COMPONENT] = { dLambda };
  for( UInt compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    const ComponentID compID = ComponentID( compIdx );
    Int chromaQPOffset       = slice->getPPS()->getQpOffset( compID ) + slice->getSliceChromaQpDelta( compID );
    Int qpc                  = ( iQP + chromaQPOffset < 0 ) ? iQP : getScaledChromaQP( iQP + chromaQPOffset, m_pcCfg->getChromaFormatIdc() );
    Double tmpWeight         = pow( 2.0, ( iQP - qpc ) / 3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
    m_pcRdCost->setDistortionWeight( compID, tmpWeight );
#if ENABLE_WPP_PARALLELISM
    for( int jId = 1; jId < ( m_pcLib->getNumWppThreads() + m_pcLib->getNumWppExtraLines() ); jId++ )
    {
      m_pcLib->getRdCost( slice->getPic()->scheduler.getWppDataId( jId ) )->setDistortionWeight( compID, tmpWeight );
    }
#endif
    dLambdas[compIdx] = dLambda / tmpWeight;
  }

#if RDOQ_CHROMA_LAMBDA
  // for RDOQ
  m_pcTrQuant->setLambdas( dLambdas );
#else
  m_pcTrQuant->setLambda( dLambda );
#endif

  // for SAO
  slice->setLambdas( dLambdas );
}



/**
 - non-referenced frame marking
 - QP computation based on temporal structure
 - lambda computation based on QP
 - set temporal layer ID and the parameter sets
 .
 \param pcPic         picture class
 \param pocLast       POC of last picture
 \param pocCurr       current POC
 \param iNumPicRcvd   number of received pictures
 \param iGOPid        POC offset for hierarchical structure
 \param rpcSlice      slice header class
 \param isField       true for field coding
 */

Void EncSlice::initEncSlice( Picture* pcPic, const Int pocLast, const Int pocCurr, const Int iGOPid, Slice*& rpcSlice, const Bool isField
#if ADJUST_QP
	,bool qpflag
#endif
)
{
  Double dQP;
  Double dLambda;

  rpcSlice = pcPic->slices[0];
  rpcSlice->setSliceBits(0);
  rpcSlice->setPic( pcPic );
  rpcSlice->initSlice();
  rpcSlice->setPicOutputFlag( true );
  rpcSlice->setPOC( pocCurr );

#if SHARP_LUMA_DELTA_QP
  pcPic->fieldPic = isField;
  m_gopID = iGOPid;
#endif

  // depth computation based on GOP size
  Int depth;
  {
    Int poc = rpcSlice->getPOC();
    if(isField)
    {
      poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
    }
    else
    {
      poc = poc % m_pcCfg->getGOPSize();
    }

    if ( poc == 0 )
    {
      depth = 0;
    }
    else
    {
      Int step = m_pcCfg->getGOPSize();
      depth    = 0;
      for( Int i=step>>1; i>=1; i>>=1 )
      {
        for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
        {
          if ( j == poc )
          {
            i=0;
            break;
          }
        }
        step >>= 1;
        depth++;
      }
    }

    if(m_pcCfg->getHarmonizeGopFirstFieldCoupleEnabled() && poc != 0)
    {
      if (isField && ((rpcSlice->getPOC() % 2) == 1))
      {
        depth++;
      }
    }
  }

  // slice type
  SliceType eSliceType;

  eSliceType=B_SLICE;
  if(!(isField && pocLast == 1) || !m_pcCfg->getEfficientFieldIRAPEnabled())
  {
    if(m_pcCfg->getDecodingRefreshType() == 3)
    {
      eSliceType = (pocLast == 0 || pocCurr % m_pcCfg->getIntraPeriod() == 0             || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }
    else
    {
      eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }
  }

  rpcSlice->setSliceType    ( eSliceType );

  // ------------------------------------------------------------------------------------------------------------------
  // Non-referenced frame marking
  // ------------------------------------------------------------------------------------------------------------------

  if(pocLast == 0)
  {
    rpcSlice->setTemporalLayerNonReferenceFlag(false);
  }
  else
  {
    rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);
  }
  pcPic->referenced = true;

  // ------------------------------------------------------------------------------------------------------------------
  // QP setting
  // ------------------------------------------------------------------------------------------------------------------

#if X0038_LAMBDA_FROM_QP_CAPABILITY

#if ADJUST_QP
  dQP = m_pcCfg->getQPForPicture(iGOPid, rpcSlice,qpflag);
#else
  dQP = m_pcCfg->getQPForPicture(iGOPid, rpcSlice);
#endif // ADJUST_QP

  
#else
  dQP = m_pcCfg->getBaseQP();
  if(eSliceType!=I_SLICE)
  {
#if SHARP_LUMA_DELTA_QP
    if (!(( m_pcCfg->getMaxDeltaQP() == 0) && (!m_pcCfg->getLumaLevelToDeltaQPMapping().isEnabled()) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnabledFlag())))
#else
    if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnabledFlag())))
#endif
    {
      dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
    }
  }

  // modify QP
  const Int* pdQPs = m_pcCfg->getdQPs();
  if ( pdQPs )
  {
    dQP += pdQPs[ rpcSlice->getPOC() ];
  }

  if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)
  {
    dQP=LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
    m_pcCfg->setDeltaQpRD(0);
  }
#endif

  // ------------------------------------------------------------------------------------------------------------------
  // Lambda computation
  // ------------------------------------------------------------------------------------------------------------------

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  const Int temporalId=m_pcCfg->getGOPEntry(iGOPid).m_temporalId;
#if !SHARP_LUMA_DELTA_QP
  const std::vector<Double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif
#endif
  Int iQP;
  Double dOrigQP = dQP;

  // pre-compute lambda and QP values for all possible QP candidates
  for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
  {
    // compute QP value
    dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
#if SHARP_LUMA_DELTA_QP
    dLambda = calculateLambda(rpcSlice, iGOPid, depth, dQP, dQP, iQP );
#else
    // compute lambda value
    Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
    Int    SHIFT_QP = 12;

#if FULL_NBIT
    Int    bitdepth_luma_qp_scale = 6 * (rpcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8);
#else
    Int    bitdepth_luma_qp_scale = 0;
#endif
    Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
    Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
    // Case #1: I or P-slices (key-frame)
    Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
    if ( eSliceType==I_SLICE )
    {
      if (m_pcCfg->getIntraQpFactor()>=0.0 && m_pcCfg->getGOPEntry(iGOPid).m_sliceType != I_SLICE)
      {
        dQPFactor=m_pcCfg->getIntraQpFactor();
      }
      else
      {
#if X0038_LAMBDA_FROM_QP_CAPABILITY
        if(m_pcCfg->getLambdaFromQPEnable())
        {
          dQPFactor=0.57;
        }
        else
        {
#endif
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );

        dQPFactor=0.57*dLambda_scale;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
        }
#endif
      }
    }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
    else if( m_pcCfg->getLambdaFromQPEnable() )
    {
      dQPFactor=0.57;
    }
#endif

    dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );

#if X0038_LAMBDA_FROM_QP_CAPABILITY
    if(!m_pcCfg->getLambdaFromQPEnable() && depth>0)
#else
    if ( depth>0 )
#endif
    {
#if FULL_NBIT
        dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
        dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
    }

    // if hadamard is used in ME process
    if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
    {
      dLambda *= 0.95;
    }

#if X0038_LAMBDA_FROM_QP_CAPABILITY
    Double lambdaModifier;
    if( rpcSlice->getSliceType( ) != I_SLICE || intraLambdaModifiers.empty())
    {
      lambdaModifier = m_pcCfg->getLambdaModifier( temporalId );
    }
    else
    {
      lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
    }
    dLambda *= lambdaModifier;
#endif

    iQP = max( -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
#endif

    m_vdRdPicLambda[iDQpIdx] = dLambda;
    m_vdRdPicQp    [iDQpIdx] = dQP;
    m_viRdPicQp    [iDQpIdx] = iQP;
  }

  // obtain dQP = 0 case
  dLambda = m_vdRdPicLambda[0];
  dQP     = m_vdRdPicQp    [0];
  iQP     = m_viRdPicQp    [0];

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
  const Int temporalId=m_pcCfg->getGOPEntry(iGOPid).m_temporalId;
  const std::vector<Double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif

#if W0038_CQP_ADJ
  if(rpcSlice->getPPS()->getSliceChromaQpFlag())
  {
    const Bool bUseIntraOrPeriodicOffset = rpcSlice->getSliceType()==I_SLICE || (m_pcCfg->getSliceChromaOffsetQpPeriodicity()!=0 && (rpcSlice->getPOC()%m_pcCfg->getSliceChromaOffsetQpPeriodicity())==0);
    Int cbQP = bUseIntraOrPeriodicOffset? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(false) : m_pcCfg->getGOPEntry(iGOPid).m_CbQPoffset;
    Int crQP = bUseIntraOrPeriodicOffset? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(true)  : m_pcCfg->getGOPEntry(iGOPid).m_CrQPoffset;

    cbQP = Clip3( -12, 12, cbQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb) ) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb);
    crQP = Clip3( -12, 12, crQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr) ) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr);
    rpcSlice->setSliceChromaQpDelta(COMPONENT_Cb, Clip3( -12, 12, cbQP));
    CHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb)<=12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb)>=-12), "Unspecified error");
    rpcSlice->setSliceChromaQpDelta(COMPONENT_Cr, Clip3( -12, 12, crQP));
    CHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr)<=12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr)>=-12), "Unspecified error");
  }
  else
  {
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
  }
#endif

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
  Double lambdaModifier;
  if( rpcSlice->getSliceType( ) != I_SLICE || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_pcCfg->getLambdaModifier( temporalId );
  }
  else
  {
    lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
  }

  dLambda *= lambdaModifier;
#endif

  setUpLambda(rpcSlice, dLambda, iQP);
  
#if WCG_EXT
  // cost = Distortion + Lambda*R,
  // when QP is adjusted by luma, distortion is changed, so we have to adjust lambda to match the distortion, then the cost function becomes
  // costA = Distortion + AdjustedLambda * R          -- currently, costA is still used when calculating intermediate cost of using SAD, HAD, resisual etc.
  // an alternative way is to weight the distortion to before the luma QP adjustment, then the cost function becomes
  // costB = weightedDistortion + Lambda * R          -- currently, costB is used to calculat final cost, and when DF_FUNC is DF_DEFAULT
  m_pcRdCost->saveUnadjustedLambda();
#endif

  if (m_pcCfg->getFastMEForGenBLowDelayEnabled())
  {
    // restore original slice type

    if(!(isField && pocLast == 1) || !m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if(m_pcCfg->getDecodingRefreshType() == 3)
      {
        eSliceType = (pocLast == 0 || (pocCurr)                     % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
      }
      else
      {
        eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
      }
    }

    rpcSlice->setSliceType        ( eSliceType );
  }

  if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
  {
    dQP = xGetQPValueAccordingToLambda( dLambda );
    iQP = max( -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
  }

  rpcSlice->setSliceQp           ( iQP );
  rpcSlice->setSliceQpDelta      ( 0 );
#if !W0038_CQP_ADJ
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
#endif
  rpcSlice->setUseChromaQpAdj( rpcSlice->getPPS()->getPpsRangeExtension().getChromaQpOffsetListEnabledFlag() );
  rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
  rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);

  if ( m_pcCfg->getDeblockingFilterMetric() )
  {
    rpcSlice->setDeblockingFilterOverrideFlag(true);
    rpcSlice->setDeblockingFilterDisable(false);
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
  }
  else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
  {
    rpcSlice->setDeblockingFilterOverrideFlag( rpcSlice->getPPS()->getDeblockingFilterOverrideEnabledFlag() );
    rpcSlice->setDeblockingFilterDisable( rpcSlice->getPPS()->getPPSDeblockingFilterDisabledFlag() );
    if ( !rpcSlice->getDeblockingFilterDisable())
    {
      if ( rpcSlice->getDeblockingFilterOverrideFlag() && eSliceType!=I_SLICE)
      {
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
      }
      else
      {
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
      }
    }
  }
  else
  {
    rpcSlice->setDeblockingFilterOverrideFlag( false );
    rpcSlice->setDeblockingFilterDisable( false );
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
  }

  rpcSlice->setDepth            ( depth );

  pcPic->layer =  temporalId;
  if(eSliceType==I_SLICE)
  {
    pcPic->layer = 0;
  }
  rpcSlice->setTLayer( pcPic->layer );

  rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
  rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
#if HEVC_DEPENDENT_SLICES
  rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );
  rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );
#endif
  rpcSlice->setMaxNumMergeCand      ( m_pcCfg->getMaxNumMergeCand()      );
  rpcSlice->setMaxBTSize            ( rpcSlice->isIntra() ? MAX_BT_SIZE : MAX_BT_SIZE_INTER );
}
#if ENCODE_BGPIC
Void EncSlice::setbgQPSlice(Slice*& rpcSlice, Int Q, Int iGOPid, bool isField, Double P)
{
	Double dQP;
	Double dLambda;

	Int i = 0, j = 0;

	// depth computation based on GOP size
	Int depth;
	{
		Int poc = rpcSlice->getPOC();
		if (isField)
		{
			poc = (poc / 2) % (m_pcCfg->getGOPSize() / 2);
		}
		else
		{
			poc = poc % m_pcCfg->getGOPSize();
		}

		if (poc == 0)
		{
			depth = 0;
		}
		else
		{
			Int step = m_pcCfg->getGOPSize();
			depth = 0;
			for (Int i = step >> 1; i >= 1; i >>= 1)
			{
				for (Int j = i; j<m_pcCfg->getGOPSize(); j += step)
				{
					if (j == poc)
					{
						i = 0;
						break;
					}
				}
				step >>= 1;
				depth++;
			}
		}

		if (m_pcCfg->getHarmonizeGopFirstFieldCoupleEnabled() && poc != 0)
		{
			if (isField && ((rpcSlice->getPOC() % 2) == 1))
			{
				depth++;
			}
		}
	}

	// ------------------------------------------------------------------------------------------------------------------
	// QP setting
	// ------------------------------------------------------------------------------------------------------------------

	dQP = m_pcCfg->getQPForPicture(iGOPid, rpcSlice
#if ADJUST_QP
		, false
#endif
	) - Q;

	// ------------------------------------------------------------------------------------------------------------------
	// Lambda computation
	// ------------------------------------------------------------------------------------------------------------------

#if X0038_LAMBDA_FROM_QP_CAPABILITY
	const Int temporalId = m_pcCfg->getGOPEntry(iGOPid).m_temporalId;
#if !SHARP_LUMA_DELTA_QP
	const std::vector<Double> &intraLambdaModifiers = m_pcCfg->getIntraLambdaModifier();
#endif
#endif
	Int iQP;
	Double dOrigQP = dQP;

	// pre-compute lambda and QP values for all possible QP candidates
	for (Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++)
	{
		// compute QP value
		dQP = dOrigQP + ((iDQpIdx + 1) >> 1)*(iDQpIdx % 2 ? -1 : 1);

		dLambda = calculateLambda(rpcSlice, iGOPid, depth, dQP, dQP, iQP);


		cout << "Lambda" << dLambda;
		double M = P;
		double beta = 0.5;
		double MM = M / (1 + (M - 1)*pow(M, beta));
		MM = pow(MM, 1 / beta);
		dLambda = dLambda*MM;
		cout << "Lambda" << dLambda;
		

		m_vdRdPicLambda[iDQpIdx] = dLambda;
		m_vdRdPicQp[iDQpIdx] = dQP;
		m_viRdPicQp[iDQpIdx] = iQP;
	}

	// obtain dQP = 0 case
	dLambda = m_vdRdPicLambda[0];
	dQP = m_vdRdPicQp[0];
	iQP = m_viRdPicQp[0];


#if W0038_CQP_ADJ
	if (rpcSlice->getPPS()->getSliceChromaQpFlag())
	{
		const Bool bUseIntraOrPeriodicOffset = rpcSlice->getSliceType() == I_SLICE || (m_pcCfg->getSliceChromaOffsetQpPeriodicity() != 0 && (rpcSlice->getPOC() % m_pcCfg->getSliceChromaOffsetQpPeriodicity()) == 0);
		Int cbQP = bUseIntraOrPeriodicOffset ? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(false) : m_pcCfg->getGOPEntry(iGOPid).m_CbQPoffset;
		Int crQP = bUseIntraOrPeriodicOffset ? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(true) : m_pcCfg->getGOPEntry(iGOPid).m_CrQPoffset;

		cbQP = Clip3(-12, 12, cbQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb)) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb);
		crQP = Clip3(-12, 12, crQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr)) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr);
		rpcSlice->setSliceChromaQpDelta(COMPONENT_Cb, Clip3(-12, 12, cbQP));
		CHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb) + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb) <= 12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb) + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb) >= -12), "Unspecified error");
		rpcSlice->setSliceChromaQpDelta(COMPONENT_Cr, Clip3(-12, 12, crQP));
		CHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr) + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr) <= 12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr) + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr) >= -12), "Unspecified error");
	}
	else
	{
		rpcSlice->setSliceChromaQpDelta(COMPONENT_Cb, 0);
		rpcSlice->setSliceChromaQpDelta(COMPONENT_Cr, 0);
	}
#endif


	setUpLambda(rpcSlice, dLambda, iQP);

	if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
	{
		dQP = xGetQPValueAccordingToLambda(dLambda);
		iQP = max(-rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), min(MAX_QP, (Int)floor(dQP + 0.5)));
	}

	rpcSlice->setSliceQp(iQP);
	rpcSlice->setSliceQpDelta(0);

}
Void EncSlice::resetQPSlice(Slice*& rpcSlice, Int Q, Int iGOPid, bool isField)
{
	Double dQP;
	Double dLambda;

	// depth computation based on GOP size
	Int depth;
	{
		Int poc = rpcSlice->getPOC();
		if (isField)
		{
			poc = (poc / 2) % (m_pcCfg->getGOPSize() / 2);
		}
		else
		{
			poc = poc % m_pcCfg->getGOPSize();
		}

		if (poc == 0)
		{
			depth = 0;
		}
		else
		{
			Int step = m_pcCfg->getGOPSize();
			depth = 0;
			for (Int i = step >> 1; i >= 1; i >>= 1)
			{
				for (Int j = i; j<m_pcCfg->getGOPSize(); j += step)
				{
					if (j == poc)
					{
						i = 0;
						break;
					}
				}
				step >>= 1;
				depth++;
			}
		}

		if (m_pcCfg->getHarmonizeGopFirstFieldCoupleEnabled() && poc != 0)
		{
			if (isField && ((rpcSlice->getPOC() % 2) == 1))
			{
				depth++;
			}
		}
	}

	// ------------------------------------------------------------------------------------------------------------------
	// QP setting
	// ------------------------------------------------------------------------------------------------------------------

	dQP = m_pcCfg->getQPForPicture(iGOPid, rpcSlice
#if ADJUST_QP
		, false
#endif
	) - Q;

	// ------------------------------------------------------------------------------------------------------------------
	// Lambda computation
	// ------------------------------------------------------------------------------------------------------------------

#if X0038_LAMBDA_FROM_QP_CAPABILITY
	const Int temporalId = m_pcCfg->getGOPEntry(iGOPid).m_temporalId;
#if !SHARP_LUMA_DELTA_QP
	const std::vector<Double> &intraLambdaModifiers = m_pcCfg->getIntraLambdaModifier();
#endif
#endif
	Int iQP;
	Double dOrigQP = dQP;

	// pre-compute lambda and QP values for all possible QP candidates
	for (Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++)
	{
		// compute QP value
		dQP = dOrigQP + ((iDQpIdx + 1) >> 1)*(iDQpIdx % 2 ? -1 : 1);

		dLambda = calculateLambda(rpcSlice, iGOPid, depth, dQP, dQP, iQP);


		m_vdRdPicLambda[iDQpIdx] = dLambda;
		m_vdRdPicQp[iDQpIdx] = dQP;
		m_viRdPicQp[iDQpIdx] = iQP;
	}
	// obtain dQP = 0 case
	dLambda = m_vdRdPicLambda[0];
	dQP = m_vdRdPicQp[0];
	iQP = m_viRdPicQp[0];


#if W0038_CQP_ADJ
	if (rpcSlice->getPPS()->getSliceChromaQpFlag())
	{
		const Bool bUseIntraOrPeriodicOffset = rpcSlice->getSliceType() == I_SLICE || (m_pcCfg->getSliceChromaOffsetQpPeriodicity() != 0 && (rpcSlice->getPOC() % m_pcCfg->getSliceChromaOffsetQpPeriodicity()) == 0);
		Int cbQP = bUseIntraOrPeriodicOffset ? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(false) : m_pcCfg->getGOPEntry(iGOPid).m_CbQPoffset;
		Int crQP = bUseIntraOrPeriodicOffset ? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(true) : m_pcCfg->getGOPEntry(iGOPid).m_CrQPoffset;

		cbQP = Clip3(-12, 12, cbQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb)) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb);
		crQP = Clip3(-12, 12, crQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr)) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr);
		rpcSlice->setSliceChromaQpDelta(COMPONENT_Cb, Clip3(-12, 12, cbQP));
		CHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb) + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb) <= 12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb) + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb) >= -12), "Unspecified error");
		rpcSlice->setSliceChromaQpDelta(COMPONENT_Cr, Clip3(-12, 12, crQP));
		CHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr) + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr) <= 12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr) + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr) >= -12), "Unspecified error");
	}
	else
	{
		rpcSlice->setSliceChromaQpDelta(COMPONENT_Cb, 0);
		rpcSlice->setSliceChromaQpDelta(COMPONENT_Cr, 0);
	}
#endif


	setUpLambda(rpcSlice, dLambda, iQP);

	if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
	{
		dQP = xGetQPValueAccordingToLambda(dLambda);
		iQP = max(-rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), min(MAX_QP, (Int)floor(dQP + 0.5)));
	}

	rpcSlice->setSliceQp(iQP);
	rpcSlice->setSliceQpDelta(0);

}
#endif

#if SHARP_LUMA_DELTA_QP
Double EncSlice::calculateLambda( const Slice*     slice,
                                  const Int        GOPid, // entry in the GOP table
                                  const Int        depth, // slice GOP hierarchical depth.
                                  const Double     refQP, // initial slice-level QP
                                  const Double     dQP,   // initial double-precision QP
                                        Int       &iQP )  // returned integer QP.
{
  enum   SliceType eSliceType    = slice->getSliceType();
  const  Bool      isField       = slice->getPic()->fieldPic;
  const  Int       NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
  const  Int       SHIFT_QP      = 12;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  const Int temporalId=m_pcCfg->getGOPEntry(GOPid).m_temporalId;
  const std::vector<Double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif

#if FULL_NBIT
  Int    bitdepth_luma_qp_scale = 6 * (slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8);
#else
  Int    bitdepth_luma_qp_scale = 0;
#endif
  Double qp_temp = dQP + bitdepth_luma_qp_scale - SHIFT_QP;
  // Case #1: I or P-slices (key-frame)
  Double dQPFactor = m_pcCfg->getGOPEntry(GOPid).m_QPFactor;
  if ( eSliceType==I_SLICE )
  {
    if (m_pcCfg->getIntraQpFactor()>=0.0 && m_pcCfg->getGOPEntry(GOPid).m_sliceType != I_SLICE)
    {
      dQPFactor=m_pcCfg->getIntraQpFactor();
    }
    else
    {
#if X0038_LAMBDA_FROM_QP_CAPABILITY
      if(m_pcCfg->getLambdaFromQPEnable())
      {
        dQPFactor=0.57;
      }
      else
      {
#endif
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
        dQPFactor=0.57*dLambda_scale;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
      }
#endif
    }
  }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  else if( m_pcCfg->getLambdaFromQPEnable() )
  {
    dQPFactor=0.57;
  }
#endif

  Double dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  if( !(m_pcCfg->getLambdaFromQPEnable()) && depth>0 )
#else
  if ( depth>0 )
#endif
  {
#if FULL_NBIT
      Double qp_temp_ref_orig = refQP - SHIFT_QP;
      dLambda *= Clip3( 2.00, 4.00, (qp_temp_ref_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
      Double qp_temp_ref = refQP + bitdepth_luma_qp_scale - SHIFT_QP;
      dLambda *= Clip3( 2.00, 4.00, (qp_temp_ref / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
  }

  // if hadamard is used in ME process
  if ( !m_pcCfg->getUseHADME() && slice->getSliceType( ) != I_SLICE )
  {
    dLambda *= 0.95;
  }

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  Double lambdaModifier;
  if( eSliceType != I_SLICE || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_pcCfg->getLambdaModifier( temporalId );
  }
  else
  {
    lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
  }
  dLambda *= lambdaModifier;
#endif

  iQP = max( -slice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );

  // NOTE: the lambda modifiers that are sometimes applied later might be best always applied in here.
  return dLambda;
}
#endif

Void EncSlice::resetQP( Picture* pic, Int sliceQP, Double lambda )
{
  Slice* slice = pic->slices[0];

  // store lambda
  slice->setSliceQp( sliceQP );
  setUpLambda(slice, lambda, sliceQP);
}

#if ENABLE_QPA
static inline Int apprI2Log2 (const double d)
{
  return d < 6.0e-20 ? -128 : Int(floor(2.0 * log(d) / log(2.0) + 0.5));
}

#ifndef HLM_L1_NORM
  #define HLM_L1_NORM
#endif

static Int filterAndCalculateAverageEnergies (const Pel* pSrc,     const Int  iSrcStride,
                                              double &hpEner,      const Int  iHeight,    const Int iWidth,
                                              const Int  iPOC = 0)
{
  Int iHpValue;
  UInt uHpERow, uHpEner = 0;

  // skip first row as there may be a black border frame
  pSrc += iSrcStride;
  // center rows
  for (Int y = 1; y < iHeight - 1; y++)
  {
    uHpERow = 0;
    // skip column as there may be a black border frame

    for (Int x = 1; x < iWidth - 1; x++) // and columns
    {
      iHpValue = 4 * (Int)pSrc[x] - (Int)pSrc[x-1] - (Int)pSrc[x+1] - (Int)pSrc[x-iSrcStride] - (Int)pSrc[x+iSrcStride];
#ifdef HLM_L1_NORM
      uHpERow += abs (iHpValue);
#else
      uHpERow += iHpValue * iHpValue;
#endif
    }
    // skip column as there may be a black border frame
#ifdef HLM_L1_NORM
    uHpEner += uHpERow;
#else
    uHpEner += (uHpERow + 64) >> 7; // avoids overflows
#endif
    pSrc += iSrcStride;
  }
  // skip last row as there may be a black border frame

  hpEner = double(uHpEner) / double((iWidth - 2) * (iHeight - 2));
#ifdef HLM_L1_NORM
  hpEner *= hpEner;
#endif
  // lower limit, compensate for highpass amplification
  if (hpEner < 64.0) hpEner = 64.0;

  if (iPOC  <= 0) return 0;
  return 1; // OK
}

#ifdef HLM_L1_NORM
  #undef HLM_L1_NORM
#endif

#if ENABLE_QPA
static bool applyQPAdaptation (Picture* const pcPic, Slice* const pcSlice,    const PreCalcValues& pcv,
                               const UInt startAddr, const UInt boundingAddr, const bool useSharpLumaDQP,
                               const int gopSize,    const double hpEnerAvg,  const double hpEnerMax)
{
  const int  iBitDepth   = pcSlice->getSPS()->getBitDepth (CHANNEL_TYPE_LUMA);
  const int  iQPIndex    = pcSlice->getSliceQp(); // initial QP index for current slice, used in following loops
#if HEVC_TILES_WPP
  const TileMap& tileMap = *pcPic->tileMap;
#endif
  bool   sliceQPModified = false;
  double hpEnerPic = 1.0 / (1.5 * double(1 << iBitDepth)); // speedup: multiply instead of divide in loops below

  if (pcv.lumaWidth > 2048 && pcv.lumaHeight > 1280) // for UHD/4K
  {
    hpEnerPic *= 1.5;
  }

  if ((pcPic->getPOC() & 1) && (iQPIndex >= MAX_QP))
  {
    int iQPFixed = Clip3 (0, MAX_QP, iQPIndex + ((apprI2Log2 (hpEnerAvg * hpEnerPic) + apprI2Log2 (hpEnerMax * hpEnerPic) + 1) >> 1)); // adapted slice QP = (mean(QP) + max(QP)) / 2
#if SHARP_LUMA_DELTA_QP

    // change new fixed QP based on average CTU luma value (Sharp)
    if (useSharpLumaDQP)
    {
      UInt64 uAvgLuma = 0;

      for (UInt ctuTsAddr = startAddr; ctuTsAddr < boundingAddr; ctuTsAddr++)
      {
#if HEVC_TILES_WPP
        const UInt ctuRsAddr = tileMap.getCtuTsToRsAddrMap (ctuTsAddr);
#else
        const UInt ctuRsAddr = ctuTsAddr;
#endif

        uAvgLuma += (UInt64)pcPic->m_iOffsetCtu[ctuRsAddr];
      }
      uAvgLuma = (uAvgLuma + ((boundingAddr - startAddr) >> 1)) / (boundingAddr - startAddr);

      iQPFixed = Clip3 (0, MAX_QP, iQPFixed + 1 - int((3 * uAvgLuma * uAvgLuma) >> UInt64(2 * iBitDepth - 1)));
    }
#endif

    if (iQPFixed < iQPIndex) iQPFixed = iQPIndex;
    else
    if (iQPFixed > iQPIndex)
    {
      const double* oldLambdas = pcSlice->getLambdas();
      const double  corrFactor = pow (2.0, double(iQPFixed - iQPIndex) / 3.0);
      const double  newLambdas[MAX_NUM_COMPONENT] = {oldLambdas[0] * corrFactor, oldLambdas[1] * corrFactor, oldLambdas[2] * corrFactor};

      CHECK (iQPIndex != pcSlice->getSliceQpBase(), "Invalid slice QP!");
      pcSlice->setLambdas (newLambdas);
      pcSlice->setSliceQp (iQPFixed); // update the slice/base QPs
      pcSlice->setSliceQpBase (iQPFixed);

      sliceQPModified = true;
    }

    for (UInt ctuTsAddr = startAddr; ctuTsAddr < boundingAddr; ctuTsAddr++)
    {
#if HEVC_TILES_WPP
      const UInt ctuRsAddr = tileMap.getCtuTsToRsAddrMap (ctuTsAddr);
#else
      const UInt ctuRsAddr = ctuTsAddr;
#endif

      pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPFixed; // fixed QPs
    }
  }
  else
  {
    for (UInt ctuTsAddr = startAddr; ctuTsAddr < boundingAddr; ctuTsAddr++)
    {
#if HEVC_TILES_WPP
      const UInt ctuRsAddr = tileMap.getCtuTsToRsAddrMap (ctuTsAddr);
#else
      const UInt ctuRsAddr = ctuTsAddr;
#endif

      int iQPAdapt = Clip3 (0, MAX_QP, iQPIndex + apprI2Log2 (pcPic->m_uEnerHpCtu[ctuRsAddr] * hpEnerPic));

#if SHARP_LUMA_DELTA_QP
      if ((pcv.widthInCtus > 1) && (gopSize > 1)) // try to enforce CTU SNR greater than zero dB
#else
      if ((!pcSlice->isIntra()) && (gopSize > 1)) // try to enforce CTU SNR greater than zero dB
#endif
      {
        const Pel      dcOffset   = pcPic->m_iOffsetCtu[ctuRsAddr];
#if SHARP_LUMA_DELTA_QP

        // change adaptive QP based on mean CTU luma value (Sharp)
        if (useSharpLumaDQP)
        {
          const UInt64 uAvgLuma   = (UInt64)dcOffset;

          iQPAdapt = max (0, iQPAdapt + 1 - int((3 * uAvgLuma * uAvgLuma) >> UInt64(2 * iBitDepth - 1)));
        }

#endif
        const UInt     uRefScale  = g_invQuantScales[iQPAdapt % 6] << ((iQPAdapt / 6) + iBitDepth - (pcSlice->isIntra() ? 4 : 3));
        const CompArea subArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area ((ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight, pcv.maxCUWidth, pcv.maxCUHeight)), pcPic->Y());
        const Pel*     pSrc       = pcPic->getOrigBuf (subArea).buf;
        const SizeType iSrcStride = pcPic->getOrigBuf (subArea).stride;
        const SizeType iSrcHeight = pcPic->getOrigBuf (subArea).height;
        const SizeType iSrcWidth  = pcPic->getOrigBuf (subArea).width;
        UInt uAbsDCless = 0;

        // compute sum of absolute DC-less (high-pass) luma values
        for (SizeType h = 0; h < iSrcHeight; h++)
        {
          for (SizeType w = 0; w < iSrcWidth; w++)
          {
            uAbsDCless += (UInt)abs (pSrc[w] - dcOffset);
          }
          pSrc += iSrcStride;
        }

        if (iSrcHeight >= 64 || iSrcWidth >= 64)  // normalization
        {
          const UInt64 blockSize = UInt64(iSrcWidth * iSrcHeight);

          uAbsDCless = UInt((UInt64(uAbsDCless) * 64*64 + (blockSize >> 1)) / blockSize);
        }

        if (uAbsDCless < 64*64) uAbsDCless = 64*64;  // limit to 1

        // reduce QP index if CTU would be fully quantized to zero
        if (uAbsDCless < uRefScale)
        {
          const int limit  = min (0, ((iQPIndex + 4) >> 3) - 6);
          const int redVal = max (limit, apprI2Log2 ((double)uAbsDCless / (double)uRefScale));

          iQPAdapt = max (0, iQPAdapt + redVal);
        }
#if SHARP_LUMA_DELTA_QP

        if (iQPAdapt > MAX_QP) iQPAdapt = MAX_QP;
#endif
      }

      pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPAdapt; // adapted QPs

      if ((pcv.widthInCtus > 1) && (gopSize > 1)) // try to reduce local bitrate peaks via minimum smoothing
      {
        iQPAdapt = ctuRsAddr % pcv.widthInCtus; // horizontal offset
        if (iQPAdapt == 0)
        {
          iQPAdapt = (ctuRsAddr > 1) ? pcPic->m_iOffsetCtu[ctuRsAddr - 2] : 0;
        }
        else // iQPAdapt >= 1
        {
          iQPAdapt = (iQPAdapt > 1) ? min (pcPic->m_iOffsetCtu[ctuRsAddr - 2], pcPic->m_iOffsetCtu[ctuRsAddr]) : pcPic->m_iOffsetCtu[ctuRsAddr];
        }
        if (ctuRsAddr > pcv.widthInCtus)
        {
          iQPAdapt = min (iQPAdapt, (int)pcPic->m_iOffsetCtu[ctuRsAddr - 1 - pcv.widthInCtus]); // min(L, T)
        }
        if ((ctuRsAddr > 0) && (pcPic->m_iOffsetCtu[ctuRsAddr - 1] < (Pel)iQPAdapt))
        {
          pcPic->m_iOffsetCtu[ctuRsAddr - 1] = (Pel)iQPAdapt;
        }
        if ((ctuTsAddr == boundingAddr - 1) && (ctuRsAddr > pcv.widthInCtus)) // last CTU in the given slice
        {
          iQPAdapt = min (pcPic->m_iOffsetCtu[ctuRsAddr - 1], pcPic->m_iOffsetCtu[ctuRsAddr - pcv.widthInCtus]);
          if (pcPic->m_iOffsetCtu[ctuRsAddr] < (Pel)iQPAdapt)
          {
            pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPAdapt;
          }
        }
      }
    } // end iteration over all CTUs in current slice
  }

  return sliceQPModified;
}
#endif // ENABLE_QPA

#endif // ENABLE_QPA || ENABLE_PRIVATE

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

//! set adaptive search range based on poc difference
Void EncSlice::setSearchRange( Slice* pcSlice )
{
  Int iCurrPOC = pcSlice->getPOC();
  Int iRefPOC;
  Int iGOPSize = m_pcCfg->getGOPSize();
  Int iOffset = (iGOPSize >> 1);
  Int iMaxSR = m_pcCfg->getSearchRange();
  Int iNumPredDir = pcSlice->isInterP() ? 1 : 2;

  for (Int iDir = 0; iDir < iNumPredDir; iDir++)
  {
    RefPicList  e = ( iDir ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for (Int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(e); iRefIdx++)
    {
      iRefPOC = pcSlice->getRefPic(e, iRefIdx)->getPOC();
      Int newSearchRange = Clip3(m_pcCfg->getMinSearchWindow(), iMaxSR, (iMaxSR*ADAPT_SR_SCALE*abs(iCurrPOC - iRefPOC)+iOffset)/iGOPSize);
      m_pcInterSearch->setAdaptiveSearchRange(iDir, iRefIdx, newSearchRange);
#if ENABLE_WPP_PARALLELISM
      for( int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++ )
      {
        m_pcLib->getInterSearch( jId )->setAdaptiveSearchRange( iDir, iRefIdx, newSearchRange );
      }
#endif
    }
  }
}

/**
 Multi-loop slice encoding for different slice QP

 \param pcPic    picture class
 */
Void EncSlice::precompressSlice( Picture* pcPic )
{
  // if deltaQP RD is not used, simply return
  if ( m_pcCfg->getDeltaQpRD() == 0 )
  {
    return;
  }

  if ( m_pcCfg->getUseRateCtrl() )
  {
    THROW("\nMultiple QP optimization is not allowed when rate control is enabled." );
  }

  Slice* pcSlice        = pcPic->slices[getSliceSegmentIdx()];

#if HEVC_DEPENDENT_SLICES
  if (pcSlice->getDependentSliceSegmentFlag())
  {
    // if this is a dependent slice segment, then it was optimised
    // when analysing the entire slice.
    return;
  }
#endif

  if (pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES)
  {
    // TODO: investigate use of average cost per CTU so that this Slice Mode can be used.
    THROW( "Unable to optimise Slice-level QP if Slice Mode is set to FIXED_NUMBER_OF_BYTES\n" );
  }

  Double     dPicRdCostBest = MAX_DOUBLE;
  UInt       uiQpIdxBest = 0;

  Double dFrameLambda;
#if FULL_NBIT
  Int    SHIFT_QP = 12 + 6 * (pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8);
#else
  Int    SHIFT_QP = 12;
#endif

  // set frame lambda
  if (m_pcCfg->getGOPSize() > 1)
  {
    dFrameLambda = 0.68 * pow (2, (m_viRdPicQp[0]  - SHIFT_QP) / 3.0) * (pcSlice->isInterB()? 2 : 1);
  }
  else
  {
    dFrameLambda = 0.68 * pow (2, (m_viRdPicQp[0] - SHIFT_QP) / 3.0);
  }

  // for each QP candidate
  for ( UInt uiQpIdx = 0; uiQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; uiQpIdx++ )
  {
    pcSlice       ->setSliceQp             ( m_viRdPicQp    [uiQpIdx] );
    setUpLambda(pcSlice, m_vdRdPicLambda[uiQpIdx], m_viRdPicQp    [uiQpIdx]);

    // try compress
	cout << "inprecom" << endl;
    compressSlice   ( pcPic, true, m_pcCfg->getFastDeltaQp());

    UInt64 uiPicDist        = m_uiPicDist; // Distortion, as calculated by compressSlice.
    // NOTE: This distortion is the chroma-weighted SSE distortion for the slice.
    //       Previously a standard SSE distortion was calculated (for the entire frame).
    //       Which is correct?
#if W0038_DB_OPT
    // TODO: Update loop filter, SAO and distortion calculation to work on one slice only.
    // uiPicDist = m_pcGOPEncoder->preLoopFilterPicAndCalcDist( pcPic );
#endif
    // compute RD cost and choose the best
    double dPicRdCost = double( uiPicDist ) + dFrameLambda * double( m_uiPicTotalBits );

    if ( dPicRdCost < dPicRdCostBest )
    {
      uiQpIdxBest    = uiQpIdx;
      dPicRdCostBest = dPicRdCost;
    }
  }

  // set best values
  pcSlice       ->setSliceQp             ( m_viRdPicQp    [uiQpIdxBest] );
  setUpLambda(pcSlice, m_vdRdPicLambda[uiQpIdxBest], m_viRdPicQp    [uiQpIdxBest]);
}

Void EncSlice::calCostSliceI(Picture* pcPic) // TODO: this only analyses the first slice segment. What about the others?
{
  Double         iSumHadSlice      = 0;
  Slice * const  pcSlice           = pcPic->slices[getSliceSegmentIdx()];
#if HEVC_TILES_WPP
  const TileMap &tileMap           = *pcPic->tileMap;
#endif
  const PreCalcValues& pcv         = *pcPic->cs->pcv;
  const SPS     &sps               = *(pcSlice->getSPS());
  const Int      shift             = sps.getBitDepth(CHANNEL_TYPE_LUMA)-8;
  const Int      offset            = (shift>0)?(1<<(shift-1)):0;

#if HEVC_DEPENDENT_SLICES
  pcSlice->setSliceSegmentBits(0);
#endif

  UInt startCtuTsAddr, boundingCtuTsAddr;
  xDetermineStartAndBoundingCtuTsAddr ( startCtuTsAddr, boundingCtuTsAddr, pcPic );

#if HEVC_TILES_WPP
  for( UInt ctuTsAddr = startCtuTsAddr, ctuRsAddr = tileMap.getCtuTsToRsAddrMap( startCtuTsAddr);
       ctuTsAddr < boundingCtuTsAddr;
       ctuRsAddr = tileMap.getCtuTsToRsAddrMap(++ctuTsAddr) )
#else
  for( UInt ctuTsAddr = startCtuTsAddr, ctuRsAddr = startCtuTsAddr;
       ctuTsAddr < boundingCtuTsAddr;
       ctuRsAddr = ++ctuTsAddr )
#endif
  {
    Position pos( (ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight);

    const Int height  = std::min( pcv.maxCUHeight, pcv.lumaHeight - pos.y );
    const Int width   = std::min( pcv.maxCUWidth,  pcv.lumaWidth  - pos.x );
    const CompArea blk( COMPONENT_Y, pcv.chrFormat, pos, Size( width, height));
    Int iSumHad = m_pcCuEncoder->updateCtuDataISlice( pcPic->getOrigBuf( blk ) );

    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra=(iSumHad+offset)>>shift;
    iSumHadSlice += (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra;

  }
  m_pcRateCtrl->getRCPic()->setTotalIntraCost(iSumHadSlice);
}

/** \param pcPic   picture class
 */
#if BLOCK_RDO
Void EncSlice::compressSliceRDO(Picture* pcPic, const Bool bCompressEntireSlice, const Bool bFastDeltaQP, Int bgBlock[], Double BlockDPP[])
{
	// if bCompressEntireSlice is true, then the entire slice (not slice segment) is compressed,
	//   effectively disabling the slice-segment-mode.

#if GENERATE_BG_PIC
	Picture* bg_NewPicYuvRecCU = getNewPicYuvRecSli();
	m_pcCuEncoder->setbgNewPicYuvRecCU(bg_NewPicYuvRecCU);
#endif
	Slice* const pcSlice = pcPic->slices[getSliceSegmentIdx()];
#if HEVC_TILES_WPP
	const TileMap&  tileMap = *pcPic->tileMap;
#endif
	UInt  startCtuTsAddr;
	UInt  boundingCtuTsAddr;

#if HEVC_DEPENDENT_SLICES
	pcSlice->setSliceSegmentBits(0);
#endif
	xDetermineStartAndBoundingCtuTsAddr(startCtuTsAddr, boundingCtuTsAddr, pcPic);
	if (bCompressEntireSlice)
	{
		boundingCtuTsAddr = pcSlice->getSliceCurEndCtuTsAddr();
#if HEVC_DEPENDENT_SLICES
		pcSlice->setSliceSegmentCurEndCtuTsAddr(boundingCtuTsAddr);
#endif
	}

	// initialize cost values - these are used by precompressSlice (they should be parameters).
	m_uiPicTotalBits = 0;
	m_uiPicDist = 0;

	pcSlice->setSliceQpBase(pcSlice->getSliceQp());

	m_CABACEstimator->initCtxModels(*pcSlice);

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
	for (int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++)
	{
		CABACWriter* cw = m_pcLib->getCABACEncoder(jId)->getCABACEstimator(pcSlice->getSPS());
		cw->initCtxModels(*pcSlice);
	}

#endif
	m_pcCuEncoder->getModeCtrl()->setFastDeltaQp(bFastDeltaQP);

	//------------------------------------------------------------------------------
	//  Weighted Prediction parameters estimation.
	//------------------------------------------------------------------------------
	// calculate AC/DC values for current picture
	if (pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred())
	{
		xCalcACDCParamSlice(pcSlice);
	}

	const Bool bWp_explicit = (pcSlice->getSliceType() == P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType() == B_SLICE && pcSlice->getPPS()->getWPBiPred());

	if (bWp_explicit)
	{
		//------------------------------------------------------------------------------
		//  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
		//------------------------------------------------------------------------------
#if HEVC_DEPENDENT_SLICES
		if (pcSlice->getSliceMode() == FIXED_NUMBER_OF_BYTES || pcSlice->getSliceSegmentMode() == FIXED_NUMBER_OF_BYTES)
#else
		if (pcSlice->getSliceMode() == FIXED_NUMBER_OF_BYTES)
#endif
		{
			EXIT("Weighted Prediction is not yet supported with slice mode determined by max number of bins.");
		}

		xEstimateWPParamSlice(pcSlice, m_pcCfg->getWeightedPredictionMethod());
		pcSlice->initWpScaling(pcSlice->getSPS());

		// check WP on/off
		xCheckWPEnable(pcSlice);
	}


#if HEVC_DEPENDENT_SLICES
#if HEVC_TILES_WPP
	// Adjust initial state if this is the start of a dependent slice.
	{
		const UInt      ctuRsAddr = tileMap.getCtuTsToRsAddrMap(startCtuTsAddr);
		const UInt      currentTileIdx = tileMap.getTileIdxMap(ctuRsAddr);
		const Tile&     currentTile = tileMap.tiles[currentTileIdx];
		const UInt      firstCtuRsAddrOfTile = currentTile.getFirstCtuRsAddr();
		if (pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != firstCtuRsAddrOfTile)
		{
			// This will only occur if dependent slice-segments (m_entropyCodingSyncContextState=true) are being used.
			if (currentTile.getTileWidthInCtus() >= 2 || !m_pcCfg->getEntropyCodingSyncEnabledFlag())
			{
				m_CABACEstimator->getCtx() = m_lastSliceSegmentEndContextState;
				m_CABACEstimator->start();
			}
		}
	}
#else
	// KJS: not sure if this works (but both dep slices and tiles shall be removed in VTM, so this code should not be used)
	if (pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != startCtuTsAddr)
	{
		if (pcPic->cs->pcv->widthInCtus >= 2 || !m_pcCfg->getEntropyCodingSyncEnabledFlag())
		{
			m_CABACEstimator->getCtx() = m_lastSliceSegmentEndContextState;
			m_CABACEstimator->start();
		}
#endif
#endif

#if HEVC_DEPENDENT_SLICES
		if (!pcSlice->getDependentSliceSegmentFlag())
		{
#endif
			pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
		}
#endif

		CHECK(pcPic->m_prevQP[0] == std::numeric_limits<Int>::max(), "Invalid previous QP");

		CodingStructure&  cs = *pcPic->cs;
#if ENABLE_QPA || ENABLE_WPP_PARALLELISM
		const PreCalcValues& pcv = *cs.pcv;
		const UInt        widthInCtus = pcv.widthInCtus;
#endif

		cs.slice = pcSlice;

		if (startCtuTsAddr == 0)
		{
			cs.initStructData(pcSlice->getSliceQp(), pcSlice->getPPS()->getTransquantBypassEnabledFlag());
		}

#if ENABLE_QPA
#if ENABLE_QPA
		double hpEnerMax = 1.0;
		double hpEnerPic = 0.0;
#endif
		Int    iSrcOffset;

#if ENABLE_QPA
		if (m_pcCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP() && !m_pcCfg->getUseRateCtrl())
#endif
		{
			for (UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++)
			{
#if HEVC_TILES_WPP
				const UInt     ctuRsAddr = tileMap.getCtuTsToRsAddrMap(ctuTsAddr);
#else
				const UInt     ctuRsAddr = ctuTsAddr;
#endif
				const Position pos((ctuRsAddr % widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / widthInCtus) * pcv.maxCUHeight);
				const CompArea subArea = clipArea(CompArea(COMPONENT_Y, pcPic->chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight)), pcPic->Y());
				const CompArea fltArea = clipArea(CompArea(COMPONENT_Y, pcPic->chromaFormat, Area(pos.x > 0 ? pos.x - 1 : 0, pos.y > 0 ? pos.y - 1 : 0, pcv.maxCUWidth + (pos.x > 0 ? 2 : 1), pcv.maxCUHeight + (pos.y > 0 ? 2 : 1))), pcPic->Y());
				const SizeType iSrcStride = pcPic->getOrigBuf(subArea).stride;
				const Pel*     pSrc = pcPic->getOrigBuf(subArea).buf;
				const SizeType iSrcHeight = pcPic->getOrigBuf(subArea).height;
				const SizeType iSrcWidth = pcPic->getOrigBuf(subArea).width;
				const SizeType iFltHeight = pcPic->getOrigBuf(fltArea).height;
				const SizeType iFltWidth = pcPic->getOrigBuf(fltArea).width;
				double hpEner = 0.0;

				DTRACE_UPDATE(g_trace_ctx, std::make_pair("ctu", ctuRsAddr));

				// compute DC offset to be subtracted from luma values
				iSrcOffset = 0;
				for (SizeType h = 0; h < iSrcHeight; h++)
				{
					for (SizeType w = 0; w < iSrcWidth; w++)
					{
						iSrcOffset += pSrc[w];
					}
					pSrc += iSrcStride;
				}
				CHECK(iSrcOffset < 0, "DC offset cannot be negative!");

				Int x = iSrcHeight * iSrcWidth;
				iSrcOffset = (iSrcOffset + (x >> 1)) / x; // slow division

				filterAndCalculateAverageEnergies(pcPic->getOrigBuf(fltArea).buf, iSrcStride,
					hpEner, iFltHeight, iFltWidth, pcPic->getPOC());

#if ENABLE_QPA
				if (hpEner > hpEnerMax) hpEnerMax = hpEner;
				hpEnerPic += hpEner;
				pcPic->m_uEnerHpCtu[ctuRsAddr] = hpEner;
				pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iSrcOffset;
#endif
			} // end iteration over all CTUs in current slice

		}

#if ENABLE_QPA
		if (m_pcCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP() && !m_pcCfg->getUseRateCtrl() && (boundingCtuTsAddr > startCtuTsAddr))
		{
			const double hpEnerAvg = hpEnerPic / double(boundingCtuTsAddr - startCtuTsAddr);

			if (applyQPAdaptation(pcPic, pcSlice, pcv, startCtuTsAddr, boundingCtuTsAddr, m_pcCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_NUM_MODES, m_pcCfg->getGOPSize(), hpEnerAvg, hpEnerMax))
			{
				m_CABACEstimator->initCtxModels(*pcSlice);
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
				for (int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++)
				{
					CABACWriter* cw = m_pcLib->getCABACEncoder(jId)->getCABACEstimator(pcSlice->getSPS());
					cw->initCtxModels(*pcSlice);
				}
#endif
#if HEVC_DEPENDENT_SLICES
				if (!pcSlice->getDependentSliceSegmentFlag())
				{
#endif
					pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
				}
#endif
				if (startCtuTsAddr == 0)
				{
					cs.currQP[0] = cs.currQP[1] = pcSlice->getSliceQp(); // cf code above
				}
			}
		}
#endif // ENABLE_QPA

#endif // ENABLE_QPA || ENABLE_PRIVATE

		cs.pcv = pcSlice->getPPS()->pcv;
		cs.fracBits = 0;


#if ENABLE_WPP_PARALLELISM
		bool bUseThreads = m_pcCfg->getNumWppThreads() > 1;
		if (bUseThreads)
		{
			CHECK(startCtuTsAddr != 0 || boundingCtuTsAddr != pcPic->cs->pcv->sizeInCtus, "not intended");

			pcPic->cs->allocateVectorsAtPicLevel();

			omp_set_num_threads(m_pcCfg->getNumWppThreads() + m_pcCfg->getNumWppExtraLines());

#pragma omp parallel for schedule(static,1) if(bUseThreads)
			for (int ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr += widthInCtus)
			{
				// wpp thread start
				pcPic->scheduler.setWppThreadId();
#if ENABLE_SPLIT_PARALLELISM
				pcPic->scheduler.setSplitThreadId(0);
#endif
				encodeCtus(pcPic, bCompressEntireSlice, bFastDeltaQP, ctuTsAddr, ctuTsAddr + widthInCtus, m_pcLib);
				// wpp thread stop
			}
		}
		else
#endif
			encodeCtusRDO(pcPic, bCompressEntireSlice, bFastDeltaQP, startCtuTsAddr, boundingCtuTsAddr, m_pcLib, bgBlock, BlockDPP);

#if HEVC_DEPENDENT_SLICES
		// store context state at the end of this slice-segment, in case the next slice is a dependent slice and continues using the CABAC contexts.
		if (pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag())
		{
			m_lastSliceSegmentEndContextState = m_CABACEstimator->getCtx();//ctx end of dep.slice
		}
#endif

	}

Void EncSlice::encodeCtusRDO(Picture* pcPic, const Bool bCompressEntireSlice, const Bool bFastDeltaQP, UInt startCtuTsAddr, UInt boundingCtuTsAddr, EncLib* pEncLib, Int bgBlock[], Double BlockDPP[])
	{
		//PROF_ACCUM_AND_START_NEW_SET( getProfilerCTU( pcPic, 0, 0 ), P_PIC_LEVEL );
		//PROF_START( getProfilerCTU( cs.slice->isIntra(), pcPic->scheduler.getWppThreadId() ), P_PIC_LEVEL, toWSizeIdx( cs.pcv->maxCUWidth ), toHSizeIdx( cs.pcv->maxCUHeight ) );
		CodingStructure&  cs = *pcPic->cs;
		Slice* pcSlice = cs.slice;
		const PreCalcValues& pcv = *cs.pcv;
		const UInt        widthInCtus = pcv.widthInCtus;
#if HEVC_TILES_WPP
		const TileMap&  tileMap = *pcPic->tileMap;
#endif
#if ENABLE_QPA
		const Int iQPIndex = pcSlice->getSliceQpBase();
		int iSrcOffset = 0;
#endif

#if ENABLE_WPP_PARALLELISM
		const int       dataId = pcPic->scheduler.getWppDataId();
#elif ENABLE_SPLIT_PARALLELISM
		const int       dataId = 0;
#endif
		CABACWriter*    pCABACWriter = pEncLib->getCABACEncoder(PARL_PARAM0(dataId))->getCABACEstimator(pcSlice->getSPS());
		TrQuant*        pTrQuant = pEncLib->getTrQuant(PARL_PARAM0(dataId));
		RdCost*         pRdCost = pEncLib->getRdCost(PARL_PARAM0(dataId));
		EncCfg*         pCfg = pEncLib;
		RateCtrl*       pRateCtrl = pEncLib->getRateCtrl();
#if ENABLE_WPP_PARALLELISM
		// first version dont use ctx from above
		pCABACWriter->initCtxModels(*pcSlice);
#endif
#if RDOQ_CHROMA_LAMBDA
		pTrQuant->setLambdas(pcSlice->getLambdas());
#else
		pTrQuant->setLambda(pcSlice->getLambdas()[0]);
#endif
		pRdCost->setLambda(pcSlice->getLambdas()[0], pcSlice->getSPS()->getBitDepths());

		int prevQP[2];
		int currQP[2];
		prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
		currQP[0] = currQP[1] = pcSlice->getSliceQp();

#if HEVC_DEPENDENT_SLICES
		if (!pcSlice->getDependentSliceSegmentFlag())
		{
#endif
			prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
		}
#endif
		Double ubadLambda = pRdCost->getLambda();
		// for every CTU in the slice segment (may terminate sooner if there is a byte limit on the slice-segment)
		for (UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++)
		{
#if HEVC_TILES_WPP
			const UInt ctuRsAddr = tileMap.getCtuTsToRsAddrMap(ctuTsAddr);
#else
			const UInt ctuRsAddr = ctuTsAddr;
#endif

#if HEVC_TILES_WPP
			// update CABAC state
			const UInt firstCtuRsAddrOfTile = tileMap.tiles[tileMap.getTileIdxMap(ctuRsAddr)].getFirstCtuRsAddr();
			const UInt tileXPosInCtus = firstCtuRsAddrOfTile % widthInCtus;
#endif
			const UInt ctuXPosInCtus = ctuRsAddr % widthInCtus;
			const UInt ctuYPosInCtus = ctuRsAddr / widthInCtus;

			const Position pos(ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
			const UnitArea ctuArea(cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight));
			DTRACE_UPDATE(g_trace_ctx, std::make_pair("ctu", ctuRsAddr));

#if ENABLE_WPP_PARALLELISM
			pcPic->scheduler.wait(ctuXPosInCtus, ctuYPosInCtus);
#endif

#if HEVC_TILES_WPP
			if (ctuRsAddr == firstCtuRsAddrOfTile)
			{
				pCABACWriter->initCtxModels(*pcSlice);
				prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
			}
			else if (ctuXPosInCtus == tileXPosInCtus && pEncLib->getEntropyCodingSyncEnabledFlag())
			{
				// reset and then update contexts to the state at the end of the top-right CTU (if within current slice and tile).
				pCABACWriter->initCtxModels(*pcSlice);
				if (cs.getCURestricted(pos.offset(pcv.maxCUWidth, -1), pcSlice->getIndependentSliceIdx(), tileMap.getTileIdxMap(pos), CH_L))
				{
					// Top-right is available, we use it.
					pCABACWriter->getCtx() = pEncLib->m_entropyCodingSyncContextState;
				}
				prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
			}
#endif

#if ENABLE_WPP_PARALLELISM
			if (ctuXPosInCtus == 0 && ctuYPosInCtus > 0 && widthInCtus > 1 && (pEncLib->getNumWppThreads() > 1 || pEncLib->getEnsureWppBitEqual()))
			{
				pCABACWriter->getCtx() = pEncLib->m_entropyCodingSyncContextStateVec[ctuYPosInCtus - 1];  // last line
			}
#else
#endif

#if RDOQ_CHROMA_LAMBDA && ENABLE_QPA
			Double oldLambdaArray[MAX_NUM_COMPONENT] = { 0.0 };
#endif
			const Double oldLambda = pRdCost->getLambda();
			if (pCfg->getUseRateCtrl())
			{
				Int estQP = pcSlice->getSliceQp();
				Double estLambda = -1.0;
				Double bpp = -1.0;

				if ((pcPic->slices[0]->getSliceType() == I_SLICE && pCfg->getForceIntraQP()) || !pCfg->getLCULevelRC())
				{
					estQP = pcSlice->getSliceQp();
				}
				else
				{
					bpp = pRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->getSliceType());
					if (pcPic->slices[0]->getSliceType() == I_SLICE)
					{
						estLambda = pRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
					}
					else
					{
						estLambda = pRateCtrl->getRCPic()->getLCUEstLambda(bpp);
						estQP = pRateCtrl->getRCPic()->getLCUEstQP(estLambda, pcSlice->getSliceQp());
					}

					estQP = Clip3(-pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP);

					pRdCost->setLambda(estLambda, pcSlice->getSPS()->getBitDepths());

#if RDOQ_CHROMA_LAMBDA
					// set lambda for RDOQ
					const Double chromaLambda = estLambda / pRdCost->getChromaWeight();
					const Double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
					pTrQuant->setLambdas(lambdaArray);
#else
					pTrQuant->setLambda(estLambda);
#endif
				}

				pRateCtrl->setRCQP(estQP);
			}
#if ENABLE_QPA
			else if (pCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
			{
				iSrcOffset = pcPic->m_iOffsetCtu[ctuRsAddr];
				const Double newLambda = oldLambda * pow(2.0, Double(iSrcOffset - iQPIndex) / 3.0);
#if RDOQ_CHROMA_LAMBDA
				pTrQuant->getLambdas(oldLambdaArray); // save the old lambdas
				const Double chromaLambda = newLambda / pRdCost->getChromaWeight();
				const Double lambdaArray[MAX_NUM_COMPONENT] = { newLambda, chromaLambda, chromaLambda };
				pTrQuant->setLambdas(lambdaArray);
#else
				pTrQuant->setLambda(newLambda);
#endif
				pRdCost->setLambda(newLambda, pcSlice->getSPS()->getBitDepths());
				currQP[0] = currQP[1] = iSrcOffset;
			}
#endif
#if BLOCK_RDO
			pRdCost->setLambda(ubadLambda, pcSlice->getSPS()->getBitDepths());
			if (bgBlock[ctuTsAddr] > 1000 && bgBlock[ctuTsAddr] < 2000) //�Ǳ����� ���ñ��뱳����Lambda
			{
				cout << "enLambda" << pRdCost->getLambda();
				Double dLambda = ubadLambda;
				Double M = bgBlock[ctuTsAddr] - 1000;
				Double beta = 0.5;
				Double MM = M / (1 + (M - 1)*pow(M, beta));
				MM = pow(MM, 1 / beta);
				Double S = 6.5;
				dLambda = dLambda*MM*S;
				
				//dLambda = dLambda*11.5;
				cout << "enLambda" << dLambda;
				pRdCost->setLambda(dLambda, pcSlice->getSPS()->getBitDepths());
			}
			/*if (bgBlock[ctuTsAddr] > 2000) //���ѱ౳���� �����ѱ౳����Lambda
			{
				cout << "reLambda" << pRdCost->getLambda();
				Double dLambda = ubadLambda;
				Double M = bgBlock[ctuTsAddr] - 2000;
				Double beta = 0.5;
				Double MM = M / (1 + (M - 1)*pow(M, beta));
				MM = pow(MM, 1 / beta);
				dLambda = dLambda*MM*M;
				cout << "reLambda" << dLambda;
				pRdCost->setLambda(dLambda, pcSlice->getSPS()->getBitDepths());
			}*/
#endif
#if ENABLE_WPP_PARALLELISM
			pEncLib->getCuEncoder(dataId)->compressCtu(cs, ctuArea, ctuRsAddr, prevQP, currQP);
#else
			m_pcCuEncoder->compressCtu(cs, ctuArea, ctuRsAddr, prevQP, currQP);
#endif

			pCABACWriter->resetBits();
			pCABACWriter->coding_tree_unit(cs, ctuArea, prevQP, ctuRsAddr, true);
			const int numberOfWrittenBits = int(pCABACWriter->getEstFracBits() >> SCALE_BITS);

			// Calculate if this CTU puts us over slice bit size.
			// cannot terminate if current slice/slice-segment would be 0 Ctu in size,
			const UInt validEndOfSliceCtuTsAddr = ctuTsAddr + (ctuTsAddr == startCtuTsAddr ? 1 : 0);
			// Set slice end parameter
			if (pcSlice->getSliceMode() == FIXED_NUMBER_OF_BYTES && pcSlice->getSliceBits() + numberOfWrittenBits > (pcSlice->getSliceArgument() << 3))
			{
#if HEVC_DEPENDENT_SLICES
				pcSlice->setSliceSegmentCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
#endif
				pcSlice->setSliceCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
				boundingCtuTsAddr = validEndOfSliceCtuTsAddr;
			}
#if HEVC_DEPENDENT_SLICES
			else if ((!bCompressEntireSlice) && pcSlice->getSliceSegmentMode() == FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits() + numberOfWrittenBits > (pcSlice->getSliceSegmentArgument() << 3))
			{
				pcSlice->setSliceSegmentCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
				boundingCtuTsAddr = validEndOfSliceCtuTsAddr;
			}
#endif
			if (boundingCtuTsAddr <= ctuTsAddr)
			{
				break;
			}

#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#pragma omp critical
#endif
			pcSlice->setSliceBits((UInt)(pcSlice->getSliceBits() + numberOfWrittenBits));
#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#pragma omp critical
#endif
#if HEVC_DEPENDENT_SLICES
			pcSlice->setSliceSegmentBits(pcSlice->getSliceSegmentBits() + numberOfWrittenBits);
#endif

#if HEVC_TILES_WPP
			// Store probabilities of second CTU in line into buffer - used only if wavefront-parallel-processing is enabled.
			if (ctuXPosInCtus == tileXPosInCtus + 1 && pEncLib->getEntropyCodingSyncEnabledFlag())
			{
				pEncLib->m_entropyCodingSyncContextState = pCABACWriter->getCtx();
			}
#endif
#if ENABLE_WPP_PARALLELISM
			if (ctuXPosInCtus == 1 && (pEncLib->getNumWppThreads() > 1 || pEncLib->getEnsureWppBitEqual()))
			{
				pEncLib->m_entropyCodingSyncContextStateVec[ctuYPosInCtus] = pCABACWriter->getCtx();
			}
#endif

#if !ENABLE_WPP_PARALLELISM
			Int actualBits = int(cs.fracBits >> SCALE_BITS);
#endif
			if (pCfg->getUseRateCtrl())
			{
#if ENABLE_WPP_PARALLELISM
				Int actualBits = int(cs.fracBits >> SCALE_BITS);
#endif
				Int actualQP = g_RCInvalidQPValue;
				Double actualLambda = pRdCost->getLambda();
				Int numberOfEffectivePixels = 0;

				for (auto &cu : cs.traverseCUs(ctuArea, CH_L))
				{
					if (!cu.skip || cu.rootCbf)
					{
						numberOfEffectivePixels += cu.lumaSize().area();
						break;
					}
				}

				CodingUnit* cu = cs.getCU(ctuArea.lumaPos(), CH_L);

				if (numberOfEffectivePixels == 0)
				{
					actualQP = g_RCInvalidQPValue;
				}
				else
				{
					actualQP = cu->qp;
				}
				pRdCost->setLambda(oldLambda, pcSlice->getSPS()->getBitDepths());
				pRateCtrl->getRCPic()->updateAfterCTU(pRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
					pcSlice->getSliceType() == I_SLICE ? 0 : pCfg->getLCULevelRC());
			}
#if ENABLE_QPA
			else if (pCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
			{
#if RDOQ_CHROMA_LAMBDA
				pTrQuant->setLambdas(oldLambdaArray);
#else
				pTrQuant->setLambda(oldLambda);
#endif
				pRdCost->setLambda(oldLambda, pcSlice->getSPS()->getBitDepths());
			}
#endif

#if !ENABLE_WPP_PARALLELISM
			m_uiPicTotalBits += actualBits;
			m_uiPicDist = cs.dist;
#endif
#if ENABLE_WPP_PARALLELISM
			pcPic->scheduler.setReady(ctuXPosInCtus, ctuYPosInCtus);
#endif
		}

		// this is wpp exclusive section

		//  m_uiPicTotalBits += actualBits;
		//  m_uiPicDist       = cs.dist;

	}
#endif

#if BLOCK_SELECT
	Void EncSlice::compressSliceSel(Picture* pcPic, const Bool bCompressEntireSlice, const Bool bFastDeltaQP, vector<int>& BgSelect)
	{
		// if bCompressEntireSlice is true, then the entire slice (not slice segment) is compressed,
		//   effectively disabling the slice-segment-mode.

#if GENERATE_BG_PIC
		Picture* bg_NewPicYuvRecCU = getNewPicYuvRecSli();
		m_pcCuEncoder->setbgNewPicYuvRecCU(bg_NewPicYuvRecCU);
#endif
		Slice* const pcSlice = pcPic->slices[getSliceSegmentIdx()];
#if HEVC_TILES_WPP
		const TileMap&  tileMap = *pcPic->tileMap;
#endif
		UInt  startCtuTsAddr;
		UInt  boundingCtuTsAddr;

#if HEVC_DEPENDENT_SLICES
		pcSlice->setSliceSegmentBits(0);
#endif
		xDetermineStartAndBoundingCtuTsAddr(startCtuTsAddr, boundingCtuTsAddr, pcPic);
		if (bCompressEntireSlice)
		{
			boundingCtuTsAddr = pcSlice->getSliceCurEndCtuTsAddr();
#if HEVC_DEPENDENT_SLICES
			pcSlice->setSliceSegmentCurEndCtuTsAddr(boundingCtuTsAddr);
#endif
		}

		// initialize cost values - these are used by precompressSlice (they should be parameters).
		m_uiPicTotalBits = 0;
		m_uiPicDist = 0;

		pcSlice->setSliceQpBase(pcSlice->getSliceQp());

		m_CABACEstimator->initCtxModels(*pcSlice);

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
		for (int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++)
		{
			CABACWriter* cw = m_pcLib->getCABACEncoder(jId)->getCABACEstimator(pcSlice->getSPS());
			cw->initCtxModels(*pcSlice);
		}

#endif
		m_pcCuEncoder->getModeCtrl()->setFastDeltaQp(bFastDeltaQP);

		//------------------------------------------------------------------------------
		//  Weighted Prediction parameters estimation.
		//------------------------------------------------------------------------------
		// calculate AC/DC values for current picture
		if (pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred())
		{
			xCalcACDCParamSlice(pcSlice);
		}

		const Bool bWp_explicit = (pcSlice->getSliceType() == P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType() == B_SLICE && pcSlice->getPPS()->getWPBiPred());

		if (bWp_explicit)
		{
			//------------------------------------------------------------------------------
			//  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
			//------------------------------------------------------------------------------
#if HEVC_DEPENDENT_SLICES
			if (pcSlice->getSliceMode() == FIXED_NUMBER_OF_BYTES || pcSlice->getSliceSegmentMode() == FIXED_NUMBER_OF_BYTES)
#else
			if (pcSlice->getSliceMode() == FIXED_NUMBER_OF_BYTES)
#endif
			{
				EXIT("Weighted Prediction is not yet supported with slice mode determined by max number of bins.");
			}

			xEstimateWPParamSlice(pcSlice, m_pcCfg->getWeightedPredictionMethod());
			pcSlice->initWpScaling(pcSlice->getSPS());

			// check WP on/off
			xCheckWPEnable(pcSlice);
		}


#if HEVC_DEPENDENT_SLICES
#if HEVC_TILES_WPP
		// Adjust initial state if this is the start of a dependent slice.
		{
			const UInt      ctuRsAddr = tileMap.getCtuTsToRsAddrMap(startCtuTsAddr);
			const UInt      currentTileIdx = tileMap.getTileIdxMap(ctuRsAddr);
			const Tile&     currentTile = tileMap.tiles[currentTileIdx];
			const UInt      firstCtuRsAddrOfTile = currentTile.getFirstCtuRsAddr();
			if (pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != firstCtuRsAddrOfTile)
			{
				// This will only occur if dependent slice-segments (m_entropyCodingSyncContextState=true) are being used.
				if (currentTile.getTileWidthInCtus() >= 2 || !m_pcCfg->getEntropyCodingSyncEnabledFlag())
				{
					m_CABACEstimator->getCtx() = m_lastSliceSegmentEndContextState;
					m_CABACEstimator->start();
				}
			}
		}
#else
		// KJS: not sure if this works (but both dep slices and tiles shall be removed in VTM, so this code should not be used)
		if (pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != startCtuTsAddr)
		{
			if (pcPic->cs->pcv->widthInCtus >= 2 || !m_pcCfg->getEntropyCodingSyncEnabledFlag())
			{
				m_CABACEstimator->getCtx() = m_lastSliceSegmentEndContextState;
				m_CABACEstimator->start();
			}
#endif
#endif

#if HEVC_DEPENDENT_SLICES
			if (!pcSlice->getDependentSliceSegmentFlag())
			{
#endif
				pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
			}
#endif

			CHECK(pcPic->m_prevQP[0] == std::numeric_limits<Int>::max(), "Invalid previous QP");

			CodingStructure&  cs = *pcPic->cs;
#if ENABLE_QPA || ENABLE_WPP_PARALLELISM
			const PreCalcValues& pcv = *cs.pcv;
			const UInt        widthInCtus = pcv.widthInCtus;
#endif

			cs.slice = pcSlice;

			if (startCtuTsAddr == 0)
			{
				cs.initStructData(pcSlice->getSliceQp(), pcSlice->getPPS()->getTransquantBypassEnabledFlag());
			}

#if ENABLE_QPA
#if ENABLE_QPA
			double hpEnerMax = 1.0;
			double hpEnerPic = 0.0;
#endif
			Int    iSrcOffset;

#if ENABLE_QPA
			if (m_pcCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP() && !m_pcCfg->getUseRateCtrl())
#endif
			{
				for (UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++)
				{
#if HEVC_TILES_WPP
					const UInt     ctuRsAddr = tileMap.getCtuTsToRsAddrMap(ctuTsAddr);
#else
					const UInt     ctuRsAddr = ctuTsAddr;
#endif
					const Position pos((ctuRsAddr % widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / widthInCtus) * pcv.maxCUHeight);
					const CompArea subArea = clipArea(CompArea(COMPONENT_Y, pcPic->chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight)), pcPic->Y());
					const CompArea fltArea = clipArea(CompArea(COMPONENT_Y, pcPic->chromaFormat, Area(pos.x > 0 ? pos.x - 1 : 0, pos.y > 0 ? pos.y - 1 : 0, pcv.maxCUWidth + (pos.x > 0 ? 2 : 1), pcv.maxCUHeight + (pos.y > 0 ? 2 : 1))), pcPic->Y());
					const SizeType iSrcStride = pcPic->getOrigBuf(subArea).stride;
					const Pel*     pSrc = pcPic->getOrigBuf(subArea).buf;
					const SizeType iSrcHeight = pcPic->getOrigBuf(subArea).height;
					const SizeType iSrcWidth = pcPic->getOrigBuf(subArea).width;
					const SizeType iFltHeight = pcPic->getOrigBuf(fltArea).height;
					const SizeType iFltWidth = pcPic->getOrigBuf(fltArea).width;
					double hpEner = 0.0;

					DTRACE_UPDATE(g_trace_ctx, std::make_pair("ctu", ctuRsAddr));

					// compute DC offset to be subtracted from luma values
					iSrcOffset = 0;
					for (SizeType h = 0; h < iSrcHeight; h++)
					{
						for (SizeType w = 0; w < iSrcWidth; w++)
						{
							iSrcOffset += pSrc[w];
						}
						pSrc += iSrcStride;
					}
					CHECK(iSrcOffset < 0, "DC offset cannot be negative!");

					Int x = iSrcHeight * iSrcWidth;
					iSrcOffset = (iSrcOffset + (x >> 1)) / x; // slow division

					filterAndCalculateAverageEnergies(pcPic->getOrigBuf(fltArea).buf, iSrcStride,
						hpEner, iFltHeight, iFltWidth, pcPic->getPOC());

#if ENABLE_QPA
					if (hpEner > hpEnerMax) hpEnerMax = hpEner;
					hpEnerPic += hpEner;
					pcPic->m_uEnerHpCtu[ctuRsAddr] = hpEner;
					pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iSrcOffset;
#endif
				} // end iteration over all CTUs in current slice

			}

#if ENABLE_QPA
			if (m_pcCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP() && !m_pcCfg->getUseRateCtrl() && (boundingCtuTsAddr > startCtuTsAddr))
			{
				const double hpEnerAvg = hpEnerPic / double(boundingCtuTsAddr - startCtuTsAddr);

				if (applyQPAdaptation(pcPic, pcSlice, pcv, startCtuTsAddr, boundingCtuTsAddr, m_pcCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_NUM_MODES, m_pcCfg->getGOPSize(), hpEnerAvg, hpEnerMax))
				{
					m_CABACEstimator->initCtxModels(*pcSlice);
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
					for (int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++)
					{
						CABACWriter* cw = m_pcLib->getCABACEncoder(jId)->getCABACEstimator(pcSlice->getSPS());
						cw->initCtxModels(*pcSlice);
					}
#endif
#if HEVC_DEPENDENT_SLICES
					if (!pcSlice->getDependentSliceSegmentFlag())
					{
#endif
						pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
					}
#endif
					if (startCtuTsAddr == 0)
					{
						cs.currQP[0] = cs.currQP[1] = pcSlice->getSliceQp(); // cf code above
					}
				}
			}
#endif // ENABLE_QPA

#endif // ENABLE_QPA || ENABLE_PRIVATE

			cs.pcv = pcSlice->getPPS()->pcv;
			cs.fracBits = 0;


#if ENABLE_WPP_PARALLELISM
			bool bUseThreads = m_pcCfg->getNumWppThreads() > 1;
			if (bUseThreads)
			{
				CHECK(startCtuTsAddr != 0 || boundingCtuTsAddr != pcPic->cs->pcv->sizeInCtus, "not intended");

				pcPic->cs->allocateVectorsAtPicLevel();

				omp_set_num_threads(m_pcCfg->getNumWppThreads() + m_pcCfg->getNumWppExtraLines());

#pragma omp parallel for schedule(static,1) if(bUseThreads)
				for (int ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr += widthInCtus)
				{
					// wpp thread start
					pcPic->scheduler.setWppThreadId();
#if ENABLE_SPLIT_PARALLELISM
					pcPic->scheduler.setSplitThreadId(0);
#endif
					encodeCtus(pcPic, bCompressEntireSlice, bFastDeltaQP, ctuTsAddr, ctuTsAddr + widthInCtus, m_pcLib);
					// wpp thread stop
				}
			}
			else
#endif
				encodeCtusSel(pcPic, bCompressEntireSlice, bFastDeltaQP, startCtuTsAddr, boundingCtuTsAddr, m_pcLib, BgSelect);

#if HEVC_DEPENDENT_SLICES
			// store context state at the end of this slice-segment, in case the next slice is a dependent slice and continues using the CABAC contexts.
			if (pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag())
			{
				m_lastSliceSegmentEndContextState = m_CABACEstimator->getCtx();//ctx end of dep.slice
			}
#endif

		}


	Void EncSlice::encodeCtusSel(Picture* pcPic, const Bool bCompressEntireSlice, const Bool bFastDeltaQP, UInt startCtuTsAddr, UInt boundingCtuTsAddr, EncLib* pEncLib, vector<int>& BgSelect)
	{
			//PROF_ACCUM_AND_START_NEW_SET( getProfilerCTU( pcPic, 0, 0 ), P_PIC_LEVEL );
			//PROF_START( getProfilerCTU( cs.slice->isIntra(), pcPic->scheduler.getWppThreadId() ), P_PIC_LEVEL, toWSizeIdx( cs.pcv->maxCUWidth ), toHSizeIdx( cs.pcv->maxCUHeight ) );
			CodingStructure&  cs = *pcPic->cs;
			Slice* pcSlice = cs.slice;
			const PreCalcValues& pcv = *cs.pcv;
			const UInt        widthInCtus = pcv.widthInCtus;
#if HEVC_TILES_WPP
			const TileMap&  tileMap = *pcPic->tileMap;
#endif
#if ENABLE_QPA
			const Int iQPIndex = pcSlice->getSliceQpBase();
			int iSrcOffset = 0;
#endif

#if ENABLE_WPP_PARALLELISM
			const int       dataId = pcPic->scheduler.getWppDataId();
#elif ENABLE_SPLIT_PARALLELISM
			const int       dataId = 0;
#endif
			CABACWriter*    pCABACWriter = pEncLib->getCABACEncoder(PARL_PARAM0(dataId))->getCABACEstimator(pcSlice->getSPS());
			TrQuant*        pTrQuant = pEncLib->getTrQuant(PARL_PARAM0(dataId));
			RdCost*         pRdCost = pEncLib->getRdCost(PARL_PARAM0(dataId));
			EncCfg*         pCfg = pEncLib;
			RateCtrl*       pRateCtrl = pEncLib->getRateCtrl();
#if ENABLE_WPP_PARALLELISM
			// first version dont use ctx from above
			pCABACWriter->initCtxModels(*pcSlice);
#endif
#if RDOQ_CHROMA_LAMBDA
			pTrQuant->setLambdas(pcSlice->getLambdas());
#else
			pTrQuant->setLambda(pcSlice->getLambdas()[0]);
#endif
			pRdCost->setLambda(pcSlice->getLambdas()[0], pcSlice->getSPS()->getBitDepths());

			int prevQP[2];
			int currQP[2];
			prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
			currQP[0] = currQP[1] = pcSlice->getSliceQp();

#if HEVC_DEPENDENT_SLICES
			if (!pcSlice->getDependentSliceSegmentFlag())
			{
#endif
				prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
			}
#endif
			// for every CTU in the slice segment (may terminate sooner if there is a byte limit on the slice-segment)
			for (UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++)
			{
#if HEVC_TILES_WPP
				const UInt ctuRsAddr = tileMap.getCtuTsToRsAddrMap(ctuTsAddr);
#else
				const UInt ctuRsAddr = ctuTsAddr;
#endif

#if HEVC_TILES_WPP
				// update CABAC state
				const UInt firstCtuRsAddrOfTile = tileMap.tiles[tileMap.getTileIdxMap(ctuRsAddr)].getFirstCtuRsAddr();
				const UInt tileXPosInCtus = firstCtuRsAddrOfTile % widthInCtus;
#endif
				const UInt ctuXPosInCtus = ctuRsAddr % widthInCtus;
				const UInt ctuYPosInCtus = ctuRsAddr / widthInCtus;

				const Position pos(ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
				const UnitArea ctuArea(cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight));
				DTRACE_UPDATE(g_trace_ctx, std::make_pair("ctu", ctuRsAddr));

				//cout << ctuArea.lumaPos().x<<"-"<<ctuArea.lumaPos().y<<"  ";
				//cout << pcv.maxCUHeight << "-" << pcv.maxCUWidth << " ";
#if ENABLE_WPP_PARALLELISM
				pcPic->scheduler.wait(ctuXPosInCtus, ctuYPosInCtus);
#endif

#if HEVC_TILES_WPP
				if (ctuRsAddr == firstCtuRsAddrOfTile)
				{
					pCABACWriter->initCtxModels(*pcSlice);
					prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
				}
				else if (ctuXPosInCtus == tileXPosInCtus && pEncLib->getEntropyCodingSyncEnabledFlag())
				{
					// reset and then update contexts to the state at the end of the top-right CTU (if within current slice and tile).
					pCABACWriter->initCtxModels(*pcSlice);
					if (cs.getCURestricted(pos.offset(pcv.maxCUWidth, -1), pcSlice->getIndependentSliceIdx(), tileMap.getTileIdxMap(pos), CH_L))
					{
						// Top-right is available, we use it.
						pCABACWriter->getCtx() = pEncLib->m_entropyCodingSyncContextState;
					}
					prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
				}
#endif

#if ENABLE_WPP_PARALLELISM
				if (ctuXPosInCtus == 0 && ctuYPosInCtus > 0 && widthInCtus > 1 && (pEncLib->getNumWppThreads() > 1 || pEncLib->getEnsureWppBitEqual()))
				{
					pCABACWriter->getCtx() = pEncLib->m_entropyCodingSyncContextStateVec[ctuYPosInCtus - 1];  // last line
				}
#else
#endif

#if RDOQ_CHROMA_LAMBDA && ENABLE_QPA
				Double oldLambdaArray[MAX_NUM_COMPONENT] = { 0.0 };
#endif
				const Double oldLambda = pRdCost->getLambda();
				if (pCfg->getUseRateCtrl())
				{
					Int estQP = pcSlice->getSliceQp();
					Double estLambda = -1.0;
					Double bpp = -1.0;

					if ((pcPic->slices[0]->getSliceType() == I_SLICE && pCfg->getForceIntraQP()) || !pCfg->getLCULevelRC())
					{
						estQP = pcSlice->getSliceQp();
					}
					else
					{
						bpp = pRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->getSliceType());
						if (pcPic->slices[0]->getSliceType() == I_SLICE)
						{
							estLambda = pRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
						}
						else
						{
							estLambda = pRateCtrl->getRCPic()->getLCUEstLambda(bpp);
							estQP = pRateCtrl->getRCPic()->getLCUEstQP(estLambda, pcSlice->getSliceQp());
						}

						estQP = Clip3(-pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP);

						pRdCost->setLambda(estLambda, pcSlice->getSPS()->getBitDepths());

#if RDOQ_CHROMA_LAMBDA
						// set lambda for RDOQ
						const Double chromaLambda = estLambda / pRdCost->getChromaWeight();
						const Double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
						pTrQuant->setLambdas(lambdaArray);
#else
						pTrQuant->setLambda(estLambda);
#endif
					}

					pRateCtrl->setRCQP(estQP);
				}
#if ENABLE_QPA
				else if (pCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
				{
					iSrcOffset = pcPic->m_iOffsetCtu[ctuRsAddr];
					const Double newLambda = oldLambda * pow(2.0, Double(iSrcOffset - iQPIndex) / 3.0);
#if RDOQ_CHROMA_LAMBDA
					pTrQuant->getLambdas(oldLambdaArray); // save the old lambdas
					const Double chromaLambda = newLambda / pRdCost->getChromaWeight();
					const Double lambdaArray[MAX_NUM_COMPONENT] = { newLambda, chromaLambda, chromaLambda };
					pTrQuant->setLambdas(lambdaArray);
#else
					pTrQuant->setLambda(newLambda);
#endif
					pRdCost->setLambda(newLambda, pcSlice->getSPS()->getBitDepths());
					currQP[0] = currQP[1] = iSrcOffset;
				}
#endif


#if ENABLE_WPP_PARALLELISM
				pEncLib->getCuEncoder(dataId)->compressCtu(cs, ctuArea, ctuRsAddr, prevQP, currQP);
#else
				m_pcCuEncoder->compressCtuSel(cs, ctuArea, ctuRsAddr, prevQP, currQP,BgSelect);
#endif


				pCABACWriter->resetBits();
				pCABACWriter->coding_tree_unit(cs, ctuArea, prevQP, ctuRsAddr, true);
				const int numberOfWrittenBits = int(pCABACWriter->getEstFracBits() >> SCALE_BITS);

				// Calculate if this CTU puts us over slice bit size.
				// cannot terminate if current slice/slice-segment would be 0 Ctu in size,
				const UInt validEndOfSliceCtuTsAddr = ctuTsAddr + (ctuTsAddr == startCtuTsAddr ? 1 : 0);
				// Set slice end parameter
				if (pcSlice->getSliceMode() == FIXED_NUMBER_OF_BYTES && pcSlice->getSliceBits() + numberOfWrittenBits > (pcSlice->getSliceArgument() << 3))
				{
#if HEVC_DEPENDENT_SLICES
					pcSlice->setSliceSegmentCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
#endif
					pcSlice->setSliceCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
					boundingCtuTsAddr = validEndOfSliceCtuTsAddr;
				}
#if HEVC_DEPENDENT_SLICES
				else if ((!bCompressEntireSlice) && pcSlice->getSliceSegmentMode() == FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits() + numberOfWrittenBits > (pcSlice->getSliceSegmentArgument() << 3))
				{
					pcSlice->setSliceSegmentCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
					boundingCtuTsAddr = validEndOfSliceCtuTsAddr;
				}
#endif
				if (boundingCtuTsAddr <= ctuTsAddr)
				{
					break;
				}

#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#pragma omp critical
#endif
				pcSlice->setSliceBits((UInt)(pcSlice->getSliceBits() + numberOfWrittenBits));
#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#pragma omp critical
#endif
#if HEVC_DEPENDENT_SLICES
				pcSlice->setSliceSegmentBits(pcSlice->getSliceSegmentBits() + numberOfWrittenBits);
#endif

#if HEVC_TILES_WPP
				// Store probabilities of second CTU in line into buffer - used only if wavefront-parallel-processing is enabled.
				if (ctuXPosInCtus == tileXPosInCtus + 1 && pEncLib->getEntropyCodingSyncEnabledFlag())
				{
					pEncLib->m_entropyCodingSyncContextState = pCABACWriter->getCtx();
				}
#endif
#if ENABLE_WPP_PARALLELISM
				if (ctuXPosInCtus == 1 && (pEncLib->getNumWppThreads() > 1 || pEncLib->getEnsureWppBitEqual()))
				{
					pEncLib->m_entropyCodingSyncContextStateVec[ctuYPosInCtus] = pCABACWriter->getCtx();
				}
#endif

#if !ENABLE_WPP_PARALLELISM
				Int actualBits = int(cs.fracBits >> SCALE_BITS);
#endif
				if (pCfg->getUseRateCtrl())
				{
#if ENABLE_WPP_PARALLELISM
					Int actualBits = int(cs.fracBits >> SCALE_BITS);
#endif
					Int actualQP = g_RCInvalidQPValue;
					Double actualLambda = pRdCost->getLambda();
					Int numberOfEffectivePixels = 0;

					for (auto &cu : cs.traverseCUs(ctuArea, CH_L))
					{
						if (!cu.skip || cu.rootCbf)
						{
							numberOfEffectivePixels += cu.lumaSize().area();
							break;
						}
					}

					CodingUnit* cu = cs.getCU(ctuArea.lumaPos(), CH_L);

					if (numberOfEffectivePixels == 0)
					{
						actualQP = g_RCInvalidQPValue;
					}
					else
					{
						actualQP = cu->qp;
					}
					pRdCost->setLambda(oldLambda, pcSlice->getSPS()->getBitDepths());
					pRateCtrl->getRCPic()->updateAfterCTU(pRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
						pcSlice->getSliceType() == I_SLICE ? 0 : pCfg->getLCULevelRC());
				}
#if ENABLE_QPA
				else if (pCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
				{
#if RDOQ_CHROMA_LAMBDA
					pTrQuant->setLambdas(oldLambdaArray);
#else
					pTrQuant->setLambda(oldLambda);
#endif
					pRdCost->setLambda(oldLambda, pcSlice->getSPS()->getBitDepths());
				}
#endif

#if !ENABLE_WPP_PARALLELISM
				m_uiPicTotalBits += actualBits;
				m_uiPicDist = cs.dist;
#endif
#if ENABLE_WPP_PARALLELISM
				pcPic->scheduler.setReady(ctuXPosInCtus, ctuYPosInCtus);
#endif
			}

			// this is wpp exclusive section

			//  m_uiPicTotalBits += actualBits;
			//  m_uiPicDist       = cs.dist;

		}
#endif
Void EncSlice::compressSlice( Picture* pcPic, const Bool bCompressEntireSlice, const Bool bFastDeltaQP )
{
  // if bCompressEntireSlice is true, then the entire slice (not slice segment) is compressed,
  //   effectively disabling the slice-segment-mode.
	
#if GENERATE_BG_PIC
	Picture* bg_NewPicYuvRecCU = getNewPicYuvRecSli();
	m_pcCuEncoder->setbgNewPicYuvRecCU(bg_NewPicYuvRecCU);
#endif
  Slice* const pcSlice    = pcPic->slices[getSliceSegmentIdx()];
#if HEVC_TILES_WPP
  const TileMap&  tileMap = *pcPic->tileMap;
#endif
  UInt  startCtuTsAddr;
  UInt  boundingCtuTsAddr;

#if HEVC_DEPENDENT_SLICES
  pcSlice->setSliceSegmentBits(0);
#endif
  xDetermineStartAndBoundingCtuTsAddr ( startCtuTsAddr, boundingCtuTsAddr, pcPic );
  if (bCompressEntireSlice)
  {
    boundingCtuTsAddr = pcSlice->getSliceCurEndCtuTsAddr();
#if HEVC_DEPENDENT_SLICES
    pcSlice->setSliceSegmentCurEndCtuTsAddr(boundingCtuTsAddr);
#endif
  }

  // initialize cost values - these are used by precompressSlice (they should be parameters).
  m_uiPicTotalBits  = 0;
  m_uiPicDist       = 0;

  pcSlice->setSliceQpBase( pcSlice->getSliceQp() );

  m_CABACEstimator->initCtxModels( *pcSlice );

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  for( int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++ )
  {
    CABACWriter* cw = m_pcLib->getCABACEncoder( jId )->getCABACEstimator( pcSlice->getSPS() );
    cw->initCtxModels( *pcSlice );
  }

#endif
  m_pcCuEncoder->getModeCtrl()->setFastDeltaQp(bFastDeltaQP);

  //------------------------------------------------------------------------------
  //  Weighted Prediction parameters estimation.
  //------------------------------------------------------------------------------
  // calculate AC/DC values for current picture
  if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
  {
    xCalcACDCParamSlice(pcSlice);
  }

  const Bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());

  if ( bWp_explicit )
  {
    //------------------------------------------------------------------------------
    //  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
    //------------------------------------------------------------------------------
#if HEVC_DEPENDENT_SLICES
    if ( pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES || pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES )
#else
    if(pcSlice->getSliceMode() == FIXED_NUMBER_OF_BYTES)
#endif
    {
      EXIT("Weighted Prediction is not yet supported with slice mode determined by max number of bins.");
    }

    xEstimateWPParamSlice( pcSlice, m_pcCfg->getWeightedPredictionMethod() );
    pcSlice->initWpScaling(pcSlice->getSPS());

    // check WP on/off
    xCheckWPEnable( pcSlice );
  }


#if HEVC_DEPENDENT_SLICES
#if HEVC_TILES_WPP
  // Adjust initial state if this is the start of a dependent slice.
  {
    const UInt      ctuRsAddr               = tileMap.getCtuTsToRsAddrMap( startCtuTsAddr);
    const UInt      currentTileIdx          = tileMap.getTileIdxMap(ctuRsAddr);
    const Tile&     currentTile             = tileMap.tiles[currentTileIdx];
    const UInt      firstCtuRsAddrOfTile    = currentTile.getFirstCtuRsAddr();
    if( pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != firstCtuRsAddrOfTile )
    {
      // This will only occur if dependent slice-segments (m_entropyCodingSyncContextState=true) are being used.
      if( currentTile.getTileWidthInCtus() >= 2 || !m_pcCfg->getEntropyCodingSyncEnabledFlag() )
      {
        m_CABACEstimator->getCtx() = m_lastSliceSegmentEndContextState;
        m_CABACEstimator->start();
      }
    }
  }
#else
  // KJS: not sure if this works (but both dep slices and tiles shall be removed in VTM, so this code should not be used)
  if( pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != startCtuTsAddr )
  {
    if( pcPic->cs->pcv->widthInCtus >= 2 || !m_pcCfg->getEntropyCodingSyncEnabledFlag() )
    {
      m_CABACEstimator->getCtx() = m_lastSliceSegmentEndContextState;
      m_CABACEstimator->start();
    }
#endif
#endif

#if HEVC_DEPENDENT_SLICES
  if( !pcSlice->getDependentSliceSegmentFlag() )
  {
#endif
    pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
  }
#endif

  CHECK( pcPic->m_prevQP[0] == std::numeric_limits<Int>::max(), "Invalid previous QP" );

  CodingStructure&  cs          = *pcPic->cs;
#if ENABLE_QPA || ENABLE_WPP_PARALLELISM
  const PreCalcValues& pcv      = *cs.pcv;
  const UInt        widthInCtus = pcv.widthInCtus;
#endif

  cs.slice = pcSlice;

  if (startCtuTsAddr == 0)
  {
    cs.initStructData (pcSlice->getSliceQp(), pcSlice->getPPS()->getTransquantBypassEnabledFlag());
  }

#if ENABLE_QPA
 #if ENABLE_QPA
  double hpEnerMax     = 1.0;
  double hpEnerPic     = 0.0;
 #endif
  Int    iSrcOffset;

 #if ENABLE_QPA
  if (m_pcCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP() && !m_pcCfg->getUseRateCtrl())
 #endif
  {
    for (UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++)
    {
 #if HEVC_TILES_WPP
      const UInt     ctuRsAddr  = tileMap.getCtuTsToRsAddrMap (ctuTsAddr);
 #else
      const UInt     ctuRsAddr  = ctuTsAddr;
 #endif
      const Position pos ((ctuRsAddr % widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / widthInCtus) * pcv.maxCUHeight);
      const CompArea subArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight)), pcPic->Y());
      const CompArea fltArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (pos.x > 0 ? pos.x - 1 : 0, pos.y > 0 ? pos.y - 1 : 0, pcv.maxCUWidth + (pos.x > 0 ? 2 : 1), pcv.maxCUHeight + (pos.y > 0 ? 2 : 1))), pcPic->Y());
      const SizeType iSrcStride = pcPic->getOrigBuf (subArea).stride;
      const Pel*     pSrc       = pcPic->getOrigBuf (subArea).buf;
      const SizeType iSrcHeight = pcPic->getOrigBuf (subArea).height;
      const SizeType iSrcWidth  = pcPic->getOrigBuf (subArea).width;
      const SizeType iFltHeight = pcPic->getOrigBuf (fltArea).height;
      const SizeType iFltWidth  = pcPic->getOrigBuf (fltArea).width;
      double hpEner = 0.0;

      DTRACE_UPDATE (g_trace_ctx, std::make_pair ("ctu", ctuRsAddr));

      // compute DC offset to be subtracted from luma values
      iSrcOffset = 0;
      for (SizeType h = 0; h < iSrcHeight; h++)
      {
        for (SizeType w = 0; w < iSrcWidth; w++)
        {
          iSrcOffset += pSrc[w];
        }
        pSrc += iSrcStride;
      }
      CHECK (iSrcOffset < 0, "DC offset cannot be negative!");

      Int x = iSrcHeight * iSrcWidth;
      iSrcOffset = (iSrcOffset + (x >> 1)) / x; // slow division

      filterAndCalculateAverageEnergies (pcPic->getOrigBuf (fltArea).buf, iSrcStride,
                                         hpEner, iFltHeight, iFltWidth, pcPic->getPOC());

 #if ENABLE_QPA
      if (hpEner > hpEnerMax) hpEnerMax = hpEner;
      hpEnerPic += hpEner;
      pcPic->m_uEnerHpCtu[ctuRsAddr] = hpEner;
      pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iSrcOffset;
 #endif
    } // end iteration over all CTUs in current slice

  }

 #if ENABLE_QPA
  if (m_pcCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP() && !m_pcCfg->getUseRateCtrl() && (boundingCtuTsAddr > startCtuTsAddr))
  {
    const double hpEnerAvg = hpEnerPic / double(boundingCtuTsAddr - startCtuTsAddr);

    if (applyQPAdaptation (pcPic, pcSlice, pcv, startCtuTsAddr, boundingCtuTsAddr, m_pcCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_NUM_MODES, m_pcCfg->getGOPSize(), hpEnerAvg, hpEnerMax))
    {
      m_CABACEstimator->initCtxModels (*pcSlice);
  #if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
      for (int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++)
      {
        CABACWriter* cw = m_pcLib->getCABACEncoder (jId)->getCABACEstimator (pcSlice->getSPS());
        cw->initCtxModels (*pcSlice);
      }
  #endif
#if HEVC_DEPENDENT_SLICES
      if (!pcSlice->getDependentSliceSegmentFlag())
      {
#endif
        pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
      }
#endif
      if (startCtuTsAddr == 0)
      {
        cs.currQP[0] = cs.currQP[1] = pcSlice->getSliceQp(); // cf code above
      }
    }
  }
 #endif // ENABLE_QPA

#endif // ENABLE_QPA || ENABLE_PRIVATE

  cs.pcv      = pcSlice->getPPS()->pcv;
  cs.fracBits = 0;


#if ENABLE_WPP_PARALLELISM
  bool bUseThreads = m_pcCfg->getNumWppThreads() > 1;
  if( bUseThreads )
  {
    CHECK( startCtuTsAddr != 0 || boundingCtuTsAddr != pcPic->cs->pcv->sizeInCtus, "not intended" );

    pcPic->cs->allocateVectorsAtPicLevel();

    omp_set_num_threads( m_pcCfg->getNumWppThreads() + m_pcCfg->getNumWppExtraLines() );

    #pragma omp parallel for schedule(static,1) if(bUseThreads)
    for( int ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr += widthInCtus )
    {
      // wpp thread start
      pcPic->scheduler.setWppThreadId();
#if ENABLE_SPLIT_PARALLELISM
      pcPic->scheduler.setSplitThreadId( 0 );
#endif
      encodeCtus( pcPic, bCompressEntireSlice, bFastDeltaQP, ctuTsAddr, ctuTsAddr + widthInCtus, m_pcLib );
      // wpp thread stop
    }
  }
  else
#endif
  encodeCtus( pcPic, bCompressEntireSlice, bFastDeltaQP, startCtuTsAddr, boundingCtuTsAddr, m_pcLib );

#if HEVC_DEPENDENT_SLICES
  // store context state at the end of this slice-segment, in case the next slice is a dependent slice and continues using the CABAC contexts.
  if( pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag() )
  {
    m_lastSliceSegmentEndContextState = m_CABACEstimator->getCtx();//ctx end of dep.slice
  }
#endif

}


Void EncSlice::encodeCtus( Picture* pcPic, const Bool bCompressEntireSlice, const Bool bFastDeltaQP, UInt startCtuTsAddr, UInt boundingCtuTsAddr, EncLib* pEncLib )
{
  //PROF_ACCUM_AND_START_NEW_SET( getProfilerCTU( pcPic, 0, 0 ), P_PIC_LEVEL );
  //PROF_START( getProfilerCTU( cs.slice->isIntra(), pcPic->scheduler.getWppThreadId() ), P_PIC_LEVEL, toWSizeIdx( cs.pcv->maxCUWidth ), toHSizeIdx( cs.pcv->maxCUHeight ) );
  CodingStructure&  cs            = *pcPic->cs;
  Slice* pcSlice                  = cs.slice;
  const PreCalcValues& pcv        = *cs.pcv;
  const UInt        widthInCtus   = pcv.widthInCtus;
#if HEVC_TILES_WPP
  const TileMap&  tileMap         = *pcPic->tileMap;
#endif
#if ENABLE_QPA
  const Int iQPIndex              = pcSlice->getSliceQpBase();
  int iSrcOffset                  = 0;
#endif

#if ENABLE_WPP_PARALLELISM
  const int       dataId          = pcPic->scheduler.getWppDataId();
#elif ENABLE_SPLIT_PARALLELISM
  const int       dataId          = 0;
#endif
  CABACWriter*    pCABACWriter    = pEncLib->getCABACEncoder( PARL_PARAM0( dataId ) )->getCABACEstimator( pcSlice->getSPS() );
  TrQuant*        pTrQuant        = pEncLib->getTrQuant( PARL_PARAM0( dataId ) );
  RdCost*         pRdCost         = pEncLib->getRdCost( PARL_PARAM0( dataId ) );
  EncCfg*         pCfg            = pEncLib;
  RateCtrl*       pRateCtrl       = pEncLib->getRateCtrl();
#if ENABLE_WPP_PARALLELISM
  // first version dont use ctx from above
  pCABACWriter->initCtxModels( *pcSlice );
#endif
#if RDOQ_CHROMA_LAMBDA
  pTrQuant    ->setLambdas( pcSlice->getLambdas() );
#else
  pTrQuant    ->setLambda ( pcSlice->getLambdas()[0] );
#endif
  pRdCost     ->setLambda ( pcSlice->getLambdas()[0], pcSlice->getSPS()->getBitDepths() );

  int prevQP[2];
  int currQP[2];
  prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
  currQP[0] = currQP[1] = pcSlice->getSliceQp();

#if HEVC_DEPENDENT_SLICES
  if( !pcSlice->getDependentSliceSegmentFlag() )
  {
#endif
    prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
  }
#endif
  // for every CTU in the slice segment (may terminate sooner if there is a byte limit on the slice-segment)
  for( UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++ )
  {
#if HEVC_TILES_WPP
    const UInt ctuRsAddr = tileMap.getCtuTsToRsAddrMap(ctuTsAddr);
#else
    const UInt ctuRsAddr = ctuTsAddr;
#endif

#if HEVC_TILES_WPP
    // update CABAC state
    const UInt firstCtuRsAddrOfTile = tileMap.tiles[tileMap.getTileIdxMap(ctuRsAddr)].getFirstCtuRsAddr();
    const UInt tileXPosInCtus       = firstCtuRsAddrOfTile % widthInCtus;
#endif
    const UInt ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const UInt ctuYPosInCtus        = ctuRsAddr / widthInCtus;

    const Position pos (ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
    const UnitArea ctuArea( cs.area.chromaFormat, Area( pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight ) );
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

	//cout << ctuArea.lumaPos().x<<"-"<<ctuArea.lumaPos().y<<"  ";
	//cout << pcv.maxCUHeight << "-" << pcv.maxCUWidth << " ";
#if ENABLE_WPP_PARALLELISM
    pcPic->scheduler.wait( ctuXPosInCtus, ctuYPosInCtus );
#endif

#if HEVC_TILES_WPP
    if (ctuRsAddr == firstCtuRsAddrOfTile)
    {
      pCABACWriter->initCtxModels( *pcSlice );
      prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
    }
    else if (ctuXPosInCtus == tileXPosInCtus && pEncLib->getEntropyCodingSyncEnabledFlag())
    {
      // reset and then update contexts to the state at the end of the top-right CTU (if within current slice and tile).
      pCABACWriter->initCtxModels( *pcSlice );
      if( cs.getCURestricted( pos.offset(pcv.maxCUWidth, -1), pcSlice->getIndependentSliceIdx(), tileMap.getTileIdxMap( pos ), CH_L ) )
      {
        // Top-right is available, we use it.
        pCABACWriter->getCtx() = pEncLib->m_entropyCodingSyncContextState;
      }
      prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
    }
#endif

#if ENABLE_WPP_PARALLELISM
    if( ctuXPosInCtus == 0 && ctuYPosInCtus > 0 && widthInCtus > 1 && ( pEncLib->getNumWppThreads() > 1 || pEncLib->getEnsureWppBitEqual() ) )
    {
      pCABACWriter->getCtx() = pEncLib->m_entropyCodingSyncContextStateVec[ctuYPosInCtus-1];  // last line
    }
#else
#endif

#if RDOQ_CHROMA_LAMBDA && ENABLE_QPA
    Double oldLambdaArray[MAX_NUM_COMPONENT] = {0.0};
#endif
    const Double oldLambda = pRdCost->getLambda();
    if ( pCfg->getUseRateCtrl() )
    {
      Int estQP        = pcSlice->getSliceQp();
      Double estLambda = -1.0;
      Double bpp       = -1.0;

      if( ( pcPic->slices[0]->getSliceType() == I_SLICE && pCfg->getForceIntraQP() ) || !pCfg->getLCULevelRC() )
      {
        estQP = pcSlice->getSliceQp();
      }
      else
      {
        bpp = pRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->getSliceType());
        if ( pcPic->slices[0]->getSliceType() == I_SLICE)
        {
          estLambda = pRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
        }
        else
        {
          estLambda = pRateCtrl->getRCPic()->getLCUEstLambda( bpp );
          estQP     = pRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );
        }
		
        estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP );

        pRdCost->setLambda(estLambda, pcSlice->getSPS()->getBitDepths());

#if RDOQ_CHROMA_LAMBDA
        // set lambda for RDOQ
        const Double chromaLambda = estLambda / pRdCost->getChromaWeight();
        const Double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
        pTrQuant->setLambdas( lambdaArray );
#else
        pTrQuant->setLambda( estLambda );
#endif
      }

      pRateCtrl->setRCQP( estQP );
    }
#if ENABLE_QPA
    else if (pCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
    {
      iSrcOffset = pcPic->m_iOffsetCtu[ctuRsAddr];
      const Double newLambda = oldLambda * pow (2.0, Double(iSrcOffset - iQPIndex) / 3.0);
#if RDOQ_CHROMA_LAMBDA
      pTrQuant->getLambdas (oldLambdaArray); // save the old lambdas
      const Double chromaLambda = newLambda / pRdCost->getChromaWeight();
      const Double lambdaArray[MAX_NUM_COMPONENT] = {newLambda, chromaLambda, chromaLambda};
      pTrQuant->setLambdas (lambdaArray);
#else
      pTrQuant->setLambda (newLambda);
#endif
      pRdCost->setLambda (newLambda, pcSlice->getSPS()->getBitDepths());
      currQP[0] = currQP[1] = iSrcOffset;
    }
#endif


#if ENABLE_WPP_PARALLELISM
    pEncLib->getCuEncoder( dataId )->compressCtu( cs, ctuArea, ctuRsAddr, prevQP, currQP );
#else
    m_pcCuEncoder->compressCtu( cs, ctuArea, ctuRsAddr, prevQP, currQP );
#endif

	
    pCABACWriter->resetBits();
    pCABACWriter->coding_tree_unit( cs, ctuArea, prevQP, ctuRsAddr, true );
    const int numberOfWrittenBits = int( pCABACWriter->getEstFracBits() >> SCALE_BITS );

    // Calculate if this CTU puts us over slice bit size.
    // cannot terminate if current slice/slice-segment would be 0 Ctu in size,
    const UInt validEndOfSliceCtuTsAddr = ctuTsAddr + (ctuTsAddr == startCtuTsAddr ? 1 : 0);
    // Set slice end parameter
    if(pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceBits()+numberOfWrittenBits > (pcSlice->getSliceArgument()<<3))
    {
#if HEVC_DEPENDENT_SLICES
      pcSlice->setSliceSegmentCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
#endif
      pcSlice->setSliceCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
      boundingCtuTsAddr=validEndOfSliceCtuTsAddr;
    }
#if HEVC_DEPENDENT_SLICES
    else if((!bCompressEntireSlice) && pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits()+numberOfWrittenBits > (pcSlice->getSliceSegmentArgument()<<3))
    {
      pcSlice->setSliceSegmentCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
      boundingCtuTsAddr=validEndOfSliceCtuTsAddr;
    }
#endif
    if (boundingCtuTsAddr <= ctuTsAddr)
    {
      break;
    }

#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#pragma omp critical
#endif
    pcSlice->setSliceBits( ( UInt ) ( pcSlice->getSliceBits() + numberOfWrittenBits ) );
#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#pragma omp critical
#endif
#if HEVC_DEPENDENT_SLICES
    pcSlice->setSliceSegmentBits( pcSlice->getSliceSegmentBits() + numberOfWrittenBits );
#endif

#if HEVC_TILES_WPP
    // Store probabilities of second CTU in line into buffer - used only if wavefront-parallel-processing is enabled.
    if( ctuXPosInCtus == tileXPosInCtus + 1 && pEncLib->getEntropyCodingSyncEnabledFlag() )
    {
      pEncLib->m_entropyCodingSyncContextState = pCABACWriter->getCtx();
    }
#endif
#if ENABLE_WPP_PARALLELISM
    if( ctuXPosInCtus == 1 && ( pEncLib->getNumWppThreads() > 1 || pEncLib->getEnsureWppBitEqual() ) )
    {
      pEncLib->m_entropyCodingSyncContextStateVec[ctuYPosInCtus] = pCABACWriter->getCtx();
    }
#endif

#if !ENABLE_WPP_PARALLELISM
    Int actualBits = int(cs.fracBits >> SCALE_BITS);
#endif
    if ( pCfg->getUseRateCtrl() )
    {
#if ENABLE_WPP_PARALLELISM
      Int actualBits      = int( cs.fracBits >> SCALE_BITS );
#endif
      Int actualQP        = g_RCInvalidQPValue;
      Double actualLambda = pRdCost->getLambda();
      Int numberOfEffectivePixels    = 0;

      for( auto &cu : cs.traverseCUs( ctuArea, CH_L ) )
      {
        if( !cu.skip || cu.rootCbf )
        {
          numberOfEffectivePixels += cu.lumaSize().area();
          break;
        }
      }

      CodingUnit* cu = cs.getCU( ctuArea.lumaPos(), CH_L );

      if ( numberOfEffectivePixels == 0 )
      {
        actualQP = g_RCInvalidQPValue;
      }
      else
      {
        actualQP = cu->qp;
      }
      pRdCost->setLambda(oldLambda, pcSlice->getSPS()->getBitDepths());
      pRateCtrl->getRCPic()->updateAfterCTU( pRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
                                             pcSlice->getSliceType() == I_SLICE ? 0 : pCfg->getLCULevelRC() );
    }
#if ENABLE_QPA
    else if (pCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
    {
#if RDOQ_CHROMA_LAMBDA
      pTrQuant->setLambdas (oldLambdaArray);
#else
      pTrQuant->setLambda (oldLambda);
#endif
      pRdCost->setLambda (oldLambda, pcSlice->getSPS()->getBitDepths());
    }
#endif

#if !ENABLE_WPP_PARALLELISM
    m_uiPicTotalBits += actualBits;
    m_uiPicDist       = cs.dist;
#endif
#if ENABLE_WPP_PARALLELISM
    pcPic->scheduler.setReady( ctuXPosInCtus, ctuYPosInCtus );
#endif
  }

  // this is wpp exclusive section

//  m_uiPicTotalBits += actualBits;
//  m_uiPicDist       = cs.dist;

}

Void EncSlice::encodeSlice   ( Picture* pcPic, OutputBitstream* pcSubstreams, UInt &numBinsCoded )
{

  Slice *const pcSlice               = pcPic->slices[getSliceSegmentIdx()];
#if HEVC_TILES_WPP
  const TileMap& tileMap             = *pcPic->tileMap;
#endif
#if HEVC_DEPENDENT_SLICES
  const UInt startCtuTsAddr          = pcSlice->getSliceSegmentCurStartCtuTsAddr();
  const UInt boundingCtuTsAddr       = pcSlice->getSliceSegmentCurEndCtuTsAddr();
  const Bool depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
#else
  const UInt startCtuTsAddr          = pcSlice->getSliceCurStartCtuTsAddr();
  const UInt boundingCtuTsAddr       = pcSlice->getSliceCurEndCtuTsAddr();
#endif
#if HEVC_TILES_WPP
  const Bool wavefrontsEnabled       = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag();
#endif


  // setup coding structure
  CodingStructure& cs = *pcPic->cs;
  cs.slice            = pcSlice;
  // initialise entropy coder for the slice
  m_CABACWriter->initCtxModels( *pcSlice );

  DTRACE( g_trace_ctx, D_HEADER, "=========== POC: %d ===========\n", pcSlice->getPOC() );

#if HEVC_DEPENDENT_SLICES
  if (depSliceSegmentsEnabled)
  {
#if HEVC_TILES_WPP
    // modify initial contexts with previous slice segment if this is a dependent slice.
    const UInt ctuRsAddr            = tileMap.getCtuTsToRsAddrMap( startCtuTsAddr );
    const UInt currentTileIdx       = tileMap.getTileIdxMap(ctuRsAddr);
    const Tile& currentTile         = tileMap.tiles[currentTileIdx];
    const UInt firstCtuRsAddrOfTile = currentTile.getFirstCtuRsAddr();

    if( pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != firstCtuRsAddrOfTile )
    {
      if( currentTile.getTileWidthInCtus() >= 2 || !wavefrontsEnabled )
      {
        m_CABACWriter->getCtx() = m_lastSliceSegmentEndContextState;
      }
    }
#else
  // KJS: not sure if this works (but both dep slices and tiles shall be removed in VTM, so this code should not be used)
  if( pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != startCtuTsAddr )
  {
    if( pcPic->cs->pcv->widthInCtus >= 2 || !m_pcCfg->getEntropyCodingSyncEnabledFlag() )
    {
        m_CABACWriter->getCtx() = m_lastSliceSegmentEndContextState;
    }
#endif
  }

  if( !pcSlice->getDependentSliceSegmentFlag() )
  {
#endif
    pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
  }
#endif

  const PreCalcValues& pcv = *cs.pcv;
  const UInt widthInCtus   = pcv.widthInCtus;

  // for every CTU in the slice segment...

  for( UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++ )
  {
#if HEVC_TILES_WPP
    const UInt ctuRsAddr            = tileMap.getCtuTsToRsAddrMap(ctuTsAddr);
    const Tile& currentTile         = tileMap.tiles[tileMap.getTileIdxMap(ctuRsAddr)];
    const UInt firstCtuRsAddrOfTile = currentTile.getFirstCtuRsAddr();
    const UInt tileXPosInCtus       = firstCtuRsAddrOfTile % widthInCtus;
    const UInt tileYPosInCtus       = firstCtuRsAddrOfTile / widthInCtus;
#else
    const UInt ctuRsAddr            = ctuTsAddr;
#endif
    const UInt ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const UInt ctuYPosInCtus        = ctuRsAddr / widthInCtus;
#if HEVC_TILES_WPP
    const UInt uiSubStrm            = tileMap.getSubstreamForCtuAddr(ctuRsAddr, true, pcSlice);
#else
    const UInt uiSubStrm            = 0;
#endif

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    const Position pos (ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
    const UnitArea ctuArea (cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight));
    m_CABACWriter->initBitstream( &pcSubstreams[uiSubStrm] );

#if HEVC_TILES_WPP
    // set up CABAC contexts' state for this CTU
    if (ctuRsAddr == firstCtuRsAddrOfTile)
    {
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_CABACWriter->initCtxModels( *pcSlice );
      }
    }
    else if (ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled)
    {
      // Synchronize cabac probabilities with upper-right CTU if it's available and at the start of a line.
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_CABACWriter->initCtxModels( *pcSlice );
      }
      if( cs.getCURestricted( pos.offset( pcv.maxCUWidth, -1 ), pcSlice->getIndependentSliceIdx(), tileMap.getTileIdxMap( pos ), CH_L ) )
      {
        // Top-right is available, so use it.
        m_CABACWriter->getCtx() = m_entropyCodingSyncContextState;
      }
    }
#endif
    m_CABACWriter->coding_tree_unit( cs, ctuArea, pcPic->m_prevQP, ctuRsAddr );

#if HEVC_TILES_WPP
    // store probabilities of second CTU in line into buffer
    if( ctuXPosInCtus == tileXPosInCtus + 1 && wavefrontsEnabled )
    {
      m_entropyCodingSyncContextState = m_CABACWriter->getCtx();
    }
#endif

    // terminate the sub-stream, if required (end of slice-segment, end of tile, end of wavefront-CTU-row):
#if HEVC_TILES_WPP
    if( ctuTsAddr + 1 == boundingCtuTsAddr ||
         (  ctuXPosInCtus + 1 == tileXPosInCtus + currentTile.getTileWidthInCtus () &&
          ( ctuYPosInCtus + 1 == tileYPosInCtus + currentTile.getTileHeightInCtus() || wavefrontsEnabled )
         )
       )
#else
    if( ctuTsAddr + 1 == boundingCtuTsAddr )
#endif
    {
      m_CABACWriter->end_of_slice();

      // Byte-alignment in slice_data() when new tile
      pcSubstreams[uiSubStrm].writeByteAlignment();

      // write sub-stream size
      if( ctuTsAddr + 1 != boundingCtuTsAddr )
      {
        pcSlice->addSubstreamSize( (pcSubstreams[uiSubStrm].getNumberOfWrittenBits() >> 3) + pcSubstreams[uiSubStrm].countStartCodeEmulations() );
      }
    }
  } // CTU-loop

#if HEVC_DEPENDENT_SLICES
  if( depSliceSegmentsEnabled )
  {
    m_lastSliceSegmentEndContextState = m_CABACWriter->getCtx();//ctx end of dep.slice
  }
#endif

#if HEVC_DEPENDENT_SLICES
  if (pcSlice->getPPS()->getCabacInitPresentFlag() && !pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag())
#else
  if(pcSlice->getPPS()->getCabacInitPresentFlag())
#endif
  {
    m_encCABACTableIdx = m_CABACWriter->getCtxInitId( *pcSlice );
  }
  else
  {
    m_encCABACTableIdx = pcSlice->getSliceType();
  }
  numBinsCoded = m_CABACWriter->getNumBins();

}

#if HEVC_TILES_WPP
Void EncSlice::calculateBoundingCtuTsAddrForSlice(UInt &startCtuTSAddrSlice, UInt &boundingCtuTSAddrSlice, Bool &haveReachedTileBoundary,
                                                   Picture* pcPic, const Int sliceMode, const Int sliceArgument)
#else
Void EncSlice::calculateBoundingCtuTsAddrForSlice(UInt &startCtuTSAddrSlice, UInt &boundingCtuTSAddrSlice,
                                                   Picture* pcPic, const Int sliceMode, const Int sliceArgument)
#endif
{
#if HEVC_TILES_WPP
  Slice* pcSlice = pcPic->slices[getSliceSegmentIdx()];
  const TileMap& tileMap = *( pcPic->tileMap );
  const PPS &pps         = *( pcSlice->getPPS() );
#endif
  const UInt numberOfCtusInFrame = pcPic->cs->pcv->sizeInCtus;
  boundingCtuTSAddrSlice=0;
#if HEVC_TILES_WPP
  haveReachedTileBoundary=false;
#endif

  switch (sliceMode)
  {
    case FIXED_NUMBER_OF_CTU:
      {
        UInt ctuAddrIncrement    = sliceArgument;
        boundingCtuTSAddrSlice  = ((startCtuTSAddrSlice + ctuAddrIncrement) < numberOfCtusInFrame) ? (startCtuTSAddrSlice + ctuAddrIncrement) : numberOfCtusInFrame;
      }
      break;
    case FIXED_NUMBER_OF_BYTES:
      boundingCtuTSAddrSlice  = numberOfCtusInFrame; // This will be adjusted later if required.
      break;
#if HEVC_TILES_WPP
    case FIXED_NUMBER_OF_TILES:
      {
        const UInt tileIdx        = tileMap.getTileIdxMap( tileMap.getCtuTsToRsAddrMap(startCtuTSAddrSlice) );
        const UInt tileTotalCount = (pps.getNumTileColumnsMinus1()+1) * (pps.getNumTileRowsMinus1()+1);
        UInt ctuAddrIncrement   = 0;

        for(UInt tileIdxIncrement = 0; tileIdxIncrement < sliceArgument; tileIdxIncrement++)
        {
          if((tileIdx + tileIdxIncrement) < tileTotalCount)
          {
            UInt tileWidthInCtus    = tileMap.tiles[tileIdx + tileIdxIncrement].getTileWidthInCtus();
            UInt tileHeightInCtus   = tileMap.tiles[tileIdx + tileIdxIncrement].getTileHeightInCtus();
            ctuAddrIncrement       += (tileWidthInCtus * tileHeightInCtus);
          }
        }

        boundingCtuTSAddrSlice  = ((startCtuTSAddrSlice + ctuAddrIncrement) < numberOfCtusInFrame) ? (startCtuTSAddrSlice + ctuAddrIncrement) : numberOfCtusInFrame;
      }
      break;
#endif
    default:
      boundingCtuTSAddrSlice    = numberOfCtusInFrame;
      break;
  }

#if HEVC_TILES_WPP
  // Adjust for tiles and wavefronts.
  const Bool wavefrontsAreEnabled = pps.getEntropyCodingSyncEnabledFlag();

  if ((sliceMode == FIXED_NUMBER_OF_CTU || sliceMode == FIXED_NUMBER_OF_BYTES) &&
      (pps.getNumTileRowsMinus1() > 0 || pps.getNumTileColumnsMinus1() > 0))
  {
    const UInt ctuRsAddr                   = tileMap.getCtuTsToRsAddrMap(startCtuTSAddrSlice);
    const UInt startTileIdx                = tileMap.getTileIdxMap(ctuRsAddr);
    const Tile& startingTile               = tileMap.tiles[startTileIdx];
    const UInt  tileStartTsAddr            = tileMap.getCtuRsToTsAddrMap(startingTile.getFirstCtuRsAddr());
    const UInt  tileStartWidth             = startingTile.getTileWidthInCtus();
    const UInt  tileStartHeight            = startingTile.getTileHeightInCtus();
    const UInt tileLastTsAddr_excl        = tileStartTsAddr + tileStartWidth*tileStartHeight;
    const UInt tileBoundingCtuTsAddrSlice = tileLastTsAddr_excl;
    const UInt ctuColumnOfStartingTile     = ((startCtuTSAddrSlice-tileStartTsAddr)%tileStartWidth);
    if (wavefrontsAreEnabled && ctuColumnOfStartingTile!=0)
    {
      // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
      const UInt numberOfCTUsToEndOfRow            = tileStartWidth - ctuColumnOfStartingTile;
      const UInt wavefrontTileBoundingCtuAddrSlice = startCtuTSAddrSlice + numberOfCTUsToEndOfRow;
      if (wavefrontTileBoundingCtuAddrSlice < boundingCtuTSAddrSlice)
      {
        boundingCtuTSAddrSlice = wavefrontTileBoundingCtuAddrSlice;
      }
    }

    if (tileBoundingCtuTsAddrSlice < boundingCtuTSAddrSlice)
    {
      boundingCtuTSAddrSlice = tileBoundingCtuTsAddrSlice;
      haveReachedTileBoundary = true;
    }
  }
  else if ((sliceMode == FIXED_NUMBER_OF_CTU || sliceMode == FIXED_NUMBER_OF_BYTES) && wavefrontsAreEnabled && ((startCtuTSAddrSlice % pcPic->cs->pcv->widthInCtus) != 0))
  {
    // Adjust for wavefronts (no tiles).
    // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
    boundingCtuTSAddrSlice = min(boundingCtuTSAddrSlice, startCtuTSAddrSlice - (startCtuTSAddrSlice % pcPic->cs->pcv->widthInCtus) + (pcPic->cs->pcv->widthInCtus));
  }
#endif
}

/** Determines the starting and bounding CTU address of current slice / dependent slice
 * \param [out] startCtuTsAddr
 * \param [out] boundingCtuTsAddr
 * \param [in]  pcPic

 * Updates startCtuTsAddr, boundingCtuTsAddr with appropriate CTU address
 */
Void EncSlice::xDetermineStartAndBoundingCtuTsAddr  ( UInt& startCtuTsAddr, UInt& boundingCtuTsAddr, Picture* pcPic )
{
  Slice* pcSlice                 = pcPic->slices[getSliceSegmentIdx()];

  // Non-dependent slice
  UInt startCtuTsAddrSlice           = pcSlice->getSliceCurStartCtuTsAddr();
#if HEVC_TILES_WPP
  Bool haveReachedTileBoundarySlice  = false;
#endif
  UInt boundingCtuTsAddrSlice;
#if HEVC_TILES_WPP
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSlice, boundingCtuTsAddrSlice, haveReachedTileBoundarySlice, pcPic,
                                     m_pcCfg->getSliceMode(), m_pcCfg->getSliceArgument());
#else
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSlice, boundingCtuTsAddrSlice, pcPic,
                                     m_pcCfg->getSliceMode(), m_pcCfg->getSliceArgument());
#endif
  pcSlice->setSliceCurEndCtuTsAddr(   boundingCtuTsAddrSlice );
  pcSlice->setSliceCurStartCtuTsAddr( startCtuTsAddrSlice    );

#if HEVC_DEPENDENT_SLICES
  // Dependent slice
  UInt startCtuTsAddrSliceSegment          = pcSlice->getSliceSegmentCurStartCtuTsAddr();
#if HEVC_TILES_WPP
  Bool haveReachedTileBoundarySliceSegment = false;
#endif
  UInt boundingCtuTsAddrSliceSegment;
#if HEVC_TILES_WPP
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSliceSegment, boundingCtuTsAddrSliceSegment, haveReachedTileBoundarySliceSegment, pcPic,
                                     m_pcCfg->getSliceSegmentMode(), m_pcCfg->getSliceSegmentArgument());
#else
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSliceSegment, boundingCtuTsAddrSliceSegment, pcPic,
                                     m_pcCfg->getSliceSegmentMode(), m_pcCfg->getSliceSegmentArgument());
#endif
  if (boundingCtuTsAddrSliceSegment>boundingCtuTsAddrSlice)
  {
    boundingCtuTsAddrSliceSegment = boundingCtuTsAddrSlice;
  }
  pcSlice->setSliceSegmentCurEndCtuTsAddr( boundingCtuTsAddrSliceSegment );
  pcSlice->setSliceSegmentCurStartCtuTsAddr(startCtuTsAddrSliceSegment);

  // Make a joint decision based on reconstruction and dependent slice bounds
  startCtuTsAddr    = max(startCtuTsAddrSlice   , startCtuTsAddrSliceSegment   );
  boundingCtuTsAddr = boundingCtuTsAddrSliceSegment;
#else
  startCtuTsAddr = startCtuTsAddrSlice;
  boundingCtuTsAddr = boundingCtuTsAddrSlice;
#endif
}

Double EncSlice::xGetQPValueAccordingToLambda ( Double lambda )
{
  return 4.2005*log(lambda) + 13.7122;
}

//! \}
