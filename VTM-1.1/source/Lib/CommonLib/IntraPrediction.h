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

/** \file     IntraPrediction.h
    \brief    prediction class (header)
*/

#ifndef __INTRAPREDICTION__
#define __INTRAPREDICTION__


// Include files
#include "Unit.h"
#include "Buffer.h"
#include "Picture.h"


//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
enum PredBuf
{
  PRED_BUF_UNFILTERED = 0,
  PRED_BUF_FILTERED   = 1,
  NUM_PRED_BUF        = 2
};

static const UInt MAX_INTRA_FILTER_DEPTHS=8;

class IntraPrediction
{
private:

  Pel* m_piYuvExt[MAX_NUM_COMPONENT][NUM_PRED_BUF];
  Int  m_iYuvExtSize;


  static const UChar m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS];

  Pel* m_piTemp;

protected:

  ChromaFormat  m_currChromaFormat;

  // prediction
  Void xPredIntraPlanar           ( const CPelBuf &pSrc, PelBuf &pDst,                                                                                                         const SPS& sps );
  Void xPredIntraDc               ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType,                                                                                          const bool enableBoundaryFilter = true );
#if HEVC_USE_HOR_VER_PREDFILTERING
  Void xPredIntraAng              ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const UInt dirMode, const ClpRng& clpRng, const Bool bEnableEdgeFilters, const SPS& sps, const bool enableBoundaryFilter = true );
#else
  Void xPredIntraAng              ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const UInt dirMode, const ClpRng& clpRng, const SPS& sps, const bool enableBoundaryFilter = true );
#endif
  Pel  xGetPredValDc              ( const CPelBuf &pSrc, const Size &dstSize );

  void xFillReferenceSamples      ( const CPelBuf &recoBuf,      Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu );
  void xFilterReferenceSamples    ( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea &area, const SPS &sps );

#if HEVC_USE_DC_PREDFILTERING
  // dc filtering
  Void xDCPredFiltering           ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType &channelType );
#endif
  Void xReferenceFilter           ( const int doubleSize, const int origWeight, const int filterOrder, Pel *piRefVector, Pel *piLowPassRef );

  Void destroy                    ();

  Void xFilterGroup               ( Pel* pMulDst[], Int i, Pel const* const piSrc, Int iRecStride, Bool bAboveAvaillable, Bool bLeftAvaillable);
public:
  IntraPrediction();
  virtual ~IntraPrediction();

  Void init                       (ChromaFormat chromaFormatIDC, const unsigned bitDepthY);

  // Angular Intra
  void predIntraAng               ( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu, const bool useFilteredPredSamples );
  Pel*  getPredictorPtr           (const ComponentID compID, const Bool bUseFilteredPredictions = false) { return m_piYuvExt[compID][bUseFilteredPredictions?PRED_BUF_FILTERED:PRED_BUF_UNFILTERED]; }
  /// set parameters from CU data for accessing intra data
  Void initIntraPatternChType     (const CodingUnit &cu, const CompArea &area, const Bool bFilterRefSamples = false );

static bool useFilteredIntraRefSamples( const ComponentID &compID, const PredictionUnit &pu, bool modeSpecific, const UnitArea &tuArea );
  static Bool useDPCMForFirstPassIntraEstimation(const PredictionUnit &pu, const UInt &uiDirMode);
};

//! \}

#endif // __INTRAPREDICTION__
