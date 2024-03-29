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

/** \file     EncSearch.cpp
 *  \brief    encoder inter search class
 */

#include "InterSearch.h"


#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "CommonLib/MotionInfo.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"


#include "EncModeCtrl.h"
#include "EncLib.h"

#include <math.h>
#include <limits>


 //! \ingroup EncoderLib
 //! \{

static const Mv s_acMvRefineH[9] =
{
  Mv(  0,  0 ), // 0
  Mv(  0, -1 ), // 1
  Mv(  0,  1 ), // 2
  Mv( -1,  0 ), // 3
  Mv(  1,  0 ), // 4
  Mv( -1, -1 ), // 5
  Mv(  1, -1 ), // 6
  Mv( -1,  1 ), // 7
  Mv(  1,  1 )  // 8
};

static const Mv s_acMvRefineQ[9] =
{
  Mv(  0,  0 ), // 0
  Mv(  0, -1 ), // 1
  Mv(  0,  1 ), // 2
  Mv( -1, -1 ), // 5
  Mv(  1, -1 ), // 6
  Mv( -1,  0 ), // 3
  Mv(  1,  0 ), // 4
  Mv( -1,  1 ), // 7
  Mv(  1,  1 )  // 8
};


InterSearch::InterSearch()
  : m_modeCtrl                    (nullptr)
  , m_pSplitCS                    (nullptr)
  , m_pFullCS                     (nullptr)
  , m_pcEncCfg                    (nullptr)
  , m_pcTrQuant                   (nullptr)
  , m_iSearchRange                (0)
  , m_bipredSearchRange           (0)
  , m_motionEstimationSearchMethod(MESEARCH_FULL)
  , m_CABACEstimator              (nullptr)
  , m_CtxCache                    (nullptr)
  , m_pTempPel                    (nullptr)
  , m_isInitialized               (false)
{
  for (Int i=0; i<MAX_NUM_REF_LIST_ADAPT_SR; i++)
  {
    memset (m_aaiAdaptSR[i], 0, MAX_IDX_ADAPT_SR * sizeof (Int));
  }
  for (Int i=0; i<AMVP_MAX_NUM_CANDS+1; i++)
  {
    memset (m_auiMVPIdxCost[i], 0, (AMVP_MAX_NUM_CANDS+1) * sizeof (UInt) );
  }

  setWpScalingDistParam( -1, REF_PIC_LIST_X, nullptr );
}


Void InterSearch::destroy()
{
  CHECK(!m_isInitialized, "Not initialized");
  if ( m_pTempPel )
  {
    delete [] m_pTempPel;
    m_pTempPel = NULL;
  }

  m_pSplitCS = m_pFullCS = nullptr;

  m_pSaveCS = nullptr;

  for(UInt i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    m_tmpPredStorage[i].destroy();
  }
  m_tmpStorageLCU.destroy();
  m_tmpAffiStorage.destroy();

  if ( m_tmpAffiError != NULL )
  {
    delete[] m_tmpAffiError;
  }
  if ( m_tmpAffiDeri[0] != NULL )
  {
    delete[] m_tmpAffiDeri[0];
  }
  if ( m_tmpAffiDeri[1] != NULL )
  {
    delete[] m_tmpAffiDeri[1];
  }
  m_isInitialized = false;
}

Void InterSearch::setTempBuffers( CodingStructure ****pSplitCS, CodingStructure ****pFullCS, CodingStructure **pSaveCS )
{
  m_pSplitCS = pSplitCS;
  m_pFullCS  = pFullCS;
  m_pSaveCS  = pSaveCS;
}

#if ENABLE_SPLIT_PARALLELISM
Void InterSearch::copyState( const InterSearch& other )
{
  if( !m_pcEncCfg->getQTBT() )
  {
    memcpy( m_integerMv2Nx2N, other.m_integerMv2Nx2N, sizeof( m_integerMv2Nx2N ) );
  }

  memcpy( m_aaiAdaptSR, other.m_aaiAdaptSR, sizeof( m_aaiAdaptSR ) );
}
#endif

InterSearch::~InterSearch()
{
  if (m_isInitialized)
  {
    destroy();
  }
}

Void InterSearch::init( EncCfg*        pcEncCfg,
                        TrQuant*       pcTrQuant,
                        Int            iSearchRange,
                        Int            bipredSearchRange,
                        MESearchMethod motionEstimationSearchMethod,
                        const UInt     maxCUWidth,
                        const UInt     maxCUHeight,
                        const UInt     maxTotalCUDepth,
                        RdCost*        pcRdCost,
                        CABACWriter*   CABACEstimator,
                        CtxCache*      ctxCache
)
{
  CHECK(m_isInitialized, "Already initialized");
  m_pcEncCfg                     = pcEncCfg;
  m_pcTrQuant                    = pcTrQuant;
  m_iSearchRange                 = iSearchRange;
  m_bipredSearchRange            = bipredSearchRange;
  m_motionEstimationSearchMethod = motionEstimationSearchMethod;
  m_CABACEstimator               = CABACEstimator;
  m_CtxCache                     = ctxCache;

  for( UInt iDir = 0; iDir < MAX_NUM_REF_LIST_ADAPT_SR; iDir++ )
  {
    for( UInt iRefIdx = 0; iRefIdx < MAX_IDX_ADAPT_SR; iRefIdx++ )
    {
      m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange;
    }
  }

  // initialize motion cost
  for( Int iNum = 0; iNum < AMVP_MAX_NUM_CANDS + 1; iNum++ )
  {
    for( Int iIdx = 0; iIdx < AMVP_MAX_NUM_CANDS; iIdx++ )
    {
      if( iIdx < iNum )
      {
        m_auiMVPIdxCost[iIdx][iNum] = xGetMvpIdxBits( iIdx, iNum );
      }
      else
      {
        m_auiMVPIdxCost[iIdx][iNum] = MAX_INT;
      }
    }
  }

  const ChromaFormat cform = pcEncCfg->getChromaFormatIdc();
  InterPrediction::init( pcRdCost, cform );

  for( UInt i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    m_tmpPredStorage[i].create( UnitArea( cform, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );
  }
  m_tmpStorageLCU.create( UnitArea( cform, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );
  m_tmpAffiStorage.create( UnitArea( cform, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );
  m_tmpAffiError   = new Int   [MAX_CU_SIZE * MAX_CU_SIZE];
  m_tmpAffiDeri[0] = new Double[MAX_CU_SIZE * MAX_CU_SIZE];
  m_tmpAffiDeri[1] = new Double[MAX_CU_SIZE * MAX_CU_SIZE];
  m_pTempPel = new Pel[maxCUWidth*maxCUHeight];

  m_isInitialized = true;
}


inline Void InterSearch::xTZSearchHelp( IntTZSearchStruct& rcStruct, const Int iSearchX, const Int iSearchY, const UChar ucPointNr, const UInt uiDistance )
{
  Distortion  uiSad = 0;

//  CHECK(!( !( rcStruct.searchRange.left > iSearchX || rcStruct.searchRange.right < iSearchX || rcStruct.searchRange.top > iSearchY || rcStruct.searchRange.bottom < iSearchY )), "Unspecified error");

  const Pel* const  piRefSrch = rcStruct.piRefY + iSearchY * rcStruct.iRefStride + iSearchX;

  m_cDistParam.cur.buf = piRefSrch;

  if( 1 == rcStruct.subShiftMode )
  {
    // motion cost
    Distortion uiBitCost = m_pcRdCost->getCostOfVectorWithPredictor( iSearchX, iSearchY );

    // Skip search if bit cost is already larger than best SAD
    if (uiBitCost < rcStruct.uiBestSad)
    {
      Distortion uiTempSad = m_cDistParam.distFunc( m_cDistParam );

      if((uiTempSad + uiBitCost) < rcStruct.uiBestSad)
      {
        // it's not supposed that any member of DistParams is manipulated beside cur.buf
        int subShift = m_cDistParam.subShift;
        const Pel* pOrgCpy = m_cDistParam.org.buf;
        uiSad += uiTempSad >> m_cDistParam.subShift;

        while( m_cDistParam.subShift > 0 )
        {
          Int isubShift           = m_cDistParam.subShift -1;
          m_cDistParam.org.buf = rcStruct.pcPatternKey->buf + (rcStruct.pcPatternKey->stride << isubShift);
          m_cDistParam.cur.buf = piRefSrch + (rcStruct.iRefStride << isubShift);
          uiTempSad            = m_cDistParam.distFunc( m_cDistParam );
          uiSad               += uiTempSad >> m_cDistParam.subShift;

          if(((uiSad << isubShift) + uiBitCost) > rcStruct.uiBestSad)
          {
            break;
          }

          m_cDistParam.subShift--;
        }

        if(m_cDistParam.subShift == 0)
        {
          uiSad += uiBitCost;

          if( uiSad < rcStruct.uiBestSad )
          {
            rcStruct.uiBestSad      = uiSad;
            rcStruct.iBestX         = iSearchX;
            rcStruct.iBestY         = iSearchY;
            rcStruct.uiBestDistance = uiDistance;
            rcStruct.uiBestRound    = 0;
            rcStruct.ucPointNr      = ucPointNr;
            m_cDistParam.maximumDistortionForEarlyExit = uiSad;
          }
        }

        // restore org ptr
        m_cDistParam.org.buf  = pOrgCpy;
        m_cDistParam.subShift = subShift;
      }
    }
  }
  else
  {
    uiSad = m_cDistParam.distFunc( m_cDistParam );

    // only add motion cost if uiSad is smaller than best. Otherwise pointless
    // to add motion cost.
    if( uiSad < rcStruct.uiBestSad )
    {
      // motion cost
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor( iSearchX, iSearchY );

      if( uiSad < rcStruct.uiBestSad )
      {
        rcStruct.uiBestSad      = uiSad;
        rcStruct.iBestX         = iSearchX;
        rcStruct.iBestY         = iSearchY;
        rcStruct.uiBestDistance = uiDistance;
        rcStruct.uiBestRound    = 0;
        rcStruct.ucPointNr      = ucPointNr;
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }
  }
}


#if HM_ME_SR_VIOLATION
inline Void InterSearch::xTZ2PointSearch( IntTZSearchStruct& rcStruct )
{
  const SearchRange& sr = rcStruct.searchRange;

  // 2 point search,                   //   1 2 3
  // check only the 2 untested points  //   4 0 5
  // around the start point            //   6 7 8
  Int iStartX = rcStruct.iBestX;
  Int iStartY = rcStruct.iBestY;
  switch( rcStruct.ucPointNr )
  {
  case 1:
  {
    if ( (iStartX - 1) >= sr.left )
    {
      xTZSearchHelp( rcStruct, iStartX - 1, iStartY, 0, 2 );
    }
    if ( (iStartY - 1) >= sr.top )
    {
      xTZSearchHelp( rcStruct, iStartX, iStartY - 1, 0, 2 );
    }
  }
  break;
  case 2:
  {
    if ( (iStartY - 1) >= sr.top )
    {
      if ( (iStartX - 1) >= sr.left )
      {
        xTZSearchHelp( rcStruct, iStartX - 1, iStartY - 1, 0, 2 );
      }
      if ( (iStartX + 1) <= sr.right )
      {
        xTZSearchHelp( rcStruct, iStartX + 1, iStartY - 1, 0, 2 );
      }
    }
  }
  break;
  case 3:
  {
    if ( (iStartY - 1) >= sr.top )
    {
      xTZSearchHelp( rcStruct, iStartX, iStartY - 1, 0, 2 );
    }
    if ( (iStartX + 1) <= sr.right )
    {
      xTZSearchHelp( rcStruct, iStartX + 1, iStartY, 0, 2 );
    }
  }
  break;
  case 4:
  {
    if ( (iStartX - 1) >= sr.left )
    {
      if ( (iStartY + 1) <= sr.bottom )
      {
        xTZSearchHelp( rcStruct, iStartX - 1, iStartY + 1, 0, 2 );
      }
      if ( (iStartY - 1) >= sr.top )
      {
        xTZSearchHelp( rcStruct, iStartX - 1, iStartY - 1, 0, 2 );
      }
    }
  }
  break;
  case 5:
  {
    if ( (iStartX + 1) <= sr.right )
    {
      if ( (iStartY - 1) >= sr.top )
      {
        xTZSearchHelp( rcStruct, iStartX + 1, iStartY - 1, 0, 2 );
      }
      if ( (iStartY + 1) <= sr.bottom )
      {
        xTZSearchHelp( rcStruct, iStartX + 1, iStartY + 1, 0, 2 );
      }
    }
  }
  break;
  case 6:
  {
    if ( (iStartX - 1) >= sr.left )
    {
      xTZSearchHelp( rcStruct, iStartX - 1, iStartY , 0, 2 );
    }
    if ( (iStartY + 1) <= sr.bottom )
    {
      xTZSearchHelp( rcStruct, iStartX, iStartY + 1, 0, 2 );
    }
  }
  break;
  case 7:
  {
    if ( (iStartY + 1) <= sr.bottom )
    {
      if ( (iStartX - 1) >= sr.left )
      {
        xTZSearchHelp( rcStruct, iStartX - 1, iStartY + 1, 0, 2 );
      }
      if ( (iStartX + 1) <= sr.right )
      {
        xTZSearchHelp( rcStruct, iStartX + 1, iStartY + 1, 0, 2 );
      }
    }
  }
  break;
  case 8:
  {
    if ( (iStartX + 1) <= sr.right )
    {
      xTZSearchHelp( rcStruct, iStartX + 1, iStartY, 0, 2 );
    }
    if ( (iStartY + 1) <= sr.bottom )
    {
      xTZSearchHelp( rcStruct, iStartX, iStartY + 1, 0, 2 );
    }
  }
  break;
  default:
  {
    THROW("Invalid point");
  }
  break;
  } // switch( rcStruct.ucPointNr )

}
#else

inline Void InterSearch::xTZ2PointSearch( IntTZSearchStruct& rcStruct )
{
  const SearchRange& sr = rcStruct.searchRange;

  static const Int xOffset[2][9] = { {  0, -1, -1,  0, -1, +1, -1, -1, +1 }, {  0,  0, +1, +1, -1, +1,  0, +1,  0 } };
  static const Int yOffset[2][9] = { {  0,  0, -1, -1, +1, -1,  0, +1,  0 }, {  0, -1, -1,  0, -1, +1, +1, +1, +1 } };

  // 2 point search,                   //   1 2 3
  // check only the 2 untested points  //   4 0 5
  // around the start point            //   6 7 8
  const Int iX1 = rcStruct.iBestX + xOffset[0][rcStruct.ucPointNr];
  const Int iX2 = rcStruct.iBestX + xOffset[1][rcStruct.ucPointNr];

  const Int iY1 = rcStruct.iBestY + yOffset[0][rcStruct.ucPointNr];
  const Int iY2 = rcStruct.iBestY + yOffset[1][rcStruct.ucPointNr];

  if( iX1 >= sr.left && iX1 <= sr.right && iY1 >= sr.top && iY1 <= sr.bottom )
  {
    xTZSearchHelp( rcStruct, iX1, iY1, 0, 2 );
  }

  if( iX2 >= sr.left && iX2 <= sr.right && iY2 >= sr.top && iY2 <= sr.bottom )
  {
    xTZSearchHelp( rcStruct, iX2, iY2, 0, 2 );
  }
}

#endif

inline Void InterSearch::xTZ8PointSquareSearch( IntTZSearchStruct& rcStruct, const Int iStartX, const Int iStartY, const Int iDist )
{
  const SearchRange& sr = rcStruct.searchRange;
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  CHECK( iDist == 0 , "Invalid distance");
  const Int iTop        = iStartY - iDist;
  const Int iBottom     = iStartY + iDist;
  const Int iLeft       = iStartX - iDist;
  const Int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iTop >= sr.top ) // check top
  {
    if ( iLeft >= sr.left ) // check top left
    {
      xTZSearchHelp( rcStruct, iLeft, iTop, 1, iDist );
    }
    // top middle
    xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );

    if ( iRight <= sr.right ) // check top right
    {
      xTZSearchHelp( rcStruct, iRight, iTop, 3, iDist );
    }
  } // check top
  if ( iLeft >= sr.left ) // check middle left
  {
    xTZSearchHelp( rcStruct, iLeft, iStartY, 4, iDist );
  }
  if ( iRight <= sr.right ) // check middle right
  {
    xTZSearchHelp( rcStruct, iRight, iStartY, 5, iDist );
  }
  if ( iBottom <= sr.bottom ) // check bottom
  {
    if ( iLeft >= sr.left ) // check bottom left
    {
      xTZSearchHelp( rcStruct, iLeft, iBottom, 6, iDist );
    }
    // check bottom middle
    xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );

    if ( iRight <= sr.right ) // check bottom right
    {
      xTZSearchHelp( rcStruct, iRight, iBottom, 8, iDist );
    }
  } // check bottom
}




inline Void InterSearch::xTZ8PointDiamondSearch( IntTZSearchStruct& rcStruct,
                                                 const Int iStartX,
                                                 const Int iStartY,
                                                 const Int iDist,
                                                 const Bool bCheckCornersAtDist1 )
{
  const SearchRange& sr = rcStruct.searchRange;
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  CHECK( iDist == 0, "Invalid distance" );
  const Int iTop        = iStartY - iDist;
  const Int iBottom     = iStartY + iDist;
  const Int iLeft       = iStartX - iDist;
  const Int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iDist == 1 )
  {
    if ( iTop >= sr.top ) // check top
    {
      if (bCheckCornersAtDist1)
      {
        if ( iLeft >= sr.left) // check top-left
        {
          xTZSearchHelp( rcStruct, iLeft, iTop, 1, iDist );
        }
        xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );
        if ( iRight <= sr.right ) // check middle right
        {
          xTZSearchHelp( rcStruct, iRight, iTop, 3, iDist );
        }
      }
      else
      {
        xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );
      }
    }
    if ( iLeft >= sr.left ) // check middle left
    {
      xTZSearchHelp( rcStruct, iLeft, iStartY, 4, iDist );
    }
    if ( iRight <= sr.right ) // check middle right
    {
      xTZSearchHelp( rcStruct, iRight, iStartY, 5, iDist );
    }
    if ( iBottom <= sr.bottom ) // check bottom
    {
      if (bCheckCornersAtDist1)
      {
        if ( iLeft >= sr.left) // check top-left
        {
          xTZSearchHelp( rcStruct, iLeft, iBottom, 6, iDist );
        }
        xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );
        if ( iRight <= sr.right ) // check middle right
        {
          xTZSearchHelp( rcStruct, iRight, iBottom, 8, iDist );
        }
      }
      else
      {
        xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );
      }
    }
  }
  else
  {
    if ( iDist <= 8 )
    {
      const Int iTop_2      = iStartY - (iDist>>1);
      const Int iBottom_2   = iStartY + (iDist>>1);
      const Int iLeft_2     = iStartX - (iDist>>1);
      const Int iRight_2    = iStartX + (iDist>>1);

      if (  iTop >= sr.top && iLeft >= sr.left &&
           iRight <= sr.right && iBottom <= sr.bottom ) // check border
      {
        xTZSearchHelp( rcStruct, iStartX,  iTop,      2, iDist    );
        xTZSearchHelp( rcStruct, iLeft_2,  iTop_2,    1, iDist>>1 );
        xTZSearchHelp( rcStruct, iRight_2, iTop_2,    3, iDist>>1 );
        xTZSearchHelp( rcStruct, iLeft,    iStartY,   4, iDist    );
        xTZSearchHelp( rcStruct, iRight,   iStartY,   5, iDist    );
        xTZSearchHelp( rcStruct, iLeft_2,  iBottom_2, 6, iDist>>1 );
        xTZSearchHelp( rcStruct, iRight_2, iBottom_2, 8, iDist>>1 );
        xTZSearchHelp( rcStruct, iStartX,  iBottom,   7, iDist    );
      }
      else // check border
      {
        if ( iTop >= sr.top ) // check top
        {
          xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );
        }
        if ( iTop_2 >= sr.top ) // check half top
        {
          if ( iLeft_2 >= sr.left ) // check half left
          {
            xTZSearchHelp( rcStruct, iLeft_2, iTop_2, 1, (iDist>>1) );
          }
          if ( iRight_2 <= sr.right ) // check half right
          {
            xTZSearchHelp( rcStruct, iRight_2, iTop_2, 3, (iDist>>1) );
          }
        } // check half top
        if ( iLeft >= sr.left ) // check left
        {
          xTZSearchHelp( rcStruct, iLeft, iStartY, 4, iDist );
        }
        if ( iRight <= sr.right ) // check right
        {
          xTZSearchHelp( rcStruct, iRight, iStartY, 5, iDist );
        }
        if ( iBottom_2 <= sr.bottom ) // check half bottom
        {
          if ( iLeft_2 >= sr.left ) // check half left
          {
            xTZSearchHelp( rcStruct, iLeft_2, iBottom_2, 6, (iDist>>1) );
          }
          if ( iRight_2 <= sr.right ) // check half right
          {
            xTZSearchHelp( rcStruct, iRight_2, iBottom_2, 8, (iDist>>1) );
          }
        } // check half bottom
        if ( iBottom <= sr.bottom ) // check bottom
        {
          xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );
        }
      } // check border
    }
    else // iDist > 8
    {
      if ( iTop >= sr.top && iLeft >= sr.left &&
           iRight <= sr.right && iBottom <= sr.bottom ) // check border
      {
        xTZSearchHelp( rcStruct, iStartX, iTop,    0, iDist );
        xTZSearchHelp( rcStruct, iLeft,   iStartY, 0, iDist );
        xTZSearchHelp( rcStruct, iRight,  iStartY, 0, iDist );
        xTZSearchHelp( rcStruct, iStartX, iBottom, 0, iDist );
        for ( Int index = 1; index < 4; index++ )
        {
          const Int iPosYT = iTop    + ((iDist>>2) * index);
          const Int iPosYB = iBottom - ((iDist>>2) * index);
          const Int iPosXL = iStartX - ((iDist>>2) * index);
          const Int iPosXR = iStartX + ((iDist>>2) * index);
          xTZSearchHelp( rcStruct, iPosXL, iPosYT, 0, iDist );
          xTZSearchHelp( rcStruct, iPosXR, iPosYT, 0, iDist );
          xTZSearchHelp( rcStruct, iPosXL, iPosYB, 0, iDist );
          xTZSearchHelp( rcStruct, iPosXR, iPosYB, 0, iDist );
        }
      }
      else // check border
      {
        if ( iTop >= sr.top ) // check top
        {
          xTZSearchHelp( rcStruct, iStartX, iTop, 0, iDist );
        }
        if ( iLeft >= sr.left ) // check left
        {
          xTZSearchHelp( rcStruct, iLeft, iStartY, 0, iDist );
        }
        if ( iRight <= sr.right ) // check right
        {
          xTZSearchHelp( rcStruct, iRight, iStartY, 0, iDist );
        }
        if ( iBottom <= sr.bottom ) // check bottom
        {
          xTZSearchHelp( rcStruct, iStartX, iBottom, 0, iDist );
        }
        for ( Int index = 1; index < 4; index++ )
        {
          const Int iPosYT = iTop    + ((iDist>>2) * index);
          const Int iPosYB = iBottom - ((iDist>>2) * index);
          const Int iPosXL = iStartX - ((iDist>>2) * index);
          const Int iPosXR = iStartX + ((iDist>>2) * index);

          if ( iPosYT >= sr.top ) // check top
          {
            if ( iPosXL >= sr.left ) // check left
            {
              xTZSearchHelp( rcStruct, iPosXL, iPosYT, 0, iDist );
            }
            if ( iPosXR <= sr.right ) // check right
            {
              xTZSearchHelp( rcStruct, iPosXR, iPosYT, 0, iDist );
            }
          } // check top
          if ( iPosYB <= sr.bottom ) // check bottom
          {
            if ( iPosXL >= sr.left ) // check left
            {
              xTZSearchHelp( rcStruct, iPosXL, iPosYB, 0, iDist );
            }
            if ( iPosXR <= sr.right ) // check right
            {
              xTZSearchHelp( rcStruct, iPosXR, iPosYB, 0, iDist );
            }
          } // check bottom
        } // for ...
      } // check border
    } // iDist <= 8
  } // iDist == 1
}

Distortion InterSearch::xPatternRefinement( const CPelBuf* pcPatternKey,
                                            Mv baseRefMv,
                                            Int iFrac, Mv& rcMvFrac,
                                            Bool bAllowUseOfHadamard )
{
  Distortion  uiDist;
  Distortion  uiDistBest  = std::numeric_limits<Distortion>::max();
  UInt        uiDirecBest = 0;

  Pel*  piRefPos;
  Int iRefStride = pcPatternKey->width + 1;
  m_pcRdCost->setDistParam( m_cDistParam, *pcPatternKey, m_filteredBlock[0][0][0], iRefStride, m_lumaClpRng.bd, COMPONENT_Y, 0, 1, m_pcEncCfg->getUseHADME() && bAllowUseOfHadamard );

  const Mv* pcMvRefine = (iFrac == 2 ? s_acMvRefineH : s_acMvRefineQ);
  for (UInt i = 0; i < 9; i++)
  {
    Mv cMvTest = pcMvRefine[i];
    cMvTest += baseRefMv;

    Int horVal = cMvTest.getHor() * iFrac;
    Int verVal = cMvTest.getVer() * iFrac;
    piRefPos = m_filteredBlock[verVal & 3][horVal & 3][0];

    if (horVal == 2 && (verVal & 1) == 0)
    {
      piRefPos += 1;
    }
    if ((horVal & 1) == 0 && verVal == 2)
    {
      piRefPos += iRefStride;
    }
    cMvTest = pcMvRefine[i];
    cMvTest += rcMvFrac;


    m_cDistParam.cur.buf   = piRefPos;
    uiDist = m_cDistParam.distFunc( m_cDistParam );
    uiDist += m_pcRdCost->getCostOfVectorWithPredictor( cMvTest.getHor(), cMvTest.getVer() );

    if ( uiDist < uiDistBest )
    {
      uiDistBest  = uiDist;
      uiDirecBest = i;
      m_cDistParam.maximumDistortionForEarlyExit = uiDist;
    }
  }

  rcMvFrac = pcMvRefine[uiDirecBest];

  return uiDistBest;
}

Distortion InterSearch::xGetInterPredictionError( PredictionUnit& pu, PelUnitBuf& origBuf, const RefPicList &eRefPicList )
{
  PelUnitBuf predBuf = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );

  motionCompensation( pu, predBuf, eRefPicList );

  DistParam cDistParam;
  cDistParam.applyWeight = false;

  m_pcRdCost->setDistParam( cDistParam, origBuf.Y(), predBuf.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, m_pcEncCfg->getUseHADME() && !pu.cu->transQuantBypass );

  return (Distortion)cDistParam.distFunc( cDistParam );
}

//! estimation of best merge coding
Void InterSearch::xMergeEstimation( PredictionUnit& pu, PelUnitBuf& origBuf, Int iPUIdx, UInt& uiMergeIdx, Distortion& ruiCost, MergeCtx &mergeCtx )
{
  PartSize partSize = pu.cu->partSize;

  if ( pu.cs->pps->getLog2ParallelMergeLevelMinus2() && partSize != SIZE_2Nx2N && pu.cu->lumaSize().width <= 8 )
  {
    if ( iPUIdx == 0 )
    {
      UnitArea unitArea = pu;

      pu.UnitArea::operator=( *pu.cu );
      pu.cu->partSize = SIZE_2Nx2N;

      PU::getInterMergeCandidates( pu, mergeCtx );

      pu.UnitArea::operator=( unitArea );
      pu.cu->partSize = partSize;
    }
  }
  else
  {
    PU::getInterMergeCandidates( pu, mergeCtx );
  }

  PU::restrictBiPredMergeCands( pu, mergeCtx );

  ruiCost = std::numeric_limits<Distortion>::max();
  for( UInt uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; ++uiMergeCand )
  {
    mergeCtx.setMergeInfo( pu, uiMergeCand );

    PU::spanMotionInfo( pu, mergeCtx );

    Distortion uiCostCand = xGetInterPredictionError( pu, origBuf );
    UInt       uiBitsCand = uiMergeCand + 1;

    if( uiMergeCand == m_pcEncCfg->getMaxNumMergeCand() - 1 )
    {
      uiBitsCand--;
    }
    uiCostCand = uiCostCand + m_pcRdCost->getCost( uiBitsCand );
    if ( uiCostCand < ruiCost )
    {
      ruiCost    = uiCostCand;
      uiMergeIdx = uiMergeCand;
    }
  }
}


//! search of the best candidate for inter prediction
#if AMP_MRG
Void InterSearch::predInterSearch(CodingUnit& cu, Partitioner& partitioner, Bool bUseMRG)
#else
Void InterSearch::predInterSearch(CodingUnit& cu, Partitioner& partitioner)
#endif
{
  CodingStructure& cs = *cu.cs;

  AMVPInfo     amvp[2];
  Mv           cMvSrchRngLT;
  Mv           cMvSrchRngRB;

  Mv           cMvZero;

  Mv           cMv[2];
  Mv           cMvBi[2];
  Mv           cMvTemp[2][33];
  Int          iNumPredDir = cs.slice->isInterP() ? 1 : 2;

  Mv           cMvPred[2][33];

  Mv           cMvPredBi[2][33];
  Int          aaiMvpIdxBi[2][33];

  Int          aaiMvpIdx[2][33];
  Int          aaiMvpNum[2][33];

  AMVPInfo     aacAMVPInfo[2][33];

  Int          iRefIdx[2]={0,0}; //If un-initialized, may cause SEGV in bi-directional prediction iterative stage.
  Int          iRefIdxBi[2];

  UInt         uiMbBits[3] = {1, 1, 0};

  UInt         uiLastMode = 0;
  Int          iRefStart, iRefEnd;

  Int          bestBiPRefIdxL1 = 0;
  Int          bestBiPMvpL1    = 0;
  Distortion   biPDistTemp     = std::numeric_limits<Distortion>::max();

  MergeCtx     mergeCtx;

  // Loop over Prediction Units
  CHECK(!cu.firstPU, "CU does not contain any PUs");
  UInt         puIdx = 0;
#if HEVC_USE_PART_SIZE
  UInt         numPUs = CU::getNumPUs(cu);

  for( auto &pu : CU::traversePUs( cu ) )
#else
  auto &pu = *cu.firstPU;
#endif
  {
    // motion estimation only evaluates luma component
    m_maxCompIDToPred = MAX_NUM_COMPONENT;
//    m_maxCompIDToPred = COMPONENT_Y;

    CHECK(pu.cu != &cu, "PU is contained in another CU");


    PU::spanMotionInfo( pu );
    Distortion   uiCost[2] = { std::numeric_limits<Distortion>::max(), std::numeric_limits<Distortion>::max() };
    Distortion   uiCostBi  =   std::numeric_limits<Distortion>::max();
    Distortion   uiCostTemp;

    UInt         uiBits[3];
    UInt         uiBitsTemp;
    Distortion   bestBiPDist = std::numeric_limits<Distortion>::max();

    Distortion   uiCostTempL0[MAX_NUM_REF];
    for (Int iNumRef=0; iNumRef < MAX_NUM_REF; iNumRef++)
    {
      uiCostTempL0[iNumRef] = std::numeric_limits<Distortion>::max();
    }
    UInt         uiBitsTempL0[MAX_NUM_REF];

    Mv           mvValidList1;
    Int          refIdxValidList1 = 0;
    UInt         bitsValidList1   = MAX_UINT;
    Distortion   costValidList1   = std::numeric_limits<Distortion>::max();

    PelUnitBuf origBuf = pu.cs->getOrgBuf( pu );

    xGetBlkBits( cu.partSize, cs.slice->isInterP(), puIdx, uiLastMode, uiMbBits );

    m_pcRdCost->selectMotionLambda( cu.transQuantBypass );

    Bool bFastSkipBi = false;
    if( auto slsCtrl = dynamic_cast< SaveLoadEncInfoCtrl* >( m_modeCtrl ) )
    {
      bFastSkipBi = ( LOAD_ENC_INFO == slsCtrl->getSaveLoadTag( pu ) && 3 != slsCtrl->getSaveLoadInterDir( pu ) );
    }


#if AMP_MRG
    Bool bTestNormalMC = true;

    if ( bUseMRG && cu.lumaSize().width > 8 && numPUs == 2 )
    {
      bTestNormalMC = false;
    }

    if( bTestNormalMC )
    {
#endif
      //  Uni-directional prediction
      for ( Int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
      {
        RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

        for ( Int iRefIdxTemp = 0; iRefIdxTemp < cs.slice->getNumRefIdx(eRefPicList); iRefIdxTemp++ )
        {
          uiBitsTemp = uiMbBits[iRefList];
          if ( cs.slice->getNumRefIdx(eRefPicList) > 1 )
          {
            uiBitsTemp += iRefIdxTemp+1;
            if ( iRefIdxTemp == cs.slice->getNumRefIdx(eRefPicList)-1 )
            {
              uiBitsTemp--;
            }
          }

          xEstimateMvPredAMVP( pu, origBuf, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], amvp[eRefPicList], false, &biPDistTemp);

          aaiMvpIdx[iRefList][iRefIdxTemp] = pu.mvpIdx[eRefPicList];
          aaiMvpNum[iRefList][iRefIdxTemp] = pu.mvpNum[eRefPicList];

          if(cs.slice->getMvdL1ZeroFlag() && iRefList==1 && biPDistTemp < bestBiPDist)
          {
            bestBiPDist = biPDistTemp;
            bestBiPMvpL1 = aaiMvpIdx[iRefList][iRefIdxTemp];
            bestBiPRefIdxL1 = iRefIdxTemp;
          }

          uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

          if ( m_pcEncCfg->getFastMEForGenBLowDelayEnabled() && iRefList == 1 )    // list 1
          {
            if ( cs.slice->getList1IdxToList0Idx( iRefIdxTemp ) >= 0 )
            {
              cMvTemp[1][iRefIdxTemp] = cMvTemp[0][cs.slice->getList1IdxToList0Idx( iRefIdxTemp )];
              uiCostTemp = uiCostTempL0[cs.slice->getList1IdxToList0Idx( iRefIdxTemp )];
              /*first subtract the bit-rate part of the cost of the other list*/
              uiCostTemp -= m_pcRdCost->getCost( uiBitsTempL0[cs.slice->getList1IdxToList0Idx( iRefIdxTemp )] );
              /*correct the bit-rate part of the current ref*/
              m_pcRdCost->setPredictor  ( cMvPred[iRefList][iRefIdxTemp] );
              uiBitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor( cMvTemp[1][iRefIdxTemp].getHor(), cMvTemp[1][iRefIdxTemp].getVer() );
              /*calculate the correct cost*/
              uiCostTemp += m_pcRdCost->getCost( uiBitsTemp );
            }
            else
            {
              xMotionEstimation( pu, origBuf, eRefPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[eRefPicList] );
            }
          }
          else
          {
            xMotionEstimation( pu, origBuf, eRefPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[eRefPicList] );
          }
          xCopyAMVPInfo( &amvp[eRefPicList], &aacAMVPInfo[iRefList][iRefIdxTemp]); // must always be done ( also when AMVP_MODE = AM_NONE )
          xCheckBestMVP( eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], amvp[eRefPicList], uiBitsTemp, uiCostTemp );

          if ( iRefList == 0 )
          {
            uiCostTempL0[iRefIdxTemp] = uiCostTemp;
            uiBitsTempL0[iRefIdxTemp] = uiBitsTemp;
          }
          if ( uiCostTemp < uiCost[iRefList] )
          {
            uiCost[iRefList] = uiCostTemp;
            uiBits[iRefList] = uiBitsTemp; // storing for bi-prediction

            // set motion
            cMv    [iRefList] = cMvTemp[iRefList][iRefIdxTemp];
            iRefIdx[iRefList] = iRefIdxTemp;
          }

          if ( iRefList == 1 && uiCostTemp < costValidList1 && cs.slice->getList1IdxToList0Idx( iRefIdxTemp ) < 0 )
          {
            costValidList1 = uiCostTemp;
            bitsValidList1 = uiBitsTemp;

            // set motion
            mvValidList1     = cMvTemp[iRefList][iRefIdxTemp];
            refIdxValidList1 = iRefIdxTemp;
          }
        }
      }
	  //cout << iRefIdx[0] << " ";  
	  //cout << iRefIdx[0] << ":" << cs.slice->getRefPOC(REF_PIC_LIST_0, iRefIdx[0])<< "-" <<iRefIdx[1] << ":" << cs.slice->getRefPOC(REF_PIC_LIST_1, iRefIdx[1])<<" ";
	  //cout << pu.lumaPos().x << ":" << pu.lumaPos().y<<" ";
	  
	 // cout << pu.lheight() <<":"<< pu.lwidth()<<"-"<<cu.lheight()<<":"<<cu.lwidth()<<"  ";

#if BLOCK_ROD
	  if (iRefIdx[0] == 3)
	  {

	  }
#endif

#if HEVC_PARTITIONER
      auto hevcPartitioner = dynamic_cast<HEVCPartitioner*>( &partitioner );

      if( !bFastSkipBi && m_pcEncCfg->getUseFastLCTU() && m_pcEncCfg->getLargeCTU() && hevcPartitioner )
      {
        unsigned minDepth = 0;
        unsigned maxDepth = g_aucLog2[pu.cs->sps->getSpsNext().getCTUSize()] - g_aucLog2[pu.cs->sps->getSpsNext().getMinQTSize( B_SLICE, CHANNEL_TYPE_LUMA )];
        hevcPartitioner->setMaxMinDepth( minDepth, maxDepth, cs );
        bFastSkipBi = !(pu.cu->qtDepth < maxDepth || pu.lumaSize().area() >= 64);
      }

#endif
      //  Bi-predictive Motion estimation
      if ( (cs.slice->isInterB()) && ( PU::isBipredRestriction(pu) == false)  && !bFastSkipBi )
      {
		  cout << "inB";
        cMvBi[0] = cMv[0];
        cMvBi[1] = cMv[1];
        iRefIdxBi[0] = iRefIdx[0];
        iRefIdxBi[1] = iRefIdx[1];

        ::memcpy( cMvPredBi,   cMvPred,   sizeof( cMvPred   ) );
        ::memcpy( aaiMvpIdxBi, aaiMvpIdx, sizeof( aaiMvpIdx ) );

        UInt uiMotBits[2];

        if(cs.slice->getMvdL1ZeroFlag())
        {
          xCopyAMVPInfo(&aacAMVPInfo[1][bestBiPRefIdxL1], &amvp[REF_PIC_LIST_1]);
          aaiMvpIdxBi[1][bestBiPRefIdxL1] = bestBiPMvpL1;
          cMvPredBi  [1][bestBiPRefIdxL1] = amvp[REF_PIC_LIST_1].mvCand[bestBiPMvpL1];

          cMvBi    [1] = cMvPredBi[1][bestBiPRefIdxL1];
          iRefIdxBi[1] = bestBiPRefIdxL1;
          pu.mv    [REF_PIC_LIST_1] = cMvBi[1];
          pu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];
          pu.mvpIdx[REF_PIC_LIST_1] = bestBiPMvpL1;

          PelUnitBuf predBufTmp = m_tmpPredStorage[REF_PIC_LIST_1].getBuf( UnitAreaRelative(cu, pu) );
          motionCompensation( pu, predBufTmp, REF_PIC_LIST_1 );

          uiMotBits[0] = uiBits[0] - uiMbBits[0];
          uiMotBits[1] = uiMbBits[1];

          if ( cs.slice->getNumRefIdx(REF_PIC_LIST_1) > 1 )
          {
            uiMotBits[1] += bestBiPRefIdxL1 + 1;
            if ( bestBiPRefIdxL1 == cs.slice->getNumRefIdx(REF_PIC_LIST_1)-1 )
            {
              uiMotBits[1]--;
            }
          }

          uiMotBits[1] += m_auiMVPIdxCost[aaiMvpIdxBi[1][bestBiPRefIdxL1]][AMVP_MAX_NUM_CANDS];

          uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];

          cMvTemp[1][bestBiPRefIdxL1] = cMvBi[1];
        }
        else
        {
          uiMotBits[0] = uiBits[0] - uiMbBits[0];
          uiMotBits[1] = uiBits[1] - uiMbBits[1];
          uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
        }

        // 4-times iteration (default)
        Int iNumIter = 4;

        // fast encoder setting: only one iteration
        if ( m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE2 || cs.slice->getMvdL1ZeroFlag() )
        {
          iNumIter = 1;
        }

        for ( Int iIter = 0; iIter < iNumIter; iIter++ )
        {
          Int         iRefList    = iIter % 2;

          if ( m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE2 )
          {
            if( uiCost[0] <= uiCost[1] )
            {
              iRefList = 1;
            }
            else
            {
              iRefList = 0;
            }
          }
          else if ( iIter == 0 )
          {
            iRefList = 0;
          }
          if ( iIter == 0 && !cs.slice->getMvdL1ZeroFlag())
          {
            pu.mv    [1 - iRefList] = cMv    [1 - iRefList];
            pu.refIdx[1 - iRefList] = iRefIdx[1 - iRefList];

            PelUnitBuf predBufTmp = m_tmpPredStorage[1 - iRefList].getBuf( UnitAreaRelative(cu, pu) );
            motionCompensation( pu, predBufTmp, RefPicList(1 - iRefList) );
          }

          RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

          if(cs.slice->getMvdL1ZeroFlag())
          {
            iRefList = 0;
            eRefPicList = REF_PIC_LIST_0;
          }

          Bool bChanged = false;

          iRefStart = 0;
          iRefEnd   = cs.slice->getNumRefIdx(eRefPicList)-1;

          for ( Int iRefIdxTemp = iRefStart; iRefIdxTemp <= iRefEnd; iRefIdxTemp++ )
          {
            uiBitsTemp = uiMbBits[2] + uiMotBits[1-iRefList];
            if ( cs.slice->getNumRefIdx(eRefPicList) > 1 )
            {
              uiBitsTemp += iRefIdxTemp+1;
              if ( iRefIdxTemp == cs.slice->getNumRefIdx(eRefPicList)-1 )
              {
                uiBitsTemp--;
              }
            }
            uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

            // call ME
            xCopyAMVPInfo(&aacAMVPInfo[iRefList][iRefIdxTemp], &amvp[eRefPicList] );
            xMotionEstimation ( pu, origBuf, eRefPicList, cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[eRefPicList], true );
            xCheckBestMVP( eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], amvp[eRefPicList], uiBitsTemp, uiCostTemp);
            if ( uiCostTemp < uiCostBi )
            {
              bChanged = true;

              cMvBi[iRefList]     = cMvTemp[iRefList][iRefIdxTemp];
              iRefIdxBi[iRefList] = iRefIdxTemp;

              uiCostBi            = uiCostTemp;
              uiMotBits[iRefList] = uiBitsTemp - uiMbBits[2] - uiMotBits[1-iRefList];
              uiBits[2]           = uiBitsTemp;

              if(iNumIter!=1)
              {
                //  Set motion
                pu.mv    [eRefPicList] = cMvBi    [iRefList];
                pu.refIdx[eRefPicList] = iRefIdxBi[iRefList];

                PelUnitBuf predBufTmp = m_tmpPredStorage[iRefList].getBuf( UnitAreaRelative(cu, pu) );
                motionCompensation( pu, predBufTmp, eRefPicList );
              }
            }
          } // for loop-iRefIdxTemp

          if ( !bChanged )
          {
            if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1] )
            {
              xCopyAMVPInfo(&aacAMVPInfo[0][iRefIdxBi[0]], &amvp[REF_PIC_LIST_0]);
              xCheckBestMVP( REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], amvp[eRefPicList], uiBits[2], uiCostBi);
              if(!cs.slice->getMvdL1ZeroFlag())
              {
                xCopyAMVPInfo(&aacAMVPInfo[1][iRefIdxBi[1]], &amvp[REF_PIC_LIST_1]);
                xCheckBestMVP( REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], amvp[eRefPicList], uiBits[2], uiCostBi);
              }
            }
            break;
          }
        } // for loop-iter
      } // if (B_SLICE)

#if AMP_MRG
    } //end if bTestNormalMC
#endif
#if PRINT_PUREF
	{
		//if (cu.slice->getPOC() > 49)
		{
			cout << pu.refIdx[REF_PIC_LIST_0] << ":" << pu.refIdx[REF_PIC_LIST_1] << " ";
			ofstream pufile;
			pufile.open("pufile.txt",ios::app);
			pufile << pu.refIdx[REF_PIC_LIST_0] << ":" << pu.refIdx[REF_PIC_LIST_1] << " ";
			//cu.cs->slice->getPOC();
		}

	ofstream mvref;
	mvref.open("D://2//mvref.txt");
	mvref << pu.refIdx[REF_PIC_LIST_0] << ":" << pu.refIdx[REF_PIC_LIST_1]<<" ";

	cout << " Hor " <<pu.mv->getHor()<<" "<<pu.mv->getVer();
	}
#endif
      //  Clear Motion Field
    pu.mv    [REF_PIC_LIST_0] = Mv();
    pu.mv    [REF_PIC_LIST_1] = Mv();


    pu.mvd   [REF_PIC_LIST_0] = cMvZero;
    pu.mvd   [REF_PIC_LIST_1] = cMvZero;
    pu.refIdx[REF_PIC_LIST_0] = NOT_VALID;
    pu.refIdx[REF_PIC_LIST_1] = NOT_VALID;
    pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
    pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
    pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
    pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;


    UInt uiMEBits = 0;

    // Set Motion Field

    cMv    [1] = mvValidList1;
    iRefIdx[1] = refIdxValidList1;
    uiBits [1] = bitsValidList1;
    uiCost [1] = costValidList1;

#if AMP_MRG
    if (bTestNormalMC)
    {
#endif
      if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1])
      {
        uiLastMode = 2;
        pu.mv    [REF_PIC_LIST_0] = cMvBi[0];
        pu.mv    [REF_PIC_LIST_1] = cMvBi[1];
        pu.mvd   [REF_PIC_LIST_0] = cMvBi[0] - cMvPredBi[0][iRefIdxBi[0]];
        pu.mvd   [REF_PIC_LIST_1] = cMvBi[1] - cMvPredBi[1][iRefIdxBi[1]];
        pu.refIdx[REF_PIC_LIST_0] = iRefIdxBi[0];
        pu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];
        pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdxBi[0][iRefIdxBi[0]];
        pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdxBi[1][iRefIdxBi[1]];
        pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdxBi[0]];
        pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdxBi[1]];
        pu.interDir = 3;

        uiMEBits = uiBits[2];
      }
      else if ( uiCost[0] <= uiCost[1] )
      {
        uiLastMode = 0;
        pu.mv    [REF_PIC_LIST_0] = cMv[0];
        pu.mvd   [REF_PIC_LIST_0] = cMv[0] - cMvPred[0][iRefIdx[0]];
        pu.refIdx[REF_PIC_LIST_0] = iRefIdx[0];
        pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdx[0][iRefIdx[0]];
        pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdx[0]];
        pu.interDir = 1;

        uiMEBits = uiBits[0];
      }
      else
      {
        uiLastMode = 1;
        pu.mv    [REF_PIC_LIST_1] = cMv[1];
        pu.mvd   [REF_PIC_LIST_1] = cMv[1] - cMvPred[1][iRefIdx[1]];
        pu.refIdx[REF_PIC_LIST_1] = iRefIdx[1];
        pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdx[1][iRefIdx[1]];
        pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdx[1]];
        pu.interDir = 2;

        uiMEBits = uiBits[1];
      }
#if AMP_MRG
    } // end if bTestNormalMC
#endif

    if ( cu.partSize != SIZE_2Nx2N )
    {
      UInt uiMRGIndex    = 0;

#if AMP_MRG
      // calculate ME cost
      Distortion uiMEError = std::numeric_limits<Distortion>::max();
      Distortion uiMECost  = std::numeric_limits<Distortion>::max();

      if( bTestNormalMC )
      {
        uiMEError = xGetInterPredictionError( pu, origBuf );
        uiMECost = uiMEError + m_pcRdCost->getCost( uiMEBits );
      }
#else
      // calculate ME cost
      Distortion uiMEError = xGetInterPredictionError( pu, origBuf );
      Distortion uiMECost = uiMEError + m_pcRdCost->getCost( uiMEBits );
#endif
      // save ME result.
      InterPredictionData savedPU = pu;

      // find Merge result
      Distortion uiMRGCost = std::numeric_limits<Distortion>::max();

      pu.initData();
      xMergeEstimation( pu, origBuf, puIdx, uiMRGIndex, uiMRGCost, mergeCtx );

      if( uiMRGCost < uiMECost )
      {
        // set Merge result
        mergeCtx.setMergeInfo( pu, uiMRGIndex );
      }
      else
      {
        pu = savedPU;
      }
    }
    CHECK( !( !cu.cs->pcv->only2Nx2N || cu.partSize == SIZE_2Nx2N ), "Unexpected part size for QTBT." );
    m_maxCompIDToPred = MAX_NUM_COMPONENT;

    {
      PU::spanMotionInfo( pu, mergeCtx );
    }

    //  MC
    PelUnitBuf predBuf = pu.cs->getPredBuf(pu);
    motionCompensation( pu, predBuf, REF_PIC_LIST_X );
    puIdx++;
  }
  
  setWpScalingDistParam( -1, REF_PIC_LIST_X, cu.cs->slice );

  return;
}
#if BLOCK_SELECT
Void InterSearch::predInterSearchSel(CodingUnit& cu, Partitioner& partitioner,vector<int>& BgSelect)

{
	CodingStructure& cs = *cu.cs;

	AMVPInfo     amvp[2];
	Mv           cMvSrchRngLT;
	Mv           cMvSrchRngRB;

	Mv           cMvZero;

	Mv           cMv[2];
	Mv           cMvBi[2];
	Mv           cMvTemp[2][33];
	Int          iNumPredDir = cs.slice->isInterP() ? 1 : 2;

	Mv           cMvPred[2][33];

	Mv           cMvPredBi[2][33];
	Int          aaiMvpIdxBi[2][33];

	Int          aaiMvpIdx[2][33];
	Int          aaiMvpNum[2][33];

	AMVPInfo     aacAMVPInfo[2][33];

	Int          iRefIdx[2] = { 0,0 }; //If un-initialized, may cause SEGV in bi-directional prediction iterative stage.
	Int          iRefIdxBi[2];

	UInt         uiMbBits[3] = { 1, 1, 0 };

	UInt         uiLastMode = 0;
	Int          iRefStart, iRefEnd;

	Int          bestBiPRefIdxL1 = 0;
	Int          bestBiPMvpL1 = 0;
	Distortion   biPDistTemp = std::numeric_limits<Distortion>::max();

	MergeCtx     mergeCtx;

	// Loop over Prediction Units
	CHECK(!cu.firstPU, "CU does not contain any PUs");
	UInt         puIdx = 0;
#if HEVC_USE_PART_SIZE
	UInt         numPUs = CU::getNumPUs(cu);

	for (auto &pu : CU::traversePUs(cu))
#else
	auto &pu = *cu.firstPU;
#endif
	{
		// motion estimation only evaluates luma component
		m_maxCompIDToPred = MAX_NUM_COMPONENT;
		//    m_maxCompIDToPred = COMPONENT_Y;

		CHECK(pu.cu != &cu, "PU is contained in another CU");


		PU::spanMotionInfo(pu);
		Distortion   uiCost[2] = { std::numeric_limits<Distortion>::max(), std::numeric_limits<Distortion>::max() };
		Distortion   uiCostBi = std::numeric_limits<Distortion>::max();
		Distortion   uiCostTemp;

		UInt         uiBits[3];
		UInt         uiBitsTemp;
		Distortion   bestBiPDist = std::numeric_limits<Distortion>::max();

		Distortion   uiCostTempL0[MAX_NUM_REF];
		for (Int iNumRef = 0; iNumRef < MAX_NUM_REF; iNumRef++)
		{
			uiCostTempL0[iNumRef] = std::numeric_limits<Distortion>::max();
		}
		UInt         uiBitsTempL0[MAX_NUM_REF];

		Mv           mvValidList1;
		Int          refIdxValidList1 = 0;
		UInt         bitsValidList1 = MAX_UINT;
		Distortion   costValidList1 = std::numeric_limits<Distortion>::max();

		PelUnitBuf origBuf = pu.cs->getOrgBuf(pu);

		xGetBlkBits(cu.partSize, cs.slice->isInterP(), puIdx, uiLastMode, uiMbBits);

		m_pcRdCost->selectMotionLambda(cu.transQuantBypass);

		Bool bFastSkipBi = false;
		if (auto slsCtrl = dynamic_cast< SaveLoadEncInfoCtrl* >(m_modeCtrl))
		{
			bFastSkipBi = (LOAD_ENC_INFO == slsCtrl->getSaveLoadTag(pu) && 3 != slsCtrl->getSaveLoadInterDir(pu));
		}


#if AMP_MRG
		Bool bTestNormalMC = true;

		if (bUseMRG && cu.lumaSize().width > 8 && numPUs == 2)
		{
			bTestNormalMC = false;
		}

		if (bTestNormalMC)
		{
#endif
			//  Uni-directional prediction
			for (Int iRefList = 0; iRefList < iNumPredDir; iRefList++)
			{
				RefPicList  eRefPicList = (iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

				for (Int iRefIdxTemp = 0; iRefIdxTemp < cs.slice->getNumRefIdx(eRefPicList); iRefIdxTemp++)
				{
					uiBitsTemp = uiMbBits[iRefList];
					if (cs.slice->getNumRefIdx(eRefPicList) > 1)
					{
						uiBitsTemp += iRefIdxTemp + 1;
						if (iRefIdxTemp == cs.slice->getNumRefIdx(eRefPicList) - 1)
						{
							uiBitsTemp--;
						}
					}

					xEstimateMvPredAMVP(pu, origBuf, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], amvp[eRefPicList], false, &biPDistTemp);

					aaiMvpIdx[iRefList][iRefIdxTemp] = pu.mvpIdx[eRefPicList];
					aaiMvpNum[iRefList][iRefIdxTemp] = pu.mvpNum[eRefPicList];

					if (cs.slice->getMvdL1ZeroFlag() && iRefList == 1 && biPDistTemp < bestBiPDist)
					{
						bestBiPDist = biPDistTemp;
						bestBiPMvpL1 = aaiMvpIdx[iRefList][iRefIdxTemp];
						bestBiPRefIdxL1 = iRefIdxTemp;
					}

					uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

					if (m_pcEncCfg->getFastMEForGenBLowDelayEnabled() && iRefList == 1)    // list 1
					{
						if (cs.slice->getList1IdxToList0Idx(iRefIdxTemp) >= 0)
						{
							cMvTemp[1][iRefIdxTemp] = cMvTemp[0][cs.slice->getList1IdxToList0Idx(iRefIdxTemp)];
							uiCostTemp = uiCostTempL0[cs.slice->getList1IdxToList0Idx(iRefIdxTemp)];
							/*first subtract the bit-rate part of the cost of the other list*/
							uiCostTemp -= m_pcRdCost->getCost(uiBitsTempL0[cs.slice->getList1IdxToList0Idx(iRefIdxTemp)]);
							/*correct the bit-rate part of the current ref*/
							m_pcRdCost->setPredictor(cMvPred[iRefList][iRefIdxTemp]);
							uiBitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor(cMvTemp[1][iRefIdxTemp].getHor(), cMvTemp[1][iRefIdxTemp].getVer());
							/*calculate the correct cost*/
							uiCostTemp += m_pcRdCost->getCost(uiBitsTemp);
						}
						else
						{
							xMotionEstimation(pu, origBuf, eRefPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[eRefPicList]);
						}
					}
					else
					{
						xMotionEstimation(pu, origBuf, eRefPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[eRefPicList]);
					}
					xCopyAMVPInfo(&amvp[eRefPicList], &aacAMVPInfo[iRefList][iRefIdxTemp]); // must always be done ( also when AMVP_MODE = AM_NONE )
					xCheckBestMVP(eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], amvp[eRefPicList], uiBitsTemp, uiCostTemp);

					if (iRefList == 0)
					{
						uiCostTempL0[iRefIdxTemp] = uiCostTemp;
						uiBitsTempL0[iRefIdxTemp] = uiBitsTemp;
					}
					if (uiCostTemp < uiCost[iRefList])
					{
						uiCost[iRefList] = uiCostTemp;
						uiBits[iRefList] = uiBitsTemp; // storing for bi-prediction

													   // set motion
						cMv[iRefList] = cMvTemp[iRefList][iRefIdxTemp];
						iRefIdx[iRefList] = iRefIdxTemp;
					}

					if (iRefList == 1 && uiCostTemp < costValidList1 && cs.slice->getList1IdxToList0Idx(iRefIdxTemp) < 0)
					{
						costValidList1 = uiCostTemp;
						bitsValidList1 = uiBitsTemp;

						// set motion
						mvValidList1 = cMvTemp[iRefList][iRefIdxTemp];
						refIdxValidList1 = iRefIdxTemp;
					}
				}
			}
			//cout << iRefIdx[0] << " ";  
			//cout << iRefIdx[0] << ":" << cs.slice->getRefPOC(REF_PIC_LIST_0, iRefIdx[0])<< "-" <<iRefIdx[1] << ":" << cs.slice->getRefPOC(REF_PIC_LIST_1, iRefIdx[1])<<" ";
			cout << pu.lumaPos().x << ":" << pu.lumaPos().y<<":"<< pu.lheight() << ":" << pu.lwidth()<<":"<<iRefIdx[0]<<" ";

			// cout << pu.lheight() <<":"<< pu.lwidth()<<"-"<<cu.lheight()<<":"<<cu.lwidth()<<"  ";

#if BLOCK_ROD
			if (iRefIdx[0] == 3)
			{

			}
#endif

#if HEVC_PARTITIONER
			auto hevcPartitioner = dynamic_cast<HEVCPartitioner*>(&partitioner);

			if (!bFastSkipBi && m_pcEncCfg->getUseFastLCTU() && m_pcEncCfg->getLargeCTU() && hevcPartitioner)
			{
				unsigned minDepth = 0;
				unsigned maxDepth = g_aucLog2[pu.cs->sps->getSpsNext().getCTUSize()] - g_aucLog2[pu.cs->sps->getSpsNext().getMinQTSize(B_SLICE, CHANNEL_TYPE_LUMA)];
				hevcPartitioner->setMaxMinDepth(minDepth, maxDepth, cs);
				bFastSkipBi = !(pu.cu->qtDepth < maxDepth || pu.lumaSize().area() >= 64);
			}

#endif
			//  Bi-predictive Motion estimation
			if ((cs.slice->isInterB()) && (PU::isBipredRestriction(pu) == false) && !bFastSkipBi)
			{
				cout << "inB";
				cMvBi[0] = cMv[0];
				cMvBi[1] = cMv[1];
				iRefIdxBi[0] = iRefIdx[0];
				iRefIdxBi[1] = iRefIdx[1];

				::memcpy(cMvPredBi, cMvPred, sizeof(cMvPred));
				::memcpy(aaiMvpIdxBi, aaiMvpIdx, sizeof(aaiMvpIdx));

				UInt uiMotBits[2];

				if (cs.slice->getMvdL1ZeroFlag())
				{
					xCopyAMVPInfo(&aacAMVPInfo[1][bestBiPRefIdxL1], &amvp[REF_PIC_LIST_1]);
					aaiMvpIdxBi[1][bestBiPRefIdxL1] = bestBiPMvpL1;
					cMvPredBi[1][bestBiPRefIdxL1] = amvp[REF_PIC_LIST_1].mvCand[bestBiPMvpL1];

					cMvBi[1] = cMvPredBi[1][bestBiPRefIdxL1];
					iRefIdxBi[1] = bestBiPRefIdxL1;
					pu.mv[REF_PIC_LIST_1] = cMvBi[1];
					pu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];
					pu.mvpIdx[REF_PIC_LIST_1] = bestBiPMvpL1;

					PelUnitBuf predBufTmp = m_tmpPredStorage[REF_PIC_LIST_1].getBuf(UnitAreaRelative(cu, pu));
					motionCompensation(pu, predBufTmp, REF_PIC_LIST_1);

					uiMotBits[0] = uiBits[0] - uiMbBits[0];
					uiMotBits[1] = uiMbBits[1];

					if (cs.slice->getNumRefIdx(REF_PIC_LIST_1) > 1)
					{
						uiMotBits[1] += bestBiPRefIdxL1 + 1;
						if (bestBiPRefIdxL1 == cs.slice->getNumRefIdx(REF_PIC_LIST_1) - 1)
						{
							uiMotBits[1]--;
						}
					}

					uiMotBits[1] += m_auiMVPIdxCost[aaiMvpIdxBi[1][bestBiPRefIdxL1]][AMVP_MAX_NUM_CANDS];

					uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];

					cMvTemp[1][bestBiPRefIdxL1] = cMvBi[1];
				}
				else
				{
					uiMotBits[0] = uiBits[0] - uiMbBits[0];
					uiMotBits[1] = uiBits[1] - uiMbBits[1];
					uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
				}

				// 4-times iteration (default)
				Int iNumIter = 4;

				// fast encoder setting: only one iteration
				if (m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE2 || cs.slice->getMvdL1ZeroFlag())
				{
					iNumIter = 1;
				}

				for (Int iIter = 0; iIter < iNumIter; iIter++)
				{
					Int         iRefList = iIter % 2;

					if (m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE2)
					{
						if (uiCost[0] <= uiCost[1])
						{
							iRefList = 1;
						}
						else
						{
							iRefList = 0;
						}
					}
					else if (iIter == 0)
					{
						iRefList = 0;
					}
					if (iIter == 0 && !cs.slice->getMvdL1ZeroFlag())
					{
						pu.mv[1 - iRefList] = cMv[1 - iRefList];
						pu.refIdx[1 - iRefList] = iRefIdx[1 - iRefList];

						PelUnitBuf predBufTmp = m_tmpPredStorage[1 - iRefList].getBuf(UnitAreaRelative(cu, pu));
						motionCompensation(pu, predBufTmp, RefPicList(1 - iRefList));
					}

					RefPicList  eRefPicList = (iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

					if (cs.slice->getMvdL1ZeroFlag())
					{
						iRefList = 0;
						eRefPicList = REF_PIC_LIST_0;
					}

					Bool bChanged = false;

					iRefStart = 0;
					iRefEnd = cs.slice->getNumRefIdx(eRefPicList) - 1;

					for (Int iRefIdxTemp = iRefStart; iRefIdxTemp <= iRefEnd; iRefIdxTemp++)
					{
						uiBitsTemp = uiMbBits[2] + uiMotBits[1 - iRefList];
						if (cs.slice->getNumRefIdx(eRefPicList) > 1)
						{
							uiBitsTemp += iRefIdxTemp + 1;
							if (iRefIdxTemp == cs.slice->getNumRefIdx(eRefPicList) - 1)
							{
								uiBitsTemp--;
							}
						}
						uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

						// call ME
						xCopyAMVPInfo(&aacAMVPInfo[iRefList][iRefIdxTemp], &amvp[eRefPicList]);
						xMotionEstimation(pu, origBuf, eRefPicList, cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[eRefPicList], true);
						xCheckBestMVP(eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], amvp[eRefPicList], uiBitsTemp, uiCostTemp);
						if (uiCostTemp < uiCostBi)
						{
							bChanged = true;

							cMvBi[iRefList] = cMvTemp[iRefList][iRefIdxTemp];
							iRefIdxBi[iRefList] = iRefIdxTemp;

							uiCostBi = uiCostTemp;
							uiMotBits[iRefList] = uiBitsTemp - uiMbBits[2] - uiMotBits[1 - iRefList];
							uiBits[2] = uiBitsTemp;

							if (iNumIter != 1)
							{
								//  Set motion
								pu.mv[eRefPicList] = cMvBi[iRefList];
								pu.refIdx[eRefPicList] = iRefIdxBi[iRefList];

								PelUnitBuf predBufTmp = m_tmpPredStorage[iRefList].getBuf(UnitAreaRelative(cu, pu));
								motionCompensation(pu, predBufTmp, eRefPicList);
							}
						}
					} // for loop-iRefIdxTemp

					if (!bChanged)
					{
						if (uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1])
						{
							xCopyAMVPInfo(&aacAMVPInfo[0][iRefIdxBi[0]], &amvp[REF_PIC_LIST_0]);
							xCheckBestMVP(REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], amvp[eRefPicList], uiBits[2], uiCostBi);
							if (!cs.slice->getMvdL1ZeroFlag())
							{
								xCopyAMVPInfo(&aacAMVPInfo[1][iRefIdxBi[1]], &amvp[REF_PIC_LIST_1]);
								xCheckBestMVP(REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], amvp[eRefPicList], uiBits[2], uiCostBi);
							}
						}
						break;
					}
				} // for loop-iter
			} // if (B_SLICE)

#if AMP_MRG
		} //end if bTestNormalMC
#endif
#if PRINT_PUREF
		{
			//if (cu.slice->getPOC() > 49)
			{
				cout << pu.refIdx[REF_PIC_LIST_0] << ":" << pu.refIdx[REF_PIC_LIST_1] << " ";
				ofstream pufile;
				pufile.open("pufile.txt", ios::app);
				pufile << pu.refIdx[REF_PIC_LIST_0] << ":" << pu.refIdx[REF_PIC_LIST_1] << " ";
				//cu.cs->slice->getPOC();
			}

			ofstream mvref;
			mvref.open("D://2//mvref.txt");
			mvref << pu.refIdx[REF_PIC_LIST_0] << ":" << pu.refIdx[REF_PIC_LIST_1] << " ";

			cout << " Hor " << pu.mv->getHor() << " " << pu.mv->getVer();
		}
#endif
		//  Clear Motion Field
		pu.mv[REF_PIC_LIST_0] = Mv();
		pu.mv[REF_PIC_LIST_1] = Mv();


		pu.mvd[REF_PIC_LIST_0] = cMvZero;
		pu.mvd[REF_PIC_LIST_1] = cMvZero;
		pu.refIdx[REF_PIC_LIST_0] = NOT_VALID;
		pu.refIdx[REF_PIC_LIST_1] = NOT_VALID;
		pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
		pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
		pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
		pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;


		UInt uiMEBits = 0;

		// Set Motion Field

		cMv[1] = mvValidList1;
		iRefIdx[1] = refIdxValidList1;
		uiBits[1] = bitsValidList1;
		uiCost[1] = costValidList1;

#if AMP_MRG
		if (bTestNormalMC)
		{
#endif
			if (uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1])
			{
				uiLastMode = 2;
				pu.mv[REF_PIC_LIST_0] = cMvBi[0];
				pu.mv[REF_PIC_LIST_1] = cMvBi[1];
				pu.mvd[REF_PIC_LIST_0] = cMvBi[0] - cMvPredBi[0][iRefIdxBi[0]];
				pu.mvd[REF_PIC_LIST_1] = cMvBi[1] - cMvPredBi[1][iRefIdxBi[1]];
				pu.refIdx[REF_PIC_LIST_0] = iRefIdxBi[0];
				pu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];
				pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdxBi[0][iRefIdxBi[0]];
				pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdxBi[1][iRefIdxBi[1]];
				pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdxBi[0]];
				pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdxBi[1]];
				pu.interDir = 3;

				uiMEBits = uiBits[2];
			}
			else if (uiCost[0] <= uiCost[1])
			{
				uiLastMode = 0;
				pu.mv[REF_PIC_LIST_0] = cMv[0];
				pu.mvd[REF_PIC_LIST_0] = cMv[0] - cMvPred[0][iRefIdx[0]];
				pu.refIdx[REF_PIC_LIST_0] = iRefIdx[0];
				pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdx[0][iRefIdx[0]];
				pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdx[0]];
				pu.interDir = 1;

				uiMEBits = uiBits[0];
			}
			else
			{
				uiLastMode = 1;
				pu.mv[REF_PIC_LIST_1] = cMv[1];
				pu.mvd[REF_PIC_LIST_1] = cMv[1] - cMvPred[1][iRefIdx[1]];
				pu.refIdx[REF_PIC_LIST_1] = iRefIdx[1];
				pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdx[1][iRefIdx[1]];
				pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdx[1]];
				pu.interDir = 2;

				uiMEBits = uiBits[1];
			}
#if AMP_MRG
		} // end if bTestNormalMC
#endif

		if (cu.partSize != SIZE_2Nx2N)
		{
			UInt uiMRGIndex = 0;

#if AMP_MRG
			// calculate ME cost
			Distortion uiMEError = std::numeric_limits<Distortion>::max();
			Distortion uiMECost = std::numeric_limits<Distortion>::max();

			if (bTestNormalMC)
			{
				uiMEError = xGetInterPredictionError(pu, origBuf);
				uiMECost = uiMEError + m_pcRdCost->getCost(uiMEBits);
			}
#else
			// calculate ME cost
			Distortion uiMEError = xGetInterPredictionError(pu, origBuf);
			Distortion uiMECost = uiMEError + m_pcRdCost->getCost(uiMEBits);
#endif
			// save ME result.
			InterPredictionData savedPU = pu;

			// find Merge result
			Distortion uiMRGCost = std::numeric_limits<Distortion>::max();

			pu.initData();
			xMergeEstimation(pu, origBuf, puIdx, uiMRGIndex, uiMRGCost, mergeCtx);

			if (uiMRGCost < uiMECost)
			{
				// set Merge result
				mergeCtx.setMergeInfo(pu, uiMRGIndex);
			}
			else
			{
				pu = savedPU;
			}
		}
		CHECK(!(!cu.cs->pcv->only2Nx2N || cu.partSize == SIZE_2Nx2N), "Unexpected part size for QTBT.");
		m_maxCompIDToPred = MAX_NUM_COMPONENT;

		{
			PU::spanMotionInfo(pu, mergeCtx);
		}

		//  MC
		PelUnitBuf predBuf = pu.cs->getPredBuf(pu);
		motionCompensation(pu, predBuf, REF_PIC_LIST_X);
		puIdx++;
	}

	setWpScalingDistParam(-1, REF_PIC_LIST_X, cu.cs->slice);

	return;
}
#endif




// AMVP
Void InterSearch::xEstimateMvPredAMVP( PredictionUnit& pu, PelUnitBuf& origBuf, RefPicList eRefPicList, Int iRefIdx, Mv& rcMvPred, AMVPInfo& rAMVPInfo, Bool bFilled, Distortion* puiDistBiP )
{
  Mv         cBestMv;
  Int        iBestIdx   = 0;
  Distortion uiBestCost = std::numeric_limits<Distortion>::max();
  Int        i;

  AMVPInfo*  pcAMVPInfo = &rAMVPInfo;

  // Fill the MV Candidates
  if (!bFilled)
  {
    PU::fillMvpCand( pu, eRefPicList, iRefIdx, *pcAMVPInfo );
  }

  // initialize Mvp index & Mvp
  iBestIdx = 0;
  cBestMv  = pcAMVPInfo->mvCand[0];

  PelUnitBuf predBuf = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );

  //-- Check Minimum Cost.
  for( i = 0 ; i < pcAMVPInfo->numCand; i++)
  {
    Distortion uiTmpCost = xGetTemplateCost( pu, origBuf, predBuf, pcAMVPInfo->mvCand[i], i, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx );
    if( uiBestCost > uiTmpCost )
    {
      uiBestCost     = uiTmpCost;
      cBestMv        = pcAMVPInfo->mvCand[i];
      iBestIdx       = i;
      (*puiDistBiP)  = uiTmpCost;
    }
  }

  // Setting Best MVP
  rcMvPred = cBestMv;
  pu.mvpIdx[eRefPicList] = iBestIdx;
  pu.mvpNum[eRefPicList] = pcAMVPInfo->numCand;

  return;
}

UInt InterSearch::xGetMvpIdxBits(Int iIdx, Int iNum)
{
  CHECK(iIdx < 0 || iNum < 0 || iIdx >= iNum, "Invalid parameters");

  if (iNum == 1)
  {
    return 0;
  }

  UInt uiLength = 1;
  Int iTemp = iIdx;
  if ( iTemp == 0 )
  {
    return uiLength;
  }

  Bool bCodeLast = ( iNum-1 > iTemp );

  uiLength += (iTemp-1);

  if( bCodeLast )
  {
    uiLength++;
  }

  return uiLength;
}

Void InterSearch::xGetBlkBits( PartSize eCUMode, Bool bPSlice, Int iPartIdx, UInt uiLastMode, UInt uiBlkBit[3])
{
  if ( eCUMode == SIZE_2Nx2N )
  {
    uiBlkBit[0] = (! bPSlice) ? 3 : 1;
    uiBlkBit[1] = 3;
    uiBlkBit[2] = 5;
  }
#if HEVC_USE_PART_SIZE
  else if ( (eCUMode == SIZE_2NxN || eCUMode == SIZE_2NxnU) || eCUMode == SIZE_2NxnD )
  {
    UInt aauiMbBits[2][3][3] = { { {0,0,3}, {0,0,0}, {0,0,0} } , { {5,7,7}, {7,5,7}, {9-3,9-3,9-3} } };
    if ( bPSlice )
    {
      uiBlkBit[0] = 3;
      uiBlkBit[1] = 0;
      uiBlkBit[2] = 0;
    }
    else
    {
      ::memcpy( uiBlkBit, aauiMbBits[iPartIdx][uiLastMode], 3*sizeof(UInt) );
    }
  }
  else if ( (eCUMode == SIZE_Nx2N || eCUMode == SIZE_nLx2N) || eCUMode == SIZE_nRx2N )
  {
    UInt aauiMbBits[2][3][3] = { { {0,2,3}, {0,0,0}, {0,0,0} } , { {5,7,7}, {7-2,7-2,9-2}, {9-3,9-3,9-3} } };
    if ( bPSlice )
    {
      uiBlkBit[0] = 3;
      uiBlkBit[1] = 0;
      uiBlkBit[2] = 0;
    }
    else
    {
      ::memcpy( uiBlkBit, aauiMbBits[iPartIdx][uiLastMode], 3*sizeof(UInt) );
    }
  }
  else if ( eCUMode == SIZE_NxN )
  {
    uiBlkBit[0] = (! bPSlice) ? 3 : 1;
    uiBlkBit[1] = 3;
    uiBlkBit[2] = 5;
  }
#endif
  else
  {
    THROW("Wrong part size!");
  }
}

Void InterSearch::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)
{
  pDst->numCand = pSrc->numCand;
  for (Int i = 0; i < pSrc->numCand; i++)
  {
    pDst->mvCand[i] = pSrc->mvCand[i];
  }
}

Void InterSearch::xCheckBestMVP ( RefPicList eRefPicList, Mv cMv, Mv& rcMvPred, Int& riMVPIdx, AMVPInfo& amvpInfo, UInt& ruiBits, Distortion& ruiCost )
{

  AMVPInfo* pcAMVPInfo = &amvpInfo;

  CHECK(pcAMVPInfo->mvCand[riMVPIdx] != rcMvPred, "Invalid MV prediction candidate");

  if (pcAMVPInfo->numCand < 2)
  {
    return;
  }

  m_pcRdCost->setCostScale ( 0    );

  Int iBestMVPIdx = riMVPIdx;

  m_pcRdCost->setPredictor( rcMvPred );
  Int iOrgMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer());
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];
  Int iBestMvBits = iOrgMvBits;

  for (Int iMVPIdx = 0; iMVPIdx < pcAMVPInfo->numCand; iMVPIdx++)
  {
    if (iMVPIdx == riMVPIdx)
    {
      continue;
    }

    m_pcRdCost->setPredictor( pcAMVPInfo->mvCand[iMVPIdx] );
    Int iMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer());
    iMvBits += m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];

    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = iMVPIdx;
    }
  }

  if (iBestMVPIdx != riMVPIdx)  //if changed
  {
    rcMvPred = pcAMVPInfo->mvCand[iBestMVPIdx];

    riMVPIdx = iBestMVPIdx;
    UInt uiOrgBits = ruiBits;
    ruiBits = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost = (ruiCost - m_pcRdCost->getCost( uiOrgBits ))  + m_pcRdCost->getCost( ruiBits );
  }
}


Distortion InterSearch::xGetTemplateCost( const PredictionUnit& pu,
                                          PelUnitBuf& origBuf,
                                          PelUnitBuf& predBuf,
                                          Mv          cMvCand,
                                          Int         iMVPIdx,
                                          Int         iMVPNum,
                                          RefPicList  eRefPicList,
                                          Int         iRefIdx
)
{
  Distortion uiCost = std::numeric_limits<Distortion>::max();

  const Picture* picRef = pu.cu->slice->getRefPic( eRefPicList, iRefIdx );

  clipMv( cMvCand, pu.cu->lumaPos(), *pu.cs->sps );


  // prediction pattern
  const Bool bi = pu.cu->slice->testWeightPred() && pu.cu->slice->getSliceType()==P_SLICE;


  xPredInterBlk( COMPONENT_Y, pu, picRef, cMvCand, predBuf, bi, pu.cu->slice->clpRng( COMPONENT_Y )
                );


  if ( bi )
  {
    xWeightedPredictionUni( pu, predBuf, eRefPicList, predBuf, iRefIdx, m_maxCompIDToPred );
  }

  // calc distortion

  uiCost  = m_pcRdCost->getDistPart( origBuf.Y(), predBuf.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, DF_SAD );
  uiCost += m_pcRdCost->getCost( m_auiMVPIdxCost[iMVPIdx][iMVPNum] );

  return uiCost;
}


Void InterSearch::xMotionEstimation(PredictionUnit& pu, PelUnitBuf& origBuf, RefPicList eRefPicList, Mv& rcMvPred, Int iRefIdxPred, Mv& rcMv, Int& riMVPIdx, UInt& ruiBits, Distortion& ruiCost, const AMVPInfo& amvpInfo, Bool bBi)
{
  Mv cMvHalf, cMvQter;

  CHECK(eRefPicList >= MAX_NUM_REF_LIST_ADAPT_SR || iRefIdxPred>=Int(MAX_IDX_ADAPT_SR), "Invalid reference picture list");
  m_iSearchRange = m_aaiAdaptSR[eRefPicList][iRefIdxPred];

  Int    iSrchRng   = (bBi ? m_bipredSearchRange : m_iSearchRange);
  Double fWeight    = 1.0;

  PelUnitBuf  origBufTmp = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );
  PelUnitBuf* pBuf       = &origBuf;

  if(bBi) // Bi-predictive ME
  {
    // NOTE: Other buf contains predicted signal from another direction
    PelUnitBuf otherBuf = m_tmpPredStorage[1 - (Int)eRefPicList].getBuf( UnitAreaRelative(*pu.cu, pu ));
    origBufTmp.copyFrom(origBuf);
    origBufTmp.removeHighFreq(otherBuf, m_pcEncCfg->getClipForBiPredMeEnabled(), pu.cu->slice->clpRngs() );

    pBuf = &origBufTmp;

    fWeight = 0.5;
  }
  m_cDistParam.isBiPred = bBi;

  //  Search key pattern initialization
  CPelBuf  tmpPattern   = pBuf->Y();
  CPelBuf* pcPatternKey = &tmpPattern;

  m_lumaClpRng = pu.cs->slice->clpRng( COMPONENT_Y );

  CPelBuf buf = pu.cu->slice->getRefPic(eRefPicList, iRefIdxPred)->getRecoBuf(pu.blocks[COMPONENT_Y]);

  IntTZSearchStruct cStruct;
  cStruct.pcPatternKey  = pcPatternKey;
  cStruct.iRefStride    = buf.stride;
  cStruct.piRefY        = buf.buf;
  auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>( m_modeCtrl );

  bool bQTBTMV  = false;
  bool bQTBTMV2 = false;
  Mv cIntMv;
#if HM_ME_SR_VIOLATION
  if( bBi )
  {
    xSetSearchRange(pu, rcMv, iSrchRng, cStruct.searchRange);
  }
  else
  {
    bool bValid = blkCache && blkCache->getMv( pu, eRefPicList, iRefIdxPred, cIntMv );

    if( bValid )
    {
      cIntMv <<= 2;
      xSetSearchRange( pu, cIntMv, 1, cStruct.searchRange );
      bQTBTMV = true;
    }
    else
    {
      xSetSearchRange( pu, rcMvPred, iSrchRng, cStruct.searchRange );
    }
  }
#else
  if( !bBi )
  {
    bool bValid = blkCache && blkCache->getMv( pu, eRefPicList, iRefIdxPred, cIntMv );
    if( bValid )
    {
      bQTBTMV2 = true;
      cIntMv <<= 2;
    }
  }

#endif

  m_pcRdCost->setPredictor( rcMvPred );

  m_pcRdCost->setCostScale(2);

  {
    setWpScalingDistParam(iRefIdxPred, eRefPicList, pu.cu->slice);
  }

  //  Do integer search
  if( ( m_motionEstimationSearchMethod == MESEARCH_FULL ) || bBi || bQTBTMV )
  {
#if !HM_ME_SR_VIOLATION
    if( !bQTBTMV )
    {
      xSetSearchRange( pu, ( bBi ? rcMv : rcMvPred ), iSrchRng, cStruct.searchRange );
    }
#endif
    cStruct.subShiftMode = m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE3 ? 2 : 0;
    xPatternSearch( cStruct, rcMv, ruiCost);
  }
  else if( bQTBTMV2 )
  {
    rcMv = cIntMv;

    cStruct.subShiftMode = ( !m_pcEncCfg->getRestrictMESampling() && m_pcEncCfg->getMotionEstimationSearchMethod() == MESEARCH_SELECTIVE ) ? 1 :
                            ( m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE3 ) ? 2 : 0;
    xTZSearch( pu, cStruct, rcMv, ruiCost, NULL, false, true );
  }
  else
  {
    cStruct.subShiftMode = ( !m_pcEncCfg->getRestrictMESampling() && m_pcEncCfg->getMotionEstimationSearchMethod() == MESEARCH_SELECTIVE ) ? 1 :
                            ( m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE3 ) ? 2 : 0;
    rcMv = rcMvPred;
    const Mv *pIntegerMv2Nx2NPred = 0;
    if( !pu.cs->pcv->only2Nx2N && ( pu.cu->partSize != SIZE_2Nx2N || pu.cu->qtDepth != 0 ) )
    {
      pIntegerMv2Nx2NPred = &( m_integerMv2Nx2N[eRefPicList][iRefIdxPred] );
    }
    xPatternSearchFast( pu, cStruct, rcMv, ruiCost, pIntegerMv2Nx2NPred );
    if( blkCache )
    {
      blkCache->setMv( pu.cs->area, eRefPicList, iRefIdxPred, rcMv );
    }
    else if( pu.cu->partSize == SIZE_2Nx2N )
    {
      m_integerMv2Nx2N[eRefPicList][iRefIdxPred] = rcMv;
    }
  }

  DTRACE( g_trace_ctx, D_ME, "%d %d %d :MECostFPel<L%d,%d>: %d,%d,%dx%d,%2d: %d", DTRACE_GET_COUNTER( g_trace_ctx, D_ME ), pu.cu->slice->getPOC(), 0, ( int ) eRefPicList, ( int ) bBi, pu.Y().x, pu.Y().y, pu.Y().width, pu.Y().height, pu.cu->partSize, ruiCost );
  // sub-pel refinement for sub-pel resolution
  {
    xPatternSearchFracDIF( pu, eRefPicList, iRefIdxPred, cStruct, rcMv, cMvHalf, cMvQter, ruiCost );
    m_pcRdCost->setCostScale( 0 );
    rcMv <<= 2;
    rcMv  += ( cMvHalf <<= 1 );
    rcMv  += cMvQter;
    UInt uiMvBits = m_pcRdCost->getBitsOfVectorWithPredictor( rcMv.getHor(), rcMv.getVer() );
    ruiBits += uiMvBits;
    ruiCost = ( Distortion ) ( floor( fWeight * ( ( Double ) ruiCost - ( Double ) m_pcRdCost->getCost( uiMvBits ) ) ) + ( Double ) m_pcRdCost->getCost( ruiBits ) );
  }
  DTRACE( g_trace_ctx, D_ME, "   MECost<L%d,%d>: %6d (%d)  MV:%d,%d\n", ( int ) eRefPicList, ( int ) bBi, ruiCost, ruiBits, rcMv.getHor(), rcMv.getVer() );
}



Void InterSearch::xSetSearchRange ( const PredictionUnit& pu,
                                    const Mv& cMvPred,
                                    const Int iSrchRng,
                                    SearchRange& sr )
{
  const Int iMvShift = 2;
  Mv cFPMvPred = cMvPred;
  clipMv( cFPMvPred, pu.cu->lumaPos(), *pu.cs->sps );

  Mv mvTL( cFPMvPred.getHor() - ( iSrchRng << iMvShift ), cFPMvPred.getVer() - ( iSrchRng << iMvShift ) );
  Mv mvBR( cFPMvPred.getHor() + ( iSrchRng << iMvShift ), cFPMvPred.getVer() + ( iSrchRng << iMvShift ) );

  clipMv( mvTL, pu.cu->lumaPos(), *pu.cs->sps );
  clipMv( mvBR, pu.cu->lumaPos(), *pu.cs->sps );

  mvTL.divideByPowerOf2( iMvShift );
  mvBR.divideByPowerOf2( iMvShift );

  sr.left   = mvTL.hor;
  sr.top    = mvTL.ver;
  sr.right  = mvBR.hor;
  sr.bottom = mvBR.ver;
}


Void InterSearch::xPatternSearch( IntTZSearchStruct&    cStruct,
                                  Mv&            rcMv,
                                  Distortion&    ruiSAD )
{
  Distortion  uiSad;
  Distortion  uiSadBest = std::numeric_limits<Distortion>::max();
  Int         iBestX = 0;
  Int         iBestY = 0;

  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode );

  const SearchRange& sr = cStruct.searchRange;

  const Pel* piRef = cStruct.piRefY + (sr.top * cStruct.iRefStride);
  for ( Int y = sr.top; y <= sr.bottom; y++ )
  {
    for ( Int x = sr.left; x <= sr.right; x++ )
    {
      //  find min. distortion position
      m_cDistParam.cur.buf = piRef + x;

      uiSad = m_cDistParam.distFunc( m_cDistParam );

      // motion cost
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor( x, y );

      if ( uiSad < uiSadBest )
      {
        uiSadBest = uiSad;
        iBestX    = x;
        iBestY    = y;
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }
    piRef += cStruct.iRefStride;
  }

  rcMv.set( iBestX, iBestY );

  cStruct.uiBestSad = uiSadBest; // th for testing
  ruiSAD = uiSadBest - m_pcRdCost->getCostOfVectorWithPredictor( iBestX, iBestY );
  return;
}


Void InterSearch::xPatternSearchFast( const PredictionUnit& pu,
                                      IntTZSearchStruct&    cStruct,
                                      Mv&                   rcMv,
                                      Distortion&           ruiSAD,
                                      const Mv* const       pIntegerMv2Nx2NPred )
{
  switch ( m_motionEstimationSearchMethod )
  {
  case MESEARCH_DIAMOND:
    xTZSearch         ( pu, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred, false );
    break;

  case MESEARCH_SELECTIVE:
    xTZSearchSelective( pu, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred );
    break;

  case MESEARCH_DIAMOND_ENHANCED:
    xTZSearch         ( pu, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred, true );
    break;

  case MESEARCH_FULL: // shouldn't get here.
  default:
    break;
  }
}


Void InterSearch::xTZSearch( const PredictionUnit& pu,
                             IntTZSearchStruct&    cStruct,
                             Mv&                   rcMv,
                             Distortion&           ruiSAD,
                             const Mv* const       pIntegerMv2Nx2NPred,
                             const Bool            bExtendedSettings,
                             const Bool            bFastSettings)
{
  const bool bUseRasterInFastMode                    = true; //toggle this to further reduce runtime

  const Bool bUseAdaptiveRaster                      = bExtendedSettings;
  const Int  iRaster                                 = (bFastSettings && bUseRasterInFastMode) ? 8 : 5;
  const Bool bTestZeroVector                         = true && !bFastSettings;
  const Bool bTestZeroVectorStart                    = bExtendedSettings;
  const Bool bTestZeroVectorStop                     = false;
  const Bool bFirstSearchDiamond                     = true;  // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const Bool bFirstCornersForDiamondDist1            = bExtendedSettings;
  const Bool bFirstSearchStop                        = m_pcEncCfg->getFastMEAssumingSmootherMVEnabled();
  const UInt uiFirstSearchRounds                     = bFastSettings ? (bUseRasterInFastMode?3:2) : 3;     // first search stop X rounds after best match (must be >=1)
  const Bool bEnableRasterSearch                     = bFastSettings ? bUseRasterInFastMode : true;
  const Bool bAlwaysRasterSearch                     = bExtendedSettings;  // true: BETTER but factor 2 slower
  const Bool bRasterRefinementEnable                 = false; // enable either raster refinement or star refinement
  const Bool bRasterRefinementDiamond                = false; // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const Bool bRasterRefinementCornersForDiamondDist1 = bExtendedSettings;
  const Bool bStarRefinementEnable                   = true;  // enable either star refinement or raster refinement
  const Bool bStarRefinementDiamond                  = true;  // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const Bool bStarRefinementCornersForDiamondDist1   = bExtendedSettings;
  const Bool bStarRefinementStop                     = false || bFastSettings;
  const UInt uiStarRefinementRounds                  = 2;  // star refinement stop X rounds after best match (must be >=1)
  const Bool bNewZeroNeighbourhoodTest               = bExtendedSettings;

  Int iSearchRange = m_iSearchRange;

  clipMv( rcMv, pu.cu->lumaPos(), *pu.cs->sps );
  rcMv.divideByPowerOf2(2);

  // init TZSearchStruct
  cStruct.uiBestSad   = MAX_UINT;

  //
  m_cDistParam.maximumDistortionForEarlyExit = cStruct.uiBestSad;
  m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode );

  // distortion


  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0 );

  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    if ((rcMv.getHor() != 0 || rcMv.getVer() != 0) &&
      (0 != cStruct.iBestX || 0 != cStruct.iBestY))
    {
      // only test 0-vector if not obviously previously tested.
      xTZSearchHelp( cStruct, 0, 0, 0, 0 );
    }
  }

#if HM_ME_SR_VIOLATION
  SearchRange sr = cStruct.searchRange;
#else
  SearchRange& sr = cStruct.searchRange;
#endif

  if (pIntegerMv2Nx2NPred != 0)
  {
    Mv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred <<= 2;
    clipMv( integerMv2Nx2NPred, pu.cu->lumaPos(), *pu.cs->sps );
    integerMv2Nx2NPred.divideByPowerOf2(2);

    if ((rcMv != integerMv2Nx2NPred) &&
      (integerMv2Nx2NPred.getHor() != cStruct.iBestX || integerMv2Nx2NPred.getVer() != cStruct.iBestY))
    {
      // only test integerMv2Nx2NPred if not obviously previously tested.
      xTZSearchHelp( cStruct, integerMv2Nx2NPred.getHor(), integerMv2Nx2NPred.getVer(), 0, 0);
    }
#if !HM_ME_SR_VIOLATION
  }
  {
#endif
    // set search range
    Mv currBestMv(cStruct.iBestX, cStruct.iBestY );
    currBestMv <<= 2;
    xSetSearchRange( pu, currBestMv, m_iSearchRange>>(bFastSettings?1:0), sr );
  }

  // start search
  Int  iDist = 0;
  Int  iStartX = cStruct.iBestX;
  Int  iStartY = cStruct.iBestY;

  const Bool bBestCandidateZero = (cStruct.iBestX == 0) && (cStruct.iBestY == 0);

  // first search around best position up to now.
  // The following works as a "subsampled/log" window search around the best candidate
  for ( iDist = 1; iDist <= iSearchRange; iDist*=2 )
  {
    if ( bFirstSearchDiamond == 1 )
    {
      xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, bFirstCornersForDiamondDist1 );
    }
    else
    {
      xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
    }

    if ( bFirstSearchStop && ( cStruct.uiBestRound >= uiFirstSearchRounds ) ) // stop criterion
    {
      break;
    }
  }

  if (!bNewZeroNeighbourhoodTest)
  {
    // test whether zero Mv is a better start point than Median predictor
    if ( bTestZeroVectorStart && ((cStruct.iBestX != 0) || (cStruct.iBestY != 0)) )
    {
      xTZSearchHelp( cStruct, 0, 0, 0, 0 );
      if ( (cStruct.iBestX == 0) && (cStruct.iBestY == 0) )
      {
        // test its neighborhood
        for ( iDist = 1; iDist <= iSearchRange; iDist*=2 )
        {
          xTZ8PointDiamondSearch( cStruct, 0, 0, iDist, false );
          if ( bTestZeroVectorStop && (cStruct.uiBestRound > 0) ) // stop criterion
          {
            break;
          }
        }
      }
    }
  }
  else
  {
    // Test also zero neighbourhood but with half the range
    // It was reported that the original (above) search scheme using bTestZeroVectorStart did not
    // make sense since one would have already checked the zero candidate earlier
    // and thus the conditions for that test would have not been satisfied
    if (bTestZeroVectorStart == true && bBestCandidateZero != true)
    {
      for ( iDist = 1; iDist <= (iSearchRange >> 1); iDist*=2 )
      {
        xTZ8PointDiamondSearch( cStruct, 0, 0, iDist, false );
        if ( bTestZeroVectorStop && (cStruct.uiBestRound > 2) ) // stop criterion
        {
          break;
        }
      }
    }
  }

  // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
  if ( cStruct.uiBestDistance == 1 )
  {
    cStruct.uiBestDistance = 0;
    xTZ2PointSearch( cStruct );
  }

  // raster search if distance is too big
  if (bUseAdaptiveRaster)
  {
    int iWindowSize     = iRaster;
    SearchRange localsr = sr;

    if (!(bEnableRasterSearch && ( ((Int)(cStruct.uiBestDistance) >= iRaster))))
    {
      iWindowSize ++;
      localsr.left   /= 2;
      localsr.right  /= 2;
      localsr.top    /= 2;
      localsr.bottom /= 2;
    }
    cStruct.uiBestDistance = iWindowSize;
    for ( iStartY = localsr.top; iStartY <= localsr.bottom; iStartY += iWindowSize )
    {
      for ( iStartX = localsr.left; iStartX <= localsr.right; iStartX += iWindowSize )
      {
        xTZSearchHelp( cStruct, iStartX, iStartY, 0, iWindowSize );
      }
    }
  }
  else
  {
    if ( bEnableRasterSearch && ( ((Int)(cStruct.uiBestDistance) >= iRaster) || bAlwaysRasterSearch ) )
    {
      cStruct.uiBestDistance = iRaster;
      for ( iStartY = sr.top; iStartY <= sr.bottom; iStartY += iRaster )
      {
        for ( iStartX = sr.left; iStartX <= sr.right; iStartX += iRaster )
        {
          xTZSearchHelp( cStruct, iStartX, iStartY, 0, iRaster );
        }
      }
    }
  }

  // raster refinement

  if ( bRasterRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      if ( cStruct.uiBestDistance > 1 )
      {
        iDist = cStruct.uiBestDistance >>= 1;
        if ( bRasterRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, bRasterRefinementCornersForDiamondDist1 );
        }
        else
        {
          xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
        }
      }

      // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( cStruct );
        }
      }
    }
  }

  // star refinement
  if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < iSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, bStarRefinementCornersForDiamondDist1 );
        }
        else
        {
          xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( cStruct );
        }
      }
    }
  }

  // write out best match
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor( cStruct.iBestX, cStruct.iBestY );
}


Void InterSearch::xTZSearchSelective( const PredictionUnit& pu,
                                      IntTZSearchStruct&    cStruct,
                                      Mv                    &rcMv,
                                      Distortion            &ruiSAD,
                                      const Mv* const       pIntegerMv2Nx2NPred )
{
  const Bool bTestZeroVector          = true;
  const Bool bEnableRasterSearch      = true;
  const Bool bAlwaysRasterSearch      = false;  // 1: BETTER but factor 15x slower
  const Bool bStarRefinementEnable    = true;   // enable either star refinement or raster refinement
  const Bool bStarRefinementDiamond   = true;   // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const Bool bStarRefinementStop      = false;
  const UInt uiStarRefinementRounds   = 2;  // star refinement stop X rounds after best match (must be >=1)
  const Int  iSearchRange             = m_iSearchRange;
  const Int  iSearchRangeInitial      = m_iSearchRange >> 2;
  const Int  uiSearchStep             = 4;
  const Int  iMVDistThresh            = 8;

  Int   iStartX                 = 0;
  Int   iStartY                 = 0;
  Int   iDist                   = 0;

  clipMv( rcMv, pu.cu->lumaPos(), *pu.cs->sps );

  rcMv.divideByPowerOf2(2);

  // init TZSearchStruct
  cStruct.uiBestSad   = MAX_UINT;
  cStruct.iBestX = 0;
  cStruct.iBestY = 0;

  m_cDistParam.maximumDistortionForEarlyExit = cStruct.uiBestSad;
  m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode );


  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0 );

  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    xTZSearchHelp( cStruct, 0, 0, 0, 0 );
  }

#if HM_ME_SR_VIOLATION
  SearchRange sr = cStruct.searchRange;
#else
  SearchRange& sr = cStruct.searchRange;
#endif

  if ( pIntegerMv2Nx2NPred != 0 )
  {
    Mv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred <<= 2;
    clipMv( integerMv2Nx2NPred, pu.cu->lumaPos(), *pu.cs->sps );
    integerMv2Nx2NPred.divideByPowerOf2(2);

    xTZSearchHelp( cStruct, integerMv2Nx2NPred.getHor(), integerMv2Nx2NPred.getVer(), 0, 0);

#if !HM_ME_SR_VIOLATION
  }
  {
#endif
    // set search range
    Mv currBestMv(cStruct.iBestX, cStruct.iBestY );
    currBestMv <<= 2;
    xSetSearchRange( pu, currBestMv, m_iSearchRange, sr );
  }

  // Initial search
  Int iBestX = cStruct.iBestX;
  Int iBestY = cStruct.iBestY;
  Int iFirstSrchRngHorLeft    = ((iBestX - iSearchRangeInitial) > sr.left)   ? (iBestX - iSearchRangeInitial) : sr.left;
  Int iFirstSrchRngVerTop     = ((iBestY - iSearchRangeInitial) > sr.top)    ? (iBestY - iSearchRangeInitial) : sr.top;
  Int iFirstSrchRngHorRight   = ((iBestX + iSearchRangeInitial) < sr.right)  ? (iBestX + iSearchRangeInitial) : sr.right;
  Int iFirstSrchRngVerBottom  = ((iBestY + iSearchRangeInitial) < sr.bottom) ? (iBestY + iSearchRangeInitial) : sr.bottom;

  for ( iStartY = iFirstSrchRngVerTop; iStartY <= iFirstSrchRngVerBottom; iStartY += uiSearchStep )
  {
    for ( iStartX = iFirstSrchRngHorLeft; iStartX <= iFirstSrchRngHorRight; iStartX += uiSearchStep )
    {
      xTZSearchHelp( cStruct, iStartX, iStartY, 0, 0 );
      xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, 1, false );
      xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, 2, false );
    }
  }

  Int iMaxMVDistToPred = (abs(cStruct.iBestX - iBestX) > iMVDistThresh || abs(cStruct.iBestY - iBestY) > iMVDistThresh);

  //full search with early exit if MV is distant from predictors
  if ( bEnableRasterSearch && (iMaxMVDistToPred || bAlwaysRasterSearch) )
  {
    for ( iStartY = sr.top; iStartY <= sr.bottom; iStartY += 1 )
    {
      for ( iStartX = sr.left; iStartX <= sr.right; iStartX += 1 )
      {
        xTZSearchHelp( cStruct, iStartX, iStartY, 0, 1 );
      }
    }
  }
  //Smaller MV, refine around predictor
  else if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    // start refinement
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < iSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, false );
        }
        else
        {
          xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( cStruct );
        }
      }
    }
  }

  // write out best match
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor( cStruct.iBestX, cStruct.iBestY );
}


Void InterSearch::xPatternSearchFracDIF(
  const PredictionUnit& pu,
  RefPicList            eRefPicList,
  Int                   iRefIdx,
  IntTZSearchStruct&    cStruct,
  const Mv&             rcMvInt,
  Mv&                   rcMvHalf,
  Mv&                   rcMvQter,
  Distortion&           ruiCost
)
{
  const Bool bIsLosslessCoded = pu.cu->transQuantBypass;

  //  Reference pattern initialization (integer scale)
  Int         iOffset    = rcMvInt.getHor() + rcMvInt.getVer() * cStruct.iRefStride;
  CPelBuf cPatternRoi(cStruct.piRefY + iOffset, cStruct.iRefStride, *cStruct.pcPatternKey);



  //  Half-pel refinement
  m_pcRdCost->setCostScale(1);
  xExtDIFUpSamplingH ( &cPatternRoi );

  rcMvHalf = rcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  Mv baseRefMv(0, 0);
  ruiCost = xPatternRefinement(cStruct.pcPatternKey, baseRefMv, 2, rcMvHalf, !bIsLosslessCoded);

  //  quarter-pel refinement
  m_pcRdCost->setCostScale( 0 );
  xExtDIFUpSamplingQ ( &cPatternRoi, rcMvHalf );
  baseRefMv = rcMvHalf;
  baseRefMv <<= 1;

  rcMvQter = rcMvInt;    rcMvQter <<= 1;    // for mv-cost
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  ruiCost = xPatternRefinement( cStruct.pcPatternKey, baseRefMv, 1, rcMvQter, !bIsLosslessCoded );
}



/**
* \brief Generate half-sample interpolated block
*
* \param pattern Reference picture ROI
* \param biPred    Flag indicating whether block is for biprediction
*/
Void InterSearch::xExtDIFUpSamplingH( CPelBuf* pattern )
{
  const ClpRng& clpRng = m_lumaClpRng;
  Int width      = pattern->width;
  Int height     = pattern->height;
  Int srcStride  = pattern->stride;

  Int intStride = width + 1;
  Int dstStride = width + 1;
  Pel *intPtr;
  Pel *dstPtr;
  Int filterSize = NTAPS_LUMA;
  Int halfFilterSize = (filterSize>>1);
  const Pel *srcPtr = pattern->buf - halfFilterSize*srcStride - 1;

  const ChromaFormat chFmt = m_currChromaFormat;

  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[0][0], intStride, width + 1, height + filterSize, 0, false, chFmt, clpRng);
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[2][0], intStride, width + 1, height + filterSize, 2, false, chFmt, clpRng);

  intPtr = m_filteredBlockTmp[0][0] + halfFilterSize * intStride + 1;
  dstPtr = m_filteredBlock[0][0][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 0, 0, false, true, chFmt, clpRng);

  intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
  dstPtr = m_filteredBlock[2][0][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 1, 2, false, true, chFmt, clpRng);

  intPtr = m_filteredBlockTmp[2][0] + halfFilterSize * intStride;
  dstPtr = m_filteredBlock[0][2][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 1, height + 0, 0, false, true, chFmt, clpRng);

  intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[2][2][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 1, height + 1, 2, false, true, chFmt, clpRng);
}





/**
* \brief Generate quarter-sample interpolated blocks
*
* \param pattern    Reference picture ROI
* \param halfPelRef Half-pel mv
* \param biPred     Flag indicating whether block is for biprediction
*/
Void InterSearch::xExtDIFUpSamplingQ( CPelBuf* pattern, Mv halfPelRef )
{
  const ClpRng& clpRng = m_lumaClpRng;
  Int width      = pattern->width;
  Int height     = pattern->height;
  Int srcStride  = pattern->stride;

  Pel const* srcPtr;
  Int intStride = width + 1;
  Int dstStride = width + 1;
  Pel *intPtr;
  Pel *dstPtr;
  Int filterSize = NTAPS_LUMA;

  Int halfFilterSize = (filterSize>>1);

  Int extHeight = (halfPelRef.getVer() == 0) ? height + filterSize : height + filterSize-1;

  const ChromaFormat chFmt = m_currChromaFormat;

  // Horizontal filter 1/4
  srcPtr = pattern->buf - halfFilterSize * srcStride - 1;
  intPtr = m_filteredBlockTmp[1][0];
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() >= 0)
  {
    srcPtr += 1;
  }
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 1, false, chFmt, clpRng);

  // Horizontal filter 3/4
  srcPtr = pattern->buf - halfFilterSize*srcStride - 1;
  intPtr = m_filteredBlockTmp[3][0];
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() > 0)
  {
    srcPtr += 1;
  }
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 3, false, chFmt, clpRng);

  // Generate @ 1,1
  intPtr = m_filteredBlockTmp[1][0] + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[1][1][0];
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, clpRng);

  // Generate @ 3,1
  intPtr = m_filteredBlockTmp[1][0] + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[3][1][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, clpRng);

  if (halfPelRef.getVer() != 0)
  {
    // Generate @ 2,1
    intPtr = m_filteredBlockTmp[1][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[2][1][0];
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true, chFmt, clpRng);

    // Generate @ 2,3
    intPtr = m_filteredBlockTmp[3][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[2][3][0];
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true, chFmt, clpRng);
  }
  else
  {
    // Generate @ 0,1
    intPtr = m_filteredBlockTmp[1][0] + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][1][0];
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true, chFmt, clpRng);

    // Generate @ 0,3
    intPtr = m_filteredBlockTmp[3][0] + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][3][0];
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true, chFmt, clpRng);
  }

  if (halfPelRef.getHor() != 0)
  {
    // Generate @ 1,2
    intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[1][2][0];
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, clpRng);

    // Generate @ 3,2
    intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[3][2][0];
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, clpRng);
  }
  else
  {
    // Generate @ 1,0
    intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
    dstPtr = m_filteredBlock[1][0][0];
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, clpRng);

    // Generate @ 3,0
    intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
    dstPtr = m_filteredBlock[3][0][0];
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, clpRng);
  }

  // Generate @ 1,3
  intPtr = m_filteredBlockTmp[3][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[1][3][0];
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, clpRng);

  // Generate @ 3,3
  intPtr = m_filteredBlockTmp[3][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[3][3][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, clpRng);
}





//! set wp tables
Void InterSearch::setWpScalingDistParam( Int iRefIdx, RefPicList eRefPicListCur, Slice *pcSlice )
{
  if ( iRefIdx<0 )
  {
    m_cDistParam.applyWeight = false;
    return;
  }

  WPScalingParam  *wp0 , *wp1;

  m_cDistParam.applyWeight = ( pcSlice->getSliceType()==P_SLICE && pcSlice->testWeightPred() ) || ( pcSlice->getSliceType()==B_SLICE && pcSlice->testWeightBiPred() ) ;

  if ( !m_cDistParam.applyWeight )
  {
    return;
  }

  Int iRefIdx0 = ( eRefPicListCur == REF_PIC_LIST_0 ) ? iRefIdx : (-1);
  Int iRefIdx1 = ( eRefPicListCur == REF_PIC_LIST_1 ) ? iRefIdx : (-1);

  getWpScaling( pcSlice, iRefIdx0, iRefIdx1, wp0 , wp1 );

  if ( iRefIdx0 < 0 )
  {
    wp0 = NULL;
  }
  if ( iRefIdx1 < 0 )
  {
    wp1 = NULL;
  }

  m_cDistParam.wpCur  = NULL;

  if ( eRefPicListCur == REF_PIC_LIST_0 )
  {
    m_cDistParam.wpCur = wp0;
  }
  else
  {
    m_cDistParam.wpCur = wp1;
  }
}

Void InterSearch::xEncodeInterResidualQT(CodingStructure &cs, Partitioner &partitioner, const ComponentID &compID)
{
  const UnitArea& currArea    = partitioner.currArea();
#if HEVC_USE_RQT
  const SPS &sps              = *cs.sps;
#endif
  const TransformUnit &currTU = *cs.getTU(currArea.lumaPos(), partitioner.chType);
  const CodingUnit &cu        = *currTU.cu;
#if HEVC_USE_RQT || ENABLE_BMS
  const unsigned currDepth    = partitioner.currTrDepth;

  const Bool bSubdiv          = currDepth != currTU.depth;
#if HEVC_USE_RQT
  const UInt uiLog2TrSize     = g_aucLog2[currArea.lumaSize().width];
#endif
#endif

  if (compID == MAX_NUM_TBLOCKS)  // we are not processing a channel, instead we always recurse and code the CBFs
  {
#if HEVC_USE_RQT || ENABLE_BMS
    if( cs.pcv->noRQT )
    {
#if ENABLE_BMS
      if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
      {
        CHECK( !bSubdiv, "Not performing the implicit TU split" );
      }
      else
#endif
      CHECK( bSubdiv, "transformsplit not supported" );
    }
#endif
#if HEVC_USE_RQT
    else if (uiLog2TrSize <= sps.getQuadtreeTULog2MaxSize() && uiLog2TrSize > CU::getQuadtreeTULog2MinSizeInCU(cu))
    {
      if (sps.getQuadtreeTUMaxDepthInter() == 1 && cu.partSize != SIZE_2Nx2N)
      {
        CHECK(!bSubdiv, "Implicit subdivision ignored"); // Inferred splitting rule - see derivation and use of interSplitFlag in the specification.
      }
      else
      {
        if( sps.getSpsNext().nextToolsEnabled() )
        {
          m_CABACEstimator->split_transform_flag( bSubdiv, sps.getQuadtreeTULog2MaxSize() - uiLog2TrSize );
        }
        else
        {
          m_CABACEstimator->split_transform_flag( bSubdiv, 5 - uiLog2TrSize );
        }
      }
    }

#endif
    CHECK(CU::isIntra(cu), "Inter search provided with intra CU");

    if( cu.chromaFormat != CHROMA_400 )
    {
#if HEVC_USE_RQT || ENABLE_BMS
      const bool firstCbfOfCU = ( currDepth == 0 );
#if HEVC_USE_RQT
      const bool allQuadrants = TU::isProcessingAllQuadrants( currArea );
#endif
#if ENABLE_CHROMA_422
      const bool twoChromaCbfs = ( cs.pcv->multiBlock422 && ( !bSubdiv || currArea.lumaSize().width == 8 ) );
      if( twoChromaCbfs )
      {
        if( firstCbfOfCU || ( allQuadrants && TU::getCbfAtDepth( currTU, COMPONENT_Cb, currDepth - 1 ) ) )
        {
          const bool  chroma_cbf1 = TU::getCbfAtDepth( currTU, COMPONENT_Cb, currDepth );
          const bool  chroma_cbf2 = TU::getCbfAtDepth( currTU, COMPONENT_Cb2, currDepth );
          m_CABACEstimator->cbf_comp( cs, chroma_cbf1, currArea.blocks[COMPONENT_Cb], currDepth );
          m_CABACEstimator->cbf_comp( cs, chroma_cbf2, currArea.blocks[COMPONENT_Cb], currDepth );
        }
        if( firstCbfOfCU || ( allQuadrants && TU::getCbfAtDepth( currTU, COMPONENT_Cr, currDepth - 1 ) ) )
        {
          const bool  chroma_cbf1 = TU::getCbfAtDepth( currTU, COMPONENT_Cr, currDepth );
          const bool  chroma_cbf2 = TU::getCbfAtDepth( currTU, COMPONENT_Cr2, currDepth );
          m_CABACEstimator->cbf_comp( cs, chroma_cbf1, currArea.blocks[COMPONENT_Cr], currDepth );
          m_CABACEstimator->cbf_comp( cs, chroma_cbf2, currArea.blocks[COMPONENT_Cr], currDepth );
        }
      }
      else
#endif
      {
#if HEVC_USE_RQT
        if( firstCbfOfCU || ( allQuadrants && TU::getCbfAtDepth( currTU, COMPONENT_Cb, currDepth - 1 ) ) )
#else
        if( firstCbfOfCU || TU::getCbfAtDepth( currTU, COMPONENT_Cb, currDepth - 1 ) )
#endif
        {
          const bool  chroma_cbf = TU::getCbfAtDepth( currTU, COMPONENT_Cb, currDepth );
          m_CABACEstimator->cbf_comp( cs, chroma_cbf, currArea.blocks[COMPONENT_Cb], currDepth );
        }
#if HEVC_USE_RQT
        if( firstCbfOfCU || ( allQuadrants && TU::getCbfAtDepth( currTU, COMPONENT_Cr, currDepth - 1 ) ) )
#else
        if( firstCbfOfCU || TU::getCbfAtDepth( currTU, COMPONENT_Cr, currDepth - 1 ) )
#endif
        {
          const bool  chroma_cbf = TU::getCbfAtDepth( currTU, COMPONENT_Cr, currDepth );
          m_CABACEstimator->cbf_comp( cs, chroma_cbf, currArea.blocks[COMPONENT_Cr], currDepth );
        }
      }
    }

    if( !bSubdiv )
    {
      m_CABACEstimator->cbf_comp( cs, TU::getCbfAtDepth( currTU, COMPONENT_Y, currDepth ), currArea.Y(), currDepth );
    }
#else
      m_CABACEstimator->cbf_comp( cs, TU::getCbf( currTU, COMPONENT_Cb ), currArea.blocks[COMPONENT_Cb] );
      m_CABACEstimator->cbf_comp( cs, TU::getCbf( currTU, COMPONENT_Cr ), currArea.blocks[COMPONENT_Cr] );
    }

    m_CABACEstimator->cbf_comp( cs, TU::getCbf( currTU, COMPONENT_Y ), currArea.Y() );
#endif
  }

#if HEVC_USE_RQT || ENABLE_BMS
  if (!bSubdiv)
#endif
  {
    if (compID != MAX_NUM_TBLOCKS) // we have already coded the CBFs, so now we code coefficients
    {
      if( currArea.blocks[compID].valid() )
      {
        if( TU::hasCrossCompPredInfo( currTU, compID ) )
        {
          m_CABACEstimator->cross_comp_pred( currTU, compID );
        }
        if( TU::getCbf( currTU, compID ) )
        {
          m_CABACEstimator->residual_coding( currTU, compID );
        }
#if ENABLE_CHROMA_422
        if( cs.pcv->multiBlock422 && compID != COMPONENT_Y )
        {
          ComponentID compID2 = ComponentID( compID + SCND_TBLOCK_OFFSET );
          if( TU::getCbf( currTU, compID2 ) )
          {
            m_CABACEstimator->residual_coding( currTU, compID2 );
          }
        }
#endif
      }
    }
  }
#if HEVC_USE_RQT || ENABLE_BMS
  else
  {
    if( compID == MAX_NUM_TBLOCKS || TU::getCbfAtDepth( currTU, compID, currDepth ) )
    {
#if ENABLE_BMS
      if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
      {
        partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
      }
      else
#endif
#if HEVC_USE_RQT
#if HM_REPRODUCE_4x4_BLOCK_ESTIMATION_ORDER
      partitioner.splitCurrArea( TU_QUAD_SPLIT_HM, cs );
#else
      partitioner.splitCurrArea( TU_QUAD_SPLIT   , cs );
#endif
#else
        THROW( "Implicit TU split not available!" );
#endif

      do
      {
        xEncodeInterResidualQT( cs, partitioner, compID );
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
    }
  }
#endif
}

Void InterSearch::xEstimateInterResidualQT(CodingStructure &cs, Partitioner &partitioner, Distortion *puiZeroDist /*= NULL*/)
{
  const UnitArea& currArea = partitioner.currArea();
  const SPS &sps           = *cs.sps;
  const PPS &pps           = *cs.pps;
  const UInt numValidComp  = getNumberValidComponents( sps.getChromaFormatIdc() );
  const UInt numTBlocks    = getNumberValidTBlocks   ( *cs.pcv );
#if HEVC_USE_RQT || ENABLE_BMS
  const unsigned currDepth = partitioner.currTrDepth;

#if HEVC_USE_RQT
  const CodingUnit &cu     = *cs.getCU( partitioner.chType );
  const UInt uiLog2TrSize  = g_aucLog2[currArea.lumaSize().width];

  Bool SplitFlag = (sps.getQuadtreeTUMaxDepthInter() == 1) && CU::isInter(cu) && cu.partSize != SIZE_2Nx2N;

  Bool bCheckFull;

  if (SplitFlag && currDepth == 0 && uiLog2TrSize > CU::getQuadtreeTULog2MinSizeInCU(cu))
  {
    bCheckFull = false;
  }
  else if ( !cs.pcv->noRQT )
  {
    bCheckFull = uiLog2TrSize <= sps.getQuadtreeTULog2MaxSize();
  }
  else
  {
    bCheckFull = true;
  }

  Bool bCheckSplit = uiLog2TrSize > CU::getQuadtreeTULog2MinSizeInCU( cu ) && !cs.pcv->noRQT;

  CHECK(!bCheckFull && !bCheckSplit, "Testing disabled for both full search and subdivision");
#else
  Bool bCheckSplit = false, bCheckFull = false;
#endif
#if ENABLE_BMS
  if( cs.pcv->noRQT )
  {
    bCheckFull  = !partitioner.canSplit( TU_MAX_TR_SPLIT, cs );
    bCheckSplit = !bCheckFull;
  }
#endif

  // get temporary data
  CodingStructure *csSplit = nullptr;
  CodingStructure *csFull  = nullptr;
#if HEVC_USE_RQT
  if (bCheckFull && bCheckSplit)
  {
    csSplit = m_pSplitCS[gp_sizeIdxInfo->idxFrom( cu.lwidth() )][gp_sizeIdxInfo->idxFrom( cu.lheight() )][currDepth];
    csFull  = m_pFullCS [gp_sizeIdxInfo->idxFrom( cu.lwidth() )][gp_sizeIdxInfo->idxFrom( cu.lheight() )][currDepth];

    cs.initSubStructure(*csSplit, partitioner.chType, cs.area, true);
    cs.initSubStructure(*csFull,  partitioner.chType, cs.area, true);

    csFull ->getOrgResiBuf().copyFrom(cs.getOrgResiBuf());
    csSplit->getOrgResiBuf().copyFrom(cs.getOrgResiBuf());
  }
  else
#endif
  if (bCheckSplit)
  {
    csSplit = &cs;
  }
  else if (bCheckFull)
  {
    csFull = &cs;
  }
#else
  bool bCheckFull = true;
  CodingStructure *csFull = &cs;
#endif

  Distortion uiSingleDist         = 0;
#if ENABLE_CHROMA_422
  Distortion uiSingleDistComp [5] = { 0, 0, 0, 0, 0 };
  TCoeff     uiAbsSum         [5] = { 0, 0, 0, 0, 0 };
#else
  Distortion uiSingleDistComp [3] = { 0, 0, 0 };
  TCoeff     uiAbsSum         [3] = { 0, 0, 0 };
#endif

  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );
  TempCtx       ctxBest   ( m_CtxCache );

  if (bCheckFull)
  {
    TransformUnit &tu = csFull->addTU(currArea, partitioner.chType);
#if HEVC_USE_RQT || ENABLE_BMS
    tu.depth          = currDepth;
#endif

    Double minCost            [MAX_NUM_TBLOCKS];
    Bool   checkTransformSkip [MAX_NUM_TBLOCKS];

    m_CABACEstimator->resetBits();

    memset(m_pTempPel, 0, sizeof(Pel) * tu.Y().area()); // not necessary needed for inside of recursion (only at the beginning)

    for (UInt i = 0; i < numTBlocks; i++)
    {
      minCost[i] = MAX_DOUBLE;
    }

    CodingStructure &saveCS = *m_pSaveCS[0];
    saveCS.pcv     = cs.pcv;
    saveCS.picture = cs.picture;
    saveCS.area.repositionTo(currArea);
    saveCS.clearTUs();

    TransformUnit &bestTU = saveCS.addTU( currArea, partitioner.chType );

#if ENABLE_CHROMA_422
    static const ComponentID c2comp[2][MAX_NUM_TBLOCKS] =
    {
      { COMPONENT_Y, COMPONENT_Cb, COMPONENT_Cr,  MAX_NUM_TBLOCKS, MAX_NUM_TBLOCKS },
      { COMPONENT_Y, COMPONENT_Cb, COMPONENT_Cb2, COMPONENT_Cr,    COMPONENT_Cr2   }
    };
    const ComponentID*  getComp     = c2comp[ numTBlocks >> 2 ];
#endif

    for( UInt c = 0; c < numTBlocks; c++ )
    {
#if ENABLE_CHROMA_422
      const ComponentID compID    = getComp[c];
#else
      const ComponentID compID    = ComponentID(c);
#endif
      const CompArea&   compArea  = tu.blocks[compID];
      const Int channelBitDepth   = sps.getBitDepth(toChannelType(compID));

      checkTransformSkip[compID]  = false;

      if( !tu.blocks[compID].valid() )
      {
        continue;
      }

      checkTransformSkip[compID] = pps.getUseTransformSkip() && TU::hasTransformSkipFlag( *tu.cs, tu.blocks[compID] ) && !cs.isLossless;

      const Bool isCrossCPredictionAvailable = TU::hasCrossCompPredInfo( tu, compID );

      SChar preCalcAlpha = 0;
      const CPelBuf lumaResi = csFull->getResiBuf(tu.Y());

      if (isCrossCPredictionAvailable)
      {
        csFull->getResiBuf( compArea ).copyFrom( cs.getOrgResiBuf( compArea ) );
        preCalcAlpha = xCalcCrossComponentPredictionAlpha( tu, compID, m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate() );
      }

      const Int crossCPredictionModesToTest = preCalcAlpha != 0 ? 2 : 1;
      const int numTransformCandidates      = checkTransformSkip[compID] ? 2 : 1;
      int lastTransformModeIndex            = numTransformCandidates - 1; //lastTransformModeIndex is the mode for transformSkip (if transformSkip is active)
      const Bool isOneMode                  = crossCPredictionModesToTest == 1 && numTransformCandidates == 1;

      Bool isLastBest = isOneMode;
      for( int transformMode = 0; transformMode < numTransformCandidates; transformMode++ )
      {
        for( Int crossCPredictionModeId = 0; crossCPredictionModeId < crossCPredictionModesToTest; crossCPredictionModeId++ )
        {
          const Bool isFirstMode  = transformMode == 0 && crossCPredictionModeId == 0;
          const Bool isLastMode   = ( transformMode + 1 ) == numTransformCandidates && ( crossCPredictionModeId + 1 ) == crossCPredictionModesToTest;
          const Bool bUseCrossCPrediction = crossCPredictionModeId != 0;

          // copy the original residual into the residual buffer
          csFull->getResiBuf(compArea).copyFrom(cs.getOrgResiBuf(compArea));

          m_CABACEstimator->getCtx() = ctxStart;
          m_CABACEstimator->resetBits();

          tu.transformSkip[compID]  = checkTransformSkip[compID] && transformMode == lastTransformModeIndex;
          tu.compAlpha[compID]      = bUseCrossCPrediction ? preCalcAlpha : 0;

          const QpParam cQP(tu, compID);  // note: uses tu.transformSkip[compID]

#if RDOQ_CHROMA_LAMBDA
          m_pcTrQuant->selectLambda(compID);
#endif

          TCoeff     currAbsSum = 0;
          uint64_t   currCompFracBits = 0;
          Distortion currCompDist = 0;
          Double     currCompCost = 0;
          uint64_t   nonCoeffFracBits = 0;
          Distortion nonCoeffDist = 0;
          Double     nonCoeffCost = 0;

          if (bUseCrossCPrediction)
          {
            PelBuf resiBuf = csFull->getResiBuf( compArea );
            crossComponentPrediction( tu, compID, lumaResi, resiBuf, resiBuf, false );
          }

          m_pcTrQuant->transformNxN(tu, compID, cQP, currAbsSum, m_CABACEstimator->getCtx());

          if (isFirstMode || (currAbsSum == 0))
          {
            const CPelBuf zeroBuf(m_pTempPel, compArea);
            const CPelBuf orgResi = csFull->getOrgResiBuf( compArea );

            if (bUseCrossCPrediction)
            {
              PelBuf resi = csFull->getResiBuf( compArea );
              crossComponentPrediction( tu, compID, lumaResi, zeroBuf, resi, true );
              nonCoeffDist = m_pcRdCost->getDistPart( orgResi, resi, channelBitDepth, compID, DF_SSE );
            }
            else
            {
              nonCoeffDist = m_pcRdCost->getDistPart( zeroBuf, orgResi, channelBitDepth, compID, DF_SSE ); // initialized with zero residual distortion
            }

#if HEVC_USE_RQT || ENABLE_BMS
            m_CABACEstimator->cbf_comp( *csFull, false, compArea, currDepth );
#else
            m_CABACEstimator->cbf_comp( *csFull, false, compArea );
#endif

            if( isCrossCPredictionAvailable )
            {
              m_CABACEstimator->cross_comp_pred( tu, compID );
            }

            nonCoeffFracBits = m_CABACEstimator->getEstFracBits();
#if WCG_EXT
            if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
            {
              nonCoeffCost   = m_pcRdCost->calcRdCost(nonCoeffFracBits, nonCoeffDist, false);
            }
            else
#endif
            nonCoeffCost     = m_pcRdCost->calcRdCost(nonCoeffFracBits, nonCoeffDist);
          }

          if ((puiZeroDist != NULL) && isFirstMode)
          {
            *puiZeroDist += nonCoeffDist; // initialized with zero residual distortion
          }

          if (currAbsSum > 0) //if non-zero coefficients are present, a residual needs to be derived for further prediction
          {
            if (isFirstMode)
            {
              m_CABACEstimator->getCtx() = ctxStart;
              m_CABACEstimator->resetBits();
            }
#if HEVC_USE_RQT || ENABLE_BMS
            m_CABACEstimator->cbf_comp( *csFull, true, compArea, currDepth );
#else
            m_CABACEstimator->cbf_comp( *csFull, true, compArea );
#endif
            if( isCrossCPredictionAvailable )
            {
              m_CABACEstimator->cross_comp_pred( tu, compID );
            }
            m_CABACEstimator->residual_coding( tu, compID );

            currCompFracBits = m_CABACEstimator->getEstFracBits();

            PelBuf resiBuf     = csFull->getResiBuf(compArea);
            CPelBuf orgResiBuf = csFull->getOrgResiBuf(compArea);

            m_pcTrQuant->invTransformNxN(tu, compID, resiBuf, cQP);

            if (bUseCrossCPrediction)
            {
              crossComponentPrediction( tu, compID, lumaResi, resiBuf, resiBuf, true );
            }

            currCompDist = m_pcRdCost->getDistPart(orgResiBuf, resiBuf, channelBitDepth, compID, DF_SSE);
            
#if WCG_EXT
            currCompCost = m_pcRdCost->calcRdCost(currCompFracBits, currCompDist, false);
#else
            currCompCost = m_pcRdCost->calcRdCost(currCompFracBits, currCompDist);
#endif

            if (csFull->isLossless)
            {
              nonCoeffCost = MAX_DOUBLE;
            }
          }
          else if( ( transformMode == lastTransformModeIndex ) && checkTransformSkip[compID] && !bUseCrossCPrediction )
          {
            currCompCost = MAX_DOUBLE;
          }
          else
          {
            currCompFracBits = nonCoeffFracBits;
            currCompDist     = nonCoeffDist;
            currCompCost     = nonCoeffCost;

            tu.cbf[compID] = 0;
          }

          // evaluate
          if( ( currCompCost < minCost[compID] ) || ( transformMode == lastTransformModeIndex && checkTransformSkip[compID] && currCompCost == minCost[compID] ) )
          {
            // copy component
            if (isFirstMode && ((nonCoeffCost < currCompCost) || (currAbsSum == 0))) // check for forced null
            {
              tu.getCoeffs( compID ).fill( 0 );
              csFull->getResiBuf( compArea ).fill( 0 );
              tu.cbf[compID]   = 0;

              currAbsSum       = 0;
              currCompFracBits = nonCoeffFracBits;
              currCompDist     = nonCoeffDist;
              currCompCost     = nonCoeffCost;
            }

            uiAbsSum[compID]         = currAbsSum;
            uiSingleDistComp[compID] = currCompDist;
            minCost[compID]          = currCompCost;

            if (uiAbsSum[compID] == 0)
            {
              if (bUseCrossCPrediction)
              {
                const CPelBuf zeroBuf( m_pTempPel, compArea );
                PelBuf resiBuf = csFull->getResiBuf( compArea );

                crossComponentPrediction( tu, compID, lumaResi, zeroBuf, resiBuf, true );
              }
            }

            if( !isLastMode )
            {
              bestTU.copyComponentFrom( tu, compID );
              saveCS.getResiBuf( compArea ).copyFrom( csFull->getResiBuf( compArea ) );
            }

            isLastBest = isLastMode;
          }
        }
      }

      if( !isLastBest )
      {
        // copy component
        tu.copyComponentFrom( bestTU, compID );
        csFull->getResiBuf( compArea ).copyFrom( saveCS.getResiBuf( compArea ) );
      }
    } // component loop

    m_CABACEstimator->getCtx() = ctxStart;
    m_CABACEstimator->resetBits();

#if HEVC_USE_RQT
    if( !cs.pcv->noRQT )
    {
      if (uiLog2TrSize > CU::getQuadtreeTULog2MinSizeInCU(cu))
      {
        if( sps.getSpsNext().nextToolsEnabled() )
        {
          m_CABACEstimator->split_transform_flag( false, sps.getQuadtreeTULog2MaxSize() - uiLog2TrSize );
        }
        else
        {
          m_CABACEstimator->split_transform_flag( false, 5 - uiLog2TrSize );
        }
      }
    }

#endif
#if ENABLE_CHROMA_422
    static const ComponentID cbf_c2comp[2][5] =
    {
      { COMPONENT_Cb, COMPONENT_Cr,  COMPONENT_Y,  MAX_NUM_TBLOCKS, MAX_NUM_TBLOCKS },
      { COMPONENT_Cb, COMPONENT_Cb2, COMPONENT_Cr, COMPONENT_Cr2,   COMPONENT_Y     }
    };
    const ComponentID*       cbf_getComp      = cbf_c2comp[ numTBlocks >> 2 ];
#else
    static const ComponentID cbf_getComp[3] = { COMPONENT_Cb, COMPONENT_Cr, COMPONENT_Y };
#endif
    for( unsigned c = 0; c < numTBlocks; c++)
    {
      const ComponentID compID = cbf_getComp[c];
      if( tu.blocks[compID].valid() )
      {
#if HEVC_USE_RQT || ENABLE_BMS
        m_CABACEstimator->cbf_comp( *csFull, TU::getCbfAtDepth( tu, compID, currDepth ), tu.blocks[compID], currDepth );
#else
        m_CABACEstimator->cbf_comp( *csFull, TU::getCbf( tu, compID ), tu.blocks[compID] );
#endif
      }
    }

    for (UInt ch = 0; ch < numValidComp; ch++)
    {
      const ComponentID compID = ComponentID(ch);

      if (tu.blocks[compID].valid())
      {
        if( cs.pps->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && isChroma(compID) && uiAbsSum[COMPONENT_Y] )
        {
          m_CABACEstimator->cross_comp_pred( tu, compID );
        }
        if( TU::getCbf( tu, compID ) )
        {
          m_CABACEstimator->residual_coding( tu, compID );
        }
        uiSingleDist += uiSingleDistComp[compID];
#if ENABLE_CHROMA_422
        if( cs.pcv->multiBlock422 && ch != COMPONENT_Y )
        {
          ComponentID compID2 = ComponentID( compID + SCND_TBLOCK_OFFSET );
          if( TU::getCbf( tu, compID2 ) )
          {
            m_CABACEstimator->residual_coding( tu, compID2 );
          }
          uiSingleDist += uiSingleDistComp[compID2];
        }
#endif
      }
    }

    csFull->fracBits += m_CABACEstimator->getEstFracBits();
    csFull->dist     += uiSingleDist;
#if WCG_EXT
    if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
    {
      csFull->cost    = m_pcRdCost->calcRdCost(csFull->fracBits, csFull->dist, false);
    }
    else
#endif
    csFull->cost      = m_pcRdCost->calcRdCost(csFull->fracBits, csFull->dist);
#if HEVC_USE_RQT
    if (bCheckSplit)
    {
      ctxBest = m_CABACEstimator->getCtx();
    }
#endif
  } // check full
#if HEVC_USE_RQT || ENABLE_BMS

  // code sub-blocks
  if( bCheckSplit )
  {
    if( bCheckFull )
    {
      m_CABACEstimator->getCtx() = ctxStart;
    }

#if ENABLE_BMS
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else
#endif
#if HEVC_USE_RQT
#if HM_REPRODUCE_4x4_BLOCK_ESTIMATION_ORDER
    partitioner.splitCurrArea( TU_QUAD_SPLIT_HM, cs );
#else
    partitioner.splitCurrArea( TU_QUAD_SPLIT, cs );
#endif
#else
      THROW( "Implicit TU split not available!" );
#endif

    do
    {
      xEstimateInterResidualQT(*csSplit, partitioner, bCheckFull ? nullptr : puiZeroDist);

      csSplit->cost = m_pcRdCost->calcRdCost( csSplit->fracBits, csSplit->dist );
    } while( partitioner.nextPart( *csSplit ) );

    partitioner.exitCurrSplit();

    unsigned        anyCbfSet   =   0;
#if ENABLE_CHROMA_422
    unsigned        compCbf[5]  = { 0, 0, 0, 0, 0 };
    const bool      is_NxN_422  = ( cs.pcv->multiBlock422 && currArea.lumaSize().width == 8 );
#else
    unsigned        compCbf[3]  = { 0, 0, 0 };
#endif

    if( !bCheckFull )
    {
      for( auto &currTU : csSplit->traverseTUs( currArea, partitioner.chType ) )
      {
        for( unsigned ch = 0; ch < numTBlocks; ch++ )
        {
          compCbf[ ch ] |= ( TU::getCbfAtDepth( currTU, ComponentID(ch), currDepth + 1 ) ? 1 : 0 );
        }
      }

#if ENABLE_CHROMA_422
      if( is_NxN_422 ) // very special case
      {
        for( auto &currTU : csSplit->traverseTUs( currArea, partitioner.chType ) )
        {
          TU::setCbfAtDepth ( currTU, COMPONENT_Y,   currDepth, compCbf[ COMPONENT_Y  ] );
          TU::setCbfAtDepth ( currTU, COMPONENT_Cb,  currDepth, compCbf[ COMPONENT_Cb ] );
          TU::setCbfAtDepth ( currTU, COMPONENT_Cr,  currDepth, compCbf[ COMPONENT_Cr ] );
          TU::setCbfAtDepth ( currTU, COMPONENT_Cb2, currDepth, compCbf[ COMPONENT_Cb2] );
          TU::setCbfAtDepth ( currTU, COMPONENT_Cr2, currDepth, compCbf[ COMPONENT_Cr2] );
        }

        anyCbfSet  = compCbf[ COMPONENT_Y  ];
        anyCbfSet |= compCbf[ COMPONENT_Cb ];
        anyCbfSet |= compCbf[ COMPONENT_Cr ];
        anyCbfSet |= compCbf[ COMPONENT_Cb2];
        anyCbfSet |= compCbf[ COMPONENT_Cr2];
      }
      else // usual case
#endif
      {
#if ENABLE_CHROMA_422
        if( cs.pcv->multiBlock422 )
        {
          compCbf[ COMPONENT_Cb ] |= compCbf[ COMPONENT_Cb2 ];
          compCbf[ COMPONENT_Cr ] |= compCbf[ COMPONENT_Cr2 ];
        }
#endif

        for( auto &currTU : csSplit->traverseTUs( currArea, partitioner.chType ) )
        {
          TU::setCbfAtDepth   ( currTU, COMPONENT_Y,  currDepth, compCbf[ COMPONENT_Y  ] );
          if( currArea.chromaFormat != CHROMA_400 )
          {
            TU::setCbfAtDepth ( currTU, COMPONENT_Cb, currDepth, compCbf[ COMPONENT_Cb ] );
            TU::setCbfAtDepth ( currTU, COMPONENT_Cr, currDepth, compCbf[ COMPONENT_Cr ] );
          }
        }

        anyCbfSet    = compCbf[ COMPONENT_Y  ];
        if( currArea.chromaFormat != CHROMA_400 )
        {
          anyCbfSet |= compCbf[ COMPONENT_Cb ];
          anyCbfSet |= compCbf[ COMPONENT_Cr ];
        }
      }

      m_CABACEstimator->getCtx() = ctxStart;
      m_CABACEstimator->resetBits();

      // when compID isn't a channel, code Cbfs:
      xEncodeInterResidualQT( *csSplit, partitioner, MAX_NUM_TBLOCKS );
      for (UInt ch = 0; ch < numValidComp; ch++)
      {
        xEncodeInterResidualQT( *csSplit, partitioner, ComponentID( ch ) );
      }

      csSplit->fracBits = m_CABACEstimator->getEstFracBits();
      csSplit->cost     = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);

      if( bCheckFull && anyCbfSet && csSplit->cost < csFull->cost )
      {
        cs.useSubStructure( *csSplit, partitioner.chType, currArea, false, false, false, true );
        cs.cost = csSplit->cost;
      }
    }

    if( !( !bCheckFull || ( anyCbfSet && csSplit->cost < csFull->cost ) ) )
    {
      CHECK( !bCheckFull, "Error!" );
      cs.useSubStructure( *csFull, partitioner.chType, currArea, false, false, false, true );
      cs.cost = csFull->cost;
      m_CABACEstimator->getCtx() = ctxBest;
    }

    if( csSplit && csFull )
    {
      csSplit->releaseIntermediateData();
      csFull ->releaseIntermediateData();
    }
  }
#endif
}

Void InterSearch::encodeResAndCalcRdInterCU(CodingStructure &cs, Partitioner &partitioner, const Bool &skipResidual)
{
  CodingUnit &cu = *cs.getCU( partitioner.chType );

  const ChromaFormat format     = cs.area.chromaFormat;;
  const Int  numValidComponents = getNumberValidComponents(format);
  const SPS &sps                = *cs.sps;
  const PPS &pps                = *cs.pps;

  if( skipResidual ) //  No residual coding : SKIP mode
  {
    cu.skip    = true;
    cu.rootCbf = false;
    cs.getResiBuf().fill(0);
    {
      cs.getRecoBuf().copyFrom(cs.getPredBuf() );
    }


    // add an empty TU
    cs.addTU(cs.area, partitioner.chType);

    Distortion distortion = 0;

    for (Int comp = 0; comp < numValidComponents; comp++)
    {
      const ComponentID compID = ComponentID(comp);

      CPelBuf reco = cs.getRecoBuf (compID);
      CPelBuf org  = cs.getOrgBuf  (compID);
#if WCG_EXT
      if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
      {
        const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMPONENT_Y] );
        distortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
      }
      else
#endif
      distortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
    }

    m_CABACEstimator->resetBits();

    if( pps.getTransquantBypassEnabledFlag() )
    {
      m_CABACEstimator->cu_transquant_bypass_flag( cu );
    }

    PredictionUnit &pu = *cs.getPU( partitioner.chType );

    m_CABACEstimator->cu_skip_flag  ( cu );
    m_CABACEstimator->merge_idx     ( pu );


    cs.dist     = distortion;
    cs.fracBits = m_CABACEstimator->getEstFracBits();
    cs.cost     = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);

    return;
  }

  //  Residual coding.
  cs.getResiBuf().copyFrom (cs.getOrgBuf());
  cs.getResiBuf().subtract (cs.getPredBuf());

  Distortion zeroDistortion = 0;

  const TempCtx ctxStart( m_CtxCache, m_CABACEstimator->getCtx() );

  cs.getOrgResiBuf().copyFrom(cs.getResiBuf());

  xEstimateInterResidualQT(cs, partitioner, &zeroDistortion);

  TransformUnit &firstTU = *cs.getTU( partitioner.chType );

  cu.rootCbf = false;
  m_CABACEstimator->resetBits();
  m_CABACEstimator->rqt_root_cbf( cu );
  const uint64_t  zeroFracBits = m_CABACEstimator->getEstFracBits();
  Double zeroCost;
  {
#if WCG_EXT
    if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
    {
      zeroCost = cs.isLossless ? ( cs.cost + 1 ) : m_pcRdCost->calcRdCost( zeroFracBits, zeroDistortion, false );
    }
    else
#endif
    zeroCost = cs.isLossless ? ( cs.cost + 1 ) : m_pcRdCost->calcRdCost( zeroFracBits, zeroDistortion );
  }

  const Int  numValidTBlocks   = ::getNumberValidTBlocks( *cs.pcv );
  for (UInt i = 0; i < numValidTBlocks; i++)
  {
#if HEVC_USE_RQT
    cu.rootCbf |= TU::getCbfAtDepth( firstTU, ComponentID( i ), 0 );
#else
    cu.rootCbf |= TU::getCbf( firstTU, ComponentID( i ) );
#endif
  }

  // -------------------------------------------------------
  // If a block full of 0's is efficient, then just use 0's.
  // The costs at this point do not include header bits.

  if (zeroCost < cs.cost || !cu.rootCbf)
  {
    cu.rootCbf = false;

    cs.clearTUs();

    // add a new "empty" TU spanning the whole CU
    TransformUnit& tu = cs.addTU(cu, partitioner.chType);

    for (Int comp = 0; comp < numValidComponents; comp++)
    {
      tu.rdpcm[comp] = RDPCM_OFF;
    }
    cu.firstTU = cu.lastTU = &tu;
  }

#if HM_REPRODUCE_4x4_BLOCK_ESTIMATION_ORDER
  xChangeInterTUsFromSearchToCode( cs, partitioner );
#endif

  // all decisions now made. Fully encode the CU, including the headers:
  m_CABACEstimator->getCtx() = ctxStart;

  UInt64 finalFracBits = xGetSymbolFracBitsInter( cs, partitioner );
  // we've now encoded the CU, and so have a valid bit cost
  if (!cu.rootCbf)
  {
    cs.getResiBuf().fill(0); // Clear the residual image, if we didn't code it.
  }

  cs.getRecoBuf().reconstruct(cs.getPredBuf(), cs.getResiBuf(), cs.slice->clpRngs());

  // update with clipped distortion and cost (previously unclipped reconstruction values were used)
  Distortion finalDistortion = 0;

  for (Int comp = 0; comp < numValidComponents; comp++)
  {
    const ComponentID compID = ComponentID(comp);

    CPelBuf reco = cs.getRecoBuf (compID);
    CPelBuf org  = cs.getOrgBuf  (compID);

#if WCG_EXT
    if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
    {
      const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMPONENT_Y] );
      finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
    }
    else
#endif
    {
      finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
    }
  }

  cs.dist     = finalDistortion;
  cs.fracBits = finalFracBits;
  cs.cost     = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);

  CHECK(cs.tus.size() == 0, "No TUs present");
}

UInt64 InterSearch::xGetSymbolFracBitsInter(CodingStructure &cs, Partitioner &partitioner)
{
  UInt64 fracBits   = 0;
  CodingUnit &cu    = *cs.getCU( partitioner.chType );

  m_CABACEstimator->resetBits();

  if( cu.partSize == SIZE_2Nx2N && cu.firstPU->mergeFlag && !cu.rootCbf )
  {
    cu.skip = true;

    if( cs.pps->getTransquantBypassEnabledFlag() )
    {
      m_CABACEstimator->cu_transquant_bypass_flag( cu );
    }

    m_CABACEstimator->cu_skip_flag  ( cu );
    m_CABACEstimator->merge_idx     ( *cu.firstPU );
    fracBits   += m_CABACEstimator->getEstFracBits();
  }
  else
  {
    CHECK( cu.skip, "Skip flag has to be off at this point!" );

    if( cs.pps->getTransquantBypassEnabledFlag() )
    {
      m_CABACEstimator->cu_transquant_bypass_flag( cu );
    }

    m_CABACEstimator->cu_skip_flag( cu );
    m_CABACEstimator->pred_mode   ( cu );
#if HEVC_USE_PART_SIZE
    m_CABACEstimator->part_mode   ( cu );
#endif
    m_CABACEstimator->cu_pred_data( cu );
    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->cu_residual ( cu, partitioner, cuCtx );
    fracBits       += m_CABACEstimator->getEstFracBits();
  }

  return fracBits;
}

#if HM_REPRODUCE_4x4_BLOCK_ESTIMATION_ORDER
Void InterSearch::xChangeInterTUsFromSearchToCode( CodingStructure &cs, Partitioner &partitioner )
{
  const TransformUnit &currTU = *cs.getTU( partitioner.currArea().lumaPos(), partitioner.chType );

  const Bool bSubdiv = partitioner.currTrDepth != currTU.depth;

  if( !bSubdiv )  // we are not processing a channel, instead we always recurse and code the CBFs
  {
  }
  else
  {
    const auto subTUs = PartitionerImpl::getTUSubPartitions( partitioner.currArea(), cs );

    // move the chroma sub-blocks from the first sub TU to the last
    if( !TU::isProcessingAllQuadrants( subTUs[0] ) )
    {
      TransformUnit &firstTU = *cs.getTU( subTUs[0].lumaPos(), partitioner.chType );
      TransformUnit &lastTU  = *cs.getTU( subTUs[3].lumaPos(), partitioner.chType );

#if ENABLE_CHROMA_422
      TCoeff *coefff[2][MAX_NUM_TBLOCKS] = { { firstTU.getCoeffs( COMPONENT_Y ).buf, nullptr, nullptr, nullptr, nullptr }, { lastTU.getCoeffs( COMPONENT_Y ).buf, nullptr, nullptr, nullptr, nullptr } };
      Pel    *pcmbuf[2][MAX_NUM_TBLOCKS] = { { firstTU.getPcmbuf( COMPONENT_Y ).buf, nullptr, nullptr, nullptr, nullptr }, { lastTU.getPcmbuf( COMPONENT_Y ).buf, nullptr, nullptr, nullptr, nullptr } };
#else
      TCoeff *coefff[2][MAX_NUM_TBLOCKS] = { { firstTU.getCoeffs( COMPONENT_Y ).buf, nullptr, nullptr }, { lastTU.getCoeffs( COMPONENT_Y ).buf, nullptr, nullptr } };
      Pel    *pcmbuf[2][MAX_NUM_TBLOCKS] = { { firstTU.getPcmbuf( COMPONENT_Y ).buf, nullptr, nullptr }, { lastTU.getPcmbuf( COMPONENT_Y ).buf, nullptr, nullptr } };
#endif

      UInt maxNumTBlocks = getNumberValidTBlocks( *cs.pcv );

      for( UInt i = 1; i < maxNumTBlocks; i++ )
      {
        ComponentID compID = ComponentID( i );

        if( !firstTU.blocks[i].valid() ) continue;

        coefff[1][i] = firstTU.getCoeffs( compID ).buf;
        pcmbuf[1][i] = firstTU.getPcmbuf( compID ).buf;

        lastTU.blocks[i] = firstTU.blocks[i];
      }

      lastTU.init( coefff[1], pcmbuf[1] );

      for( UInt i = 1; i < maxNumTBlocks; i++ )
      {
        if( !firstTU.blocks[i].valid() ) continue;

        lastTU.copyComponentFrom( firstTU, ComponentID( i ) );
        firstTU.blocks[i] = CompArea();
      }

      firstTU.init( coefff[0], pcmbuf[0] );
    }
    else
    {
      partitioner.splitCurrArea( TU_QUAD_SPLIT, cs );

      do
      {
        xChangeInterTUsFromSearchToCode( cs, partitioner );
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
    }
  }
}
#endif

