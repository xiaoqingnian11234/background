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

/** \file     EncGOP.cpp
    \brief    GOP encoder class
*/

#include <list>
#include <algorithm>
#include <functional>

#include "EncLib.h"
#include "EncGOP.h"
#include "Analyze.h"
#include "libmd5/MD5.h"
#include "CommonLib/SEI.h"
#include "CommonLib/NAL.h"
#include "NALwrite.h"

#include <math.h>
#include <deque>
#include <chrono>

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"

#include "DecoderLib/DecLib.h"

#define ENCODE_SUB_SET 0

#if PRINT_UPDATEBG_RESI || PRINT_UPDATE_TRCOEFF || PRINT_OrgDIFF
#include <fstream>
#endif
#if GENERATE_BG_PIC
#include <iomanip>
#endif

using namespace std;

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================
Int getLSB(Int poc, Int maxLSB)
{
  if (poc >= 0)
  {
    return poc % maxLSB;
  }
  else
  {
    return (maxLSB - ((-poc) % maxLSB)) % maxLSB;
  }
}


EncGOP::EncGOP()
{
  m_iLastIDR            = 0;
  m_iGopSize            = 0;
  m_iNumPicCoded        = 0; //Niko
  m_bFirst              = true;
  m_iLastRecoveryPicPOC = 0;
  m_lastRasPoc          = MAX_INT;

  m_pcCfg               = NULL;
  m_pcSliceEncoder      = NULL;
  m_pcListPic           = NULL;
  m_HLSWriter           = NULL;
  m_bSeqFirst           = true;

  m_bRefreshPending     = 0;
  m_pocCRA              = 0;
  m_numLongTermRefPicSPS = 0;
  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_ltRefPicUsedByCurrPicFlag, 0, sizeof(m_ltRefPicUsedByCurrPicFlag));
  m_lastBPSEI           = 0;
  m_bufferingPeriodSEIPresentInAU = false;
  m_associatedIRAPType  = NAL_UNIT_CODED_SLICE_IDR_N_LP;
  m_associatedIRAPPOC   = 0;
#if W0038_DB_OPT
  m_pcDeblockingTempPicYuv = NULL;
#endif

  m_bInitAMaxBT         = true;
}

EncGOP::~EncGOP()
{
  if( !m_pcCfg->getDecodeBitstream(0).empty() || !m_pcCfg->getDecodeBitstream(1).empty() )
  {
    // reset potential decoder resources
    tryDecodePicture( NULL, 0, std::string("") );
  }
}

/** Create list to contain pointers to CTU start addresses of slice.
 */
Void  EncGOP::create()
{
  m_bLongtermTestPictureHasBeenCoded = 0;
  m_bLongtermTestPictureHasBeenCoded2 = 0;
}

Void  EncGOP::destroy()
{
#if W0038_DB_OPT
  if (m_pcDeblockingTempPicYuv)
  {
    m_pcDeblockingTempPicYuv->destroy();
    delete m_pcDeblockingTempPicYuv;
    m_pcDeblockingTempPicYuv = NULL;
  }
#endif
}

Void EncGOP::init ( EncLib* pcEncLib )
{
  m_pcEncLib     = pcEncLib;
  m_pcCfg                = pcEncLib;
  m_seiEncoder.init(m_pcCfg, pcEncLib, this);
  m_pcSliceEncoder       = pcEncLib->getSliceEncoder();
  m_pcListPic            = pcEncLib->getListPic();
  m_HLSWriter            = pcEncLib->getHLSWriter();
  m_pcLoopFilter         = pcEncLib->getLoopFilter();
  m_pcSAO                = pcEncLib->getSAO();
  m_pcRateCtrl           = pcEncLib->getRateCtrl();
  m_lastBPSEI          = 0;
  m_totalCoded         = 0;

  m_AUWriterIf = pcEncLib->getAUWriterIf();
  
#if WCG_EXT
  pcEncLib->getRdCost()->initLumaLevelToWeightTable();
#endif
}

#if HEVC_VPS
Int EncGOP::xWriteVPS (AccessUnit &accessUnit, const VPS *vps)
{
  OutputNALUnit nalu(NAL_UNIT_VPS);
  m_HLSWriter->setBitstream( &nalu.m_Bitstream );
  m_HLSWriter->codeVPS( vps );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (Int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}
#endif

Int EncGOP::xWriteSPS (AccessUnit &accessUnit, const SPS *sps)
{
  OutputNALUnit nalu(NAL_UNIT_SPS);
  m_HLSWriter->setBitstream( &nalu.m_Bitstream );
  m_HLSWriter->codeSPS( sps );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (Int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;

}

Int EncGOP::xWritePPS (AccessUnit &accessUnit, const PPS *pps)
{
  OutputNALUnit nalu(NAL_UNIT_PPS);
  m_HLSWriter->setBitstream( &nalu.m_Bitstream );
  m_HLSWriter->codePPS( pps );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (Int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}


Int EncGOP::xWriteParameterSets (AccessUnit &accessUnit, Slice *slice, const Bool bSeqFirst)
{
  Int actualTotalBits = 0;

#if HEVC_VPS
  if (bSeqFirst)
  {
    actualTotalBits += xWriteVPS(accessUnit, m_pcEncLib->getVPS());
  }
#endif
  if (m_pcEncLib->SPSNeedsWriting(slice->getSPS()->getSPSId())) // Note this assumes that all changes to the SPS are made at the EncLib level prior to picture creation (EncLib::xGetNewPicBuffer).
  {
    CHECK(!(bSeqFirst), "Unspecified error"); // Implementations that use more than 1 SPS need to be aware of activation issues.
    actualTotalBits += xWriteSPS(accessUnit, slice->getSPS());
  }
  if (m_pcEncLib->PPSNeedsWriting(slice->getPPS()->getPPSId())) // Note this assumes that all changes to the PPS are made at the EncLib level prior to picture creation (EncLib::xGetNewPicBuffer).
  {
    actualTotalBits += xWritePPS(accessUnit, slice->getPPS());
  }

  return actualTotalBits;
}

Void EncGOP::xWriteAccessUnitDelimiter (AccessUnit &accessUnit, Slice *slice)
{
  AUDWriter audWriter;
  OutputNALUnit nalu(NAL_UNIT_ACCESS_UNIT_DELIMITER);

  Int picType = slice->isIntra() ? 0 : (slice->isInterP() ? 1 : 2);

  audWriter.codeAUD(nalu.m_Bitstream, picType);
  accessUnit.push_front(new NALUnitEBSP(nalu));
}

// write SEI list into one NAL unit and add it to the Access unit at auPos
Void EncGOP::xWriteSEI (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, Int temporalId, const SPS *sps)
{
  // don't do anything, if we get an empty list
  if (seiMessages.empty())
  {
    return;
  }
  OutputNALUnit nalu(naluType, temporalId);
  m_seiWriter.writeSEImessages(nalu.m_Bitstream, seiMessages, sps, false);
  auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));
  auPos++;
}

Void EncGOP::xWriteSEISeparately (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, Int temporalId, const SPS *sps)
{
  // don't do anything, if we get an empty list
  if (seiMessages.empty())
  {
    return;
  }
  for (SEIMessages::const_iterator sei = seiMessages.begin(); sei!=seiMessages.end(); sei++ )
  {
    SEIMessages tmpMessages;
    tmpMessages.push_back(*sei);
    OutputNALUnit nalu(naluType, temporalId);
    m_seiWriter.writeSEImessages(nalu.m_Bitstream, tmpMessages, sps, false);
    auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));
    auPos++;
  }
}

Void EncGOP::xClearSEIs(SEIMessages& seiMessages, Bool deleteMessages)
{
  if (deleteMessages)
  {
    deleteSEIs(seiMessages);
  }
  else
  {
    seiMessages.clear();
  }
}

// write SEI messages as separate NAL units ordered
Void EncGOP::xWriteLeadingSEIOrdered (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const SPS *sps, Bool testWrite)
{
  AccessUnit::iterator itNalu = accessUnit.begin();

  while ( (itNalu!=accessUnit.end())&&
    ( (*itNalu)->m_nalUnitType==NAL_UNIT_ACCESS_UNIT_DELIMITER
#if HEVC_VPS
    || (*itNalu)->m_nalUnitType==NAL_UNIT_VPS
#endif
    || (*itNalu)->m_nalUnitType==NAL_UNIT_SPS
    || (*itNalu)->m_nalUnitType==NAL_UNIT_PPS
    ))
  {
    itNalu++;
  }

  SEIMessages localMessages = seiMessages;
  SEIMessages currentMessages;

#if ENABLE_TRACING
  g_HLSTraceEnable = !testWrite;
#endif
  // The case that a specific SEI is not present is handled in xWriteSEI (empty list)

  // Active parameter sets SEI must always be the first SEI
  currentMessages = extractSeisByType(localMessages, SEI::ACTIVE_PARAMETER_SETS);
  CHECK(!(currentMessages.size() <= 1), "Unspecified error");
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(currentMessages, !testWrite);

  // Buffering period SEI must always be following active parameter sets
  currentMessages = extractSeisByType(localMessages, SEI::BUFFERING_PERIOD);
  CHECK(!(currentMessages.size() <= 1), "Unspecified error");
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(currentMessages, !testWrite);

  // Picture timing SEI must always be following buffering period
  currentMessages = extractSeisByType(localMessages, SEI::PICTURE_TIMING);
  CHECK(!(currentMessages.size() <= 1), "Unspecified error");
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(currentMessages, !testWrite);

  // Decoding unit info SEI must always be following picture timing
  if (!duInfoSeiMessages.empty())
  {
    currentMessages.push_back(duInfoSeiMessages.front());
    if (!testWrite)
    {
      duInfoSeiMessages.pop_front();
    }
    xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
    xClearSEIs(currentMessages, !testWrite);
  }

  // Scalable nesting SEI must always be the following DU info
  currentMessages = extractSeisByType(localMessages, SEI::SCALABLE_NESTING);
  xWriteSEISeparately(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(currentMessages, !testWrite);

  // And finally everything else one by one
  xWriteSEISeparately(NAL_UNIT_PREFIX_SEI, localMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(localMessages, !testWrite);

  if (!testWrite)
  {
    seiMessages.clear();
  }
}


Void EncGOP::xWriteLeadingSEIMessages (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const SPS *sps, std::deque<DUData> &duData)
{
  AccessUnit testAU;
  SEIMessages picTimingSEIs = getSeisByType(seiMessages, SEI::PICTURE_TIMING);
  CHECK(!(picTimingSEIs.size() < 2), "Unspecified error");
  SEIPictureTiming * picTiming = picTimingSEIs.empty() ? NULL : (SEIPictureTiming*) picTimingSEIs.front();

  // test writing
  xWriteLeadingSEIOrdered(seiMessages, duInfoSeiMessages, testAU, temporalId, sps, true);
  // update Timing and DU info SEI
  xUpdateDuData(testAU, duData);
  xUpdateTimingSEI(picTiming, duData, sps);
  xUpdateDuInfoSEI(duInfoSeiMessages, picTiming);
  // actual writing
  xWriteLeadingSEIOrdered(seiMessages, duInfoSeiMessages, accessUnit, temporalId, sps, false);

  // testAU will automatically be cleaned up when losing scope
}

Void EncGOP::xWriteTrailingSEIMessages (SEIMessages& seiMessages, AccessUnit &accessUnit, Int temporalId, const SPS *sps)
{
  // Note: using accessUnit.end() works only as long as this function is called after slice coding and before EOS/EOB NAL units
  AccessUnit::iterator pos = accessUnit.end();
  xWriteSEISeparately(NAL_UNIT_SUFFIX_SEI, seiMessages, accessUnit, pos, temporalId, sps);
  deleteSEIs(seiMessages);
}

Void EncGOP::xWriteDuSEIMessages (SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const SPS *sps, std::deque<DUData> &duData)
{
  const HRD *hrd = sps->getVuiParameters()->getHrdParameters();

  if( m_pcCfg->getDecodingUnitInfoSEIEnabled() && hrd->getSubPicCpbParamsPresentFlag() )
  {
    Int naluIdx = 0;
    AccessUnit::iterator nalu = accessUnit.begin();

    // skip over first DU, we have a DU info SEI there already
    while (naluIdx < duData[0].accumNalsDU && nalu!=accessUnit.end())
    {
      naluIdx++;
      nalu++;
    }

    SEIMessages::iterator duSEI = duInfoSeiMessages.begin();
    // loop over remaining DUs
    for (Int duIdx = 1; duIdx < duData.size(); duIdx++)
    {
      if (duSEI == duInfoSeiMessages.end())
      {
        // if the number of generated SEIs matches the number of DUs, this should not happen
        CHECK(!(false), "Unspecified error");
        return;
      }
      // write the next SEI
      SEIMessages tmpSEI;
      tmpSEI.push_back(*duSEI);
      xWriteSEI(NAL_UNIT_PREFIX_SEI, tmpSEI, accessUnit, nalu, temporalId, sps);
      // nalu points to the position after the SEI, so we have to increase the index as well
      naluIdx++;
      while ((naluIdx < duData[duIdx].accumNalsDU) && nalu!=accessUnit.end())
      {
        naluIdx++;
        nalu++;
      }
      duSEI++;
    }
  }
  deleteSEIs(duInfoSeiMessages);
}


Void EncGOP::xCreateIRAPLeadingSEIMessages (SEIMessages& seiMessages, const SPS *sps, const PPS *pps)
{
  OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);

  if(m_pcCfg->getActiveParameterSetsSEIEnabled())
  {
    SEIActiveParameterSets *sei = new SEIActiveParameterSets;
#if HEVC_VPS
    m_seiEncoder.initSEIActiveParameterSets (sei, m_pcCfg->getVPS(), sps);
#else
    m_seiEncoder.initSEIActiveParameterSets(sei, sps);
#endif
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getFramePackingArrangementSEIEnabled())
  {
    SEIFramePacking *sei = new SEIFramePacking;
    m_seiEncoder.initSEIFramePacking (sei, m_iNumPicCoded);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getSegmentedRectFramePackingArrangementSEIEnabled())
  {
    SEISegmentedRectFramePacking *sei = new SEISegmentedRectFramePacking;
    m_seiEncoder.initSEISegmentedRectFramePacking(sei);
    seiMessages.push_back(sei);
  }

  if (m_pcCfg->getDisplayOrientationSEIAngle())
  {
    SEIDisplayOrientation *sei = new SEIDisplayOrientation;
    m_seiEncoder.initSEIDisplayOrientation(sei);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getToneMappingInfoSEIEnabled())
  {
    SEIToneMappingInfo *sei = new SEIToneMappingInfo;
    m_seiEncoder.initSEIToneMappingInfo (sei);
    seiMessages.push_back(sei);
  }

#if HEVC_TILES_WPP
  if(m_pcCfg->getTMCTSSEIEnabled())
  {
    SEITempMotionConstrainedTileSets *sei = new SEITempMotionConstrainedTileSets;
    m_seiEncoder.initSEITempMotionConstrainedTileSets(sei, pps);
    seiMessages.push_back(sei);
  }
#endif

  if(m_pcCfg->getTimeCodeSEIEnabled())
  {
    SEITimeCode *seiTimeCode = new SEITimeCode;
    m_seiEncoder.initSEITimeCode(seiTimeCode);
    seiMessages.push_back(seiTimeCode);
  }

  if(m_pcCfg->getKneeSEIEnabled())
  {
    SEIKneeFunctionInfo *sei = new SEIKneeFunctionInfo;
    m_seiEncoder.initSEIKneeFunctionInfo(sei);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getMasteringDisplaySEI().colourVolumeSEIEnabled)
  {
    const SEIMasteringDisplay &seiCfg=m_pcCfg->getMasteringDisplaySEI();
    SEIMasteringDisplayColourVolume *sei = new SEIMasteringDisplayColourVolume;
    sei->values = seiCfg;
    seiMessages.push_back(sei);
  }
  if(m_pcCfg->getChromaResamplingFilterHintEnabled())
  {
    SEIChromaResamplingFilterHint *seiChromaResamplingFilterHint = new SEIChromaResamplingFilterHint;
    m_seiEncoder.initSEIChromaResamplingFilterHint(seiChromaResamplingFilterHint, m_pcCfg->getChromaResamplingHorFilterIdc(), m_pcCfg->getChromaResamplingVerFilterIdc());
    seiMessages.push_back(seiChromaResamplingFilterHint);
  }
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  if(m_pcCfg->getSEIAlternativeTransferCharacteristicsSEIEnable())
  {
    SEIAlternativeTransferCharacteristics *seiAlternativeTransferCharacteristics = new SEIAlternativeTransferCharacteristics;
    m_seiEncoder.initSEIAlternativeTransferCharacteristics(seiAlternativeTransferCharacteristics);
    seiMessages.push_back(seiAlternativeTransferCharacteristics);
  }
#endif
}

Void EncGOP::xCreatePerPictureSEIMessages (Int picInGOP, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, Slice *slice)
{
  if( ( m_pcCfg->getBufferingPeriodSEIEnabled() ) && ( slice->getSliceType() == I_SLICE ) &&
    ( slice->getSPS()->getVuiParametersPresentFlag() ) &&
    ( ( slice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
    || ( slice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
  {
    SEIBufferingPeriod *bufferingPeriodSEI = new SEIBufferingPeriod();
    m_seiEncoder.initSEIBufferingPeriod(bufferingPeriodSEI, slice);
    seiMessages.push_back(bufferingPeriodSEI);
    m_bufferingPeriodSEIPresentInAU = true;

    if (m_pcCfg->getScalableNestingSEIEnabled())
    {
      SEIBufferingPeriod *bufferingPeriodSEIcopy = new SEIBufferingPeriod();
      bufferingPeriodSEI->copyTo(*bufferingPeriodSEIcopy);
      nestedSeiMessages.push_back(bufferingPeriodSEIcopy);
    }
  }

  if (picInGOP ==0 && m_pcCfg->getSOPDescriptionSEIEnabled() ) // write SOP description SEI (if enabled) at the beginning of GOP
  {
    SEISOPDescription* sopDescriptionSEI = new SEISOPDescription();
    m_seiEncoder.initSEISOPDescription(sopDescriptionSEI, slice, picInGOP, m_iLastIDR, m_iGopSize);
    seiMessages.push_back(sopDescriptionSEI);
  }

  if( ( m_pcEncLib->getRecoveryPointSEIEnabled() ) && ( slice->getSliceType() == I_SLICE ) )
  {
    if( m_pcEncLib->getGradualDecodingRefreshInfoEnabled() && !slice->getRapPicFlag() )
    {
      // Gradual decoding refresh SEI
      SEIGradualDecodingRefreshInfo *gradualDecodingRefreshInfoSEI = new SEIGradualDecodingRefreshInfo();
      gradualDecodingRefreshInfoSEI->m_gdrForegroundFlag = true; // Indicating all "foreground"
      seiMessages.push_back(gradualDecodingRefreshInfoSEI);
    }
    // Recovery point SEI
    SEIRecoveryPoint *recoveryPointSEI = new SEIRecoveryPoint();
    m_seiEncoder.initSEIRecoveryPoint(recoveryPointSEI, slice);
    seiMessages.push_back(recoveryPointSEI);
  }
  if (m_pcCfg->getTemporalLevel0IndexSEIEnabled())
  {
    SEITemporalLevel0Index *temporalLevel0IndexSEI = new SEITemporalLevel0Index();
    m_seiEncoder.initTemporalLevel0IndexSEI(temporalLevel0IndexSEI, slice);
    seiMessages.push_back(temporalLevel0IndexSEI);
  }

  if( m_pcEncLib->getNoDisplaySEITLayer() && ( slice->getTLayer() >= m_pcEncLib->getNoDisplaySEITLayer() ) )
  {
    SEINoDisplay *seiNoDisplay = new SEINoDisplay;
    seiNoDisplay->m_noDisplay = true;
    seiMessages.push_back(seiNoDisplay);
  }

  // insert one Colour Remapping Info SEI for the picture (if the file exists)
  if (!m_pcCfg->getColourRemapInfoSEIFileRoot().empty())
  {
    SEIColourRemappingInfo *seiColourRemappingInfo = new SEIColourRemappingInfo();
    const Bool success = m_seiEncoder.initSEIColourRemappingInfo(seiColourRemappingInfo, slice->getPOC() );

    if(success)
    {
      seiMessages.push_back(seiColourRemappingInfo);
    }
    else
    {
      delete seiColourRemappingInfo;
    }
  }
}

Void EncGOP::xCreateScalableNestingSEI (SEIMessages& seiMessages, SEIMessages& nestedSeiMessages)
{
  SEIMessages tmpMessages;
  while (!nestedSeiMessages.empty())
  {
    SEI* sei=nestedSeiMessages.front();
    nestedSeiMessages.pop_front();
    tmpMessages.push_back(sei);
    SEIScalableNesting *nestingSEI = new SEIScalableNesting();
    m_seiEncoder.initSEIScalableNesting(nestingSEI, tmpMessages);
    seiMessages.push_back(nestingSEI);
    tmpMessages.clear();
  }
}

Void EncGOP::xCreatePictureTimingSEI  (Int IRAPGOPid, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, SEIMessages& duInfoSeiMessages, Slice *slice, Bool isField, std::deque<DUData> &duData)
{
  const VUI *vui = slice->getSPS()->getVuiParameters();
  const HRD *hrd = vui->getHrdParameters();

  // update decoding unit parameters
  if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
    ( slice->getSPS()->getVuiParametersPresentFlag() ) &&
    (  hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() ) )
  {
    Int picSptDpbOutputDuDelay = 0;
    SEIPictureTiming *pictureTimingSEI = new SEIPictureTiming();

    // DU parameters
    if( hrd->getSubPicCpbParamsPresentFlag() )
    {
      UInt numDU = (UInt) duData.size();
      pictureTimingSEI->m_numDecodingUnitsMinus1     = ( numDU - 1 );
      pictureTimingSEI->m_duCommonCpbRemovalDelayFlag = false;
      pictureTimingSEI->m_numNalusInDuMinus1.resize( numDU );
      pictureTimingSEI->m_duCpbRemovalDelayMinus1.resize( numDU );
    }
    pictureTimingSEI->m_auCpbRemovalDelay = std::min<Int>(std::max<Int>(1, m_totalCoded - m_lastBPSEI), static_cast<Int>(pow(2, static_cast<Double>(hrd->getCpbRemovalDelayLengthMinus1()+1)))); // Syntax element signalled as minus, hence the .
    pictureTimingSEI->m_picDpbOutputDelay = slice->getSPS()->getNumReorderPics(slice->getSPS()->getMaxTLayers()-1) + slice->getPOC() - m_totalCoded;
    if(m_pcCfg->getEfficientFieldIRAPEnabled() && IRAPGOPid > 0 && IRAPGOPid < m_iGopSize)
    {
      // if pictures have been swapped there is likely one more picture delay on their tid. Very rough approximation
      pictureTimingSEI->m_picDpbOutputDelay ++;
    }
    Int factor = hrd->getTickDivisorMinus2() + 2;
    pictureTimingSEI->m_picDpbOutputDuDelay = factor * pictureTimingSEI->m_picDpbOutputDelay;
    if( m_pcCfg->getDecodingUnitInfoSEIEnabled() )
    {
      picSptDpbOutputDuDelay = factor * pictureTimingSEI->m_picDpbOutputDelay;
    }
    if (m_bufferingPeriodSEIPresentInAU)
    {
      m_lastBPSEI = m_totalCoded;
    }

    if( hrd->getSubPicCpbParamsPresentFlag() )
    {
      Int i;
      UInt64 ui64Tmp;
      UInt uiPrev = 0;
      UInt numDU = ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 );
      std::vector<UInt> &rDuCpbRemovalDelayMinus1 = pictureTimingSEI->m_duCpbRemovalDelayMinus1;
      UInt maxDiff = ( hrd->getTickDivisorMinus2() + 2 ) - 1;

      for( i = 0; i < numDU; i ++ )
      {
        pictureTimingSEI->m_numNalusInDuMinus1[ i ]       = ( i == 0 ) ? ( duData[i].accumNalsDU - 1 ) : ( duData[i].accumNalsDU- duData[i-1].accumNalsDU - 1 );
      }

      if( numDU == 1 )
      {
        rDuCpbRemovalDelayMinus1[ 0 ] = 0; /* don't care */
      }
      else
      {
        rDuCpbRemovalDelayMinus1[ numDU - 1 ] = 0;/* by definition */
        UInt tmp = 0;
        UInt accum = 0;

        for( i = ( numDU - 2 ); i >= 0; i -- )
        {
          ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );
          if( (UInt)ui64Tmp > maxDiff )
          {
            tmp ++;
          }
        }
        uiPrev = 0;

        UInt flag = 0;
        for( i = ( numDU - 2 ); i >= 0; i -- )
        {
          flag = 0;
          ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );

          if( (UInt)ui64Tmp > maxDiff )
          {
            if(uiPrev >= maxDiff - tmp)
            {
              ui64Tmp = uiPrev + 1;
              flag = 1;
            }
            else                            ui64Tmp = maxDiff - tmp + 1;
          }
          rDuCpbRemovalDelayMinus1[ i ] = (UInt)ui64Tmp - uiPrev - 1;
          if( (Int)rDuCpbRemovalDelayMinus1[ i ] < 0 )
          {
            rDuCpbRemovalDelayMinus1[ i ] = 0;
          }
          else if (tmp > 0 && flag == 1)
          {
            tmp --;
          }
          accum += rDuCpbRemovalDelayMinus1[ i ] + 1;
          uiPrev = accum;
        }
      }
    }

    if( m_pcCfg->getPictureTimingSEIEnabled() )
    {
      pictureTimingSEI->m_picStruct = (isField && slice->getPic()->topField)? 1 : isField? 2 : 0;
      seiMessages.push_back(pictureTimingSEI);

      if ( m_pcCfg->getScalableNestingSEIEnabled() ) // put picture timing SEI into scalable nesting SEI
      {
        SEIPictureTiming *pictureTimingSEIcopy = new SEIPictureTiming();
        pictureTimingSEI->copyTo(*pictureTimingSEIcopy);
        nestedSeiMessages.push_back(pictureTimingSEIcopy);
      }
    }

    if( m_pcCfg->getDecodingUnitInfoSEIEnabled() && hrd->getSubPicCpbParamsPresentFlag() )
    {
      for( Int i = 0; i < ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 ); i ++ )
      {
        SEIDecodingUnitInfo *duInfoSEI = new SEIDecodingUnitInfo();
        duInfoSEI->m_decodingUnitIdx = i;
        duInfoSEI->m_duSptCpbRemovalDelay = pictureTimingSEI->m_duCpbRemovalDelayMinus1[i] + 1;
        duInfoSEI->m_dpbOutputDuDelayPresentFlag = false;
        duInfoSEI->m_picSptDpbOutputDuDelay = picSptDpbOutputDuDelay;

        duInfoSeiMessages.push_back(duInfoSEI);
      }
    }

    if( !m_pcCfg->getPictureTimingSEIEnabled() && pictureTimingSEI )
    {
      delete pictureTimingSEI;
    }
  }
}

Void EncGOP::xUpdateDuData(AccessUnit &testAU, std::deque<DUData> &duData)
{
  if (duData.empty())
  {
    return;
  }
  // fix first
  UInt numNalUnits = (UInt)testAU.size();
  UInt numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = testAU.begin(); it != testAU.end(); it++)
  {
    numRBSPBytes += UInt((*it)->m_nalUnitData.str().size());
  }
  duData[0].accumBitsDU += ( numRBSPBytes << 3 );
  duData[0].accumNalsDU += numNalUnits;

  // adapt cumulative sums for all following DUs
  // and add one DU info SEI, if enabled
  for (Int i=1; i<duData.size(); i++)
  {
    if (m_pcCfg->getDecodingUnitInfoSEIEnabled())
    {
      numNalUnits  += 1;
      numRBSPBytes += ( 5 << 3 );
    }
    duData[i].accumBitsDU += numRBSPBytes; // probably around 5 bytes
    duData[i].accumNalsDU += numNalUnits;
  }

  // The last DU may have a trailing SEI
  if (m_pcCfg->getDecodedPictureHashSEIType()!=HASHTYPE_NONE)
  {
    duData.back().accumBitsDU += ( 20 << 3 ); // probably around 20 bytes - should be further adjusted, e.g. by type
    duData.back().accumNalsDU += 1;
  }

}
Void EncGOP::xUpdateTimingSEI(SEIPictureTiming *pictureTimingSEI, std::deque<DUData> &duData, const SPS *sps)
{
  if (!pictureTimingSEI)
  {
    return;
  }
  const VUI *vui = sps->getVuiParameters();
  const HRD *hrd = vui->getHrdParameters();
  if( hrd->getSubPicCpbParamsPresentFlag() )
  {
    Int i;
    UInt64 ui64Tmp;
    UInt uiPrev = 0;
    UInt numDU = ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 );
    std::vector<UInt> &rDuCpbRemovalDelayMinus1 = pictureTimingSEI->m_duCpbRemovalDelayMinus1;
    UInt maxDiff = ( hrd->getTickDivisorMinus2() + 2 ) - 1;

    for( i = 0; i < numDU; i ++ )
    {
      pictureTimingSEI->m_numNalusInDuMinus1[ i ]       = ( i == 0 ) ? ( duData[i].accumNalsDU - 1 ) : ( duData[i].accumNalsDU- duData[i-1].accumNalsDU - 1 );
    }

    if( numDU == 1 )
    {
      rDuCpbRemovalDelayMinus1[ 0 ] = 0; /* don't care */
    }
    else
    {
      rDuCpbRemovalDelayMinus1[ numDU - 1 ] = 0;/* by definition */
      UInt tmp = 0;
      UInt accum = 0;

      for( i = ( numDU - 2 ); i >= 0; i -- )
      {
        ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );
        if( (UInt)ui64Tmp > maxDiff )
        {
          tmp ++;
        }
      }
      uiPrev = 0;

      UInt flag = 0;
      for( i = ( numDU - 2 ); i >= 0; i -- )
      {
        flag = 0;
        ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );

        if( (UInt)ui64Tmp > maxDiff )
        {
          if(uiPrev >= maxDiff - tmp)
          {
            ui64Tmp = uiPrev + 1;
            flag = 1;
          }
          else                            ui64Tmp = maxDiff - tmp + 1;
        }
        rDuCpbRemovalDelayMinus1[ i ] = (UInt)ui64Tmp - uiPrev - 1;
        if( (Int)rDuCpbRemovalDelayMinus1[ i ] < 0 )
        {
          rDuCpbRemovalDelayMinus1[ i ] = 0;
        }
        else if (tmp > 0 && flag == 1)
        {
          tmp --;
        }
        accum += rDuCpbRemovalDelayMinus1[ i ] + 1;
        uiPrev = accum;
      }
    }
  }
}
Void EncGOP::xUpdateDuInfoSEI(SEIMessages &duInfoSeiMessages, SEIPictureTiming *pictureTimingSEI)
{
  if (duInfoSeiMessages.empty() || (pictureTimingSEI == NULL))
  {
    return;
  }

  Int i=0;

  for (SEIMessages::iterator du = duInfoSeiMessages.begin(); du!= duInfoSeiMessages.end(); du++)
  {
    SEIDecodingUnitInfo *duInfoSEI = (SEIDecodingUnitInfo*) (*du);
    duInfoSEI->m_decodingUnitIdx = i;
    duInfoSEI->m_duSptCpbRemovalDelay = pictureTimingSEI->m_duCpbRemovalDelayMinus1[i] + 1;
    duInfoSEI->m_dpbOutputDuDelayPresentFlag = false;
    i++;
  }
}

static Void
cabac_zero_word_padding(Slice *const pcSlice, Picture *const pcPic, const std::size_t binCountsInNalUnits, const std::size_t numBytesInVclNalUnits, std::ostringstream &nalUnitData, const Bool cabacZeroWordPaddingEnabled)
{
  const SPS &sps=*(pcSlice->getSPS());
  const ChromaFormat format = sps.getChromaFormatIdc();
  const Int log2subWidthCxsubHeightC = (::getComponentScaleX(COMPONENT_Cb, format)+::getComponentScaleY(COMPONENT_Cb, format));
  const Int minCuWidth  = pcPic->cs->pcv->minCUWidth;
  const Int minCuHeight = pcPic->cs->pcv->minCUHeight;
  const Int paddedWidth = ((sps.getPicWidthInLumaSamples()  + minCuWidth  - 1) / minCuWidth) * minCuWidth;
  const Int paddedHeight= ((sps.getPicHeightInLumaSamples() + minCuHeight - 1) / minCuHeight) * minCuHeight;
  const Int rawBits = paddedWidth * paddedHeight *
                         (sps.getBitDepth(CHANNEL_TYPE_LUMA) + 2*(sps.getBitDepth(CHANNEL_TYPE_CHROMA)>>log2subWidthCxsubHeightC));
  const std::size_t threshold = (32/3)*numBytesInVclNalUnits + (rawBits/32);
  if (binCountsInNalUnits >= threshold)
  {
    // need to add additional cabac zero words (each one accounts for 3 bytes (=00 00 03)) to increase numBytesInVclNalUnits
    const std::size_t targetNumBytesInVclNalUnits = ((binCountsInNalUnits - (rawBits/32))*3+31)/32;

    if (targetNumBytesInVclNalUnits>numBytesInVclNalUnits) // It should be!
    {
      const std::size_t numberOfAdditionalBytesNeeded=targetNumBytesInVclNalUnits - numBytesInVclNalUnits;
      const std::size_t numberOfAdditionalCabacZeroWords=(numberOfAdditionalBytesNeeded+2)/3;
      const std::size_t numberOfAdditionalCabacZeroBytes=numberOfAdditionalCabacZeroWords*3;
      if (cabacZeroWordPaddingEnabled)
      {
        std::vector<UChar> zeroBytesPadding(numberOfAdditionalCabacZeroBytes, UChar(0));
        for(std::size_t i=0; i<numberOfAdditionalCabacZeroWords; i++)
        {
          zeroBytesPadding[i*3+2]=3;  // 00 00 03
        }
        nalUnitData.write(reinterpret_cast<const TChar*>(&(zeroBytesPadding[0])), numberOfAdditionalCabacZeroBytes);
        msg( NOTICE, "Adding %d bytes of padding\n", UInt( numberOfAdditionalCabacZeroWords * 3 ) );
      }
      else
      {
        msg( NOTICE, "Standard would normally require adding %d bytes of padding\n", UInt( numberOfAdditionalCabacZeroWords * 3 ) );
      }
    }
  }
}

class EfficientFieldIRAPMapping
{
  private:
    Int  IRAPGOPid;
    Bool IRAPtoReorder;
    Bool swapIRAPForward;

  public:
    EfficientFieldIRAPMapping() :
      IRAPGOPid(-1),
      IRAPtoReorder(false),
      swapIRAPForward(false)
    { }

    Void initialize(const Bool isField, const Int gopSize, const Int POCLast, const Int numPicRcvd, const Int lastIDR, EncGOP *pEncGop, EncCfg *pCfg);

    Int adjustGOPid(const Int gopID);
    Int restoreGOPid(const Int gopID);
    Int GetIRAPGOPid() const { return IRAPGOPid; }
};

Void EfficientFieldIRAPMapping::initialize(const Bool isField, const Int gopSize, const Int POCLast, const Int numPicRcvd, const Int lastIDR, EncGOP *pEncGop, EncCfg *pCfg )
{
  if(isField)
  {
    Int pocCurr;
    for ( Int iGOPid=0; iGOPid < gopSize; iGOPid++ )
    {
      // determine actual POC
      if(POCLast == 0) //case first frame or first top field
      {
        pocCurr=0;
      }
      else if(POCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
      {
        pocCurr = 1;
      }
      else
      {
        pocCurr = POCLast - numPicRcvd + pCfg->getGOPEntry(iGOPid).m_POC - isField;
      }

      // check if POC corresponds to IRAP
      NalUnitType tmpUnitType = pEncGop->getNalUnitType(pocCurr, lastIDR, isField);
      if(tmpUnitType >= NAL_UNIT_CODED_SLICE_BLA_W_LP && tmpUnitType <= NAL_UNIT_CODED_SLICE_CRA) // if picture is an IRAP
      {
        if(pocCurr%2 == 0 && iGOPid < gopSize-1 && pCfg->getGOPEntry(iGOPid).m_POC == pCfg->getGOPEntry(iGOPid+1).m_POC-1)
        { // if top field and following picture in enc order is associated bottom field
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true;
          swapIRAPForward = true;
          break;
        }
        if(pocCurr%2 != 0 && iGOPid > 0 && pCfg->getGOPEntry(iGOPid).m_POC == pCfg->getGOPEntry(iGOPid-1).m_POC+1)
        {
          // if picture is an IRAP remember to process it first
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true;
          swapIRAPForward = false;
          break;
        }
      }
    }
  }
}

Int EfficientFieldIRAPMapping::adjustGOPid(const Int GOPid)
{
  if(IRAPtoReorder)
  {
    if(swapIRAPForward)
    {
      if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid +1;
      }
      else if(GOPid == IRAPGOPid +1)
      {
        return IRAPGOPid;
      }
    }
    else
    {
      if(GOPid == IRAPGOPid -1)
      {
        return IRAPGOPid;
      }
      else if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid -1;
      }
    }
  }
  return GOPid;
}

Int EfficientFieldIRAPMapping::restoreGOPid(const Int GOPid)
{
  if(IRAPtoReorder)
  {
    if(swapIRAPForward)
    {
      if(GOPid == IRAPGOPid)
      {
        IRAPtoReorder = false;
        return IRAPGOPid +1;
      }
      else if(GOPid == IRAPGOPid +1)
      {
        return GOPid -1;
      }
    }
    else
    {
      if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid -1;
      }
      else if(GOPid == IRAPGOPid -1)
      {
        IRAPtoReorder = false;
        return IRAPGOPid;
      }
    }
  }
  return GOPid;
}


#if X0038_LAMBDA_FROM_QP_CAPABILITY
static UInt calculateCollocatedFromL0Flag(const Slice *pSlice)
{
  const Int refIdx = 0; // Zero always assumed
  const Picture *refPicL0 = pSlice->getRefPic(REF_PIC_LIST_0, refIdx);
  const Picture *refPicL1 = pSlice->getRefPic(REF_PIC_LIST_1, refIdx);
  return refPicL0->slices[0]->getSliceQp() > refPicL1->slices[0]->getSliceQp();
}
#else
static UInt calculateCollocatedFromL1Flag(EncCfg *pCfg, const Int GOPid, const Int gopSize)
{
  Int iCloseLeft=1, iCloseRight=-1;
  for(Int i = 0; i<pCfg->getGOPEntry(GOPid).m_numRefPics; i++)
  {
    Int iRef = pCfg->getGOPEntry(GOPid).m_referencePics[i];
    if(iRef>0&&(iRef<iCloseRight||iCloseRight==-1))
    {
      iCloseRight=iRef;
    }
    else if(iRef<0&&(iRef>iCloseLeft||iCloseLeft==1))
    {
      iCloseLeft=iRef;
    }
  }
  if(iCloseRight>-1)
  {
    iCloseRight=iCloseRight+pCfg->getGOPEntry(GOPid).m_POC-1;
  }
  if(iCloseLeft<1)
  {
    iCloseLeft=iCloseLeft+pCfg->getGOPEntry(GOPid).m_POC-1;
    while(iCloseLeft<0)
    {
      iCloseLeft+=gopSize;
    }
  }
  Int iLeftQP=0, iRightQP=0;
  for(Int i=0; i<gopSize; i++)
  {
    if(pCfg->getGOPEntry(i).m_POC==(iCloseLeft%gopSize)+1)
    {
      iLeftQP= pCfg->getGOPEntry(i).m_QPOffset;
    }
    if (pCfg->getGOPEntry(i).m_POC==(iCloseRight%gopSize)+1)
    {
      iRightQP=pCfg->getGOPEntry(i).m_QPOffset;
    }
  }
  if(iCloseRight>-1&&iRightQP<iLeftQP)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}
#endif


static Void
printHash(const HashType hashType, const std::string &digestStr)
{
  const TChar *decodedPictureHashModeName;
  switch (hashType)
  {
    case HASHTYPE_MD5:
      decodedPictureHashModeName = "MD5";
      break;
    case HASHTYPE_CRC:
      decodedPictureHashModeName = "CRC";
      break;
    case HASHTYPE_CHECKSUM:
      decodedPictureHashModeName = "Checksum";
      break;
    default:
      decodedPictureHashModeName = NULL;
      break;
  }
  if (decodedPictureHashModeName != NULL)
  {
    if (digestStr.empty())
    {
      msg( NOTICE, " [%s:%s]", decodedPictureHashModeName, "?");
    }
    else
    {
      msg( NOTICE, " [%s:%s]", decodedPictureHashModeName, digestStr.c_str());
    }
  }
}

bool isPicEncoded( int targetPoc, int curPoc, int curTLayer, int gopSize, int intraPeriod )
{
  int  tarGop = targetPoc / gopSize;
  int  curGop = curPoc / gopSize;

  if( tarGop + 1 == curGop )
  {
    // part of next GOP only for tl0 pics
    return curTLayer == 0;
  }

  int  tarIFr = ( targetPoc / intraPeriod ) * intraPeriod;
  int  curIFr = ( curPoc / intraPeriod ) * intraPeriod;

  if( curIFr != tarIFr )
  {
    return false;
  }

  int  tarId = targetPoc - tarGop * gopSize;

  if( tarGop > curGop )
  {
    return ( tarId == 0 ) ? ( 0 == curTLayer ) : ( 1 >= curTLayer );
  }

  if( tarGop + 1 < curGop )
  {
    return false;
  }

  int  curId = curPoc - curGop * gopSize;
  int  tarTL = 0;

  while( tarId != 0 )
  {
    gopSize /= 2;
    if( tarId >= gopSize )
    {
      tarId -= gopSize;
      if( curId != 0 ) curId -= gopSize;
    }
    else if( curId == gopSize )
    {
      curId = 0;
    }
    tarTL++;
  }

  return curTLayer <= tarTL && curId == 0;
}

void trySkipOrDecodePicture( bool& decPic, bool& encPic, const EncCfg& cfg, Picture* pcPic )
{
  // check if we should decode a leading bitstream
  if( !cfg.getDecodeBitstream( 0 ).empty() )
  {
    static bool bDecode1stPart = true; /* TODO: MT */
    if( bDecode1stPart )
    {
      if( cfg.getForceDecodeBitstream1() )
      {
        if( ( bDecode1stPart = tryDecodePicture( pcPic, pcPic->getPOC(), cfg.getDecodeBitstream( 0 ), false ) ) )
        {
          decPic = bDecode1stPart;
        }
      }
      else
      {
        // update decode decision
        if( ( bDecode1stPart = ( cfg.getSwitchPOC() != pcPic->getPOC() )) && ( bDecode1stPart = tryDecodePicture( pcPic, pcPic->getPOC(), cfg.getDecodeBitstream( 0 ), false ) ) )
        {
          decPic = bDecode1stPart;
          return;
        }
        else if( pcPic->getPOC() )
        {
          // reset decoder if used and not required any further
          tryDecodePicture( NULL, 0, std::string( "" ) );
        }
      }
    }

    encPic |= cfg.getForceDecodeBitstream1() && !decPic;
    if( cfg.getForceDecodeBitstream1() ) { return; }
  }


  // check if we should decode a trailing bitstream
  if( ! cfg.getDecodeBitstream(1).empty() )
  {
    const int  iNextKeyPOC    = (1+cfg.getSwitchPOC()  / cfg.getGOPSize())     *cfg.getGOPSize();
    const int  iNextIntraPOC  = (1+(cfg.getSwitchPOC() / cfg.getIntraPeriod()))*cfg.getIntraPeriod();
    const int  iRestartIntraPOC   = iNextIntraPOC + (((iNextKeyPOC == iNextIntraPOC) && cfg.getSwitchDQP() ) ? cfg.getIntraPeriod() : 0);

    bool bDecode2ndPart = (pcPic->getPOC() >= iRestartIntraPOC);
    int expectedPoc = pcPic->getPOC();
    Slice slice0;
    if ( cfg.getBs2ModPOCAndType() )
    {
      expectedPoc = pcPic->getPOC() - iRestartIntraPOC;
      slice0.copySliceInfo( pcPic->slices[ 0 ], false );
    }
    if( bDecode2ndPart && (bDecode2ndPart = tryDecodePicture( pcPic, expectedPoc, cfg.getDecodeBitstream(1), true )) )
    {
      decPic = bDecode2ndPart;
      if ( cfg.getBs2ModPOCAndType() )
      {
        for( int i = 0; i < pcPic->slices.size(); i++ )
        {
          pcPic->slices[ i ]->setPOC              ( slice0.getPOC()            );
          if ( pcPic->slices[ i ]->getNalUnitType() != slice0.getNalUnitType()
              && pcPic->slices[ i ]->getIdrPicFlag()
              && slice0.getRapPicFlag()
              && slice0.isIntra() )
          {
            // patch IDR-slice to CRA-Intra-slice
            pcPic->slices[ i ]->setNalUnitType    ( slice0.getNalUnitType()    );
            pcPic->slices[ i ]->setLastIDR        ( slice0.getLastIDR()        );
            pcPic->slices[ i ]->setEnableTMVPFlag ( slice0.getEnableTMVPFlag() );
            if ( slice0.getEnableTMVPFlag() )
            {
              pcPic->slices[ i ]->setColFromL0Flag( slice0.getColFromL0Flag()  );
              pcPic->slices[ i ]->setColRefIdx    ( slice0.getColRefIdx()      );
            }
          }
        }
      }
      return;
    }
  }

  // leave here if we do not use forward to poc
  if( ! cfg.useFastForwardToPOC() )
  {
    // let's encode
    encPic   = true;
    return;
  }

  // this is the forward to poc section
  static bool bHitFastForwardPOC = false; /* TODO: MT */
  if( bHitFastForwardPOC || isPicEncoded( cfg.getFastForwardToPOC(), pcPic->getPOC(), pcPic->layer, cfg.getGOPSize(), cfg.getIntraPeriod() ) )
  {
    bHitFastForwardPOC |= cfg.getFastForwardToPOC() == pcPic->getPOC(); // once we hit the poc we continue encoding

    if( bHitFastForwardPOC && cfg.getStopAfterFFtoPOC() && cfg.getFastForwardToPOC() != pcPic->getPOC() )
    {
      return;
    }

    //except if FastForwardtoPOC is meant to be a SwitchPOC in thist case drop all preceding pictures
    if( bHitFastForwardPOC && ( cfg.getSwitchPOC() == cfg.getFastForwardToPOC() ) && ( cfg.getFastForwardToPOC() > pcPic->getPOC() ) )
    {
      return;
    }
    // let's encode
    encPic   = true;
  }
}



#if HIERARCHY_GENETATE_OrgBGP
Void EncGOP::CompDiffOrg(Int uiWidth, Int uiHeight, Picture* pcPic, /*double& diff,*/ Int lev, Bool divflag
#if PRINT_OrgDIFF
	, ofstream &os
#endif
)
{

	Int uiPartitionNum = 1;
	if (divflag)
	{
		uiPartitionNum = 4;
		divflag = false;
	}
	for (Int uiPartition = 0; uiPartition < uiPartitionNum; uiPartition++)
	{
		Int level = lev;
		Int uiW = uiWidth;
		Int uiH = uiHeight;
		switch (uiPartition)
		{
		case 0:
			break;
		case 1:
			uiW += UNIT_LEN >> level;
			break;
		case 2:
			uiH += UNIT_LEN >> level;
			break;
		case 3:
			uiW += UNIT_LEN >> level;
			uiH += UNIT_LEN >> level;
			break;
		default:
			break;
		}
		double diff = 0;
		

		pcPic->xCompDiffOrg(uiW, uiH, pcPic, diff, level);  //get 分块的 diff
		Int Th = 8;//背景生成阈值
		/*if (pcPic->getPOC() < 20)
		{
			Th = 7;
		}*/

#if PRINT_OrgDIFF
		if (level == 0)
		{
			if (os.is_open())
			{
				if (uiW % 128 == 0 && uiW != 0)
					os << endl;
				os << setiosflags(ios::right) << setw(5) << diff;
			}
		}
#endif

		switch (level)
		{

		case 0:
			if (diff < Th)
			{
				//pcPic->Copy2OrgBackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, level);
				/*315*/
				if (pcPic->IsEmpty(m_bgNewPicYuvOrgGop, uiW, uiH, level))
				{//YuvOrg为0则直接添加
					pcPic->Copy2OrgBackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, level);
					//pcPic->Copy2BackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, level);
				}
				else
				{//否则计算pic与YuvOrg的差值 相同则取均值 不相同比较第三个
					double blockdiff = 0;					
					pcPic->xCompDiffBlock(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, blockdiff, level);  //get 分块的 diff
					if (blockdiff < Th)
					{//相同 （PicYuvOrg+Pic+PicRef）/3
						pcPic->CopyOrgPicMean(m_bgNewPicYuvOrgGop, m_bgNewPicYuvOrgGop, pcPic, uiW, uiH, level);
						//pcPic->CopyRecPicMean(m_bgNewPicYuvOrgGop, m_bgNewPicYuvOrgGop, pcPic, uiW, uiH, level);
					}
					else
					{//不相同 比较第三个
						double diffblock3 = 0;
						pcPic->xCompDiffBlock(m_bgNewBlockOrgGop, uiW, uiH, pcPic, diffblock3, level);
						if (diffblock3 < Th)
						{//与第三个相同 PicYuvOrg=（pcpic+ref+第三个）/3
							pcPic->CopyOrgPicMean(m_bgNewPicYuvOrgGop, m_bgNewBlockOrgGop, pcPic, uiW, uiH, level);
							//pcPic->CopyRecPicMean(m_bgNewPicYuvOrgGop, m_bgNewBlockOrgGop, pcPic, uiW, uiH, level);
						}
						else
						{//不同 则第三个=（pcpic+ref）/2
							pcPic->Copy2OrgBackPic(m_bgNewBlockOrgGop, uiW, uiH, pcPic, level);
							//pcPic->Copy2BackPic(m_bgNewBlockOrgGop, uiW, uiH, pcPic, level);
						}
					}
				}
				divflag = false;
			}
			else if (diff >= Th && diff < Th+6) //阈值之间的递归调用本函数
			{
				divflag = true;
				level += 1;

				CompDiffOrg(uiW, uiH, pcPic, level, divflag
#if PRINT_OrgDIFF
					, os
#endif
				);

			}
			else
			{
				divflag = false;
			}
			break;
		case 1:
			if (diff < Th-2)
			{
				//pcPic->Copy2OrgBackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, level);
				/*315*/
				if (pcPic->IsEmpty(m_bgNewPicYuvOrgGop, uiW, uiH, level))
				{//YuvOrg为0则直接添加
					pcPic->Copy2OrgBackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, level);
					//pcPic->Copy2BackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, level);//rec 双背景
				}
				else
				{//否则计算pic与YuvOrg的差值 相同则取均值 不相同比较第三个
					double blockdiff = 0;
					pcPic->xCompDiffBlock(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, blockdiff, level);  //get 分块的 diff
					if (blockdiff < Th-2)
					{//相同 （PicYuvOrg+Pic+PicRef）/3
						pcPic->CopyOrgPicMean(m_bgNewPicYuvOrgGop, m_bgNewPicYuvOrgGop, pcPic, uiW, uiH, level);
						//pcPic->CopyRecPicMean(m_bgNewPicYuvOrgGop, m_bgNewPicYuvOrgGop, pcPic, uiW, uiH, level);
					}
					else
					{//不相同 比较第三个
						double diffblock3 = 0;
						pcPic->xCompDiffBlock(m_bgNewBlockOrgGop, uiW, uiH, pcPic, diffblock3, level);
						if (diffblock3 < Th-2)
						{//PicYuvOrg=（pcpic+ref+第三个）/3
							pcPic->CopyOrgPicMean(m_bgNewPicYuvOrgGop, m_bgNewBlockOrgGop, pcPic, uiW, uiH, level);
							//pcPic->CopyRecPicMean(m_bgNewPicYuvOrgGop, m_bgNewBlockOrgGop, pcPic, uiW, uiH, level);
						}
						else
						{
							pcPic->Copy2OrgBackPic(m_bgNewBlockOrgGop, uiW, uiH, pcPic, level);
							//pcPic->Copy2BackPic(m_bgNewBlockOrgGop, uiW, uiH, pcPic, level);
						}
					}
				}
				divflag = false;
			}
			else if (diff >= Th-2 && diff < Th+2)
			{
				divflag = true;
				level += 1;

				CompDiffOrg(uiW, uiH, pcPic, level, divflag
#if PRINT_OrgDIFF
					, os
#endif
				);

			}
			else
				divflag = false;
			break;
		case 2:
			if (diff < Th-3)
			{
				//pcPic->Copy2OrgBackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, level);
				/*315*/
				if (pcPic->IsEmpty(m_bgNewPicYuvOrgGop, uiW, uiH, level))
				{//YuvOrg为0则直接添加
					pcPic->Copy2OrgBackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, level);
					//pcPic->Copy2BackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, level);
				}
				else
				{//否则计算pic与YuvOrg的差值 相同则取均值 不相同比较第三个
					double blockdiff = 0;
					pcPic->xCompDiffBlock(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, blockdiff, level);  //get 分块的 diff
					if (blockdiff < Th - 3)
					{//相同 （PicYuvOrg+Pic+PicRef）/3
						pcPic->CopyOrgPicMean(m_bgNewPicYuvOrgGop, m_bgNewPicYuvOrgGop, pcPic, uiW, uiH, level);
						//pcPic->CopyRecPicMean(m_bgNewPicYuvOrgGop, m_bgNewPicYuvOrgGop, pcPic, uiW, uiH, level);
					}
					else
					{//不相同 比较第三个
						double diffblock3 = 0;
						pcPic->xCompDiffBlock(m_bgNewBlockOrgGop, uiW, uiH, pcPic, diffblock3, level);
						if (diffblock3 < Th - 3)
						{//PicYuvOrg=（pcpic+ref+第三个）/3
							pcPic->CopyOrgPicMean(m_bgNewPicYuvOrgGop, m_bgNewBlockOrgGop, pcPic, uiW, uiH, level);
							//pcPic->CopyRecPicMean(m_bgNewPicYuvOrgGop, m_bgNewBlockOrgGop, pcPic, uiW, uiH, level);
						}
						else
						{
							pcPic->Copy2OrgBackPic(m_bgNewBlockOrgGop, uiW, uiH, pcPic, level);
							//pcPic->Copy2BackPic(m_bgNewBlockOrgGop, uiW, uiH, pcPic, level);
						}
					}
				}
				divflag = false;
			}
			else if (diff >= Th - 3 && diff < Th)
			{
				divflag = true;
				level += 1;

				CompDiffOrg(uiW, uiH, pcPic, level, divflag
#if PRINT_OrgDIFF
					, os
#endif
				);

			}
			else
				divflag = false;
			break;
		case 3:
			if (diff < Th - 4)
			{
				//pcPic->Copy2OrgBackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, level);
				/*315*/
				if (pcPic->IsEmpty(m_bgNewPicYuvOrgGop, uiW, uiH, level))
				{//YuvOrg为0则直接添加
					pcPic->Copy2OrgBackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, level);
					//pcPic->Copy2BackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, level);
				}
				else
				{//否则计算pic与YuvOrg的差值 相同则取均值 不相同比较第三个
					double blockdiff = 0;
					pcPic->xCompDiffBlock(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic, blockdiff, level);  //get 分块的 diff
					if (blockdiff < Th - 4)
					{//相同 （PicYuvOrg+Pic+PicRef）/3
						pcPic->CopyOrgPicMean(m_bgNewPicYuvOrgGop, m_bgNewPicYuvOrgGop, pcPic, uiW, uiH, level);
						//pcPic->CopyRecPicMean(m_bgNewPicYuvOrgGop, m_bgNewPicYuvOrgGop, pcPic, uiW, uiH, level);
					}
					else
					{//不相同 比较第三个
						double diffblock3 = 0;
						pcPic->xCompDiffBlock(m_bgNewBlockOrgGop, uiW, uiH, pcPic, diffblock3, level);
						if (diffblock3 < Th - 4)
						{//PicYuvOrg=（pcpic+ref+第三个）/3
							pcPic->CopyOrgPicMean(m_bgNewPicYuvOrgGop, m_bgNewBlockOrgGop, pcPic, uiW, uiH, level);
							//pcPic->CopyRecPicMean(m_bgNewPicYuvOrgGop, m_bgNewBlockOrgGop, pcPic, uiW, uiH, level);
						}
						else
						{
							pcPic->Copy2OrgBackPic(m_bgNewBlockOrgGop, uiW, uiH, pcPic, level);
							//pcPic->Copy2BackPic(m_bgNewBlockOrgGop, uiW, uiH, pcPic, level);
						}
					}
				}
				divflag = false;
			}
			else
				divflag = false;
			break;

		default:

			break;
		}

}
				
}
#endif

#if HIERARCHY_GENETATE_BGP

Void EncGOP::CompDiff(Int uiWidth, Int uiHeight, Picture* pcPic, Int lev, Bool divflag
#if PRINT_DIFF
	, ofstream &os
#endif
)
{

	Int uiPartitionNum = 1;
	if (divflag)
	{
		uiPartitionNum = 4;
		divflag = false;
	}
	for (Int uiPartition = 0; uiPartition < uiPartitionNum; uiPartition++)
	{
		Int level = lev;
		Int uiW = uiWidth;
		Int uiH = uiHeight;
		switch (uiPartition)
		{
		case 0:
			break;
		case 1:
			uiW += UNIT_LEN >> level;
			break;
		case 2:
			uiH += UNIT_LEN >> level;
			break;
		case 3:
			uiW += UNIT_LEN >> level;
			uiH += UNIT_LEN >> level;
			break;
		default:
			break;
		}
		double diff = 0;
		

		pcPic->xCompDiff(uiW, uiH, pcPic, diff, level);


#if PRINT_DIFF
		if (level == 0)
		{
			if (os.is_open())
			{
				if (uiW % 128 == 0 && uiW != 0)
					os << endl;
				os << setiosflags(ios::right) << setw(5) << diff;
		}
	}
#endif
		int th = 3;
		switch (level)
		{

		case 0:
			if (diff < 3)
			{
				
				pcPic->Copy2BackPic(m_bgNewPicYuvRecGop, uiW, uiH, pcPic, level);

				divflag = false;
			}
			else if (diff >= th && diff < th+6)
			{
				divflag = true;
				level += 1;

				CompDiff(uiW, uiH, pcPic, level, divflag
#if PRINT_DIFF
					, os
#endif
				);

			}
			else
			{
				divflag = false;
			}
			break;
		case 1:
			if (diff < th-1)
			{
				pcPic->Copy2BackPic(m_bgNewPicYuvRecGop, uiW, uiH, pcPic, level);
				divflag = false;
			}
			else if (diff >= th-1 && diff < th+2)
			{
				divflag = true;
				level += 1;

				CompDiff(uiW, uiH, pcPic, level, divflag
#if PRINT_DIFF
					, os
#endif
				);

			}
			else
				divflag = false;
			break;
		case 2:
			if (diff < th-2)
			{
				pcPic->Copy2BackPic(m_bgNewPicYuvRecGop, uiW, uiH, pcPic, level);
				divflag = false;
			}
			else if (diff >= th-2 && diff < th+1)
			{
				divflag = true;
				level += 1;

				CompDiff(uiW, uiH, pcPic, level, divflag
#if PRINT_DIFF
					, os
#endif
				);

			}
			else
				divflag = false;
			break;
		case 3:
			if (diff < th-3)
			{
				
				pcPic->Copy2BackPic(m_bgNewPicYuvRecGop, uiW, uiH, pcPic, level);
				divflag = false;
			}
			else
				divflag = false;
			break;

		default:

			break;
		}

}
}
#endif

#if OrgBG_BLOCK_SUBSTITUTION
Double EncGOP::CompNablaOrg(Int compId, UInt uiH, UInt uiW, Picture* pcPic)
{
	Pel* piOrg;
	const Pel* piRef;
	Double* piNabla;
	UInt uiStride;
	UInt uiHeight;
	UInt uiWidth;
	UInt unit_len;
	Double  Fc;
	switch (compId)
	{
	case 0:
		piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
		/*if(pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getRecoBuf().Y().bufAt(0, 0);
		else*/
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Y().bufAt(0, 0);
			

		uiStride = pcPic->getOrigBuf().Y().stride;
		uiHeight = pcPic->getOrigBuf().Y().height;
		uiWidth = pcPic->getOrigBuf().Y().width;
		unit_len = BLOCK_LEN;
		piNabla = m_pcNablaY;
		break;
	case 1:
		piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
		/*if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getRecoBuf().Cb().bufAt(0, 0);
		else*/
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Cb().bufAt(0, 0);

		uiStride = pcPic->getOrigBuf().Cb().stride;
		uiHeight = pcPic->getOrigBuf().Cb().height;
		uiWidth = pcPic->getOrigBuf().Cb().width;
		unit_len = BLOCK_LEN >> 1;
		uiH >>= 1;
		uiW >>= 1;
		piNabla = m_pcNablaCb;
		break;
	case 2:
		piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
		/*if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getRecoBuf().Cr().bufAt(0, 0);
		else*/
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Cr().bufAt(0, 0);

		uiStride = pcPic->getOrigBuf().Cr().stride;
		uiHeight = pcPic->getOrigBuf().Cr().height;
		uiWidth = pcPic->getOrigBuf().Cr().width;
		unit_len = BLOCK_LEN >> 1;
		uiH >>= 1;
		uiW >>= 1;
		piNabla = m_pcNablaCr;
		break;
	default:
		break;
	}

	{
		Double mu = 0;
		Double sigma = 0;
		Double* pNabla = piNabla;
		const Pel* pRef = piRef;
		Pel* pOrg = piOrg;
		for (UInt uiY = 0; uiY < unit_len && (uiH + uiY < uiHeight); uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len && (uiW + uiX < uiWidth); uiX++)
			{
				if (uiH != (uiHeight - unit_len) && uiW != (uiWidth - unit_len))
				{
					pNabla[uiX] =
						((pOrg[uiH * uiStride + uiW + uiX] - pRef[uiH * uiStride + uiW + uiX]) - (pOrg[(uiH + unit_len) * uiStride + uiW + uiX] - pRef[(uiH + unit_len) * uiStride + uiW + uiX]))
						* ((pOrg[uiH * uiStride + uiW + uiX] - pRef[uiH * uiStride + uiW + uiX]) - (pOrg[(uiH + unit_len) * uiStride + uiW + uiX] - pRef[(uiH + unit_len) * uiStride + uiW + uiX]))
						+ ((pOrg[uiH * uiStride + uiW + uiX] - pRef[uiH * uiStride + uiW + uiX]) - (pOrg[uiH * uiStride + uiW + unit_len + uiX] - pRef[uiH * uiStride + uiW + unit_len + uiX]))
						* ((pOrg[uiH * uiStride + uiW + uiX] - pRef[uiH * uiStride + uiW + uiX]) - (pOrg[uiH * uiStride + uiW + unit_len + uiX] - pRef[uiH * uiStride + uiW + unit_len + uiX]))
						;
					mu = mu + sqrt(pNabla[uiX]);/*sqrt((double)(pNabla[uiX]))*/
				}
				else if (uiH == (uiHeight - unit_len) && uiW != (uiWidth - unit_len))
				{
					pNabla[uiX] =
						abs(
						(pOrg[uiH * uiStride + uiW + uiX] - pRef[uiH * uiStride + uiW + uiX]) - (pOrg[uiH * uiStride + uiW + unit_len + uiX] - pRef[uiH * uiStride + uiW + unit_len + uiX])
						);
					mu = mu + pNabla[uiX];
				}
				else if (uiH != (uiHeight - unit_len) && uiW == (uiWidth - unit_len))
				{
					pNabla[uiX] =
						abs(
						(pOrg[uiH * uiStride + uiW + uiX] - pRef[uiH * uiStride + uiW + uiX]) - (pOrg[(uiH + unit_len) * uiStride + uiW + uiX] - pRef[(uiH + unit_len) * uiStride + uiW + uiX])
						);
					mu = mu + pNabla[uiX];
				}
			}
			pOrg += uiStride;
			pRef += uiStride;
			pNabla += unit_len;
		}

		mu = mu / (unit_len * unit_len);

		for (UInt uiY = 0; uiY < unit_len && (uiH + uiY < uiHeight); uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len && (uiW + uiX < uiWidth); uiX++)
			{
				sigma += (piNabla[uiX] - mu) * (piNabla[uiX] - mu);
			}
			piNabla += unit_len;
		}
		sigma = sqrt(sigma / (unit_len * unit_len));
		Fc = log(1.0 + (sigma / (mu + 1)));
	}
	return Fc;
}
#endif

#if BG_BLOCK_SUBSTITUTION
Double EncGOP::CompNabla(Int compId, UInt uiH, UInt uiW, Picture* pcPic)
{
	Pel* piOrg;
	const Pel* piRef;
	Double* piNabla;
	UInt uiStride;
	UInt uiHeight;
	UInt uiWidth;
	UInt unit_len;
	Double  Fc;
	switch (compId)
	{
	case 0:
		piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
		piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Y().bufAt(0, 0);

		uiStride = pcPic->getRecoBuf().Y().stride;
		uiHeight = pcPic->getRecoBuf().Y().height;
		uiWidth = pcPic->getRecoBuf().Y().width;
		unit_len = BLOCK_LEN;
		piNabla = m_pcNablaY;
		break;
	case 1:
		piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);
		piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Cb().bufAt(0, 0);

		uiStride = pcPic->getRecoBuf().Cb().stride;
		uiHeight = pcPic->getRecoBuf().Cb().height;
		uiWidth = pcPic->getRecoBuf().Cb().width;
		unit_len = BLOCK_LEN >> 1;
		uiH >>= 1;
		uiW >>= 1;
		piNabla = m_pcNablaCb;
		break;
	case 2:
		piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);
		piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Cr().bufAt(0, 0);

		uiStride = pcPic->getRecoBuf().Cr().stride;
		uiHeight = pcPic->getRecoBuf().Cr().height;
		uiWidth = pcPic->getRecoBuf().Cr().width;
		unit_len = BLOCK_LEN >> 1;
		uiH >>= 1;
		uiW >>= 1;
		piNabla = m_pcNablaCr;
		break;
	default:
		break;
	}

	{
		Double mu = 0;
		Double sigma = 0;
		Double* pNabla = piNabla;
		const Pel* pRef = piRef;
		Pel* pOrg = piOrg;
		for (UInt uiY = 0; uiY < unit_len && (uiH + uiY < uiHeight); uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len && (uiW + uiX < uiWidth); uiX++)
			{
				if (uiH != (uiHeight - unit_len) && uiW != (uiWidth - unit_len))
				{
					pNabla[uiX] =
						((pOrg[uiH * uiStride + uiW + uiX] - pRef[uiH * uiStride + uiW + uiX]) - (pOrg[(uiH + unit_len) * uiStride + uiW + uiX] - pRef[(uiH + unit_len) * uiStride + uiW + uiX]))
						* ((pOrg[uiH * uiStride + uiW + uiX] - pRef[uiH * uiStride + uiW + uiX]) - (pOrg[(uiH + unit_len) * uiStride + uiW + uiX] - pRef[(uiH + unit_len) * uiStride + uiW + uiX]))
						+ ((pOrg[uiH * uiStride + uiW + uiX] - pRef[uiH * uiStride + uiW + uiX]) - (pOrg[uiH * uiStride + uiW + unit_len + uiX] - pRef[uiH * uiStride + uiW + unit_len + uiX]))
						* ((pOrg[uiH * uiStride + uiW + uiX] - pRef[uiH * uiStride + uiW + uiX]) - (pOrg[uiH * uiStride + uiW + unit_len + uiX] - pRef[uiH * uiStride + uiW + unit_len + uiX]))
						;
					mu = mu + sqrt(pNabla[uiX]);
				}
				else if (uiH == (uiHeight - unit_len) && uiW != (uiWidth - unit_len))
				{
					pNabla[uiX] =
						abs(
						(pOrg[uiH * uiStride + uiW + uiX] - pRef[uiH * uiStride + uiW + uiX]) - (pOrg[uiH * uiStride + uiW + unit_len + uiX] - pRef[uiH * uiStride + uiW + unit_len + uiX])
						);
					mu = mu + pNabla[uiX];
				}
				else if (uiH != (uiHeight - unit_len) && uiW == (uiWidth - unit_len))
				{
					pNabla[uiX] =
						abs(
						(pOrg[uiH * uiStride + uiW + uiX] - pRef[uiH * uiStride + uiW + uiX]) - (pOrg[(uiH + unit_len) * uiStride + uiW + uiX] - pRef[(uiH + unit_len) * uiStride + uiW + uiX])
						);
					mu = mu + pNabla[uiX];
				}
			}
			pOrg += uiStride;
			pRef += uiStride;
			pNabla += unit_len;
		}
		mu = mu / (unit_len * unit_len);

		for (UInt uiY = 0; uiY < unit_len && (uiH + uiY < uiHeight); uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len && (uiW + uiX < uiWidth); uiX++)
			{
				sigma += (piNabla[uiX] - mu) * (piNabla[uiX] - mu);
			}
			piNabla += unit_len;
		}
		sigma = sqrt(sigma / (unit_len * unit_len));
		Fc = log(1.0 + (sigma / (mu + 1)));
	}
	return Fc;
}
#endif

#if BG_BLOCK_SUBSTITUTION
Double EncGOP::CompBlockDiff(UInt uiH, UInt uiW, Picture* pcPicYuv, Picture* pcPic)
{
	Pel* piOrg;
	UInt uiStride;
	UInt uiHeight;
	UInt uiWidth;
	UInt unit_len;
	Double diff = 0;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piOrg = pcPicYuv->getOrigBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Y().stride;
			uiHeight = pcPic->getOrigBuf().Y().height;
			uiWidth = pcPic->getOrigBuf().Y().width;
			unit_len = BLOCK_LEN;
			break;
		case 1:
			piOrg = pcPicYuv->getOrigBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiHeight = pcPic->getOrigBuf().Cb().height;
			uiWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = BLOCK_LEN >> 1;
			uiH >>= 1;
			uiW >>= 1;
			break;
		case 2:
			piOrg = pcPicYuv->getOrigBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiHeight = pcPic->getOrigBuf().Cr().height;
			uiWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = BLOCK_LEN >> 1;
			//uiH >>= 1;
			//uiW >>= 1;
			break;
		default:
			break;
		}

		{
			Pel* pOrg = piOrg;
			if (uiH != 0 && uiH != (uiHeight - unit_len) && uiW != 0 && uiW != (uiWidth - unit_len))
			{
				for (UInt uiY = 0; uiY < uiH + unit_len && (uiH + uiY < uiHeight); uiY++)
				{
					diff = diff + abs(pOrg[uiH * uiStride + uiW] - pOrg[uiH * uiStride + uiW - 1])
						+ abs(pOrg[uiH * uiStride + uiW + unit_len - 1] - pOrg[uiH * uiStride + uiW + unit_len]);
					pOrg += uiStride;
				}
				for (UInt uiX = 0; uiX < uiW + unit_len && (uiW + uiX < uiWidth); uiX++)
				{
					diff = diff + abs(piOrg[uiH * uiStride + uiW + uiX] - piOrg[(uiH - 1) * uiStride + uiW + uiX])
						+ abs(piOrg[(uiH + unit_len - 1) * uiStride + uiW + uiX] - piOrg[(uiH + unit_len) * uiStride + uiW + uiX]);
				}
			}
			else if (uiH == 0 && uiW == 0)
			{
				for (UInt uiY = 0; uiY < uiH + unit_len && (uiH + uiY < uiHeight); uiY++)
				{
					diff = diff + abs(pOrg[uiH * uiStride + uiW + unit_len - 1] - pOrg[uiH * uiStride + uiW + unit_len]);
					pOrg += uiStride;
				}
				for (UInt uiX = 0; uiX < uiW + unit_len && (uiW + uiX < uiWidth); uiX++)
				{
					diff = diff + abs(piOrg[(uiH + unit_len - 1) * uiStride + uiW + uiX] - piOrg[(uiH + unit_len) * uiStride + uiW + uiX]);
				}
			}
			else if (uiH == (uiHeight - unit_len) && uiW == 0)
			{
				for (UInt uiY = 0; uiY < uiH + unit_len && (uiH + uiY < uiHeight); uiY++)
				{
					diff = diff + abs(pOrg[uiH * uiStride + uiW + unit_len - 1] - pOrg[uiH * uiStride + uiW + unit_len]);
					pOrg += uiStride;
				}
				for (UInt uiX = 0; uiX < uiW + unit_len && (uiW + uiX < uiWidth); uiX++)
				{
					diff = diff + abs(piOrg[uiH * uiStride + uiW + uiX] - piOrg[(uiH - 1) * uiStride + uiW + uiX]);
				}
			}
			else if (uiH == 0 && uiW == (uiWidth - unit_len))
			{
				for (UInt uiY = 0; uiY < uiH + unit_len && (uiH + uiY < uiHeight); uiY++)
				{
					diff = diff + abs(pOrg[uiH * uiStride + uiW] - pOrg[uiH * uiStride + uiW - 1]);
					pOrg += uiStride;
				}
				for (UInt uiX = 0; uiX < uiW + unit_len && (uiW + uiX < uiWidth); uiX++)
				{
					diff = diff + abs(piOrg[(uiH + unit_len - 1) * uiStride + uiW + uiX] - piOrg[(uiH + unit_len) * uiStride + uiW + uiX]);
				}
			}
			else if (uiH == (uiHeight - unit_len) && uiW == (uiWidth - unit_len))
			{
				for (UInt uiY = 0; uiY < uiH + unit_len && (uiH + uiY < uiHeight); uiY++)
				{
					diff = diff + abs(pOrg[uiH * uiStride + uiW] - pOrg[uiH * uiStride + uiW - 1]);
					pOrg += uiStride;
				}
				for (UInt uiX = 0; uiX < uiW + unit_len && (uiW + uiX < uiWidth); uiX++)
				{
					diff = diff + abs(piOrg[uiH * uiStride + uiW + uiX] - piOrg[(uiH - 1) * uiStride + uiW + uiX]);
				}
			}
			else if (uiH == 0)
			{
				for (UInt uiY = 0; uiY < uiH + unit_len && (uiH + uiY < uiHeight); uiY++)
				{
					diff = diff + abs(pOrg[uiH * uiStride + uiW] - pOrg[uiH * uiStride + uiW - 1])
						+ abs(pOrg[uiH * uiStride + uiW + unit_len - 1] - pOrg[uiH * uiStride + uiW + unit_len]);
					pOrg += uiStride;
				}
				for (UInt uiX = 0; uiX < uiW + unit_len && (uiW + uiX < uiWidth); uiX++)
				{
					diff = diff + abs(piOrg[(uiH + unit_len - 1) * uiStride + uiW + uiX] - piOrg[(uiH + unit_len) * uiStride + uiW + uiX]);
				}
			}
			else if (uiH == (uiHeight - unit_len))
			{
				for (UInt uiY = 0; uiY < uiH + unit_len && (uiH + uiY < uiHeight); uiY++)
				{
					diff = diff + abs(pOrg[uiH * uiStride + uiW] - pOrg[uiH * uiStride + uiW - 1])
						+ abs(pOrg[uiH * uiStride + uiW + unit_len - 1] - pOrg[uiH * uiStride + uiW + unit_len]);
					pOrg += uiStride;
				}
				for (UInt uiX = 0; uiX < uiW + unit_len && (uiW + uiX < uiWidth); uiX++)
				{
					diff = diff + abs(piOrg[uiH * uiStride + uiW + uiX] - piOrg[(uiH - 1) * uiStride + uiW + uiX]);
				}
			}
			else if (uiW == 0)
			{
				for (UInt uiY = 0; uiY < uiH + unit_len && (uiH + uiY < uiHeight); uiY++)
				{
					diff = diff + abs(pOrg[uiH * uiStride + uiW + unit_len - 1] - pOrg[uiH * uiStride + uiW + unit_len]);
					pOrg += uiStride;
				}
				for (UInt uiX = 0; uiX < uiW + unit_len && (uiW + uiX < uiWidth); uiX++)
				{
					diff = diff + abs(piOrg[uiH * uiStride + uiW + uiX] - piOrg[(uiH - 1) * uiStride + uiW + uiX])
						+ abs(piOrg[(uiH + unit_len - 1) * uiStride + uiW + uiX] - piOrg[(uiH + unit_len) * uiStride + uiW + uiX]);
				}
			}
			else if (uiW == (uiWidth - unit_len))
			{
				for (UInt uiY = 0; uiY < uiH + unit_len && (uiH + uiY < uiHeight); uiY++)
				{
					diff = diff + abs(pOrg[uiH * uiStride + uiW] - pOrg[uiH * uiStride + uiW - 1]);
					pOrg += uiStride;
				}
				for (UInt uiX = 0; uiX < uiW + unit_len && (uiW + uiX < uiWidth); uiX++)
				{
					diff = diff + abs(piOrg[uiH * uiStride + uiW + uiX] - piOrg[(uiH - 1) * uiStride + uiW + uiX])
						+ abs(piOrg[(uiH + unit_len - 1) * uiStride + uiW + uiX] - piOrg[(uiH + unit_len) * uiStride + uiW + uiX]);
				}
			}
		}
	}
	return diff / unit_len;
}
#endif

#if ENCODE_BGP
Void EncGOP::copyCoeff(TCoeff* pcTempCoeff, TCoeff* pcCoeff, UInt uiWidth, UInt uiHeight, UInt uiAbsPartIdx)
{
	UInt uitmp = uiWidth * uiHeight;
	for (Int i = 0; i < uitmp; i++)
	{
		pcCoeff[i] = pcTempCoeff[i];
	}
}
#endif

#if TRANSFORM_BGP
Void EncGOP::TransformBGP(Int itransId, UInt uiAbsPartIdx, UInt uiH, UInt uiW, Int qpi)
{
	Pel*		piOrgBGP;
	Pel*		piRecBGP;
	Pel*		piResi;
	Pel*		piReco;
	UInt		uiStride;
	UInt		uiHeight;
	UInt		uiWidth;
	UInt		unit_len;
	TCoeff*		pcCoeff;
	ChannelType    TType;

	switch (itransId)
	{
	case 0:
		piOrgBGP = m_bgNewPicYuvOrgGop->getOrigBuf().Y().bufAt(0,0);
		piRecBGP = m_bgNewPicYuvRecGop->getOrigBuf().Y().bufAt(0,0);
		piResi = m_bgNewPicYuvResiGop->getOrigBuf().Y().bufAt(0,0);
		piReco = m_bgNewPicYuvRecoGop->getOrigBuf().Y().bufAt(0,0);

		uiStride = m_bgNewPicYuvOrgGop->getOrigBuf().Y().stride;
		uiHeight = m_bgNewPicYuvOrgGop->getOrigBuf().Y().height;
		uiWidth = m_bgNewPicYuvOrgGop->getOrigBuf().Y().width;
		unit_len = TRANS_UNIT_LEN;
		pcCoeff = m_pcTempCoeffY;
		TType = CHANNEL_TYPE_LUMA;
		break;
	case 1:
		piOrgBGP = m_bgNewPicYuvOrgGop->getOrigBuf().Cb().bufAt(0, 0);
		piRecBGP = m_bgNewPicYuvRecGop->getOrigBuf().Cb().bufAt(0, 0);
		piResi = m_bgNewPicYuvResiGop->getOrigBuf().Cb().bufAt(0, 0);
		piReco = m_bgNewPicYuvRecoGop->getOrigBuf().Cb().bufAt(0, 0);

		uiStride = m_bgNewPicYuvOrgGop->getOrigBuf().Y().stride;
		uiHeight = m_bgNewPicYuvOrgGop->getOrigBuf().Y().height >> 1;
		uiWidth = m_bgNewPicYuvOrgGop->getOrigBuf().Y().width >> 1;
		unit_len = TRANS_UNIT_LEN >> 1;
		uiH >>= 1;
		uiW >>= 1;
		pcCoeff = m_pcTempCoeffCb;
		TType = CHANNEL_TYPE_CHROMA;
		break;
	case 2:
		piOrgBGP = m_bgNewPicYuvOrgGop->getOrigBuf().Cr().bufAt(0, 0);
		piRecBGP = m_bgNewPicYuvRecGop->getOrigBuf().Cr().bufAt(0, 0);
		piResi = m_bgNewPicYuvResiGop->getOrigBuf().Cr().bufAt(0, 0);
		piReco = m_bgNewPicYuvRecoGop->getOrigBuf().Cr().bufAt(0, 0);

		uiStride = m_bgNewPicYuvOrgGop->getOrigBuf().Y().stride;
		uiHeight = m_bgNewPicYuvOrgGop->getOrigBuf().Y().height;
		uiWidth = m_bgNewPicYuvOrgGop->getOrigBuf().Y().width;
		unit_len = TRANS_UNIT_LEN >> 1;
		uiH >>= 1;
		uiW >>= 1;
		pcCoeff = m_pcTempCoeffCr;
		TType = MAX_NUM_CHANNEL_TYPE;
		break;
	default:
		break;
	}
	// get residual
	{
		Pel* pOrgBGP = piOrgBGP;
		Pel* pRecBGP = piRecBGP;
		Pel* pResi = piResi;

		for (UInt uiY = 0; uiY < unit_len && (uiH + uiY < uiHeight); uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len && (uiW + uiX < uiWidth); uiX++)
			{
				pResi[uiH * uiStride + uiW + uiX] = (pOrgBGP[uiH * uiStride + uiW + uiX] - pRecBGP[uiH * uiStride + uiW + uiX]);
			}

			pOrgBGP += uiStride;
			pResi += uiStride;
			pRecBGP += uiStride;
		}
	}

	UInt  uiAbsSum = 0;
	m_pcTrQuantGOP->setQPforQuant(qpi, TType, 0, 0);
	m_pcTrQuantGOP->xTransformBGP(&piResi[uiH * uiStride + uiW], uiStride, pcCoeff, uiAbsSum, TType, unit_len, unit_len);


#if ENCODE_BGP

	UInt uiMinCoeffSize = TRANS_UNIT_LEN * TRANS_UNIT_LEN;
	UInt uiLumaOffset = uiMinCoeffSize*uiAbsPartIdx;
	UInt uiChromaOffset = uiLumaOffset >> 2;
	if (TType == CHANNEL_TYPE_LUMA)
		copyCoeff(pcCoeff, m_pcCoeffY + uiLumaOffset, unit_len, unit_len, uiAbsPartIdx);
	else if (TType == CHANNEL_TYPE_CHROMA)
		copyCoeff(pcCoeff, m_pcCoeffCb + uiChromaOffset, unit_len, unit_len, uiAbsPartIdx);
	else
		copyCoeff(pcCoeff, m_pcCoeffCr + uiChromaOffset, unit_len, unit_len, uiAbsPartIdx);


#endif 

	//===== inverse transform =====
	if (uiAbsSum)
	{
		Int scalingListType = 0 + g_eTTable[(Int)TType];
		assert(scalingListType < SCALING_LIST_NUM);
		m_pcTrQuantGOP->invtransformNxN(false, itransId == 0 ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA, &piResi[uiH * uiStride + uiW], uiStride, pcCoeff, unit_len, unit_len, scalingListType, false);
	}
	else
	{
		Pel* pResi = &piResi[uiH * uiStride + uiW];
		memset(pcCoeff, 0, sizeof(TCoeff) * unit_len * unit_len);
		for (UInt uiY = 0; uiY < unit_len; uiY++)
		{
			memset(pResi, 0, sizeof(Pel) * unit_len);
			pResi += uiStride;
		}
	}

	//===== reconstruction =====
	{
		Pel* pRecBGP = &piRecBGP[uiH * uiStride + uiW];
		Pel* pResi = &piResi[uiH * uiStride + uiW];
		Pel* pReco = &piReco[uiH * uiStride + uiW];
		for (UInt uiY = 0; uiY < unit_len; uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len; uiX++)
			{
				pReco[uiX] = ClipC(pRecBGP[uiX] + pResi[uiX]);

			}
			pRecBGP += uiStride;
			pResi += uiStride;
			pReco += uiStride;
		}
	}



}
#endif

#if UPDATE_BGP
Void TEncGOP::TransformUpdateBGP(Int itransId, UInt uiAbsPartIdx, UInt uiH, UInt uiW, Int qpi
#if PRINT_UPDATEBG_RESI || PRINT_UPDATE_TRCOEFF
	, ofstream &ocout
#endif
)
{
	Pel*		piOrgBG;
	Pel*		piUpdateBG;
	//Pel*        piRecBG;
	Pel*		piResi;
	Pel*		piReco;
	UInt		uiStride;
	UInt		uiHeight;
	UInt		uiWidth;
	UInt		unit_len;
	TCoeff*		pcCoeff;
	TextType    TType;

	switch (itransId)
	{
	case 0:
		piUpdateBG = m_bgNewPicYuvOrgGop->getLumaAddr();
		piOrgBG = m_bgNewPicYuvRecoGop->getLumaAddr();
		//piRecBG = m_bgNewPicYuvRecGop->getLumaAddr();
		piResi = m_bgNewPicYuvUpdateResiGop->getLumaAddr();
		piReco = m_bgNewPicYuvRecoGop->getLumaAddr();

		uiStride = m_bgNewPicYuvOrgGop->getStride();
		uiHeight = m_bgNewPicYuvOrgGop->getHeight();
		uiWidth = m_bgNewPicYuvOrgGop->getWidth();
		unit_len = TRANS_UNIT_LEN;
		pcCoeff = m_pcTempCoeffY;
		TType = TEXT_LUMA;
		break;
	case 1:
		piUpdateBG = m_bgNewPicYuvOrgGop->getCbAddr();
		piOrgBG = m_bgNewPicYuvRecoGop->getCbAddr();
		//piRecBG = m_bgNewPicYuvRecGop->getCbAddr();
		piResi = m_bgNewPicYuvUpdateResiGop->getCbAddr();
		piReco = m_bgNewPicYuvRecoGop->getCbAddr();

		uiStride = m_bgNewPicYuvOrgGop->getCStride();
		uiHeight = m_bgNewPicYuvOrgGop->getHeight() >> 1;
		uiWidth = m_bgNewPicYuvOrgGop->getWidth() >> 1;
		unit_len = TRANS_UNIT_LEN >> 1;
		uiH >>= 1;
		uiW >>= 1;
		pcCoeff = m_pcTempCoeffCb;
		TType = TEXT_CHROMA_U;
		break;
	case 2:
		piUpdateBG = m_bgNewPicYuvOrgGop->getCrAddr();
		piOrgBG = m_bgNewPicYuvRecoGop->getCrAddr();
		//piRecBG = m_bgNewPicYuvRecGop->getCrAddr();
		piResi = m_bgNewPicYuvUpdateResiGop->getCrAddr();
		piReco = m_bgNewPicYuvRecoGop->getCrAddr();

		uiStride = m_bgNewPicYuvOrgGop->getCStride();
		uiHeight = m_bgNewPicYuvOrgGop->getHeight() >> 1;
		uiWidth = m_bgNewPicYuvOrgGop->getWidth() >> 1;
		unit_len = TRANS_UNIT_LEN >> 1;
		uiH >>= 1;
		uiW >>= 1;
		pcCoeff = m_pcTempCoeffCr;
		TType = TEXT_CHROMA_V;
		break;
	default:
		break;
	}
	///* 
	// get residual用平均差值判断
	{
		Pel* pOrgBG = piOrgBG;
		Pel* pUpdateBG = piUpdateBG;
		Pel* pResi = piResi;
		Short avgDiff = 0;
		Short min = 0x7fff;

		for (UInt uiY = 0; uiY < unit_len && (uiH + uiY < uiHeight); uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len && (uiW + uiX < uiWidth); uiX++)
			{
				avgDiff += (pUpdateBG[uiH * uiStride + uiW + uiX] - pOrgBG[uiH * uiStride + uiW + uiX]);
				min = pUpdateBG[uiH * uiStride + uiW + uiX] < min ? pUpdateBG[uiH * uiStride + uiW + uiX] : min;
				pResi[uiH * uiStride + uiW + uiX] = 0;
			}

			pOrgBG += uiStride;
			pResi += uiStride;
			pUpdateBG += uiStride;
		}

		//Pel* pRecBG = piRecBG;
		pOrgBG = piOrgBG;
		pUpdateBG = piUpdateBG;
		pResi = piResi;

		if (avgDiff / (unit_len*unit_len) > 5 && min > 20)
		{
			for (UInt uiY = 0; uiY < unit_len && (uiH + uiY < uiHeight); uiY++)
			{
				for (UInt uiX = 0; uiX < unit_len && (uiW + uiX < uiWidth); uiX++)
				{
					pResi[uiH * uiStride + uiW + uiX] = (pUpdateBG[uiH * uiStride + uiW + uiX] - pOrgBG[uiH * uiStride + uiW + uiX]);
					//pResi[uiH * uiStride + uiW + uiX] = (pUpdateBG[uiH * uiStride + uiW + uiX] - pRecBG[uiH * uiStride + uiW + uiX]);
#if PRINT_UPDATEBG_RESI
					if (ocout.is_open()) {
						ocout << setiosflags(ios::right) << setw(5) << pResi[uiH * uiStride + uiW + uiX];//printf("%5d", pResi[uiH * uiStride + uiW + uiX]);
					}
#endif
				}
#if PRINT_UPDATEBG_RESI
				ocout << endl;//printf("\n");
#endif
				pOrgBG += uiStride;
				//pRecBG += uiStride;
				pResi += uiStride;
				pUpdateBG += uiStride;
			}
		}
	}
	//*/
	/*
	{
	Pel* pOrgBG = piOrgBG;
	Pel* pUpdateBG = piUpdateBG;
	Pel* pResi = piResi;
	Short diff = 0x7fff;
	Short min = 0x7fff;

	for (UInt uiY = 0; uiY < unit_len && (uiH + uiY < uiHeight); uiY++)
	{
	for (UInt uiX = 0; uiX < unit_len && (uiW + uiX < uiWidth); uiX++)
	{
	diff = (pUpdateBG[uiH * uiStride + uiW + uiX] - pOrgBG[uiH * uiStride + uiW + uiX]) < diff ?
	(pUpdateBG[uiH * uiStride + uiW + uiX] - pOrgBG[uiH * uiStride + uiW + uiX]) : diff;
	min = pUpdateBG[uiH * uiStride + uiW + uiX] < min ? pUpdateBG[uiH * uiStride + uiW + uiX] : min;
	pResi[uiH * uiStride + uiW + uiX] = 0;
	}
	pOrgBG += uiStride;
	pResi += uiStride;
	pUpdateBG += uiStride;
	}

	if (diff > 5 && min > 20)
	{
	pOrgBG = piOrgBG;
	pUpdateBG = piUpdateBG;
	pResi = piResi;

	for (UInt uiY = 0; uiY < unit_len && (uiH + uiY < uiHeight); uiY++)
	{
	for (UInt uiX = 0; uiX < unit_len && (uiW + uiX < uiWidth); uiX++)
	{
	pResi[uiH * uiStride + uiW + uiX] = (pUpdateBG[uiH * uiStride + uiW + uiX] - pOrgBG[uiH * uiStride + uiW + uiX]);
	}
	pOrgBG += uiStride;
	pResi += uiStride;
	pUpdateBG += uiStride;
	}
	}
	}
	*/
	// 全部更新
	/*
	{
	Pel* pOrgBG = piOrgBG;
	Pel* pUpdateBG = piUpdateBG;
	Pel* pResi = piResi;
	Short avgDiff = 0;

	for (UInt uiY = 0; uiY < unit_len && (uiH + uiY < uiHeight); uiY++)
	{
	for (UInt uiX = 0; uiX < unit_len && (uiW + uiX < uiWidth); uiX++)
	{
	pResi[uiH * uiStride + uiW + uiX] = (pUpdateBG[uiH * uiStride + uiW + uiX] - pOrgBG[uiH * uiStride + uiW + uiX]);
	}

	pOrgBG += uiStride;
	pResi += uiStride;
	pUpdateBG += uiStride;
	}
	}
	*/

	UInt  uiAbsSum = 0;
	m_pcTrQuantGOP->setQPforQuant(qpi, TType, 0, 0);
	m_pcTrQuantGOP->xTransformBGP(&piResi[uiH * uiStride + uiW], uiStride, pcCoeff, uiAbsSum, TType, unit_len, unit_len
#if PRINT_UPDATE_TRCOEFF
		, ocout
#endif
	);

#if ENCODE_BGP
	UInt uiMinCoeffSize = TRANS_UNIT_LEN * TRANS_UNIT_LEN;
	UInt uiLumaOffset = uiMinCoeffSize*uiAbsPartIdx;
	UInt uiChromaOffset = uiLumaOffset >> 2;
	if (TType == TEXT_LUMA)
		copyCoeff(pcCoeff, m_pcCoeffY + uiLumaOffset, unit_len, unit_len, uiAbsPartIdx);
	else if (TType == TEXT_CHROMA_U)
		copyCoeff(pcCoeff, m_pcCoeffCb + uiChromaOffset, unit_len, unit_len, uiAbsPartIdx);
	else
		copyCoeff(pcCoeff, m_pcCoeffCr + uiChromaOffset, unit_len, unit_len, uiAbsPartIdx);
#endif

	//===== inverse transform =====
	if (uiAbsSum)
	{
		Int scalingListType = 0 + g_eTTable[(Int)TType];
		assert(scalingListType < SCALING_LIST_NUM);
		m_pcTrQuantGOP->invtransformNxN(false, itransId == 0 ? TEXT_LUMA : TEXT_CHROMA, REG_DCT, &piResi[uiH * uiStride + uiW], uiStride, pcCoeff, unit_len, unit_len, scalingListType, false);
	}
	else
	{
		Pel* pResi = &piResi[uiH * uiStride + uiW];
		memset(pcCoeff, 0, sizeof(TCoeff) * unit_len * unit_len);
		for (UInt uiY = 0; uiY < unit_len; uiY++)
		{
			memset(pResi, 0, sizeof(Pel) * unit_len);
			pResi += uiStride;
		}
	}

	//===== reconstruction =====
	{
		Pel* pOrgBG = &piOrgBG[uiH * uiStride + uiW];
		//Pel* pRecBG = &piRecBG[uiH * uiStride + uiW];;
		Pel* pResi = &piResi[uiH * uiStride + uiW];
		Pel* pReco = &piReco[uiH * uiStride + uiW];
		for (UInt uiY = 0; uiY < unit_len; uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len; uiX++)
			{
				pReco[uiX] = ClipC(pOrgBG[uiX] + pResi[uiX]);
				//pReco[uiX] = (pResi[uiX] == 0) ? ClipC(pOrgBG[uiX] + pResi[uiX]) : ClipC(piRecBG[uiX] + pResi[uiX]);
			}
			pOrgBG += uiStride;
			//pRecBG += uiStride;
			pResi += uiStride;
			pReco += uiStride;
		}
	}
}
#endif

#if TRANSFORM_BGP
/*
Void TEncGOP::TransformTempBGP(Int itransId, UInt uiAbsPartIdx, UInt uiH, UInt uiW, Int qpi)
{
Pel*		piOrgBGP;
Pel*		piRecBGP;
Pel*		piResi;
Pel*		piReco;
UInt		uiStride;
UInt		uiPicHeight;
UInt		uiPicWidth;
UInt		unit_len;
TCoeff*		pcCoeff;
TextType    TType;

switch (itransId)
{
case 0:
piOrgBGP = m_bgNewPicYuvOrgGop->getLumaAddr();
piRecBGP = m_bgNewPicYuvRecGop->getLumaAddr();
piResi = m_bgNewPicYuvResiGop->getLumaAddr();
piReco = m_bgNewPicYuvTempUpdateRecoGop->getLumaAddr();

uiStride = m_bgNewPicYuvOrgGop->getStride();
uiPicHeight = m_bgNewPicYuvOrgGop->getHeight();
uiPicWidth = m_bgNewPicYuvOrgGop->getWidth();
unit_len = TRANS_UNIT_LEN;
pcCoeff = m_pcTempCoeffY;
TType = TEXT_LUMA;
break;
case 1:
piOrgBGP = m_bgNewPicYuvOrgGop->getCbAddr();
piRecBGP = m_bgNewPicYuvRecGop->getCbAddr();
piResi = m_bgNewPicYuvResiGop->getCbAddr();
piReco = m_bgNewPicYuvTempUpdateRecoGop->getCbAddr();

uiStride = m_bgNewPicYuvOrgGop->getCStride();
uiPicHeight = m_bgNewPicYuvOrgGop->getHeight() >> 1;
uiPicWidth = m_bgNewPicYuvOrgGop->getWidth() >> 1;
unit_len = TRANS_UNIT_LEN >> 1;
uiH >>= 1;
uiW >>= 1;
pcCoeff = m_pcTempCoeffCb;
TType = TEXT_CHROMA_U;
break;
case 2:
piOrgBGP = m_bgNewPicYuvOrgGop->getCrAddr();
piRecBGP = m_bgNewPicYuvRecGop->getCrAddr();
piResi = m_bgNewPicYuvResiGop->getCrAddr();
piReco = m_bgNewPicYuvTempUpdateRecoGop->getCrAddr();

uiStride = m_bgNewPicYuvOrgGop->getCStride();
uiPicHeight = m_bgNewPicYuvOrgGop->getHeight() >> 1;
uiPicWidth = m_bgNewPicYuvOrgGop->getWidth() >> 1;
unit_len = TRANS_UNIT_LEN >> 1;
uiH >>= 1;
uiW >>= 1;
pcCoeff = m_pcTempCoeffCr;
TType = TEXT_CHROMA_V;
break;
default:
break;
}
// get residual
{
Pel* pOrgBGP = piOrgBGP;
Pel* pRecBGP = piRecBGP;
Pel* pResi = piResi;

for (UInt uiY = 0; uiY < unit_len && (uiH + uiY < uiPicHeight); uiY++)
{
for (UInt uiX = 0; uiX < unit_len && (uiW + uiX < uiPicWidth); uiX++)
{
//量化残差---
pResi[uiH * uiStride + uiW + uiX] = (pOrgBGP[uiH * uiStride + uiW + uiX] - pRecBGP[uiH * uiStride + uiW + uiX]);
}

pOrgBGP += uiStride;
pResi += uiStride;
pRecBGP += uiStride;
}
}

UInt  uiAbsSum = 0;
m_pcTrQuantGOP->setQPforQuant(qpi, TType, 0, 0);
m_pcTrQuantGOP->xTransformBGP(&piResi[uiH * uiStride + uiW], uiStride, pcCoeff, uiAbsSum, TType, unit_len, unit_len);


//===== inverse transform =====
if (uiAbsSum)
{
Int scalingListType = 0 + g_eTTable[(Int)TType];
assert(scalingListType < SCALING_LIST_NUM);
m_pcTrQuantGOP->invtransformNxN(false, itransId == 0 ? TEXT_LUMA : TEXT_CHROMA, REG_DCT, &piResi[uiH * uiStride + uiW], uiStride, pcCoeff, unit_len, unit_len, scalingListType, false);
}
else
{
Pel* pResi = &piResi[uiH * uiStride + uiW];
memset(pcCoeff, 0, sizeof(TCoeff) * unit_len * unit_len);
for (UInt uiY = 0; uiY < unit_len; uiY++)
{
memset(pResi, 0, sizeof(Pel) * unit_len);
pResi += uiStride;
}
}

//===== reconstruction =====
{
Pel* pRecBGP = &piRecBGP[uiH * uiStride + uiW];
Pel* pResi = &piResi[uiH * uiStride + uiW];
Pel* pReco = &piReco[uiH * uiStride + uiW];
for (UInt uiY = 0; uiY < unit_len; uiY++)
{
for (UInt uiX = 0; uiX < unit_len; uiX++)
{
pReco[uiX] = ClipC(pRecBGP[uiX] + pResi[uiX]);

}
pRecBGP += uiStride;
pResi += uiStride;
pReco += uiStride;
}
}

}*/
#endif









// ====================================================================================================================
// Public member functions
// ====================================================================================================================
Void EncGOP::compressGOP( Int iPOCLast, Int iNumPicRcvd, PicList& rcListPic,
                          std::list<PelUnitBuf*>& rcListPicYuvRecOut,
                          Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE )
{
  // TODO: Split this function up.
#if GENERATE_OrgBG_PIC
	Picture* bg_NewPicYuvOrgSli = getbgNewPicYuvOrgGop();
	m_pcSliceEncoder->setNewPicYuvOrgSli(bg_NewPicYuvOrgSli);
#endif
#if GENERATE_BG_PIC
	Picture* bg_NewPicYuvRecSli = getbgNewPicYuvRecGop();
	m_pcSliceEncoder->setNewPicYuvRecSli(bg_NewPicYuvRecSli);
#endif
  Picture*        pcPic = NULL;
  Slice*      pcSlice;
  OutputBitstream  *pcBitstreamRedirect;
  pcBitstreamRedirect = new OutputBitstream;
  AccessUnit::iterator  itLocationToPushSliceHeaderNALU; // used to store location where NALU containing slice header is to be inserted

  xInitGOP( iPOCLast, iNumPicRcvd, isField );

  m_iNumPicCoded = 0;
  SEIMessages leadingSeiMessages;
  SEIMessages nestedSeiMessages;
  SEIMessages duInfoSeiMessages;
  SEIMessages trailingSeiMessages;
  std::deque<DUData> duData;
  SEIDecodingUnitInfo decodingUnitInfoSEI;

  EfficientFieldIRAPMapping effFieldIRAPMap;
  if (m_pcCfg->getEfficientFieldIRAPEnabled())
  {
    effFieldIRAPMap.initialize(isField, m_iGopSize, iPOCLast, iNumPicRcvd, m_iLastIDR, this, m_pcCfg);
  }

  // reset flag indicating whether pictures have been encoded
  for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
  {
    m_pcCfg->setEncodedFlag(iGOPid, false);
  }

  for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
  {

#if OrgBG_BLOCK_SUBSTITUTION
	  m_pcNablaY = NULL;
	  m_pcNablaCb = NULL;
	  m_pcNablaCr = NULL;
	  m_pcNablaY = new Double[BLOCK_LEN * BLOCK_LEN];
	  m_pcNablaCb = new Double[(BLOCK_LEN >> 1) * (BLOCK_LEN >> 1)];
	  m_pcNablaCr = new Double[(BLOCK_LEN >> 1) * (BLOCK_LEN >> 1)];
	  memset(m_pcNablaY, 0, BLOCK_LEN * BLOCK_LEN * sizeof(Double));  //填充m_pcNablaY从当前位置到BLOCK_LEN * BLOCK_LEN * sizeof(Double)为0
	  memset(m_pcNablaCb, 0, (BLOCK_LEN >> 1) * (BLOCK_LEN >> 1) * sizeof(Double));
	  memset(m_pcNablaCr, 0, (BLOCK_LEN >> 1) * (BLOCK_LEN >> 1) * sizeof(Double));
#endif
    if (m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      iGOPid=effFieldIRAPMap.adjustGOPid(iGOPid);
    }

    //-- For time output for each slice
    auto beforeTime = std::chrono::steady_clock::now();

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
    UInt uiColDir = calculateCollocatedFromL1Flag(m_pcCfg, iGOPid, m_iGopSize);
#endif

    /////////////////////////////////////////////////////////////////////////////////////////////////// Initial to start encoding
    Int iTimeOffset;
    Int pocCurr;

    if(iPOCLast == 0) //case first frame or first top field
    {
      pocCurr=0;
      iTimeOffset = 1;
    }
    else if(iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
    {
      pocCurr = 1;
      iTimeOffset = 1;
    }
    else
    {
      pocCurr = iPOCLast - iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - ((isField && m_iGopSize>1) ? 1:0);
      iTimeOffset = m_pcCfg->getGOPEntry(iGOPid).m_POC;
    }

    if(pocCurr>=m_pcCfg->getFramesToBeEncoded())
    {
      if (m_pcCfg->getEfficientFieldIRAPEnabled())
      {
        iGOPid=effFieldIRAPMap.restoreGOPid(iGOPid);
      }
      continue;
    }

    if( getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_N_LP )
    {
      m_iLastIDR = pocCurr;
    }

    // start a new access unit: create an entry in the list of output access units
    AccessUnit accessUnit;
    xGetBuffer( rcListPic, rcListPicYuvRecOut,
                iNumPicRcvd, iTimeOffset, pcPic, pocCurr, isField ); //此时pic内有

    // th this is a hot fix for the choma qp control
    if( m_pcEncLib->getWCGChromaQPControl().isEnabled() && m_pcEncLib->getSwitchPOC() != -1 )
    {
      static int usePPS = 0; /* TODO: MT */
      if( pocCurr == m_pcEncLib->getSwitchPOC() )
      {
        usePPS = 1;
      }
      const PPS *pPPS = m_pcEncLib->getPPS(usePPS);
      // replace the pps with a more appropriated one
      pcPic->cs->pps = pPPS;
    }

#if ENABLE_SPLIT_PARALLELISM && ENABLE_WPP_PARALLELISM
    pcPic->scheduler.init( pcPic->cs->pcv->heightInCtus, pcPic->cs->pcv->widthInCtus, m_pcCfg->getNumWppThreads(), m_pcCfg->getNumWppExtraLines(), m_pcCfg->getNumSplitThreads() );
#elif ENABLE_SPLIT_PARALLELISM
    pcPic->scheduler.init( pcPic->cs->pcv->heightInCtus, pcPic->cs->pcv->widthInCtus, 1                          , 0                             , m_pcCfg->getNumSplitThreads() );
#elif ENABLE_WPP_PARALLELISM
    pcPic->scheduler.init( pcPic->cs->pcv->heightInCtus, pcPic->cs->pcv->widthInCtus, m_pcCfg->getNumWppThreads(), m_pcCfg->getNumWppExtraLines(), 1                             );
#endif
    pcPic->createTempBuffers( pcPic->cs->pps->pcv->maxCUWidth );
    pcPic->cs->createCoeffs();

    //  Slice data initialization
    pcPic->clearSliceBuffer();
    pcPic->allocateNewSlice();
    m_pcSliceEncoder->setSliceSegmentIdx(0);

#if ADJUST_QP

	if (afterbg && (pcPic->getPOC() - 1) % 4 == 0 && pcPic->getPOC() != 1)    //
	{

		Pel* piBac;
		Pel* piOrg;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;
		Int num = 0;   // bg pixels number

		piBac = m_bgNewBlocksOrgGop->getOrigBuf().Y().bufAt(0,0);
		piOrg = pcPic->getOrigBuf().Y().bufAt(0,0);
		uiStride = pcPic->getOrigBuf().Y().stride;
		uiPicHeight = pcPic->getOrigBuf().Y().height;
		uiPicWidth = pcPic->getOrigBuf().Y().width;

		for (Int p = 0; p < uiPicHeight / 4; p++)
		{
			for (Int q = 0; q < uiPicWidth / 4; q++)
			{
				Int   Diff =
					abs(piOrg[4 * p * uiStride + 4 * q] - piBac[4 * p * uiStride + 4 * q])
					+ abs(piOrg[(4 * p + 1) * uiStride + 4 * q + 1] - piBac[(4 * p + 1) * uiStride + 4 * q + 1])
					+ abs(piOrg[(4 * p + 2) * uiStride + 4 * q + 2] - piBac[(4 * p + 2) * uiStride + 4 * q + 2])
					+ abs(piOrg[(4 * p + 3) * uiStride + 4 * q + 3] - piBac[(4 * p + 3) * uiStride + 4 * q + 3]);
				if (Diff <= 80)
				{
					num++;
				}
			}
		}
		Double bgblock = (double)(num * 16);
		Double picSize = (double)(uiPicHeight * uiPicWidth);

		if (bgblock / picSize > 0.6)
		{
			m_BGQPflag = true;
			cout << "true" << endl;
		}
		else
		{
			m_BGQPflag = false;
			cout << "false" << endl;
		}

	}
#endif

    m_pcSliceEncoder->initEncSlice ( pcPic, iPOCLast, pocCurr, iGOPid, pcSlice, isField 
#if ADJUST_QP
		,m_BGQPflag

#endif // ADJUST_QP
	);

    DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "poc", pocCurr ) ) );
    DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "final", 0 ) ) );

#if !SHARP_LUMA_DELTA_QP
    //Set Frame/Field coding
    pcPic->fieldPic = isField;
#endif

    pcSlice->setLastIDR(m_iLastIDR);
#if HEVC_DEPENDENT_SLICES
    pcSlice->setSliceSegmentIdx(0);
#endif
    pcSlice->setIndependentSliceIdx(0);
    //set default slice level flag to the same as SPS level flag
    pcSlice->setLFCrossSliceBoundaryFlag(  pcSlice->getPPS()->getLoopFilterAcrossSlicesEnabledFlag()  );

    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='P')
    {
      pcSlice->setSliceType(P_SLICE);
    }
    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='I')
    {
      pcSlice->setSliceType(I_SLICE);
    }

    // Set the nal unit type
    pcSlice->setNalUnitType(getNalUnitType(pocCurr, m_iLastIDR, isField));
    if(pcSlice->getTemporalLayerNonReferenceFlag())
    {
      if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_R &&
          !(m_iGopSize == 1 && pcSlice->getSliceType() == I_SLICE))
        // Add this condition to avoid POC issues with encoder_intra_main.cfg configuration (see #1127 in bug tracker)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TRAIL_N);
      }
      if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RADL_R)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RADL_N);
      }
      if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RASL_R)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RASL_N);
      }
    }

    if (m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
      {
        m_associatedIRAPType = pcSlice->getNalUnitType();
        m_associatedIRAPPOC = pocCurr;
      }
      pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
      pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
    }

    pcSlice->decodingRefreshMarking(m_pocCRA, m_bRefreshPending, rcListPic, m_pcCfg->getEfficientFieldIRAPEnabled());
    m_pcEncLib->selectReferencePictureSet(pcSlice, pocCurr, iGOPid);
    if (!m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
      {
        m_associatedIRAPType = pcSlice->getNalUnitType();
        m_associatedIRAPPOC = pocCurr;
      }
      pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
      pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
    }

    if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), false, m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3) != 0) || (pcSlice->isIRAP())
      || (m_pcCfg->getEfficientFieldIRAPEnabled() && isField && pcSlice->getAssociatedIRAPType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getAssociatedIRAPType() <= NAL_UNIT_CODED_SLICE_CRA && pcSlice->getAssociatedIRAPPOC() == pcSlice->getPOC()+1)
      )
    {
      pcSlice->createExplicitReferencePictureSetFromReference(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP(), m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3, m_pcCfg->getEfficientFieldIRAPEnabled());
    }

    pcSlice->applyReferencePictureSet(rcListPic, pcSlice->getRPS());

    if(pcSlice->getTLayer() > 0
      &&  !( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N     // Check if not a leading picture
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R )
        )
    {
      if(pcSlice->isTemporalLayerSwitchingPoint(rcListPic) || pcSlice->getSPS()->getTemporalIdNestingFlag())
      {
        if(pcSlice->getTemporalLayerNonReferenceFlag())
        {
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_N);
        }
        else
        {
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_R);
        }
      }
      else if(pcSlice->isStepwiseTemporalLayerSwitchingPointCandidate(rcListPic))
      {
        Bool isSTSA=true;
        for(Int ii=iGOPid+1;(ii<m_pcCfg->getGOPSize() && isSTSA==true);ii++)
        {
          Int lTid= m_pcCfg->getGOPEntry(ii).m_temporalId;
          if(lTid==pcSlice->getTLayer())
          {
            const ReferencePictureSet* nRPS = pcSlice->getSPS()->getRPSList()->getReferencePictureSet(ii);
            for(Int jj=0;jj<nRPS->getNumberOfPictures();jj++)
            {
              if(nRPS->getUsed(jj))
              {
                Int tPoc=m_pcCfg->getGOPEntry(ii).m_POC+nRPS->getDeltaPOC(jj);
                Int kk=0;
                for(kk=0;kk<m_pcCfg->getGOPSize();kk++)
                {
                  if(m_pcCfg->getGOPEntry(kk).m_POC==tPoc)
                  {
                    break;
                  }
                }
                Int tTid=m_pcCfg->getGOPEntry(kk).m_temporalId;
                if(tTid >= pcSlice->getTLayer())
                {
                  isSTSA=false;
                  break;
                }
              }
            }
          }
        }
        if(isSTSA==true)
        {
          if(pcSlice->getTemporalLayerNonReferenceFlag())
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_N);
          }
          else
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_R);
          }
        }
      }
    }
    arrangeLongtermPicturesInRPS(pcSlice, rcListPic);
    RefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
    refPicListModification->setRefPicListModificationFlagL0(0);
    refPicListModification->setRefPicListModificationFlagL1(0);
    pcSlice->setNumRefIdx(REF_PIC_LIST_0,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));
    pcSlice->setNumRefIdx(REF_PIC_LIST_1,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));
	
    //  Set reference list
	Int SetRefPoc = -999;
#if BG_REFERENCE_SUBSTITUTION
	if (afterbg//&&pcSlice->getPOC()!= BGPICPOC
#if israndom
		&&pcSlice->getPOC()%16!=0   //randomaccess.cfg 16
#endif
		)
	{
#if addbg3ref
		pcSlice->setNumRefIdx(REF_PIC_LIST_0, min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive, pcSlice->getRPS()->getNumberOfPictures())+1);
		pcSlice->setNumRefIdx(REF_PIC_LIST_1, min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive, pcSlice->getRPS()->getNumberOfPictures())+1);
		//cout << "REF_PIC_LIST_0   " << pcSlice->getNumRefIdx(REF_PIC_LIST_0) << " REF_PIC_LIST_11" << pcSlice->getNumRefIdx(REF_PIC_LIST_1) << endl;
#endif
#if BLOCK_ENCODE
		//cout << "inref" << endl;
		//if (pcPic->getPOC() <= 50)  //<=50帧时 替换参考块    50帧时bg编码结束，替换参考帧
		{
			pcSlice->setRefPicListaddbgBlock(rcListPic, m_bgNewPicYuvRecoGop, m_rcPicYuvTempGop, SetRefPoc, BgBlock);
		}

		//else
		{
			//pcSlice->setRefPicListaddbg(rcListPic, m_bgNewPicYuvRecoGop, m_rcPicYuvTempGop,SetRefPoc);
		}
#else
		pcSlice->setRefPicListaddbg(rcListPic, m_bgNewPicYuvRecoGop, m_rcPicYuvTempGop, SetRefPoc);
#endif
		/*{
			ofstream inencode;
			inencode.open("D://2//Reco.txt");
			for (Int compId = 0; compId < 3; compId++)
			{
				Pel* piOrg;
				UInt uiStride;
				UInt uiPicHeight;
				UInt uiPicWidth;
				switch (compId)
				{
				case 0:
					piOrg = m_bgNewPicYuvRecoGop->getRecoBuf().Y().bufAt(0,0);

					uiStride = m_bgNewPicYuvRecoGop->getRecoBuf().Y().stride;
					uiPicHeight = m_bgNewPicYuvRecoGop->getRecoBuf().Y().height;
					uiPicWidth = m_bgNewPicYuvRecoGop->getRecoBuf().Y().width;
					break;
				case 1:
					piOrg = m_bgNewPicYuvRecoGop->getRecoBuf().Cb().bufAt(0,0);

					uiStride = m_bgNewPicYuvRecoGop->getRecoBuf().Cb().stride;
					uiPicHeight = m_bgNewPicYuvRecoGop->getRecoBuf().Cb().height;
					uiPicWidth = m_bgNewPicYuvRecoGop->getRecoBuf().Cb().width;
					break;
				case 2:
					piOrg = m_bgNewPicYuvRecoGop->getRecoBuf().Cr().bufAt(0,0);

					uiStride = m_bgNewPicYuvRecoGop->getRecoBuf().Cr().stride;
					uiPicHeight = m_bgNewPicYuvRecoGop->getRecoBuf().Cr().height;
					uiPicWidth = m_bgNewPicYuvRecoGop->getRecoBuf().Cr().width;
					break;
				default:
					break;
				}
				for (Int i = 0; i < uiPicHeight; i++)
				{
					for (Int j = 0; j < uiPicWidth; j++)
					{
						inencode << piOrg[i * uiStride + j] << " ";
					}
				}
			}
		}*/


	}
	else
	{
		pcSlice->setRefPicList(rcListPic);
	}
	/*if (0&&isselect&&pcPic->getPOC()>7)
	{
		cout << "add Rec:" << endl;
		pcSlice->setRefPicListaddbgBlockRec(rcListPic, m_bgNewBlocksOrgGop, m_rcPicYuvTempGop, SetRefPoc, BgBlock);
	}*/
#else
	pcSlice->setRefPicList(rcListPic);
#endif // BG_REFERENCE_SUBSTITUTION

    if( m_pcCfg->getUseAMaxBT() )
    {
      if( !pcSlice->isIntra() )
      {
        Int refLayer = pcSlice->getDepth();
        if( refLayer > 9 ) refLayer = 9; // Max layer is 10

        if( m_bInitAMaxBT && pcSlice->getPOC() > m_uiPrevISlicePOC )
        {
          ::memset( m_uiBlkSize, 0, sizeof( m_uiBlkSize ) );
          ::memset( m_uiNumBlk,  0, sizeof( m_uiNumBlk ) );
          m_bInitAMaxBT = false;
        }

        if( refLayer >= 0 && m_uiNumBlk[refLayer] != 0 )
        {
          Double dBlkSize = sqrt( ( Double ) m_uiBlkSize[refLayer] / m_uiNumBlk[refLayer] );
          if( dBlkSize < AMAXBT_TH32 )
          {
            pcSlice->setMaxBTSize( 32 > MAX_BT_SIZE_INTER ? MAX_BT_SIZE_INTER : 32 );
          }
          else if( dBlkSize < AMAXBT_TH64 )
          {
            pcSlice->setMaxBTSize( 64 > MAX_BT_SIZE_INTER ? MAX_BT_SIZE_INTER : 64 );
          }
          else
          {
            pcSlice->setMaxBTSize( 128 > MAX_BT_SIZE_INTER ? MAX_BT_SIZE_INTER : 128 );
          }

          m_uiBlkSize[refLayer] = 0;
          m_uiNumBlk [refLayer] = 0;
        }
      }
      else
      {
        if( m_bInitAMaxBT )
        {
          ::memset( m_uiBlkSize, 0, sizeof( m_uiBlkSize ) );
          ::memset( m_uiNumBlk,  0, sizeof( m_uiNumBlk ) );
        }

        m_uiPrevISlicePOC = pcSlice->getPOC();
        m_bInitAMaxBT = true;
      }
    }

    //  Slice info. refinement
    if ( (pcSlice->getSliceType() == B_SLICE) && (pcSlice->getNumRefIdx(REF_PIC_LIST_1) == 0) )
    {
      pcSlice->setSliceType ( P_SLICE );
    }

    xUpdateRasInit( pcSlice );

    // Do decoding refresh marking if any
#if COM16_C806_ALF_TEMPPRED_NUM
    if ( pcSlice->getPendingRasInit() || pcSlice->isIDRorBLA() )
    {
      m_pcALF->refreshAlfTempPred();
    }
#endif

    if ( pcSlice->getPendingRasInit() )
    {
      // this ensures that independently encoded bitstream chunks can be combined to bit-equal
      pcSlice->setEncCABACTableIdx( pcSlice->getSliceType() );
    }
    else
    {
      pcSlice->setEncCABACTableIdx( m_pcSliceEncoder->getEncCABACTableIdx() );
    }

    if (pcSlice->getSliceType() == B_SLICE)
    {
#if X0038_LAMBDA_FROM_QP_CAPABILITY
      const UInt uiColFromL0 = calculateCollocatedFromL0Flag(pcSlice);
      pcSlice->setColFromL0Flag(uiColFromL0);
#else
      pcSlice->setColFromL0Flag(1-uiColDir);
#endif
      Bool bLowDelay = true;
      Int  iCurrPOC  = pcSlice->getPOC();
      Int iRefIdx = 0;

      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_0) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }
      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }

      pcSlice->setCheckLDC(bLowDelay);
    }
    else
    {
      pcSlice->setCheckLDC(true);
    }

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
    uiColDir = 1-uiColDir;
#endif

    //-------------------------------------------------------------
    pcSlice->setRefPOCList();

    pcSlice->setList1IdxToList0Idx();

    if (m_pcEncLib->getTMVPModeId() == 2)
    {
      if (iGOPid == 0) // first picture in SOP (i.e. forward B)
      {
        pcSlice->setEnableTMVPFlag(0);
      }
      else
      {
        // Note: pcSlice->getColFromL0Flag() is assumed to be always 0 and getcolRefIdx() is always 0.
        pcSlice->setEnableTMVPFlag(1);
      }
    }
    else if (m_pcEncLib->getTMVPModeId() == 1)
    {
      pcSlice->setEnableTMVPFlag(1);
    }
    else
    {
      pcSlice->setEnableTMVPFlag(0);
    }

    // set adaptive search range for non-intra-slices
    if (m_pcCfg->getUseASR() && pcSlice->getSliceType()!=I_SLICE)
    {
      m_pcSliceEncoder->setSearchRange(pcSlice);
    }

    Bool bGPBcheck=false;
    if ( pcSlice->getSliceType() == B_SLICE)
    {
      if ( pcSlice->getNumRefIdx(RefPicList( 0 ) ) == pcSlice->getNumRefIdx(RefPicList( 1 ) ) )
      {
        bGPBcheck=true;
        Int i;
        for ( i=0; i < pcSlice->getNumRefIdx(RefPicList( 1 ) ); i++ )
        {
          if ( pcSlice->getRefPOC(RefPicList(1), i) != pcSlice->getRefPOC(RefPicList(0), i) )
          {
            bGPBcheck=false;
            break;
          }
        }
      }
    }
    if(bGPBcheck)
    {
      pcSlice->setMvdL1ZeroFlag(true);
    }
    else
    {
      pcSlice->setMvdL1ZeroFlag(false);
    }
#if HEVC_DEPENDENT_SLICES
    pcPic->slices[pcSlice->getSliceSegmentIdx()]->setMvdL1ZeroFlag(pcSlice->getMvdL1ZeroFlag());
#endif


    Double lambda            = 0.0;
    Int actualHeadBits       = 0;
    Int actualTotalBits      = 0;
    Int estimatedBits        = 0;
    Int tmpBitsBeforeWriting = 0;
    if ( m_pcCfg->getUseRateCtrl() ) // TODO: does this work with multiple slices and slice-segments?
    {
      Int frameLevel = m_pcRateCtrl->getRCSeq()->getGOPID2Level( iGOPid );
      if ( pcPic->slices[0]->getSliceType() == I_SLICE )
      {
        frameLevel = 0;
      }
      m_pcRateCtrl->initRCPic( frameLevel );
      estimatedBits = m_pcRateCtrl->getRCPic()->getTargetBits();

#if U0132_TARGET_BITS_SATURATION
      if (m_pcRateCtrl->getCpbSaturationEnabled() && frameLevel != 0)
      {
        Int estimatedCpbFullness = m_pcRateCtrl->getCpbState() + m_pcRateCtrl->getBufferingRate();

        // prevent overflow
        if (estimatedCpbFullness - estimatedBits > (Int)(m_pcRateCtrl->getCpbSize()*0.9f))
        {
          estimatedBits = estimatedCpbFullness - (Int)(m_pcRateCtrl->getCpbSize()*0.9f);
        }

        estimatedCpbFullness -= m_pcRateCtrl->getBufferingRate();
        // prevent underflow
#if V0078_ADAPTIVE_LOWER_BOUND
        if (estimatedCpbFullness - estimatedBits < m_pcRateCtrl->getRCPic()->getLowerBound())
        {
          estimatedBits = max(200, estimatedCpbFullness - m_pcRateCtrl->getRCPic()->getLowerBound());
        }
#else
        if (estimatedCpbFullness - estimatedBits < (Int)(m_pcRateCtrl->getCpbSize()*0.1f))
        {
          estimatedBits = max(200, estimatedCpbFullness - (Int)(m_pcRateCtrl->getCpbSize()*0.1f));
        }
#endif

        m_pcRateCtrl->getRCPic()->setTargetBits(estimatedBits);
      }
#endif

      Int sliceQP = m_pcCfg->getInitialQP();
      if ( ( pcSlice->getPOC() == 0 && m_pcCfg->getInitialQP() > 0 ) || ( frameLevel == 0 && m_pcCfg->getForceIntraQP() ) ) // QP is specified
      {
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)NumberBFrames );
        Double dQPFactor     = 0.57*dLambda_scale;
        Int    SHIFT_QP      = 12;
        Int    bitdepth_luma_qp_scale = 0;
        Double qp_temp = (Double) sliceQP + bitdepth_luma_qp_scale - SHIFT_QP;
        lambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
      }
      else if ( frameLevel == 0 )   // intra case, but use the model
      {
        m_pcSliceEncoder->calCostSliceI(pcPic); // TODO: This only analyses the first slice segment - what about the others?

        if ( m_pcCfg->getIntraPeriod() != 1 )   // do not refine allocated bits for all intra case
        {
          Int bits = m_pcRateCtrl->getRCSeq()->getLeftAverageBits();
          bits = m_pcRateCtrl->getRCPic()->getRefineBitsForIntra( bits );

#if U0132_TARGET_BITS_SATURATION
          if (m_pcRateCtrl->getCpbSaturationEnabled() )
          {
            Int estimatedCpbFullness = m_pcRateCtrl->getCpbState() + m_pcRateCtrl->getBufferingRate();

            // prevent overflow
            if (estimatedCpbFullness - bits > (Int)(m_pcRateCtrl->getCpbSize()*0.9f))
            {
              bits = estimatedCpbFullness - (Int)(m_pcRateCtrl->getCpbSize()*0.9f);
            }

            estimatedCpbFullness -= m_pcRateCtrl->getBufferingRate();
            // prevent underflow
#if V0078_ADAPTIVE_LOWER_BOUND
            if (estimatedCpbFullness - bits < m_pcRateCtrl->getRCPic()->getLowerBound())
            {
              bits = estimatedCpbFullness - m_pcRateCtrl->getRCPic()->getLowerBound();
            }
#else
            if (estimatedCpbFullness - bits < (Int)(m_pcRateCtrl->getCpbSize()*0.1f))
            {
              bits = estimatedCpbFullness - (Int)(m_pcRateCtrl->getCpbSize()*0.1f);
            }
#endif
          }
#endif

          if ( bits < 200 )
          {
            bits = 200;
          }
          m_pcRateCtrl->getRCPic()->setTargetBits( bits );
        }

        list<EncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
        m_pcRateCtrl->getRCPic()->getLCUInitTargetBits();
        lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->getSliceType());
        sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
      }
      else    // normal case
      {
        list<EncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
        lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->getSliceType());
        sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
      }

      sliceQP = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, sliceQP );
      m_pcRateCtrl->getRCPic()->setPicEstQP( sliceQP );

      m_pcSliceEncoder->resetQP( pcPic, sliceQP, lambda );
    }

	UInt uiNumSliceSegments = 1;

	{
		pcSlice->setDefaultClpRng(*pcSlice->getSPS());
	}

	// Allocate some coders, now the number of tiles are known.
	const UInt numberOfCtusInFrame = pcPic->cs->pcv->sizeInCtus;
#if HEVC_TILES_WPP
	const Int numSubstreamsColumns = (pcSlice->getPPS()->getNumTileColumnsMinus1() + 1);
	const Int numSubstreamRows = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag() ? pcPic->cs->pcv->heightInCtus : (pcSlice->getPPS()->getNumTileRowsMinus1() + 1);
	const Int numSubstreams = numSubstreamRows * numSubstreamsColumns;
#else
	const Int numSubstreams = 1;
#endif
	std::vector<OutputBitstream> substreamsOut(numSubstreams);

#if ENABLE_QPA
	pcPic->m_uEnerHpCtu.resize(numberOfCtusInFrame);
	pcPic->m_iOffsetCtu.resize(numberOfCtusInFrame);
#endif
	if (pcSlice->getSPS()->getUseSAO())
	{
		pcPic->resizeSAO(numberOfCtusInFrame, 0);
		pcPic->resizeSAO(numberOfCtusInFrame, 1);
	}

	//生成背景帧org 在 编码前面  生成完成直接编码
#if BBB
#if HIERARCHY_GENETATE_OrgBGP
	if (pcPic->getPOC() != 0)
	{

#if israndom  //32时没有参考帧
		if (pcPic->getPOC() % 32 != 0)
		{
#endif // israndom

			const Int  iWidth = pcPic->getOrigBuf().Y().width;
			const Int  iHeight = pcPic->getOrigBuf().Y().height;
#if PRINT_OrgDIFF
			ofstream os("test.txt", ofstream::app);
#endif
			for (Int i = 0; i < iHeight; i += UNIT_LEN)
			{
				for (Int j = 0; j < iWidth; j += UNIT_LEN)
				{
					Int level = 0; //分几级
					Bool divflag = false; //是否再分
					CompDiffOrg(j, i, pcPic, level, divflag
#if PRINT_OrgDIFF
						, os
#endif
					);


				}
#if PRINT_OrgDIFF
				os << endl;
#endif
			}


#if PRINT_OrgDIFF
			os.close();
#endif
#if israndom
		}
#endif // israndom


	}
	else
	{
		Pel* piBac;
		Pel* piOrg;

		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;


		for (Int compId = 0; compId < 3; compId++)
		{
			switch (compId)
			{
			case 0:
				piBac = m_bgNewPicYuvOrgGop->getOrigBuf().Y().bufAt(0, 0);
				piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);

				uiStride = pcPic->getOrigBuf().Y().stride;
				uiPicHeight = pcPic->getOrigBuf().Y().height;
				uiPicWidth = pcPic->getOrigBuf().Y().width;
				break;
			case 1:
				piBac = m_bgNewPicYuvOrgGop->getOrigBuf().Cb().bufAt(0, 0);
				piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);

				uiStride = pcPic->getOrigBuf().Cb().stride;
				uiPicHeight = pcPic->getOrigBuf().Cb().height;
				uiPicWidth = pcPic->getOrigBuf().Cb().width;
				break;
			case 2:
				piBac = m_bgNewPicYuvOrgGop->getOrigBuf().Cr().bufAt(0, 0);
				piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);

				uiStride = pcPic->getOrigBuf().Cr().stride;
				uiPicHeight = pcPic->getOrigBuf().Cr().height;
				uiPicWidth = pcPic->getOrigBuf().Cr().width;
				break;
			default:
				break;
			}

			for (Int unit_i = 0; unit_i < uiPicHeight; unit_i++)
			{
				for (Int unit_j = 0; unit_j < uiPicWidth; unit_j++)
				{
					piBac[unit_j] = 0;   //-------------315初始为0
				}
				piOrg += uiStride;
				piBac += uiStride;
			}
		}
	}
#endif

#if OrgBG_BLOCK_SUBSTITUTION
	if (pcPic->getPOC() != 0)
	{//空间相关性
#if israndom
		if (pcPic->getPOC() % 32 != 0)
		{
#endif // israndom

			const UInt  uiWidth = pcPic->getOrigBuf().Y().width;
			const UInt  uiHeight = pcPic->getOrigBuf().Y().height;
#if PRINT_FCVALUE
			ofstream ocout("test.txt", ofstream::app);
#endif
			for (UInt uiH = 0; uiH < uiHeight; uiH += BLOCK_LEN)
			{
				for (UInt uiW = 0; uiW < uiWidth; uiW += BLOCK_LEN)
				{
					Double Fc[3];
					for (Int transId = 0; transId < 3; transId++)
					{
						Fc[transId] = CompNablaOrg(transId, uiH, uiW, pcPic);
					}
					Double dNabla = Fc[0] + Fc[1] / 4 + Fc[2] / 4;
#if PRINT_FCVALUE
					if (ocout.is_open())
					{
						if (uiW % 128 == 0 && uiW != 0)
							ocout << endl;
						ocout << setiosflags(ios::right) << setw(10) << dNabla;
					}
#endif
					if (dNabla < 1.5)   // if Nabla < Omegaba then substitute
					{
						//cout << "in orgblock <1.5" << endl;

						pcPic->Copy2OrgBackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic); //m_bgNewPicYuvOrgGop=pcpic

					}
					else if (/*Omegaba < Nabla < Omega*/ dNabla >= 1.5 && dNabla < 3.5 && uiH != 0 && uiH != (uiHeight - BLOCK_LEN) && uiW != 0 && uiW != (uiWidth - BLOCK_LEN))  // Omegaba < Nabla < Omega
					{
						Double DiffOrg = 0;
						Double DiffBG = 0;

						DiffOrg += CompBlockDiff(uiH, uiW, pcPic, pcPic);
						DiffBG += CompBlockDiff(uiH, uiW, m_bgNewPicYuvOrgGop, pcPic);
						if (DiffOrg < DiffBG)
						{
							//cout << "in orgblock DiffOrg < DiffBG" << endl;							
							pcPic->Copy2OrgBackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic);
						}
					}

				}
#if PRINT_FCVALUE
				ocout << endl;
#endif
			}
#if PRINT_FCVALUE
			ocout.close();
#endif
#if israndom
		}
#endif // israndom

	}
	/*else
	{
	Pel* piBac;
	Pel* piOrg;

	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;

	for (Int compId = 0; compId < 3; compId++)
	{
	switch (compId)
	{
	case 0:
	piBac = m_bgNewPicYuvOrgGop->getOrigBuf().Y().bufAt(0,0);
	piOrg = pcPic->getOrigBuf().Y().bufAt(0,0);

	uiStride = pcPic->getOrigBuf().Y().stride;
	uiPicHeight = pcPic->getOrigBuf().Y().height;
	uiPicWidth = pcPic->getOrigBuf().Y().width;
	break;
	case 1:
	piBac = m_bgNewPicYuvOrgGop->getOrigBuf().Cb().bufAt(0, 0);
	piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);

	uiStride = pcPic->getOrigBuf().Cb().stride;
	uiPicHeight = pcPic->getOrigBuf().Cb().height;
	uiPicWidth = pcPic->getOrigBuf().Cb().width;
	break;
	case 2:
	piBac = m_bgNewPicYuvOrgGop->getOrigBuf().Cr().bufAt(0, 0);
	piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);

	uiStride = pcPic->getOrigBuf().Cr().stride;
	uiPicHeight = pcPic->getOrigBuf().Cr().height;
	uiPicWidth = pcPic->getOrigBuf().Cr().width;
	break;
	default:
	break;
	}
	for (Int unit_i = 0; unit_i < uiPicHeight; unit_i++)
	{
	for (Int unit_j = 0; unit_j < uiPicWidth; unit_j++)
	{
	piBac[unit_j] = piOrg[unit_j];
	}
	piOrg += uiStride;
	piBac += uiStride;
	}
	}
	}*/
#endif

#if BLOCK_GEN //生成块
	if (pcPic->getPOC() > 5 && pcPic->getPOC() < 300)
	{
		int num_block = 0;
		for (UInt uiH = 0; uiH < pcPic->getOrigBuf().Y().height; uiH += BLOCK_GEN_LEN)
		{
			for (UInt uiW = 0; uiW < pcPic->getOrigBuf().Y().width; uiW += BLOCK_GEN_LEN)
			{
				//if (BgBlock[num_block] >= 5 && BgBlock[num_block] <= 1000) //已编
				{
					//BgBlock[num_block]++;
				}
				//else
				if (BgBlock[num_block] < 1000)
				{
					double diff = 0;

					//pcPic->CompBlockDiff(uiW, uiH, pcPic, diff);  //get  block diff
					pcPic->CompBlockPicOrgDiff(uiW, uiH, pcPic, m_bgNewPicYuvOrgGop, diff);//判断背景帧与当前帧得到背景块
					cout << "diff" << diff;
					if (diff < 12)
						//if(pcPic->CompBlockOrgIsFull(uiW, uiH, m_bgNewPicYuvOrgGop))//判断背景帧该块是否已满
					{
						BgBlock[num_block]++;
						isencode = true; //确定编
						pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, uiW, uiH, m_bgNewPicYuvOrgGop);
						//pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, uiW, uiH, pcPic);  //block 编码

					}
					//if (BgBlock[num_block] == 5)
					{
						//isencode = 1;
					}
				}
				

				/*{
					double diff = 0;
					pcPic->CompBlockPicOrgDiff(uiW, uiH, pcPic, m_bgNewPicYuvOrgGop, diff);
					if (diff < 12)
					{
						if (BgBlock[num_block] == 0)
						{
							BgBlock[num_block] = 1;
							isencode = 1; //确定编
							pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, uiW, uiH, m_bgNewPicYuvOrgGop);
						}
						else if (BgBlock[num_block] > 15) //已有 且不相似
						{//考虑更新：判断BlocksOrg与PicYuvOrg 相似则不更新、不相似则更新。
							double updatediff1 = 0;
							pcPic->CompBlockPicOrgDiff(uiW, uiH, m_bgNewBlocksOrgGop, m_bgNewPicYuvOrgGop, updatediff1);
							if (updatediff1 > 50)
							{
								if (pcPic->IsEmpty(m_bgNewBlockRecGop, uiW, uiH, 0)) //判断更新第三者
								{
									pcPic->CopyOrg2Block(m_bgNewBlockRecGop, uiW, uiH, m_bgNewPicYuvOrgGop);
								}
								else
								{
									double updatediff2 = 0;
									pcPic->CompBlockPicOrgDiff(uiW, uiH, m_bgNewBlockRecGop, m_bgNewPicYuvOrgGop, updatediff2);
									if (updatediff2 < 12)//则为新的背景块
									{
										BgBlock[num_block] = 1;
										isupdate = true;//确定更新
										pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, uiW, uiH, m_bgNewPicYuvOrgGop);
										updatenum++;
									}
									else
									{
										pcPic->CopyOrg2Block(m_bgNewBlockRecGop, uiW, uiH, m_bgNewPicYuvOrgGop);
									}
								}
							}
						}
					}
				}*/


				num_block++;  //UNITxUNIT
			}
		}
		cout << "num_block" << num_block << endl;

		{ // cout
			Int i = 0;
			Int j = 0;
			Int k = 0;
			//ofstream bgBlock;
			//bgBlock.open("D://2//BgBlock32_5_8补50.txt", ios::app);
			//bgBlock << pcPic->getPOC()<<" ";
			cout << endl;
			while (i < num_block)
			{
				cout << BgBlock[i] << " ";// << BlockDPP[i] << " ";
										  //bgBlock << BgBlock[i] <<" ";					
				if (BgBlock[i] > 1000)
				{
					j++;
				}
				if (BgBlock[i] > 0)
					k++;
				i++;
			}
			//bgBlock << endl;
			//bgBlock <<" j "<< j << endl;
			cout << endl;
			cout << " Blockj " << j << " " << k << endl;
			cout << "updatenum" << updatenum << endl;
		}
	}
#if BLOCK_RDO
	if (pcPic->getPOC() > 5 && pcPic->getPOC() < 300)
	{
		Int num_CTU = 0;
		for (UInt uiH = 0; uiH < (pcPic->getOrigBuf().Y().height + BLOCK_CTU - 1) / BLOCK_CTU; uiH++)
		{
			for (UInt uiW = 0; uiW < pcPic->getOrigBuf().Y().width / BLOCK_CTU; uiW++)
			{
				Int Bstride = pcPic->getOrigBuf().Y().width / BLOCK_GEN_LEN; //40
				Int CTUnumBlock = BLOCK_CTU / BLOCK_GEN_LEN;  //4
				Double num = 0;
				Double Sum = 0;
				for (Int i = 0; i < CTUnumBlock&&i*BLOCK_GEN_LEN + uiH*BLOCK_CTU <= pcPic->getOrigBuf().Y().height; i++)
				{
					for (Int j = 0; j < CTUnumBlock; j++)
					{
						if (BgBlock[uiH*Bstride*CTUnumBlock + uiW*CTUnumBlock + i*Bstride + j] >= 5 && BgBlock[uiH*Bstride*CTUnumBlock + uiW*CTUnumBlock + i*Bstride + j] < 2000)
						{
							num++;
						}
						Sum++;
					}
				}
				if (num == Sum && BgCTU[num_CTU] == 0)//CTU内大部分块 都大于等于5 即认为该CTU为BCTU
				{
					//编
					//计算该CTU的M
					CTUisencode = true;
					Double P = 0;
					Int Blocknum = 0;
					for (Int i = 0; i < CTUnumBlock; i++)
					{
						for (Int j = 0; j < CTUnumBlock; j++)
						{
							Int num_block = uiH*Bstride*CTUnumBlock + uiW*CTUnumBlock + i*Bstride + j;
							Double Plowerdpp = 0;
							Int H = num_block / Bstride;
							Int W = num_block%Bstride;
							pcPic->CompBlockPicBgPdpp(W, H, pcPic, m_bgNewBlocksOrgGop, BlockDPP[num_block], Plowerdpp);
							P += Plowerdpp*BgBlock[num_block] / (pcPic->getPOC() - 5);
							BgBlock[num_block] = 2000; //该块已被CTU编码
							Blocknum++;
						}
					}
					P = P / Blocknum;
					Int M = int(P*(m_pcCfg->getFramesToBeEncoded() - pcPic->getPOC()));
					BgCTU[num_CTU] = M + 1000;
				}
				num_CTU++;
			}
		}

		{ // cout
			Int i = 0;
			Int j = 0;
			cout << endl;
			while (i < num_CTU)
			{
				cout << BgCTU[i] << " ";
				if (BgCTU[i] > 1000)
				{
					j++;
				}
				i++;
			}
			//bgBlock << endl;
			//bgBlock <<" j "<< j << endl;
			cout << endl;
			cout << " CTUj " << j << endl;
		}
	}
#endif
#endif 
#endif


	bool decPic = false;
	bool encPic = false;
	// test if we can skip the picture entirely or decode instead of encoding
	trySkipOrDecodePicture(decPic, encPic, *m_pcCfg, pcPic);
	pcPic->cs->slice = pcSlice; // please keep this
#if PRINT_PUREF
	//print pumv
	ofstream pufile;
	pufile.open("pufile.txt", ios::app);
	pufile << endl;
	pufile << "POC:" << pcPic->getPOC() << endl;
#endif

#if BLOCK_RDO
	//前50帧编码BCTU  50帧后编码BBlock 改变λ
#if ENCODE_BGPIC
#if BLOCK_ENCODE
	if (CTUisencode&&pcPic->getPOC() < 50) 
#else
	if (pcPic->getPOC() == 20)//&& !afterbg)
#endif 

	{

#if ifif
		cout << pcPic->getPOC() << "---------------------------------------" << endl;
		/*pcSlice->setRefPicListaddRecbg(rcListPic, m_bgNewPicYuvRecGop, m_rcPicYuvTempGop,j);//将Rec加入参考帧预测Org
		pcSlice->setRefPOCList();
		pcSlice->setList1IdxToList0Idx();*/

		//m_pcSliceEncoder->setbgQPSlice(pcSlice, 20, 1, isField); //set QP   

		//m_pcSliceEncoder->resetQPSlice(pcSlice, 17, 1, isField);

		pcPic->CopyOrg(pcPic, m_bgNewPicYuvResiGop);
		pcPic->CopyReco(pcPic, m_bgNewPicYuvResiGop);
		//pcPic->SetOrg0(pcPic);
		//pcPic->SetReco0(pcPic);
		//pcPic->CopyOrg(m_bgNewPicYuvOrgGop, pcPic);
		//pcPic->CopyReco2Org(m_bgNewPicYuvRecGop,pcPic);
		//pcPic->CopyReco(m_bgNewPicYuvRecoGop, pcPic);
#if BLOCK_ENCODE
		//只编上次生成的块。
		//双背景块
		/*Int setpic = 0;
		pcSlice->setRefPicListaddRecbg(rcListPic, m_bgNewBlocksRecGop, m_bgNewBlockRecGop, setpic);//将Rec加入参考帧预测Org
		pcSlice->setRefPOCList();
		pcSlice->setList1IdxToList0Idx();*/
		CTUisencode = false;
		isupdate = 0;
		pcPic->DeleteOrg(pcPic);
		pcPic->DeleteReco(pcPic);
		pcPic->CopyOrg(PrePicRecoGop, pcPic);  //将上一帧的Reco 加到 Org
		//pcPic->CopyReco(m_bgNewPicYuvRecoGop, pcPic);
		Int num_block = 0;
		for (Int i = 0; i < pcPic->getOrigBuf().Y().height; i += BLOCK_CTU)
		{
			for (Int j = 0; j < pcPic->getOrigBuf().Y().width; j += BLOCK_CTU)
			{

				if (BgCTU[num_block] >0 && BgCTU[num_block] < 2000)  
				{
					pcPic->CopyOrg2CTU(pcPic, j, i, m_bgNewBlocksOrgGop);
				}
				num_block++;
			}
		}
#endif		
		if (encPic)
			// now compress (trial encode) the various slice segments (slices, and dependent slices)
		{
			DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", pocCurr)));

			pcSlice->setSliceCurStartCtuTsAddr(0);
#if HEVC_DEPENDENT_SLICES
			pcSlice->setSliceSegmentCurStartCtuTsAddr(0);
#endif

			for (UInt nextCtuTsAddr = 0; nextCtuTsAddr < numberOfCtusInFrame; )
			{  //每个CTU
				m_pcSliceEncoder->precompressSlice(pcPic);
				m_pcSliceEncoder->compressSliceRDO(pcPic, false, false, BgCTU, BlockDPP);
				//m_pcSliceEncoder->compressSlice(pcPic, false, false);
				cout << endl;
				const UInt curSliceEnd = pcSlice->getSliceCurEndCtuTsAddr();
				if (curSliceEnd < numberOfCtusInFrame)
				{
					UInt independentSliceIdx = pcSlice->getIndependentSliceIdx();
					pcPic->allocateNewSlice();
					m_pcSliceEncoder->setSliceSegmentIdx(uiNumSliceSegments);
					// prepare for next slice
					pcSlice = pcPic->slices[uiNumSliceSegments];
					CHECK(!(pcSlice->getPPS() != 0), "Unspecified error");
					pcSlice->copySliceInfo(pcPic->slices[uiNumSliceSegments - 1]);
					pcSlice->setSliceCurStartCtuTsAddr(curSliceEnd);
					pcSlice->setSliceBits(0);
					independentSliceIdx++;
					pcSlice->setIndependentSliceIdx(independentSliceIdx);
					uiNumSliceSegments++;
				}
				nextCtuTsAddr = curSliceEnd;
			}
			duData.clear();

			CodingStructure& cs = *pcPic->cs;
			pcSlice = pcPic->slices[0];

			// SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas
			if (pcSlice->getSPS()->getUseSAO() && m_pcCfg->getSaoCtuBoundary())
			{
				m_pcSAO->getPreDBFStatistics(cs);
			}

			//-- Loop filter
			if (m_pcCfg->getDeblockingFilterMetric())
			{
#if W0038_DB_OPT
				if (m_pcCfg->getDeblockingFilterMetric() == 2)
				{
					applyDeblockingFilterParameterSelection(pcPic, uiNumSliceSegments, iGOPid);
				}
				else
				{
#endif
					applyDeblockingFilterMetric(pcPic, uiNumSliceSegments);
#if W0038_DB_OPT
				}
#endif
			}

			m_pcLoopFilter->loopFilterPic(cs);

			DTRACE_UPDATE(g_trace_ctx, (std::make_pair("final", 1)));

			if (pcSlice->getSPS()->getUseSAO())
			{
				Bool sliceEnabled[MAX_NUM_COMPONENT];
				m_pcSAO->initCABACEstimator(m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), pcSlice);
				m_pcSAO->SAOProcess(cs, sliceEnabled, pcSlice->getLambdas(), m_pcCfg->getTestSAODisableAtPictureLevel(), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma(), m_pcCfg->getSaoCtuBoundary());
				//assign SAO slice header
				for (Int s = 0; s< uiNumSliceSegments; s++)
				{
					pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, sliceEnabled[COMPONENT_Y]);
					CHECK(!(sliceEnabled[COMPONENT_Cb] == sliceEnabled[COMPONENT_Cr]), "Unspecified error");
					pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, sliceEnabled[COMPONENT_Cb]);
				}
			}

		}
		else // skip enc picture
		{
			pcSlice->setSliceQpBase(pcSlice->getSliceQp());

			if (pcSlice->getSPS()->getUseSAO())
			{
				m_pcSAO->disabledRate(*pcPic->cs, pcPic->getSAO(1), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma());
			}
		}

		if (m_pcCfg->getUseAMaxBT())
		{
			for (const CodingUnit *cu : pcPic->cs->cus)
			{
				if (!pcSlice->isIntra())
				{
					m_uiBlkSize[pcSlice->getDepth()] += cu->Y().area();
					m_uiNumBlk[pcSlice->getDepth()]++;
				}
			}
		}


		if (encPic || decPic)
		{

			pcSlice = pcPic->slices[0];

			//////////////////////////// File writing

			// write various parameter sets
			actualTotalBits += xWriteParameterSets(accessUnit, pcSlice, m_bSeqFirst);

			if (m_pcCfg->getAccessUnitDelimiter())
			{
				xWriteAccessUnitDelimiter(accessUnit, pcSlice);
			}

			// reset presence of BP SEI indication
			m_bufferingPeriodSEIPresentInAU = false;
			// create prefix SEI associated with a picture
			xCreatePerPictureSEIMessages(iGOPid, leadingSeiMessages, nestedSeiMessages, pcSlice);

			// pcSlice is currently slice 0.
			std::size_t binCountsInNalUnits = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)
			std::size_t numBytesInVclNalUnits = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)

#if HEVC_DEPENDENT_SLICES
			for (UInt sliceSegmentStartCtuTsAddr = 0, sliceSegmentIdxCount = 0; sliceSegmentStartCtuTsAddr < numberOfCtusInFrame; sliceSegmentIdxCount++, sliceSegmentStartCtuTsAddr = pcSlice->getSliceSegmentCurEndCtuTsAddr())
#else
			for (UInt sliceSegmentStartCtuTsAddr = 0, sliceSegmentIdxCount = 0; sliceSegmentStartCtuTsAddr < numberOfCtusInFrame; sliceSegmentIdxCount++, sliceSegmentStartCtuTsAddr = pcSlice->getSliceCurEndCtuTsAddr())
#endif
			{
				pcSlice = pcPic->slices[sliceSegmentIdxCount];
				if (sliceSegmentIdxCount > 0 && pcSlice->getSliceType() != I_SLICE)
				{
					pcSlice->checkColRefIdx(sliceSegmentIdxCount, pcPic);
				}
				m_pcSliceEncoder->setSliceSegmentIdx(sliceSegmentIdxCount);

				pcSlice->setRPS(pcPic->slices[0]->getRPS());
				pcSlice->setRPSidx(pcPic->slices[0]->getRPSidx());

				for (UInt ui = 0; ui < numSubstreams; ui++)
				{
					substreamsOut[ui].clear();
				}

				/* start slice NALunit */
				OutputNALUnit nalu(pcSlice->getNalUnitType(), pcSlice->getTLayer());
				m_HLSWriter->setBitstream(&nalu.m_Bitstream);

				pcSlice->setNoRaslOutputFlag(false);
				if (pcSlice->isIRAP())
				{
					if (pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_IDR_N_LP)
					{
						pcSlice->setNoRaslOutputFlag(true);
					}
					//the inference for NoOutputPriorPicsFlag
					// KJS: This cannot happen at the encoder
					if (!m_bFirst && pcSlice->isIRAP() && pcSlice->getNoRaslOutputFlag())
					{
						if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
						{
							pcSlice->setNoOutputPriorPicsFlag(true);
						}
					}
				}

				tmpBitsBeforeWriting = m_HLSWriter->getNumberOfWrittenBits();
				m_HLSWriter->codeSliceHeader(pcSlice);


				actualHeadBits += (m_HLSWriter->getNumberOfWrittenBits() - tmpBitsBeforeWriting);

				pcSlice->setFinalized(true);

				pcSlice->clearSubstreamSizes();
				{
					UInt numBinsCoded = 0;
					m_pcSliceEncoder->encodeSlice(pcPic, &(substreamsOut[0]), numBinsCoded);
					binCountsInNalUnits += numBinsCoded;
				}
				{
					// Construct the final bitstream by concatenating substreams.
					// The final bitstream is either nalu.m_Bitstream or pcBitstreamRedirect;
					// Complete the slice header info.
					m_HLSWriter->setBitstream(&nalu.m_Bitstream);
#if HEVC_TILES_WPP
					m_HLSWriter->codeTilesWPPEntryPoint(pcSlice);
#endif

					// Append substreams...
					OutputBitstream *pcOut = pcBitstreamRedirect;
#if HEVC_TILES_WPP
#if HEVC_DEPENDENT_SLICES

					const Int numZeroSubstreamsAtStartOfSlice = pcPic->tileMap->getSubstreamForCtuAddr(pcSlice->getSliceSegmentCurStartCtuTsAddr(), false, pcSlice);
#else
					const Int numZeroSubstreamsAtStartOfSlice = pcPic->tileMap->getSubstreamForCtuAddr(pcSlice->getSliceCurStartCtuTsAddr(), false, pcSlice);
#endif
					const Int numSubstreamsToCode = pcSlice->getNumberOfSubstreamSizes() + 1;
#else
					const Int numZeroSubstreamsAtStartOfSlice = 0;
					const Int numSubstreamsToCode = pcSlice->getNumberOfSubstreamSizes() + 1;
#endif
					for (UInt ui = 0; ui < numSubstreamsToCode; ui++)
					{
						pcOut->addSubstream(&(substreamsOut[ui + numZeroSubstreamsAtStartOfSlice]));
					}
				}

				// If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple dependent slices) then buffer it.
				// If current NALU is the last NALU of slice and a NALU was buffered, then (a) Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
				Bool bNALUAlignedWrittenToList = false; // used to ensure current NALU is not written more than once to the NALU list.
				xAttachSliceDataToNalUnit(nalu, pcBitstreamRedirect);
				accessUnit.push_back(new NALUnitEBSP(nalu));
				actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
				numBytesInVclNalUnits += (std::size_t)(accessUnit.back()->m_nalUnitData.str().size());
				bNALUAlignedWrittenToList = true;

				if (!bNALUAlignedWrittenToList)
				{
					nalu.m_Bitstream.writeAlignZero();
					accessUnit.push_back(new NALUnitEBSP(nalu));
				}

				if ((m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled()) &&
					(pcSlice->getSPS()->getVuiParametersPresentFlag()) &&
					((pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag())
						|| (pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag())) &&
						(pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getSubPicCpbParamsPresentFlag()))
				{
					UInt numNalus = 0;
					UInt numRBSPBytes = 0;
					for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
					{
						numRBSPBytes += UInt((*it)->m_nalUnitData.str().size());
						numNalus++;
					}
					duData.push_back(DUData());
					duData.back().accumBitsDU = (numRBSPBytes << 3);
					duData.back().accumNalsDU = numNalus;
				}
			} // end iteration over slices


			  // cabac_zero_words processing
			cabac_zero_word_padding(pcSlice, pcPic, binCountsInNalUnits, numBytesInVclNalUnits, accessUnit.back()->m_nalUnitData, m_pcCfg->getCabacZeroWordPaddingEnabled());

			/*//-- For time output for each slice
			auto elapsed = std::chrono::steady_clock::now() - beforeTime;
			auto encTime = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

			std::string digestStr;
			if (m_pcCfg->getDecodedPictureHashSEIType() != HASHTYPE_NONE)
			{
			SEIDecodedPictureHash *decodedPictureHashSei = new SEIDecodedPictureHash();
			PelUnitBuf recoBuf = pcPic->cs->getRecoBuf();
			m_seiEncoder.initDecodedPictureHashSEI(decodedPictureHashSei, recoBuf, digestStr, pcSlice->getSPS()->getBitDepths());
			trailingSeiMessages.push_back(decodedPictureHashSei);
			}

			m_pcCfg->setEncodedFlag(iGOPid, true);

			Double PSNR_Y;
			xCalculateAddPSNRs(isField, isTff, iGOPid, pcPic, accessUnit, rcListPic, encTime, snr_conversion, printFrameMSE, &PSNR_Y);
			*/
			/*// Only produce the Green Metadata SEI message with the last picture.
			if (m_pcCfg->getSEIGreenMetadataInfoSEIEnable() && pcSlice->getPOC() == (m_pcCfg->getFramesToBeEncoded() - 1))
			{
			SEIGreenMetadataInfo *seiGreenMetadataInfo = new SEIGreenMetadataInfo;
			m_seiEncoder.initSEIGreenMetadataInfo(seiGreenMetadataInfo, (UInt)(PSNR_Y * 100 + 0.5));
			trailingSeiMessages.push_back(seiGreenMetadataInfo);
			}

			xWriteTrailingSEIMessages(trailingSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS());

			printHash(m_pcCfg->getDecodedPictureHashSEIType(), digestStr);

			if (m_pcCfg->getUseRateCtrl())
			{
			Double avgQP = m_pcRateCtrl->getRCPic()->calAverageQP();
			Double avgLambda = m_pcRateCtrl->getRCPic()->calAverageLambda();
			if (avgLambda < 0.0)
			{
			avgLambda = lambda;
			}

			m_pcRateCtrl->getRCPic()->updateAfterPicture(actualHeadBits, actualTotalBits, avgQP, avgLambda, pcSlice->getSliceType());
			m_pcRateCtrl->getRCPic()->addToPictureLsit(m_pcRateCtrl->getPicList());

			m_pcRateCtrl->getRCSeq()->updateAfterPic(actualTotalBits);
			if (pcSlice->getSliceType() != I_SLICE)
			{
			m_pcRateCtrl->getRCGOP()->updateAfterPicture(actualTotalBits);
			}
			else    // for intra picture, the estimated bits are used to update the current status in the GOP
			{
			m_pcRateCtrl->getRCGOP()->updateAfterPicture(estimatedBits);
			}
			#if U0132_TARGET_BITS_SATURATION
			if (m_pcRateCtrl->getCpbSaturationEnabled())
			{
			m_pcRateCtrl->updateCpbState(actualTotalBits);
			msg(NOTICE, " [CPB %6d bits]", m_pcRateCtrl->getCpbState());
			}
			#endif
			}

			xCreatePictureTimingSEI(m_pcCfg->getEfficientFieldIRAPEnabled() ? effFieldIRAPMap.GetIRAPGOPid() : 0, leadingSeiMessages, nestedSeiMessages, duInfoSeiMessages, pcSlice, isField, duData);
			if (m_pcCfg->getScalableNestingSEIEnabled())
			{
			xCreateScalableNestingSEI(leadingSeiMessages, nestedSeiMessages);
			}
			xWriteLeadingSEIMessages(leadingSeiMessages, duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS(), duData);
			xWriteDuSEIMessages(duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS(), duData);

			m_AUWriterIf->outputAU(accessUnit);
			*/
			//msg(NOTICE, "\n");
			//fflush(stdout);
		}

#if BLOCK_ENCODE


		//BlockReco加入BlocksReco
		num_block = 0;
		for (Int i = 0; i < pcPic->getOrigBuf().Y().height; i += BLOCK_CTU)
		{
			for (Int j = 0; j < pcPic->getOrigBuf().Y().width; j += BLOCK_CTU)
			{
				if (BgCTU[num_block] > 0 && BgCTU[num_block] < 2000)
				{
					pcPic->CopyReco2CTU(m_bgNewPicYuvRecoGop, j, i, pcPic);
					BgCTU[num_block] += 1000;
				}
				num_block++;
			}
		}
#else
		pcPic->CopyReco(pcPic, m_bgNewPicYuvRecoGop);
		pcPic->CopyOrg(pcPic, m_bgNewPicYuvRecoGop);
#endif

		pcPic->CopyOrg(m_bgNewPicYuvResiGop, pcPic);
		pcPic->CopyReco(m_bgNewPicYuvResiGop, pcPic);

		m_pcSliceEncoder->resetQPSlice(pcSlice, 0, iGOPid, isField);

		/*pcSlice->resetRefPicList(rcListPic, m_bgNewBlockRecGop,setpic);
		pcSlice->setRefPOCList();
		pcSlice->setList1IdxToList0Idx();*/

		afterbg = true; //bg has encode
#endif
	}
#endif // ENCODE_BGPIC
#endif
#if !ENCODE_BGPIC
#if BLOCK_ENCODE
	if (pcPic->getPOC()==7)//isencode)//||isselect)//||isupdate)//&&pcPic->getPOC()==50)
#else
	if (pcPic->getPOC() == 6)//&& !afterbg)
#endif
	{
#if ifif
		cout << pcPic->getPOC() << "---------------------------------------" << endl;
		//pcSlice->setRefPicListaddRecbg(rcListPic, m_bgNewPicYuvRecGop, m_rcPicYuvTempGop,j);//将Rec加入参考帧预测Org
		/*pcSlice->setRefPicListaddbgBlockRec(rcListPic, m_bgNewPicYuvRecGop, m_bgNewBlockRecoGop, SetRefPoc, BgBlock);
		pcSlice->setRefPOCList();
		pcSlice->setList1IdxToList0Idx();*/

		//m_pcSliceEncoder->setbgQPSlice(pcSlice, 20, 1, isField); //set QP   

		m_pcSliceEncoder->resetQPSlice(pcSlice, 17, 1, isField);

		pcPic->CopyOrg(pcPic, m_bgNewPicYuvResiGop);
		pcPic->CopyReco(pcPic, m_bgNewPicYuvResiGop);
		//pcPic->SetOrg0(pcPic);
		//pcPic->SetReco0(pcPic);

		pcPic->CopyOrg(m_bgNewPicYuvOrgGop, pcPic);
		//pcPic->CopyReco2Org(m_bgNewPicYuvRecGop,pcPic);
		pcPic->CopyReco(m_bgNewPicYuvRecoGop, pcPic);

#if BLOCK_ENCODE
		//只编上次生成的块。
		//双背景块
		//Int setpic = 0;
		//pcSlice->setRefPicListaddbgBlockRec(rcListPic, m_bgNewBlocksOrgGop, m_bgNewBlockRecoGop, setpic, BgBlock);//将Rec加入参考帧预测Org
		//pcSlice->setRefPOCList();
		//pcSlice->setList1IdxToList0Idx();

		//isencode = false;
		isupdate = 0;
		pcPic->DeleteOrg(pcPic);
		pcPic->DeleteReco(pcPic);
		pcPic->CopyOrg(PrePicRecoGop, pcPic);  //将上一帧的Reco 加到 Org  伪skip
		//pcPic->CopyReco(m_bgNewPicYuvRecoGop, pcPic);
		Int num_block = 0;
		Int numMax = 0;
		Int Maxx = 0;
		if (pcPic->getPOC() % 4 == 0)
			Maxx = 50;
		else
			Maxx = 100;
		for (Int i = 0; i < pcPic->getOrigBuf().Y().height; i += BLOCK_GEN_LEN)
		{
			for (Int j = 0; j < pcPic->getOrigBuf().Y().width; j += BLOCK_GEN_LEN)
			{
				if (BgBlock[num_block] > 0 && BgBlock[num_block] < 2000)
				{
					//if (numMax > 100) //一次只编
					//break;
					numMax++;
					pcPic->CopyOrg2Block(pcPic, j, i, m_bgNewBlocksOrgGop);
				}
				else if (BgBlock2[num_block] > 0 && BgBlock2[num_block] < 2000)
				{
					//pcPic->CopyOrg2Block(pcPic, j, i, m_bgNewBlocksOrgGop);
				}
				//else
				//pcPic->CopyPreReco2Block(pcPic, j, i, PrePicRecoGop);//找到非背景块
				num_block++;
			}
		}
		cout << "编" << numMax << endl;
		//if (numMax < 100)
		//isencode = false;

		/*if (pcPic->getPOC() == 50)  //50帧时 直接把剩下的块补齐
		{
		num_block = 0;
		for (Int i = 0; i < pcPic->getOrigBuf().Y().height; i += BLOCK_GEN_LEN)
		{
		for (Int j = 0; j < pcPic->getOrigBuf().Y().width; j += BLOCK_GEN_LEN)
		{

		if (BgBlock[num_block] == 0)
		{
		BgBlock[num_block] = 1;
		pcPic->CopyOrg2Block(pcPic, j, i, m_bgNewPicYuvOrgGop);
		pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, j, i, m_bgNewPicYuvOrgGop);
		}
		num_block++;
		}
		}
		}*/
#endif		
		if (encPic)
			// now compress (trial encode) the various slice segments (slices, and dependent slices)
		{
			DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", pocCurr)));

			pcSlice->setSliceCurStartCtuTsAddr(0);
#if HEVC_DEPENDENT_SLICES
			pcSlice->setSliceSegmentCurStartCtuTsAddr(0);
#endif

			for (UInt nextCtuTsAddr = 0; nextCtuTsAddr < numberOfCtusInFrame; )
			{  //每个CTU
				m_pcSliceEncoder->precompressSlice(pcPic);
				m_pcSliceEncoder->compressSlice(pcPic, false, false);

				const UInt curSliceEnd = pcSlice->getSliceCurEndCtuTsAddr();
				if (curSliceEnd < numberOfCtusInFrame)
				{
					UInt independentSliceIdx = pcSlice->getIndependentSliceIdx();
					pcPic->allocateNewSlice();
					m_pcSliceEncoder->setSliceSegmentIdx(uiNumSliceSegments);
					// prepare for next slice
					pcSlice = pcPic->slices[uiNumSliceSegments];
					CHECK(!(pcSlice->getPPS() != 0), "Unspecified error");
					pcSlice->copySliceInfo(pcPic->slices[uiNumSliceSegments - 1]);
					pcSlice->setSliceCurStartCtuTsAddr(curSliceEnd);
					pcSlice->setSliceBits(0);
					independentSliceIdx++;
					pcSlice->setIndependentSliceIdx(independentSliceIdx);
					uiNumSliceSegments++;
				}
				nextCtuTsAddr = curSliceEnd;
			}
			duData.clear();

			CodingStructure& cs = *pcPic->cs;
			pcSlice = pcPic->slices[0];

			// SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas
			if (pcSlice->getSPS()->getUseSAO() && m_pcCfg->getSaoCtuBoundary())
			{
				m_pcSAO->getPreDBFStatistics(cs);
			}

			//-- Loop filter
			if (m_pcCfg->getDeblockingFilterMetric())
			{
#if W0038_DB_OPT
				if (m_pcCfg->getDeblockingFilterMetric() == 2)
				{
					applyDeblockingFilterParameterSelection(pcPic, uiNumSliceSegments, iGOPid);
				}
				else
				{
#endif
					applyDeblockingFilterMetric(pcPic, uiNumSliceSegments);
#if W0038_DB_OPT
				}
#endif
			}

			m_pcLoopFilter->loopFilterPic(cs);

			DTRACE_UPDATE(g_trace_ctx, (std::make_pair("final", 1)));

			if (pcSlice->getSPS()->getUseSAO())
			{
				Bool sliceEnabled[MAX_NUM_COMPONENT];
				m_pcSAO->initCABACEstimator(m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), pcSlice);
				m_pcSAO->SAOProcess(cs, sliceEnabled, pcSlice->getLambdas(), m_pcCfg->getTestSAODisableAtPictureLevel(), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma(), m_pcCfg->getSaoCtuBoundary());
				//assign SAO slice header
				for (Int s = 0; s< uiNumSliceSegments; s++)
				{
					pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, sliceEnabled[COMPONENT_Y]);
					CHECK(!(sliceEnabled[COMPONENT_Cb] == sliceEnabled[COMPONENT_Cr]), "Unspecified error");
					pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, sliceEnabled[COMPONENT_Cb]);
				}
			}

		}
		else // skip enc picture
		{
			pcSlice->setSliceQpBase(pcSlice->getSliceQp());

			if (pcSlice->getSPS()->getUseSAO())
			{
				m_pcSAO->disabledRate(*pcPic->cs, pcPic->getSAO(1), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma());
			}
		}

		if (m_pcCfg->getUseAMaxBT())
		{
			for (const CodingUnit *cu : pcPic->cs->cus)
			{
				if (!pcSlice->isIntra())
				{
					m_uiBlkSize[pcSlice->getDepth()] += cu->Y().area();
					m_uiNumBlk[pcSlice->getDepth()]++;
				}
			}
		}


		if (encPic || decPic)
		{

			pcSlice = pcPic->slices[0];

			//////////////////////////// File writing

			// write various parameter sets
			actualTotalBits += xWriteParameterSets(accessUnit, pcSlice, m_bSeqFirst);

			if (m_pcCfg->getAccessUnitDelimiter())
			{
				xWriteAccessUnitDelimiter(accessUnit, pcSlice);
			}

			// reset presence of BP SEI indication
			m_bufferingPeriodSEIPresentInAU = false;
			// create prefix SEI associated with a picture
			xCreatePerPictureSEIMessages(iGOPid, leadingSeiMessages, nestedSeiMessages, pcSlice);

			// pcSlice is currently slice 0.
			std::size_t binCountsInNalUnits = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)
			std::size_t numBytesInVclNalUnits = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)

#if HEVC_DEPENDENT_SLICES
			for (UInt sliceSegmentStartCtuTsAddr = 0, sliceSegmentIdxCount = 0; sliceSegmentStartCtuTsAddr < numberOfCtusInFrame; sliceSegmentIdxCount++, sliceSegmentStartCtuTsAddr = pcSlice->getSliceSegmentCurEndCtuTsAddr())
#else
			for (UInt sliceSegmentStartCtuTsAddr = 0, sliceSegmentIdxCount = 0; sliceSegmentStartCtuTsAddr < numberOfCtusInFrame; sliceSegmentIdxCount++, sliceSegmentStartCtuTsAddr = pcSlice->getSliceCurEndCtuTsAddr())
#endif
			{
				pcSlice = pcPic->slices[sliceSegmentIdxCount];
				if (sliceSegmentIdxCount > 0 && pcSlice->getSliceType() != I_SLICE)
				{
					pcSlice->checkColRefIdx(sliceSegmentIdxCount, pcPic);
				}
				m_pcSliceEncoder->setSliceSegmentIdx(sliceSegmentIdxCount);

				pcSlice->setRPS(pcPic->slices[0]->getRPS());
				pcSlice->setRPSidx(pcPic->slices[0]->getRPSidx());

				for (UInt ui = 0; ui < numSubstreams; ui++)
				{
					substreamsOut[ui].clear();
				}

				/* start slice NALunit */
				OutputNALUnit nalu(pcSlice->getNalUnitType(), pcSlice->getTLayer());
				m_HLSWriter->setBitstream(&nalu.m_Bitstream);

				pcSlice->setNoRaslOutputFlag(false);
				if (pcSlice->isIRAP())
				{
					if (pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_IDR_N_LP)
					{
						pcSlice->setNoRaslOutputFlag(true);
					}
					//the inference for NoOutputPriorPicsFlag
					// KJS: This cannot happen at the encoder
					if (!m_bFirst && pcSlice->isIRAP() && pcSlice->getNoRaslOutputFlag())
					{
						if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
						{
							pcSlice->setNoOutputPriorPicsFlag(true);
						}
					}
				}

				tmpBitsBeforeWriting = m_HLSWriter->getNumberOfWrittenBits();
				m_HLSWriter->codeSliceHeader(pcSlice);


				actualHeadBits += (m_HLSWriter->getNumberOfWrittenBits() - tmpBitsBeforeWriting);

				pcSlice->setFinalized(true);

				pcSlice->clearSubstreamSizes();
				{
					UInt numBinsCoded = 0;
					m_pcSliceEncoder->encodeSlice(pcPic, &(substreamsOut[0]), numBinsCoded);
					binCountsInNalUnits += numBinsCoded;
				}
				{
					// Construct the final bitstream by concatenating substreams.
					// The final bitstream is either nalu.m_Bitstream or pcBitstreamRedirect;
					// Complete the slice header info.
					m_HLSWriter->setBitstream(&nalu.m_Bitstream);
#if HEVC_TILES_WPP
					m_HLSWriter->codeTilesWPPEntryPoint(pcSlice);
#endif

					// Append substreams...
					OutputBitstream *pcOut = pcBitstreamRedirect;
#if HEVC_TILES_WPP
#if HEVC_DEPENDENT_SLICES

					const Int numZeroSubstreamsAtStartOfSlice = pcPic->tileMap->getSubstreamForCtuAddr(pcSlice->getSliceSegmentCurStartCtuTsAddr(), false, pcSlice);
#else
					const Int numZeroSubstreamsAtStartOfSlice = pcPic->tileMap->getSubstreamForCtuAddr(pcSlice->getSliceCurStartCtuTsAddr(), false, pcSlice);
#endif
					const Int numSubstreamsToCode = pcSlice->getNumberOfSubstreamSizes() + 1;
#else
					const Int numZeroSubstreamsAtStartOfSlice = 0;
					const Int numSubstreamsToCode = pcSlice->getNumberOfSubstreamSizes() + 1;
#endif
					for (UInt ui = 0; ui < numSubstreamsToCode; ui++)
					{
						pcOut->addSubstream(&(substreamsOut[ui + numZeroSubstreamsAtStartOfSlice]));
					}
				}

				// If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple dependent slices) then buffer it.
				// If current NALU is the last NALU of slice and a NALU was buffered, then (a) Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
				Bool bNALUAlignedWrittenToList = false; // used to ensure current NALU is not written more than once to the NALU list.
				xAttachSliceDataToNalUnit(nalu, pcBitstreamRedirect);
				accessUnit.push_back(new NALUnitEBSP(nalu));
				actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
				numBytesInVclNalUnits += (std::size_t)(accessUnit.back()->m_nalUnitData.str().size());
				bNALUAlignedWrittenToList = true;

				if (!bNALUAlignedWrittenToList)
				{
					nalu.m_Bitstream.writeAlignZero();
					accessUnit.push_back(new NALUnitEBSP(nalu));
				}

				if ((m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled()) &&
					(pcSlice->getSPS()->getVuiParametersPresentFlag()) &&
					((pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag())
						|| (pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag())) &&
						(pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getSubPicCpbParamsPresentFlag()))
				{
					UInt numNalus = 0;
					UInt numRBSPBytes = 0;
					for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
					{
						numRBSPBytes += UInt((*it)->m_nalUnitData.str().size());
						numNalus++;
					}
					duData.push_back(DUData());
					duData.back().accumBitsDU = (numRBSPBytes << 3);
					duData.back().accumNalsDU = numNalus;
				}
			} // end iteration over slices


			  // cabac_zero_words processing
			cabac_zero_word_padding(pcSlice, pcPic, binCountsInNalUnits, numBytesInVclNalUnits, accessUnit.back()->m_nalUnitData, m_pcCfg->getCabacZeroWordPaddingEnabled());

			/*//-- For time output for each slice
			auto elapsed = std::chrono::steady_clock::now() - beforeTime;
			auto encTime = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

			std::string digestStr;
			if (m_pcCfg->getDecodedPictureHashSEIType() != HASHTYPE_NONE)
			{
			SEIDecodedPictureHash *decodedPictureHashSei = new SEIDecodedPictureHash();
			PelUnitBuf recoBuf = pcPic->cs->getRecoBuf();
			m_seiEncoder.initDecodedPictureHashSEI(decodedPictureHashSei, recoBuf, digestStr, pcSlice->getSPS()->getBitDepths());
			trailingSeiMessages.push_back(decodedPictureHashSei);
			}

			m_pcCfg->setEncodedFlag(iGOPid, true);

			Double PSNR_Y;
			xCalculateAddPSNRs(isField, isTff, iGOPid, pcPic, accessUnit, rcListPic, encTime, snr_conversion, printFrameMSE, &PSNR_Y);
			*/
			/*// Only produce the Green Metadata SEI message with the last picture.
			if (m_pcCfg->getSEIGreenMetadataInfoSEIEnable() && pcSlice->getPOC() == (m_pcCfg->getFramesToBeEncoded() - 1))
			{
			SEIGreenMetadataInfo *seiGreenMetadataInfo = new SEIGreenMetadataInfo;
			m_seiEncoder.initSEIGreenMetadataInfo(seiGreenMetadataInfo, (UInt)(PSNR_Y * 100 + 0.5));
			trailingSeiMessages.push_back(seiGreenMetadataInfo);
			}

			xWriteTrailingSEIMessages(trailingSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS());

			printHash(m_pcCfg->getDecodedPictureHashSEIType(), digestStr);

			if (m_pcCfg->getUseRateCtrl())
			{
			Double avgQP = m_pcRateCtrl->getRCPic()->calAverageQP();
			Double avgLambda = m_pcRateCtrl->getRCPic()->calAverageLambda();
			if (avgLambda < 0.0)
			{
			avgLambda = lambda;
			}

			m_pcRateCtrl->getRCPic()->updateAfterPicture(actualHeadBits, actualTotalBits, avgQP, avgLambda, pcSlice->getSliceType());
			m_pcRateCtrl->getRCPic()->addToPictureLsit(m_pcRateCtrl->getPicList());

			m_pcRateCtrl->getRCSeq()->updateAfterPic(actualTotalBits);
			if (pcSlice->getSliceType() != I_SLICE)
			{
			m_pcRateCtrl->getRCGOP()->updateAfterPicture(actualTotalBits);
			}
			else    // for intra picture, the estimated bits are used to update the current status in the GOP
			{
			m_pcRateCtrl->getRCGOP()->updateAfterPicture(estimatedBits);
			}
			#if U0132_TARGET_BITS_SATURATION
			if (m_pcRateCtrl->getCpbSaturationEnabled())
			{
			m_pcRateCtrl->updateCpbState(actualTotalBits);
			msg(NOTICE, " [CPB %6d bits]", m_pcRateCtrl->getCpbState());
			}
			#endif
			}

			xCreatePictureTimingSEI(m_pcCfg->getEfficientFieldIRAPEnabled() ? effFieldIRAPMap.GetIRAPGOPid() : 0, leadingSeiMessages, nestedSeiMessages, duInfoSeiMessages, pcSlice, isField, duData);
			if (m_pcCfg->getScalableNestingSEIEnabled())
			{
			xCreateScalableNestingSEI(leadingSeiMessages, nestedSeiMessages);
			}
			xWriteLeadingSEIMessages(leadingSeiMessages, duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS(), duData);
			xWriteDuSEIMessages(duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS(), duData);

			m_AUWriterIf->outputAU(accessUnit);
			*/
			//msg(NOTICE, "\n");
			//fflush(stdout);
		}

#if BLOCK_ENCODE


		//BlockReco加入BlocksReco
		num_block = 0;
		numMax = 0;
		for (Int i = 0; i < pcPic->getOrigBuf().Y().height; i += BLOCK_GEN_LEN)
		{
			for (Int j = 0; j < pcPic->getOrigBuf().Y().width; j += BLOCK_GEN_LEN)
			{
				if (BgBlock[num_block] > 0 && BgBlock[num_block] < 2000)
				{
					//if (numMax > 100)
						//break;
					numMax++;
					pcPic->CopyReco2Block(m_bgNewBlocksOrgGop, j, i, pcPic);
					BgBlock[num_block] = 1001;
					//pcPic->CopyOrg2Block(m_bgNewPicYuvRecoGop, j, i, pcPic);
				}
				else if (BgBlock2[num_block] > 0 && BgBlock2[num_block] < 2000)
				{
					//pcPic->CopyReco2Block(m_bgNewPicYuvRecoGop, j, i, pcPic);
					//BgBlock2[num_block] = 2000;
				}
				num_block++;
			}
		}
		/*if (pcPic->getPOC() == 50)  //50
		{
		num_block = 0;
		for (Int i = 0; i < pcPic->getOrigBuf().Y().height; i += BLOCK_GEN_LEN)
		{
		for (Int j = 0; j < pcPic->getOrigBuf().Y().width; j += BLOCK_GEN_LEN)
		{

		if (BgBlock[num_block] == 0)
		{
		pcPic->CopyReco2Block(m_bgNewPicYuvRecoGop, j, i, pcPic);
		}
		num_block++;
		}
		}
		}*/
#else
		pcPic->CopyReco(pcPic, m_bgNewPicYuvRecoGop);
		pcPic->CopyOrg(pcPic, m_bgNewPicYuvRecoGop);
#endif

		pcPic->CopyOrg(m_bgNewPicYuvResiGop, pcPic);
		pcPic->CopyReco(m_bgNewPicYuvResiGop, pcPic);

		m_pcSliceEncoder->resetQPSlice(pcSlice, 0, iGOPid, isField);

		/*pcSlice->resetRefPicList(rcListPic, m_bgNewBlockRecoGop,setpic);
		pcSlice->setRefPOCList();
		pcSlice->setList1IdxToList0Idx();*/

		afterbg = true; //bg has encode
#endif
	}
#endif // ENCODE_BGPIC

#if ENCODE_BGPIC



	//判断是否够
	Bool isoktoen = false;
	Int numsx = (pcPic->getOrigBuf().Y().width + BLOCK_GEN_LEN - 1) / BLOCK_GEN_LEN;
	Int numsy = (pcPic->getOrigBuf().Y().height + BLOCK_GEN_LEN - 1) / BLOCK_GEN_LEN;
	Int maxencodenum = numsx*numsy / 12;
	Int num = 0;
	Int num_block = 0;
	for (Int i = 0; i < pcPic->getOrigBuf().Y().height; i += BLOCK_GEN_LEN)
	{
		for (Int j = 0; j < pcPic->getOrigBuf().Y().width; j += BLOCK_GEN_LEN)
		{
			Bgselect[num_block] = 0;
			Bgselect1[num_block] = 0;
			if (BgBlock[num_block] > 3 && BgBlock[num_block] < 2000) //W>5的个数
			{
				double dpp = 0; //计算dpp加入BlockDPP[]
				pcPic->CompBlockPicbgdpp(j, i, pcPic, m_bgNewBlocksOrgGop, dpp); //计算该帧 该块的dpp 需要org和reco。
				Bgselect[num] = double(BgBlock[num_block]) / dpp;
				Bgselect1[num_block] = double(BgBlock[num_block]) / dpp;  //importance map
				//cout << dpp<<"-"<<Bgselect[num] << "__";
				num++;
			}
			num_block++;
		}
	}
	if (num > maxencodenum / 4) //最小为最大/4
	{
		isoktoen = true;
		sort(Bgselect, Bgselect + num, greater<double>()); //找到
		//cout << endl << endl;
		for (int i = 0; i < num; i++)
		{
			//cout << Bgselect[i] << "-";
		}
	}
	double minwd = Bgselect[maxencodenum];
	cout <<endl<< isoktoen<<" "<<minwd <<"  "<<maxencodenum<<"  "<<num<< endl;
	
	if (pcPic->getPOC()==1000)
	{
		cout << "===" << endl;
		//pcPic->DeleteOrg(pcPic);
		//pcPic->DeleteReco(pcPic);
		//pcPic->CopyOrg(m_bgNewPicYuvOrgGop, pcPic);
		//pcPic->CopyReco(m_bgNewPicYuvRecoGop, pcPic);
		int num_block = 0;
		int num = 0;
		for (Int i = 0; i < pcPic->getOrigBuf().Y().height; i += BLOCK_GEN_LEN)
		{
			for (Int j = 0; j < pcPic->getOrigBuf().Y().width; j += BLOCK_GEN_LEN)
			{
				cout << Bgselect1[num_block] << " ";
				/*if (BgBlock[num_block] > 2000)// && BgBlock[num_block] < 2000 && Bgselect1[num_block]>minwd)
				{
					num++;
					pcPic->CopyOrg2Block(pcPic, j, i, m_bgNewBlocksOrgGop);
					pcPic->DrawRef(uiW, uiH, pcPic, BgBlock1[num_block]);
				}*/
				pcPic->DrawRef(j, i, pcPic, BgBlock[num_block]);
				num_block++;
			}
		}
		cout << num << "num" << endl;
	}


#if BLOCK_ENCODE
	if(isoktoen&&pcPic->getPOC()%4!=0)///*isselectencode)//*/isencode)//||isselect)//||isupdate)//&&pcPic->getPOC()==50)
#else
	if (pcPic->getPOC() == 5 )//&& !afterbg)
#endif 
	
	{
		
#if ifif
		cout << pcPic->getPOC() << "---------------------------------------" << endl;
		//pcSlice->setRefPicListaddRecbg(rcListPic, m_bgNewPicYuvRecGop, m_rcPicYuvTempGop,j);//将Rec加入参考帧预测Org
		/*pcSlice->setRefPicListaddbgBlockRec(rcListPic, m_bgNewPicYuvRecGop, m_bgNewBlockRecoGop, SetRefPoc, BgBlock);
		pcSlice->setRefPOCList();
		pcSlice->setList1IdxToList0Idx();*/
		
		//m_pcSliceEncoder->setbgQPSlice(pcSlice, 20, 1, isField); //set QP   
		
		m_pcSliceEncoder->resetQPSlice(pcSlice, 17, 1, isField);
		
		pcPic->CopyOrg(pcPic, m_bgNewPicYuvResiGop);
		pcPic->CopyReco(pcPic, m_bgNewPicYuvResiGop);
		//pcPic->SetOrg0(pcPic);
		//pcPic->SetReco0(pcPic);

		//pcPic->CopyOrg(m_bgNewPicYuvOrgGop, pcPic);
		//pcPic->CopyReco2Org(m_bgNewPicYuvRecGop,pcPic);
		//pcPic->CopyReco(m_bgNewPicYuvRecoGop, pcPic);

#if BLOCK_ENCODE
		//只编上次生成的块。
		//双背景块
		//Int setpic = 0;
		//pcSlice->setRefPicListaddbgBlockRec(rcListPic, m_bgNewBlocksOrgGop, m_bgNewBlockRecoGop, setpic, BgBlock);//将Rec加入参考帧预测Org
		//pcSlice->setRefPOCList();
		//pcSlice->setList1IdxToList0Idx();

		isencode = false;
		isselectencode = false;
		isupdate = 0;
		pcPic->DeleteOrg(pcPic);
		pcPic->DeleteReco(pcPic);
		pcPic->CopyOrg(PrePicRecoGop, pcPic);  //将上一帧的Reco 加到 Org  伪skip
		//pcPic->CopyReco(m_bgNewPicYuvRecoGop, pcPic);
		
		Int num_block = 0;
		Int numMax = 0;
		Int Maxx = maxencodenum;
		//if (pcPic->getPOC() % 4 == 0)
			//Maxx = maxencodenum;
		for (Int i = 0; i < pcPic->getOrigBuf().Y().height; i += BLOCK_GEN_LEN)
		{
			for (Int j = 0; j < pcPic->getOrigBuf().Y().width; j += BLOCK_GEN_LEN)
			{
				if (BgBlock[num_block] > 0 && BgBlock[num_block] < 2000 && Bgselect1[num_block]>minwd)
				{
					//if (numMax > Maxx) //一次只编
						//break;
					numMax++;
					pcPic->CopyOrg2Block(pcPic, j, i, m_bgNewBlocksOrgGop);
				}
				//else
					//pcPic->CopyPreReco2Block(pcPic, j, i, PrePicRecoGop);//找到非背景块
				num_block++;
			}
		}
		cout << "编" << numMax << endl;
		//if (numMax < 100)
			//isencode = false;

		/*if (pcPic->getPOC() == 50)  //50帧时 直接把剩下的块补齐
		{
			num_block = 0;
			for (Int i = 0; i < pcPic->getOrigBuf().Y().height; i += BLOCK_GEN_LEN)
			{
				for (Int j = 0; j < pcPic->getOrigBuf().Y().width; j += BLOCK_GEN_LEN)
				{

					if (BgBlock[num_block] == 0)
					{
						BgBlock[num_block] = 1;
						pcPic->CopyOrg2Block(pcPic, j, i, m_bgNewPicYuvOrgGop);
						pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, j, i, m_bgNewPicYuvOrgGop);
					}
					num_block++;
				}
			}
		}*/
#endif		
		if (encPic)
			// now compress (trial encode) the various slice segments (slices, and dependent slices)
		{
			DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", pocCurr)));

			pcSlice->setSliceCurStartCtuTsAddr(0);
#if HEVC_DEPENDENT_SLICES
			pcSlice->setSliceSegmentCurStartCtuTsAddr(0);
#endif

			for (UInt nextCtuTsAddr = 0; nextCtuTsAddr < numberOfCtusInFrame; )
			{  //每个CTU
				m_pcSliceEncoder->precompressSlice(pcPic);
				m_pcSliceEncoder->compressSlice(pcPic, false, false);

				const UInt curSliceEnd = pcSlice->getSliceCurEndCtuTsAddr();
				if (curSliceEnd < numberOfCtusInFrame)
				{
					UInt independentSliceIdx = pcSlice->getIndependentSliceIdx();
					pcPic->allocateNewSlice();
					m_pcSliceEncoder->setSliceSegmentIdx(uiNumSliceSegments);
					// prepare for next slice
					pcSlice = pcPic->slices[uiNumSliceSegments];
					CHECK(!(pcSlice->getPPS() != 0), "Unspecified error");
					pcSlice->copySliceInfo(pcPic->slices[uiNumSliceSegments - 1]);
					pcSlice->setSliceCurStartCtuTsAddr(curSliceEnd);
					pcSlice->setSliceBits(0);
					independentSliceIdx++;
					pcSlice->setIndependentSliceIdx(independentSliceIdx);
					uiNumSliceSegments++;
				}
				nextCtuTsAddr = curSliceEnd;
			}
			duData.clear();

			CodingStructure& cs = *pcPic->cs;
			pcSlice = pcPic->slices[0];

			// SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas
			if (pcSlice->getSPS()->getUseSAO() && m_pcCfg->getSaoCtuBoundary())
			{
				m_pcSAO->getPreDBFStatistics(cs);
			}

			//-- Loop filter
			if (m_pcCfg->getDeblockingFilterMetric())
			{
#if W0038_DB_OPT
				if (m_pcCfg->getDeblockingFilterMetric() == 2)
				{
					applyDeblockingFilterParameterSelection(pcPic, uiNumSliceSegments, iGOPid);
				}
				else
				{
#endif
					applyDeblockingFilterMetric(pcPic, uiNumSliceSegments);
#if W0038_DB_OPT
				}
#endif
			}

			m_pcLoopFilter->loopFilterPic(cs);

			DTRACE_UPDATE(g_trace_ctx, (std::make_pair("final", 1)));

			if (pcSlice->getSPS()->getUseSAO())
			{
				Bool sliceEnabled[MAX_NUM_COMPONENT];
				m_pcSAO->initCABACEstimator(m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), pcSlice);
				m_pcSAO->SAOProcess(cs, sliceEnabled, pcSlice->getLambdas(), m_pcCfg->getTestSAODisableAtPictureLevel(), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma(), m_pcCfg->getSaoCtuBoundary());
				//assign SAO slice header
				for (Int s = 0; s< uiNumSliceSegments; s++)
				{
					pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, sliceEnabled[COMPONENT_Y]);
					CHECK(!(sliceEnabled[COMPONENT_Cb] == sliceEnabled[COMPONENT_Cr]), "Unspecified error");
					pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, sliceEnabled[COMPONENT_Cb]);
				}
			}

		}
		else // skip enc picture
		{
			pcSlice->setSliceQpBase(pcSlice->getSliceQp());

			if (pcSlice->getSPS()->getUseSAO())
			{
				m_pcSAO->disabledRate(*pcPic->cs, pcPic->getSAO(1), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma());
			}
		}

		if (m_pcCfg->getUseAMaxBT())
		{
			for (const CodingUnit *cu : pcPic->cs->cus)
			{
				if (!pcSlice->isIntra())
				{
					m_uiBlkSize[pcSlice->getDepth()] += cu->Y().area();
					m_uiNumBlk[pcSlice->getDepth()]++;
				}
			}
		}


		if (encPic || decPic)
		{
			
			pcSlice = pcPic->slices[0];

			//////////////////////////// File writing

			// write various parameter sets
			actualTotalBits += xWriteParameterSets(accessUnit, pcSlice, m_bSeqFirst);

			if (m_pcCfg->getAccessUnitDelimiter())
			{
				xWriteAccessUnitDelimiter(accessUnit, pcSlice);
			}

			// reset presence of BP SEI indication
			m_bufferingPeriodSEIPresentInAU = false;
			// create prefix SEI associated with a picture
			xCreatePerPictureSEIMessages(iGOPid, leadingSeiMessages, nestedSeiMessages, pcSlice);

			// pcSlice is currently slice 0.
			std::size_t binCountsInNalUnits = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)
			std::size_t numBytesInVclNalUnits = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)

#if HEVC_DEPENDENT_SLICES
			for (UInt sliceSegmentStartCtuTsAddr = 0, sliceSegmentIdxCount = 0; sliceSegmentStartCtuTsAddr < numberOfCtusInFrame; sliceSegmentIdxCount++, sliceSegmentStartCtuTsAddr = pcSlice->getSliceSegmentCurEndCtuTsAddr())
#else
			for (UInt sliceSegmentStartCtuTsAddr = 0, sliceSegmentIdxCount = 0; sliceSegmentStartCtuTsAddr < numberOfCtusInFrame; sliceSegmentIdxCount++, sliceSegmentStartCtuTsAddr = pcSlice->getSliceCurEndCtuTsAddr())
#endif
			{
				pcSlice = pcPic->slices[sliceSegmentIdxCount];
				if (sliceSegmentIdxCount > 0 && pcSlice->getSliceType() != I_SLICE)
				{
					pcSlice->checkColRefIdx(sliceSegmentIdxCount, pcPic);
				}
				m_pcSliceEncoder->setSliceSegmentIdx(sliceSegmentIdxCount);

				pcSlice->setRPS(pcPic->slices[0]->getRPS());
				pcSlice->setRPSidx(pcPic->slices[0]->getRPSidx());

				for (UInt ui = 0; ui < numSubstreams; ui++)
				{
					substreamsOut[ui].clear();
				}

				/* start slice NALunit */
				OutputNALUnit nalu(pcSlice->getNalUnitType(), pcSlice->getTLayer());
				m_HLSWriter->setBitstream(&nalu.m_Bitstream);

				pcSlice->setNoRaslOutputFlag(false);
				if (pcSlice->isIRAP())
				{
					if (pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_IDR_N_LP)
					{
						pcSlice->setNoRaslOutputFlag(true);
					}
					//the inference for NoOutputPriorPicsFlag
					// KJS: This cannot happen at the encoder
					if (!m_bFirst && pcSlice->isIRAP() && pcSlice->getNoRaslOutputFlag())
					{
						if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
						{
							pcSlice->setNoOutputPriorPicsFlag(true);
						}
					}
				}

				tmpBitsBeforeWriting = m_HLSWriter->getNumberOfWrittenBits();
				m_HLSWriter->codeSliceHeader(pcSlice);


				actualHeadBits += (m_HLSWriter->getNumberOfWrittenBits() - tmpBitsBeforeWriting);

				pcSlice->setFinalized(true);

				pcSlice->clearSubstreamSizes();
				{
					UInt numBinsCoded = 0;
					m_pcSliceEncoder->encodeSlice(pcPic, &(substreamsOut[0]), numBinsCoded);
					binCountsInNalUnits += numBinsCoded;
				}
				{
					// Construct the final bitstream by concatenating substreams.
					// The final bitstream is either nalu.m_Bitstream or pcBitstreamRedirect;
					// Complete the slice header info.
					m_HLSWriter->setBitstream(&nalu.m_Bitstream);
#if HEVC_TILES_WPP
					m_HLSWriter->codeTilesWPPEntryPoint(pcSlice);
#endif

					// Append substreams...
					OutputBitstream *pcOut = pcBitstreamRedirect;
#if HEVC_TILES_WPP
#if HEVC_DEPENDENT_SLICES

					const Int numZeroSubstreamsAtStartOfSlice = pcPic->tileMap->getSubstreamForCtuAddr(pcSlice->getSliceSegmentCurStartCtuTsAddr(), false, pcSlice);
#else
					const Int numZeroSubstreamsAtStartOfSlice = pcPic->tileMap->getSubstreamForCtuAddr(pcSlice->getSliceCurStartCtuTsAddr(), false, pcSlice);
#endif
					const Int numSubstreamsToCode = pcSlice->getNumberOfSubstreamSizes() + 1;
#else
					const Int numZeroSubstreamsAtStartOfSlice = 0;
					const Int numSubstreamsToCode = pcSlice->getNumberOfSubstreamSizes() + 1;
#endif
					for (UInt ui = 0; ui < numSubstreamsToCode; ui++)
					{
						pcOut->addSubstream(&(substreamsOut[ui + numZeroSubstreamsAtStartOfSlice]));
					}
				}

				// If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple dependent slices) then buffer it.
				// If current NALU is the last NALU of slice and a NALU was buffered, then (a) Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
				Bool bNALUAlignedWrittenToList = false; // used to ensure current NALU is not written more than once to the NALU list.
				xAttachSliceDataToNalUnit(nalu, pcBitstreamRedirect);
				accessUnit.push_back(new NALUnitEBSP(nalu));
				actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
				numBytesInVclNalUnits += (std::size_t)(accessUnit.back()->m_nalUnitData.str().size());
				bNALUAlignedWrittenToList = true;

				if (!bNALUAlignedWrittenToList)
				{
					nalu.m_Bitstream.writeAlignZero();
					accessUnit.push_back(new NALUnitEBSP(nalu));
				}

				if ((m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled()) &&
					(pcSlice->getSPS()->getVuiParametersPresentFlag()) &&
					((pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag())
						|| (pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag())) &&
						(pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getSubPicCpbParamsPresentFlag()))
				{
					UInt numNalus = 0;
					UInt numRBSPBytes = 0;
					for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
					{
						numRBSPBytes += UInt((*it)->m_nalUnitData.str().size());
						numNalus++;
					}
					duData.push_back(DUData());
					duData.back().accumBitsDU = (numRBSPBytes << 3);
					duData.back().accumNalsDU = numNalus;
				}
			} // end iteration over slices

			
			  // cabac_zero_words processing
			cabac_zero_word_padding(pcSlice, pcPic, binCountsInNalUnits, numBytesInVclNalUnits, accessUnit.back()->m_nalUnitData, m_pcCfg->getCabacZeroWordPaddingEnabled());
			
			/*//-- For time output for each slice
			auto elapsed = std::chrono::steady_clock::now() - beforeTime;
			auto encTime = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

			std::string digestStr;
			if (m_pcCfg->getDecodedPictureHashSEIType() != HASHTYPE_NONE)
			{
				SEIDecodedPictureHash *decodedPictureHashSei = new SEIDecodedPictureHash();
				PelUnitBuf recoBuf = pcPic->cs->getRecoBuf();
				m_seiEncoder.initDecodedPictureHashSEI(decodedPictureHashSei, recoBuf, digestStr, pcSlice->getSPS()->getBitDepths());
				trailingSeiMessages.push_back(decodedPictureHashSei);
			}

			m_pcCfg->setEncodedFlag(iGOPid, true);

			Double PSNR_Y;
			xCalculateAddPSNRs(isField, isTff, iGOPid, pcPic, accessUnit, rcListPic, encTime, snr_conversion, printFrameMSE, &PSNR_Y);
			*/
			/*// Only produce the Green Metadata SEI message with the last picture.
			if (m_pcCfg->getSEIGreenMetadataInfoSEIEnable() && pcSlice->getPOC() == (m_pcCfg->getFramesToBeEncoded() - 1))
			{
			SEIGreenMetadataInfo *seiGreenMetadataInfo = new SEIGreenMetadataInfo;
			m_seiEncoder.initSEIGreenMetadataInfo(seiGreenMetadataInfo, (UInt)(PSNR_Y * 100 + 0.5));
			trailingSeiMessages.push_back(seiGreenMetadataInfo);
			}

			xWriteTrailingSEIMessages(trailingSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS());

			printHash(m_pcCfg->getDecodedPictureHashSEIType(), digestStr);

			if (m_pcCfg->getUseRateCtrl())
			{
			Double avgQP = m_pcRateCtrl->getRCPic()->calAverageQP();
			Double avgLambda = m_pcRateCtrl->getRCPic()->calAverageLambda();
			if (avgLambda < 0.0)
			{
			avgLambda = lambda;
			}

			m_pcRateCtrl->getRCPic()->updateAfterPicture(actualHeadBits, actualTotalBits, avgQP, avgLambda, pcSlice->getSliceType());
			m_pcRateCtrl->getRCPic()->addToPictureLsit(m_pcRateCtrl->getPicList());

			m_pcRateCtrl->getRCSeq()->updateAfterPic(actualTotalBits);
			if (pcSlice->getSliceType() != I_SLICE)
			{
			m_pcRateCtrl->getRCGOP()->updateAfterPicture(actualTotalBits);
			}
			else    // for intra picture, the estimated bits are used to update the current status in the GOP
			{
			m_pcRateCtrl->getRCGOP()->updateAfterPicture(estimatedBits);
			}
			#if U0132_TARGET_BITS_SATURATION
			if (m_pcRateCtrl->getCpbSaturationEnabled())
			{
			m_pcRateCtrl->updateCpbState(actualTotalBits);
			msg(NOTICE, " [CPB %6d bits]", m_pcRateCtrl->getCpbState());
			}
			#endif
			}
			
			xCreatePictureTimingSEI(m_pcCfg->getEfficientFieldIRAPEnabled() ? effFieldIRAPMap.GetIRAPGOPid() : 0, leadingSeiMessages, nestedSeiMessages, duInfoSeiMessages, pcSlice, isField, duData);
			if (m_pcCfg->getScalableNestingSEIEnabled())
			{
			xCreateScalableNestingSEI(leadingSeiMessages, nestedSeiMessages);
			}
			xWriteLeadingSEIMessages(leadingSeiMessages, duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS(), duData);
			xWriteDuSEIMessages(duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS(), duData);

			m_AUWriterIf->outputAU(accessUnit);
			*/
			//msg(NOTICE, "\n");
			//fflush(stdout);
		}
		
#if BLOCK_ENCODE
		

		//BlockReco加入BlocksReco
		num_block = 0;
		numMax = 0;
		for (Int i = 0; i < pcPic->getOrigBuf().Y().height; i += BLOCK_GEN_LEN)
		{
			for (Int j = 0; j < pcPic->getOrigBuf().Y().width; j += BLOCK_GEN_LEN)
			{
				if (BgBlock[num_block] > 0 && BgBlock[num_block] < 2000 && Bgselect1[num_block]>minwd)
				{
					//if (numMax > Maxx) //一次只编
						//break;
					numMax++;
					pcPic->CopyReco2Block(m_bgNewPicYuvRecoGop, j, i, pcPic);
					//BgBlock2[num_block] = 2000;
					BgBlock[num_block] = 2000 + pcPic->getPOC();
					//pcPic->CopyOrg2Block(m_bgNewPicYuvRecoGop, j, i, pcPic);
				}
				else if (BgBlock2[num_block] > 0 && BgBlock2[num_block] < 2000)
				{
					//pcPic->CopyReco2Block(m_bgNewPicYuvRecoGop, j, i, pcPic);
					//BgBlock2[num_block] = 2000;
				}
				num_block++;
			}
		}
		/*if (pcPic->getPOC() == 50)  //50
		{
			num_block = 0;
			for (Int i = 0; i < pcPic->getOrigBuf().Y().height; i += BLOCK_GEN_LEN)
			{
				for (Int j = 0; j < pcPic->getOrigBuf().Y().width; j += BLOCK_GEN_LEN)
				{

					if (BgBlock[num_block] == 0)
					{
						pcPic->CopyReco2Block(m_bgNewPicYuvRecoGop, j, i, pcPic);
					}
					num_block++;
				}
			}
		}*/
#else
		pcPic->CopyReco(pcPic, m_bgNewPicYuvRecoGop);
		pcPic->CopyOrg(pcPic,m_bgNewPicYuvRecoGop);
#endif
		
		pcPic->CopyOrg(m_bgNewPicYuvResiGop, pcPic);
		pcPic->CopyReco(m_bgNewPicYuvResiGop, pcPic);

		m_pcSliceEncoder->resetQPSlice(pcSlice, 0, iGOPid, isField);

		/*pcSlice->resetRefPicList(rcListPic, m_bgNewBlockRecoGop,setpic);
		pcSlice->setRefPOCList();
		pcSlice->setList1IdxToList0Idx();*/

		afterbg = true; //bg has encode
#endif
	}
#endif // ENCODE_BGPIC

	

	if (pcPic->getPOC() == 700)
	{
		Int num_block = 0;
		for (UInt uiH = 0; uiH < pcPic->getOrigBuf().Y().height; uiH += BLOCK_GEN_LEN)
		{
			for (UInt uiW = 0; uiW < pcPic->getOrigBuf().Y().width; uiW += BLOCK_GEN_LEN)
			{
				//if(BgBlock1[num_block]!=0)
				pcPic->DrawRef(uiW, uiH, pcPic, BgBlock1[num_block]);
				num_block++;
			}
		}
	}
	if (pcPic->getPOC() == 1000)
	{
		pcPic->DeleteOrg(pcPic);
		pcPic->DeleteReco(pcPic);
		pcPic->CopyOrg(m_bgNewPicYuvOrgGop, pcPic);
	}


	if (encPic)
		// now compress (trial encode) the various slice segments (slices, and dependent slices)
	{

		DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", pocCurr)));

		pcSlice->setSliceCurStartCtuTsAddr(0);
#if HEVC_DEPENDENT_SLICES
		pcSlice->setSliceSegmentCurStartCtuTsAddr(0);
#endif
		for (UInt nextCtuTsAddr = 0; nextCtuTsAddr < numberOfCtusInFrame; )
		{
			m_pcSliceEncoder->precompressSlice(pcPic);
#if BLOCK_RDO
			m_pcSliceEncoder->compressSliceRDO(pcPic, false, false, BgCTU, BlockLambda);
			cout << endl;
#else

			m_pcSliceEncoder->compressSlice(pcPic, false, false);

#endif

#if HEVC_DEPENDENT_SLICES
			const UInt curSliceSegmentEnd = pcSlice->getSliceSegmentCurEndCtuTsAddr();
			if (curSliceSegmentEnd < numberOfCtusInFrame)
			{
				const Bool bNextSegmentIsDependentSlice = curSliceSegmentEnd < pcSlice->getSliceCurEndCtuTsAddr();
				const UInt sliceBits = pcSlice->getSliceBits();
				UInt independentSliceIdx = pcSlice->getIndependentSliceIdx();
				pcPic->allocateNewSlice();
				// prepare for next slice
				m_pcSliceEncoder->setSliceSegmentIdx(uiNumSliceSegments);
				pcSlice = pcPic->slices[uiNumSliceSegments];
				CHECK(!(pcSlice->getPPS() != 0), "Unspecified error");
				pcSlice->copySliceInfo(pcPic->slices[uiNumSliceSegments - 1]);
				pcSlice->setSliceSegmentIdx(uiNumSliceSegments);
				if (bNextSegmentIsDependentSlice)
				{
					pcSlice->setSliceBits(sliceBits);
				}
				else
				{
					pcSlice->setSliceCurStartCtuTsAddr(curSliceSegmentEnd);
					pcSlice->setSliceBits(0);
					independentSliceIdx++;
				}
				pcSlice->setIndependentSliceIdx(independentSliceIdx);
				pcSlice->setDependentSliceSegmentFlag(bNextSegmentIsDependentSlice);
				pcSlice->setSliceSegmentCurStartCtuTsAddr(curSliceSegmentEnd);
				// TODO: optimise cabac_init during compress slice to improve multi-slice operation
				// pcSlice->setEncCABACTableIdx(m_pcSliceEncoder->getEncCABACTableIdx());
				uiNumSliceSegments++;
			}
			nextCtuTsAddr = curSliceSegmentEnd;
#else
			const UInt curSliceEnd = pcSlice->getSliceCurEndCtuTsAddr();
			if (curSliceEnd < numberOfCtusInFrame)
			{
				UInt independentSliceIdx = pcSlice->getIndependentSliceIdx();
				pcPic->allocateNewSlice();
				m_pcSliceEncoder->setSliceSegmentIdx(uiNumSliceSegments);
				// prepare for next slice
				pcSlice = pcPic->slices[uiNumSliceSegments];
				CHECK(!(pcSlice->getPPS() != 0), "Unspecified error");
				pcSlice->copySliceInfo(pcPic->slices[uiNumSliceSegments - 1]);
				pcSlice->setSliceCurStartCtuTsAddr(curSliceEnd);
				pcSlice->setSliceBits(0);
				independentSliceIdx++;
				pcSlice->setIndependentSliceIdx(independentSliceIdx);
				uiNumSliceSegments++;
			}
			nextCtuTsAddr = curSliceEnd;
#endif
		}
#if BLOCK_SELECT
		//if (isselect&&pcPic->getPOC()<50)
		if (pcPic->getPOC()>7)
		{

			//pcSlice->resetRefPicListRec(rcListPic, m_rcPicYuvTempGop, SetRefPoc);

			int blockSize = BLOCK_GEN_LEN;
			int numsx = (pcPic->getOrigBuf().Y().width + blockSize - 1) / blockSize;
			int numsy = (pcPic->getOrigBuf().Y().height + blockSize - 1) / blockSize;
			int j = 0;
			cout << endl;
			for (int i = 0; i < pcPic->cs->pus.size(); i++)
			{
				int refIdx = pcPic->cs->pus[i]->refIdx[0];
				
				cout<< pcPic->cs->pus[i]->lx()<<"-"<< pcPic->cs->pus[i]->ly()<<"-"<< pcPic->cs->pus[i]->lwidth()<<"-"<< pcPic->cs->pus[i]->lheight()<<" ";
				if (refIdx == 3)//将要判断的rec放入REF的3中
				{
					int lx = pcPic->cs->pus[i]->lx();
					int ly = pcPic->cs->pus[i]->ly();
					int lwidth = pcPic->cs->pus[i]->lwidth();
					int lheight = pcPic->cs->pus[i]->lheight();
					int rx = lx + lwidth;
					int ry = ly + lheight;
					int lblockx = lx / blockSize;
					int lblocky = ly / blockSize;
					int rblockx = (rx - 1) / blockSize;
					int rblocky = (ry - 1) / blockSize;
					/*if (lblockx - 1 >= 0) lblockx -= 1;  //扩大一圈
					if (lblocky - 1 >= 0) lblocky -= 1;
					if (rblockx + 1 <= numsx) rblockx += 1;
					if (rblocky + 1 <= numsy) rblocky += 1;*/
					for (int x = lblockx; x <= rblockx; x++)
					{
						for (int y = lblocky; y <= rblocky; y++)
						{
							if (BgBlock[y*numsx + x] > 0)//&& BgBlock[y*numsx + x]<1000)
							//if (BgCTU[y*numsx + x] > 0 && BgCTU[y*numsx + x] < 1000)
							{//已有 且未编   编码
								BlockSel[y*numsx + x]++;
							}
						}
					}

					//int blockx = ly / blockSize*numsx + lx / blockSize;
					//double P = lwidth*lheight / (blockSize*blockSize);
					//BlockSel[blockx]++;
				}
			}
			cout << endl;
			for (int i = 0; i < numsx*numsy; i++)
			{
				cout << BlockSel[i] << " ";
				if (BlockSel[i] != 0)
				{
					j++;
					BgBlock2[i]++;
				}
				BlockSel[i] = 0;
				
			}
			cout << endl << numsx*numsy << "BlockSel-" << j << endl;
			j = 0;
			for (int i = 0; i < numsx*numsy; i++)
			{
				cout << BgBlock2[i] << "-"<<BgBlock[i]<<" ";
				if (BgBlock2[i] != 0)
					j++;
				if (BgBlock2[i] > 0 && BgBlock2[i] < 2000)
					isselectencode = 1;
			}
			cout << endl << numsx*numsy << "BgBlock2-" << j << endl;
		}
		/*if (pcPic->getPOC() == 10)
		{
			Int num_block = 0;
			for (UInt uiH = 0; uiH < pcPic->getOrigBuf().Y().height; uiH += BLOCK_GEN_LEN)
			{
				for (UInt uiW = 0; uiW < pcPic->getOrigBuf().Y().width; uiW += BLOCK_GEN_LEN)
				{
					pcPic->DrawRef(uiW, uiH, pcPic, BlockSel[num_block]);
					num_block++;
				}
			}
		}*/
		
#endif
#if PRINT_PUREF
	  if (pcPic->getPOC() == BGPICPOC)
	  {
		  //ofstream pufile;
		  //pufile.open("pufile.txt");
	  }
#endif
	  
#if BBB
#if HIERARCHY_GENETATE_BGP //Rec
	  if (pcPic->getPOC() != 0)
	  {
#if israndom
		  if (pcPic->getPOC() % 32 != 0)
		  {
#endif // israndom

			  const Int  iWidth = pcPic->getRecoBuf().Y().width;
			  const Int  iHeight = pcPic->getRecoBuf().Y().height;
#if PRINT_DIFF
			  ofstream os("test.txt", ofstream::app);
#endif
			  for (Int i = 0; i < iHeight; i += UNIT_LEN)
			  {
				  for (Int j = 0; j < iWidth; j += UNIT_LEN)
				  {
					  Int level = 0;
					  Bool divflag = false;

					  CompDiff(j, i, pcPic, level, divflag
#if PRINT_DIFF
						  , os
#endif
					  );

				  }
#if PRINT_DIFF
				  os << endl;
#endif
			  }
#if PRINT_DIFF
			  os.close();
#endif
#if israndom
		  }
#endif // israndom

		}
	  else
	  {
		  Pel* piBac;
		  Pel* piOrg;

		  UInt uiStride;
		  UInt uiPicHeight;
		  UInt uiPicWidth;

		  for (Int compId = 0; compId < 3; compId++)
		  {
			  switch (compId)
			  {
			  case 0:
				  piBac = m_bgNewPicYuvRecGop->getRecoBuf().Y().bufAt(0, 0);
				  piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);

				  uiStride = pcPic->getRecoBuf().Y().stride;
				  uiPicHeight = pcPic->getRecoBuf().Y().height;
				  uiPicWidth = pcPic->getRecoBuf().Y().width;
				  break;
			  case 1:
				  piBac = m_bgNewPicYuvRecGop->getRecoBuf().Cb().bufAt(0, 0);
				  piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);

				  uiStride = pcPic->getRecoBuf().Cb().stride;
				  uiPicHeight = pcPic->getRecoBuf().Cb().height;
				  uiPicWidth = pcPic->getRecoBuf().Cb().width;
				  break;
			  case 2:
				  piBac = m_bgNewPicYuvRecGop->getRecoBuf().Cr().bufAt(0, 0);
				  piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);

				  uiStride = pcPic->getRecoBuf().Cr().stride;
				  uiPicHeight = pcPic->getRecoBuf().Cr().height;
				  uiPicWidth = pcPic->getRecoBuf().Cr().width;
				  break;
			  default:
				  break;
			  }
			  for (Int unit_i = 0; unit_i < uiPicHeight; unit_i++)
			  {
				  for (Int unit_j = 0; unit_j < uiPicWidth; unit_j++)
				  {
					  piBac[unit_j] = 0;
				  }
				  piOrg += uiStride;
				  piBac += uiStride;
			  }
		  }
	  }
#endif


#if BG_BLOCK_SUBSTITUTION
	  if (pcPic->getPOC() != 0)
	  {
#if israndom
		  if (pcPic->getPOC() % 32 != 0)
		  {

#endif // israndom

			  const UInt  uiWidth = pcPic->getRecoBuf().Y().width;
			  const UInt  uiHeight = pcPic->getRecoBuf().Y().height;
			  for (UInt uiH = 0; uiH < uiHeight; uiH += BLOCK_LEN)
			  {
				  for (UInt uiW = 0; uiW < uiWidth; uiW += BLOCK_LEN)
				  {
					  Double Fc[3];
					  for (Int transId = 0; transId < 3; transId++)
					  {
						  Fc[transId] = CompNabla(transId, uiH, uiW, pcPic);
					  }
					  Double dNabla = Fc[0] + Fc[1] / 4 + Fc[2] / 4;

					  if (dNabla < 1.5)   // if Nabla < Omegaba then substitute
					  {
						  pcPic->Copy2BackPic(m_bgNewPicYuvRecGop, uiW, uiH, pcPic);
					  }
					  else if (dNabla >= 1.5 && dNabla < 3.5 && uiH != 0 && uiH != (uiHeight - BLOCK_LEN) && uiW != 0 && uiW != (uiWidth - BLOCK_LEN))  // Omegaba < Nabla < Omega
					  {
						  Double DiffOrg = 0;
						  Double DiffBG = 0;

						  DiffOrg += CompBlockDiff(uiH, uiW, pcPic, pcPic); //maybe
						  DiffBG += CompBlockDiff(uiH, uiW, m_bgNewPicYuvRecGop, pcPic);

						  if (DiffOrg < DiffBG)
						  {
							  pcPic->Copy2BackPic(m_bgNewPicYuvRecGop, uiW, uiH, pcPic);
						  }
					  }
				  }
			  }
#if israndom
		  }
#endif // israndom

	  }
	  /*else
	  {
		  Pel* piBac;
		  Pel* piOrg;

		  UInt uiStride;
		  UInt uiPicHeight;
		  UInt uiPicWidth;

		  for (Int compId = 0; compId < 3; compId++)
		  {
			  switch (compId)
			  {
			  case 0:
				  piBac = m_bgNewPicYuvRecGop->getRecoBuf().Y().bufAt(0, 0);
				  piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);

				  uiStride = pcPic->getRecoBuf().Y().stride;
				  uiPicHeight = pcPic->getRecoBuf().Y().height;
				  uiPicWidth = pcPic->getRecoBuf().Y().width;
				  break;
			  case 1:
				  piBac = m_bgNewPicYuvRecGop->getRecoBuf().Cb().bufAt(0, 0);
				  piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);

				  uiStride = pcPic->getRecoBuf().Cb().stride;
				  uiPicHeight = pcPic->getRecoBuf().Cb().height;
				  uiPicWidth = pcPic->getRecoBuf().Cb().width;
				  break;
			  case 2:
				  piBac = m_bgNewPicYuvRecGop->getRecoBuf().Cr().bufAt(0, 0);
				  piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);

				  uiStride = pcPic->getRecoBuf().Cr().stride;
				  uiPicHeight = pcPic->getRecoBuf().Cr().height;
				  uiPicWidth = pcPic->getRecoBuf().Cr().width;
				  break;
			  default:
				  break;
			  }
			  for (Int unit_i = 0; unit_i < uiPicHeight; unit_i++)
			  {
				  for (Int unit_j = 0; unit_j < uiPicWidth; unit_j++)
				  {
					  piBac[unit_j] = piOrg[unit_j];
				  }
				  piOrg += uiStride;
				  piBac += uiStride;
			  }
		  }
	  }*/
#endif

#if !BLOCK_GEN //生成块

	  if (pcPic->getPOC() > 5 && pcPic->getPOC() < 300)
	  {
		  int num_block = 0;
		  for (UInt uiH = 0; uiH < pcPic->getRecoBuf().Y().height; uiH += BLOCK_GEN_LEN)
		  {
			  for (UInt uiW = 0; uiW < pcPic->getRecoBuf().Y().width; uiW += BLOCK_GEN_LEN)
			  {

				  if (BgBlock[num_block] != 0)
				  {
					  BgBlock[num_block]++;
				  }
				  else
				  {
					  double diff = 0;

					  //pcPic->CompBlockDiff(uiW, uiH, pcPic, diff);  //get  block diff
					  pcPic->CompBlockPicRecoDiff(uiW, uiH, pcPic, m_bgNewPicYuvRecGop, diff);//计算Rec与PicRec的diff

					  cout << "diff" << diff;
					  if (diff < 1)
					  {
						  BgBlock[num_block]++;
						  isencode = 1; //确定编
						  pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, uiW, uiH, m_bgNewPicYuvOrgGop);  //用Rec判断 Org有可能为空。
						  //pcPic->CopyReco2Block(m_bgNewBlocksRecGop, uiW, uiH, m_bgNewPicYuvRecGop);
						  //pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, uiW, uiH, pcPic);  //block 编码
					  }
				  }


				  num_block++;  //UNITxUNIT
			  }
		  }
		  cout << "num_block" << num_block << endl;

		  { // cout
			  Int i = 0;
			  Int j = 0;
			  
			  cout << endl;
			  while (i < num_block)
			  {
				  cout << BgBlock[i] << " ";
				  if (BgBlock[i] != 0)
					  j++;
				  i++;
			  }

			  cout << endl;
			  cout << " j " << j << endl;
		  }
	  }

#endif
	 



#endif


#if BG_REFERENCE_SUBSTITUTION
	  if (afterbg&&SetRefPoc!=-999//&&pcSlice->getPOC()!=BGPICPOC
#if israndom
		  &&pcSlice->getPOC()%16!=0
#endif
		  )  
	  {
		  pcSlice->resetRefPicList(rcListPic, m_rcPicYuvTempGop, SetRefPoc); //还原参考帧列表
	  }
#endif // BG_REFERENCE_SUBSTITUTION

      duData.clear();



      CodingStructure& cs = *pcPic->cs;
      pcSlice = pcPic->slices[0];

      // SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas
      if( pcSlice->getSPS()->getUseSAO() && m_pcCfg->getSaoCtuBoundary() )
      {
        m_pcSAO->getPreDBFStatistics( cs );
      }
      //-- Loop filter
      if ( m_pcCfg->getDeblockingFilterMetric() )
      {
  #if W0038_DB_OPT
        if ( m_pcCfg->getDeblockingFilterMetric()==2 )
        {
          applyDeblockingFilterParameterSelection(pcPic, uiNumSliceSegments, iGOPid);
        }
        else
        {
  #endif
          applyDeblockingFilterMetric(pcPic, uiNumSliceSegments);
  #if W0038_DB_OPT
        }
  #endif
      }

      m_pcLoopFilter->loopFilterPic( cs );
      DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "final", 1 ) ) );

      if( pcSlice->getSPS()->getUseSAO() )
      {
        Bool sliceEnabled[MAX_NUM_COMPONENT];
        m_pcSAO->initCABACEstimator( m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), pcSlice );
        m_pcSAO->SAOProcess(cs, sliceEnabled, pcSlice->getLambdas(), m_pcCfg->getTestSAODisableAtPictureLevel(), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma(), m_pcCfg->getSaoCtuBoundary());
        //assign SAO slice header
        for(Int s=0; s< uiNumSliceSegments; s++)
        {
          pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, sliceEnabled[COMPONENT_Y]);
          CHECK(!(sliceEnabled[COMPONENT_Cb] == sliceEnabled[COMPONENT_Cr]), "Unspecified error");
          pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, sliceEnabled[COMPONENT_Cb]);
        }
      }
    }
    else // skip enc picture
    {
      pcSlice->setSliceQpBase( pcSlice->getSliceQp() );

      if( pcSlice->getSPS()->getUseSAO() )
      {
        m_pcSAO->disabledRate( *pcPic->cs, pcPic->getSAO(1), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma());
      }
    }
    if( m_pcCfg->getUseAMaxBT() )
    {
      for( const CodingUnit *cu : pcPic->cs->cus )
      {
        if( !pcSlice->isIntra() )
        {
          m_uiBlkSize[pcSlice->getDepth()] += cu->Y().area();
          m_uiNumBlk [pcSlice->getDepth()]++;
        }
      }
    }
    if( encPic || decPic )
    {
      pcSlice = pcPic->slices[0];

      /////////////////////////////////////////////////////////////////////////////////////////////////// File writing

      // write various parameter sets
      actualTotalBits += xWriteParameterSets( accessUnit, pcSlice, m_bSeqFirst );

      if ( m_bSeqFirst )
      {
        // create prefix SEI messages at the beginning of the sequence
        CHECK(!(leadingSeiMessages.empty()), "Unspecified error");
        xCreateIRAPLeadingSEIMessages(leadingSeiMessages, pcSlice->getSPS(), pcSlice->getPPS());

        m_bSeqFirst = false;
      }
      if (m_pcCfg->getAccessUnitDelimiter())
      {
        xWriteAccessUnitDelimiter(accessUnit, pcSlice);
      }

      // reset presence of BP SEI indication
      m_bufferingPeriodSEIPresentInAU = false;
      // create prefix SEI associated with a picture
      xCreatePerPictureSEIMessages(iGOPid, leadingSeiMessages, nestedSeiMessages, pcSlice);

      // pcSlice is currently slice 0.
      std::size_t binCountsInNalUnits   = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)
      std::size_t numBytesInVclNalUnits = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)
#if HEVC_DEPENDENT_SLICES
      for( UInt sliceSegmentStartCtuTsAddr = 0, sliceSegmentIdxCount=0; sliceSegmentStartCtuTsAddr < numberOfCtusInFrame; sliceSegmentIdxCount++, sliceSegmentStartCtuTsAddr=pcSlice->getSliceSegmentCurEndCtuTsAddr() )
#else
      for(UInt sliceSegmentStartCtuTsAddr = 0, sliceSegmentIdxCount = 0; sliceSegmentStartCtuTsAddr < numberOfCtusInFrame; sliceSegmentIdxCount++, sliceSegmentStartCtuTsAddr = pcSlice->getSliceCurEndCtuTsAddr())
#endif
      {
        pcSlice = pcPic->slices[sliceSegmentIdxCount];
        if(sliceSegmentIdxCount > 0 && pcSlice->getSliceType()!= I_SLICE)
        {
          pcSlice->checkColRefIdx(sliceSegmentIdxCount, pcPic);
        }
        m_pcSliceEncoder->setSliceSegmentIdx(sliceSegmentIdxCount);

        pcSlice->setRPS   (pcPic->slices[0]->getRPS());
        pcSlice->setRPSidx(pcPic->slices[0]->getRPSidx());

        for ( UInt ui = 0 ; ui < numSubstreams; ui++ )
        {
          substreamsOut[ui].clear();
        }

        /* start slice NALunit */
        OutputNALUnit nalu( pcSlice->getNalUnitType(), pcSlice->getTLayer() );
        m_HLSWriter->setBitstream( &nalu.m_Bitstream );

        pcSlice->setNoRaslOutputFlag(false);
        if (pcSlice->isIRAP())
        {
          if (pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_IDR_N_LP)
          {
            pcSlice->setNoRaslOutputFlag(true);
          }
          //the inference for NoOutputPriorPicsFlag
          // KJS: This cannot happen at the encoder
          if (!m_bFirst && pcSlice->isIRAP() && pcSlice->getNoRaslOutputFlag())
          {
            if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
            {
              pcSlice->setNoOutputPriorPicsFlag(true);
            }
          }
        }

        tmpBitsBeforeWriting = m_HLSWriter->getNumberOfWrittenBits();
        m_HLSWriter->codeSliceHeader( pcSlice );
#if BLOCK_ENCODE
		pcPic->CopyReco2Org(pcPic, PrePicRecoGop); //将pcpic的Reco放入BlockReco的Org中
		pcPic->CopyReco(pcPic, PrePicRecoGop);
#endif
#if TRANSFORM_BGP
		if (pcPic->getPOC() == BGPICPOC)
		{
			//TransformBGP
			const UInt uiPicHeight = pcPic->getPicYuvOrg()->getHeight();
			const UInt uiPicWidth = pcPic->getPicYuvOrg()->getWidth();

			UInt uiAbsPartIdx = 0;
			for (UInt uiH = 0; uiH < uiPicHeight; uiH += TRANS_UNIT_LEN)
				for (UInt uiW = 0; uiW < uiPicWidth; uiW += TRANS_UNIT_LEN)
				{
					for (Int transId = 0; transId < 3; transId++)
					{
						TransformBGP(transId, uiAbsPartIdx, uiH, uiW, pcSlice->getSliceQp() - 17);
					}
					uiAbsPartIdx++;
				}
		}
#endif	
#if BLOCK_RDO
		if (pcPic->getPOC() < 300)
		{
			//计算dpp
			int num_block = 0;
			for (UInt uiH = 0; uiH < pcPic->getOrigBuf().Y().height; uiH += BLOCK_GEN_LEN)
			{
				for (UInt uiW = 0; uiW < pcPic->getOrigBuf().Y().width; uiW += BLOCK_GEN_LEN)
				{
					//if (BgBlock[num_block] != 0 && BgBlock[num_block] < 1000) //是背景块计算dpp
					{
						double dpp = 0; //计算dpp加入BlockDPP[]
						pcPic->CompBlockPicdpp(uiW, uiH, pcPic, dpp); //计算该帧 该块的dpp 需要org和reco。
						BlockDPP[num_block] = (BlockDPP[num_block] + dpp) / 2; //该帧之前包括该帧 所得到的dpp
						cout << BlockDPP[num_block] << " ";
					}
					num_block++;
				}
			}
			cout << "Dpp" << endl;
		}
		/*int num_CTU = 0;
		for (UInt uiH = 0; uiH < pcPic->getOrigBuf().Y().height; uiH += BLOCK_CTU)
		{
			for (UInt uiW = 0; uiW < pcPic->getOrigBuf().Y().width; uiW += BLOCK_CTU)
			{
				if (BgCTU[num_CTU] > 1000 && BgCTU[num_CTU] < 2000) //刚编码的背景块  将编码后的背景块加入参考
				{
					pcPic->CopyReco2CTU(m_bgNewPicYuvRecoGop, uiW, uiH, pcPic);
					BgCTU[num_CTU] += 1000; //代表已编>2000
					afterbg = 1;
				}
				num_CTU++;
			}
		}*/
#endif
#if !BBB
#if HIERARCHY_GENETATE_OrgBGP
		if (pcPic->getPOC() != 0)  
		{

#if israndom  //32时没有参考帧
			if (pcPic->getPOC() % 32 != 0)
			{
#endif // israndom

				const Int  iWidth = pcPic->getOrigBuf().Y().width;
				const Int  iHeight = pcPic->getOrigBuf().Y().height;
#if PRINT_OrgDIFF
				ofstream os("test.txt", ofstream::app);
#endif
				for (Int i = 0; i < iHeight; i += UNIT_LEN)
				{
					for (Int j = 0; j < iWidth; j += UNIT_LEN)
					{
						Int level = 0; //分几级
						Bool divflag = false; //是否再分
						CompDiffOrg(j, i, pcPic, level, divflag
#if PRINT_OrgDIFF
							, os
#endif
						);


					}
#if PRINT_OrgDIFF
					os << endl;
#endif
				}
				

#if PRINT_OrgDIFF
				os.close();
#endif
#if israndom
			}
#endif // israndom

		}
		else
		{
			Pel* piBac;
			Pel* piOrg;

			UInt uiStride;
			UInt uiPicHeight;
			UInt uiPicWidth;
			

			for (Int compId = 0; compId < 3; compId++)
			{
				switch (compId)
				{
				case 0:
					piBac = m_bgNewPicYuvOrgGop->getOrigBuf().Y().bufAt(0,0);
					piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);

					uiStride = pcPic->getOrigBuf().Y().stride;
					uiPicHeight = pcPic->getOrigBuf().Y().height;
					uiPicWidth = pcPic->getOrigBuf().Y().width;
					break;
				case 1:
					piBac = m_bgNewPicYuvOrgGop->getOrigBuf().Cb().bufAt(0, 0);
					piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);

					uiStride = pcPic->getOrigBuf().Cb().stride;
					uiPicHeight = pcPic->getOrigBuf().Cb().height;
					uiPicWidth = pcPic->getOrigBuf().Cb().width;
					break;
				case 2:
					piBac = m_bgNewPicYuvOrgGop->getOrigBuf().Cr().bufAt(0, 0);
					piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);

					uiStride = pcPic->getOrigBuf().Cr().stride;
					uiPicHeight = pcPic->getOrigBuf().Cr().height;
					uiPicWidth = pcPic->getOrigBuf().Cr().width ;
					break;
				default:
					break;
				}

				for (Int unit_i = 0; unit_i < uiPicHeight; unit_i++)
				{
					for (Int unit_j = 0; unit_j < uiPicWidth; unit_j++)
					{
						piBac[unit_j] = 0;   //-------------315初始为0
					}
					piOrg += uiStride;
					piBac += uiStride;
				}
			}
		}
#endif

#if OrgBG_BLOCK_SUBSTITUTION
		if (pcPic->getPOC() != 0 )
		{//空间相关性
#if israndom
			if (pcPic->getPOC() % 32 != 0)
			{
#endif // israndom

				const UInt  uiWidth = pcPic->getOrigBuf().Y().width;
				const UInt  uiHeight = pcPic->getOrigBuf().Y().height;
#if PRINT_FCVALUE
				ofstream ocout("test.txt", ofstream::app);
#endif
				for (UInt uiH = 0; uiH < uiHeight; uiH += BLOCK_LEN)
				{
					for (UInt uiW = 0; uiW < uiWidth; uiW += BLOCK_LEN)
					{
						Double Fc[3];
						for (Int transId = 0; transId < 3; transId++)
						{
							Fc[transId] = CompNablaOrg(transId, uiH, uiW, pcPic);
						}
						Double dNabla = Fc[0] + Fc[1] / 4 + Fc[2] / 4;
#if PRINT_FCVALUE
						if (ocout.is_open())
						{
							if (uiW % 128 == 0 && uiW != 0)
								ocout << endl;
							ocout << setiosflags(ios::right) << setw(10) << dNabla;
						}
#endif
						if (dNabla < 1.5)   // if Nabla < Omegaba then substitute
						{
							//cout << "in orgblock <1.5" << endl;
							
							pcPic->Copy2OrgBackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic); //m_bgNewPicYuvOrgGop=pcpic

						}
						else if (/*Omegaba < Nabla < Omega*/ dNabla >= 1.5 && dNabla < 3.5 && uiH != 0 && uiH != (uiHeight - BLOCK_LEN) && uiW != 0 && uiW != (uiWidth - BLOCK_LEN))  // Omegaba < Nabla < Omega
						{
							Double DiffOrg = 0;
							Double DiffBG = 0;

							DiffOrg += CompBlockDiff(uiH, uiW, pcPic, pcPic);
							DiffBG += CompBlockDiff(uiH, uiW, m_bgNewPicYuvOrgGop, pcPic);
							if (DiffOrg < DiffBG)
							{
								//cout << "in orgblock DiffOrg < DiffBG" << endl;							
								pcPic->Copy2OrgBackPic(m_bgNewPicYuvOrgGop, uiW, uiH, pcPic);
							}
						}

					}
#if PRINT_FCVALUE
					ocout << endl;
#endif
				}
#if PRINT_FCVALUE
				ocout.close();
#endif
#if israndom
			}
#endif // israndom

		}
		/*else
		{
			Pel* piBac;
			Pel* piOrg;

			UInt uiStride;
			UInt uiPicHeight;
			UInt uiPicWidth;

			for (Int compId = 0; compId < 3; compId++)
			{
				switch (compId)
				{
				case 0:
					piBac = m_bgNewPicYuvOrgGop->getOrigBuf().Y().bufAt(0,0);
					piOrg = pcPic->getOrigBuf().Y().bufAt(0,0);

					uiStride = pcPic->getOrigBuf().Y().stride;
					uiPicHeight = pcPic->getOrigBuf().Y().height;
					uiPicWidth = pcPic->getOrigBuf().Y().width;
					break;
				case 1:
					piBac = m_bgNewPicYuvOrgGop->getOrigBuf().Cb().bufAt(0, 0);
					piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);

					uiStride = pcPic->getOrigBuf().Cb().stride;
					uiPicHeight = pcPic->getOrigBuf().Cb().height;
					uiPicWidth = pcPic->getOrigBuf().Cb().width;
					break;
				case 2:
					piBac = m_bgNewPicYuvOrgGop->getOrigBuf().Cr().bufAt(0, 0);
					piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);

					uiStride = pcPic->getOrigBuf().Cr().stride;
					uiPicHeight = pcPic->getOrigBuf().Cr().height;
					uiPicWidth = pcPic->getOrigBuf().Cr().width;
					break;
				default:
					break;
				}
				for (Int unit_i = 0; unit_i < uiPicHeight; unit_i++)
				{
					for (Int unit_j = 0; unit_j < uiPicWidth; unit_j++)
					{	
						piBac[unit_j] = piOrg[unit_j];
					}
					piOrg += uiStride;
					piBac += uiStride;
				}
			}
		}*/
#endif
#if BLOCK_GEN //生成块

		if (pcPic->getPOC() > 5 && pcPic->getPOC() < 300)
		{
			int num_block = 0;
			for (UInt uiH = 0; uiH < pcPic->getOrigBuf().Y().height; uiH += BLOCK_GEN_LEN)
			{
				for (UInt uiW = 0; uiW < pcPic->getOrigBuf().Y().width; uiW += BLOCK_GEN_LEN)
				{
					if (BgBlock[num_block] < 1000)
					{
						double diff = 0;

						//pcPic->CompBlockDiff(uiW, uiH, pcPic, diff);  //get  block diff
						pcPic->CompBlockPicOrgDiff(uiW, uiH, pcPic, m_bgNewPicYuvOrgGop, diff);//判断背景帧与当前帧得到背景块
						//pcPic->CompBlockPicRecoDiff(uiW, uiH, pcPic, m_bgNewPicYuvRecGop, diff);//计算Rec与PicRec的diff
						cout << "diff" << diff;
						if (diff < 8)//if(pcPic->CompBlockOrgIsFull(uiW, uiH, m_bgNewPicYuvOrgGop))//判断背景帧该块是否已满
						{
							BgBlock[num_block]++;
							isencode = true; //确定编
							isselect = true;
							pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, uiW, uiH, m_bgNewPicYuvOrgGop);
							//pcPic->CopyReco2Block(m_bgNewBlocksOrgGop, uiW, uiH, m_bgNewPicYuvOrgGop);//双背景
							//pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, uiW, uiH, pcPic);  //block 编码
						}
					}


					/*{ //更新
					double diff = 0;
					pcPic->CompBlockPicOrgDiff(uiW, uiH, pcPic, m_bgNewPicYuvOrgGop, diff);
					if (diff < 12)
					{
					if (BgBlock[num_block] == 0)
					{
					BgBlock[num_block] = 1;
					isencode = 1; //确定编
					pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, uiW, uiH, m_bgNewPicYuvOrgGop);
					}
					else if (BgBlock[num_block] > 15) //已有 且不相似
					{//考虑更新：判断BlocksOrg与PicYuvOrg 相似则不更新、不相似则更新。
					double updatediff1 = 0;
					pcPic->CompBlockPicOrgDiff(uiW, uiH, m_bgNewBlocksOrgGop, m_bgNewPicYuvOrgGop, updatediff1);
					if (updatediff1 > 50)
					{
					if (pcPic->IsEmpty(m_bgNewBlockRecGop, uiW, uiH, 0)) //判断更新第三者
					{
					pcPic->CopyOrg2Block(m_bgNewBlockRecGop, uiW, uiH, m_bgNewPicYuvOrgGop);
					}
					else
					{
					double updatediff2 = 0;
					pcPic->CompBlockPicOrgDiff(uiW, uiH, m_bgNewBlockRecGop, m_bgNewPicYuvOrgGop, updatediff2);
					if (updatediff2 < 12)//则为新的背景块
					{
					BgBlock[num_block] = 1;
					isupdate = true;//确定更新
					pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, uiW, uiH, m_bgNewPicYuvOrgGop);
					updatenum++;
					}
					else
					{
					pcPic->CopyOrg2Block(m_bgNewBlockRecGop, uiW, uiH, m_bgNewPicYuvOrgGop);
					}
					}
					}
					}
					}
					}*/

					num_block++;  //UNITxUNIT
				}
			}
			//BgBlock[num_block] = -1;
			cout << "num_block" << num_block << endl;

			{ // cout
				Int i = 0;
				Int j = 0;
				//ofstream bgBlock;
				//bgBlock.open("D://2//BgBlock32_5_8补50.txt", ios::app);
				//bgBlock << pcPic->getPOC()<<" ";
				cout << endl;
				while (i < num_block)
				{
					cout << BgBlock[i] << " ";// << BlockDPP[i] << " ";
										  //bgBlock << BgBlock[i] <<" ";					
					if (BgBlock[i] > 0)
					{
						j++;
					}
					i++;
				}
				//bgBlock << endl;
				//bgBlock <<" j "<< j << endl;
				cout << endl;
				cout << " 已生成的背景块 " << j << endl;
			}

		}
		
	
#endif 
#if BLOCK_RDO
		if (pcPic->getPOC() > 5 && pcPic->getPOC() < 300)
		{
			Int num_CTU = 0;
			for (UInt uiH = 0; uiH < (pcPic->getOrigBuf().Y().height + BLOCK_CTU - 1) / BLOCK_CTU; uiH++)
			{
				for (UInt uiW = 0; uiW < pcPic->getOrigBuf().Y().width / BLOCK_CTU; uiW++)
				{
					Int Bstride = pcPic->getOrigBuf().Y().width / BLOCK_GEN_LEN; //40
					Int CTUnumBlock = BLOCK_CTU / BLOCK_GEN_LEN;  //4
					Double num = 0;
					Double Sum = 0;
					for (Int i = 0; i < CTUnumBlock&&i*BLOCK_GEN_LEN + uiH*BLOCK_CTU <= pcPic->getOrigBuf().Y().height; i++)
					{
						for (Int j = 0; j < CTUnumBlock; j++)
						{
							if (BgBlock[uiH*Bstride*CTUnumBlock + uiW*CTUnumBlock + i*Bstride + j] >= 5 && BgBlock[uiH*Bstride*CTUnumBlock + uiW*CTUnumBlock + i*Bstride + j] < 2000)
							{
								num++;
							}
							Sum++;
						}
					}
					if (num == Sum && BgCTU[num_CTU] == 0)//CTU内大部分块 都大于等于5 即认为该CTU为BCTU
					{
						//编
						//计算该CTU的M
						CTUisencode = true;
						Double P = 0;
						Int Blocknum = 0;
						for (Int i = 0; i < CTUnumBlock; i++)
						{
							for (Int j = 0; j < CTUnumBlock; j++)
							{
								Int num_block = uiH*Bstride*CTUnumBlock + uiW*CTUnumBlock + i*Bstride + j;
								Double Plowerdpp = 0;
								Int H = num_block / Bstride;
								Int W = num_block%Bstride;
								pcPic->CompBlockPicBgPdpp(W, H, pcPic, m_bgNewBlocksOrgGop, BlockDPP[num_block], Plowerdpp);
								P += Plowerdpp*BgBlock[num_block] / (pcPic->getPOC() - 5);
								BgBlock[num_block] = 2000; //该块已被CTU编码
								Blocknum++;
							}
						}
						P = P / Blocknum;
						Int M = int(P*(m_pcCfg->getFramesToBeEncoded() - pcPic->getPOC()));
						BgCTU[num_CTU] = M + 1000;
				}
					num_CTU++;
			}
		}

			{ // cout
				Int i = 0;
				Int j = 0;
				cout << endl;
				while (i < num_CTU)
				{
					cout << BgCTU[i] << " ";
					if (BgCTU[i] > 1000)
					{
						j++;
					}
					i++;
				}
				//bgBlock << endl;
				//bgBlock <<" j "<< j << endl;
				cout << endl;
				cout << " CTUj " << j << endl;
			}
	}
#endif
#endif
#if UPDATE_BGP
		if (pcPic->getPOC() == 250/* % 300 == 0 && pcPic->getPOC() > 0*/)
		{
			//TransformBGP
			const UInt uiPicHeight = pcPic->getPicYuvOrg()->getHeight();
			const UInt uiPicWidth = pcPic->getPicYuvOrg()->getWidth();

			UInt uiAbsPartIdx = 0;

#if PRINT_UPDATEBG_RESI || PRINT_UPDATE_TRCOEFF
			Int i = 0;
			ofstream ocout("test.txt", ofstream::app);

#endif
			/*
			memset(m_bgNewPicYuvUpdateResiGop->getLumaAddr(), 0, uiPicHeight * uiPicWidth * sizeof(Pel));
			memset(m_bgNewPicYuvUpdateResiGop->getCbAddr(), 0, (uiPicHeight >> 1) * (uiPicWidth >> 1) * sizeof(Pel));
			memset(m_bgNewPicYuvUpdateResiGop->getCrAddr(), 0, (uiPicHeight >> 1) * (uiPicWidth >> 1) * sizeof(Pel));
			*/
			for (Int iH = 0; iH < uiPicHeight; iH += TRANS_UNIT_LEN)
				for (Int iW = 0; iW < uiPicWidth; iW += TRANS_UNIT_LEN)
				{
#if PRINT_UPDATEBG_RESI || PRINT_UPDATE_TRCOEFF
					if (ocout.is_open())
					{
						i++;
						ocout << i << endl;//printf("%d\n", i);
					}
#endif

					for (Int transId = 0; transId < 3; transId++)
					{
						//update
						TransformUpdateBGP(transId, uiAbsPartIdx, iH, iW, pcSlice->getSliceQp() - 17
#if PRINT_UPDATEBG_RESI || PRINT_UPDATE_TRCOEFF
							, ocout
#endif
						);
					}
					uiAbsPartIdx++;
				}
#if PRINT_UPDATEBG_RESI || PRINT_UPDATE_TRCOEFF
			ocout.close();
#endif
		}
#endif


#if ENCODE_BGP
		if (pcPic->getPOC() == BGPICPOC)
		{
			//init 
			m_pcSbacCoder->init((TEncBinIf*)m_pcBinCABAC);
			pcSlice->allocSubstreamSizes(iNumSubstreams);
			for (UInt ui = 0; ui < iNumSubstreams; ui++)
			{
				pcSubstreamsOut[ui].clear();
			}
			m_pcEntropyCoder->setEntropyCoder(m_pcSbacCoder, pcSlice);
			m_pcEntropyCoder->resetEntropy();
			m_pcEntropyCoder->setBitstream(pcSubstreamsOut);

			// Encode Coefficients
			UInt uiHeight = pcPic->getPicYuvOrg()->getHeight();
			UInt uiWidth = pcPic->getPicYuvOrg()->getWidth();

			UInt uiAbsPartIdx = 0;
			for (Int iH = 0; iH < uiHeight; iH += TRANS_UNIT_LEN)
				for (Int iW = 0; iW < uiWidth; iW += TRANS_UNIT_LEN)
				{

					for (Int transId = 0; transId < 3; transId++)
					{
						TextType    TType;
						if (transId == 0)
						{
							TType = TEXT_LUMA;
							m_pcEntropyCoder->encodeBGCoeff(m_pcCoeffY, TRANS_UNIT_LEN, TRANS_UNIT_LEN, TType, uiAbsPartIdx);
						}
						else if (transId == 1)
						{
							TType = TEXT_CHROMA_U;
							m_pcEntropyCoder->encodeBGCoeff(m_pcCoeffCb, TRANS_UNIT_LEN >> 1, TRANS_UNIT_LEN >> 1, TType, uiAbsPartIdx);
						}
						else
						{
							TType = TEXT_CHROMA_V;
							m_pcEntropyCoder->encodeBGCoeff(m_pcCoeffCr, TRANS_UNIT_LEN >> 1, TRANS_UNIT_LEN >> 1, TType, uiAbsPartIdx);
						}
					}
					uiAbsPartIdx++;
				}


			m_pcEntropyCoder->encodeTerminatingBit(1);
			m_pcEntropyCoder->encodeSliceFinish();
			pcSubstreamsOut->writeByteAlignment();
		}
#endif

#if UPDATE_BGP
		if (pcPic->getPOC() == 250/* % 300 == 0 && pcPic->getPOC() > 0*/)
		{
			//init 
			m_pcSbacCoder->init((TEncBinIf*)m_pcBinCABAC);
			pcSlice->allocSubstreamSizes(iNumSubstreams);
			for (UInt ui = 0; ui < iNumSubstreams; ui++)
			{
				pcSubstreamsOut[ui].clear();
			}
			m_pcEntropyCoder->setEntropyCoder(m_pcSbacCoder, pcSlice);
			m_pcEntropyCoder->resetEntropy();
			m_pcEntropyCoder->setBitstream(pcSubstreamsOut);

			// Encode Coefficients
			UInt uiHeight = pcPic->getPicYuvOrg()->getHeight();
			UInt uiWidth = pcPic->getPicYuvOrg()->getWidth();

			UInt uiAbsPartIdx = 0;
			for (Int iH = 0; iH < uiHeight; iH += TRANS_UNIT_LEN)
				for (Int iW = 0; iW < uiWidth; iW += TRANS_UNIT_LEN)
				{

					for (Int transId = 0; transId < 3; transId++)
					{
						TextType    TType;
						if (transId == 0)
						{
							TType = TEXT_LUMA;
							m_pcEntropyCoder->encodeBGCoeff(m_pcCoeffY, TRANS_UNIT_LEN, TRANS_UNIT_LEN, TType, uiAbsPartIdx);
						}
						else if (transId == 1)
						{
							TType = TEXT_CHROMA_U;
							m_pcEntropyCoder->encodeBGCoeff(m_pcCoeffCb, TRANS_UNIT_LEN >> 1, TRANS_UNIT_LEN >> 1, TType, uiAbsPartIdx);
						}
						else
						{
							TType = TEXT_CHROMA_V;
							m_pcEntropyCoder->encodeBGCoeff(m_pcCoeffCr, TRANS_UNIT_LEN >> 1, TRANS_UNIT_LEN >> 1, TType, uiAbsPartIdx);
						}
					}
					uiAbsPartIdx++;
				}


			m_pcEntropyCoder->encodeTerminatingBit(1);
			m_pcEntropyCoder->encodeSliceFinish();
			pcSubstreamsOut->writeByteAlignment();
		}
#endif

#if BBB
#if HIERARCHY_GENETATE_BGP //Rec
		if (pcPic->getPOC() != 0)
		{
#if israndom
			if (pcPic->getPOC() % 32 != 0)
			{
#endif // israndom

				const Int  iWidth = pcPic->getRecoBuf().Y().width;
				const Int  iHeight = pcPic->getRecoBuf().Y().height;
#if PRINT_DIFF
				ofstream os("test.txt", ofstream::app);
#endif
				for (Int i = 0; i < iHeight; i += UNIT_LEN)
				{
					for (Int j = 0; j < iWidth; j += UNIT_LEN)
					{
						Int level = 0;
						Bool divflag = false;

						CompDiff(j, i, pcPic, level, divflag
#if PRINT_DIFF
							, os
#endif
						);

					}
#if PRINT_DIFF
					os << endl;
#endif
				}
#if PRINT_DIFF
				os.close();
#endif
#if israndom
			}
#endif // israndom

		}
		else
		{
			Pel* piBac;
			Pel* piOrg;

			UInt uiStride;
			UInt uiPicHeight;
			UInt uiPicWidth;

			for (Int compId = 0; compId < 3; compId++)
			{
				switch (compId)
				{
				case 0:
					piBac = m_bgNewPicYuvRecGop->getRecoBuf().Y().bufAt(0, 0);
					piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);

					uiStride = pcPic->getRecoBuf().Y().stride;
					uiPicHeight = pcPic->getRecoBuf().Y().height;
					uiPicWidth = pcPic->getRecoBuf().Y().width;
					break;
				case 1:
					piBac = m_bgNewPicYuvRecGop->getRecoBuf().Cb().bufAt(0, 0);
					piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);

					uiStride = pcPic->getRecoBuf().Cb().stride;
					uiPicHeight = pcPic->getRecoBuf().Cb().height;
					uiPicWidth = pcPic->getRecoBuf().Cb().width;
					break;
				case 2:
					piBac = m_bgNewPicYuvRecGop->getRecoBuf().Cr().bufAt(0, 0);
					piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);

					uiStride = pcPic->getRecoBuf().Cr().stride;
					uiPicHeight = pcPic->getRecoBuf().Cr().height;
					uiPicWidth = pcPic->getRecoBuf().Cr().width;
					break;
				default:
					break;
				}
				for (Int unit_i = 0; unit_i < uiPicHeight; unit_i++)
				{
					for (Int unit_j = 0; unit_j < uiPicWidth; unit_j++)
					{
						piBac[unit_j] = 0;
					}
					piOrg += uiStride;
					piBac += uiStride;
				}
			}
		}
#endif

/*#if BLOCK_GEN //生成块

		if (pcPic->getPOC() > 5 && pcPic->getPOC() < 300)
		{
			int num_block = 0;
			
			for (UInt uiH = 0; uiH < pcPic->getRecoBuf().Y().height; uiH += BLOCK_GEN_LEN)
			{
				for (UInt uiW = 0; uiW < pcPic->getRecoBuf().Y().width; uiW += BLOCK_GEN_LEN)
				{

					if (BgBlock[num_block] != 0)
					{
						BgBlock[num_block]++;
					}
					else
					{
						double diff = 0;

						//pcPic->CompBlockDiff(uiW, uiH, pcPic, diff);  //get  block diff
						pcPic->CompBlockPicRecoDiff(uiW, uiH, pcPic, m_bgNewPicYuvRecGop, diff);//计算Rec与PicRec的diff

						cout << "diff" << diff;
						if (diff < 1)
						{
							BgBlock[num_block]++;
							isencode = 1; //确定编
							pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, uiW, uiH, m_bgNewPicYuvOrgGop);  //用Rec判断 Org有可能为空。
							//pcPic->CopyReco2Block(m_bgNewBlocksRecGop, uiW, uiH, m_bgNewPicYuvRecGop);
							//pcPic->CopyOrg2Block(m_bgNewBlocksOrgGop, uiW, uiH, pcPic);  //block 编码
						}
					}


					num_block++;  //UNITxUNIT
				}
			}
			cout << "num_block" << num_block << endl;

			{ // cout
				Int i = 0;
				Int j = 0;
				//ofstream bgBlock;
				//bgBlock.open("D://2//BgBlock32_5_8补50.txt", ios::app);
				//bgBlock << pcPic->getPOC()<<" ";
				cout << endl;
				while (i < (pcPic->getOrigBuf().Y().width / BLOCK_GEN_LEN)*(pcPic->getOrigBuf().Y().height / BLOCK_GEN_LEN))
				{
					cout << BgBlock[i] << " ";
					//bgBlock << BgBlock[i] <<" ";
					i++;
					if (BgBlock[i] != 0)
						j++;

				}
				//bgBlock << endl;
				//bgBlock <<" j "<< j << endl;
				cout << endl;
				cout << " j " << j << endl;
			}
		}

#endif*/

#if BG_BLOCK_SUBSTITUTION
		if (pcPic->getPOC() != 0)
		{
#if israndom
			if (pcPic->getPOC() % 32 != 0)
			{

#endif // israndom

				const UInt  uiWidth = pcPic->getRecoBuf().Y().width;
				const UInt  uiHeight = pcPic->getRecoBuf().Y().height;
				for (UInt uiH = 0; uiH < uiHeight; uiH += BLOCK_LEN)
				{
					for (UInt uiW = 0; uiW < uiWidth; uiW += BLOCK_LEN)
					{
						Double Fc[3];
						for (Int transId = 0; transId < 3; transId++)
						{
							Fc[transId] = CompNabla(transId, uiH, uiW, pcPic);
						}
						Double dNabla = Fc[0] + Fc[1] / 4 + Fc[2] / 4;

						if (dNabla < 1.5)   // if Nabla < Omegaba then substitute
						{
							pcPic->Copy2BackPic(m_bgNewPicYuvRecGop, uiW, uiH, pcPic);
						}
						else if (dNabla >= 1.5 && dNabla < 3.5 && uiH != 0 && uiH != (uiHeight - BLOCK_LEN) && uiW != 0 && uiW != (uiWidth - BLOCK_LEN))  // Omegaba < Nabla < Omega
						{
							Double DiffOrg = 0;
							Double DiffBG = 0;

							DiffOrg += CompBlockDiff(uiH, uiW, pcPic, pcPic); //maybe
							DiffBG += CompBlockDiff(uiH, uiW, m_bgNewPicYuvRecGop, pcPic);

							if (DiffOrg < DiffBG)
							{
								pcPic->Copy2BackPic(m_bgNewPicYuvRecGop, uiW, uiH, pcPic);
							}
						}
					}
				}
#if israndom
			}
#endif // israndom

		}
		/*else
		{
			Pel* piBac;
			Pel* piOrg;

			UInt uiStride;
			UInt uiPicHeight;
			UInt uiPicWidth;

			for (Int compId = 0; compId < 3; compId++)
			{
				switch (compId)
				{
				case 0:
					piBac = m_bgNewPicYuvRecGop->getRecoBuf().Y().bufAt(0, 0);
					piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);

					uiStride = pcPic->getRecoBuf().Y().stride;
					uiPicHeight = pcPic->getRecoBuf().Y().height;
					uiPicWidth = pcPic->getRecoBuf().Y().width;
					break;
				case 1:
					piBac = m_bgNewPicYuvRecGop->getRecoBuf().Cb().bufAt(0, 0);
					piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);

					uiStride = pcPic->getRecoBuf().Cb().stride;
					uiPicHeight = pcPic->getRecoBuf().Cb().height;
					uiPicWidth = pcPic->getRecoBuf().Cb().width;
					break;
				case 2:
					piBac = m_bgNewPicYuvRecGop->getRecoBuf().Cr().bufAt(0, 0);
					piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);

					uiStride = pcPic->getRecoBuf().Cr().stride;
					uiPicHeight = pcPic->getRecoBuf().Cr().height;
					uiPicWidth = pcPic->getRecoBuf().Cr().width;
					break;
				default:
					break;
				}
				for (Int unit_i = 0; unit_i < uiPicHeight; unit_i++)
				{
					for (Int unit_j = 0; unit_j < uiPicWidth; unit_j++)
					{
						piBac[unit_j] = piOrg[unit_j];
					}
					piOrg += uiStride;
					piBac += uiStride;
				}
			}
		}*/
#endif

		/*if (pcPic->getPOC() == 6)
		{
			{
				ofstream inencode;
				inencode.open("D://2//6Reco2.txt");
				for (Int compId = 0; compId < 3; compId++)
				{
					Pel* piOrg;
					UInt uiStride;
					UInt uiPicHeight;
					UInt uiPicWidth;
					switch (compId)
					{
					case 0:
						piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);

						uiStride = pcPic->getRecoBuf().Y().stride;
						uiPicHeight = pcPic->getRecoBuf().Y().height;
						uiPicWidth = pcPic->getRecoBuf().Y().width;
						break;
					case 1:
						piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);

						uiStride = pcPic->getRecoBuf().Cb().stride;
						uiPicHeight = pcPic->getRecoBuf().Cb().height;
						uiPicWidth = pcPic->getRecoBuf().Cb().width;
						break;
					case 2:
						piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);

						uiStride = pcPic->getRecoBuf().Cr().stride;
						uiPicHeight = pcPic->getRecoBuf().Cr().height;
						uiPicWidth = pcPic->getRecoBuf().Cr().width;
						break;
					default:
						break;
					}
					for (Int i = 0; i < uiPicHeight; i++)
					{
						for (Int j = 0; j < uiPicWidth; j++)
						{
							inencode << piOrg[i * uiStride + j] << " ";
						}
					}
				}
			}


			{
				ofstream inencode;
				inencode.open("D://2//6blockReco1.txt");

				int num_block = 0;
	
				for (UInt uiH = 0; uiH < pcPic->getRecoBuf().Y().height; uiH += BLOCK_GEN_LEN)
				{
					for (UInt uiW = 0; uiW < pcPic->getRecoBuf().Y().width; uiW += BLOCK_GEN_LEN)
					{
						if (BgBlock[num_block] == 0)
						{
							for (Int compId = 0; compId < 3; compId++)
							{

								Pel* piOrg;
								UInt uiStride;
								UInt uiHeight;
								UInt uiWidth;
								UInt unit_len;
								UInt H, W;
								switch (compId)
								{
								case 0:
									piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
									uiStride = pcPic->getRecoBuf().Y().stride;
									uiHeight = pcPic->getRecoBuf().Y().height;
									uiWidth = pcPic->getRecoBuf().Y().width;
									unit_len = BLOCK_GEN_LEN;
									H = uiH;
									W = uiW;
									break;
								case 1:
									piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);

									uiStride = pcPic->getRecoBuf().Cb().stride;
									uiHeight = pcPic->getRecoBuf().Cb().height;
									uiWidth = pcPic->getRecoBuf().Cb().width;
									unit_len = BLOCK_GEN_LEN >> 1;
									H = uiH >> 1;
									W = uiW >> 1;
									break;
								case 2:
									piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);

									uiStride = pcPic->getRecoBuf().Cr().stride;
									uiHeight = pcPic->getRecoBuf().Cr().height;
									uiWidth = pcPic->getRecoBuf().Cr().width;
									unit_len = BLOCK_GEN_LEN >> 1;
									H = uiH >> 1;
									W = uiW >> 1;
									break;
								default:
									break;
								}

								for (UInt uiY = 0; uiY < unit_len && (H + uiY < uiHeight); uiY++)
								{
									for (UInt uiX = 0; uiX < unit_len && (W + uiX < uiWidth); uiX++)
									{
										inencode << piOrg[H * uiStride + W + uiX] << " ";
									}
									piOrg += uiStride;
								}
							}
						}
						num_block++;
					}
				}
				cout << "num_block" << num_block << endl;

			}

		}
		*/

#endif
        actualHeadBits += ( m_HLSWriter->getNumberOfWrittenBits() - tmpBitsBeforeWriting );

        pcSlice->setFinalized(true);

        pcSlice->clearSubstreamSizes(  );
        {
          UInt numBinsCoded = 0;
          m_pcSliceEncoder->encodeSlice(pcPic, &(substreamsOut[0]), numBinsCoded);
          binCountsInNalUnits+=numBinsCoded;
        }
        {
          // Construct the final bitstream by concatenating substreams.
          // The final bitstream is either nalu.m_Bitstream or pcBitstreamRedirect;
          // Complete the slice header info.
          m_HLSWriter->setBitstream( &nalu.m_Bitstream );
#if HEVC_TILES_WPP
          m_HLSWriter->codeTilesWPPEntryPoint( pcSlice );
#endif

          // Append substreams...
          OutputBitstream *pcOut = pcBitstreamRedirect;
#if HEVC_TILES_WPP
#if HEVC_DEPENDENT_SLICES

          const Int numZeroSubstreamsAtStartOfSlice = pcPic->tileMap->getSubstreamForCtuAddr(pcSlice->getSliceSegmentCurStartCtuTsAddr(), false, pcSlice);
#else
          const Int numZeroSubstreamsAtStartOfSlice  = pcPic->tileMap->getSubstreamForCtuAddr(pcSlice->getSliceCurStartCtuTsAddr(), false, pcSlice);
#endif
          const Int numSubstreamsToCode  = pcSlice->getNumberOfSubstreamSizes()+1;
#else
          const Int numZeroSubstreamsAtStartOfSlice  = 0;
          const Int numSubstreamsToCode  = pcSlice->getNumberOfSubstreamSizes()+1;
#endif
          for ( UInt ui = 0 ; ui < numSubstreamsToCode; ui++ )
          {
            pcOut->addSubstream(&(substreamsOut[ui+numZeroSubstreamsAtStartOfSlice]));
          }
        }

        // If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple dependent slices) then buffer it.
        // If current NALU is the last NALU of slice and a NALU was buffered, then (a) Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
        Bool bNALUAlignedWrittenToList    = false; // used to ensure current NALU is not written more than once to the NALU list.
        xAttachSliceDataToNalUnit(nalu, pcBitstreamRedirect);
        accessUnit.push_back(new NALUnitEBSP(nalu));
        actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
        numBytesInVclNalUnits += (std::size_t)(accessUnit.back()->m_nalUnitData.str().size());
        bNALUAlignedWrittenToList = true;

        if (!bNALUAlignedWrittenToList)
        {
          nalu.m_Bitstream.writeAlignZero();
          accessUnit.push_back(new NALUnitEBSP(nalu));
        }

        if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
            ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
            ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
           || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) &&
            ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getSubPicCpbParamsPresentFlag() ) )
        {
            UInt numNalus = 0;
          UInt numRBSPBytes = 0;
          for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
          {
            numRBSPBytes += UInt((*it)->m_nalUnitData.str().size());
            numNalus ++;
          }
          duData.push_back(DUData());
          duData.back().accumBitsDU = ( numRBSPBytes << 3 );
          duData.back().accumNalsDU = numNalus;
        }
      } // end iteration over slices


      // cabac_zero_words processing
      cabac_zero_word_padding(pcSlice, pcPic, binCountsInNalUnits, numBytesInVclNalUnits, accessUnit.back()->m_nalUnitData, m_pcCfg->getCabacZeroWordPaddingEnabled());

      //-- For time output for each slice
      auto elapsed = std::chrono::steady_clock::now() - beforeTime;
      auto encTime = std::chrono::duration_cast<std::chrono::seconds>( elapsed ).count();

      std::string digestStr;
      if (m_pcCfg->getDecodedPictureHashSEIType()!=HASHTYPE_NONE)
      {
        SEIDecodedPictureHash *decodedPictureHashSei = new SEIDecodedPictureHash();
        PelUnitBuf recoBuf = pcPic->cs->getRecoBuf();
        m_seiEncoder.initDecodedPictureHashSEI(decodedPictureHashSei, recoBuf, digestStr, pcSlice->getSPS()->getBitDepths());
        trailingSeiMessages.push_back(decodedPictureHashSei);
      }

      m_pcCfg->setEncodedFlag(iGOPid, true);

      Double PSNR_Y;
      xCalculateAddPSNRs( isField, isTff, iGOPid, pcPic, accessUnit, rcListPic, encTime, snr_conversion, printFrameMSE, &PSNR_Y );

      // Only produce the Green Metadata SEI message with the last picture.
      if( m_pcCfg->getSEIGreenMetadataInfoSEIEnable() && pcSlice->getPOC() == ( m_pcCfg->getFramesToBeEncoded() - 1 )  )
      {
        SEIGreenMetadataInfo *seiGreenMetadataInfo = new SEIGreenMetadataInfo;
        m_seiEncoder.initSEIGreenMetadataInfo(seiGreenMetadataInfo, (UInt)(PSNR_Y * 100 + 0.5));
        trailingSeiMessages.push_back(seiGreenMetadataInfo);
      }

      xWriteTrailingSEIMessages(trailingSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS());

      printHash(m_pcCfg->getDecodedPictureHashSEIType(), digestStr);

      if ( m_pcCfg->getUseRateCtrl() )
      {
        Double avgQP     = m_pcRateCtrl->getRCPic()->calAverageQP();
        Double avgLambda = m_pcRateCtrl->getRCPic()->calAverageLambda();
        if ( avgLambda < 0.0 )
        {
          avgLambda = lambda;
        }

        m_pcRateCtrl->getRCPic()->updateAfterPicture( actualHeadBits, actualTotalBits, avgQP, avgLambda, pcSlice->getSliceType());
        m_pcRateCtrl->getRCPic()->addToPictureLsit( m_pcRateCtrl->getPicList() );

        m_pcRateCtrl->getRCSeq()->updateAfterPic( actualTotalBits );
        if ( pcSlice->getSliceType() != I_SLICE )
        {
          m_pcRateCtrl->getRCGOP()->updateAfterPicture( actualTotalBits );
        }
        else    // for intra picture, the estimated bits are used to update the current status in the GOP
        {
          m_pcRateCtrl->getRCGOP()->updateAfterPicture( estimatedBits );
        }
  #if U0132_TARGET_BITS_SATURATION
        if (m_pcRateCtrl->getCpbSaturationEnabled())
        {
          m_pcRateCtrl->updateCpbState(actualTotalBits);
          msg( NOTICE, " [CPB %6d bits]", m_pcRateCtrl->getCpbState() );
        }
  #endif
      }

      xCreatePictureTimingSEI( m_pcCfg->getEfficientFieldIRAPEnabled() ? effFieldIRAPMap.GetIRAPGOPid() : 0, leadingSeiMessages, nestedSeiMessages, duInfoSeiMessages, pcSlice, isField, duData );
      if( m_pcCfg->getScalableNestingSEIEnabled() )
      {
        xCreateScalableNestingSEI( leadingSeiMessages, nestedSeiMessages );
      }
      xWriteLeadingSEIMessages( leadingSeiMessages, duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS(), duData );
      xWriteDuSEIMessages( duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS(), duData );

      m_AUWriterIf->outputAU( accessUnit );

      msg( NOTICE, "\n" );
      fflush( stdout );
    }
	 
    DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "final", 0 ) ) );
	

    pcPic->reconstructed = true;
    m_bFirst = false;
    m_iNumPicCoded++;
    m_totalCoded ++;
    /* logging: insert a newline at end of picture period */

    if (m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      iGOPid=effFieldIRAPMap.restoreGOPid(iGOPid);
    }

    pcPic->destroyTempBuffers();
    pcPic->cs->destroyCoeffs();
    pcPic->cs->releaseIntermediateData();

#if TRANSFORM_BGP
	delete[] m_pcCoeffY;
	delete[] m_pcCoeffCb;
	delete[] m_pcCoeffCr;
#endif

#if OrgBG_BLOCK_SUBSTITUTION
	delete[] m_pcNablaY;
	delete[] m_pcNablaCb;
	delete[] m_pcNablaCr;
#endif
  } // iGOPid-loop
   
  delete pcBitstreamRedirect;

  CHECK(!( (m_iNumPicCoded == iNumPicRcvd) ), "Unspecified error");

}

Void EncGOP::printOutSummary(UInt uiNumAllPicCoded, Bool isField, const Bool printMSEBasedSNR, const Bool printSequenceMSE, const BitDepths &bitDepths)
{
#if ENABLE_QPA
  const bool    useWPSNR = m_pcEncLib->getUseWPSNR();
#endif
#if WCG_WPSNR
  const bool    useLumaWPSNR = m_pcEncLib->getLumaLevelToDeltaQPMapping().isEnabled();
#endif

  if( m_pcCfg->getDecodeBitstream(0).empty() && m_pcCfg->getDecodeBitstream(1).empty() && !m_pcCfg->useFastForwardToPOC() )
  {
    CHECK( !( uiNumAllPicCoded == m_gcAnalyzeAll.getNumPic() ), "Unspecified error" );
  }

  //--CFG_KDY
  const Int rateMultiplier=(isField?2:1);
  m_gcAnalyzeAll.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier / (Double)m_pcCfg->getTemporalSubsampleRatio());
  m_gcAnalyzeI.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier / (Double)m_pcCfg->getTemporalSubsampleRatio());
  m_gcAnalyzeP.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier / (Double)m_pcCfg->getTemporalSubsampleRatio());
  m_gcAnalyzeB.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier / (Double)m_pcCfg->getTemporalSubsampleRatio());
#if WCG_WPSNR
  if (useLumaWPSNR)
  {
    m_gcAnalyzeWPSNR.setFrmRate(m_pcCfg->getFrameRate()*rateMultiplier / (Double)m_pcCfg->getTemporalSubsampleRatio());
  }
#endif
  
  const ChromaFormat chFmt = m_pcCfg->getChromaFormatIdc();

  //-- all
  msg( INFO, "\n" );
  msg( DETAILS,"\nSUMMARY --------------------------------------------------------\n" );
#if ENABLE_QPA
  m_gcAnalyzeAll.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, bitDepths, useWPSNR);
#else
  m_gcAnalyzeAll.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, bitDepths);
#endif
  msg( DETAILS,"\n\nI Slices--------------------------------------------------------\n" );
  m_gcAnalyzeI.printOut('i', chFmt, printMSEBasedSNR, printSequenceMSE, bitDepths);

  msg( DETAILS,"\n\nP Slices--------------------------------------------------------\n" );
  m_gcAnalyzeP.printOut('p', chFmt, printMSEBasedSNR, printSequenceMSE, bitDepths);

  msg( DETAILS,"\n\nB Slices--------------------------------------------------------\n" );
  m_gcAnalyzeB.printOut('b', chFmt, printMSEBasedSNR, printSequenceMSE, bitDepths);
#if WCG_WPSNR
  if (useLumaWPSNR)
  {
    msg(DETAILS, "\nWPSNR SUMMARY --------------------------------------------------------\n");
    m_gcAnalyzeWPSNR.printOut('w', chFmt, printMSEBasedSNR, printSequenceMSE, bitDepths, useLumaWPSNR);
  }
#endif
  if (!m_pcCfg->getSummaryOutFilename().empty())
  {
    m_gcAnalyzeAll.printSummary(chFmt, printSequenceMSE, bitDepths, m_pcCfg->getSummaryOutFilename());
  }

  if (!m_pcCfg->getSummaryPicFilenameBase().empty())
  {
    m_gcAnalyzeI.printSummary(chFmt, printSequenceMSE, bitDepths, m_pcCfg->getSummaryPicFilenameBase()+"I.txt");
    m_gcAnalyzeP.printSummary(chFmt, printSequenceMSE, bitDepths, m_pcCfg->getSummaryPicFilenameBase()+"P.txt");
    m_gcAnalyzeB.printSummary(chFmt, printSequenceMSE, bitDepths, m_pcCfg->getSummaryPicFilenameBase()+"B.txt");
  }

#if WCG_WPSNR
  if (!m_pcCfg->getSummaryOutFilename().empty() && useLumaWPSNR)
  {
    m_gcAnalyzeWPSNR.printSummary(chFmt, printSequenceMSE, bitDepths, m_pcCfg->getSummaryOutFilename());
  }
#endif
  if(isField)
  {
    //-- interlaced summary
    m_gcAnalyzeAll_in.setFrmRate( m_pcCfg->getFrameRate() / (Double)m_pcCfg->getTemporalSubsampleRatio());
    m_gcAnalyzeAll_in.setBits(m_gcAnalyzeAll.getBits());
    // prior to the above statement, the interlace analyser does not contain the correct total number of bits.

    msg( DETAILS,"\n\nSUMMARY INTERLACED ---------------------------------------------\n" );
#if ENABLE_QPA
    m_gcAnalyzeAll_in.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, bitDepths, useWPSNR);
#else
    m_gcAnalyzeAll_in.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, bitDepths);
#endif
    if (!m_pcCfg->getSummaryOutFilename().empty())
    {
      m_gcAnalyzeAll_in.printSummary(chFmt, printSequenceMSE, bitDepths, m_pcCfg->getSummaryOutFilename());
#if WCG_WPSNR
      if (useLumaWPSNR)
      {
        m_gcAnalyzeWPSNR.printSummary(chFmt, printSequenceMSE, bitDepths, m_pcCfg->getSummaryOutFilename());
      }
#endif
    }
  }

  msg( DETAILS,"\nRVM: %.3lf\n", xCalculateRVM() );
}

#if W0038_DB_OPT
UInt64 EncGOP::preLoopFilterPicAndCalcDist( Picture* pcPic )
{
  CodingStructure& cs = *pcPic->cs;
  m_pcLoopFilter->loopFilterPic( cs );

  const CPelUnitBuf picOrg = pcPic->getRecoBuf();
  const CPelUnitBuf picRec = cs.getRecoBuf();

  UInt64 uiDist = 0;
  for( UInt comp = 0; comp < (UInt)picRec.bufs.size(); comp++)
  {
    const ComponentID compID = ComponentID(comp);
    const UInt rshift  = 2 * DISTORTION_PRECISION_ADJUSTMENT(cs.sps->getBitDepth(toChannelType(compID)) - 8);
#if ENABLE_QPA
    CHECK( rshift >= 8, "shifts greater than 7 are not supported." );
#endif
    uiDist += xFindDistortionPlane( picOrg.get(compID), picRec.get(compID), rshift );
  }
  return uiDist;
}
#endif

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================


Void EncGOP::xInitGOP( Int iPOCLast, Int iNumPicRcvd, Bool isField )
{
  CHECK(!( iNumPicRcvd > 0 ), "Unspecified error");
  //  Exception for the first frames
  if ( ( isField && (iPOCLast == 0 || iPOCLast == 1) ) || (!isField  && (iPOCLast == 0))  )
  {
    m_iGopSize    = 1;
  }
  else
  {
    m_iGopSize    = m_pcCfg->getGOPSize();
  }
  CHECK(!(m_iGopSize > 0), "Unspecified error");

  return;
}


Void EncGOP::xGetBuffer( PicList&                  rcListPic,
                         std::list<PelUnitBuf*>&   rcListPicYuvRecOut,
                         Int                       iNumPicRcvd,
                         Int                       iTimeOffset,
                         Picture*&                 rpcPic,
                         Int                       pocCurr,
                         Bool                      isField )
{
  Int i;
  //  Rec. output
  std::list<PelUnitBuf*>::iterator     iterPicYuvRec = rcListPicYuvRecOut.end();

  if (isField && pocCurr > 1 && m_iGopSize!=1)
  {
    iTimeOffset--;
  }

  for ( i = 0; i < (iNumPicRcvd - iTimeOffset + 1); i++ )
  {
    iterPicYuvRec--;
  }

  //  Current pic.
  PicList::iterator        iterPic       = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    rpcPic = *(iterPic);
    if (rpcPic->getPOC() == pocCurr)
    {
      break;
    }
    iterPic++;
  }

  CHECK(!(rpcPic != NULL), "Unspecified error");
  CHECK(!(rpcPic->getPOC() == pocCurr), "Unspecified error");

  (**iterPicYuvRec) = rpcPic->getRecoBuf();
  return;
}

#if ENABLE_QPA

#ifndef BETA
 #define BETA (2.0 / 3.0)  // value between 0 and 1; use 0.0 for traditional PSNR
#endif
#define GLOBAL_AVERAGING 1 // "global" averaging of a_k across a set instead of one picture
#if FRAME_WEIGHTING
static const UInt DQP[16] = { 4, 12, 11, 12,  9, 12, 11, 12,  6, 12, 11, 12,  9, 12, 11, 12 };
#endif

static inline double calcWeightedSquaredError(const CPelBuf& org,    const CPelBuf& rec,     double &sumAct,
                                              const UInt imageWidth, const UInt imageHeight, const UInt offsetX,  const UInt offsetY, int blockWidth, int blockHeight)
{
  const int    O = org.stride;
  const int    R = rec.stride;
  const Pel   *o = org.bufAt(offsetX, offsetY);
  const Pel   *r = rec.bufAt(offsetX, offsetY);
  const int yAct = offsetY > 0 ? 0 : 1;
  const int xAct = offsetX > 0 ? 0 : 1;

  if (offsetY + (UInt)blockHeight > imageHeight) blockHeight = imageHeight - offsetY;
  if (offsetX + (UInt)blockWidth  > imageWidth ) blockWidth  = imageWidth  - offsetX;

  const int hAct = offsetY + (UInt)blockHeight < imageHeight ? blockHeight : blockHeight - 1;
  const int wAct = offsetX + (UInt)blockWidth  < imageWidth  ? blockWidth  : blockWidth  - 1;
  UInt64 ssErr   = 0; // sum of squared diffs
  UInt64 saAct   = 0; // sum of abs. activity
  double msAct;
  int x, y;

  // calculate image differences and activity
  for (y = 0; y < blockHeight; y++)  // error
  {
    for (x = 0; x < blockWidth; x++)
    {
      register  Int64 iDiff = (Int64)o[y*O + x] - (Int64)r[y*R + x];
      ssErr += UInt64(iDiff * iDiff);
    }
  }
  if (wAct <= xAct || hAct <= yAct) return (double)ssErr;

  for (y = yAct; y < hAct; y++)   // activity
  {
    for (x = xAct; x < wAct; x++)
    {
      saAct += UInt64(abs(4 * (Int64)o[y*O + x] - (Int64)o[y*O + x-1] - (Int64)o[y*O + x+1] - (Int64)o[(y-1)*O + x] - (Int64)o[(y+1)*O + x]));
    }
  }

  // calculate weight (mean squared activity)
  msAct = (double)saAct / (double(wAct - xAct) * double(hAct - yAct));
  if (msAct < 8.0) msAct = 8.0;
  msAct *= msAct; // because ssErr is squared

  sumAct += msAct; // includes high-pass gain

  // calculate activity weighted error square
  return (double)ssErr * pow(msAct, -1.0 * BETA);
}
#endif // ENABLE_QPA

UInt64 EncGOP::xFindDistortionPlane(const CPelBuf& pic0, const CPelBuf& pic1, const UInt rshift
#if ENABLE_QPA
                                  , const UInt chromaShift /*= 0*/
#endif
                                   )
{
  UInt64 uiTotalDiff;
  const  Pel*  pSrc0 = pic0.bufAt(0, 0);
  const  Pel*  pSrc1 = pic1.bufAt(0, 0);

  CHECK(pic0.width  != pic1.width , "Unspecified error");
  CHECK(pic0.height != pic1.height, "Unspecified error");

  if( rshift > 0 )
  {
#if ENABLE_QPA
    const   UInt  BD = rshift;      // image bit-depth
    if (BD >= 8)
    {
      const UInt   W = pic0.width;  // image width
      const UInt   H = pic0.height; // image height
      const double R = double(W * H) / (1920.0 * 1080.0);
      const UInt   B = Clip3<UInt>(0, 128 >> chromaShift, 4 * UInt(16.0 * sqrt(R) + 0.5)); // WPSNR block size in integer multiple of 4 (for SIMD, = 64 at full-HD)

      UInt x, y;

      if (B < 4) // image is too small to use WPSNR, resort to traditional PSNR
      {
        uiTotalDiff = 0;
        for (y = 0; y < H; y++)
        {
          for (x = 0; x < W; x++)
          {
            register Int64 iDiff = (Int64)pSrc0[x] - (Int64)pSrc1[x];
            uiTotalDiff += UInt64(iDiff * iDiff);
          }
          pSrc0 += pic0.stride;
          pSrc1 += pic1.stride;
        }
        return uiTotalDiff;
      }

      double wmse = 0.0, sumAct = 0.0; // compute activity normalized SNR value
#if !GLOBAL_AVERAGING
      double numAct = 0.0;
#endif
      for (y = 0; y < H; y += B)
      {
        for (x = 0; x < W; x += B)
        {
          wmse += calcWeightedSquaredError(pic1, pic0, sumAct, W, H, x, y, B, B);
#if !GLOBAL_AVERAGING
          numAct += 1.0;
#endif
        }
      }

      // integer weighted distortion
#if GLOBAL_AVERAGING
      sumAct = 1.5 * double(1 << BD);
      if ((W << chromaShift) > 2048 && (H << chromaShift) > 1280)   // UHD luma
      {
        sumAct /= 1.5;
      }
      return (wmse <= 0.0) ? 0 : UInt64(wmse * pow(sumAct, BETA) + 0.5);
#else
      return (wmse <= 0.0 || numAct <= 0.0) ? 0 : UInt64(wmse * pow(sumAct / numAct, BETA) + 0.5);
#endif
    }
#endif // ENABLE_QPA
    uiTotalDiff = 0;
    for (Int y = 0; y < pic0.height; y++)
    {
      for (Int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        uiTotalDiff += UInt64((iTemp * iTemp) >> rshift);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
    }
  }
  else
  {
    uiTotalDiff = 0;
    for (Int y = 0; y < pic0.height; y++)
    {
      for (Int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        uiTotalDiff += UInt64(iTemp * iTemp);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
    }
  }

  return uiTotalDiff;
}
#if WCG_WPSNR
Double EncGOP::xFindDistortionPlaneWPSNR(const CPelBuf& pic0, const CPelBuf& pic1, const UInt rshift, const CPelBuf& picLuma0, 
  ComponentID compID, const ChromaFormat chfmt    )
{
  const bool    useLumaWPSNR = m_pcEncLib->getLumaLevelToDeltaQPMapping().isEnabled();
  if (!useLumaWPSNR)
  {
    return 0;
  }

  Double uiTotalDiffWPSNR;
  const  Pel*  pSrc0 = pic0.bufAt(0, 0);
  const  Pel*  pSrc1 = pic1.bufAt(0, 0);
  const  Pel*  pSrcLuma = picLuma0.bufAt(0, 0);
  CHECK(pic0.width  != pic1.width , "Unspecified error");
  CHECK(pic0.height != pic1.height, "Unspecified error");

  if( rshift > 0 )
  {
    uiTotalDiffWPSNR = 0;
    for (Int y = 0; y < pic0.height; y++)
    {
      for (Int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        Double dW = m_pcEncLib->getRdCost()->getWPSNRLumaLevelWeight(pSrcLuma[(x << getComponentScaleX(compID, chfmt))]);
        uiTotalDiffWPSNR += ((dW * (Double)iTemp * (Double)iTemp)) * (Double)(1 >> rshift);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
      pSrcLuma += picLuma0.stride << getComponentScaleY(compID, chfmt);
    }
  }
  else
  {
    uiTotalDiffWPSNR = 0;
    for (Int y = 0; y < pic0.height; y++)
    {
      for (Int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        Double dW = m_pcEncLib->getRdCost()->getWPSNRLumaLevelWeight(pSrcLuma[x << getComponentScaleX(compID, chfmt)]);
        uiTotalDiffWPSNR += dW * (Double)iTemp * (Double)iTemp;
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
      pSrcLuma += picLuma0.stride << getComponentScaleY(compID, chfmt);
    }
  }

  return uiTotalDiffWPSNR;
}
#endif

Void EncGOP::xCalculateAddPSNRs( const Bool isField, const Bool isFieldTopFieldFirst, const Int iGOPid, Picture* pcPic, const AccessUnit&accessUnit, PicList &rcListPic, const int64_t dEncTime, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE, Double* PSNR_Y )
{
  xCalculateAddPSNR( pcPic, pcPic->getRecoBuf(), accessUnit, (double) dEncTime, snr_conversion, printFrameMSE, PSNR_Y );

  //In case of field coding, compute the interlaced PSNR for both fields
  if(isField)
  {
    Bool bothFieldsAreEncoded = false;
    Int correspondingFieldPOC = pcPic->getPOC();
    Int currentPicGOPPoc = m_pcCfg->getGOPEntry(iGOPid).m_POC;
    if(pcPic->getPOC() == 0)
    {
      // particular case for POC 0 and 1.
      // If they are not encoded first and separately from other pictures, we need to change this
      // POC 0 is always encoded first then POC 1 is encoded
      bothFieldsAreEncoded = false;
    }
    else if(pcPic->getPOC() == 1)
    {
      // if we are at POC 1, POC 0 has been encoded for sure
      correspondingFieldPOC = 0;
      bothFieldsAreEncoded = true;
    }
    else
    {
      if(pcPic->getPOC()%2 == 1)
      {
        correspondingFieldPOC -= 1; // all odd POC are associated with the preceding even POC (e.g poc 1 is associated to poc 0)
        currentPicGOPPoc      -= 1;
      }
      else
      {
        correspondingFieldPOC += 1; // all even POC are associated with the following odd POC (e.g poc 0 is associated to poc 1)
        currentPicGOPPoc      += 1;
      }
      for(Int i = 0; i < m_iGopSize; i ++)
      {
        if(m_pcCfg->getGOPEntry(i).m_POC == currentPicGOPPoc)
        {
          bothFieldsAreEncoded = m_pcCfg->getGOPEntry(i).m_isEncoded;
          break;
        }
      }
    }

    if(bothFieldsAreEncoded)
    {
      //get complementary top field
      PicList::iterator   iterPic = rcListPic.begin();
      while ((*iterPic)->getPOC() != correspondingFieldPOC)
      {
        iterPic ++;
      }
      Picture* correspondingFieldPic = *(iterPic);

      if( (pcPic->topField && isFieldTopFieldFirst) || (!pcPic->topField && !isFieldTopFieldFirst))
      {
        xCalculateInterlacedAddPSNR(pcPic, correspondingFieldPic, pcPic->getRecoBuf(), correspondingFieldPic->getRecoBuf(), snr_conversion, printFrameMSE, PSNR_Y );
      }
      else
      {
        xCalculateInterlacedAddPSNR(correspondingFieldPic, pcPic, correspondingFieldPic->getRecoBuf(), pcPic->getRecoBuf(), snr_conversion, printFrameMSE, PSNR_Y );
      }
    }
  }
}

Void EncGOP::xCalculateAddPSNR( Picture* pcPic, PelUnitBuf cPicD, const AccessUnit& accessUnit, Double dEncTime, const InputColourSpaceConversion conversion, const Bool printFrameMSE, Double* PSNR_Y )
{
  const SPS&         sps = *pcPic->cs->sps;
  const CPelUnitBuf& pic = cPicD;
  CHECK(!(conversion == IPCOLOURSPACE_UNCHANGED), "Unspecified error");
//  const CPelUnitBuf& org = (conversion != IPCOLOURSPACE_UNCHANGED) ? pcPic->getPicYuvTrueOrg()->getBuf() : pcPic->getPicYuvOrg()->getBuf();
  const CPelUnitBuf& org = pcPic->getOrigBuf();
#if ENABLE_QPA
  const bool    useWPSNR = m_pcEncLib->getUseWPSNR();
#endif
  Double  dPSNR[MAX_NUM_COMPONENT];
#if WCG_WPSNR
  const bool    useLumaWPSNR = m_pcEncLib->getLumaLevelToDeltaQPMapping().isEnabled();
  Double  dPSNRWeighted[MAX_NUM_COMPONENT];
  Double  MSEyuvframeWeighted[MAX_NUM_COMPONENT];
#endif
  for(Int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    dPSNR[i]=0.0;
#if WCG_WPSNR
    dPSNRWeighted[i]=0.0;
    MSEyuvframeWeighted[i] = 0.0;
#endif
  }

  PelStorage interm;

  if (conversion != IPCOLOURSPACE_UNCHANGED)
  {
    interm.create(pic.chromaFormat, Area(Position(), pic.Y()));
    VideoIOYuv::ColourSpaceConvert(pic, interm, conversion, false);
  }

  const CPelUnitBuf& picC = (conversion == IPCOLOURSPACE_UNCHANGED) ? pic : interm;

  //===== calculate PSNR =====
  Double MSEyuvframe[MAX_NUM_COMPONENT] = {0, 0, 0};
  const ChromaFormat formatD = pic.chromaFormat;
  const ChromaFormat format  = sps.getChromaFormatIdc();

  const bool bPicIsField     = pcPic->fieldPic;
  const Slice*  pcSlice      = pcPic->slices[0];
#if ENABLE_QPA && FRAME_WEIGHTING
  const UInt    currDQP      = (pcSlice->getPOC() % m_pcEncLib->getIntraPeriod()) == 0 ? 0 : DQP[pcSlice->getPOC() % m_pcEncLib->getGOPSize()];
  const double  frameWeight  = pow(2.0, (double)currDQP / -3.0);

  if (useWPSNR) m_gcAnalyzeAll.addWeight(frameWeight);
#endif
  for (Int comp = 0; comp < ::getNumberValidComponents(formatD); comp++)
  {
    const ComponentID compID = ComponentID(comp);
    const CPelBuf&    p = picC.get(compID);
    const CPelBuf&    o = org.get(compID);

    CHECK(!( p.width  == o.width), "Unspecified error");
    CHECK(!( p.height == o.height), "Unspecified error");

    const UInt   width  = p.width  - (m_pcEncLib->getPad(0) >> ::getComponentScaleX(compID, format));
    const UInt   height = p.height - (m_pcEncLib->getPad(1) >> (!!bPicIsField+::getComponentScaleY(compID,format)));

    // create new buffers with correct dimensions
    const CPelBuf recPB(p.bufAt(0, 0), p.stride, width, height);
    const CPelBuf orgPB(o.bufAt(0, 0), o.stride, width, height);
    const UInt    bitDepth = sps.getBitDepth(toChannelType(compID));
#if ENABLE_QPA
    const UInt64 uiSSDtemp = xFindDistortionPlane(recPB, orgPB, useWPSNR ? bitDepth : 0, ::getComponentScaleX(compID, format));
    const UInt maxval = /*useWPSNR ? (1 << bitDepth) - 1 :*/ 255 << (bitDepth - 8); // fix with WPSNR: 1023 (4095) instead of 1020 (4080) for bit-depth 10 (12)
#else
    const UInt64 uiSSDtemp = xFindDistortionPlane(recPB, orgPB, 0);
#if WCG_WPSNR
  const Double uiSSDtempWeighted = xFindDistortionPlaneWPSNR(recPB, orgPB, 0, org.get(COMPONENT_Y), compID, format);
#endif
    const UInt maxval = 255 << (bitDepth - 8);
#endif
    const UInt size   = width * height;
    const Double fRefValue = (Double)maxval * maxval * size;
    dPSNR[comp]       = uiSSDtemp ? 10.0 * log10(fRefValue / (Double)uiSSDtemp) : 999.99;
    MSEyuvframe[comp] = (Double)uiSSDtemp / size;
#if WCG_WPSNR
    if (useLumaWPSNR)
    {
      dPSNRWeighted[comp] = uiSSDtempWeighted ? 10.0 * log10(fRefValue / (Double)uiSSDtempWeighted) : 999.99;
      MSEyuvframeWeighted[comp] = (Double)uiSSDtempWeighted / size;
    }
#endif

#if ENABLE_QPA && FRAME_WEIGHTING
    if (useWPSNR) m_gcAnalyzeAll.addWeightedSSD(frameWeight * (double)uiSSDtemp / fRefValue, compID);
#endif
  }


  /* calculate the size of the access unit, excluding:
   *  - any AnnexB contributions (start_code_prefix, zero_byte, etc.,)
   *  - SEI NAL units
   */
  UInt numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
  {
    UInt numRBSPBytes_nal = UInt((*it)->m_nalUnitData.str().size());
    if (m_pcCfg->getSummaryVerboseness() > 0)
    {
      msg( NOTICE, "*** %6s numBytesInNALunit: %u\n", nalUnitTypeToString((*it)->m_nalUnitType), numRBSPBytes_nal);
    }
    if( ( *it )->m_nalUnitType != NAL_UNIT_PREFIX_SEI && ( *it )->m_nalUnitType != NAL_UNIT_SUFFIX_SEI )
    {
      numRBSPBytes += numRBSPBytes_nal;
#if HEVC_VPS
      if( it == accessUnit.begin() || ( *it )->m_nalUnitType == NAL_UNIT_VPS || ( *it )->m_nalUnitType == NAL_UNIT_SPS || ( *it )->m_nalUnitType == NAL_UNIT_PPS )
#else
      if (it == accessUnit.begin() || (*it)->m_nalUnitType == NAL_UNIT_SPS || (*it)->m_nalUnitType == NAL_UNIT_PPS)
#endif
      {
        numRBSPBytes += 4;
      }
      else
      {
        numRBSPBytes += 3;
      }
    }
  }

  UInt uibits = numRBSPBytes * 8;
  m_vRVM_RP.push_back( uibits );

  //===== add PSNR =====
  m_gcAnalyzeAll.addResult (dPSNR, (Double)uibits, MSEyuvframe);
  if (pcSlice->isIntra())
  {
    m_gcAnalyzeI.addResult (dPSNR, (Double)uibits, MSEyuvframe);
    *PSNR_Y = dPSNR[COMPONENT_Y];
  }
  if (pcSlice->isInterP())
  {
    m_gcAnalyzeP.addResult (dPSNR, (Double)uibits, MSEyuvframe);
    *PSNR_Y = dPSNR[COMPONENT_Y];
  }
  if (pcSlice->isInterB())
  {
    m_gcAnalyzeB.addResult (dPSNR, (Double)uibits, MSEyuvframe);
    *PSNR_Y = dPSNR[COMPONENT_Y];
  }
#if WCG_WPSNR
  if (useLumaWPSNR)
  {
    m_gcAnalyzeWPSNR.addResult(dPSNRWeighted, (Double)uibits, MSEyuvframeWeighted);
  }
#endif

  TChar c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');
  if (! pcPic->referenced)
  {
    c += 32;
  }

  if( g_verbosity >= NOTICE )
  {
    msg( NOTICE, "POC %4d TId: %1d ( %c-SLICE, QP %d ) %10d bits",
         pcSlice->getPOC() - pcSlice->getLastIDR(),
         pcSlice->getTLayer(),
         c,
         pcSlice->getSliceQp(),
         uibits );

    msg( NOTICE, " [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", dPSNR[COMPONENT_Y], dPSNR[COMPONENT_Cb], dPSNR[COMPONENT_Cr] );
    if( printFrameMSE )
    {
      msg( NOTICE, " [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", MSEyuvframe[COMPONENT_Y], MSEyuvframe[COMPONENT_Cb], MSEyuvframe[COMPONENT_Cr] );
    }
#if WCG_WPSNR
    if (useLumaWPSNR)
    {
      msg(NOTICE, " [WY %6.4lf dB    WU %6.4lf dB    WV %6.4lf dB]", dPSNRWeighted[COMPONENT_Y], dPSNRWeighted[COMPONENT_Cb], dPSNRWeighted[COMPONENT_Cr]);
    }
#endif
    msg( NOTICE, " [ET %5.0f ]", dEncTime );

    // msg( SOME, " [WP %d]", pcSlice->getUseWeightedPrediction());

    for( Int iRefList = 0; iRefList < 2; iRefList++ )
    {
      msg( NOTICE, " [L%d ", iRefList );
      for( Int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx( RefPicList( iRefList ) ); iRefIndex++ )
      {
        msg( NOTICE, "%d ", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ) - pcSlice->getLastIDR() );
      }
      msg( NOTICE, "]" );
    }
  }
  else if( g_verbosity >= INFO )
  {
    std::cout << "\r\t" << pcSlice->getPOC();
    std::cout.flush();
  }
}

Void EncGOP::xCalculateInterlacedAddPSNR( Picture* pcPicOrgFirstField, Picture* pcPicOrgSecondField,
                                          PelUnitBuf cPicRecFirstField, PelUnitBuf cPicRecSecondField,
                                          const InputColourSpaceConversion conversion, const Bool printFrameMSE, Double* PSNR_Y )
{
  const SPS &sps = *pcPicOrgFirstField->cs->sps;
  const ChromaFormat format = sps.getChromaFormatIdc();
  Double  dPSNR[MAX_NUM_COMPONENT];
  Picture    *apcPicOrgFields[2] = {pcPicOrgFirstField, pcPicOrgSecondField};
  PelUnitBuf acPicRecFields[2]   = {cPicRecFirstField, cPicRecSecondField};
#if ENABLE_QPA
  const bool    useWPSNR = m_pcEncLib->getUseWPSNR();
#endif
  for(Int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    dPSNR[i]=0.0;
  }

  PelStorage cscd[2 /* first/second field */];
  if (conversion!=IPCOLOURSPACE_UNCHANGED)
  {
    for(UInt fieldNum=0; fieldNum<2; fieldNum++)
    {
      PelUnitBuf& reconField= (acPicRecFields[fieldNum]);
      cscd[fieldNum].create( reconField.chromaFormat, Area( Position(), reconField.Y()) );
      VideoIOYuv::ColourSpaceConvert(reconField, cscd[fieldNum], conversion, false);
      acPicRecFields[fieldNum]=cscd[fieldNum];
    }
  }

  //===== calculate PSNR =====
  Double MSEyuvframe[MAX_NUM_COMPONENT] = {0, 0, 0};

  CHECK(!(acPicRecFields[0].chromaFormat==acPicRecFields[1].chromaFormat), "Unspecified error");
  const UInt numValidComponents = ::getNumberValidComponents( acPicRecFields[0].chromaFormat );
#if ENABLE_QPA && FRAME_WEIGHTING
  const Slice*  pcSlice      = pcPicOrgFirstField->slices[0];
  const UInt    currDQP      = (pcSlice->getPOC() % m_pcEncLib->getIntraPeriod()) == 0 ? 0 : DQP[pcSlice->getPOC() % m_pcEncLib->getGOPSize()];
  const double  frameWeight  = pow(2.0, (double)currDQP / -3.0);

  if (useWPSNR) m_gcAnalyzeAll_in.addWeight(frameWeight);
#endif
  for (Int chan = 0; chan < numValidComponents; chan++)
  {
    const ComponentID ch=ComponentID(chan);
    CHECK(!(acPicRecFields[0].get(ch).width==acPicRecFields[1].get(ch).width), "Unspecified error");
    CHECK(!(acPicRecFields[0].get(ch).height==acPicRecFields[0].get(ch).height), "Unspecified error");

    UInt64 uiSSDtemp=0;
    const UInt width    = acPicRecFields[0].get(ch).width - (m_pcEncLib->getPad(0) >> ::getComponentScaleX(ch, format));
    const UInt height   = acPicRecFields[0].get(ch).height - ((m_pcEncLib->getPad(1) >> 1) >> ::getComponentScaleY(ch, format));
    const UInt bitDepth = sps.getBitDepth(toChannelType(ch));

    for(UInt fieldNum=0; fieldNum<2; fieldNum++)
    {
      CHECK(!(conversion == IPCOLOURSPACE_UNCHANGED), "Unspecified error");
#if ENABLE_QPA
      uiSSDtemp += xFindDistortionPlane( acPicRecFields[fieldNum].get(ch), apcPicOrgFields[fieldNum]->getOrigBuf().get(ch), useWPSNR ? bitDepth : 0, ::getComponentScaleX(ch, format) );
#else
      uiSSDtemp += xFindDistortionPlane( acPicRecFields[fieldNum].get(ch), apcPicOrgFields[fieldNum]->getOrigBuf().get(ch), 0 );
#endif
    }
#if ENABLE_QPA
    const UInt maxval = /*useWPSNR ? (1 << bitDepth) - 1 :*/ 255 << (bitDepth - 8); // fix with WPSNR: 1023 (4095) instead of 1020 (4080) for bit-depth 10 (12)
#else
    const UInt maxval = 255 << (bitDepth - 8);
#endif
    const UInt size   = width * height * 2;
    const Double fRefValue = (Double)maxval * maxval * size;
    dPSNR[ch]         = uiSSDtemp ? 10.0 * log10(fRefValue / (Double)uiSSDtemp) : 999.99;
    MSEyuvframe[ch]   = (Double)uiSSDtemp / size;
#if ENABLE_QPA && FRAME_WEIGHTING
    if (useWPSNR) m_gcAnalyzeAll_in.addWeightedSSD(frameWeight * (double)uiSSDtemp / fRefValue, ch);
#endif
  }

  UInt uibits = 0; // the number of bits for the pair is not calculated here - instead the overall total is used elsewhere.

  //===== add PSNR =====
  m_gcAnalyzeAll_in.addResult (dPSNR, (Double)uibits, MSEyuvframe);

  *PSNR_Y = dPSNR[COMPONENT_Y];

  msg( DETAILS, "\n                                      Interlaced frame %d: [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", pcPicOrgSecondField->getPOC()/2 , dPSNR[COMPONENT_Y], dPSNR[COMPONENT_Cb], dPSNR[COMPONENT_Cr] );
  if (printFrameMSE)
  {
    msg( DETAILS, " [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", MSEyuvframe[COMPONENT_Y], MSEyuvframe[COMPONENT_Cb], MSEyuvframe[COMPONENT_Cr] );
  }

  for(UInt fieldNum=0; fieldNum<2; fieldNum++)
  {
    cscd[fieldNum].destroy();
  }
}

/** Function for deciding the nal_unit_type.
 * \param pocCurr POC of the current picture
 * \param lastIDR  POC of the last IDR picture
 * \param isField  true to indicate field coding
 * \returns the NAL unit type of the picture
 * This function checks the configuration and returns the appropriate nal_unit_type for the picture.
 */
NalUnitType EncGOP::getNalUnitType(Int pocCurr, Int lastIDR, Bool isField)
{
  if (pocCurr == 0)
  {
    return NAL_UNIT_CODED_SLICE_IDR_W_RADL;
  }

  if(m_pcCfg->getEfficientFieldIRAPEnabled() && isField && pocCurr == 1)
  {
    // to avoid the picture becoming an IRAP
    return NAL_UNIT_CODED_SLICE_TRAIL_R;
  }

  if(m_pcCfg->getDecodingRefreshType() != 3 && (pocCurr - isField) % m_pcCfg->getIntraPeriod() == 0)
  {
    if (m_pcCfg->getDecodingRefreshType() == 1)
    {
      return NAL_UNIT_CODED_SLICE_CRA;
    }
    else if (m_pcCfg->getDecodingRefreshType() == 2)
    {
      return NAL_UNIT_CODED_SLICE_IDR_W_RADL;
    }
  }
  if(m_pocCRA>0)
  {
    if(pocCurr<m_pocCRA)
    {
      // All leading pictures are being marked as TFD pictures here since current encoder uses all
      // reference pictures while encoding leading pictures. An encoder can ensure that a leading
      // picture can be still decodable when random accessing to a CRA/CRANT/BLA/BLANT picture by
      // controlling the reference pictures used for encoding that leading picture. Such a leading
      // picture need not be marked as a TFD picture.
      return NAL_UNIT_CODED_SLICE_RASL_R;
    }
  }
  if (lastIDR>0)
  {
    if (pocCurr < lastIDR)
    {
      return NAL_UNIT_CODED_SLICE_RADL_R;
    }
  }
  return NAL_UNIT_CODED_SLICE_TRAIL_R;
}

Void EncGOP::xUpdateRasInit(Slice* slice)
{
  slice->setPendingRasInit( false );
  if ( slice->getPOC() > m_lastRasPoc )
  {
    m_lastRasPoc = MAX_INT;
    slice->setPendingRasInit( true );
  }
  if ( slice->isIRAP() )
  {
    m_lastRasPoc = slice->getPOC();
  }
}

Double EncGOP::xCalculateRVM()
{
  Double dRVM = 0;

  if( m_pcCfg->getGOPSize() == 1 && m_pcCfg->getIntraPeriod() != 1 && m_pcCfg->getFramesToBeEncoded() > RVM_VCEGAM10_M * 2 )
  {
    // calculate RVM only for lowdelay configurations
    std::vector<Double> vRL , vB;
    size_t N = m_vRVM_RP.size();
    vRL.resize( N );
    vB.resize( N );

    Int i;
    Double dRavg = 0 , dBavg = 0;
    vB[RVM_VCEGAM10_M] = 0;
    for( i = RVM_VCEGAM10_M + 1 ; i < N - RVM_VCEGAM10_M + 1 ; i++ )
    {
      vRL[i] = 0;
      for( Int j = i - RVM_VCEGAM10_M ; j <= i + RVM_VCEGAM10_M - 1 ; j++ )
      {
        vRL[i] += m_vRVM_RP[j];
      }
      vRL[i] /= ( 2 * RVM_VCEGAM10_M );
      vB[i] = vB[i-1] + m_vRVM_RP[i] - vRL[i];
      dRavg += m_vRVM_RP[i];
      dBavg += vB[i];
    }

    dRavg /= ( N - 2 * RVM_VCEGAM10_M );
    dBavg /= ( N - 2 * RVM_VCEGAM10_M );

    Double dSigamB = 0;
    for( i = RVM_VCEGAM10_M + 1 ; i < N - RVM_VCEGAM10_M + 1 ; i++ )
    {
      Double tmp = vB[i] - dBavg;
      dSigamB += tmp * tmp;
    }
    dSigamB = sqrt( dSigamB / ( N - 2 * RVM_VCEGAM10_M ) );

    Double f = sqrt( 12.0 * ( RVM_VCEGAM10_M - 1 ) / ( RVM_VCEGAM10_M + 1 ) );

    dRVM = dSigamB / dRavg * f;
  }

  return( dRVM );
}

/** Attaches the input bitstream to the stream in the output NAL unit
    Updates rNalu to contain concatenated bitstream. rpcBitstreamRedirect is cleared at the end of this function call.
 *  \param codedSliceData contains the coded slice data (bitstream) to be concatenated to rNalu
 *  \param rNalu          target NAL unit
 */
Void EncGOP::xAttachSliceDataToNalUnit (OutputNALUnit& rNalu, OutputBitstream* codedSliceData)
{
  // Byte-align
  rNalu.m_Bitstream.writeByteAlignment();   // Slice header byte-alignment

  // Perform bitstream concatenation
  if (codedSliceData->getNumberOfWrittenBits() > 0)
  {
    rNalu.m_Bitstream.addSubstream(codedSliceData);
  }
  codedSliceData->clear();
}

// Function will arrange the long-term pictures in the decreasing order of poc_lsb_lt,
// and among the pictures with the same lsb, it arranges them in increasing delta_poc_msb_cycle_lt value
Void EncGOP::arrangeLongtermPicturesInRPS(Slice *pcSlice, PicList& rcListPic)
{
  if(pcSlice->getRPS()->getNumberOfLongtermPictures() == 0)
  {
    return;
  }
  // we can only modify the local RPS!
  CHECK(!(pcSlice->getRPSidx()==-1), "Unspecified error");
  ReferencePictureSet *rps = pcSlice->getLocalRPS();

  // Arrange long-term reference pictures in the correct order of LSB and MSB,
  // and assign values for pocLSBLT and MSB present flag
  Int longtermPicsPoc[MAX_NUM_REF_PICS], longtermPicsLSB[MAX_NUM_REF_PICS], indices[MAX_NUM_REF_PICS];
  Int longtermPicsMSB[MAX_NUM_REF_PICS];
  Bool mSBPresentFlag[MAX_NUM_REF_PICS];
  ::memset(longtermPicsPoc, 0, sizeof(longtermPicsPoc));    // Store POC values of LTRP
  ::memset(longtermPicsLSB, 0, sizeof(longtermPicsLSB));    // Store POC LSB values of LTRP
  ::memset(longtermPicsMSB, 0, sizeof(longtermPicsMSB));    // Store POC LSB values of LTRP
  ::memset(indices        , 0, sizeof(indices));            // Indices to aid in tracking sorted LTRPs
  ::memset(mSBPresentFlag , 0, sizeof(mSBPresentFlag));     // Indicate if MSB needs to be present

  // Get the long-term reference pictures
  Int offset = rps->getNumberOfNegativePictures() + rps->getNumberOfPositivePictures();
  Int i, ctr = 0;
  Int maxPicOrderCntLSB = 1 << pcSlice->getSPS()->getBitsForPOC();
  for(i = rps->getNumberOfPictures() - 1; i >= offset; i--, ctr++)
  {
    longtermPicsPoc[ctr] = rps->getPOC(i);                                  // LTRP POC
    longtermPicsLSB[ctr] = getLSB(longtermPicsPoc[ctr], maxPicOrderCntLSB); // LTRP POC LSB
    indices[ctr]      = i;
    longtermPicsMSB[ctr] = longtermPicsPoc[ctr] - longtermPicsLSB[ctr];
  }
  Int numLongPics = rps->getNumberOfLongtermPictures();
  CHECK(!(ctr == numLongPics), "Unspecified error");

  // Arrange pictures in decreasing order of MSB;
  for(i = 0; i < numLongPics; i++)
  {
    for(Int j = 0; j < numLongPics - 1; j++)
    {
      if(longtermPicsMSB[j] < longtermPicsMSB[j+1])
      {
        std::swap(longtermPicsPoc[j], longtermPicsPoc[j+1]);
        std::swap(longtermPicsLSB[j], longtermPicsLSB[j+1]);
        std::swap(longtermPicsMSB[j], longtermPicsMSB[j+1]);
        std::swap(indices[j]        , indices[j+1]        );
      }
    }
  }

  for(i = 0; i < numLongPics; i++)
  {
    // Check if MSB present flag should be enabled.
    // Check if the buffer contains any pictures that have the same LSB.
    PicList::iterator  iterPic = rcListPic.begin();
    Picture*                      pcPic;
    while ( iterPic != rcListPic.end() )
    {
      pcPic = *iterPic;
      if( (getLSB(pcPic->getPOC(), maxPicOrderCntLSB) == longtermPicsLSB[i])   &&     // Same LSB
                                      (pcPic->referenced)     &&    // Reference picture
                                        (pcPic->getPOC() != longtermPicsPoc[i])    )  // Not the LTRP itself
      {
        mSBPresentFlag[i] = true;
        break;
      }
      iterPic++;
    }
  }

  // tempArray for usedByCurr flag
  Bool tempArray[MAX_NUM_REF_PICS]; ::memset(tempArray, 0, sizeof(tempArray));
  for(i = 0; i < numLongPics; i++)
  {
    tempArray[i] = rps->getUsed(indices[i]);
  }
  // Now write the final values;
  ctr = 0;
  Int currMSB = 0, currLSB = 0;
  // currPicPoc = currMSB + currLSB
  currLSB = getLSB(pcSlice->getPOC(), maxPicOrderCntLSB);
  currMSB = pcSlice->getPOC() - currLSB;

  for(i = rps->getNumberOfPictures() - 1; i >= offset; i--, ctr++)
  {
    rps->setPOC                   (i, longtermPicsPoc[ctr]);
    rps->setDeltaPOC              (i, - pcSlice->getPOC() + longtermPicsPoc[ctr]);
    rps->setUsed                  (i, tempArray[ctr]);
    rps->setPocLSBLT              (i, longtermPicsLSB[ctr]);
    rps->setDeltaPocMSBCycleLT    (i, (currMSB - (longtermPicsPoc[ctr] - longtermPicsLSB[ctr])) / maxPicOrderCntLSB);
    rps->setDeltaPocMSBPresentFlag(i, mSBPresentFlag[ctr]);

    CHECK(!(rps->getDeltaPocMSBCycleLT(i) >= 0), "Unspecified error");   // Non-negative value
  }
  for(i = rps->getNumberOfPictures() - 1, ctr = 1; i >= offset; i--, ctr++)
  {
    for(Int j = rps->getNumberOfPictures() - 1 - ctr; j >= offset; j--)
    {
      // Here at the encoder we know that we have set the full POC value for the LTRPs, hence we
      // don't have to check the MSB present flag values for this constraint.
      CHECK(!( rps->getPOC(i) != rps->getPOC(j) ), "Unspecified error"); // If assert fails, LTRP entry repeated in RPS!!!
    }
  }
}

Void EncGOP::applyDeblockingFilterMetric( Picture* pcPic, UInt uiNumSlices )
{
  PelBuf cPelBuf = pcPic->getRecoBuf().get( COMPONENT_Y );
  Pel* Rec    = cPelBuf.buf;
  const Int  stride = cPelBuf.stride;
  const UInt picWidth = cPelBuf.width;
  const UInt picHeight = cPelBuf.height;

  Pel* tempRec = Rec;
  const Slice* pcSlice = pcPic->slices[0];
  UInt log2maxTB = pcSlice->getSPS()->getQuadtreeTULog2MaxSize();
  UInt maxTBsize = (1<<log2maxTB);
  const UInt minBlockArtSize = 8;
  const UInt noCol = (picWidth>>log2maxTB);
  const UInt noRows = (picHeight>>log2maxTB);
  CHECK(!(noCol > 1), "Unspecified error");
  CHECK(!(noRows > 1), "Unspecified error");
  std::vector<UInt64> colSAD(noCol,  UInt64(0));
  std::vector<UInt64> rowSAD(noRows, UInt64(0));
  UInt colIdx = 0;
  UInt rowIdx = 0;
  Pel p0, p1, p2, q0, q1, q2;

  Int qp = pcSlice->getSliceQp();
  const Int bitDepthLuma=pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
  Int bitdepthScale = 1 << (bitDepthLuma-8);
  Int beta = LoopFilter::getBeta( qp ) * bitdepthScale;
  const Int thr2 = (beta>>2);
  const Int thr1 = 2*bitdepthScale;
  UInt a = 0;

  if (maxTBsize > minBlockArtSize)
  {
    // Analyze vertical artifact edges
    for(Int c = maxTBsize; c < picWidth; c += maxTBsize)
    {
      for(Int r = 0; r < picHeight; r++)
      {
        p2 = Rec[c-3];
        p1 = Rec[c-2];
        p0 = Rec[c-1];
        q0 = Rec[c];
        q1 = Rec[c+1];
        q2 = Rec[c+2];
        a = ((abs(p2-(p1<<1)+p0)+abs(q0-(q1<<1)+q2))<<1);
        if ( thr1 < a && a < thr2)
        {
          colSAD[colIdx] += abs(p0 - q0);
        }
        Rec += stride;
      }
      colIdx++;
      Rec = tempRec;
    }

    // Analyze horizontal artifact edges
    for(Int r = maxTBsize; r < picHeight; r += maxTBsize)
    {
      for(Int c = 0; c < picWidth; c++)
      {
        p2 = Rec[c + (r-3)*stride];
        p1 = Rec[c + (r-2)*stride];
        p0 = Rec[c + (r-1)*stride];
        q0 = Rec[c + r*stride];
        q1 = Rec[c + (r+1)*stride];
        q2 = Rec[c + (r+2)*stride];
        a = ((abs(p2-(p1<<1)+p0)+abs(q0-(q1<<1)+q2))<<1);
        if (thr1 < a && a < thr2)
        {
          rowSAD[rowIdx] += abs(p0 - q0);
        }
      }
      rowIdx++;
    }
  }

  UInt64 colSADsum = 0;
  UInt64 rowSADsum = 0;
  for(Int c = 0; c < noCol-1; c++)
  {
    colSADsum += colSAD[c];
  }
  for(Int r = 0; r < noRows-1; r++)
  {
    rowSADsum += rowSAD[r];
  }

  colSADsum <<= 10;
  rowSADsum <<= 10;
  colSADsum /= (noCol-1);
  colSADsum /= picHeight;
  rowSADsum /= (noRows-1);
  rowSADsum /= picWidth;

  UInt64 avgSAD = ((colSADsum + rowSADsum)>>1);
  avgSAD >>= (bitDepthLuma-8);

  if ( avgSAD > 2048 )
  {
    avgSAD >>= 9;
    Int offset = Clip3(2,6,(Int)avgSAD);
    for (Int i=0; i<uiNumSlices; i++)
    {
      Slice* pcLocalSlice = pcPic->slices[i];
      pcLocalSlice->setDeblockingFilterOverrideFlag   ( true);
      pcLocalSlice->setDeblockingFilterDisable        ( false);
      pcLocalSlice->setDeblockingFilterBetaOffsetDiv2 ( offset );
      pcLocalSlice->setDeblockingFilterTcOffsetDiv2   ( offset );
    }
  }
  else
  {
    for (Int i=0; i<uiNumSlices; i++)
    {
      Slice* pcLocalSlice = pcPic->slices[i];
      const PPS* pcPPS = pcSlice->getPPS();
      pcLocalSlice->setDeblockingFilterOverrideFlag  ( false);
      pcLocalSlice->setDeblockingFilterDisable       ( pcPPS->getPPSDeblockingFilterDisabledFlag() );
      pcLocalSlice->setDeblockingFilterBetaOffsetDiv2( pcPPS->getDeblockingFilterBetaOffsetDiv2() );
      pcLocalSlice->setDeblockingFilterTcOffsetDiv2  ( pcPPS->getDeblockingFilterTcOffsetDiv2()   );
    }
  }
}

#if W0038_DB_OPT
Void EncGOP::applyDeblockingFilterParameterSelection( Picture* pcPic, const UInt numSlices, const Int gopID )
{
  enum DBFltParam
  {
    DBFLT_PARAM_AVAILABLE = 0,
    DBFLT_DISABLE_FLAG,
    DBFLT_BETA_OFFSETD2,
    DBFLT_TC_OFFSETD2,
    //NUM_DBFLT_PARAMS
  };
  const Int MAX_BETA_OFFSET = 3;
  const Int MIN_BETA_OFFSET = -3;
  const Int MAX_TC_OFFSET = 3;
  const Int MIN_TC_OFFSET = -3;

  PelUnitBuf reco = pcPic->getRecoBuf();

  const Int currQualityLayer = (pcPic->slices[0]->getSliceType() != I_SLICE) ? m_pcCfg->getGOPEntry(gopID).m_temporalId+1 : 0;
  CHECK(!(currQualityLayer <MAX_ENCODER_DEBLOCKING_QUALITY_LAYERS), "Unspecified error");

  CodingStructure& cs = *pcPic->cs;

  if(!m_pcDeblockingTempPicYuv)
  {
    m_pcDeblockingTempPicYuv = new PelStorage;
    m_pcDeblockingTempPicYuv->create( cs.area );
    memset(m_DBParam, 0, sizeof(m_DBParam));
  }

  //preserve current reconstruction
  m_pcDeblockingTempPicYuv->copyFrom ( reco );

  const Bool bNoFiltering      = m_DBParam[currQualityLayer][DBFLT_PARAM_AVAILABLE] && m_DBParam[currQualityLayer][DBFLT_DISABLE_FLAG]==false /*&& pcPic->getTLayer()==0*/;
  const Int  maxBetaOffsetDiv2 = bNoFiltering? Clip3(MIN_BETA_OFFSET, MAX_BETA_OFFSET, m_DBParam[currQualityLayer][DBFLT_BETA_OFFSETD2]+1) : MAX_BETA_OFFSET;
  const Int  minBetaOffsetDiv2 = bNoFiltering? Clip3(MIN_BETA_OFFSET, MAX_BETA_OFFSET, m_DBParam[currQualityLayer][DBFLT_BETA_OFFSETD2]-1) : MIN_BETA_OFFSET;
  const Int  maxTcOffsetDiv2   = bNoFiltering? Clip3(MIN_TC_OFFSET, MAX_TC_OFFSET, m_DBParam[currQualityLayer][DBFLT_TC_OFFSETD2]+2)       : MAX_TC_OFFSET;
  const Int  minTcOffsetDiv2   = bNoFiltering? Clip3(MIN_TC_OFFSET, MAX_TC_OFFSET, m_DBParam[currQualityLayer][DBFLT_TC_OFFSETD2]-2)       : MIN_TC_OFFSET;

  UInt64 distBetaPrevious      = std::numeric_limits<UInt64>::max();
  UInt64 distMin               = std::numeric_limits<UInt64>::max();
  Bool   bDBFilterDisabledBest = true;
  Int    betaOffsetDiv2Best    = 0;
  Int    tcOffsetDiv2Best      = 0;

  for(Int betaOffsetDiv2=maxBetaOffsetDiv2; betaOffsetDiv2>=minBetaOffsetDiv2; betaOffsetDiv2--)
  {
    UInt64 distTcMin = std::numeric_limits<UInt64>::max();
    for(Int tcOffsetDiv2=maxTcOffsetDiv2; tcOffsetDiv2 >= minTcOffsetDiv2; tcOffsetDiv2--)
    {
      for (Int i=0; i<numSlices; i++)
      {
        Slice* pcSlice = pcPic->slices[i];
        pcSlice->setDeblockingFilterOverrideFlag  ( true);
        pcSlice->setDeblockingFilterDisable       ( false);
        pcSlice->setDeblockingFilterBetaOffsetDiv2( betaOffsetDiv2 );
        pcSlice->setDeblockingFilterTcOffsetDiv2  ( tcOffsetDiv2 );
      }

      // restore reconstruction
      reco.copyFrom( *m_pcDeblockingTempPicYuv );

      const UInt64 dist = preLoopFilterPicAndCalcDist( pcPic );

      if(dist < distMin)
      {
        distMin = dist;
        bDBFilterDisabledBest = false;
        betaOffsetDiv2Best  = betaOffsetDiv2;
        tcOffsetDiv2Best = tcOffsetDiv2;
      }
      if(dist < distTcMin)
      {
        distTcMin = dist;
      }
      else if(tcOffsetDiv2 <-2)
      {
        break;
      }
    }
    if(betaOffsetDiv2<-1 && distTcMin >= distBetaPrevious)
    {
      break;
    }
    distBetaPrevious = distTcMin;
  }

  //update:
  m_DBParam[currQualityLayer][DBFLT_PARAM_AVAILABLE] = 1;
  m_DBParam[currQualityLayer][DBFLT_DISABLE_FLAG]    = bDBFilterDisabledBest;
  m_DBParam[currQualityLayer][DBFLT_BETA_OFFSETD2]   = betaOffsetDiv2Best;
  m_DBParam[currQualityLayer][DBFLT_TC_OFFSETD2]     = tcOffsetDiv2Best;

  // restore reconstruction
  reco.copyFrom( *m_pcDeblockingTempPicYuv );

  const PPS* pcPPS = pcPic->slices[0]->getPPS();
  if(bDBFilterDisabledBest)
  {
    for (Int i=0; i<numSlices; i++)
    {
      Slice* pcSlice = pcPic->slices[i];
      pcSlice->setDeblockingFilterOverrideFlag( true);
      pcSlice->setDeblockingFilterDisable     ( true);
    }
  }
  else if(betaOffsetDiv2Best == pcPPS->getDeblockingFilterBetaOffsetDiv2() &&  tcOffsetDiv2Best == pcPPS->getDeblockingFilterTcOffsetDiv2())
  {
    for (Int i=0; i<numSlices; i++)
    {
      Slice*      pcSlice = pcPic->slices[i];
      pcSlice->setDeblockingFilterOverrideFlag   ( false);
      pcSlice->setDeblockingFilterDisable        ( pcPPS->getPPSDeblockingFilterDisabledFlag() );
      pcSlice->setDeblockingFilterBetaOffsetDiv2 ( pcPPS->getDeblockingFilterBetaOffsetDiv2() );
      pcSlice->setDeblockingFilterTcOffsetDiv2   ( pcPPS->getDeblockingFilterTcOffsetDiv2()   );
    }
  }
  else
  {
    for (Int i=0; i<numSlices; i++)
    {
      Slice* pcSlice = pcPic->slices[i];
      pcSlice->setDeblockingFilterOverrideFlag   ( true);
      pcSlice->setDeblockingFilterDisable        ( false );
      pcSlice->setDeblockingFilterBetaOffsetDiv2 ( betaOffsetDiv2Best);
      pcSlice->setDeblockingFilterTcOffsetDiv2   ( tcOffsetDiv2Best);
    }
  }
}
#endif
//! \}
