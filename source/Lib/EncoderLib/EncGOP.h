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

/** \file     EncGOP.h
    \brief    GOP encoder class (header)
*/

#ifndef __ENCGOP__
#define __ENCGOP__

#include <list>

#include <stdlib.h>

#include "CommonLib/Picture.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/NAL.h"
#include "EncSampleAdaptiveOffset.h"
#include "EncSlice.h"
#include "VLCWriter.h"
#include "CABACWriter.h"
#include "SEIwrite.h"
#include "SEIEncoder.h"

#include "Analyze.h"
#include "RateCtrl.h"
#include <vector>
#include <iostream>
#include <algorithm>
using namespace std;
//! \ingroup EncoderLib
//! \{

class EncLib;

// ====================================================================================================================
// Class definition
// ====================================================================================================================
class AUWriterIf
{
public:
  virtual void outputAU( const AccessUnit& ) = 0;
};


class EncGOP
{
  class DUData
  {
  public:
    DUData()
    :accumBitsDU(0)
    ,accumNalsDU(0) {};

    Int accumBitsDU;
    Int accumNalsDU;
  };

private:

  Analyze                 m_gcAnalyzeAll;
  Analyze                 m_gcAnalyzeI;
  Analyze                 m_gcAnalyzeP;
  Analyze                 m_gcAnalyzeB;
#if WCG_WPSNR
  Analyze                 m_gcAnalyzeWPSNR;
#endif
  Analyze                 m_gcAnalyzeAll_in;

#if TRANSFORM_BGP
  TComTrQuant*     m_pcTrQuantGOP;
  TCoeff*			m_pcCoeffY;
  TCoeff*			m_pcCoeffCb;
  TCoeff*			m_pcCoeffCr;
  TCoeff*			m_pcTempCoeffY;
  TCoeff*			m_pcTempCoeffCb;
  TCoeff*			m_pcTempCoeffCr;
#endif
#if OrgBG_BLOCK_SUBSTITUTION
  Double*            m_pcNablaY;
  Double*            m_pcNablaCb;
  Double*            m_pcNablaCr;
#endif
#if ADJUST_QP
  Bool					m_BGQPflag = false;
#endif

#if ENCODE_BGPIC
  Picture* m_bgPic;
#endif

#if BLOCK_GEN
  Picture* m_bgNewBlocksOrgGop;  //存放背景org未编块
  Picture* m_bgNewBlocksRecGop;  
  Picture* PrePicRecoGop;  //skip
  Picture* m_bgNewBlockOrgGop;  //生成背景时第三方
  Picture* m_bgNewBlockRecGop; 
  Picture* m_bgNewBlockRecoGop;
  Picture* DoubleBgRecGop;
  Int BgBlock[12000] = {0};//32x32 最大4000  16x16 1920 720最大为12000
  Bool isencode = false;
  Bool CTUisencode = false;
  Bool isupdate = false;
  Bool isselect = false;
  Bool isselectencode = false;
  
  Int updatenum = 0;
  Double BlockDPP[12000] = { 0.0 };
  Double BlockLambda[1000] = { 0.0 };
  Int BgCTU[1000] = { 0 };
  Int BlockSel[12000] = { 0 };

  Int BgBlock0[12000] = { 0 };
  double Bgselect[12000] = { 0 };
  double Bgselect1[12000] = { 0 };
  Int BgBlock1[12000] = { 0,0,0,0,0,0,1,1,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,1,0,0,2,2,27,19,95,94,11,5,0,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,1,19,175,100,4,2,0,1,0,0,2,0,0,0,1,1,0,1,0,0,0,0,0,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,2,0,0,108,270,294,2,4,0,0,0,0,2,0,0,0,1,1,0,1,0,1,1,0,0,2,0,1,2,1,0,0,0,0,0,0,1,0,2,1,2,0,43,5,6,4,0,2,0,0,0,0,0,0,0,0,1,1,0,0,2,0,1,2,1,0,1,1,4,4,0,0,1,0,0,0,0,5,4,5,4,0,19,7,6,5,4,4,0,0,1,1,1,1,0,1,2,2,0,0,2,1,1,3,1,0,3,2,2,1,1,1,0,20,12,10,20,20,2,10,8,0,0,17,5,3,2,4,0,0,2,2,1,0,1,1,6,6,11,6,3,5,2,1,5,13,0,0,1,3,0,1,0,27,6,6,18,8,0,0,11,0,0,45,22,13,1,3,0,0,1,1,0,0,1,8,3,13,0,27,10,7,12,32,0,18,12,11,6,2,0,1,0,22,1,1,12,15,4,16,22,0,2,29,13,2,1,3,2,1,0,2,2,4,12,6,3,1,0,20,5,6,20,19,20,1,0,0,1,1,1,0,1,29,21,20,2,6,1,22,8,9,1,32,8,4,2,3,3,1,2,2,2,3,7,0,0,1,0,0,13,7,20,19,17,0,1,2,3,1,0,5,12,28,18,9,3,2,3,4,2,5,7,29,1,0,1,4,2,2,2,3,3,4,0,2,1,0,0,3,14,8,20,19,31,7,0,10,0,3,0,6,16,60,11,18,4,1,2,1,2,11,72,49,8,2,1,2,0,0,2,5,1,0,0,0,0,1,0,0,14,11,16,22,9,11,3,5,1,5,2,0,3,24,38,31,1,0,2,1,6,33,34,62,11,10,14,61,2,7,4,4,1,0,0,0,0,0,0,0,14,13,20,20,16,11,3,0,1,2,2,1,3,5,29,20,3,1,1,1,13,98,69,0,0,17,111,186,3,7,1,1,0,0,1,2,0,0,0,0,13,9,22,13,17,15,6,0,3,1,1,4,0,12,31,0,29,15,49,10,21,4,24,133,66,13,81,38,2,6,2,1,0,0,0,1,0,1,0,0,1,7,16,9,1,11,1,0,2,4,8,12,0,0,13,44,64,38,3,0,0,2,5,180,122,20,34,8,1,3,5,2,0,0,0,0,0,0,1,0,0,0,12,2,2,8,1,4,3,4,0,0,0,0,19,0,0,0,0,0,0,0,0,11,17,1,4,4,3,3,12,8,3,0,0,0,0,0,0,0,1,0,0,6,3,3,2,8,0,1,0,2,1,4,1,2,0,0,0,1,0,0,0,3,2,0,0,1,1,3,10,1,0,0,1,0,0,0,0,0,0,3,9,7,10,14,1,2,0,0,1,0,2,8,0,0,1,0,0,1,0,0,0,0,0,0,2,2,3,0,1,3,1,1,0,0,0,0,0,0,1,4,7,0,8,10,1,1,1,0,12,54,141,260,174,157,11,0,1,12,7,1,79,267,1,0,2,0,27,16,6,0,7,1,0,0,0,0,0,0,1,4,2,5,11,7,2,1,42,70,225,219,251,129,100,27,5,2,17,12,3,1,118,351,32,3,3,1,5,1,9,2,2,1,1,0,0,0,3,1,1,0,1,189,6,0,13,58,154,0,196,134,40,15,2,1,52,9,8,142,141,128,203,185,96,2,0,1,8,4,3,5,2,0,1,0,0,0,8,10,0,15,77,111,0,4,5,7,4,106,27,4,3,4,3,3,17,5,0,0,15,24,241,366,87,2,0,1,3,2,8,12,24,6,17,0,0,1,3,11,11,6,92,137,6,3,0,3,2,9,8,2,3,2,3,4,50,99,19,27,6,0,154,226,4,2,0,0};
  Int BgBlock2[12000] = { 0 };
  bool compare(int a, int b)
  {
	  return a>b;
  }
#endif
  Bool afterbg = false;
#if GENERATE_OrgBG_PIC
  Picture* m_bgNewPicYuvOrgGop;
#endif

#if GENERATE_BG_PIC
  Picture* m_bgNewPicYuvRecGop;
#endif

#if GENERATE_RESI_PIC
  Picture* m_bgNewPicYuvResiGop;
#endif

#if GENERATE_UPDATE_RESI_PIC
  Picture* m_bgNewPicYuvUpdateResiGop;
#endif

#if GENERATE_TEMPRECO_PIC
  Picture* m_bgNewPicYuvRecoGop;
#endif

#if GENERATE_RECO_PIC
  Picture* m_bgNewPicYuvTempUpdateRecoGop;
#endif

#if BG_REFERENCE_SUBSTITUTION
  Picture* m_rcPicYuvTempGop;
#endif
  //  Data
  Bool                    m_bLongtermTestPictureHasBeenCoded;
  Bool                    m_bLongtermTestPictureHasBeenCoded2;
  UInt                    m_numLongTermRefPicSPS;
  UInt                    m_ltRefPicPocLsbSps[MAX_NUM_LONG_TERM_REF_PICS];
  Bool                    m_ltRefPicUsedByCurrPicFlag[MAX_NUM_LONG_TERM_REF_PICS];
  Int                     m_iLastIDR;
  Int                     m_iGopSize;
  Int                     m_iNumPicCoded;
  Bool                    m_bFirst;
  Int                     m_iLastRecoveryPicPOC;
  Int                     m_lastRasPoc;

  //  Access channel
  EncLib*                 m_pcEncLib;
  EncCfg*                 m_pcCfg;
  EncSlice*               m_pcSliceEncoder;
  PicList*                m_pcListPic;

  HLSWriter*              m_HLSWriter;
  LoopFilter*             m_pcLoopFilter;

  SEIWriter               m_seiWriter;

  //--Adaptive Loop filter
  EncSampleAdaptiveOffset*  m_pcSAO;
  RateCtrl*                 m_pcRateCtrl;
  // indicate sequence first
  Bool                    m_bSeqFirst;

  // clean decoding refresh
  Bool                    m_bRefreshPending;
  Int                     m_pocCRA;
  NalUnitType             m_associatedIRAPType;
  Int                     m_associatedIRAPPOC;

  std::vector<Int>        m_vRVM_RP;
  UInt                    m_lastBPSEI;
  UInt                    m_totalCoded;
  Bool                    m_bufferingPeriodSEIPresentInAU;
  SEIEncoder              m_seiEncoder;
#if W0038_DB_OPT
  PelStorage*             m_pcDeblockingTempPicYuv;
  Int                     m_DBParam[MAX_ENCODER_DEBLOCKING_QUALITY_LAYERS][4];   //[layer_id][0: available; 1: bDBDisabled; 2: Beta Offset Div2; 3: Tc Offset Div2;]
#endif

  // members needed for adaptive max BT size
  UInt                    m_uiBlkSize[10];
  UInt                    m_uiNumBlk[10];
  UInt                    m_uiPrevISlicePOC;
  Bool                    m_bInitAMaxBT;

  AUWriterIf*             m_AUWriterIf;

public:
  EncGOP();
  virtual ~EncGOP();

  Void  create      ();
  Void  destroy     ();

  Void  init        ( EncLib* pcEncLib );
  Void  compressGOP ( Int iPOCLast, Int iNumPicRcvd, PicList& rcListPic, std::list<PelUnitBuf*>& rcListPicYuvRec,
                      Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
  Void  xAttachSliceDataToNalUnit (OutputNALUnit& rNalu, OutputBitstream* pcBitstreamRedirect);


  Int   getGOPSize()          { return  m_iGopSize;  }

  PicList*   getListPic()      { return m_pcListPic; }

  Void  printOutSummary      ( UInt uiNumAllPicCoded, Bool isField, const Bool printMSEBasedSNR, const Bool printSequenceMSE, const BitDepths &bitDepths );
#if W0038_DB_OPT
  UInt64  preLoopFilterPicAndCalcDist( Picture* pcPic );
#endif
  EncSlice*  getSliceEncoder()   { return m_pcSliceEncoder; }
  NalUnitType getNalUnitType( Int pocCurr, Int lastIdr, Bool isField );
  Void arrangeLongtermPicturesInRPS(Slice *, PicList& );

#if ENCODE_BGPIC
  Void setbgPic(Picture* m) { m_bgPic = m; }
  Picture* getbgPic() { return m_bgPic; }
#endif

#if BLOCK_GEN
  Void setbgNewBlocksOrgGop(Picture* m) { m_bgNewBlocksOrgGop = m; }
  Picture* getbgNewBlocksOrgGop() { return m_bgNewBlocksOrgGop; }

  Void setbgNewBlocksRecGop(Picture* m) { m_bgNewBlocksRecGop = m; }
  Picture* getbgNewBlocksRecGop() { return m_bgNewBlocksRecGop; }

  Void setPrePicRecoGop(Picture* m) { PrePicRecoGop = m; }
  Picture* getPrePicRecoGop() { return PrePicRecoGop; }

  Void setbgNewBlockOrgGop(Picture* m) { m_bgNewBlockOrgGop = m; }
  Picture* getbgNewBlockOrgGop() { return m_bgNewBlockOrgGop; }

  Void setbgNewBlockRecGop(Picture* m) { m_bgNewBlockRecGop = m; }
  Picture* getbgNewBlockRecGop() { return m_bgNewBlockRecGop; }

  Void setbgNewBlockRecoGop(Picture* m) { m_bgNewBlockRecoGop = m; }
  Picture* getbgNewBlockRecoGop() { return m_bgNewBlockRecoGop; }
#endif

#if GENERATE_OrgBG_PIC
  Void setbgNewPicYuvOrgGop(Picture* m) { m_bgNewPicYuvOrgGop = m; }
  Picture* getbgNewPicYuvOrgGop() { return m_bgNewPicYuvOrgGop; }
#endif

#if GENERATE_BG_PIC
  Void setbgNewPicYuvRecGop(Picture* m) { m_bgNewPicYuvRecGop = m; }
  Picture* getbgNewPicYuvRecGop() { return m_bgNewPicYuvRecGop; }
#endif

#if GENERATE_RESI_PIC
  Void setbgNewPicYuvResiGop(Picture* m) { m_bgNewPicYuvResiGop = m; }
  Picture* getbgNewPicYuvResiGop() { return m_bgNewPicYuvResiGop; }
#endif

#if GENERATE_UPDATE_RESI_PIC
  Void setbgNewPicYuvUpdateResiGop(Picture* m) { m_bgNewPicYuvUpdateResiGop = m; }
  Picture* getbgNewPicYuvUpdateResiGop() { return m_bgNewPicYuvUpdateResiGop; }
#endif

#if GENERATE_TEMPRECO_PIC
  Void setbgNewPicYuvRecoGop(Picture* m) { m_bgNewPicYuvRecoGop = m; }
  Picture* getbgNewPicYuvRecoGop() { return m_bgNewPicYuvRecoGop; }
#endif

#if GENERATE_RECO_PIC
  Void setbgNewPicYuvTempUpdateRecoGop(Picture* m) { m_bgNewPicYuvTempUpdateRecoGop = m; }
  Picture* getbgNewPicYuvTempUpdateRecoGop() { return m_bgNewPicYuvTempUpdateRecoGop; }
#endif

#if BG_REFERENCE_SUBSTITUTION
  Void setrcPicYuvTempGop(Picture* m) { m_rcPicYuvTempGop = m; }
  Picture* getrcPicYuvTempGop() { return m_rcPicYuvTempGop; }
#endif

#if HIERARCHY_GENETATE_OrgBGP
  Void CompDiffOrg(Int uiW, Int uiH, Picture* pcPic, Int level, Bool divflag
#if PRINT_OrgDIFF
	  , ofstream &ocout
#endif
  );
#endif

#if OrgBG_BLOCK_SUBSTITUTION
  Double CompNablaOrg(Int compId, UInt uiH, UInt uiW, Picture* pcPic);
  Double CompNabla(Int compId, UInt uiH, UInt uiW, Picture* pcPic);
  Double CompBlockDiff(UInt uiH, UInt uiW, Picture* pcPicYuv, Picture* pcPic);
#endif
  
#if HIERARCHY_GENETATE_BGP
  Void CompDiff(Int uiW, Int uiH, Picture* pcPic, Int level, Bool divflag
#if PRINT_DIFF
	  , ofstream &os
#endif
  );
#endif

#if TRANSFORM_BGP
  Void TransformTempBGP(Int itransId, UInt uiAbsPartIdx, UInt uiHeight, UInt uiWidth, Int qpi);
  Void TransformBGP(Int itransId, UInt uiAbsPartIdx, UInt uiHeight, UInt uiWidth, Int qpi);
  Void TransformUpdateBGP(Int itransId, UInt uiAbsPartIdx, UInt uiHeight, UInt uiWidth, Int qpi
#if PRINT_UPDATEBG_RESI || PRINT_UPDATE_TRCOEFF
	  , ofstream &ocout
#endif
  );
  Void setTrQuant(TComTrQuant* m) { m_pcTrQuantGOP = m; }
#endif

#if ENCODE_BGP
  Void copyCoeff(TCoeff* pcTempCoeff, TCoeff* pcCoeff, UInt uiWidth, UInt uiHeight, UInt uiAbsPartIdx);
#endif

protected:
  RateCtrl* getRateCtrl()       { return m_pcRateCtrl;  }

protected:

  Void  xInitGOP          ( Int iPOCLast, Int iNumPicRcvd, Bool isField );
  Void  xGetBuffer        ( PicList& rcListPic, std::list<PelUnitBuf*>& rcListPicYuvRecOut,
                            Int iNumPicRcvd, Int iTimeOffset, Picture*& rpcPic, Int pocCurr, Bool isField );

  Void  xCalculateAddPSNRs         ( const Bool isField, const Bool isFieldTopFieldFirst, const Int iGOPid, Picture* pcPic, const AccessUnit&accessUnit, PicList &rcListPic, int64_t dEncTime, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE, Double* PSNR_Y );
  Void  xCalculateAddPSNR          ( Picture* pcPic, PelUnitBuf cPicD, const AccessUnit&, Double dEncTime, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE, Double* PSNR_Y );
  Void  xCalculateInterlacedAddPSNR( Picture* pcPicOrgFirstField, Picture* pcPicOrgSecondField,
                                     PelUnitBuf cPicRecFirstField, PelUnitBuf cPicRecSecondField,
                                     const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE, Double* PSNR_Y );

  UInt64 xFindDistortionPlane(const CPelBuf& pic0, const CPelBuf& pic1, const UInt rshift
#if ENABLE_QPA
                            , const UInt chromaShift = 0
#endif
                             );
#if WCG_WPSNR
  Double xFindDistortionPlaneWPSNR(const CPelBuf& pic0, const CPelBuf& pic1, const UInt rshift, const CPelBuf& picLuma0, ComponentID compID, const ChromaFormat chfmt );
#endif
  Double xCalculateRVM();

  Void xUpdateRasInit(Slice* slice);

  Void xWriteAccessUnitDelimiter (AccessUnit &accessUnit, Slice *slice);

  Void xCreateIRAPLeadingSEIMessages (SEIMessages& seiMessages, const SPS *sps, const PPS *pps);
  Void xCreatePerPictureSEIMessages (Int picInGOP, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, Slice *slice);
  Void xCreatePictureTimingSEI  (Int IRAPGOPid, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, SEIMessages& duInfoSeiMessages, Slice *slice, Bool isField, std::deque<DUData> &duData);
  Void xUpdateDuData(AccessUnit &testAU, std::deque<DUData> &duData);
  Void xUpdateTimingSEI(SEIPictureTiming *pictureTimingSEI, std::deque<DUData> &duData, const SPS *sps);
  Void xUpdateDuInfoSEI(SEIMessages &duInfoSeiMessages, SEIPictureTiming *pictureTimingSEI);

  Void xCreateScalableNestingSEI (SEIMessages& seiMessages, SEIMessages& nestedSeiMessages);
  Void xWriteSEI (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, Int temporalId, const SPS *sps);
  Void xWriteSEISeparately (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, Int temporalId, const SPS *sps);
  Void xClearSEIs(SEIMessages& seiMessages, Bool deleteMessages);
  Void xWriteLeadingSEIOrdered (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const SPS *sps, Bool testWrite);
  Void xWriteLeadingSEIMessages  (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const SPS *sps, std::deque<DUData> &duData);
  Void xWriteTrailingSEIMessages (SEIMessages& seiMessages, AccessUnit &accessUnit, Int temporalId, const SPS *sps);
  Void xWriteDuSEIMessages       (SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const SPS *sps, std::deque<DUData> &duData);

#if HEVC_VPS
  Int xWriteVPS (AccessUnit &accessUnit, const VPS *vps);
#endif
  Int xWriteSPS (AccessUnit &accessUnit, const SPS *sps);
  Int xWritePPS (AccessUnit &accessUnit, const PPS *pps);
  Int xWriteParameterSets (AccessUnit &accessUnit, Slice *slice, const Bool bSeqFirst);

  Void applyDeblockingFilterMetric( Picture* pcPic, UInt uiNumSlices );
#if W0038_DB_OPT
  Void applyDeblockingFilterParameterSelection( Picture* pcPic, const UInt numSlices, const Int gopID );
#endif
};// END CLASS DEFINITION EncGOP

//! \}

#endif // __ENCGOP__

