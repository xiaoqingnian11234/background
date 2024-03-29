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

/** \file     DecLib.h
    \brief    decoder class (header)
*/

#ifndef __DECLIB__
#define __DECLIB__

#include "DecSlice.h"
#include "CABACReader.h"
#include "VLCReader.h"
#include "SEIread.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/SEI.h"
#include "CommonLib/Unit.h"

class InputNALUnit;

//! \ingroup DecoderLib
//! \{

bool tryDecodePicture( Picture* pcPic, const int expectedPoc, const std::string& bitstreamFileName, bool bDecodeUntilPocFound = false );
// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// decoder class
class DecLib
{
private:

#if GENERATE_BG_PIC
	Picture* bg_NewPicYuvRec;
#endif

#if BLOCK_GEN
	Picture* bg_NewBlocksRec;
	Int BgBlock[12000] = { 0 };
	Int BgBlock1[12000] = { 0 };
	Bool isdecode = false;
	Bool afterdebg = false;
	Int bgpoc = 0;
	Int bgQp = 0;
	Bool a = true;
	Bool isBgBlock = false;
#endif

#if GENERATE_RESI_PIC
	//Picture* bg_NewPicYuvResi;
#endif

#if GENERATE_UPDATE_RESI_PIC
	Picture* bg_NewPicYuvUpdateResi;
#endif

#if GENERATE_TEMPRECO_PIC
	Picture* bg_NewPicYuvReco;
#endif

#if BG_REFERENCE_SUBSTITUTION
	Picture* m_PicYuvTemp;
#endif

#if BG_BLOCK_SUBSTITUTION
	Double*            m_pcNablaY;
	Double*            m_pcNablaCb;
	Double*            m_pcNablaCr;
#endif

  Int                     m_iMaxRefPicNum;

  NalUnitType             m_associatedIRAPType; ///< NAL unit type of the associated IRAP picture
  Int                     m_pocCRA;            ///< POC number of the latest CRA picture
  Int                     m_pocRandomAccess;   ///< POC number of the random access point (the first IDR or CRA picture)
  Int                     m_lastRasPoc;

  PicList                 m_cListPic;         //  Dynamic buffer
  ParameterSetManager     m_parameterSetManager;  // storage for parameter sets
  Slice*                  m_apcSlicePilot;


  SEIMessages             m_SEIs; ///< List of SEI messages that have been received before the first slice and between slices, excluding prefix SEIs...

  // functional classes
  IntraPrediction         m_cIntraPred;
  InterPrediction         m_cInterPred;
  TrQuant                 m_cTrQuant;
  DecSlice                m_cSliceDecoder;
  DecCu                   m_cCuDecoder;
  HLSyntaxReader          m_HLSReader;
  CABACDecoder            m_CABACDecoder;
  SEIReader               m_seiReader;
  LoopFilter              m_cLoopFilter;
  SampleAdaptiveOffset    m_cSAO;
  // decoder side RD cost computation
  RdCost                  m_cRdCost;                      ///< RD cost computation class

  Bool isSkipPictureForBLA(Int& iPOCLastDisplay);
  Bool isRandomAccessSkipPicture(Int& iSkipFrame,  Int& iPOCLastDisplay);
  Picture*                m_pcPic;



  UInt                    m_uiSliceSegmentIdx;
  Int                     m_prevPOC;
  Int                     m_prevTid0POC;
  Bool                    m_bFirstSliceInPicture;
  Bool                    m_bFirstSliceInSequence;
  Bool                    m_prevSliceSkipped;
  Int                     m_skippedPOC;
  Bool                    m_bFirstSliceInBitstream;
  Int                     m_lastPOCNoOutputPriorPics;
  Bool                    m_isNoOutputPriorPics;
  Bool                    m_craNoRaslOutputFlag;    //value of variable NoRaslOutputFlag of the last CRA pic
  std::ostream           *m_pDecodedSEIOutputStream;

  Int                     m_decodedPictureHashSEIEnabled;  ///< Checksum(3)/CRC(2)/MD5(1)/disable(0) acting on decoded picture hash SEI message
  UInt                    m_numberOfChecksumErrorsDetected;

  Bool                    m_warningMessageSkipPicture;

  std::list<InputNALUnit*> m_prefixSEINALUs; /// Buffered up prefix SEI NAL Units.
public:
  DecLib();
  virtual ~DecLib();

  Void  create  ();
  Void  destroy ();

  Void  setDecodedPictureHashSEIEnabled(Int enabled) { m_decodedPictureHashSEIEnabled=enabled; }

  Void  init();
  Bool  decode(InputNALUnit& nalu, Int& iSkipFrame, Int& iPOCLastDisplay
#if ENCODE_BGPIC
	  ,Bool& isO
#endif // ENCODE_BGPIC
  );
  Void  deletePicBuffer();

  Void  executeLoopFilters();
  Void  finishPicture(Int& poc, PicList*& rpcListPic, MsgLevel msgl = INFO);
  Void  finishPictureLight(Int& poc, PicList*& rpcListPic );
  Void  checkNoOutputPriorPics (PicList* rpcListPic);

  Bool  getNoOutputPriorPicsFlag () const   { return m_isNoOutputPriorPics; }
  Void  setNoOutputPriorPicsFlag (Bool val) { m_isNoOutputPriorPics = val; }
  Void  setFirstSliceInPicture (bool val)  { m_bFirstSliceInPicture = val; }
  Bool  getFirstSliceInSequence () const   { return m_bFirstSliceInSequence; }
  Void  setFirstSliceInSequence (bool val) { m_bFirstSliceInSequence = val; }
  Void  setDecodedSEIMessageOutputStream(std::ostream *pOpStream) { m_pDecodedSEIOutputStream = pOpStream; }
  UInt  getNumberOfChecksumErrorsDetected() const { return m_numberOfChecksumErrorsDetected; }
#if GENERATE_BG_PIC
  Picture* getpcPic() { return m_pcPic; };
#endif
#if GENERATE_BG_PIC
  Void setbgNewPicYuvRec(Picture* m) { bg_NewPicYuvRec = m; }
  Picture* getbgNewPicYuvRec() { return bg_NewPicYuvRec; }
#endif
#if BLOCK_GEN
  Void setbgNewBlocksRec(Picture* m) { bg_NewBlocksRec = m; }
  Picture* getbgNewBlocksRec() { return bg_NewBlocksRec; }
#endif

#if GENERATE_RESI_PIC
  /*Void setbgNewPicYuvResi(Picture* m) { bg_NewPicYuvResi = m; }
  Picture* getbgNewPicYuvResi() { return bg_NewPicYuvResi; }*/
#endif

#if GENERATE_UPDATE_RESI_PIC
  Void setbgNewPicYuvUpdateResi(Picture* m) { bg_NewPicYuvUpdateResi = m; }
  Picture* getbgNewPicYuvUpdateResi() { return bg_NewPicYuvUpdateResi; }
#endif

#if GENERATE_TEMPRECO_PIC
  Void setbgNewPicYuvReco(Picture* m) { bg_NewPicYuvReco = m; }
  Picture* getbgNewPicYuvReco() { return bg_NewPicYuvReco; }
#endif

#if BG_REFERENCE_SUBSTITUTION
  Void setPicYuvTemp(Picture* m) { m_PicYuvTemp = m; }
#endif

#if HIERARCHY_GENETATE_BGP
  Void CompDiff(Int uiW, Int uiH, Picture* pcPic, Int level, Bool divflag);
#endif
#if BG_BLOCK_SUBSTITUTION
  Double CompNabla(Int compId, UInt uiH, UInt uiW, Picture* pcPic);
  Double CompBlockDiff(UInt uiH, UInt uiW, Picture * pcPicYuv, Picture * pcPic);
  
#endif
protected:
  Void  xUpdateRasInit(Slice* slice);

  Picture * xGetNewPicBuffer(const SPS &sps, const PPS &pps, const UInt temporalLayer);
  Void  xCreateLostPicture (Int iLostPOC);

  Void      xActivateParameterSets();
  Bool      xDecodeSlice(InputNALUnit &nalu, Int &iSkipFrame, Int iPOCLastDisplay
#if ENCODE_BGPIC
  ,Bool& isO
#endif
  );
#if HEVC_VPS
  Void      xDecodeVPS( InputNALUnit& nalu );
#endif
  Void      xDecodeSPS( InputNALUnit& nalu );
  Void      xDecodePPS( InputNALUnit& nalu );
  Void      xUpdatePreviousTid0POC( Slice *pSlice ) { if ((pSlice->getTLayer()==0) && (pSlice->isReferenceNalu() && (pSlice->getNalUnitType()!=NAL_UNIT_CODED_SLICE_RASL_R)&& (pSlice->getNalUnitType()!=NAL_UNIT_CODED_SLICE_RADL_R))) { m_prevTid0POC=pSlice->getPOC(); } }
  Void      xParsePrefixSEImessages();
  Void      xParsePrefixSEIsForUnknownVCLNal();

};// END CLASS DEFINITION DecLib


//! \}

#endif // __DECTOP__

