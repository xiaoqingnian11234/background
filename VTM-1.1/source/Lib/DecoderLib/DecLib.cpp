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

/** \file     DecLib.cpp
    \brief    decoder class
*/

#include "NALread.h"
#include "DecLib.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/Buffer.h"
#include "CommonLib/UnitTools.h"

#include <fstream>
#include <stdio.h>
#include <fcntl.h>
#include "AnnexBread.h"
#include "NALread.h"

bool tryDecodePicture( Picture* pcEncPic, const int expectedPoc, const std::string& bitstreamFileName, bool bDecodeUntilPocFound /* = false */ )
{
  int      poc;
  PicList* pcListPic = NULL;

  static bool bFirstCall      = true;             /* TODO: MT */
  static bool loopFiltered    = false;            /* TODO: MT */
  static int  iPOCLastDisplay = -MAX_INT;         /* TODO: MT */

  static std::ifstream* bitstreamFile = nullptr;  /* TODO: MT */
  static InputByteStream* bytestream  = nullptr;  /* TODO: MT */
  bool bRet = false;

  // create & initialize internal classes
  static DecLib *pcDecLib = nullptr;              /* TODO: MT */

  if( pcEncPic )
  {
    if( bFirstCall )
    {
      bitstreamFile = new std::ifstream( bitstreamFileName.c_str(), std::ifstream::in | std::ifstream::binary );
      bytestream    = new InputByteStream( *bitstreamFile );

      CHECK( !*bitstreamFile, "failed to open bitstream file " << bitstreamFileName.c_str() << " for reading" ) ;
      // create decoder class
      pcDecLib = new DecLib;
      pcDecLib->create();

      // initialize decoder class
      pcDecLib->init();

      pcDecLib->setDecodedPictureHashSEIEnabled( true );

      bFirstCall = false;
      msg( INFO, "start to decode %s \n", bitstreamFileName.c_str() );
    }

    bool goOn = true;

    // main decoder loop
    while( !!*bitstreamFile && goOn )
    {
      /* location serves to work around a design fault in the decoder, whereby
       * the process of reading a new slice that is the first slice of a new frame
       * requires the DecApp::decode() method to be called again with the same
       * nal unit. */
      std::streampos location = bitstreamFile->tellg();
      AnnexBStats stats       = AnnexBStats();

      InputNALUnit nalu;
      byteStreamNALUnit( *bytestream, nalu.getBitstream().getFifo(), stats );

      // call actual decoding function
      Bool bNewPicture = false;
      if( nalu.getBitstream().getFifo().empty() )
      {
        /* this can happen if the following occur:
         *  - empty input file
         *  - two back-to-back start_code_prefixes
         *  - start_code_prefix immediately followed by EOF
         */
        msg( ERROR, "Warning: Attempt to decode an empty NAL unit\n");
      }
      else
      {
        read( nalu );
        int iSkipFrame = 0;
		Bool b = false;
        bNewPicture = pcDecLib->decode( nalu, iSkipFrame, iPOCLastDisplay 
#if ENCODE_BGPIC
			,b
#endif
		);
        if( bNewPicture )
        {
          bitstreamFile->clear();
          /* location points to the current nalunit payload[1] due to the
            * need for the annexB parser to read three extra bytes.
            * [1] except for the first NAL unit in the file
            *     (but bNewPicture doesn't happen then) */
          bitstreamFile->seekg( location - std::streamoff( 3 ) );
          bytestream->reset();
        }
      }

      if( ( bNewPicture || !*bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) && !pcDecLib->getFirstSliceInSequence() )
      {
        if( !loopFiltered || *bitstreamFile )
        {
          pcDecLib->finishPictureLight( poc, pcListPic );

          if( pcListPic )
          {
            for( auto & pic : *pcListPic )
            {
              if( pic->poc == poc && (!bDecodeUntilPocFound || expectedPoc == poc ) )
              {
                CHECK( pcEncPic->slices.size() == 0, "at least one slice should be available" );

                CHECK( expectedPoc != poc, "mismatch in POC - check encoder configuration" );

                for( int i = 0; i < pic->slices.size(); i++ )
                {
                  if( pcEncPic->slices.size() <= i )
                  {
                    pcEncPic->slices.push_back( new Slice );
                    pcEncPic->slices.back()->initSlice();
                  }
                  pcEncPic->slices[i]->copySliceInfo( pic->slices[i], false );
                }

                pcEncPic->cs->slice = pcEncPic->slices.back();

                if ( pic->cs->sps->getUseSAO() )
                {
                  pcEncPic->copySAO( *pic, 0 );
                }

                pcDecLib->executeLoopFilters();
                if ( pic->cs->sps->getUseSAO() )
                {
                  pcEncPic->copySAO( *pic, 1 );
                }

                pcEncPic->cs->copyStructure( *pic->cs, CH_L, true, true );

                if( CS::isDualITree( *pcEncPic->cs ) )
                {
                  pcEncPic->cs->copyStructure( *pic->cs, CH_C, true, true );
                }
                goOn = false; // exit the loop return
                bRet = true;
                break;
              }
            }
          }
          // postpone loop filters
          if (!bRet)
          {
            pcDecLib->executeLoopFilters();
          }
          pcDecLib->finishPicture( poc, pcListPic, DETAILS );

          // write output
          if( ! pcListPic->empty())
          {
            PicList::iterator iterPic   = pcListPic->begin();
            Int numPicsNotYetDisplayed = 0;
            Int dpbFullness = 0;
            const SPS* activeSPS = (pcListPic->front()->cs->sps);
            UInt maxNrSublayers = activeSPS->getMaxTLayers();
            UInt numReorderPicsHighestTid = activeSPS->getNumReorderPics(maxNrSublayers-1);
            UInt maxDecPicBufferingHighestTid =  activeSPS->getMaxDecPicBuffering(maxNrSublayers-1);

            while (iterPic != pcListPic->end())
            {
              Picture* pcCurPic = *(iterPic);
              if(pcCurPic->neededForOutput && pcCurPic->getPOC() > iPOCLastDisplay)
              {
                numPicsNotYetDisplayed++;
                dpbFullness++;
              }
              else if(pcCurPic->referenced)
              {
                dpbFullness++;
              }
              iterPic++;
            }

            iterPic = pcListPic->begin();

            if (numPicsNotYetDisplayed>2)
            {
              iterPic++;
            }

            Picture* pcCurPic = *(iterPic);
            if( numPicsNotYetDisplayed>2 && pcCurPic->fieldPic ) //Field Decoding
            {
              THROW( "no field coding support ");
            }
            else if( !pcCurPic->fieldPic ) //Frame Decoding
            {
              iterPic = pcListPic->begin();

              while (iterPic != pcListPic->end())
              {
                pcCurPic = *(iterPic);

                if(pcCurPic->neededForOutput && pcCurPic->getPOC() > iPOCLastDisplay &&
                  (numPicsNotYetDisplayed >  numReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid))
                {
                    numPicsNotYetDisplayed--;
                  if( ! pcCurPic->referenced )
                  {
                    dpbFullness--;
                  }
                  // update POC of display order
                  iPOCLastDisplay = pcCurPic->getPOC();

                  // erase non-referenced picture in the reference picture list after display
                  if( ! pcCurPic->referenced && pcCurPic->reconstructed )
                  {
                    pcCurPic->reconstructed = false;
                  }
                  pcCurPic->neededForOutput = false;
                }

                iterPic++;
              }
            }
          }


        }
        loopFiltered = ( nalu.m_nalUnitType == NAL_UNIT_EOS );
        if( nalu.m_nalUnitType == NAL_UNIT_EOS )
        {
          pcDecLib->setFirstSliceInSequence( true );
        }

      }
      else if( ( bNewPicture || !*bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) && pcDecLib->getFirstSliceInSequence() )
      {
        pcDecLib->setFirstSliceInPicture( true );
      }
    }
  }

  if( !bRet )
  {
    CHECK( bDecodeUntilPocFound, " decoding failed - check decodeBitstream2 parameter File: " << bitstreamFileName.c_str() );
    if( pcDecLib )
    {
      pcDecLib->destroy();
      pcDecLib->deletePicBuffer();
      delete pcDecLib;
      pcDecLib = nullptr;
    }
    bFirstCall   = true;
    loopFiltered = false;
    iPOCLastDisplay = -MAX_INT;

    if( bytestream )
    {
      delete bytestream;
      bytestream = nullptr;
    }

    if( bitstreamFile )
    {
      delete bitstreamFile;
      bitstreamFile = nullptr;
    }
  }

  return bRet;
}


//! \ingroup DecoderLib
//! \{

DecLib::DecLib()
  : m_iMaxRefPicNum(0)
  , m_associatedIRAPType(NAL_UNIT_INVALID)
  , m_pocCRA(0)
  , m_pocRandomAccess(MAX_INT)
  , m_lastRasPoc(MAX_INT)
  , m_cListPic()
  , m_parameterSetManager()
  , m_apcSlicePilot(NULL)
  , m_SEIs()
  , m_cIntraPred()
  , m_cInterPred()
  , m_cTrQuant()
  , m_cSliceDecoder()
  , m_cCuDecoder()
  , m_HLSReader()
  , m_seiReader()
  , m_cLoopFilter()
  , m_cSAO()
  , m_pcPic(NULL)
  , m_prevPOC(MAX_INT)
  , m_prevTid0POC(0)
  , m_bFirstSliceInPicture(true)
  , m_bFirstSliceInSequence(true)
  , m_prevSliceSkipped(false)
  , m_skippedPOC(0)
  , m_bFirstSliceInBitstream(true)
  , m_lastPOCNoOutputPriorPics(-1)
  , m_isNoOutputPriorPics(false)
  , m_craNoRaslOutputFlag(false)
  , m_pDecodedSEIOutputStream(NULL)
  , m_decodedPictureHashSEIEnabled(false)
  , m_numberOfChecksumErrorsDetected(0)
  , m_warningMessageSkipPicture(false)
  , m_prefixSEINALUs()
{
#if ENABLE_SIMD_OPT_BUFFER
  g_pelBufOP.initPelBufOpsX86();
#endif
}

DecLib::~DecLib()
{
  while (!m_prefixSEINALUs.empty())
  {
    delete m_prefixSEINALUs.front();
    m_prefixSEINALUs.pop_front();
  }
}

Void DecLib::create()
{
  m_apcSlicePilot = new Slice;
  m_uiSliceSegmentIdx = 0;
}

Void DecLib::destroy()
{
  delete m_apcSlicePilot;
  m_apcSlicePilot = NULL;

  m_cSliceDecoder.destroy();
}

Void DecLib::init()
{
  m_cSliceDecoder.init( &m_CABACDecoder, &m_cCuDecoder );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 1 ) );
}

Void DecLib::deletePicBuffer ( )
{
  PicList::iterator  iterPic   = m_cListPic.begin();
  Int iSize = Int( m_cListPic.size() );

  for (Int i = 0; i < iSize; i++ )
  {
    Picture* pcPic = *(iterPic++);
    pcPic->destroy();

    delete pcPic;
    pcPic = NULL;
  }
  m_cSAO.destroy();
  m_cLoopFilter.destroy();
}

Picture* DecLib::xGetNewPicBuffer ( const SPS &sps, const PPS &pps, const UInt temporalLayer )
{
  Picture * pcPic = nullptr;
  m_iMaxRefPicNum = sps.getMaxDecPicBuffering(temporalLayer);     // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
  if (m_cListPic.size() < (UInt)m_iMaxRefPicNum)
  {
    pcPic = new Picture();

    pcPic->create( sps.getChromaFormatIdc(), Size( sps.getPicWidthInLumaSamples(), sps.getPicHeightInLumaSamples() ), sps.getMaxCUWidth(), sps.getMaxCUWidth() + 16, true );
    m_cListPic.push_back( pcPic );

    return pcPic;
  }

  Bool bBufferIsAvailable = false;
  for(auto * p: m_cListPic)
  {
    pcPic = p;  // workaround because range-based for-loops don't work with existing variables
    if ( pcPic->reconstructed == false && ! pcPic->neededForOutput )
    {
      pcPic->neededForOutput = false;
      bBufferIsAvailable = true;
      break;
    }

    if( ! pcPic->referenced  && ! pcPic->neededForOutput )
    {
      pcPic->neededForOutput = false;
      pcPic->reconstructed = false;
      bBufferIsAvailable = true;
      break;
    }
  }

  if( ! bBufferIsAvailable )
  {
    //There is no room for this picture, either because of faulty encoder or dropped NAL. Extend the buffer.
    m_iMaxRefPicNum++;

    pcPic = new Picture();

    m_cListPic.push_back( pcPic );

    pcPic->create( sps.getChromaFormatIdc(), Size( sps.getPicWidthInLumaSamples(), sps.getPicHeightInLumaSamples() ), sps.getMaxCUWidth(), sps.getMaxCUWidth() + 16, true );
  }
  else
  {
    if( !pcPic->Y().Size::operator==( Size( sps.getPicWidthInLumaSamples(), sps.getPicHeightInLumaSamples() ) ) || pcPic->cs->pcv->maxCUWidth != sps.getMaxCUWidth() || pcPic->cs->pcv->maxCUHeight != sps.getMaxCUHeight() )
    {
      pcPic->destroy();
      pcPic->create( sps.getChromaFormatIdc(), Size( sps.getPicWidthInLumaSamples(), sps.getPicHeightInLumaSamples() ), sps.getMaxCUWidth(), sps.getMaxCUWidth() + 16, true );
    }
  }

  pcPic->setBorderExtension( false );
  pcPic->neededForOutput = false;
  pcPic->reconstructed = false;

  return pcPic;
}

#if HIERARCHY_GENETATE_BGP

Void DecLib::CompDiff(Int uiWidth, Int uiHeight, Picture* pcPic, Int lev, Bool divflag)
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


		switch (level)
		{

		case 0:
			if (diff < 4)
			{

				pcPic->Copy2BackPic(bg_NewPicYuvRec, uiW, uiH, pcPic, level);

				divflag = false;
			}
			else if (diff >= 4 && diff < 10)
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
			if (diff < 3)
			{

				pcPic->Copy2BackPic(bg_NewPicYuvRec, uiW, uiH, pcPic, level);
				divflag = false;
			}
			else if (diff >= 3 && diff < 6)
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
			if (diff < 3)
			{
				pcPic->Copy2BackPic(bg_NewPicYuvRec, uiW, uiH, pcPic, level);
				divflag = false;
			}
			else if (diff >= 3 && diff < 5)
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
			if (diff < 3)
			{

				pcPic->Copy2BackPic(bg_NewPicYuvRec, uiW, uiH, pcPic, level);
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
#if BG_BLOCK_SUBSTITUTION

Double DecLib::CompNabla(Int compId, UInt uiH, UInt uiW, Picture* pcPic)
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
Double DecLib::CompBlockDiff(UInt uiH, UInt uiW, Picture* pcPicYuv, Picture* pcPic)
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

Void DecLib::executeLoopFilters()
{

#if BG_BLOCK_SUBSTITUTION
	
	m_pcNablaY = NULL;
	m_pcNablaCb = NULL;
	m_pcNablaCr = NULL;
	m_pcNablaY = new Double[BLOCK_LEN * BLOCK_LEN];
	m_pcNablaCb = new Double[(BLOCK_LEN >> 1) * (BLOCK_LEN >> 1)];
	m_pcNablaCr = new Double[(BLOCK_LEN >> 1) * (BLOCK_LEN >> 1)];
	memset(m_pcNablaY, 0, BLOCK_LEN * BLOCK_LEN * sizeof(Double));
	memset(m_pcNablaCb, 0, (BLOCK_LEN >> 1) * (BLOCK_LEN >> 1) * sizeof(Double));
	memset(m_pcNablaCr, 0, (BLOCK_LEN >> 1) * (BLOCK_LEN >> 1) * sizeof(Double));
#endif
  if( !m_pcPic )
  {
    return; // nothing to deblock
  }

  CodingStructure& cs = *m_pcPic->cs;
  // deblocking filter
  m_cLoopFilter.loopFilterPic( cs );
  
  if( cs.sps->getUseSAO() )
  {
    m_cSAO.SAOProcess( cs, cs.picture->getSAO() );
  }
#if BLOCK_ENCODE
  /*int num_block = 0;
  for (UInt uiH = 0; uiH < m_pcPic->getRecoBuf().Y().height; uiH += BLOCK_GEN_LEN)
  {
	  for (UInt uiW = 0; uiW < m_pcPic->getRecoBuf().Y().width; uiW += BLOCK_GEN_LEN)
	  {
		  if (BgBlock[num_block] ==1000 )
		  {
			  BgBlock[num_block] = 2000;
			  //isdecode = true;
			  afterdebg = true;
		  }
		  num_block++;
	  }
  }
  if (isBgBlock)
  {
	  cout << "*******************" << endl;
	  bg_NewPicYuvRec->DeleteReco(bg_NewPicYuvRec);
	  isBgBlock = false;
	  int num_block = 0;
	  for (UInt uiH = 0; uiH < m_pcPic->getRecoBuf().Y().height; uiH += BLOCK_GEN_LEN)
	  {
		  for (UInt uiW = 0; uiW < m_pcPic->getRecoBuf().Y().width; uiW += BLOCK_GEN_LEN)
		  {
			  if (!BgBlock[num_block] && !m_pcPic->CompBlockRecoIsSimilar(uiW, uiH, m_pcPic, bg_NewPicYuvRec))
			  {
				  BgBlock[num_block] = 1000;
				  isdecode = true;
				  //afterdebg = true;
				  //m_pcPic->CopyReco2Block(bg_NewPicYuvReco, uiW, uiH, m_pcPic);
				  
			  }
			  num_block++;
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
  */
  m_pcPic->CopyPic2Reco(bg_NewPicYuvRec,m_pcPic); //
#endif
 

#if BLOCK_ENCODE
  if (isdecode) //此帧为背景块帧
  {
	  /*cout << "vector" << endl;
	  vector<int> bg = m_pcPic->getBgblock();
	  for (int i = 0; i < bg.size(); i++)
	  {
		  cout << bg[i] << " ";
	  }
	  cout << "vector" << endl;*/

	  isdecode = false;
	  //afterdebg = true; //已经有解码块了
	  a = false;
	  //将m_pcpic的解码块
	  Int num_block = 0;
	  for (Int i = 0; i < m_pcPic->getRecoBuf().Y().height; i += BLOCK_GEN_LEN)
	  {
		  for (Int j = 0; j < m_pcPic->getRecoBuf().Y().width; j += BLOCK_GEN_LEN)
		  {
			  if (BgBlock[num_block] == 1000) //将上一帧的块加入reco
			  {
				  m_pcPic->CopyReco2Block(bg_NewPicYuvReco, j, i, m_pcPic);
				  //BgBlock[num_block] = 2000;
			  }
			  num_block++;
		  }
	  }
  }
#endif
  //if (m_pcPic->referenced) //是正常帧，不是编码块帧
  /*{
#if HIERARCHY_GENETATE_BGP
	  if (m_pcPic->getPOC() != 0)
	  {
#if israndom
		  if (pcPic->getPOC() % 32 != 0)
		  {
#endif // israndom

			  const Int  iWidth = m_pcPic->getRecoBuf().Y().width;
			  const Int  iHeight = m_pcPic->getRecoBuf().Y().height;

			  for (Int i = 0; i < iHeight; i += UNIT_LEN)
			  {
				  for (Int j = 0; j < iWidth; j += UNIT_LEN)
				  {
					  Int level = 0;
					  Bool divflag = false;

					  CompDiff(j, i, m_pcPic, level, divflag);
				  }
			  }
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
				  piBac = bg_NewPicYuvRec->getRecoBuf().Y().bufAt(0, 0);
				  piOrg = m_pcPic->getRecoBuf().Y().bufAt(0, 0);

				  uiStride = m_pcPic->getRecoBuf().Y().stride;
				  uiPicHeight = m_pcPic->getRecoBuf().Y().height;
				  uiPicWidth = m_pcPic->getRecoBuf().Y().width;
				  break;
			  case 1:
				  piBac = bg_NewPicYuvRec->getRecoBuf().Cb().bufAt(0, 0);
				  piOrg = m_pcPic->getRecoBuf().Cb().bufAt(0, 0);

				  uiStride = m_pcPic->getRecoBuf().Cb().stride;
				  uiPicHeight = m_pcPic->getRecoBuf().Cb().height;
				  uiPicWidth = m_pcPic->getRecoBuf().Cb().width;
				  break;
			  case 2:
				  piBac = bg_NewPicYuvRec->getRecoBuf().Cr().bufAt(0, 0);
				  piOrg = m_pcPic->getRecoBuf().Cr().bufAt(0, 0);

				  uiStride = m_pcPic->getRecoBuf().Cr().stride;
				  uiPicHeight = m_pcPic->getRecoBuf().Cr().height;
				  uiPicWidth = m_pcPic->getRecoBuf().Cr().width;
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
	  }
#endif

#if BG_BLOCK_SUBSTITUTION
	  if (m_pcPic->getPOC() != 0)
	  {
#if israndom
		  if (pcPic->getPOC() % 32 != 0)
		  {

#endif // israndom

			  const UInt  uiWidth = m_pcPic->getRecoBuf().Y().width;
			  const UInt  uiHeight = m_pcPic->getRecoBuf().Y().height;
			  for (UInt uiH = 0; uiH < uiHeight; uiH += BLOCK_LEN)
			  {
				  for (UInt uiW = 0; uiW < uiWidth; uiW += BLOCK_LEN)
				  {
					  Double Fc[3];
					  for (Int transId = 0; transId < 3; transId++)
					  {
						  Fc[transId] = CompNabla(transId, uiH, uiW, m_pcPic);
					  }
					  Double dNabla = Fc[0] + Fc[1] / 4 + Fc[2] / 4;

					  if (dNabla < 1.5)   // if Nabla < Omegaba then substitute
					  {
						  m_pcPic->Copy2BackPic(bg_NewPicYuvRec, uiW, uiH, m_pcPic);
					  }
					  else if (dNabla >= 1.5 && dNabla < 3.5 && uiH != 0 && uiH != (uiHeight - BLOCK_LEN) && uiW != 0 && uiW != (uiWidth - BLOCK_LEN))  // Omegaba < Nabla < Omega
					  {
						  Double DiffOrg = 0;
						  Double DiffBG = 0;

						  DiffOrg += CompBlockDiff(uiH, uiW, m_pcPic, m_pcPic); //maybe
						  DiffBG += CompBlockDiff(uiH, uiW, bg_NewPicYuvRec, m_pcPic);

						  if (DiffOrg < DiffBG)
						  {
							  m_pcPic->Copy2BackPic(bg_NewPicYuvRec, uiW, uiH, m_pcPic);
						  }
					  }
				  }
			  }
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
				  piBac = bg_NewPicYuvRec->getRecoBuf().Y().bufAt(0, 0);
				  piOrg = m_pcPic->getRecoBuf().Y().bufAt(0, 0);

				  uiStride = m_pcPic->getRecoBuf().Y().stride;
				  uiPicHeight = m_pcPic->getRecoBuf().Y().height;
				  uiPicWidth = m_pcPic->getRecoBuf().Y().width;
				  break;
			  case 1:
				  piBac = bg_NewPicYuvRec->getRecoBuf().Cb().bufAt(0, 0);
				  piOrg = m_pcPic->getRecoBuf().Cb().bufAt(0, 0);

				  uiStride = m_pcPic->getRecoBuf().Cb().stride;
				  uiPicHeight = m_pcPic->getRecoBuf().Cb().height;
				  uiPicWidth = m_pcPic->getRecoBuf().Cb().width;
				  break;
			  case 2:
				  piBac = bg_NewPicYuvRec->getRecoBuf().Cr().bufAt(0, 0);
				  piOrg = m_pcPic->getRecoBuf().Cr().bufAt(0, 0);

				  uiStride = m_pcPic->getRecoBuf().Cr().stride;
				  uiPicHeight = m_pcPic->getRecoBuf().Cr().height;
				  uiPicWidth = m_pcPic->getRecoBuf().Cr().width;
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
	  }
#endif
#if BLOCK_GEN
	  if (m_pcPic->getPOC() > 5 && m_pcPic->getPOC() < 300)
	  {
		  int num_block = 0;
		  for (UInt uiH = 0; uiH < m_pcPic->getRecoBuf().Y().height; uiH += BLOCK_GEN_LEN)
		  {
			  for (UInt uiW = 0; uiW < m_pcPic->getRecoBuf().Y().width; uiW += BLOCK_GEN_LEN)
			  {

				  if (BgBlock[num_block] != 0)
				  {
					  BgBlock[num_block]++;
				  }
				  else
				  {
					  double diff = 0;

					  //pcPic->CompBlockDiff(uiW, uiH, pcPic, diff);  //get  block diff
					  m_pcPic->CompBlockPicRecoDiff(uiW, uiH, m_pcPic, bg_NewPicYuvRec, diff);//计算Rec与PicRec的diff

					  //cout << "diff" << diff;
					  if (diff < 4)
					  {
						  BgBlock[num_block]++;
						  isdecode = true; //确定编
						  //m_pcPic->CopyReco2Block(bg_NewBlocksRec, uiW, uiH, bg_NewPicYuvRec);
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
			  while (i < (m_pcPic->getRecoBuf().Y().width / BLOCK_GEN_LEN)*(m_pcPic->getRecoBuf().Y().height / BLOCK_GEN_LEN))
			  {
				  //cout << BgBlock[i] << " ";
				  i++;
				  if (BgBlock[i] != 0)
					  j++;

			  }
			  cout << endl;
			  cout << " j " << j << endl;
		  }
	  }
#endif
  }*/
}

Void DecLib::finishPictureLight(Int& poc, PicList*& rpcListPic )
{
  Slice*  pcSlice = m_pcPic->cs->slice;

  m_pcPic->neededForOutput = (pcSlice->getPicOutputFlag() ? true : false);
  m_pcPic->reconstructed = true;

  Slice::sortPicList( m_cListPic ); // sorting for application output
  poc                 = pcSlice->getPOC();
  rpcListPic          = &m_cListPic;
#if OrgBG_BLOCK_SUBSTITUTION
  /*delete[] m_pcNablaY;
  delete[] m_pcNablaCb;
  delete[] m_pcNablaCr;*/
#endif
}

Void DecLib::finishPicture(Int& poc, PicList*& rpcListPic, MsgLevel msgl )
{
  Slice*  pcSlice = m_pcPic->cs->slice;

  TChar c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');
  if (!m_pcPic->referenced)
  {
    c += 32;  // tolower
  }

  //-- For time output for each slice
  msg( msgl, "POC %4d TId: %1d ( %c-SLICE, QP%3d ) ", pcSlice->getPOC(),
         pcSlice->getTLayer(),
         c,
         pcSlice->getSliceQp() );
  msg( msgl, "[DT %6.3f] ", pcSlice->getProcessingTime() );

  for (Int iRefList = 0; iRefList < 2; iRefList++)
  {
    msg( msgl, "[L%d ", iRefList);
    for (Int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx(RefPicList(iRefList)); iRefIndex++)
    {
      msg( msgl, "%d ", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex));
    }
    msg( msgl, "] ");
  }
  if (m_decodedPictureHashSEIEnabled)
  {
    SEIMessages pictureHashes = getSeisByType(m_pcPic->SEIs, SEI::DECODED_PICTURE_HASH );
    const SEIDecodedPictureHash *hash = ( pictureHashes.size() > 0 ) ? (SEIDecodedPictureHash*) *(pictureHashes.begin()) : NULL;
    if (pictureHashes.size() > 1)
    {
      msg( WARNING, "Warning: Got multiple decoded picture hash SEI messages. Using first.");
    }
    m_numberOfChecksumErrorsDetected += calcAndPrintHashStatus(((const Picture*) m_pcPic)->getRecoBuf(), hash, pcSlice->getSPS()->getBitDepths(), msgl);
  }

  msg( msgl, "\n");

  m_pcPic->neededForOutput = (pcSlice->getPicOutputFlag() ? true : false);
  m_pcPic->reconstructed = true;


  Slice::sortPicList( m_cListPic ); // sorting for application output
  poc                 = pcSlice->getPOC();
  rpcListPic          = &m_cListPic;
  m_bFirstSliceInPicture  = true; // TODO: immer true? hier ist irgendwas faul

  m_pcPic->destroyTempBuffers();
  m_pcPic->cs->destroyCoeffs();
  m_pcPic->cs->releaseIntermediateData();
}

Void DecLib::checkNoOutputPriorPics (PicList* pcListPic)
{
  if (!pcListPic || !m_isNoOutputPriorPics)
  {
    return;
  }

  PicList::iterator  iterPic   = pcListPic->begin();

  while (iterPic != pcListPic->end())
  {
    Picture* pcPicTmp = *(iterPic++);
    if (m_lastPOCNoOutputPriorPics != pcPicTmp->getPOC())
    {
      pcPicTmp->neededForOutput = false;
    }
  }
}

Void DecLib::xUpdateRasInit(Slice* slice)
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

Void DecLib::xCreateLostPicture(Int iLostPoc)
{
  msg( INFO, "\ninserting lost poc : %d\n",iLostPoc);
  Picture *cFillPic = xGetNewPicBuffer(*(m_parameterSetManager.getFirstSPS()), *(m_parameterSetManager.getFirstPPS()), 0);

  CHECK( !cFillPic->slices.size(), "No slices in picture" );

  cFillPic->slices[0]->initSlice();

  PicList::iterator iterPic = m_cListPic.begin();
  Int closestPoc = 1000000;
  while ( iterPic != m_cListPic.end())
  {
    Picture * rpcPic = *(iterPic++);
    if(abs(rpcPic->getPOC() -iLostPoc)<closestPoc&&abs(rpcPic->getPOC() -iLostPoc)!=0&&rpcPic->getPOC()!=m_apcSlicePilot->getPOC())
    {
      closestPoc=abs(rpcPic->getPOC() -iLostPoc);
    }
  }
  iterPic = m_cListPic.begin();
  while ( iterPic != m_cListPic.end())
  {
    Picture *rpcPic = *(iterPic++);
    if(abs(rpcPic->getPOC() -iLostPoc)==closestPoc&&rpcPic->getPOC()!=m_apcSlicePilot->getPOC())
    {
      msg( INFO, "copying picture %d to %d (%d)\n",rpcPic->getPOC() ,iLostPoc,m_apcSlicePilot->getPOC());
      cFillPic->getRecoBuf().copyFrom( rpcPic->getRecoBuf() );
      break;
    }
  }

//  for(Int ctuRsAddr=0; ctuRsAddr<cFillPic->getNumberOfCtusInFrame(); ctuRsAddr++)  { cFillPic->getCtu(ctuRsAddr)->initCtu(cFillPic, ctuRsAddr); }
  cFillPic->referenced = true;
  cFillPic->slices[0]->setPOC(iLostPoc);
  xUpdatePreviousTid0POC(cFillPic->slices[0]);
  cFillPic->reconstructed = true;
  cFillPic->neededForOutput = true;
  if(m_pocRandomAccess == MAX_INT)
  {
    m_pocRandomAccess = iLostPoc;
  }

}


Void DecLib::xActivateParameterSets()
{
  if (m_bFirstSliceInPicture)
  {
    const PPS *pps = m_parameterSetManager.getPPS(m_apcSlicePilot->getPPSId()); // this is a temporary PPS object. Do not store this value
    CHECK(pps == 0, "No PPS present");

    const SPS *sps = m_parameterSetManager.getSPS(pps->getSPSId());             // this is a temporary SPS object. Do not store this value
    CHECK(sps == 0, "No SPS present");

    if (NULL == pps->pcv)
    {
      m_parameterSetManager.getPPS( m_apcSlicePilot->getPPSId() )->pcv = new PreCalcValues( *sps, *pps, false );
    }
    m_parameterSetManager.clearSPSChangedFlag(sps->getSPSId());
    m_parameterSetManager.clearPPSChangedFlag(pps->getPPSId());

    if (false == m_parameterSetManager.activatePPS(m_apcSlicePilot->getPPSId(),m_apcSlicePilot->isIRAP()))
    {
      THROW("Parameter set activation failed!");
    }

    xParsePrefixSEImessages();

#if RExt__HIGH_BIT_DEPTH_SUPPORT==0
    if (sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag() || sps->getBitDepth(CHANNEL_TYPE_LUMA)>12 || sps->getBitDepth(CHANNEL_TYPE_CHROMA)>12 )
    {
      THROW("High bit depth support must be enabled at compile-time in order to decode this bitstream\n");
    }
#endif

    //  Get a new picture buffer. This will also set up m_pcPic, and therefore give us a SPS and PPS pointer that we can use.
    m_pcPic = xGetNewPicBuffer (*sps, *pps, m_apcSlicePilot->getTLayer());

    m_apcSlicePilot->applyReferencePictureSet(m_cListPic, m_apcSlicePilot->getRPS());

    m_pcPic->finalInit( *sps, *pps );

    m_pcPic->createTempBuffers( m_pcPic->cs->pps->pcv->maxCUWidth );
    m_pcPic->cs->createCoeffs();

    m_pcPic->allocateNewSlice();
    // make the slice-pilot a real slice, and set up the slice-pilot for the next slice
    CHECK(m_pcPic->slices.size() != (m_uiSliceSegmentIdx + 1), "Invalid number of slices");
    m_apcSlicePilot = m_pcPic->swapSliceObject(m_apcSlicePilot, m_uiSliceSegmentIdx);

    // we now have a real slice:
    Slice *pSlice = m_pcPic->slices[m_uiSliceSegmentIdx];

    // Update the PPS and SPS pointers with the ones of the picture.
    pps=pSlice->getPPS();
    sps=pSlice->getSPS();

    // fix Parameter Sets, now that we have the real slice
    m_pcPic->cs->slice = pSlice;
    m_pcPic->cs->sps   = sps;
    m_pcPic->cs->pps   = pps;
#if HEVC_VPS
    m_pcPic->cs->vps   = pSlice->getVPS();
#endif
    m_pcPic->cs->pcv   = pps->pcv;

    // Initialise the various objects for the new set of settings
    m_cSAO.create( sps->getPicWidthInLumaSamples(), sps->getPicHeightInLumaSamples(), sps->getChromaFormatIdc(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), sps->getMaxCodingDepth(), pps->getPpsRangeExtension().getLog2SaoOffsetScale(CHANNEL_TYPE_LUMA), pps->getPpsRangeExtension().getLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA) );
    m_cLoopFilter.create( sps->getMaxCodingDepth() );
    m_cIntraPred.init( sps->getChromaFormatIdc(), sps->getBitDepth( CHANNEL_TYPE_LUMA ) );
    m_cInterPred.init( &m_cRdCost, sps->getChromaFormatIdc() );


    Bool isField = false;
    Bool isTopField = false;

    if(!m_SEIs.empty())
    {
      // Check if any new Picture Timing SEI has arrived
      SEIMessages pictureTimingSEIs = getSeisByType(m_SEIs, SEI::PICTURE_TIMING);
      if (pictureTimingSEIs.size()>0)
      {
        SEIPictureTiming* pictureTiming = (SEIPictureTiming*) *(pictureTimingSEIs.begin());
        isField    = (pictureTiming->m_picStruct == 1) || (pictureTiming->m_picStruct == 2) || (pictureTiming->m_picStruct == 9) || (pictureTiming->m_picStruct == 10) || (pictureTiming->m_picStruct == 11) || (pictureTiming->m_picStruct == 12);
        isTopField = (pictureTiming->m_picStruct == 1) || (pictureTiming->m_picStruct == 9) || (pictureTiming->m_picStruct == 11);
      }
    }

    //Set Field/Frame coding mode
    m_pcPic->fieldPic = isField;
    m_pcPic->topField = isTopField;

    // transfer any SEI messages that have been received to the picture
    m_pcPic->SEIs = m_SEIs;
    m_SEIs.clear();

    // Recursive structure
    m_cCuDecoder.init( &m_cTrQuant, &m_cIntraPred, &m_cInterPred );
    m_cTrQuant  .init( nullptr, sps->getMaxTrSize(), false, false, false, false, false, pps->pcv->rectCUs );

    // RdCost
    m_cRdCost.setCostMode ( COST_STANDARD_LOSSY ); // not used in decoder side RdCost stuff -> set to default
    m_cRdCost.setUseQtbt  ( sps->getSpsNext().getUseQTBT() );

    m_cSliceDecoder.create();
  }
  else
  {
    // make the slice-pilot a real slice, and set up the slice-pilot for the next slice
    m_pcPic->allocateNewSlice();
    CHECK(m_pcPic->slices.size() != (size_t)(m_uiSliceSegmentIdx + 1), "Invalid number of slices");
    m_apcSlicePilot = m_pcPic->swapSliceObject(m_apcSlicePilot, m_uiSliceSegmentIdx);

    Slice *pSlice = m_pcPic->slices[m_uiSliceSegmentIdx]; // we now have a real slice.

    const SPS *sps = pSlice->getSPS();
    const PPS *pps = pSlice->getPPS();

    // fix Parameter Sets, now that we have the real slice
    m_pcPic->cs->slice = pSlice;
    m_pcPic->cs->sps   = sps;
    m_pcPic->cs->pps   = pps;
#if HEVC_VPS
    m_pcPic->cs->vps   = pSlice->getVPS();
#endif
    m_pcPic->cs->pcv   = pps->pcv;

    // check that the current active PPS has not changed...
    if (m_parameterSetManager.getSPSChangedFlag(sps->getSPSId()) )
    {
      EXIT("Error - a new SPS has been decoded while processing a picture");
    }
    if (m_parameterSetManager.getPPSChangedFlag(pps->getPPSId()) )
    {
      EXIT("Error - a new PPS has been decoded while processing a picture");
    }

    xParsePrefixSEImessages();

    // Check if any new SEI has arrived
     if(!m_SEIs.empty())
     {
       // Currently only decoding Unit SEI message occurring between VCL NALUs copied
       SEIMessages& picSEI = m_pcPic->SEIs;
       SEIMessages decodingUnitInfos = extractSeisByType( picSEI, SEI::DECODING_UNIT_INFO);
       picSEI.insert(picSEI.end(), decodingUnitInfos.begin(), decodingUnitInfos.end());
       deleteSEIs(m_SEIs);
     }
  }
}


Void DecLib::xParsePrefixSEIsForUnknownVCLNal()
{
  while (!m_prefixSEINALUs.empty())
  {
    // do nothing?
    msg( NOTICE, "Discarding Prefix SEI associated with unknown VCL NAL unit.\n");
    delete m_prefixSEINALUs.front();
  }
  // TODO: discard following suffix SEIs as well?
}


Void DecLib::xParsePrefixSEImessages()
{
  while (!m_prefixSEINALUs.empty())
  {
    InputNALUnit &nalu=*m_prefixSEINALUs.front();
    m_seiReader.parseSEImessage( &(nalu.getBitstream()), m_SEIs, nalu.m_nalUnitType, m_parameterSetManager.getActiveSPS(), m_pDecodedSEIOutputStream );
    delete m_prefixSEINALUs.front();
    m_prefixSEINALUs.pop_front();
  }
}

Bool DecLib::xDecodeSlice(InputNALUnit &nalu, Int &iSkipFrame, Int iPOCLastDisplay 
#if ENCODE_BGPIC
,Bool &isO
#endif
)
{

#if GENERATE_TEMPRECO_PIC
	
	Picture* bg_NewPicYuvRecoSli = getbgNewPicYuvReco();
	m_cSliceDecoder.setbgNewPicYuvRecoGOP(bg_NewPicYuvRecoSli);
#endif

  m_apcSlicePilot->initSlice(); // the slice pilot is an object to prepare for a new slice
                                // it is not associated with picture, sps or pps structures.

  if (m_bFirstSliceInPicture)
  {
    m_uiSliceSegmentIdx = 0;
  }
  else
  {
    m_apcSlicePilot->copySliceInfo( m_pcPic->slices[m_uiSliceSegmentIdx-1] );
  }
#if HEVC_DEPENDENT_SLICES
  m_apcSlicePilot->setSliceSegmentIdx(m_uiSliceSegmentIdx);
#endif

  m_apcSlicePilot->setNalUnitType(nalu.m_nalUnitType);
  Bool nonReferenceFlag = (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_N ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N   ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA_N  ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N  ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N);
  m_apcSlicePilot->setTemporalLayerNonReferenceFlag(nonReferenceFlag);
  m_apcSlicePilot->setTLayer(nalu.m_temporalId);

  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parseSliceHeader( m_apcSlicePilot, &m_parameterSetManager, m_prevTid0POC );

  // update independent slice index
  UInt uiIndependentSliceIdx = 0;
  if (!m_bFirstSliceInPicture)
  {
    uiIndependentSliceIdx = m_pcPic->slices[m_uiSliceSegmentIdx-1]->getIndependentSliceIdx();
#if HEVC_DEPENDENT_SLICES
    if (!m_apcSlicePilot->getDependentSliceSegmentFlag())
    {
#endif
      uiIndependentSliceIdx++;
#if HEVC_DEPENDENT_SLICES
    }
#endif
  }
  m_apcSlicePilot->setIndependentSliceIdx(uiIndependentSliceIdx);

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "poc", m_apcSlicePilot->getPOC() ) );

#if HEVC_DEPENDENT_SLICES
  // set POC for dependent slices in skipped pictures
  if(m_apcSlicePilot->getDependentSliceSegmentFlag() && m_prevSliceSkipped)
  {
    m_apcSlicePilot->setPOC(m_skippedPOC);
  }
#endif

  xUpdatePreviousTid0POC(m_apcSlicePilot);

  m_apcSlicePilot->setAssociatedIRAPPOC(m_pocCRA);
  m_apcSlicePilot->setAssociatedIRAPType(m_associatedIRAPType);

  //For inference of NoOutputOfPriorPicsFlag
  if (m_apcSlicePilot->getRapPicFlag())
  {
    if ((m_apcSlicePilot->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && m_apcSlicePilot->getNalUnitType() <= NAL_UNIT_CODED_SLICE_IDR_N_LP) ||
        (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA && m_bFirstSliceInSequence) ||
        (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA && m_apcSlicePilot->getHandleCraAsBlaFlag()))
    {
      m_apcSlicePilot->setNoRaslOutputFlag(true);
    }
    //the inference for NoOutputPriorPicsFlag
    if (!m_bFirstSliceInBitstream && m_apcSlicePilot->getRapPicFlag() && m_apcSlicePilot->getNoRaslOutputFlag())
    {
      if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
      {
        m_apcSlicePilot->setNoOutputPriorPicsFlag(true);
      }
    }
    else
    {
      m_apcSlicePilot->setNoOutputPriorPicsFlag(false);
    }

    if(m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
    {
      m_craNoRaslOutputFlag = m_apcSlicePilot->getNoRaslOutputFlag();
    }
  }
  if (m_apcSlicePilot->getRapPicFlag() && m_apcSlicePilot->getNoOutputPriorPicsFlag())
  {
    m_lastPOCNoOutputPriorPics = m_apcSlicePilot->getPOC();
    m_isNoOutputPriorPics = true;
  }
  else
  {
    m_isNoOutputPriorPics = false;
  }

  //For inference of PicOutputFlag
  if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R)
  {
    if ( m_craNoRaslOutputFlag )
    {
      m_apcSlicePilot->setPicOutputFlag(false);
    }
  }

  if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA && m_craNoRaslOutputFlag) //Reset POC MSB when CRA has NoRaslOutputFlag equal to 1
  {
    PPS *pps = m_parameterSetManager.getPPS(m_apcSlicePilot->getPPSId());
    CHECK(pps == 0, "No PPS present");
    SPS *sps = m_parameterSetManager.getSPS(pps->getSPSId());
    CHECK(sps == 0, "No SPS present");
    Int iMaxPOClsb = 1 << sps->getBitsForPOC();
    m_apcSlicePilot->setPOC( m_apcSlicePilot->getPOC() & (iMaxPOClsb - 1) );
    xUpdatePreviousTid0POC(m_apcSlicePilot);
  }

  // Skip pictures due to random access

  if (isRandomAccessSkipPicture(iSkipFrame, iPOCLastDisplay))
  {
    m_prevSliceSkipped = true;
    m_skippedPOC = m_apcSlicePilot->getPOC();
    return false;
  }
  // Skip TFD pictures associated with BLA/BLANT pictures
  if (isSkipPictureForBLA(iPOCLastDisplay))
  {
    m_prevSliceSkipped = true;
    m_skippedPOC = m_apcSlicePilot->getPOC();
    return false;
  }
  // clear previous slice skipped flag
  m_prevSliceSkipped = false;

  //we should only get a different poc for a new picture (with CTU address==0)
#if HEVC_DEPENDENT_SLICES
  if (!m_apcSlicePilot->getDependentSliceSegmentFlag() && m_apcSlicePilot->getPOC()!=m_prevPOC && !m_bFirstSliceInSequence && (m_apcSlicePilot->getSliceCurStartCtuTsAddr() != 0))
#else
  if(m_apcSlicePilot->getPOC() != m_prevPOC && !m_bFirstSliceInSequence && (m_apcSlicePilot->getSliceCurStartCtuTsAddr() != 0))
#endif
  {
    msg( WARNING, "Warning, the first slice of a picture might have been lost!\n");
  }

  // leave when a new picture is found
#if HEVC_DEPENDENT_SLICES
  if (!m_apcSlicePilot->getDependentSliceSegmentFlag() && (m_apcSlicePilot->getSliceCurStartCtuTsAddr() == 0 && !m_bFirstSliceInPicture) )
#else
  if(m_apcSlicePilot->getSliceCurStartCtuTsAddr() == 0 && !m_bFirstSliceInPicture)
#endif
  {
    if (m_prevPOC >= m_pocRandomAccess)
    {
      DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 0 ) );
      m_prevPOC = m_apcSlicePilot->getPOC();
      return true;
    }
    m_prevPOC = m_apcSlicePilot->getPOC();
  }
  else
  {
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 1 ) );
  }

  //detect lost reference picture and insert copy of earlier frame.
  {
    Int lostPoc;
    while((lostPoc=m_apcSlicePilot->checkThatAllRefPicsAreAvailable(m_cListPic, m_apcSlicePilot->getRPS(), true, m_pocRandomAccess)) > 0)
    {
      xCreateLostPicture(lostPoc-1);
    }
  }

#if HEVC_DEPENDENT_SLICES
  if (!m_apcSlicePilot->getDependentSliceSegmentFlag())
  {
#endif
    m_prevPOC = m_apcSlicePilot->getPOC();
#if HEVC_DEPENDENT_SLICES
  }
#endif
  if (m_bFirstSliceInPicture)
  {
    xUpdateRasInit(m_apcSlicePilot);
  }

  // actual decoding starts here
  xActivateParameterSets();

  m_bFirstSliceInSequence = false;
  m_bFirstSliceInBitstream  = false;

  Slice* pcSlice = m_pcPic->slices[m_uiSliceSegmentIdx];
  pcSlice->setPic( m_pcPic );
  m_pcPic->poc         = pcSlice->getPOC();
  m_pcPic->layer       = pcSlice->getTLayer();
  m_pcPic->referenced  = true;
  m_pcPic->layer       = nalu.m_temporalId;

  // When decoding the slice header, the stored start and end addresses were actually RS addresses, not TS addresses.
  // Now, having set up the maps, convert them to the correct form.
#if HEVC_TILES_WPP
  const TileMap& tileMap = *(m_pcPic->tileMap);
#endif
#if HEVC_DEPENDENT_SLICES
#if HEVC_TILES_WPP
  pcSlice->setSliceSegmentCurStartCtuTsAddr( tileMap.getCtuRsToTsAddrMap(pcSlice->getSliceSegmentCurStartCtuTsAddr()) );
  pcSlice->setSliceSegmentCurEndCtuTsAddr( tileMap.getCtuRsToTsAddrMap(pcSlice->getSliceSegmentCurEndCtuTsAddr()) );
#endif
  if(!pcSlice->getDependentSliceSegmentFlag())
  {
#endif
#if HEVC_TILES_WPP
    pcSlice->setSliceCurStartCtuTsAddr( tileMap.getCtuRsToTsAddrMap(pcSlice->getSliceCurStartCtuTsAddr()) );
    pcSlice->setSliceCurEndCtuTsAddr( tileMap.getCtuRsToTsAddrMap(pcSlice->getSliceCurEndCtuTsAddr()) );
#endif
#if HEVC_DEPENDENT_SLICES
  }
#endif

#if HEVC_DEPENDENT_SLICES
  if (!pcSlice->getDependentSliceSegmentFlag())
  {
#endif
    pcSlice->checkCRA(pcSlice->getRPS(), m_pocCRA, m_associatedIRAPType, m_cListPic );
    // Set reference list
#if BG_REFERENCE_SUBSTITUTION
	//cout << "setRefPicList" << endl;
	Int SetRefPoc = -999;
	cout <<a<< "afterdebg" << afterdebg<<endl;
#if BLOCK_ENCODE
	if (afterdebg)
	{
		//cout << "in set" << endl;
		pcSlice->setRefPicListaddbgBlock(m_cListPic, bg_NewPicYuvReco, m_PicYuvTemp, SetRefPoc, BgBlock);
	}
#else
	if (pcSlice->getPOC()!=50&&!isO)
	{
		cout << "in setbg" << pcSlice->getPOC() << endl;
		pcSlice->setRefPicListaddbg(m_cListPic, bg_NewPicYuvReco, m_PicYuvTemp, SetRefPoc, true, true);
	}
#endif
	else
	{
		pcSlice->setRefPicList(m_cListPic, true, true);
	}
#else
	pcSlice->setRefPicList(m_cListPic, true, true);

#endif // BG_REFERENCE_SUBSTITUTION
#if ENCODE_BGPIC
	//设置背景帧的参考帧
	if (pcSlice->getPOC()==50&&isO)
	{
		//pcSlice->setRefPicListaddbg(m_cListPic,bg_NewPicYuvRec,m_PicYuvTemp,true,true);
	}
#endif
    if (!pcSlice->isIntra())
    {
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
      if (pcSlice->isInterB())
      {
        for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1) && bLowDelay; iRefIdx++)
        {
          if ( pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
          {
            bLowDelay = false;
          }
        }
      }

      pcSlice->setCheckLDC(bLowDelay);
    }

    //---------------
    pcSlice->setRefPOCList();


#if HEVC_DEPENDENT_SLICES
  }
#endif

#if HEVC_USE_SCALING_LISTS
  Quant *quant = m_cTrQuant.getQuant();

  if(pcSlice->getSPS()->getScalingListFlag())
  {
    ScalingList scalingList;
    if(pcSlice->getPPS()->getScalingListPresentFlag())
    {
      scalingList = pcSlice->getPPS()->getScalingList();
    }
    else if (pcSlice->getSPS()->getScalingListPresentFlag())
    {
      scalingList = pcSlice->getSPS()->getScalingList();
    }
    else
    {
      scalingList.setDefaultScalingList();
    }
    quant->setScalingListDec(scalingList);
    quant->setUseScalingList(true);
  }
  else
  {
    quant->setUseScalingList(false);
  }
#endif
  //  Decode a picture

  m_cSliceDecoder.decompressSlice( pcSlice, &(nalu.getBitstream()) );
  
  
#if BLOCK_ENCODE
  {
	  int num_block = 0;
	  for (UInt uiH = 0; uiH < m_pcPic->getRecoBuf().Y().height; uiH += BLOCK_GEN_LEN)
	  {
		  for (UInt uiW = 0; uiW < m_pcPic->getRecoBuf().Y().width; uiW += BLOCK_GEN_LEN)
		  {
			  if (BgBlock[num_block] == 1000)
			  {
				  BgBlock[num_block] = 2000;
				  //isdecode = true;
				  afterdebg = true;
			  }
			  num_block++;
		  }
	  }
  }

  if (pcSlice->getPOC() == 2)
	  bgQp = pcSlice->getSliceQp() - 17;
  if (pcSlice->getSliceQp() == bgQp) //
  {
	  isBgBlock = true;
	  pcSlice->setPicOutputFlag(true);
	  pcSlice->getPic()->referenced = false;
	  bgpoc = pcSlice->getPOC();
	 

	  {
		  bg_NewPicYuvRec->DeleteReco(bg_NewPicYuvRec);
		  int num_block = 0;
		  for (UInt uiH = 0; uiH < m_pcPic->getRecoBuf().Y().height; uiH += BLOCK_GEN_LEN)
		  {
			  for (UInt uiW = 0; uiW < m_pcPic->getRecoBuf().Y().width; uiW += BLOCK_GEN_LEN)
			  {
				  if (!BgBlock[num_block] && !m_pcPic->CompBlockRecoIsSimilar(uiW, uiH, m_pcPic, bg_NewPicYuvRec))
				  {
					  BgBlock[num_block] = 1000;
					  isdecode = true;
					  //afterdebg = true;
					  //m_pcPic->CopyReco2Block(bg_NewPicYuvReco, uiW, uiH, m_pcPic);

				  }
				  num_block++;
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
			  cout << endl <<" bgBlock " << j << endl;
		  }
	  }
	 
  }
  //if (isdecode)
  {
	//  pcSlice->setPicOutputFlag(true);
	 // pcSlice->getPic()->referenced = false;
  }
#endif

#if ENCODE_BGPIC
  if (pcSlice->getPOC() == BGPICPOC)
  {
	  cout << isO << "  " << pcSlice->getPOC() << endl;
  }
  if (pcSlice->getPOC() == BGPICPOC && isO)
  {
	  pcSlice->setPicOutputFlag(true);
	  pcSlice->getPic()->referenced = false;

	  pcSlice->getPic()->CopyPic2Reco(bg_NewPicYuvReco,pcSlice->getPic());

	  isO = false;
	  cout << isO << "  " << pcSlice->getPOC() << endl;
  }
  
  
  
#endif
  
#if BG_REFERENCE_SUBSTITUTION 
#if BLOCK_ENCODE
  if(afterdebg&&SetRefPoc!=-999)//&&a)
#else
  if (pcSlice->getPOC() != BGPICPOC&&!isO) 
#endif
  {
	  pcSlice->resetRefPicList(m_cListPic, m_PicYuvTemp,SetRefPoc);
	  //cout << "in resetbg" << pcSlice->getPOC() << endl;
  }
#endif 
  a = true;
  m_bFirstSliceInPicture = false;
  m_uiSliceSegmentIdx++;
  return false;
}

#if HEVC_VPS
Void DecLib::xDecodeVPS( InputNALUnit& nalu )
{
  VPS* vps = new VPS();
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parseVPS( vps );
  m_parameterSetManager.storeVPS( vps, nalu.getBitstream().getFifo() );
}
#endif

Void DecLib::xDecodeSPS( InputNALUnit& nalu )
{
  SPS* sps = new SPS();
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parseSPS( sps );
  m_parameterSetManager.storeSPS( sps, nalu.getBitstream().getFifo() );

  DTRACE( g_trace_ctx, D_QP_PER_CTU, "CTU Size: %dx%d", sps->getMaxCUWidth(), sps->getMaxCUHeight() );
}

Void DecLib::xDecodePPS( InputNALUnit& nalu )
{
  PPS* pps = new PPS();
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parsePPS( pps );
  m_parameterSetManager.storePPS( pps, nalu.getBitstream().getFifo() );
}

Bool DecLib::decode(InputNALUnit& nalu, Int& iSkipFrame, Int& iPOCLastDisplay
#if ENCODE_BGPIC
,Bool& isO
#endif
)
{
  // ignore all NAL units of layers > 0
  if (nalu.m_nuhLayerId > 0)
  {
    msg( WARNING, "Warning: found NAL unit with nuh_layer_id equal to %d. Ignoring.\n", nalu.m_nuhLayerId);
    return false;
  }

  switch (nalu.m_nalUnitType)
  {
#if HEVC_VPS
    case NAL_UNIT_VPS:
      xDecodeVPS( nalu );
      return false;
#endif

    case NAL_UNIT_SPS:
      xDecodeSPS( nalu );
      return false;

    case NAL_UNIT_PPS:
      xDecodePPS( nalu );
      return false;

    case NAL_UNIT_PREFIX_SEI:
      // Buffer up prefix SEI messages until SPS of associated VCL is known.
      m_prefixSEINALUs.push_back(new InputNALUnit(nalu));
      return false;

    case NAL_UNIT_SUFFIX_SEI:
      if (m_pcPic)
      {
        m_seiReader.parseSEImessage( &(nalu.getBitstream()), m_pcPic->SEIs, nalu.m_nalUnitType, m_parameterSetManager.getActiveSPS(), m_pDecodedSEIOutputStream );
      }
      else
      {
        msg( NOTICE, "Note: received suffix SEI but no picture currently active.\n");
      }
      return false;

    case NAL_UNIT_CODED_SLICE_TRAIL_R:
    case NAL_UNIT_CODED_SLICE_TRAIL_N:
    case NAL_UNIT_CODED_SLICE_TSA_R:
    case NAL_UNIT_CODED_SLICE_TSA_N:
    case NAL_UNIT_CODED_SLICE_STSA_R:
    case NAL_UNIT_CODED_SLICE_STSA_N:
    case NAL_UNIT_CODED_SLICE_BLA_W_LP:
    case NAL_UNIT_CODED_SLICE_BLA_W_RADL:
    case NAL_UNIT_CODED_SLICE_BLA_N_LP:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_RADL_N:
    case NAL_UNIT_CODED_SLICE_RADL_R:
    case NAL_UNIT_CODED_SLICE_RASL_N:
    case NAL_UNIT_CODED_SLICE_RASL_R:
      return xDecodeSlice(nalu, iSkipFrame, iPOCLastDisplay
#if ENCODE_BGPIC
	  ,isO
#endif
	  );
      break;

    case NAL_UNIT_EOS:
      m_associatedIRAPType = NAL_UNIT_INVALID;
      m_pocCRA = 0;
      m_pocRandomAccess = MAX_INT;
      m_prevPOC = MAX_INT;
      m_prevSliceSkipped = false;
      m_skippedPOC = 0;
      return false;

    case NAL_UNIT_ACCESS_UNIT_DELIMITER:
      {
        AUDReader audReader;
        UInt picType;
        audReader.parseAccessUnitDelimiter(&(nalu.getBitstream()),picType);
        msg( NOTICE, "Note: found NAL_UNIT_ACCESS_UNIT_DELIMITER\n");
        return false;
      }

    case NAL_UNIT_EOB:
      return false;

    case NAL_UNIT_FILLER_DATA:
      {
        FDReader fdReader;
        UInt size;
        fdReader.parseFillerData(&(nalu.getBitstream()),size);
        msg( NOTICE, "Note: found NAL_UNIT_FILLER_DATA with %u bytes payload.\n", size);
        return false;
      }

    case NAL_UNIT_RESERVED_VCL_N10:
    case NAL_UNIT_RESERVED_VCL_R11:
    case NAL_UNIT_RESERVED_VCL_N12:
    case NAL_UNIT_RESERVED_VCL_R13:
    case NAL_UNIT_RESERVED_VCL_N14:
    case NAL_UNIT_RESERVED_VCL_R15:

    case NAL_UNIT_RESERVED_IRAP_VCL22:
    case NAL_UNIT_RESERVED_IRAP_VCL23:

    case NAL_UNIT_RESERVED_VCL24:
    case NAL_UNIT_RESERVED_VCL25:
    case NAL_UNIT_RESERVED_VCL26:
    case NAL_UNIT_RESERVED_VCL27:
    case NAL_UNIT_RESERVED_VCL28:
    case NAL_UNIT_RESERVED_VCL29:
    case NAL_UNIT_RESERVED_VCL30:
    case NAL_UNIT_RESERVED_VCL31:
#if !HEVC_VPS
    case NAL_UNIT_RESERVED_32:
#endif
      msg( NOTICE, "Note: found reserved VCL NAL unit.\n");
      xParsePrefixSEIsForUnknownVCLNal();
      return false;

    case NAL_UNIT_RESERVED_NVCL41:
    case NAL_UNIT_RESERVED_NVCL42:
    case NAL_UNIT_RESERVED_NVCL43:
    case NAL_UNIT_RESERVED_NVCL44:
    case NAL_UNIT_RESERVED_NVCL45:
    case NAL_UNIT_RESERVED_NVCL46:
    case NAL_UNIT_RESERVED_NVCL47:
      msg( NOTICE, "Note: found reserved NAL unit.\n");
      return false;
    case NAL_UNIT_UNSPECIFIED_48:
    case NAL_UNIT_UNSPECIFIED_49:
    case NAL_UNIT_UNSPECIFIED_50:
    case NAL_UNIT_UNSPECIFIED_51:
    case NAL_UNIT_UNSPECIFIED_52:
    case NAL_UNIT_UNSPECIFIED_53:
    case NAL_UNIT_UNSPECIFIED_54:
    case NAL_UNIT_UNSPECIFIED_55:
    case NAL_UNIT_UNSPECIFIED_56:
    case NAL_UNIT_UNSPECIFIED_57:
    case NAL_UNIT_UNSPECIFIED_58:
    case NAL_UNIT_UNSPECIFIED_59:
    case NAL_UNIT_UNSPECIFIED_60:
    case NAL_UNIT_UNSPECIFIED_61:
    case NAL_UNIT_UNSPECIFIED_62:
    case NAL_UNIT_UNSPECIFIED_63:
      msg( NOTICE, "Note: found unspecified NAL unit.\n");
      return false;
    default:
      THROW( "Invalid NAL unit type" );
      break;
  }

  return false;
}

/** Function for checking if picture should be skipped because of association with a previous BLA picture
 *  This function skips all TFD pictures that follow a BLA picture in decoding order and precede it in output order.
 */
Bool DecLib::isSkipPictureForBLA( Int& iPOCLastDisplay )
{
  if( ( m_associatedIRAPType == NAL_UNIT_CODED_SLICE_BLA_N_LP || m_associatedIRAPType == NAL_UNIT_CODED_SLICE_BLA_W_LP || m_associatedIRAPType == NAL_UNIT_CODED_SLICE_BLA_W_RADL ) &&
        m_apcSlicePilot->getPOC() < m_pocCRA && ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N ) )
  {
    iPOCLastDisplay++;
    return true;
  }
  return false;
}

/** Function for checking if picture should be skipped because of random access. This function checks the skipping of pictures in the case of -s option random access.
 *  All pictures prior to the random access point indicated by the counter iSkipFrame are skipped.
 *  It also checks the type of Nal unit type at the random access point.
 *  If the random access point is CRA/CRANT/BLA/BLANT, TFD pictures with POC less than the POC of the random access point are skipped.
 *  If the random access point is IDR all pictures after the random access point are decoded.
 *  If the random access point is none of the above, a warning is issues, and decoding of pictures with POC
 *  equal to or greater than the random access point POC is attempted. For non IDR/CRA/BLA random
 *  access point there is no guarantee that the decoder will not crash.
 */
Bool DecLib::isRandomAccessSkipPicture( Int& iSkipFrame, Int& iPOCLastDisplay )
{
  if (iSkipFrame)
  {
    iSkipFrame--;   // decrement the counter
    return true;
  }
  else if (m_pocRandomAccess == MAX_INT) // start of random access point, m_pocRandomAccess has not been set yet.
  {
    if (   m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA
        || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
        || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL )
    {
      // set the POC random access since we need to skip the reordered pictures in the case of CRA/CRANT/BLA/BLANT.
      m_pocRandomAccess = m_apcSlicePilot->getPOC();
    }
    else if ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP )
    {
      m_pocRandomAccess = -MAX_INT; // no need to skip the reordered pictures in IDR, they are decodable.
    }
    else
    {
      if(!m_warningMessageSkipPicture)
      {
        msg( WARNING, "\nWarning: this is not a valid random access point and the data is discarded until the first CRA picture");
        m_warningMessageSkipPicture = true;
      }
      return true;
    }
  }
  // skip the reordered pictures, if necessary
  else if (m_apcSlicePilot->getPOC() < m_pocRandomAccess && (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N))
  {
    iPOCLastDisplay++;
    return true;
  }
  // if we reach here, then the picture is not skipped.
  return false;
}




//! \}
