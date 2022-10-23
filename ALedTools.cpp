
#include "Arduino.h"
#include "ALedTools.h"

//#define PLOT_FADERS

ALedStrip::ALedStrip() = default;

ALedStrip::ALedStrip(CRGBPtr pLeds, uint16_t numLeds) {
   m_leds = pLeds;
   m_numLeds = numLeds;
   m_numSegments = 0;
}

ALedStrip::ALedStrip(CRGBPtr pLeds,uint16_t numLeds,uint8_t pin,uint8_t brightness) {
   
   delay(500);

   m_leds=pLeds;
   m_numLeds=numLeds;
   m_brightness=brightness;
   m_numSegments=0;
   
   //
   // the template class requires that the pin parameter is a constant
   // known at compile-time, hence this crazy-looking case statement
   //
   CLEDController* p;

   switch (pin) {
      case 2: p = &FastLED.addLeds<LED_TYPE,2,GRB>(pLeds,m_numLeds);break;
      case 3: p = &FastLED.addLeds<LED_TYPE,3,GRB>(pLeds,m_numLeds);break;
      case 4: p = &FastLED.addLeds<LED_TYPE,4,GRB>(pLeds,m_numLeds);break;
      case 5: p = &FastLED.addLeds<LED_TYPE,5,GRB>(pLeds,m_numLeds);break;
      case 6: p = &FastLED.addLeds<LED_TYPE,6,GRB>(pLeds,m_numLeds);break;
      case 7: p = &FastLED.addLeds<LED_TYPE,7,GRB>(pLeds,m_numLeds);break;
      case 8: p = &FastLED.addLeds<LED_TYPE,8,GRB>(pLeds,m_numLeds);break;
      case 9: p = &FastLED.addLeds<LED_TYPE,9,GRB>(pLeds,m_numLeds);break;
      case 10: p = &FastLED.addLeds<LED_TYPE,10,GRB>(pLeds,m_numLeds);break;
      case 11: p = &FastLED.addLeds<LED_TYPE,11,GRB>(pLeds,m_numLeds);break;
      case 12: p = &FastLED.addLeds<LED_TYPE,12,GRB>(pLeds,m_numLeds);break;  
      case 14: p = &FastLED.addLeds<LED_TYPE,14,GRB>(pLeds,m_numLeds);break;  
      case 15: p = &FastLED.addLeds<LED_TYPE,15,GRB>(pLeds,m_numLeds);break;  
   }
   
   p->setCorrection(TypicalLEDStrip);
   //p->setDither(brightness < 255);
   
    
   FastLED.setBrightness(brightness);
   FastLED.clear();
   FastLED.show();
   
   delay(500);
}

ALedSegmentPtr ALedStrip::addSegment(ALedSegmentPtr pSeg) {
   if (m_numSegments < cMaxNumSegments) {
      m_numSegments += 1;
      m_segmentPtr[m_numSegments-1] = pSeg;
      pSeg->setStrip(this);
      pSeg->m_index = m_numSegments;
   }
   return pSeg;
}

ALedSegmentPtr ALedStrip::addSegment(ALedSegment& seg) {
  return this->addSegment(&seg);
}

//
// add a new segment just past the last
// existing segment
//
ALedSegmentPtr ALedStrip::addNextSegment(ALedSegmentPtr pSeg) {
   if (m_numSegments < cMaxNumSegments) {
      uint16_t offset = 0;
      if (m_numSegments > 0) {
         for(uint8_t i=0; i < m_numSegments; i++) {
            if(!(m_segmentPtr[i]->getOverlay())) {
            	offset += m_segmentPtr[i]->getLength();
            }
         }
      }
      pSeg->setLeds(m_leds+offset);
      m_numSegments += 1;
      m_segmentPtr[m_numSegments-1] = pSeg;
      pSeg->setStrip(this);
      pSeg->m_index = m_numSegments;
   }
   return pSeg;
}

ALedSegmentPtr ALedStrip::addNextSegment(ALedSegment& seg) {
  return this->addNextSegment(&seg);
}

ALedSegmentPtr ALedStrip::addOverlappingSegment(ALedSegmentPtr pNewSeg, ALedSegmentPtr pOldSeg) {
   if (m_numSegments < cMaxNumSegments) {
      pNewSeg->setLeds(pOldSeg->getLeds());
      //pNewSeg->setLength(pOldSeg->getLength());
      pNewSeg->setOverlay(true);
      m_numSegments += 1;
      m_segmentPtr[m_numSegments-1] = pNewSeg;
      pNewSeg->setStrip(this);
      pNewSeg->m_index = m_numSegments;
   }
   return pNewSeg;
}

ALedSegmentPtr ALedStrip::addOverlappingSegment(ALedSegment& newSeg, ALedSegment& oldSeg) {
  return this->addOverlappingSegment(&newSeg, &oldSeg);
}

void ALedStrip::loop() {
   uint32_t t = millis();
   for(uint8_t i=0; i < m_numSegments; i++) {
      m_segmentPtr[i]->loop(t);
   }
   #ifdef PLOT_FADERS
   Serial.println();
   #endif
}
 
ALedSegment::ALedSegment() = default;

ALedSegment::ALedSegment(CRGB* leds,uint16_t length,uint32_t delta, uint32_t delay) {
   m_length = length;
   m_delta = delta;
   m_leds = leds;
   m_lastEffect = millis()+delay;
   m_startTime = m_lastEffect;
   m_isActive = delay==0;
   m_isOverlay=false;
   m_onOff = true;
}

void ALedSegment::turnOn() {
   m_onOff = true;
}

void ALedSegment::turnOff() {
   m_onOff = false;
}

void ALedSegment::toggleOnOff() {
   m_onOff = !m_onOff;
}

void ALedSegment::loop(uint32_t ms) {
   if (m_onOff) {
      if (!m_isActive) {
         m_isActive = ms >= m_startTime;
      }
   
      if (m_isActive) {   
         if (ms - m_lastEffect >= m_delta) {
            m_lastEffect = ms;
            doEffect(ms);
         }
      }
   }
   return;
}

ALedTestSegment::ALedTestSegment(CRGBPtr leds, uint16_t length, CRGB color) : 
 ALedSegment(leds, length, 0, 0), m_color(color) {}

void ALedTestSegment::doEffect(uint32_t t) {
  fill_solid(m_leds, m_length, m_color);
}

ALedWarbler::ALedWarbler(CRGB* leds,uint16_t length,uint32_t delta,uint32_t delay, 
 uint8_t bpm, uint8_t offset,CHSV color1,CHSV color2) : 
 ALedSegment(leds,length,delta,delay) {
   m_bpm = bpm;
   m_offset = offset;
   m_color1 = color1;
   m_color2 = color2;
   //byte* ptr = malloc(sizeof(CHSV)*length);
   m_chsv = (CHSV*)malloc(sizeof(CHSV)*length);
   fill_gradient(m_chsv,0,color1,length-1,color2);
}

void ALedWarbler::doEffect(uint32_t t) {
   uint8_t beats = beatsin8(m_bpm,0,255,0,m_offset);
   for(uint16_t i=0; i<m_length; i++) {
      m_leds[i] = CHSV(m_chsv[i].h,m_chsv[i].s,beats);    
   }
}

ALed2ColorGradient::ALed2ColorGradient(CRGB* leds,uint16_t length,uint32_t delta,
 uint32_t delay,CRGB color1,CRGB color2) : ALedSegment(leds,length,delta,delay) {
      m_color1 = color1;
      m_color2 = color2;
      m_flag = true;
      fill_gradient_RGB(m_leds,m_length,color1,color2);
}

void ALed2ColorGradient::doEffect(uint32_t t) {   
   if (m_flag) {
      fill_gradient_RGB(m_leds,m_length,m_color1,m_color2);
   }
   else {
      fill_solid(m_leds,m_length,CRGB::Black);
   }
   //m_flag = !m_flag;
   //FastLED.show();
}

ALed2ColorSpinner::ALed2ColorSpinner(CRGB* leds,uint16_t length,uint32_t delta,
 uint32_t delay,uint8_t bpm,uint8_t offset,CRGB color1,CRGB color2) :
 ALed2ColorGradient(leds,length,delta,delay,color1,color2) {
   m_index = 0;
   m_direction = 1;
   m_colorFlag = true;
   m_bpm=bpm;
   m_offset=offset;
}
   
void ALed2ColorSpinner::doEffect(uint32_t t) {
   m_index = beatsin8(m_bpm,0,m_length/2,0,m_offset);
   m_colorFlag = beatsin8(m_bpm/2,0,1,0,m_offset);
   
   fill_gradient_RGB(m_leds + m_index,m_length-2*m_index,
    m_colorFlag ? m_color1 : m_color2,
    m_colorFlag ? m_color2 : m_color1);
    
   fill_solid(m_leds,m_index,CRGB::Black);
   fill_solid(m_leds+m_length-m_index,m_index,CRGB::Black);
}


ALedSinelon::ALedSinelon(CRGB* leds,uint16_t length,uint32_t delta,uint32_t delay,
 CRGB color,uint8_t bpm,uint8_t fade,uint8_t offset) : 
 ALedSegment(leds,length,delta,delay) {
   m_color = color;
   m_bpm = bpm;
   m_fade = fade;
   m_offset=offset;
}
	
void ALedSinelon::doEffect(uint32_t t) {
  fadeToBlackBy(m_leds, m_length, m_fade);
  int8_t pos = beatsin8(m_bpm, 0, m_length - 1, 0, m_offset);
  //*(m_leds+pos) += m_color;
  m_leds[pos] += m_color;
}

ALedSparkler::ALedSparkler(LedArray leds, Length length, SparkleDensity density) :
   ALedSegment(leds.value, length.value, 0, 0) {
    m_density = density.value;
}

void ALedSparkler::doEffect(uint32_t t) {
  uint16_t r;
  for (uint16_t i=0; i < m_length; i++) {
    r = random16(LIM);
    if (r <= m_density) {
      m_leds[i] = CRGB::White;
      //m_leds[i].maximizeBrightness();
    }
  }
}


ALedFader::ALedFader(LedArray leds, Length length, FadeInTime P0, PlateauTime P1, FadeOutTime P2, DarkTime P3, 
 Offset offset = Offset{0}, MinimumBrightness minVal = MinimumBrightness{0}) :
 ALedSegment(leds.value, length.value, 0, 0) {
   m_pBeat = new ATrapezoidWaveBeat(P0.value, P1.value, P2.value, P3.value, offset.value, minVal.value);
}



inline uint8_t maxColor(CRGB c) { 
  (c.r > c.g) ? max(c.r,c.b) : max(c.g,c.b);
}

void ALedFader::doEffect(uint32_t t) {
   uint8_t beat = (uint8_t)m_pBeat->getBeat(t);
   
   #ifdef PLOT_FADERS
    Serial.print(beat);
    Serial.print(",");
   #endif
   
   for (i=0; i < m_length; i++) {
     m_leds[i] %= beat;
   }
}

ALedColorWave::ALedColorWave(LedArray leds, Length length) : 
 ALedSegment(leds.value, length.value, 0, 0) {
}



ALedBreather::ALedBreather(LedArray leds, Length length, StartHue huea, EndHue hueb, 
 StartSat sata, EndSat satb, ValueMin valuemin, ValueMax valuemax) : 
 ALedSegment(leds.value, length.value, 0, 0), hueA(huea.value), hueB(hueb.value),
 satA(sata.value), satB(satb.value), valueMin(valuemin.value), valueMax(valuemax.value) {
  hue = hueA;  // Do Not Edit
  sat = satA;  // Do Not Edit
  val = valueMin;  // Do Not Edit
  hueDelta = hueA - hueB;  // Do Not Edit
  delta = (valueMax - valueMin) / 2.35040238;  // Do Not Edit
}


void ALedBreather::doEffect(uint32_t t) {
  float dV = ((exp(sin(pulseSpeed * t/2000.0*PI)) -0.36787944) * delta);
  val = valueMin + dV;
  hue = map(val, valueMin, valueMax, hueA, hueB);  // Map hue based on current val
  sat = map(val, valueMin, valueMax, satA, satB);  // Map sat based on current val

  for (int i = 0; i < m_length; i++) {
    m_leds[i] = CHSV(hue, sat, val);

    // You can experiment with commenting out these dim8_video lines
    // to get a different sort of look.
    m_leds[i].r = dim8_video(m_leds[i].r);
    m_leds[i].g = dim8_video(m_leds[i].g);
    m_leds[i].b = dim8_video(m_leds[i].b);
  }
}

extern CRGBPalette16 gCurrentPalette;

void ALedColorWave::doEffect(uint32_t t) {

// This function draws color waves with an ever-changing,
// widely-varying set of parameters, using a color palette.
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;
 
  //uint8_t sat8 = beatsin88(87, 220, 250);
  uint8_t brightdepth = beatsin88(341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88(203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 300, 1500);
  
  uint16_t ms = (uint16_t)t;
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88(400, 5,9);
  uint16_t brightnesstheta16 = sPseudotime;
  
  for(uint16_t i = 0 ; i < m_length; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;
    uint16_t h16_128 = hue16 >> 7;
    if(h16_128 & 0x100) {
      hue8 = 255 - (h16_128 >> 1);
    } else {
      hue8 = h16_128 >> 1;
    }

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16(brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);
    
    uint8_t index = hue8;
    //index = triwave8(index);
    index = scale8(index, 240);

    CRGB newcolor = ColorFromPalette(gCurrentPalette, index, bri8);

    uint16_t pixelnumber = i;
    pixelnumber = (m_length-1) - pixelnumber;
    
    nblend(m_leds[pixelnumber], newcolor, 128);
  }

}


ATrapezoidWaveBeat::ATrapezoidWaveBeat(float P0,float P1,float P2,float P3, uint8_t offset, uint8_t minVal) 
 : m_minVal(minVal) {    
    uint32_t P[] ={(uint32_t)(1000*P0), (uint32_t)(1000*P1), (uint32_t)(1000*P2), (uint32_t)(1000*P3)};
    m_period = P[0] +P[1] + P[2] + P[3];
    m_bpm = (60000ULL)/m_period;
    m_t[0] = 0;
   for (int i = 1; i < 5; i++) {
      m_t[i] = m_t[i-1] + P[i-1];
    }
    m_offset = (m_period*(int32_t)offset)/255ULL;
}

uint8_t ATrapezoidWaveBeat::getBeat(uint32_t t) {
     
    t = (t + m_offset) % m_period;
    uint32_t ret;
    
    if (t < m_t[1]) {
      ret = ((255 - m_minVal)*t)/m_t[1] + m_minVal;
    } 
    else if ((m_t[1] <= t) && (t < m_t[2])) {
        ret = 255;  
    } 
    else if ((m_t[2] <= t) && (t < m_t[3])) {
        ret = (255 - m_minVal)*(m_t[3] - t)/(m_t[3] - m_t[2]) + m_minVal ;  
    } 
    else if ((m_t[3] <= t) && (t <= m_t[4])) {
        ret = m_minVal;
    }
    
    return (uint8_t)ret;    
  }
 
