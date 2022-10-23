/// this is the one that works 

#ifndef ALedTools_h
#define ALedTools_h
#include "Arduino.h"
#include "FastLED.h"


#define LED_TYPE WS2812B     // WS2811? 

class ALedSegment;
class ALedStrip;
class ALedColorWave;
class ATrapezoidWaveBeat;
class ALedFader;
typedef ATrapezoidWaveBeat* ATrapezoidWaveBeatPtr;
typedef ALedSegment* ALedSegmentPtr;
typedef ALedStrip* ALedStripPtr;
typedef CRGB* CRGBPtr;
typedef ALedColorWave* AWavePtr;
typedef ALedFader* ALedFaderPtr;

const uint8_t cMaxNumSegments=20;

class ALedStrip {

public:
   ALedStrip();
   ALedStrip(CRGBPtr pLeds, uint16_t numLeds);
   ALedStrip(CRGBPtr pLeds, uint16_t numLeds,uint8_t pin,uint8_t brightness);
   
   ALedSegmentPtr addSegment(ALedSegmentPtr pSeg);
   ALedSegmentPtr addSegment(ALedSegment& seg);
   ALedSegmentPtr addNextSegment(ALedSegmentPtr pSeg);
   ALedSegmentPtr addNextSegment(ALedSegment& seg);
   ALedSegmentPtr addOverlappingSegment(ALedSegmentPtr pNewSeg, ALedSegment* pOldSeg);
   ALedSegmentPtr addOverlappingSegment(ALedSegment& newSeg, ALedSegment& oldSeg);
   void loop();
   
protected:
  uint16_t m_numLeds;
  uint8_t m_brightness; 
  CRGBPtr m_leds;
  uint8_t m_numSegments;
  ALedSegmentPtr m_segmentPtr[cMaxNumSegments];
};

class ALedSegment {

public:
	ALedSegment();
  	ALedSegment(CRGBPtr leds,uint16_t length,uint32_t delta,uint32_t delay);
     
  	void loop(uint32_t ms);
	virtual void doEffect(uint32_t t) = 0;
	void setLeds(CRGBPtr leds) {m_leds=leds;}
	CRGBPtr getLeds() {return m_leds;}
	uint16_t getLength() {return m_length;}
	void setLength(uint16_t len) {m_length=len;}
	bool getOverlay() {return m_isOverlay;}
	void setOverlay(bool ovr) {m_isOverlay=ovr;}
	void setStrip(ALedStripPtr pStrip) {m_strip=pStrip;}
   uint8_t m_index;
   void turnOn();
   void turnOff();
   void toggleOnOff();
 
protected:
   
   uint32_t m_delta;
   uint32_t m_lastEffect;
   uint32_t m_startTime;
   bool m_isActive;
   CRGBPtr m_leds;
   uint16_t m_length;
   bool m_isOverlay;
   ALedStripPtr m_strip;
   bool m_onOff;
   

};

class ALedTestSegment : public ALedSegment {

public:
  ALedTestSegment(CRGBPtr leds, uint16_t length, CRGB color);
  void doEffect(uint32_t t);
protected:
  CRGB m_color;
};

class ALedWarbler : public ALedSegment {

public:
   ALedWarbler(CRGBPtr leds,uint16_t length,uint32_t delta,uint32_t delay, 
    uint8_t bpm, uint8_t offset,CHSV color1, CHSV color2);
   void doEffect(uint32_t t);
   
protected:
   uint8_t m_bpm;
   uint8_t m_offset;
   CHSV m_color1;
   CHSV m_color2;
   CHSV* m_chsv;
};


class ALed2ColorGradient : public ALedSegment {

public: 
   ALed2ColorGradient(CRGBPtr leds,uint16_t length,uint32_t delta,
    uint32_t delay,CRGB color1,CRGB color2);

   void doEffect(uint32_t t);
   
protected:
   CRGB m_color1;
   CRGB m_color2;   
   bool m_flag;
   
};

class ALed2ColorSpinner : public ALed2ColorGradient {
public:
   ALed2ColorSpinner(CRGBPtr leds,uint16_t length,uint32_t delta,uint32_t delay,uint8_t bpm,
    uint8_t offset,CRGB color1,CRGB color2);     
   void doEffect(uint32_t t);
protected:
   int16_t m_index;
   int8_t m_direction;
   bool m_colorFlag;
   uint8_t m_bpm;
   uint8_t m_offset;
};

class ALedSinelon : public ALedSegment {

public:
  	ALedSinelon(CRGBPtr leds,uint16_t length,uint32_t delta,uint32_t delay,CRGB color,
  	 uint8_t bpm, uint8_t fade,uint8_t offset);       	 
	void doEffect(uint32_t t);

protected:
   CRGB m_color;
   uint8_t m_bpm;
   uint8_t m_fade;
   uint8_t m_offset;
};

inline uint8_t scale(uint8_t val, uint8_t numerator) {
   uint16_t step = val*numerator;
   return step/255;
}

struct LedArray {CRGB* value;};
struct Length {uint16_t value;};
struct FadeInTime {float value;};
struct PlateauTime {float value;};
struct FadeOutTime {float value;};
struct DarkTime {float value;};
struct Offset {uint8_t value;};
struct MinimumBrightness {uint8_t value;};
struct SparkleDensity {uint16_t value;};

class ALedFader : public ALedSegment { 

public:
   ALedFader() = default;
   ALedFader(LedArray leds, Length length, FadeInTime P0, PlateauTime P1, FadeOutTime P2, DarkTime P3, 
    Offset offset = Offset{0}, MinimumBrightness minVal = MinimumBrightness{0});
   void doEffect(uint32_t t);

protected:
   uint8_t m_bpm;
   //uint8_t m_phase;
   uint8_t m_offset;
   uint16_t i;
   ATrapezoidWaveBeatPtr m_pBeat;
};


#define LIM 10000


class ALedSparkler : public ALedSegment {

public:
  ALedSparkler(LedArray leds, Length length, SparkleDensity density);
  void doEffect(uint32_t t);
  void setDensity(uint16_t density) {m_density = density;}

protected:
  uint16_t m_density;
};

class ALedColorWave : public ALedSegment {

public:   
   ALedColorWave(LedArray leds, Length length);
   void doEffect(uint32_t t);
};

class ATrapezoidWaveBeat {

public:
  ATrapezoidWaveBeat(float P0, float P1, float P2, float P3, uint8_t offset = 0, uint8_t minVal = 0);
  uint8_t getBeat(uint32_t t);
    
protected:
  uint32_t m_bpm;
  uint32_t m_t[5];
  uint32_t m_period;
  uint32_t m_offset;
  uint8_t m_minVal;

};

struct StartHue {uint8_t value;};
struct EndHue {uint8_t value;};
struct StartSat {uint8_t value;};
struct EndSat {uint8_t value;};
struct ValueMin {float value;};
struct ValueMax {float value;};

class ALedBreather : public ALedSegment {

public:
  ALedBreather(LedArray leds, Length length, StartHue huea = StartHue{15}, EndHue hueb = EndHue{95}, 
   StartSat sata = StartSat{230}, EndSat satb = EndSat{255}, ValueMin valuemin = ValueMin{120.0}, 
   ValueMax valuemax = ValueMax{255.0});
  void doEffect(uint32_t t);

protected:
  float pulseSpeed = 2;  // Larger value gives faster pulse.
  
  uint8_t hueA;  // Start hue at valueMin.
  uint8_t satA;  // Start saturation at valueMin.
  float valueMin;  // Pulse minimum value (Should be less then valueMax).
  
  uint8_t hueB;  // End hue at valueMax.
  uint8_t satB;  // End saturation at valueMax.
  float valueMax;  // Pulse maximum value (Should be larger then valueMin).
  
  uint8_t hue = hueA;  // Do Not Edit
  uint8_t sat = satA;  // Do Not Edit
  float val = valueMin;  // Do Not Edit
  uint8_t hueDelta = hueA - hueB;  // Do Not Edit
  float delta = (valueMax - valueMin) / 2.35040238;  // Do Not Edit

};


#endif
