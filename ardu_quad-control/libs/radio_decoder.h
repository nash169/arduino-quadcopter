#ifndef RADIO_DECODER_H
#define RADIO_DECODER_H

#include <Arduino.h>

struct ChPin
{
	uint8_t analogPin;
	int status; // ?

	uint16_t rc_values;
	uint32_t rc_start;
	volatile uint16_t rc_shared;
	
	uint16_t minIn, maxIn;
	float minOut, maxOut;

	float output;
};

class RadioDecoder
{
public:
	RadioDecoder(uint8_t const *pins, uint8_t const numPins);
	RadioDecoder();
	~RadioDecoder();
	void Initialize(float *minOuts, float *maxOuts, bool CALIBRATE);
	void rc_read_values();
	inline uint16_t GetChannel(uint8_t chIndex) { return channel[chIndex].rc_values; }
	float CtrInput(int ch);
	void PrintIn();
	void PrintOut();

protected:
	void CalcInput(uint8_t channel, uint8_t input_pin);
	void Calibrate();

private:
	static void CallbackGlue();
	void Callback();

	static RadioDecoder *isr;
	int numChannels;
	ChPin *channel;
};

#endif // RADIO_DECODER_H