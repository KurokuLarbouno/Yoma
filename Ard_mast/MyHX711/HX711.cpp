#ifndef HX711_h
#define HX711_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#if ARDUINO_VERSION <= 106
    // "yield" is not implemented as noop in older Arduino Core releases, so let's define it.
    // See also: https://stackoverflow.com/questions/34497758/what-is-the-secret-of-the-arduino-yieldfunction/34498165#34498165
    void yield(void) {};
#endif
#include "WProgram.h"
#endif

class HX711
{
	private:
		int PD_SCK;	// Power Down and Serial Clock Input Pin
		int DOUT;		// Serial Data Output Pin
		byte GAIN;		// amplification factor
		long OFFSET = 0;	// used for tare weight
		float SCALE = 1;	// used to return weight in grams, kg, ounces, whatever

	public:
		// define clock and data pin, channel, and gain factor
		// channel selection is made by passing the appropriate gain: 128 or 64 for channel A, 32 for channel B
		// gain: 128 or 64 for channel A; channel B works with 32 gain factor only
		HX711(byte dout, byte pd_sck, byte gain) {
			begin(dout, pd_sck, gain);
		};

		HX711() {
		};

		~HX711() {
		};

		// Allows to set the pins and gain later than in the constructor
		void begin(byte dout, byte pd_sck, byte gain = 128) {
			PD_SCK = pd_sck;
			DOUT = dout;

			pinMode(PD_SCK, OUTPUT);
			pinMode(DOUT, INPUT);

			set_gain(gain);
		};
		void begin(int dout, int pd_sck) {
			byte gain = 128;
			PD_SCK = pd_sck;
			DOUT = dout;

			pinMode(PD_SCK, OUTPUT);
			pinMode(DOUT, INPUT);

			set_gain(gain);
		};
		// check if HX711 is ready
		// from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
		// input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
		bool is_ready() {
			return digitalRead(DOUT) == LOW;
		}

		// set the gain factor; takes effect only after a call to read()
		// channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
		// depending on the parameter, the channel is also set to either A or B
		void set_gain(byte gain) {
			switch (gain) {
				case 128:		// channel A, gain factor 128
					GAIN = 1;
					break;
				case 64:		// channel A, gain factor 64
					GAIN = 3;
					break;
				case 32:		// channel B, gain factor 32
					GAIN = 2;
					break;
			}
			digitalWrite(PD_SCK, LOW);
			read();
		};
		// waits for the chip to be ready and returns a reading
		long read() {
			// wait for the chip to become ready
			while (!is_ready()) {
				// Will do nothing on Arduino but prevent resets of ESP8266 (Watchdog Issue)
				yield();
			}

			unsigned long value = 0;
			uint8_t data[3] = { 0 };
			uint8_t filler = 0x00;

			// pulse the clock pin 24 times to read the data
			data[2] = shiftIn(DOUT, PD_SCK, MSBFIRST);
			data[1] = shiftIn(DOUT, PD_SCK, MSBFIRST);
			data[0] = shiftIn(DOUT, PD_SCK, MSBFIRST);

			// set the channel and the gain factor for the next reading using the clock pin
			for (unsigned int i = 0; i < GAIN; i++) {
				digitalWrite(PD_SCK, HIGH);
				digitalWrite(PD_SCK, LOW);
			}

			// Replicate the most significant bit to pad out a 32-bit signed integer
			if (data[2] & 0x80) {
				filler = 0xFF;
			} else {
				filler = 0x00;
			}

			// Construct a 32-bit signed integer
			value = ( static_cast<unsigned long>(filler) << 24
					| static_cast<unsigned long>(data[2]) << 16
					| static_cast<unsigned long>(data[1]) << 8
					| static_cast<unsigned long>(data[0]) );

			return static_cast<long>(value);
		};

		// returns an average reading; times = how many times to read
		long read_average(byte times = 10) {
			long sum = 0;
			for (byte i = 0; i < times; i++) {
				sum += read();
				yield();
			}
			return sum / times;
		};

		// returns (read_average() - OFFSET), that is the current value without the tare weight; times = how many readings to do
		double get_value(byte times = 1) {
			return read_average(times) - OFFSET;
		};

		// returns get_value() divided by SCALE, that is the raw value divided by a value obtained via calibration
		// times = how many readings to do
		float get_units(byte times = 1) {
			return get_value(times) / SCALE;
		};

		// set the OFFSET value for tare weight; times = how many times to read the tare value
		void tare(byte times = 10) {
			double sum = read_average(times);
			set_offset(sum);
		};

		// set the SCALE value; this value is used to convert the raw data to "human readable" data (measure units)
		void set_scale(float scale = 1.f) {
			SCALE = scale;
		};

		// get the current SCALE
		float get_scale() {
			return SCALE;
		};
		// set OFFSET, the value that's subtracted from the actual reading (tare weight)
		void set_offset(long offset = 0) {
			OFFSET = offset;
		};

		// get the current OFFSET
		long get_offset() {
			return OFFSET;
		};

		// puts the chip into power down mode
		void power_down() {
			digitalWrite(PD_SCK, LOW);
			digitalWrite(PD_SCK, HIGH);
		};
		// wakes up the chip after power down mode
		void power_up() {
			digitalWrite(PD_SCK, LOW);
		};
};

#endif /* HX711_h */
