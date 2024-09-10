#include <cstdint>
#include <linux/can.h>
#include <linux/can/raw.h>

class PowerDistributionPanel{
	private:
		int canID;
		float voltage;
		float temperature;
		float current[16];

	public:
		const uint32_t STATUS_1 = 0x08041400; //Channels 0-5
		const uint32_t STATUS_2 = 0x08041440; //Channels 6-11
		const uint32_t STATUS_3 = 0x08041480; //Channels 12-15

		PowerDistributionPanel(int canID);
		float getCurrent(int source);
		float getVoltage();
		float getTemperature();
		void parseFrame(struct can_frame);	
		void parseVoltage(struct can_frame frame);
		void parseTemperature(struct can_frame frame);
		void parseCurrent(struct can_frame frame);
};
