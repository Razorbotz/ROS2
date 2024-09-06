
#include <linux/can.h>
#include <linux/can/raw.h>

class PowerDistributionPanel{

	private:
		float voltage;
		float temperature;
		float currentA[15];
		float currentB[15];
		float currentC[15];

	public:
		PowerDistributionPanel();
		float getCurrent(int source);
		float getCurrentA(int source);
		float getCurrentB(int source);
		float getCurrentC(int source);
		float getVoltage();
		float getTemperature();
		void parseFrame(struct can_frame);	
		void parseVoltage(struct can_frame frame);
		void parseTemperature(struct can_frame frame);
		void parseCurrent(struct can_frame frame);
};
