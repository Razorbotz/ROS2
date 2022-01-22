
#include <linux/can.h>
#include <linux/can/raw.h>

class PowerDistributionPanel {
	// Names CAN ID hex values from querying the CAN bus.
	enum CAN_ID {
		id1 = 0x88041401,
		id2 = 0x88041441,
		id3 = 0x88041481,
		id4 = 0x880414C1,
		id5 = 0x88041501,
		id6 = 0x88041541,
		id7 = 0x88041581,
		id8 = 0x880415C1,
		id9 = 0x88041601,
		SOMETHING_ELSE = 0x88041641, /// <This is pretty much the same as unknown, but different
		UNKNOWN = 0xFFFFFFFF		 /// <This isn't really even used
	};

  private:
	float voltage = 0.f;
	float currentA[15];
	float currentB[15];
	float currentC[15];

  public:
	float getCurrentA(int source);
	float getCurrentB(int source);
	float getCurrentC(int source);
	float getVoltage();
	void parseFrame(struct can_frame);
	void parseVoltage(struct can_frame frame);
	void parseCurrent(struct can_frame frame);
};
