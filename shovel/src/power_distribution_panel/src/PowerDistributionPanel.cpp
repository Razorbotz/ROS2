
#include "power_distribution_panel/PowerDistributionPanel.hpp"
#include <ncurses.h>
#include <iostream>

/** @file
 * @brief Brief description of file
 * Detailed description of file 
 * 
 * */

// This is hardcoded to a power panel at id 1   Sad!!!

/** @brief Brief description of function
 * Detailed description of function
 * */
PowerDistributionPanel::PowerDistributionPanel(){
	this->voltage=0;
}

/** @brief Brief description of function
 * Detailed description of function
 * @param source
 * @return currentC[source]
 * */
float PowerDistributionPanel::getCurrent(int source){
	return currentA[source];
}

/** @brief Brief description of function
 * Detailed description of function
 * @param source
 * @return currentA[source]
 * */
float PowerDistributionPanel::getCurrentA(int source){
	return currentA[source];
}

/** @brief Brief description of function
 * Detailed description of function
 * @param source
 * @return currentB[source]
 * */
float PowerDistributionPanel::getCurrentB(int source){
	return currentB[source];
}

/** @brief Brief description of function
 * Detailed description of function
 * @param source
 * @return currentC[source]
 * */
float PowerDistributionPanel::getCurrentC(int source){
	return currentC[source];
}

/** @brief Brief description of function
 * Detailed description of function
 * @param source
 * @return voltage
 * */
float PowerDistributionPanel::getVoltage(){
	return voltage;
}

float PowerDistributionPanel::getTemperature(){
	return temperature;
}

/** @brief Brief description of function
 * Detailed description of function
 * @param frame
 * @return void
 * */
void PowerDistributionPanel::parseFrame(struct can_frame frame){

        if(frame.can_id==0x88041481 || frame.can_id==0x88041541 || frame.can_id==0x88041601){
		parseVoltage(frame);
		parseTemperature(frame);
	}
	
	if(frame.can_id==0x88041401||
           frame.can_id==0x88041441||
           frame.can_id==0x88041481||
           frame.can_id==0x880414C1||
           frame.can_id==0x88041501||
           frame.can_id==0x88041541||
           frame.can_id==0x88041581||
           frame.can_id==0x880415C1||
           frame.can_id==0x88041601){
		parseCurrent(frame);
	}
        if(frame.can_id==0x88041641){
		//who knows
	}
}

/** @brief Brief description of function
 * Detailed description of function
 * @param frame
 * @return void
 * */
void PowerDistributionPanel::parseVoltage(struct can_frame frame){
	if(frame.can_id==0x88041481 || frame.can_id==0x88041541 || frame.can_id==0x88041601){
		this->voltage=.05*frame.data[6]+4;
        }
}

void PowerDistributionPanel::parseTemperature(struct can_frame frame){
	if(frame.can_id==0x88041481 || frame.can_id==0x88041541 || frame.can_id==0x88041601){
		this->temperature = 1.03250836957542 * frame.data[7] - 67.8564500484966;
        }
}
/** @brief Brief description of function
 * Detailed description of function
 * @param frame
 * @return void
 * */
void PowerDistributionPanel::parseCurrent(struct can_frame frame){
        float currentScalar = 0.125f;
        int i1=frame.data[0];
        i1=i1<<2;
        i1=i1|(frame.data[1]>>6 & 0x03);
        float current1=i1*currentScalar;

        int i2=frame.data[1] & 0x3f;
        i2=i2<<4;
        i2=i2|(frame.data[2]>>4 & 0x0f);
        float current2=i2*currentScalar;

        int i3=frame.data[2] & 0x0f;
        i3=i3<<6;
        i3=i3|(frame.data[3]>>2 & 0x3f);
        float current3=i3*currentScalar;

        int i4=frame.data[3] & 0x03;
        i4=i4<<8;
        i4=i4|(frame.data[4]);
        float current4=i4*currentScalar;

        int i5=frame.data[5];
        i5=i5<<2;
        i5=i5|(frame.data[6]>>6 & 0x03);
        float current5=i5*currentScalar;

        int i6=frame.data[6] & 0x3f;
        i6=i6<<4;
        i6=i6|(frame.data[7]>>4 & 0x7);
        float current6=i6*currentScalar;



	if(frame.can_id==0x88041401){
		this->currentA[0]=current1;
		this->currentA[1]=current2;
		this->currentA[2]=current3;
		this->currentA[3]=current4;
		this->currentA[4]=current5;
		this->currentA[5]=current6;
	}
        if(frame.can_id==0x88041441){
		this->currentA[6]=current1;
		this->currentA[7]=current2;
		this->currentA[8]=current3;
		this->currentA[9]=current4;
		this->currentA[10]=current5;
		this->currentA[11]=current6;
	}
        if(frame.can_id==0x88041481){
		this->currentA[12]=current1;
		this->currentA[13]=current2;
		this->currentA[14]=current3;
		this->currentA[15]=current4;
	}
        if(frame.can_id==0x880414C1){
		this->currentB[0]=current1;
		this->currentB[1]=current2;
		this->currentB[2]=current3;
		this->currentB[3]=current4;
		this->currentB[4]=current5;
		this->currentB[5]=current6;
	}
        if(frame.can_id==0x88041501){
		this->currentC[6]=current1;
		this->currentC[7]=current2;
		this->currentC[8]=current3;
		this->currentC[9]=current4;
		this->currentC[10]=current5;
		this->currentC[11]=current6;
	}
        if(frame.can_id==0x88041541){
		this->currentB[12]=current1;
		this->currentB[13]=current2;
		this->currentB[14]=current3;
		this->currentB[15]=current4;
	}
        if(frame.can_id==0x88041581){
		this->currentC[0]=current1;
		this->currentC[1]=current2;
		this->currentC[2]=current3;
		this->currentC[3]=current4;
		this->currentC[4]=current5;
		this->currentC[5]=current6;
	}
        if(frame.can_id==0x880415C1){
		this->currentC[6]=current1;
		this->currentC[7]=current2;
		this->currentC[8]=current3;
		this->currentC[9]=current4;
		this->currentC[10]=current5;
		this->currentC[11]=current6;
	}
        if(frame.can_id==0x88041601){
		this->currentC[12]=current1;
		this->currentC[13]=current2;
		this->currentC[14]=current3;
		this->currentC[15]=current4;
	}
}
