
#include "power_distribution_panel/PowerDistributionPanel.hpp"
#include <ncurses.h>
#include <iostream>

/** @file
 * @brief Functions that parse the PDP CAN messages
 * 
 * This file contains functions that parse the CAN messages
 * published by the PDP.   
 * 
 * */


/** @brief Brief description of function
 * Detailed description of function
 * */
PowerDistributionPanel::PowerDistributionPanel(int canID){
	this->voltage=0;
	this->canID = canID;
}


/** @brief Brief description of function
 * Detailed description of function
 * @param source
 * @return current[source]
 * */
float PowerDistributionPanel::getCurrent(int source){
	return current[source];
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

	if(frame.can_id == (this->STATUS_3 | this->canID)){
		parseVoltage(frame);
		parseTemperature(frame);
	}
	
	if(frame.can_id == (this->STATUS_1 | this->canID) ||
	   frame.can_id == (this->STATUS_2 | this->canID) || 
	   frame.can_id == (this->STATUS_3 | this->canID)){
		parseCurrent(frame);
	}
}


/** @brief Function that parses the voltage\n
 * here
 * 
 * @param frame
 * @return void
 * */
void PowerDistributionPanel::parseVoltage(struct can_frame frame){
	if(frame.can_id == (this->STATUS_3 | this->canID)){
		this->voltage = .05 * frame.data[6] + 4;
	}
}


void PowerDistributionPanel::parseTemperature(struct can_frame frame){
	if(frame.can_id == (this->STATUS_3 | this->canID)){
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
        int i1 = (int16_t)((char)frame.data[0]);
        i1 = i1 << 2;
        i1 = i1 | (int16_t)(frame.data[1] >> 6 & 0x03);
        float current1 = i1 * currentScalar;

        int i2 = (int16_t)((frame.data[1]) & 0x3f);
        i2 = i2 << 4;
        i2 = i2 | (int16_t)(frame.data[2] >> 4 & 0x0f);
        float current2 = i2 * currentScalar;

        int i3 = (int16_t)((frame.data[2]) & 0x0f);
        i3 = i3 << 6;
        i3 = i3 | (int16_t)(frame.data[3] >> 2 & 0x3f);
        float current3 = i3 * currentScalar;

        int i4 = (int16_t)((frame.data[3]) & 0x03);
        i4 = i4 << 8;
        i4 = i4 | (int16_t)(frame.data[4]);
        float current4 = i4 * currentScalar;

        int i5 = (int16_t)((frame.data[5]));
        i5 = i5 << 2;
        i5 = i5 | (int16_t)(frame.data[6] >> 6 & 0x03);
        float current5 = i5 * currentScalar;

        int i6 = (int16_t)((frame.data[6]) & 0x3f);
        i6 = i6 << 4;
        i6 = i6 | (int16_t)(frame.data[7] >> 4 & 0x0f);
        float current6 = i6 * currentScalar;



	if(frame.can_id == (this->STATUS_1 | this->canID)){
		this->current[0]=current1;
		this->current[1]=current2;
		this->current[2]=current3;
		this->current[3]=current4;
		this->current[4]=current5;
		this->current[5]=current6;
	}
	if(frame.can_id == (this->STATUS_2 | this->canID)){
		this->current[6]=current1;
		this->current[7]=current2;
		this->current[8]=current3;
		this->current[9]=current4;
		this->current[10]=current5;
		this->current[11]=current6;
	}
	if(frame.can_id == (this->STATUS_3 | this->canID)){
		this->current[12]=current1;
		this->current[13]=current2;
		this->current[14]=current3;
		this->current[15]=current4;
	}
}
