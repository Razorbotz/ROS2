#include "power_distribution_panel/PowerDistributionPanel.hpp"
#include <iostream>
#include <cmath>

PowerDistributionPanel::PowerDistributionPanel(int canID){
    this->voltage = 0;
    this->temperature = 0;
    this->canID = canID;
    for(int i=0; i<16; i++) this->current[i] = 0;
}

float PowerDistributionPanel::getCurrent(int source){
    if(source < 0 || source > 15) return 0.0f;
    return current[source];
}

float PowerDistributionPanel::getVoltage(){
    return voltage;
}

float PowerDistributionPanel::getTemperature(){
    return temperature;
}

void PowerDistributionPanel::parseFrame(struct can_frame frame){
    unsigned int can_id = frame.can_id & ID_MASK;
    
    // Status 3 contains Voltage, Temperature, and Channels 12-15
    if(can_id == (this->STATUS_3 | this->canID)){
        parseVoltage(frame);
        parseTemperature(frame);
        parseCurrent(frame);
    }
    
    // Status 1 (0-5) and Status 2 (6-11)
    else if(can_id == (this->STATUS_1 | this->canID) ||
            can_id == (this->STATUS_2 | this->canID)){
        parseCurrent(frame);
    }
}

void PowerDistributionPanel::parseVoltage(struct can_frame frame){
    uint8_t raw = frame.data[6];
    this->voltage = (raw * 0.05f) + 4.0f;
}

void PowerDistributionPanel::parseTemperature(struct can_frame frame){
    uint8_t raw = frame.data[7];
    this->temperature = (1.03250836957542 * raw) - 67.8564500484966; 
}

void PowerDistributionPanel::parseCurrent(struct can_frame frame){
    const uint8_t* d = frame.data; 
    float currentScalar = 0.125f;

    uint16_t i1 = ((uint16_t)d[0] << 2) | ((d[1] >> 6) & 0x03);
    float val1 = i1 * currentScalar;

    uint16_t i2 = ((uint16_t)(d[1] & 0x3F) << 4) | ((d[2] >> 4) & 0x0F);
    float val2 = i2 * currentScalar;

    uint16_t i3 = ((uint16_t)(d[2] & 0x0F) << 6) | ((d[3] >> 2) & 0x3F);
    float val3 = i3 * currentScalar;

    uint16_t i4 = ((uint16_t)(d[3] & 0x03) << 8) | d[4];
    float val4 = i4 * currentScalar;

    uint16_t i5 = ((uint16_t)d[5] << 2) | ((d[6] >> 6) & 0x03);
    float val5 = i5 * currentScalar;

    uint16_t i6 = ((uint16_t)(d[6] & 0x3F) << 4) | ((d[7] >> 4) & 0x0F);
    float val6 = i6 * currentScalar;


    unsigned int can_id = frame.can_id & ID_MASK;

    if(can_id == (this->STATUS_1 | this->canID)){
        // Channels 0-5
        this->current[0] = val1;
        this->current[1] = val2;
        this->current[2] = val3;
        this->current[3] = val4;
        this->current[4] = val5;
        this->current[5] = val6;
    }
    else if(can_id == (this->STATUS_2 | this->canID)){
        // Channels 6-11
        this->current[6] = val1;
        this->current[7] = val2;
        this->current[8] = val3;
        this->current[9] = val4;
        this->current[10] = val5;
        this->current[11] = val6;
    }
    else if(can_id == (this->STATUS_3 | this->canID)){
        this->current[12] = val1;
        this->current[13] = val2;
        this->current[14] = val3;
        this->current[15] = val4;
    }
}