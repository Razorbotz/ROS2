
#include "power_distribution_panel/PowerDistributionPanel.hpp"
#include <iostream>
#include <ncurses.h>

/** @file
 * @brief Defines all of the functions in PowerDistributionPanel.hpp
 * Processes all of the data for the Power Distribution Panel.
 *
 * */

// This is hardcoded to a power panel at id 1   Sad!!!

/** @brief Gets current A from the power panel
 * Gets the current A from the power panel from the specified source
 * @param source
 * @return currentA[source]
 * */
float PowerDistributionPanel::getCurrentA(int source) {
	return currentA[source];
}

/** @brief Gets current B from the power panel
 * Gets the current B from the power panel from the specified source
 * @param source
 * @return currentB[source]
 * */
float PowerDistributionPanel::getCurrentB(int source) {
	return currentB[source];
}

/** @brief Gets current C from the power panel
 * Gets the current C from the power panel from the specified source
 * @param source
 * @return currentC[source]
 * */
float PowerDistributionPanel::getCurrentC(int source) {
	return currentC[source];
}

/** @brief Returns voltage
 * Returns the voltage (as a float)
 * @param source
 * @return voltage
 * */
float PowerDistributionPanel::getVoltage() {
	return voltage;
}

/** @brief Parse a frame from the PDP
 * Parses current (and voltage if CAN ID is 3, 6, or 9) data from the PDP.
 * @param frame
 * @return void
 * */
void PowerDistributionPanel::parseFrame(struct can_frame frame) {
	switch(frame.can_id) {
		// Parse voltage (and also current)
		case id3:
		case id6:
		case id9:
			parseVoltage(frame);
			// Continue to parse current

		// Parse current
		case id1:
		case id2:
		case id4:
		case id5:
		case id7:
		case id8:
			parseCurrent(frame);
			break;

		// Unknown ID
		default:
			// Should not get here
			break;
	}
}

/** @brief Parses voltage from a CAN frame
 * Parses voltage from a CAN frame (if the CAN ID is 3, 6, or 9) doing some operations on the data and setting the voltage member variable.
 * @param frame
 * @return void
 * */
void PowerDistributionPanel::parseVoltage(struct can_frame frame) {
	switch(frame.can_id) {
		case id3:
		case id6:
		case id9:
			this->voltage = 0.05f * frame.data[6] + 4;
			break;

		default:
			break;
	}
}

/** @brief Proceses and sets the current for the PDP class
 * Does a bunch of ugly bitwise manipulations on the frame data to get the current and then sets the corresponding current member variable (A, B, or C)
 * @param frame
 * @return void
 * */
void PowerDistributionPanel::parseCurrent(struct can_frame frame) {
	// A bunch of ugly bit manipulations
	constexpr float currentScalar = 0.125f;

	// Current 1
	int i1 = frame.data[0];
	i1 = i1 << 2;
	i1 = i1 | (frame.data[1] >> 6 & 0x03);
	const float current1 = i1 * currentScalar;

	// Current 2
	int i2 = frame.data[1] & 0x3f;
	i2 = i2 << 4;
	i2 = i2 | (frame.data[2] >> 4 & 0x0f);
	const float current2 = i2 * currentScalar;

	// Current 3
	int i3 = frame.data[2] & 0x0f;
	i3 = i3 << 6;
	i3 = i3 | (frame.data[3] >> 2 & 0x3f);
	const float current3 = i3 * currentScalar;

	// Current 4
	int i4 = frame.data[3] & 0x03;
	i4 = i4 << 8;
	i4 = i4 | (frame.data[4]);
	const float current4 = i4 * currentScalar;

	// Current 5
	int i5 = frame.data[5];
	i5 = i5 << 2;
	i5 = i5 | (frame.data[6] >> 6 & 0x03);
	const float current5 = i5 * currentScalar;

	// Current 6
	int i6 = frame.data[6] & 0x3f;
	i6 = i6 << 4;
	i6 = i6 | (frame.data[7] >> 4 & 0x7);
	const float current6 = i6 * currentScalar;

	// Set the frame's current
	size_t offset = 0;
	switch(frame.can_id) {
		case id1:
			offset = 0;
			this->currentA[offset + 0] = current1;
			this->currentA[offset + 1] = current2;
			this->currentA[offset + 2] = current3;
			this->currentA[offset + 3] = current4;
			this->currentA[offset + 4] = current5;
			this->currentA[offset + 5] = current6;
			break;

		case id2:
			offset = 6;
			this->currentA[offset + 0] = current1;
			this->currentA[offset + 1] = current2;
			this->currentA[offset + 2] = current3;
			this->currentA[offset + 3] = current4;
			this->currentA[offset + 4] = current5;
			this->currentA[offset + 5] = current6;
			break;

		case id4:
			offset = 0;
			this->currentB[offset + 0] = current1;
			this->currentB[offset + 1] = current2;
			this->currentB[offset + 2] = current3;
			this->currentB[offset + 3] = current4;
			this->currentB[offset + 4] = current5;
			this->currentB[offset + 5] = current6;
			break;

		case id5:
			offset = 6;
			this->currentC[offset + 0] = current1;
			this->currentC[offset + 1] = current2;
			this->currentC[offset + 2] = current3;
			this->currentC[offset + 3] = current4;
			this->currentC[offset + 4] = current5;
			this->currentC[offset + 5] = current6;
			break;

		case id7:
			offset = 0;
			this->currentC[offset + 0] = current1;
			this->currentC[offset + 1] = current2;
			this->currentC[offset + 2] = current3;
			this->currentC[offset + 3] = current4;
			this->currentC[offset + 4] = current5;
			this->currentC[offset + 5] = current6;
			break;

		case id8:
			offset = 6;
			this->currentC[offset + 0] = current1;
			this->currentC[offset + 1] = current2;
			this->currentC[offset + 2] = current3;
			this->currentC[offset + 3] = current4;
			this->currentC[offset + 4] = current5;
			this->currentC[offset + 5] = current6;
			break;

		case id3:
			offset = 12;
			this->currentA[offset + 0] = current1;
			this->currentA[offset + 1] = current2;
			this->currentA[offset + 2] = current3;
			this->currentA[offset + 3] = current4;
			break;

		case id6:
			offset = 12;
			this->currentB[offset + 0] = current1;
			this->currentB[offset + 1] = current2;
			this->currentB[offset + 2] = current3;
			this->currentB[offset + 3] = current4;
			break;

		case id9:
			offset = 12;
			this->currentC[offset + 0] = current1;
			this->currentC[offset + 1] = current2;
			this->currentC[offset + 2] = current3;
			this->currentC[offset + 3] = current4;
			break;

		default:
			// Should not get here
			break;
	}
}
