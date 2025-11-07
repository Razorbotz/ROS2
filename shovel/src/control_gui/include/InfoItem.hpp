#pragma once

#include <iostream>

#include <gtkmm.h>
#include <gdkmm.h>

class InfoItem:public Gtk::Box{
	private:
	Gtk::Label* nameLabel;
	Gtk::Label* valueLabel;
    int order;

    int decimalPlaces = 6;

	public:
	InfoItem(std::string name);

	void setName(std::string name);
	std::string getName();

    void setDecimalPlaces(int places);

    
    void setValue(long value);
    void setValue(double value);
    void setValue(float value);
    void setValue(std::string value);
    void setValue(const char* value);
    template<typename T>
    void setValue(T value) {
        std::string valueString = std::to_string(value);
        setValue(valueString);
    }

	bool getValueAsBool();
    int getValueAsInt();
    long getValueAsLong();
    float getValueAsFloat();
    double getValueAsDouble();
    std::string getValue();
    void setBackground(std::string color);
    void setTextColor(std::string color, bool bold);
};
