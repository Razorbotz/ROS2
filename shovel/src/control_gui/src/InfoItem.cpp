#include <string>
#include <cstdlib>
#include "InfoItem.hpp"

InfoItem::InfoItem(std::string name):Gtk::Box(Gtk::ORIENTATION_HORIZONTAL,2){
    nameLabel=Gtk::manage(new Gtk::Label(name));
    valueLabel=Gtk::manage(new Gtk::Label("0"));

    this->pack_start(*nameLabel,false,true,10);
    this->pack_end(*valueLabel,false,true,10);
}

void InfoItem::setName(std::string name){
    nameLabel->set_text(name);
}

std::string InfoItem::getName(){
    return nameLabel->get_text();
}

void InfoItem::setDecimalPlaces(int places){
    decimalPlaces = places;
}

void InfoItem::setValue(std::string value) {
    valueLabel->set_text(value);
}

void InfoItem::setValue(const char* value) {
    valueLabel->set_text(value);
}

// Specializations or overloads for types with string truncation
void InfoItem::setValue(float value) {
    std::string valueString = std::to_string(value);
    if (valueString.length() > decimalPlaces) valueString.resize(decimalPlaces);
    setValue(valueString);
}

void InfoItem::setValue(double value) {
    std::string valueString = std::to_string(value);
    if (valueString.length() > decimalPlaces) valueString.resize(decimalPlaces);
    setValue(valueString);
}

void InfoItem::setValue(long value) {
    std::string valueString = std::to_string(value);
    if (valueString.length() > decimalPlaces) valueString.resize(decimalPlaces);
    setValue(valueString);
}

std::string InfoItem::getValue(){
    return valueLabel->get_text();
}
bool InfoItem::getValueAsBool(){
    bool valueBool;
    std::string valueString=getValue();
    if(valueString=="true"){
        valueBool = true;
    }else{
        valueBool = false;
    }
    return valueBool;
}
int InfoItem::getValueAsInt(){
    return std::stoi(getValue());
}
long InfoItem::getValueAsLong(){
    return std::stol(getValue());
}
float InfoItem::getValueAsFloat(){
    return std::stof(getValue());
}
double InfoItem::getValueAsDouble(){
    return std::stod(getValue());
}

void InfoItem::setBackground(std::string color){
    this->valueLabel->override_background_color(Gdk::RGBA(color));
}

void InfoItem::setTextColor(std::string color, bool bold){
    if(bold){
        Pango::FontDescription font;
        font.set_weight(Pango::WEIGHT_BOLD);
        this->valueLabel->override_font(font);
    }
    this->valueLabel->override_color(Gdk::RGBA(color));
}