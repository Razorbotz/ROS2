#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "InfoFrame.hpp"


InfoFrame::InfoFrame(std::string frameName):Gtk::Frame(frameName){
    contentsBox=Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL,5));
    this->add(*contentsBox);    
}


void InfoFrame::addItem(std::string itemName){
    std::shared_ptr<InfoItem> infoItem=std::make_shared<InfoItem>(itemName); 
    this->itemList.push_back(infoItem);
    this->contentsBox->add(*(infoItem));
}

void InfoFrame::removeItem(std::string itemName) {
    for (auto it = itemList.begin(); it != itemList.end(); ++it) {
        if ((*it)->getName() == itemName) {
            this->contentsBox->remove(*(*it));
            itemList.erase(it);
            break;
        }
    }
}

void InfoFrame::addWidget(Gtk::Widget& widget) {
    contentsBox->pack_start(widget, Gtk::PACK_SHRINK);
}

void InfoFrame::removeAllItems() {
    for (const auto& item : itemList) {
        contentsBox->remove(*item);
    }
    itemList.clear();
}

void InfoFrame::setBackground(std::string itemName, std::string color){
    for(std::shared_ptr<InfoItem> infoItem:itemList){
        if(infoItem->getName()==itemName){
            infoItem->setBackground(color);
            return;
        }
    }
    addItem(itemName);
    for(std::shared_ptr<InfoItem> infoItem:itemList){
        if(infoItem->getName()==itemName){
            infoItem->setBackground(color);
            return;
        }
    }
}

void InfoFrame::setTextColor(std::string itemName, std::string color, bool bold){
    for(std::shared_ptr<InfoItem> infoItem:itemList){
        if(infoItem->getName()==itemName){
            infoItem->setTextColor(color, bold);
            return;
        }
    }
    addItem(itemName);
    for(std::shared_ptr<InfoItem> infoItem:itemList){
        if(infoItem->getName()==itemName){
            infoItem->setTextColor(color, bold);
            return;
        }
    }
}

void InfoFrame::setAllItemsStale() {
    for (const auto& infoItem : itemList) {
        if (infoItem) {
            infoItem->setValue("---");
        }
    }
}